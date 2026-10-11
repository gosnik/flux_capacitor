"""Replay the straight degree-two bend audit from final copper telemetry.

Diagnostic only: this is not a replacement for Flux or KiCad validation.
"""
import argparse
import collections
import json
import math


def xy(point):
    return point["x_nm"], point["y_nm"]


def distance(point, first, last):
    dx, dy = last[0] - first[0], last[1] - first[1]
    t = max(0, min(1, ((point[0] - first[0]) * dx + (point[1] - first[1]) * dy) / (dx * dx + dy * dy)))
    return math.hypot(point[0] - first[0] - t * dx, point[1] - first[1] - t * dy)


def arc_geometry(primitive):
    pts = [xy(p) for p in primitive["points"]]
    if primitive["kind"] != "track_arc" or len(pts) != 3:
        return None
    a = (pts[1][0] - pts[0][0], pts[1][1] - pts[0][1])
    b = (pts[2][0] - pts[0][0], pts[2][1] - pts[0][1])
    cross = a[0] * b[1] - a[1] * b[0]
    if not cross:
        return None
    aa, bb = a[0] ** 2 + a[1] ** 2, b[0] ** 2 + b[1] ** 2
    center = (pts[0][0] + (aa * b[1] - bb * a[1]) / (2 * cross),
              pts[0][1] + (a[0] * bb - b[0] * aa) / (2 * cross))
    return center, 1 if cross > 0 else -1


def arc_distance(primitive, point):
    geometry = arc_geometry(primitive)
    if geometry is None:
        return math.inf
    center, sign = geometry
    start, end = xy(primitive["points"][0]), xy(primitive["points"][-1])
    angle = lambda p: math.atan2(p[1] - center[1], p[0] - center[0])
    span = (sign * (angle(end) - angle(start))) % math.tau
    progress = (sign * (angle(point) - angle(start))) % math.tau
    if progress <= span + 1e-12:
        return abs(math.dist(point, center) - math.dist(start, center))
    return min(math.dist(point, start), math.dist(point, end))


def outward_tangent(primitive, point):
    pts = [xy(p) for p in primitive["points"]]
    if primitive["kind"] == "track":
        other = pts[-1] if pts[0] == point else pts[0]
        return other[0] - point[0], other[1] - point[1]
    geometry = arc_geometry(primitive)
    if geometry is None:
        return None
    center, sign = geometry
    sign *= 1 if pts[0] == point else -1
    return -(point[1] - center[1]) * sign, (point[0] - center[0]) * sign


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("telemetry")
    args = parser.parse_args()
    boards = {}
    with open(args.telemetry) as stream:
        for line in stream:
            try:
                event = json.loads(line)
            except ValueError:
                continue
            for board in event.get("source_boards", []):
                boards[board["id"]] = board
    generated = [board for key, board in boards.items() if key.startswith("generated-copper:")]
    if not generated:
        parser.error("no final generated copper snapshot")
    copper = generated[-1]["primitives"]
    pads = [p for key, board in boards.items() if not key.startswith("generated-copper:")
            for p in board["primitives"] if p["kind"] == "pad"]
    nodes = collections.defaultdict(list)
    tracks = [p for p in copper if p["kind"] == "track" and xy(p["points"][0]) != xy(p["points"][-1])]
    for primitive in [*copper, *pads]:
        if primitive["kind"] not in ("track", "track_arc", "via", "pad"):
            continue
        endpoints = [primitive["points"][0]]
        if primitive["kind"] in ("track", "track_arc"):
            endpoints.append(primitive["points"][-1])
        for layer in primitive["layers"]:
            for endpoint in endpoints:
                nodes[primitive["net"], layer, xy(endpoint)].append(primitive)
    bends = []
    arc_joins = []
    for (net, layer, point), incident in sorted(nodes.items()):
        touching = any(p["net"] == net and layer in p["layers"] and p not in incident and
                       any(distance(point, xy(p["points"][0]), xy(p["points"][-1])) <=
                           (p["width_nm"] + q["width_nm"]) / 2 for q in incident) for p in tracks)
        if len(incident) == 2 and not touching and any(p["kind"] == "track_arc" for p in incident):
            arc_contact = any(p["kind"] == "track_arc" and p["net"] == net and
                              layer in p["layers"] and p not in incident and
                              any(arc_distance(p, point) <= (p["width_nm"] + q["width_nm"]) / 2
                                  for q in incident) for p in copper)
            if arc_contact:
                continue
            tangents = [outward_tangent(p, point) for p in incident]
            if all(t is not None and math.hypot(*t) > 0 for t in tangents):
                a, b = tangents
                cosine = -(a[0] * b[0] + a[1] * b[1]) / (math.hypot(*a) * math.hypot(*b))
                angle = math.degrees(math.acos(max(-1, min(1, cosine))))
                if angle > 2:
                    arc_joins.append({"net": net, "layer": layer, "point_nm": point,
                                      "angle_degrees": angle, "ids": [p["id"] for p in incident]})
        if len(incident) != 2 or any(p["kind"] != "track" for p in incident):
            continue
        others = [xy(p["points"][-1]) if xy(p["points"][0]) == point else xy(p["points"][0]) for p in incident]
        first, last = [(p[0] - point[0], p[1] - point[1]) for p in others]
        if first[0] * last[1] == first[1] * last[0]:
            continue
        if not touching:
            bends.append({"net": net, "layer": layer, "point_nm": point,
                          "tracks": [{"id": p["id"], "points_nm": [xy(q) for q in p["points"]],
                                      "width_nm": p["width_nm"]} for p in incident]})
    print(json.dumps({"schema": 1, "authoritative": False, "unrounded_bends": bends,
                      "diagnostic_arc_join_angles_over_2_degrees": arc_joins}, indent=2))


if __name__ == "__main__":
    main()
