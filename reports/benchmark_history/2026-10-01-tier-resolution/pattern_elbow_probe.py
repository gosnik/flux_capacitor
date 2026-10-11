"""Bounded angle-legal elbow-orientation diagnostic; it cannot qualify a tier.

The probe holds every route except one selected route fixed.  It only exchanges
the middle point of a same-layer, monotone shortest octilinear two-segment run:
orthogonal H/V elbows, or an axial and a 45-degree diagonal in either order.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import time


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def point(x, y, layer):
    return {"x_nm": x, "y_nm": y, "layer": layer}


def direction(a, b):
    dx, dy = b["x_nm"] - a["x_nm"], b["y_nm"] - a["y_nm"]
    if not dx and not dy:
        return "zero"
    if not dx:
        return "v"
    if not dy:
        return "h"
    if abs(dx) == abs(dy):
        return "d"
    return "offangle"


def sign(value):
    return (value > 0) - (value < 0)


def monotone(a, b, c):
    return ((b["x_nm"] - a["x_nm"]) * (c["x_nm"] - b["x_nm"]) >= 0 and
            (b["y_nm"] - a["y_nm"]) * (c["y_nm"] - b["y_nm"]) >= 0)


def alternative_middle(a, b, c):
    """Return the other shortest legal elbow, or None outside the stated scope."""
    if a["layer"] != b["layer"] or b["layer"] != c["layer"] or not monotone(a, b, c):
        return None
    first, second = direction(a, b), direction(b, c)
    if {first, second} == {"h", "v"}:
        candidate = point(a["x_nm"], c["y_nm"], a["layer"])
    elif first == "d" and second in ("h", "v"):
        # Move the axial run before the diagonal while retaining its vector.
        if second == "h":
            candidate = point(c["x_nm"] - (b["x_nm"] - a["x_nm"]), a["y_nm"], a["layer"])
        else:
            candidate = point(a["x_nm"], c["y_nm"] - (b["y_nm"] - a["y_nm"]), a["layer"])
    elif second == "d" and first in ("h", "v"):
        # Move the axial run after the diagonal while retaining its vector.
        if first == "h":
            candidate = point(c["x_nm"] - (b["x_nm"] - a["x_nm"]), c["y_nm"], a["layer"])
        else:
            candidate = point(c["x_nm"], c["y_nm"] - (b["y_nm"] - a["y_nm"]), a["layer"])
    else:
        return None
    if candidate == b or direction(a, candidate) not in ("h", "v", "d") or direction(candidate, c) not in ("h", "v", "d"):
        return None
    return candidate


def angle_report(routes):
    offangle = []
    for route in routes:
        for index, (a, b) in enumerate(zip(route["points"], route["points"][1:])):
            kind = direction(a, b)
            if kind in ("offangle", "zero"):
                offangle.append({"route": route["id"], "segment": index, "kind": kind})
    return offangle


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--board", required=True, type=Path)
    parser.add_argument("--scene", required=True, type=Path)
    parser.add_argument("--sequence", required=True, type=int)
    parser.add_argument("--oracle", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--max-checks", type=int, default=100)
    parser.add_argument("--max-rounds", type=int, default=2)
    args = parser.parse_args()
    if args.max_checks < 1 or args.max_rounds < 1:
        parser.error("positive search bounds required")
    args.output.mkdir(parents=True, exist_ok=False)
    snapshot = next(json.loads(line) for line in args.scene.open()
                    if json.loads(line).get("type") == "snapshot" and json.loads(line).get("sequence") == args.sequence)
    selected = [route for route in snapshot["routes"] if route["id"].startswith("global-route:")
                and route["family"] == "selected" and route["lifecycle"] == "active"]
    selected_ids = {route["id"] for route in selected}
    original_routes = {route["id"]: json.dumps(route, sort_keys=True) for route in snapshot["routes"]}
    endpoints = {route["id"]: [copy.deepcopy(route["points"][0]), copy.deepcopy(route["points"][-1])]
                 for route in selected}
    source_offangle = angle_report(selected)
    checks, rows, accepted = 0, [], []
    started = time.monotonic()

    def run(scene, label, move=None):
        nonlocal checks
        checks += 1
        trial_path = args.output / f"trial-{checks:03d}.jsonl"
        trial_path.write_text(json.dumps(scene) + "\n")
        command = [str(args.oracle.resolve()), str(args.board.resolve()), str(trial_path.resolve()), str(args.sequence)]
        completed = subprocess.run(command, capture_output=True, text=True, check=True, timeout=60)
        result = json.loads(completed.stdout.splitlines()[-1])
        if result["pitch_nm"] != 1 or result["paths"] != len(selected):
            raise RuntimeError("oracle coverage or exact pitch changed")
        trial_selected = [r for r in scene["routes"] if r["id"] in selected_ids]
        endpoint_ok = all([r["points"][0], r["points"][-1]] == endpoints[r["id"]] for r in trial_selected)
        angles = angle_report(trial_selected)
        new_offangle = [item for item in angles if item not in source_offangle]
        row = {"check": checks, "label": label, "trial_scene": trial_path.name,
               "endpoint_snapshot_unchanged": endpoint_ok, "new_offangle": new_offangle,
               "move": move, **result}
        rows.append(row)
        with (args.output / "trials.jsonl").open("a") as out:
            out.write(json.dumps(row) + "\n")
        if not endpoint_ok or new_offangle:
            raise RuntimeError("trial violated endpoint or angle invariant")
        return result, row

    def valid(result):
        return result["malformed"] == 0 and result["materialized"] == result["paths"]

    incumbent = copy.deepcopy(snapshot)
    before, _ = run(incumbent, "baseline")
    proof = before
    for round_index in range(args.max_rounds):
        affected_nets = []
        for group in proof["largest_pair_groups"]:
            first, second, _ = group["net_pair_and_layer"].split(":")
            for net in (first, second):
                if net not in affected_nets:
                    affected_nets.append(net)
        moves = []
        for route in incumbent["routes"]:
            if route["id"] not in selected_ids or route["net"] not in affected_nets:
                continue
            for index in range(1, len(route["points"]) - 1):
                middle = alternative_middle(route["points"][index - 1], route["points"][index], route["points"][index + 1])
                if middle:
                    moves.append({"route": route["id"], "net": route["net"], "point_index": index,
                                  "from": route["points"][index], "to": middle})
        improved = False
        for move in moves:
            if checks >= args.max_checks:
                break
            scene = copy.deepcopy(incumbent)
            route = next(r for r in scene["routes"] if r["id"] == move["route"])
            # A prior accepted move can change a neighbouring triplet.  Do not
            # replay a candidate generated for that superseded geometry.
            if route["points"][move["point_index"]] != move["from"]:
                continue
            route["points"][move["point_index"]] = move["to"]
            trial, row = run(scene, {"round": round_index, "route": move["route"], "point_index": move["point_index"]}, move)
            accept = valid(trial) and trial["opens"] <= proof["opens"] and trial["drc_violations"] < proof["drc_violations"]
            row["accepted"] = accept
            if accept:
                accepted.append({"round": round_index, "move": move,
                                 "before": {"physical_errors": proof["drc_violations"], "opens": proof["opens"],
                                            "categories": proof["categories"], "vias": proof["vias"], "routecount": proof["paths"]},
                                 "after": {"physical_errors": trial["drc_violations"], "opens": trial["opens"],
                                           "categories": trial["categories"], "vias": trial["vias"], "routecount": trial["paths"]}})
                incumbent, proof, improved = scene, trial, True
                (args.output / "incumbent.jsonl").write_text(json.dumps(incumbent) + "\n")
                # Regenerate remaining candidates against the new incumbent.
                break
        if not improved or checks >= args.max_checks:
            break
    changed_ids = {item["move"]["route"] for item in accepted}
    final_routes = {route["id"]: json.dumps(route, sort_keys=True) for route in incumbent["routes"]}
    unchanged_ids = sorted(set(original_routes) - changed_ids)
    summary = {"schema": 1, "authoritative": False, "tier_qualified": False,
               "algorithm": "bounded same-layer monotone shortest octilinear elbow orientation",
               "board": str(args.board), "scene": str(args.scene), "sequence": args.sequence,
               "source_hashes": {"board_sha256": sha256(args.board),
                                 "project_sha256": sha256(args.board.with_suffix(".kicad_pro")),
                                 "scene_sha256": sha256(args.scene), "oracle_sha256": sha256(args.oracle)},
               "bounds": {"max_checks": args.max_checks, "max_rounds": args.max_rounds},
               "checks": checks, "seconds": time.monotonic() - started,
               "before": before, "after": proof, "accepted_move_proof": accepted,
               "full_scene_recheck": {"result": proof, "changed_route_ids": sorted(changed_ids),
                                      "all_other_routes_unchanged": all(original_routes[key] == final_routes[key]
                                                                          for key in unchanged_ids),
                                      "unchanged_route_count": len(unchanged_ids)},
               "endpoint_snapshot": endpoints, "source_offangle": source_offangle,
               "final_offangle": angle_report([r for r in incumbent["routes"] if r["id"] in selected_ids]),
               "limitations": "No resource/capacity allocation, native fitting, or full KiCad qualification. A reduction only diagnoses the elbow-orientation mechanism; it is not an active routing heuristic."}
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({"checks": checks, "seconds": summary["seconds"], "before_errors": before["drc_violations"],
                      "after_errors": proof["drc_violations"], "opens": proof["opens"], "accepted_moves": len(accepted)}, indent=2))


if __name__ == "__main__":
    main()
