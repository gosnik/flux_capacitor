#!/usr/bin/env python3
"""Render bounded near-result regional spline candidates from a Flux telemetry trace.

The generated boards and images are diagnostic previews.  They are deliberately reconstructed
from non-authoritative telemetry and must never be treated as accepted routed copper.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
import tempfile
import uuid
from dataclasses import dataclass, field
from pathlib import Path

from PIL import Image


START = re.compile(
    r"regional spline realization started: window=(\d+), assignment=(\d+)/(\d+)"
)
CANDIDATE = re.compile(
    r"candidate_rank=(\d+), topology_legal=(yes|no), "
    r"endpoint_topology=(yes|no), interior_topology=(yes|no), "
    r"gate_sequence=(yes|no), gate_count=(\d+).*mutually_legal=(yes|no)"
)
NET_ID = re.compile(r"^kicad-net-(\d+)$")
COPPER_LAYER = re.compile(r'^\s*\(\d+\s+"([^"]+\.Cu)"\s+')


@dataclass
class NearResult:
    sequence: int
    window: int
    assignment: int
    assignment_count: int
    candidate_rank: int = 0
    routes: list[dict] = field(default_factory=list)
    topology_failures: int = 0
    mutual_failures: int = 0

    def score(self) -> tuple[int, int, int, int, int, int]:
        # Feasibility first. Among equally legal local realizations, prefer the candidate exposing
        # more of the coupled window, then the earlier spline rank and the newest observation.
        total_failures = self.topology_failures + self.mutual_failures
        return (
            total_failures,
            self.topology_failures,
            self.mutual_failures,
            -len(self.routes),
            self.candidate_rank,
            -self.sequence,
        )

    def fingerprint(self) -> str:
        digest = hashlib.sha256()
        for route in sorted(self.routes, key=lambda value: value.get("id", "")):
            digest.update(str(route.get("id", "")).encode())
            digest.update(str(route.get("revision", 0)).encode())
            for point in route.get("points", []):
                digest.update(
                    f"{point.get('x_nm', 0)},{point.get('y_nm', 0)},{point.get('layer', '')};".encode()
                )
        return digest.hexdigest()


def run(command: list[str]) -> None:
    completed = subprocess.run(command, check=False, capture_output=True, text=True)
    if completed.returncode != 0:
        raise RuntimeError(
            f"command failed ({completed.returncode}): {' '.join(command)}\n"
            f"stdout:\n{completed.stdout}\nstderr:\n{completed.stderr}"
        )


def read_near_results(trace: Path, limit: int) -> tuple[list[NearResult], list[dict]]:
    retained: list[NearResult] = []
    failure_categories: dict[str, list[NearResult]] = {
        "mutual": [],
        "topology": [],
        "combined": [],
    }
    fingerprints: set[str] = set()
    current: NearResult | None = None
    latest_snapshot_raw: bytes | None = None

    def retain() -> None:
        nonlocal current
        if current is None or not current.routes:
            current = None
            return
        fingerprint = current.fingerprint()
        if fingerprint in fingerprints:
            current = None
            return
        fingerprints.add(fingerprint)
        retained.append(current)
        retained.sort(key=NearResult.score)
        # Keep extra diversity while streaming so a later duplicate does not evict every useful
        # alternate. The final result is still bounded by the caller's requested limit.
        if len(retained) > max(limit * 4, limit):
            removed = retained.pop()
            fingerprints.discard(removed.fingerprint())
        category = None
        if current.topology_failures > 0 and current.mutual_failures > 0:
            category = "combined"
        elif current.topology_failures > 0:
            category = "topology"
        elif current.mutual_failures > 0:
            category = "mutual"
        if category is not None:
            failure_categories[category].append(current)
            failure_categories[category].sort(key=NearResult.score)
            if len(failure_categories[category]) > limit:
                failure_categories[category].pop()
        current = None

    with trace.open("rb") as stream:
        for raw in stream:
            if b'"type":"snapshot"' in raw:
                latest_snapshot_raw = raw
            if b"regional spline realization started" not in raw and b"candidate regional quintic PH curve tessellation" not in raw:
                continue
            try:
                event = json.loads(raw)
            except json.JSONDecodeError:
                # A caller timeout may leave one incomplete final telemetry line. Earlier complete
                # candidates remain usable diagnostic evidence.
                continue
            message = event.get("message", "")
            started = START.search(message)
            if started:
                retain()
                window, assignment, assignment_count = map(int, started.groups())
                current = NearResult(
                    sequence=int(event.get("sequence", 0)),
                    window=window,
                    assignment=assignment,
                    assignment_count=assignment_count,
                )
                continue
            parsed = CANDIDATE.search(message)
            if current is None or parsed is None:
                continue
            rank, topology, _endpoint, _interior, _gate, _gate_count, mutual = parsed.groups()
            current.candidate_rank = int(rank)
            current.topology_failures += topology != "yes"
            current.mutual_failures += mutual != "yes"
            for route in event.get("routes", []):
                if route.get("geometry") == "quintic_ph" and len(route.get("points", [])) >= 2:
                    current.routes.append(route)
    retain()
    context_routes: list[dict] = []
    if latest_snapshot_raw is not None:
        try:
            snapshot = json.loads(latest_snapshot_raw)
            context_routes = [
                route
                for route in snapshot.get("routes", [])
                if route.get("family") == "selected"
                and route.get("lifecycle") == "active"
                and route.get("state") != "ripped_up"
                and len(route.get("points", [])) >= 2
            ]
        except json.JSONDecodeError:
            pass
    # A short or successful run may not enter regional repair. Its latest selected checkpoint is
    # still useful analysis evidence, so render that snapshot instead of producing no PNGs.
    if not retained:
        retained.append(
            NearResult(
                sequence=int(json.loads(latest_snapshot_raw).get("sequence", 0))
                if latest_snapshot_raw is not None
                else 0,
                window=-1,
                assignment=0,
                assignment_count=0,
                routes=context_routes,
            )
        )
        context_routes = []

    # Keep most slots feasibility-first, but guarantee representative physical/topological
    # failures when the trace contains them. These are often more useful for debugging than eight
    # locally clean windows embedded in the same globally failed board.
    selected = retained[: max(1, limit - 3)]
    selected_fingerprints = {candidate.fingerprint() for candidate in selected}
    for category in ("mutual", "topology", "combined"):
        candidate = next(
            (
                value
                for value in failure_categories[category]
                if value.fingerprint() not in selected_fingerprints
            ),
            None,
        )
        if candidate is not None and len(selected) < limit:
            selected.append(candidate)
            selected_fingerprints.add(candidate.fingerprint())
    for candidate in [*retained, *failure_categories["mutual"], *failure_categories["topology"], *failure_categories["combined"]]:
        if len(selected) >= limit:
            break
        fingerprint = candidate.fingerprint()
        if fingerprint not in selected_fingerprints:
            selected.append(candidate)
            selected_fingerprints.add(fingerprint)
    return selected, context_routes


def enabled_copper_layers(board_text: str) -> list[str]:
    layers: list[str] = []
    in_layers = False
    for line in board_text.splitlines():
        stripped = line.lstrip()
        if not in_layers:
            if stripped.startswith("(layers"):
                in_layers = True
            continue
        if stripped.startswith(")"):
            break
        match = COPPER_LAYER.match(line)
        if match:
            layers.append(match.group(1))
    if not layers:
        raise RuntimeError("source board has no enabled copper layers")
    return layers


def mm(value_nm: int) -> str:
    return f"{value_nm / 1_000_000.0:.6f}"


def object_uuid(result: NearResult, route_id: str, kind: str, index: int) -> str:
    key = f"flux-failed-route:{result.sequence}:{route_id}:{kind}:{index}"
    return str(uuid.uuid5(uuid.NAMESPACE_URL, key))


def diagnostic_objects(
    result: NearResult, include_controls: bool, context_routes: list[dict]
) -> str:
    objects: list[str] = []
    for route in [*context_routes, *result.routes]:
        route_id = str(route.get("id", "route"))
        net_match = NET_ID.match(str(route.get("net", "")))
        net = int(net_match.group(1)) if net_match else 0
        width = max(1, int(route.get("width_nm", 1)))
        points = route.get("points", [])
        for index, (first, second) in enumerate(zip(points, points[1:])):
            if first.get("x_nm") == second.get("x_nm") and first.get("y_nm") == second.get("y_nm"):
                continue
            layer = str(first.get("layer") or second.get("layer") or "F.Cu")
            objects.append(
                "\n\t(segment"
                f"\n\t\t(start {mm(int(first['x_nm']))} {mm(int(first['y_nm']))})"
                f"\n\t\t(end {mm(int(second['x_nm']))} {mm(int(second['y_nm']))})"
                f"\n\t\t(width {mm(width)})"
                f"\n\t\t(layer \"{layer}\")"
                f"\n\t\t(net {net})"
                f"\n\t\t(uuid \"{object_uuid(result, route_id, 'segment', index)}\")"
                "\n\t)"
            )
        if not include_controls or route not in result.routes:
            continue
        controls = route.get("control_points", [])
        for index, (first, second) in enumerate(zip(controls, controls[1:])):
            objects.append(
                "\n\t(gr_line"
                f"\n\t\t(start {mm(int(first['x_nm']))} {mm(int(first['y_nm']))})"
                f"\n\t\t(end {mm(int(second['x_nm']))} {mm(int(second['y_nm']))})"
                "\n\t\t(stroke (width 0.060000) (type dash))"
                "\n\t\t(layer \"Eco1.User\")"
                f"\n\t\t(uuid \"{object_uuid(result, route_id, 'control-line', index)}\")"
                "\n\t)"
            )
        for index, control in enumerate(controls):
            radius_nm = max(80_000, width // 3)
            objects.append(
                "\n\t(gr_circle"
                f"\n\t\t(center {mm(int(control['x_nm']))} {mm(int(control['y_nm']))})"
                f"\n\t\t(end {mm(int(control['x_nm']) + radius_nm)} {mm(int(control['y_nm']))})"
                "\n\t\t(stroke (width 0.050000) (type default))"
                "\n\t\t(fill no)"
                "\n\t\t(layer \"Eco1.User\")"
                f"\n\t\t(uuid \"{object_uuid(result, route_id, 'control-point', index)}\")"
                "\n\t)"
            )
    return "".join(objects)


def diagnostic_board(
    source_text: str,
    result: NearResult,
    include_controls: bool,
    context_routes: list[dict],
) -> str:
    closing = source_text.rfind(")")
    if closing < 0:
        raise RuntimeError("source board has no closing expression")
    return (
        source_text[:closing]
        + diagnostic_objects(result, include_controls, context_routes)
        + "\n)\n"
    )


def render_svg_png(
    board: Path,
    svg: Path,
    png: Path,
    layers: list[str],
    kicad_cli: Path,
    inkscape: Path,
    width: int,
) -> None:
    run(
        [
            str(kicad_cli),
            "pcb",
            "export",
            "svg",
            "--mode-single",
            "--layers",
            ",".join([*layers, "Edge.Cuts"]),
            "--page-size-mode",
            "2",
            "--fit-page-to-board",
            "--exclude-drawing-sheet",
            "--drill-shape-opt",
            "2",
            "--output",
            str(svg),
            str(board),
        ]
    )
    run(
        [
            str(inkscape),
            str(svg),
            "--export-type=png",
            f"--export-width={width}",
            f"--export-filename={png}",
        ]
    )
    if not png.is_file() or png.stat().st_size == 0:
        raise RuntimeError(f"renderer did not produce {png}")


def safe_layer(layer: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", layer).replace(".", "_")


def write_control_closeup(source: Path, destination: Path) -> None:
    image = Image.open(source).convert("RGB")
    pixels = image.load()
    control_pixels: list[tuple[int, int]] = []
    # KiCad's Eco1.User plot colour is a pale green/teal. Copper is red/blue and the board edge is
    # neutral grey, so this condition remains stable through Inkscape antialiasing without baking
    # in one exact RGB value.
    for y in range(image.height):
        for x in range(image.width):
            red, green, blue = pixels[x, y]
            if green - red > 10 and abs(green - blue) < 30 and green > 100:
                control_pixels.append((x, y))
    if not control_pixels:
        raise RuntimeError(f"cannot locate spline controls in {source}")
    minimum_x = min(point[0] for point in control_pixels)
    minimum_y = min(point[1] for point in control_pixels)
    maximum_x = max(point[0] for point in control_pixels)
    maximum_y = max(point[1] for point in control_pixels)
    extent = max(maximum_x - minimum_x + 1, maximum_y - minimum_y + 1)
    margin = max(40, extent // 5)
    crop = (
        max(0, minimum_x - margin),
        max(0, minimum_y - margin),
        min(image.width, maximum_x + margin + 1),
        min(image.height, maximum_y + margin + 1),
    )
    image.crop(crop).save(destination)


def render(
    source: Path,
    trace: Path,
    output_dir: Path,
    kicad_cli: Path,
    inkscape: Path,
    mode: str,
    limit: int,
    width: int,
) -> list[dict]:
    source_text = source.read_text(encoding="utf-8")
    enabled_layers = set(enabled_copper_layers(source_text))
    results, context_routes = read_near_results(trace, limit)
    output_dir.mkdir(parents=True, exist_ok=True)
    manifest_results: list[dict] = []
    # Snap-packaged Inkscape has a private /tmp, so keep every intermediate beside the outputs.
    with tempfile.TemporaryDirectory(prefix=".flux-failed-route-render-", dir=output_dir) as temp:
        temporary = Path(temp)
        for result_index, result in enumerate(results):
            result_dir = output_dir / f"near-{result_index:02d}-window-{result.window}"
            result_dir.mkdir(parents=True, exist_ok=True)
            route_layers = sorted(
                {
                    str(point.get("layer", "F.Cu"))
                    for route in result.routes
                    for point in route.get("points", [])
                    if str(point.get("layer", "F.Cu")) in enabled_layers
                }
            )
            if not route_layers:
                route_layers = sorted(enabled_layers)
            files: list[str] = []
            if mode in {"copper", "both"}:
                board = temporary / f"near-{result_index:02d}-copper.kicad_pcb"
                board.write_text(
                    diagnostic_board(source_text, result, False, context_routes), encoding="utf-8"
                )
                for layer_index, layer in enumerate(route_layers):
                    stem = f"copper-{layer_index:02d}-{safe_layer(layer)}"
                    png = result_dir / f"{stem}.png"
                    render_svg_png(
                        board,
                        temporary / f"near-{result_index:02d}-{stem}.svg",
                        png,
                        [layer],
                        kicad_cli,
                        inkscape,
                        width,
                    )
                    files.append(str(png.relative_to(output_dir)))
            has_controls = any(route.get("control_points") for route in result.routes)
            if mode in {"spline", "both"} and has_controls:
                board = temporary / f"near-{result_index:02d}-spline-focus.kicad_pcb"
                board.write_text(
                    diagnostic_board(source_text, result, True, []), encoding="utf-8"
                )
                png = result_dir / "spline-controls.png"
                render_svg_png(
                    board,
                    temporary / f"near-{result_index:02d}-spline-focus.svg",
                    png,
                    [*route_layers, "Eco1.User"],
                    kicad_cli,
                    inkscape,
                    width,
                )
                files.append(str(png.relative_to(output_dir)))
                closeup = result_dir / "spline-controls-closeup.png"
                write_control_closeup(png, closeup)
                files.append(str(closeup.relative_to(output_dir)))
                overview_board = temporary / f"near-{result_index:02d}-overview.kicad_pcb"
                overview_board.write_text(
                    diagnostic_board(source_text, result, True, context_routes), encoding="utf-8"
                )
                overview_png = result_dir / "provisional-overview.png"
                render_svg_png(
                    overview_board,
                    temporary / f"near-{result_index:02d}-overview.svg",
                    overview_png,
                    [*route_layers, "Eco1.User"],
                    kicad_cli,
                    inkscape,
                    width,
                )
                files.append(str(overview_png.relative_to(output_dir)))
            manifest_results.append(
                {
                    "rank": result_index,
                    "window": result.window,
                    "assignment": result.assignment,
                    "assignment_count": result.assignment_count,
                    "candidate_rank": result.candidate_rank,
                    "route_count": len(result.routes),
                    "context_route_count": len(context_routes),
                    "topology_failures": result.topology_failures,
                    "mutual_clearance_failures": result.mutual_failures,
                    "route_ids": [route.get("id", "") for route in result.routes],
                    "files": files,
                }
            )
    manifest = {
        "schema": 1,
        "diagnostic_only": True,
        "authoritative_copper": False,
        "source_board": str(source),
        "telemetry_trace": str(trace),
        "ranking": (
            "feasibility-first near results plus reserved mutual-clearance, topology, and combined "
            "failure representatives; then most realized routes, spline rank, and recency"
        ),
        "results": manifest_results,
    }
    (output_dir / "failed-route-previews.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    return manifest_results


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--board", required=True, type=Path)
    parser.add_argument("--telemetry-record", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--kicad-cli", required=True, type=Path)
    parser.add_argument("--inkscape", required=True, type=Path)
    parser.add_argument("--mode", choices=("copper", "spline", "both"), default="both")
    parser.add_argument("--max-results", type=int, default=8)
    parser.add_argument("--width", type=int, default=2400)
    args = parser.parse_args()
    if args.max_results <= 0:
        raise RuntimeError("--max-results must be positive")
    if args.width <= 0:
        raise RuntimeError("--width must be positive")
    for path, label in (
        (args.board, "board"),
        (args.telemetry_record, "telemetry record"),
        (args.kicad_cli, "kicad-cli"),
        (args.inkscape, "Inkscape"),
    ):
        if not path.is_file():
            raise RuntimeError(f"missing {label}: {path}")
    results = render(
        args.board.resolve(),
        args.telemetry_record.resolve(),
        args.output_dir.resolve(),
        args.kicad_cli,
        args.inkscape,
        args.mode,
        args.max_results,
        args.width,
    )
    print(f"rendered {len(results)} non-authoritative near results to {args.output_dir.resolve()}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError) as error:
        print(f"FAIL {error}")
        raise SystemExit(1)
