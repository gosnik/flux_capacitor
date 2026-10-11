#!/usr/bin/env python3
"""Verify the progressively enabled real-board routing corpus without mutating it."""

from __future__ import annotations

import argparse
import datetime
import decimal
import hashlib
import json
import os
import platform
import re
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_json_once(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True)
        stream.write("\n")


def command_identity(command: list[str], timeout: int = 30) -> dict:
    try:
        completed = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        return {"command": command, "available": False, "diagnostic": str(error)}
    return {
        "command": command,
        "available": completed.returncode == 0,
        "returncode": completed.returncode,
        "stdout": completed.stdout.strip(),
        "stderr": completed.stderr.strip(),
    }


def artifact_inventory(root: Path) -> list[dict]:
    result = []
    for path in sorted(candidate for candidate in root.rglob("*") if candidate.is_file()):
        relative = path.relative_to(root).as_posix()
        if relative == "run-result.json":
            continue
        result.append({"path": relative, "bytes": path.stat().st_size, "sha256": sha256(path)})
    return result


def run_json(
    command: list[str], timeout: int, allowed_returncodes: tuple[int, ...] = (0,)
) -> dict:
    completed = subprocess.run(
        command,
        check=False,
        capture_output=True,
        text=True,
        timeout=timeout,
    )
    if completed.returncode not in allowed_returncodes:
        raise RuntimeError(
            f"command failed ({completed.returncode}): {' '.join(command)}\n"
            f"stdout:\n{completed.stdout}\nstderr:\n{completed.stderr}"
        )
    try:
        return json.loads(completed.stdout)
    except json.JSONDecodeError as error:
        raise RuntimeError(
            f"command did not produce JSON: {' '.join(command)}\n{completed.stdout}"
        ) from error


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def kicad_cli_command(executable: Path) -> list[str]:
    # The official KiCad 10 AppImage is a multi-call launcher. Its bare invocation reports the
    # launcher version and interprets `pcb` as an embedded executable name; select `kicad-cli`
    # explicitly before forwarding normal CLI arguments.
    command = [str(executable)]
    if executable.name.lower().endswith(".appimage"):
        command.append("kicad-cli")
    return command


def verify_hash(path: Path, expected: str, label: str) -> None:
    require(path.is_file(), f"missing {label}: {path}")
    actual = sha256(path)
    require(actual == expected, f"{label} hash mismatch: expected {expected}, got {actual}")


def render_copper_layers(
    board: Path, output_dir: Path, kicad_cli: Path, inkscape: Path
) -> None:
    script = Path(__file__).with_name("export_copper_pngs.py")
    completed = subprocess.run(
        [
            sys.executable,
            str(script),
            "--board",
            str(board),
            "--output-dir",
            str(output_dir),
            "--kicad-cli",
            str(kicad_cli),
            "--inkscape",
            str(inkscape),
        ],
        check=False,
        capture_output=True,
        text=True,
        timeout=180,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"copper render failed ({completed.returncode}) for {board}\n"
            f"stdout:\n{completed.stdout}\nstderr:\n{completed.stderr}"
        )


def render_failed_route_previews(
    board: Path,
    telemetry_record: Path,
    output_dir: Path,
    kicad_cli: Path,
    inkscape: Path,
    mode: str,
    max_results: int,
) -> None:
    script = Path(__file__).with_name("export_failed_route_pngs.py")
    completed = subprocess.run(
        [
            sys.executable,
            str(script),
            "--board",
            str(board),
            "--telemetry-record",
            str(telemetry_record),
            "--output-dir",
            str(output_dir),
            "--kicad-cli",
            str(kicad_cli),
            "--inkscape",
            str(inkscape),
            "--mode",
            mode,
            "--max-results",
            str(max_results),
        ],
        check=False,
        capture_output=True,
        text=True,
        timeout=300,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"analysis render failed ({completed.returncode}) for {board}\n"
            f"stdout:\n{completed.stdout}\nstderr:\n{completed.stderr}",
        )
    print(f"ANALYSIS {board.stem}: {completed.stdout.strip()}")


def stage_kicad_rule_context(source: Path, candidate: Path) -> list[dict]:
    """Bind a renamed candidate to the original project and optional custom rules.

    KiCad loads sidecars by the candidate's basename, not from the input board's
    directory. Missing context can turn real rule violations into false passes.
    Never rewrite a project, reduce rules, or overwrite a different sidecar.
    """
    context = []
    for suffix in (".kicad_pro", ".kicad_dru"):
        original = source.with_suffix(suffix)
        staged = candidate.with_suffix(suffix)
        if not original.exists():
            require(suffix != ".kicad_pro", f"missing source KiCad project: {original}")
            require(not staged.exists(), f"unexpected candidate rule context: {staged}")
            continue
        require(original.is_file(), f"source rule context is not a file: {original}")
        expected = sha256(original)
        if not staged.exists():
            with original.open("rb") as input_stream, staged.open("xb") as output_stream:
                shutil.copyfileobj(input_stream, output_stream)
        verify_hash(staged, expected, "candidate KiCad rule context")
        context.append({"source": str(original), "candidate": str(staged), "sha256": expected})
    return context


def require_kicad_drc(board: Path, report: Path, kicad_cli: Path, source: Path) -> None:
    context = stage_kicad_rule_context(source, board)
    write_json_once(report.with_suffix(".context.json"), {
        "schema": 1, "source": str(source), "candidate": str(board),
        "candidate_sha256": sha256(board), "rule_context": context,
        "severity": "error",
    })
    completed = subprocess.run(
        [
            *kicad_cli_command(kicad_cli),
            "pcb",
            "drc",
            "--format",
            "json",
            "--severity-error",
            "--exit-code-violations",
            "--output",
            str(report),
            str(board),
        ],
        check=False,
        capture_output=True,
        text=True,
        timeout=300,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"KiCad DRC rejected final copper ({completed.returncode}): {board}\n"
            f"stdout:\n{completed.stdout}\nstderr:\n{completed.stderr}"
        )


def audit_straight_copper_angles(candidate: Path) -> dict:
    """Audit exact decimal coordinates in exported KiCad segment records, independent of Flux."""
    source = candidate.read_text(encoding="utf-8")
    records = re.findall(r"\(segment\b((?:[^()]|\([^()]*\))*)\)", source)
    require(len(records) == len(re.findall(r"\(segment\s", source)),
            "cannot audit every exported track segment")
    counts = {"horizontal": 0, "vertical": 0, "diagonal_45": 0, "other": 0}
    for record in records:
        start = re.search(r"\(start\s+([^\s()]+)\s+([^\s()]+)\)", record)
        end = re.search(r"\(end\s+([^\s()]+)\s+([^\s()]+)\)", record)
        require(start is not None and end is not None, "track angle audit needs exact endpoints")
        x, y = map(decimal.Decimal, start.groups())
        xx, yy = map(decimal.Decimal, end.groups())
        dx, dy = abs(xx - x), abs(yy - y)
        kind = "horizontal" if dy == 0 else "vertical" if dx == 0 else "diagonal_45" if dx == dy else "other"
        counts[kind] += 1
    arcs = len(re.findall(r"\(arc\s", source))
    return {"tracks": len(records), "angles": counts, "arcs": arcs,
            "beziers": len(re.findall(r"\(gr_curve\s", source)),
            "passed": counts["other"] == 0 and arcs == 0 and not re.search(r"\(gr_curve\s", source)}


def path_inventory_acceptance(routed: dict, expected: dict, qualification: str) -> str | None:
    if routed.get("paths") == expected["global_paths"]:
        return "exact_global_path_count"
    # Physical suffix demands may add paths without altering any checked original demand.
    # This only admits the candidate to detailed validation; it never bypasses final DRC.
    scaffold = routed.get("copper_scaffold", {})
    count = scaffold.get("request_count", 0)
    if (qualification == "detailed_complete"
            and routed.get("routing_plan_complete") is True
            and scaffold.get("complete") is True
            and count >= expected["global_paths"]
            and scaffold.get("satisfied_connections") == count
            and scaffold.get("missing_connections") == 0
            and scaffold.get("physical_open_connections") == 0
            and scaffold.get("physical_unrouted_nets") == 0):
        return "expanded_physical_demand"
    return None


def detailed_copper_ready_for_final_drc(detailed: dict, expected: dict, candidate: Path) -> bool:
    # A written diagnostic preview is not evidence of detailed completion.
    return (
        detailed.get("complete") is True
        and detailed.get("validation_errors") == 0
        and detailed.get("open_connections") == 0
        and detailed.get("copper_violations") == 0
        and detailed.get("added_track_arcs", 0) >= expected.get("min_track_arcs", 0)
        and detailed.get("added_track_beziers", 0) >= expected.get("min_track_beziers", 0)
        and detailed.get("added_track_beziers", 0) <= expected.get("max_track_beziers", sys.maxsize)
        and detailed.get("added_track_arcs", 0) <= expected.get("max_track_arcs", sys.maxsize)
        and detailed.get("unrounded_non_junction_bends", sys.maxsize)
        <= expected.get("max_unrounded_non_junction_bends", sys.maxsize)
        and detailed.get("artifact_written") is True
        and candidate.is_file()
    )


def verify_board(
    flux_cli: Path,
    corpus: Path,
    artifacts: Path,
    kicad_cli: Path,
    inkscape: Path,
    board: dict,
    telemetry_level: str | None,
    telemetry_record_dir: Path | None,
    failed_route_png_dir: Path | None,
    failed_route_png_mode: str,
    max_failed_route_png_results: int,
    detailed_timeout_seconds: int,
    preflight_only: bool,
) -> dict:
    board_id = board["id"]
    source = corpus / f"{board_id}.kicad_pcb"
    project = corpus / f"{board_id}.kicad_pro"
    reference = corpus / "reference" / f"{board_id}.kicad_pcb"
    license_path = corpus / board["license_file"]

    verify_hash(source, board["board_sha256"], f"{board_id} stripped board")
    verify_hash(reference, board["reference_sha256"], f"{board_id} routed reference")
    verify_hash(project, board["project_sha256"], f"{board_id} project")
    verify_hash(license_path, board["license_sha256"], f"{board_id} license")
    before = sha256(source)
    reference_before = sha256(reference)

    inspected = run_json([str(flux_cli), "inspect-kicad", str(source)], timeout=30)
    require(inspected.get("strict") is True, f"{board_id}: import is not strict")
    require(inspected.get("lossy") is False, f"{board_id}: import is lossy")
    expected = board["expected"]
    for field in ("footprints", "nets", "tracks", "vias", "zones"):
        require(
            inspected.get(field) == expected[field],
            f"{board_id}: expected {field}={expected[field]}, got {inspected.get(field)}",
        )
    require(
        inspected.get("core_validation_errors", sys.maxsize)
        <= expected["max_core_validation_errors"],
        f"{board_id}: structural errors regressed from the recorded bound",
    )

    board_result = {
        "id": board_id,
        "level": board["level"],
        "declared_status": board.get("manifest_status", board["status"]),
        "effective_status": board["status"],
        "route_locked_diagnostic": board.get("route_locked_diagnostic", False),
        "qualification": board.get("qualification"),
        "source_revision": inspected.get("source_revision"),
        "source_sha256": before,
        "reference_sha256": reference_before,
        "inspection": inspected,
        "global": None,
        "detailed": None,
        "qualified": None,
    }

    if preflight_only:
        require(sha256(source) == before, f"{board_id}: source board was modified")
        require(sha256(reference) == reference_before, f"{board_id}: routed reference was modified")
        print(f"PREFLIGHT {board_id}: source and strict import identity verified")
        return board_result

    if board["status"] in {"qualified", "unlocked"}:
        if board["status"] == "qualified":
            require(
                inspected.get("open_connections") == expected["open_connections"],
                f"{board_id}: connection inventory changed",
            )
        router_args = board.get("router_args", [])
        require(
            isinstance(router_args, list) and all(isinstance(arg, str) for arg in router_args),
            f"{board_id}: router_args must be a list of strings",
        )
        probe_timeout = board.get("probe_timeout_seconds", 180)
        require(isinstance(probe_timeout, int) and probe_timeout > 0,
                f"{board_id}: probe_timeout_seconds must be a positive integer")
        tier_dir = artifacts / f"tier-{board['level']}-{board_id}"
        tier_dir.mkdir(parents=True, exist_ok=True)
        analysis_dir = (
            failed_route_png_dir / board_id
            if failed_route_png_dir is not None
            else tier_dir / "analysis"
        )
        # Analysis images are part of the tier contract. Regional PH controls are emitted only at
        # diagnostic level, so a caller may increase this to trace but may not reduce it to normal.
        effective_telemetry_level = (
            "trace" if telemetry_level == "trace" else "diagnostic"
        )
        effective_telemetry_record_dir = (
            telemetry_record_dir if telemetry_record_dir is not None else tier_dir / "telemetry"
        )
        route_telemetry_args: list[str] = [
            "--telemetry-level",
            effective_telemetry_level,
        ]
        route_telemetry_record: Path | None = None
        if effective_telemetry_record_dir is not None:
            effective_telemetry_record_dir.mkdir(parents=True, exist_ok=True)
            route_telemetry_record = effective_telemetry_record_dir / f"{board_id}-global.jsonl"
            route_telemetry_args.extend(
                ["--telemetry-record", str(route_telemetry_record)]
            )
        routing_timeout_args: list[str] = []
        if board.get("routing_timeout_seconds") is not None:
            routing_timeout_args = [
                "--routing-timeout-ms",
                str(board["routing_timeout_seconds"] * 1000),
            ]
        global_started = time.monotonic()
        try:
            routed = run_json(
                [
                    str(flux_cli),
                    "route-kicad",
                    str(source),
                    "--max-router-iterations",
                    str(board.get("probe_iterations", 1)),
                    "--max-expansions",
                    "50000",
                    "--max-static-seed-workers",
                    "2",
                    "--search-diversification",
                    *routing_timeout_args,
                    *router_args,
                    *route_telemetry_args,
                ],
                timeout=probe_timeout,
                # route-kicad uses 3 for a structured invalid-input result. An
                # unlocked tier still has to report that result so its blocker is
                # visible; a qualified tier remains strict about exit status.
                # Exit 3 is a structured incomplete/invalid routing result. Always parse it so
                # mandatory analysis PNGs are exported before the qualification gate reports the
                # regression. The strict qualified-tier failure remains enforced below.
                allowed_returncodes=(0, 3),
            )
        except subprocess.TimeoutExpired:
            if (
                route_telemetry_record is not None
                and route_telemetry_record.is_file()
            ):
                render_failed_route_previews(
                    source,
                    route_telemetry_record,
                    analysis_dir / "global",
                    kicad_cli,
                    inkscape,
                    failed_route_png_mode,
                    max_failed_route_png_results,
                )
            raise
        global_duration_seconds = time.monotonic() - global_started
        global_record = {
            "duration_seconds": global_duration_seconds,
            "command_configuration": {
                "max_router_iterations": board.get("probe_iterations", 1),
                "max_expansions": 50000,
                "max_static_seed_workers": 2,
                "search_diversification": True,
                "routing_timeout_seconds": board.get("routing_timeout_seconds"),
                "router_args": router_args,
                "telemetry_level": effective_telemetry_level,
            },
            "result": routed,
        }
        write_json_once(tier_dir / "global-result.json", global_record)
        board_result["global"] = global_record
        global_qualified = (
            routed.get("complete") is True
            and routed.get("unrouted_nets") == 0
            and routed.get("total_overflow") == 0
            and routed.get("static_seed_terminals_without_seed") == 0
        )
        if route_telemetry_record is not None and route_telemetry_record.is_file():
            render_failed_route_previews(
                source,
                route_telemetry_record,
                analysis_dir / "global",
                kicad_cli,
                inkscape,
                failed_route_png_mode,
                max_failed_route_png_results,
            )
        qualification = board.get("qualification")
        require(
            qualification in {"global_complete", "detailed_complete"},
            f"{board_id}: unsupported qualification {qualification}",
        )
        planning_ready = global_qualified
        if qualification == "detailed_complete":
            planning_ready = (
                routed.get("routing_plan_complete") is True
                and routed.get("unrouted_nets") == 0
                and routed.get("static_seed_terminals_without_seed") == 0
            )
        if board["status"] == "qualified":
            require(planning_ready, f"{board_id}: qualified routing plan regressed")
            inventory = path_inventory_acceptance(routed, expected, qualification)
            board_result["path_inventory_acceptance"] = inventory
            require(
                inventory is not None,
                f"{board_id}: expected {expected['global_paths']} paths or a certified physical expansion, "
                f"got {routed.get('paths')}",
            )

        # A global checkpoint is not renderable copper. When a manifest tier advances to exact
        # detailed completion, require its accepted board and publish every enabled copper layer
        # as a PNG. Failed/incomplete transactions never produce misleading "final" images.
        detailed_qualified = False
        if qualification == "detailed_complete" and planning_ready:
            final_board = tier_dir / f"{board_id}.kicad_pcb"
            # Flux deliberately refuses to overwrite output boards. Route into a fresh temporary
            # path, validate that exact artifact, and publish it only after every acceptance gate
            # passes. This keeps repeated corpus runs deterministic without deleting prior output
            # before a replacement has been certified.
            with tempfile.TemporaryDirectory(prefix=".flux-candidate-", dir=tier_dir) as temporary:
                candidate_board = Path(temporary) / f"{board_id}.kicad_pcb"
                detail_telemetry_args: list[str] = []
                if effective_telemetry_level is not None:
                    detail_telemetry_args.extend(
                        ["--telemetry-level", effective_telemetry_level]
                    )
                detail_telemetry_record: Path | None = None
                if effective_telemetry_record_dir is not None:
                    effective_telemetry_record_dir.mkdir(parents=True, exist_ok=True)
                    detail_telemetry_record = (
                        effective_telemetry_record_dir / f"{board_id}-detail.jsonl"
                    )
                    detail_telemetry_args.extend(
                        [
                            "--telemetry-record",
                            str(detail_telemetry_record),
                        ]
                    )
                detailed_started = time.monotonic()
                try:
                    detailed = run_json(
                        [
                            str(flux_cli),
                            "detail-kicad",
                            str(source),
                            "--max-router-iterations",
                            str(
                                board.get(
                                    "detail_iterations", board.get("probe_iterations", 30)
                                )
                            ),
                            "--max-expansions",
                            "50000",
                            "--max-static-seed-workers",
                            "2",
                            "--search-diversification",
                            *routing_timeout_args,
                            *router_args,
                            *detail_telemetry_args,
                            *(["--arcs"] if board.get("track_geometry") == "curved" else []),
                            "--write-incomplete-preview",
                            str(candidate_board),
                        ],
                        timeout=detailed_timeout_seconds,
                        # Failed qualified-board detail still emits useful structured metrics.
                        # Record them before the unchanged qualification gates reject the run.
                        allowed_returncodes=(0, 3, 5, 7),
                    )
                except subprocess.TimeoutExpired:
                    if (
                        detail_telemetry_record is not None
                        and detail_telemetry_record.is_file()
                    ):
                        render_failed_route_previews(
                            source,
                            detail_telemetry_record,
                            analysis_dir / "detail",
                            kicad_cli,
                            inkscape,
                            failed_route_png_mode,
                            max_failed_route_png_results,
                        )
                    raise
                detailed_duration_seconds = time.monotonic() - detailed_started
                detailed_record = {
                    "duration_seconds": detailed_duration_seconds,
                    "command_configuration": {
                        "max_router_iterations": board.get(
                            "detail_iterations", board.get("probe_iterations", 30)
                        ),
                        "max_expansions": 50000,
                        "max_static_seed_workers": 2,
                        "search_diversification": True,
                        "routing_timeout_seconds": board.get("routing_timeout_seconds"),
                        "outer_timeout_seconds": detailed_timeout_seconds,
                        "router_args": router_args,
                        "telemetry_level": effective_telemetry_level,
                        "arcs": board.get("track_geometry") == "curved",
                        "write_incomplete_preview": True,
                    },
                    "result": detailed,
                }
                write_json_once(tier_dir / "detail-result.json", detailed_record)
                board_result["detailed"] = detailed_record
                detailed_qualified = detailed_copper_ready_for_final_drc(
                    detailed, expected, candidate_board)
                if detail_telemetry_record is not None and detail_telemetry_record.is_file():
                    render_failed_route_previews(
                        source,
                        detail_telemetry_record,
                        analysis_dir / "detail",
                        kicad_cli,
                        inkscape,
                        failed_route_png_mode,
                        max_failed_route_png_results,
                    )
                if candidate_board.is_file() and board.get("track_geometry") == "straight":
                    angle_audit = audit_straight_copper_angles(candidate_board)
                    write_json_once(tier_dir / "straight-angle-audit.json", angle_audit)
                    board_result["straight_angle_audit"] = angle_audit
                    detailed_qualified = detailed_qualified and angle_audit["passed"]
                if detailed_qualified:
                    try:
                        require_kicad_drc(candidate_board, tier_dir / "kicad-drc.json",
                                          kicad_cli, source)
                    except RuntimeError:
                        # Preserve rejected copper and its exact rules before the temporary
                        # export directory is removed. It is diagnostic, never a final board.
                        rejected = analysis_dir / "kicad-drc-rejected"
                        rejected.mkdir(parents=True, exist_ok=False)
                        rejected_board = rejected / candidate_board.name
                        shutil.copy2(candidate_board, rejected_board)
                        stage_kicad_rule_context(source, rejected_board)
                        raise
                    shutil.copy2(candidate_board, final_board)
                    stage_kicad_rule_context(source, final_board)
                    render_copper_layers(final_board, tier_dir / "copper", kicad_cli, inkscape)
                    print(f"RENDER {board_id}: final copper PNGs in {tier_dir / 'copper'}")
                elif candidate_board.is_file():
                    # Preserve the actual rejected candidate, not a reconstruction from spline
                    # telemetry. A diagnostic DRC pass cannot override the curve/completion gate.
                    rejected = analysis_dir / "detailed-rejected"
                    rejected.mkdir(parents=True, exist_ok=False)
                    rejected_board = rejected / candidate_board.name
                    shutil.copy2(candidate_board, rejected_board)
                    render_copper_layers(
                        rejected_board, rejected / "copper", kicad_cli, inkscape
                    )
                    diagnostic_drc = {"authoritative": False, "tier_qualified": False,
                                      "reason": detailed.get("diagnostic"), "passed": False}
                    try:
                        require_kicad_drc(rejected_board, rejected / "kicad-drc.json",
                                          kicad_cli, source)
                        diagnostic_drc["passed"] = True
                    except (RuntimeError, subprocess.TimeoutExpired) as error:
                        diagnostic_drc["error"] = str(error)
                    write_json_once(rejected / "diagnostic-status.json", diagnostic_drc)

        qualified = global_qualified if qualification == "global_complete" else detailed_qualified
        board_result["qualified"] = qualified
        if board["status"] == "qualified":
            require(qualified, f"{board_id}: qualified {qualification} route regressed")
            suffix = "zero-overflow global plan"
            if qualification == "detailed_complete":
                suffix = "exact copper and external KiCad DRC"
            print(f"PASS {board_id}: {routed['paths']} paths, {suffix}")
        else:
            if board.get("route_locked_diagnostic", False):
                print(
                    f"DIAGNOSTIC {board_id}: qualified={qualified}; "
                    f"stop={routed.get('stop_reason')}, paths={routed.get('paths')}, "
                    f"unrouted={routed.get('unrouted_nets')}, overflow={routed.get('total_overflow')}"
                )
            else:
                require(
                    not qualified,
                    f"{board_id}: unlocked tier now qualifies; permanently promote it to qualified "
                    "and activate the next locked tier in manifest.json",
                )
                require(routed.get("diagnostic"), f"{board_id}: incomplete probe has no diagnosis")
                print(
                    f"PROBE {board_id}: incomplete as expected; "
                    f"stop={routed.get('stop_reason')}, paths={routed.get('paths')}, "
                    f"unrouted={routed.get('unrouted_nets')}, overflow={routed.get('total_overflow')}"
                )
    else:
        print(
            f"PASS {board_id}: strict import gate "
            f"({inspected['core_validation_errors']} current structural blockers)"
        )
        board_result["qualified"] = False

    require(sha256(source) == before, f"{board_id}: source board was modified")
    require(sha256(reference) == reference_before, f"{board_id}: routed reference was modified")
    return board_result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--flux-cli", required=True, type=Path)
    parser.add_argument("--corpus", required=True, type=Path)
    parser.add_argument(
        "--level", default="unlocked", help="maximum inspection level, 'all', or 'unlocked'"
    )
    parser.add_argument("--board", action="append", default=[])
    parser.add_argument(
        "--route-locked",
        action="store_true",
        help="diagnostically route selected locked boards without changing manifest promotion status",
    )
    parser.add_argument("--track-geometry", choices=("straight", "curved"), default="curved")
    parser.add_argument("--artifacts", type=Path, default=Path("build/routing-corpus-artifacts"))
    parser.add_argument("--kicad-cli", type=Path)
    parser.add_argument("--inkscape", type=Path)
    parser.add_argument(
        "--telemetry-level", choices=("normal", "diagnostic", "trace")
    )
    parser.add_argument("--telemetry-record-dir", type=Path)
    parser.add_argument(
        "--telemetry-listen",
        help="optional telemetry listener ADDRESS:PORT forwarded to routing commands",
    )
    parser.add_argument(
        "--failed-route-png-dir",
        type=Path,
        help=(
            "override the default artifact root for mandatory non-authoritative analysis PNGs"
        ),
    )
    parser.add_argument(
        "--failed-route-png-mode", choices=("copper", "spline", "both"), default="both"
    )
    parser.add_argument("--max-failed-route-png-results", type=int, default=8)
    parser.add_argument(
        "--preflight-only",
        action="store_true",
        help="verify identities and emit run manifests without starting routing",
    )
    parser.add_argument(
        "--iteration-budget", type=int,
        help="experimental override for both planning and detailed routing iterations; leaves the corpus manifest unchanged",
    )
    parser.add_argument("--physical-feedback-retries", type=int)
    parser.add_argument("--feedback-shadow-policy", choices=("enforced", "advisory"))
    parser.add_argument("--oracle-first-feedback", action="store_true")
    parser.add_argument("--sampled-control", action="store_true",
                        help="diagnostic sampled-graph control; removes medial topology flags but keeps dimensions and DRC rules")
    parser.add_argument("--strict-medial-control", action="store_true",
                        help="diagnostic backend control: replace direct-medial with strict-medial while retaining physical resources, pitch, rules and repair policy")
    parser.add_argument("--grid-pitch-nm-control", type=int,
                        help="diagnostic sampling-pitch override; keeps copper dimensions and source rules unchanged")
    parser.add_argument("--dense-candidate-grid", action="store_true",
                        help="experimental dense planar samples; requires enforced physical clearance conflicts")
    strength_policy = parser.add_mutually_exclusive_group()
    strength_policy.add_argument("--full-strength-first-pass", action="store_true")
    strength_policy.add_argument("--no-full-strength-first-pass", action="store_true",
                                 help="use legacy scheduling for controlled comparisons")
    parser.add_argument("--frozen-copper-context", action="store_true")
    parser.add_argument("--local-channel-occupancy", action="store_true")
    parser.add_argument("--shared-channel-copper", action="store_true")
    parser.add_argument("--centerline-channel-cuts", action="store_true")
    parser.add_argument("--max-regional-bundle-states", type=int)
    parser.add_argument("--max-straight-portfolio-checks", type=int,
                        help="experimental bound on physical checks of retained straight bundle alternatives; leaves the corpus manifest unchanged")
    parser.add_argument("--expand-frozen-copper-scope", action="store_true")
    parser.add_argument("--prioritize-frozen-scope-expansion", action="store_true")
    parser.add_argument("--planning-timeout-seconds", type=int,
                        help="diagnostic planning timeout override (1..86400); does not change the corpus manifest or qualify its normal runtime budget")
    parser.add_argument(
        "--detailed-timeout-seconds",
        type=int,
        help="diagnostic detailed-routing harness timeout (1..86400; default: 2400)",
    )
    parser.add_argument(
        "--routing-timeout-seconds",
        type=int,
        help="route-kicad/detail-kicad internal timeout (1..86400), forwarded as milliseconds",
    )
    args = parser.parse_args()
    if args.sampled_control and args.strict_medial_control:
        parser.error("sampled and strict-medial controls are mutually exclusive")
    if args.max_regional_bundle_states is not None and not 1 <= args.max_regional_bundle_states <= 1000000:
        parser.error("--max-regional-bundle-states must be in 1..1000000")
    if args.max_straight_portfolio_checks is not None and not 0 <= args.max_straight_portfolio_checks <= 4096:
        parser.error("--max-straight-portfolio-checks must be in 0..4096")
    if args.iteration_budget is not None and args.iteration_budget < 1:
        parser.error("--iteration-budget must be positive")
    if args.grid_pitch_nm_control is not None and args.grid_pitch_nm_control < 1:
        parser.error("--grid-pitch-nm-control must be positive")
    if args.planning_timeout_seconds is not None and not 1 <= args.planning_timeout_seconds <= 86400:
        parser.error("--planning-timeout-seconds must be 1..86400")
    if args.detailed_timeout_seconds is not None and not 1 <= args.detailed_timeout_seconds <= 86400:
        parser.error("--detailed-timeout-seconds must be 1..86400")
    if args.routing_timeout_seconds is not None and not 1 <= args.routing_timeout_seconds <= 86400:
        parser.error("--routing-timeout-seconds must be 1..86400")
    if args.oracle_first_feedback and args.full_strength_first_pass:
        parser.error("oracle-first and full-strength first-pass are alternative scheduling experiments")
    if args.physical_feedback_retries is not None and (
        args.iteration_budget is None or not 0 <= args.physical_feedback_retries < args.iteration_budget
        or args.physical_feedback_retries > 64
    ):
        parser.error("feedback retries must be 0..64 and smaller than an explicit iteration budget")

    flux_cli = args.flux_cli.resolve()
    corpus = args.corpus.resolve()
    artifacts = args.artifacts.resolve()
    telemetry_record_dir = (
        args.telemetry_record_dir.resolve() if args.telemetry_record_dir is not None else None
    )
    failed_route_png_dir = (
        args.failed_route_png_dir.resolve() if args.failed_route_png_dir is not None else None
    )
    kicad_cli = args.kicad_cli or Path(shutil.which("kicad-cli") or "")
    inkscape = args.inkscape or Path(shutil.which("inkscape") or "")
    require(flux_cli.is_file(), f"missing flux-cli: {flux_cli}")
    if not args.preflight_only:
        require(kicad_cli.is_file(), f"missing kicad-cli: {kicad_cli}")
        require(inkscape.is_file(), f"missing Inkscape: {inkscape}")
    require(
        args.max_failed_route_png_results > 0,
        "--max-failed-route-png-results must be positive",
    )
    manifest_path = corpus / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    require(manifest.get("schema") == 1, "unsupported routing corpus manifest schema")
    boards = manifest["boards"]
    require(len({board["id"] for board in boards}) == len(boards), "duplicate board ID")
    require(
        [board["level"] for board in boards] == sorted(board["level"] for board in boards),
        "boards must remain ordered by progressive level",
    )
    statuses = [board["status"] for board in boards]
    require(set(statuses) <= {"qualified", "unlocked", "locked"}, "invalid tier status")
    require(statuses.count("unlocked") <= 1, "at most one routing tier may be unlocked")
    if "unlocked" in statuses:
        unlocked_index = statuses.index("unlocked")
        require(
            all(status == "qualified" for status in statuses[:unlocked_index]),
            "every tier preceding the unlocked tier must be qualified",
        )
        require(
            all(status == "locked" for status in statuses[unlocked_index + 1 :]),
            "every tier after the unlocked tier must remain locked",
        )

    selected_ids = set(args.board)
    if selected_ids:
        known = {board["id"] for board in boards}
        require(selected_ids <= known, f"unknown board IDs: {sorted(selected_ids - known)}")
        selected = [board for board in boards if board["id"] in selected_ids]
    else:
        if args.level == "unlocked":
            maximum = max(board["level"] for board in boards if board["status"] != "locked")
        elif args.level == "qualified":
            maximum = max(board["level"] for board in boards if board["status"] == "qualified")
        else:
            maximum = (
                max(board["level"] for board in boards)
                if args.level == "all"
                else int(args.level)
            )
        selected = [board for board in boards if board["level"] <= maximum]
    require(selected, "no corpus boards selected")
    if args.iteration_budget is not None:
        selected = [dict(board, probe_iterations=args.iteration_budget,
                         detail_iterations=args.iteration_budget) for board in selected]
    selected = [
        dict(
            board,
            manifest_status=board["status"],
            expected=dict(board["expected"]),
            track_geometry=args.track_geometry,
        )
        for board in selected
    ]
    if args.route_locked:
        selected = [
            dict(board, status="unlocked", route_locked_diagnostic=True)
            if board["manifest_status"] == "locked"
            else board
            for board in selected
        ]
    for board in selected:
        if args.track_geometry == "curved":
            board["expected"].pop("max_track_arcs", None)
            board["expected"].update(board.get("curved_expected", {}))
            if "min_track_arcs" in board["expected"]:
                board["expected"]["min_track_beziers"] = board["expected"].pop("min_track_arcs")
    experiment_args = ["--curved-tracks"] if args.track_geometry == "curved" else ["--straight-tracks"]
    if args.local_channel_occupancy:
        experiment_args += ["--local-channel-occupancy"]
    if args.shared_channel_copper:
        experiment_args += ["--shared-channel-copper"]
    if args.centerline_channel_cuts:
        experiment_args += ["--centerline-channel-cuts"]
    if args.max_regional_bundle_states is not None:
        experiment_args += ["--max-regional-bundle-states", str(args.max_regional_bundle_states)]
    if args.max_straight_portfolio_checks is not None:
        experiment_args += ["--max-straight-portfolio-checks", str(args.max_straight_portfolio_checks)]
    if args.dense_candidate_grid:
        experiment_args += ["--dense-candidate-grid"]
    if args.sampled_control:
        medial_flags = {"--direct-medial", "--strict-medial", "--negotiate-lane-conflicts",
                        "--physical-overflow-objective", "--physical-overflow-history",
                        "--physical-envelope-ordering", "--rollback-failed-hard-capacity-ripup",
                        "--defer-hard-capacity-ripup-victims-to-pass-tail",
                        "--protect-recovered-hard-capacity-ripup-nets"}
        medial_values = {"--hard-capacity-start-iteration",
                         "--max-hard-capacity-ripup-attempts-per-pass",
                         "--max-hard-capacity-ripup-nets-per-attempt",
                         "--max-hard-capacity-conflict-set-states-per-attempt",
                         "--max-hard-capacity-blocker-reuse-invalidations-per-failed-net"}
        for board in selected:
            kept = []
            iterator = iter(board.get("router_args", []))
            for arg in iterator:
                if arg in medial_values:
                    next(iterator)
                elif arg not in medial_flags:
                    kept.append(arg)
            board["router_args"] = kept
    if args.strict_medial_control:
        selected = [dict(board, router_args=["--strict-medial" if arg == "--direct-medial"
                                            else arg for arg in board.get("router_args", [])])
                    for board in selected]
    if args.grid_pitch_nm_control is not None:
        for board in selected:
            router_args = list(board.get("router_args", []))
            if "--grid-pitch-nm" in router_args:
                router_args[router_args.index("--grid-pitch-nm") + 1] = str(args.grid_pitch_nm_control)
            else:
                router_args += ["--grid-pitch-nm", str(args.grid_pitch_nm_control)]
            board["router_args"] = router_args
    if args.planning_timeout_seconds is not None:
        selected = [dict(board, probe_timeout_seconds=args.planning_timeout_seconds)
                    for board in selected]
    if args.routing_timeout_seconds is not None:
        selected = [dict(board, routing_timeout_seconds=args.routing_timeout_seconds)
                    for board in selected]
    detailed_timeout_seconds = args.detailed_timeout_seconds or 2400
    if args.physical_feedback_retries is not None:
        experiment_args += ["--physical-feedback-retries", str(args.physical_feedback_retries)]
    if args.feedback_shadow_policy is not None:
        experiment_args += ["--feedback-shadow-policy", args.feedback_shadow_policy]
    if args.oracle_first_feedback:
        experiment_args += ["--oracle-first-feedback"]
    if args.full_strength_first_pass:
        experiment_args += ["--full-strength-first-pass"]
    if args.no_full_strength_first_pass:
        experiment_args += ["--no-full-strength-first-pass"]
    if args.frozen_copper_context:
        experiment_args += ["--frozen-copper-context"]
    if args.expand_frozen_copper_scope:
        experiment_args += ["--expand-frozen-copper-scope"]
    if args.prioritize_frozen_scope_expansion:
        experiment_args += ["--prioritize-frozen-scope-expansion"]
    if args.telemetry_listen is not None:
        experiment_args += ["--telemetry-listen", args.telemetry_listen]
    selected = [dict(board, router_args=[*board.get("router_args", []), *experiment_args])
                for board in selected]

    artifacts.mkdir(parents=True, exist_ok=True)
    system_info = run_json([str(flux_cli), "system-info", "--json"], timeout=30)
    capabilities = run_json([str(flux_cli), "capabilities", "--json"], timeout=30)
    metadata = {
        "schema": "flux_routing_baseline",
        "schema_version": 1,
        "created_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "mode": "preflight" if args.preflight_only else "routing",
        "git_revision": command_identity(["git", "-C", str(Path(__file__).resolve().parents[2]), "rev-parse", "HEAD"]),
        "invocation": [str(Path(sys.argv[0]).resolve()), *sys.argv[1:]],
        "environment": {
            "platform": platform.platform(),
            "python": sys.version,
            "flux_vulkan_device_override": os.environ.get("FLUX_VULKAN_DEVICE"),
            "flux": system_info,
            "kicad_cli": command_identity([*kicad_cli_command(kicad_cli), "--version"])
            if kicad_cli.is_file()
            else {"available": False},
            "inkscape": command_identity([str(inkscape), "--version"])
            if inkscape.is_file()
            else {"available": False},
        },
        "executable": {
            "path": str(flux_cli),
            "sha256": sha256(flux_cli),
        },
        "capability_contract": {
            "schema": capabilities.get("schema"),
            "schema_version": capabilities.get("schema_version"),
            "reference": capabilities.get("reference"),
            "sha256": hashlib.sha256(
                json.dumps(capabilities, sort_keys=True, separators=(",", ":")).encode("utf-8")
            ).hexdigest(),
        },
        "corpus": {
            "path": str(corpus),
            "manifest_sha256": sha256(manifest_path),
            "source_corpus": manifest.get("source_corpus"),
        },
        "selection": {
            "level": args.level,
            "explicit_boards": args.board,
            "iteration_budget_override": args.iteration_budget,
            "track_geometry": args.track_geometry,
            "sampled_control_override": args.sampled_control,
            "strict_medial_control_override": args.strict_medial_control,
            "grid_pitch_nm_control_override": args.grid_pitch_nm_control,
            "planning_timeout_override_seconds": args.planning_timeout_seconds,
            "detailed_timeout_override_seconds": args.detailed_timeout_seconds,
            "effective_detailed_timeout_seconds": detailed_timeout_seconds,
            "routing_timeout_override_seconds": args.routing_timeout_seconds,
            "route_locked_override": args.route_locked,
            "telemetry_listen_override": args.telemetry_listen,
            "max_straight_portfolio_checks_override": args.max_straight_portfolio_checks,
            "boards": [
                {
                    "id": board["id"],
                    "level": board["level"],
                    "status": board["manifest_status"],
                    "effective_status": board["status"],
                    "route_locked_diagnostic": board.get("route_locked_diagnostic", False),
                    "qualification": board.get("qualification"),
                    "source_sha256": board["board_sha256"],
                    "project_sha256": board["project_sha256"],
                    "reference_sha256": board["reference_sha256"],
                    "router_args": board.get("router_args", []),
                    "effective_planning_timeout_seconds": board.get("probe_timeout_seconds", 180),
                    "effective_detailed_timeout_seconds": detailed_timeout_seconds,
                    "effective_routing_timeout_seconds": board.get("routing_timeout_seconds"),
                }
                for board in selected
            ],
        },
        "artifact_contract": {
            "telemetry_level": args.telemetry_level,
            "failed_route_png_mode": args.failed_route_png_mode,
            "max_failed_route_png_results": args.max_failed_route_png_results,
            "analysis_pngs_required_for_routing": True,
            "telemetry_required_for_routing": True,
            "source_boards_immutable": True,
            "files_write_once": True,
        },
    }
    write_json_once(artifacts / "run-metadata.json", metadata)

    board_results = []
    try:
        for board in selected:
            board_results.append(
                verify_board(
                    flux_cli,
                    corpus,
                    artifacts,
                    kicad_cli,
                    inkscape,
                    board,
                    args.telemetry_level,
                    telemetry_record_dir,
                    failed_route_png_dir,
                    args.failed_route_png_mode,
                    args.max_failed_route_png_results,
                    detailed_timeout_seconds,
                    args.preflight_only,
                )
            )
    except (RuntimeError, ValueError, subprocess.TimeoutExpired) as error:
        write_json_once(
            artifacts / "run-result.json",
            {
                "schema": "flux_routing_baseline_result",
                "schema_version": 1,
                "status": "failed",
                "diagnostic": str(error),
                "boards": board_results,
                "artifacts": artifact_inventory(artifacts),
            },
        )
        raise
    write_json_once(
        artifacts / "run-result.json",
        {
            "schema": "flux_routing_baseline_result",
            "schema_version": 1,
            "status": "preflight_complete" if args.preflight_only else "complete",
            "diagnostic": "",
            "boards": board_results,
            "artifacts": artifact_inventory(artifacts),
        },
    )
    assessed = sum(result["qualified"] is not None for result in board_results)
    qualified = sum(result["qualified"] is True for result in board_results)
    qualification_summary = (
        f"{qualified}/{assessed} assessed boards qualified"
        if assessed
        else "routing qualification not assessed"
    )
    print(
        f"{len(selected)}/{len(selected)} selected corpus checks passed; "
        f"{qualification_summary}"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, subprocess.TimeoutExpired) as error:
        print(f"FAIL {error}", file=sys.stderr)
        raise SystemExit(1)
