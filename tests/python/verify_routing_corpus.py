#!/usr/bin/env python3
"""Verify the progressively enabled real-board routing corpus without mutating it."""

from __future__ import annotations

import argparse
import datetime
import hashlib
import json
import os
import platform
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
            str(kicad_cli),
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
        "declared_status": board["status"],
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
                    *router_args,
                    *route_telemetry_args,
                ],
                timeout=probe_timeout,
                # route-kicad uses 3 for a structured invalid-input result. An
                # unlocked tier still has to report that result so its blocker is
                # visible; a qualified tier remains strict about exit status.
                allowed_returncodes=(0, 3) if board["status"] == "unlocked" else (0,),
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
            require(
                routed.get("paths") == expected["global_paths"],
                f"{board_id}: expected {expected['global_paths']} paths, "
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
                            *router_args,
                            *detail_telemetry_args,
                            "--arcs",
                            str(candidate_board),
                        ],
                        timeout=2400,
                        allowed_returncodes=(0, 3, 5)
                        if board["status"] == "unlocked"
                        else (0,),
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
                        "router_args": router_args,
                        "telemetry_level": effective_telemetry_level,
                        "arcs": True,
                    },
                    "result": detailed,
                }
                write_json_once(tier_dir / "detail-result.json", detailed_record)
                board_result["detailed"] = detailed_record
                detailed_qualified = (
                    detailed.get("complete") is True
                    and detailed.get("validation_errors") == 0
                    and detailed.get("open_connections") == 0
                    and detailed.get("copper_violations") == 0
                    and detailed.get("added_track_arcs", 0)
                    >= expected.get("min_track_arcs", 0)
                    and detailed.get("unrounded_non_junction_bends", sys.maxsize)
                    <= expected.get("max_unrounded_non_junction_bends", sys.maxsize)
                    and detailed.get("artifact_written") is True
                    and candidate_board.is_file()
                )
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

        qualified = global_qualified if qualification == "global_complete" else detailed_qualified
        board_result["qualified"] = qualified
        if board["status"] == "qualified":
            require(qualified, f"{board_id}: qualified {qualification} route regressed")
            suffix = "zero-overflow global plan"
            if qualification == "detailed_complete":
                suffix = "exact copper and external KiCad DRC"
            print(f"PASS {board_id}: {routed['paths']} paths, {suffix}")
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
    parser.add_argument("--artifacts", type=Path, default=Path("build/routing-corpus-artifacts"))
    parser.add_argument("--kicad-cli", type=Path)
    parser.add_argument("--inkscape", type=Path)
    parser.add_argument(
        "--telemetry-level", choices=("normal", "diagnostic", "trace")
    )
    parser.add_argument("--telemetry-record-dir", type=Path)
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
    args = parser.parse_args()

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

    artifacts.mkdir(parents=True, exist_ok=True)
    system_info = run_json([str(flux_cli), "system-info", "--json"], timeout=30)
    capabilities = run_json([str(flux_cli), "capabilities", "--json"], timeout=30)
    metadata = {
        "schema": "flux_routing_baseline",
        "schema_version": 1,
        "created_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "mode": "preflight" if args.preflight_only else "routing",
        "invocation": [str(Path(sys.argv[0]).resolve()), *sys.argv[1:]],
        "environment": {
            "platform": platform.platform(),
            "python": sys.version,
            "flux_vulkan_device_override": os.environ.get("FLUX_VULKAN_DEVICE"),
            "flux": system_info,
            "kicad_cli": command_identity([str(kicad_cli), "--version"])
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
            "boards": [
                {
                    "id": board["id"],
                    "level": board["level"],
                    "status": board["status"],
                    "qualification": board.get("qualification"),
                    "source_sha256": board["board_sha256"],
                    "project_sha256": board["project_sha256"],
                    "reference_sha256": board["reference_sha256"],
                    "router_args": board.get("router_args", []),
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
    print(f"{len(selected)}/{len(selected)} selected corpus boards passed")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, subprocess.TimeoutExpired) as error:
        print(f"FAIL {error}", file=sys.stderr)
        raise SystemExit(1)
