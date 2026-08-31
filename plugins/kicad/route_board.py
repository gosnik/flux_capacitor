#!/usr/bin/env python3
"""Create, verify, preview, and optionally apply a Flux autoroute candidate."""

from __future__ import annotations

import argparse
import json
import pathlib
import shutil
import subprocess
import sys
import tempfile
from collections.abc import Mapping, Sequence
from typing import Any

from apply_candidate import ChangeSetError, validate_change_set


class WorkflowError(RuntimeError):
    """The bounded autoroute workflow failed before a candidate was approved."""


def _run_json(command: Sequence[str], label: str) -> tuple[int, Mapping[str, Any], str]:
    completed = subprocess.run(command, check=False, capture_output=True, text=True)
    try:
        document = json.loads(completed.stdout)
    except json.JSONDecodeError as error:
        detail = completed.stderr.strip() or completed.stdout.strip() or "no process output"
        raise WorkflowError(f"{label} returned invalid JSON: {detail}") from error
    if not isinstance(document, Mapping):
        raise WorkflowError(f"{label} returned a non-object JSON document")
    return completed.returncode, document, completed.stderr.strip()


def _resolve_new_output(path: pathlib.Path, source: pathlib.Path) -> pathlib.Path:
    parent = path.expanduser().parent.resolve(strict=True)
    output = parent / path.name
    if output == source:
        raise WorkflowError("candidate output must differ from the source board")
    if output.exists():
        raise WorkflowError(f"candidate output already exists: {output}")
    return output


def _resolve_sidecar(path: pathlib.Path | None) -> pathlib.Path | None:
    return None if path is None else path.expanduser().resolve(strict=True)


def _positive_int(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("value must be positive")
    return parsed


def _change_counts(change_set: Mapping[str, Any]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for kind in ("tracks", "track_arcs", "vias"):
        changes = change_set.get(kind)
        if not isinstance(changes, list):
            raise WorkflowError(f"change set has no {kind} array")
        counts[kind] = len(changes)
    transforms = change_set.get("footprint_transforms")
    if not isinstance(transforms, list):
        raise WorkflowError("change set has no footprint_transforms array")
    counts["footprint_transforms"] = len(transforms)
    return counts


def _external_drc(candidate: pathlib.Path, executable: str | None) -> dict[str, Any]:
    resolved = executable or shutil.which("kicad-cli")
    if not resolved:
        return {"status": "unavailable", "executable": None}
    with tempfile.TemporaryDirectory(prefix="flux-kicad-drc-") as directory:
        report = pathlib.Path(directory) / "drc.json"
        completed = subprocess.run(
            [
                resolved,
                "pcb",
                "drc",
                "--output",
                str(report),
                "--format",
                "json",
                "--severity-error",
                "--exit-code-violations",
                str(candidate),
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        return {
            "status": "passed" if completed.returncode == 0 else "failed",
            "executable": resolved,
            "returncode": completed.returncode,
            "stderr": completed.stderr.strip(),
        }


def _write_report(path: pathlib.Path | None, report: Mapping[str, Any]) -> None:
    if path is None:
        return
    destination = path.expanduser()
    destination.parent.resolve(strict=True)
    with destination.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, sort_keys=True)
        stream.write("\n")


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Run bounded Flux routing or integrated placement/routing for a saved KiCad board"
        )
    )
    parser.add_argument("--board", required=True, type=pathlib.Path)
    parser.add_argument("--output", required=True, type=pathlib.Path)
    parser.add_argument("--flux-cli", default="flux-cli")
    parser.add_argument("--netclasses", type=pathlib.Path)
    parser.add_argument("--diffpairs", type=pathlib.Path)
    parser.add_argument("--buses", type=pathlib.Path)
    parser.add_argument("--envelopes", type=pathlib.Path)
    parser.add_argument("--placement-groups", type=pathlib.Path)
    parser.add_argument("--placement-regions", type=pathlib.Path)
    parser.add_argument("--relative-placements", type=pathlib.Path)
    parser.add_argument("--placement-move-priorities", type=pathlib.Path)
    parser.add_argument(
        "--refillable-zones",
        action="store_true",
        help=(
            "route without saved fill copper, then require KiCad refill, strict re-import, "
            "and external DRC before publishing the candidate"
        ),
    )
    parser.add_argument(
        "--route-net",
        action="append",
        default=[],
        help="exact net name to route in refillable-zone mode; repeat for multiple nets",
    )
    parser.add_argument(
        "--route-first",
        action="append",
        default=[],
        help="schedule an exact net ahead of ordinary routing pressure; repeat in priority order",
    )
    parser.add_argument(
        "--repair-sampled-conflicts",
        action="store_true",
        help=(
            "reroute only sampled/shared conflict participants with diversified terminal access "
            "and hard capacity after the negotiated whole-board pass"
        ),
    )
    parser.add_argument(
        "--keep-incomplete-preview",
        action="store_true",
        help=(
            "retain a review-only KiCad layout when Flux is incomplete; the workflow still "
            "fails and never refills, approves, or applies that artifact"
        ),
    )
    parser.add_argument(
        "--curved-traces",
        action="store_true",
        help=(
            "enable crossing-safe two-diagonal discovery plus cubic B-spline fitting, collision "
            "envelopes, bounded PNS, and native line/arc output; allocation remains polyline based"
        ),
    )
    parser.add_argument(
        "--integrated-placement",
        action="store_true",
        help=(
            "run bounded global placement plus routing and emit a review-only candidate; "
            "suggested resistor-first/connector-last priorities are used unless a reviewed "
            "priority sidecar is supplied"
        ),
    )
    parser.add_argument(
        "--tentative-envelopes",
        action="store_true",
        help="derive provisional placement bounds and require mechanical review",
    )
    parser.add_argument("--max-placement-passes", type=_positive_int)
    parser.add_argument("--max-candidate-evaluations", type=_positive_int)
    parser.add_argument(
        "--capture-netclasses",
        action="store_true",
        help="capture authoritative net classes from the matching open KiCad board",
    )
    parser.add_argument("--kicad-cli", help="KiCad CLI executable used for external DRC")
    parser.add_argument(
        "--skip-external-drc",
        action="store_true",
        help="skip the additional KiCad CLI DRC (live application is then unavailable)",
    )
    parser.add_argument(
        "--apply-routing",
        action="store_true",
        help="apply the verified routing to the matching open board as one unsaved transaction",
    )
    parser.add_argument("--report", type=pathlib.Path, help="write the summary as a new JSON file")
    args = parser.parse_args(argv)

    workflow = (
        "place_route_v1"
        if args.integrated_placement
        else "refillable_zones_v1"
        if args.refillable_zones
        else "autoroute_v1"
    )
    report: dict[str, Any] = {"ok": False, "workflow": workflow}
    exit_code = 2
    try:
        if args.capture_netclasses and args.netclasses:
            raise WorkflowError("--capture-netclasses and --netclasses are mutually exclusive")
        if args.integrated_placement and args.apply_routing:
            raise WorkflowError(
                "integrated placement candidates require file-level human review and cannot be "
                "applied through the routing-only live bridge"
            )
        if args.refillable_zones and args.integrated_placement:
            raise WorkflowError("--refillable-zones is currently routing-only")
        if args.curved_traces and not args.refillable_zones:
            raise WorkflowError("--curved-traces currently requires --refillable-zones")
        if args.refillable_zones and args.apply_routing:
            raise WorkflowError(
                "refilled candidates require file-level review and cannot use the live bridge"
            )
        if args.refillable_zones and args.skip_external_drc:
            raise WorkflowError("refillable-zone candidates require external KiCad DRC")
        if args.route_net and not args.refillable_zones:
            raise WorkflowError("--route-net currently requires --refillable-zones")
        if args.route_first and not args.refillable_zones:
            raise WorkflowError("--route-first currently requires --refillable-zones")
        if args.repair_sampled_conflicts and not args.refillable_zones:
            raise WorkflowError("--repair-sampled-conflicts currently requires --refillable-zones")
        if args.keep_incomplete_preview and not args.refillable_zones:
            raise WorkflowError("--keep-incomplete-preview currently requires --refillable-zones")
        if args.keep_incomplete_preview and args.apply_routing:
            raise WorkflowError("an incomplete preview cannot be applied to the live board")
        placement_options = (
            args.envelopes,
            args.placement_groups,
            args.placement_regions,
            args.relative_placements,
            args.placement_move_priorities,
            args.max_placement_passes,
            args.max_candidate_evaluations,
        )
        if not args.integrated_placement and (
            args.tentative_envelopes or any(value is not None for value in placement_options)
        ):
            raise WorkflowError("placement options require --integrated-placement")
        source = args.board.expanduser().resolve(strict=True)
        candidate = _resolve_new_output(args.output, source)
        netclasses = _resolve_sidecar(args.netclasses)
        diffpairs = _resolve_sidecar(args.diffpairs)
        buses = _resolve_sidecar(args.buses)
        envelopes = _resolve_sidecar(args.envelopes)
        placement_groups = _resolve_sidecar(args.placement_groups)
        placement_regions = _resolve_sidecar(args.placement_regions)
        relative_placements = _resolve_sidecar(args.relative_placements)
        placement_priorities = _resolve_sidecar(args.placement_move_priorities)

        with tempfile.TemporaryDirectory(prefix=f"flux-{workflow}-") as directory:
            if args.capture_netclasses:
                captured = subprocess.run(
                    [
                        sys.executable,
                        str(pathlib.Path(__file__).with_name("export_netclasses.py")),
                        "--board",
                        str(source),
                        "--flux-cli",
                        args.flux_cli,
                    ],
                    check=False,
                    capture_output=True,
                    text=True,
                )
                if captured.returncode != 0:
                    raise WorkflowError(
                        "live net-class capture failed: "
                        + (captured.stderr.strip() or "no diagnostic")
                    )
                netclasses = pathlib.Path(directory) / "captured.netclasses"
                netclasses.write_text(captured.stdout, encoding="utf-8")

            provisional = pathlib.Path(directory) / "provisional.kicad_pcb"
            engine_output = (
                candidate
                if args.refillable_zones and args.keep_incomplete_preview
                else provisional
                if args.refillable_zones
                else candidate
            )
            command = [
                args.flux_cli,
                (
                    "global-optimize-kicad"
                    if args.integrated_placement
                    else "detail-kicad"
                    if args.refillable_zones
                    else "autoroute-kicad"
                ),
                str(source),
            ]
            for option, value in (
                ("--netclasses", netclasses),
                ("--diffpairs", diffpairs),
                ("--buses", buses),
                ("--envelopes", envelopes),
                ("--placement-groups", placement_groups),
                ("--placement-regions", placement_regions),
                ("--relative-placements", relative_placements),
            ):
                if value is not None:
                    command.extend((option, str(value)))
            if args.integrated_placement:
                if placement_priorities is not None:
                    command.extend(("--placement-move-priorities", str(placement_priorities)))
                else:
                    command.append("--suggested-placement-move-priorities")
                command.extend(
                    (
                        "--routing-guided-global-placement",
                        "--max-placement-passes",
                        str(args.max_placement_passes or 2),
                        "--max-candidate-evaluations",
                        str(args.max_candidate_evaluations or 64),
                    )
                )
                if args.tentative_envelopes:
                    command.append("--tentative-envelopes")
            elif args.refillable_zones:
                command.extend(
                    (
                        "--refillable-zones",
                        "--grid-pitch-nm",
                        "1000000",
                        "--local-pin-escape-pitch-nm",
                        "250000",
                        "--max-local-pin-escape-expansions",
                        "20000",
                        "--pin-escape-radius",
                        "6",
                        "--max-pin-access-nodes",
                        "32",
                        (
                            "--crossing-safe-diagonal-grid"
                            if args.curved_traces
                            else "--diagonal-grid"
                        ),
                        "--max-router-iterations",
                        "30",
                        "--max-expansions",
                        "1000000",
                        "--expansion-retry-heuristic-weight-per-mille",
                        "10000",
                        "--reuse-weighted-search-frontier",
                        "--present-congestion-multiplier-per-mille",
                        "1100",
                        "--max-present-congestion-scale-per-mille",
                        "100000",
                        "--present-congestion-escalation-start-iteration",
                        "2",
                    )
                )
                if args.curved_traces:
                    command.append("--arcs")
                for name in args.route_net:
                    command.extend(("--route-net", name))
                for name in args.route_first:
                    command.extend(("--route-first", name))
                if args.repair_sampled_conflicts:
                    command.append("--repair-sampled-conflicts")
                if args.keep_incomplete_preview:
                    command.append("--write-incomplete-preview")
            command.append(str(engine_output))
            route_code, engine, route_stderr = _run_json(command, f"{workflow} engine")
            engine_report = {
                "returncode": route_code,
                "complete": engine.get("complete"),
                "stop_reason": engine.get("stop_reason"),
                "copper_violations": engine.get("copper_violations"),
                "open_connections": engine.get("open_connections"),
                "global_complete": engine.get("global_complete"),
                "global_iterations": engine.get("global_iterations"),
                "global_unrouted_nets": engine.get("global_unrouted_nets"),
                "global_unrouted_net_ids": engine.get("global_unrouted_net_ids"),
                "global_connection_failures": engine.get("global_connection_failures"),
                "global_selected_net_pressure": engine.get("global_selected_net_pressure"),
                "global_total_overflow": engine.get("global_total_overflow"),
                "global_sampled_overflow": engine.get("global_sampled_overflow"),
                "global_corridor_overflow": engine.get("global_corridor_overflow"),
                "artifact_written": engine.get("artifact_written"),
                "artifact_preview_only": engine.get("artifact_preview_only"),
                "global_congested_sampled_resource_usage": engine.get(
                    "global_congested_sampled_resource_usage"
                ),
                "static_seed_connections": engine.get("static_seed_connections"),
                "static_seed_searches": engine.get("static_seed_searches"),
                "static_seed_candidates": engine.get("static_seed_candidates"),
                "static_seed_terminals_without_seed": engine.get(
                    "static_seed_terminals_without_seed"
                ),
                "static_seed_reprojections_accepted": engine.get(
                    "static_seed_reprojections_accepted"
                ),
                "iteration_zero_paths": engine.get("iteration_zero_paths"),
                "iteration_zero_unrouted_nets": engine.get("iteration_zero_unrouted_nets"),
                "iteration_zero_total_overflow": engine.get("iteration_zero_total_overflow"),
                "iteration_zero_seed_history_increments": engine.get(
                    "iteration_zero_seed_history_increments"
                ),
                "configured_crossing_safe_diagonal_grid_edges": engine.get(
                    "configured_crossing_safe_diagonal_grid_edges"
                ),
                "configured_circular_arc_refinement": engine.get(
                    "configured_circular_arc_refinement"
                ),
                "attempted_circular_arc_candidates": engine.get(
                    "attempted_circular_arc_candidates"
                ),
                "rejected_circular_arc_candidates": engine.get(
                    "rejected_circular_arc_candidates"
                ),
                "fitted_circular_arcs": engine.get("fitted_circular_arcs"),
                "radius_reduced_circular_arcs": engine.get(
                    "radius_reduced_circular_arcs"
                ),
                "attempted_cubic_bspline_runs": engine.get(
                    "attempted_cubic_bspline_runs"
                ),
                "fitted_cubic_bspline_runs": engine.get("fitted_cubic_bspline_runs"),
                "adaptive_spline_tessellation_samples": engine.get(
                    "adaptive_spline_tessellation_samples"
                ),
                "spline_collision_envelopes": engine.get("spline_collision_envelopes"),
                "spline_control_point_shoves": engine.get("spline_control_point_shoves"),
                "spline_refits": engine.get("spline_refits"),
                "pns_shove_attempts": engine.get("pns_shove_attempts"),
                "accepted_pns_shoves": engine.get("accepted_pns_shoves"),
                "pns_moved_tracks": engine.get("pns_moved_tracks"),
                "global_diagnostic": engine.get("global_diagnostic"),
                "artifact_written": engine.get("artifact_written"),
                "diagnostic": engine.get("diagnostic"),
                "stderr": route_stderr,
            }
            if args.integrated_placement:
                engine_report.update(
                    {
                        "feasible": engine.get("feasible"),
                        "configured_max_placement_passes": engine.get(
                            "configured_max_placement_passes"
                        ),
                        "configured_max_candidate_evaluations": engine.get(
                            "configured_max_candidate_evaluations"
                        ),
                        "accepted_moves": engine.get("accepted_moves"),
                        "tentative_placement_envelopes": engine.get(
                            "tentative_placement_envelopes"
                        ),
                        "human_placement_review_required": engine.get(
                            "human_placement_review_required"
                        ),
                        "placement_move_priority_review_required": engine.get(
                            "placement_move_priority_review_required"
                        ),
                    }
                )
            else:
                engine_report.update(
                    {
                        "candidate_available": engine.get("candidate_available"),
                        "provisional_complete": engine.get("provisional_complete"),
                        "zone_refill_required": engine.get("zone_refill_required"),
                    }
                )
            report.update(
                {
                    "source": str(source),
                    "candidate": str(candidate),
                    "engine": engine_report,
                }
            )
            engine_approved = (
                (
                    engine.get("provisional_complete") is True
                    and engine.get("zone_refill_required") is True
                    if args.refillable_zones
                    else engine.get("complete") is True
                )
                and engine.get("artifact_written") is True
                and (
                    engine.get("feasible") is True
                    if args.integrated_placement
                    else engine.get("workflow") == workflow
                )
            )
            if route_code != 0 or not engine_approved:
                exit_code = route_code or 2
                if (
                    args.keep_incomplete_preview
                    and engine.get("artifact_written") is True
                    and engine.get("artifact_preview_only") is True
                    and engine_output.is_file()
                ):
                    report["incomplete_preview"] = {
                        "candidate": str(engine_output),
                        "review_only": True,
                        "refill_withheld": True,
                        "external_drc_withheld": True,
                        "live_apply_withheld": True,
                    }
                raise WorkflowError(f"{workflow} engine did not produce an approved candidate")
            if not engine_output.is_file() or not engine.get("artifact_written"):
                raise WorkflowError(f"{workflow} engine reported success without a candidate file")

            change_code, change_set, change_stderr = _run_json(
                [args.flux_cli, "changeset-kicad", str(source), str(engine_output)],
                "candidate change set",
            )
            if change_code != 0 or change_set.get("ok") is not True:
                raise WorkflowError(
                    "candidate change-set verification failed: " + (change_stderr or "no diagnostic")
                )
            try:
                validate_change_set(change_set)
            except ChangeSetError as error:
                raise WorkflowError(f"candidate change-set contract failed: {error}") from error
            preview = {
                "source_revision": change_set.get("source_revision"),
                **_change_counts(change_set),
            }
            report["preview"] = preview
            review_reasons: list[str] = []
            if preview["footprint_transforms"]:
                review_reasons.append("footprint-transforms")
            tentative_envelopes = engine.get("tentative_placement_envelopes")
            if isinstance(tentative_envelopes, int) and tentative_envelopes > 0:
                review_reasons.append("tentative-placement-envelopes")
            if engine.get("placement_move_priority_review_required") is True:
                review_reasons.append("suggested-placement-classification")
            if args.refillable_zones:
                review_reasons.append("refilled-zone-copper")
            report["review"] = {
                "required": bool(review_reasons),
                "reasons": review_reasons,
                "candidate_only": args.integrated_placement or args.refillable_zones,
            }

            if args.refillable_zones:
                staged_policy: list[str] = []
                for suffix in (".kicad_pro", ".kicad_dru"):
                    source_policy = source.with_suffix(suffix)
                    if source_policy.is_file():
                        staged = provisional.with_suffix(suffix)
                        shutil.copy2(source_policy, staged)
                        staged_policy.append(str(staged))
                refilled = pathlib.Path(directory) / "refilled.kicad_pcb"
                refill_code, refill, refill_stderr = _run_json(
                    [
                        sys.executable,
                        str(pathlib.Path(__file__).with_name("refill_zones.py")),
                        "--board",
                        str(provisional),
                        "--output",
                        str(refilled),
                    ],
                    "KiCad zone refill",
                )
                report["zone_refill"] = {
                    **refill,
                    "returncode": refill_code,
                    "stderr": refill_stderr,
                    "staged_project_policy": staged_policy,
                }
                if refill_code != 0 or refill.get("ok") is not True or not refilled.is_file():
                    raise WorkflowError("KiCad zone refill did not produce a candidate")
                inspect_code, inspected, inspect_stderr = _run_json(
                    [args.flux_cli, "inspect-kicad", str(refilled)],
                    "refilled candidate re-import",
                )
                report["refilled_import"] = {
                    "returncode": inspect_code,
                    "ok": inspected.get("ok"),
                    "strict": inspected.get("strict"),
                    "open_connections": inspected.get("open_connections"),
                    "stderr": inspect_stderr,
                }
                if inspect_code != 0 or inspected.get("strict") is not True:
                    raise WorkflowError("refilled candidate failed strict Flux re-import")
                if inspected.get("open_connections") != 0:
                    raise WorkflowError("refilled candidate remains electrically disconnected")
                engine_output = refilled

            drc = (
                {"status": "skipped", "executable": None}
                if args.skip_external_drc
                else _external_drc(engine_output, args.kicad_cli)
            )
            report["external_drc"] = drc
            if drc["status"] == "failed":
                exit_code = 8
                raise WorkflowError("candidate failed external KiCad DRC")
            if args.refillable_zones:
                if drc["status"] != "passed":
                    raise WorkflowError("refillable-zone publication requires passing KiCad DRC")
                shutil.copy2(engine_output, candidate)

            report["live"] = {
                "mode": (
                    "withheld-integrated-placement-review"
                    if args.integrated_placement
                    else "not-requested"
                ),
                "applied": False,
            }
            if args.apply_routing:
                if drc["status"] != "passed":
                    raise WorkflowError("live application requires a passing external KiCad DRC")
                apply_code, applied, apply_stderr = _run_json(
                    [
                        sys.executable,
                        str(pathlib.Path(__file__).with_name("apply_candidate.py")),
                        "--board",
                        str(source),
                        "--candidate",
                        str(candidate),
                        "--flux-cli",
                        args.flux_cli,
                        "--apply-routing",
                    ],
                    "live candidate bridge",
                )
                if apply_code != 0 or applied.get("ok") is not True:
                    raise WorkflowError(
                        "live routing transaction failed: " + (apply_stderr or "no diagnostic")
                    )
                report["live"] = applied

        report["ok"] = True
        exit_code = 0
    except (OSError, WorkflowError, ValueError) as error:
        report["error"] = str(error)

    try:
        _write_report(args.report, report)
    except OSError as error:
        report["ok"] = False
        report["error"] = f"unable to write report: {error}"
        exit_code = 2
    print(json.dumps(report, indent=2, sort_keys=True))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
