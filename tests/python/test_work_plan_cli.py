#!/usr/bin/env python3
"""Exercise the write-once, source-bound, non-mutating FluxWorkPlan CLI boundary."""

import argparse
import copy
import hashlib
import json
import os
import subprocess
import tempfile
from pathlib import Path


def run(
    command: list[str], environment: dict[str, str] | None = None
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command, check=False, capture_output=True, text=True, timeout=60, env=environment
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--flux-cli", required=True, type=Path)
    parser.add_argument("--board", required=True, type=Path)
    args = parser.parse_args()

    with tempfile.TemporaryDirectory(prefix="flux-work-plan-") as temporary:
        plan = Path(temporary) / "plan.flux.json"
        template = run(
            [str(args.flux_cli), "work-plan-template-kicad", str(args.board), str(plan)]
        )
        if template.returncode != 0:
            print(template.stdout + template.stderr)
            return 1
        document = json.loads(plan.read_text(encoding="utf-8"))
        if not (
            document["schema"] == "flux_work_plan"
            and document["schema_version"] == 1
            and document["source"]["revision"]
            and document["mode"] == "preview"
            and document["telemetry"]["enabled"] is True
            and document["steps"] == []
        ):
            print("work-plan template omitted its strict source/default contract")
            return 1

        inspect = run(
            [str(args.flux_cli), "inspect-work-plan-kicad", str(args.board), str(plan)]
        )
        dry_run = run(
            [str(args.flux_cli), "dry-run-work-plan-kicad", str(args.board), str(plan)]
        )
        if inspect.returncode != 0 or dry_run.returncode != 0:
            print(inspect.stdout + inspect.stderr + dry_run.stdout + dry_run.stderr)
            return 1
        inspected = json.loads(inspect.stdout)
        previewed = json.loads(dry_run.stdout)
        if not (
            inspected["ok"] is True
            and inspected["canonical_plan"] == document
            and inspected["expanded_steps"] == []
            and inspected["existing_copper_authority"]["reroutable_nets"] == []
            and inspected["existing_copper_authority"]["immutable_tracks"] == 1
            and previewed["ok"] is True
            and previewed["dry_run"] is True
            and previewed["would_mutate_board"] is False
            and previewed["capability_preflight_ok"] is True
        ):
            print("work-plan inspect/dry-run changed semantics or claimed mutation")
            return 1

        executable_preview = copy.deepcopy(document)
        executable_preview["telemetry"]["enabled"] = False
        executable_preview_plan = Path(temporary) / "preview.flux.json"
        executable_preview_plan.write_text(json.dumps(executable_preview), encoding="utf-8")
        executed_preview = run(
            [
                str(args.flux_cli),
                "execute-work-plan-kicad",
                str(args.board),
                str(executable_preview_plan),
            ]
        )
        if executed_preview.returncode != 0:
            print(executed_preview.stdout + executed_preview.stderr)
            return 1
        execution_summary = json.loads(executed_preview.stdout)
        if not (
            execution_summary["preview"] is True
            and execution_summary["internally_complete"] is True
            and execution_summary["candidate_ready_for_export"] is False
            and execution_summary["candidate_ready_for_zone_refill"] is False
            and execution_summary["zone_refill_required"] is False
            and execution_summary["zone_refill_run"] is False
            and execution_summary["zone_refill_passed"] is False
            and execution_summary["candidate_is_incomplete_diagnostic_preview"] is False
            and execution_summary["export_attempted"] is False
            and execution_summary["output"] is None
        ):
            print("empty preview plan did not execute as a non-mutating deterministic no-op")
            return 1

        publish_document = copy.deepcopy(document)
        publish_document["mode"] = "new-routing"
        publish_document["telemetry"]["enabled"] = False
        publish_plan = Path(temporary) / "publish.flux.json"
        publish_plan.write_text(json.dumps(publish_document), encoding="utf-8")

        passing_kicad = Path(temporary) / "fake kicad-cli pass"
        passing_kicad.write_text(
            """#!/bin/sh
report=
while [ "$#" -gt 0 ]; do
  if [ "$1" = "--output" ] && [ "$#" -gt 1 ]; then
    report=$2
    shift 2
  else
    shift
  fi
done
[ -n "$report" ] || exit 90
printf '{"violations":[]}\\n' > "$report"
exit 0
""",
            encoding="utf-8",
        )
        passing_kicad.chmod(0o755)
        passing_environment = os.environ.copy()
        passing_environment["FLUX_KICAD_CLI"] = str(passing_kicad)
        published_output = Path(temporary) / "published.kicad_pcb"
        published = run(
            [
                str(args.flux_cli),
                "execute-work-plan-kicad",
                str(args.board),
                str(publish_plan),
                str(published_output),
            ],
            passing_environment,
        )
        published_summary = json.loads(published.stdout)
        if not (
            published.returncode == 0
            and published_output.is_file()
            and published_summary["internally_complete"] is True
            and published_summary["external_kicad_drc_run"] is True
            and published_summary["external_kicad_drc_passed"] is True
            and published_summary["external_kicad_drc_exit_code"] == 0
            and published_summary["candidate_ready_for_export"] is True
            and published_summary["output_published"] is True
        ):
            print(published.stdout + published.stderr)
            return 1

        failing_kicad = Path(temporary) / "fake kicad-cli fail"
        failing_kicad.write_text(
            """#!/bin/sh
report=
while [ "$#" -gt 0 ]; do
  if [ "$1" = "--output" ] && [ "$#" -gt 1 ]; then
    report=$2
    shift 2
  else
    shift
  fi
done
[ -n "$report" ] || exit 90
printf '{"violations":[{}]}\\n' > "$report"
echo 'intentional DRC violation' >&2
exit 5
""",
            encoding="utf-8",
        )
        failing_kicad.chmod(0o755)
        failing_environment = os.environ.copy()
        failing_environment["FLUX_KICAD_CLI"] = str(failing_kicad)
        rejected_output = Path(temporary) / "rejected.kicad_pcb"
        rejected = run(
            [
                str(args.flux_cli),
                "execute-work-plan-kicad",
                str(args.board),
                str(publish_plan),
                str(rejected_output),
            ],
            failing_environment,
        )
        rejected_summary = json.loads(rejected.stdout)
        if not (
            rejected.returncode == 13
            and not rejected_output.exists()
            and rejected_summary["external_kicad_drc_run"] is True
            and rejected_summary["external_kicad_drc_passed"] is False
            and rejected_summary["external_kicad_drc_exit_code"] == 5
            and rejected_summary["candidate_ready_for_export"] is False
            and rejected_summary["output_published"] is False
        ):
            print(rejected.stdout + rejected.stderr)
            return 1

        unavailable_environment = os.environ.copy()
        unavailable_environment["FLUX_KICAD_CLI"] = str(
            Path(temporary) / "missing-kicad-cli"
        )
        unavailable_output = Path(temporary) / "unavailable.kicad_pcb"
        unavailable = run(
            [
                str(args.flux_cli),
                "execute-work-plan-kicad",
                str(args.board),
                str(publish_plan),
                str(unavailable_output),
            ],
            unavailable_environment,
        )
        unavailable_summary = json.loads(unavailable.stdout)
        if not (
            unavailable.returncode == 13
            and not unavailable_output.exists()
            and unavailable_summary["external_kicad_drc_run"] is False
            and unavailable_summary["external_kicad_drc_passed"] is False
            and unavailable_summary["candidate_ready_for_export"] is False
            and unavailable_summary["output_published"] is False
        ):
            print(unavailable.stdout + unavailable.stderr)
            return 1

        if list(Path(temporary).glob(".flux-work-plan-drc-*")):
            print("WorkPlan DRC staging directories were not cleaned up")
            return 1

        policy = Path(temporary) / "reviewed.policy"
        policy_template = run(
            [str(args.flux_cli), "placement-groups-template-kicad", str(args.board)]
        )
        if policy_template.returncode != 0:
            print(policy_template.stdout + policy_template.stderr)
            return 1
        policy.write_text(policy_template.stdout, encoding="utf-8")
        policy_document = copy.deepcopy(document)
        policy_document["telemetry"]["enabled"] = False
        policy_document["policies"] = [
            {
                "kind": "placement-groups",
                "path": policy.name,
                "sha256": hashlib.sha256(policy.read_bytes()).hexdigest(),
                "source_revision": document["source"]["revision"],
            }
        ]
        policy_plan = Path(temporary) / "policy.flux.json"
        policy_plan.write_text(json.dumps(policy_document), encoding="utf-8")
        bound_policy = run(
            [str(args.flux_cli), "inspect-work-plan-kicad", str(args.board), str(policy_plan)]
        )
        bound_policy_result = json.loads(bound_policy.stdout)
        if not (
            bound_policy.returncode == 0
            and bound_policy_result["ok"] is True
            and bound_policy_result["applied_policy_kinds"] == ["placement-groups"]
        ):
            print(bound_policy.stdout + bound_policy.stderr)
            return 1
        policy_execution = run(
            [str(args.flux_cli), "execute-work-plan-kicad", str(args.board), str(policy_plan)]
        )
        if not (
            policy_execution.returncode == 0
            and json.loads(policy_execution.stdout)["internally_complete"] is True
        ):
            print(policy_execution.stdout + policy_execution.stderr)
            return 1
        policy.write_text("changed policy\n", encoding="utf-8")
        stale_policy = run(
            [str(args.flux_cli), "inspect-work-plan-kicad", str(args.board), str(policy_plan)]
        )
        if stale_policy.returncode != 11 or not any(
            diagnostic["code"] == "stale_work_plan_policy_content"
            for diagnostic in json.loads(stale_policy.stdout)["diagnostics"]
        ):
            print(stale_policy.stdout + stale_policy.stderr)
            return 1

        scoped_document = copy.deepcopy(document)
        scoped_document["mode"] = "force-reroute-selected"
        scoped_document["steps"] = [
            {
                "id": "route-signal",
                "kind": "route",
                "depends_on": [],
                "selectors": {
                    "all_nets": False,
                    "exact_nets": ["SIGNAL"],
                    "include_globs": [],
                    "exclude_globs": [],
                    "net_classes": [],
                    "footprints": [],
                    "placement_groups": [],
                    "group_scope": "selected",
                },
                "options": {
                    "circular_arc_refinement": True,
                    "max_final_routability_feedback_passes": 5,
                    "max_post_negotiation_sampled_conflict_repair_cones": 4,
                    "max_router_iterations": 9,
                    "post_negotiation_sampled_conflict_repair": True,
                    "priority_nets": ["SIGNAL"],
                    "refillable_zone_fills": True,
                    "xpbd_feedback": True,
                },
            }
        ]
        scoped_plan = Path(temporary) / "scoped.flux.json"
        scoped_plan.write_text(json.dumps(scoped_document), encoding="utf-8")
        scoped = run(
            [str(args.flux_cli), "inspect-work-plan-kicad", str(args.board), str(scoped_plan)]
        )
        if scoped.returncode != 0:
            print(scoped.stdout + scoped.stderr)
            return 1
        scoped_result = json.loads(scoped.stdout)
        if not (
            scoped_result["ok"] is True
            and scoped_result["expanded_steps"][0]["nets"] == ["SIGNAL"]
            and scoped_result["expanded_steps"][0]["wildcard_only_nets"] == []
            and scoped_result["existing_copper_authority"]["reroutable_nets"]
            == ["SIGNAL"]
            and scoped_result["existing_copper_authority"]["removable_unlocked_tracks"]
            == 1
            and scoped_result["existing_copper_authority"]["immutable_zones"] == 1
            and sum(scoped_result["selected_net_mask"]) == 1
            and sum(scoped_result["reroutable_net_mask"]) == 1
            and scoped_result["compiled_route_steps"][0]["priority_nets"] == ["SIGNAL"]
            and scoped_result["compiled_route_steps"][0]["configuration"]
            ["max_router_iterations"]
            == 9
            and scoped_result["compiled_route_steps"][0]["configuration"]
            ["max_final_routability_feedback_passes"]
            == 5
            and scoped_result["compiled_route_steps"][0]["configuration"]
            ["post_negotiation_sampled_conflict_repair"]
            is True
            and scoped_result["compiled_route_steps"][0]["configuration"]
            ["max_post_negotiation_sampled_conflict_repair_cones"]
            == 4
            and scoped_result["compiled_route_steps"][0]["configuration"]
            ["refillable_zone_fills"]
            is True
            and scoped_result["compiled_route_steps"][0]["configuration"]
            ["xpbd_relaxation"]
            is True
            and scoped_result["compiled_route_steps"][0]["configuration"]
            ["circular_arc_refinement"]
            is True
        ):
            print("exact route selector did not produce explicit protected-copper authority")
            return 1

        diagnostic_preview_document = copy.deepcopy(scoped_document)
        diagnostic_preview_document["mode"] = "preview"
        diagnostic_preview_document["steps"][0]["options"].pop("refillable_zone_fills")
        diagnostic_preview_document["steps"][0]["options"][
            "retain_incomplete_diagnostic_preview"
        ] = True
        diagnostic_preview_plan = Path(temporary) / "diagnostic-preview.flux.json"
        diagnostic_preview_plan.write_text(
            json.dumps(diagnostic_preview_document), encoding="utf-8"
        )
        diagnostic_preview = run(
            [
                str(args.flux_cli),
                "inspect-work-plan-kicad",
                str(args.board),
                str(diagnostic_preview_plan),
            ]
        )
        diagnostic_preview_result = json.loads(diagnostic_preview.stdout)
        if not (
            diagnostic_preview.returncode == 0
            and diagnostic_preview_result["compiled_route_steps"][0]["configuration"]
            ["retain_incomplete_diagnostic_preview"]
            is True
        ):
            print(diagnostic_preview.stdout + diagnostic_preview.stderr)
            return 1

        blocked_output = Path(temporary) / "must-not-exist.kicad_pcb"
        blocked_execution = run(
            [
                str(args.flux_cli),
                "execute-work-plan-kicad",
                str(args.board),
                str(scoped_plan),
                str(blocked_output),
            ]
        )
        if blocked_execution.returncode != 12 or blocked_output.exists():
            print(blocked_execution.stdout + blocked_execution.stderr)
            return 1

        wildcard_document = copy.deepcopy(scoped_document)
        wildcard_document["steps"][0]["selectors"]["exact_nets"] = []
        wildcard_document["steps"][0]["selectors"]["include_globs"] = ["SIG*"]
        wildcard_plan = Path(temporary) / "wildcard.flux.json"
        wildcard_plan.write_text(json.dumps(wildcard_document), encoding="utf-8")
        wildcard = run(
            [str(args.flux_cli), "inspect-work-plan-kicad", str(args.board), str(wildcard_plan)]
        )
        if wildcard.returncode != 11:
            print(wildcard.stdout + wildcard.stderr)
            return 1
        wildcard_result = json.loads(wildcard.stdout)
        if not any(
            diagnostic["code"] == "wildcard_only_force_reroute_scope"
            for diagnostic in wildcard_result["diagnostics"]
        ):
            print("wildcard force-reroute scope was not rejected with attributed evidence")
            return 1

        overwrite = run(
            [str(args.flux_cli), "work-plan-template-kicad", str(args.board), str(plan)]
        )
        if overwrite.returncode == 0:
            print("work-plan template silently overwrote an existing plan")
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
