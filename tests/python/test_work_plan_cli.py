#!/usr/bin/env python3
"""Exercise the write-once, source-bound, non-mutating FluxWorkPlan CLI boundary."""

import argparse
import json
import subprocess
import tempfile
from pathlib import Path


def run(command: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(command, check=False, capture_output=True, text=True, timeout=60)


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
            and previewed["ok"] is True
            and previewed["dry_run"] is True
            and previewed["would_mutate_board"] is False
            and previewed["capability_preflight_ok"] is True
        ):
            print("work-plan inspect/dry-run changed semantics or claimed mutation")
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
