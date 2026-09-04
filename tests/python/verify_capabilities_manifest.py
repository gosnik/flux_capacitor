#!/usr/bin/env python3
"""Verify that flux-cli advertises the checked-in parity contract exactly."""

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path


def digest(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--flux-cli", required=True, type=Path)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--fixtures", required=True, type=Path)
    parser.add_argument("--reference-cases", required=True, type=Path)
    parser.add_argument("--reference-report", required=True, type=Path)
    args = parser.parse_args()

    expected = json.loads(args.manifest.read_text(encoding="utf-8"))
    fixture_catalog = json.loads(args.fixtures.read_text(encoding="utf-8"))
    reference_cases = json.loads(args.reference_cases.read_text(encoding="utf-8"))
    reference_report = json.loads(args.reference_report.read_text(encoding="utf-8"))
    completed = subprocess.run(
        [str(args.flux_cli), "capabilities", "--json"],
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        print(completed.stderr, file=sys.stderr)
        return completed.returncode
    actual = json.loads(completed.stdout)
    if actual != expected:
        print("flux-cli capability output differs from docs/parity_manifest.json", file=sys.stderr)
        return 1

    allowed = set(expected["status_values"])
    capabilities = expected["capabilities"]
    ids = [capability["id"] for capability in capabilities]
    if len(ids) != len(set(ids)):
        print("capability IDs must be unique", file=sys.stderr)
        return 1
    fixtures = fixture_catalog["fixtures"]
    fixture_ids = [fixture["id"] for fixture in fixtures]
    if len(fixture_ids) != len(set(fixture_ids)):
        print("fixture IDs must be unique", file=sys.stderr)
        return 1
    fixtures_by_id = {fixture["id"]: fixture for fixture in fixtures}
    project_root = args.manifest.resolve().parent.parent
    reference = fixture_catalog["reference"]
    if reference["commit"] != expected["reference"]["commit"]:
        print("fixture catalog and capability contract pin different references", file=sys.stderr)
        return 1
    license_path = project_root / reference["license_file"]
    if not license_path.is_file() or digest(license_path) != reference["license_sha256"]:
        print("pinned KiCadRoutingTools license identity mismatch", file=sys.stderr)
        return 1

    known_states = {"contract", "qualified", "frontier", "import-blocked"}
    for fixture in fixtures:
        gate = fixture["current_gate"]
        if gate["state"] not in known_states:
            print(f"{fixture['id']} has invalid gate state {gate['state']}", file=sys.stderr)
            return 1
        if gate["state"] == "import-blocked" and (gate["strict_import"] or not gate["blockers"]):
            print(f"{fixture['id']} must retain its attributed strict-import blockers", file=sys.stderr)
            return 1
        if fixture["origin"] == "KiCadRoutingTools" and (
            fixture["license"] != "MIT" or not fixture["upstream_path"]
        ):
            print(f"{fixture['id']} lacks pinned upstream/license provenance", file=sys.stderr)
            return 1
        for file_record in fixture["files"]:
            path = project_root / file_record["path"]
            if not path.is_file() or digest(path) != file_record["sha256"]:
                print(f"{fixture['id']} file identity mismatch: {path}", file=sys.stderr)
                return 1
        board_files = [record for record in fixture["files"] if record["path"].endswith(".kicad_pcb")]
        if len(board_files) != 1:
            print(f"{fixture['id']} must identify exactly one board", file=sys.stderr)
            return 1
        inspection = subprocess.run(
            [str(args.flux_cli), "inspect-kicad", str(project_root / board_files[0]["path"])],
            check=False,
            capture_output=True,
            text=True,
        )
        try:
            inspection_result = json.loads(inspection.stdout)
        except json.JSONDecodeError:
            print(f"{fixture['id']} inspection did not produce JSON", file=sys.stderr)
            return 1
        if inspection_result.get("strict") is not gate["strict_import"]:
            print(f"{fixture['id']} strict-import gate drifted", file=sys.stderr)
            return 1
        if gate["state"] != "import-blocked" and inspection_result.get("core_validation_errors") != 0:
            print(f"{fixture['id']} passes syntax import but fails canonical validation", file=sys.stderr)
            return 1
        actual_issues = set(inspection_result.get("import_issue_counts", {}))
        if gate["state"] == "import-blocked" and not set(gate["blockers"]).issubset(actual_issues):
            print(
                f"{fixture['id']} no longer reports blockers {gate['blockers']}",
                file=sys.stderr,
            )
            return 1
        unknown_capabilities = set(fixture["capabilities"]).difference(ids)
        if unknown_capabilities:
            print(
                f"{fixture['id']} references unknown capabilities {sorted(unknown_capabilities)}",
                file=sys.stderr,
            )
            return 1
    required_fields = {
        "id",
        "title",
        "status",
        "reference_evidence",
        "flux_evidence",
        "focused_fixture",
        "stress_sample",
        "public_commands",
        "acceptance",
        "gap",
    }
    for capability in capabilities:
        missing = required_fields.difference(capability)
        if missing:
            print(f"{capability.get('id', '<unknown>')} lacks {sorted(missing)}", file=sys.stderr)
            return 1
        if capability["status"] not in allowed:
            print(f"{capability['id']} has invalid status {capability['status']}", file=sys.stderr)
            return 1
        fixture_id = capability["focused_fixture"]
        if fixture_id not in fixtures_by_id:
            print(f"{capability['id']} references unknown fixture {fixture_id}", file=sys.stderr)
            return 1
        if capability["id"] not in fixtures_by_id[fixture_id]["capabilities"]:
            print(
                f"{fixture_id} is not cataloged for capability {capability['id']}",
                file=sys.stderr,
            )
            return 1
        if capability["status"] == "present" and (
            not capability["focused_fixture"] or not capability["public_commands"]
        ):
            print(
                f"{capability['id']} cannot be present without a fixture and public command",
                file=sys.stderr,
            )
            return 1

    case_reference = reference_cases.get("reference", {})
    report_reference = reference_report.get("reference", {})
    expected_commit = expected["reference"]["commit"]
    if not (
        case_reference.get("commit") == expected_commit
        and report_reference.get("commit") == expected_commit
    ):
        print("reference cases, report, and capability contract pin different revisions", file=sys.stderr)
        return 1
    comparison_cases = reference_cases.get("cases", [])
    reported_cases = reference_report.get("cases", [])
    comparison_ids = [case.get("id") for case in comparison_cases]
    reported_ids = [case.get("id") for case in reported_cases]
    if not comparison_ids or len(comparison_ids) != len(set(comparison_ids)):
        print("reference comparison case IDs must be non-empty and unique", file=sys.stderr)
        return 1
    if reported_ids != comparison_ids:
        print("checked reference comparison report does not cover the case catalog", file=sys.stderr)
        return 1
    fixture_gate_by_id = {
        fixture["id"]: fixture["current_gate"]["state"] for fixture in fixtures
    }
    for case, reported in zip(comparison_cases, reported_cases):
        fixture_ids_for_case = case.get("fixture_ids", [])
        if not fixture_ids_for_case or any(
            fixture_id not in fixtures_by_id for fixture_id in fixture_ids_for_case
        ):
            print(f"{case.get('id')} references an unknown or empty fixture set", file=sys.stderr)
            return 1
        if any(capability not in ids for capability in case.get("capabilities", [])):
            print(f"{case['id']} references an unknown capability", file=sys.stderr)
            return 1
        if case.get("kind") not in {"route", "upstream-test"}:
            print(f"{case['id']} has unsupported comparison kind", file=sys.stderr)
            return 1
        if reported.get("fixture_ids") != fixture_ids_for_case:
            print(f"{case['id']} report fixture coverage drifted", file=sys.stderr)
            return 1
        if reported.get("reference_status") != "pass":
            print(f"{case['id']} lacks a passing recorded reference outcome", file=sys.stderr)
            return 1
        import_blocked = any(
            fixture_gate_by_id[fixture_id] == "import-blocked"
            for fixture_id in fixture_ids_for_case
        )
        expected_comparison = (
            "reference-pass/flux-import-blocked"
            if import_blocked
            else "reference-pass/flux-import-pass"
        )
        if reported.get("comparison") != expected_comparison:
            print(f"{case['id']} comparison status no longer matches fixture gates", file=sys.stderr)
            return 1
    conclusion = reference_report.get("conclusion", {})
    if not (
        conclusion.get("reference_cases_passed") == len(comparison_cases)
        and conclusion.get("reference_cases_total") == len(comparison_cases)
    ):
        print("reference comparison conclusion does not match its case inventory", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
