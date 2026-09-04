#!/usr/bin/env python3
"""Record source-bound KiCadRoutingTools outcomes beside Flux import behavior."""

from __future__ import annotations

import argparse
import datetime
import hashlib
import json
import os
import platform
import shlex
import shutil
import subprocess
import sys
import time
from pathlib import Path


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


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


def run_capture(
    command: list[str],
    cwd: Path,
    case_dir: Path,
    stem: str,
    timeout: int,
    environment: dict[str, str] | None = None,
) -> dict:
    started = time.monotonic()
    completed = subprocess.run(
        command,
        cwd=cwd,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
        timeout=timeout,
    )
    duration = time.monotonic() - started
    stdout_path = case_dir / f"{stem}.stdout"
    stderr_path = case_dir / f"{stem}.stderr"
    stdout_path.write_text(completed.stdout, encoding="utf-8")
    stderr_path.write_text(completed.stderr, encoding="utf-8")
    return {
        "command": command,
        "returncode": completed.returncode,
        "duration_seconds": duration,
        "stdout": stdout_path.relative_to(case_dir.parent.parent).as_posix(),
        "stdout_sha256": sha256(stdout_path),
        "stderr": stderr_path.relative_to(case_dir.parent.parent).as_posix(),
        "stderr_sha256": sha256(stderr_path),
        "stdout_text": completed.stdout,
        "stderr_text": completed.stderr,
    }


def public_run(record: dict) -> dict:
    return {key: value for key, value in record.items() if not key.endswith("_text")}


def kicad_command(executable: Path, arguments: list[str]) -> list[str]:
    if executable.suffix.lower() == ".appimage":
        return [str(executable), "--appimage-extract-and-run", "kicad-cli", *arguments]
    return [str(executable), *arguments]


def make_kicad_wrapper(artifacts: Path, executable: Path) -> Path:
    wrapper = artifacts / "tools" / "kicad-cli"
    wrapper.parent.mkdir(parents=True, exist_ok=True)
    if executable.suffix.lower() == ".appimage":
        command = (
            f"exec {shlex.quote(str(executable))} --appimage-extract-and-run "
            'kicad-cli "$@"\n'
        )
    else:
        command = f'exec {shlex.quote(str(executable))} "$@"\n'
    wrapper.write_text("#!/usr/bin/env bash\n" + command, encoding="utf-8")
    wrapper.chmod(0o755)
    return wrapper


def inspect_flux(flux_cli: Path, board: Path) -> dict:
    completed = subprocess.run(
        [str(flux_cli), "inspect-kicad", str(board)],
        check=False,
        capture_output=True,
        text=True,
        timeout=60,
    )
    require(completed.stdout, f"Flux inspection produced no JSON for {board}")
    result = json.loads(completed.stdout)
    return {
        "command": ["flux-cli", "inspect-kicad", board.name],
        "returncode": completed.returncode,
        "strict": result.get("strict"),
        "source_revision": result.get("source_revision"),
        "import_issue_counts": result.get("import_issue_counts", {}),
        "core_validation_errors": result.get("core_validation_errors"),
    }


def artifact_inventory(root: Path) -> list[dict]:
    result = []
    for path in sorted(candidate for candidate in root.rglob("*") if candidate.is_file()):
        if path.name == "comparison-result.json":
            continue
        result.append(
            {
                "path": path.relative_to(root).as_posix(),
                "bytes": path.stat().st_size,
                "sha256": sha256(path),
            }
        )
    return result


def run_route_case(
    case: dict,
    fixture: dict,
    project_root: Path,
    reference: Path,
    python: Path,
    kicad_cli: Path,
    case_dir: Path,
) -> dict:
    source_record = next(
        record for record in fixture["files"] if record["path"].endswith(".kicad_pcb")
    )
    source = project_root / source_record["path"]
    staged_source = case_dir / "source.kicad_pcb"
    routed = case_dir / "routed.kicad_pcb"
    route_json = case_dir / "route-result.json"
    shutil.copy2(source, staged_source)

    route = run_capture(
        [
            str(python),
            str(reference / "py_router" / "route.py"),
            str(staged_source),
            "--output",
            str(routed),
            "--json-out",
            str(route_json),
            *case["route_args"],
        ],
        reference,
        case_dir,
        "route",
        600,
    )
    require(route["returncode"] == 0, f"{case['id']}: reference route failed")
    route_result = json.loads(route_json.read_text(encoding="utf-8"))

    connectivity = run_capture(
        [
            str(python),
            str(reference / "py_router" / "check_connected.py"),
            str(routed),
            "--routed-only",
            "--verbose",
        ],
        reference,
        case_dir,
        "connectivity",
        300,
    )
    require(
        connectivity["returncode"] == 0
        and "ALL NETS FULLY CONNECTED" in connectivity["stdout_text"],
        f"{case['id']}: reference connectivity failed",
    )

    krt_drc_json = case_dir / "krt-drc.json"
    krt_drc = run_capture(
        [
            str(python),
            str(reference / "py_router" / "check_drc.py"),
            str(routed),
            "--json",
            str(krt_drc_json),
            "--quiet",
        ],
        reference,
        case_dir,
        "krt-drc",
        300,
    )
    require(krt_drc["returncode"] == 0, f"{case['id']}: reference DRC failed")
    krt_drc_result = json.loads(krt_drc_json.read_text(encoding="utf-8"))

    kicad_drc_json = case_dir / "kicad-drc.json"
    kicad_drc = run_capture(
        kicad_command(
            kicad_cli,
            [
                "pcb",
                "drc",
                "--format",
                "json",
                "--output",
                str(kicad_drc_json),
                "--severity-all",
                "--exit-code-violations",
                str(routed),
            ],
        ),
        project_root,
        case_dir,
        "kicad-drc",
        300,
    )
    require(kicad_drc["returncode"] == 0, f"{case['id']}: external KiCad DRC failed")
    kicad_drc_result = json.loads(kicad_drc_json.read_text(encoding="utf-8"))

    expected = case["expected"]
    require(
        sorted(route_result.get("routed_single", [])) == expected["routed_single"]
        and route_result.get("failed") == expected["failed"]
        and route_result.get("open_single") == expected["open_single"]
        and route_result.get("pad_pairs_connected") == expected["pad_pairs_connected"]
        and route_result.get("pad_pairs_total") == expected["pad_pairs_total"]
        and route_result.get("total_vias") == expected["total_vias"],
        f"{case['id']}: route behavior differs from the pinned expectation",
    )
    require(
        krt_drc_result.get("violations") == expected["krt_drc_violations"],
        f"{case['id']}: reference DRC count drifted",
    )
    require(
        len(kicad_drc_result.get("violations", [])) == expected["kicad_drc_violations"]
        and len(kicad_drc_result.get("unconnected_items", []))
        == expected["kicad_unconnected_items"],
        f"{case['id']}: external KiCad result drifted",
    )
    return {
        "status": "pass",
        "route": public_run(route),
        "route_result": route_result,
        "connectivity": public_run(connectivity),
        "krt_drc": {
            "run": public_run(krt_drc),
            "violations": krt_drc_result.get("violations"),
            "graded_at": krt_drc_result.get("graded_at"),
        },
        "kicad_drc": {
            "run": public_run(kicad_drc),
            "kicad_version": kicad_drc_result.get("kicad_version"),
            "violations": len(kicad_drc_result.get("violations", [])),
            "unconnected_items": len(kicad_drc_result.get("unconnected_items", [])),
        },
        "feature_behavior": {
            "scope": route_result.get("scope"),
            "routed_single": sorted(route_result.get("routed_single", [])),
            "ordering": "mps",
            "vias": route_result.get("total_vias"),
        },
    }


def run_upstream_test_case(
    case: dict,
    fixtures: dict[str, dict],
    project_root: Path,
    reference: Path,
    python: Path,
    wrapper: Path,
    case_dir: Path,
) -> dict:
    test_path = reference / case["test"]
    require(test_path.is_file(), f"{case['id']}: missing upstream test {case['test']}")
    environment = dict(os.environ)
    environment["KICAD_CLI"] = str(wrapper)
    command = [str(python), str(test_path)]
    if case.get("stage_fixture_directory"):
        staged = case_dir / "staged-fixtures"
        staged.mkdir()
        for fixture_id in case["fixture_ids"]:
            fixture = fixtures[fixture_id]
            board_record = next(
                record for record in fixture["files"] if record["path"].endswith(".kicad_pcb")
            )
            shutil.copy2(project_root / board_record["path"], staged / Path(board_record["path"]).name)
        command = [
            str(python),
            "-c",
            "import runpy,sys; n=runpy.run_path(sys.argv[1], run_name='reference_case'); "
            "n['main'].__globals__['FIXTURES']=sys.argv[2]; n['main']()",
            str(test_path),
            str(staged),
        ]
    run = run_capture(
        command,
        reference,
        case_dir,
        "upstream-test",
        1200,
        environment,
    )
    require(run["returncode"] == 0, f"{case['id']}: upstream behavior test failed")
    require(
        case["expected_stdout"] in run["stdout_text"],
        f"{case['id']}: expected behavior evidence was not emitted",
    )
    return {
        "status": "pass",
        "test": case["test"],
        "test_sha256": sha256(test_path),
        "run": public_run(run),
        "feature_behavior": case["expected_stdout"],
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference-checkout", required=True, type=Path)
    parser.add_argument("--python", required=True, type=Path)
    parser.add_argument("--flux-cli", required=True, type=Path)
    parser.add_argument("--kicad-cli", required=True, type=Path)
    parser.add_argument("--fixtures", required=True, type=Path)
    parser.add_argument("--cases", required=True, type=Path)
    parser.add_argument("--artifacts", required=True, type=Path)
    parser.add_argument("--case", action="append", default=[])
    args = parser.parse_args()

    project_root = args.fixtures.resolve().parents[3]
    reference = args.reference_checkout.resolve()
    # Keep the venv launcher path: resolving its interpreter symlink would bypass the venv's
    # pyvenv.cfg and silently lose the pinned reference dependencies.
    python = args.python.absolute()
    flux_cli = args.flux_cli.resolve()
    kicad_cli = args.kicad_cli.resolve()
    artifacts = args.artifacts.resolve()
    require(reference.is_dir(), "reference checkout does not exist")
    require(python.is_file(), "reference Python interpreter does not exist")
    require(flux_cli.is_file(), "Flux CLI does not exist")
    require(kicad_cli.is_file(), "KiCad CLI/AppImage does not exist")
    artifacts.mkdir(parents=True, exist_ok=False)

    fixture_catalog = json.loads(args.fixtures.read_text(encoding="utf-8"))
    case_catalog = json.loads(args.cases.read_text(encoding="utf-8"))
    expected_commit = case_catalog["reference"]["commit"]
    actual_commit = subprocess.run(
        ["git", "-C", str(reference), "rev-parse", "HEAD"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    require(actual_commit == expected_commit, "reference checkout revision does not match catalog")
    require(
        fixture_catalog["reference"]["commit"] == expected_commit,
        "fixture and comparison catalogs pin different reference revisions",
    )

    fixtures = {fixture["id"]: fixture for fixture in fixture_catalog["fixtures"]}
    cases = case_catalog["cases"]
    if args.case:
        requested = set(args.case)
        cases = [case for case in cases if case["id"] in requested]
        require(len(cases) == len(requested), "unknown or duplicate comparison case requested")
    require(cases, "no comparison cases selected")

    for case in cases:
        for fixture_id in case["fixture_ids"]:
            require(fixture_id in fixtures, f"{case['id']}: unknown fixture {fixture_id}")
            fixture = fixtures[fixture_id]
            for record in fixture["files"]:
                local = project_root / record["path"]
                require(local.is_file(), f"missing comparison fixture {local}")
                require(sha256(local) == record["sha256"], f"fixture hash drifted: {local}")
            if fixture["origin"] == "KiCadRoutingTools":
                upstream = reference / fixture["upstream_path"]
                board_record = next(
                    record for record in fixture["files"] if record["path"].endswith(".kicad_pcb")
                )
                require(upstream.is_file(), f"missing upstream fixture {upstream}")
                require(
                    sha256(upstream) == board_record["sha256"],
                    f"upstream fixture bytes drifted: {upstream}",
                )

    wrapper = make_kicad_wrapper(artifacts, kicad_cli)
    kicad_version = subprocess.run(
        kicad_command(kicad_cli, ["--version"]),
        check=True,
        capture_output=True,
        text=True,
        timeout=120,
    ).stdout.strip()
    python_identity = subprocess.run(
        [
            str(python),
            "-c",
            "import json,platform,numpy,scipy,shapely; "
            "print(json.dumps({'python':platform.python_version(),"
            "'numpy':numpy.__version__,'scipy':scipy.__version__,"
            "'shapely':shapely.__version__}))",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    metadata = {
        "schema": "flux_reference_comparison_run",
        "schema_version": 1,
        "created_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "host": platform.platform(),
        "reference": {
            **case_catalog["reference"],
            "checkout": str(reference),
            "actual_commit": actual_commit,
        },
        "tools": {
            "python": json.loads(python_identity.stdout),
            "python_executable_sha256": sha256(python),
            "flux_cli_sha256": sha256(flux_cli),
            "kicad_cli": kicad_version,
            "kicad_cli_sha256": sha256(kicad_cli),
        },
        "contracts": {
            "fixtures_sha256": sha256(args.fixtures),
            "cases_sha256": sha256(args.cases),
        },
        "selected_cases": [case["id"] for case in cases],
    }
    write_json_once(artifacts / "run-metadata.json", metadata)

    flux_inspections: dict[str, dict] = {}
    for case in cases:
        for fixture_id in case["fixture_ids"]:
            if fixture_id in flux_inspections:
                continue
            fixture = fixtures[fixture_id]
            board_record = next(
                record for record in fixture["files"] if record["path"].endswith(".kicad_pcb")
            )
            flux_inspections[fixture_id] = inspect_flux(
                flux_cli, project_root / board_record["path"]
            )

    results = []
    for case in cases:
        case_dir = artifacts / "cases" / case["id"]
        case_dir.mkdir(parents=True)
        if case["kind"] == "route":
            reference_result = run_route_case(
                case,
                fixtures[case["fixture_ids"][0]],
                project_root,
                reference,
                python,
                kicad_cli,
                case_dir,
            )
        elif case["kind"] == "upstream-test":
            reference_result = run_upstream_test_case(
                case, fixtures, project_root, reference, python, wrapper, case_dir
            )
        else:
            raise RuntimeError(f"{case['id']}: unsupported comparison kind {case['kind']}")
        flux_results = {
            fixture_id: flux_inspections[fixture_id] for fixture_id in case["fixture_ids"]
        }
        results.append(
            {
                "id": case["id"],
                "description": case["description"],
                "capabilities": case["capabilities"],
                "fixture_ids": case["fixture_ids"],
                "reference": reference_result,
                "flux": flux_results,
                "comparison": (
                    "reference-pass/flux-import-blocked"
                    if any(not value["strict"] for value in flux_results.values())
                    else "reference-pass/flux-import-pass"
                ),
            }
        )
        print(f"PASS {case['id']}")

    write_json_once(
        artifacts / "comparison-result.json",
        {
            "schema": "flux_reference_comparison_result",
            "schema_version": 1,
            "status": "complete",
            "methodology": {
                "compare_byte_identical_copper": False,
                "compare_connectivity": True,
                "compare_drc": True,
                "compare_route_scope": True,
                "compare_feature_behavior": True,
                "flux_import_is_authoritative_before_routing": True,
            },
            "cases": results,
            "artifacts": artifact_inventory(artifacts),
        },
    )
    print(f"{len(results)}/{len(results)} reference comparison cases passed")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, subprocess.SubprocessError, OSError, ValueError) as error:
        print(f"reference comparison failed: {error}", file=sys.stderr)
        raise SystemExit(1)
