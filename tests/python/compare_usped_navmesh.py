#!/usr/bin/env python3
"""Bounded, sequential USPED on/off global-stage experiment; never qualifies copper."""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--flux-cli", required=True, type=Path)
    parser.add_argument("--board", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--timeout", type=float, default=120)
    parser.add_argument("--router-args", nargs=argparse.REMAINDER, default=[])
    args = parser.parse_args()
    if args.timeout <= 0 or args.output.exists() or "--usped-navmesh" in args.router_args:
        parser.error("require positive timeout, a new output file and no preselected USPED flag")
    result = {
        "schema": "flux_usped_global_ablation_v1",
        "date": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "git_revision": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
        "dirty_worktree": bool(subprocess.check_output(["git", "status", "--porcelain"], text=True)),
        "board": str(args.board.resolve()),
        "board_sha256": hashlib.sha256(args.board.read_bytes()).hexdigest(),
        "binary_sha256": hashlib.sha256(args.flux_cli.read_bytes()).hexdigest(),
        "scope": "global only; no detailed copper, external DRC or tier qualification",
        "runs": [],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as output:
        for enabled in (False, True):
            command = [str(args.flux_cli.resolve()), "route-kicad", str(args.board.resolve()),
                       *args.router_args, *(["--usped-navmesh"] if enabled else [])]
            record = {"enabled": enabled, "command": command, "timeout_seconds": args.timeout}
            record["binary_sha256"] = hashlib.sha256(args.flux_cli.read_bytes()).hexdigest()
            record["matches_initial_binary"] = record["binary_sha256"] == result["binary_sha256"]
            started = time.monotonic()
            try:
                run = subprocess.run(command, capture_output=True, text=True, timeout=args.timeout)
                record.update(returncode=run.returncode, stderr=run.stderr)
                try:
                    record["result"] = json.loads(run.stdout)
                except json.JSONDecodeError:
                    record["invalid_json_stdout"] = run.stdout
            except subprocess.TimeoutExpired:
                record["timeout"] = True
            record["wall_seconds"] = time.monotonic() - started
            result["runs"].append(record)
            output.seek(0)
            json.dump(result, output, indent=2)
            output.truncate()
            output.flush()
            print(json.dumps({key: value for key, value in record.items()
                              if key not in ("result", "stderr")} ), flush=True)


if __name__ == "__main__":
    main()
