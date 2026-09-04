#!/usr/bin/env python3
"""Exercise the write-once, source-bound routing baseline preflight."""

import argparse
import hashlib
import json
import subprocess
import sys
import tempfile
from pathlib import Path


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--flux-cli", required=True, type=Path)
    parser.add_argument("--corpus", required=True, type=Path)
    parser.add_argument("--verifier", required=True, type=Path)
    args = parser.parse_args()

    with tempfile.TemporaryDirectory(prefix="flux-baseline-test-") as temporary:
        artifacts = Path(temporary) / "artifacts"
        command = [
            sys.executable,
            str(args.verifier),
            "--flux-cli",
            str(args.flux_cli),
            "--corpus",
            str(args.corpus),
            "--artifacts",
            str(artifacts),
            "--board",
            "pt2399_delay",
            "--preflight-only",
        ]
        first = subprocess.run(command, check=False, capture_output=True, text=True, timeout=60)
        if first.returncode != 0:
            print(first.stdout, file=sys.stderr)
            print(first.stderr, file=sys.stderr)
            return 1

        metadata = json.loads((artifacts / "run-metadata.json").read_text(encoding="utf-8"))
        result = json.loads((artifacts / "run-result.json").read_text(encoding="utf-8"))
        manifest_path = args.corpus / "manifest.json"
        if not (
            metadata["mode"] == "preflight"
            and metadata["environment"]["flux"]["gpu_available"] is True
            and metadata["executable"]["sha256"] == digest(args.flux_cli)
            and metadata["corpus"]["manifest_sha256"] == digest(manifest_path)
            and result["status"] == "preflight_complete"
            and len(result["boards"]) == 1
            and result["boards"][0]["source_revision"]
        ):
            print("baseline metadata omitted required source/tool/GPU evidence", file=sys.stderr)
            return 1
        for artifact in result["artifacts"]:
            path = artifacts / artifact["path"]
            if not (
                path.is_file()
                and path.stat().st_size == artifact["bytes"]
                and digest(path) == artifact["sha256"]
            ):
                print(f"baseline artifact inventory mismatch: {path}", file=sys.stderr)
                return 1

        second = subprocess.run(command, check=False, capture_output=True, text=True, timeout=60)
        if second.returncode == 0:
            print("baseline artifact destination was overwritten", file=sys.stderr)
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
