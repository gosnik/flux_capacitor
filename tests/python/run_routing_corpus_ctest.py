#!/usr/bin/env python3
"""Run a corpus tier in a disposable write-once artifact directory for CTest."""

import argparse
import subprocess
import sys
import tempfile
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--verifier", required=True, type=Path)
    parser.add_argument("--flux-cli", required=True, type=Path)
    parser.add_argument("--corpus", required=True, type=Path)
    parser.add_argument("--level", required=True)
    parser.add_argument("--kicad-cli", required=True, type=Path)
    parser.add_argument("--inkscape", required=True, type=Path)
    args = parser.parse_args()

    with tempfile.TemporaryDirectory(prefix=f"flux-corpus-{args.level}-") as temporary:
        completed = subprocess.run(
            [
                sys.executable,
                str(args.verifier),
                "--flux-cli",
                str(args.flux_cli),
                "--corpus",
                str(args.corpus),
                "--level",
                args.level,
                "--artifacts",
                str(Path(temporary) / "artifacts"),
                "--kicad-cli",
                str(args.kicad_cli),
                "--inkscape",
                str(args.inkscape),
            ],
            check=False,
        )
        return completed.returncode


if __name__ == "__main__":
    raise SystemExit(main())
