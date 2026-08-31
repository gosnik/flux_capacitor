#!/usr/bin/env python3
"""Render every enabled copper layer of an accepted KiCad board as a PNG."""

from __future__ import annotations

import argparse
import json
import re
import shutil
import subprocess
import tempfile
from pathlib import Path


COPPER_LAYER = re.compile(r'^\s*\(\d+\s+"([^"]+\.Cu)"\s+')


def copper_layers(board: Path) -> list[str]:
    layers: list[str] = []
    in_layers = False
    for line in board.read_text(encoding="utf-8").splitlines():
        stripped = line.lstrip()
        if not in_layers:
            if stripped.startswith("(layers"):
                in_layers = True
            continue
        if stripped.startswith(")"):
            break
        match = COPPER_LAYER.match(line)
        if match:
            layers.append(match.group(1))
    if not layers:
        raise RuntimeError(f"no enabled copper layers found in {board}")
    return layers


def run(command: list[str]) -> None:
    completed = subprocess.run(command, check=False, capture_output=True, text=True)
    if completed.returncode != 0:
        raise RuntimeError(
            f"command failed ({completed.returncode}): {' '.join(command)}\n"
            f"stdout:\n{completed.stdout}\nstderr:\n{completed.stderr}"
        )


def render(board: Path, output_dir: Path, kicad_cli: Path, inkscape: Path, width: int) -> list[Path]:
    layers = copper_layers(board)
    output_dir.mkdir(parents=True, exist_ok=True)
    outputs: list[Path] = []
    # Snap-packaged Inkscape has a private /tmp. Keep intermediates beside the requested outputs
    # so both native and confined renderer installations see the same absolute path.
    with tempfile.TemporaryDirectory(prefix=".flux-copper-render-", dir=output_dir) as temporary:
        temporary_dir = Path(temporary)
        for index, layer in enumerate(layers):
            safe_layer = re.sub(r"[^A-Za-z0-9_.-]+", "_", layer).replace(".", "_")
            stem = f"{index:02d}-{safe_layer}"
            svg = temporary_dir / f"{stem}.svg"
            png = output_dir / f"{stem}.png"
            run(
                [
                    str(kicad_cli),
                    "pcb",
                    "export",
                    "svg",
                    "--mode-single",
                    "--layers",
                    f"{layer},Edge.Cuts",
                    "--page-size-mode",
                    "2",
                    "--fit-page-to-board",
                    "--exclude-drawing-sheet",
                    "--drill-shape-opt",
                    "2",
                    "--output",
                    str(svg),
                    str(board),
                ]
            )
            run(
                [
                    str(inkscape),
                    str(svg),
                    "--export-type=png",
                    f"--export-width={width}",
                    f"--export-filename={png}",
                ]
            )
            if not png.is_file() or png.stat().st_size == 0:
                raise RuntimeError(f"renderer did not produce {png}")
            outputs.append(png)
    manifest = {
        "schema": 1,
        "source_board": str(board),
        "layers": [
            {"layer": layer, "png": output.name} for layer, output in zip(layers, outputs)
        ],
    }
    (output_dir / "copper-layers.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    return outputs


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--board", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--kicad-cli", required=True, type=Path)
    parser.add_argument("--inkscape", required=True, type=Path)
    parser.add_argument("--width", type=int, default=2400)
    args = parser.parse_args()
    if args.width <= 0:
        raise RuntimeError("--width must be positive")
    for executable, label in ((args.kicad_cli, "kicad-cli"), (args.inkscape, "Inkscape")):
        if not executable.is_file():
            raise RuntimeError(f"missing {label}: {executable}")
    board = args.board.resolve()
    if not board.is_file():
        raise RuntimeError(f"missing board: {board}")
    outputs = render(
        board,
        args.output_dir.resolve(),
        args.kicad_cli,
        args.inkscape,
        args.width,
    )
    print(f"rendered {len(outputs)} copper layers to {args.output_dir.resolve()}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError) as error:
        print(f"FAIL {error}")
        raise SystemExit(1)
