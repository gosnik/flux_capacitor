#!/usr/bin/env python3
"""Refill every zone in a provisional KiCad board and save a new candidate."""

from __future__ import annotations

import argparse
import json
import pathlib
from collections.abc import Sequence


class RefillError(RuntimeError):
    """The bounded file-level refill transaction could not be completed."""


def _new_output(path: pathlib.Path, source: pathlib.Path) -> pathlib.Path:
    parent = path.expanduser().parent.resolve(strict=True)
    output = parent / path.name
    if output == source:
        raise RefillError("refilled output must differ from the provisional board")
    if output.exists():
        raise RefillError(f"refilled output already exists: {output}")
    return output


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="refill zones in a provisional Flux candidate using KiCad's zone filler"
    )
    parser.add_argument("--board", required=True, type=pathlib.Path)
    parser.add_argument("--output", required=True, type=pathlib.Path)
    args = parser.parse_args(argv)

    report: dict[str, object] = {"ok": False, "zone_refill_required": True}
    try:
        source = args.board.expanduser().resolve(strict=True)
        output = _new_output(args.output, source)

        # KiCad 9 has no CLI refill operation. Keep this legacy file adapter isolated here; the
        # live-board transaction remains on the official kicad-python IPC boundary.
        import pcbnew  # type: ignore[import-not-found]

        board = pcbnew.LoadBoard(str(source))
        zones = board.Zones()
        if len(zones) == 0:
            raise RefillError("provisional board contains no zones to refill")
        pcbnew.ZONE_FILLER(board).Fill(zones)
        if not pcbnew.SaveBoard(str(output), board):
            raise RefillError("KiCad did not save the refilled board")
        if not output.is_file():
            raise RefillError("KiCad reported success without a refilled artifact")
        report.update(
            {
                "ok": True,
                "provisional": str(source),
                "candidate": str(output),
                "zones_refilled": len(zones),
                "kicad_version": pcbnew.Version(),
            }
        )
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0
    except (ImportError, OSError, RuntimeError) as error:
        report["error"] = str(error)
        print(json.dumps(report, indent=2, sort_keys=True))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
