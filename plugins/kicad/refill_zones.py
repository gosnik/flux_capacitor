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
    if output.exists() or output.is_symlink():
        raise RefillError(f"refilled output already exists: {output}")
    return output


def _write_report(path: pathlib.Path | None, report: dict[str, object]) -> str:
    serialized = json.dumps(report, indent=2, sort_keys=True)
    if path is not None:
        parent = path.expanduser().parent.resolve(strict=True)
        destination = parent / path.name
        with destination.open("x", encoding="utf-8") as stream:
            stream.write(serialized)
            stream.write("\n")
    print(serialized)
    return serialized


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="refill zones in a provisional Flux candidate using KiCad's zone filler"
    )
    parser.add_argument("--board", required=True, type=pathlib.Path)
    parser.add_argument("--output", required=True, type=pathlib.Path)
    parser.add_argument(
        "--report",
        type=pathlib.Path,
        help="write the bounded machine-readable result to a new file",
    )
    parser.add_argument(
        "--normalize-saved-fills-only", action="store_true",
        help="normalize existing filled polygons with KiCad without recomputing pours",
    )
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
        normalized_layers = 0
        normalized_fills: list[dict[str, object]] = []
        if args.normalize_saved_fills_only:
            # Use KiCad's own integer-grid interpretation of fractured fills. Raw saved
            # clipping walks can contain sub-grid intersections that Flux cannot represent
            # as simple contours. Do not change zone definitions or refill their copper.
            for zone in zones:
                if zone.GetIsRuleArea():
                    continue
                for layer in zone.GetLayerSet().Seq():
                    if not zone.HasFilledPolysForLayer(layer):
                        continue
                    polygons = zone.GetFilledPolysList(layer)
                    if not polygons.IsSelfIntersecting():
                        continue
                    vertices_before = polygons.TotalVertices()
                    polygons.Unfracture()
                    polygons.Simplify()
                    polygons.Fracture()
                    if polygons.TotalVertices() == 0:
                        raise RefillError("normalization removed all copper from a saved zone layer")
                    zone.SetFilledPolysList(layer, polygons)
                    if not zone.HasFilledPolysForLayer(layer):
                        raise RefillError("normalization lost a saved zone layer")
                    normalized_layers += 1
                    normalized_fills.append({
                        "zone_uuid": zone.m_Uuid.AsString(),
                        "layer": board.GetLayerName(layer),
                        "vertices_before": vertices_before,
                        "vertices_after": polygons.TotalVertices(),
                    })
        else:
            pcbnew.ZONE_FILLER(board).Fill(zones)
        if not pcbnew.SaveBoard(str(output), board):
            raise RefillError("KiCad did not save the refilled board")
        if output.is_symlink() or not output.is_file():
            raise RefillError("KiCad reported success without a refilled artifact")
        report.update(
            {
                "ok": True,
                "provisional": str(source),
                "candidate": str(output),
                "operation": "normalize_saved_fills" if args.normalize_saved_fills_only else "refill",
                "zones_refilled": 0 if args.normalize_saved_fills_only else len(zones),
                "zone_layers_normalized": normalized_layers,
                "normalized_fills": normalized_fills,
                "kicad_version": pcbnew.Version(),
            }
        )
        _write_report(args.report, report)
        return 0
    except (ImportError, OSError, RuntimeError) as error:
        report["error"] = str(error)
        try:
            _write_report(args.report, report)
        except OSError as report_error:
            print(json.dumps({**report, "report_error": str(report_error)}, indent=2, sort_keys=True))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
