#!/usr/bin/env python3
"""Compare strict Flux pad geometry with KiCad's independent board loader.

Requires pcbnew (run with KiCad's system Python). This is a geometry check, not
routing qualification. Original board/project files are only read.
"""
import argparse
import json
from pathlib import Path
import subprocess

import pcbnew


def compare(inspector, path):
    result = subprocess.run([str(inspector), str(path)], check=True,
                            text=True, capture_output=True)
    imported = json.loads(result.stdout)
    board = pcbnew.LoadBoard(str(path))
    if board is None:
        raise RuntimeError(f"KiCad could not read {path}")
    expected = {pad.m_Uuid.AsString(): pad for footprint in board.GetFootprints()
                for pad in footprint.Pads() if pad.GetLayerSet().CuStack()}
    actual = {row["id"]: row for row in imported["pads"]}
    mismatches = []
    if len(actual) != len(imported["pads"]) or set(actual) != set(expected):
        mismatches.append({"field": "pad_identity", "missing": sorted(set(expected) - set(actual)),
                           "extra": sorted(set(actual) - set(expected))})
    for identity in sorted(set(actual) & set(expected)):
        row, pad = actual[identity], expected[identity]
        position, size = pad.ShapePos(pad.GetLayer()), pad.GetSize()
        dimensions = (size.x, size.y)
        if pad.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
            position = pad.GetPosition()
            dimensions = (pad.GetDrillSize().x, pad.GetDrillSize().y)
        values = {"x_nm": position.x, "y_nm": position.y,
                  "width_nm": dimensions[0], "height_nm": dimensions[1],
                  "clockwise_microdegrees": round(pad.GetOrientationDegrees() * 1_000_000)}
        if pad.GetDrillSize().x and pad.GetDrillSize().y:
            values["drill_x_nm"] = pad.GetPosition().x
            values["drill_y_nm"] = pad.GetPosition().y
        for field, value in values.items():
            lhs, rhs = row.get(field), value
            if field == "clockwise_microdegrees":
                lhs, rhs = lhs % 360_000_000, rhs % 360_000_000
            # This corpus uses exact integer geometry; no coordinate tolerance is applied.
            if lhs != rhs:
                mismatches.append({"id": identity, "reference": row["reference"],
                                   "field": field, "flux": lhs, "kicad": rhs})
    return {"board": str(path), "source_revision": imported["source_revision"],
            "pads_compared": len(expected), "passed": not mismatches,
            "mismatches": mismatches}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inspector", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("boards", type=Path, nargs="+")
    args = parser.parse_args()
    reports = [compare(args.inspector.resolve(), board) for board in args.boards]
    result = {"schema": 1, "kicad_version": pcbnew.GetBuildVersion(),
              "passed": all(row["passed"] for row in reports), "boards": reports}
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(result, indent=2) + "\n")
    for row in reports:
        print(f"{'PASS' if row['passed'] else 'FAIL'} {row['board']}: "
              f"{row['pads_compared']} pads, {len(row['mismatches'])} mismatches")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
