#!/usr/bin/env python3
"""Contract tests for saved-fill-only KiCad zone normalization."""

from __future__ import annotations

import contextlib
import importlib.util
import io
import json
import pathlib
import sys
import tempfile
import types
import unittest
from unittest.mock import patch


MODULE_PATH = pathlib.Path(__file__).resolve().parents[2] / "plugins" / "kicad" / "refill_zones.py"
SPEC = importlib.util.spec_from_file_location("flux_refill_zones", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
REFILL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(REFILL)


class FakePolygons:
    def __init__(self, *, self_intersecting: bool, vertices: int = 12):
        self.self_intersecting = self_intersecting
        self.vertices = vertices
        self.calls: list[str] = []

    def IsSelfIntersecting(self):
        return self.self_intersecting

    def TotalVertices(self):
        return self.vertices

    def Unfracture(self):
        self.calls.append("unfracture")

    def Simplify(self):
        self.calls.append("simplify")

    def Fracture(self):
        self.calls.append("fracture")
        self.vertices -= 2


class FakeZone:
    def __init__(self, uuid: str, layers: dict[int, FakePolygons], *, lose_on_set: bool = False):
        self.m_Uuid = types.SimpleNamespace(AsString=lambda: uuid)
        self.layers = layers
        self.lose_on_set = lose_on_set
        self.set_layers: list[int] = []

    def GetIsRuleArea(self):
        return False

    def GetLayerSet(self):
        return types.SimpleNamespace(Seq=lambda: list(self.layers))

    def HasFilledPolysForLayer(self, layer):
        return layer in self.layers

    def GetFilledPolysList(self, layer):
        return self.layers[layer]

    def SetFilledPolysList(self, layer, polygons):
        self.set_layers.append(layer)
        if self.lose_on_set:
            del self.layers[layer]
        else:
            self.layers[layer] = polygons


class FakeBoard:
    def __init__(self, zones):
        self._zones = zones

    def Zones(self):
        return self._zones

    def GetLayerName(self, layer):
        return {0: "F.Cu", 31: "B.Cu"}[layer]


class ZoneNormalizationTests(unittest.TestCase):
    def run_normalize(self, board: FakeBoard, *, dangling_output: bool = False):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        directory = pathlib.Path(temporary.name)
        source = directory / "source.kicad_pcb"
        output = directory / "normalized.kicad_pcb"
        source.write_text("(kicad_pcb)", encoding="utf-8")
        if dangling_output:
            output.symlink_to(directory / "missing-target.kicad_pcb")

        save_calls: list[pathlib.Path] = []

        def save_board(filename, saved_board):
            self.assertIs(saved_board, board)
            destination = pathlib.Path(filename)
            save_calls.append(destination)
            destination.write_text("saved", encoding="utf-8")
            return True

        pcbnew = types.SimpleNamespace(
            LoadBoard=lambda filename: board,
            SaveBoard=save_board,
            Version=lambda: "mock-9.0",
        )
        stdout = io.StringIO()
        with patch.dict(sys.modules, {"pcbnew": pcbnew}), contextlib.redirect_stdout(stdout):
            code = REFILL.main(
                ["--board", str(source), "--output", str(output), "--normalize-saved-fills-only"]
            )
        return code, output, save_calls, json.loads(stdout.getvalue())

    def test_only_flagged_saved_fill_layers_are_normalized(self):
        flagged = FakePolygons(self_intersecting=True, vertices=12)
        untouched = FakePolygons(self_intersecting=False, vertices=7)
        zone = FakeZone("zone-1", {0: flagged, 31: untouched})

        code, output, save_calls, report = self.run_normalize(FakeBoard([zone]))

        self.assertEqual(code, 0)
        self.assertTrue(output.is_file())
        self.assertEqual(save_calls, [output])
        self.assertEqual(flagged.calls, ["unfracture", "simplify", "fracture"])
        self.assertEqual(untouched.calls, [])
        self.assertEqual(zone.set_layers, [0])
        self.assertEqual(report["zone_layers_normalized"], 1)
        self.assertEqual(report["normalized_fills"], [{"zone_uuid": "zone-1", "layer": "F.Cu",
                                                          "vertices_before": 12, "vertices_after": 10}])

    def test_empty_normalization_fails_without_output(self):
        emptied = FakePolygons(self_intersecting=True, vertices=2)

        code, output, save_calls, report = self.run_normalize(
            FakeBoard([FakeZone("zone-1", {0: emptied})]))

        self.assertEqual(code, 2)
        self.assertFalse(output.exists())
        self.assertEqual(save_calls, [])
        self.assertFalse(report["ok"])

    def test_lost_layer_after_set_fails_without_output(self):
        flagged = FakePolygons(self_intersecting=True)

        code, output, save_calls, report = self.run_normalize(
            FakeBoard([FakeZone("zone-1", {0: flagged}, lose_on_set=True)])
        )

        self.assertEqual(code, 2)
        self.assertFalse(output.exists())
        self.assertEqual(save_calls, [])
        self.assertFalse(report["ok"])

    def test_dangling_output_symlink_fails_without_creating_target(self):
        flagged = FakePolygons(self_intersecting=True)

        code, output, save_calls, report = self.run_normalize(
            FakeBoard([FakeZone("zone-1", {0: flagged})]), dangling_output=True
        )

        self.assertEqual(code, 2)
        self.assertTrue(output.is_symlink())
        self.assertFalse(output.resolve(strict=False).exists())
        self.assertEqual(save_calls, [])
        self.assertFalse(report["ok"])


if __name__ == "__main__":
    unittest.main()
