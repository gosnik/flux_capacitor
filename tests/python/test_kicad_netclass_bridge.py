#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import json
import pathlib
import tempfile
import types
import unittest


MODULE_PATH = (
    pathlib.Path(__file__).resolve().parents[2] / "plugins" / "kicad" / "export_netclasses.py"
)
SPEC = importlib.util.spec_from_file_location("flux_export_netclasses", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
BRIDGE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BRIDGE)

BOARD_RULES = {
    field: 100_000 + index for index, (field, _) in enumerate(BRIDGE.PROJECT_RULE_FIELDS)
}


def net_class(
    name,
    priority,
    clearance=None,
    track=None,
    via=None,
    drill=None,
    diff_width=None,
    diff_gap=None,
    diff_via_gap=None,
    constituents=None,
):
    return types.SimpleNamespace(
        name=name,
        priority=priority,
        clearance=clearance,
        track_width=track,
        via_diameter=via,
        via_drill=drill,
        diff_pair_track_width=diff_width,
        diff_pair_gap=diff_gap,
        diff_pair_via_gap=diff_via_gap,
        constituents=constituents,
    )


class BridgeTests(unittest.TestCase):
    def test_kicad_9_singleton_project_classes_are_accepted(self):
        default = net_class(
            "Default", 2_147_483_647, 200_000, 250_000, 600_000, 300_000,
            250_000, 200_000, 250_000, constituents=["Default"]
        )
        signal = types.SimpleNamespace(name="SIGNAL")
        snapshot = BRIDGE.build_snapshot(
            "revision", "sha256:project", "9.0.8", BOARD_RULES,
            [default], [signal], {"SIGNAL": default}
        )
        self.assertIn('(api_version "9.0.8")', snapshot)
        self.assertIn('(project_revision "sha256:project")', snapshot)
        self.assertIn('(hole_to_hole_clearance_nm 100007)', snapshot)
        self.assertIn('(name "SIGNAL")\n    (classes)', snapshot)

    def test_snapshot_is_deterministic_complete_and_escaped(self):
        default = net_class(
            "Default", 2_147_483_647, 200_000, 250_000, 600_000, 300_000,
            250_000, 200_000, 250_000
        )
        high = net_class('A "high"', 0, 600_000)
        physical = net_class("Physical", 10, track=450_000, via=900_000, drill=400_000)
        nets = [types.SimpleNamespace(name="Z"), types.SimpleNamespace(name="A"),
                types.SimpleNamespace(name="")]
        effective = {
            "Z": net_class("Effective Z", -1, constituents=["Default", "Physical", 'A "high"']),
            "A": default,
            "": default,
        }
        first = BRIDGE.build_snapshot("revision", "sha256:project", "10.0.1", BOARD_RULES,
                                      [physical, default, high], nets, effective)
        second = BRIDGE.build_snapshot("revision", "sha256:project", "10.0.1", BOARD_RULES,
                                       [high, physical, default], list(reversed(nets)), effective)
        self.assertEqual(first, second)
        self.assertIn('(name "A \\"high\\"")', first)
        self.assertIn('(classes "A \\"high\\"" "Physical")', first)
        self.assertIn('(name "A")\n    (classes)', first)
        self.assertIn('(diff_pair_gap_nm 200000)', first)
        self.assertLess(first.index('(name "A")'), first.index('(name "Z")'))

    def test_missing_effective_net_fails_closed(self):
        default = net_class(
            "Default", 2_147_483_647, 200_000, 250_000, 600_000, 300_000,
            250_000, 200_000, 250_000
        )
        with self.assertRaisesRegex(ValueError, "omitted effective classes"):
            BRIDGE.build_snapshot("revision", "sha256:project", "10", BOARD_RULES, [default],
                                  [types.SimpleNamespace(name="SIGNAL")], {})

    def test_unknown_constituent_fails_closed(self):
        default = net_class(
            "Default", 2_147_483_647, 200_000, 250_000, 600_000, 300_000,
            250_000, 200_000, 250_000
        )
        net = types.SimpleNamespace(name="SIGNAL")
        effective = {"SIGNAL": net_class("effective", -1, constituents=["Missing"])}
        with self.assertRaisesRegex(ValueError, "undefined constituent"):
            BRIDGE.build_snapshot("revision", "sha256:project", "10", BOARD_RULES,
                                  [default], [net], effective)

    def test_saved_project_rules_are_source_bound_and_rounded_to_nanometres(self):
        rules = {project: index / 10 for index, (_, project) in
                 enumerate(BRIDGE.PROJECT_RULE_FIELDS, start=1)}
        rules["min_track_width"] = 0.19999999999999998
        document = {"board": {"design_settings": {"rules": rules}}}
        with tempfile.TemporaryDirectory() as directory:
            board = pathlib.Path(directory) / "sample.kicad_pcb"
            board.write_text("(kicad_pcb)", encoding="utf-8")
            project = board.with_suffix(".kicad_pro")
            project.write_text(json.dumps(document), encoding="utf-8")
            revision, captured, captured_path = BRIDGE.load_project_rules(board)
        self.assertEqual(captured_path, project)
        self.assertRegex(revision, r"^sha256:[0-9a-f]{64}$")
        self.assertEqual(captured["minimum_track_width_nm"], 200_000)
        self.assertEqual(captured["hole_to_hole_clearance_nm"], 800_000)


if __name__ == "__main__":
    unittest.main()
