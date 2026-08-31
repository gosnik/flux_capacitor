#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import pathlib
import types
import unittest


MODULE_PATH = (
    pathlib.Path(__file__).resolve().parents[2] / "plugins" / "kicad" / "export_netclasses.py"
)
SPEC = importlib.util.spec_from_file_location("flux_export_netclasses", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
BRIDGE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BRIDGE)


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
            "revision", "9.0.8", [default], [signal], {"SIGNAL": default}
        )
        self.assertIn('(api_version "9.0.8")', snapshot)
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
        first = BRIDGE.build_snapshot("revision", "10.0.1", [physical, default, high], nets,
                                      effective)
        second = BRIDGE.build_snapshot("revision", "10.0.1", [high, physical, default],
                                       list(reversed(nets)), effective)
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
            BRIDGE.build_snapshot("revision", "10", [default],
                                  [types.SimpleNamespace(name="SIGNAL")], {})

    def test_unknown_constituent_fails_closed(self):
        default = net_class(
            "Default", 2_147_483_647, 200_000, 250_000, 600_000, 300_000,
            250_000, 200_000, 250_000
        )
        net = types.SimpleNamespace(name="SIGNAL")
        effective = {"SIGNAL": net_class("effective", -1, constituents=["Missing"])}
        with self.assertRaisesRegex(ValueError, "undefined constituent"):
            BRIDGE.build_snapshot("revision", "10", [default], [net], effective)


if __name__ == "__main__":
    unittest.main()
