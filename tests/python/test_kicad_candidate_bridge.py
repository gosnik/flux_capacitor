#!/usr/bin/env python3

from __future__ import annotations

import copy
import importlib.util
import pathlib
import sys
import types
import unittest


MODULE_PATH = pathlib.Path(__file__).resolve().parents[2] / "plugins" / "kicad" / "apply_candidate.py"
SPEC = importlib.util.spec_from_file_location("flux_apply_candidate", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
BRIDGE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = BRIDGE
SPEC.loader.exec_module(BRIDGE)


FP_ID = "00000000-0000-0000-0000-000000000001"
TRACK_1_ID = "10000000-0000-0000-0000-000000000001"
TRACK_2_ID = "10000000-0000-0000-0000-000000000002"
ARC_ID = "20000000-0000-0000-0000-000000000001"
VIA_ID = "30000000-0000-0000-0000-000000000001"


class Point:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y


class Angle:
    def __init__(self, degrees=0.0):
        self.degrees = degrees


class FakeRoute:
    def __init__(self):
        self.proto = types.SimpleNamespace(id=types.SimpleNamespace(value=""))
        self.net = types.SimpleNamespace(name="SIGNAL")
        self.locked = False

    @property
    def id(self):
        return self.proto.id


class FakeTrack(FakeRoute):
    def __init__(self):
        super().__init__()
        self.layer = "F.Cu"
        self.start = Point()
        self.end = Point()
        self.width = 200_000


class FakeArc(FakeRoute):
    def __init__(self):
        super().__init__()
        self.layer = "F.Cu"
        self.start = Point()
        self.mid = Point()
        self.end = Point()
        self.width = 200_000


class FakeVia(FakeRoute):
    def __init__(self):
        super().__init__()
        self.position = Point()
        self.diameter = 600_000
        self.drill_diameter = 300_000
        drill = types.SimpleNamespace(start_layer="F.Cu", end_layer="B.Cu")
        self.padstack = types.SimpleNamespace(layers=["F.Cu", "B.Cu"], drill=drill)
        self.type = "through"


class FakeFootprint:
    def __init__(self):
        self.id = types.SimpleNamespace(value=FP_ID)
        text = types.SimpleNamespace(value="U1")
        self.reference_field = types.SimpleNamespace(text=text)
        self.position = Point(1_000_000, 2_000_000)
        self.orientation = Angle(90.0)
        self.layer = "F.Cu"


def make_track(item_id, start_x, end_x):
    item = FakeTrack()
    item.proto.id.value = item_id
    item.start = Point(start_x, 0)
    item.end = Point(end_x, 0)
    return item


class FakeBoard:
    def __init__(self, *, fail_create=False, mutate_on_begin=False):
        self.footprints = [FakeFootprint()]
        self.tracks = [make_track(TRACK_1_ID, 0, 1_000_000), make_track(TRACK_2_ID, 0, 2_000_000)]
        self.vias = []
        self.fail_create = fail_create
        self.mutate_on_begin = mutate_on_begin
        self.begin_calls = 0
        self.push_calls = 0
        self.drop_calls = 0
        self.remove_calls = 0
        self.update_calls = 0
        self.create_calls = 0

    def get_footprints(self):
        return list(self.footprints)

    def get_tracks(self):
        return list(self.tracks)

    def get_vias(self):
        return list(self.vias)

    def get_nets(self):
        return [types.SimpleNamespace(name="SIGNAL")]

    def begin_commit(self):
        self.begin_calls += 1
        if self.mutate_on_begin:
            self.tracks[0].end.x += 1
        return object()

    def remove_items(self, items):
        self.remove_calls += 1
        for item in items:
            if item in self.tracks:
                self.tracks.remove(item)
            elif item in self.vias:
                self.vias.remove(item)

    def update_items(self, items):
        self.update_calls += 1
        return list(items)

    def create_items(self, items):
        self.create_calls += 1
        self.tracks.extend(item for item in items if isinstance(item, (FakeTrack, FakeArc)))
        self.vias.extend(item for item in items if isinstance(item, FakeVia))
        if self.fail_create:
            return []
        return list(items)

    def push_commit(self, commit, message):
        del commit
        self.push_calls += 1
        self.commit_message = message

    def drop_commit(self, commit):
        del commit
        self.drop_calls += 1


BINDINGS = BRIDGE.LiveBindings(
    track_type=FakeTrack,
    arc_type=FakeArc,
    via_type=FakeVia,
    vector=Point,
    layer_name=lambda layer: layer,
    layer_from_name=lambda name: name if name in {"F.Cu", "B.Cu"} else "unknown",
    unknown_layer="unknown",
    via_through="through",
    via_blind_buried="blind-buried",
)


def track_state(item_id, start_x, end_x):
    return {
        "id": item_id,
        "net": "SIGNAL",
        "layer": "F.Cu",
        "start_nm": [start_x, 0],
        "end_nm": [end_x, 0],
        "width_nm": 200_000,
        "locked": False,
    }


def arc_state():
    return {
        "id": ARC_ID,
        "net": "SIGNAL",
        "layer": "F.Cu",
        "start_nm": [1_000_000, 0],
        "middle_nm": [1_500_000, 500_000],
        "end_nm": [2_000_000, 0],
        "width_nm": 200_000,
        "locked": False,
    }


def via_state():
    return {
        "id": VIA_ID,
        "net": "SIGNAL",
        "position_nm": [2_000_000, 0],
        "diameter_nm": 600_000,
        "drill_nm": 300_000,
        "layers": ["F.Cu", "B.Cu"],
        "locked": False,
    }


def change_set():
    first = track_state(TRACK_1_ID, 0, 1_000_000)
    second = track_state(TRACK_2_ID, 0, 2_000_000)
    updated = track_state(TRACK_1_ID, 0, 1_500_000)
    return {
        "ok": True,
        "version": 1,
        "board_schema": 18,
        "source_revision": "sha256:test-revision",
        "source_inventory": {
            "footprints": [
                {
                    "id": FP_ID,
                    "reference": "U1",
                    "state": {
                        "position_nm": [1_000_000, 2_000_000],
                        "rotation_microdegrees": 90_000_000,
                        "side": "front",
                    },
                }
            ],
            "tracks": [first, second],
            "track_arcs": [],
            "vias": [],
        },
        "footprint_transforms": [],
        "tracks": [
            {"operation": "update", "source": copy.deepcopy(first), "candidate": updated},
            {"operation": "remove", "source": copy.deepcopy(second), "candidate": None},
        ],
        "track_arcs": [{"operation": "create", "source": None, "candidate": arc_state()}],
        "vias": [{"operation": "create", "source": None, "candidate": via_state()}],
        "diagnostics": [],
    }


class CandidateBridgeTests(unittest.TestCase):
    def test_schema_is_strict_and_internally_source_bound(self):
        document = change_set()
        self.assertIs(BRIDGE.validate_change_set(document), document)
        self.assertEqual(
            BRIDGE.change_set_counts(document),
            {
                "footprint_transforms": 0,
                "track_operations": 2,
                "track_arc_operations": 1,
                "via_operations": 1,
            },
        )

        unknown = copy.deepcopy(document)
        unknown["unexpected"] = True
        with self.assertRaisesRegex(BRIDGE.ChangeSetError, "fields differ"):
            BRIDGE.validate_change_set(unknown)

        stale = copy.deepcopy(document)
        stale["tracks"][0]["source"]["end_nm"] = [999, 0]
        with self.assertRaisesRegex(BRIDGE.ChangeSetError, "source precondition mismatch"):
            BRIDGE.validate_change_set(stale)

        reordered = copy.deepcopy(document)
        reordered["source_inventory"]["tracks"].reverse()
        with self.assertRaisesRegex(BRIDGE.ChangeSetError, "stable UUID"):
            BRIDGE.validate_change_set(reordered)

        stale_schema = copy.deepcopy(document)
        stale_schema["board_schema"] = 17
        with self.assertRaisesRegex(BRIDGE.ChangeSetError, "unsupported board schema"):
            BRIDGE.validate_change_set(stale_schema)

    def test_live_snapshot_requires_exact_membership_and_geometry(self):
        document = change_set()
        board = FakeBoard()
        live = BRIDGE.verify_live_source(board, document, BINDINGS)
        self.assertEqual(set(live["track"]), {TRACK_1_ID, TRACK_2_ID})

        board.tracks[0].end.x += 1
        with self.assertRaisesRegex(BRIDGE.ChangeSetError, "stale live track precondition"):
            BRIDGE.verify_live_source(board, document, BINDINGS)

        board = FakeBoard()
        board.tracks.append(make_track("10000000-0000-0000-0000-000000000003", 0, 3_000_000))
        with self.assertRaisesRegex(BRIDGE.ChangeSetError, "membership differs"):
            BRIDGE.verify_live_source(board, document, BINDINGS)

        board = FakeBoard()
        board.footprints[0].reference_field.text.value = "U2"
        with self.assertRaisesRegex(BRIDGE.ChangeSetError, "stale live footprint reference"):
            BRIDGE.verify_live_source(board, document, BINDINGS)

    def test_routing_apply_is_one_verified_unsaved_transaction(self):
        document = change_set()
        board = FakeBoard()
        counts = BRIDGE.apply_routing_transaction(board, document, BINDINGS)
        self.assertEqual(counts["track_operations"], 2)
        self.assertEqual((board.begin_calls, board.push_calls, board.drop_calls), (1, 1, 0))
        self.assertEqual((board.remove_calls, board.update_calls, board.create_calls), (1, 1, 2))
        self.assertFalse(hasattr(board, "save"))
        self.assertEqual(board.commit_message, "Apply Flux placement/routing candidate (routing only)")

    def test_bad_kicad_response_drops_commit(self):
        board = FakeBoard(fail_create=True)
        with self.assertRaisesRegex(BRIDGE.ChangeSetError, "incomplete track_arc create"):
            BRIDGE.apply_routing_transaction(board, change_set(), BINDINGS)
        self.assertEqual((board.begin_calls, board.push_calls, board.drop_calls), (1, 0, 1))

    def test_in_commit_stale_reread_drops_before_mutation(self):
        board = FakeBoard(mutate_on_begin=True)
        with self.assertRaisesRegex(BRIDGE.ChangeSetError, "stale live track precondition"):
            BRIDGE.apply_routing_transaction(board, change_set(), BINDINGS)
        self.assertEqual((board.begin_calls, board.push_calls, board.drop_calls), (1, 0, 1))
        self.assertEqual((board.remove_calls, board.update_calls, board.create_calls), (0, 0, 0))

    def test_footprint_transform_is_withheld_before_commit(self):
        document = change_set()
        source = copy.deepcopy(document["source_inventory"]["footprints"][0]["state"])
        candidate = copy.deepcopy(source)
        candidate["position_nm"] = [1_500_000, 2_000_000]
        document["footprint_transforms"] = [
            {
                "id": FP_ID,
                "reference": "U1",
                "source": source,
                "candidate": candidate,
            }
        ]
        board = FakeBoard()
        with self.assertRaisesRegex(BRIDGE.ChangeSetError, "footprint apply is withheld"):
            BRIDGE.apply_routing_transaction(board, document, BINDINGS)
        self.assertEqual((board.begin_calls, board.push_calls, board.drop_calls), (0, 0, 0))


if __name__ == "__main__":
    unittest.main()
