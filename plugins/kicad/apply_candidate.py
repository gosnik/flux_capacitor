#!/usr/bin/env python3
"""Validate, preview, or transactionally apply a verified Flux routing candidate.

The C++ engine remains authoritative for source/candidate provenance and geometry. This module
imports kicad-python only after offline validation in main(), and its transaction functions accept
injected bindings so the fail-closed behavior can be tested without a KiCad installation.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import re
import subprocess
import sys
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from typing import Any


UUID_PATTERN = re.compile(
    r"^[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-"
    r"[0-9a-fA-F]{4}-[0-9a-fA-F]{12}$"
)
SUPPORTED_CHANGE_SET_VERSION = 1
SUPPORTED_BOARD_SCHEMA = 22
TOP_LEVEL_KEYS = {
    "ok",
    "version",
    "board_schema",
    "source_revision",
    "source_inventory",
    "footprint_transforms",
    "tracks",
    "track_arcs",
    "track_beziers",
    "vias",
    "diagnostics",
}
INVENTORY_KEYS = {"footprints", "tracks", "track_arcs", "track_beziers", "vias"}
FOOTPRINT_KEYS = {"id", "reference", "state"}
FOOTPRINT_CHANGE_KEYS = {"id", "reference", "source", "candidate"}
FOOTPRINT_STATE_KEYS = {"position_nm", "rotation_microdegrees", "side"}
CHANGE_KEYS = {"operation", "source", "candidate"}
TRACK_KEYS = {"id", "net", "layer", "start_nm", "end_nm", "width_nm", "locked"}
ARC_KEYS = {
    "id",
    "net",
    "layer",
    "start_nm",
    "middle_nm",
    "end_nm",
    "width_nm",
    "locked",
}
BEZIER_KEYS = {
    "id",
    "net",
    "layer",
    "start_nm",
    "control1_nm",
    "control2_nm",
    "end_nm",
    "width_nm",
    "locked",
}
VIA_KEYS = {"id", "net", "position_nm", "diameter_nm", "drill_nm", "layers", "locked"}


class ChangeSetError(ValueError):
    """A candidate or live board failed a fail-closed change-set gate."""


@dataclass(frozen=True)
class LiveBindings:
    track_type: type
    arc_type: type
    via_type: type
    vector: Callable[[int, int], Any]
    layer_name: Callable[[Any], str]
    layer_from_name: Callable[[str], Any]
    unknown_layer: Any
    via_through: Any
    via_blind_buried: Any
    bezier_type: type | None = None


def _mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ChangeSetError(f"{path} must be an object")
    return value


def _sequence(value: Any, path: str) -> Sequence[Any]:
    if not isinstance(value, list):
        raise ChangeSetError(f"{path} must be an array")
    return value


def _exact_keys(value: Mapping[str, Any], expected: set[str], path: str) -> None:
    actual = set(value)
    if actual != expected:
        missing = sorted(expected - actual)
        extra = sorted(actual - expected)
        raise ChangeSetError(f"{path} fields differ: missing={missing!r}, extra={extra!r}")


def _integer(value: Any, path: str, *, minimum: int | None = None) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ChangeSetError(f"{path} must be an integer")
    if minimum is not None and value < minimum:
        raise ChangeSetError(f"{path} must be at least {minimum}")
    return value


def _string(value: Any, path: str, *, nonempty: bool = True) -> str:
    if not isinstance(value, str) or (nonempty and not value):
        raise ChangeSetError(f"{path} must be a non-empty string")
    return value


def _uuid(value: Any, path: str) -> str:
    text = _string(value, path)
    if UUID_PATTERN.fullmatch(text) is None:
        raise ChangeSetError(f"{path} must be a KiCad UUID")
    return text.lower()


def _point(value: Any, path: str) -> tuple[int, int]:
    point = _sequence(value, path)
    if len(point) != 2:
        raise ChangeSetError(f"{path} must contain exactly two coordinates")
    return _integer(point[0], f"{path}[0]"), _integer(point[1], f"{path}[1]")


def _footprint_state(value: Any, path: str) -> Mapping[str, Any]:
    state = _mapping(value, path)
    _exact_keys(state, FOOTPRINT_STATE_KEYS, path)
    _point(state["position_nm"], f"{path}.position_nm")
    rotation = _integer(state["rotation_microdegrees"], f"{path}.rotation_microdegrees")
    if not 0 <= rotation < 360_000_000:
        raise ChangeSetError(f"{path}.rotation_microdegrees must be normalized")
    if state["side"] not in {"front", "back"}:
        raise ChangeSetError(f"{path}.side must be front or back")
    return state


def _route_state(value: Any, kind: str, path: str) -> Mapping[str, Any]:
    state = _mapping(value, path)
    expected = {"track": TRACK_KEYS, "track_arc": ARC_KEYS, "track_bezier": BEZIER_KEYS, "via": VIA_KEYS}[kind]
    _exact_keys(state, expected, path)
    _uuid(state["id"], f"{path}.id")
    _string(state["net"], f"{path}.net")
    if not isinstance(state["locked"], bool):
        raise ChangeSetError(f"{path}.locked must be boolean")
    if kind == "track":
        _string(state["layer"], f"{path}.layer")
        _point(state["start_nm"], f"{path}.start_nm")
        _point(state["end_nm"], f"{path}.end_nm")
        _integer(state["width_nm"], f"{path}.width_nm", minimum=1)
    elif kind == "track_arc":
        _string(state["layer"], f"{path}.layer")
        _point(state["start_nm"], f"{path}.start_nm")
        _point(state["middle_nm"], f"{path}.middle_nm")
        _point(state["end_nm"], f"{path}.end_nm")
        _integer(state["width_nm"], f"{path}.width_nm", minimum=1)
    elif kind == "track_bezier":
        _string(state["layer"], f"{path}.layer")
        _point(state["start_nm"], f"{path}.start_nm")
        _point(state["control1_nm"], f"{path}.control1_nm")
        _point(state["control2_nm"], f"{path}.control2_nm")
        _point(state["end_nm"], f"{path}.end_nm")
        _integer(state["width_nm"], f"{path}.width_nm", minimum=1)
    else:
        _point(state["position_nm"], f"{path}.position_nm")
        diameter = _integer(state["diameter_nm"], f"{path}.diameter_nm", minimum=1)
        drill = _integer(state["drill_nm"], f"{path}.drill_nm", minimum=1)
        if drill >= diameter:
            raise ChangeSetError(f"{path}.drill_nm must be smaller than diameter_nm")
        layers = _sequence(state["layers"], f"{path}.layers")
        if len(layers) < 2 or len(set(layers)) != len(layers):
            raise ChangeSetError(f"{path}.layers must contain a unique via span")
        for index, layer in enumerate(layers):
            _string(layer, f"{path}.layers[{index}]")
    return state


def _indexed_states(values: Any, kind: str, path: str) -> dict[str, Mapping[str, Any]]:
    result: dict[str, Mapping[str, Any]] = {}
    previous = ""
    for index, value in enumerate(_sequence(values, path)):
        state = _route_state(value, kind, f"{path}[{index}]")
        item_id = str(state["id"]).lower()
        if item_id in result:
            raise ChangeSetError(f"{path} repeats routing id {item_id!r}")
        if previous and item_id <= previous:
            raise ChangeSetError(f"{path} must be ordered by stable UUID")
        previous = item_id
        result[item_id] = state
    return result


def validate_change_set(document: Any) -> Mapping[str, Any]:
    root = _mapping(document, "change_set")
    _exact_keys(root, TOP_LEVEL_KEYS, "change_set")
    if root["ok"] is not True:
        raise ChangeSetError("C++ change-set generation did not succeed")
    if _integer(root["version"], "change_set.version") != SUPPORTED_CHANGE_SET_VERSION:
        raise ChangeSetError("unsupported change-set version")
    if _integer(root["board_schema"], "change_set.board_schema") != SUPPORTED_BOARD_SCHEMA:
        raise ChangeSetError("unsupported board schema")
    _string(root["source_revision"], "change_set.source_revision")
    if _sequence(root["diagnostics"], "change_set.diagnostics"):
        raise ChangeSetError("successful change set must not contain diagnostics")

    inventory = _mapping(root["source_inventory"], "change_set.source_inventory")
    _exact_keys(inventory, INVENTORY_KEYS, "change_set.source_inventory")
    footprint_inventory: dict[str, Mapping[str, Any]] = {}
    previous = ""
    for index, value in enumerate(_sequence(inventory["footprints"], "source_inventory.footprints")):
        item = _mapping(value, f"source_inventory.footprints[{index}]")
        _exact_keys(item, FOOTPRINT_KEYS, f"source_inventory.footprints[{index}]")
        item_id = _uuid(item["id"], f"source_inventory.footprints[{index}].id")
        _string(item["reference"], f"source_inventory.footprints[{index}].reference")
        _footprint_state(item["state"], f"source_inventory.footprints[{index}].state")
        if item_id in footprint_inventory or (previous and item_id <= previous):
            raise ChangeSetError("source footprint inventory must have unique ordered UUIDs")
        previous = item_id
        footprint_inventory[item_id] = item

    inventory_by_kind = {
        "track": _indexed_states(inventory["tracks"], "track", "source_inventory.tracks"),
        "track_arc": _indexed_states(
            inventory["track_arcs"], "track_arc", "source_inventory.track_arcs"
        ),
        "track_bezier": _indexed_states(
            inventory["track_beziers"], "track_bezier", "source_inventory.track_beziers"
        ),
        "via": _indexed_states(inventory["vias"], "via", "source_inventory.vias"),
    }
    all_source_route_ids: set[str] = set()
    for states in inventory_by_kind.values():
        overlap = all_source_route_ids.intersection(states)
        if overlap:
            raise ChangeSetError(f"source inventory repeats IDs across kinds: {sorted(overlap)!r}")
        all_source_route_ids.update(states)

    seen_transforms: set[str] = set()
    previous = ""
    for index, value in enumerate(_sequence(root["footprint_transforms"], "footprint_transforms")):
        change = _mapping(value, f"footprint_transforms[{index}]")
        _exact_keys(change, FOOTPRINT_CHANGE_KEYS, f"footprint_transforms[{index}]")
        item_id = _uuid(change["id"], f"footprint_transforms[{index}].id")
        reference = _string(change["reference"], f"footprint_transforms[{index}].reference")
        source = _footprint_state(change["source"], f"footprint_transforms[{index}].source")
        candidate = _footprint_state(
            change["candidate"], f"footprint_transforms[{index}].candidate"
        )
        if item_id not in footprint_inventory:
            raise ChangeSetError(f"footprint transform references unknown source id {item_id!r}")
        snapshot = footprint_inventory[item_id]
        if snapshot["reference"] != reference or snapshot["state"] != source:
            raise ChangeSetError(f"footprint transform source mismatch for {item_id!r}")
        if source == candidate:
            raise ChangeSetError(f"footprint transform for {item_id!r} is a no-op")
        if source["side"] != candidate["side"]:
            raise ChangeSetError("live change-set v1 does not support footprint side changes")
        if item_id in seen_transforms or (previous and item_id <= previous):
            raise ChangeSetError("footprint transforms must have unique ordered UUIDs")
        previous = item_id
        seen_transforms.add(item_id)

    final_route_ids = set(all_source_route_ids)
    for kind, field in (("track", "tracks"), ("track_arc", "track_arcs"),
                        ("track_bezier", "track_beziers"), ("via", "vias")):
        seen_changes: set[str] = set()
        source_phase = True
        previous_source = ""
        previous_create = ""
        for index, value in enumerate(_sequence(root[field], field)):
            change = _mapping(value, f"{field}[{index}]")
            _exact_keys(change, CHANGE_KEYS, f"{field}[{index}]")
            operation = change["operation"]
            if operation not in {"create", "update", "remove"}:
                raise ChangeSetError(f"{field}[{index}].operation is unsupported")
            source = change["source"]
            candidate = change["candidate"]
            if operation == "create":
                if source is not None or candidate is None:
                    raise ChangeSetError(f"{field}[{index}] has invalid create endpoints")
                source_phase = False
            elif operation == "update":
                if source is None or candidate is None:
                    raise ChangeSetError(f"{field}[{index}] has invalid update endpoints")
                if not source_phase:
                    raise ChangeSetError(f"{field} source operations must precede creates")
            else:
                if source is None or candidate is not None:
                    raise ChangeSetError(f"{field}[{index}] has invalid remove endpoints")
                if not source_phase:
                    raise ChangeSetError(f"{field} source operations must precede creates")
            before = None if source is None else _route_state(source, kind, f"{field}[{index}].source")
            after = (
                None
                if candidate is None
                else _route_state(candidate, kind, f"{field}[{index}].candidate")
            )
            effective = after if operation == "create" else before
            assert effective is not None
            item_id = str(effective["id"]).lower()
            previous = previous_source if source_phase else previous_create
            if item_id in seen_changes or (previous and item_id <= previous):
                raise ChangeSetError(f"{field} operations must have unique stable UUID ordering")
            if operation == "create":
                if item_id in final_route_ids:
                    raise ChangeSetError(f"create reuses live routing id {item_id!r}")
                final_route_ids.add(item_id)
            else:
                expected = inventory_by_kind[kind].get(item_id)
                if expected is None or expected != before:
                    raise ChangeSetError(f"{operation} source precondition mismatch for {item_id!r}")
                if operation == "remove":
                    final_route_ids.remove(item_id)
                else:
                    assert after is not None
                    if str(after["id"]).lower() != item_id:
                        raise ChangeSetError("route update cannot change a KiCad UUID")
                    if before == after:
                        raise ChangeSetError(f"route update for {item_id!r} is a no-op")
            seen_changes.add(item_id)
            if source_phase:
                previous_source = item_id
            else:
                previous_create = item_id
    return root


def change_set_counts(change_set: Mapping[str, Any]) -> dict[str, int]:
    return {
        "footprint_transforms": len(change_set["footprint_transforms"]),
        "track_operations": len(change_set["tracks"]),
        "track_arc_operations": len(change_set["track_arcs"]),
        "track_bezier_operations": len(change_set["track_beziers"]),
        "via_operations": len(change_set["vias"]),
    }


def _normalized_rotation_microdegrees(angle: Any) -> int:
    return int(round(float(angle.degrees) * 1_000_000.0)) % 360_000_000


def _live_footprint_state(item: Any, bindings: LiveBindings) -> dict[str, Any]:
    layer = bindings.layer_name(item.layer)
    if layer not in {"F.Cu", "B.Cu"}:
        raise ChangeSetError(f"live footprint {item.id.value!r} is not on an outer copper layer")
    return {
        "position_nm": [int(item.position.x), int(item.position.y)],
        "rotation_microdegrees": _normalized_rotation_microdegrees(item.orientation),
        "side": "front" if layer == "F.Cu" else "back",
    }


def _live_route_state(item: Any, kind: str, bindings: LiveBindings) -> dict[str, Any]:
    common = {"id": str(item.id.value).lower(), "net": str(item.net.name), "locked": bool(item.locked)}
    if kind == "track":
        return {
            **common,
            "layer": bindings.layer_name(item.layer),
            "start_nm": [int(item.start.x), int(item.start.y)],
            "end_nm": [int(item.end.x), int(item.end.y)],
            "width_nm": int(item.width),
        }
    if kind == "track_arc":
        return {
            **common,
            "layer": bindings.layer_name(item.layer),
            "start_nm": [int(item.start.x), int(item.start.y)],
            "middle_nm": [int(item.mid.x), int(item.mid.y)],
            "end_nm": [int(item.end.x), int(item.end.y)],
            "width_nm": int(item.width),
        }
    if kind == "track_bezier":
        return {
            **common,
            "layer": bindings.layer_name(item.layer),
            "start_nm": [int(item.start.x), int(item.start.y)],
            "control1_nm": [int(item.control1.x), int(item.control1.y)],
            "control2_nm": [int(item.control2.x), int(item.control2.y)],
            "end_nm": [int(item.end.x), int(item.end.y)],
            "width_nm": int(item.attributes.stroke.width),
        }
    start_layer = bindings.layer_name(item.padstack.drill.start_layer)
    end_layer = bindings.layer_name(item.padstack.drill.end_layer)
    return {
        **common,
        "position_nm": [int(item.position.x), int(item.position.y)],
        "diameter_nm": int(item.diameter),
        "drill_nm": int(item.drill_diameter),
        "layers": [start_layer, end_layer],
    }


def _route_states_equal(actual: Mapping[str, Any], expected: Mapping[str, Any], kind: str) -> bool:
    if kind != "via":
        return actual == expected
    expected_span = list(expected["layers"])
    normalized = {**expected, "layers": [expected_span[0], expected_span[-1]]}
    return actual == normalized


def verify_live_source(
    board: Any, change_set: Mapping[str, Any], bindings: LiveBindings
) -> dict[str, dict[str, Any]]:
    inventory = change_set["source_inventory"]
    live: dict[str, dict[str, Any]] = {
        "footprint": {}, "track": {}, "track_arc": {}, "track_bezier": {}, "via": {}
    }
    for item in board.get_footprints():
        item_id = str(item.id.value).lower()
        if item_id in live["footprint"]:
            raise ChangeSetError(f"live board repeats footprint id {item_id!r}")
        live["footprint"][item_id] = item

    for item in board.get_tracks():
        if isinstance(item, bindings.arc_type):
            kind = "track_arc"
        elif isinstance(item, bindings.track_type):
            kind = "track"
        else:
            raise ChangeSetError(f"live board returned unsupported routed item {type(item)!r}")
        item_id = str(item.id.value).lower()
        if item_id in live[kind]:
            raise ChangeSetError(f"live board repeats {kind} id {item_id!r}")
        live[kind][item_id] = item

    if bindings.bezier_type is not None:
        for item in board.get_shapes():
            if not isinstance(item, bindings.bezier_type) or not str(item.net.name):
                continue
            item_id = str(item.id.value).lower()
            if item_id in live["track_bezier"]:
                raise ChangeSetError(f"live board repeats track_bezier id {item_id!r}")
            live["track_bezier"][item_id] = item

    for item in board.get_vias():
        item_id = str(item.id.value).lower()
        if item_id in live["via"]:
            raise ChangeSetError(f"live board repeats via id {item_id!r}")
        live["via"][item_id] = item

    footprint_expected = {str(item["id"]).lower(): item for item in inventory["footprints"]}
    if set(live["footprint"]) != set(footprint_expected):
        raise ChangeSetError("live footprint membership differs from the saved source inventory")
    for item_id, expected in footprint_expected.items():
        item = live["footprint"][item_id]
        if str(item.reference_field.text.value) != expected["reference"]:
            raise ChangeSetError(f"stale live footprint reference for {item_id!r}")
        if _live_footprint_state(item, bindings) != expected["state"]:
            raise ChangeSetError(f"stale live footprint precondition for {item_id!r}")

    for kind, field in (("track", "tracks"), ("track_arc", "track_arcs"),
                        ("track_bezier", "track_beziers"), ("via", "vias")):
        expected = {str(item["id"]).lower(): item for item in inventory[field]}
        if set(live[kind]) != set(expected):
            raise ChangeSetError(f"live {kind} membership differs from the saved source inventory")
        for item_id, state in expected.items():
            actual = _live_route_state(live[kind][item_id], kind, bindings)
            if not _route_states_equal(actual, state, kind):
                raise ChangeSetError(f"stale live {kind} precondition for {item_id!r}")
    return live


def _layer(value: str, bindings: LiveBindings) -> Any:
    layer = bindings.layer_from_name(value)
    if layer == bindings.unknown_layer:
        raise ChangeSetError(f"KiCad does not expose candidate layer {value!r}")
    return layer


def _apply_route_state(
    item: Any,
    state: Mapping[str, Any],
    kind: str,
    bindings: LiveBindings,
    nets: Mapping[str, Any],
    *,
    assign_id: bool,
) -> Any:
    if assign_id:
        item.proto.id.value = str(state["id"])
    try:
        item.net = nets[str(state["net"])]
    except KeyError as error:
        raise ChangeSetError(f"candidate references missing live net {state['net']!r}") from error
    item.locked = bool(state["locked"])
    if kind == "track":
        item.layer = _layer(str(state["layer"]), bindings)
        item.start = bindings.vector(*_point(state["start_nm"], "track.start_nm"))
        item.end = bindings.vector(*_point(state["end_nm"], "track.end_nm"))
        item.width = int(state["width_nm"])
    elif kind == "track_arc":
        item.layer = _layer(str(state["layer"]), bindings)
        item.start = bindings.vector(*_point(state["start_nm"], "track_arc.start_nm"))
        item.mid = bindings.vector(*_point(state["middle_nm"], "track_arc.middle_nm"))
        item.end = bindings.vector(*_point(state["end_nm"], "track_arc.end_nm"))
        item.width = int(state["width_nm"])
    elif kind == "track_bezier":
        item.layer = _layer(str(state["layer"]), bindings)
        item.start = bindings.vector(*_point(state["start_nm"], "track_bezier.start_nm"))
        item.control1 = bindings.vector(*_point(state["control1_nm"], "track_bezier.control1_nm"))
        item.control2 = bindings.vector(*_point(state["control2_nm"], "track_bezier.control2_nm"))
        item.end = bindings.vector(*_point(state["end_nm"], "track_bezier.end_nm"))
        item.attributes.stroke.width = int(state["width_nm"])
    else:
        item.position = bindings.vector(*_point(state["position_nm"], "via.position_nm"))
        layers = [_layer(str(name), bindings) for name in state["layers"]]
        item.type = (
            bindings.via_through
            if state["layers"][0] == "F.Cu" and state["layers"][-1] == "B.Cu"
            else bindings.via_blind_buried
        )
        item.diameter = int(state["diameter_nm"])
        item.drill_diameter = int(state["drill_nm"])
        item.padstack.layers = layers
        item.padstack.drill.start_layer = layers[0]
        item.padstack.drill.end_layer = layers[-1]
    return item


def apply_routing_transaction(
    board: Any, change_set: Mapping[str, Any], bindings: LiveBindings
) -> dict[str, int]:
    validate_change_set(change_set)
    if change_set["footprint_transforms"]:
        raise ChangeSetError(
            "live footprint apply is withheld because whole-footprint IPC updates can damage "
            "schematic linkage; use the verified candidate artifact"
        )
    verify_live_source(board, change_set, bindings)

    changes = [
        (kind, change)
        for kind, field in (("track", "tracks"), ("track_arc", "track_arcs"),
                            ("track_bezier", "track_beziers"), ("via", "vias"))
        for change in change_set[field]
    ]
    counts = change_set_counts(change_set)
    if not changes:
        return counts

    commit = board.begin_commit()
    commit_open = True
    try:
        live = verify_live_source(board, change_set, bindings)
        nets: dict[str, Any] = {}
        for net in board.get_nets():
            name = str(net.name)
            if name:
                if name in nets:
                    raise ChangeSetError(f"live board repeats net name {name!r}")
                nets[name] = net

        removed = [
            live[kind][str(change["source"]["id"]).lower()]
            for kind, change in changes
            if change["operation"] == "remove"
        ]
        if removed:
            board.remove_items(removed)

        for kind in ("track", "track_arc", "track_bezier", "via"):
            updated_expected = [
                change["candidate"]
                for change_kind, change in changes
                if change_kind == kind and change["operation"] == "update"
            ]
            if updated_expected:
                updated_items = [
                    _apply_route_state(
                        live[kind][str(state["id"]).lower()],
                        state,
                        kind,
                        bindings,
                        nets,
                        assign_id=False,
                    )
                    for state in updated_expected
                ]
                returned = board.update_items(updated_items)
                returned_by_id = {str(item.id.value).lower(): item for item in returned}
                if set(returned_by_id) != {str(state["id"]).lower() for state in updated_expected}:
                    raise ChangeSetError(f"KiCad returned an incomplete {kind} update response")
                for state in updated_expected:
                    actual = _live_route_state(returned_by_id[str(state["id"]).lower()], kind, bindings)
                    if not _route_states_equal(actual, state, kind):
                        raise ChangeSetError(f"KiCad altered requested {kind} update geometry")

            created_expected = [
                change["candidate"]
                for change_kind, change in changes
                if change_kind == kind and change["operation"] == "create"
            ]
            if created_expected:
                item_type = {
                    "track": bindings.track_type,
                    "track_arc": bindings.arc_type,
                    "track_bezier": bindings.bezier_type,
                    "via": bindings.via_type,
                }[kind]
                if item_type is None:
                    raise ChangeSetError("KiCad bridge does not expose BoardBezier support")
                requested = [
                    _apply_route_state(
                        item_type(), state, kind, bindings, nets, assign_id=True
                    )
                    for state in created_expected
                ]
                returned = board.create_items(requested)
                returned_by_id = {str(item.id.value).lower(): item for item in returned}
                if set(returned_by_id) != {str(state["id"]).lower() for state in created_expected}:
                    raise ChangeSetError(f"KiCad returned an incomplete {kind} create response")
                for state in created_expected:
                    actual = _live_route_state(returned_by_id[str(state["id"]).lower()], kind, bindings)
                    if not _route_states_equal(actual, state, kind):
                        raise ChangeSetError(f"KiCad altered requested {kind} create geometry")

        board.push_commit(commit, "Apply Flux placement/routing candidate (routing only)")
        commit_open = False
    except Exception:
        if commit_open:
            board.drop_commit(commit)
        raise
    return counts


def _load_change_set(flux_cli: str, source: pathlib.Path, candidate: pathlib.Path) -> Mapping[str, Any]:
    completed = subprocess.run(
        [flux_cli, "changeset-kicad", str(source), str(candidate)],
        check=False,
        capture_output=True,
        text=True,
    )
    try:
        document = json.loads(completed.stdout)
    except json.JSONDecodeError as error:
        raise ChangeSetError("C++ engine did not return a JSON change set") from error
    if completed.returncode != 0:
        diagnostics = document.get("diagnostics", []) if isinstance(document, Mapping) else []
        raise ChangeSetError(f"C++ change-set generation failed: {diagnostics!r}")
    return validate_change_set(document)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Validate, preview, or apply a verified Flux routing candidate to the active KiCad "
            "board"
        )
    )
    parser.add_argument("--board", required=True, type=pathlib.Path)
    parser.add_argument("--candidate", required=True, type=pathlib.Path)
    parser.add_argument("--flux-cli", default="flux-cli")
    parser.add_argument(
        "--validate-only",
        action="store_true",
        help="validate the current C++ change-set contract without connecting to KiCad",
    )
    parser.add_argument(
        "--apply-routing",
        action="store_true",
        help="apply routing as one unsaved undoable transaction; placement changes remain withheld",
    )
    args = parser.parse_args(argv)
    try:
        if args.validate_only and args.apply_routing:
            raise ChangeSetError("--validate-only and --apply-routing are mutually exclusive")
        source = args.board.resolve(strict=True)
        candidate = args.candidate.resolve(strict=True)
        source_bytes_before = source.read_bytes()
        change_set = _load_change_set(args.flux_cli, source, candidate)
        counts = change_set_counts(change_set)
        if args.validate_only:
            if source_bytes_before != source.read_bytes():
                raise ChangeSetError("saved source board changed while the candidate was verified")
            output = {
                "ok": True,
                "mode": "validate-only",
                "source_revision": change_set["source_revision"],
                **counts,
                "footprint_apply_withheld": bool(change_set["footprint_transforms"]),
                "applied": False,
                "saved_source_modified": False,
            }
            print(json.dumps(output, indent=2, sort_keys=True))
            return 0

        from kipy import KiCad
        from kipy.board_types import ArcTrack, BoardBezier, Track, Via, ViaType
        from kipy.geometry import Vector2
        from kipy.proto.board.board_types_pb2 import BoardLayer
        from kipy.util.board_layer import canonical_name, layer_from_canonical_name

        bindings = LiveBindings(
            track_type=Track,
            arc_type=ArcTrack,
            via_type=Via,
            vector=Vector2.from_xy,
            layer_name=canonical_name,
            layer_from_name=layer_from_canonical_name,
            unknown_layer=BoardLayer.BL_UNKNOWN,
            via_through=ViaType.VT_THROUGH,
            via_blind_buried=ViaType.VT_BLIND_BURIED,
            bezier_type=BoardBezier,
        )
        kicad = KiCad(client_name="flux_capacitor candidate preview/apply")
        kicad.check_version()
        board = kicad.get_board()
        live_path = (
            pathlib.Path(board.document.project.path) / board.document.board_filename
        ).resolve(strict=True)
        if live_path != source:
            raise ChangeSetError(
                f"active KiCad board {str(live_path)!r} does not match {str(source)!r}"
        )
        verify_live_source(board, change_set, bindings)
        source_bytes_ready = source.read_bytes()
        if source_bytes_before != source_bytes_ready:
            raise ChangeSetError("saved source board changed while the candidate was verified")

        applied = False
        if args.apply_routing:
            counts = apply_routing_transaction(board, change_set, bindings)
            applied = any(
                counts[key]
                for key in ("track_operations", "track_arc_operations", "track_bezier_operations",
                            "via_operations")
            )
        if source_bytes_ready != source.read_bytes():
            raise ChangeSetError("live candidate operation unexpectedly modified the saved source")
    except Exception as error:  # IPC and transaction failures stay at the process boundary.
        print(f"flux KiCad candidate bridge failed: {error}", file=sys.stderr)
        return 2

    output = {
        "ok": True,
        "mode": "apply-routing" if args.apply_routing else "preview",
        "source_revision": change_set["source_revision"],
        **counts,
        "footprint_apply_withheld": bool(change_set["footprint_transforms"]),
        "applied": applied,
        "saved_source_modified": False,
    }
    print(json.dumps(output, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
