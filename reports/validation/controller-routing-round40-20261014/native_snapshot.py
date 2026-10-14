#!/usr/bin/env python3
"""Write a source-bound Flux net-class snapshot from native KiCad SWIG data.

This is deliberately an offline, read-only adapter.  It opens the requested
saved board with ``pcbnew.LoadBoard`` and its sibling project, but does not use
KiCad IPC, start a GUI, invoke Flux, or alter either input.  The caller must
supply the already-recorded ``flux-cli inspect-kicad`` JSON so the snapshot is
bound to the same strict Flux import.

KiCad resolves netclass-pattern precedence internally.  This script does not
reimplement or pattern-match that policy: each named net is queried through
``NET_SETTINGS.GetEffectiveNetClass``.  A result is accepted only when it is
exactly one declared class with identical seven physical routing values.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import pathlib
import sys
from collections.abc import Mapping, Sequence
from types import SimpleNamespace
from typing import Any


ROOT = pathlib.Path(__file__).resolve().parents[3]
EXPORTER_PATH = ROOT / "plugins" / "kicad" / "export_netclasses.py"
PHYSICAL_METHODS = (
    ("clearance", "GetClearance"),
    ("track_width", "GetTrackWidth"),
    ("via_diameter", "GetViaDiameter"),
    ("via_drill", "GetViaDrill"),
    ("diff_pair_track_width", "GetDiffPairWidth"),
    ("diff_pair_gap", "GetDiffPairGap"),
    ("diff_pair_via_gap", "GetDiffPairViaGap"),
)


def _load_exporter() -> Any:
    specification = importlib.util.spec_from_file_location("flux_export_netclasses", EXPORTER_PATH)
    if specification is None or specification.loader is None:
        raise RuntimeError(f"cannot load {EXPORTER_PATH}")
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def _sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return "sha256:" + digest.hexdigest()


def _stable_file(path: pathlib.Path, before: str, label: str) -> None:
    after = _sha256(path)
    if after != before:
        raise ValueError(f"{label} changed while the native snapshot was captured")


def _native_class(native: Any, *, expected_name: str | None = None) -> SimpleNamespace:
    if native is None:
        raise ValueError("KiCad returned no net class")
    name = str(native.GetName())
    if not name:
        raise ValueError("KiCad returned an unnamed net class")
    if expected_name is not None and name != expected_name:
        raise ValueError(f"KiCad class map key {expected_name!r} disagrees with class name {name!r}")
    values: dict[str, int] = {}
    for attribute, method_name in PHYSICAL_METHODS:
        method = getattr(native, method_name, None)
        if method is None:
            raise ValueError(f"native KiCad class {name!r} lacks {method_name}")
        value = method()
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ValueError(f"native KiCad class {name!r} has invalid {method_name}")
        values[attribute] = value
    priority = native.GetPriority()
    if isinstance(priority, bool) or not isinstance(priority, int):
        raise ValueError(f"native KiCad class {name!r} has invalid priority")
    return SimpleNamespace(name=name, priority=priority, constituents=[name], **values)


def _explicit_classes(board: Any) -> tuple[dict[str, SimpleNamespace], Any]:
    settings = board.GetDesignSettings()
    net_settings = getattr(settings, "m_NetSettings", None)
    if net_settings is None:
        raise ValueError("native KiCad board has no NET_SETTINGS")
    default = _native_class(net_settings.GetDefaultNetclass(), expected_name="Default")
    result = {default.name: default}
    for map_name, native in net_settings.GetNetclasses().items():
        name = str(map_name)
        if name == "Default":
            raise ValueError("native KiCad non-default map unexpectedly contains Default")
        converted = _native_class(native, expected_name=name)
        if name in result:
            raise ValueError(f"native KiCad returned duplicate class {name!r}")
        result[name] = converted
    if set(result) != {"Default", "Gate", "Motor"}:
        raise ValueError(f"expected exactly Default, Gate, Motor native classes; got {sorted(result)!r}")
    return result, net_settings


def _named_nets(board: Any) -> list[SimpleNamespace]:
    result: list[SimpleNamespace] = []
    seen: set[str] = set()
    for map_name, net in board.GetNetsByName().items():
        name = str(net.GetNetname())
        if name != str(map_name):
            raise ValueError(f"native net map key {str(map_name)!r} disagrees with net name {name!r}")
        if not name:
            # KiCad maps net code zero by the empty name.  It has no legal
            # snapshot record but is retained in GetNetCount() below as the
            # single required sentinel.
            continue
        if name in seen:
            raise ValueError(f"native KiCad returned duplicate net {name!r}")
        seen.add(name)
        result.append(SimpleNamespace(name=name, native=net))
    if not result:
        raise ValueError("native KiCad board has no named nets")
    return result


def _verify_project_patterns(project_path: pathlib.Path, explicit_names: set[str]) -> None:
    """Validate the saved declarations; native per-net resolution remains authoritative.

    KiCad 10 exposes ``GetNetclassPatternAssignments`` as a non-iterable SWIG
    vector.  Reading its JSON source is safe for declaration validation, but
    the code never evaluates a pattern or derives a net membership from it.
    """
    try:
        document = json.loads(project_path.read_text(encoding="utf-8"))
        patterns = document["net_settings"]["netclass_patterns"]
    except (OSError, KeyError, TypeError, json.JSONDecodeError) as error:
        raise ValueError("saved KiCad project has no readable netclass_patterns") from error
    if not isinstance(patterns, list):
        raise ValueError("saved KiCad netclass_patterns is not a list")
    if len(patterns) != 13:
        raise ValueError(f"expected 13 saved netclass patterns; found {len(patterns)}")
    seen: set[tuple[str, str]] = set()
    for index, assignment in enumerate(patterns):
        if not isinstance(assignment, Mapping):
            raise ValueError(f"saved netclass pattern {index} is not an object")
        pattern = assignment.get("pattern")
        class_name = assignment.get("netclass")
        if not isinstance(pattern, str) or not pattern:
            raise ValueError(f"saved netclass pattern {index} has no exact pattern name")
        if not isinstance(class_name, str) or not class_name:
            raise ValueError(f"saved netclass pattern {index} has no exact class name")
        if (pattern, class_name) in seen:
            raise ValueError(f"saved netclass pattern {index} duplicates {pattern!r}")
        seen.add((pattern, class_name))
        if class_name not in explicit_names:
            raise ValueError(
                f"saved netclass pattern {index} targets undefined class {class_name!r}"
            )


def _effective_singletons(
    net_settings: Any,
    nets: Sequence[SimpleNamespace],
    explicit: Mapping[str, SimpleNamespace],
) -> dict[str, SimpleNamespace]:
    result: dict[str, SimpleNamespace] = {}
    for net in nets:
        effective = net_settings.GetEffectiveNetClass(net.name)
        if effective is None:
            raise ValueError(f"native KiCad has no effective class for net {net.name!r}")
        name = str(effective.GetName())
        declared = explicit.get(name)
        if declared is None:
            raise ValueError(
                f"native effective class {name!r} for net {net.name!r} is not an explicit class"
            )
        resolved = _native_class(effective, expected_name=name)
        if resolved.priority != declared.priority or any(
            getattr(resolved, field) != getattr(declared, field)
            for field, _ in PHYSICAL_METHODS
        ):
            raise ValueError(
                f"native effective class {name!r} for net {net.name!r} differs from its "
                "explicit definition (composite effective classes are unsupported)"
            )
        result[net.name] = SimpleNamespace(name=name, constituents=[name])
    return result


def _inspect_source(path: pathlib.Path, named_count: int, native_total: int) -> str:
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"cannot read Flux inspect JSON {path}: {error}") from error
    if document.get("strict") is not True:
        raise ValueError("Flux inspect JSON is not a strict saved-board import")
    source_revision = document.get("source_revision")
    if not isinstance(source_revision, str) or not source_revision:
        raise ValueError("Flux inspect JSON has no source_revision")
    inspect_total = document.get("nets")
    if isinstance(inspect_total, bool) or not isinstance(inspect_total, int):
        raise ValueError("Flux inspect JSON has no integer net count")
    if native_total != named_count + 1:
        raise ValueError("native KiCad net census does not contain exactly one unnamed net-0 sentinel")
    if inspect_total != native_total:
        raise ValueError(
            f"Flux inspect reports {inspect_total} nets, native KiCad reports {native_total}"
        )
    return source_revision


def capture(board_path: pathlib.Path, inspect_path: pathlib.Path) -> str:
    exporter = _load_exporter()
    board_path = board_path.resolve(strict=True)
    inspect_path = inspect_path.resolve(strict=True)
    project_path = board_path.with_suffix(".kicad_pro")
    if not project_path.is_file():
        raise ValueError("matching KiCad project file is absent")
    board_hash = _sha256(board_path)
    project_hash = _sha256(project_path)

    import pcbnew

    board = pcbnew.LoadBoard(str(board_path))
    if board is None:
        raise ValueError(f"native KiCad could not load {board_path}")
    explicit, net_settings = _explicit_classes(board)
    _verify_project_patterns(project_path, set(explicit))
    nets = _named_nets(board)
    source_revision = _inspect_source(
        inspect_path, len(nets), int(board.GetNetCount())
    )
    effective = _effective_singletons(net_settings, nets, explicit)
    project_revision, board_rules, loaded_project = exporter.load_project_rules(board_path)
    if loaded_project.resolve() != project_path:
        raise ValueError("project-rule loader resolved an unexpected project")
    if project_revision != project_hash:
        raise ValueError("project-rule loader digest differs from independently captured project hash")
    snapshot = exporter.build_snapshot(
        source_revision=source_revision,
        project_revision=project_revision,
        api_version=f"native-swig-pcbnew-{pcbnew.Version()}",
        board_rules=board_rules,
        classes=list(explicit.values()),
        nets=nets,
        effective_by_net=effective,
    )
    _stable_file(board_path, board_hash, "saved board")
    _stable_file(project_path, project_hash, "saved project")
    return snapshot


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--board", required=True, type=pathlib.Path)
    parser.add_argument("--inspect", required=True, type=pathlib.Path,
                        help="previous flux-cli inspect-kicad JSON for --board")
    parser.add_argument("--output", required=True, type=pathlib.Path)
    args = parser.parse_args(argv)
    try:
        output = args.output.resolve()
        if output.exists():
            raise ValueError(f"refusing to overwrite existing snapshot {output}")
        snapshot = capture(args.board, args.inspect)
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("x", encoding="utf-8") as stream:
            stream.write(snapshot)
    except Exception as error:
        print(f"native KiCad netclass snapshot failed: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
