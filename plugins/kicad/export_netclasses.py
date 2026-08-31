#!/usr/bin/env python3
"""Export authoritative KiCad IPC net-class policy as a Flux snapshot.

The module imports kicad-python only in main(), so its deterministic rendering can be tested
without KiCad or third-party packages. Optimization remains in the C++ core.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import subprocess
import sys
from collections.abc import Mapping, Sequence
from typing import Any


PHYSICAL_FIELDS = (
    ("clearance_nm", "clearance"),
    ("track_width_nm", "track_width"),
    ("via_diameter_nm", "via_diameter"),
    ("via_drill_nm", "via_drill"),
    ("diff_pair_track_width_nm", "diff_pair_track_width"),
    ("diff_pair_gap_nm", "diff_pair_gap"),
    ("diff_pair_via_gap_nm", "diff_pair_via_gap"),
)


def _quote(value: str) -> str:
    replacements = {
        "\\": "\\\\",
        '"': '\\"',
        "\n": "\\n",
        "\r": "\\r",
        "\t": "\\t",
    }
    return '"' + "".join(replacements.get(character, character) for character in value) + '"'


def _explicit_classes(classes: Sequence[Any]) -> list[Any]:
    result: list[Any] = []
    names: set[str] = set()
    for net_class in classes:
        name = str(net_class.name)
        if not name:
            raise ValueError("KiCad returned a net class with an empty name")
        if name in names:
            raise ValueError(f"KiCad returned duplicate net class {name!r}")
        constituents = net_class.constituents
        # KiCad 9 reports project definitions through the IPC API as implicit
        # singleton classes.  This is equivalent to an explicit definition,
        # but accepting any other implicit shape would blur definitions with
        # effective composite memberships.
        if constituents is not None and [str(value) for value in constituents] != [name]:
            raise ValueError(f"project class {name!r} is unexpectedly implicit")
        names.add(name)
        result.append(net_class)
    if "Default" not in names:
        raise ValueError("KiCad project has no Default net class")
    default = next(net_class for net_class in result if net_class.name == "Default")
    for _, attribute in PHYSICAL_FIELDS:
        if getattr(default, attribute) is None:
            raise ValueError(f"KiCad Default net class has no {attribute}")
    return sorted(result, key=lambda net_class: (int(net_class.priority), str(net_class.name)))


def _memberships(effective: Any, explicit_names: set[str]) -> list[str]:
    constituents = effective.constituents
    names = [str(effective.name)] if constituents is None else [str(name) for name in constituents]
    result: list[str] = []
    seen: set[str] = set()
    for name in names:
        if name == "Default":
            continue
        if name not in explicit_names:
            raise ValueError(f"effective net class references undefined constituent {name!r}")
        if name in seen:
            raise ValueError(f"effective net class repeats constituent {name!r}")
        seen.add(name)
        result.append(name)
    return result


def build_snapshot(
    source_revision: str,
    api_version: str,
    classes: Sequence[Any],
    nets: Sequence[Any],
    effective_by_net: Mapping[str, Any],
) -> str:
    if not source_revision:
        raise ValueError("source revision must be non-empty")
    if not api_version:
        raise ValueError("KiCad API version must be non-empty")
    explicit = _explicit_classes(classes)
    explicit_names = {str(net_class.name) for net_class in explicit}
    named_nets: dict[str, Any] = {}
    for net in nets:
        name = str(net.name)
        if not name:
            continue
        if name in named_nets:
            raise ValueError(f"KiCad returned duplicate net {name!r}")
        named_nets[name] = net
    missing = sorted(set(named_nets) - set(effective_by_net))
    extra = sorted(set(effective_by_net) - set(named_nets) - {""})
    if missing:
        raise ValueError(f"KiCad omitted effective classes for nets: {missing!r}")
    if extra:
        raise ValueError(f"KiCad returned effective classes for unknown nets: {extra!r}")

    lines = [
        "(flux_kicad_netclass_snapshot",
        "  (version 2)",
        f"  (source_revision {_quote(source_revision)})",
        f"  (api_version {_quote(api_version)})",
    ]
    for net_class in explicit:
        lines.extend(
            [
                "  (class",
                f"    (name {_quote(str(net_class.name))})",
                f"    (source_priority {int(net_class.priority)})",
            ]
        )
        for field, attribute in PHYSICAL_FIELDS:
            value = getattr(net_class, attribute)
            if value is not None:
                lines.append(f"    ({field} {int(value)})")
        lines.append("  )")
    precedence = {str(net_class.name): index for index, net_class in enumerate(explicit)}
    for name in sorted(named_nets):
        memberships = _memberships(effective_by_net[name], explicit_names)
        memberships.sort(key=lambda class_name: (precedence[class_name], class_name))
        rendered = "".join(f" {_quote(class_name)}" for class_name in memberships)
        lines.extend(
            [
                "  (net",
                f"    (name {_quote(name)})",
                f"    (classes{rendered})",
                "  )",
            ]
        )
    lines.append(")")
    return "\n".join(lines) + "\n"


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Export live KiCad project net classes for flux_capacitor"
    )
    parser.add_argument("--board", required=True, type=pathlib.Path)
    parser.add_argument("--flux-cli", default="flux-cli")
    args = parser.parse_args(argv)
    try:
        from kipy import KiCad

        board_path = args.board.resolve(strict=True)
        stat_before = board_path.stat()
        kicad = KiCad(client_name="flux_capacitor net-class bridge")
        kicad.check_version()
        kicad_version = kicad.get_version()
        board = kicad.get_board()
        live_path = (
            pathlib.Path(board.document.project.path) / board.document.board_filename
        ).resolve(strict=True)
        if live_path != board_path:
            raise ValueError(
                f"active KiCad board {str(live_path)!r} does not match {str(board_path)!r}"
            )
        project = kicad.get_project(board.document)
        nets = board.get_nets()
        inspected = subprocess.run(
            [args.flux_cli, "inspect-kicad", str(board_path)],
            check=True,
            capture_output=True,
            text=True,
        )
        import_result = json.loads(inspected.stdout)
        if not import_result.get("strict"):
            raise ValueError("C++ engine did not strictly import the saved KiCad board")
        source_revision = import_result.get("source_revision")
        if not isinstance(source_revision, str) or not source_revision:
            raise ValueError("C++ import did not return a source revision")
        snapshot = build_snapshot(
            source_revision,
            str(kicad_version),
            project.get_net_classes(),
            nets,
            board.get_netclass_for_nets(nets),
        )
        stat_after = board_path.stat()
        if (stat_before.st_mtime_ns, stat_before.st_size) != (
            stat_after.st_mtime_ns,
            stat_after.st_size,
        ):
            raise ValueError("saved board changed while its net-class snapshot was captured")
    except Exception as error:  # IPC failures are reported at the process boundary.
        print(f"flux KiCad net-class export failed: {error}", file=sys.stderr)
        return 2
    sys.stdout.write(snapshot)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
