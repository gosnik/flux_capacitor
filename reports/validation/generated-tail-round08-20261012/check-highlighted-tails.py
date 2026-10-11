#!/usr/bin/env python3
"""Check whether known bare copper tips survive in exported KiCad boards."""
import json
import math
import re
from pathlib import Path

ROOT = Path(__file__).parent
BASELINE = Path("reports/validation/pad-access-round07-20261012/tier1/tier-1-bm_svf/bm_svf.kicad_pcb")
NEW = Path("reports/validation/generated-tail-round08-20261012/tier1/tier-1-bm_svf/bm_svf.kicad_pcb")
TIPS = [
    {"net": 8, "label": "V_CV", "layer": "F.Cu", "x": 55.44, "y": 55.59},
    {"net": 12, "label": "V_SUPPLY", "layer": "F.Cu", "x": 58.04, "y": 52.99},
    {"net": 12, "label": "V_SUPPLY", "layer": "F.Cu", "x": 59.99, "y": 51.69},
    {"net": 21, "label": "Net-(U5B-+)", "layer": "F.Cu", "x": 66.49, "y": 56.89},
]

def blocks(text):
    for m in re.finditer(r"\((segment|arc|gr_curve)\b", text):
        depth, i, quoted, escaped = 0, m.start(), False, False
        while i < len(text):
            c = text[i]
            if c == '"' and not escaped: quoted = not quoted
            elif c == "(" and not quoted: depth += 1
            elif c == ")" and not quoted:
                depth -= 1
                if depth == 0:
                    yield m.group(1), text[m.start():i + 1]
                    break
            escaped = (c == "\\" and not escaped)
            if c != "\\": escaped = False
            i += 1

def inventory(path):
    if not path.exists():
        return {"path": str(path), "exists": False}
    text = path.read_text(errors="replace")
    labels = {int(n): s for n, s in re.findall(r'\(net\s+(\d+)\s+"([^"]*)"\)', text)}
    prims = []
    for kind, block in blocks(text):
        lm = re.search(r'\(layer\s+"([^"]+)"\)', block)
        nm = re.search(r'\(net\s+(\d+)\)', block)
        named = re.search(r'\(net\s+"([^"]+)"\)', block)
        net = int(nm.group(1)) if nm else next((n for n, s in labels.items() if named and s == named.group(1)), None)
        if not (lm and net is not None):
            continue
        pts = [(float(x), float(y)) for x, y in re.findall(r'\(xy\s+([-+]?\d+(?:\.\d+)?)\s+([-+]?\d+(?:\.\d+)?)\)', block)]
        for tag in ("start", "end"):
            q = re.search(r'\(%s\s+([-+]?\d+(?:\.\d+)?)\s+([-+]?\d+(?:\.\d+)?)\)' % tag, block)
            if q: pts.append((float(q.group(1)), float(q.group(2))))
        if kind == "gr_curve" and pts:
            pts = [pts[0], pts[-1]]
        if pts:
            prims.append({"kind": kind, "layer": lm.group(1), "net": net, "points": pts})
    counts = {}
    for p in prims: counts[p["kind"]] = counts.get(p["kind"], 0) + 1
    hits = []
    for tip in TIPS:
        candidates = [p for p in prims if p["net"] == tip["net"] and p["layer"] == tip["layer"]]
        exact = [p["kind"] for p in candidates if any(math.isclose(x, tip["x"], abs_tol=1e-6) and math.isclose(y, tip["y"], abs_tol=1e-6) for x, y in p["points"])]
        on_segment = []
        for p in candidates:
            if p["kind"] != "segment" or len(p["points"]) < 2: continue
            for a, b in zip(p["points"][::2], p["points"][1::2]):
                ax, ay, bx, by = *a, *b
                cross = abs((tip["x"]-ax)*(by-ay) - (tip["y"]-ay)*(bx-ax))
                dot = (tip["x"]-ax)*(tip["x"]-bx) + (tip["y"]-ay)*(tip["y"]-by)
                if cross <= 1e-5 and dot <= 1e-8: on_segment.append("segment")
        hits.append({**tip, "exact_endpoint_kinds": exact, "on_segment": bool(on_segment)})
    return {"path": str(path), "exists": True, "net_labels": labels, "primitive_counts": counts, "total_primitives": len(prims), "tips": hits, "precision": "exact endpoints; straight-segment containment only; arcs/gr_curve interiors are not geometrically evaluated"}

print(json.dumps({"baseline": inventory(BASELINE), "new": inventory(NEW)}, sort_keys=True, indent=2))
