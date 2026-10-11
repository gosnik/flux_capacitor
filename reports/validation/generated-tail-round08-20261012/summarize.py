"""Summarize whichever round-08 tier results are currently available."""
import collections
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent
OUT = ROOT.parent.parent / "benchmark_history" / "2026-10-12-generated-tail-round08.json"
TIERS = ("tier0", "tier1")
SCAFFOLD_KEYS = (
    "request_count", "satisfied_connections", "routed_connections", "missing_connections",
    "physical_open_connections", "physical_unrouted_nets", "complete", "missing_indices",
    "source_attachments", "target_attachments", "searched_connections", "retained_paths",
    "attachment_exhausted", "exchange_rollbacks", "victim_exchanges_attempted",
    "victim_exchanges_accepted", "displacement_attempts", "displacement_accepted",
    "joint_group_attempts", "joint_group_accepted", "coupled_victim_exchanges_attempted",
    "coupled_victim_exchanges_accepted",
    "compact_search_attempts", "compact_search_expansions", "compact_search_fallbacks",
    "compact_search_successes", "expansions", "goal_rejections",
)


def read(path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return None


def drc_summary(path):
    d = read(path)
    if not d:
        return None
    violations = d.get("violations", [])
    return {"path": str(path), "violations": len(violations),
            "unconnected_items": len(d.get("unconnected_items", [])),
            "categories": dict(collections.Counter(v.get("type") for v in violations))}


def peak_rss(tier):
    p = ROOT / f"{tier}-resource.txt"
    if not p.exists():
        return None
    m = re.search(r"Maximum resident set size \(kbytes\):\s*(\d+)", p.read_text())
    return int(m.group(1)) if m else None


def board_summary(tier, board):
    detailed = board.get("detailed") or {}
    global_ = board.get("global") or {}
    drc = next((drc_summary(p) for p in (ROOT / tier).glob("tier-*/*kicad-drc.json")), None)
    g = global_.get("result") or {}
    d = detailed.get("result") or {}
    scaffold = g.get("copper_scaffold") or {}
    missing = set(scaffold.get("missing_indices", []))
    # request_evidence is positional; entries do not carry request_index in the emitted schema.
    term = collections.Counter(e.get("termination") for i, e in enumerate(scaffold.get("request_evidence", []))
                               if i in missing)
    runtime = detailed.get("duration_seconds") or global_.get("duration_seconds")
    complete = d.get("complete", g.get("complete"))
    stop = d.get("stop_reason", g.get("stop_reason"))
    return {"board": board.get("id"), "level": board.get("level"),
            "status": board.get("effective_status", board.get("declared_status")),
            "qualified": board.get("qualified"),
            "runtime_seconds": runtime,
            "global_runtime_seconds": global_.get("duration_seconds"),
            "peak_rss_kib": peak_rss(tier) or g.get("peak_rss_kib", d.get("peak_rss_kib")),
            "complete": complete, "stop_reason": stop,
            "open_connections": d.get("open_connections"),
            "scaffold": {k: scaffold.get(k) for k in SCAFFOLD_KEYS if k in scaffold},
            "missing_termination_counts": term,
            "tail_cleanup": {k: v for k, v in d.items() if "generated_tail_cleanup" in k},
            "final_counts": {k: d[k] for k in d if k.startswith(("added_", "unrounded_", "internal_"))},
            "internal_violations": d.get("copper_violations", d.get("violations")),
            "final_curves": {k: d[k] for k in d if k.startswith(("added_", "attempted_")) and
                             any(s in k for s in ("curve", "bezier", "arc"))},
            "kicad_drc": drc}


def main():
    frozen = read(ROOT / "source-build.json") or {}
    tiers = {}
    for tier in TIERS:
        run = read(ROOT / tier / "run-result.json")
        if not run:
            tiers[tier] = {"available": False}
            continue
        tiers[tier] = {"available": True, "boards": [board_summary(tier, b) for b in run.get("boards", [])],
                       "runner_exit": (ROOT / f"{tier}-exit.txt").read_text().strip()
                       if (ROOT / f"{tier}-exit.txt").exists() else None}
    source_hashes = frozen.get("source_sha256") or {}
    source_manifest_hash = __import__("hashlib").sha256(
        json.dumps(source_hashes, sort_keys=True, separators=(",", ":")).encode()).hexdigest() if source_hashes else None
    out = {"round": 8, "experiment": "generated-tail-cleanup", "git_revision": frozen.get("git_revision"),
           "configuration": {"lower_tier_iterations": "manifest/default", "routing_timeout_seconds": 900,
                              "curved": True, "telemetry": "diagnostic", "sequential": True},
           "frozen_source_manifest_sha256": source_manifest_hash,
           "binary_sha256": frozen.get("binary_sha256"), "tiers": tiers,
           "diagnosis": read(ROOT / "diagnosis.json")}
    OUT.write_text(json.dumps(out, indent=2, sort_keys=True, default=lambda x: dict(x)) + "\n")
    print(json.dumps({k: v.get("available") for k, v in tiers.items()}))


if __name__ == "__main__":
    main()
