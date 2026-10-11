import json, re, datetime
from pathlib import Path
root = Path(__file__).resolve().parent
repo = root.parents[2]
boards = []
for tier, name in [(0, "pt2399_delay"), (1, "bm_svf")]:
    folder = root / f"tier{tier}"
    run = folder / f"tier-{tier}-{name}"
    if not (run / "global-result.json").exists():
        continue
    global_run = json.loads((run / "global-result.json").read_text())
    g = global_run["result"]
    b = {"board": name, "tier": tier, "global_seconds": global_run["duration_seconds"],
         "engine_outcome": g.get("routing_engine_outcome"), "global_scaffold": g.get("copper_scaffold"),
         "engine_metrics": {k:v for k,v in g.items() if k.startswith("routing_engine_")},
         "board_terrain_guidance": {k:v for k,v in g.items() if k.startswith("board_terrain_guidance_") or k == "configured_board_terrain_guidance"},
         "regional_hc": {k:v for k,v in g.items() if k.startswith("regional_hc_")},
         "layered_navmesh": {k:v for k,v in g.items() if k.startswith("regional_layered_navmesh_")},
         "global_metrics": {k:g.get(k) for k in ["paths", "unrouted_nets", "total_overflow", "clearance_shadow_overflow", "corridor_overflow", "sampled_conflicts", "regional_conflict_directed_no_good_unknown_withheld", "regional_conflict_directed_no_good_rejections", "regional_candidate_preflight_rejections"]},
         "configuration": global_run.get("command_configuration"), "final_drc": None,
         "artifacts": str(folder.relative_to(repo))}
    detail = run / "detail-result.json"
    if detail.exists():
        d = json.loads(detail.read_text())
        b["detail_seconds"] = d["duration_seconds"]
        b["detailed_metrics"] = {k:d["result"].get(k) for k in ["complete", "stop_reason", "open_connections", "unrouted_nets", "copper_violations", "copper_violation_counts", "unrounded_non_junction_bends", "unrounded_straight_track_bends", "unrounded_curve_tangent_bends", "residual_bend_repair_attempted", "residual_bend_repair_accepted", "residual_bend_repair_rejected", "added_tracks", "added_track_beziers", "added_track_arcs", "added_vias"]}
    drcs = list(folder.rglob("kicad-drc.json"))
    if drcs:
        drc = json.loads(drcs[0].read_text())
        counts = {}
        for v in drc["violations"]: counts[v.get("type", "unknown")] = counts.get(v.get("type", "unknown"), 0) + 1
        b["final_drc"] = {"violations": len(drc["violations"]), "unconnected_items": len(drc["unconnected_items"]), "categories": counts, "path": str(drcs[0].relative_to(repo)), "scope": "rejected_diagnostic_copper" if "detailed-rejected" in drcs[0].parts else "final_output"}
    resource = root / f"tier{tier}-resource.txt"
    if resource.exists():
        peak = re.search(r"Maximum resident set size \(kbytes\): (\d+)", resource.read_text())
        if peak: b["peak_rss_kib"] = int(peak[1])
    result = folder / "run-result.json"
    if result.exists():
        r = json.loads(result.read_text())
        b["runner_status"] = r.get("status")
        b["runner_diagnostic"] = r.get("diagnostic")
    b["tier_passed"] = bool(b.get("detailed_metrics", {}).get("complete") and b["final_drc"] and b["final_drc"]["scope"] == "final_output" and b["final_drc"]["violations"] == 0 and b["final_drc"]["unconnected_items"] == 0 and b.get("runner_status") == "complete")
    boards.append(b)
summary = {"experiment": "compact physical search with rich-state fallback", "recorded_at": datetime.datetime.now(datetime.timezone.utc).isoformat(), "build": json.loads((root / "source-build.json").read_text()), "baseline": "reports/benchmark_history/2026-10-11-joint-passages.json", "configuration": {"order": ["tier0", "tier1"], "telemetry": "diagnostic", "routing_timeout_seconds": 900, "track_geometry": "curved", "compact_search": {"maximum_expansions": 25000, "maximum_share_of_connection_budget": 0.5, "exact_admission": True, "rich_state_fallback": True}}, "validation": json.loads((root / "validation-result.json").read_text()) if (root / "validation-result.json").exists() else {"status": "pending final validation"}, "boards": boards}
(repo / "reports/benchmark_history/2026-10-11-compact-search.json").write_text(json.dumps(summary, indent=2)+"\n")
print(json.dumps([{k:v for k,v in b.items() if k in ["board", "global_seconds", "detail_seconds", "engine_outcome", "detailed_metrics", "final_drc", "peak_rss_kib"]} for b in boards], indent=2))
