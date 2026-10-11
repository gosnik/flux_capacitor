import collections, json
from pathlib import Path
root=Path(__file__).resolve().parent
out={"binary_sha256":json.loads((root/"source-build.json").read_text())["binary_sha256"],"tiers":{}}
for tier in ("tier0","tier1"):
    t={}
    for p in (root/tier).glob("tier-*/*result.json"):
        d=json.loads(p.read_text());x=d["result"]
        keys=["complete","stop_reason","diagnostic","open_connections","copper_violations","added_tracks","added_track_arcs","added_track_beziers","added_vias","unrounded_non_junction_bends"]
        keys += [k for k in x if k.startswith("residual_bend_repair_")]
        t[p.stem]={"seconds":d["duration_seconds"],**{k:x.get(k) for k in keys}}
        if p.name=="global-result.json":t[p.stem]["scaffold"]={k:x.get("copper_scaffold",{}).get(k) for k in ["request_count","satisfied_connections","physical_open_connections","physical_unrouted_nets"]}
    for p in (root/tier).glob("tier-*/kicad-drc.json"):
        d=json.loads(p.read_text());t["kicad"]={"path":str(p),"violations":len(d.get("violations",[])),"unconnected_items":len(d.get("unconnected_items",[])),"categories":dict(collections.Counter(v.get("type") for v in d.get("violations",[])))}
    exitfile=root/f"{tier}-exit.txt"
    if exitfile.exists():t["runner_exit"]=exitfile.read_text().strip()
    t["qualified"]=t.get("detail-result",{}).get("complete") is True and t.get("kicad",{}).get("violations")==0 and t.get("kicad",{}).get("unconnected_items")==0
    out["tiers"][tier]=t
(root/"validation-result.json").write_text(json.dumps(out,indent=2)+"\n")
print(json.dumps({k:{"qualified":v["qualified"],"runner_exit":v.get("runner_exit")} for k,v in out["tiers"].items()}))
