import json,re
from pathlib import Path
r=Path(__file__).resolve().parent
repo=r.parents[2]
history=repo/'reports/benchmark_history/round40-regional-realization.json'
d=json.loads(history.read_text());d['tiers']=[]
old_root=repo/'reports/validation/eviction-liveness-round39-20261014'
for tier in range(3):
 f=r/f'tier{tier}/run-result.json'
 if not f.exists():continue
 run=json.loads(f.read_text());b=run['boards'][0];detail=b['detailed'];x=detail['result']
 case=r/f'tier{tier}/tier-{tier}-{b["id"]}';old=old_root/f'tier{tier}/tier-{tier}-{b["id"]}'
 drc=json.loads((case/'kicad-drc.json').read_text());prior=json.loads((old_root/f'tier{tier}/run-result.json').read_text())['boards'][0]
 resource=(r/f'tier{tier}-resource.txt').read_text();rss=re.search(r'Maximum resident set size \(kbytes\): (\d+)',resource)
 row={'tier':tier,'board':b['id'],'passed':run['status']=='complete' and b['qualified'],
 'detail_seconds':detail['duration_seconds'],'previous_detail_seconds':prior['detailed']['duration_seconds'],
 'global_seconds':b['global']['duration_seconds'],'opens':x['open_connections'],
 'internal_copper_violations':x['copper_violations'],'copper_violation_categories':x.get('copper_violation_counts',{}),
 'tracks':x['added_tracks'],'beziers':x['added_track_beziers'],'arcs':x['added_track_arcs'],'vias':x['added_vias'],
 'copper_identical':(case/f'{b["id"]}.kicad_pcb').read_bytes()==(old/f'{b["id"]}.kicad_pcb').read_bytes(),
 'kicad_violations':len(drc.get('violations',[])),'kicad_reported_unconnected_items':len(drc.get('unconnected_items',[])),
 'peak_process_tree_rss_kib':int(rss.group(1)) if rss else None,
 'scaffold_search_expansions':x.get('retained_scaffold_search',{}).get('expansions'),
 'router_global_iterations':x.get('global_iterations'),
 'recorded_telemetry':str((r/f'tier{tier}/telemetry').relative_to(repo))}
 d['tiers'].append(row)
history.write_text(json.dumps(d,indent=2)+'\n');print(json.dumps(d['tiers'],indent=2))
