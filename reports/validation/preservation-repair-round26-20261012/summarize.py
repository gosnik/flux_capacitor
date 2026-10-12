import json, re
from pathlib import Path
root=Path(__file__).resolve().parent
manifest=json.loads((root/'source-build.json').read_text())
summary={'git_revision':manifest['git_revision'],'binary_sha256':manifest['binary_sha256'],'configuration':{'sequential':True,'curved':True,'telemetry':'diagnostic','iterations':30,'routing_timeout_seconds':900,'outer_timeout_seconds':960},'tiers':{}}
for tier in ('tier2',):
 data={}
 for stage in ('global','detail'):
  files=list((root/tier).glob(f'tier-*/{stage}-result.json'))
  if not files: continue
  r=json.loads(files[0].read_text()); x=r.get('result',{})
  keys=['complete','stop_reason','diagnostic','routing_plan_complete','paths','unrouted_nets','unrouted_net_ids','total_overflow','corridor_overflow','clearance_shadow_overflow','open_connections','copper_violations','copper_violation_counts','added_tracks','added_track_arcs','added_track_beziers','added_vias','unrounded_non_junction_bends','routing_engine_outcome']
  data[stage]={'duration_seconds':r.get('duration_seconds'),'command_configuration':r.get('command_configuration'),**{k:x[k] for k in keys if k in x},'scaffold':x.get('copper_scaffold'),'phases':[{k:a.get(k) for k in ['action','elapsed_microseconds','work_units','accepted','diagnostic']} for a in x.get('routing_engine_attempts',[])]}
 drcs=list((root/tier).glob('**/kicad-drc.json'))
 data['kicad_drc']=[{'path':str(p.relative_to(root)),'violations':len((d:=json.loads(p.read_text())).get('violations',[])),'unconnected_items':len(d.get('unconnected_items',[]))} for p in drcs]
 data['final_copper_files']=[str(p.relative_to(root)) for p in (root/tier).glob('tier-*/*.kicad_pcb')]
 resource=root/f'{tier}-resource.txt'
 if resource.exists():
  m=re.search(r'Maximum resident set size \(kbytes\): (\d+)',resource.read_text()); data['peak_rss_kib']=int(m[1]) if m else None
 exitfile=root/f'{tier}-exit.txt'
 if exitfile.exists(): data['harness_exit']=int(exitfile.read_text())
 summary['tiers'][tier]=data
for data in summary['tiers'].values():
 for stage in ('global', 'detail'):
  scaffold=data.get(stage,{}).get('scaffold')
  if scaffold: scaffold['request_evidence']=[dict(e, request_index=i) for i,e in enumerate(scaffold.get('request_evidence',[])) if i in scaffold.get('missing_indices',[])]
(root/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps({t:{k:v for k,v in x.items() if k not in ['global','detail']} for t,x in summary['tiers'].items()}))
