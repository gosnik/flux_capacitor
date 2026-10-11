import datetime,json,re,subprocess
from pathlib import Path
root=Path(__file__).resolve().parent
out={'round':2,'experiment':'physical producer rebind after expansion exhaustion','baseline_commit':'6a9ae7f','date':datetime.datetime.now(datetime.timezone.utc).isoformat(),'source_manifest':str(root/'source-build.json'),'tiers':{},'goal_achieved':False}
for tier in ('tier0','tier1'):
    row={}
    for p in sorted((root/tier).glob('tier-*/*result.json')):
        raw=json.loads(p.read_text()); x=raw['result'];s=x.get('copper_scaffold',{})
        row[p.stem]={'duration_seconds':raw.get('duration_seconds'),'complete':x.get('complete'),'stop_reason':x.get('stop_reason'),'copper_scaffold':s,'metrics':{k:v for k,v in x.items() if k in ['open_connections','copper_violations','added_tracks','added_track_beziers','added_track_arcs','added_vias','unrounded_curve_tangent_bends','unrounded_non_junction_bends','routing_engine_outcome','routing_engine_scaffold_terminal_rebinds','routing_engine_scaffold_oracle']}}
    for p in (root/tier).rglob('kicad-drc.json'):
        d=json.loads(p.read_text());row['diagnostic_kicad_drc']={'path':str(p),'violations':len(d.get('violations',[])),'unconnected_items':len(d.get('unconnected_items',[])),'ignored_checks':d.get('ignored_checks')}
    resource=root/f'{tier}-resource.txt'
    if resource.exists():
        m=re.search(r'Maximum resident set size \(kbytes\): (\d+)',resource.read_text())
        if m:row['peak_rss_kib']=int(m[1])
    out['tiers'][tier]=row
(root/'benchmark-summary.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps({t:{k:v for k,v in x.items() if k in ['peak_rss_kib','diagnostic_kicad_drc']} for t,x in out['tiers'].items()}))
