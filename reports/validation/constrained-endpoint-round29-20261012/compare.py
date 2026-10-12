import collections, hashlib, json, re
from pathlib import Path
root = Path(__file__).resolve().parent
baseline = root.parent / 'residual-frontier-round27-20261012'
out = {}
for tier in ('tier0', 'tier1'):
    paths = list((root/tier).glob('tier-*/detail-result.json'))
    if not paths: continue
    p=paths[0]; d=json.loads(p.read_text()); x=d['result']
    oldp=next((baseline/tier).glob('tier-*/detail-result.json')); old=json.loads(oldp.read_text())
    board=next(p.parent.glob('*.kicad_pcb')); oldboard=next(oldp.parent.glob('*.kicad_pcb'))
    digest=lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    drcp=p.parent/'kicad-drc.json'; drc=json.loads(drcp.read_text()) if drcp.exists() else {}
    resource=(root/f'{tier}-resource.txt').read_text() if (root/f'{tier}-resource.txt').exists() else ''
    rss=re.search(r'Maximum resident set size \(kbytes\): (\d+)', resource)
    out[tier]={
      'baseline_detail_seconds':old['duration_seconds'],'detail_seconds':d['duration_seconds'],
      'baseline_global_seconds':json.loads((oldp.parent/'global-result.json').read_text())['duration_seconds'],
      'global_seconds':json.loads((p.parent/'global-result.json').read_text())['duration_seconds'],
      'change_percent':100*(d['duration_seconds']/old['duration_seconds']-1),
      'complete':x.get('complete'),'stop_reason':x.get('stop_reason'),'open_connections':x.get('open_connections'),
      'counts':{k:v for k,v in x.items() if k in ['added_tracks','added_track_arcs','added_track_beziers','added_vias','unrounded_non_junction_bends','copper_violations','copper_violation_counts']},
      'kicad':{'available':drcp.exists(),'violations':len(drc.get('violations',[])), 'unconnected_items':len(drc.get('unconnected_items',[])), 'categories':dict(collections.Counter(v['type'] for v in drc.get('violations',[])))},
      'copper_byte_identical':digest(board)==digest(oldboard),'copper_sha256':digest(board),
      'peak_rss_kib':int(rss[1]) if rss else None,
      'baseline_peak_rss_kib':int(re.search(r'Maximum resident set size \(kbytes\): (\d+)', (baseline/f'{tier}-resource.txt').read_text())[1]),
      'baseline_navigation_preparation_microseconds':old['result']['spline_navigation_preparation_microseconds'],
      'timings_and_gpu':{k:v for k,v in x.items() if k.startswith(('incremental_transaction_', 'spline_obstacle_', 'spline_free_space_', 'spline_span_', 'spline_equivalent_', 'spline_non_improving_', 'spline_screening_','spline_navmesh_prepared_','spline_navmesh_geometry_','spline_clearance_','spline_envelope_','spline_gpu_')) or k in ['spline_navigation_preparation_microseconds','spline_navigation_mesh_microseconds','spline_route_refinement_microseconds','spline_trajectory_microseconds','spline_realization_microseconds','spline_exact_merge_microseconds','residual_bend_repair_microseconds']},
      'retained_scaffold_search':x.get('retained_scaffold_search'),
      'baseline_engine_phases':[{k:a.get(k) for k in ['action','elapsed_microseconds','work_units']} for a in old['result'].get('routing_engine_attempts',[])],
      'engine_phases':[{k:a.get(k) for k in ['action','elapsed_microseconds','work_units']} for a in x.get('routing_engine_attempts',[])]}
(root/'comparison.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(out,indent=2))
