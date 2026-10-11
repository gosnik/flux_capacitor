"""Compare actual detailed-command cost and copper against the frozen prior round."""
import hashlib
import json
from pathlib import Path
root = Path(__file__).resolve().parent
baseline = root.parent / 'generated-tail-round08-20261012'
rows = {}
for tier in ('tier0', 'tier1'):
    current = list((root/tier).glob('tier-*/detail-result.json'))
    previous = list((baseline/tier).glob('tier-*/detail-result.json'))
    if not current or not previous:
        continue
    a, b = json.loads(previous[0].read_text()), json.loads(current[0].read_text())
    aa = next(previous[0].parent.glob('*.kicad_pcb'))
    bb = next(current[0].parent.glob('*.kicad_pcb'))
    timings = {k:v for k,v in b['result'].items() if k.endswith('_microseconds') or 'spline_navigation_preparation' in k}
    rows[tier] = {'baseline_detail_seconds':a['duration_seconds'], 'detail_seconds':b['duration_seconds'],
                  'change_percent':100*(b['duration_seconds']/a['duration_seconds']-1),
                  'copper_byte_identical':aa.read_bytes()==bb.read_bytes(),
                  'baseline_copper_sha256':hashlib.sha256(aa.read_bytes()).hexdigest(),
                  'copper_sha256':hashlib.sha256(bb.read_bytes()).hexdigest(),
                  'timings_and_cache':timings,
                  'engine_phases':[{k:x[k] for k in ('action','elapsed_microseconds','work_units') if k in x} for x in b['result'].get('routing_engine_attempts',[])],
                  'candidate_checks': b['result'].get('spline_trajectory_candidate_checks')}
(root/'comparison.json').write_text(json.dumps(rows,indent=2)+'\n')
print(json.dumps(rows,indent=2))
