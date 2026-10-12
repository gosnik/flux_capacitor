import json
from pathlib import Path
root=Path(__file__).resolve().parent
out={}
for tier in ('tier0','tier1','tier2'):
 records=[]
 for path in sorted((root/tier/'telemetry').glob('*.jsonl')):
  with path.open() as f:
   for line in f:
    if 'scaffold-search-result' not in line and 'scaffold-request-result' not in line: continue
    x=json.loads(line)
    records.append({'file':path.name,'sequence':x.get('sequence'),'monotonic_time_us':x.get('monotonic_time_us'),'message':x.get('message')})
 if records: out[tier]=records
(root/'search-evidence.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(out,indent=2))
