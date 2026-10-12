import json
from pathlib import Path
root=Path(__file__).resolve().parent
for tier in ('tier0','tier1','tier2','tier3'):
    result=[]
    for p in (root/tier).glob('tier-*/*result.json'):
        d=json.loads(p.read_text()); x=d.get('result',{})
        s=x.get('copper_scaffold',{})
        result.append({'file':p.name,'duration':d.get('duration_seconds'),'complete':x.get('complete'),'stop':x.get('stop_reason'),'scaffold':{k:s.get(k) for k in ['request_count','satisfied_connections','physical_open_connections','missing_indices']},'opens':x.get('open_connections')})
    traces=[]
    for p in (root/tier/'telemetry').glob('*.jsonl'):
        with p.open('rb') as f:
            f.seek(max(0,p.stat().st_size-8*1024*1024)); lines=f.read().splitlines()
        for line in reversed(lines):
            try:d=json.loads(line)
            except Exception:continue
            traces.append({'file':p.name,'sequence':d.get('sequence'),'seconds':d.get('monotonic_time_us',0)/1e6,'progress':d.get('progress'),'message':d.get('message','')[:200]});break
    print(json.dumps({'tier':tier,'results':result,'telemetry':traces}))
