import datetime,json,subprocess,sys
from pathlib import Path
root=Path(__file__).resolve().parent
phases=['tier0','tier1','tier2','esc-check']
results=[]
for phase in phases:
 record={'phase':phase,'started_at':datetime.datetime.now(datetime.timezone.utc).isoformat(),'results':results}
 (root/'suite-progress.json').write_text(json.dumps(record,indent=2)+'\n')
 cmd=['bash',str(root/'run_tiers.sh'),phase] if phase.startswith('tier') else [sys.executable,str(root/'esc-check/run.py')]
 code=subprocess.run(cmd).returncode
 results.append({'phase':phase,'return_code':code})
 print(json.dumps(results[-1]),flush=True)
 if code:raise SystemExit(code)
 if phase.startswith('tier'):subprocess.run([sys.executable,str(root/'summarize.py')],check=True,stdout=subprocess.DEVNULL)
(root/'suite-progress.json').write_text(json.dumps({'phase':'complete','results':results},indent=2)+'\n')
