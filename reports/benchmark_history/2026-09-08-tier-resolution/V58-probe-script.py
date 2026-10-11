"""Diagnostic whole-net layer alternatives; never an accepted routing transaction."""
import collections,copy,json,subprocess,sys,hashlib,time
from pathlib import Path
cli,board,scene_path,out_path=sys.argv[1:]
out=Path(out_path);out.mkdir(exist_ok=True)
scene=json.loads(Path(scene_path).read_text());scene['sequence']=1
routes=[r for r in scene['routes'] if r['id'].startswith('global-route:') and r['family']=='selected' and r['lifecycle']=='active']
nets=sorted({r['net'] for r in routes}); layers=sorted({p['layer'] for r in routes for p in r['points']})
def probe(path):
 p=subprocess.run([cli,board,str(path),'1'],capture_output=True,text=True,timeout=60)
 if p.returncode:raise RuntimeError(p.stderr)
 return [json.loads(l) for l in p.stdout.splitlines()][-1]
baseline=probe(scene_path);pressure=collections.Counter()
for group in baseline['largest_pair_groups']:
 for net in nets:
  if net+':' in group['net_pair_and_layer']:pressure[net]+=group['coalesced_diagnoses']
ranked=sorted(nets,key=lambda n:(-pressure[n],n))[:5];results=[];started=time.monotonic()
for net in ranked:
 for layer in layers:
  candidate=copy.deepcopy(scene);changed=False;valid=True
  for r in candidate['routes']:
   if r['net']!=net or not r['id'].startswith('global-route:'):continue
   points=[]
   for point in r['points']:
    changed|=point['layer']!=layer;point['layer']=layer
    if not points or points[-1]!=point:points.append(point)
   if len(points)<2:valid=False;break
   r['points']=points
  if not changed or not valid:continue
  path=out/f'{net}-{layer}.jsonl';path.write_text(json.dumps(candidate)+'\n');r=probe(path)
  results.append({'net':net,'layer':layer,'scene':str(path),'result':r})
  print(json.dumps({'net':net,'layer':layer,'drc':r['drc_violations'],'opens':r['opens'],'vias':r['vias']}),flush=True)
report={'authoritative':False,'qualified':False,'method':'Flatten every branch of one complete selected net to one existing copper layer. Reject via collapse with fewer than two vertices. Whole-scene exact connectivity/DRC checks all changed terminal and static-copper interactions.','baseline':baseline,'candidates':results,'elapsed_seconds':time.monotonic()-started,'cli_sha256':hashlib.sha256(Path(cli).read_bytes()).hexdigest()}
(out/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
