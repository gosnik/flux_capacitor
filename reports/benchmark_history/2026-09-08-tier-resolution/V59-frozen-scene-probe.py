import json,math,pathlib,subprocess,copy,time
p=pathlib.Path('build/tier-resolution-v59');solutions=json.load(open(p/'compatible-runups.json'));base=json.load(open(p/'selected-source.jsonl'))
def length(pair):return sum(math.dist(a,b) for v in pair for r in v['runups'] for a,b in zip(r['points'],r['points'][1:]))
solutions.sort(key=lambda pair:(sum(not v['global'] for v in pair),length(pair),repr(pair)))
rows=[];out=p/'scene-probe';out.mkdir(exist_ok=True)
for index,pair in enumerate(solutions[:48]):
 e=copy.deepcopy(base)
 for v,conn in zip(pair,[7,0]):
  r=next(r for r in e['routes'] if r['net']==v['net'] and r['id'].endswith(':'+str(conn)))
  first,last=v['runups'];a=next(i for i,q in enumerate(r['points']) if [q['x_nm'],q['y_nm']]==first['anchor'] and q['layer']==first['layer']);b=next(i for i,q in enumerate(r['points']) if i>a and [q['x_nm'],q['y_nm']]==last['anchor'] and q['layer']==last['layer'])
  r['points'][a:b+1]=[{'x_nm':x,'y_nm':y,'layer':run['layer']} for run in v['runups'] for x,y in (run['points'] if run is first else reversed(run['points']))]
 f=out/f'{index}.jsonl';f.write_text(json.dumps(e)+'\n')
 start=time.monotonic();run=subprocess.run(['build/release/flux-replay-quantized-copper','tests/fixtures/routing-corpus/bm_svf.kicad_pcb',str(f),'1'],capture_output=True,text=True,check=True)
 (out/f'{index}-proof.jsonl').write_text(run.stdout);d=json.loads(run.stdout.splitlines()[-1]);rows.append({'index':index,'choice_ids':[v['id'] for v in pair],'length_nm':length(pair),'seconds':time.monotonic()-start,**{k:d[k] for k in ['tracks','vias','drc_violations','categories','opens']}})
(out/'summary.json').write_text(json.dumps(rows,indent=2)+'\n');print(json.dumps(sorted(rows,key=lambda r:(r['opens'],r['drc_violations']))[:3],indent=2))
