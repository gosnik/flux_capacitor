import datetime, hashlib, json, os, re, signal, subprocess, time
from decimal import Decimal
from pathlib import Path
root=Path(__file__).resolve().parent
board=root/'prepared-selective.kicad_pcb'; project=board.with_suffix('.kicad_pro')
p=json.loads(project.read_text()); settings=p['net_settings']; assert len(settings['classes'])==1 and settings['classes'][0]['name']=='Default'
assert not settings.get('netclass_patterns') and not settings.get('netclass_assignments')
inspect=json.loads((root/'source-inspect.json').read_text())
nets=sorted({json.loads(m) for m in re.findall(r'\(net\s+("(?:[^"\\]|\\.)*")\s*\)',board.read_text())}); assert len(nets)+1==inspect['nets'],(len(nets),inspect['nets']) # importer includes the unnamed net-0 sentinel
q=json.dumps; nm=lambda x:int(Decimal(str(x))*1000000)
r=p['board']['design_settings']['rules']; c=settings['classes'][0]
rows=['(flux_kicad_netclass_snapshot','  (version 3)',f'  (source_revision {q(inspect["source_revision"])})',f'  (project_revision "sha256:{hashlib.sha256(project.read_bytes()).hexdigest()}")','  (api_version "offline-kicad-project-single-default-class")','  (board_rules']
for field,key in [('minimum_clearance_nm','min_clearance'),('minimum_track_width_nm','min_track_width'),('minimum_via_diameter_nm','min_via_diameter'),('minimum_via_annular_width_nm','min_via_annular_width'),('minimum_through_drill_nm','min_through_hole_diameter'),('copper_to_edge_clearance_nm','min_copper_edge_clearance'),('hole_clearance_nm','min_hole_clearance'),('hole_to_hole_clearance_nm','min_hole_to_hole')]:rows.append(f'    ({field} {nm(r[key])})')
rows+=['  )','  (class','    (name "Default")',f'    (source_priority {c["priority"]})']
for field,key in [('clearance_nm','clearance'),('track_width_nm','track_width'),('via_diameter_nm','via_diameter'),('via_drill_nm','via_drill'),('diff_pair_track_width_nm','diff_pair_width'),('diff_pair_gap_nm','diff_pair_gap'),('diff_pair_via_gap_nm','diff_pair_via_gap')]:rows.append(f'    ({field} {nm(c[key])})')
rows+=['  )']
rows += [f'  (net (name {q(n)}) (classes))' for n in nets]; rows+=[')']
(root/'routing.netclasses').write_text('\n'.join(rows)+'\n')
cli=root.parent/'flux-cli'
cmd=[str(cli),'detail-kicad',str(board),str(root/'esc-flux-candidate.kicad_pcb'),'--netclasses',str(root/'routing.netclasses'),'--refillable-zones','--write-incomplete-preview','--curved-tracks','--routing-timeout-ms','3600000','--max-router-iterations','30','--max-expansions','50000','--max-static-seed-workers','2','--search-diversification','--telemetry-level','diagnostic','--telemetry-record',str(root/'routing.jsonl')]
record={'command':cmd,'started_at':datetime.datetime.now(datetime.timezone.utc).isoformat(),'routing_limit_seconds':3600,'outer_limit_seconds':900,'provisional_zone_refill_required':True,'status':'running','git_revision':subprocess.check_output(['git','rev-parse','HEAD'],cwd=root.parents[3],text=True).strip(),'binary_sha256':hashlib.sha256(cli.read_bytes()).hexdigest()}
(root/'run.json').write_text(json.dumps(record,indent=2)+'\n'); start=time.monotonic()
with (root/'result.json').open('w') as out,(root/'stderr.log').open('w') as err:
 proc=subprocess.Popen(['/usr/bin/time','-v','-o',str(root/'resources.txt'),*cmd],stdout=out,stderr=err,start_new_session=True)
 try:
  deadline=time.monotonic()+900; offset=0; rejections=[]
  while proc.poll() is None:
   if time.monotonic()>deadline: raise subprocess.TimeoutExpired(cmd,900)
   telemetry=root/'routing.jsonl'
   if telemetry.exists():
    with telemetry.open() as stream:
     stream.seek(offset)
     while True:
      pos=stream.tell(); line=stream.readline()
      if not line or not line.endswith('\n'): stream.seek(pos); break
      if 'regional-realization-rejection:' in line: rejections.append(json.loads(line))
     offset=stream.tell()
   if len(rejections)>=4:
    (root/'realization-rejections.json').write_text(json.dumps(rejections,indent=2)+'\n')
    record['stop_reason']='four typed regional realization rejections captured'
    os.killpg(proc.pid,signal.SIGTERM)
    break
   time.sleep(2)
  rc=proc.wait(timeout=10)
 except subprocess.TimeoutExpired:
  os.killpg(proc.pid,signal.SIGTERM)
  try:proc.wait(timeout=10)
  except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
  rc=124
record.update(return_code=rc,elapsed_seconds=time.monotonic()-start,status='finished' if rc!=124 else 'diagnostic_timeout')
(root/'run.json').write_text(json.dumps(record,indent=2)+'\n'); print(json.dumps(record))
