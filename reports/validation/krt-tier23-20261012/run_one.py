"""Sequential KRT oracle run with unchanged source-project validation."""
import collections, datetime, hashlib, json, os, pathlib, shutil, subprocess, sys, time
import pcbnew
root=pathlib.Path(__file__).resolve().parent
repo=root.parents[2]
tier=int(sys.argv[1]); board_name={2:'ft2232h_jtag',3:'gb_nintendo_power'}[tier]
mode=sys.argv[2] if len(sys.argv)>2 else 'project-defaults'
out=root/(f'tier{tier}-matched' if mode=='matched' else f'tier{tier}'); out.mkdir(exist_ok=False)
source=repo/'tests/fixtures/routing-corpus'/f'{board_name}.kicad_pcb'
project=source.with_suffix('.kicad_pro')
input_file=out/f'{board_name}.kicad_pcb'; output=out/f'{board_name}-krt.kicad_pcb'
shutil.copy2(source,input_file); shutil.copy2(project,input_file.with_suffix('.kicad_pro'))
shutil.copy2(project,output.with_suffix('.kicad_pro'))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
cmd=['/usr/bin/python3',str(root/'krt-source/py_router/route.py'),str(input_file),str(output),
 '--escalation','off','--strict-sizes','--no-fix-drc-settings','--fab-tier','standard',
 '--stats','--verbose','--debug-memory','--json-out',str(out/'krt-summary.json')]
if mode=='matched': cmd += ['--track-width','0.25','--clearance','0.2','--via-size','0.6','--via-drill','0.3']
def drc(board,target):
 c=['/usr/bin/kicad-cli','pcb','drc','--format','json','--exit-code-violations','--output',str(target),str(board)]
 p=subprocess.run(c,capture_output=True,text=True,timeout=180)
 target.with_suffix('.log').write_text(p.stdout+p.stderr)
 d=json.loads(target.read_text()) if target.exists() else {}
 return {'returncode':p.returncode,'available':target.exists(),'violations':len(d.get('violations',[])),
 'categories':dict(collections.Counter(v['type'] for v in d.get('violations',[]))),
 'unconnected_items':len(d.get('unconnected_items',[]))}
record={'date':datetime.datetime.now(datetime.timezone.utc).isoformat(),'tier':tier,'board':board_name,
 'dimension_mode':mode,'krt_revision':'023d3f79027d5406e4ea8e68291f588c5c135673','command':cmd,
 'source_sha256':sha(source),'project_sha256':sha(project),'krt_native_sha256':sha(root/'krt-source/rust_router/grid_router.so'),
 'kicad_version':pcbnew.GetBuildVersion(),'route_timeout_seconds':900,'telemetry':'KICAD_ROUTE_TRACE=1; stats; verbose; debug-memory',
 'input_drc':drc(input_file,out/'input-drc.json')}
(out/'run-result.json').write_text(json.dumps(record,indent=2)+'\n')
env=dict(os.environ,KICAD_ROUTE_TRACE='1',PYTHONUNBUFFERED='1')
started=time.monotonic()
with (out/'route.log').open('w') as log:
 p=subprocess.run(['/usr/bin/time','-v','-o',str(out/'resource.txt'),'/usr/bin/timeout','--signal=TERM','--kill-after=15','900',*cmd],env=env,stdout=log,stderr=subprocess.STDOUT,cwd=root/'krt-source')
record.update(returncode=p.returncode,wall_seconds=time.monotonic()-started)
if output.exists():
 output_project=output.with_suffix('.kicad_pro')
 if output_project.exists():
  shutil.copy2(output_project,out/'krt-written-project.json')
  record['written_project_sha256']=sha(output_project)
 # Check against the authoritative original project regardless of KRT's output metadata.
 shutil.copy2(project,output_project)
 record['output_drc']=drc(output,out/'output-drc.json')
 b=pcbnew.LoadBoard(str(output)); items=list(b.GetTracks())
 vias=[x for x in items if isinstance(x,pcbnew.PCB_VIA)]; tracks=[x for x in items if not isinstance(x,pcbnew.PCB_VIA)]
 record['copper']={'tracks_and_arcs':len(tracks),'vias':len(vias),'length_mm':sum(float(x.GetLength()) for x in tracks)/1e6,'sha256':sha(output)}
 p=subprocess.run([str(repo/'reports/validation/preservation-repair-round26-20261012/flux-cli'),'inspect-kicad',str(output)],capture_output=True,text=True,timeout=90)
 (out/'flux-inspect.json').write_text(p.stdout);(out/'flux-inspect.log').write_text(p.stderr)
 record['flux_inspect_returncode']=p.returncode
 with (out/'render.log').open('w') as log:
  p=subprocess.run(['/usr/bin/python3',str(repo/'tests/python/export_copper_pngs.py'),'--board',str(output),'--output-dir',str(out/'copper'),'--kicad-cli','/usr/bin/kicad-cli','--inkscape',shutil.which('inkscape')],stdout=log,stderr=subprocess.STDOUT,timeout=180)
 record['render_returncode']=p.returncode
record['source_unchanged']=sha(source)==record['source_sha256'] and sha(project)==record['project_sha256']
record['telemetry_files']=[str(p.relative_to(out)) for p in out.rglob('*') if p.is_file() and ('trace' in p.name or 'summary' in p.name)]
(out/'run-result.json').write_text(json.dumps(record,indent=2)+'\n')
print(json.dumps(record,indent=2))
