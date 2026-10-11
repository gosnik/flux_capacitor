import json,math,itertools
from pathlib import Path
c=json.load(open('build/tier-resolution-v59/window25-7.json'))
a=[v for v in c['vias'] if v['region']==4];b=[v for v in c['vias'] if v['region']==64]
def ps(p,a,b):
 d=[b[i]-a[i] for i in (0,1)];l=sum(x*x for x in d);t=max(0,min(1,sum((p[i]-a[i])*d[i] for i in (0,1))/l)) if l else 0
 return math.hypot(*(p[i]-a[i]-t*d[i] for i in (0,1)))
def ss(a,b,c,d):
 def cross(p,q,r):return (q[0]-p[0])*(r[1]-p[1])-(q[1]-p[1])*(r[0]-p[0])
 if cross(a,b,c)*cross(a,b,d)<0 and cross(c,d,a)*cross(c,d,b)<0:return 0
 return min(ps(a,c,d),ps(b,c,d),ps(c,a,b),ps(d,a,b))
def conflicts(x,y):
 bad=[]
 if math.dist(x['site'],y['site'])<900000:bad.append(('disks',round(math.dist(x['site'],y['site']))))
 for v,r in [(x,y),(y,x)]:
  for run in r['runups']:
   d=min(ps(v['site'],p,q) for p,q in zip(run['points'],run['points'][1:]))
   if d<675000:bad.append(('via-run',v['id'],r['id'],run['layer'],round(d)))
 for r in x['runups']:
  for s in y['runups']:
   d=min(ss(p,q,t,u) for p,q in zip(r['points'],r['points'][1:]) for t,u in zip(s['points'],s['points'][1:]))
   if r['layer']==s['layer'] and d<450000:bad.append(('run-run',r['layer'],round(d)))
 return bad
for x in a:
 for y in b:print(x['id'],y['id'],conflicts(x,y))
import copy
def paths(a,b):
 dx,dy=b[0]-a[0],b[1]-a[1];k=min(abs(dx),abs(dy));sx=1 if dx>=0 else -1;sy=1 if dy>=0 else -1
 mids=[[a[0]+sx*k,a[1]+sy*k],[b[0]-sx*k,b[1]-sy*k],[b[0],a[1]],[a[0],b[1]]]
 out=[]
 for m in mids:
  p=[a]+([m] if m!=a and m!=b else [])+[b]
  if p not in out:out.append(p)
 return out
def variants(v):
 for combo in itertools.product(*(paths(r['anchor'],v['site']) for r in v['runups'])):
  w=copy.deepcopy(v)
  for r,p in zip(w['runups'],combo):r['points']=p
  yield w
solutions=[]
for x,y in itertools.product(a,b):
 for xx,yy in itertools.product(variants(x),variants(y)):
  if not conflicts(xx,yy):solutions.append([xx,yy])
print('octilinear compatible pairs',len(solutions))
Path('build/tier-resolution-v59/compatible-runups.json').write_text(json.dumps(solutions,indent=2))
if solutions: print(solutions[0])
