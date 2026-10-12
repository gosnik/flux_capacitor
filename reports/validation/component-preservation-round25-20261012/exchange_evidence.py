import json
from pathlib import Path
from urllib.parse import unquote
root=Path(__file__).resolve().parent
baseline=root.parent/'branch-rebind-round24-20261012'
r=json.loads(next((baseline/'tier2').glob('tier-*/global-result.json')).read_text())['result']['copper_scaffold']
identities={e['request_identity']:i for i,e in enumerate(r['request_evidence'])}
current=json.loads(next((root/'tier2').glob('tier-*/global-result.json')).read_text())['result']['copper_scaffold']
identities.update({e['request_identity']:i for i,e in enumerate(current['request_evidence'])})
original=json.loads(next((root.parent/'grid-contact-round21-20261012'/'tier2').glob('tier-*/global-result.json')).read_text())['result']['copper_scaffold']
identities.update({e['request_identity']:i for i,e in enumerate(original['request_evidence'])})
def decode(value):
    count,value=value.split(':',1)
    result=[]
    for _ in range(int(count)):
        length,value=value.split(':',1);length=int(length)
        result.append(unquote(value[:length]));value=value[length:]
    assert not value
    return result
out={'passes':[],'rollbacks':[],'missing':[]}
for line in next((root/'tier2/telemetry').glob('*.jsonl')).open():
    event=json.loads(line);message=event.get('message','')
    fields=dict(item.split('=',1) for item in message.split('; ') if '=' in item)
    seconds=event.get('monotonic_time_us',0)/1e6
    if 'rollback_incoming' in fields:out['passes'].append({'seconds':seconds,'message':message})
    if 'blocker_ids' not in fields:continue
    if 'incoming' in fields:
        out['rollbacks'].append({'seconds':seconds,'incoming':identities.get(decode(fields['incoming'])[0]),'victims':[identities.get(x) for x in decode(fields['victims'])],'reason':fields['reason'],'disconnected_requests':[identities.get(x) for x in decode(fields.get('disconnected_requests','0:'))],'pad_components_split':fields.get('pad_components_split')=='true','blocker_ids':decode(fields['blocker_ids'])})
    elif 'request' in fields:
        out['missing'].append({'seconds':seconds,'request':int(fields['request']),'termination':fields['termination'],'blocker_ids':decode(fields['blocker_ids'])})
(root/'exchange-evidence.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps({'passes':len(out['passes']),'rollbacks':len(out['rollbacks']),'missing':len(out['missing'])}))
