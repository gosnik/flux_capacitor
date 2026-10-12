import datetime, hashlib, json
from pathlib import Path
root = Path(__file__).resolve().parent
repo = root.parents[2]
load = lambda p: json.loads(p.read_text())
source = load(root/'source-build.json')
experiment = root.with_name('progressive-attachments-round12a-20261012')
data = {
 'round':12, 'experiment':'progressive-endpoint-attachment-generation',
 'recorded_at':datetime.datetime.now(datetime.timezone.utc).isoformat(),
 'git_revision':source['git_revision'],
 'revision_note':'Pre-commit HEAD; frozen source hashes identify tested implementation.',
 'binary_sha256':source['binary_sha256'],
 'frozen_source_manifest_sha256':hashlib.sha256((root/'source-build.json').read_bytes()).hexdigest(),
 'configuration':{'sequential':True,'telemetry':'diagnostic','curved':True,'routing_timeout_seconds':900,'external_kicad_drc':True,'progressive_default':False},
 'diagnosis':load(root/'diagnosis.json'),
 'validation':load(root/'validation.json'),
 'progressive_experiment':load(experiment/'experiment-summary.json'),
 'comparison':load(root/'comparison.json'),
 'decision':'Keep progressive mode opt-in after lower-tier regression; retain default eager search, shared rejection deduplication and telemetry.',
 'tier2':'Deferred because progressive experiment regressed Tier1; no claim of improvement.',
 'artifacts':{'final':str(root.relative_to(repo)),'progressive_trial':str(experiment.relative_to(repo))}
}
path=repo/'reports/benchmark_history/2026-10-12-progressive-attachments-round12.json'
path.write_text(json.dumps(data,indent=2)+'\n')
print(path)
