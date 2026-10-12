import datetime, hashlib, json
from pathlib import Path
root = Path(__file__).resolve().parent
repo = root.parents[2]
load = lambda p: json.loads(p.read_text())
source = load(root/'source-build.json')
data = {
 'round':13, 'experiment':'incremental-transactions-obstacle-reuse-span-certificates',
 'recorded_at':datetime.datetime.now(datetime.timezone.utc).isoformat(),
 'git_revision':source['git_revision'],
 'revision_note':'Pre-commit HEAD; frozen source hashes identify tested implementation.',
 'binary_sha256':source['binary_sha256'],
 'frozen_source_manifest_sha256':hashlib.sha256((root/'source-build.json').read_bytes()).hexdigest(),
 'configuration':{'sequential':True,'telemetry':'diagnostic','curved':True,'routing_timeout_seconds':900,'external_kicad_drc':True},
 'diagnosis':load(root/'diagnosis.json'),
 'validation':load(root/'verification.json'),
 'regression_isolation':load(root/'isolation.json'),
 'comparison':load(root/'comparison.json'),
 'copper_lengths':load(root/'copper-lengths.json'),
 'decision':'Keep: both lower tiers pass with around20% lower detailed-command wall time; peak RSS increases around10%.',
 'tier2':'Not rerun; previously recorded failure precedes curved materialization.' ,
 'artifacts':str(root.relative_to(repo))
}
path=repo/'reports/benchmark_history/2026-10-12-incremental-validation-round13.json'
path.write_text(json.dumps(data,indent=2)+'\n')
print(path)
