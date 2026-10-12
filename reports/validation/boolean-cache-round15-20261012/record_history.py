"""Record frozen sequential regression measurements and their validation evidence."""
import datetime
import hashlib
import json
from pathlib import Path
root = Path(__file__).resolve().parent
repo = root.parents[2]
load = lambda p: json.loads(p.read_text())
source = load(root / 'source-build.json')
comparison = load(root / 'comparison.json')
data = {
    'round': 15,
    'experiment': 'boolean-ready-geometry-and-reusable-spatial-unions',
    'recorded_at': datetime.datetime.now(datetime.timezone.utc).isoformat(),
    'git_revision': source['git_revision'],
    'revision_note': 'Pre-commit HEAD; frozen source hashes identify tested implementation.',
    'binary_sha256': source['binary_sha256'],
    'frozen_source_manifest_sha256': hashlib.sha256((root / 'source-build.json').read_bytes()).hexdigest(),
    'configuration': {'sequential': True, 'telemetry': 'diagnostic', 'curved': True,
                      'routing_timeout_seconds': 900, 'external_kicad_drc': True},
    'diagnosis': load(root / 'diagnosis.json'),
    'validation': load(root / 'verification.json'),
    'comparison': comparison,
    'copper_lengths': load(root / 'copper-lengths.json'),
    'decision': load(root / 'decision.json'),
    'measurement_note': 'Single runs against Round14; worker timers overlap and are not additive wall-time stages. Desktop viewer remained running. No concurrent builds/tests or other tier runs.',
    'tier2': 'Not rerun; known search frontier failure precedes curved materialization.',
    'artifacts': str(root.relative_to(repo)),
}
path = repo / 'reports/benchmark_history/2026-10-12-boolean-cache-round15.json'
path.write_text(json.dumps(data, indent=2) + '\n')
print(path)
