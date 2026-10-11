"""Read only newly appended telemetry so monitoring does not repeatedly parse large scenes."""
from pathlib import Path
import json, sys
root = Path(__file__).resolve().parent
folder = root / sys.argv[1]
cache_path = root / f'{sys.argv[1]}-progress-cache.json'
try:
    cache = json.loads(cache_path.read_text())
except (OSError, ValueError):
    cache = {}
for path in sorted((folder / 'telemetry').glob('*.jsonl')):
    stat = path.stat()
    saved = cache.get(path.name, {})
    if saved.get('inode') != stat.st_ino or saved.get('offset', 0) > stat.st_size:
        saved = {}
    last, engine = saved.get('last', {}), saved.get('engine', {})
    with path.open('rb') as stream:
        stream.seek(saved.get('offset', 0))
        offset = stream.tell()
        while line := stream.readline():
            if not line.endswith(b'\n'):
                break
            offset = stream.tell()
            try:
                event = json.loads(line)
            except (ValueError, TypeError):
                continue
            if event.get('message') or event.get('progress'):
                last = {key:event[key] for key in ('sequence', 'progress', 'message') if key in event}
            if str(event.get('active_group', '')).startswith('engine-action:'):
                engine = last
    cache[path.name] = {'inode':stat.st_ino, 'offset':offset, 'last':last, 'engine':engine}
    print(path.name, json.dumps({"engine": engine, "latest": last}))
folder.mkdir(parents=True, exist_ok=True)
cache_path.write_text(json.dumps(cache))
