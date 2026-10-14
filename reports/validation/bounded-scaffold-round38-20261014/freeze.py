"""Freeze the validated CLI and record all implementation inputs before tier runs."""
import datetime
import hashlib
import json
import shutil
import subprocess
import sys
from pathlib import Path

root = Path(__file__).resolve().parent
repo = root.parents[2]

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

manifest = root / "source-build.json"
if "--verify" in sys.argv:
    data = json.loads(manifest.read_text())
    changed = [name for name, value in data["source_sha256"].items()
               if not (repo / name).exists() or digest(repo / name) != value]
    if digest(root / "flux-cli") != data["binary_sha256"]:
        changed.append("frozen flux-cli")
    print(json.dumps({"drift": changed}))
    raise SystemExit(bool(changed))

paths = [repo / "CMakeLists.txt", repo / "tests/CMakeLists.txt"]
for directory in ("src", "include", "apps", "shaders", "tests/unit", "tests/python"):
    paths.extend(p for p in (repo / directory).rglob("*")
                 if p.is_file() and "__pycache__" not in p.parts)
shutil.copy2(repo / "build/release/flux-cli", root / "flux-cli")
data = {
    "recorded_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
    "git_revision": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=repo, text=True).strip(),
    "working_tree": "Bounded scaffold transactions and early physical repair; source hashes are authoritative",
    "source_sha256": {str(p.relative_to(repo)): digest(p) for p in sorted(set(paths))},
    "binary_sha256": digest(root / "flux-cli"),
}
for name in data["source_sha256"]:
    saved = root / "source-snapshot" / name
    saved.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(repo / name, saved)
manifest.write_text(json.dumps(data, indent=2) + "\n")
print(json.dumps({"source_files": len(data["source_sha256"]), "binary_sha256": data["binary_sha256"]}))
