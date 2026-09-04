# Feature-parity fixtures

This directory is the source-bound fixture catalog for `docs/parity_manifest.json`. Its manifest
selects the smallest available board that exposes each pinned KiCadRoutingTools capability. A
fixture being cataloged does not mean Flux currently supports it or that it may bypass strict
import. `current_gate` records whether it is qualified, a frontier board, or an intentional
fail-closed import fixture.

Files under `krt/` are byte-identical copies from KiCadRoutingTools v0.21.5 commit
`e239e067c204870b4ac570fc9e931a374bdfd5b4`. The upstream repository distributes them under the MIT
license reproduced in `LICENSE.KiCadRoutingTools.txt`; no separate notice was present beside these
synthetic/reproduction fixtures at the pinned revision. Every copied file retains its upstream
path and SHA-256 in `manifest.json`.

The KiCad 10 fixtures deliberately remain in their original 2026 syntax. Flux derives stable net
identity directly from their quoted name-valued references when no ordinal table exists and emits
the same syntax for new source-preserving copper. An unfilled source zone is retained as an
immutable requires-refill marker rather than discarded or invented as copper, so all nine pinned
boards now pass strict import. Do not normalize, down-convert, or silently edit the fixtures:
remaining capability blockers must stay attributable to actual routing semantics.

Run the catalog and capability-contract check through:

```sh
ctest --test-dir build/dev --output-on-failure -R flux_cli_capabilities_manifest
```

`reference_cases.json` defines the pinned outcome comparisons. Replay every case into a new
write-once artifact directory with:

```sh
python3 tests/python/record_reference_parity.py \
  --reference-checkout /path/to/KiCadRoutingTools \
  --python /path/to/reference-venv/bin/python \
  --flux-cli build/dev/flux-cli \
  --kicad-cli /path/to/kicad-or-kicad.AppImage \
  --fixtures tests/fixtures/parity/manifest.json \
  --cases tests/fixtures/parity/reference_cases.json \
  --artifacts build/reference-comparison-$(date +%Y%m%d-%H%M%S)
```

The runner verifies the upstream commit and both copies of every fixture before execution. It
records reference connectivity, DRC, route scope, and feature behavior beside Flux's current
strict-import result. Copper coordinates are deliberately not compared byte-for-byte.
