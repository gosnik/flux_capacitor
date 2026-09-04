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

The KiCad 10 fixtures deliberately remain in their original 2026 syntax. Flux currently rejects
their string-valued net references with attributed `unknown_net_ordinal` errors. Do not normalize,
down-convert, or silently edit them to make a test pass: strict KiCad 10 import is Milestone 2 work.

Run the catalog and capability-contract check through:

```sh
ctest --test-dir build/dev --output-on-failure -R flux_cli_capabilities_manifest
```
