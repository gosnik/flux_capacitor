# Selected-allocation quantization replay

Build `flux-replay-quantized-copper` explicitly, then run:

```sh
build/release/flux-replay-quantized-copper SOURCE.kicad_pcb TELEMETRY.jsonl SNAPSHOT_SEQUENCE
```

The exact numbered snapshot must contain active, selected `global-route:`
polylines. Retained route-portfolio alternatives are excluded. The tool checks
the original PCB revision, duplicate identities and resolved trace widths, then
compares the same allocation at 100 micrometres, 10 micrometres and 1 nanometre.
It reports internal DRC categories, opens, generated primitives and timings.
The five largest net-pair/layer groups count coalesced diagnoses, not raw DRC
reports. Each includes a bounded witness and, where available, original source
connection/segment identities for tracing the conflict back to allocation.

This is sampled-allocation diagnostic evidence, not a routing result or export.
It uses the source board's **current** sibling project rules; compare the original
run's recorded project hash before interpreting it as a matched historical replay.
The PCB revision alone does not prove equality of project rules or rule overlays.
It writes no board files and cannot qualify a tier. Exit 1 rejects malformed or
missing snapshot/route inputs. Exit 0 means the comparison ran, not that copper
passed DRC.

Optionally append `NET_A NET_B` to compare complete retained **whole-net** seed
slots for two selected nets, with all other selected paths fixed. This mode
rejects missing nets, omits partial/duplicate connection sets and caps portfolios
at 32 slots per net. It does not mix individual tree branches across seed slots.
Each pair reports total copper errors, pair diagnoses and opens, so a lower DRC
count cannot silently disguise lost connectivity. This is an allocation oracle
experiment only: it does not reallocate resource metadata, fit curves or export.

Append `--detail-preview OUTPUT.kicad_pcb` instead of net IDs to replay conventional
straight detailed materialization (including its transactional pull-tight/fallback)
and export its candidate for external DRC. The output is explicitly diagnostic:
telemetry does not retain allocation certificates, so the tool marks only a private
copy complete to enter materialization. It never qualifies the original plan or a
tier. Source validation and strict export round-trip remain enforced, existing output
files are refused, and opens/internal DRC are reported even for a written preview.
Stage byte-identical sibling project/custom-rule files before external KiCad DRC.

Append `--regional-straight MAX_STATES` (1..4096) to test straight regional
realization on the selected snapshot, rebuilding scope from exact copper
witnesses. This omits unavailable global resource/corridor certificates and
disables automatic scope expansion, so its windows may differ from the original
run. It prints scene DRC categories, opens, via counts and bounded physical
witnesses; zero scene-check count means assembly never reached the oracle.
It does not export copper or qualify a tier. Exit 0 means the diagnostic ran.
