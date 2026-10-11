# Tier-1 repair-scope scheduling experiment

Frontier: `bm_svf`. Tier 0 is DRC-clean in two retained baseline runs.
The normal Tier-1 N30/7200-second baseline stopped without a final result during
an interruption; its termination cause is unknown. Earlier 900-second diagnostic
timeouts are not normal-budget failures. Subsequent outcomes are preserved in
the benchmark histories and `docs/router_research_log.md`.

## Small physical interaction

The ordinary native solver repeatedly rejects fragment `kicad-net-3:0:0:1`
against frozen fragment `kicad-net-2:2:0:1` on F.Cu (450000 nm clearance deficit).
The normal baseline reproduced it at every candidate rank 0–7 by approximately
467 seconds. Net names are `Net-(U3A--)` and `Net-(C1-Pad2)` respectively.

Selected global geometry explains the crossing before splines:

- Net 3 travels vertically from (72.34,93.98) to (72.34,92.64) mm on F.Cu,
  then transitions to B.Cu.
- Net 2's branch from R5 runs from (73.91,93.29) to (72.34,93.29) mm, meeting
  its own existing tree exactly on Net 3's vertical segment.
- The source pads nearby are C1.1 (72.136,93.98), C1.2 (71.176,93.98),
  U3.1 (71.12,91.375), U3.2 (72.39,91.375), and R5.1 (73.91,93.98) mm.

These are diagnostic observations, not router constants. Coordinates came from
the selected global telemetry and KiCad's source-board parser. No reference
tracks were copied. A different via location or branch corridor might work;
feasibility is not proved by this inspection.

The native constraint contexts do contain B.Cu alternatives for Net 3 and for
part of Net 2. This is not evidence that Flux lacks layer alternatives entirely.
Several later retained/fallback states put Net 3 back on the original F.Cu
segment while varying Net 2, recreating the same outside-branch conflict.

## Pipeline trace

Global allocation selects crossing planar copper despite complete combinatorial
connectivity. Regional geometry retains a fixed endpoint on the Net-2 tree.
Native certification correctly detects the crossing with an outside fragment.
The wrapper cannot enlarge ownership until conflict-directed and residual-cone
searches return. Thus rejection evidence exists earlier than the authority to
move its blocker. Quantisation is not the first cause of this particular pair.

One further limitation to watch: residual stub expansion only admits adjacent
fragments within the same connection/run. A tree junction shared by different
connections can remain fixed even when one branch is rerouted. Do not change
this yet; first observe whether the existing exact scope expansion resolves the
pair when scheduled earlier.

## Isolated change and acceptance

`--prioritize-frozen-scope-expansion` opts into early expansion after ordinary
native certification. It requires frozen context and scope expansion (both
already enabled by normal full-strength routing). It uses existing witness
validation/deduplication. No remaining retry, duplicate witness, or legal
checkpoint means normal same-scope search continues. The final allowed scope
always retains full local repair. Production default remains unchanged.

Compare exact scope progress, remaining conflicts, runtime and final copper—not
just how soon expansion happens. Tier 0 must remain KiCad-DRC clean before any
default promotion. The initial experimental binary is
`d54d2d9d20a57b8090d9f8a13955ca5196e1102841311a49b87ad82a1752c6a1`.
All 44 selected CTest checks passed (126.51 seconds under concurrent benchmark
load), including core tests and both native export/KiCad DRC fixtures.
