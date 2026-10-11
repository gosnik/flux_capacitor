# KRT alternatives for Tiers 2–4 — 2026-09-07

Screened **99 candidates** from the KiCadRoutingTools stress catalogs at revision
`ec2bf0e1021c0ed95c45b14664d02c56777a0974`. Selected and staged three independently
licensed, pinned two-layer inputs. This is source qualification; no routing
benchmark, routing performance result or tier promotion is claimed.

| Proposed tier | Board | KRT set | Footprints | Routable nets | Connections | Core errors | Prepared DRC errors excluding opens |
| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: |
| 2 | FT2232H JTAG/UART | set5 | 41 | 34 | 117 | 0 | 0 |
| 3 | Nintendo Power cartridge | set24 | 22 | 65 | 148 | 0 | 0 |
| 4 | Nascom/Gemini EPROM card | set23 | 79 | 116 | 415 | 0 | 0 |

All three have strict, lossless imports with zero import errors and preserve
original projects, placements, pad geometry and rule values. Source projects
contain no DRC exclusions. Expected unconnected items are 117, 148 and 415 and
match Flux's connectivity measurements. Tracks/arcs/vias/copper zones are zero.

## Why this progression

Tier 1 currently requires 78 connections across 25 routable nets. FT2232H increases
this to 117 with a compact 64-pin package. Nintendo Power increases net count to
65 and includes an 80-pad IC and parallel memory buses. Nascom raises demand to
415 connections across repeated DIP sockets and a card-edge bus on a larger board.
Connection and net counts justify a provisional size progression; these do not
prove monotonically increasing routing difficulty. Nascom is a substantial step
and may merit an additional intermediate tier once actual routing is measured.

Original routed references contain respectively 311 / 965 / 1,300 track segments,
44 / 143 / 199 vias, and approximately 594 / 2,952 / 14,869 mm of track. These are
upstream reference statistics, not Flux output or performance targets.

## Reference caveat

FT2232H and Nascom references pass error-level KiCad DRC with zero opens.
Nintendo Power's reference has one starved thermal, two copper-edge violations
and one open connection. The violations concern routed copper stripped from the
input; the prepared input passes its physical source checks. The reference is
retained as evidence with its defects recorded, and must not be called a clean
routing oracle. Final Flux output must meet zero-error/zero-open gates regardless.

## Rejected candidates and screen correction

An early screen counted core structural errors without also requiring strict
import. The existing corpus runner correctly rejected provisional PicoFX and
GB-CART256K inputs. PicoFX has 40 invalid-pad-drill import errors and two pad
approximations; GB-CART256K has four pad approximations. Both were excluded and
the shortlist was rebuilt using the full gate. This correction is recorded to
prevent reuse of a core-error-only screen.

Other examples: the DMX board requires pad approximations and has no discovered
board license; BMS sensor has a source copper-edge violation; EIT mux has drill
and solder-mask defects. The SATA backplane has no discovered repository license
and its reference has two intersecting-zone errors. Rotten pedal is clean but
only requires 66 connections, below Tier 1's 78, so it is a useful side regression
rather than a stronger tier. Some upstream branches now contain KiCad 10 boards
that this KiCad 9 environment cannot load. All 99 outcomes, source hashes, import
categories, observed DRC categories and screening durations are in `summary.json`.
Exploratory rejected downloads were branch tips; selected files were downloaded
again from exact commits and required to match their screened PCB/project bytes.

## Artifacts and validation

- [Staged candidate corpus](../../../tests/fixtures/routing-corpus/alternatives/README.md): prepared boards, original sources, normalized references, identical projects, licenses and manifest hashes.
- [Structured summary](summary.json): candidate inventory, source/rule evidence, selection and rejected screens.
- `*.inspect.json`, `*.geometry.json`, `*.prep.log`: selected input and preparation measurements. Early provisional candidates' logs are also retained as rejected-screen evidence.
- `*.prepared.drc.json` and `*.reference.drc.json`: raw external DRC reports.
- `staged-drc-validation.json`: DRC commands, elapsed times and input/project/report hashes for the final staged copies.
- `alternatives-run-result.json`: **3/3 strict corpus preflights passed**.
- `active_corpus-run-result.json`: **5/5 existing source preflights passed under their existing manifest allowances**. This does not remove the old Tier 2–4 blockers or claim routing passes.

The active ladder and all routing code remain unchanged by this search. Candidates
are stored in a separate all-locked manifest. No tests for routing algorithms were
added or run because this task only found and staged fixtures. Exact source
provenance, attribution, licenses and preparation steps are documented in the
candidate README and manifest.
