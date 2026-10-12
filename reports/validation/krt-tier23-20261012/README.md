# KRT versus Flux — Tier 2 and Tier 3, 2026-10-12

All runs were sequential and recorded. Neither router completed either board. Connectivity below was independently checked on exported copper with KiCad 10.0.6 and Flux inspection, against the original project rules.

| Board / router | Routing wall time | Physical opens | Tracks | Vias | Additional routing DRC |
|---|---:|---:|---:|---:|---|
| Tier2 Flux Round27 | 309.40s | 1 | 1322 | 74 | 21 dangling tracks |
| Tier2 KRT matched | 14.77s | 10 | 528 | 36 | 1 dangling track, 6 dangling vias |
| Tier3 Flux Round27 | 140.90s | 97 | 279 | 20 | none beyond opens |
| Tier3 KRT matched | 162.17s | 83 | 939 | 143 | 44 dangling vias |

Flux artifacts are **provisional straight scaffolds** from curved-mode runs which did not reach final materialization. KRT exported straight copper. No row qualifies for promotion. The times measure the routing command, including its internal pipeline; independent post-run validation and rendering are excluded. Flux's separate successful planning/detail runs on lower tiers are not included in these frontier times.

The matched KRT runs explicitly request0.25mm tracks,0.20mm copper clearance,0.60mm via diameter and0.30mm drill, matching Flux's observed scaffold dimensions. KRT still honors project classes and fabrication constraints. Its standard fabrication floor raises Tier2 edge clearance from the source0mm to0.2mm, so this is not perfect algorithm-only equivalence. Source project floors were never relaxed. The original project was restored next to each exported KRT board before independent KiCad DRC.

## Retained project-default controls

KRT's project-default Tier2 run used0.25mm tracks and0.80/0.40mm vias:15.84s,11 opens,518tracks,35vias,603.88mm copper. It added1 dangling-track and7 dangling-via warnings.

KRT's project-default Tier3 run took165.99s with73 opens,1203tracks,150vias and1506.74mm copper. It returned3 under `--strict-sizes`: two rescue features were below requested sizes despite `--escalation off`. KiCad also found2 hole-clearance violations,1 dangling track and34 dangling vias. Its lower open count must not be mistaken for a valid solution. The matched run had no reported size escalation or clearance/hole violations, but83 opens.

Tier2's41 library-footprint warnings and Tier3's22 library-footprint/6silkscreen warnings also exist on their unrouted inputs; they are excluded from the table's additional routing DRC column. Opens are a separate DRC category.

## What the comparison establishes

* Flux's new per-net residual admission reduces Tier2 from14 physical opens to1, independently confirmed by KiCad. KRT remains faster but leaves10 opens across9 nets. Both fail `/TCK_TXD_A`; KRT therefore supplies no successful oracle topology for Flux's last connection.
* Tier3 favors KRT in completed physical connectivity at this budget, with83 opens versus97. Both need better fine-pitch fanout and coordinated repair; neither result establishes an adequate end-to-end implementation.
* Flux Tier2 engine time is dominated by scaffold work192.69s, followed by global48.71s and USPED48.43s; regional work8.20s. Tier3 is dominated by regional work88.41s, versus38.43s scaffold and12.39s global. These are measured engine action times, not inferred GPU or CPU utilization.
* KRT logs repeatedly report boxed-in searches and failed rescues without rippable blockers. Its printed main-loop times and merged `routed` tally describe narrower internal scopes; they are not whole-board completion or total wall-time measures. Use exported geometry and `run-result.json` for comparisons.

## Reproduction and artifacts

`run_one.py 2` / `run_one.py 3` reproduce the project-default controls; add `matched` for the explicit-dimension controls. It refuses to overwrite an existing run directory. Commands, source hashes, native binary hash, wall time, peak memory logs, route trace, merged KRT summary, input/output KiCad DRC and independent Flux inspection are retained with each run. Matched per-layer images are under `tier2-matched/copper/` and `tier3-matched/copper/`.

KRT source is a clean snapshot of committed revision `023d3f79027d5406e4ea8e68291f588c5c135673` from the local installation. The installation's uncommitted QFN patch was deliberately excluded and left untouched. Its installed native `grid_router.so` was copied and hashed. `source-manifest.json` records the exact Python/native inputs. A new checkout of that revision plus the recorded native build is required to regenerate the untracked source snapshot; the runner does not download or modify the installed checkout.

Flux source/build/results are linked by `../residual-frontier-round27-20261012/source-build.json`, `summary.json`, `export-verification.json`, and the `codex/residual-frontier-round-27` tag. Its provisional copper and KiCad reports are in the corresponding `tier2/diagnostic-scaffold/` and `tier3/diagnostic-scaffold/` folders.

## Subsequent Flux progress (Round32, 2026-10-13)

The comparison above retains its original Round27 measurements. Round31's exhausted-pocket blocker diagnosis subsequently closed Tier2's last physical connection. Round32 retained curved materialization:131/131 scaffold requests, zero opens,889 tracks,163 arcs,421 Beziers and75 vias. Global command166.07s; separate detailed command225.25s (which repeats planning), so these are not additive stages within one routing call. Diagnostic KiCad error-severity DRC found zero errors/opens. One unresolved non-junction corner still prevents Tier2 promotion. Tier0/1 remained byte-identical and clean. See `../final-drc-parity-round32-20261013/summary.json`. Tier3 has not been rerun since the original comparison.

## Current comparison after Tier2 promotion (Round33)

| Board/router | Routing command wall time | Physical opens | Outcome |
|---|---:|---:|---|
| Tier2 Flux Round33 |224.17s detailed end-to-end|0|Pass:421 Beziers,164 arcs,75vias; zero unrounded bends and KiCad DRC errors|
| Tier2 KRT matched |14.77s|10|Incomplete|
| Tier3 Flux Round33 |140.71s global/scaffold attempt|97|Incomplete; no final materialization|
| Tier3 KRT matched |162.17s|83|Incomplete|

Flux's additional standalone Tier2 global command took161.16s; it is a separate repeated benchmark, not additional time inside the224.17s detailed command. Tier0/1 stayed byte-identical and DRC-clean. Configuration caveats from the original matched comparison still apply. `../residual-corner-round33-20261013/` holds the new evidence and final Tier2 board.
