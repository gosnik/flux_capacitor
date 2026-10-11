# Straight regional routing experiments V49–V52

Tier 0 remains qualified. Tier 1 remains unqualified; its V52 run completed in 1,698.736 seconds without qualifying. Tiers 2–4 retain their clean KRT replacement inputs and are still locked behind Tier 1.

## Changes

- V49 added an integer H/V/45 realization of selected regional lanes, terminal access and via access. Existing PH/spline code remains available. Whole-scene native copper and exact quantized copper gate acceptance.
- V50 removes redundant same-site layer-return adapters introduced while stitching replacement and source runs. Complete connectivity remains mandatory; an intermediate-layer branch cannot silently lose its via.
- V51 compares the allocator’s retained, already-packed source-topology/source-via fallback against its preferred discrete state using exact physical results. At most one extra scene is evaluated.
- V52 preserves existing same-net tree-attachment vertices during collinear fragment compression. Detours and layer alternatives retain those electrical boundaries.

## Evidence

| Matched first Tier-1 allocation | Exact DRC reports | Opens | Vias |
|---|---:|---:|---:|
| Source global allocation | 560 | 0 | 48 |
| V50 preferred straight geometry | 796 | 2 | 94 |

The V50 scene was reconstructed from emitted geometry and matched all 490 source fragments and the production oracle exactly. Both opens were traced to tree contacts erased during compression: net 21 at (66.49,56.89 mm), F.Cu, displaced 0.65 mm; net 2 at (72.34,95.89 mm), F.Cu, displaced 1.066 mm and moved to another layer. Original vias on these nets were retained.

A separate exact-witness-only replay has 15 windows, unlike the production allocation’s 44; it is an isolated diagnostic and cannot qualify a tier. V51 improved that comparison from 953 to 892 DRC reports, 1 to 0 opens, and 70 to 60 planned via transitions.

V49 completed its 256-state bounded Tier-1 run in 877.552 seconds but did not qualify. V50 was deliberately stopped after six rejected exact scenes to test the measured junction correction; see its `intentional-stop.json`. It is not reported as a completed full-tier run or a timeout.

## Verification

- Release core: 48/48, including existing spline tests and new straight-lane, adapter, and tree-junction fixtures.
- Tier 0: 50 paths, 243 ordinary-angle tracks, 5 vias, zero internal opens and original-project KiCad DRC errors.
- Corpus rule/angle Python checks: 9/9.
- Broader CLI regression result is recorded in the experiment JSON and verification logs. Seven known baseline demo failures remain separately identified.

## Limits and next decision

Only final copper accepted by KiCad DRC can promote Tier 1. The current straight constructor still uses a deterministic diagonal-first elbow, and a feasible discrete lane order may produce illegal copper. The tree fix preserves recorded vertices; an attachment strictly inside an unsplit source edge is a separate remaining case. The completed run checked 12 regional scenes; the first portfolio improved to 644 internal reports with zero opens. A diagnostic pair sweep reduced provisional reports to 419, but final KiCad verification still rejected the result with 209 errors and 9 unconnected items. That sweep is not promoted.

Source/project rules and KRT input provenance are unchanged. Per-experiment snapshots and focused patches preserve the pre-existing dirty working tree.

## Completed benchmark and result selection

See `V52-completed-tier1.json` for all 12 exact scene results. Final V52 selection incorrectly retained an earlier deferred regional assessment, whose empty assignment list looked better than examined failures. V54 adds explicit evaluation state and a regression that reproduces the bug before the fix. This changes result retention, not copper legality. The V52 benchmark is not relabeled as a V54 run.

V54 verification: 48/48 core tests pass; the new selection assertion failed on the prior executable. Broader tests remain 126/133 with the same seven baseline demo failures. The final Tier-0 export still has 243 ordinary-angle tracks, 5 vias, zero opens and zero KiCad errors; its PCB hash matches V46. Tier 1 has not been rerun after this selection-only correction.
