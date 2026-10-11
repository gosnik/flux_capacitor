# Tier resolution — 2026-09-07

The retained implementation preserves spline routing and makes the ordinary straight export use horizontal, vertical and 45-degree sections. Tier 0 is accepted; Tier 1 remains the frontier. Source preflight does not promote a board.

| Tier | Active board | Verified state |
| --- | --- | --- |
| 0 | PT2399 delay | 50 connections; 243 ordinary-angle tracks; zero arcs; five vias; zero internal or original-project KiCad errors/opens |
| 1 | BM SVF | Full Release run finishes in 1,115.174 s; 78 paths and 44/44 lane assignments, but 560 exact copper violations and no accepted physical replacement |
| 2 | FT2232H JTAG/UART | Clean source preflight; locked |
| 3 | Nintendo Power cartridge | Clean source preflight; locked |
| 4 | Nascom EPROM/80bus | Clean source preflight; locked |

The replaced levels 2–4 remain in `tests/fixtures/routing-corpus/compatibility/` with their original source-error expectations. Active preflight passes 5/5; compatibility preflight passes 3/3.

## Experiments

- V42: 0.65 mm sampled control stagnated; 0.25 mm Debug control reached only three iterations before timeout. No density override promoted.
- V43: blanket single-lane sampled capacity regressed the WorkPlan differential-pair transaction. Reverted; rejected source snapshot retained.
- V44: 15 outer feedback epochs within the same 30 inner iterations did not converge. The 14 completed intermediate oracle counts were 733, 520, 807, 738, 554, 712, 571, 687, 791, 798, 599, 832, 808 and 901; timeout at 900 seconds. This run used the rejected V43 base.
- V45: clearance-qualified soft-cut attribution excludes 117 legal proximity observations and preserves actual blockers. Focused fixture plus 44/44 core tests passed. Tier-1 copper counts remained 733/560/614/633 and timed out at 600 seconds. Retained as an attribution fix, not a tier-completion claim.
- V46: converted off-angle generated copper through bounded, exact-clearance-checked elbows. Preserved exact branch/via contacts and existing source copper. Export audit independently rejects arbitrary angles and arcs in straight runs. Tier 0 passes with 498.861562 mm track length. All 45 core tests and nine Python checks pass; broader CTest retains its seven baseline demo failures (126/133 selected checks pass).
- V47: optimized build reproduces the same four Tier-1 oracle counts approximately five times faster. Tier 0 and all 45 Release core tests pass. Full frontier run remains unqualified after 1,115.174 s. Dense 0.25 mm control times out at 900 s after ten iterations, with six sampled and 231 shadow conflicts in its last completed iteration. The redundant Debug control was explicitly cancelled after Release reproduced its checkpoints.

- V48: the 256-state regional control finishes in 617.164 s with the same selected 44/44 bundle completion and 560 copper violations. Selected regional search states fall from 371,615 to 2,529 and generated candidates from 205,287 to 1,609. This bound is an exploratory control, not a promoted default.

The completed frontier result reports 436 track-clearance, 105 track-via-clearance and 19 hole-clearance violations in the exact internal copper probe. No final Tier-1 candidate passed the gate for external KiCad acceptance. The next experiment is a separate piecewise-linear realization of selected regional lane/template/via assignments; assignment flags must never substitute for a full copper proof.

`summary.json` retains executable hashes, configurations, outcomes, global/checkpoint metrics, physical-feedback histories, final angle audit and external DRC results. `frontier-diagnosis.json` preserves example pairwise physical witnesses: nets 4 and 5 (`Net-(U4B--)`, `Net-(C2-Pad2)`) conflict around x=50.89–53.49 mm, y=82.565–93.2465 mm on F.Cu/B.Cu. Telemetry witnesses are coalesced/accumulated observations, not unique collision counts. Final KiCad DRC is unavailable for unqualified Tier-1 planning runs.

Detailed hypotheses, primary research references and keep/revert decisions are in `docs/router_research_log.md`. Existing unrelated work was preserved; focused retained experiment diffs are in `patches/`; turn-specific source snapshots are under `build/tier-resolution-v43-baseline`, `v43-rejected`, `v45-baseline` and `v46-baseline`.

Latest continuation: [straight regional realization V49–V54](straight-realization.md), [completed V52 Tier-1 benchmark](V52-completed-tier1.json), and [final V54 verification](V54-verification.json). Tier 0 remains qualified; Tier 1 remains the frontier.

Subsequent continuation: [V55–V58 canonical-geometry corrections and verification](../2026-09-08-tier-resolution/README.md). Historical Tier-1 internal connectivity metrics predate the corrected back-side pad import; compare them with the independent KiCad evidence in that report.
