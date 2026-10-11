# Full-strength first-pass experiment

## Request and scope

The user requested the routing/repair "kitchen sink" in the first planning pass,
not increased CPU/GPU parallelism. This is deliberately a bundled experiment,
not a causal attribution to any one algorithm. It is opt-in and does not promote
a tier or change default routing.

## Activation

`route-kicad` / `detail-kicad --full-strength-first-pass`:

- Front-load the primary global iteration budget: with N iterations and R retries,
  the first epoch gets N-R and each retry gets one. Early legal success may finish
  without consuming the entire budget. The core rejects N <= R for this mode.
- Enable seed/order/beam diversification, later-ring terminal alternatives,
  multi-source Steiner growth, bidirectional growth from iteration one, weighted
  A* frontier reuse (at least 1.25 weight), and the existing bounded LNS operators.
- Begin hard capacity on iteration one. Enable at least 8 same-pass rip-up
  attempts, 4 victims per attempt, 64 joint victim-set states, rollback of failed
  attempts, deferred victims, protection of recovered nets, selective whole-net
  rerouting, periodic full refresh, and bounded blocker invalidation.
- Enable USPED-guided NavMesh refinement; retain ordinary RRT/hydraulic/static
  seed discovery. Shared-medial lane conflict negotiation is enabled when that
  topology/resource backend is selected, without silently replacing the backend.
- Offer up to two sampled-conflict cone repairs, using that existing mechanism's
  separate local budgets (up to 12 iterations per cone, bounded by the first
  epoch's iteration count). These are additional repair work, not hidden inside
  the primary PathFinder iteration total; existing cone counters remain visible.
- Give first-epoch regional search full configured portfolio, template, lane,
  terminal, layer/via, joint repair, spline, XPBD, native-copper and feedback
  budgets rather than the cheap intermediate-probe limits.
- Let PH/shove try colliding straight scaffolds. Only the preliminary scaffold
  rejection is bypassed: ownership, topology, native-scene, quantization and final
  DRC gates still decide acceptance.
- On detailed routing, enable arc refinement and applicable differential-pair
  skew/batch tuning, retaining the existing zone-refill safety conditions.

Not enabled blindly: mutually exclusive graph backends, unspecified neck-down
widths, external learned models without a supplied model, placement changes,
source-rule relaxation, lossy export, or increased worker counts. Existing stage
triggers still apply; enabling a repair does not force it on already legal copper.

## Validation and experiment

The two-pad regression checks six first-epoch iterations from N=8/R=2, full
regional realization, activated global repair controls, USPED and exact detailed
completion. A budget with no initial iteration left is rejected.

The Tier-0 experiment uses N=8/R=2, explicit enforced shadows, 50,000 expansions,
two static-seed workers and the existing 600-second planning limit. Higher runtime
is expected; success still requires complete physical copper and final KiCad DRC.

## First integration failure

The first trial stopped after 118.886 seconds with invalid input rather than a
routing outcome. Sampled-cone repair disabled selective rerouting but retained
`max_hard_capacity_blocker_reuse_invalidations_per_failed_net`, whose prerequisite
had just been disabled. Its invalid empty result could then win the ordinary
checkpoint comparator and erase the incumbent. Zero paths/conflicts in this
result are not evidence of successful routing.

Fix: clear the dependent option in cone repair and reject invalid/cancelled repair
results before comparing them with the incumbent. The crossing regression now
requires actual cone-repair attempts and a nonempty, non-invalid returned route.
Preserved result: `reports/benchmark_history/2026-09-05-full-strength-invalid-config.json`.

## Corrected trial

Executable SHA-256: `f256dd3ac2d657831b944fd8919b42cdc0f53f70c02587fead90c68ef774b099`.
The full core suite passed in 146.36 seconds; nine native-arc/multilayer
export/DRC checks also passed (172.67 seconds combined).

The corrected Tier-0 run timed out at 600 seconds without final plan JSON or
accepted copper, so final KiCad DRC was not evaluated. Telemetry records:

- 18 PathFinder iterations completed: six primary plus two six-iteration cone
  searches. This distinguishes primary scheduling budget from repair work.
- First full epoch completed at 511.083 seconds; next final result was not
  published before the timeout. Later work's exact timeout location is unknown.
- First-epoch provisional oracle: 32 violations, zero opens.
- Regional analysis: four windows, three complete bundles; four joint-repair
  windows reported complete at the discrete stage; three conflict-directed cores,
  three master reruns, 18 generated columns, two conflict-directed recoveries.
- 228 joint spline states generated, 30 active, 198 reserve; four joint spline
  candidates evaluated. Native assembled spline candidate remained unavailable.
- One confined-anisotropic erosion success from 14 attempts. This is diagnostic
  routing evidence, not final copper qualification.

Interpretation: the expanded algorithm combination executes and exposed a real
integration fault now covered by regression. It did not solve Tier 0 within the
existing budget. More enabled methods do not establish physical completion.
Retain the preset as disabled-by-default experimental configuration, keep the
configuration/invalid-incumbent fix, and do not promote any tier.

Structured history: `reports/benchmark_history/2026-09-05-full-strength-first-pass.json`.
Raw diagnostics and non-authoritative previews are under
`build/full-strength-first-pass-fixed-tier0/`.
