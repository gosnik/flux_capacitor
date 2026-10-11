# Vulkan XPBD Routing Relaxation Report

Date: 2026-09-03

## Implementation

Flux now has an experimental PCB-specific joint relaxation stage behind
`RegionalRouterConfig::enable_xpbd_relaxation`.

The public compute boundary accepts fixed pad/via endpoints, movable control particles,
route segments with width and clearance, route-specific static obstacle segments, corridor
anchors, and an optional routing-pressure field. The Vulkan backend keeps two position buffers
resident on the device and records the complete iteration sequence in one command buffer. Each
particle invocation applies compliant stretch, route-Laplacian bend, segment/obstacle,
segment/segment, corridor, and terrain-gradient corrections. CPU readback occurs only after all
iterations.

The existing Vulkan segment-conflict workload supplies the conservative inter-route broadphase.
The regional router extracts route-specific foreign copper and keepout boundaries from the
canonical board, so same-net terminal copper is not accidentally treated as an obstacle.

Regional acceptance remains transactional:

```text
selected lane/corridor/via state
        -> initial quintic PH interpolation nodes
        -> joint Vulkan XPBD relaxation
        -> adaptive whole-window topology and clearance assessment
        -> retain only a strict feasibility-first improvement
        -> rebuild PH Bezier segments / quantization feedback
        -> exact connectivity and copper DRC
```

An XPBD proposal cannot change layers or silently cross a selected NavMesh gate. Particle output
moves PH interpolation nodes, never arbitrary Bezier controls; every proposal must successfully
rebuild the Pythagorean hodograph before assessment. Rejected deformations leave the original
candidate unchanged.

## Physical feedback

Unresolved per-particle constraint stress is deposited into a bounded per-window pressure raster
and diffused into adjacent cells. The highest bounded pressure cells become ordinary typed
`shove_deadlock` routability diagnoses. The existing routability-knowledge reducer converts these
diagnoses into spatial PathFinder history for the next outer routing pass; this state remains
non-authoritative and cannot bypass exact routing gates.

Normal telemetry reports one semantic event per regional invocation, including particle, segment,
obstacle, candidate-pair and iteration counts, final violation count, accepted state, and selected
backend. CLI JSON exposes aggregate attempt, improvement, legalization, GPU/fallback, topology
rejection, obstacle-budget, pressure-cell, and peak-pressure metrics.

## Controls

```text
--xpbd-relaxation
--xpbd-feedback
--no-xpbd-relaxation
--no-xpbd-feedback
--xpbd-iterations N
--xpbd-max-step-nm NM
--xpbd-cpu
```

Relaxation and feedback are enabled by default. `--no-xpbd-relaxation` disables both;
`--no-xpbd-feedback` retains relaxation without exporting unresolved pressure into routability
history. `--xpbd-feedback` also enables relaxation, and `--xpbd-cpu` selects the deterministic
reference implementation. Every proposal remains non-authoritative until the unchanged topology,
continuous-envelope, quantized-copper, connectivity, and DRC gates accept it.

## Verification

| Case | Baseline | XPBD result |
|---|---:|---:|
| Two interacting spline-control chains | coincident interior controls | separated; terminal controls fixed |
| Finite static obstacle crossing | positive segment/obstacle residual | residual reduced |
| Pressure raster | absent | bounded 32 x 32 field produced and diffused |
| Vulkan execution | n/a | complete iteration sequence with one final readback |
| Tier 0 `pt2399_delay` | qualified | complete; one PH run committed; 0 validation errors, 0 opens, 0 represented copper violations |

Focused test:

```text
routing_xpbd_joint_relaxation_preserves_endpoints_and_reports_stress: PASS
```

That one bounded matrix covers two crossing routes, parallel routes that are too close, three
routes competing in a neck, a finite obstacle, a rectangle, a narrow passage, a concave corner,
a fixed-pad escape, terrain-gradient motion, per-layer pressure isolation, resident-buffer reuse,
and CPU/Vulkan agreement. It also converts a diffused XPBD pressure cell into ordinary bounded
routability history and proves that a subsequent connected PathFinder route has fewer exact
intersections with the stressed region. Thus the feedback test checks direction, not merely that
the returned path differs.

The final PH/XPBD Tier-0 artifact and trace are under
`build/ph-tier0-final-20260902/pt2399-delay-ph-exact-domain.kicad_pcb` and
`build/ph-tier0-final-20260902/telemetry-exact-domain/`. Flux committed one complete
quintic-PH-derived run, materialized 273 straight pieces plus 325 arcs and three vias, and accepted
the board with zero structural, connectivity, or represented copper errors. System KiCad DRC
reported zero unconnected items and no error-severity findings; its warnings were 27 local-library
footprint mismatches and six duplicate reports covering three same-net via-to-PTH hole-spacing
conditions.

## Tier comparison

The fixed Tier 1 `bm_svf` run is recorded under:

```text
build/xpbd-tier1-final-20260902/
```

The numeric baseline below is the retained pre-XPBD `bm_svf` result in
`build/verification/bm_svf-tier1-current.json`. The relaxation-only column is the retained
44.6-minute Vulkan run in
`build/xpbd-tier1-authoritative-20260902/telemetry/bm_svf-global.jsonl`. The feedback column is the
44.8-minute run in `build/xpbd-tier1-final-20260902/telemetry/bm_svf-detail.jsonl`.

These are implementation-iteration benchmarks, not a perfectly controlled microbenchmark: the
Vulkan broadphase and regional candidate code improved between the relaxation-only and feedback
runs. Aggregate violation counts therefore describe the candidates each run actually reached and
must not be interpreted as identical input populations. The comparable conclusions are that both
paths exercised real Tier-1 windows for the full budget, that feedback was sampled by 178 of 182
later physical solves, and that the easier Tier-0 board still completed through all exact gates.

| Metric | Baseline | XPBD relaxation only | XPBD + feedback |
|---|---:|---:|---:|
| Global unrouted nets at first whole-board best checkpoint | 0 | 0 | 0 |
| Sampled PathFinder overflow at that checkpoint | 25 | 13 | 3 |
| Clearance-shadow conflicts at that checkpoint | 111 | 98 | 69 |
| Aggregate XPBD input violations | n/a | 7,127 | 2,794 |
| Aggregate XPBD output violations | n/a | 1,573 | 1,293 |
| XPBD windows attempted | 0 | 146 | 182 |
| Windows with lower residual | 0 | 126 | 116 |
| Retained geometry improvements | 0 | 21 | 52 |
| Accepted conflict repairs ending at zero XPBD violations | 0 | 12 | 24 |
| Pressure-terrain invocations | 0 | 0 | 178 |
| Peak final XPBD residual | n/a | 451,024 nm | 451,024 nm |
| GPU/CPU invocations | 0 / 0 | 146 / 0 | 182 / 0 |
| Aggregate GPU solve time | n/a | 8.229 s | 3.351 s; maximum 98.8 ms/window |
| Trace duration | prior bounded run | 2,674.1 s | 2,686.6 s |
| Final DRC-clean | no | no; bounded search cutoff | no; bounded search cutoff |

The feedback run met the mechanism-level success hypothesis: real Tier-1 windows were improved and
legalized on Vulkan, retained only through the feasibility-first comparator, and their unresolved
stress was published into later feedback portfolios. It did not complete the board inside the
45-minute budget. The final trace stopped during candidate 5/8 of residual window 3; no partial
board transaction was written. Four non-authoritative human-review results are under
`build/xpbd-tier1-final-20260902/failed-route-previews/`. The best exported regional result has zero
local topology and mutual-clearance failures, but other board regions still prevent an atomic
complete-board commit.

## Success-criteria audit

| Requirement | Evidence | Result |
|---|---|---:|
| Existing Vulkan compute architecture | XPBD uses the shared compute context, resident buffers, SPIR-V build path and one GPU-resident iteration command sequence | pass |
| Joint displacement with fixed endpoints | CPU/Vulkan two-route and three-route cases; terminal particles compare bit-for-bit with inputs | pass |
| Reliable static and inter-trace convergence | The bounded synthetic geometry matrix described above passes on both backends | pass |
| Resolve a real conflict without topology change | Tier-1 retained 52 improved deformations; 24 accepted repairs ended with zero XPBD violations while preserving the selected layer/corridor gate sequence | pass |
| Residual/stress identifies difficult regions | Per-type particle, net, corridor, unresolved-pair and peak-residual telemetry is present in every invocation | pass |
| Spatial pressure/terrain conversion | Per-layer 32x32 synthetic fields and 64x64 regional fields are deposited, diffused, bounded and emitted as attributed diagnoses | pass |
| Feedback changes route selection in the expected direction | The focused test proves fewer exact route/pressure-window intersections after history injection; 178 real Tier-1 calls sampled retained terrain | pass |
| Tier improvement without easy-tier regression | Tier-1 first-checkpoint shadow conflicts improved 111 -> 98 -> 69; Tier 0 remained structurally valid, connected and accepted by represented copper DRC | pass |

## Experimental decision

Retain XPBD relaxation and spatial feedback as bounded, optional regional mechanisms. The evidence
supports using the GPU solver for joint geometry repair and for learning physical pressure: it
legalized real windows, more than doubled retained improvements in the feedback run, and consumed
only 3.351 seconds of aggregate GPU solve time during a 44.8-minute router run. It should not be
expanded into a topology planner. The unfinished Tier-1 board and the surviving 451,024 nm peak
residual show that lane, escape, layer and via alternatives still need to make the discrete
topology change when bounded physical deformation reports that a corridor is exhausted.
