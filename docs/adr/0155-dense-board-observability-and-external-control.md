# ADR 0155: Dense-board observability and external control qualification

## Status

Accepted.

## Decision

The live viewer presents routing activity newest-first while telemetry storage and trace replay
remain chronological. Regional routing emits a normal-level `SearchSummary` when conflict
extraction starts and before each complete window assignment enters B-spline realization. These
compact stage/window observations carry stable regional IDs and progress ordinals, do not publish
geometry, do not trigger recovery snapshots, and are never read by routing code.
The telemetry consumer treats `SearchSummary` as activity-only when deciding whether a periodic
recovery snapshot is warranted. Search-only bursts therefore remain visible without serializing a
full dynamic snapshot for every spline assignment.

Exact-static-checked terminal portal choices remain implemented, including pad-to-portal access
geometry and actual B-spline endpoint selection, but portfolio expansion remains behind
`RegionalRouterConfig::enable_terminal_portal_templates`. Automatically enabling portal templates
for four residual windows was stopped after 45:05 at 1,567,720 KiB RSS without producing a result.
It therefore failed the production qualification gate and is not part of the ordinary ESC path.

The Specctra comparison commands accept the same strict source-bound netclass snapshot as native
routing. `export-dsn-kicad --refillable-zones` explicitly omits canonical saved fill snapshots from
the temporary DSN board, matching an external-router-then-refill workflow without modifying the
KiCad source. The ordinary export remains fail-closed for filled regions with holes.

## ESC control measurement

Freerouting v2.3.0 was run headlessly against a fresh four-layer, 438-pin DSN from isolated source
SHA-256 `4aec9a33978c47593fd77352603d5926b27986700fe28fa34521d5d5cb9455db`, with the source-bound
netclass snapshot, no initial tracks/vias, refillable saved zones, a 100-pass ceiling, one routing
thread, and the persisted four-thread optimizer bound.

The reproducible run stopped after 25 autorouting passes and one non-improving optimizer pass. It
wrote an SES and exited successfully in 5:21.05 wall time at 2,436,164 KiB peak RSS, but retained
six unrouted connections and 194 Freerouting-reported violations. The remaining list covered
`3V3`, `AGND`, `FDCAN_RX`, `GHS1_DRV`, `MOTOR_NTC_MCU`, `PHASE_U`, `SCREF`, and `SPI1_MISO`.
Consequently it is a useful runtime/completion control, not a successful routed candidate and not
eligible for KiCad refill or DRC acceptance. The 194 count is not equated with Flux clearance
shadows because the engines use different physical models and the DSN conservatively encloses
oval and round-rectangle pads.

## Consequences

Dense regional work remains visible after the global `RoutingFinished` event instead of leaving a
silent live viewer. Activity ordering is a presentation choice only. Terminal portals can be
developed with focused fixtures without imposing an unqualified combinatorial cost on production.
Freerouting comparisons now bind current source rules and report their zone/pad approximation
boundary rather than using stale project artifacts.

An additional Flux run used the same explicit refillable-zone assumption with crossing-safe
diagonals and diagnostic telemetry. Its ordinary regional wave enumerated 821 spline assignments;
the residual wave enumerated another 819. The ordinary wave completed, but the residual wave had
realized only 11 assignments when the diagnostic was stopped at 1:00:18 and 921,728 KiB peak RSS.
It wrote no result board and the source hash remained unchanged. This mode is not promoted as the
production ESC preset: removing saved fills broadened shared-corridor coupling and made discrete
regional preparation substantially more expensive than the retained filled-zone checkpoint.

The residual retry now extracts only bounded merged cones and reuses every unaffected ordinary
assignment. Cone construction takes a transitive fragment-owner closure, preventing one shared
fragment from being realized by both retained and retry transactions. Retry-local channel,
terminal, via, via-region, template, selection, and blocker IDs are deterministically offset before
append. Every recomputed cone assignment must complete and the combined feasibility-first
checkpoint must strictly improve before replacement; otherwise the ordinary checkpoint remains
unchanged. The 384 ceiling now applies to recomputed cone assignments. Reused, recomputed, and
withheld counts are separately observable.

Prepared regional geometry is retained only under explicit memory bounds. Offset-result validation
shares prepared polygon bounds and boundary indices, required-clearance queries use capped BVH
distance traversal, NavMesh cells store inline corners and share immutable collections, and cutout
tests use a deterministic AABB hierarchy. Alternative routes prefer a proven in-outline local
domain and fail closed to full-board extraction. Worker caches clear above eight geometry or 32
navigation entries. Indexed envelope and coupled-order checks reject non-fitting lane permutations
before full position materialization. Telemetry exposes portfolio and realization progress without
instrumenting the query loops.

The isolated-ESC qualification with those changes completes in 18:08.16 wall time at 264,772 KiB
peak RSS. It reports 288 reused, 15 recomputed, and zero withheld residual-cone assignments. The
regional analysis creates 320 windows and 192 complete bundles, but the selected global checkpoint
still has 22 unrouted nets, 1,114 sampled overflow, and 4,756 clearance-shadow conflicts. Therefore
this result qualifies the bounded preparation path only; detailed materialization, refill, and
external KiCad DRC remain prohibited. The source board retains SHA-256
`4aec9a33978c47593fd77352603d5926b27986700fe28fa34521d5d5cb9455db`.

The later refillable-zone diagnosis establishes a different source-correct planning boundary.
Fixed saved fills retain voids left by removed source tracks and can therefore reject legal
same-net terminal fanout. Under the explicit route-then-refill policy, the ESC reaches all 87 nets
and 320 connections globally; its 18th pass retains 55 sampled overflow and 497 clearance-shadow
conflicts. This does not make saved fills mutable in ordinary routing and does not authorize copper
without the refill transaction.

Regional realization now indexes merged topology cells by deterministic one-millimetre bins and
shares that immutable lookup through spline trials. Lane-order and packing loops consume the
already-built stable fragment-ID index rather than repeatedly scanning every regional fragment.
Both changes preserve their original exact predicates and deterministic comparators.

Manual development validation now uses the read-only Teensy CNC controller, while project-owned
small boards remain the focused automated suite and ESC remains a dense acceptance benchmark. The
first v0.128 one-pass Teensy measurement was stopped at 40:39.26 and 698,224 KiB peak RSS in its
second 100,000-state lane-order window. It selected 418 paths with 106/110 nets, reached the fixed
384 topology-check portfolio and 931 accepted alternatives, emitted no candidate copper, and
preserved source SHA-256
`033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.

The like-for-like indexed rerun completes in 50:19.32 at 703,284 KiB peak RSS and clears all lane
windows by approximately 33 minutes. It realizes 1,077 splines and completes incremental residual
repair, but retains four unrouted nets and 212 unresolved regional windows. Its 5,324 telemetry
events are all consumed with zero queue drops, transport failures, or recorder failures. No
candidate copper is authorized, and the source hash is unchanged.

The same source revision reaches all 110 named nets when the already explicit 0.15 mm terminal
neck-down policy is combined with the 125 micrometre local escape mesh. Its complete one-pass
diagnostic takes 55:01.75, builds 555 regional windows, completes 328, recovers 68 through joint
spline optimization, and leaves 227 unresolved. This establishes global reachability but not an
accepted board.

After human placement changes, normal telemetry exposed two access-stage failures that previously
had no `connection_failures` record. Flux now emits `NetFailed` directly from selected
`pin_escape_failures` and retains nominal/narrow center blocker IDs and board-boundary status. On
source revision `b04ec917...`, both failures point to the stable ID of
`GEN_NT10_TRACK_KEEPOUT`, which forbids tracks and vias around the net tie. Revision `38856ea2...`
moves `NT10` and narrows the keepout between its pads; unchanged instrumentation then reports zero
terminal failures and the iteration-zero seed connects 110/110 named nets. The observer reports
both outcomes but cannot influence them.

The same full-board measurement identifies a bounded-search observability gap. Joint-repair
screening had consumed its 100,000 lane states while continuing to inspect later bundle choices.
Screening now stops at that ceiling and emits normal, non-blocking progress every 4,096 states.
The correction changes neither the lane predicate nor the exact transaction boundary.

The pre-correction run completes in 1:18:50 at 785,000 KiB peak RSS. It resolves 364 of 489
regional windows directly and recovers 75 with joint B-spline repair, leaving 125 unresolved. Its
5,528 recorded events have zero drops. Thirty-one residual cones fail to select a complete exact
candidate, so telemetry reports the improvement without causing export or partial commit.

The rebuilt comparison completes in 1:18:07 at 783,944 KiB peak RSS with identical routing and
regional metrics. It adds 264 scoped lane-progress events, records 5,791 events in total, and drops
none. The observer and earlier screening exit therefore preserve deterministic routing behavior;
the exact bundle-assignment loop remains the dominant runtime boundary.
