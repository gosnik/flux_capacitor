# Teensy CNC controller end-to-end verification target

> Active manual validation target: routine developer routing runs now use this read-only Teensy
> board. The redistributable 4- to 20-component progression remains the automated/focused fixture
> set and this external board must not become a CTest dependency or be modified in place.

## Scope and safety boundary

The external project at
`/home/adam/qtbits/teensy_cnc/teensy_cnc_controller/teensy_cnc_controller.kicad_pcb`
is the first real placement/routing verification target for the final application. It is a 48 V
machine-control design. Its local `AGENTS.md`, `plan.md`, pin contract, safety invariants, and
`DO NOT FABRICATE` status remain authoritative.

Flux must treat the project as read-only. Generated snapshots, reports, and candidate boards go
under `flux_capacitor/build/verification/teensy_cnc/`; no command may save, refill, patch, or
overwrite the source project. Clean CAD is evidence of represented consistency only, not
electrical performance, functional safety, or permission to fabricate.

## Baseline captured 2026-08-17

- Source revision reported by Flux: `kicad-20241229:pcbnew:9a3fbe9d94f50afb`.
- Source SHA-256: `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.
- KiCad 9.0.8 source DRC: 47 non-copper presentation violations (38 silk overlaps, four
  silk-over-copper, and five text-height issues) and 420 unconnected items; no copper-clearance
  violation is reported.
- Board scale: 198 footprints, 111 named canonical nets, two inner-layer filled zones, and no
  routed tracks or vias.
- Flux v0.21 imports both `power`-typed inner copper layers and decomposes both saved KiCad
  fractured fills into two exact regions containing 127 hole contours, with zero import errors
  and zero core validation errors. The fills are no longer blockers.
- Authoritative placement remains unavailable for 166 footprints with no explicit source
  courtyard. Flux v0.75 may now derive explicit tentative pad/Fab/Silk bounds for a bounded
  placement proposal, but any resulting placement requires human mechanical review before
  acceptance.
  Thirty-one circular graphics are currently tessellated, making import explicitly lossy until
  exact circular graphic geometry is available.
- Flux v0.93 reports 424 required canonical open pad-group connections after excluding 34
  intentionally independent pads owned by KiCad's empty-name net-zero inventory entry, versus
  KiCad DRC's 420 unconnected items. These metrics use different counting semantics and are not
  asserted equivalent; final acceptance still requires both Flux connectivity and KiCad's
  external result to reach zero.
- The v0.22 read-only shared-domain `X_STEP_3V3`/`F.Cu` routing-geometry probe retains all terminal
  pads and completes with 575 polygonal obstacles, 236 identified conservative envelopes, zero
  unsupported obstacles, one guarded union region with 4,420 vertices, and 5,671 exact-rechecked
  straight skeleton edges. Of those, 100 have zero capacity for the selected net geometry; 5,190
  curved Voronoi edges are observably deferred. The peak advisory lane capacity is 130. This
  proves common-domain construction on the real board, not placement legality, full graph mapping,
  complete routing, or strict import.
- Flux v0.23/schema 10 adds source-bound explicit routing-bus policy without inferring any bus from
  this board's net names. The 2026-08-17 re-probe reproduced all metrics above and the source
  SHA-256 before and after remained
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`. No bus policy has been
  asserted for this controller yet.
- Flux v0.24's 2026-08-18 read-only re-probe again reproduced the same 575/236/0 obstacle counts,
  4,420 offset vertices, 7,684 skeleton nodes, 5,671 retained straight edges, 100 zero-capacity
  edges, 5,190 withheld curves, and peak lane capacity 130. The source SHA-256 was identical before
  and after. Strict import remains intentionally blocked by the same 166 missing courtyards and 31
  tessellated circles, so topology-locked routing and candidate export were not run on this board.
- Flux v0.25/schema 11 adds exact native top-level track arcs, analytic arc-aware legality, and
  transactional post-allocation fillets. This controller currently contains zero track arcs, so
  that support does not alter the separate 31 circular-graphic losses. The 2026-08-18 read-only
  re-probe again reported 0 import/core errors, 166 missing placement envelopes, 31 tessellated
  graphics, 458 canonical open connections, and the same 575/236/0 obstacle counts, 4,420 offset
  vertices, 7,684 skeleton nodes, 5,671 retained edges, 100 zero-capacity edges, 5,190 withheld
  curves, and peak lane capacity 130. The source SHA-256 remained
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e` before and after.
- Flux v0.26/schema 12 retains the 31 circular courtyards as exact center/edge primitives and uses
  separately identified conservative placement proxies. The 2026-08-18 read-only import is now
  strict and lossless: 0 errors, 197 non-lossy warnings (`circle_conservative_courtyard_proxy` 31,
  `missing_courtyard` 166), and 0 core validation errors. The envelope template emits exactly 166
  review-required `approved no` entries; none has been approved or applied. The source SHA-256
  remained `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.
  The first strict read-only full-board global route reached deterministic stagnation after 17
  negotiations in about 12 minutes: 300 paths, 36 unrouted nets, 736 total sampled-resource
  overflow, 715 congested resources, 20 congested corridors, peak corridor demand 34, 43 failed
  pin escapes, and 2,194,145 obstacle-rejected search edges. This is routing/placement-pressure
  evidence, not an acceptable candidate. The source hash was again identical after the run.
  The reproducible smoke budget `--max-router-iterations 1 --max-expansions 10000` stops at
  `iteration_limit` with 217 paths, 66 unrouted nets, 291 overflow, the same 43 failed pin escapes,
  and 505,299 obstacle rejections. An un-enveloped `optimize-kicad` run fails immediately with
  `invalid_config`; it does not route, move, materialize, or export anything.
- Flux v0.27 keeps exact nearest-node access when legal and otherwise searches bounded neighboring
  grid rings. Repeating that same one-iteration/10,000-expansion smoke budget on 2026-08-18
  produced 290 paths, 44 unrouted nets, 566 overflow, 647,405 obstacle rejections, and only 4
  failed pin escapes. It exposed 823 legal terminal access nodes: 95 adaptive nodes across 39
  terminals. The increased overflow is expected because 73 additional paths now participate in
  the bounded PathFinder allocation instead of failing before it. The result remains incomplete
  and is not an acceptable candidate. The source SHA-256 was still
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e` after the run.
  A second release-build smoke run with `--pin-escape-radius 6` returned the same paths, unrouted
  nets, overflow, access-node metrics, and 4 failed escapes (647,933 obstacle rejections), showing
  that the default radius-three boundary is not the cause of those remaining failures.
- Flux v0.28 increases the integer free-space offset rounding guard from one to two nanometres.
  Exact revalidation had caught the one-nanometre result touching filled obstacle 0 on `In1.Cu`;
  the guarded shared-domain probe now completes with 118 regions, 7,666 offset vertices, 14,874
  skeleton nodes, and 11,232 retained edges. Under the same one-iteration/10,000-expansion budget,
  `--strict-medial` reaches `iteration_limit` rather than failing geometry construction: 107
  paths, 109 unrouted nets, zero overflow, 4 failed pin escapes, 18,902 topology-rejected search
  edges, zero published topology breaks, 312 interior-unmapped nodes, and 74 topology-complete
  paths. The shared graph exposes 8,623 resources, 83 packed lane assignments, and 11 accepted
  junction transitions. Because the route is incomplete, detailed materialization and export are
  correctly withheld; this is readiness evidence, not a candidate. The source SHA-256 remained
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.
- Flux v0.29 adds exact-rechecked straight endpoint-chord surrogates for finite curved
  point/segment Voronoi bisectors in strict-medial mode. The selected `X_STEP_3V3`/`F.Cu`
  shared-domain probe accepts 2,585 of 2,595 unique curved proposals, withholds 10, and grows the
  retained graph to 8,253 edges while preserving the prior 100 zero-capacity edges and peak lane
  capacity 130. Under the unchanged one-iteration/10,000-expansion strict budget, the shared graph
  grows from 8,623 to 13,146 resources, including 4,523 checked surrogates, while 2,884 failed
  curved proposals remain withheld. Routed paths remain 107 with 109 unrouted nets, zero overflow,
  and 4 failed pin escapes; topology-complete paths rise
  from 74 to 76, interior-unmapped nodes fall from 312 to 273, packed lane assignments rise from
  83 to 92, and accepted junction transitions rise from 11 to 18. Raw rejected transition checks
  rise from 18,902 to 27,014 because the larger labeled graph creates more comparisons, so that
  counter is not normalized across the two graph sizes. The incomplete result is still not a
  candidate and does not pass the detailed/export gate. The source SHA-256 before and after was
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.
- Flux v0.30 adds joint multi-layer via-array materialization for complete differential-pair and
  ordered-bus topology evidence. It does not change global allocation. The unchanged bounded
  strict probe deterministically reproduces the v0.29 result: 107 paths, 109 unrouted nets, zero
  overflow, 4 failed pin escapes, 27,014 topology-rejected checks, 273 interior-unmapped nodes, 76
  topology-complete paths, 13,146 shared resources, 4,523 checked curved-bisector chords, 2,884
  withheld curved proposals, 92 lane assignments, and 18 accepted junction transitions. Because
  the route is incomplete, the detailed coupled-group stage and export remain correctly withheld.
  The source SHA-256 before and after was
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.
- Flux v0.31 restricts medial junction evidence to resource pairs selected by routed PathFinder
  sequences, adds bounded cross-section orientation propagation, and represents nonparallel
  shifted-line joins as exact-checked polyline miters. The unchanged strict smoke budget preserves
  all allocation results: 107 paths, 109 unrouted nets, zero overflow, 4 failed pin escapes,
  27,014 topology-rejected checks, 273 interior-unmapped nodes, 76 topology-complete paths, 13,146
  shared resources, 4,523 checked curved-bisector chords, 2,884 withheld curved proposals, and 92
  packed lanes. Fifteen selected junction transitions are all accepted; three unused incident
  alternatives are counted instead of published. This bounded route has no multi-net packed
  junction requiring an orientation flip or miter. Its incompleteness continues to withhold
  detailed materialization and export. The source SHA-256 before and after was
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.
- Flux v0.32 adds bounded coupled fanout at pad and layer-run boundaries, joint via-to-track escape
  checks, deterministic longitudinal via-array translation, and ordered staircase packing. These
  are detailed-materialization changes and do not alter global allocation. The unchanged Release
  strict smoke budget again reports 107 paths, 109 unrouted nets, zero overflow, 4 failed pin
  escapes, 27,014 topology-rejected checks, 273 interior-unmapped nodes, 76 topology-complete
  paths, 13,146 shared resources, 4,523 checked curved-bisector chords, 2,884 withheld curved
  proposals, and 92 packed lanes. All 15 selected junction transitions are accepted, with three
  unused alternatives; no orientation flip or miter is required. Because the route remains
  incomplete, coupled fanout, detailed copper, and export are correctly withheld. The source
  SHA-256 before and after was
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.
- Flux v0.33 adds bounded exact-clearance backtracking to continuous detailed polyline refinement.
  It does not alter PathFinder allocation. The unchanged Release strict smoke budget again reports
  107 paths, 109 unrouted nets, zero overflow, 4 failed pin escapes, 27,014 topology-rejected
  checks, 273 interior-unmapped nodes, 76 topology-complete paths, 13,146 shared resources, 4,523
  checked curved-bisector chords, 2,884 withheld curved proposals, and 92 packed lanes. All 15
  selected junction transitions are accepted and three unused alternatives remain unselected.
  Because the route is incomplete, detailed continuous refinement and export are correctly
  withheld. The source SHA-256 before and after was
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.
- Flux v0.34 adds pressure-aware, group-preserving net order between incomplete PathFinder passes.
  The one-iteration Release smoke deliberately has no next pass, so it reports zero reordered
  iterations/nets, zero congested-resource pressure, and the complete deterministic initial order
  for 110 rerouted nets. Allocation remains unchanged: 107 paths, 109 unrouted nets, zero
  overflow, 4 failed pin escapes, 27,014 topology-rejected checks, 273 interior-unmapped nodes, 76
  topology-complete paths, 13,146 shared resources, 4,523 checked curved-bisector chords, 2,884
  withheld curved proposals, 92 packed lanes, and 15/15 accepted selected transitions with three
  unused alternatives. The source SHA-256 before and after was
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.
- Flux v0.35 adds opt-in direct PathFinder transitions for independently recertified shared
  Voronoi chords. The bounded Release probe used `--strict-medial --direct-medial` and retained
  the sampled baseline's 107 paths, 109 unrouted nets, zero overflow, and four failed pin escapes.
  It published 1,174 direct edges, withheld 11,972 collapsed, adjacent, unsafe, duplicate, or
  topology-incompatible candidates, and selected one direct edge that bypassed one intermediate
  grid step. The expanded graph exhausted its per-connection search on 108 attempts; each received
  the separately bounded sampled retry, restoring baseline path count. Returned-path evidence has
  291 interior-unmapped nodes, 72 topology-complete paths, 85 packed resources, and 16 accepted
  selected transitions. The 54,529 topology-rejected checks include both hybrid attempts and
  sampled retries. Shared allocation still has 13,146 physical resources, zero congested resources,
  and zero nanometre overflow. The one-iteration budget has no reroute-order pass. The source
  SHA-256 before and after remained
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.
- Flux v0.36 adds explicit bounded terminal neck-down access. A read-only Release comparison used
  a 0.15 mm access width, 3 mm maximum access length, one PathFinder pass, and 10,000 expansions
  per connection. On the conventional sampled graph, the same run without neck-downs produced 290
  paths, 44 unrouted nets, 566 first-pass overflow, and four failed pin escapes. Enabling the policy
  exposed ten narrow portals across six terminals, eliminated all four escape failures, produced
  305 paths, reduced unrouted nets to 39, used ten narrow segments totalling 10,699,924 nm, and
  reported 634 first-pass overflow. The increased one-pass overflow is negotiation pressure, not
  an accepted board; incomplete copper was correctly not materialized or exported. Under the
  stricter `--strict-medial --direct-medial` comparison, the policy likewise reduced failed escapes
  from four to zero and exposed the same ten portals, while topology limits retained 107 paths,
  109 unrouted nets, and zero overflow. No completed strict path used a narrow segment; direct
  evidence remained one selected edge/one bypassed sample, with 1,174 available edges, 11,972
  withheld edges, 110 sampled retries, 54,821 topology rejections, 291 interior-unmapped nodes, 72
  topology-complete paths, 85 packed resources, and 16 selected transitions. The source SHA-256
  before and after remained
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.
- Flux v0.37 carries explicit width state into bounded exact direct-medial bottlenecks. A read-only
  Release probe used `--direct-medial`, a 0.15 mm width, 3 mm maximum chord length, one PathFinder
  pass, and 10,000 expansions per connection, without enabling terminal neck-downs. It found 660
  net/chord combinations that passed capacity, length, exact narrow-chord, and both nominal portal
  gates. The hybrid search produced 292 paths, 47 unrouted nets, 461 first-pass overflow, and four
  failed pin escapes; it used 492 direct edges to bypass 1,425 sampled grid steps, with 49 sampled
  retries. None of the selected routes required a narrow chord, so the result reports zero used
  medial neck-down segments. It remains incomplete and was correctly not materialized or exported.
  The source SHA-256 before and after remained
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.
- Flux v0.38 permits continuous refinement inside equal-width portions of a mixed-width path while
  holding every width transition fixed. The focused fixture proves a deterministic shorter route,
  preservation of the original narrow segment, zero open connections, and zero copper violations.
  A read-only Release `detail-kicad` attempt on this controller used the established 0.15 mm / 3 mm
  terminal policy with one PathFinder pass and 10,000 expansions per connection. The global plan
  remained incomplete, so detailed routing stopped with `incomplete_global_route`, published no
  candidate, added no tracks/arcs/vias, and attempted no continuous moves. This is the required
  fail-closed result, not evidence that mixed-width refinement can yet run on the full controller.
  The source SHA-256 before and after remained
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.
- Flux v0.39 adds bounded width-mode allocation across sampled free-space corridors. A read-only
  Release probe used a 0.15 mm corridor width, 3 mm maximum narrow-run length, a 5,000,000-state
  guard, one PathFinder pass, and 10,000 expansions per connection. The derived expanded graph had
  421,296 states. It selected 114 one-millimetre narrow segments across otherwise independent runs,
  charged 1,737 narrow-corridor resources, reported peak narrow demand 2, and had zero narrow
  corridor overflow. The bounded whole-board result still had 289 paths, 45 unrouted nets, and 571
  aggregate overflow, so it stopped at the iteration limit. A matching `detail-kicad` attempt
  correctly stopped at `incomplete_global_route`, exposed no candidate, and added no tracks, arcs,
  or vias. Neither command was given an output path. The source SHA-256 before and after both runs
  remained
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.
- Flux v0.40 makes the selected edge width authoritative in the overlapping shared medial-capacity
  model. The same read-only Release policy automatically enabled shared allocation while retaining
  the 0.15 mm width, 3 mm run limit, 5,000,000-state guard, one PathFinder pass, and 10,000
  expansions per connection. The run used 421,296 expanded states, selected 115 narrow sampled
  edges, and allocated 1,737 narrow-corridor resources with zero narrow-corridor overflow. Fifty
  shared resource/net allocations used reduced width, removing 5,000,000 nm of aggregate envelope
  demand; the graph published 8,878 resources and 2,527 lane assignments. The optional shared
  model also exposed 91 congested resources and 49,095,168 nm of shared-resource overflow. The
  complete one-pass result therefore remained correctly incomplete with 283 paths, 49 unrouted
  nets, and 629 aggregate overflow. A matching `detail-kicad` run stopped at
  `incomplete_global_route`, exposed no candidate, and added no tracks, arcs, or vias. Neither
  command received an output path. The source SHA-256 before and after both runs remained
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.
- Flux v0.41 charges normalized width loss during routing and exposes the same term to integrated
  placement. Repeating the read-only Release policy with its default cost of 100 retained the
  421,296-state bound and 1,737 narrow-corridor resources with zero narrow-corridor overflow. The
  quality term changed the bounded one-pass allocation: 67 selected one-millimetre narrow segments
  contributed 26,800,000 nm of equivalent width loss and 6,700 route-cost units. Forty shared
  resource/net allocations removed 4,000,000 nm of demand. The 8,878-resource graph reported 93
  congested resources, 50,751,968 nm of shared overflow, 1,933 packed resources, and 2,605 lane
  assignments. The result remained correctly incomplete with 284 paths, 49 unrouted nets, and 648
  aggregate overflow. A matching `detail-kicad` run stopped at `incomplete_global_route`, exposed
  no candidate, and added no tracks, arcs, or vias. Neither command received an output path. The
  source SHA-256 before and after both runs remained
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.
- Flux v0.42 lets 45-degree pull-tight transform equal-width portions of a mixed-width path while
  holding every unequal-width boundary fixed. The focused exact fixture shortens a 100,000,000 nm
  orthogonal mixed-width route to 79,790,368 nm, accepts four width-aware corners, preserves the
  original 0.1 mm terminal segment, and deterministically returns to the legal orthogonal route
  when a foreign pad blocks one diagonal. On the controller, the matching read-only Release
  `detail-kicad` policy still stopped at `incomplete_global_route` before materialization, reported
  no candidate, no tracks/arcs/vias, and zero attempted width-aware paths or corners. This is the
  required fail-closed application-scale result. No output path was supplied, and the source
  SHA-256 before and after remained
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.
- Flux v0.43 extends native post-polyline circular fillets to equal-width corners on a mixed-width
  path. The focused fixture emits a deterministic 0.1 mm `TrackArc`, holds the following nominal
  transition fixed, reuses the narrow arc from a compatible checkpoint, and discards the complete
  curved transaction when a foreign pad blocks the fillet. Native KiCad arc export and external
  DRC fixtures remain clean. A read-only Release controller probe explicitly enabled `--arcs` with
  the established corridor policy; the incomplete global route correctly prevented detailed
  materialization, so it exposed no candidate, tracks, arcs, or vias and attempted no fillets.
  No output path was supplied, and the source SHA-256 before and after remained
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.
- Flux v0.44 adds bounded atomic position swaps between route-ranked unlocked footprint pairs.
  The focused crossed-assignment fixture proves that the same two-evaluation translation-only
  neighborhood cannot improve, while one atomic swap unions both nets, receives an incremental
  screen plus a fresh full route, and reaches zero overflow, opens, and copper violations with
  deterministic two-footprint provenance. A read-only Release `optimize-kicad` controller probe
  explicitly enabled `--swaps` without an output path. The existing 166 unreviewed missing
  placement envelopes correctly stopped the run at `invalid_config` before any candidate, global
  route, detailed route, or move was attempted. The source SHA-256 before and after remained
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.
- Flux v0.45/schema 13 adds source-bound explicit placement groups and bounded atomic rigid-group
  translation. The focused two-net fixture accepts one group move in two routed evaluations after
  one incremental screen, moves both declared members by the same vector, records the group and
  companion transform, and reaches zero overflow, open connections, and copper violations. A
  read-only Release `placement-groups-template-kicad` probe on the controller emitted an empty
  version-1 policy bound to revision `kicad-20241229:pcbnew:9a3fbe9d94f50afb`; Flux inferred no
  group from references, connectivity, buses, or differential-pair membership. Group movement was
  not run because the design has no reviewed designer group declarations and its 166 missing
  placement envelopes remain unapproved. No output path was supplied, and the source SHA-256
  before and after remained
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.
- Flux v0.46/schema 14 adds source-bound explicit placement regions whose hard gate contains each
  member's complete transformed placement envelope rather than only its anchor. The focused fixture
  accepts closed-boundary contact, rejects a one-micrometre envelope crossing, rejects missing
  authoritative envelopes, and prevents an otherwise useful region-crossing route repair before a
  second full-route acceptance evaluation. A read-only Release
  `placement-regions-template-kicad` probe on the controller emitted an empty version-1 policy
  bound to revision `kicad-20241229:pcbnew:9a3fbe9d94f50afb`; Flux inferred no partition from
  coordinates, references, or connectivity. Region-constrained optimization was not run because
  the design has no reviewed region declarations and its 166 missing placement envelopes remain
  unapproved. No output path was supplied, and the source SHA-256 before and after remained
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.
- Flux v0.47/schema 15 adds source-bound relative-placement tolerance pairs. The focused fixture
  captures its nominal anchor vector from the immutable source, accepts inclusive X/Y tolerance
  boundaries and common translation, rejects a one-micrometre excess and reversed duplicate pair,
  preserves a zero-tolerance relationship through global legalization, and prevents an independent
  route repair from separating that pair before another full-route acceptance evaluation. A
  read-only Release `relative-placements-template-kicad` probe on the controller emitted an empty
  version-1 policy bound to revision `kicad-20241229:pcbnew:9a3fbe9d94f50afb`; no relationship was
  inferred from proximity, connectivity, or references. Relative-constrained optimization was not
  run because the design has no reviewed relationship declarations and its 166 missing placement
  envelopes remain unapproved. No output path was supplied, and the source SHA-256 before and after
  remained `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.
- Flux v0.48 retains bounded nondominated placement/routing checkpoints only after fresh full-route,
  detailed, connectivity, and represented copper-DRC acceptance. The focused routed-quality
  fixture retains both the zero-displacement source and a shorter displaced route, reports
  deterministic bound pruning, and rejects a zero archive bound before evaluation. A read-only
  Release `optimize-kicad` probe on the controller reported `invalid_config`, zero candidate/full
  evaluations, an empty Pareto archive, and no accepted moves because the same 166 placement
  envelopes remain unapproved. This is the required fail-closed result rather than an application-
  scale trade-off archive. No output path was supplied, and the source SHA-256 before and after
  remained `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.
- Flux v0.49 adds opt-in atomic translations of the complete fixed-point closure of intersecting
  relative-placement constraints and rigid groups. The focused three-footprint/two-net fixture
  follows a relative edge through a touching rigid group, moves all three footprints by one common
  vector in two full-route evaluations and one incremental screen, records both policy IDs plus
  both companion transforms, and reaches zero overflow, open connections, and copper violations.
  A read-only Release `relative-placements-template-kicad` probe on the controller again emitted an
  empty version-1 policy bound to revision `kicad-20241229:pcbnew:9a3fbe9d94f50afb`; Flux inferred
  no relationship or closure. Relative-component movement was therefore not enabled. The ordinary
  `optimize-kicad` probe remained fail-closed at `invalid_config` with zero candidate, incremental,
  or full-route evaluations, zero accepted moves, and an empty Pareto archive because the same 166
  placement envelopes remain unapproved. No output path was supplied, and the source SHA-256
  before and after remained
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.
- Flux v0.50 projects analytical global-placement forces onto explicit constraint components and
  legalizes every component atomically. The focused group-plus-relation fixture moves three
  footprints by one common vector, reduces HPWL, propagates an external member lock to the whole
  component, and is deterministic. The routed CLI fixture crosses the same policy closure over a
  full-height barrier in two full-route evaluations, emits one accepted transaction with both
  policy IDs and companion transforms, and finishes with zero overflow, open connections, or copper
  violations. A read-only Release `global-optimize-kicad` controller probe remained fail-closed at
  `invalid_config`: zero candidate or full-route evaluations, no global-placement attempt, zero
  constraint components, and no accepted moves because the same 166 placement envelopes remain
  unapproved. No policy component was inferred and no output path was supplied. The source SHA-256
  before and after remained
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.
- Flux v0.51 can feed bounded PathFinder pressure, selected width reductions, and attributed
  obstacle escapes into analytical global placement. In the focused comparison, HPWL and density
  alone leave a self-contained two-pad footprint on an over-capacity lane under a two-evaluation
  budget; three deterministic routing guides move it about 1.54 mm onto a free lane, after which a
  fresh full route and detailed candidate have zero overflow, open connections, and copper
  violations. Duplicate or lock-immobilized public guides fail closed. A read-only Release
  `global-optimize-kicad --routing-guided-global-placement` controller probe was accepted by the
  CLI but stopped at the existing authoritative-envelope gate: `invalid_config`, zero source or
  full-route evaluations, no global-placement attempt, zero guides, and no accepted moves. No
  output path was supplied. The source SHA-256 before and after remained
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.
- Flux v0.52 adds source-bound radial-distance and circular relative-rotation tolerances to the
  relative-placement sidecar while preserving version 1 compatibility. Focused tests cover exact
  radial and angular boundaries, one-unit failures, wraparound across zero degrees, malformed
  limits, nominal-value consistency, optimizer rejection, and unchanged KiCad export. A read-only
  Release probe emitted an empty version 2 relationship template for source revision
  `kicad-20241229:pcbnew:9a3fbe9d94f50afb`; no relationship was inferred. `optimize-kicad`
  remained fail-closed at `invalid_config`, with zero candidate or route evaluations, because all
  166 generated placement envelopes remain unapproved. No output path was supplied. The source
  SHA-256 before and after remained
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.
- Flux v0.53 adds bounded simultaneous assignment of occupied sites to singleton or explicit-policy
  placement components. The focused policy fixture exchanges two rigid relative pairs, reaches zero
  analytical-target distance, and remains deterministic; component-limit withholding and one-state
  exhaustion both retain the legal greedy incumbent. A second fixture proves integrated value:
  sequential legalization leaves two crossed nets over capacity, while simultaneous reassignment is
  accepted after two fresh full-route evaluations with zero overflow, opens, or copper violations.
  A read-only Release `global-optimize-kicad --assignment-global-placement` controller probe was
  accepted by the CLI but remained fail-closed at `invalid_config`: zero candidate, assignment,
  global-route, or detailed-route evaluations because the same 166 authoritative placement
  envelopes remain unapproved. No output path was supplied. The source SHA-256 before and after
  remained `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.
- Flux v0.54 corrects the hard spatial differential-pair skew metric to KiCad's full absolute
  member-length difference. Flux v0.55 adds opt-in bounded point-to-point skew tuning using
  ordinary rectangular polyline detours over newly generated copper. The focused fixture rejects a
  blocked first side, accepts the opposite side after two exact candidates, adds exactly 20 mm to
  the 60 mm member, and finishes with two 80 mm members, zero opens, and zero copper violations. A
  separate integrated obstacle fixture reaches the same DRC-clean result without a placement move;
  one-candidate exhaustion, invalid bounds, deterministic repetition, and protected source copper
  all fail or withhold as designed. Developer, Release, and ASan suites each pass 106 unit tests and
  all 74 configured tests, including the external KiCad DRC fixtures. A read-only Release
  `global-optimize-kicad --tune-differential-pair-skew` controller probe accepted the new CLI and
  emitted all tuning metrics, but remained fail-closed at `invalid_config` with zero candidate,
  global-route, detailed-route, or tuning evaluations because the same 166 authoritative placement
  envelopes remain unapproved. No output path was supplied. The source SHA-256 before and after
  remained `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.
- Flux v0.56 extends the opt-in tuner with free-space-certified multi-period rectangular patterns.
  The focused fixture distributes a 24 mm half-length correction across five excursions capped at
  5 mm, adds exactly 48 mm, and produces two deterministic 108 mm pair members. Four-period and
  two-period longitudinal-capacity cases remain withheld, while connected pad/trace obstacle
  contours are assessed as a union without relaxing strict Voronoi topology. Developer, Release,
  and ASan suites each pass 106 unit tests and all 74 configured tests, including every external
  KiCad DRC fixture. A read-only Release
  `global-optimize-kicad --tune-differential-pair-skew` controller probe emitted the resolved
  0.5 mm spacing and eight-period bound plus all new region/capacity metrics, then remained
  fail-closed at `invalid_config`: zero candidate, global-route, detailed-route, region, or tuning
  evaluations because the same 166 authoritative placement envelopes remain unapproved. No output
  path was supplied. The source SHA-256 before and after remained
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.
- Flux v0.57 adds opt-in conflict-aware batch assignment across independently certified skew-tuning
  candidates. Its adversarial two-pair fixture proves that stable greedy order repairs only one
  pair, while bounded exact assignment rejects the combined conflict, selects the first pair's
  alternate region, and produces two deterministic DRC-clean 80 mm pairs. A one-state bound retains
  an exact one-pair incumbent and reports exhaustion. The fixture also exposed and fixed a legacy
  use-after-free caused by retaining pair pointers across a sequential board transaction; ASan now
  validates stable pair snapshots. Developer, Release, and ASan suites each pass 107 unit tests and
  all 75 configured tests, including every external KiCad DRC fixture. A read-only Release
  `global-optimize-kicad --batch-assign-differential-pair-skew` controller probe correctly reported
  batch assignment enabled with eight candidates per pair and 4,096 assignment states, then
  remained fail-closed at `invalid_config`: zero candidate, full-route, detailed-route, candidate,
  or assignment evaluations because the same 166 authoritative placement envelopes remain
  unapproved. No output path was supplied. The source SHA-256 before and after remained
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.

- Flux v0.58 extends the same free-space-certified skew tuner to non-axis-aligned straight copper
  through a canonical tangent/normal frame and integer-nanometre vertex quantization. The focused
  45-degree fixture rejects a blocked physical side even when the source track is reversed, then
  materializes the opposite side in both greedy and batch modes. The measured correction is
  14,142,137 nm rather than the ideal 14,142,136 nm, and the accepted pair remains within its 10 nm
  full-difference limit with zero opens or copper violations. Developer, Release, and ASan/UBSan
  suites each pass 108 unit tests and all 76 configured tests, including every external KiCad DRC
  fixture. A read-only Release
  `global-optimize-kicad --batch-assign-differential-pair-skew` controller probe emitted the new
  any-direction assessment, quantization-withholding, and materialization metrics, all zero because
  it correctly remained fail-closed at `invalid_config`: zero candidate, full-route,
  detailed-route, tuning-candidate, or assignment evaluations while the same 166 authoritative
  placement envelopes remain unapproved. No output path was supplied. The source SHA-256 before
  and after remained
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.

- Flux v0.59/schema 17 adds exact internal board cutouts and multiply connected free-space
  domains. The rectangular-hole regression proves structural validation, board-edge copper DRC,
  placement legalization, PathFinder detouring, guarded integer offset expansion, and Voronoi
  boundary construction against the same physical void. KiCad regressions cover board-owned and
  transformed footprint-owned `Edge.Cuts`, completion of an outer contour by a footprint edge,
  exact source-preserving round-trip, and fail-closed disjoint or overlapping contours. Developer,
  Release, and ASan/UBSan suites each pass 110 unit tests and all 77 configured tests, including
  every external KiCad DRC fixture. A read-only Release inspection of this controller reports zero
  board cutouts, strict lossless import, zero import/core errors, and the same 166 missing
  authoritative placement envelopes. The no-output
  `global-optimize-kicad --batch-assign-differential-pair-skew` probe therefore remained
  fail-closed at `invalid_config` before any candidate, placement, routing, detailed-routing, or
  tuning evaluation. The source SHA-256 before and after remained
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.

- Flux v0.60/schema 17 adds deterministic source-bound KiCad change sets and a thin live candidate
  bridge. C++ regressions cover exact artifact reproduction, stable complete source inventories,
  footprint-transform previews, and track/arc/via creation, update, removal, and provenance
  rejection. Injected Python IPC tests cover strict DTO validation, exact live membership and
  geometry, an in-commit stale reread, one pushed undo step, returned-object verification, rollback,
  no save API, and footprint-apply refusal. Developer, Release, and ASan/UBSan suites each pass 111
  unit tests and all 79 configured tests, including every external KiCad DRC fixture. A read-only
  Release inspection of this controller again reports strict lossless import, zero import/core
  errors, zero cutouts, zero tracks/arcs/vias, and the same 166 missing authoritative placement
  envelopes. The no-output optimizer probe stopped at `invalid_config` before any candidate,
  placement, global-route, detailed-route, or change-set work. No verified candidate therefore
  exists to preview, and a live IPC apply was not attempted without a compatible running KiCad
  instance. `apply_candidate.py --help` runs without importing `kicad-python`, confirming that the
  optional binding remains isolated at the live boundary. The source SHA-256 before and after was
  unchanged at
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.

- Flux v0.61/schema 17 adds deterministic source-bound Specctra DSN export and routing-only SES
  import. The redistributable two-pin fixture passed exact DSN provenance, placement-drift,
  protected-track, via, deterministic-ID, overwrite-refusal, candidate round-trip, and external
  KiCad DRC regressions. The official packaged freerouting v2.3.0 routed that generated DSN with 0
  unrouted items and 0 violations; Flux imported its 7 tracks and 2 vias with 0 opens or represented
  copper violations, and KiCad 9.0.8 independently reported 0 violations and 0 unconnected items.
  Resolution 1000 was rejected after freerouting produced no accessible expansion doors; its
  conventional resolution 10 routed the identical geometry and is now the checked 100 nm session
  contract. Developer, Release, and ASan/UBSan suites each pass 112 unit tests and all 85 configured
  tests. A read-only Release inspection of this controller remains strict and lossless with 198
  footprints, no source tracks/arcs/vias, zero core validation errors, 458 open connections, and
  127 saved-fill holes. DSN export therefore failed closed with two `unsupported_zone_holes`
  diagnostics and wrote no artifact. The source SHA-256 before and after remained
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.

- Flux v0.62/schema 17 adds opt-in congestion-aware multi-source growth for multi-terminal nets
  plus one transactional fixed-order fallback for an incomplete topology. The current Release
  fixed-order comparison used the established 0.15 mm / 3 mm terminal policy, one PathFinder pass,
  and 10,000 expansions per connection: 305 paths, 39 incomplete nets, 634 sampled overflow, 593
  congested resources, 29 congested corridors, and peak corridor demand 20. Under the identical
  budget, `--multisource-steiner` committed 294 multi-source connections, selected a non-fixed
  terminal 187 times, attempted 39 fixed-order fallbacks, and recovered nine complete net
  topologies. It retained 334 branches while holding the same 39 incomplete nets; the additional
  routed demand raised sampled overflow to 753, congested resources to 695, congested corridors to
  41, and peak corridor demand to 21. This is richer negotiation pressure, not a feasible
  candidate or an unqualified quality improvement. Ten narrow terminal portals eliminated all pin
  escape failures; the committed topology used 12 narrow stub segments totalling 12,799,797 nm.
  Developer, Release, and ASan/UBSan suites each pass 113 unit tests and all 86 configured tests,
  including the external KiCad DRC fixtures.
  A matching no-output `detail-kicad` probe stopped at `incomplete_global_route`, exposed no
  candidate, and added zero tracks, arcs, or vias. The source SHA-256 before and after both probes
  remained
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.

- Flux v0.63/schema 17 adds opt-in atomic ownership of present routing capacity by complete net
  trees. Under the established one-pass, 10,000-expansion, 0.15 mm / 3 mm terminal policy,
  fixed-order `--atomic-net-trees` rolled back 36 incomplete trees, 19 partial paths, and 164,406
  cost units. It retained 293 complete-tree paths, reduced incomplete nets from the comparable
  v0.62 fixed-order result's 39 to 36, and reduced sampled overflow from 634 to 587. Over three
  passes, the fair fixed-order comparison improved from 337 paths, 34 incomplete nets, and 494
  overflow without atomic ownership to 321 complete-tree paths, 33 incomplete nets, and 453
  overflow with it; the selected pass rolled back 33 trees, 19 paths, and 189,724 cost units.
  Combining atomic ownership with multi-source growth in one pass retained 303 paths and reduced
  overflow to 553, but still left 39 nets incomplete, so it is recorded as a lower-pressure
  experiment rather than the preferred feasibility result. Both modes reported zero failed pin
  escapes. A matching no-output detailed-routing probe remained fail-closed at
  `incomplete_global_route`, exposed no candidate, and added no tracks, arcs, or vias. Developer,
  Release, and ASan/UBSan suites each pass 114 unit tests and all 87 configured tests, including
  every external KiCad DRC fixture. The source SHA-256 before and after all read-only probes
  remained
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.

- Flux v0.64/schema 17 distinguishes live-frontier A* expansion limits from proved reachable-graph
  exhaustion and adds an opt-in weighted-A* retry under the same per-invocation ceiling. The
  established one-pass 10,000-expansion atomic baseline retained 293 paths, 36 incomplete nets,
  587 overflow, and cost 2,174,738. A 1.2x retry recovered no search and reproduced that allocation;
  2x recovered one search but regressed to 37 incomplete nets. A 5x intermediate reduced the count
  to 28 with 982 overflow. The selected 10x experiment invoked search 438 times, expanded 1,056,039
  valid states, observed 67 limited invocations, and recovered 29 of 48 retries. It retained 368
  complete-tree paths and reduced incomplete nets to 19, while overflow rose to 1,464 and actual
  cost to 4,935,768. For comparison, giving ordinary A* a global 50,000 ceiling retained 400 paths
  and 12 incomplete nets with 1,513 overflow and cost 5,301,949, but required 1,892,035 expansions
  across 415 invocations. Over three negotiated passes, the 10x policy selected 417 paths with six
  incomplete nets, 1,774 overflow, and cost 6,804,951, compared with the v0.63 three-pass atomic
  result's 321 paths, 33 incomplete nets, 453 overflow, and cost 2,360,819. The selected pass
  reports 476 invocations, 962,413 expansions, 53 retries, 47 recoveries, and rollback of six
  incomplete trees with no partial paths. Two additional passes found no better checkpoint. This
  is a substantial feasibility improvement but not a completed route; its higher congestion
  remains explicit. A matching no-output `detail-kicad` run stopped at
  `incomplete_global_route`, exposed no candidate, and added zero tracks, arcs, or vias. Developer,
  Release, and ASan/UBSan suites each pass 115 unit tests and all 88 configured tests, including
  every external KiCad DRC fixture. The source SHA-256 before and after every read-only experiment
  remained
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.

- Flux v0.65/schema 17 adds opt-in bounded PathFinder present-cost escalation across negotiated
  passes, plus a configurable warm-up and explicit selected-pass/effective-cost metrics. The
  deterministic two-layer crossing regression retains overflow with a small constant present
  penalty, then starts a 10x per-pass ramp at pass two and takes a legal via detour to zero overflow
  on pass three at scale 100,000. Invalid multipliers, caps, and start passes fail closed. On the
  unchanged controller,
  starting pressure at pass two with 2x retained the one-pass 19-incomplete checkpoint; 1.3x
  selected pass two with 400 paths, 12 incomplete nets, 1,472 overflow, and scale 1300; 1.05x
  selected pass two with 414 paths, eight incomplete nets, 1,666 overflow; and 1.01x selected pass
  two with 416 paths, seven incomplete nets, and 1,749 overflow. None dominated the v0.64 unscaled
  six-incomplete checkpoint. Six-pass warm-up comparisons starting escalation at pass four with
  1.3x and 1.05x therefore correctly retained the unscaled winner: 417 paths, six incomplete nets,
  1,774 overflow, actual cost 6,804,951, scale 1000, effective sampled/corridor/Voronoi present
  costs 2,000/400/400, 476 search invocations, 962,413 expansions, 53 retries, and 47 recoveries.
  This is a bounded negative benchmark result, and escalation remains disabled by default. A
  matching no-output detailed-routing run remained fail-closed at `incomplete_global_route`,
  exposed no candidate, and added no tracks, arcs, or vias. Developer, Release, and ASan/UBSan
  suites each pass 116 unit tests and all 89 configured tests, including every external KiCad DRC
  fixture. The source SHA-256 before and after every read-only experiment remained
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.

- Flux v0.66/schema 17 adds opt-in weighted continuation from retained live-frontier labels, while
  preserving every successful established from-scratch retry path, and publishes stable unrouted
  net IDs. The focused 16-expansion regression proves that two restarted phases fail after 32
  expansions while the three-phase portfolio succeeds after 46 by retaining 33 states; a companion
  30-expansion case proves restart remains authoritative when both candidates succeed. On this
  controller the established 10,000-state policy left `MCU_RUN_PERMISSION`, `RS485_RX_DNP`,
  `RS485_TX_DNP`, `SPINDLE_PWM_3V3`, `FEED_HOLD_3V3`, and `ESTOP_STATUS_SINK` incomplete. Frontier
  continuation reduced the selected result to three incomplete nets at 10,000 states and two at
  20,000. At 30,000 states it connected every net for the first time: 430 paths, zero unrouted net
  IDs, 1,983 sampled overflow, and actual cost 7,992,158. The selected pass used 463 initialized
  searches and 2,051,390 expansions; all 33 portfolio attempts recovered, 32 continued searches
  independently reached a goal, 20,453 live states were retained, and one continuation consumed
  its second ceiling. Six passes with a delayed 1.05x present-cost ramp retained full connectivity
  and selected pass five at scale 1,102, reducing overflow to 1,764 at actual cost 10,573,655. A
  1.3x ramp did not dominate the unscaled checkpoint. The route remains over capacity and is not a
  candidate for detailed materialization. A matching no-output detailed-routing run remained
  fail-closed at `incomplete_global_route`, exposed no candidate, and added no tracks, arcs, or
  vias. Developer, Release, and ASan/UBSan suites each pass 116 unit tests and all 91 configured
  tests, including every external KiCad DRC fixture. The source SHA-256 before and after every
  read-only experiment remained
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.

- Flux v0.67/schema 17 separates checkpoint feasibility dimensions when shared-medial physical
  capacity is enabled. The default aggregate objective remains byte-compatible; the opt-in
  `--physical-overflow-objective` ranks exact total and peak physical excess before sampled
  edge/node conflicts, corridor-lane excess, physical-resource count, and cost. A focused pure
  comparator regression covers every tie-break, and configuration/CLI tests reject the policy
  without shared allocation and reject duplicate options. With the existing six-pass direct-medial
  1.05x policy, the corrected objective selected the same pass-four checkpoint as v0.66: 430 paths,
  zero unrouted net IDs, 764 sampled overflow, 75 corridor overflow, 220 congested physical
  resources, 212,454,532 nm total physical overflow, 3,853,992 nm peak physical overflow, 1,059
  aggregate overflow, and actual cost 9,869,034. This is a bounded negative benchmark result: the
  prior checkpoint was already best on exact physical excess. A delayed 1.3x ramp was also
  counterproductive and correctly retained unscaled pass two at 224,879,898 nm. The route remains
  over capacity. Its no-output detailed-routing probe remained fail-closed at
  `incomplete_global_route`, exposed no candidate, and added no tracks, track arcs, or vias.
  Developer, Release, and ASan/UBSan suites each pass 117 unit tests and all 94 configured tests,
  including every external KiCad DRC fixture. The source SHA-256 before and after every read-only
  experiment remained
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.

- Flux v0.68/schema 17 adds opt-in magnitude-aware history for overfull shared physical resources.
  `--physical-overflow-history` quantizes each resource's nanometre excess by the minimum enabled
  width-plus-clearance envelope and retains the legacy one-unit update when disabled. A focused
  eight-net throat fixture reports a 150,000 nm quantum, a legacy peak increment of one, and a
  deterministic scaled increment greater than one without changing selected-pass physical demand
  or capacity. Configuration and CLI tests reject unbacked and duplicate use. At the established
  30,000-expansion ceiling the policy reduces total physical overflow to 174,437,892 nm but leaves
  two nets incomplete, so that checkpoint is rejected. Raising only the ceiling to 50,000 recovers
  all 430 paths and selects pass five: zero unrouted net IDs, 860 sampled overflow, 83 corridor
  overflow, 204 congested physical resources, 155,897,292 nm total physical overflow, 1,930,204 nm
  peak physical overflow, 1,147 aggregate overflow, and actual cost 11,985,834. Its 650,000 nm
  quantum produces 310 selected-pass history units with a peak increment of three. Relative to
  v0.67, total and peak physical overflow fall by 26.6% and 49.9% respectively, though discrete
  overflow worsens; two extra passes do not improve the selected checkpoint. The route remains
  over capacity. Its matching no-output detailed-routing probe remained fail-closed at
  `incomplete_global_route`, exposed no candidate, and added no tracks, track arcs, or vias.
  Developer, Release, and ASan/UBSan suites each pass 118 unit tests and all 97 configured tests,
  including every external KiCad DRC fixture. The source SHA-256 before and after every read-only
  experiment remained
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.

- Flux v0.69/schema 17 adds opt-in least-flexibility-first scheduling for heterogeneous physical
  routing envelopes. Within equal priority and completion tiers,
  `--physical-envelope-ordering` schedules groups by decreasing allocator-consistent nominal
  occupied demand before dynamic PathFinder pressure while preserving bus/pair order.
  Focused 150,000/550,000 nm singleton and 700,000 nm grouped fixtures prove the tie-break,
  explicit-priority dominance, group aggregation, determinism, and fail-closed configuration/CLI
  handling. The equal-budget six-pass controller run is exactly unchanged from v0.68: pass five,
  430 paths, zero unrouted net IDs, 860 sampled overflow, 83 corridor overflow, 204 congested
  physical resources, 155,897,292 nm total physical overflow, 1,930,204 nm peak physical overflow,
  1,147 aggregate overflow, actual cost 11,985,834, 441 initialized searches, 3,117,351 expansions,
  and 11 successful weighted recoveries. No live IPC net-class snapshot was supplied, so every task
  resolves the 650,000 nm imported board default and the new tie-break has no unequal envelopes to
  distinguish. The route remains over capacity. Its matching no-output detailed-routing probe
  remained fail-closed at `incomplete_global_route`, exposed no candidate, and added no tracks,
  track arcs, or vias. Developer, Release, and ASan/UBSan suites each pass 119 unit tests and all
  100 configured tests, including every external KiCad DRC fixture. The source SHA-256 before and
  after every read-only experiment remained
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.

- Flux v0.70/schema 17 adds opt-in selective conflict-cone rerouting under atomic complete-net
  ownership. Between incomplete passes it revalidates and retains complete trees outside every
  overfull sampled, ordinary-corridor, controlled-width-corridor, and shared physical resource;
  affected buses and pairs expand to their complete declared group. The focused three-net fixture
  retains one clean path while revisiting an unavoidable two-net cone, and separate checkpoint
  cases prove complete-bus expansion plus the corrected ordinary-corridor invalidation. Library
  and CLI configurations without atomic trees, and duplicate CLI flags, fail closed. On the
  equal-budget six-pass controller run, the policy selects pass five with 430 paths and zero
  unrouted IDs. Aggregate overflow improves from 1,147 to 1,133, sampled overflow falls from 860
  to 855, and congested physical resources fall from 204 to 192. Total physical overflow worsens
  by 827,548 nm (0.53%) to 156,724,840 nm, peak physical overflow worsens to 2,553,992 nm, corridor
  overflow rises from 83 to 86, and actual cost rises to 12,009,505. Five pass transitions retain
  10 net and 16 path observations while
  revisiting 540 conflict-net observations; the run loads no coupled-group sidecars. It initializes
  437 searches, four fewer than v0.69, and expansions fall 4.8% to 2,968,578; all ten weighted
  retries recover. This is a bounded discrete-congestion and search-effort improvement with worse
  total/peak physical overflow, corridor overflow, and cost—not an unqualified win. The matching
  no-output detailed probe remains fail-closed at `incomplete_global_route`, exposes no candidate,
  and adds zero tracks, track arcs, or vias. Developer, Release, and ASan/UBSan builds each pass all
  120 unit tests and 103 configured CTest scenarios, including the external KiCad DRC fixtures. The
  source SHA-256 before and after every read-only experiment remains
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.

- Flux v0.71/schema 17 makes the existing dimensioned physical-overflow policy authoritative for
  integrated placement as well as router checkpoint selection. Integrated scores now retain total
  and peak physical excess, sampled overflow, corridor-lane overflow, and congested physical-
  resource count separately; physical-objective footprint ordering consumes all three resource
  families, corridor medial segments become bounded move targets, and accepted moves preserve
  separate sampled/corridor/physical pressure provenance. Aggregate mode retains its established
  ordering and all zero-overflow materialization gates. The focused score, feedback, integrated,
  and CLI cases pass. Developer, Release, and ASan/UBSan builds each pass all 121 unit tests and 104
  configured CTest scenarios, including external KiCad DRC fixtures. A Release
  `global-optimize-kicad --direct-medial --physical-overflow-objective` probe on this controller
  reports the requested dimensioned policy but stops at `invalid_config` with zero candidate and
  full-route evaluations and zero accepted moves: reviewed authoritative envelopes are still
  missing for 166 footprints. No equal-budget routing rerun is claimed because v0.71 changes
  placement feedback, not global-router search; the mixed v0.70 checkpoint above remains the
  routing baseline. The source SHA-256 before and after the read-only probe remains
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.

- Flux v0.72/schema 17 publishes stable, saturating, unit-separated PathFinder pressure for each
  net and a deduplicated selected-group view. Route and integrated JSON expose run-wide peak
  sampled/corridor excess, total/peak physical excess in nanometres, and congested physical-
  resource participation. The production scheduler retains ADR 0043's legacy participation count.
  An authoritative fixed-order-tree experiment used the v0.70 six-pass, 50,000-expansion,
  direct-medial, dimensioned-objective, quantized-history, physical-envelope, atomic-tree, and
  selective-reroute configuration with multisource growth explicitly disabled. Dimensioned serial
  scheduling kept 430 paths and zero unrouted nets but worsened aggregate overflow from 1,133 to
  1,163, sampled overflow from 855 to 884, congested physical resources from 192 to 198, total
  physical overflow from 156,724,840 nm to 159,572,408 nm, cost from 12,009,505 to 12,226,542, and
  expansions from 2,968,578 to 3,106,190. Corridor overflow improved from 86 to 81, peak physical
  overflow stayed 2,553,992 nm, and search initializations fell from 437 to 436; retries rose from
  10 to 11. This whole-vector negative result rejected activation. The diagnostic peaks were
  215 sampled excess, 63 corridor excess, 84,869,854 nm total per-net physical excess, 3,769,904 nm
  peak per-resource excess, and 88 physical resources. The user-directed risk policy consolidates
  the compiled unit inventory from 122 passed cases to six broad invariants while retaining 104
  CLI, bridge, export, and external-DRC CTest scenarios. Developer, Release, and ASan/UBSan builds
  each pass all six unit cases and all 104 scenarios. The final fixed-order no-output detailed probe
  remains closed at `incomplete_global_route`, exposes no candidate, and adds or removes zero
  tracks, track arcs, or vias. Placement remains fail-closed without reviewed envelopes for 166
  footprints. The source SHA-256 before and after every read-only experiment remains
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.

- Flux v0.73/schema 17 adds opt-in longitudinal ownership for ordinary shared-medial resources;
  direct PathFinder chords retain their atomic snapped-chord certificates. The authoritative
  single-variable comparison uses the exact v0.70 fixed-order six-pass configuration. Its zero-
  limit baseline reproduces pass five, 430 paths, zero unrouted nets, 1,133 aggregate overflow,
  855 sampled overflow, 86 corridor overflow, 192 congested physical resources, 156,724,840 nm
  total physical overflow, 2,553,992 nm peak physical overflow, cost 12,009,505, 437 searches, and
  2,968,578 expansions. A 1 mm interval limit selects pass six with the same paths/connectivity,
  divides 549 source resources into 1,268 additional intervals, and retains 1,338 direct resources
  atomically. Total physical overflow improves 13.5% to 135,496,736 nm and searches fall to 436.
  Aggregate overflow worsens to 1,170, sampled/corridor overflow to 885/88, congested physical
  resources to 197, peak physical overflow to 3,203,992 nm, cost to 12,728,061, and expansions to
  3,169,834; retries rise from 10 to 11. This is a positive result for the configured dimensioned
  total-physical objective, not a broad congestion or runtime win, so segmentation remains opt-in
  and zero remains the default. Both routes remain over capacity and non-materializable. The
  matching no-output detailed probe stops at `incomplete_global_route`, exposes no candidate, and
  adds or removes zero tracks, track arcs, or vias. Developer, Release, and ASan/UBSan builds each
  pass all six compiled risk cases and all 105 configured integration/acceptance scenarios,
  including every available external KiCad DRC fixture. The
  source SHA-256 before and after remains
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.

- Flux v0.74/schema 17 packages the measured routing policies as the fixed routing-only
  `autoroute_v1` MVP. On this unchanged controller, `autoroute-kicad` uses six passes and a 50,000
  expansion ceiling per connection, takes approximately 19 minutes on the verification host, and
  was observed above 1.3 GiB resident memory. It truthfully stops at
  `incomplete_global_route`: no candidate is exposed, the requested new artifact remains absent,
  and zero tracks, track arcs, or vias are added or removed. Placement, neck-downs, arc fitting,
  and live application are not attempted. The source SHA-256 after the run remains
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`. This confirms the MVP's
  fail-closed large-board boundary while also identifying progress/cancellation UX and
  memory/runtime profiling as immediate follow-up work. Developer, Release, and ASan/UBSan builds
  each pass all six compiled risk cases and all 109 configured integration/acceptance scenarios.

- Flux v0.75/schema 18 permits an explicitly provisional, post-process-reviewed placement
  experiment. `--tentative-envelopes` derives non-serialized bounds from pads and supported
  Fab/Silk graphics for the 166 footprints without courtyards, preserves tentative provenance,
  and reports mandatory human review. Default integrated placement still rejects those bounds.
  The live bridge still refuses every footprint transform, so a resulting candidate can only be
  reviewed as a new source-bound file. No successful full-board placement/routing candidate is
  claimed by this policy change. The read-only tentative import is strict and lossless with 166
  `tentative_placement_envelope` diagnostics. Its conservative bounds expose 20 current-placement
  overlaps and two board-edge crossings. A deliberately tiny integrated probe with one router
  iteration and one expansion per connection passed the former missing-envelope gate and entered
  analytical placement/legalization, but remained CPU-bound at low memory after five minutes and
  was interrupted without an artifact. The source SHA-256 remained
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`. Developer, Release, and
  ASan/UBSan builds each pass all six compiled risk cases and all 110 configured
  integration/acceptance scenarios.

- Flux v0.76/schema 18 adds source-bound staged placement-move priorities without claiming a new
  full-board placement or route. Read-only template generation classified all 198 footprints:
  74 resistor, 35 capacitor, one transistor, 55 other, 23 miscellaneous IC, and 10 connector/pin
  header suggestions. It intentionally suggested zero MCU or SoC entries because `U*` cannot
  distinguish those types; a human must review the 23 miscellaneous-IC entries and promote the
  actual MCU/SoC parts before using the policy. The default cumulative order leaves the 10
  connector/header footprints until the final tier. The source SHA-256 before and after generation
  remained `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`. The small routing
  repair acceptance case reached all eight tiers before accepting a connector move. Developer,
  Release, and ASan/UBSan builds each pass all 113 scenarios while retaining six compiled risk
  cases. No Teensy optimizer run or candidate artifact was produced by this classification check.

- Flux v0.77/schema 18 makes proposal-first evaluation and legalization practical enough to expose
  the next routing boundary, without claiming a completed board. A source-certified baseline now
  reuses saved-zone topology validation only when the outline, cutouts, and complete zones remain
  exactly equal. The identical one-router-iteration/one-expansion Release probe that previously
  timed out at 60 seconds completed its 200-candidate local loop in 12.7 seconds, performing 226
  board candidate validations and reusing certified static-zone evidence for all 226. It accepted
  one local move and reduced represented tentative placement violations from 22 to 20 before the
  configured evaluation limit.

  The global placer now accepts those 22 source position defects only under explicit tentative-
  envelope opt-in and only because every error is one of 20 footprint overlaps or two envelope
  board crossings. Default behavior and all other source errors remain fail-closed, and a global
  proposal still requires complete canonical validation. An exact AABB broad phase keeps distant
  envelope comparisons out of the polygon predicate. A debugger sample of the subsequent bounded
  run showed that global legalization completed and Flux was routing the legal global seed; the
  60-second bound was then exhausted constructing free-space corridors against the board's two
  saved fills and 127 holes. This is the next measured performance blocker. No output artifact was
  written and no successful full-board route is claimed. The source SHA-256 remained
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`. Developer,
  Release, and ASan/UBSan builds each pass all 113 configured scenarios while retaining exactly
  six compiled risk cases.

- Flux v0.78/schema 18 adds an exact indexed broad phase for saved-fill holes. On the unchanged
  source placement, the Release one-iteration/one-expansion routing probe fell from 2.86 to 1.17
  seconds while preserving every compared result: zero paths, 110 unrouted nets, zero overflow,
  1,057 blocked search edges, zero board-edge rejections, four failed pin escapes, 823 pin-access
  nodes, 95 adaptive nodes, 39 terminals with adaptive access, and 1,686 corridors. The indexed
  pass rejected 11,139,704 point/segment-hole comparisons by proven bounds and retained 5,079 exact
  hole checks.

  The matching tentative `global-optimize-kicad` probe now finishes its complete 200-evaluation
  budget in 35.0 seconds rather than timing out. It performs 227 candidate validations with 227
  exact static-zone reuses, accepts the global seed, moves 190 footprints, performs 13,209
  legalization attempts, and reaches zero represented placement violations. The result records
  191 accepted move entries because every moved global-seed footprint remains individually
  auditable plus one later local repair. Its deliberately artificial one-expansion ceiling leaves
  all 110 routed nets incomplete, so detailed copper remains withheld and no candidate artifact or
  full-board routing success is claimed. The next end-to-end gate is routing feasibility and zero
  negotiated capacity overflow under a meaningful bounded search budget, followed by detailed
  connectivity/DRC and human review. The source SHA-256 remains
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.

- Flux v0.79/schema 18 makes the outer integrated experiment bound explicit and exposes exact pin
  escape failures. Both meaningful Release comparisons were limited to one placement pass and two
  complete candidate evaluations: the imported source and its legal tentative global-placement
  seed. At six PathFinder iterations and 50,000 expansions per connection, the seed completed in
  122.54 seconds using about 1.30 GB. It accepted the 190-footprint global seed with zero
  represented placement violations, then reported eight unrouted nets, 498 aggregate overflow,
  47,354,492 nm physical overflow, four failed pin escapes, and 21 weighted-frontier recoveries.
  Doubling the expansion ceiling completed in 119.14 seconds, retained exactly the same eight
  unrouted nets and four failed escapes, and reported zero expansion-limited searches. The eight
  nets were `EXTS`, `X_STEP_3V3`, `X_DIR_3V3`, `AXIS_ENABLE_SAFE_3V3`, `X_DIR_5V`, `Y_STEP_5V`,
  `Z_DIR_5V`, and `AXIS_ENABLE_5V`. Stable failure evidence identifies two terminals each on D20
  and D21 for the four 5 V nets.

  A second pair of comparisons explicitly authorized the existing 0.15 mm terminal width for at
  most 3 mm. At 50,000 expansions this exposed 11 narrow access nodes over seven terminals, used
  eight narrow segments totalling 8,399,746 nm, removed every failed escape, and reduced the
  unrouted set to `EXTS`, `X_STEP_3V3`, `X_DIR_3V3`, and `AXIS_ENABLE_SAFE_3V3`. At 100,000
  expansions the same four remained with no expansion-limited searches; the route reported 517
  aggregate overflow, 54,632,958 nm physical overflow, and 111 congested physical resources in
  126.32 seconds using about 1.41 GB. Thus bounded terminal geometry explains half the unrouted
  set, while the remaining four and negotiated overflow require further routing/placement work.
  Neither run produced detailed copper or an output board, and no complete route is claimed. The
  source SHA-256 after all probes remained
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.

- Flux v0.80/schema 18 identifies and repairs isolated pin portals without claiming detailed
  routing success. Connection-level evidence on the v0.79 legal tentative seed showed that all
  four remaining nets exhausted the represented search frontier after only two expansions, with
  zero topology rejections and no expansion limit. Three attempted terminals belonged to U20 and
  one `EXTS` terminal belonged to U10. Their legal access stubs ended at grid nodes with no
  exact-legal outgoing transition.

  Egress-aware access rejected exactly four isolated portals. With the explicit 0.15 mm / 3 mm
  terminal policy, it found connected alternatives for all three U20 terminals and reduced the
  result from four unrouted nets to only `EXTS`; U10 had no connected portal within the permitted
  access length. Increasing the search radius from three to six was bit-for-bit identical for the
  relevant result fields. Eight deterministic 1 mm U10 placement proposals were incrementally
  screened and fresh-full-route checked where promising, but none was accepted.

  Extending only the explicitly authorized maximum 0.15 mm access length to 6 mm exposes 19 narrow
  portal choices across eight terminals and selects nine narrow segments totalling 12,597,628 nm.
  The six-pass Release run reaches zero unrouted nets, zero failed pin escapes, and no connection
  failures for the first time on the legal global seed. It remains globally infeasible with 606
  aggregate overflow and 54,392,070 nm physical overflow. An otherwise identical twelve-pass run
  selects pass 12 with 523 aggregate overflow, 383 sampled overflow, 34 corridor overflow, 106
  congested physical resources, and 49,862,166 nm total physical overflow. It completed in 200.99
  seconds using about 1.71 GB and had no expansion-limited searches. Stronger 1.5x present-cost
  escalation reduced aggregate overflow only to 597 while worsening physical overflow to
  65,284,524 nm, so it was rejected as the next policy direction.

  These are abstract PathFinder plans, not copper. Detailed materialization, connectivity, DRC,
  candidate export, and human tentative-envelope review remain withheld. The source SHA-256 after
  every probe remained
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.
  Developer, Release, and ASan/UBSan builds each pass all 113 configured scenarios while retaining
  exactly six compiled risk cases.

- Flux v0.81/schema 18 bounds selective whole-net retention with an optional periodic full
  refresh. The same twelve-pass Release configuration was first rerun without selective
  retention. It selected pass 9 with zero unrouted nets and pin failures, 478 aggregate overflow,
  346 sampled overflow, 29 corridor overflow, 103 congested physical resources, 48,935,776 nm
  total physical overflow, and 1,280,204 nm peak physical overflow. It completed in 198.55 seconds
  using about 1.73 GB. This improves every v0.80 selected-checkpoint congestion dimension and
  demonstrates that complete-tree retention can hold this board in a worse negotiation basin.

  With selective retention restored and a full refresh every three pass transitions, the router
  performs three refreshes and selects pass 10. It retains zero unrouted nets and pin failures,
  reduces total physical overflow further to 47,866,766 nm, and reports 492 aggregate overflow,
  347 sampled overflow, 44 corridor overflow, 101 congested physical resources, and 1,930,204 nm
  peak physical overflow. It completes in 198.00 seconds using about 1.67 GB. The selected
  per-net pressure identifies `kicad-net-11` as the broadest conflict participant with 48
  congested resources; `kicad-net-22` has the largest reported selected-net total physical excess
  among the twelve broadest records at 6,015,886 nm. These IDs are diagnostic source-stable net
  identities, not inferred electrical intent.

  The interval-three result wins the configured total-physical-overflow-first checkpoint
  objective, while its worse corridor and peak overflow remain explicit. It is still abstract and
  over capacity: no detailed copper or candidate was produced. The source SHA-256 after both
  no-output experiments remains
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.
  Developer, Release, and ASan/UBSan builds each pass all 113 configured scenarios while retaining
  exactly six compiled risk cases.

- Flux v0.82/schema 18 removes repeated-pad bias from integrated placement pressure. Each distinct
  connected net now contributes at most once per footprint, and integrated JSON publishes the
  stable footprint rank with separate aggregate, sampled, corridor, and physical dimensions. On
  this source, `kicad-net-11` is `+3V3_TEENSY`; footprints with several pads on that net previously
  multiplied one conflict record. The corrected six-pass baseline ranks locked Teensy module U1
  first, then U20, U71, U10, U31, U70, and R75 by dimensioned pressure. This is routing evidence,
  not permission to move a locked or mechanically constrained component.

  An otherwise identical Release comparison explicitly enables the complete in-memory prefix
  suggestion. It reports mandatory priority-classification review, evaluates 16 routed local
  candidates while only the resistor tier is eligible, and accepts no local move. The suggested
  and unrestricted runs both select pass 5 with zero unrouted nets and failed pin escapes, 599
  aggregate overflow, 435 sampled overflow, 39 corridor overflow, 125 congested physical
  resources, 53,622,760 nm total physical overflow, and 1,280,204 nm peak physical overflow. The
  unrestricted run completes in 127.97 seconds using about 1.46 GB; tier-scoped global placement
  raises this to 511.19 seconds and about 1.54 GB without improving the route. The feature remains
  opt-in and every `U` suggestion remains `misc_ic` until reviewed.

  A routing-pitch congestion-displacement experiment likewise accepts no local move and produces
  identical routing evidence in 250.97 seconds, so that change was rejected. These are abstract,
  over-capacity routes. No detailed copper or output artifact was produced. The source SHA-256
  after all no-output experiments remains
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.
  Developer, Release, and ASan/UBSan builds each pass all 113 configured scenarios while retaining
  exactly six compiled risk cases.

- Flux v0.83/schema 18 uses explicitly selected narrow segments as exact placement evidence. With
  the same six-pass legal tentative seed and 0.15 mm / 6 mm terminal access, enabling the existing
  0.15 mm / 3 mm sampled-corridor neck-down policy retains zero unrouted nets and zero failed pin
  escapes while reducing aggregate overflow from 599 to 482 and total physical overflow from
  53,622,760 nm to 35,937,240 nm. Sampled overflow falls from 435 to 369, corridor overflow from
  39 to 24, and congested physical resources from 125 to 89; peak physical overflow remains
  1,280,204 nm. The selected route uses 90 narrow corridor segments totalling 90,000,000 nm and
  reduces 49 shared-resource allocations by a combined 4,900,000 nm of demand.

  Nominal-width rechecks produce 126 stable object-blocker records after ordinary search feedback
  is finalized. No record belongs to a movable footprint on this candidate, so the dedicated
  placement neighborhood generates and evaluates zero proposals instead of pretending fixed or
  atomic board geometry can move independently. The 18-evaluation integrated run takes 125.94
  seconds and about 1.42 GiB.
  It remains an abstract over-capacity plan: detailed copper, connectivity acceptance, export, and
  mandatory tentative-envelope review remain incomplete. The source SHA-256 remains
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`. Developer,
  Release, and ASan/UBSan builds each pass all 113 configured scenarios while retaining exactly
  six compiled risk cases.

- Flux v0.84/schema 18 can explicitly retain exact-legal pin-access portals beyond the first
  successful Chebyshev ring. On the established 1 mm grid, a three-ring radius and 32-node cap
  expose 15,500 later-ring nodes across 539 terminals. The equal six-pass comparison retains zero
  unrouted nets and failed pin escapes while reducing aggregate overflow from 482 to 264, sampled
  overflow from 369 to 212, corridor overflow from 24 to 4, physical overflow from 35,937,240 nm
  to 18,479,474 nm, and congested physical resources from 89 to 48. It completes in 108.31 seconds
  using about 1.04 GiB.

  Twelve passes with a 64-node cap select pass 9 and retain zero unrouted nets, 181 aggregate
  overflow, 157 sampled overflow, 4 corridor overflow, 8,645,882 nm total physical overflow,
  756,482 nm peak physical overflow, and 20 congested physical resources. The route exposes 20,574
  later-ring nodes among 21,363 terminal portals, retains 139 compatible net checkpoints and 262
  paths across selective passes, and completes in 172.51 seconds using about 1.21 GiB. Raising the
  cap to 96 worsens aggregate/sample overflow to 199/167 and total physical overflow to 10,643,620
  nm, so it is not the configured physical-objective winner.

  A legal 0.8 mm grid loses three nets without diversification and one with it. A 0.5 mm request
  fails before routing because it cannot preserve the resolved via/clearance envelope. The prior
  1 mm longitudinal segmentation policy also regresses physical overflow to 41,826,340 nm in the
  combined neck-down regime. These negative results keep pitch, segmentation, and portal count
  explicit rather than silently tuning the fixed workflow. The best route still has non-zero
  global overflow, so detailed copper, connectivity, external DRC, export, and human review remain
  withheld. The source SHA-256 remains
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`. Developer,
  Release, and ASan/UBSan builds each pass all 114 configured scenarios while retaining exactly
  six compiled risk cases.

- Flux v0.85/schema 18 preserves the represented source-object IDs that establish conservative
  shared-channel width. Repeating the exact v0.84 twelve-pass experiment produces the identical
  selected checkpoint: zero unrouted nets, 181 aggregate overflow, 157 sampled overflow, 4
  corridor overflow, 8,645,882 nm physical overflow, 756,482 nm peak physical overflow, and 20
  congested physical resources. Eighteen object-impact records attribute 14 resources to seven
  resistors, one capacitor, one diode, and three miscellaneous IC footprints;
  six outline-limited or unmatched resources correctly receive no invented owner. The run takes
  167.39 seconds and about 1.21 GiB.

  With one placement pass, a ten-candidate budget, and the default 5 mm move step, the direct-first
  scheduler screens eight distinct channel-owner proposals instead of allowing the first generic
  footprint neighborhood to exhaust the budget. Three fresh full routes are evaluated, but no
  move improves the exactly accepted checkpoint. The 327.51-second run uses about 1.23 GiB and
  retains the same objective vector. Repeating the exact ten-candidate experiment with an explicit
  1 mm move step likewise screens eight direct owners, performs eight incremental and three full
  route evaluations, and accepts no local move. It retains the identical checkpoint in 815.95
  seconds using about 1.24 GiB, so the finer displacement is rejected as the baseline. Detailed
  copper, connectivity acceptance, external DRC, export, and mandatory tentative-envelope review
  therefore remain withheld. The source SHA-256 remains
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`. Developer,
  Release, and ASan/UBSan builds each pass all 114 configured scenarios while retaining exactly
  six compiled risk cases.

- Flux v0.86/schema 18 can begin an opt-in constructive hard-capacity phase at an explicit routing
  pass. A physical-only prototype reaches zero shared-channel overflow on pass 12, but transfers
  the unresolved pressure into 218 sampled and 12 corridor conflicts, so it is not materializable
  and was generalized before release. With the complete policy starting at pass 10, three strict
  passes reject 622,696 sampled, 8,552 corridor, and 40,421 physical over-capacity transitions.
  None completes every net, so the best earlier soft checkpoint is retained rather than publishing
  partial strict geometry.

  Starting at pass 7 initially exposed that the ordinary stagnation window stopped after only four
  of six requested strict passes. After correcting that control boundary, the router honors all 12
  total passes and all six strict passes. It rejects 1,023,766 sampled, 16,352 corridor, and 78,177
  physical transitions, but again finds no all-net strict ordering. The diagnostic run therefore
  retains soft pass 5 with zero unrouted nets and failed pin escapes, 230 aggregate overflow, 185
  sampled overflow, 4 corridor overflow, 15,923,768 nm total and 850,004 nm peak physical overflow,
  and 41 congested physical resources. It completes in 192.22 seconds using about 1.57 GiB. This is
  evidence for targeted strict-phase scheduling/topology work, not a replacement for v0.85's lower
  soft checkpoint. Detailed copper, connectivity acceptance, external DRC, export, and mandatory
  tentative-envelope review remain withheld. The source SHA-256 remains
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`. Developer,
  Release, and ASan/UBSan builds each pass all 114 configured scenarios while retaining exactly
  six compiled risk cases.

- Flux v0.87/schema 18 records a constructive pass's complete attempted order, final failed nets,
  connection-level failure reason, repeated-failure count, and directed failed-net/blocking-net
  transition counts separately from the selected soft checkpoint. Pass-dependent equal-tier
  permutation moves 650 scheduling positions, but 85 distinct nets fail at least once and the best
  strict pass still misses 24 nets. All failures exhaust the search frontier and none reports a
  strict-topology transition rejection. Previous-pass blocker guidance observes 3,890 directed
  pairs and 784,874 rejected-transition attributions but also retains a 24-net best deficit, showing
  that static ordering alone cannot resolve the dense cyclic allocation.

  With 16 one-victim repairs per strict pass, all 96 bounded attempts execute, 96 completed
  singleton nets are displaced and rerouted, and 49 failed-net retries immediately complete. The
  best strict deficit improves to 23 nets. The run takes 249.27 seconds and about 1.78 GiB. Raising
  only the victim bound to two executes the same 96 attempts, displaces 192 complete nets, and
  completes 70 immediate retries. Two passes finish with 22 unrouted nets, the best constructive
  result so far, in 262.40 seconds using about 1.76 GiB. Every deferred victim remains mandatory;
  neither run reaches an all-net checkpoint, so neither can materialize detailed copper or replace
  the established v0.85 soft checkpoint. External DRC, export, and mandatory tentative-envelope
  review remain withheld. The source SHA-256 remains
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`. Developer,
  Release, and ASan/UBSan builds each pass all 114 configured scenarios while retaining exactly
  six compiled risk cases.

- Flux v0.88/schema 18 retains exact observed hard-capacity blocker sets under an explicit bound
  and scores jointly sufficient eligible victim sets before falling back to individual blocker
  ranking. With a 2,048-state bound and the established 16-attempt/two-victim policy, six strict
  passes evaluate 3,352 conflict-set states with zero withheld sets. Eighty-two of 96 repairs use
  joint two-net evidence; 50 immediately recover and 43 remain recovered at pass end. The run
  displaces 178 nets, completes in 256.70 seconds using about 1.76 GiB, and produces strict deficits
  of 38, 25, 29, 34, 22, and 30 nets. This is slightly cheaper than greedy two-victim repair but
  does not improve its best 22-net deficit.

  No strict pass completes every net, so detailed copper, connectivity acceptance, external DRC,
  export, and mandatory tentative-envelope review remain withheld. The v0.85 soft checkpoint
  remains the best verified abstract plan, and the source SHA-256 remains
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.

- Flux v0.89/schema 18 can retain one exact pre-rip-up transaction and restore it when the failed
  net's immediate retry remains incomplete. The established joint/two-victim run rolls back 33 of
  96 attempts, completes 63 immediate retries, and retains 50 at pass end. Its strict deficits are
  41, 22, 31, 30, 27, and 24 nets. It completes in 253.58 seconds using about 1.84 GiB: slightly
  faster than v0.88, with about 85 MiB additional peak memory, but no improvement to the best
  deficit.

  Rollback prevents a failed local transaction from sacrificing its already routed victims; it
  does not solve cyclic allocation after successful retries. No strict pass completes every net,
  so detailed copper, connectivity acceptance, external DRC, export, and mandatory tentative-
  envelope review remain withheld. The v0.85 soft checkpoint remains the best verified abstract
  plan, and the source SHA-256 remains
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`. Developer,
  Release, and ASan/UBSan builds each pass all 114 configured scenarios while retaining exactly
  six compiled risk cases.

- Flux v0.90/schema 18 can defer strict-repair victims to the stable pass tail while retaining the
  failed net's immediate retry and every victim obligation. With 16 two-victim attempts, 2,048
  joint states, and failed-retry rollback, six strict passes schedule 178 victims, roll back 39
  failed retries, complete 57 immediate retries, and retain 45 at pass end. Their deficits are 29,
  25, 28, 28, 18, and 27 nets. The 18-net best improves v0.89 by four and the run completes in
  263.01 seconds using about 2.02 GiB.

  Doubling attempts regresses the best to 19 nets in 299.26 seconds. Allowing three victims also
  reaches only 19 in 236.59 seconds. Disabling rollback regresses to 23, and doubling the number of
  strict passes reproduces 18 without improving it while taking 380.10 seconds and about 2.19 GiB.
  The bounded six-pass/two-victim/rollback combination is therefore the measured winner. It still
  produces no all-net strict checkpoint, detailed copper, connectivity acceptance, external DRC,
  export, or reviewable candidate. The v0.85 soft checkpoint remains the best abstract all-net
  plan, and the unchanged source SHA-256 is
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`. Developer,
  Release, and ASan/UBSan builds each pass all 114 configured scenarios while retaining exactly
  six compiled risk cases.

- Flux v0.91/schema 18 carries complete hard-routing identity on in-progress checkpoints and can
  add a bounded evidence-ranked blocker cone to each failed net's selective reuse invalidation.
  A two-blocker bound with a refresh immediately before strict routing performs 75 distinct blocker
  selections and retains 393 net plus 885 path observations. Its strict passes attempt 110, 59,
  52, 39, 37, and 37 nets; their deficits are 26, 17, 19, 21, 16, and 17. This improves the v0.90
  best by two while completing in 231.59 seconds with about 1.60 GiB peak RSS.

  A one-blocker cone retains more work but only ties the prior 18-net best. Three blockers regress
  to an 18-net peak and a 19-net final pass; the prior three-pass refresh reaches 17 with two
  blockers. The winning run executes all 96 repair attempts, schedules 147 victims, completes 57
  immediate retries, retains 39 at pass end, and rolls back 39 failures. Its selected soft
  checkpoint has aggregate overflow 204: 172 sampled and three corridor units, with 12,776,164 nm
  total physical overflow, a 1,200,080 nm peak, and 29 congested physical resources. It is still
  not legal copper.

  No strict pass completes every net, so detailed copper, connectivity acceptance, external DRC,
  export, and mandatory tentative-envelope review remain withheld. The unchanged source SHA-256
  is `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`. Developer,
  Release, and ASan/UBSan builds each pass all 114 configured scenarios while retaining exactly
  six compiled risk cases.

- Flux v0.92/schema 18 can protect a successfully recovered strict-repair target from becoming a
  later victim during the same pass. Protection does not relax capacity and clears before the next
  pass. With the v0.91 16-attempt configuration, all 47 immediate recoveries remain retained and
  the final deficit improves from 17 to 16, but the best remains tied. Twelve strict passes retain
  all 88 recoveries yet fail to improve that best, taking 332.96 seconds and about 1.82 GiB.

  Raising the protected repair bound to 48 lets late tail victims participate without sacrificing
  recovered targets. Six strict passes execute 186 attempts, schedule 251 victims, roll back 114
  failures, and retain all 72 successful retries. Their attempted-net counts are 110, 56, 52, 44,
  35, and 27; their deficits are 26, 25, 21, 18, 16, and 11. The 11-net result improves v0.91 by
  five and completes in 264.73 seconds with about 1.67 GiB peak RSS.

  A 32-attempt ceiling reaches 13. A 64-attempt ceiling exhausts eligible initial repairs after 50
  transactions and regresses to 16, so 48 is the measured bounded winner rather than a prematurely
  truncated budget. The 64-attempt run executes 200 repairs in 271.73 seconds.

  The selected soft checkpoint remains at aggregate overflow 204: 172 sampled and three corridor
  units, with 12,776,164 nm total physical overflow, a 1,200,080 nm peak, and 29 congested physical
  resources. No strict pass completes every net, so detailed copper, connectivity acceptance,
  external DRC, export, and mandatory tentative-envelope review remain withheld. The unchanged
  source SHA-256 is `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.
  Developer, Release, and ASan/UBSan builds each pass all 114 configured scenarios while retaining
  exactly six compiled risk cases.

- Flux v0.93/schema 18 charges an ordinary sampled path's initial terminal portal consistently
  with every destination node, preserves its strict blocker evidence, and honors an explicitly
  delayed hard-capacity phase. It also removes every unselected first-terminal access alternative
  from a multi-terminal tree after the first route chooses its actual portal. KiCad empty-name
  net-zero pads remain inventoried but are no longer counted as one incomplete electrical net.

  The latter correction invalidates the apparent zero-overflow tentative-placement result. Of its
  109 detailed opens, 34 were intentional net-zero pads and 75 were real disconnected branches
  ending on phantom terminal portals. Replaying the same 190 moved footprints with only selected
  portal copper finds all 430 paths but retains aggregate overflow 80: 66 sampled units, one
  corridor unit, and 4,536,926 nm shared physical excess. It reaches the configured 12-pass limit
  in 167.39 seconds using about 1.18 GiB. Detailed materialization is correctly withheld.

  On the untouched source placement, the same corrected bounded configuration selects iteration
  five with all 430 paths but aggregate overflow 277: 209 sampled units, 13 corridor units, and
  27,206,378 nm shared physical excess across 55 congested resources. Its six strict deficits are
  27, 23, 25, 25, 22, and 22 nets. The 12-pass run takes 380.02 seconds and about 1.57 GiB. The
  tentative placement therefore improves honest routing pressure substantially, but neither
  placement reaches detailed-routing eligibility.

  This is a measured regression from an invalid optimistic result to an honest incomplete one.
  The source remains read-only, no candidate is exported or applied, and every tentative move still
  requires post-process review. The unchanged source SHA-256 is
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.
  Release, Developer, and ASan/UBSan builds each pass all 114 configured scenarios while retaining
  exactly six compiled risk cases.

- Flux v0.94/schema 18 carries retained constructive hard-capacity evidence into integrated
  placement ranking, exact-terminal proposals, and incremental affected cones. It also compares
  the minimum constructive unrouted-net count before every selected soft-overflow dimension.

  The objective ordering is necessary, not cosmetic. A feedback-only 12-evaluation experiment
  moved resistor R112 and improved aggregate overflow from 80 to 60, sampled overflow from 66 to
  49, corridor overflow from one to zero, and physical overflow from 4,536,926 to 4,353,514 nm.
  Independent replay showed that its best strict deficit regressed from five to eight nets. That
  placement is rejected after adding the strict-deficit dimension and is retained only as a
  negative experiment.

  Under the corrected score, 12- and 24-evaluation prefixes accept no move and preserve the
  five-net minimum. A 48-evaluation resistor-first run performs 47 incremental screens and eight
  fresh full routes. It accepts one R74 translation from `(105000000,71000000)` to
  `(105000000,69457391)` nm, reducing the strict deficit from five to four. Aggregate overflow also
  improves from 80 to 64, sampled overflow from 66 to 52, and shared physical excess from
  4,536,926 to 3,100,656 nm; corridor overflow remains one. The bounded comparison takes 35:46.10
  and peaks at 1,347,684 KiB RSS.

  A second 48-evaluation run from the certified R74 placement performs 47 more incremental screens
  and seven full routes. It finds no four-to-three improvement, accepts no move, and preserves the
  R74 checkpoint. The run takes 31:04.75 and peaks at 1,303,980 KiB RSS. Subsequent deterministic
  pages use the explicit candidate offset from ADR 0106 so this separately certified prefix is not
  routed again; an offset page cannot independently claim a local optimum. The offset-47 page
  skips exactly that prefix, routes the remaining 11 legal candidates with two full checks, and
  likewise accepts no move. It takes 18:02.59 and peaks at 1,422,248 KiB. Together the two pages
  exhaust the 59-candidate legal 2 mm neighborhood without reducing the four-net deficit.

  Independent replay identifies the remaining best-pass nets as `+3V3_TEENSY`,
  `CYCLE_START_3V3`, `Z_DIR_5V`, and `ESTOP_STATUS_LED_A`. No strict pass is complete, so detailed
  copper, connectivity acceptance, external DRC, export, and mandatory tentative-envelope review
  remain withheld. The external source remains read-only with SHA-256
  `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.

- Flux v0.128/schema 18 re-establishes this board as the primary manual routing/validation target.
  A refillable-zone, crossing-safe-diagonal, one-pass run with a 10,000-expansion connection bound
  generated the complete source scene and live provisional geometry through the default WebSocket.
  Its selected iteration-zero seed retained 418 paths, 106/110 routed named nets, 5,014 sampled
  overflow, 325 corridor overflow, and 15,818 clearance-shadow conflicts. The negotiated first
  pass itself routed 91/110 named nets with 86 sampled overflow and 581 shadows; feasibility-first
  selection correctly retained the seed with fewer unrouted nets.

  Regional preparation consumed the fixed 384 topology-check portfolio and reached 931 accepted
  reusable/joint alternatives. The run was deliberately stopped after 40:39.26 at 698,224 KiB
  peak RSS while reducing the second 100,000-state lane-order window. Debugger evidence showed
  repeated scans of all 2,409 regional fragments from per-state geometric ordering and packing.
  This is a scalability measurement, not a completed regional route. Flux now uses the existing
  stable fragment-ID index in those loops and a one-millimetre cell-bin index for merged spline
  topology guards; the indexed rerun below measures those corrections. The source hash
  remained `033bd52abdcc6f8439d9f955947dd9a559654b475a045ae5baa9fefca282174e`.

  The like-for-like indexed rerun completes in 50:19.32 at 703,284 KiB peak RSS. It clears every
  lane-order window by approximately 33 minutes—where the original was still inside window 5 at
  40:39—and completes all 1,165 ordinary spline assignments before residual repair. The final
  diagnostic contains 463 regional windows, 251 complete bundles, three exhausted bundle searches,
  209 spline failures, 1,077 retained spline realizations, and 212 unresolved windows. Four named
  nets remain globally unrouted (`X_DIR_5V`, `Y_STEP_5V`, `Z_DIR_5V`, and `AXIS_ENABLE_5V`), with
  5,014 sampled overflow, 325 corridor overflow, and 15,818 clearance-shadow conflicts. Fourteen
  residual cones recompute 14 assignments and reuse 435, but do not improve the checkpoint. The
  telemetry observer consumes all 5,324 events with zero queue drops, transport failures, or
  recorder failures. No spline candidate or copper transaction is available; this qualifies the
  indexing correction, not route completion. The source hash remains unchanged.

  Run-history phase timestamps subsequently identify lane-order allocation as the largest single
  avoidable CPU cost: visible ordinary and residual lane searches consume about 37.4 minutes of the
  78-minute baseline. Lane allocation now rejects a cross section in constant time when its width
  is below the sound physical lower bound `sum(widths) + sum(clearances) + max(clearance)`, and
  memoizes every complete channel/occupant answer across screening and assignment. Physically
  plausible cases retain the established lexicographic order selection.

  An intermediate Pareto-DP experiment finishes in 53:30.70 at 2,006,960 KiB peak RSS. It reports
  2,007,612 lane queries, 2,002,129 cache hits, 22,030 DP states, 896,506 analytical cross-section
  rejections, and no lane-budget exhaustion. However, width dominance changed equal-feasibility
  lane choices: only 340 bundles remain complete and 146 fail spline realization, compared with
  364 and 122 in the qualified baseline. That result is rejected and authorizes no candidate. The
  retained implementation restores exact lexicographic selection after the analytical/cache fast
  path.

  The follow-up implementation carries the sound physical envelope bound into recursive bundle
  construction and prunes a partial choice only when its selected occupants already exceed every
  represented cross section. Its source-identical run completes in 56:39.78 at 2,004,508 KiB peak
  RSS. It rejects 13,793 impossible prefixes, reduces bounded bundle-search states from 302,770 to
  202,855, reduces complete access-assignment states from 1,586,411 to 1,335,666, and removes all
  three bundle-search exhaustions. The run completes 341 bundles, evaluates 1,049 joint spline
  candidates, recovers 79 windows, performs 860 spline realizations, and leaves 148 windows
  unresolved. Its 2,377,034 lane queries still contain 2,371,645 cache hits, so equivalent-prefix
  access work—not lane-order computation—is now the clearer regional optimization target.

  Telemetry produces and consumes all 7,310 router events with zero bounded-queue drops, 22
  recovery snapshots, no recorder or transport failure, and one non-authoritative WebSocket
  message drop. The complete JSONL trace remains available for replay. No full-board spline or
  copper candidate is authorized, and the source board remains unchanged at
  `38856ea2c4e6030125727a5933e77f882ef66b4b96e469cbce7df305321f26dc`.

- A later explicit-width comparison on that same source uses a reviewed-for-test 0.15 mm terminal
  neck-down, two-millimetre maximum run, and 125 micrometre local escape mesh. It routes all 110
  named nets globally. The complete diagnostic takes 55:01.75 at 826,164 KiB peak RSS, creates 555
  regional windows, completes 328 bundle assignments, recovers 68 windows through the joint spline
  solver, and leaves 227 unresolved windows. Remaining global pressure is 5,202 sampled overflow,
  363 corridor overflow, and 17,121 clearance-shadow conflicts. No spline/copper candidate is
  authorized, and this explicit controlled width is not a default fabrication rule.

## Placement revision captured 2026-08-30

- Current source SHA-256 is
  `38856ea2c4e6030125727a5933e77f882ef66b4b96e469cbce7df305321f26dc`.
- `NT10` is now at `(140,130)`, with `PWR_GND` and `GND` pad centres at `(138,130)` and
  `(142,130)`. The source-owned `GEN_NT10_TRACK_KEEPOUT`
  (`9774c3ab-35c2-4fe9-904d-d054ef1685bb`) is narrowed to `(138.5,129)` through `(141.5,131)`,
  between those centres.
- The same exact nominal/narrow centre test that attributed both earlier failures to the keepout
  now reports no inaccessible terminal. No exception or keepout override was added to Flux.
- A bounded global probe with the explicit reviewed-for-test 0.15 mm terminal neck-down and 125
  micrometre local mesh selects 430 connection paths covering all 110 named nets. Iteration zero
  has 2,485 sampled overflow, 56 corridor overflow, and 7,484 clearance-shadow conflicts. The
  first PathFinder pass reaches 89/110 nets with 152 sampled overflow, zero corridor overflow, and
  483 shadows, so the feasibility-first comparator correctly retains the complete iteration-zero
  checkpoint.
- Full regional/spline validation of this source revision is the current benchmark. Candidate
  export, refill, re-import, and external KiCad DRC remain forbidden until an exact complete copper
  transaction exists.
- The pre-correction one-pass regional baseline completes in 1:18:50 at 785,000 KiB peak RSS. It
  builds 489 eligible windows and 2,086 attributed conflict edges. Of those windows, 364 form
  complete bundles, three exhaust bounded bundle search, 122 fail spline bundle realization, and
  125 remain unresolved after 822 B-spline realizations.
- Joint repair attempts 488 windows, completes 292, and recovers 75 through simultaneous spline
  optimization. Residual processing builds 31 cones, reuses 423 assignments, recomputes 31, and
  selects none; 1,341 residual conflicts remain attributed. No exact full-board spline candidate
  is available.
- The recorded WebSocket/trace stream contains 5,528 events with zero drops. The source SHA-256 is
  unchanged after the run. The focused unit, viewer, and live-client risk suite passes.
- Profiling by phase identified a non-authoritative runtime tail after the 100,000-state lane-order
  ceiling. The rebuilt router stops the surrounding choice screen at that same ceiling and emits
  normal telemetry every 4,096 states. Its like-for-like run finishes in 1:18:07 at 783,944 KiB
  peak RSS, 43 seconds faster, with every authoritative routing/regional count unchanged. The trace
  contains 5,791 events, including 264 lane-progress events, with zero drops. The comparison shows
  that exact bundle assignment remains the dominant runtime cost; no physical or exact-DRC gate is
  weakened.
- The first nested-routability-feedback run retains the same 110/110-net, 430-path seed with 2,485
  sampled overflow, 56 corridor overflow, 7,484 clearance shadows, and route cost 811,434. Its
  first pressure-informed probe routes 95 nets; its post-regional probe routes 59. Both are
  rejected by the feasibility-first archive, and checkpoint telemetry restores the selected
  complete topology. The regional pass takes about 79:33, constructs 489 windows and 2,086
  conflict edges, completes 340 bundles, evaluates 1,010 joint spline candidates, recovers 75,
  and leaves 73 of 151 typed diagnoses unresolved after 78 local resolutions. The run was stopped
  at 1:22:14 when the pre-shortcut binary began replaying the retained checkpoint before an
  otherwise identical second regional calculation; it peaked at 2,027,408 KiB while the archived
  regional result and final global probe coexisted. No preview or candidate board was written.
  The rebuilt planner now returns the archived regional result immediately when that final global
  probe loses the comparator. Focused unit, viewer, and live-client tests pass after the change,
  and the source hash remains unchanged.

The external project is intentionally not copied into this repository. This document records the
local target and reproducible acceptance contract without redistributing its design data.

## Final acceptance workflow

1. Record the source bytes/revision and verify the source path remains unchanged throughout the
   run.
2. Import the board strictly with zero errors and zero unacknowledged loss. This gate now passes.
   Circular courtyards, native copper track arcs, inner copper layers, and saved-fill topology are
   represented. Tentative integrated placement may run before envelope review, but its candidate
   cannot pass final acceptance until the design owner reviews every moved footprint against the
   exact mechanical requirements.
3. Read project net classes and effective membership through the official KiCad IPC bridge and
   bind the snapshot to the recorded source revision. Import additional explicit placement,
   orientation, differential-pair, grouping, safety, and routing constraints with provenance.
4. Run placement and routing with deterministic configuration and retain the complete before/after
   objective, moves, corridor capacity/demand, routes, unrouted items, and violations. Locked and
   out-of-scope objects must not move.
5. Require a fully connected candidate with no global overflow, internal validation errors, or
   represented copper DRC violations. Preserve the two plane fills and all unsupported/user-owned
   constructs byte-for-byte.
6. Export only to a new candidate path below `build/verification/teensy_cnc/`, perform strict
   semantic re-import, and verify the change set is limited to explicitly accepted footprint
   transforms and generated tracks/track-arcs/vias.
7. Run `kicad-cli pcb drc --format json --exit-code-violations` on the candidate and require zero
   violations and zero unconnected items. Do not refill or save the source board.
8. Run the controller project's own required checks only when their owners authorize the resulting
   controlled-source change. Review the candidate against its requirements matrix, decision log,
   interface contract, calculations, safety analysis, and component datasheets outside Flux.
9. Keep `DO NOT FABRICATE` in place. Passing these gates does not establish machinery safety or
   fabrication readiness.

The current read-only baseline command is:

```sh
./build/dev/flux-cli inspect-kicad \
  /home/adam/qtbits/teensy_cnc/teensy_cnc_controller/teensy_cnc_controller.kicad_pcb
```

Selected net/layer routing geometry can be exercised independently of the blocked placement gate:

```sh
./build/dev/flux-cli inspect-free-space-kicad \
  /home/adam/qtbits/teensy_cnc/teensy_cnc_controller/teensy_cnc_controller.kicad_pcb \
  X_STEP_3V3 F.Cu --shared-domain
```

This diagnostic does not relax the strict-import or final-acceptance requirements above.

The review-required placement-envelope template can be inspected without applying it:

```sh
./build/dev/flux-cli envelopes-template-kicad \
  /home/adam/qtbits/teensy_cnc/teensy_cnc_controller/teensy_cnc_controller.kicad_pcb
```

This template remains the pre-approved authoritative-envelope workflow. Under the independent
proposal-first policy, `global-optimize-kicad --tentative-envelopes` may run without editing it,
but every resulting move must still be checked against the exact STEP model, assembly process,
connector access, thermal spacing, service clearance, and machine-safety needs.

The connector-last placement policy can likewise be generated without applying it:

```sh
./build/dev/flux-cli placement-move-priorities-template-kicad \
  /home/adam/qtbits/teensy_cnc/teensy_cnc_controller/teensy_cnc_controller.kicad_pcb
```

Review every `U*` suggestion and explicitly classify the Teensy MCU and any SoC before passing the
saved sidecar to `--placement-move-priorities`. Priority never overrides source locks, placement
constraints, tentative-envelope review, routing completion, connectivity, or DRC.

For a proposal-only comparison, `--suggested-placement-move-priorities` applies the same complete
prefix suggestion in memory and sets `placement_move_priority_review_required` and
`human_placement_review_required`. It cannot be combined with the reviewed sidecar, and its result
must not be accepted as reviewed MCU/SoC classification.

Tentative optimize/export may produce a review candidate after step 3 passes strictly. That
candidate must not be accepted, applied live, or fabricated until post-process envelope review and
all remaining final-acceptance steps pass.
