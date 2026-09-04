# Router research log

## Experiment 1 — Capacity-reserve pressure from tscircuit capacity routing

### Failure

The qualified `pt2399_delay` Tier 0 board completed with accepted copper, external KiCad DRC,
and layer PNGs.  The unlocked Tier 1 `bm_svf` probe did not reach a checkpoint inside its
180-second bound using the manifest configuration.  This is a global-allocation/search-cost
problem, not evidence that DRC or detailed routing may be relaxed.

### Hypothesis

Flux already represents sampled resources, free-space corridors, shared physical-medial
resources, and clearance shadows.  Its ordinary corridor/medial present cost was mainly linear,
so a path could consume the final useful fraction of a physically packable resource before any
overflow signal changed the choice.  A normalized soft reserve should preserve manoeuvring room
and leave fewer late physical conflicts for the regional solver.

### Research

`tscircuit-autorouter` uses a capacity mesh and allows negative capacity during search, but
raises cost as remaining capacity diminishes and more strongly when it becomes negative.  Its
capacity model is deliberately a planning approximation, followed by dedicated high-density
realization.  Flux already has more specific free-space and exact-DRC representations, so copying
the upstream quadtree and rectangle assumptions would duplicate weaker geometry rather than
improve routing.

Useful transferred idea: a soft, normalized reserve cost.  Rejected for now: replacing Flux's
NavMesh/corridor/Voronoi representations with the upstream capacity mesh; doing so would discard
physical-width provenance, curve topology gates, and exact KiCad geometry validation.

Sources:

- https://github.com/tscircuit/tscircuit-autorouter
- https://github.com/tscircuit/tscircuit-autorouter/tree/main/lib/solvers/CapacityPathingSolver
- https://blog.autorouting.com/p/hypergraph-autorouting

### Experiment

Added a bounded `capacity_reserve_per_mille` / `capacity_reserve_cost` signal to the global
router.  It is applied to:

- free-space corridor capacity when a route enters a newly claimed corridor;
- shared medial physical width using the actual resolved copper-envelope demand in nanometres;
- positive-but-fragile clearance-shadow margins, without double-charging already overlapping
  shadows.

The reserve remains non-authoritative.  It cannot block a transition, relax an existing hard
capacity, or certify copper.  It is escalated with the established PathFinder present-cost scale,
exposed in route JSON, retained in checkpoint compatibility, and bounded by validation.

NavMesh decision: no duplicate NavMesh-edge occupancy table was introduced.  In Flux a NavMesh
edge/portal is topology evidence, while the carried free-space corridor and physical medial
cross-section are the capacity-bearing representation.  Charging both would double-count the same
constriction and make topology selection depend on a second, less physical raster.  The
iteration-zero portfolio therefore applies the reserve to the candidate's carried corridor and
medial resources; its directed NavMesh topology signature remains a separate realization gate.

### Before

- Tier 0: pass — 50 paths, accepted final copper, external KiCad DRC, copper PNG export.
- Tier 1: the 30-pass `bm_svf` manifest probe exceeded its 180-second runner limit before
  producing an end-of-run checkpoint.

### Success criterion

The change must keep Tier 0 accepted and make the Tier 1 prefix expose nonzero reserve pressure
without increasing hard-rule violations.  Tier 1 remains the frontier only if its bounded probe
still has a precise diagnostic.

### Verification

- Focused global/detailed transactional and capacity-gate regression: **7/7 passed**.
- Tier 0 (`pt2399_delay`): **pass** — 50 paths, accepted copper, external KiCad DRC, and final
  copper PNG export.
- Tier 1 (`bm_svf`, manifest configuration): **timeout at 180 seconds**, the same bounded outcome
  as the baseline.  It is not promoted.
- The matching four-pass Tier 1 diagnostic also did not publish a checkpoint before its 180-second
  bound.  The pre-change four-pass checkpoint had 78 routed paths and 101 aggregate conflicts;
  consequently there is no honest before/after conflict-count comparison for the modified run.

### Status

Implemented and retained.  The feature is active in early portfolio selection and negotiated
search, is observable in JSON/telemetry-facing route results, and does not alter DRC acceptance.
It needs a later, longer Tier 1 run or a separately bounded prefix benchmark before its effect on
completion can be claimed.

## Experiment 2 — Bound exact regional work to eligible conflict cones

### Failure

The first Tier 1 prefix reached every coarse connection, but the physical probe reported 179
conflicts, including 161 clearance-shadow conflicts in one 21-net residual cone.  That cone is
larger than the configured regional joint-routing bound.  Prior to this change, Flux nevertheless
entered the terminal/via access CSP while constructing every bundle-template branch.  One access
budget was applied per partial branch rather than per regional portfolio, so nested CSP work could
vastly exceed the intended bounded conflict-cone budget.

### Hypothesis

Regional routing is a detailed physical-packing stage, not a substitute for global reachability
or negotiated congestion.  A regional CSP should only run after all coarse connections are routed
and only for a conflict cone at or below its declared local-net bound.  Within an eligible cone,
its terminal/via access search must share one budget across partial template combinations.

This matches the separation used by detailed-routing systems: pin/via access and track assignment
are local repair stages following global routing, not unbounded work repeated for every global
candidate.  Forward checking remains useful, but exhausting its bound is uncertainty and must not
be recorded as proof that a route candidate is illegal.

Sources:

- https://openroad.readthedocs.io/en/latest/main/src/drt/README.html
- https://vlsicad.ucsd.edu/Publications/Journals/j136.pdf
- https://www.cs.cmu.edu/~15281-s24/coursenotes/constraints/index.html

### Change

- Made `max_access_assignment_states` a shared total budget for one regional bundle portfolio.
  Exhaustion now stops that portfolio without pruning the current branch as geometrically
  impossible.
- Deferred regional realization when coarse topology is incomplete, or when the largest residual
  cone exceeds `max_nets_per_window`.  The explicit diagnostic records the reason and size.
- Added regression coverage for both the shared access-CSP limit and the global-to-regional
  hand-off gate.

No route is accepted through the deferral path.  The global result remains incomplete until the
ordinary overflow/physical-conflict gates are satisfied, and exact detailed copper plus external
KiCad DRC remain authoritative.

### Measurements

On the `bm_svf` one-iteration, 1,000-expansion diagnostic, stopping the oversized regional
portfolio reduced the end-to-end diagnostic time from 78.10 s to 38.55 s.  The resulting report
was more useful: 75 paths, 2 unrouted nets, 524 aggregate overflow, 402 clearance-shadow
conflicts, and one 22-net residual cone.  These are global allocation failures, correctly handed
back to the global loop rather than hidden in detailed CSP work.

With a 50,000-expansion valid one-pass diagnostic, Flux reaches all 78 connections but has 179
aggregate overflow (16 sampled and 161 clearance-shadow) in one 21-net cone.  That establishes
the next Tier 1 frontier as collective physical packing / global allocation, not terminal
reachability.  The standard 30-pass Tier 1 manifest run still exceeds its 180-second bound and is
not promoted.

The final-code Tier 0 requalification produced both copper-layer PNGs and a KiCad DRC report with
zero violations and zero unconnected items.  The focused C++ regression suite remained 7/7.

### Status

Implemented and retained.  This is a performance and architecture correction, not a Tier 1
completion claim.  It protects the regional spline/PNS solver from an oversized global problem
while preserving its exact local role for bounded cones.

## Experiment 3 — Make staged allocation operational and remove medial-probe allocation

### Failure

Tier 1 telemetry showed that the static priority analysis correctly placed the scarce differential
pair and high-interference nets first, but the first negotiated PathFinder pass still used the
ordinary stable-ID task order whenever the baseline weak-occupancy static portfolio won the
iteration-zero comparator.  The priority feature could therefore be observable without affecting
the allocation that creates the physical conflict cone.

A live stack sample of the full run also found the hard-capacity search repeatedly copying an
entire physical-medial resource's occupant `std::set` and demand `std::map` for every candidate
edge just to calculate a hypothetical capacity.  This was especially costly in the dense phase
where the route must inspect many already-occupied resources.

### Hypothesis

The staged task order is the original first-pass allocation decision, not an in-pass reorder.  It
can therefore be used for the first negotiated pass while retaining ADR 0043's restriction that
subsequent pressure-driven reorders happen only between incomplete passes.  The same order remains
atomic for declared pairs and buses.

For physical medial capacity, virtual membership for one candidate net is sufficient.  Compute
the candidate resource envelope directly against immutable existing occupants instead of making
temporary containers.  The calculation must preserve pair and complete-bus envelope rules exactly.

### Change

- Feed `staged_iteration_zero_order()` into the first negotiated PathFinder pass even if the
  baseline static portfolio is retained.  Expose `initial_pathfinder_order` separately from later
  pressure reorders in CLI JSON and telemetry-facing state.
- Add a regression assertion that a differential pair remains contiguous in that first-pass order.
- Replace copying physical-medial capacity probes with an allocation-free virtual include/exclude
  calculation.  Complete buses are accounted once at their declared first member and differential
  pairs once by stable member order, matching the prior `occupied_demand()` semantics.

This borrows the useful *low-capacity-first allocation* idea from tscircuit without adopting its
coarser capacity mesh.  Flux retains conservative corridor/medial geometry, temporary negotiated
overflow, and exact KiCad validation.  The dynamic-capacity penalty in tscircuit's A* heuristic
was not copied because Flux keeps its physical-distance heuristic admissible.

Sources:

- https://raw.githubusercontent.com/tscircuit/tscircuit-autorouter/main/lib/solvers/CapacityPathingSolver/CapacityPathingSolver.ts
- https://raw.githubusercontent.com/tscircuit/tscircuit-autorouter/main/lib/solvers/CapacityPathingSolver/CapacityPathingSolver5.ts
- https://raw.githubusercontent.com/tscircuit/tscircuit-autorouter/main/lib/utils/getTunedTotalCapacity1.ts

### Measurements

On the valid one-iteration / 50,000-expansion `bm_svf` prefix, the operational staged order keeps
all 78 connections routed and reduces aggregate overflow from 179 to 146 and clearance-shadow
conflicts from 161 to 120.  The resulting residual component remains one 23-net cone, correctly
too large for the bounded regional solver.

After the allocation-free capacity probe, the same prefix has identical routing metrics and
completes in 42.72 s with a 105,568 KiB peak resident set.  This establishes semantic parity for
the optimization.  No historical elapsed time was captured for the pre-optimization equivalent,
so no numeric speedup is claimed.

The final-code Tier 0 board still exports both copper PNGs and has zero KiCad DRC violations and
zero unconnected items.  The standard Tier 1 manifest run still exceeds 180 seconds and is not
promoted.

### Status

Implemented and retained.  The next Tier 1 change must reduce or partition the 23-net physical
packing cone; it should not weaken the regional hand-off or exact DRC gates.

## Experiment 4 — Spatial regional hand-off and multi-blocker detours

### Failure

The one-pass staged Tier 1 allocation connects all 78 paths but leaves 146 aggregate conflicts.
Its conflict graph is one 23-net connected component.  Treating that global component as a single
regional transaction incorrectly deferred the detailed stage even though the regional router
already derives biconnected, spatially bounded windows from it.

The remaining per-window diagnostics also showed an unnecessary restriction: same-layer NavMesh
detours were attempted only for a window with one conflict edge.  A three-way crossing therefore
gave its spline/PNS realization one selected template for each participant, despite the detour
generator already constructing dynamic walls from all attributed blockers.

### Hypothesis

A global connected component is evidence for feedback, not the local solver's state size.  Once
all coarse paths exist, the regional decomposition should choose its own bounded windows and
withhold only the individual windows that cannot be partitioned safely.

For an eligible multi-blocker window, a bounded same-layer detour can preserve the selected layer
and change homotopy without committing to a via.  It must clear every attributed route envelope,
retain the NavMesh topology certificate, and remain subject to the existing complete exact
materialization / connectivity / DRC transaction.

### Change

- Changed the global-to-regional hand-off gate to require complete coarse topology only.  The
  regional biconnected spatial splitter remains responsible for enforcing the fragment/net window
  bounds.
- Permitted the existing same-layer detour generator in a joint-repair window with more than one
  conflict edge.  Its existing bounded check count and per-fragment template cap remain in force;
  it computes obstacle walls from every relevant blocker rather than treating a dense window as a
  pairwise special case.

### Measurements

The one-iteration / 50,000-expansion `bm_svf` prefix now reaches 72 eligible regional windows.
The router completes 58 bundle assignments and evaluates 205 joint spline candidates; 15 windows
are recovered by that local transaction.  Access-assignment exhaustion is zero, so this is not an
unbounded terminal/via CSP failure.

The multi-blocker wave accepts 47 exact-prevalidated same-layer detours.  It reduces selected
residual conflicts from 26 to 25 while retaining the same 146 global aggregate overflow.  The
dominant unresolved three-fragment B.Cu cross still has one retained template per fragment:
layer/via alternatives are rejected by exact run-up or NavMesh feasibility, and its three spline
realizations retain mutual/quantized clearance conflicts.  This identifies the next issue as
upstream allocation / local escape topology, rather than a reason to weaken spline or DRC rules.

The focused C++ suite remains 7/7.  The one-pass prefix is intentionally incomplete (exit status
3, `iteration_limit`), and Tier 1 has not been promoted.

The standard progressive corpus was also rerun on this code.  Tier 0 requalified with regenerated
`F.Cu`/`B.Cu` PNGs and an external KiCad report containing zero violations and zero unconnected
items.  The normal 30-pass Tier 1 `bm_svf` probe again exceeded its 180-second harness deadline
before publishing a checkpoint, so that CTest invocation is a timeout rather than a pass.  This
does not change the Tier 1 policy: it remains permanently unlocked and not promoted.

### Status

Implemented and retained.  Regional work is now correctly spatially partitioned and can try a
safe homotopy change for dense same-layer windows.  The next effective Tier 1 change should feed
these exact local diagnoses into the global routability history early enough to select different
coarse corridors, terminal escapes, or layer sequences.

## Experiment 5 — Physical-feedback checkpoints and bounded capacity-cone replanning

### Failure

The global planner could discover a substantially more physically realizable allocation and then
discard it because its provisional scalar overflow was higher.  On the two-iteration `bm_svf`
prefix, the initial checkpoint had aggregate overflow 146 and 14 failed regional spline windows.
A history-informed allocation had higher provisional overflow but only 9 failed windows and 27
regional spline recoveries.  Retaining the former would make subsequent work start from a worse
continuous/lane state.

The first feedback implementation also interpreted raw physical-medial nanometre deficit as a
PathFinder history multiplier.  A 0.63 mm deficit was therefore charged as 630,000 generic cost
units rather than 13 configured 0.05 mm clearance quanta.  That distorted the next allocation.

### Research

The tscircuit capacity solvers make diminishing free capacity more expensive, but intentionally
leave exact high-density routing for a later stage.  Their hypergraph work further separates
coarse region/port selection from a small local zero-obstacle solver.  Flux already has the
stronger equivalents: NavMesh topology/corridor geometry, physical-medial envelope capacity,
then regional lane/via/B-spline/PNS realization.  The transferable lesson is therefore feedback
discipline, not replacement of Flux's geometry with a rectangular capacity mesh.

Sources:

- https://raw.githubusercontent.com/tscircuit/tscircuit-autorouter/main/lib/solvers/CapacityPathingSolver/CapacityPathingSolver5.ts
- https://raw.githubusercontent.com/tscircuit/tscircuit-autorouter/main/lib/utils/getTunedTotalCapacity1.ts
- https://blog.autorouting.com/p/hypergraph-autorouting

### Change

- Normalize physical-medial over-capacity before it enters generic routability history using the
  configured clearance quantum.
- Publish an outer feedback ledger per probe: source type, affected-net count, reused/rerouted
  nets, overflow, shadows, lane-transition state, regional outcome, and the next repair seed.
- Compare planning candidates lexicographically as complete plan, connected topology, incomplete
  regional assignments, then provisional global overflow/cost.  No exact acceptance gate changed.
- Preserve a selected checkpoint and use `route_global_incremental()` only for a genuinely local
  cone.  A cone must leave at least 25% of routed net groups intact; otherwise Flux explicitly
  falls back to a whole-board global reallocation.
- Export an unaccepted packed-medial bridge as attributed diagnostic state so its two physical
  resources can expand a repair cone.  It does **not** contribute generic outer history because
  the dedicated PathFinder lane-transition history already owns that cost.  Coalescing keeps the
  two history domains separate.

This implements the useful capacity-router ideas at all relevant Flux representations:

- NavMesh portals remain topology-only; their carried corridors own capacity, preventing a
  duplicate raster occupancy model.
- Corridors and physical-medial resources receive the soft reserve preference from Experiment 1.
- Clearance shadows continue to attribute exact sub-grid incompatibility, with a reserve cost only
  for positive but fragile margin.
- Regional windows are Flux's local high-density/zero-obstacle solver and remain the only stage
  that decides lane order, via sites, terminal escapes, and continuous realization together.

### Measurements

With direct medial edges and exact selected-lane negotiation, the two-iteration / 50,000-expansion
Tier 1 prefix now selects the physically stronger history-informed checkpoint:

- 78 paths, 0 unrouted nets, and 0 unaccepted packed-lane transitions;
- 93 regional windows, 84 complete bundle assignments, 27 recovered spline windows, and 9 failed
  spline assignments (previously 72, 58, 15, and 14 respectively);
- 242 provisional global overflow with 198 clearance shadows.  This is intentionally not called
  legal copper: the 9 failed regional windows keep the result incomplete.

The probe took 104.43 s and 117,156 KiB peak RSS.  A deliberately narrow 23-net repair attempt
ran in 78.19 s, but it could retain only two of 25 net groups and did not improve physical
realizability.  The 25%-preserved-group rule therefore correctly chooses a full global feedback
wave for this board while retaining incremental replay for actually local cones.  The focused C++
suite remains 7/7, including explicit coverage of physical-unit normalization, non-duplicating
lane diagnostics, and broad-cone fallback.

Tier 0 was requalified after the preceding implementation: `pt2399_delay` materialized legal
copper with 234 accepted arcs, zero Flux validation errors, zero opens, zero Flux copper DRC
violations, and zero external KiCad DRC violations/unconnected items.  Fresh F.Cu/B.Cu PNGs were
written under `build/tier0-tscircuit-feedback-final-copper`.

### Status

Implemented and retained.  Tier 1 remains unlocked and incomplete; it must not be promoted until
the remaining nine regional windows materialize, connect, and pass whole-board exact DRC.

## Experiment 6 — Independent residual conflict-cone transactions

### Failure

Residual conflict windows were already decomposed into bounded cones, but retrying them was
needlessly all-or-nothing. If three independent cone transactions were constructed and one could
not yet be realized, Flux discarded the other two even when they had complete lane, via,
topology, and spline evidence. That held the regional checkpoint at nine failed windows and made
the retry a diagnostic only.

### Change

- Preserve the transitive source-window membership of each merged residual cone.
- Recompute one merged cone at a time, with fresh channel, terminal, via, lane-order, template,
  B-spline, and bounded PNS state.
- Replace only that cone's source assignments when its complete transaction strictly improves the
  feasibility-first regional objective. Other windows remain immutable.
- Keep retry work bounded by the existing cone and assignment limits. The regional replacement
  remains provisional; canonical copper, connectivity, and whole-board exact DRC remain required
  before board commit.

### Measurements

The same two-iteration / 50,000-expansion `bm_svf` prefix took 105.99 s and 117,804 KiB peak RSS.
It extracted three cones covering six source windows, recomputed three merged assignments, and
selected the legal improvements independently. Failed spline assignments fell from **9 to 6**;
the regional decomposition contracted from 93 to 91 windows. The remaining failures are now
explicitly concentrated in two mutual/quantized B.Cu regions (nets 12/24/5/8 and 1/14/7), two
related small quantization windows, and two topology-gate windows involving net 21.

This is still intentionally incomplete (`iteration_limit`, exit status 3): 78 paths are connected
in the coarse planner, but the result has 242 provisional overflow and 198 clearance-shadow
conflicts, so it cannot be materialized or promoted.

### Status

Implemented and retained. The next local work should target quantization-aware mutual spline
deformation and topology-gate alternatives inside those six residual windows, rather than
expanding global search breadth.

## Experiment 7 — Exact local deformation and topology diagnostics

### Change

- Extend adaptive two-spline PNS from a single 50/50 displacement to bounded symmetric,
  one-sided, and biased force splits. Every trial remains inside its selected topology corridor
  and is accepted only on the whole-window feasibility objective.
- Treat quantization as a joint local optimization boundary: after a continuous legal route is
  discretized, try a bounded family of two-spline force splits and retain only a strict reduction
  in exact quantized copper-envelope conflicts that is also continuously legal.
- Retain the precise NavMesh topology-check diagnostic with every B-spline realization and expose
  it for unresolved windows. This distinguishes a cell escape from a missing, repeated, reversed,
  or reordered directed portal crossing.
- When ordinary B-spline corner concentration cannot retain a topology certificate, attempt a
  compact before/corner/after control guard seed. It is still a continuous B-spline, not a
  polyline fallback or an unreviewed homotopy change.

### Measurements

On the fixed two-iteration `bm_svf` prefix, asymmetric PNS lowered retained quantized conflict
observations from 219 to 187 and legalized one adaptive window, with no material runtime change
(105.71 s, 118,324 KiB). The six failed windows did not yet reduce: the joint quantization pass
tested six exact candidates but found no strict legal improvement. The topology diagnostics show
that two routes are blocked by directed-gate behavior (one repeated gate and one missing gate),
not merely by leaving a free-space cell. The guarded control seed was not selected for this board,
so it adds no unmeasured route-quality claim.

A four-iteration global comparison is not a Tier 1 improvement: it selected a coarse-incomplete
141-overflow checkpoint with two unaccepted lane transitions and therefore correctly withheld
regional realization. The fixed two-iteration physical checkpoint remains the relevant diagnostic
baseline until the global allocation reaches coarse topology completion.

### Regression qualification

Tier 0 (`pt2399_delay`) requalified after these changes in 115.44 s / 50,336 KiB: Flux reported
zero validation errors, opens, and copper violations, with 234 accepted arcs and no unrounded
non-junction bends. External KiCad DRC reported zero violations and zero unconnected items.
Fresh copper PNGs are under `build/tier0-guarded-topology-copper`.

### Status

The effective retained Tier 1 improvement is independent residual-cone commitment (9 failed
regional assignments to 6). The new local solvers and diagnostics are active but have not yet
made Tier 1 complete; it remains unlocked and unpromoted.

## Experiment 8 — Regional-to-global feedback completion

### Failure

The routing-plan type allowed two upstream feedback retries, but normal CLI and integrated
optimizer entry points constructed it with a hard-coded value of two. The initial global/history
comparison consumed those passes, so six residual regional diagnoses could never influence a
subsequent global allocation.

### Change

- Make four bounded upstream retries the production default: two establish and compare physical
  checkpoints; two consume retained regional diagnoses in new global allocations.
- Remove the production hard-coded two-retry aggregate initializers. Test-specific retry budgets
  remain explicit.

### Measurements

The full five-probe `bm_svf` prefix completed in 180.07 s / 161,880 KiB. The first physical
checkpoint still had six incomplete regional assignments. The two extra feedback passes generated
coarse-incomplete alternatives (five then six unaccepted packed-lane transitions), so the
feasibility-first comparator retained the prior 78-path, topology-complete physical checkpoint
with 242 provisional overflow and 198 clearance shadows. It did not falsely claim an improvement.

### Status

Implemented and retained. This completes the intended global ↔ regional feedback loop, but Tier
1 remains incomplete. The next productive change must improve the actual constrained route
templates/layer allocation for the six surviving windows rather than run more equivalent passes.

Tier 0 was requalified once more through the production four-retry entry point: `pt2399_delay`
completed in 115.82 s / 50,752 KiB with zero Flux validation errors, opens, or copper violations;
234 accepted arcs; and zero external KiCad DRC violations or unconnected items. Fresh copper PNGs
are under `build/tier0-production-feedback-copper`.

## Experiment 9 — Residual-cone continuous-search diagnosis

### Failure

The retained Tier 1 checkpoint has six unresolved regional windows.  The highest-pressure pair,
windows 0 and 9, share a fragment and is correctly merged into one atomic residual cone: seven
fragments across four nets (`kicad-net-12`, `kicad-net-24`, `kicad-net-5`, and
`kicad-net-8`).  It remains incomplete despite an exact topology certificate.  Previously, the
only evidence was the final least-illegal spline realization, so it was unclear whether the
failure was a short shortlist, ownership leak, or a genuine lack of continuous freedom.

### Hypothesis

If the cone had a valid lane/template state outside the ordinary shortlist, evaluating a modestly
larger set of diverse complete joint states would expose it.  If all such states have the same
minimum physical failure profile, the next change must alter corridor/layer/via topology rather
than tune B-spline geometry or PNS force weights.

### Change

- Added compact per-cone telemetry: source windows, fragment/net counts, completeness,
  topology/mutual/quantized rejection counts, candidate count, and selection status.
- Retained per-window minima across the evaluated joint B-spline shortlist.  This is a bounded
  summary, not a second routing representation and not trial geometry copied into telemetry.
- Temporarily tested an eight-candidate residual-cone-only shortlist, then restored the normal
  four-candidate limit when it showed no improvement.
- Temporarily tested a same-side multi-spline bundle PNS.  It generated zero legal candidate
  moves because this cone has no second movable neighbour behind the blocking route; it was
  removed rather than retained as dormant heuristic code.

### Measurements

On the fixed two-iteration / 50,000-expansion `bm_svf` prefix, the residual retry constructs
three cones. Two are complete and selected. The unresolved 0/9 cone reports:

- 7 fragments, 4 nets, and 1 recomputed atomic assignment;
- zero topology rejections;
- at least 1 mutual-clearance and 2 quantized-clearance rejections in every retained candidate;
- 0 legal candidates out of 4. The temporary 8-candidate run also found 0 legal candidates and
  the same minimum physical profile.

The coarse checkpoint remains 78 connected paths with zero unaccepted packed-lane transitions,
242 provisional overflow, 198 clearance-shadow conflicts, 91 regional windows, and six failed
regional assignments. No exact-DRC claim is made. Focused routing and telemetry tests remain
green.

### Interpretation

The cone is already joint in the correct ownership sense, and its B-splines preserve the selected
NavMesh homotopy. The blocker is not a missing pair or bundle shove: every bounded continuous
choice still needs one route-to-route separation and then fails quantization twice. This is
evidence for a missing *discrete* local alternative—different corridor side, layer span/via
region, or terminal/fragment run-up—not a reason to relax clearance or add cosmetic curve work.

### Decision

Keep the diagnostics. Reject the wider continuous shortlist and same-side bundle PNS. The next
experiment should generate a bounded topology-changing route template for the diagnosed physical
pair and compare it with an external routed topology before expanding global search.

## Experiment 10 — KiCad-correct differential-pair identity and early regional handoff

### Failure

`bm_svf` imported one apparent differential pair from the autogenerated analogue names
`Net-(U5A-+)` and `Net-(U5A--)`. Flux's suffix scanner stepped past the closing `)` and treated
it as transparent. KiCad's [`BOARD::MatchDpSuffix`](https://raw.githubusercontent.com/KiCad/kicad-source-mirror/master/pcbnew/board.cpp)
instead stops at the first non-suffix character. The false atom coupled unrelated global paths,
and then blocked an otherwise bounded residual alternate-layer state at the coupled-template gate.

### Change

- Match KiCad exactly: only trailing digits and underscores may follow a `P`/`N` or `+`/`-`
  differential suffix; any other character terminates matching.
- Add a focused import regression that keeps `DATA_P`/`DATA_N` paired while proving that the two
  autogenerated `Net-(...)` labels stay independent.
- Permit regional conflict analysis once every coarse connection has a route, even when an
  optional packed-medial lane bridge is unaccepted. That bridge remains a hard final
  materialization/connectivity/DRC gate. The relaxation supplies regional lane/via/spline
  diagnoses to the global feedback loop; it does not authorize copper.

### Measurements

With the fixed two-iteration / 50,000-expansion `bm_svf` profile, correcting only the false pair
reduced the retained global checkpoint from **242 to 131** provisional overflow and from **198 to
113** clearance-shadow conflicts. Runtime fell from **181.55 s to 130.53 s** and peak RSS from
161,584 KiB to 135,436 KiB.

The earlier handoff then analysed the remaining physical state rather than deferring it: 67
eligible regional windows, 63 complete bundle assignments, 216 joint B-spline candidates, and
140 legal candidates. Four spline windows remain unresolved and the global plan still has two
unaccepted packed-medial transitions, so Flux correctly reports `iteration_limit` and does not
materialize or claim DRC success. This richer diagnostic pass took 290.72 s / 143,788 KiB.

### Decision

Retain both changes. The pair correction removes an invalid KiCad interpretation and materially
improves the global allocation. The handoff implements the intended global-to-regional feedback
boundary while preserving the final strict gate. The next Tier 1 work is now narrow: repair the
four reported regional spline windows and resolve the two packed-medial bridge transitions.

Tier 0 was requalified after these changes in 154.02 s / 280,460 KiB. `pt2399_delay` again
completed 50 paths with accepted copper and external KiCad DRC clean; final layer PNGs are under
`build/tier0-dp-suffix-requal/tier-0-pt2399_delay/copper`.

## Experiment 11 — Gate-preserving regional B-spline initialization

### Failure

Three of the four remaining regional failures were geometrically clear but rejected because the
B-spline tessellation did not cross its selected directed NavMesh gate sequence. The seed
string-puller checked only containment in the union of corridor cells, so it could remain in free
space while silently bypassing a required portal. Its guarded fallback was then constructed from
that already topology-losing shortcut.

### Change

- Require every string-pull shortcut, together with its unchanged prefix and suffix, to preserve
  the complete directed gate certificate.
- Retain the raw gate-aware corridor anchors as the guarded fallback source.
- Try a bounded deterministic control-point multiplicity progression and retain exact adaptive
  spline topology checking as the acceptance gate.
- Concatenate and validate per-fragment topology certificates before merging a continuous run;
  cell-only legacy fragments retain their existing containment guard without inventing a gate
  claim.

### Measurements

On the fixed `bm_svf` profile, complete regional bundle assignments increased from **63 to 65**,
failed spline assignments fell from **4 to 2**, and legal joint spline candidates increased from
**140 to 148** out of the unchanged 216 evaluations. The global checkpoint remained identical at
78 connected paths, 131 provisional overflow, 16 sampled overflow, 113 clearance-shadow
conflicts, and two unaccepted packed-medial transitions.

### Decision

Retain. This removes two false topology failures without changing allocation or weakening any
copper gate. The two surviving windows are now one genuine directed-gate realization and one
fixed-endpoint physical collision.

## Experiment 12 — Residual endpoint-stub expansion and run-level spline detours

### Failure

The simple residual window 34 contains two different-net fragments whose fixed regional endpoints
coincide. The window owns only one fragment, so neither a same-layer detour nor elastic spline/PNS
movement can separate the endpoint. The owner-closure retry imported the adjacent window but still
cut both route runs at the collision.

### Change

- When a residual conflict reaches an ordinary, same-run fragment endpoint, absorb at most one
  adjacent fragment on that affected side and move the fixed regional boundary outward.
- Keep expansion inside the existing fragment/net/window limits; coupled and explicit-width runs
  remain ineligible until they can expand atomically with their governed state.
- Allow contiguous owned alternative-corridor fragments to merge with their adjacent stubs.
- Build the merged spline seed from every member's corridor anchors, concatenate all available
  directed gate certificates, and accept the merge only when the complete certificate remains
  valid. Cell-only members retain conservative containment checking.
- Export stub and merge counts through CLI and live regional telemetry.

### Measurements

The fixed profile absorbed **3** stubs into the two-window residual cone, expanding it from 3 to 6
fragments. Its mutual-clearance failures fell from **2 to 1**. Four bounded candidate states
performed **5** run merges covering **15** fragment realizations, proving the alternative corridor
can now use the expanded run. The cone still has zero exact-clean candidates: its best state has
one continuous mutual-clearance failure and two post-quantization failures. The ordinary
checkpoint therefore remains authoritative and unchanged.

### Decision

Retain the bounded expansion and topology-certified run merging: they produce measurable physical
improvement and are required to make an endpoint conflict deformable. Add compact failed-fragment
attribution before choosing the next layer/corridor repair; do not broaden the cone speculatively.

Follow-up attribution showed that the remaining continuous failure was `kicad-net-16`, connection
1/run 1/fragment 13 with `merged_fragments = 1`; the opposing `kicad-net-3` failure began at
fragment 2 with `merged_fragments = 3`. This isolated an ordering defect rather than insufficient
PNS force: scarcity-ordered CSP templates could present fragment 13 before its absorbed fragment
12, and the forward-only merger consumed it before the contiguous run could form. Regional routes
are now canonicalized by stable fragment ID before run assembly. The fixed-profile run showed no
change: fragment 13 still had no compatible adjacent discrete template. A follow-up experiment
that spline-realized every member before stitching also produced the identical rejection profile
while adding runtime, so it was removed. The evidence now calls for one run-level discrete
topology template, not further continuous preconditioning.

## Experiment 13 — Mandatory future demand on physical medial resources

### Failure

Flux's residual-capacity lookahead protected sampled free-space corridors, but its shared medial
resources considered only current occupancy, reserve margin, and accumulated history. A route
could therefore consume the last physically usable width of a sub-grid cross-section even when
every static topology for a still-unrouted net required that same resource. This left the most
important Tier 1 failure mode—clearance-shadow conflicts—less informed than the coarse corridor
model.

### Research

The MIT-licensed [tscircuit autorouter](https://github.com/tscircuit/tscircuit-autorouter) uses an
adaptive capacity mesh, penalizes diminishing remaining capacity, reserves route targets and via
opportunities, and distributes detailed route ports across shared region boundaries. The useful
principle for Flux is forward-looking capacity allocation followed by a separate high-density
realizer. Its rectangular quadtree, scalar tuned-via capacity, and axis-aligned obstacle model are
not imported: Flux already carries exact KiCad rules, clearance-expanded polygonal free space,
NavMesh topology signatures, physical medial widths, ordered regional lanes, and transactional
DRC.

### Change

- Extend each net's static mandatory-resource intersection with shared medial resource IDs and
  the minimum unavoidable resolved `track width + 2 * clearance` demand in nanometres.
- Sum only pending nets' mandatory physical demand during iteration-zero staged allocation and
  each negotiated PathFinder task.
- When a candidate would leave less width than that future demand, charge a soft deficit cost
  normalized by the graph's minimum physical envelope. Existing temporary overflow, history,
  rip-up, exact topology, and DRC behavior remains unchanged.
- Publish mandatory physical net/resource/demand totals and selected-path penalty totals in CLI
  JSON for the benchmark.

### NavMesh, corridor, medial, and shadow decisions

- NavMesh portals remain topology certificates rather than duplicate occupancy resources. Their
  selected corridor and medial cross-section carry capacity, avoiding double charging an
  artificial cell boundary.
- Free-space corridors retain their existing mandatory future-lane lookahead and critical-cut
  pressure.
- Shared medial resources were the only representation changed by this experiment.
- Clearance shadows retain exact attributed conflict/history plus the existing positive-margin
  reserve. A generic proximity field was rejected because Flux's terminal reservation policy
  intentionally charges another net only when it would eliminate the last viable escape.

### Measurement

The fixed `bm_svf` profile proved that the signal was active: **23** nets retained **186**
mandatory physical resources with **120.9 mm** aggregate envelope demand, and six selected
transitions paid **3,600** cost. It removed both unaccepted medial lane transitions, but the
feasibility-first result regressed:

- total overflow increased **131 → 142**;
- sampled overflow increased **16 → 21**;
- clearance-shadow conflicts increased **113 → 120**;
- failed regional spline assignments increased **2 → 8**;
- legal regional candidates decreased **148/216 → 145/213**.

The run took 383.32 seconds and 157,020 KiB peak RSS. It remained incomplete and made no DRC
claim.

### Decision

Remove the physical-demand penalty and its derived state. The static portfolio's shared-medial
labels are not sufficiently strong proof that a resource is physically unavoidable: intersecting
two bounded trees can preserve a common flooded label even when later continuous topology has
other options. The useful tscircuit transfer remains the already-retained normalized capacity
reserve on Flux's actual corridor/medial/shadow representations. A future lookahead must be based
on explicit corridor/lane templates or critical physical cuts, not common label membership.

## Experiment 14 — Corridor-constrained continuous B-spline optimization

### Failure

The regional optimizer sampled the B-spline densely, but its legality model was still local and
discrete. A curve could leave the selected NavMesh homotopy between samples, clip an obstacle
between samples, or acquire a loop/wiggle while improving a scalar objective. Conversely, the
optimizer's reciprocal obstacle field acted at unlimited range and could bend an already-clear
route for no physical reason. Control points were updated one coordinate at a time, so the search
did not behave like a coherent elastic band.

### Research and change

Retain the existing Flux pipeline and spline representation, but make its continuous stage carry
the selected geometry rather than reconstructing topology from samples:

- Carry the selected convex NavMesh cells and checked portal strips into every detailed and
  regional `CubicBSplineRoute`. Project moving controls back into this corridor after every
  optimizer and shove proposal.
- Convert every exact cubic B-spline knot span to its equivalent cubic Bezier segment. Recursively
  certify its convex hull against the corridor union and use a 1-Lipschitz signed-clearance bound
  to prove the complete curve envelope. Fail closed when the subdivision or hull budget expires.
- Return the worst attributed parameter and obstacle normal. Use that witness for an EGO-style
  local rebound distributed by the spline basis functions.
- Replace independent coordinate descent with one joint control update. Analytical length,
  compact-support obstacle, anchor, second-difference, and third-difference gradients are
  preconditioned by a bounded Sobolev/CHOMP-style system and accepted through deterministic
  backtracking.
- Make obstacle repulsion compact: an obstacle outside the configured influence radius contributes
  no force. Keep loops, cusps, station reversal, excessive turn, bend radius, corridor escape, and
  continuous clearance as hard certificates rather than merely objective terms.
- Use the continuous certificate for final regional route-to-route clearance, with the tessellation
  deviation included in the opposing route envelope. Materialization, connectivity, copper DRC,
  and transactional fallback remain authoritative.

The high-risk regression constructs an off-centre obstacle between ordinary samples. The
continuous certificate detects the initial collision, the joint elastic optimizer rebounds inside
the selected convex corridor, and the final spline passes both the continuous envelope and shape
certificates. The existing obstacle-aware curved-route gate and telemetry viewer tests also pass.

### Full corpus measurements

The qualified Tier 0 `pt2399_delay` board still passes with **50 paths**, exact copper, and external
KiCad DRC in **30.44 s** (282,072 KiB peak RSS). Its final copper PNGs were exported under the run
artifact directory.

The full unlocked Tier 1 `bm_svf` run completed in **253.09 s** (156,272 KiB peak RSS). All **78**
paths were found, but the feasibility-first checkpoint remained incomplete with **207** aggregate
overflow/conflict units. Regional analysis produced **125** windows, **118** complete discrete
bundles, **384** joint spline candidates, **39** spline recoveries, and **97** completed joint
repairs. Across the live trace, **674 / 722** selected regional spline realizations were both
topology-legal and mutually legal. No complete board spline transaction was available because
seven regional bundles remained incomplete. The same run reported **zero complete via-dependent
routes**, zero planned layer runs, and zero planned via transitions.

The preceding full Tier 1 checkpoint retained 118 aggregate overflow/conflict units, 115 complete
bundles, and 101 completed joint repairs. The new continuous implementation therefore improved
local spline legality and complete-bundle count but did **not** improve the authoritative board
checkpoint; Tier 1 remains unqualified.

### Decision

Retain the continuous certificates, corridor ownership, compact obstacle field, and joint elastic
optimizer because they close concrete correctness gaps and recover regional candidates without
weakening any exact gate. Do not tune the spline into compensating for missing discrete state.
The next routing blocker is the incomplete regional lane/escape/layer/via assignment—especially
the absence of complete via-dependent plans—and the global feedback/selection that retained a
worse physical checkpoint. Those states must become complete before spline/PNS can commit a legal
whole-board transaction.

## Experiment 15 — Complete-leaf regional access and proactive via planning

### Failure

The Tier 1 spline run left seven regional bundles incomplete and reported no complete via-dependent
transaction. The regional state model already represented lane orders, terminal portals, layers,
via regions, and bounded sites, but three search defects prevented that state from being explored
as intended:

- exact terminal portal alternatives were generated but disabled by the production default;
- an existing global via received alternate legal sites only when its current site was already
  invalid, so a legal-but-blocking via could not move proactively;
- the full access CSP was solved at every partial template prefix. Those repeated prefix solves
  consumed the fixed access budget before many complete lane/layer combinations reached a leaf.

A fourth defect excluded pure lane-capacity failures from joint repair: the expansion queue
required an attributed via-pair conflict, so a physically overfull channel received a diagnosis
but no alternate-layer/corridor search.

### Change

- Enable the bounded terminal portal portfolio by default. Keep every portal exact static-checked
  and select it atomically with lanes and vias.
- Generate up to the existing fixed per-region bound of legal via alternatives even when the
  selected site remains legal. Preserve deterministic clearance/contention ordering.
- Run the access CSP only at complete bundle leaves. Retain partial via-region identity and channel
  capacity forward checks, but spend access states only on complete routing decisions.
- Increase the now leaf-only access budget from 512 to 2,048 states and the regional continuous
  beam from four to eight states. Both remain deterministic hard bounds.
- Require adjacent changed fragments of one route run to use one coherent alternate routing layer;
  a different internal layer would require an explicit via state rather than an implicit change.
- Before candidate materialization, join adjacent same-layer alternate fragments into one physical
  excursion when the intervening alternate-layer chord has an independent free-space clearance
  certificate. Retain only the outer entry/exit transitions, preventing redundant same-site
  return/outbound via stacks.
- Admit lane-capacity, terminal-access, and bounded-search failures to the same attributed joint
  alternate-layer/corridor repair queue previously reserved for via-pair conflicts.
- Publish every incomplete window's exact stop class and lane/access/via rejection profile through
  diagnostic telemetry.

All continuous geometry, whole-board connectivity, exact copper DRC, and transactional commit
requirements remain unchanged.

### Measurements

The existing high-risk global-to-detailed test still passes its relocated-via, two-transition
multilayer plan, exact materialization, connectivity, and DRC assertions. The qualified Tier 0
`pt2399_delay` board also remains complete with 50 paths and external KiCad DRC; fresh F.Cu/B.Cu
review PNGs were exported.

On the fixed one-pass `bm_svf` prefix, widening the physical beam and moving access checks to
complete leaves changed the selected regional result from **100 complete / 5 failed spline**
bundles to **109 complete / 3 failed spline** bundles. Access-budget exhaustions fell **6 -> 2**,
38 windows were recovered by the joint spline beam, and no selected assignment stopped for
missing topology, lane fit, ineligibility, or state exhaustion. Total overflow was essentially
unchanged at **207 -> 208**, so this was not a board-quality improvement.

The full 30-iteration Tier 1 run completed in **310.22 s** with 167,804 KiB peak RSS and the live
viewer attached. It found all 78 paths but retained 208 aggregate overflow/conflict units. The
selected regional checkpoint again contained 109 complete bundles and three continuous
realization failures. Across the run, 753 of 780 selected regional splines were both topology- and
mutual-clearance legal. A later rejected feedback candidate exposed one pure lane-capacity failure;
the corrected handoff now schedules that window for joint repair, though its bounded search found
no legal alternate template. Tier 1 remains unqualified and no complete board-wide multilayer
spline transaction exists, so complete via-dependent route, layer-run, and via-transition commit
counts correctly remain zero.

### Decision

Retain the regional planning changes: they remove discrete incompleteness from the authoritative
selected result and make terminal/via choices genuinely negotiable without weakening legality.
Do not describe the board as routed. The remaining selected blockers are continuous topology/PNS
realization in three windows plus the global checkpoint's 208 physical conflicts. Via-run
stitching is ready for the first complete changed-layer bundle, but cannot be exercised by an
authoritative board candidate until those continuous windows are legal.

## Experiment 16 — Residual owner closure, topology-seed diagnosis, and feedback stagnation

### Failure

The full Tier 1 profile found every `bm_svf` connection but the selected physical checkpoint still
contained six incomplete regional bundles. Two windows contained only the directly conflicting
fragments even though an endpoint collision was owned by the continuation of a neighbouring route
outside the original window. Four other failures were directed NavMesh-gate failures during
continuous realization. Repeated physical-feedback passes then spent most of the 45-minute budget
on checkpoints that were worse than the first physically evaluated topology.

The first retained checkpoint was:

- 78 global paths and zero unrouted nets;
- 132 aggregate overflow/conflict units (17 sampled, zero corridor, and 112 clearance-shadow
  conflicts, plus the remaining transition terms);
- 78 regional windows, 72 complete bundles, and six failed spline bundles;
- 467 bounded joint spline candidates, 265 legal candidates, and 26 windows recovered by a
  non-primary discrete state;
- zero complete board-wide via-dependent spline transactions.

The six incomplete windows were the smallest authoritative residual set. They involved windows
0, 30, 40, 63, 65, and 66. Windows 30 and 40 exposed frozen continuation ownership; the remaining
windows were dominated by missing or repeated directed NavMesh gates.

### Research

OpenROAD/TritonRoute separates pin access and track assignment from detailed search-and-repair,
then uses bounded worker regions and explicit escalation for stubborn violations. Conflict-Based
Search similarly keeps a low-level path planner but branches at a high level on the specific
agent conflict, while meta-agent variants merge repeatedly interacting agents. These support a
bounded conflict-directed regional state reserve rather than another global spline-objective
change.

Sources:

- OpenROAD detailed-routing flow and repair states:
  <https://github.com/The-OpenROAD-Project/OpenROAD/blob/master/src/drt/docs/RoutingFlow.md>
- OpenROAD detailed-router implementation:
  <https://github.com/The-OpenROAD-Project/OpenROAD/tree/master/src/drt>
- Conflict-Based Search and meta-agent conflict merging:
  <https://doi.org/10.1016/j.artint.2014.11.006>

### Change

- Import the owner of each residual window's endpoint continuation once, then apply ordinary
  transitive owner closure over the resulting physical fragments. Do not recursively manufacture
  endpoint stubs from every newly imported owner's unrelated conflicts; that expanded local cones
  into near-global windows and timed out.
- Stop physical-feedback search after two complete regional passes fail to improve the retained
  feasibility-first checkpoint. A worse checkpoint remains learning evidence but cannot cause an
  unlimited sequence of expensive physical realizations.
- Preserve the known-good topology-preserving cubic guard sequence: ordinary string pull, followed
  by raw topology anchors at bounded 250/125/62.5/31.25 micrometre corner guards, followed by the
  ordinary raw fallback.

Rejected variants were: widening PNS from 16 to 128 candidates (1,308 attempts, zero accepts),
doubling topology-guard attempts (15-minute prefix timeout), building guarded splines from the
simplified rather than raw seed (68/78 complete), a non-monotonic string pull (high-risk
mixed-width regression), and recursively expanding endpoint owners (unbounded local growth).

### Measurement

The standard unlocked Tier 1 run completed in **2,147.64 seconds** with the live diagnostic viewer
attached. The benchmark contract passed: all **78** paths were present, no net was unrouted, and
the selected checkpoint retained **132** overflow/conflict units. The first regional checkpoint
remained authoritative at **72 / 78** complete bundles. Subsequent checkpoints were inferior at
48/59, 63/73, and 112/125 complete bundles; after the latter two non-improvements the new
stagnation rule returned the best checkpoint rather than starting a fifth physical evaluation.

The trace is retained at
`build/regional-planning-final-v7/telemetry/bm_svf-global.jsonl`. It contains four complete
regional summaries and the provisional cubic B-splines observed by the live viewer. Tier 1 remains
incomplete and no final copper or DRC claim is made.

### Interpretation

The bounded owner closure is correct and the stagnation cutoff converts repeated inferior passes
into a finite search. The remaining topology telemetry exposes a separate defect: regional code
reconstructs its supposedly raw topology seed from `CubicBSplineRoute::corridor_anchors`, which
are the generic spline control polygon, rather than retaining the selected NavMesh funnel
centerline. A control polygon is not a directed-gate certificate and can omit a portal crossed by
the source route.

The run also confirms that the active spline shortlist is not an exhaustive route portfolio.
Discarding the remainder of the bounded joint pool prevents later physical diagnoses from
promoting a corridor, portal, layer, lane, or via state that was generated but pruned before its
relevance was known.

### Decision

Retain one-hop endpoint-owner import, ordinary owner closure, the known-good cubic seed guard
sequence, and the two-pass physical stagnation bound. Next, carry the immutable source NavMesh
centerline separately from optimizer control anchors. Preserve pruned joint states as a compact,
bounded fallback reserve and promote a targeted second wave only after spline/PNS failure
identifies the implicated decisions. Telemetry must distinguish generated, active, reserve,
promoted, and physically evaluated states.

## Experiment 17 — Immutable funnel seeds and conflict-directed fallback reserves

### Failure

Regional spline preparation called its fallback input a raw topology seed, but reconstructed it
from `CubicBSplineRoute::corridor_anchors`. Those anchors are the generic spline control polygon,
not the selected NavMesh funnel centerline. A control polygon can omit or repeat a directed portal
even when the source route proves the correct homotopy.

The joint discrete solver also inserted candidates by scalar `BundleObjective` and discarded any
later state with an equal objective. Consequently, different terminal portals, corridor
templates, layers, lane orders, and via sites could disappear merely because their aggregate cost
matched. After selecting the active spline beam, all remaining bounded states were destroyed, so
later topology/PNS diagnostics could not reconsider them.

### Change

- Carry an immutable `topology_seed` in each working regional spline. Populate it from the
  selected route template's NavMesh funnel centerline, concatenate it when same-run fragments
  merge, and use it for guarded topology-preserving cubic reconstruction. Keep control-space
  corridor anchors separate.
- Define duplicate joint states by full `RegionalBundleSelection` identity rather than equal
  scalar objective. The objective remains only the stable ordering key.
- Retain states outside the active beam as compact discrete fallbacks. They contain template,
  portal, layer, lane, and via decisions but no spline samples or copper.
- After an active spline/PNS failure, promote at most eight reserve states. A topology failure must
  change an implicated fragment template; a clearance failure may also change via assignment or
  lane packing. Previously legal windows remain frozen during this second wave.
- Publish generated, active, reserve, promoted, evaluated, and recovered counts. Each promotion
  is a semantic telemetry event with its failure class.

All promoted states remain provisional until continuous topology/clearance, quantized geometry,
whole-board connectivity, exact copper DRC, and transactional acceptance pass.

### Measurement

The 64-state-per-window reserve build passed all eight high-risk tests and the telemetry viewer
tests. Tier 0 `pt2399_delay` remained green with 50 paths, exact Flux copper, clean external KiCad
DRC, and fresh F.Cu/B.Cu PNGs.

The full unlocked Tier 1 run completed in **2,217.30 seconds** with a **411,059,840-byte**
diagnostic trace. It retained all 78 coarse paths, zero unrouted nets, and overflow 132. The first
physical checkpoint improved from **72 / 78** to **73 / 78** complete regional bundles:

- 1,034 distinct complete discrete states generated;
- 486 active states and 548 compact reserve states retained;
- 16 reserve states promoted for two diagnosed windows;
- 483 spline states physically evaluated;
- 28 windows recovered by non-primary states, up from 26;
- one window recovered specifically by the fallback wave.

The recovered window reduced the next incremental affected cone from 14 nets to seven. Later
checkpoints were inferior at 50/59, 65/74, and 110/116 complete bundles, so the feasibility-first
comparator and two-pass stagnation cutoff retained the improved first checkpoint. No board-wide
spline transaction exists yet, and complete via-dependent routes, planned layer runs, and planned
via transitions remain zero.

### Interpretation

The hypothesis is supported: preserving information that was already generated recovered one
authoritative residual window without weakening legality, changing global overflow, or causing a
material memory increase (observed resident memory remained around 130–138 MiB). The active beam
did not need to grow.

The 64-state reserve was still too narrow in dense later checkpoints. One pass generated 1,151
states but retained only 826 active-plus-reserve states; another generated 1,700 and retained
1,376. These counts are aggregate across windows, but they prove that bounded discrete states are
still being dropped before diagnosis.

### Decision

Retain the immutable funnel seed, full-state identity, compact reserve, targeted promotion, and
telemetry. Increase compact reserve capacity from 64 to 256 states per window and report every
capacity eviction explicitly. Keep active and promoted continuous beams at eight. Re-run Tier 0
and full Tier 1; retain the wider reserve only if memory remains bounded and the authoritative
checkpoint does not regress.

## Experiment 18 — Wider compact reserve and active-beam separation

### Measurement

The 256-state reserve build passed the high-risk and viewer tests and requalified Tier 0 with 50
paths, exact copper, clean external KiCad DRC, and fresh layer PNGs. The full Tier 1 run completed
in **2,442.77 seconds** with a **390,556,794-byte** diagnostic trace. Resident memory remained
about 137 MiB, close to the 64-state run.

The selected first checkpoint reproduced 73/78 complete bundles exactly and dropped zero states.
The denser second checkpoint retained 788 fallback states instead of 470 and reduced observed
capacity evictions to seven, while keeping physical evaluation at 382 candidates and producing
the same 50/59 result. The third checkpoint improved from 65/74 to 66/74 complete bundles, retained
1,057 fallbacks, and dropped 132 states. The final feedback problem shrank from 116 to 109 windows
and from 720 to 609 evaluated spline candidates, but finished at 102/109 complete bundles rather
than 110/116. Neither later state beat the selected checkpoint's five incomplete bundles.

All 78 coarse paths remained present, no net was unrouted, overflow remained 132, and no
board-wide spline/via transaction was available. Complete via-dependent route, planned layer-run,
and planned via-transition counts therefore remain zero.

### Interpretation

The larger compact reserve substantially reduces information loss without increasing spline
evaluation or resident memory. It also influences the ordinary active diversity beam because that
beam currently selects farthest states from the entire retained pool. This confounds two separate
questions: how wide normal routing should be, and how much late-stage fallback information should
survive.

### Decision

Keep the compact-reserve mechanism, but separate its capacity from active selection. Freeze the
ordinary diversity horizon at 72 complete states, retain up to 512 additional compact fallback
states, and continue to promote at most eight only after diagnosis. Report all evictions. This
should preserve the measured 64-state active routing behavior while retaining every state observed
in the 256-state benchmark's densest window. Re-run Tier 0 and Tier 1 before retaining the change.

## Experiment 19 — Separated complete-state fallback portfolio

### Change

- Freeze ordinary diversity selection to the first 72 objective-ordered complete states per
  window, independent of fallback capacity.
- Retain up to 512 additional compact discrete states per window. These preserve terminal portal,
  corridor topology, layer, lane order, and via choices without constructing spline samples or
  copper.
- Keep the normal and conflict-directed continuous beams at eight candidates. Promote reserve
  states only after a failed active wave identifies the implicated fragment and failure class.
- Remove only the promoted entries from the reserve. Carry every unpromoted state with the
  selected assignment so later residual and materialization phases retain bounded backtracking
  choices; expose the live remaining count separately from cumulative retention and promotion.
- Freeze successful windows during promotion and expose every generated, retained, evicted,
  promoted, evaluated, and recovered state through telemetry.

### Measurement

All high-risk tests and the telemetry viewer tests passed. Tier 0 `pt2399_delay` remained green
with 50 paths, exact Flux copper, clean external KiCad DRC, and fresh F.Cu/B.Cu review PNGs. After
the reserve-lifecycle correction, the high-risk tests and Tier 0 were run again with the same clean
result; current review images are under `build/regional-planning-final-v11/artifacts/`.

The full unlocked Tier 1 run completed in **2,222.84 seconds** with the running viewer attached.
All **78** global paths were present, no net was unrouted, and the selected checkpoint retained
overflow 132. Its first physical checkpoint reproduced **73 / 78** complete bundles and remained
authoritative. The four regional evaluations retained 548, 795, 1,189, and 958 compact fallback
states respectively and reported **zero evictions**. Conflict-directed waves promoted 16, 40, 16,
and 16 states and recovered one, two, one, and one windows specifically from the fallback reserve.

The later checkpoints remained inferior at 50/59, 66/74, and 102/109 complete bundles. No
board-wide spline transaction exists yet, and complete via-dependent routes, planned layer runs,
and planned via transitions remain zero. The 404,286,327-byte diagnostic trace is retained at
`build/regional-planning-final-v10/telemetry/bm_svf-global.jsonl`; observed peak resident memory
was about 147 MiB.

### Interpretation

Separating the active horizon removed the confound introduced by the 256-state experiment: normal
routing reproduced the established 72-state behavior while the wider reserve preserved every
complete state generated by this benchmark. The promoted reserve repeatedly recovered windows,
so these alternatives are operational fallbacks rather than telemetry-only records.

The remaining five incomplete windows are therefore no longer explained by pruning of already
generated complete assignments. Their discrete portfolios do not contain a legal combination.
The next implementation must generate new conflict-directed terminal escape, homotopy, layer, and
via alternatives for the implicated fragments instead of widening this reserve again or tuning the
spline objective.

### Decision

Retain the 72-state active horizon, 512-state compact fallback reserve, eight-state promotion
wave, full-state identity, immutable funnel seed, and exact transactional gates. Do not increase
the reserve further for Tier 1: it has zero observed evictions. Next expand discrete alternative
generation only for failed regional fragments, preserving the current portfolio as backtracking
state across later regional and materialization phases.

## Experiment 20 — Per-layer hydraulic-erosion topology family

### Change

- Adapt the batch-synchronous Vulkan compute and integer hydrology pattern from the local
  `LittlePlanets/terrain` reference into a bounded coarse-routing field.
- Treat the clearance-expanded invalid cells of each actual copper layer as infinite mountains;
  use the destination terminal or via portal as a zero-height sink.
- Relax a reverse integer gravity potential, dispatch deterministic droplet batches, atomically
  accumulate flow, and apply erosion only between batches. This makes batch accumulation
  commutative and gives later droplets a progressively stronger riverbed preference.
- Extract the highest-flow descending riverbed, open only a bounded tube around it in the prepared
  layer NavMesh, and reconstruct the candidate with the ordinary flood, funnel, topology, and
  exact chord certificates. Never accept shader cells as copper or graph authority.
- Preserve a deterministic CPU fallback for machines without Vulkan compute and expose the route
  family, layer coordinates, GPU usage, fallback count, relaxation work, and droplet work through
  result JSON and telemetry.

### Initial verification

The Vulkan shader compiled with `glslc`, the persistent Vulkan compute pipeline built, and the
focused obstacle/NavMesh test completed with a hydraulic proposal whose certified funnel retained
its directed topology signature. The same fixture compares CPU and GPU flow, erosion, and final
centerline bytes. Producer/viewer tests also reconstruct bounded layer heatmaps through live state
and timeline history.

Tier 0 `pt2399_delay` requalified on the final build with **50 paths**, exact Flux copper, clean
external KiCad DRC, and fresh F.Cu/B.Cu review PNGs under
`build/hydraulic-erosion-v3/artifacts/tier-0-pt2399_delay/copper/`. Its diagnostic trace contains
independent erosion fields for both copper layers; recovery snapshots coalesce them to the newest
field per layer while retaining every earlier field update in chronological replay. Full Tier 1
measurement remains in progress.

The full Tier 1 `bm_svf` probe then consumed its complete **2,700 second** allowance and timed out
during the second regional feedback pass at assignment **217 / 362**. The 459,920,413-byte trace is
`build/hydraulic-erosion-v3/telemetry/bm_svf-global.jsonl`; it recorded 5,949 produced/serialized
events, 2,037 coalesced state updates, and **zero dropped events**. Eighty incremental heatmap
events covered F.Cu and B.Cu, and retained shortest, diversified, RRT-Connect, hydraulic-erosion,
and river families were all observed.

The best published full-net global checkpoint connected 25/25 nets but retained 20 sampled
conflicts, one corridor overflow, 1,191,862 nm physical overflow, and 128 clearance-shadow
conflicts at cost 567,831. This is worse on feasibility than the preceding v12 baseline (17 sampled,
813,012 nm physical, 112 shadows), despite a slightly lower scalar cost, so the new portfolio has
not yet improved Tier 1's selected global allocation. A later six-net feedback wave reached nine
sampled conflicts and 836,410 nm physical overflow locally but left two of those six tasks
unrouted, so feasibility-first selection correctly rejected it.

The completed first regional pass produced 74 windows, 61 complete bundles, 84 residual conflicts,
1,923 generated spline states, 55 promoted fallbacks, 38 ordinary recoveries, three fallback
recoveries, and 11 improved failed windows. It still produced no complete via-dependent route or
board-wide spline candidate. The next pass demonstrated the new generation paths before timeout:
terminal templates reached eight and via templates reached 35. One residual cone diagnosed an
exact `shove_deadlock`, rejected the partial transaction, promoted another eight topology states,
and continued into later cones.

### Decision

Retain this as a mandatory additional portfolio family. Evaluate every same-layer run against that
layer's independent obstacle/free-space domain and join layer-specific results solely through
explicit via planning. The exact graph reprojection, connectivity, quantized-copper, and KiCad DRC
gates remain authoritative.

The Tier 1 result does not justify giving hydraulic routes privileged selection weight. Keep them
as retained alternatives and preserve the exact fallback/telemetry implementation, but treat the
next optimization target as regional realization throughput and better feasibility ranking of
terminal/via-rich states. Do not advance the tier ladder while Tier 1 remains incomplete.

## Experiment 21 — Complete hydraulic heightmap and bank erosion

### Change

- Retain the complete reverse destination potential and the post-erosion terrain instead of
  discarding both after riverbed extraction.
- Carry a separate free-space coverage field so pads, vias, keepouts, and board exterior are
  excluded while all legal cells remain visible and reusable as terrain.
- Add bounded early downhill exploration and immediate-bank erosion. Exploration decays by batch;
  bank erosion widens the learned river without changing its exact extracted centerline.
- Carry normalized height/free-space/flow/erosion fields through NavMesh evidence, telemetry,
  snapshots, replay, and the live viewer.

### Isolated verification

The synthetic 256 by 160 fixture contains circular expanded pad/via obstacles, rectangular
keepouts, islands, and a constrained neck. Its final render is
`build/hydraulic-isolated/heightmap-gpu-v2.png`: terrain, erosion, flow, and composite panels from
left to right. All **33,117** legal free-space cells received finite terrain; obstacle cells were
excluded. Bank erosion increased non-zero erosion coverage from 691 to **1,160** cells, all 32,768
droplets reached the target, and CPU/Vulkan PPM files were byte-identical. The focused obstacle,
NavMesh, topology, telemetry, and viewer checks remained clean.

### Mandatory policy

Hydraulic erosion is no longer an optional global-router family. The public disable switch was
removed, its candidate and solver budgets are validated unconditionally, and the complete static
portfolio must reserve enough slots to reach at least one hydraulic candidate. Failure remains an
ordinary bounded candidate failure—never authority for route legality—but every connection now
attempts the family on every global seed run.

## Experiment 22 — Shared GPU routing-compute pass

### Change

One Vulkan/CPU advisory boundary now covers the six identified data-parallel workloads: batched
cubic B-spline position evaluation, clearance/wavefront fields, physical conflict discovery,
via/terminal candidate screening, portfolio/placement pre-scoring, and micromouse/NavMesh flood
families. Dense spline and segment work uses conservative relative-coordinate float broad phases
with a 1,024 nm guard, followed by the existing exact long-double checks for every possible
collision. Degenerate segments are handled explicitly so point-like via envelopes do not appear
to intersect every other point. Via span and layer filtering remains on the CPU before dispatch.

Dispatch thresholds keep small work on the CPU. Segment-pair buffers are capped at two million
pairs and disable the optimization, without losing candidates, if that bound is reached. The GPU
wavefront uses a geometric pass ceiling and falls back to the ordinary queue when a long detour is
not reached. Regional and placement scores affect only bounded evaluation order; stable proposal
indices, feasibility-first reduction, spline/corridor certification, exact copper, connectivity,
and KiCad DRC remain authoritative.

### Verification

The combined shader passed SPIR-V validation. An initial NVIDIA driver compiler crash exposed an
unnecessary 64-bit shader arithmetic dependency; replacing it with explicitly saturating 32-bit
advisory scoring removed that dependency. The focused obstacle/NavMesh/regional test then passed
CPU/GPU equivalence for crossing and degenerate point-via conflicts, a 64 by 64 obstructed
wavefront, a 128-sample cubic spline batch, and 64 candidate scores. The same core also builds with
Vulkan disabled, proving the deterministic CPU fallback remains complete.

The concurrent pre-change Tier 1 `bm_svf` run exhausted its approximately 45-minute allowance and
finished without a routed artifact. Its final published best checkpoint remained globally
connected but infeasible: 20 sampled conflicts, one corridor overflow, 1,191,862 nm physical
overflow, 128 clearance shadows, 84 regional conflicts, and cost 567,831. That trace is preserved
at `build/mandatory-hydraulic-tier1/telemetry/bm_svf-global.jsonl`; it is the comparison baseline,
not evidence for the new compute pass.

## Experiment 23 — Hydraulic discovery before coarse NavMesh projection

### Hypothesis

Using the prepared fixed-pitch NavMesh as both the hydraulic terrain and the resource graph loses
free-space structure before erosion can discover it. Rasterizing the clearance-expanded polygonal
domain at a finer bounded pitch should let the hydraulic field choose obstacle side and broad
valley first, while retaining the NavMesh only as the coarse occupancy/resource projection.

### Change

- Rasterize each prepared polygonal free-space region directly at half the coarse cell pitch,
  bounded by one million discovery cells.
- Bind terminal endpoints through exact-certified local chords; forbid diagonal corner cutting.
- Compute an obstacle-clearance field and add a bounded clearance reward to droplet and riverbed
  selection so equal-potential routes prefer physical margin.
- Exact-certify and visibility-pull the discovered river in polygon space before sampling it into
  coarse NavMesh cells. Open only a bounded tube around those projected cells, then retain the
  existing flood, funnel, graph reprojection, connectivity, and DRC gates.
- Publish fine-field origin/pitch, discovery-cell count, certified-chord count, and projection
  failures separately from coarse NavMesh evidence.
- Require Vulkan plus `glslc` at configuration and a successful Vulkan compute startup probe at
  routing time. Preserve CPU hydraulic code only as an explicit deterministic test/oracle path;
  requested GPU work no longer silently falls back after a device/dispatch failure.

### Initial verification

The isolated 256 by 160 terrain completed on Vulkan with all **33,117** legal cells assigned finite
height, **1,167** eroded cells, a **233**-cell riverbed, and all **32,768** droplets reaching the
target. Its CPU oracle and Vulkan output were byte-identical before the production fail-closed
policy was enabled. The mandatory-GPU release configuration and shader build completed against
Vulkan 1.4.341. A dedicated polygon-discovery test then passed the complete boundary: fine
per-layer raster, GPU/CPU field equivalence, exact river-chord certification, topology signature,
and coarse NavMesh projection.

An initial focused regional run failed after temporarily increasing the default portfolio from
four to five candidates, so the portfolio width was restored to keep this experiment independent
of routing-order policy. The repeated established crossing/via fixture still stops later in its
regional PH realization: both retained routes are topology-, mutual-clearance-, and
post-quantization-legal, but the complete candidate fails the separate PH shape certificate. That
downstream curve-shape frontier predates and does not exercise the hydraulic block, which occurs
later in the broad fixture; it remains an explicit unresolved verification failure rather than
being weakened or relabelled as an erosion success.

### Decision

Retain hydraulic erosion as the free-space topology discoverer and treat the fixed NavMesh as a
coarse resource/certificate projection. Neither representation authorizes copper: exact chord
certificates, regional realization, quantization, connectivity, and DRC remain authoritative.
The next measured tier comparison should examine discovery-to-projection failures independently
from downstream regional packing failures.

## Experiment 24 — Broad regional PH certificate repair

### Failure decomposition

The established broad obstacle/regional fixture exposed four independent failures in sequence:

- a multi-corner regional PH chain was rejected only because one fixed whole-chain excess-turn
  allowance accumulated no allowance for its many intended anchor turns;
- topology guards inserted around funnel corners were immediately removed by the ordinary
  forward-collinear polyline canonicalizer, allowing a terminal PH span to miss a selected gate by
  approximately 0.13 mm;
- adaptive node insertion evaluated the old curve at the new node but then solved two new PH
  Hermite problems, allowing either half to select a different algebraic branch; and
- detailed and regional routing had separate guard constructors. The detailed copy retained the
  sharp apex and then canonicalized its guards, leaving all 12 seed candidates outside the
  continuous corridor certificate and expanding one initial sampled violation into 1,091 final
  violations.

### Change

- Scale the permitted absolute-rotation surplus by the number of intentional non-collinear anchor
  turns. Self-intersections, cusps, control/sample station reversals, unintended inflections,
  minimum bend radius, rational-offset intersections, topology, quantization, connectivity, and
  DRC remain hard independent gates.
- Preserve explicit topology-guard nodes instead of sending them back through ordinary polyline
  canonicalization. Regional seeds retain the apex as an exact obstacle-side witness. Detailed
  corner realization instead trims the incident spans and bridges the two guards without forcing
  a C1 curve through the sharp apex.
- Replace adaptive PH re-interpolation with exact de Casteljau subdivision of the quadratic
  preimage. For split parameter `u`, use `sqrt(u) w(us)` and
  `sqrt(1-u) w(u + (1-u)s)` for the two local preimages. Rebuild each segment's cached Bézier,
  power-basis, speed, arc-length, curvature, extrema, and bounds data once.
- Route both the regional and detailed constructors through the shared PH implementation, while
  keeping their distinct apex-retention policies explicit.
- Carry shape-certificate diagnostics and interpolation parameters through regional results and
  CLI JSON so topology, mutual-copper, quantization, and curve-shape rejection can be distinguished.

The `mitxela/kicad-round-tracks` reference confirmed the useful local construction invariant:
shorten both incident tracks by one bounded amount, limit that amount by local segment length and
angle, and bridge the trimmed endpoints; junctions require separate handling. Flux retains that
geometric principle but continues to use PH trajectory optimization and exact transactional DRC
rather than adopting the plugin's post-process-only legality model.

### Verification

The obstructed curved-route case passed after the shared detailed constructor began omitting the
apex, with a legal seed available before expensive recovery. The complete
`obstacle_aware_diagonals_and_curves_remain_capacity_and_drc_gated` fixture then passed in its
480-second bound, including regional multilayer lane/layer/via selection, directed NavMesh gate
preservation, PH realization, native line/arc copper, PNS, connectivity, and represented DRC. The
focused inter-sample PH obstacle, PH geometry/rational-offset, and hydraulic polygon-discovery
checks also passed. The adaptive-node assertion now samples both sides of the inserted parameter
and requires the subdivided curve to remain within one nanometre of the original shape.

### Decision

Retain the repair. A topology seed and a finished rounded trajectory are deliberately different:
the former may retain a corner witness, while the latter must trim the apex. Adaptive refinement
must subdivide an existing PH curve exactly; changing interpolation branches is a separate bounded
search operation and must never be an accidental side effect of adding a control node.

## Experiment 25 — Clearance-boundary adaptive NavMesh with GPU sparse wavefront

### Hypothesis

A fixed rectilinear decomposition can discard a complete coarse cell when only one corner clips a
clearance boundary, disconnecting a physically usable narrow passage before hydraulic, funnel, or
regional routing can exploit it. Recursively certifying only those ambiguous boundary cells should
recover the passage without applying a uniformly fine raster to every route portfolio.

### Change

- Split each uncertified boundary cell into four deterministic children under explicit depth,
  minimum-pitch, accepted-cell, and search-expansion bounds. Interior rejected cells remain walls.
- Preserve a flat coarse-bin-to-leaf index and construct immutable exact shared-face adjacency,
  portals, and aperture widths for mixed-size cells.
- Use one subdivision level for global candidate portfolios and two inside bounded regional and
  detailed windows. The focused API remains configurable for deeper isolated searches.
- Route adaptive graphs using a compact weighted CSR wavefront. Large fields use Vulkan relaxation;
  if the bounded GPU wave does not reach the requested start, deterministic CPU Dijkstra completes
  the same graph. Exact polygon/chord certificates, topology guards, continuous geometry,
  quantization, connectivity, and DRC remain authoritative CPU gates.
- Export refined-parent, accepted/rejected-leaf, GPU-dispatch, and CPU-completion counts through
  global and detailed result JSON.

### Verification

The isolated 10 mm square fixture contains a 1.2 mm passage that a 2 mm fixed mesh reports as
disconnected. Four refinement levels at a 0.125 mm floor recover a legal mixed-size corridor, exact
portals, guard containment, and a valid directed topology signature. A 4,096-node weighted CSR
fixture produces byte-identical GPU and CPU distance fields. Hydraulic projection and inter-sample
PH obstacle certification also pass independently.

After a clean release rebuild, the broad obstacle/regional/copper fixture passes. A diagnostic
Tier 0 `pt2399_delay` run reaches 20/20 globally allocated nets and zero overflow after six
PathFinder iterations. Detailed routing attempts 44 PH runs, accepts 43 route-level replacements,
dispatches seven adaptive graph wavefronts on the GPU, and retains zero copper-clearance or open-net
violations. The authoritative run still fails its separate curved-output quality gate because 13
non-junction direction changes remain without PH realization. Three detailed NavMesh seeds fail
(one disconnected and two conservative funnel/string-pull failures), and one otherwise legal PH
replacement is transactionally rejected for connectivity. Eight non-authoritative near-result
PNGs and the complete telemetry trace are retained under
`build/tiers-check-20260903-adaptive-navmesh-depth1/`.

A separate one-iteration global measurement prepared 196 route/domain combinations into
2,113,748 cumulative leaf cells. It refined 253,872 boundary parents, retained 214,364 refined
leaves, rejected 932,444 terminal leaves, and dispatched all 196 selected NavMesh wavefront routes
on Vulkan without CPU completion. It connected every net but, as expected for the bounded first
pass, stopped with six overflow resources.

### Decision

Retain adaptive clearance-boundary subdivision and the weighted GPU graph wavefront. Do not deepen
the global default: two and three global levels raised preparation memory and runtime sharply while
adding resolution before candidate topology was narrowed. Use deeper refinement regionally, and
continue to fail closed when the funnel, exact chord, connectivity, or curved-output gates reject a
candidate.

## Experiment 26 — Complete PH realization across heterogeneous clearance and branch topology

### Failure decomposition

The 13 remaining Tier-0 bends did not share one curve-fitting cause. Detailed inspection separated
three representation mismatches:

- detailed free-space preparation used the largest resolved board clearance for every obstacle,
  erasing valid corridors beside low-clearance nets even though exact pairwise DRC allowed them;
- a same-net stub could terminate exactly in the interior of an unsplit carrier segment, so
  smoothing the carrier's adjacent corner pulled copper away from an electrical T that was absent
  from the endpoint graph; and
- the PH line/arc fitter applied its bounded arc look-ahead to straight fitting as well, forcing a
  long tolerance-flat PH span into several slightly different straight chords. The final audit also
  omitted explicit pad/via incidences even though chain construction correctly treated them as
  fixed electrical attachments.

### Change

- Preserve aligned obstacle net ownership during board free-space extraction. Expand board edges
  by trace radius and expand each represented copper obstacle by trace radius plus its exact
  pairwise clearance. Keepouts remain inflated by trace radius. Validate every resulting contour
  against the same per-object distance requirement.
- Add a bounded source-polyline corridor certificate as a fail-closed fallback when the general
  NavMesh cannot seed a route that already has exact legal source topology.
- Before PH chain collection, split unlocked same-net/same-layer carriers at exact interior
  centreline branch contacts. Generated piece IDs are deterministic, and near contacts are not
  promoted into topology.
- Let line fitting examine the complete bounded sample sequence while keeping the circular-arc
  candidate window independently bounded. Prefer a certified turning arc over a partial straight
  approximation so a PH curve cannot silently degrade back into a polygonal chain.
- Use one physical junction definition in PH collection and final quality auditing: exact pad/via
  attachments and additional same-net copper contacts are anchored junctions; an ordinary
  degree-two direction change remains an error regardless of angle.

### Verification

Focused PH power-basis/rational-offset, long tolerance-flat fitting, exact interior T-junction,
terminal-junction, heterogeneous-clearance, and adaptive-NavMesh checks pass in the release build.

The complete Tier-0 `pt2399_delay` run routes all 50 connections, commits every accepted PH
transaction, and reports zero unrounded non-junction bends. The exported board contains 45 native
KiCad track arcs, 117 straight segments, and 11 vias. Flux structural, connectivity, and copper
checks pass; KiCad reports zero DRC violations and zero unconnected items. Final F.Cu and B.Cu PNGs,
the routed board, external DRC report, diagnostic telemetry, and non-authoritative regional near
results are retained under `build/tiers-check-20260903-ph-complete-v6/`.

### Decision

Retain the complete repair. Clearance inflation must be pairwise rather than board-global, and PH
topology must be built from physical attachment points rather than only serialized segment
endpoints. Arc-search budgets must never fragment an otherwise valid straight PH span. Keep exact
connectivity, quantized copper clearance, and external KiCad DRC as the authoritative acceptance
gates.

## Experiment 27 — Tier-1 regional fallback, selected-state feedback, and XPBD PH contact

### Failure decomposition

The first valid `bm_svf` Tier-1 run exhausted its 2,700-second allowance after connecting every
terminal but repeatedly retaining topology-valid PH routes with one or two mutual copper-envelope
conflicts. Three distinct defects were visible in diagnostic telemetry:

- fallback diversity was retained but a residual cone consumed only one promoted wave;
- failure history was classified from rank zero even when reduction retained a different,
  feasibility-first least-illegal candidate; and
- a single-span PH route contributed only its two fixed endpoints to XPBD, while separate same-net
  fragments were incorrectly assigned distinct XPBD collision identities.

The full post-repair benchmark reached a second 307-candidate global topology and its residual
repair stage, but still timed out. One same-net-sensitive residual window improved from zero to four
PH realizations. The remaining different-net failures exposed a further representation mismatch:
XPBD could report zero violations for its interpolation-node chords while rational PH offset checks
still found two curve-envelope intersections.

### Change

- Allow three deterministic, re-ranked fallback waves only after a residual conflict cone has been
  spatially isolated; ordinary regional analysis retains one wave.
- Select the least-illegal candidate with a diagnosis-independent feasibility tuple, then classify
  and weight the selected state for routability history.
- Enable GPU XPBD relaxation and pressure feedback for the Tier-1 corpus board.
- Exactly subdivide a two-node PH preimage before XPBD so fixed endpoints no longer leave the
  elastic solver with zero movable degrees of freedom.
- Give all fragments of one electrical net the same XPBD collision group while retaining a
  separate fragment index for reconstruction and telemetry. This matches the exact regional
  clearance rule and preserves legal same-net T-junctions.
- Adaptively subdivide each exact PH preimage at the regional 0.02 mm curve tolerance, bounded to
  64 interpolation nodes per route, before generating GPU contact links. Reconstruction remains a
  PH-chain operation and every result still passes topology, shape, rational-offset, quantization,
  connectivity, and DRC gates.

### Verification

The release build and focused GPU XPBD, PH invariant/rational-offset, and physical T-junction tests
pass. The uninterrupted Tier-1 run timed out after 2,700 seconds and therefore did not qualify;
eight non-authoritative near-result copper and PH-control preview sets plus the full trace are under
`build/tiers-check-20260903-tier1-xpbd-net-groups/`.

In the midpoint integration run, a formerly fixed two-route case changed from four particles/two
links to six particles/four links, reduced four solver violations to one, and was accepted for
exact PH evaluation. In the same-net run, residual window 23 produced four realizations instead of
none. A bounded post-run smoke check confirmed that adaptive PH subdivision supplies 119–122 GPU
particles in a curved multi-route window rather than only its sparse interpolation-node chords;
invalid deformations remained rejected. That final adaptive-contact change has not yet completed a
full Tier-1 qualification run.

### Decision

Retain the selected-state diagnosis, residual fallback waves, movable single-span PH subdivision,
and same-net XPBD grouping. Retain adaptive PH contact subdivision behind its explicit per-route
bound and re-run the complete Tier-1 benchmark before promoting `bm_svf` to passing. The primary
remaining performance target is deterministic route/window-level parallelism for recursive exact
PH envelope checks; the Vulkan XPBD solve itself is millisecond-scale and is not the wall-clock
bottleneck.

## Experiment 28 — KRT-inspired MPS allocation and large-neighborhood recovery

### Hypothesis

The remaining Tier-1 sampled conflicts are partly a consequence of independently attractive seed
trees consuming scarce crossing, terminal, and via resources. A deterministic maximal-planar-
subset-style seed order, bounded physical alternative rescoring, soft-capacity blocker probes,
progressive rip-up, clearance-envelope ghost memory, and graded terminal proximity should retain
complete connectivity while exposing a lower-overflow checkpoint to regional realization.

### Change

- Retain the terminal-MST crossing graph and build deterministic maximal non-crossing rounds.
  Evaluate forward and reverse MPS schedules as additional occupancy-aware iteration-zero
  portfolios, retaining explicit-priority and coupled-group order. Publish conflict degree and
  round with every routing-priority telemetry record.
- On dense boards, score each retained layer/stub/via tree against already selected same-layer
  crossings and via-clearance-node contention. This selects among bounded, already validated
  route/via alternatives; it does not authorize a new via or copper geometry.
- When hard capacity prevents a connection, run one bounded, non-committing soft-capacity search
  and attribute every crossed sampled, shadow, corridor, neck-down, and medial resource to its
  routed owner. Feed the resulting min-cut-style blocker hint into normal victim selection.
- Expand repeated hard-capacity repairs from the direct blocker through progressively larger,
  non-repeated victim sets. Preserve transactional rollback and the existing per-pass attempt and
  conflict-set state bounds.
- Rasterize each ripped route's complete width-plus-clearance track and via-span envelope into
  pass-local ghost nodes and edges. Penalize, but never prohibit, an immediate takeover of that
  space while the victim is rebuilt.
- Add a graded terminal-proximity cost before the existing last-escape reservation and targeted
  same-layer spreading only during hard-capacity recovery outside represented multi-lane
  corridors. Tier 1 enables the proximity term explicitly at 500 cost units.
- Restrict physical alternative rescoring and MPS portfolios to boards with more than 20 routing
  tasks. The first bundled run applied the new crossing/via tie-break to Tier 0 and displaced its
  stable small-board seed without a packing benefit; the measured scale guard restores the prior
  Tier-0 global checkpoint.

### Verification

The development build, the focused transactional hard-capacity test, and the telemetry-viewer test
pass. The focused test now proves that a blocked hard-capacity search produces a successful
soft-cut probe with attributed blockers and that ripped-route memory contains physical-envelope
nodes as well as centreline edges.

The full 2,700-second Tier-1 `bm_svf` run did not qualify. Its initial selected checkpoint connected
all 25 nets with 12 sampled overflows, one corridor overflow, 2,921,704 nm physical overflow, and
100 clearance-shadow conflicts. A downstream feedback pass retained every affected connection
while improving the selected checkpoint to three sampled overflows, no corridor overflow,
544,100 nm physical overflow, and 43 clearance-shadow conflicts. This is stronger than the prior
three-overflow checkpoint, which left two connections unrouted, but it remains illegal. The run
emitted 50 MPS queue records, 304 bounded soft-cut diagnoses, 253 rip-up events, and nine regional
XPBD attempts. It timed out while exactly certifying regional assignment 47 of 124. Eight exported
near results are topology-clean but each retains one mutual-clearance failure.

The guarded Tier-0 rerun reproduced its prior global seed and its final feedback checkpoint reached
zero sampled, corridor, physical, and shadow overflow. Detailed materialization still failed the
current PH quality gate: 45 of 46 quintic PH runs were fitted, leaving nine non-junction direction
changes. Structural validation, connectivity, and copper DRC were otherwise clean (zero errors,
opens, and copper violations). Both tier runs exported eight non-authoritative copper/PH preview
sets under `build/tiers-check-20260903-krt-final/` and
`build/tiers-check-20260903-krt-2/` respectively.

### Decision

Retain the six bounded recovery mechanisms and the dense-board scale guard. They convert the
previous three-overflow/incomplete Tier-1 state into a three-overflow/connected state, but neither
tier currently qualifies. Do not unlock another tier. The next blocker is the regional/detail
boundary: exact PH certification is orders of magnitude slower than GPU XPBD, and the best
Tier-1 regional transactions are one mutual copper conflict from acceptance. Tier 0 separately
needs the remaining non-junction PH run realized without weakening its exact curve-quality gate.

## Experiment 29 — Batched regional PH realization and conservative offset screening

### Failure

The September 4 Tier-1 trace spent at least 1,760 seconds inside 257 spline-realization starts.
The first 218-assignment wave required 1,406.58 seconds, with individual gaps up to 300.29 seconds.
GPU XPBD was not responsible: 16 solves consumed 0.171 seconds in total. Code tracing identified
the earliest unbounded-looking decision as repeated degree-nine rational-offset intersection:
every interpolation trial rebuilt unchanged neighbour tessellations and signed offsets, then each
ambiguous curve pair could visit one million long-double subdivision nodes.

### Hypothesis

A complete interpolation portfolio can be sampled and ranked as one GPU bundle, while cached
neighbour geometry and fixed-depth rational convex hulls can eliminate separated offset pairs.
Only the best two approximate candidates need the existing topology, PH-shape, adaptive-clearance,
and exact-offset certificates. This should remove repeated CPU realization work without allowing a
float/GPU result to authorize copper.

### Change

- Cache adaptive tessellations, distance-keyed signed PH offsets, and pairwise envelope verdicts
  for unchanged routes, clearing only entries touching the changed route after a deformation.
- Evaluate each route's complete deterministic interpolation portfolio as one candidate-major
  bundle. The existing compute backend scores all same-layer route pairs in one dispatch and
  returns a deterministic top-two shortlist.
- CPU-certify only those two candidates with the unchanged exact topology, PH-shape, adaptive
  clearance, and rational-offset gates.
- Subdivide every positive-weight degree-nine offset into 16 outward-rounded convex-hull boxes.
  Batch those boxes through the shared GPU broad phase; no box overlap proves separation, while
  only ambiguous signed segment pairs reach exact long-double root isolation.
- Bound regional exact isolation at 50,000 nodes and fail closed on exhaustion. The public exact
  primitive retains its one-million-node default and now reports nodes, isolated leaves, maximum
  depth, and explicit budget exhaustion.
- Record GPU batches/fallbacks, candidates ranked/certified, geometry-cache hits/misses,
  broad-phase decisions, exact offset work, and microsecond timings in the structured CLI result.
  Wall-clock fields are deliberately excluded from semantic result equality.

### Verification

The development build passes. Focused PH invariant/rational-offset, advanced GPU-shortlist, and
same-net physical T-junction tests pass. New deterministic assertions cover conservative offset
hulls, subdivision work, and explicit node-budget exhaustion. On the existing crossing fixture,
four observed spline-realization calls each took approximately 1.55 seconds; interpolation exact
certification used about 0.14 seconds and offset broad phase about 0.24 seconds per call. All
candidate offset pairs were conservatively rejected before exact subdivision (zero exact nodes).
The fixture's later global-feedback portion remained long-running and was not used as a spline
timing result. A full Tier-1 qualification run remains outstanding.
A bounded one-iteration/50,000-expansion `bm_svf` integration probe consumed its 180-second limit
during global seed generation (335,440 KiB peak RSS), before reaching spline realization, so it is
recorded only as an inconclusive integration timeout rather than evidence for this experiment.
The mandatory Tier-0 `pt2399_delay` harness passes with 50 paths, zero structural errors, zero
opens, zero represented copper violations, and zero error-level KiCad 9.0.8 DRC findings. Final
copper and analysis artifacts are under `build/spline-gpu-batch-tier0/`.

### Decision

Retain pending the complete Tier-0/Tier-1 benchmark. The GPU remains advisory: topology, exact PH
shape, adaptive physical clearance, bounded fail-closed rational isolation, quantized copper, and
KiCad DRC remain CPU-authoritative. Compare the new counters on `bm_svf`; if exact-node exhaustion
dominates, improve the conservative hull hierarchy rather than increasing the exact budget.

## Experiment 30 — Hardware Simulation Board V2 two-hour routing attempt

### Failure

The requested external board contains 198 nets, 167 footprints, 26 imported layers, 58 existing
vias, and four zones. Flux's inspection boundary reports a lossy import with 415 errors, 34
warnings, and 175 core validation failures. Dominant blocking causes are 270 open geometry items,
an unsupported multiple-outline board, invalid pad dimensions, pads/placements outside the
represented board, placement overlaps, and malformed or unsupported courtyard geometry.

### Hypothesis

No routing algorithm or additional runtime can produce authoritative copper until the immutable
KiCad source can be represented strictly and losslessly. The correct experiment is therefore to
invoke both full detail and global-only routing under bounded execution and verify whether either
entry point reaches route search without relaxing import or DRC rules.

### Change

No router or source-board changes were made. The detail attempt used the current 30-iteration,
50,000-expansion, direct-medial, crossing-safe, atomic-tree, XPBD-enabled configuration with eight
seed workers, three final feedback passes, arc output, refillable zones, diagnostic telemetry, and
incomplete-preview capture. A 7,200-second external timeout bounded the command.

### Verification

The detail entry point rejected the board before routing after 15.34 seconds and 143,096 KiB peak
RSS: `detail-kicad requires a strict, lossless import`. The global-only entry point independently
returned the equivalent rejection after 15.38 seconds. No routed board, incomplete preview, or
route telemetry was created. KiCad 9.0.8 source DRC reports 696 violations and 408 unconnected
items, including 11 invalid-outline, 200 malformed-courtyard, seven clearance, and four
hole-clearance findings. Artifacts are under `build/hardware-sim-v2-2h-20260904/`.

### Decision

Treat this as an input-representation blocker, not a routing timeout. Preserve the source board
unchanged. Before another two-hour attempt, repair or deliberately normalize the board outline,
open/malformed geometry, invalid pads, and placement envelopes in a reviewable copy, then require
a strict lossless Flux import and rerun source KiCad DRC.

## Experiment 31 — Explicit lossy copper preview for an Altium-converted board

### Failure

The user identified Hardware Simulation Board V2 as an Altium-to-KiCad conversion and explicitly
requested visible routing copper without using DRC as an export gate. The ordinary incomplete
preview could not start because import fidelity and canonical validation were hard preconditions.
The earlier diagnostics are consistent with conversion damage: 199 KiCad padstack findings, 200
malformed courtyards, 270 Flux open-geometry errors, a multiple-outline projection, invalid pads,
and missing or multi-contour courtyards.

### Hypothesis

A tightly scoped diagnostic mode can preserve the qualified workflow while allowing route search
and source-preserving copper patching on a lossy projection. It must bypass only the initial
import/source-validation and final strict-round-trip suppression gates. Final validation,
connectivity, and copper DRC should still be computed and keep the result non-qualified.

### Change

- Add `detail-kicad --diagnostic-lossy-preview OUTPUT`; it implies incomplete-preview export and is
  unavailable to ordinary route, optimization, or live-apply commands.
- Allow the global and detailed routers to pass their initial source-validation gate only when the
  diagnostic flag is explicit. All geometric checks and final result classification remain active.
- Add a diagnostic KiCad export option that retains source revision/snapshot identity, supported
  design-change checks, syntax patching, and no-overwrite behavior while permitting a non-strict or
  structurally invalid projection. It emits `lossy_diagnostic_preview` and
  `invalid_diagnostic_preview` warnings and disables strict semantic round-trip qualification.
- Keep preview export non-authoritative: JSON uses workflow `diagnostic_lossy_preview_v1`, marks
  `artifact_preview_only`, and returns status 5 when ordinary completion gates fail.

### Verification

The development build and focused export-boundary unit test pass. The test proves that ordinary
export still rejects both a non-strict source and a structurally invalid source while explicit
diagnostic export produces bytes with warning evidence. Existing strict `detail-kicad` and
fail-closed incomplete-preview CLI regressions pass unchanged.

The requested direct-medial attempt reached routing but stopped before iteration 1 because shared
Voronoi allocation could not represent every physical obstacle. It returned status 5 after 64.50
seconds and 339,480 KiB peak RSS, writing a clearly marked diagnostic artifact with zero newly
added tracks/arcs/vias. This demonstrates export behavior but is not useful routed copper.

A sampled-grid fallback then reached real search over all 197 named nets. Epoch 1 selected a
checkpoint with 120 routed and 77 unrouted nets, 976 sampled overflows, six corridor overflows, and
2,686 clearance-shadow conflicts. It began a second feedback epoch with 1,174 learned history
entries. At the user's request it was stopped after 15:28.27 and 2,855,716 KiB peak RSS. The
current SIGINT path terminated before detailed materialization, so no second KiCad artifact was
written; 115 MiB of diagnostic telemetry preserves the attempt.

### Decision

Retain the explicit diagnostic preview facility because it is isolated, reversible, and keeps
qualified routing fail-closed. Do not interpret the HardwareSim artifact or telemetry as legal
copper. Do not restart this board until its Altium conversion artifacts are normalized. A separate
future improvement should make cancellation hand a retained global checkpoint to preview
materialization instead of exiting before export.

## Experiment 32 — Source-bound hole-clearance legality

### Failure

Milestone 2 retained KiCad's project `min_hole_clearance` value but used it only as metadata. The
exact DRC accepted different-net copper inside that drill-boundary clearance, the global router
ignored holes for planar steps, and the regional router considered only copper envelopes and
hole-to-hole spacing. Conversely, the existing hole-to-hole check incorrectly included milled oval
holes. This was a rule-fidelity failure upstream of any tier result: an internally clean route could
still fail KiCad's `hole_clearance` check.

### Hypothesis

A single exact drill representation carrying traversed layers and mechanical-drill eligibility can
make global/regional screening predict the same failure that whole-board and dynamic DRC enforce.
This should prevent avoidable late rejection without weakening ordinary clearance or final KiCad
authority.

### Research

KiCad's official custom-rule documentation defines `hole_clearance` as drilled-hole boundary to
different-net copper. It defines `hole_to_hole` only for mechanically drilled holes and explicitly
excludes laser microvias and milled oval holes. KiCad's official board-format documentation also
identifies `micro` and `blind` as explicit via types.

### Change

- Drill shapes now carry all traversed copper layers and mechanical-drill eligibility.
- Exact whole/dynamic DRC checks straight tracks, arcs, vias, supported pads, and saved fills
  against different-net drill boundaries.
- Global planar/via legality and regional run-up/via selection use the same shapes and project
  floor, including both directions of via copper/drill interaction.
- Milled oval holes no longer enter hole-to-hole checks. Laser-drilled microvias fail strict import
  until their fabrication type and rule set are canonical.

### Verification

The retained source-bound rule test proves same-net exemption, different-net whole/dynamic
rejection, exact oval geometry, and mechanical-hole filtering. The retained KiCad import/export
boundary test proves attributed microvia rejection. BM-SVF, CH32V003, Teensy, and ESC sources still
import strictly; BM-SVF, Teensy, and ESC retain zero canonical validation errors while CH32V003
retains its three pre-existing source findings. Qualified Tier 0 passes all 50 paths, exact copper,
and external KiCad DRC. Detailed runtime was 325.35 seconds versus 324.57 seconds in the retained
baseline (approximately +0.24%, within run noise). Diagnostic telemetry and mandatory global/detail
analysis PNGs are under `build/suite-per-change-20260904-140424/`.

### Decision

Keep. This closes a demonstrated acceptance mismatch and moves physical feedback earlier without
adding a parallel legality model. Focused KiCad rule-query agreement and custom/physical-hole rules
remain Milestone 2 work.

## Experiment 33 — Bounded PH reconstruction after local deformation

### Failure

The long BM-SVF Tier-1 run spent its post-global-routing time almost entirely in regional PH
realization. An eight-second root-privileged `perf` sample captured 798 cycles samples without loss;
771 sampled call chains passed through `rebuild_quintic_ph_route()` and segment reconstruction.
Symbolized leaf work was concentrated in long-double power-polynomial evaluation, recursive root
isolation, complex multiplication, and rotation evaluation. GPU XPBD and broadphase were not the
limiting stage. Inspection found two avoidable rebuild widths: attaching an immutable corridor
reconstructed a curve whose interpolation state had not changed, and exact polishing of one moved
interpolation node reconstructed every span in the chain.

### Hypothesis

PH geometry has local support even though its final certificate is whole-chain. Attaching corridor
metadata should preserve all prepared geometry. Moving one interpolation node can change only the
two adjacent chord lengths, the tangents at that node and its immediate neighbours, and therefore
at most four PH spans. Recomputing cheap normalized parameters and node derivatives globally,
reconstructing that bounded support, and then running the unchanged whole-chain certificates should
produce the same curve as a complete rebuild at substantially lower cost.

### Change

- Corridor attachment now validates a prepared-chain cardinality invariant without rebuilding the
  quadratic preimage, branch portfolio, power bases, extrema, or bounds.
- PH reconstruction separates cheap interpolation-state preparation and whole-chain join
  certification from exact per-segment reconstruction.
- Single-node trajectory candidates rebuild only segment indices `[node - 2, node + 1]`; malformed
  or unavailable prepared state still falls back through the complete public rebuild boundary.
- GPU candidate ranking and CPU exact shape, corridor, clearance, connectivity, and DRC authority
  are unchanged.

### Verification

The retained PH invariant test now moves an interior node in a six-node chain and proves that the
bounded reconstruction matches a complete reconstruction to one picometre over 33 route
parameters. The retained continuous-envelope rebound test also passes. The qualified Tier-0 board
passes all 50 paths, represented copper DRC, and external KiCad DRC, with 47/47 PH runs fitted and
48 native arcs. Required global/detail analysis images and final copper-layer PNGs are under
`build/suite-per-change-20260904-144713/`; visual inspection shows continuous rounded runs without
new loop or staircase artifacts.

The detailed stage took 290.63 seconds versus 325.35 seconds for the immediately preceding retained
run, a 10.67% reduction while processing the same 10,488 PH candidates and 3,648 exact GPU-shortlist
checks. The final KiCad board is byte-identical in both runs
(`ff67a745e609253a5ba919c482ab570f0402c7ad564ad5695f0d09a79c230d57`). A separate old-binary
BM-SVF run was active during both measurements, so this is strong
directional evidence rather than an uncontended canonical timing result. That Tier-1 run remains
the before-checkpoint for the subsequent frontier comparison.

### Decision

Keep. The optimization removes recomputation without relaxing or replacing any geometric
certificate. Run the rebuilt executable through BM-SVF after the isolated before-run completes;
profile again if exact PH reconstruction remains the dominant regional cost.
