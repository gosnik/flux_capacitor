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

## Experiment 34 — Discrete global feedback and homotopy-constrained PH realization

### Failure

The retained old-binary BM-SVF run exhausted its 7,200-second Tier-1 budget after completing only
one full global/regional epoch and entering the second global epoch near the timeout. The first
global allocation found all 25 coarse paths, but every nominally cheap intermediate physical probe
then ran full PH realization, XPBD/PNS, residual-cone repair, and quantization over 81 windows and
16 residual cones. This prevented the five configured global-feedback epochs from consuming the
breadth of discrete packing, terminal, lane, layer, and via evidence they were designed to learn.

During the correction, the retained curved-materialization risk fixture exposed a second boundary
failure. Its five-node route created a 5,359-cell NavMesh guard union. Every PH candidate copied
that union and repeatedly sorted all cell boundaries, taking 155.45 CPU seconds for one fixture.
The coarse whole-cell certificate also omitted legal clearance-boundary slivers, while an
unconstrained GPU precondition found physically clear geometry by changing obstacle-side homotopy.

### Hypothesis

Intermediate epochs need only the regional combinatorial diagnosis; continuous realization is
authoritative only for the final retained checkpoint. Separately, sharing one prepared corridor,
filtering hull queries by cached bounds, using the exact offset domain for bounded sub-cell
recovery, and applying the selected route-class predicate during candidate selection should make
PH realization both faster and topologically correct.

### Change

- Intermediate feedback probes disable spline realization explicitly. They stop after conflict
  decomposition and bounded corridor/packing/terminal/lane/layer/via analysis; CLI evidence reports
  both the disabled mode and zero evaluated spline candidates.
- PH routes share one immutable prepared corridor across candidate copies. Cached cell bounds
  reject unrelated regions before exact union interval construction.
- A subdivided hull omitted by conservative whole-cell rasterization may use the exact prepared
  offset-domain distance only while remaining within one hull diameter plus tolerance of the
  selected guard union.
- Seed candidates are scored against the same exact offset domain used by trajectory optimization.
  This retained the legal tight corner around the risk fixture's blocker.
- GPU preconditioning, interpolation multistarts, adaptive controls, and CPU-shortlisted moves now
  reject a candidate before promotion when its ordered topology gates or obstacle winding differ.
  Continuous collision normals are projected to nearby PH interpolation nodes, and exact clearance
  improvement breaks violation-count plateaus before soft curve cost.
- The former monolithic risk test was split so curved materialization can be qualified directly
  without rerunning unrelated global/regional scenarios.

### Verification

The focused curved transaction now produces native connected line/arc copper and passes exact
represented DRC in about 14 seconds. Before the prepared corridor it failed after 155.45 CPU
seconds with 1,801 residual continuous-corridor violations; the prepared but otherwise unchanged
certificate reproduced the same failure in 8.93 seconds, isolating a roughly 17-fold certificate
speedup before the correctness repair. The retained PH invariant and inter-sample obstacle rebound
tests pass. The complete obstacle/capacity/regional/via/PNS risk test passes in 208.43 seconds and
proves every intermediate feedback candidate reports spline realization disabled with zero spline
candidates evaluated. Qualified Tier-0 and rebuilt Tier-1 results are recorded separately below
once their mandatory telemetry, PNG, connectivity, and KiCad DRC gates complete.

### Decision

Keep pending board qualification. This implements the intended nested-loop boundary without
weakening any regional, continuous, quantized, connectivity, or DRC authority. The BM-SVF rerun
must demonstrate that the global loop now reaches additional feedback epochs within its two-hour
budget.

## Experiment 34 — Conflict-directed exact regional master

### Failure

The bounded continuous shortlist could reject every realization while retaining the same corridor
side, layer span, via region, or run-up decisions. Promoting more continuous candidates therefore
repeated an already disproved discrete state. The later residual-cone retry could aggregate nearby
windows, but it did not use the PH/copper rejection as a lazy constraint or generate new columns
specifically for the implicated runs.

### Hypothesis

The retained regional templates, access choices, via sites, layer sequences, and packed cross
sections already form a finite exact-master domain. Keeping PH geometry outside that master avoids
an intractable mixed continuous search. A failed exact realization can instead forbid its stable
complete discrete selection, identify the route runs in its failure core, and generate new
topology/layer/via columns only for those runs.

### Change

- Added stable physical master keys for template geometry and layer sequence, via site/span, and
  ordered cross sections. Retry-local IDs are deliberately excluded.
- Added bounded lazy no-good filtering after the complete access and lane assignment has been
  constructed. The constraints cannot prune an unrelated partial state or authorize copper.
- Expanded each failed PH realization to its complete `(net, connection, run)` fragment set within
  the window. Only that bounded set owns new homotopy/corridor/layer/via generation; surrounding
  window routes remain fixed master variables.
- Reran the existing deterministic exact CSP for several bounded separation rounds. Every failed
  PH/quantized candidate contributes another no-good; a complete realization may replace only its
  own window on a strict feasibility-first improvement.
- Exposed core, column, master-rerun, clause, rejection, recovery, and exhaustion counts through
  result state, CLI JSON, and normal telemetry.

### Decision

Retain pending fixture and tier qualification. The implementation does not treat the discrete
master as copper authority: PH continuous-envelope and topology checks, quantized mutual
clearance, whole-board connectivity, represented copper DRC, and external KiCad DRC remain the
commit gates. Bounded failure still escalates to the broader residual-cone/global feedback loop.

## Production feature — Confined anisotropic conflict erosion

### Failure

The ordinary hydraulic family discovers global layer-local topology with an isotropic raster
metric. Once PH or quantized copper rejects a complete regional state, that global field does not
encode the implicated run direction, the attributed same-layer blocker envelopes, or the need to
try both lateral sides without perturbing unrelated nets.

### Research

Anisotropic fast-marching and Hamiltonian fast-marching methods model direction-dependent motion
through a local metric rather than changing obstacle membership. That separation matches Flux:
the metric may rank topology proposals, while conservative polygon free space and exact copper
predicates remain authoritative.

- J. A. Sethian and A. Vladimirsky, anisotropic front propagation and ordered upwind methods:
  <https://doi.org/10.1073/pnas.201222998>
- J.-M. Mirebeau, Hamiltonian fast marching for anisotropic/non-holonomic models:
  <https://www.ipol.im/pub/art/2019/227/>

### Feature

- Added an integer direction-dependent hydraulic metric. The raster-axis cross product scales
  transverse reverse-potential steps and bank-erosion diffusion; ratio 1000 preserves isotropy.
- Implemented identical bounded arithmetic in the CPU oracle and Vulkan shader.
- Added a pressure-side-preserving visibility-pull mode without allowing pressure to change the
  polygon-derived walkable mask.
- Integrated two signed lateral erosion priors only into the exact conflict-directed regional
  master. Work is confined to the attributed window and route run, separately budgeted, and
  blocker-clearing proposals become ordinary same-layer topology columns.
- Published configuration, check/accept/reject, backend, and exact-chord diagnostics through the
  regional result, CLI JSON, and telemetry. ADR 0169 records the production contract.

### Verification

The focused directional field test proves greater transverse potential, reduced transverse bank
erosion, unchanged walkability, validation of a missing axis, and byte-identical CPU/Vulkan
potential, erosion, flow, and riverbed output. Existing polygon-discovery and concurrent Vulkan
batch equivalence tests pass with anisotropy enabled. The broad obstacle/capacity/regional fixture
passes and asserts that a present exact conflict core invokes the confined phase, with every check
accounted as accepted or rejected.

The qualified Tier-0 board's global phase retained 50 paths, zero unrouted nets, and zero sampled,
corridor, or clearance-shadow overflow. It correctly invoked the feature zero times because no
regional physical failure existed. Current dirty-worktree detailed routing nevertheless stopped on
one unrelated pre-existing PH trajectory with one unintended inflection and seven unrounded
non-junction bends; it reported zero opens and zero represented copper violations but did not reach
external KiCad DRC. The non-canonical contended result is recorded in
`reports/benchmark_history/2026-09-04-confined-anisotropic-erosion-feature.json`.

### Decision

Keep the feature. It is deterministic, bounded, dormant without an exact attributed conflict, and
cannot weaken free-space or copper legality. Tier-0 promotion remains blocked by the separate PH
shape failure and must not be reported as green until that route completes and external KiCad DRC
passes.

## Experiment 35 — Persistent failure-directed routing session

### Failure

The interleaved planner could label a discrete-only regional probe complete and return before PH
realization. If detailed materialization later rejected that topology, final feedback accumulated
every previously affected net, reran global PathFinder for lane/via/quantization failures, rebuilt
the complete regional state, and anchored later attempts to the last exact-score winner. A worse
but structurally different exploratory state was discarded and could therefore be regenerated on
the next pass. Regional diagnoses from discarded global portfolios also entered shared history.

### Hypothesis

Successful copper should appear sooner if each failure returns to the nearest solver with enough
authority to change it, while global negotiation remains live only for an explicit or repeated
topology failure. Exact promotion at the first plausible discrete checkpoint should either finish
early or create a useful physical failure core before the planner spends further PathFinder
epochs.

### Change

- Added immediate full-regional promotion for a complete discrete intermediate probe. The
  provisional and promoted outcomes are separately observable.
- Made regional failure history private to each retained global portfolio and merged only the
  selected portfolio into the next epoch.
- Added deterministic failure dispatch for local geometry, regional lane/escape/layer/via state,
  and global topology. A repeated unchanged downstream diagnosis escalates one level.
- Rebuilt the affected-net cone from the current diagnosis each pass instead of retaining a
  monotonically growing union.
- Retained the newest exploratory routing state as the next working checkpoint while preserving
  the feasibility-first best exact result as the only selectable transaction.
- Added incremental exact materialization for a complete regional PH/multilayer plan.
- Added one persistent planning-to-copper coordinator and adopted it in direct KiCad detailed
  routing and WorkPlan execution. Fresh full integrated-placement candidates use the same bounded
  feedback loop; incremental proposal screens remain cheap.

### Verification

The focused global/regional/detailed transaction test proves immediate promotion, full promotion
success, incremental regional rematerialization, persistent-session completion, and the three
failure-dispatch classes on the deterministic two-pad board. Build, broader risk tests, and tier
measurements follow in the associated benchmark record.

### Decision

Keep pending tier comparison. No path, overflow, connectivity, clearance, or DRC acceptance rule
was relaxed; the change removes redundant upstream work and stale cross-topology feedback.

## Experiment 36 — Advisory boosted-tree regional candidate ranking

### Failure

Regional windows retain several discrete corridor/lane/layer/via states, then pay the expensive
PH/XPBD and quantized-envelope realization in heuristic order. The first exact-legal state stops
the window search. Existing scalar and diversity objectives have no learned estimate of which
physically plausible state is most likely to survive continuous realization.

### Hypothesis

A gradient-boosted learning-to-rank model trained on pre-realization state features and exact
post-realization outcomes can move legal states earlier in the bounded shortlist. Reordering alone
should reduce continuous candidates and time-to-first-legal without reducing search coverage or
weakening exact authority.

### Research

- XGBoost implements grouped LambdaMART learning-to-rank with `rank:ndcg`:
  <https://xgboost.readthedocs.io/en/stable/tutorials/learning_to_rank.html>.
- LightGBM exposes `lambdarank` and `rank_xendcg`, but the first experiment uses XGBoost because a
  single stable exporter is sufficient to compare the mechanism:
  <https://lightgbm.readthedocs.io/en/latest/Parameters.html#objective>.

### Change

- Added an 18-feature, schema-versioned pre-realization contract covering current rank, template,
  via, layer, channel, topology-cost, lane-slack, contention, and search-state evidence.
- Added append-only JSONL observations labelled with the exact topology, shape, mutual-clearance,
  quantization, completion, and runtime outcomes of every evaluated candidate.
- Added a training-only XGBoost/LambdaMART script. It groups candidates by regional window,
  validates by board revision, verifies exported inference against XGBoost margins, and emits a
  compact Flux-owned tree ensemble.
- Added bounded C++ inference with strict feature-schema, size, finite-number, child-reference,
  reachability, and cycle validation. No XGBoost runtime dependency enters `flux_core`.
- Integrated ranking immediately before regional realization. It performs a stable descending
  learned-score reorder but never prunes a retained state. A missing or rejected model preserves
  heuristic order. Exact PH/XPBD, quantization, connectivity, copper DRC, and commit gates are
  unchanged.
- Exposed collection and model paths through CLI flags and published load, scoring, movement,
  write, and diagnostic counters in CLI JSON.

### Before

No learned candidate evidence existed. Candidate order came exclusively from the discrete
objective and diversity beam, and no reusable per-candidate exact training record was emitted.

### After

The focused C++ test loads and evaluates a synthetic split tree, verifies deterministic scores,
and rejects feature-schema drift. The checked-in six-row/two-window observation fixture trains a
16-tree XGBoost model whose exported scorer reproduces XGBoost margins and ranks the best fixture
candidate first in both groups. The development build and focused unit test pass.

The complete disabled-ranker Tier-0 regression passes: `pt2399_delay` produces all 50 paths and
then exact copper with zero opens, zero internal copper violations, 110 straight tracks, 59 arcs,
two vias, and zero external KiCad 9.0.8 error-level DRC violations. Global and detailed stages took
148.46 and 368.02 seconds respectively. The run reports the ranker unrequested with zero scored,
reordered, or logged candidates, demonstrating that the default path remained dormant. Structured
results are in
`reports/benchmark_history/2026-09-05-regional-candidate-ranker-tier0.json`.

### Interpretation

The integration boundary is safe enough for a real benchmark experiment: model error changes
work order only. The fixture demonstrates format and inference equivalence, not routing-quality
generalisation. Real results require observations from multiple board families; row-random
validation would leak nearly identical candidates and is explicitly avoided.

### Decision

Retain as disabled-by-default instrumentation and advisory search ordering pending Tier-0/Tier-1
data collection and board-held-out comparison. Do not use learned scores for pruning or legality.

### Next hypothesis

Collect baseline observations across every practical lower/frontier board, measure top-1/top-4
exact-candidate recall and PH realization time, then compare the same fixed model against
unchanged heuristic ordering. Promote only if lower tiers remain green and median/time-to-first-
legal improves without a completion regression.

## Experiment 37 — Preserve global rip-up authority before regional PH

### Failure

The first Tier-1 feedback epoch performed 13 hard-capacity rip-ups, but the selected checkpoint
still had 26 sampled-resource overflows, three corridor overflows, 332 clearance-shadow
conflicts, and 143 regional conflicts. The planner then spent roughly ten minutes building the
regional discrete portfolio and more than thirty minutes attempting exact PH candidates before a
second PathFinder epoch could run. Rip-up was active, but immediate exact promotion starved it of
the remaining 29 configured global iterations.

### Hypothesis

Sampled and corridor overflow, plus unresolved global lane transitions, are owned directly by
PathFinder and its rip-up/LNS operators. Sending such a checkpoint to regional continuous
realization cannot repair its still-negotiable global allocation efficiently. Conversely,
clearance-shadow and physical-width pressure must remain eligible for regional diagnosis because
they represent the sub-grid packing failure that the global model only approximates.

### Change

- Deferred intermediate regional work while sampled overflow, corridor overflow, or unaccepted
  global lane transitions remain.
- Preserved intermediate regional handoff for globally connected checkpoints whose remaining
  pressure is clearance-shadow or physical-envelope based.
- Required the same global-resource gate before immediate exact PH promotion.
- Kept the final epoch's full regional/PH/XPBD/copper transaction mandatory even if global
  negotiation exhausts its budget.
- Added an explicit `regional-probe-deferred` telemetry event and separate sampled/corridor fields
  to each feedback-candidate record.

### Before

`build/tiers-check-20260905-xpbd-gpu-ph-tier1/tier-1-bm_svf/telemetry/bm_svf-global.jsonl`
shows one short PathFinder pass followed by a long regional/PH phase. The run was stopped after
about 41 minutes when it entered its fallback exact wave without returning to global negotiation.

### Verification

The transactional global-to-detailed regression proves that intermediate candidates with
PathFinder-owned overflow do not enter regional or PH realization, while later/final exact gates
remain enabled. Tier benchmark comparison follows.

### Decision

Keep pending Tier-0 and Tier-1 measurements. This changes solver scheduling only; it does not
relax connectivity, capacity, physical-copper, or DRC acceptance.

## Experiment 38 — Strict KiCadRoutingTools oracle trace on Tier 1

### Failure

The post-sequencing `bm_svf` run exhausted its 7,200-second allowance. Flux completed all 30
PathFinder passes and reported 9,801,533 search expansions, but its retained all-net checkpoint
still had 19 sampled overflows, three corridor overflows and 324 clearance shadows. It constructed
a 542-assignment regional portfolio, repeated several bounded conflict waves, constructed a second
410-assignment portfolio, and timed out while starting assignment 56. No copper transaction was
committed.

### Hypothesis

An external successful route on the identical stripped board can distinguish intrinsic board
infeasibility from a Flux search/representation failure. A time-ordered copper trace can identify
whether success depends on global negotiation, many simultaneous route changes, or small targeted
victim exchanges.

### Research

KiCadRoutingTools documents a direct Rust-accelerated octilinear grid A*, MPS endpoint ordering,
incremental per-net obstacle caching, forward/backward failure probes, exact blocker attribution,
and bounded N+1 rip-up. Its route-trace mode records every copper commit, rip and restore without
changing routing decisions.

### Change

No Flux algorithm changed. KiCadRoutingTools main revision
`ec2bf0e1021c0ed95c45b14664d02c56777a0974` (v0.22.0) was installed in an isolated build
directory. The exact `bm_svf` fixture was routed with a 0.2 mm track, 0.2 mm clearance,
0.7/0.3 mm via, escalation disabled, standard fabrication floor, strict sizes and external KiCad
10 DRC. Full copper tracing, verbose diagnostics, A* statistics, memory diagnostics, layer PNGs
and a 71-frame replay were retained.

### Before

Flux timed out after 7,200 seconds with no committed board. The trace contains 8,494 semantic
events, 155 route rip-ups and 342 hydraulic-field updates and occupies 831,964,957 bytes.

### After

KiCadRoutingTools routed 25/25 nets and 78/78 pad pairs in 7.30 seconds wall time using 699,298 A*
expansions, 615 final segments, 73 through-vias and 925.353 mm of copper. There were no rule
escalations. KRT's internal checker reports zero represented DRC violations and two permitted
same-net crossings. KiCad 10 reports zero opens and no routing error under the untouched project;
the only generated finding is one 0.0452 mm dangling-track warning. The 69-event trace contains
eight rips and one restore and occupies 79,915 bytes.

The route uses long board-perimeter detours and both layers. The useful repair chain was local:
`Net-(U4A--)` completed after ripping `Net-(R4-Pad1)`; the latter then completed after ripping and
successfully re-routing `Net-(U4B--)`; an unsafe `V_SUPPLY` restore was rebuilt at the end.

### Interpretation

The strict reference route proves that the placement is physically routable on two layers. Flux's
frontier failure is therefore search/representation/scheduling, not intrinsic geometry. KRT avoids
Flux's principal semantic gap by searching and committing in one conservative quantized-copper
occupancy representation. It preserves long detours automatically, treats vias as ordinary A*
states, and invokes expensive repair only for the specific failed frontier. Flux instead spends
most of the run exact-realizing large portfolios whose global allocations already retain physical
overflow, and its coarse no-goods often preserve the responsible run/layer/via relation.

### Decision

Keep the trace as a differential oracle. The highest-value next experiment is a strict KRT-like
quantized-copper feasibility master between global allocation and PH realization. It should search
the full-board fine domain, commit exact-size via/layer states into an incremental obstacle map,
use bidirectional frontier attribution for bounded N+1 victim exchanges, and pass the resulting
complete legal topology to PH fitting. PH remains final geometry; the oracle is a fast physical
legalizer/candidate generator, not a replacement router.

Full results and artifact paths are in
`reports/krt_bm_svf_comparison_2026-09-05.md` and
`reports/benchmark_history/2026-09-05-krt-bm-svf-reference.json`.

## Experiment 39 — Early quantized-copper feasibility oracle

### Failure

The strict KRT comparison proved that `bm_svf` is routable on two layers, while Flux passed an
all-net but physically incompatible global allocation into hours of regional realization. The
global representations did not provide an exact integer-copper verdict before PH work, and useful
long perimeter/layer-changing alternatives were too weakly represented.

### Hypothesis

Materializing the selected global allocation on a fine manufacturing lattice and exact-checking
its real track/via envelopes between PathFinder epochs will expose opens and physical conflicts
cheaply enough to feed the responsible nets and regions back upstream. Retaining one explicit
perimeter homotopy and reducing the blanket via bias should preserve the two structural choices
demonstrated by KRT's successful route.

### Research

KRT's Rust router searches a 0.1 mm grid using clearance-aware swept moves, ordinary layer-change
states, incremental obstacle maps, and failed-frontier blocker attribution. Flux already has
stronger polygonal topology discovery, layer-wise hydraulic fields, exact clearance shadows, and
continuous PH realization; the missing useful invariant was KRT's shared search/physical copper
representation. The appropriate integration is therefore a critic/feasibility master, not a
replacement of NavMesh or PH.

### Change

- Added a default-enabled 0.1 mm quantized-copper oracle between global allocation and regional
  realization.
- Preserved exact path endpoints, snapped interior vertices/vias, represented complete multi-layer
  via chains, and reused same-net vias or plated through-hole pads.
- Ran exact dynamic-copper DRC and connectivity on the provisional primitive set.
- Converted exact violations and opens into localized, net-attributed quantization diagnoses,
  telemetry events, routability history, and the next incremental reroute cone.
- Added oracle opens and exact violation counts to retained global-portfolio comparison after the
  existing topology/regional feasibility dimensions and before abstract overflow/cost.
- Added one protected perimeter-biased static seed family and prevented endpoint-only NavMesh
  shortening from erasing its homotopy.
- Reduced the ordinary via preference from eight planar steps to three while leaving every exact
  via legality gate unchanged.
- Made every global lattice node an implicit via candidate; the former rule-spacing stride pruned
  legal individual sites merely to guarantee that adjacent unselected candidates could coexist.
  Exact selected via envelopes and DRC now own that compatibility decision.
- Made clearance shadows advisory when the multi-epoch quantized oracle owns physical feedback.
  Shadow raster samples are coalesced into the strongest witness per net-pair/kind/layer/local
  region and retained for telemetry, conflict graphs, and regional repair scope. They no longer
  add `total_overflow`, hard-block transitions, affect present cost, or write generic edge/node
  history in that mode.
- Retained connection-run identity in exact oracle diagnoses. Run/run conflicts now request a
  lane/template change; generated-via conflicts request a via-site change; malformed transition
  chains request a layer-sequence change. These typed exact no-goods replace raster-shadow
  history in later global epochs.

### Before

The prior 7,200-second Tier-1 run ended with 19 sampled overflows, three corridor overflows, 324
clearance shadows, no accepted copper, and no direct measurement of the selected checkpoint's
integer-copper connectivity/DRC state.

### After

The focused deterministic test rejects crossing quantized routes and accepts a legal
`F.Cu -> B.Cu -> F.Cu` route with two vias. On the first `bm_svf` global checkpoint the oracle
measured 769 exact dynamic-copper violations and 11 opens despite 25/25 abstractly routed nets;
the selected allocation used zero vias. The oracle plus the deferred regional probe completed in
approximately 0.63 seconds after PathFinder, so it did not create a new material bottleneck. The
five-minute bounded run then remained inside construction of the next global epoch, identifying
portfolio rebuilding rather than physical probing as the immediate runtime cost.

The focused shadow-ownership regression retains the crossing's regional conflict witness while
reporting shadow history increment zero and excluding its witness count from `total_overflow`.
Only the exact quantization diagnosis enters physical no-good history. Direct single-pass planner
configurations retain legacy shadow negotiation so disabling feedback cannot silently discard the
only physical signal.

### Interpretation

The hypothesis is strongly supported as a diagnostic and feedback mechanism. The earliest
physical failure is now observable before PH, and the zero-via checkpoint directly supports more
permissive layer exploration. The current oracle evaluates selected topology; it does not yet run
KRT-style local fine-grid repair. Its attributed conflict cone is the correct boundary for that
next experiment.

### Decision

Keep pending Tier-0 requalification. Next, retain immutable discovery/domain products across
feedback epochs and use the oracle's failed region for bounded fine-grid multilayer victim exchange
before invoking regional PH.

Detailed architecture and measurements are in
`reports/path_discovery_quantized_oracle_2026-09-05.md`.

## Experiment 40 — Incremental hierarchical PH realization

### Failure

Regional realization repeatedly rebuilt complete PH tessellations and signed degree-nine offsets
after a local interpolation-control move. Exact mutual-clearance closure then used midpoint
subdivision for every surviving offset pair, with a 50,000-node allowance per pair. Independent
regional windows were evaluated serially even though they read the same immutable board snapshot.

### Hypothesis

A persistent per-span certificate hierarchy should preserve unaffected geometry when a local PH
control changes. Conservative centreline-distance bounds should prove most route pairs separated
or colliding before rational offsets are constructed. Processing independent windows concurrently
should reduce elapsed realization time without changing candidate order or exact acceptance.

### Research

[Sederberg and Nishita's Bézier clipping](https://doi.org/10.1016/0010-4485(90)90039-F)
uses the convex-hull property to exclude parameter regions that cannot contain an intersection and
reports an advantage for higher-degree curves. [Chang, Choi, Kim and
Wang](https://doi.org/10.1016/j.cag.2011.03.025) formulate minimum distance between Bézier curves
using culling, bounding volume hierarchies and dynamic subdivision, specifically to converge faster
than ordinary binary subdivision. Flux needs a clearance predicate rather than a complete
intersection enumeration, so the minimum-distance hierarchy is the more direct first-stage test;
the existing rational PH offset isolator remains the fail-closed ambiguous fallback.

### Change

- Added prepared quintic Bézier certificate trees with outward-rounded double AABBs, retained
  long-double controls, deterministic geometry fingerprints and bounded traversal.
- Added a precision cascade: cheap conservative double separation, native-SIMD batches of
  subdivision bounds, long-double on-curve penetration witnesses, 50-decimal comparison only for
  genuinely near-tangent samples, then the existing long-double degree-nine rational-offset
  isolation only for ambiguous leaves.
- Changed exact regional pair closure to skip rational-offset construction for centreline pairs
  whose required copper-envelope separation is already certified.
- Made regional geometry refresh compare span fingerprints. Only changed PH spans rebuild their
  adaptive tessellation, hierarchy and signed offsets; route-pair verdicts are revision-keyed so
  unrelated certificates remain usable.
- Added bounded content-addressed segment and route-pair caches keyed by geometry, layer,
  clearance, corridor revision and board obstacle snapshot. Their deterministic lifetime covers
  one complete realization invocation, including recursive shortlist and fallback waves.
- Grouped spline assignments by regional window, retained sequential first-exact-legal evaluation
  within each group, and scheduled up to four groups against private result/geometry state.
  Results are reduced in original window order and nested exact-candidate workers remain one.
- Added telemetry/CLI counters for window workers, incremental span rebuild/reuse and centreline
  certificate outcomes.
- Corrected the KiCad 10 AppImage benchmark adapter: the AppImage is a multi-call launcher and must
  receive `kicad-cli` before ordinary PCB subcommands. Analysis PNG export now uses that form.

### Before

The preceding Tier-0 qualification reached its ten-minute board limit and then failed while
rendering diagnostics because the harness invoked the KiCad 10 AppImage as a direct `kicad-cli`
binary. Regional exact work still reconstructed complete per-candidate offset state.

### After

The focused PH test deterministically proves a 2 mm-separated pair from double hulls and proves an
orthogonal collision from long-double curve points. The regional crossing regression remains
deterministic and accepts the new centreline hierarchy as the preferred exact path without
requiring rational-offset broadphase work. The KiCad 10 AppImage successfully exports both copper
layers through the corrected adapter.

On the bounded Tier-0 run, five regional windows used four workers. All 36 route-pair certificate
queries were proved separated at the root double hull: zero pairs reached rational-offset
construction, zero exact offset nodes were visited, and 13 unchanged spans were reused while 62
were rebuilt. PH realization itself accounted for 72.46 seconds of the 562.86-second routing run;
the remaining time was dominated by alternative corridor/run-up construction. The board did not
qualify: all 20 nets had global paths and sampled/corridor overflow were zero, but windows 0 and 2
could not construct a topology-and-shape-legal PH seed for the `kicad-net-10`/`kicad-net-11` and
`kicad-net-11`/`kicad-net-8` crossings. The quantized-copper oracle independently reported four
physical conflict diagnoses and zero opens. Three legal near-result windows were exported as
analysis PNGs after correcting the harness to parse structured exit-code-3 results before applying
the qualified-tier gate.

### Interpretation

The local mechanism is strongly supported: exact legality remains fail-closed, the observed
route-pair workload no longer constructs rational offsets, and window concurrency is active.
Tier evidence moves the frontier upstream: route-pair certification is not the reason Tier 0 is
currently incomplete. The earliest remaining failure is topology-seed realization for two
crossing windows, while most wall time is spent generating and screening alternative regional
corridors rather than checking PH pair clearance.

### Decision

Keep. Do not spend the next experiment refining offset intersection because ambiguity was zero on
the frontier run. Instrument the topology-seed rejection split (corridor/gate versus PH shape),
then change the smallest upstream decision that prevents the two unresolved crossing windows from
forming any spline candidate.

## Experiment 41 — Exact-static regional detour topology strip

### Failure

The instrumented Tier-0 run rejected 1,312 of 1,342 topology-seed candidates on corridor or gate
preservation, while only eight candidates failed PH shape. Both unresolved windows required one
exact-static-checked regional detour around a crossing.

### Hypothesis

Regional detours might be physically legal but unable to round a NavMesh portal because only
selected global routes received a narrow route-specific topology strip. Giving checked detours the
same strip would permit a same-homotopy PH corner cut while downstream obstacle and copper checks
remained authoritative.

### Change

Temporarily attached the existing half-cell checked-centreline strip to exact-static regional
detours as well as selected global routes. No clearance or DRC gate was relaxed.

### Before

The adaptive-precision Tier-0 run took 572.73 seconds, rejected 1,312 topology seeds on corridor or
gate preservation and eight on shape, retained three legal window candidates, and left two windows
unresolved.

### After

The strip run took 565.41 seconds and produced exactly the same 1,312 corridor/gate rejections,
eight shape rejections, three legal window candidates and two unresolved windows. It again emitted
three near-result PNG sets.

### Interpretation

The hypothesis is disproven. The failing candidates do not consult this strip, or their failure is
the directed gate sequence rather than cell-union containment. The small runtime difference is
noise relative to the unchanged search and result metrics.

### Decision

Revert the strip. Next split topology failure into cell containment versus directed-gate order and
record the first failed sample/gate. Do not broaden topology corridors without identifying which
certificate is rejecting the curve.

## Experiment 42 — Ordered gate subsequence and retry ownership transfer

### Failure

Tier 0 rejected 1,130 PH seeds because a smooth curve did not cross every raster-derived NavMesh
gate even though it remained inside the selected guard-cell union. After allowing those redundant
gates to be omitted, five windows found exact-legal local bundles, but final assembly rejected two
different replacements for fragment `kicad-net-11/0/0/1`.

### Hypothesis

Guard containment plus the unique, forward, ordered sequence of every gate actually crossed is a
stronger representation of the selected geometric corridor than requiring a smoothed curve to
visit raster-only gates. The duplicate writer is independent: a conflict-directed retry promotes
an adjacent articulation fragment but replaces only its source window, leaving the old owner
assignment authoritative.

### Change

Regional PH certification first proves guard-cell containment, then accepts an ordered subsequence
of the selected directed gates. Strict gate equality remains the default everywhere else. Residual
and conflict-directed checkpoint merge now transfers publication ownership for every fragment
owned by the retry, so frozen context can participate in certification but cannot emit a second
copper replacement.

### Before

The strict-gate run took 562–573 seconds, retained three legal candidates, and left two windows
unresolved. Topology seed rejection comprised 1,130 missing-gate failures, 182 cell-containment
failures, and eight shape failures.

### After

Tier-0 global/regional routing took 188.09 seconds. All 50 connections had paths, sampled and
corridor overflow were zero, all five regional windows had complete bundles, nine PH realizations
were retained, and `regional_spline_candidate_available` became true. The candidate diagnostic was
`complete regional spline transaction assembled`; conflicting articulation was false. Detailed
materialization was reached for the first time in this sequence.

### Interpretation

Strongly supported. The strict raster gate list was rejecting same-corridor continuous geometry,
and retry ownership—not PH legality—prevented atomic assembly. The next failure moved downstream:
exact materialization reports three open connections and 37 copper violations after its fallback.

### Decision

Keep both scoped changes. Exact connectivity and copper DRC remain authoritative.

## Experiment 43 — Adaptive-precision PH certificates and prepared chord domains

### Failure

PH curve-pair predicates used long-double work throughout. A live stack sample of the newly
reachable detailed phase also showed one route worker repeatedly revalidating the same immutable
free-space polygon domain for every rectangle chord, spending more than thirteen minutes in
polygon-boundary distance work.

### Hypothesis

Outward-rounded double hulls can prove the common separated case, SIMD can screen batches of
subdivision nodes, long double can retain authority for ambiguous leaves, and multiprecision is
needed only for degenerate contact comparisons. Separately, validating a source-run domain once
and batching its six rectangle chords should remove invariant O(boundary²) work without changing
legality.

### Change

Prepared PH certificate nodes now retain outward-rounded double bounds. Native SIMD evaluates
AABB lower-distance candidates; every apparent separation receives a directed-rounding scalar
confirmation so multiply/add rounding cannot turn an upper estimate into a proof. Ambiguous nodes
use the existing long-double hierarchy and rational-offset isolator, with 50-digit comparison only
inside a 64-epsilon near-tangent band. Counters expose SIMD batches/lanes and multiprecision use.
The detailed source-polyline fallback now validates its immutable free-space domain once and uses
the same exact chord predicates through a prevalidated six-chord batch.

### Before

The first post-ownership detailed command remained active for 13:46; its sampled worker stack was
`assess_free_space_chord -> valid_domain -> polygon_boundary_distance_nm`.

### After

The comparable detailed command completed in about 7:13. A second stack sample contained no
`valid_domain`; four route workers were concurrently performing NavMesh preparation, polygon
Boolean operations, and PH containment. Tier-0 global PH metrics reported 454 SIMD batches/lanes,
zero ambiguous pairs, and zero multiprecision comparisons. Root-hull separation means this board
offers no cross-call SIMD lane occupancy; the scalar double proof is already constant-time there.

### Interpretation

The prepared-domain hypothesis is strongly supported and removed the measured hot stack. The
precision cascade is validated and fail-closed, but its SIMD throughput benefit is inconclusive on
this board because every call separates at its single root pair.

### Decision

Keep. Future performance work should batch independent segment-pair roots across routes rather
than claiming speedup from one-lane SIMD calls.

## Experiment 44 — Interior pad/via attachment boundaries

### Failure

`kicad-net-16` was one of three nets open after exact detailed materialization. Its selected coarse
tree passes within the copper of a through-hole terminal while another connection ends at the pad.
The PH chain collector recognizes only exact endpoint incidences.

### Hypothesis

Splitting a straight carrier at an exact same-net pad/via centre before PH collection would retain
an otherwise bypassed electrical attachment.

### Change

Exact pad/via centres lying in a carrier interior become PH chain boundaries. A synthetic
four-terminal route verifies that an interior pad remains connected after curved realization.

### After

The focused regression passes, but Tier 0 is unchanged: the open nets remain 2, 16, and 17, with
the same 348 fallback tracks and 37 copper violations.

### Interpretation

The mechanism is real in isolation but is not the frontier-board cause. The Tier-0 open exists
outside this exact-centre attachment case, likely in regional multilayer stitching or a
non-centreline copper contact.

### Decision

Keep the narrowly defined topology-preservation behavior and its regression. Do not broaden it to
arbitrary copper proximity without an exact failing witness. Next preserve/export the failing
detailed candidate and trace the three pad groups through materialization.

## Experiment 45 — Static-tree attachment certificates

### Failure

The saved no-arc Tier-0 candidate confirmed that nets 2, 16 and 17 each contained two physically
separate copper components. KiCad's connectivity engine independently reported the same three
opens. The loose components ended at ordinary lattice nodes rather than at failed PH or
quantization boundaries.

### Hypothesis

Static portfolios are generated as connected growing trees, but their branches are subsequently
NavMesh/PH-reprojected independently. A child branch can terminate at an interior node of an
earlier carrier; if that carrier's endpoint-only reprojection removes the node, the portfolio
remains syntactically complete while its physical tree becomes open.

### Change

Before static refinement, each complete tree now records every downstream branch-attachment node
on its earlier carrier. A carrier reprojection is accepted only if its refined lattice path still
contains all those nodes. Failure retains the original connected carrier and increments explicit
attachment-certificate telemetry. This changes neither clearance rules nor route scoring.

### Before

Tier 0 had 50 global paths, zero sampled/corridor overflow, three open connections on nets 2, 16
and 17, and 37 copper violations (32 track-clearance and five track-via-clearance).

### After

The full diagnostic Tier-0 run certified 150 attachment nodes and rejected nine destructive
reprojections. Exact detailed connectivity improved from three opens to zero. Copper violations
also fell from 37 to 26, all track-clearance violations, distributed across five attributed
net-pair families in six regional windows. The exact regional transaction rejected the remaining
conflicts, so no illegal
board was committed. Runtime was approximately 185 seconds for the standalone global pass and
535 seconds for detailed routing. All 24 core tests pass.

### Interpretation

Strongly supported. All observed opens were caused by loss of an interior static-tree junction,
not by PH fitting, pad contact, or the connectivity checker. Preserving the junction moved the
frontier cleanly from topology/connectivity failure to physical regional packing.

### Decision

Keep the certificate and regression. The next experiment must address the exact same-layer
packing conflicts; it must not weaken the exact regional transaction or KiCad DRC gates.

## Experiment 46 — Fair G2 PH selection and single native-copper handoff

### Failure

Beautiful flowing curves are a product objective, but two different representations still made
the realization boundary both expensive and ambiguous. Algebraically valid quintic PH branches
were ranked without an explicit fairness term, the optional higher-order join compared
parameter-dependent second derivatives, and regional routing could spend bounded rational-offset
work on a curve that would ultimately be discarded in favour of KiCad arcs and segments. Worse,
detailed routing reconstructed a new PH route from the accepted regional tessellation instead of
consuming the native pieces that regional routing had checked.

### Hypothesis

Fairness-aware PH branch selection plus one post-optimization geometric-G2 promotion should improve
visible flow without entering the inner candidate loop. Fitting the selected PH route immediately
and making represented native copper authoritative should remove unnecessary rational-offset work
in the common case. Carrying those exact pieces into detailed routing should eliminate the
regional/detailed realization mismatch.

### Research

Quintic PH Hermite interpolation retains polynomial speed, exact arc length, and rational offsets,
while fair-PH construction treats curvature distribution as a primary quality measure rather than
an endpoint-only property. Geometric G2 continuity requires tangent direction and scalar curvature
continuity; equality of parameter-scaled second-derivative vectors is the stronger and
parameterization-dependent C2 condition. KiCad has no electrical Bézier track primitive, so its
straight-track and circular-track-arc representation is the physical object Flux must certify.
The resulting architecture is recorded in ADR 0171.

### Change

- PH branch scoring now penalizes scale-normalized total curvature variation and more than one
  interior curvature extremum, after the existing cusp, loop, inflection, and shape gates.
- Join certification compares signed scalar curvature. A damped finite-difference Newton solve
  changes only internal tangent magnitudes, preserving interpolation points and G1 directions.
- Detailed routing makes one bounded G2 promotion attempt after trajectory optimization and before
  arc fitting. A failed promotion retains the already legal G1 curve.
- Final regional windows are fitted to native arcs/segments and checked together on a disposable
  board. Clean native scenes skip rational-offset intersection; rejected scenes retain continuous
  envelope/offset diagnostics for attribution.
- Regional realizations retain their native pieces. Detailed materialization locates the exact
  centerline interval, emits those pieces byte-for-byte (including reversed path orientation), and
  protects native straight pieces from a second spline collector pass.
- CLI metrics expose G2 attempts/promotions/fallbacks, regional native checks, and exact native
  pieces consumed at the detailed handoff.

### Before

The first integration version checked native pieces regionally but discarded them at handoff. Its
Tier-0 detailed pass took roughly 650 seconds, fell back to 332 straight tracks and one via, and
ended with zero opens but 26 track-clearance violations. Ten G2 promotions were attempted; one
succeeded. This reproduced the existing physical-packing frontier rather than qualifying Tier 0.

### After

All focused PH, curved-materialization, regional-transaction, and impossible-crossing regressions
pass. The full C++ unit suite passes in 119.83 seconds. A synthetic transaction proves an exact
regional arc crosses detailed materialization byte-for-byte as connected, DRC-clean copper.

The qualified Tier-0 global pass took 187.62 seconds, retained all 50 paths, and checked six native
windows containing 114 fitted pieces. Five windows were legal; one rejected portfolio candidate
reported nine native violations. No route pair required exact rational-offset isolation. Regional
PH realization consumed 48.90 seconds.

The detailed pass took 366.86 seconds and consumed 12 accepted regional PH runs directly as 40
arcs and 10 straight pieces. Structural validation and connectivity were clean, but full-board DRC
reported 29 violations: 27 track-to-arc and two track-to-track clearance failures. The first was a
2,775 nm clearance deficit between a non-window `kicad-net-19` track and a retained
`kicad-net-3` regional arc. The board was correctly withheld and external KiCad DRC did not run.

That first handoff protected each complete connection containing a regional span, which retained
avoidable raster geometry outside the immutable interval. A focused refinement now applies the
ordinary pull-tight and continuous path optimizer independently to the open intervals on both
sides while preserving the native span exactly. The targeted detailed rerun consumed the same 12
regional runs as 40 arcs and 10 straight pieces, reduced tracks from 368 to 240, and reduced
full-scene violations from 29 to 19 (17 track-to-arc and two track-to-track). It took 383.15
seconds. Connectivity and structural validation remained clean; the board was again withheld.

### Interpretation

The representation-boundary hypothesis is strongly supported: exact native pieces now survive
the handoff, the common regional path avoids rational-offset isolation, and runtime is comparable
to the last recorded successful detailed-pass duration rather than the initial 650-second
reconstruction/fallback run. The experiment also moved the earliest observable mismatch upstream.
Per-window native DRC is incomplete when a curve approaches route copper outside that window;
full-board detailed DRC is now exposing those cross-window interactions precisely. Preserving only
the exact native interval helps but does not solve that missing cross-window coordination.

### Decision

Keep the fair branch ranking, geometric-G2 promotion, native regional gate, exact handoff, metrics,
and regression. Tier 0 is not promoted by this run. The next hypothesis is that complete-candidate
native DRC evidence must be fed back to the responsible regional windows (or those windows must
include a halo of immutable/non-window route copper) before local native legality can authorize the
all-window candidate.

## Experiment 47 — Native realization review corrections

### Failure

The Experiment-46 Tier-0 pt2399_delay run had zero opens but 19 represented-copper violations
(17 track/arc, two track/track), 147 unrounded bends, 0/4 G2 promotions and 383.15 seconds in
detailed materialization. External KiCad DRC was not reached. The working tree was already dirty;
these records are a historical comparison, not an isolated clean-commit A/B result.

Review identified whole-supporting-disk arc obstacles, inconsistent regional ownership at native
handoff, incomplete outside-window scene evidence, missing exported join-angle checks, early G2
fallback commitment, configuration bypass, endpoint-extremum miscounting, a dense G2 Jacobian
solve, and a chord-proxy gate still vetoing the native representation.

### Hypothesis and ranked changes

1. Sweep-local conservative obstacles preserve physically empty space around shallow arcs.
2. An ownership-resolved manifest prevents duplicate/context copper publication.
3. Assembled native/provisional/via scene evidence moves cross-window failures upstream.
4. Quantized join-angle validation and complete native-transaction G1 fallback preserve the
   intended flowing geometry without weakening copper DRC.
5. Shared fit-policy checks, correct interior extrema, local tridiagonal promotion and removal
   of the redundant chord-proxy physical veto improve consistency and bounded cost.

### Research

- KiCad SHAPE_ARC exposes bounded-error arc-to-polyline conversion:
  https://docs.kicad.org/doxygen/classSHAPE__ARC.html . Flux uses its own conservative chord
  envelopes, expanded by the exact circular sagitta plus integer-rounding guard; no code copied.
- LAPACK documents tridiagonal linear systems:
  https://netlib.org/lapack/explore-html/d6/ddb/group__gtsv_ga6b06a11b8e2dced10bc9cd3c20a09484.html .
  Flux's local join residual has three neighboring derivative dependencies. A bounded Thomas
  solve replaces dense elimination; unlike pivoting DGTSV it deliberately declines small pivots,
  since G2 is optional and the G1 transaction is retained.

### Verification criteria

Focused tests must preserve both sides of the arc copper envelope while freeing unswept space;
reject policy/budget mismatches; publish shared boundary copper once; distinguish fixed from
provisional cross-window conflicts; bound quantized join angles; and preserve cancelled G1 input.
Run the complete core suite and Tier 0. A reduction in raw DRC report count is not sufficient:
record opens, physical categories, runtime and publication outcome. Do not promote a tier without
external KiCad DRC.

### Result

The core suite passed in 224.34 seconds (with a concurrent tier run), then 220.02 seconds on
the subsequent build. The broader non-corpus run passed 115/122 tests. Five CLI expectation
failures match the pre-change log `build/dev/Testing/Temporary/LastTest.log.tmpbc337`. A constrained
refinement export timed out at the explicit 300-second bound, and its dependent DRC could not
load the missing board. No pre-change completed timing is available for that constrained test.

Tier 0 finished global/regional search after 531.27 seconds: 50 paths, zero unrouted nets,
overflow 16, three failed spline bundles, 22 evaluated joint candidates, two legal native
windows from 11 checks, and no complete routing plan. Detailed routing and KiCad DRC were not
reached. The actual harness uses one router iteration for this qualified-tier probe; the earlier
Experiment-46 hand-written configuration said 30, so use the captured command configurations
rather than that summary for comparison. Artifacts: `build/native-review-fixes-tier0`.

### Refinement — construct native tangency instead of only rejecting kinks

A diagnostic replay of the first eight rejected-realization events reconstructed their PH
Bezier controls from telemetry. Six failed the 0.035-radian join bound even after all eight
positional refinement attempts; their remaining errors ranged from about 0.050 to 0.531 radians.
The two straight examples already fit. This demonstrates that denser independent circle fitting
is not an adequate general tangent-continuity mechanism. Adjacent integer-point deduplication
and invalid quantized-circle handling are useful robustness corrections, but no collapsed
pieces occurred in these eight examples and they do not explain this failure.

The smallest targeted correction is equal-distance Hermite biarc fitting after the cheap
positional fit fails its angle check. The construction follows the geometric derivation in
Ryan Juckett's primary technical account: https://www.ryanjuckett.com/biarc-interpolation/ .
Only the equations are used; no implementation is copied. PH design, topology, DRC and routing
search are unchanged. Farthest-fitting bounded local intervals preserve native tangency, and
integer tangent/position checks reject rounding damage. The initial diagnostic replay recovered
all six failed fits, using 16–82 pieces rather than requiring ever denser independent circles.
This replay is geometry evidence, not board qualification. Final core and tier results follow.

The final tangent-matched implementation passed the complete core suite in 119.25 seconds,
restoring approximately the pre-experiment core-suite runtime instead of the 220-second
positional-refinement trial. New tests cover one-attempt native fitting, dense positional probes,
piece-budget exhaustion, reversed native handoff, selected width provenance, and proposed vias.
Final source/executable hashes and tier artifacts are recorded in
`reports/benchmark_history/2026-09-05-native-review-fixes.json`.

The final native-arc/multilayer export regression selection passed all nine tests, including
external KiCad DRC, in 30.30 seconds. The final Tier-0 run took 443.09 seconds in global/regional
search: 50 paths, zero unrouted nets, overflow 16, one failed spline bundle (versus three in the
positional-refinement trial), 12 evaluated joint candidates, four legal native windows from five
checks, 275 native pieces checked, and zero native-fit failures. The 35 native copper violations
are aggregated rejected-candidate evidence, not a final-board DRC count. No complete regional
transaction was available, so detailed export and Tier-0 KiCad DRC were not run.

The remaining window is 2: nets `kicad-net-11` and `kicad-net-8` on B.Cu around
x=42.5–43.5 mm, y=28.5–29.5 mm, with a 0.45 mm coarse conflict deficit. Net 11 is immutable
articulation context; net 8 owns three templates, including two statically checked alternatives.
All three candidate states emitted zero spline realizations. This precedes native fitting, and
the generic `curvature_infeasible` diagnosis does not yet prove the exact constructor mechanism.
The next focused investigation should inspect PH construction/guard binding and immutable
articulation context, not further tune the native fitter.

Decision: retain the locally verified representation/correctness fixes and tangent-matched
fitting; keep ADR 0171 experimental. The refinement recovered the intended fit mechanism and
restored core-suite runtime, but did not qualify Tier 0. No higher tier is promoted or claimed.

## Experiment 48 — Directed adaptive-portal predicates

### Failure

The fresh Tier-0 baseline (`build/frontier-20260905/baseline`) took 453.93 seconds,
found all 50 paths and no unrouted nets, but retained overflow 16 and no complete
regional plan. Detailed export and KiCad DRC were not reached. The core baseline
passed in 133.43 seconds. All five corpus boards passed strict import/provenance
preflight. These runs preserve the already modified working tree; its initial
patch and executable hash are retained under the baseline directory.

New seed diagnostics isolate window 2 on B.Cu: the immutable net-11 context is a
straight horizontal seed from (51,29) to (37.5,29) mm. For each of the three
net-8 assignments, all 160 seed candidates fail the directed-gate check, with
zero construction, containment or shape rejections. This is a certificate-stage
failure, not evidence of curvature infeasibility. Structured diagnosis:
`reports/diagnoses/2026-09-05-tier0-directed-gates.json`.

### Ranked hypotheses and research

1. **Portal-side predicate depends on tessellation.** Adaptive neighboring cells
   have offset centers. Their center vector is not a portal normal. A synthetic
   vertical portal reproduced the defect: a forward straight chord passes,
   while insertion of two collinear samples makes the same crossing fail.
   Smallest experiment: orient the true portal normal by the cell transition.
   Cost stays O(gates × segments); no extra route search or copper relaxation.
2. **Concatenated gate sequences disagree with a shortened seed.** Individual
   gate membership does not establish sequence order. Record the failing gate
   and replay its complete signature before changing sequence policy.
3. **Immutable articulation context carries a different owner's topology.**
   Reuse of the owner's exact realization may be necessary, but it is a larger
   transaction change and is ranked after correcting/replaying the predicate.

Primary references inspected:

- KiCadRoutingTools `ec2bf0e1021c0ed95c45b14664d02c56777a0974`,
  `docs/rip-up-reroute.md`, `docs/routing-architecture.md` and the existing strict
  bm_svf comparison. https://github.com/drandyhaas/KiCadRoutingTools/blob/main/docs/rip-up-reroute.md
  Its useful invariant is attribution at the failed geometric boundary, with
  bounded blocker exchanges and preserved/restored surrounding copper. Flux
  retains NavMesh/hydraulic discovery, PH geometry and exact native validation;
  no upstream implementation is copied.
- Kallmann and Kapadia, SIGGRAPH 2014 navigation course, geometric path-planning
  and funnel sections: https://people.cs.rutgers.edu/~mk1353/pdfs/2014-siggraph-course-navigation-slides.pdf
  Portal orientation and ordered traversal are geometric predicates; route
  sampling should not change their answer. This motivates a predicate repair,
  not replacement of Flux's continuous routing architecture.

### Minimum experiment and verification

Replace the skew cell-center normal by the portal normal, retaining cell centers
only to orient the transition. Add a deterministic micro-test for subdivision
invariance, reverse traversal, repeated crossing, tangency and reversed portal
vertex order. The micro-test failed on the original implementation. Measure the
frontier again and run core and external-DRC export regressions. Reject any
change that merely skips a true reversed or repeated portal crossing. Results
and subsequent refinements will be recorded below.

### First result and causal refinement

The portal-normal micro-test now passes, including reversed/repeated-crossing
rejection, and the complete core suite passed in 135.17 seconds. This fixes a
real adaptive-mesh defect, but the detailed Tier-0 replay establishes a different
direct cause of the persistent straight-context rejection: gates 12–23 are exact
copies of gates 0–11. The raw straight seed itself fails at gate 12.

Tracing those copies upstream found that `assemble_window_templates` enumerated
all channels after alternative generation. A channel's occupant was incorrectly
interpreted as retained global membership, so alternative-template channels were
concatenated into the original template. Two unselected alternatives consequently
imposed duplicate directed gates on immutable context. Their free-space cells
also enlarged the original template's guard. Removing duplicate gates downstream
would hide that provenance mistake and would leave the incorrect guard union.

The refinement explicitly marks alternative-only channels at all four candidate
generators. Retained template assembly and retained packing assessment exclude
those channels; each alternative still explicitly selects its own channel. The
crossing micro-board checks both retained-template isolation and the presence of
independently selectable alternatives. This preserves Flux's portfolio and
continuous realization interfaces while correcting the earliest bad binding.

### Measured planning outcome

| Tier-0 configuration | Planning seconds | Complete regional plan | Failed spline bundles | Evaluated joint candidates | Seed gate rejections |
| --- | ---: | --- | ---: | ---: | ---: |
| Initial dirty-tree baseline | 453.93 | no | 1 | 12 | 1120 |
| Additional diagnostics only | 466.65 | no | 1 | 12 | 1120 |
| Portal-normal correction only | 483.61 | no | 1 | 12 | 1120 |
| Alternative-channel isolation plus portal correction | 277.06 | yes | 0 | 5 | 115 |

The normal correction is independently supported by its failing-before/passing-after
micro-test, but did not solve this board's direct blocker. Channel provenance is
strongly supported: all five ordinary repair windows now complete, and the
retained assembled plan becomes available. Its 36 native-versus-provisional
outside-window conflicts remain advisory; there are zero native/fixed-scene
conflicts. These are not final-board DRC results. Runs overlapped other bounded
benchmarks, so timings are indicative rather than isolated performance estimates.

The crossing/multilayer regression passed with the channel-provenance assertion.
All nine native-arc/multilayer export regressions passed, including external KiCad
DRC. The full non-corpus suite passed 116/123 tests: the same five CLI expectation
failures, 300-second constrained-export timeout, and dependent missing-board DRC
failure documented in Experiment 47 remain. No new failure category appeared.

The manifest's remaining `global_complete` promotion targets were tightened to
`detailed_complete`, preserving tier statuses. A topology-only success must not
promote any of the five boards under the current acceptance contract. The
source-bound benchmark-manifest test passed after this policy correction.


## Experiment 49 — Exact materialization witnesses define regional repair scope

### Failure and hypothesis

The corrected Tier-0 one-iteration detailed attempt completed in 876.3 seconds:
zero opens, 16 track/arc clearance violations, five implicated net pairs, two
feedback passes and zero improvements. The first pair is net 17 versus net 1,
33,444 nm below its required 450,000 nm centerline separation. Ten coalesced
physical regions cover pairs (1,17), (3,4), (10,11), (7,8), and (8,11), all B.Cu.
Regional feedback currently transfers these witnesses only as XPBD history costs;
conflict extraction still reads the original global shadows/capacity reports.
Thus downstream conflicts cannot enlarge the exact joint repair scope.

### Research and ranking

1. Pass current exact failure witnesses to regional conflict extraction, keeping
   their net pairs, physical bounds and deficits. This directly tests the missing
   representation boundary with existing regional partition/search machinery.
2. Expand every regional window by a fixed margin. Less targeted; risks large
   coupled regions and arbitrary parameter tuning, so defer.
3. Replace local search by full CBS. Much larger change; defer until evidence
   shows Flux's existing exact regional master cannot solve an accurately scoped
   conflict set.

KiCadRoutingTools' blocker-attributed bounded reroute and Sharon et al.'s CBS
(https://webdocs.cs.ualberta.ca/~nathanst/papers/sharon2015cbsjournal.pdf, 2015,
sections 1 and 4) both motivate conflict-directed scope/constraints. CBS assumes
discrete time agents and has exponential worst-case search; Flux's static copper
packing differs, so this experiment borrows only witness-driven decomposition,
not its temporal search or optimality guarantees. Extraction adds bounded
O(witnesses × net-pairs × fragments) work; regional search budgets stay intact.

### Minimum experiment

Supply current final-validation/quantization pair witnesses only while global
paths are unchanged. Attribute them to same-layer intersecting fragments, expose
mapped/unmapped counts, and create ordinary regional conflict edges. Do not reuse
stale exact witnesses after global replanning. A synthetic shadow-free crossing
must create a repair window only with a valid exact witness; unknown nets/layers
must remain explicitly unmapped. Success requires fewer final physical conflicts
on Tier 0; scope extraction alone does not establish board improvement.

## Experiment — USPED-informed initial NavMesh refinement (2026-09-05)

### Failure and hypothesis

User-requested experiment: use terrain structure to inform NavMesh subdivision before routing.
The latest recorded Tier-0 physical frontier is a regional construction failure before native
fitting; it has not been attributed to insufficient mesh resolution. Hypothesis: clearance-derived
flow concentration and transport divergence identify useful extra refinement regions. This is not
a claim that USPED will fix the frontier.

### Research and choice

Inspected the local LittlePlanets terrain CPU USPED routine and GPU capacity/divergence shaders,
README, and MIT licence. Reference physics: `A^1.6 sin(slope)^1.3` transport and flux divergence.
See https://grass.osgeo.org/grass80/manuals/addons/r.divergence.html .
Chose one bounded, geometry-only CPU prepass rather than iterative landscape modification or
per-net sediment simulation. Clearance-only refinement remains the control and the default.

### Change

Mapped immutable polygon obstacles/board exterior to masked mountains and residual clearance to
descending slopes/valleys. Implemented D8 accumulation, rill capacity, conservative masked-edge
divergence and normalized refinement priority. No rain or flux enters masked cells; flats remain
sinks and unit rainfall alone does not trigger arbitrary subdivision. No terrain displacement.

Integrated opt-in subdivision of certified coarse cells, preserving ordinary boundary refinement,
existing legal-region coverage, exact child checks, and a baseline rebuild on mesh-budget overflow.
Added prepared-domain diagnostics/fields, configuration provenance, CLI `--usped-navmesh`, and a
bounded on/off JSON benchmark runner. ADR 0172 gives contracts, complexity and limitations.

### Before / after

- Existing clipped-passage regression passed before integration.
- Synthetic mesh: 192 baseline cells -> 258 cells, 22 USPED-refined parents from 120 raster cells;
  certified route retained, mountain exclusion preserved. Tight budget restores the 192-cell mesh.
- Mountain/valley, flat, diagonal-mask, signed divergence/conservation, repeatability, invalid-input,
  config mismatch, raster-budget and mesh-budget tests pass.
- Initial full core suite passed 28/28; final verification is recorded in the USPED summary report.
- Final cap-chain global probe: both modes 4 paths, zero unrouted, zero overflow, zero NavMesh
  failures. Route-summed cells 33,208 -> 45,496; enabled route-summed extra parents 4,096.
  Wall times 1.282s / 1.208s are contended single runs and do not establish a speedup.
- Tier-0 bounded smoke: both modes exceeded 60s before publishing JSON. Concurrent build/test
  activity and rebuilding between trials also prevent a controlled comparison. No detailed board
  or external KiCad DRC was produced. Tier 1 and higher were not promoted or evaluated here.

### Interpretation / decision

Mechanism and integration verified; board-quality hypothesis remains inconclusive. Retain as an
explicitly disabled-by-default experiment, not a mandatory routing stage. No tier qualification.
Reports: `2026-09-05-usped-cap-chain-final.json` and `2026-09-05-usped-tier0-bounded.json` under
`reports/benchmark_history/`. No source board was edited.

### Next experiment

Run isolated, full-duration Tier-0 on/off trials with a fixed executable and the same hard mesh
budget, reporting actual cells as well. Measure surviving detailed realizations and final DRC;
compare against clearance-only refinement at a matched actual cell count before adding GPU work.

## Experiment — 2026-09-05: Restore rip-up opportunities through iteration budgets

### Failure / hypothesis

Tier-0 planning defaults to one global iteration. The CLI also caps physical
feedback retries at iterations minus one, disabling feedback. Test whether
budgets 4 and 8 improve physical completion versus a fresh one-iteration run.

### Change / experiment

Added an experimental `--iteration-budget` override to the corpus runner, applied
to planning and detail phases without editing the manifest. Added a structured
summary tool that retains oracle metrics, feedback probes and timeout telemetry;
missing DRC remains unevaluated. No routing algorithm or default was changed.
Ran the same executable sequentially with 1, 4 and 8 iterations and the existing
600-second planning limit. Source hashes and binary identity are preserved.

### Before / after

- 1: 169.455 seconds, incomplete; 50 paths, zero unrouted nets, overflow 16,
  76 native-scene conflicts, zero feedback passes. Provisional oracle: 32
  violations, zero opens. No final KiCad DRC.
- 4: timeout at 600 seconds; first epoch completed with 48 oracle violations and
  zero opens. Second epoch started with 21 physical-history entries. No final JSON
  or final DRC; final conflict count unknown.
- 8: timeout at 600 seconds; same first-epoch evidence; second epoch started with
  a two-iteration PathFinder budget. No final JSON or final DRC.

### Interpretation / decision

Extra feedback opportunities are activated, but improved copper is unproven.
Code inspection revealed that enabling feedback also makes clearance shadows
advisory, and changes first-epoch regional probing/promotion. This is a comparison
of the coupled CLI modes, not an isolated test of extra identical rip-up rounds.
Keep the reproducible experiment tooling; leave router defaults unchanged.
No tier promotion. The first intermediate round consumes most of the time budget.

### Next hypothesis

Separate feedback rounds, global iteration budget and physical-conflict policy;
hold policy fixed and profile/bound intermediate regional work. Raising the
iteration constant alone has not demonstrated a practical fix.
Detailed diagnosis: `reports/diagnoses/2026-09-05-ripup-budget-coupling.md`.
Structured history: `reports/benchmark_history/2026-09-05-ripup-budget-sweep.json`.

## Experiment — 2026-09-05: Oracle-first versus full-strength first-pass scheduling

The explicit feedback-count and shadow-policy controls isolate scheduling from
the previous coupled CLI modes. Oracle-first scheduling returns known exact
provisional-copper failures upstream before intermediate regional search. The
final full regional/DRC transaction remains mandatory.

At fixed N=4/R=3 with advisory shadows, enabled scheduling completed epochs at
105.281, 213.688 and 318.918 seconds, with provisional violation counts 48, 38, 50
and zero opens. Its final epoch timed out at 600 seconds. The disabled control
completed only one epoch and also timed out. It overlapped an independent Tier-0
process, so timings do not establish an isolated speedup. Both used the same
binary. Full core and nine arc/multilayer export/DRC checks passed. Keep oracle-first
disabled by default: more feedback is observed, but final copper benefit is unproven.
See `reports/diagnoses/2026-09-05-oracle-first-feedback.md` and the corresponding
structured benchmark history.

The user then explicitly requested a bundled first-pass "kitchen sink" experiment.
`--full-strength-first-pass` front-loads compatible bounded global and regional
operators, while retaining later feedback and every final legality gate. This is
the opposite scheduling hypothesis: broad early coordinated repair may avoid
revisiting an under-explored first topology. It does not imply maximum hardware
parallelism, a topology-backend rewrite or relaxed clearances. The exact scope,
budgets and exceptions are documented in
`reports/diagnoses/2026-09-05-full-strength-first-pass.md`.

Full-strength outcome: first exposed a cone-repair integration bug (selective-only
invalidation remained enabled after selective rerouting was disabled; an invalid
empty repair could replace the incumbent). Cleared the dependent option and
rejected invalid/cancelled repair candidates; an actual cone-repair regression
passes. The full core suite and nine arc/multilayer export/DRC checks pass.

Corrected Tier-0 trial: timeout at 600 seconds, no final board/DRC. Telemetry
records 18 completed primary/cone PathFinder iterations and a completed first
epoch at 511.083 seconds. Provisional oracle: 32 violations, zero opens. Four
regional windows had three complete bundles, with two conflict-directed
recoveries; assembled spline candidate remained unavailable. The experiment
exercises substantially more machinery but does not qualify the tier. Keep the
general integration fix and retain the broad mode only as an opt-in experiment.
Structured result: `reports/benchmark_history/2026-09-05-full-strength-first-pass.json`.

## Experiment — 2026-09-06: Frozen outside-copper context at local native selection

Failure: Tier-0 pt2399_delay, N6/R0/enforced/full-strength, completes global
topology (50 paths, zero unrouted nets) in a failed 518.570-second plan. Local
native checks report zero violations, but full native assembly reports 39
conflicts, six classified fixed. Window 3's selected F.Cu bulge crosses the
untouched continuation of another net outside its owned stub.

Hypothesis: the first-locally-legal exit selects a candidate without considering
the global copper that assembly subsequently retains. Complete local context
will reject that candidate while alternate templates remain available.

Research: KiCad NODE branch/root collision queries preserve unchanged obstacles
and exclude explicitly overridden items. Concept only, no GPL code copied.
Source, ranked alternatives and geometry are recorded in
`reports/diagnoses/2026-09-06-frozen-copper-context.md` (plus JSON).

Change: opt-in `--frozen-copper-context`, temporary exact-check Board containing
source copper plus every global planar fragment not owned by the current
window. Fragment-level exclusion preserves same-net outside continuations.
Contexts are cached per window within a realization invocation. Missing owners
fail closed. Neither source nor accepted copper is modified. Proposed vias and
joint inter-window motion remain separate issues; final checks are unchanged.

Success criterion: more useful candidate exploration and fewer actual final
physical conflicts, ideally complete DRC-clean copper; a drop in assembly
conflicts due solely to withholding assembly is not improvement. Regression
criterion: default tests remain green; added micro-test proves outside-copper
crossing detection and ownership/layer/source invariants. Baseline structured
history: `reports/benchmark_history/2026-09-06-frozen-context-baseline.json`.

After: 431.656 seconds, still 50 paths/zero unrouted nets and three complete
bundles out of four. Candidates evaluated 4 → 21; locally legal candidates
4 → 3; native local violation reports 0 → 87. Window 3's final eight retained
states all fail native clearance, including B.Cu alternatives. Assembly was
withheld: its zero conflicts are NOT an improvement over the baseline's 39.
Physical-scope retries 1 → 0. Provisional oracle remains 32 violations/zero
opens. No final board or KiCad DRC; no tier promotion. The faster failure is
consistent with avoiding a late assembly/scope retry, not faster successful
routing. The core suite and nine arc/multilayer checks (including two KiCad
DRCs) passed in 173.18 seconds; the focused transactional test also passed.

Interpretation: missing context is supported as a local acceptance gap. Simply
freezing it does not make this portfolio geometrically feasible. Keep only as
an opt-in diagnostic experiment, disabled by default. It has not earned default
routing-policy status. Structured comparison:
`reports/benchmark_history/2026-09-06-frozen-context-comparison.json`.

Next hypothesis: preserve the identities and physical regions of outside
fragments rejected during native validation and feed them into coordinated
scope expansion. Currently these become generic window-level clearance
failures, while scope retries consume only assembled-native/linear-scaffold
witnesses. Candidate failure before assembly therefore loses the very evidence
needed to enlarge ownership. Establish the exact blocker pairs before changing
scope; telemetry snapshots may contain stale repair-cone alternatives and are
not an authoritative reconstruction of the selected global board.

## Experiment — 2026-09-06: Exact frozen-fragment scope feedback

Failure: the frozen-context experiment exposed 87 local native violation reports
but triggered zero physical-scope retries; window 3 remained unresolved. Local
assessment retained no outside fragment identity. Existing nearest-net/layer
witness mapping also cannot represent a candidate that changes layers while
retaining its original fragment identity.

Hypothesis: retain exact candidate/blocker IDs and couple those original
fragments after the retained candidate fails, rather than repeatedly repairing
the same two clipped stubs. Research: MA-CBS selective coupling
(https://ojs.aaai.org/index.php/SOCS/article/download/18244/18035/21760).
Its time-based MAPF assumptions and completeness/optimality guarantees do not
transfer to persistent PCB copper and bounded Flux search. Concept only, no
external code copied. Scope-size limits control exponential joint-search risk.

Change: collision-safe frozen-track provenance maps; deduplicated native
candidate/blocker records on each assignment; telemetry and unresolved JSON;
opt-in `--expand-frozen-copper-scope` to seed the existing conflict graph using
exact IDs. Only retained unresolved assignments contribute. Missing IDs,
already-owned blockers and duplicate pairs do not trigger another retry.
The existing physical retry/net/fragment/search limits remain unchanged.

Focused ownership/provenance/deduplication tests pass. A synthetic candidate
layer-change test confirms exact IDs create a joint cross-layer repair window
where nearest-net/layer attribution cannot. Full regression and frontier trial
pending. Detailed diagnosis/ranking:
`reports/diagnoses/2026-09-06-frozen-scope-feedback.md`.

Normal-budget outcome: 600-second timeout during the second scope retry, no
final board/DRC. Scope attribution works: exact pairs 4 → 5 → 8, all mapped.
First analysis at 437.168s and first retry at 571.961s both retain 3/4 complete
bundles; discrete candidates 228 → 654. Core plus nine arc/multilayer checks
passed in 173.94s. Keep scope expansion disabled by default. One identical-binary
900-second diagnostic trial was run to finish the bounded work; normal
600-second qualification remains failed. The runner records its timeout
override explicitly and leaves the corpus manifest unchanged.

Oracle inspection with KiCad's parser: the preserved reference keeps REF on
F.Cu nearly straight and routes +5V around its left side; GND occupies the
B.Cu horizontal channel. This is topology evidence only, not a fresh DRC
certification. It suggests testing a broader +5V/connection corridor change
if fragment-scope expansion still fails. No reference tracks are copied.

Extended outcome: finished in 730.098 seconds, still no accepted board or DRC.
Two scope retries, 3940 generated discrete states, 10 evaluated/4 locally legal
candidates in the last analysis. The original region is no longer the reported
unresolved window; the remaining window is +9V/+5V with two locally legal F.Cu
templates. Last native-scene counters report 2 fixed conflicts (earlier
full-strength-only control: 39). Global overflow remains 19 with 50 paths and zero
unrouted nets. Assembly is subsequently withdrawn, so these counters may
describe an earlier assessed scene, not the final retained local state.

Decision: partial support for precise feedback/coupling, not a tier pass or a
normal-budget success. Keep disabled by default. Preserve provenance and
add last-scene diagnostic/object output plus manifest status: the benchmark
binary omitted the exact remaining pairs/categories. Do not guess a via failure
or immediately retune topology. Classify that separate assembly failure next.
Full comparison: `reports/benchmark_history/2026-09-06-frozen-scope-feedback-final.json`.

Final reporting-only build verification: all 10 selected checks passed again
in 171.65 seconds (core 144.95 seconds; two KiCad DRC checks included). Python
compilation and invalid-timeout rejection passed; `git diff --check` is clean.
The final binary differs from the benchmark binary only through the subsequent
native-scene reporting changes described above; no new routing policy was
enabled after the recorded experiment.

## Experiment — 2026-09-06: Candidate-via local context

Reporting-only reproduction finished in 718.651 s and identifies both last-scene
reports as track_via_clearance, 125000 nm deficit each: GND B.Cu native track
versus two new +5V vias. Local native assessment omitted these vias. Whole-net
assembly invalidation also marked an unrelated +5V/+9V window incomplete.
Exact details and ranked hypotheses:
`reports/diagnoses/2026-09-06-candidate-via-context.md` and `.json`.

Research: KiCad interactive routing validates vias as physical collision objects
(https://docs.kicad.org/7.0/en/pcbnew/pcbnew.html). Reuse Flux's exact predicates,
not external source code. Minimum experiment: include resolved candidate vias in
the opt-in frozen-context native scene, retain all attached fragment owners for
rejection/feedback, use assembly dimensions and same-net via/plated-pad reuse.
No change to normal defaults, clearances or search/retry budgets. Synthetic
planar-fit/via-collision, shared-via provenance/reuse and invalid-span tests added.
Verification and identical-configuration frontier trial pending.

Rejection-only outcome: 900-second timeout in enlarged-scope candidate search.
Local via/blocker telemetry and exact scope feedback now work, but this does not
establish better routing quality. Regression checks passed 10/10 (181.52 s),
including two KiCad DRCs; expanded ownership/reuse micro-test also passed.
History: `reports/benchmark_history/2026-09-06-candidate-via-context.json`.

Refinement: feed the selected candidate via copper disks into the existing
regional clearance evaluator and CPU/GPU proximity broad phase, on every spanned
layer. Net and layer filtering remains explicit. Via dimensions use assembly
rules, and candidate-scene cache fingerprints include via geometry/span/net.
This is still gated by frozen context. The existing trajectory optimiser, not
a new solver, successfully rebounds around the synthetic 125000 nm via deficit;
shape/corridor/continuous certification passes. Added disk broad-phase checks.
Full regression and identical-configuration refined frontier trial pending.

Refined outcome: also a 900-second timeout. First analysis 433.404 s; enlarged
retry has 3 windows / 50 templates at 497.552 s. Retained six-route profiles still
show one mutual/quantisation failure. GND is no longer falsely reported mutually
legal at the spline stage, but no legal real-region deformation has been found.
The isolated rebound works in the actual regional two-iteration/64-check budget
(44 checks, no exhaustion); it also remains legal after native arc/segment fitting.
10/10 regressions pass (183.53 s, two KiCad DRCs); expanded micro-test passes.
History: `reports/benchmark_history/2026-09-06-candidate-via-guidance.json`.

Keep default off; correctness/observability improved, benchmark completion did
not. Do not increase local iteration constants on this evidence. Next isolate
the real six-fragment window with its actual ownership and guard constraints,
then determine whether corridor topology or search blocks the otherwise available
rebound. Escalate to whole-connection/corridor feedback if locally trapped.
Net-wide full-scene invalidation is a separate known repair-targeting defect.

## Experiment — 2026-09-06: Default repeated physics and conflict-directed rerouting

### Failure and hypothesis

User requested the kitchen-sink policy as the default, including physical push/shove
followed by rip-up/reroute and another physical attempt. The former full-strength
option only applied to epoch zero; later intermediate epochs disabled realization.
Prior frozen-context N6/R0 trials also exhausted 900 seconds in local rebuilding.
Hypothesis: maintain full physical feedback on every epoch and give failed repair
an upstream topology retry before spending the local scope-rebuild budget.

### Research and ranking

KiCad documents complementary shove and obstacle-walkaround operations
(https://docs.kicad.org/7.0/en/pcbnew/pcbnew.html); VPR documents bounded
PathFinder iterations (https://docs.verilogtorouting.org/_/downloads/en/v8.0.0/pdf/).
These support separating local displacement from topology search, not a claim
that either guarantees legal Flux copper. Ranked: (1) reuse the existing physical
and typed-feedback loop on all epochs; (2) enlarge local scope repeatedly;
(3) introduce a new joint solver. Select (1), the smallest integration change;
retain (2) for the final epoch and defer (3).

### Change

Default full-strength core and CLI policy, compatible global operators and full
regional physics on every epoch, frozen copper/via context, bounded retries.
Intermediate scope retries are deferred; final epoch keeps configured scope repair.
Explicit legacy/oracle scheduling remains available. Default retry counts clamp to
short total budgets; explicit invalid CLI budgets remain rejected. No DRC changes.
Added default/legacy CLI tests, short-budget and three-epoch failure tests.

Follow-up inspection found retained exact outside-copper pairs were not converted
to upstream diagnoses: window-only summaries could omit the blocker net. Feed the
exact candidate/blocker net pair, layer and deficit into corridor-change history;
do not feed pairs from now-complete assignments. Add focused regression coverage.

### Verification and benchmark interpretation

Policy-only Release build passed 45 selected checks (10 core/arc/multilayer,
32 route/detail CLI, 3 new default/opt-out parser tests), including two KiCad DRC
fixtures. Outside-blocker follow-up verification pending. The retry scheduling test
forces a window-size failure; it checks physical consultation across epochs, not
successful XPBD movement. Existing rebound tests exercise actual deformation.

Earlier timing trials used Debug, not Release. Record build identity separately;
do not attribute optimization-build speedup to policy. Policy-only normal Tier 0
and N6 repeated-repair trials are running at the unchanged 600-second limit.
No tier promotion until complete output and final DRC pass.

Final outside-blocker build verification: 45/45 selected checks passed in 84.50 s,
core 79.62 s; two KiCad DRC fixtures passed. The initial new test expected exact
fragment IDs in output, but the existing diagnostic builder coalesces IDs by
physical region. Corrected the assertion to check kind, actual layer, deficit,
relaxation and both net owners. No coalescing-policy change was needed.
Structured verification: `reports/benchmark_history/2026-09-06-default-physics-verification.json`.

Policy-only N6/R4/enforced-shadow Release trial timed out at 600 s. Three physical
epochs completed; XPBD ran again after the first upstream reroute, verifying the
new scheduling mechanism on a real board. The intermediate oracle changed from
32 violations / 0 opens to 0 violations / 9 opens: **not an improvement in legal
completion**. History retained the earlier portfolio. Concurrent jobs and a build
type change prevent a clean performance comparison to prior Debug timings.
History: `reports/benchmark_history/2026-09-06-default-physics-policy-control.json`.

The normal one-iteration planning probe finished in 435.384 s with 50 paths,
zero unrouted nets, three complete regional assignments and an available native
manifest with zero final-scene conflicts. It required two local scope retries.
Raster shadow overflow remained 19; exact scene acceptance is distinct from that
proxy. The corpus subsequently launches a **30-iteration** detail command, not a
one-iteration command. Detailed output and final KiCad DRC are still pending.
This normal run used the policy-only binary for planning and the final
outside-blocker build for detail; preserve phase identities in its report.

Outside-blocker N6/R4 trial also timed out at 600 s. Its first epoch carries
24 history entries versus 23 in the policy-only control, consistent with the
new blocker-pair diagnosis. Three epochs complete; later oracle results still
have 9 opens. The policy and attribution fixes are verified, but reliable tier
completion is not established. Keep the broad default as explicitly requested;
do not describe this as benchmark qualification. Comparison:
`reports/benchmark_history/2026-09-06-default-physics-blocker-feedback.json`.

Normal detail audit ended by explicit SIGTERM just after the 600-second audit
cap (608 s observed before termination), not by the unchanged 2400-second corpus
timeout. Last completed detail epoch: 58 oracle violations / 10 opens, after an
earlier 98 / 10. No final output or KiCad DRC. This is **partial verification**,
not proof that the full-budget corpus run fails. Normal partial history and phase
binary identities: `reports/benchmark_history/2026-09-06-default-physics-normal-partial.json`.
All experiment subprocesses are stopped. No source board was modified.

## Experiment — 2026-09-06: Certify the seed before further negotiation

Failure and ranked hypotheses are preserved in
`reports/diagnoses/2026-09-06-seed-certification.md` and `.json`.
N1 previously accepted an internal native scene, while N6/N30 delayed or denied
that seed's physical certification opportunity. Full strength used N-R initial
iterations, disabled initial scope retries, and enabled advisory shadows solely
because later retries were available. More budget therefore changed the seed
and its legalization allowance before testing physical feasibility.

Research: VPR distinguishes first-iteration costs and keeps the best legal
solution on reconvergence (https://docs.verilogtorouting.org/en/latest/vpr/command_line_usage/).
OpenROAD separates global guides from detailed search/repair and DRC
(https://openroad.readthedocs.io/en/latest/main/src/drt/README.html).
Inference for Flux: preserve the opportunity to certify the initial seed; do not
equate a better proxy-ranked topology with a better physical incumbent.

Minimum change: reuse the existing seed-first epoch allocation for full strength,
allow the configured scope budget on the initial/final epochs, and keep default
initial shadows consistent with N1. Explicit shadow overrides still apply.
Subsequent epochs retain all compatible operators, physics and bounded feedback.
Failed seeds consume the remaining iteration budget through existing negotiation.
No new clearance rules, solver, topology backend or board-specific conditions.

Focused regression passes: N8 certifies on epoch one with full scope allowance,
and retains the same paths as N1; native detailed handoff remains legal.
Existing three-epoch failed-window test checks repeated physical consultation.
Full regressions and normal Tier 0 qualification are running. A separate N1
control runs both planning and detail at one iteration to test final legality,
not merely planning acceptance. Control planning binary is the prior Release
974f5509ed80b5c1fc05b3d31cb8920d33dd8629916f151a7f0a082f4dcbaf79;
its detail phase will use the new Release binary
40c9e0203cdcc5f574d0ec211f53bdab592ec3f713d04befe4cf4e1612c74cfb.
The changed branches do not alter N1/R0 behavior. Normal qualification uses the
new binary for both phases. Concurrent timing is not a controlled speed test.

Outcome: both N1 and normal-budget detail reach an internally connected,
copper-legal candidate (0 opens, 0 copper violations), but fail the curve gate
with six unrounded bends. Normal detail now accepts its first global seed rather
than consuming 26 global iterations first. Scheduling hypothesis supported;
keep it, but no tier promotion. Full history:
`reports/benchmark_history/2026-09-06-seed-certification-results.json`.
The extra failed-seed budget test passes: N6/R2 consumes epochs 1+3+2=6.

## Experiment — 2026-09-06: Native straight adapter seams

Final copper replay identifies all six unresolved bends at native straight
adapter boundaries; all 54 eligible board-level PH tasks committed successfully.
Those adapters were excluded from chain collection by blanket native protection.
Exact pairs/coordinates are in `reports/diagnoses/2026-09-06-native-straight-seams.json`;
ranked hypotheses and physical interpretation are in the companion `.md`.
This is a geometry-ownership boundary failure, not missing global routability.

Research: CGAL topology-preserving polyline simplification retains endpoints and
intersections as constraints while permitting interior geometry changes
(https://doc.cgal.org/5.4.4/Polyline_simplification_2/index.html).
Flux uses its existing PH and exact copper/connectivity transactions, not CGAL
code. Minimum change permits unlocked native straight adapters to enter the
board-level degree-two chain solver. Native arcs remain immutable; ordinary
straight runs without bends remain untouched. No final bend or DRC relaxation.

Micro-test: two separately certified straight native fragments meeting at a bend
must become connected, copper-legal curved geometry with zero unrounded bends.
It passes; the existing native-arc byte-for-byte test also passes. A first test
incorrectly encoded the bend inside one native fragment, which the existing
native join-angle gate correctly rejects before handoff; corrected the fixture
to reproduce the observed inter-fragment seam.
Full regression and normal Tier 0 rerun pending. Release binary:
ae82b85d29aa40fbc826fbecb257e1d5d2ef0ac3ee7052c36c38bc602dbd367b.
Benchmark recorder now retains structured failed-detail output before the same
qualification checks, instead of burying its metrics in a command-error string.

Outcome: 46 selected regression checks passed. Normal Tier-0 detail finished in
393.135 s with zero opens and zero internal copper violations, but six bends.
Those six now belong to one +5V chain: other native adapter seams were repaired.
53/54 PH tasks committed, 82 added tracks, 291 arcs and 5 vias. The remaining
chain fails eight minimum-radius checks and no clearance/topology checks, after
falling back to a source-polyline corridor. Keep for refinement; no tier promotion
and no final KiCad DRC evaluated. Diagnostic replay also finds one pre-existing
sharp native/ordinary arc join; this was not introduced by adapter unprotection
and remains a separate curve-quality issue, not a reason to relax qualification.

## Experiment — 2026-09-06: Curvature-sized certified fallback corridor

Failure: the remaining Tier-0 staircase uses a fallback guard initially only
31.25 micrometres wide per side (250 micrometre cell pitch divided by eight).
Required minimum bend radius remains 250 micrometres. A circular right-angle
rounding at that radius needs about 73.2 micrometres of centreline deviation.
Hypothesis: fallback guard construction, rather than available physical free
space, prevents sufficient rounding. Candidate ranking: first widen the proven
guard independently of sample spacing; next investigate seed simplification;
only then reconsider global corridor topology if exact physical space is absent.

Research: Zayou and Arslan, *Graph-Theoretic Bezier Curve Optimization over Safe
Corridors for Safe and Smooth Motion Planning*, https://arxiv.org/abs/2310.01190,
construct convex safe corridors around clearance-aware reference paths before
curve optimization. This supports separating corridor geometry from reference
sampling; it is not evidence that Flux's particular chain is feasible. No code
copied. Complexity here remains sampled cells times obstacle predicates, with
bounded logarithmic width shrinking; wider cells may increase clearance probes.

Minimum change: fallback requests initial half-width equal to the configured
minimum radius, capped by the control corridor, while retaining cell-pitch/4
sampling. Every cell still shrinks until exact free-space certification succeeds.
Also reject any cell containing a hole vertex: edge/diagonal chords alone miss
an entirely enclosed off-diagonal hole. No clearance, radius, connectivity or
final DRC requirement changes. Other callers retain their initial width policy.

Micro-test forces coarse NavMesh search exhaustion and requires a legal curved
fallback at 1 mm minimum radius. It passes before the change as well, so it is
coverage, not a reproduction of the measured staircase failure. Real Tier-0
normal-budget before/after comparison is the mechanism test; pending in
`build/curvature-corridor-tier0`. Keep only if benchmark evidence supports it.

Local evidence: an additional open right-angle micro-case also passes with either
width. Built `flux-replay-detailed-chain` (explicit, excluded-from-all diagnostic
target) to reconstruct the final recorded surrounding copper, freeze it, remove
exact named straight segments, and rerun just their unchanged chain. It checks
source revision, track identity, layer, width and every segment before replay.
Input is `reports/diagnoses/2026-09-06-tier0-staircase-chain.json`; geometry is
diagnostic data only, never selected by router code. The same real-chain replay
fails with six bends under the old width and succeeds with zero under the new
width; both have zero opens and zero internal copper violations. This supports
the hypothesis with a controlled mechanism reproduction. Full-board context can
still differ because other PH chains are realized concurrently before merging.
History: `reports/benchmark_history/2026-09-06-curvature-corridor-local.json`.
44 selected checks passed (102.04 s), including two KiCad DRC fixtures; the new
right-angle micro-case passed separately. Final Tier-0 DRC remains pending.

Full-board outcome: **keep**. Normal Tier 0 passes all 50 connections, zero opens,
zero internal copper violations and zero unrounded bends. All 54 PH tasks commit;
75 added straight tracks, 299 arcs, 5 vias. Planning 347.618 s, detail 397.483 s
(prior detail 393.135 s; concurrent build/test activity makes this only indicative).
KiCad 9.0.8 reports no error violations, no unconnected items and no schematic
parity errors under the unchanged benchmark rule set. Final board and layer PNGs
are in `build/curvature-corridor-tier0/tier-0-pt2399_delay/`.
History: `reports/benchmark_history/2026-09-06-curvature-corridor-tier0.json`.
The first failing tier is now Tier 1, `bm_svf`; begin a recorded N1 seed diagnostic
with a 900 s planning cap before selecting its next mechanism experiment.

Visual/shape caveat: the final Tier-0 front layer retains the previously diagnosed
native/ordinary arc join and some visibly wavy native curves. Passing the current
benchmark is not a claim that all inter-fragment curvature is visually ideal.
Keep this separate quality debt visible; do not conflate it with KiCad copper DRC
or the now-resolved straight-adapter/fallback-radius failure.

## Experiment — 2026-09-06: Tier-1 capacity warm-up comparison

Initial evidence in `reports/diagnoses/2026-09-06-tier1-seed.{md,json}`: the first
seed routes 25 nets topologically but retains sampled and corridor
overflow; exact physical extraction produces 129 conflict edges and 23 windows.
The committed reference uses layer excursions for the most repeated net pair.
Hypothesis: a brief capacity-negotiation warm-up before expensive regional work
may select necessary vias and substantially reduce physical scope.

Research: VPR explicitly separates first-iteration congestion from later present
and historical costs (https://docs.verilogtorouting.org/en/latest/vpr/command_line_usage/).
OpenROAD normally rejects congested global output unless `-allow_congestion` is
requested (https://openroad.readthedocs.io/en/latest/main/src/grt/README.html).
These support testing capacity convergence separately from geometry; they do not
prove that zero abstract overflow guarantees PCB legality. No external code used.

Minimum experiment: N4/R0 versus N1/R0, keeping full-strength physical operators,
rules, route backend and pitch unchanged. Four reaches this board configuration's
existing hard-capacity start; this is a diagnostic configuration comparison, not
a filename-dependent router policy. Measure vias, overflow, conflict component
size, runtime, and exact copper acceptance. Keep no new heuristic without an
actual outcome; both bounded comparison and seed result are pending.

Telemetry correction: the via counter is not the number of planned layer
transitions. The 78 selected paths have 24 transitions overall, but all seven
selected paths for the implicated pair remain on F.Cu with zero transitions.
Use path geometry and final materialized via counts, not that zero counter, for
the comparison. N4's early evidence already shows later global iterations lose
nets and the completeness-first checkpoint restores the original seed; simply
running four iterations may therefore not change physical work at all.

Tier-0 repeatability: a second normal-budget run passes the same 50 connections,
75 tracks, 299 arcs, 5 vias, zero opens/bends/internal violations and external
KiCad DRC. Detail wall time 494.367 s while frontier probes ran concurrently;
do not infer an algorithmic timing regression from that load. Record:
`reports/benchmark_history/2026-09-06-tier0-repeatability.json`.

Tier-1 N1/R0 reaches the 900 s diagnostic cap in residual alternative generation;
it has not exhausted the normal 7200 s benchmark budget. No final copper/DRC.
N1 and N4 selected 78-path geometry hashes are identical:
`b1c1535af4da65f70c943fa1608999ed771aa0355aba1356a149a79390554a43`.
Thus the warm-up-only hypothesis is unsupported at the selected checkpoint.
No warm-up policy change is retained. N4 residual pass still pending.

Next isolated control uses existing oracle-first feedback with 30 iterations,
legacy scheduling explicitly selected, and a 900 s diagnostic cap. This tests
whether physical history can change topology before exhaustive local repair.
Full-strength remains the normal default; the control is not a policy change.
Artifact root: `build/curvature-corridor-tier1-oracle-control`.

N4/R0 also reaches the 900 s cap in residual repair. The selected path geometry
is exactly identical to N1/R0; no capacity warm-up change is justified. The
oracle-first mode reaches successive global epochs quickly but reproduces the
previously documented Tier-1 804→724→804 provisional-violation oscillation,
with 6→8→8 opens. This is revalidation on the current Release build, not a new
algorithm discovery. It also uses the legacy advisory-shadow policy, so it is a
mode-level control, not an isolated change of timing alone.

Next backend control: two unaccepted direct-medial lane transitions persist in
the oracle epochs despite zero sampled/corridor overflow. Compare the same N1
full-strength configuration with only the manifest's `--direct-medial` omitted.
This tests whether the direct-medial embedding/handoff is the earlier blocker,
without changing pitch, clearance or physical operators. Added an explicitly
recorded diagnostic runner option `--disable-direct-medial`; no corpus manifest
or default routing policy changed. Do not infer a backend fix until measured.

The omit-only control is rejected at CLI validation: configured hard-capacity
repair requires a medial resource backing. No routing occurred and no geometry
claim is made. Correct the control to replace direct-medial with strict-medial,
which retains that backing and all repair knobs while testing grid-edge search
instead of direct medial edges. Runner option is `--strict-medial-control`;
the unusable omit-only option was removed. The failed invocation remains recorded
at `build/curvature-corridor-tier1-grid-control`.

Strict-medial control outcome: 16.91 s to epoch completion, only 21 selected
paths, 15 unrouted nets, 57 provisional opens and 2 provisional copper violations.
The runner's exit 0 is an expected unlocked-board incomplete probe, not a tier
pass. Exact metadata comparison confirms only direct→strict medial changed.
Reject a backend replacement: reduced collisions come with lost connectivity.
The diagnostic option remains available, but the corpus/default is unchanged.
History: `reports/benchmark_history/2026-09-06-tier1-strict-medial-control.json`.

Standalone width regression established: add a central track keepout to the
right-angle micro-case, preventing the open-space detour from disappearing.
The new width passes; temporarily restoring the old fallback call leaves two
unrounded bends and fails the test. Restored the curvature-aware call afterward.
This reproduces the general mechanism without any benchmark coordinates or
external telemetry fixture. The proposed larger recorded-copper fixture was not
added, since the compact synthetic case now catches the regression directly.

Oracle-first control also reaches its 900 s diagnostic planning cap, in residual
repair. It changes topology earlier, but does not establish a legal solution or
justify a default-policy change. All Tier-1 controls retain exact rules and none
qualifies the board. Keep the verified Tier-0 fixes and the diagnostic tooling;
reject warm-up-only/backend-switch remedies. Next frontier work should isolate
one connected native conflict window and feed its repeated outside-copper or
infeasible-lane witness into alternative allocation, rather than enlarging all
local search budgets. Full-strength remains the default.

Final verification: restored production CLI hash remains
`a207a0253daff3a0e09a663b9b85f86c2cf6b4a94ca054bfbfa5b2822fd0129e`,
identical to both Tier-0 DRC-passing runs. All 44 selected CTest checks pass in
81.36 s, including all 30 core cases (58.63 s), the obstacle-width regression and
two native-copper KiCad DRC fixtures. Python compilation and diff whitespace
checks pass. All benchmark/control subprocesses have finished; no abandoned
experiment is enabled in normal routing. Existing unrelated work is preserved.

## Experiment — Tier-1 early frozen-scope escalation (2026-09-06)

Failure: the N1 full-strength probe repeatedly rejects native candidates against
an outside-owned trace, then spends the rest of its 900-second diagnostic budget
in same-scope residual repair. Diagnosis and exact fragment identities are in
`reports/diagnoses/2026-09-06-tier1-scope-scheduling.json`. A normal 7200-second
manifest baseline is running on a frozen copy of the verified Tier-0 binary.

Hypothesis: changing repair ownership earlier avoids searching an unnecessarily
frozen problem. This is not a proof that every outside collision needs merging:
an alternative could avoid the blocker, so benchmark comparison is essential.

Research: [Tolpin, Justifying and Improving Meta-Agent CBS](https://arxiv.org/abs/1410.6519)
studies the tradeoff between separate-agent search and merging conflicting agents.
Flux is static copper routing, not time-indexed MAPF; use the selective grouping
idea only, without importing completeness or optimality claims. [KiCad shove
implementation](https://docs.kicad.org/doxygen/pns__shove_8cpp_source.html)
updates colliding line geometry and propagates it through a line stack; locked
joints can prevent the move. This reinforces the distinction between geometry
search and authority to move its blocker. No source code is copied.

Ranking: (1) reorder existing scope expansion; (2) push repeated pair evidence
into global allocation; (3) measure alternative-cache reuse. The first has the
smallest integration footprint. It uses the existing bounded, deduplicated exact
fragment witness collector and leaves all legality gates unchanged. Scope growth
can increase combinatorial cost, so this starts as an opt-in experiment.

Success: earlier scope progress and better final feasibility/runtime, with Tier 0
still DRC-clean. Regression: lost connections, new DRC failures, or expanded
search costing more without solving the targeted mechanism. The final allowed
scope pass must still run all existing local repair algorithms; no-budget and
duplicate-witness cases must not skip them. Tests will exercise these decisions.

Initial checks: all 44 selected CTests passed (126.51 s), including the new
budget/duplicate/legal-candidate scheduling assertions. The experimental CLI
hash is `d54d2d9d20a57b8090d9f8a13955ca5196e1102841311a49b87ad82a1752c6a1`.
Selected global path hashes match each control exactly: Tier 0 has 50 paths,
Tier 1 has 78. This isolates the regional scheduling comparison at the seed.

Intermediate evidence, not final routing success: Tier 0 follows the same
4→3→3 windows and 4→6→9 exact-witness sequence without the intervening
conflict-directed retries; planning finished in 414.572 s under concurrent load
with a native candidate (detailed/KiCad validation pending). Tier 1 escalates at
568.32 s after ordinary certification, growing exact witnesses 146→205,
conflict edges 129→187, and owned fragments 114→151. The frozen baseline was
still in residual repair after 1045 s. Scope growth may create oversized
components, so earlier escalation alone does not establish a routing improvement.

Broader verification: 120/127 non-corpus CTests passed in 231.39 s. Seven demo
expectations failed. Each failed command was rerun on the frozen baseline and
experimental binaries; full stdout hashes and exit codes match exactly. Record
these pre-existing failures without weakening their expectations. Details are in
`reports/benchmark_history/2026-09-06-early-scope-verification.json`.

Tier-0 result: PASS with the opt-in experiment, zero opens/copper violations/
unrounded bends and external KiCad DRC clean. The exported board is byte-for-byte
identical to the retained passing baseline (SHA256
`459bffc8258348a89eafcc275a35decdfc59e8afb1278b0731a58e0fd9df35ac`):
75 tracks, 299 arcs, 5 vias. Planning 414.572 s; detail 566.803 s. Concurrent load
precludes claiming a wall-time speedup. Full history:
`reports/benchmark_history/2026-09-06-early-scope-tier0.json`.

Tier-1 next observed limit: at 1115.95 s the expanded scope reports window 0
ineligible and window 2 exhausting 100000 discrete states. Earlier ownership
growth works, but now exceeds bounded regional solving capability. This is not
a Tier-1 solution. Investigate moving that explicit scope-size failure upstream
before spending on other independent native windows: enlarging an already
oversized exact component cannot make it fit the same regional size limit.

Refinement hypothesis: the first expansion contains a 15-net exact component,
exceeding the existing 12-net local limit. Its global-topology failure is known
at window construction (568.4 s), but is not published until after unrelated
alternative/native searches (1115.95 s), then expansion continues. Preserve the
size limit and return its existing `replan_global_topology` diagnosis immediately.
This is a search-budget limitation, not proof of physical unroutability. Selective
coupling research above explicitly warns that joint search grows with group size;
Flux already has the required bounded-window failure/feedback interface.

Small refinement to the opt-in scheduler: before corridor/portfolio construction,
if any required exact region exceeds the existing net/fragment bounds, emit its
ineligible assignment and return the checkpoint failure upstream. Do not fabricate
failures for unsearched eligible windows, publish partial success, relax bounds,
or claim a board solution. Decomposition-only inspection remains unchanged.
Add a micro-test containing one oversized interaction and a separate eligible
crossing, checking that the latter is not needlessly searched and that all source
fragments plus the explicit global-replan diagnosis remain available. Tier-0
output must remain identical; Tier-1 must actually reach new global epochs.

Refinement local verification: the oversized-plus-independent-crossing test
passes; all 44 focused CTests pass (110.38 s, core suite 80.54 s). CLI SHA256 is
`e44f6f83c0e98654aecb03d36bb8b90c5a26333eb0b792856f9c41fb7053e967`.
Normal-budget runs started in `build/tier1-early-scope-v2-20260906` and
`build/tier0-early-scope-v2-20260906`. V1 is frozen separately for reproduction.

Source-policy caveat found while reviewing the prior KRT oracle: Flux corpus
commands omit optional `--netclasses` snapshots. Their 0.25 mm default tracks
are not the 0.2 mm tracks used by the recorded KRT run; the project's 0.7 mm
minimum via diameter also needs authoritative ingestion before relying on the
internal checker alone. The external benchmark DRC still enforces the project.
Added an explicit caveat to the KRT report. No source rules, dimensions, manifest
or acceptance thresholds changed in the scheduling experiment. A future rule-
ingestion audit must stay separate from this comparison, not quietly substitute
narrower traces to obtain a tier pass.

## Control — strict-medial sampling resolution (2026-09-06)

Failure: the earlier 650000 nm strict-medial seed had 15 unrouted nets, 57
terminals without seeds, and no expansion-limited searches. Thus rejecting the
backend from that one resolution did not isolate whether coarse sampling lost
usable routes. It also had 31 accepted continuous-to-lattice reprojections.

Hypothesis: finer sampling improves terminal/tree connectivity without changing
copper width or clearance. [LaValle, notions of completeness](https://lavalle.pl/planning/node182.html)
distinguishes failure at a finite resolution from absence of a continuous path.
Flux's finite searches and adaptive domains are not proved resolution-complete;
this motivates a control, not a correctness claim. Test 250000 nm (a detailed
corridor scale, not Tier 0's global pitch) against the preserved 650000 nm
strict-medial N1 control. Keep all
other operators, dimensions and rules unchanged, without the new escalation flag.
Bound the diagnostic run at 900 s. A lower conflict count caused by missing nets
does not count as improvement. No backend/pitch default or manifest change.

Outcome: invalid input in 0.240 s, before routing. The existing planar-resource
model requires pitch at least width-plus-clearance, multiplied by sqrt(2) for
diagonals. With 0.25 mm traces and 0.2 mm clearance, the diagonal minimum is about
0.6364 mm. The guard correctly rejects 0.25 mm; do not bypass it without a
multi-cell copper occupancy model. Zero paths/zero reported unrouted nets here
are not successful connectivity. The unlocked harness's expected-probe exit 0
is not tier qualification. Reject this parameter-only remedy and preserve its
artifact in `build/tier1-strict-250-control-20260906`.

### Scheduler V2 intermediate result and next refinement

Tier 0 passes again with V2: planning 412.584 s, detail 435.301 s, external DRC
clean, identical board SHA256 and 75/299/5 tracks/arcs/vias. V2's first Tier-1
epoch finishes at 563.0 s (684 provisional violations, 5 provisional opens),
versus the baseline still working on that epoch after 2600 s. New topology is
indeed explored in an eight-iteration global epoch. At 643.9 s its oversized
regions return promptly, but at 644.04 s the scheduler replays the original
406-fragment / 146-witness / 483-alternative seed as a retained fallback, delaying
the next global feedback epoch again. This is not a solved tier.

Next bounded scheduler refinement: intermediate feedback epochs should physically
evaluate their newly generated global checkpoint before replaying old portfolios.
Keep all portfolios retained and evaluate them with full strength in the final
epoch. The initial and final epochs are unchanged; no search family or legality
gate is disabled. This trades intermediate fallback effort for earlier global
feedback, not a claim that an old topology is mathematically infeasible. Tests
must verify the same total global budget, one fresh intermediate candidate,
continued physical consultation, and retained final fallback availability.

Preserve V2's running executable unchanged while building a private V3 binary
against the updated static core; corpus detail phases must not silently switch
binaries midway through a run.

V3 verification: the new scheduler micro-test and all 44 focused CTests pass
(93.78 s; core 83.19 s). Private and release CLI SHA256:
`7f0a2894d1b42ae55bb9926a2713c0b41b6cd11dd1a6167bff6bebbcb0405e5f`.
On resumption both earlier long Tier-1 processes were absent with no run-result;
termination cause is unknown. Preserve these as inconclusive, not timeouts.
Baseline telemetry reached epoch 2 at 3120.26 s; V2 at 563.09 s but then replayed
the seed. Structured partial history is `2026-09-06-early-scope-interrupted-controls.json`.
V3 Tier-0 verification and Tier-1 N30/1200 s diagnostic are running separately.

## Experiment — exact source-segment conflict attribution (2026-09-06)

Failure: a synthetic two-net T-junction reproduces the suspected Tier-1 scope
loss without expensive native search. Net A has a horizontal carrier connection
and a vertical tap; net B crosses their junction diagonally. The quantized oracle
reports four raw violations, coalesced into two diagnoses identifying A/0–B/0 and
A/1–B/0. Regional mapping reports two mapped pairs but creates only A/0–B/0 and
owns two fragments instead of three. Nearest-fragment attribution discards the
connection identities and tie-breaks both reports onto the carrier. This is a
proven micro-test bug; its contribution to the complete Tier-1 failure remains
to be measured. No final physical solution is claimed.

Candidate ranking: (1) preserve typed source segment pairs through the oracle
and coalescing, (2) infer connection IDs by parsing diagnostic strings, (3) own
every nearby same-net branch. Prefer (1): precise, independent of diagnostic text,
and does not enlarge windows with unrelated copper. Reject (2)'s brittle string
contract and (3)'s unmeasured over-expansion.

Research: [KiCad SHOVE](https://docs.kicad.org/doxygen/classPNS_1_1SHOVE.html)
retains item/line links and operates on collision participants and connected
clusters. This supports preserving physical participant identity, not copying
its Manhattan/line solver or treating proximity as ownership. No external code
is copied. The source-page fetch timed out; the available class documentation
is sufficient only for this representation-level comparison.

Hypothesis: exact planar source-edge pairs let regional extraction own both
branches immediately, avoiding retries against an accidentally frozen tap.
Keep coarse witnesses' spatial fallback; typed witnesses that no longer match
source geometry must remain explicitly unmapped, never silently reassigned.
Coalescing must retain every distinct pair. Match source edge ranges across
collinear fragment merging and validate layer/source geometry. Complexity is
linear in fragments per reported participant (same order as existing nearest
search), with bounded witness storage. Via ownership is unchanged.

Success criterion: two exact edges and three owned fragments on the junction,
including after path reordering and collinear splitting; stale provenance is
unmapped. Run the existing suite and both frontier/lower-tier benchmarks before
promotion. V3 benchmark binaries remain unchanged to isolate scheduling results.

Local result: all junction variants pass, including preserving three source pairs
inside two coalesced diagnoses after splitting the carrier. All 44 focused CTests
pass (104.90 s; core 94.55 s), and three interrupted-history tests pass. Frozen
attribution CLI SHA256 is `ff4f0e19f1abb1bc4e631df434a0adc2c332a4c7c94ec0eec4d76dba5c231909`.
Full builds succeed without new missing-initializer warnings. Exact source pairs
and regional mapping counts are exposed in CLI JSON. Broader CTests and corpus
runs remain in progress; no tier promotion yet.

Tier-1 intermediate evidence: the same seed has 146 oracle diagnoses and 684
provisional violations/5 opens. Typed mapping identifies 317 exact pairs with
zero unmapped (previously 135/3) and 19 windows owning 127 fragments. It exposes a
13-net/59-fragment required component immediately at 74.20 s, before native
search. V3 spatial attribution reached an oversized scope only after native
search/expansion at 517.61 s in its concurrent run. This supports earlier precise
ownership, not a controlled whole-run speedup. The third V4 epoch still has
2663 provisional violations and 3 opens, so additional global feedback has not
yet produced physically feasible copper. Do not optimize for mapped-pair counts
or interpret earlier budget rejection as routing completion.

V4 Tier-1 outcome: the full N30 budget finishes in 457.946 s, below its diagnostic
1200 s cap. Still incomplete: 78 combinatorial paths, sampled overflow 42,
corridor overflow 1, total overflow 50, selected provisional oracle 1428 copper
violations/2 opens, 2 malformed paths, 1196 tracks and 41 vias. No detailed export
or KiCad DRC was reached. Harness expected-PROBE success is not tier qualification.
The attribution mechanism is fixed locally and upstream feedback is reachable;
physical routability is not solved. Keep the scheduling policy opt-in.

V3 Tier 0 passes and remains byte-identical to the restored baseline
(`459bffc8258348a89eafcc275a35decdfc59e8afb1278b0731a58e0fd9df35ac`).
V4's broader suite passes 121/128; all seven failures match the frozen baseline
exit codes and JSON after removing only new diagnostic fields. Expectations were
not relaxed. V4 Tier-0 detail verification remains running.

## Experiment — coherent quantization of endpoint via chains (2026-09-06)

Failure: completed V4 Tier-1 report now makes two malformed-path diagnoses visible.
The last telemetry checkpoint has coincident source transitions at path ends
that become non-coincident under the oracle's snapping rule: e.g. net 12,
connection 2, endpoint (56.74, 91.99) mm stays fixed while its adjacent via-layer
point snaps to (56.7, 92.0) mm. Similar examples occur on nets 21 and 14. The last
telemetry checkpoint need not be the final selected portfolio, so three observed
examples must not be equated with its two selected malformed paths.

Earliest loss: the provisional oracle quantizes vertices independently and exempts
only first/last array entries, splitting a single physical via site across layers.
This is not evidence that the global router selected a lateral layer jump.
Conversely, rounding can conceal a genuinely non-coincident source transition.

Research: [CGAL snap-rounding manual](https://doc.cgal.org/latest/Snap_rounding_2/index.html)
treats an arrangement vertex consistently when replacing it on a finite lattice.
Flux needs only the shared-site invariant here, not a new arrangement solver or
an ISR/topology-preservation claim. No CGAL dependency/code is introduced.

Ranked options: (1) snap consecutive equal-position source vertices as one site,
preserving the site if it touches a protected endpoint; (2) stop protecting pad
endpoints; (3) remove the coincidence check. Choose (1), linear in path length.
Reject (2) because it changes terminal policy and (3) because it hides real
malformation. Validate source transition coincidence before quantization so it
cannot be accidentally repaired by rounding.

Micro-test: off-grid start/end F.Cu→In1.Cu→B.Cu chains must yield two vias and one
connected track both with and without endpoint protection. A source transition
displaced by one nanometre, but rounded into the same cell, must remain malformed.
Success is removal of false malformation, not acceptance of colliding copper;
all DRC/connection checks remain authoritative. Preserve the V4 binary/results
for a matched next experiment.

Micro-test outcome: fails on V4 with one malformed path/one open; passes after
shared-site quantization and pre-snap source validation. All 45 focused CTests
pass (101.02 s, core 90.37 s). V5 is frozen at
`892d1078ace2857b425ef23c1ffd1a67429156ef1f2c817bf00b27f479186ce4`.
Tier-1 initial selected paths match V4 exactly (including widths; comparison
hash `a88a7b34663d93454f018cc283935a853fca18e759df99835e5599ca197166bd`).
Provisional opens drop 5→0, while violations rise 684→764 because previously
omitted malformed paths now participate in DRC. This supports the false-oracle-
failure hypothesis, not routing legality. V4 Tier 0 has also now passed external
DRC with a byte-identical output board. V5 Tier-0 default and opt-in controls and
the complete-budget Tier-1 search remain in progress.

V5 intermediate refinement evidence: epochs 1–3 each retain zero provisional
opens (764, 1297, 2216 copper violations respectively). Epoch 4 constructs 25
required windows at 463.57 s with no window exceeding the existing size limits,
so it enters ordinary portfolio/native search instead of returning immediately.
This differs from V4, where every checkpoint was locally oversized. It is not a
proof that the geometry fits or that the extra runtime is worthwhile. Kernel
VmHWM sampled during the V5 run was 727036 KiB; this is an observed high-water
mark up to the sample, not the final whole-run memory peak.

Separate next audit, not implemented: the planner's partial-candidate ranking
counts incomplete *assignments*, while early oversized-scope return emits only
the ineligible assignments and leaves other windows unsearched. Unsearched
windows must not be mistaken for solved windows when comparing progress. Any
ranking refinement needs a synthetic comparison and benchmark isolation; do not
fabricate failure assignments merely to influence scoring.

Final V5 verification: Tier 0 passes under both default kitchen-sink policy and
the opt-in scheduler. Both produce the same baseline board SHA256, 75 tracks,
299 arcs, 5 vias, zero opens, zero internal copper violations and clean external
KiCad DRC. Default planning/detail times are 432.762/521.395 s; opt-in times are
439.395/501.928 s under concurrent load. No shape or clearance relaxation.
The broader suite passes 121/128 in 177.35 s, with the same seven baseline-
identical demo contracts; focused verification passes 45/45.

Tier 1 reaches the 1200 s diagnostic cap during ordinary native realization of
epoch 4. Portfolio construction alone spans 463.72–918.20 s (89 corridors,
649 alternatives, 25 assignments). Native candidates then expose movable outside
blockers; e.g. candidate `kicad-net-5:3:0:5` versus blocker
`kicad-net-9:1:0:6` on F.Cu, including 450000 nm track/arc clearance deficit.
The ordinary pass has not finished by the cap, so after-pass escalation cannot
yet act. This is not a normal 7200 s timeout verdict and no final DRC is available.
Do not claim that V5 preserves V4's 458 s full-search runtime: corrected feedback
now leads to actual native searches, which remain expensive and unsuccessful in
this bounded run. Retain the two proven correctness fixes and keep scheduling
changes experimental. All run histories and blocker telemetry are preserved.

Next frontier hypothesis: the whole ordinary portfolio/native batch delays
action on exact scope blockers already known during candidate certification.
First reproduce a multi-window case where an exact outside participant is needed,
then test bounded early scope escalation inside the batch while retaining final
fallback search. Separately audit unknown-versus-solved window ranking above.
Neither hypothesis authorizes accepting incomplete copper or skipping final DRC.

Handoff state: all benchmark subprocesses finished; reports are persisted, no
tier was promoted, and no clearance or fixture rule changed. Changes remain
uncommitted: these fixes depend on substantial pre-existing uncommitted router
infrastructure, so committing isolated hunks would not form a buildable revision
and committing the whole worktree would mix unrelated experiments. Frozen CLI
binaries and source/test logs preserve the individual comparisons without
discarding or staging the user's other work.

## Experiment — in-batch frozen-scope escalation (2026-09-06)

### Failure and hypothesis

V5 reaches Tier 1's fourth physical epoch with 25 windows and then spends the
rest of a 1200-second diagnostic budget in native realization. Exact frozen-
copper conflicts are visible before the whole batch returns. The hypothesis was
that a retained exact outside participant could interrupt unrelated native work,
expand ownership, and rerun the regional problem earlier.

This follows KiCad PNS's participant/linked-item model rather than treating
proximity as ownership: [KiCad `SHOVE`](https://docs.kicad.org/doxygen/classPNS_1_1SHOVE.html)
and its [implementation](https://docs.kicad.org/doxygen/pns__shove_8cpp_source.html)
operate on explicit collision participants and connected items. Flux retained
its own continuous/corridor representation and copied no external code.

### Changes and controls

V6 interrupted immediately on a rank-zero native witness and marked untouched
candidates fail-closed. V7 removed a deterministic parallel batch barrier but
kept immediate interruption. Both failed Tier 0: earlier physical epochs
expanded on transient candidate conflicts, selected 14 rather than 10 native
spline runs, and left 30 non-junction bends without PH realization. Zero opens
and zero copper reports do not rescue malformed geometry; both variants are
rejected.

V8 required at least eight windows and waited for a complete per-window
shortlist reduction. Tier 0 has four windows, so the scheduler remained dormant
and passed with the exact V5 output SHA256: 75 tracks, 299 arcs, 5 vias, zero
opens, zero internal violations, and clean KiCad DRC. V8's Tier-1 run observed
the known net-5/net-9 blocker at ranks 2–4, but ordinary failure reduction chose
a different diagnostic candidate and no escalation fired. It was stopped at
980 seconds once that no-op was demonstrated.

V9 preferred an exact expandable witness only among failed candidates in a
large interruptible batch; any legal candidate still won. All 31 core tests
passed. The 1200-second Tier-1 run reproduced the same first three exact-oracle
checkpoints (764, 1297, 2216 violations; zero opens) and the same 25-window,
226-fragment, 89-corridor, 649-alternative fourth checkpoint. Portfolio
generation completed at 705.598 seconds. No completed window retained a new
outside participant as its decisive failure, so scope escalation remained zero.

### Interpretation and decision

The hypothesis is disproven. The net-5/net-9 conflict was real but transient
candidate evidence, not proof that every candidate in its window required wider
ownership. Immediate use regressed Tier 0; delayed use never fired on Tier 1.
Revert the scheduling code rather than retain an inactive heuristic. Preserve
the frozen binaries and artifacts as negative results.

The run exposes a stronger next mechanism. Windows 2–11 and several later
windows each started 16 candidates: the full eight-state active tier followed
immediately by an eight-state fallback tier, while later windows remained
uncharacterised at timeout. Regional spline tiers are scheduled depth-first per
window. Next experiment: evaluate the active tier breadth-first across every
independent window, reduce that complete checkpoint, then promote and evaluate
fallback tiers only for failed windows, still in parallel. Do not remove any
push/shove, rip-up, topology, quantization, native-copper, connectivity, or KiCad
DRC gate; change scheduling order only.

## Experiment — Breadth-first regional spline tiers

### Failure

On Tier-1 `bm_svf`, four independent native workers evaluated each window's active shortlist and
fallback reserve depth-first. In the V9 1,200-second trace, thirteen early windows reached sixteen
candidate starts while several later windows received only one to six. Topological connectivity
was complete, but the batch did not establish a complete first-tier physical checkpoint.

### Hypothesis

Completing the active tier for every independent window before promoting any fallback state would
remove scheduler starvation without pruning route diversity or bypassing PH, XPBD, native-copper,
connectivity, or DRC gates.

### Research

OpenROAD TritonRoute documents a staged detailed-routing flow with search-and-repair boundaries.
Flux applies the same bounded-stage principle to its continuous regional windows while retaining
its existing exact algorithms: https://openroad.readthedocs.io/en/latest/main/src/drt/README.html

### Change

The parallel regional executor now runs active candidate shortlists with fallback promotion
disabled, performs the existing stable reduction, then advances failed windows through fallback
waves behind explicit deterministic barriers. Completed/ineligible windows are not copied into a
fallback task. Shared PH caches retain their invocation lifetime, and telemetry publishes every
tier transition and pending-window count.

### Before

Controlled V9 (`--prioritize-frozen-scope-expansion`, 1,200 seconds) reached the same epoch-4
portfolio at 705.598 seconds. Fallback work began at 788.886 seconds. Thirteen early windows
consumed all sixteen active-plus-fallback candidates while later windows remained only partially
characterised. There was no final DRC result.

### After

V10 reproduced the first three checkpoints exactly: 764, 1,297, and 2,216 exact-copper
violations, all with zero opens. Its identical epoch-4 problem contained 25 windows, 226 owned
fragments, 89 corridors, and 649 alternatives. The portfolio was ready at 684.785 seconds. Every
eligible window received its active tier (at most eight candidates) before the barrier fired at
1,011.128 seconds; only then were fifteen failed windows admitted to fallback. Eleven fallback
windows were promoted before the same 1,200-second timeout.

Tier 0 passed with 50 paths, 75 tracks, 299 arcs, 5 vias, zero opens, zero internal violations,
and zero external KiCad DRC violations. Its generated board SHA-256 remained byte-identical to the
V5/V8 reference (`459bffc8258348a89eafcc275a35decdfc59e8afb1278b0731a58e0fd9df35ac`).

### Interpretation

The hypothesis is strongly supported. The scheduler now obtains complete active-tier evidence
before reserve work and lower-tier copper is unchanged. Tier 1 still does not finish: epoch-4
portfolio generation costs about 356 seconds and its active native tier costs about 326 seconds.

### Decision

Keep. The next isolated experiment is an eight-worker regional width on the 24-core benchmark
host; candidate budgets, stable reduction, tier ordering, and exact gates remain unchanged.

## Experiment — Eight regional spline workers

### Hypothesis

The four-worker breadth-first batch underuses the 24-core host. Doubling only the independent
window-worker bound should shorten the active tier while preserving stable reduction, candidate
ordering, and all exact legality gates.

### Before and after

The controlled four-worker V10 active tier took 326.343 seconds after its portfolio became ready.
V11 reached the same 25-window, 226-fragment, 89-corridor, 649-alternative problem and reproduced
the preceding exact checkpoints (764, 1,297 and 2,216 violations, all with zero opens). Although
its absolute barrier occurred 23.5 seconds earlier, its portfolio was ready 24.5 seconds earlier.
The isolated eight-worker active tier therefore took 327.324 seconds: about one second slower.
Observed resident memory rose from roughly 1.0 to 1.6 GiB and CPU demand from 119% to 205%.

Tier 0 remained clean and byte-identical with 75 tracks, 299 arcs, 5 vias, zero opens, zero internal
violations, and zero KiCad DRC violations, but it exposed only three independent windows and did
not exercise the increased bound.

### Interpretation and decision

The hypothesis is disproven. A single large window, whose candidates remain deliberately
sequential, determines the barrier. Revert the worker bound to four and retain the V11 binary and
run artifacts as a negative result.

The candidate trace localizes the next experiment: each slow window-1 candidate takes roughly
40–45 seconds, while its GPU XPBD solve takes only about 0.3 seconds. Native candidate assessment
then calls whole-board copper DRC and discards every report that lacks a generated object. Test the
existing exact dynamic-object DRC API at that call site; it retains generated-to-static and mutual
generated-copper checks while omitting immutable source-only pairs. Final whole-board DRC and
external KiCad DRC remain mandatory.

## Experiment — Exact dynamic regional native DRC

### Failure and hypothesis

The Tier-1 active spline tier remained a 326–327 second barrier. Candidate traces appeared to put
roughly 30 seconds between GPU XPBD completion and native-copper rejection, and regional assessment
ran canonical whole-board DRC even though it retained only reports involving proposal objects. The
hypothesis was that the existing selected-object DRC API could perform the exact proposal-facing
check without revisiting immutable source-only pairs.

TritonRoute uses the analogous ownership boundary: a detailed-routing worker queries intersecting
shapes and owners, modifies only objects in its standard box, retains fixed surrounding shapes in
its extended box for cost and DRC, and commits only a clean private result. See the
[TritonRoute paper](https://vlsicad.ucsd.edu/Publications/Journals/j133.pdf) and the maintained
[OpenROAD DRT integration](https://github.com/The-OpenROAD-Project/OpenROAD/blob/master/src/drt/README.md).
Flux keeps its continuous window model and final whole-board/KiCad validation; the comparison
supports only the selected-proposal versus fixed-context boundary.

### Parity failure and correction

The first V12 implementation failed the Tier-0 control despite reporting zero opens and zero copper
violations: it selected a different regional trajectory and detailed realization rejected 79
non-PH bends. A byte-identical candidate differential isolated window 3, rank 6. Canonical DRC
marked both routes illegal, while dynamic DRC marked the route owning endpoint vias legal.

`check_dynamic_copper_rules` checked selected tracks and arcs against static vias, but its selected-
via loop omitted the symmetric checks against static tracks and arcs. V13 added those relationships
with the same distance predicates, clearance rules, codes and object order as canonical DRC. A
deterministic micro-test now proves equality between the canonical proposal-relevant subset and
dynamic DRC for selected-track/static-via and selected-via/static-track cases, while an unrelated
static-static violation remains excluded.

### Results

V13 restored exact Tier-0 behaviour: 50 paths, 75 tracks, 299 arcs, 5 vias, zero opens, zero internal
violations and zero KiCad DRC violations. The generated board SHA-256 is still
`459bffc8258348a89eafcc275a35decdfc59e8afb1278b0731a58e0fd9df35ac`. Regional metrics also returned
to the control values: 5 native windows, 299 pieces, 29 violations and 3 legal windows.

The controlled Tier-1 run reproduced all upstream checkpoints and the same 25-window,
226-fragment, 89-corridor, 649-alternative fourth epoch. Its portfolio was ready at 698.526 seconds
and the active-tier barrier fired at 1025.571 seconds, a 327.045-second active duration. This is
indistinguishable from V10's 326.343 seconds and V11's 327.324 seconds. Across all active and
fallback work observed before timeout, 209 bounded DRC calls consumed 14.335 seconds total with a
33.885 ms median. The critical early calls were about 25–36 ms.

### Interpretation and decision

The correctness boundary is supported and the asymmetric-via defect was real, so keep the fixed
selected-object DRC and its parity tests. The performance hypothesis is disproven: native DRC is
not the 30–40 second candidate bottleneck. A replay of the first slow window's 20 recorded PH
Bezier chains likewise completed all native line/arc fits in about 2.4 ms, disproving native fitting
as the cause. Retain phase-level timing and the diagnostic native-fit replay tool; next isolate PH
interpolation selection, XPBD preparation, elastic/PNS shoves, trajectory polish, adaptive pair
repair, post-polish certification and quantization feedback on that exact window.

## Experiment — Phase profile and parallel exact spline neighborhoods

### Failure and measured mechanism

V14 added behavior-free timing around each regional candidate phase. A release-build control
reproduced the exact first three Tier-1 checkpoints and the same 25-window, 226-fragment,
89-corridor, 649-alternative fourth epoch. Two window-1 candidates took 40.78 and 42.22 seconds.
Joint shove consumed 13.28/13.41 seconds, trajectory polish 12.31/12.14 seconds, adaptive pair
repair 4.18/4.29 seconds, and interpolation selection 6.01/8.19 seconds. XPBD consumed only
0.43/0.57 seconds; native fit, quantization and exact DRC remained milliseconds.

The three largest repair phases all use `optimize_quintic_ph_trajectory`. Its exact move proposals
are immutable and already stored by candidate slot, but regional calls left its bounded worker pool
at one. The experiment therefore changes execution width, not search breadth: four workers evaluate
the same exact neighborhood, and the caller reduces results in the original deterministic order.
The four outer window workers bound nested exact work to at most sixteen threads on the 24-core
benchmark host.

This follows the independent-private-work pattern documented by
[TritonRoute](https://vlsicad.ucsd.edu/Publications/Journals/j133.pdf) and maintained in
[OpenROAD DRT](https://github.com/The-OpenROAD-Project/OpenROAD/blob/master/src/drt/README.md),
without importing their grid representation or relaxing Flux's exact continuous/native/KiCad
acceptance gates.

### Results

All 32 core units pass. Tier 0 remains byte-identical and fully legal: 75 tracks, 299 arcs, 5 vias,
zero opens, zero malformed bends, zero internal violations and zero KiCad DRC violations. Global
runtime fell from 298.82 to 254.10 seconds, detailed runtime from 400.07 to 366.22 seconds, and
global regional-spline realization from 10.62 to 6.44 seconds.

On Tier 1, window-1 rank 0 fell from 40.78 to 26.99 seconds. Joint shove fell 13.28→6.00 seconds,
trajectory polish 12.31→5.06 seconds, and adaptive pair repair 4.18→2.54 seconds. The complete
active-tier duration fell from 327.05 to 205.86 seconds (37.1 percent) after subtracting portfolio
readiness. All fifteen failed windows completed their fallback wave by 1104.14 seconds, whereas
V13 promoted only eleven before the 1200-second cap.

The additional time exposed the next stage rather than a legal solution. A conflict-directed
master built an eight-fragment core but found zero complete states (`stop=2`). Three residual cones
were then built; the run reached 170/384 residual alternative checks with 168 accepted at timeout.
No final KiCad DRC result exists, so Tier 1 remains unresolved.

### Decision and next hypothesis

Keep. The hypothesis is strongly supported by exact lower-tier output identity, measured phase
speedups and greater Tier-1 search coverage. The benchmark frontier has moved downstream: run V15
beyond the short diagnostic cap to finish residual-cone generation/realization and identify the
first physical failure after conflict-directed repair.

## Experiment — Forward checking in residual bundle allocation (V16)

### Failure and control

An extended V15 run reached the ordinary active-tier barrier in 205.883 seconds,
reproducing the shorter control's 205.858 seconds. Ordinary spline work completed
at 1067.534 seconds. Residual cone 0 then exhausted 100,000 bundle states with no
access/lane-feasible selection; cone 1 retained 584 selections but had 19–22
native-illegal routes in each observed active realization. The 1800-second limit
expired in cone 1's first reserve wave. Cone 2 remained unexamined.

The 178-second cone-0 portfolio duration includes generation, preliminary
screening, DFS and leaf validation. It cannot be attributed solely to enumeration.
The original trace also does not distinguish raw DFS leaves from late access or
lane rejection. This measurement gap is part of the experiment.

### Hypothesis and ranked alternatives

The bounded DFS previously checked via identity and an aggregate channel envelope,
then applied access and run-layer constraints only to complete choices. Moving
proved pairwise contradictions into forward checking should reach useful choices
under the same state bound. This is the smallest change to the existing allocator.
Maintaining arc consistency is the second candidate; an exact SAT/CP-SAT regional
backend is third because it requires a larger model and integration. Increasing
the state limit is not justified by the current evidence.

[Haralick and Elliott](https://www.ijcai.org/Proceedings/79-1/Papers/004.pdf)
motivate forward checking and fail-first search. Their binary constraint setting
matches template-pair compatibility, but Flux still needs higher-order lane and
coupled-net constraints. [Wood and Rutenbar](https://users.ece.cmu.edu/~rutenbar/pdf/rutenbar-satisfpga97.pdf)
show how simultaneous routing resource constraints can express regional
feasibility; their fixed FPGA fabric differs from Flux's continuous copper.
No external implementation was copied.

### Change and bounds

The final bundle search keeps the existing variable and value order, caches exact
pairwise access support, rejects incompatible adjacent alternative layers, and
checks remaining domains before descending. Complete leaves check coupled-net
and access constraints. A bounded access query that exhausts its budget remains
unknown and cannot prune a branch. Preliminary screening and column generation
keep their control behavior, isolating the final enumeration experiment.

Pair support has at most quadratic storage in the template count; the existing
DFS remains exponentially bounded by `max_bundle_search_states`. Each distinct
pair receives one access query with its own bounded budget, avoiding Experiment
15's rejected shared-budget prefix behavior. Full access/lane certification still
runs downstream. This may add overhead on weakly constrained products, which the
new per-search timer and rejection counters will expose.

Diagnostic telemetry records replayable allocator inputs, raw retained leaf
counts, early access/layer pruning, domain wipeouts, late access/coupling/packing
rejections and search duration. The standalone replay compares both policies on
the same scene without rerunning global routing or generating copper.

### Local verification

A synthetic twelve-fragment binary product contains an early terminal escape
contradicted by the final fragment. At a 128-state bound the control remains below
the bad escape; forward checking reaches the good escape. Exhaustive checking
retains all 2048 feasible combinations. A nominated via collision remains
supported when relocation is available; exhausting a one-state access query also
retains the topology. Removing relocation proves and rejects the fixed collision.

The initial full core run passed 32/33 tests. The remaining crossing fixture
completed with legal copper, but its historical assertion expected one shared
partial-access budget. The assertion now measures the per-query bound; the new
micro-test checks the stronger requirement that exhaustion never prunes a viable
route. The focused crossing test passes after the worker shutdown correction
below. A further micro-test preserves a completion where an earlier fragment
supplies another runup representative of the same physical via. Pairwise queries
relax to all equivalent representatives; complete-leaf checks still certify the
actual representative. The GPU reservoir now applies the same leaf checks.

### Tier-0 result and Tier-1 verification

V16 passes Tier 0: 50 paths, 75 tracks, 299 arcs, 5 vias, zero opens, zero internal
copper violations and zero KiCad DRC violations. Copper SHA-256 remains
`459bffc8258348a89eafcc275a35decdfc59e8afb1278b0731a58e0fd9df35ac`.
Global/detail durations were 269.751/329.697 seconds with some concurrent unit-test
work, so this is a correctness comparison rather than isolated timing evidence.
Tier 1 is running with V16b, which includes the representative guard, GPU-reservoir
leaf checks and the independent worker-lifetime correction below.

## Correctness repair — Exact spline worker shutdown

The focused crossing test stalled at 71 CPU seconds, with all nine threads waiting
on futexes and no progress over several minutes. Source audit found two shutdown
hazards in `BoundedWorkerPool`: the stop predicate changed outside the condition
variable's mutex, permitting a missed wakeup; and implicit member destruction
destroyed synchronization state before the worker vector joined its threads.
The [C++ condition-variable requirements](https://eel.is/c++draft/thread.condition.condvar)
and [jthread lifetime rules](https://eel.is/c++draft/thread.jthread.cons) establish
the required wait-state lifetime. Debugger attachment was unavailable, so the
test's precise blocked stack was not captured.

A regression repeatedly starts and destroys four-worker pools through short
public spline optimizations. The old implementation passed three batches of 512
optimizations, then hung in batch four under a three-second process timeout.
After requesting stop under the mutex, notifying after unlock, and explicitly
joining before member destruction, all 40 batches (20,480 optimizations) passed
in 25.321 seconds. The previously stalled crossing regression also passes.

Keep the lifetime fix. It changes no routing or geometric acceptance policy.
Earlier Tier-1 diagnostic timeouts showed active search; this finding is not
evidence that those runs were deadlocked. Full core and benchmark verification
continue alongside the forward-checking experiment; the complete core suite
passes 34/34. Results are separately recorded
outcomes in `reports/benchmark_history/2026-09-06-spline-worker-shutdown.json`.

The isolated worker fix and lifecycle regression are committed as `d1a06aa`;
the other ongoing router changes remain uncommitted. The broad integration run
passed 118/129 checks. Seven failures reproduce the previously recorded demo
expectation failures. Two duplicate corpus jobs were deliberately terminated
because the exclusion expression missed their `flux_cli_` prefix; these are
cancellations, not routing verdicts. A constrained-export test hit the imposed
120-second limit and its dependent DRC lacked an output board. That pair needs a
rerun with its normal budget. Raw results: `build/forward-check-v16b-ctest.xml`.

The constrained-export rerun passes all four setup/export/KiCad-DRC/cleanup
checks in 85.30 seconds with a 600-second ceiling; the export itself takes
84.88 seconds. Combining that rerun with the first integration pass gives
120/127 passing non-core, non-corpus checks, with the same seven previously
recorded demo expectation failures. The independently run core suite passes
34/34. Rerun artifact: `build/forward-check-v16b-constrained-rerun.xml`.

### Additional V16 diagnosis — Forced channel occupancy

Replay of ordinary window 0 isolates a two-fragment aggregate capacity conflict:
channel 61 offers 1.092 mm, but two singleton-domain occupants require 1.100 mm.
Both policies stop at 20 states without exhaustion. The machine-readable diagnosis
records exact fragment identities and coordinates in
`reports/diagnoses/2026-09-06-tier1-forced-channel-pair.json`. This is an infeasible
recorded discrete portfolio, not proof that physical copper cannot fit. These
fragments do not occupy parallel full-length strips.

For a follow-up, rank forced-occupant-directed column generation first, a
longitudinal cross-section occupancy micro-test second, and upstream complete-run
rerouting third. [TritonRoute's guide preprocessing](https://vlsicad.ucsd.edu/Publications/Conferences/363/c363.pdf)
splits, merges and bridges guides before panel-level constrained routing. Its
preferred-direction panels differ from Flux's continuous corridors; the useful
idea is revisiting the representation before committing detailed geometry.
[Current OpenROAD track assignment](https://raw.githubusercontent.com/The-OpenROAD-Project/OpenROAD/master/src/drt/src/ta/FlexTA.cpp)
also separates initial assignment from search/repair. No implementation was
copied, and no new capacity heuristic has been activated while V16 is measured.

## Experiment — Forward propagation of existing resource constraints (V17)

### Failure and hypothesis

V16b residual cone 0 still reaches 100,000 states without a choice, but its
portfolio duration falls from 178.032 to 84.850 seconds. Exact replay explains the
mechanism: the control emits 87,447 unchecked leaves; access forward checking
rejects five impossible via pairs and then repeatedly hits the capacity gate
(21,422 capacity rejections). Bundle enumeration itself takes only 0.315 seconds
in the instrumented run. Much of the old cost was downstream screening.

Hypothesis: future domains are checked for access support, but not for the same
aggregate capacity and nominated-via identity constraints already enforced on a
selected fragment. Checking those existing constraints before descending should
escape the capacity-dead prefix without increasing the state budget or changing
the physical model. This refines the same forward-checking family researched in
V16; it adds no new routing algorithm or relaxed rule.

### Minimal change and local result

Precompute each template's resource use. During forward checking, reject a future
option if it cannot join the current nominated via identities or channel envelope.
Counters distinguish capacity and identity support failures. The extra checks
are bounded by the existing DFS state/domain sizes; storage is linear in template
resource memberships. They remain opt-in while the benchmark variant is tested.

On exactly the recorded cone-0 scene, access-only search retains zero choices;
resource propagation retains 48,550 access/coupling/aggregate-capacity-screened
choices under the same 100,000-state cap. No access query exhausts its bound.
Replay took 22.902 seconds under concurrent benchmark load. This is not a lane-
packing or copper result. A twelve-fragment capacity ordering trap is escaped
under 128 states, and all 2048 feasible combinations survive exhaustive checking.
An analogous nominated-via-identity ordering trap also passes, including exhaustive
preservation of all 2048 feasible choices.

### Verification plan

Freeze a binary with resource propagation enabled, then restore the working-tree
default to disabled until Tier-0, full-core and Tier-1 results justify retention.
Use unchanged source boards, clearances and 100,000-state bounds. Success means
the newly reachable choices survive downstream packing and improve physical
repair; producing more combinatorial choices alone does not promote a tier.

The enabled V17 binary (`205a54a904c6cb98d61239fda1d613710f749ccf3a365787aa1fb49c7af80445`)
passes all 34 core tests. Its Tier-0 global stage finishes in 261.619 seconds;
detailed routing and final KiCad validation are still pending. Both the source
default and the normal Release CLI have been restored to resource propagation
disabled; only the frozen V17 benchmark binaries enable it.

Tier 0 passes with V17: 50 paths, 75 tracks, 299 arcs, 5 vias, zero opens, zero
internal copper/validation errors and zero KiCad DRC violations. Global/detail
durations are 261.619/356.473 seconds; some work overlapped regression checks and
compilation, so this is not isolated timing evidence. The copper hash remains
`459bffc8258348a89eafcc275a35decdfc59e8afb1278b0731a58e0fd9df35ac`.
Tier 1 is now running with the same 1800-second planning limit as the V15/V16
controls. No tier has been promoted from discrete replay results.

### Independent oracle at 0.25 mm track width

The earlier reference used narrower 0.20 mm tracks. A new KiCadRoutingTools run
uses 0.25 mm tracks, 0.20 mm clearance, 0.70 mm vias and 0.30 mm drills with
strict sizes, escalation off and DRC-setting repair disabled. It connects all
78 pad pairs / 25 nets with 601 tracks and 68 vias in 8.61 seconds wall time,
307,984 KiB peak RSS. Output dimensions were independently inspected. KiCad
9.0.8 reports zero errors and zero unconnected items with a byte-identical
original project copied beside the output. This is an existence oracle, not
a Flux pass or an implementation of the curved-trace objective.

For the forced channel-61 pair, the oracle uses a different layer/escape topology:
net 25 approaches from the west after a nearby layer transition, while net 26
escapes southeast and remains on F.Cu. This supports regenerating meaningful
terminal/corridor/layer choices rather than relaxing the recorded aggregate
capacity gate. No oracle tracks or coordinates were integrated into Flux.
Full evidence: `reports/diagnoses/2026-09-06-tier1-025mm-topology-oracle.md` and
`reports/benchmark_history/2026-09-06-krt-bm-svf-025mm-oracle.json`.

## Correctness audit — Missing project rules invalidated Tier-0 qualification

### Failure and evidence

The oracle's explicit original-project validation exposed a harness asymmetry:
Flux's corpus checked the temporary final board without copying its project.
KiCad therefore used defaults. The unchanged Tier-0 copper hash previously marked
green has **208 errors under its original project: 199 track-width and 9
hole-to-hole violations, zero opens**. The project requires 0.70 mm tracks and
0.25 mm drill spacing; Flux routed with 0.25 mm tracks and no imported drill floor.
The patched harness independently reproduces the rejection with KiCad 9.0.8.

All earlier Tier-0 zero-DRC claims for copper hash
`459bffc8258348a89eafcc275a35decdfc59e8afb1278b0731a58e0fd9df35ac`
are corrected: they establish default-rule legality only, not benchmark acceptance
under the source project. This is a validation defect, not a routing regression
caused by forward checking. The manifest's mandatory Tier-0 gate remains active
and now fails honestly; work returns to Tier 0 before further tier promotion.

### Research, ranked fix and verification

[KiCad's PCB documentation](https://docs.kicad.org/master/en/pcbnew/pcbnew.html)
distinguishes board constraints from netclass defaults and stores custom rules
in a sibling `.kicad_dru`. Preserve all relevant context, not just the PCB file.
First fix external validation, then import the already-supported minimum rules
before routing; audit netclass/custom-rule completeness separately.

The harness now copies and hashes the unchanged project and optional custom rules
under the candidate basename, rejects missing/mismatched context, preserves rules
beside accepted boards and keeps DRC-rejected boards as diagnostic artifacts.
Six focused Python tests pass. The exact existing Tier-0 artifact is rejected with
the expected 208 errors through the repaired gate. No rule or geometry was relaxed.
Machine-readable diagnosis: `reports/diagnoses/2026-09-06-tier0-project-rule-context.json`.
Next experiment: feed the project's minimum fabrication rules into canonical file
import, preserving nominal routing policy and using existing rule resolution/DRC.

The isolated validation fix is committed as `601228a`.

## Experiment V18 — Project minimum rules before topology selection

File-based import now loads the sibling project's eight minimum fabrication
constraints atomically, with bounded JSON input and explicit rule provenance.
It preserves nominal routing defaults, so net-specific policy is not silently
rewritten. Missing/invalid fields in a present project fail closed; standalone
boards explicitly report that no project minima were loaded. String-only import
remains independent of the filesystem. This is not full netclass/custom-rule
resolution. Regional diagnostic schema 2 also binds resolved rules so pre-audit
traces cannot silently be replayed under different dimensions.

The focused import/export/invalid-input regression passes. All 35 core tests pass
in 97.52 seconds, all five corpus boards pass source preflight, and six context
tests, four summary tests and three preflight/bridge integration checks pass.

The corrected Tier-0 default fails before search: its 0.50 mm orthogonal lattice
cannot guarantee spacing for 0.70 mm tracks plus 0.20 mm clearance. This is the
next upstream defect, not evidence that the board cannot route. A controlled
0.90 mm pitch run is in progress with unchanged rules.

An independent strict 0.70 mm KiCadRoutingTools run connects all 50 pad pairs /
20 nets with 148 straight tracks and one via. KiCad 9.0.8 reports zero errors and
zero opens under a byte-identical original project. The reference tool's own
modified output project was deliberately not used. This establishes feasibility
at the actual project constraints, not a Flux pass or a flowing-curve solution.
Structured evidence: `2026-09-06-project-minimum-import.json` and
`2026-09-06-krt-pt2399-070mm-oracle.json` in `reports/benchmark_history/`.

### V17 final disposition after the running control finished

The 1800-second historical Tier-1 run timed out. Resource propagation let residual
cone 0 retain 584 access/lane states from 48,550 screened choices, but all eight
active native scenes remained illegal (20/24, 24/29, 24/27, 22/26, 23/27, 22/26,
24/28, 24/28 routes). Twenty-one native evaluations completed before timeout;
none had fewer than 20 illegal routes, and neither later cone was reached.
The whole cone-0 portfolio cost 159.841 seconds, including 23.292 seconds of DFS.

The discrete-search hypothesis is supported, but the physical improvement
hypothesis is not. Leave resource propagation **disabled**; it moved the dead end
downstream and consumed time that previously reached cone 1. Record this rather
than promoting a combinatorial success. This run also predates project-minimum
import and cannot qualify under corrected rules. Future frontier work is Tier 0.

## Experiment V19 — Separate candidate pitch from occupied copper spacing

V18's legal-pitch control (0.90 mm) returns all 50 paths, zero missing nets and
zero quantized opens in 322.248 seconds, but retains 26 global overflow, 25
clearance-shadow conflicts and 50 exact quantized-copper violations. One of three
regional assignments completes. No final copper is accepted. The coarse samples
solve configuration validity, not physical allocation.

The [reference configuration](https://github.com/drandyhaas/KiCadRoutingTools/blob/main/docs/configuration.md)
and pinned local source use 0.10 mm samples independently of 0.70 mm routed width.
The [sampling-based motion-planning framework](https://msl.cs.uiuc.edu/~lavalle/planning/node219.html)
likewise separates graph samples from collision-free connections. The useful
concept is configuration-space sampling, not copying the reference implementation
or its output-project rewrites.

Rank a bounded dense-candidate experiment first, automatic coarsening of default
pitch second, and terminal/navmesh alternative generation third. V19 permits a
sub-envelope planar pitch only when sampled copper-clearance conflicts are
reserved and enforced, never when disabled or advisory. This changes a search
domain gate, not the required copper spacing. Retain exact physical negotiation,
quantized-copper checks and original-project final DRC. Keep the option disabled
while testing. Full diagnosis: `reports/diagnoses/2026-09-06-tier0-lattice-vs-copper.json`.

V19 passes all 36 core tests (77.33 seconds), including a 0.70 mm route on a
0.50 mm candidate grid, rejection without enforced shadows, and exact rejection
of overlapping wide routes on different sampled lines. The stable non-core,
non-corpus regression run passes 121/128 checks with the same seven known demo
expectation failures. A first run overlapped CLI relinking and is superseded by
the stable rerun; its 30 failed process launches are not code regression evidence.

Tier 0 does not improve: 50 paths and zero opens remain, but overflow rises from
26 to 40, shadow conflicts from 25 to 40, and coalesced quantized diagnoses from
9 to 12. Raw DRC counts rise from 50 to 149 with more track primitives (586 to
1083), so raw count alone is not a physical severity comparison. Runtime falls
from 322.248 to 228.967 seconds under non-isolated load. No final copper passes.
Leave dense candidates **disabled**, and preserve the experiment and tests.

## Experiment V20 — Give measured conflicts another upstream epoch

The corrected-rule Tier-0 control already finds all connections. Its remaining
defects include unlike-net crossing/overlapping tracks (seven coalesced diagnoses)
and a via/track plus drill/copper conflict (two diagnoses), chiefly involving GND
and its crossing neighbours. The mandatory Tier-0 manifest supplies only one
global iteration, leaving no later feedback epoch for these physical witnesses.

Rank a bounded iteration/feedback budget increase first, expanded joint scope
second, and new terminal/corridor alternatives third. This reuses existing
negotiated congestion and upstream physical feedback; it adds no new algorithm.
Test four total iterations with three feedback retries at the better 0.90 mm
sampling pitch, keeping physical shadows enforced and all project rules fixed.
The manifest is unchanged during the experiment. A successful result still needs
final original-project KiCad DRC and an explicit, documented budget decision.

V20 timed out at 600 seconds during epoch 4. Epochs 2 and 3 reduce the raw copper
count from 50 to 2 only by leaving eight connections open. They repeatedly rip
up net 11 for net 8 and roll the unsuccessful retry back. This is not improvement.

## Experiment V21 — Fresh scope priority at the same four-epoch budget

The existing scope-priority option finishes all four epochs in 357.412 seconds,
evaluating six regional portfolios instead of repeatedly deepening each retained
checkpoint in intermediate epochs. All physical dimensions and the binary are
identical to V20. The final selected connected checkpoint remains 50 paths,
26 overflow, 25 shadows and 50 quantized DRC errors (46 track-clearance,
2 track-via-clearance, 2 hole-clearance), with no opens. Fresh later checkpoints
still have eight opens and two errors. No final copper or KiCad DRC pass exists.
Keep this as an experimental scheduling control, not a tier promotion.

## Experiment V22 — More inner negotiation before adding heuristics

Control: sixteen total PathFinder iterations, three feedback retries, scope
priority and enforced shadows at 0.90 mm; original project rules unchanged.
This checks whether resetting local negotiation after one iteration hides useful
ordering/rip-up diversification. The frozen pre-fix binary is used throughout.

## Experiment V23 — Found paths lost by terminal membership lookup

V21's first three attempted nets fail after only 4–10 expansions, before any
other net routes. Crucially, the recorded reason is `unmatched_multisource_terminal`,
not frontier exhaustion: search returned a path and terminal attribution lost it.
`filter_pin_access` retains candidates by radius/manhattan preference; the consumer
uses binary search on that unsorted numeric node sequence.

The [C++ binary-search precondition](https://eel.is/c++draft/alg.binary.search)
requires partitioning by the comparison. Ordinary
[find](https://eel.is/c++draft/alg.find) imposes no ordering requirement and is
linear in the bounded access domain (16 by default). Rank direct membership first,
numeric re-sorting second (changes preference/tie order), an auxiliary index third
(unnecessary state). The smallest correction preserves the geometric preference
order and fixes only membership. It changes no physical acceptance criterion.

Diagnosis: `reports/diagnoses/2026-09-06-tier0-terminal-membership.json`.
Add a deterministic adaptive-ring multi-source micro-test and explicitly inspect
hard-capacity pass failures so a successful static seed cannot hide this defect.

V22 completed in 361.519 seconds and reproduced the same selected 50 violations,
zero opens, 26 overflow and eight unmatched-terminal failures in the constructive
pass. Additional nominal budget did not repair attribution. Do not promote the
larger budget. V23's short offset two-pad micro-board, with 0.70 mm traces and
0.90 mm samples, fails before the fix even though its static seed succeeds;
inspecting the constructive pass catches the hidden failure.

V23 result: all 37 core tests pass. Global allocation completes in 62.437 seconds
with all 50 paths, no opens, no overflow and no shadow conflicts. Detailed routing
takes 127.457 seconds and produces 95 tracks, 118 arcs and four vias with zero
internal copper violations and zero opens. It correctly refuses completion because
two chains retain eleven unrounded corners: net 17 on F.Cu (five) and net 4 on
B.Cu (six). Of 68 parallel spline tasks, 66 commit and none fails merge checks.
No original-project KiCad DRC has yet accepted final copper. Keep the focused
terminal fix (`263aa64`); the tier remains unqualified under corrected rules.

## Experiment V24 — Immutable source holes during diagnostic quantization

V23's two remaining oracle errors are distinct from its curve failure. Selected
paths reuse Q1 pad 2 at (20.04, 51.5) mm and U1 pad 2 at (35.54, 57) mm. The
0.10 mm diagnostic grid shifts both interior transitions by 40 micrometres,
misses exact source-hole reuse, and emits duplicate vias. Canonical pad drill
diameters are 0.8 and 1.1 mm, independently verified with KiCad's board API.

[KiCad's pad model](https://docs.kicad.org/master/en/pcbnew/pcbnew.html)
distinguishes plated multilayer holes from new vias and non-plated holes. Flux
already models the necessary net/layer/position identity; preserve exact matching
source features as immutable anchors before rounding. Reject proximity-based
inference, wrong-net ownership, unplated holes and incomplete layer coverage.

The new interior-anchor regression fails before the change and passes afterward
for existing vias and plated pads, with both endpoint-preservation settings.
Negative ownership/plating/layer cases remain rejected. This fixes diagnostic
feedback; it does not exempt generated copper from hole or clearance DRC.

V24 result: all 37 core tests pass. The real-board oracle now reports zero errors
and zero opens, without any fabricated source-pad vias. The detailed candidate
still has eleven unfinished corners, but its preserved strict export passes
KiCad 9.0.8 under the byte-identical original project: zero errors and zero opens.
This is a physical milestone, **not tier promotion**: the benchmark additionally
requires completed flowing geometry. The harness now preserves rejected detailed
copper and its explicitly non-qualifying diagnostic DRC separately from final
artifacts; a successful diagnostic check cannot change qualification.

The V23 broad regression run passed 119/128 checks: seven prior demo expectation
failures plus one constrained-placement export stopped at 645 seconds and its
dependent DRC (missing output). The export was still consuming CPU, so this is a
bounded runtime failure, not a proven deadlock. A 300-second isolated rerun is in
progress; do not silently count the interrupted check as passing.

## Experiment V25 — Isolate the two remaining curve chains

Detailed replay freezes the recorded final neighbouring copper, verifies source
revision and exact track identity, and removes only the selected chain. Net 4's
six corners become legal curved copper in this context; net 17's five do not.
Thus a bounded retry against committed surrounding curves is a plausible recovery
for the first chain. It cannot by itself explain the second failure.

Net 17 tests 24 seeds, none exact-legal. Optimization reduces continuous-envelope
violations from 377 to 140 with zero final shape or bend-radius violations and no
certificate budget exhaustion. Rank stale-context retry (net 4), incomplete guard
coverage near the board edge (net 17), conservative hull-union certification, and
insufficient seed geometry as separate hypotheses. Instrument corridor versus
actual clearance rejections before modifying physical acceptance or adding retries.

The net-17 diagnostic records seven sampled-clearance violations and 140 corridor
violations, with zero continuous-clearance violations. Reapplying the existing
exact source-polyline guard augmentation when no terminal stub changed leaves all
these metrics identical. Revert this experimental condition; additional guard
geometry did not earn a default change. The relevant research directions are
[CHOMP's collision/smoothness refinement](https://publications.ri.cmu.edu/storage/publications/pub_files/2009/5/icra09-chomp.pdf)
and [IRIS's certified convex free-space regions](https://groups.csail.mit.edu/robotics-center/public_papers/Deits14.pdf),
but neither is a reason to bypass exact clearance or rewrite the spline system.

Next isolated experiment: a bounded second realization attempt only after other
chains actually commit. Use a two-net nested-corner micro-board to distinguish
stale surrounding straight copper from genuine insufficient bend room.

## Experiment V26 — Bounded committed-context spline retry

The synthetic nested-corner board reproduces the mechanism without benchmark net
names or coordinates in routing policy. At a fixed 1 mm minimum bend radius, one
parallel transaction commits and the other retains a sharp corner. With one
bounded retry against committed copper, the remaining chain also completes with
zero opens and exact copper DRC clean. The single-pass negative control is part
of the regression, so a successful seed cannot hide the mechanism.

Implement at most two board-level realization batches. Collect only currently
unrounded chains, retain immutable scene snapshots per batch and the same exact
transaction/DRC/connectivity gates, and stop immediately if a batch commits nothing.
Expose retry task/commit counts and context-pass attribution in failure diagnostics.
This is a local context refresh, not another global routing algorithm or relaxed
spline criterion. Run the real Tier-0 control unchanged apart from this mechanism.

V26 result: the retry attempts two chains and commits one. Unfinished bends fall
from eleven to five; net 4 now completes, leaving only net 17 on F.Cu. Global
allocation takes 63.677 seconds with 50 paths, zero overflow and zero oracle errors.
Detailed realization takes 140.810 seconds (V24: 121.396), with 88 tracks, 136 arcs,
four vias, zero internal copper errors and zero opens. The exact diagnostic export
also passes original-project KiCad DRC with zero errors and zero opens. Keep the
bounded retry: its synthetic negative control and measured real-board recovery
support the hypothesis. It does not solve the remaining chain or qualify the tier.

Verification: 38/38 core tests, 7/7 rule-context tests, 4/4 summary tests. The other
non-corpus regression run passes 119/126, retaining the seven recorded demo
expectation failures. Its constrained-placement export and dependent DRC were
explicitly excluded after the separate 300-second export timeout. A frozen V20
control completes that same export in 66.462 seconds; the frozen V24 diagnostic
times out at 180.400 seconds while continuing routing work. This is an unresolved
performance regression, not a successful check or a proven deadlock. Details are
in `reports/benchmark_history/2026-09-06-constrained-placement-runtime.json`.

Do not report pcbnew's naive total route length: its arc GetLength call emits an
integer-overflow assertion on both V24 and V26. Preserve the exact PCB hashes and
leave this metric unavailable until independently calculated and checked.

Independent length check: evaluate the final telemetry's exact three-point native
arcs with a translated circumcircle/angle formula and cross-check each arc using
the two constituent chord/asin lengths (agreement within one nanometre). V24 is
415.751 mm; V26 is 410.745 mm. All sweeps are below 58.340 degrees, but one nearly
straight fitted arc has a 6520.115 mm radius. The pcbnew GetLength sum undercounts
and is discarded; this also identifies a native-fit robustness issue to inspect.

## Experiment V27 — Test one-sided source-corridor room

The remaining net-17 chain runs 50 micrometres from the offset left board edge.
Its ordinary NavMesh search succeeds (zero certificate fallbacks), but no seed
passes exact certification. The existing source-corridor augmentation shrinks all
four rectangle sides together when any one side is blocked. Hypothesis: independent
exact-certified inward expansion can recover useful bend room without moving a
boundary or changing clearance. First reproduce on a two-terminal edge-adjacent
micro-board with a fixed one-millimetre bend radius and forced coarse-search
fallback. Only then test certified augmentation on the real recorded chain.

Rank bounded axis-aligned inflation first, a full convex regional inflation solver
second, and extra trajectory iterations third. The general rationale is the
collision-free convex-region approach described by
[Deits and Tedrake](https://groups.csail.mit.edu/robotics-center/public_papers/Deits14.pdf).
This experiment would reuse Flux's exact rectangle boundary/diagonal and enclosed
hole tests; it is not an IRIS implementation or a port of third-party code.
Success requires recovered exact-legal curves, unchanged original-project DRC,
and lower-tier regression coverage. Otherwise revert the experiment.

The synthetic negative control fails with two unfinished bends and 306 continuous
corridor violations, despite zero sampled-clearance failures. Independent bounded
side inflation recovers both bends under the same radius, exact copper DRC and
connectivity checks. Test all four X/Y reflections before retention.

The real net-17 replay does **not** improve, including an isolated control that
applies the expanded source-corridor augmentation to every successful offset
NavMesh route. It retains exactly 140 corridor defects and seven sampled-clearance
failures. Revert that unconditional augmentation; this experiment supports a
fallback-corridor correctness fix but does not explain the frontier chain. Keep
the two conclusions separate and rerun the full board before retention.

V27 verification: all four reflected synthetic cases pass, as do all 39 core
tests, 7 rule-context tests and 4 summary tests. The other regression set remains
119/126 with the same seven demo expectation failures (32.31 seconds); the
separately measured constrained-placement timeout is still unresolved, not green.
The full Tier-0 control takes 59.532 seconds globally and 134.280 seconds in detail.
Its strict diagnostic export is **byte-identical to V26** (`b892957d…`): 88 tracks,
136 arcs, four vias, five unfinished bends, zero original-project KiCad errors and
zero opens. Keep the bounded fallback correction and its mirrored tests, but do
not attribute timing noise or frontier progress to it. Tier 0 remains unqualified,
and later tiers remain locked behind it. Raw and concise V27 results are preserved
under `reports/benchmark_history/2026-09-06-one-sided-corridor*.json`.

Next: determine why all real-chain spline seeds leave represented free space,
including whether whole-route source-corridor augmentation declines because one
source span has no positive-width cell. The synthetic failure was real but not
the frontier mechanism; further symmetric width tuning is not justified. Also
retain the separate open placement candidate-evaluation performance regression.

## Experiment V28 — Preserve straight-run tangents in rounded PH seeds

The unchanged V27 binary reproduces the five-corner net-17 failure. New sampled
diagnostics locate a physical, not merely certificate, defect: at a 0.25 mm corner
guard the long vertical PH span bows from x=15.900 mm to x=15.634 mm, while the
offset edge permits only 0.050 mm spare clearance. The worst point is midway along
the nominally straight run, not at a corner. Increasing the guard worsens the bow.
All seven source polyline nodes are inside the offset free space.

Hypothesis: automatic bisector tangents at trimmed corner shoulders contaminate
the intervening straight spans. Rank a bounded run-aligned Hermite seed first,
joint derivative/position optimization second, and upstream rerouting third.
PH path-planning literature explicitly constructs local corner transitions from
endpoint/tangent boundary data; see
[Farouki et al., Path planning with Pythagorean-hodograph curves](https://escholarship.org/content/qt1p74v16x/qt1p74v16x_noSplash_b96f4af98e043267182c1ebac949ed85.pdf).
Use Flux's existing quintic Hermite solver, not copied implementation code.

Minimum experiment: retain incident run directions as explicit shoulder tangent
constraints, preserving them across rebuilds and adaptive control insertion. Add
this seed family only after the existing portfolio fails exact certification.
Keep its tangent magnitudes and node positions optimizable and keep every final
physical/shape/topology gate. A mirrored U-shaped micro-route must reproduce the
old outward bow and demonstrate no outward bow with the new seed, including after
rebuild and control insertion. A successful seed is not yet a tier pass.

The mirrored micro-test passes: the existing seed bows more than 50 micrometres
outward while the aligned seed retains the exact straight-run boundary before
and after rebuild, exact subdivision and another rebuild. Malformed direction
metadata fails closed. All 40 core tests pass. The other regression set remains
119/126 (34.09 seconds), with the seven previously recorded expectation failures
and the separately unresolved constrained-placement timeout not counted as green.

The real frozen-context chain replay now completes: four of eight aligned seeds
pass exact certification, zero initial/final continuous defects, zero shape/radius
defects, 16 fitted arcs, zero opens and exact copper DRC clean. The original 24
seeds all fail. New sampled probes show the aligned source seeds no longer bow
outside, though some still fail curvature/shape checks; the selected legal seed
must pass the full certificate. Full-board and normal-budget controls are pending.

V28 full-board result: Tier 0 passes original-project KiCad DRC with all 50 paths,
82 tracks, 152 arcs, four vias, zero opens and zero unfinished bends. Detailed
runtime is 130.693 seconds versus the fresh baseline's 133.192 seconds; timing
differences under shared load are not a speedup claim. A second run using the
normal corpus budget also passes with only the required 0.90 mm pitch override.
Keep the run-aligned seed family. A frozen copy of the verified binary is used
for the newly reopened Tier-1 probe.

## Experiment V29 — Derive the CLI's unspecified pitch from source rules

The geometry fix passes both controlled and normal-budget Tier-0 runs. The
unmodified CLI sampling default is still 0.50 mm, incompatible with this board's
original 0.70 mm minimum trace width plus 0.20 mm clearance. An explicit 0.90 mm
control works, so this is a default-configuration failure, not routing congestion.
Do not encode the benchmark identity or relax spacing in the manifest.

Factor the existing global validator's required-pitch calculation into a shared
read-only helper, preserving selected-net/open-net scope, pair gaps and the
diagonal spacing factor. The CLI uses that same floor only when the user omitted
the pitch; explicit pitches and the opt-in dense-grid experiment keep their
existing semantics and validation. Retain the reported configured pitch so the
selection is inspectable. Test the helper independently, then run the Tier-0
corpus command with no configuration overrides and the existing regression set.

V29 result: the no-overrides corpus invocation passes original-project KiCad DRC,
all 50 connections, zero opens and zero unfinished bends. The selected pitch is
0.90 mm. Global/detail runtimes are 73.308/131.839 seconds; final copper is
byte-identical to both V28 successful controls (SHA-256 `2e0197607f1b…`), with
82 tracks, 152 arcs and four vias. All 41 core tests pass. The other regression
set remains 119/126 (42.19 seconds), with the same seven known demo expectations
and the separately unresolved constrained-placement timeout excluded explicitly.
Keep the rule-derived default. No manifest routing parameters or source rules
changed. Focused PH kernel commit: `7cd59fc`; detailed integration and other
pre-existing experiments remain in the working tree.

## Experiment V30 — Match physical feedback to actual copper precision

The reopened Tier-1 `bm_svf` probe times out at 600 seconds during regional repair.
At 38.619 seconds it selects 78 connection paths covering 25 nets, but retains
17 sampled overflows, three corridor overflows and 4.551 mm physical overflow.
The earlier hard-capacity iteration had zero overflow but six unrouted nets;
these are different checkpoints, not a physically complete solution.

A matched replay of snapshot 587 materializes only the 78 active selected paths,
excluding 388 retained portfolio alternatives. Original project hash is unchanged.
The default 100 micrometre probe reports 930 raw copper violations: 774 track,
65 edge, 10 pad, 51 track/via, 28 hole and two via clearances. At both 10 micrometres
and exact integer nanometres there are 853: 752 track, 71 track/via, 28 hole and
two via clearances. All versions have zero opens, 22 new vias, no malformed paths.
The 64 conflict events in the original telemetry are an emission cap, not the
total physical conflict count (261 coalesced diagnoses at the original pitch).

Failure mechanism: feedback snaps already legal static-obstacle paths onto an
unrelated coarse lattice, creating pad/edge conflicts and hiding some via/track
conflicts before expensive regional solving. This does not explain away the
853 real remaining collisions. Final PH copper already rounds to integer
nanometres, matching [KiCad's documented internal precision](https://docs.kicad.org/9.0/en/pcbnew/pcbnew.html#capabilities).
[OpenROAD's detailed-routing separation](https://openroad.readthedocs.io/en/latest/main/src/drt/README.html)
also reinforces distinguishing assignment from exact DRC/search-and-repair;
no external algorithm code is copied.

Ranked candidates: (1) remove artificial coarse rounding from default physical
feedback, retaining explicit coarse probes for controlled experiments;
(2) conflict-directed packing/layer alternatives for the remaining physical
interaction graph; (3) expanded regional joint search, which currently reaches
an 11-net scope with zero complete discrete states after 557 seconds.
First hypothesis: matching the probe to actual copper precision removes false
history and uncovers hidden collisions without changing any clearance rule.
Micro-tests must show both false-positive and false-negative coarse rounding,
including mirrored geometry; then rerun Tier 0 and a matched bounded Tier 1.
Full tier completion remains the acceptance authority, not the replay count.

V30 result: mirrored false-positive/false-negative edge controls pass, as do
42/42 core tests, 7/7 project-rule-context tests and 4/4 summary tests. Existing
via/anchor regressions explicitly retain a 100 micrometre control so the new
default cannot bypass their snapping coverage. Other regressions remain 119/126
(38.78 seconds), with the same seven known failures and the previously measured
constrained-placement export timeout still separately unresolved. The CLI reports
the physical probe's configured pitch.

Tier 0 passes again with no overrides: global/detail 62.575/136.415 seconds,
50 connections, 82 tracks, 152 arcs, four vias, zero unfinished bends and clean
original-project KiCad DRC. Final copper remains byte-identical (`2e0197607f1b…`).
Tier 1 still times out at 600 seconds, now with correctly attributed physical
witnesses. Keep native precision as a feedback-correctness fix; do not count the
77-report difference as a routing improvement or a tier resolution.

A bounded whole-net portfolio replay tests all 25 recorded seed-slot combinations
for the largest exact pair (nets 1 and 7). Both selected paths share the segment
(48.290,61.440) to (47.640,62.090) mm on F.Cu. The selected net-1 connection 9 is
the perimeter fallback; net-7 connection 0 matches shortest/perimeter. All 25
combinations preserve zero opens, but none eliminates their mutual conflicts.
Best pair count falls from 21 to three coalesced diagnoses; best whole-board raw
count falls only from 853 to 833 with two additional vias. No alternative is
accepted or exported. These portfolios do not contain a clean answer for even
this pair in the frozen surrounding allocation. Results are preserved in
`reports/benchmark_history/2026-09-07-native-probe-portfolio-replay.json`.

### Next frontier mechanism — Longitudinal channel overcount

The native-precision first ordinary window has 53 fragments across eleven nets.
It produces no complete state after only 13 DFS states (no budget exhaustion).
Exact replay with access checking disabled, enabled, and resource-forward checking
all reproduces the capacity rejection. Channel 0 has maximum recorded width
1.940 mm, yet 15 forced fragments spanning different physical regions are charged
as 6.950 mm of simultaneous lanes. Four mutually separated source spans already
form a minimal aggregate contradiction: 2.000 mm demand versus 1.940 mm, although
their minimum pairwise centreline distance is 0.615 mm, above the 0.450 mm
required spacing. These four spans are not a board unsatisfiability core.

This strengthens the previously unresolved V16 longitudinal-occupancy hypothesis,
without rediscovering it as a new tuning idea. The downstream lane solver repeats
the same all-occupants-in-one-section assumption, so disabling just the DFS
capacity gate would only move the failure. The next minimum experiment must
represent local cross-section occupancy consistently in both stages, start with
a disjoint longitudinal micro-case and retain exact topology/copper certification.
Compare guide decomposition in
[Kahng, Wang and Xu's TritonRoute paper, section 3.2](https://vlsicad.ucsd.edu/Publications/Conferences/363/c363.pdf);
its preferred-direction panels are not a drop-in model for Flux's curved corridors.
The source spans, selected channel and replay counts are recorded in
`reports/diagnoses/2026-09-07-tier1-longitudinal-overcount.json`.

## Experiment V31 — Local longitudinal channel demand

Fresh replay reproduces the first Tier-1 window's capacity rejection in 13 states,
with no search exhaustion; Tier 0 is rerun unchanged before integration. Rank
(1) shared local-demand scopes for DFS and lane packing, (2) complete geometric
channel decomposition, (3) forced-occupant alternative generation. The first is
the smallest experiment that addresses both consumers of the false inequality.

Classical channel density measures simultaneous longitudinal occupancy, not the
total number of route fragments; see the original research abstract on
[channel-routing lower bounds](https://www.sciencedirect.com/science/article/abs/pii/S0167926098000042).
The full text was unavailable. TritonRoute's
[guide preprocessing and panel assignment](https://vlsicad.ucsd.edu/Publications/Conferences/363/c363.pdf)
also separates physical resource regions before detailed solving. Flux has arbitrary
directions and varying widths, so neither Manhattan assumption is imported.

Minimum experiment: group existing cross sections by template copper-envelope
support along each wall's normal. Reuse the same scope in capacity checks and
the existing exact lane-order solver; retain every local lane target for PH.
Unknown/degenerate evidence retains conservative demand, and final topology,
native copper, connectivity and original KiCad DRC remain mandatory. Preparation
cost is proportional to channel sections times candidate polyline vertices;
cache the derived model outside candidate enumeration. Test disjoint longitudinal
runs, truly competing runs, mirrored walls, envelope-only contact and missing
evidence. Success requires useful discrete candidates and lower-tier legality,
not merely fewer pruned states or a lower raw DRC count.

V31 result: 43/43 core tests passed, plus 7/7 rule-context and 4/4 summary tests.
The other bounded CTest set remains 119/126 with the same seven known failures;
the previously measured constrained-placement export timeout and its dependent
DRC test remain excluded and unresolved. Tier 0 passes with all 50 connections,
82 tracks, 152 arcs, four vias, zero opens/unfinished bends and zero original-project
KiCad DRC reports. Copper is byte-identical to V30 (`2e0197607f1b…`). Global/detail
times are 69.696/133.117 seconds versus the fresh control's 59.489/131.568 seconds;
these shared-machine timings do not establish a speedup. Tier 1 still times out
at the matched 600-second bound. The first ordinary window remains empty in ten
states; later windows reach spline realization, but this was also true of V30.
Decision: refine, not a tier promotion or a demonstrated board-level improvement.

## Experiment V32 — Finite cross-section support

The schema-3 ablation exposes a weakness of V31's infinite transverse strips:
a section at y=83 mm can charge an x-aligned branch near y=92 mm. Replace strip
projection by segment-to-finite-wall distance using the existing geometry kernel,
expanded by half trace width plus its unchanged clearance. Retain conservative
whole-channel demand when no sampled section is supported. Reversed walls,
true competition and endpoint-envelope contact remain covered; add a lateral
separation micro-case with positive local support for both branches.

Result: 43/43 core tests pass. Replaying the identical first Tier-1 context
still produces no choices (ten states with ordinary access checking). The
finite-only variation is not sent through another full board run: its intended
first-window improvement is already absent in exact allocator replay. A newly
exposed unsupported fragment, net 13 / connection 1 / run 0 / fragment 0, touches
none of channel 0's finite walls and therefore conservatively participates in
every local constraint. Its recorded source span is x=49.653 mm, y=83.566–84.840 mm;
channel membership came from overlapping corridor cells, not a usable local
cross section. This is not justification for silently dropping its demand.
Decision: refine upstream; finite occupancy alone does not resolve the frontier.
The full six-policy records and forced demand witnesses are preserved in
`reports/diagnoses/2026-09-07-tier1-local-support-ablation.json`.

## Experiment V33 — Regenerate missing local resource evidence

Hypothesis: coarse cell overlap marks a fragment as already assigned, suppressing
the existing per-fragment physical-section fallback even when all sampled walls
are remote. Before the existing fallback, remove only unsupported governed
memberships (never the route fragments) when finite, nondegenerate geometry
demonstrates absence of local support. Existing free-space chord certification
then builds their local section; its existing narrow provisional fallback and
all exact final legality gates are unchanged. This applies only with the local
occupancy experiment enabled. No board identity, net name or coordinate is used.

Minimum test: two conflicting synthetic routes lie inside a broad corridor cell
whose only wall sample is remote. Both governed fragments must survive and acquire
local finite-section support. Benchmark the frontier and Tier 0 after the test.
The integration point follows the previously cited TritonRoute guide-preprocessing
principle: repair resource-region evidence before assigning physical tracks.

V33 result: the synthetic coverage test, all 43 core tests, and the new isolated
finite-support test pass. The broader integration checks are 120/127 in 33.42
seconds (the same seven failures; the existing constrained-placement timeout and
dependent DRC remain excluded). Tier 0 again passes original KiCad rules with
identical final copper: global/detail 60.130/133.710 seconds. Tier 1 still times
out at 600 seconds. Its first ordinary window now has 164 templates/111 alternatives
but zero choices in eight states, not an exhausted search. Regeneration removes
the remote unknown occupant from the demonstrated forced-capacity cores, but does
not resolve shared same-net demand. This supports the missing-evidence diagnosis,
not a claim of improved board routing.

At snapshot 587 the actual Tier-1 allocation is unchanged: all 78 connections,
but 853 native-grid internal copper reports (752 track, 71 track/via, 28 hole,
two via clearances), zero opens, 1,288 straight tracks and 22 vias. These are
diagnostic polylines, not completed curved copper or final KiCad acceptance.
The largest pair is still nets 1/7 on F.Cu (21 coalesced diagnoses); first-window
nets 13/14 also retain ten actual pair diagnoses. Model contradictions and real
physical collisions therefore coexist and must not be conflated.

Decision: do not promote V31–V33's integration. Restore the verified default
whole-channel policy and retain `--local-channel-occupancy` as an explicit,
reported experiment. Commit only the independently tested finite-support kernel,
public interface and isolated tests (`e9cebf0`), preserving the unrelated dirty
worktree. Full before/after records are in
`reports/benchmark_history/2026-09-07-local-support-summary.json` and linked raw runs.

Next ranked hypothesis: canonical shared-copper demand, then connected local
junction demand, then upstream topology alternatives for any genuine surviving
bottleneck. Two net-14 connection fragments contain exactly the same source span;
two adjacent net-5 fragments meet at one bend yet cause a 1.100 mm two-lane
inequality at a 0.794 mm section. Blanket per-net deduplication would be wrong
for distant branches or different candidate geometry. Start with a micro-case
containing duplicate shared trunks, an adjoining bend and two independent
parallel branches. Any capacity change must also preserve joint physical lane
targets, all connection identities and exact copper certification.

Research context: [OpenROAD's Steiner-tree module](https://openroad.readthedocs.io/en/latest/main/src/stt/README.html)
represents multi-pin net topology as trees; its rectilinear assumptions are not
imported. [Its detailed router](https://openroad.readthedocs.io/en/latest/main/src/drt/README.html)
separately reports guide coverage and DRC. These reinforce distinguishing physical
resource evidence and shared topology from final copper legality. No external
code is copied, and the next shared-demand mechanism is not yet implemented.

Final default verification: local demand is explicitly reported as disabled,
43/43 core tests pass, both CLI policy tests pass, and the broader CTest set is
122/129 in 34.49 seconds with no new failures. A final no-override Tier-0 run
passes original-project KiCad DRC with the same accepted PCB hash. Its structured
record is `reports/benchmark_history/2026-09-07-local-support-default-tier0-raw.json`.
Tier 1 remains the frontier; no higher tier is promoted or silently skipped as solved.

## Experiment V34 — Exact shared-copper lane identities

Failure reproduced on V33's source-bound first ordinary Tier-1 context: no choices
in eight states. One core charges two identical net-14 spans separately; other
cores also include adjacent bends, so exact-duplicate removal alone may be insufficient.
Rank (1) exact template-copper identities, (2) local connected junction demand,
(3) physical interval-union packing. The first has a monotone capacity bound:
adding a selected duplicate does not consume a second resource; adding an independent
branch cannot reduce previous demand. Selection-dependent connected-component
merging would not have this property and cannot simply reuse partial-DFS pruning.

[VPR's route-tree documentation](https://docs.verilogtorouting.org/en/latest/api/vpr/route_tree/)
updates occupancy from a net tree while preserving sink identities. Its
[routing-file contract](https://docs.verilogtorouting.org/en/latest/vpr/file_formats/#routing-file-format-route)
also warns against counting shared segments repeatedly. Flux needs geometric
identities instead of FPGA node IDs; no external source code is copied.

Minimum experiment: canonicalize exact same-net/layer/dimension polylines (including
reversal), keep missing geometry unique, count identities in DFS, forward checking
and GPU-shortlist CPU validation, and assign duplicate owners the same packed lane.
Keep access/via choices and all original fragment IDs. Distinct branches, different
widths, different layers and different alternative geometry remain independent.
Preparation is O(total polyline size plus ordered-map identity lookup); shared
occupancy is monotone and avoids selected-dependent graph recomputation. Test
duplicate trunks against independent branches, then replay the frozen frontier,
then run both tiers. Baseline executables are frozen before editing.

Frozen replay result: one of 164 templates aliases an identical trunk. The first
local core drops from 2.000 to 1.550 mm demand but remains wider than its 1.220 mm
section; zero choices still result in eight states (seven with resource-forward
checking). The 43 core tests and identity/CLI micro-tests pass. Broad CTest is
124/131 with the same seven known demo failures; the known constrained-export
timeout and dependent DRC remain explicitly excluded. Board runs are pending.

## Experiment V35 — Cut crossing versus clearance-only demand

V34's contact instrumentation locates a smaller mechanism: channel 20 section 1
charges both sides of one bend, but only fragment 2 crosses it. Fragment 3 ends
281.5 um away, outside its 125 um copper radius and inside its 325 um clearance
envelope. Channel 0's net-13 fragments and channel 20 section 2's net-5 fragments
show the same clearance-only contact. See the source-bound diagnosis
`reports/diagnoses/2026-09-07-tier1-shadow-vs-cut.json`.

Rank: (1) cut-crossing allocation demand, (2) same-net capsule interval-union
packing, (3) upstream topology changes for surviving actual copper conflicts.
The first is the smallest isolated experiment and remains a monotone estimate.
[Maurer's routing tutorial, pp. 4–5](https://cs.baylor.edu/~maurer/aida/tech-reports/da10_90.pdf)
describes cut density as a lower bound, separate from completing legal routing.
Its fixed-width rectilinear channels do not match Flux's free-space geometry;
only the distinction between crossing demand and geometric legality is reused.
No external code is copied.

Experiment: require finite centerline contact for local allocation scopes and
regenerate missing local samples with that same criterion. Retain width and
clearance in the packing inequality, every fragment, full clearance obstacles,
via/access checks and final copper DRC. Unknown geometry remains conservative.
Do not interpret a passing cut estimate as physical packing certification.
Test an adjacent bend with a clearance-only neighbor and a genuine two-route
crossing; replay before spending another full board run. Keep the default intact.

Result: the same first window now retains 584 lane-packed states (eight active,
512 reserve), versus zero before, with no late access/coupling/packing rejections.
However, it enumerates 91,114 choices at the 100,000-state limit. Enumeration costs
26.31 s and the full first-window selection finishes 131.27 s after context
publication. All ordinary portfolios finish at 579.79 s versus 364.18 s in V34.
The 600 s board run reaches native spline repair but cannot export final copper.
The unchanged allocation still has 78 paths, zero diagnostic opens and 853 native
internal DRC reports. No tier promotion is justified.

Tier 0 passes with both options and reproduces the accepted PCB hash. Core tests
pass 43/43, including duplicate-owner lane placement and alternative-order rollback;
the isolated kernel and six CLI policy tests pass. Broad CTest is 126/133 with
the same seven demo failures and the same explicitly excluded constrained-export
timeout. Seven project-rule and four history-summary Python tests pass. Commit
`94aa691` contains only the independently compiled/tested demand kernel, public
API and micro-tests; integration remains opt-in in the pre-existing dirty tree.
Structured frozen and regenerated replay records are in
`reports/benchmark_history/2026-09-07-cut-demand-{frozen,fresh}-replay.json`.

Decision: retain as a measured experimental allocation correction, not a default.
It removes the diagnosed false rejection but moves the bottleneck into evaluating
far more complete assignments. Physical clearance/width rules remain unchanged.

## Experiment V36 — Bound existing bundle search after cut-demand recovery

Hypothesis: exhaustive evaluation of tens of thousands of admissible allocations
starves continuous copper repair. This was previously invisible because the
same window rejected in eight states. The existing bounded DFS already has a
state limit, and its retained physical portfolio is capped at 584 states; test
a 4,096-state control before introducing another search algorithm. A frozen probe
still yields 3,686 access-screened allocations in about 1.1 s at that limit.

Change: expose the existing regional bundle-state limit as a reported CLI/benchmark
override, retaining the 100,000 default. Run the same first failing board and Tier 0
with exact identity/cut demand and the smaller bound. No routing family or DRC
gate is disabled. Success requires earlier physical repair with useful candidate
diversity and ultimately certified Tier-1 completion. Regressions in Tier 0 or
reduced geometric progress reject default promotion. If this fails, investigate
lazy/diversity-preserving candidate consumption rather than blindly lowering limits.

Result: the 4,096-state limit is present in the actual recorded Tier-1 context.
First-window enumeration falls from 26.307 to 1.065 s; complete selection from
131.274 to 5.289 s. Both runs retain 584 complete lane-packed states, eight active
and 512 reserve. Counts do not establish identical candidate identities/diversity.
All 22 ordinary portfolios finish at 373.169 s instead of 579.792 s. At the same
600 s limit, continuous routing reaches all 22 windows instead of four. The first
window completes four candidate evaluations, each taking 45.6–58.3 s; the longer
budget run had only started its first one. Nevertheless, final copper is not
exported and Tier 1 remains unresolved. This supports the starvation mechanism,
not a copper-completion claim.

The first window's native-context diagnostics contain only 1–2 individually legal
native routes per candidate, from 26–29 reported route entries; these are not
whole-board DRC results. Time is dominated by interpolation, joint shove and
trajectory polish, not native arc fitting or DRC. Next: inspect the failing
candidate geometry and boundary/corridor constraints before further parameter
tuning. In particular, audit whether source-relative lane-order preservation can
retain an invalid crossing; rotations between sections must be distinguished from
actual route-order reversals before changing that rule. This remains a hypothesis,
not a diagnosed cause on the benchmark.

Tier 0 passes unchanged at this bound (61.312 s planning, 143.217 s detail), with
the same PCB hash, 50 connections, zero opens and original-project KiCad DRC.
The final no-override V35 run also passes. V36 changes only CLI/benchmark exposure
of the existing bound; the routing core remains the 43/43-tested implementation.
The latest broad CTest result is 129/136 in 37.87 s, with exactly the same seven
known failures/exclusions. All ten isolated demand/CLI checks pass after rebuilding.

Decision: keep the explicit override and the experimental corrections available;
leave production defaults unchanged pending physical completion evidence. Complete
raw runs, source/executable identities and comparison metrics are linked from
`reports/benchmark_history/2026-09-07-channel-demand-summary.json`. No higher tier
is promoted and no DRC or fabrication rule is relaxed.

## Experiment V37 — Restore straight-section routing as the default

User direction: preserve the spline implementation but route straight sections until
all tiers pass. Baseline workspace was already dirty at `94aa691`; a file snapshot
of touched files is retained in `build/straight-change-baseline`. Prior experiments
remain intact. The ladder is Tier 0 pt2399_delay, Tier 1 bm_svf, Tier 2 zigbee_sensor,
Tier 3 ch32v003_usb, Tier 4 usbgpib; only Tier 0 was previously qualified.

Failure: V36 reaches complete abstract assignments but spends 45–58 s per first-window
continuous candidate and cannot export Tier 1 in 600 s. The earlier exact replay
contains 853 physical reports even at 1 nm quantization, including track/track,
track/via, hole and via/via collisions. Rounding is therefore not the only blocker.
Current controls are in `build/straight-baseline-tier{0,1}`; the Tier 1 control has
an explicitly bounded 180 s planning budget, not a full-budget qualification run.

Hypothesis: removing compulsory curve realization isolates straight copper packing
and returns time to congestion negotiation. Merely emitting a spline tessellation
as segments would not test this hypothesis. Gate regional realization, automatic
promotion, global spline seeds, and residual PH repair; preserve all implementations.
Disable arbitrary continuous point refinement in straight CLI output, retain the
existing 45-degree pull-tight and discrete fallbacks. Require zero output arcs in
the corpus, and retain the old rounded-shape gates for explicit curved controls.

Research: KiCad separates mitered 45/90-degree corners from rounded modes and checks
clearance while routing; see https://docs.kicad.org/9.0/en/pcbnew/pcbnew.pdf and
https://gitlab.com/kicad/code/kicad/-/blob/HEAD/pcbnew/router/pns_routing_settings.cpp .
This supports a distinct representation policy, without changing fabrication rules.
No external implementation is copied. Ranked next mechanisms: (1) avoid mandatory
continuous realization, a small representation-policy change with no new solver;
(2) feed actual straight-copper conflicts to existing negotiated rerouting if the
same physical pairs remain; (3) expand coherent conflict regions only if (2) stalls.
The first is explicitly requested and experimentally separable; the others await
straight-mode evidence. Cost is the existing polyline materialization/search cost;
loss of curved deformations may expose additional routing failures.

Tests: deterministic full-strength straight-plan micro-test plus the retained
explicit curved-plan regression; Python acceptance tests require no arcs without
relaxing opens, structural or copper checks. Before/after tier results pending.

V37 first results: straight Tier 0 passes all 50 connections, zero opens/errors,
239 straight tracks, zero arcs and five vias. Planning took 9.057 s and detail
9.359 s in `build/straight-v1-tier0`. The newly run original-binary Tier 0 control
actually timed out at 600 s (it does not reproduce the earlier recorded green
baseline within that bound); the 180 s original-binary Tier 1 control also timed
out. Concurrent runs/build activity mean these timings are not isolated performance
measurements. Tier 1 straight snapshot 613 has all 78 paths but 23 sampled
and one corridor overflow. Native-coordinate replay has 733 raw DRC reports:
632 track clearances, 71 track/via clearances and 30 hole clearances; no opens or
malformed primitives. It is diagnostic evidence, not a KiCad-qualified board.

## Experiment V38 — Return straight-copper conflicts to negotiation before regional search

Failure: straight Tier 1 spends over 300 s in the first regional portfolio after
only one global epoch. Its exact native-coordinate scaffold is already colliding.
A smallest witness is net 1 connection 6 segment 4 and net 16 connection 3 segment 5:
both use F.Cu from (50.240,78.340) to (49.590,78.990) mm. This collision is in the
selected coarse paths before regional routing, curve fitting or quantization.
The original-project external oracle proves a solution exists, not that this
particular allocation can be deformed legally. See the V37 replay JSONL for bounds
and the larger net-pair groups; 202 coalesced diagnoses are not 733 separate collisions.

Ranked hypotheses: (1) eager regional enumeration consumes the negotiation budget
on a known colliding allocation; (2) shared direct-medial capacity assumes multiple
lanes which straight fallback does not materialize; (3) sampling lacks useful lanes.
Test (1) with the existing intermediate-probe deferral: straight full-strength
routing may retain global operators, but cannot spend the full regional portfolio
while sampled/corridor overflow remains. Curved scheduling stays unchanged. Exact
copper remains the final acceptance gate. Expected effect: more global epochs in
the same runtime; success requires fewer actual copper conflicts and eventual DRC.

Research: VPR's negotiated routing accumulates congestion history between iterations
and treats channel width as a separate routability control:
https://docs.verilogtorouting.org/en/latest/vpr/command_line_usage/ .
Fixed FPGA channels differ from Flux's geometry, so this experiment only changes
when Flux returns to its existing negotiated search. No solver code is copied.
Cost remains bounded by the existing global iteration and regional state limits.
The risk is deferring a regional deformation that could legalize a congested seed;
retain that full attempt in the final epoch and retain the global portfolio beam.

Independent controls use existing sampled routing at 0.65 mm and a copper-rule
0.45 mm candidate spacing, with unchanged board rules and enforced shadows.
The first sampled invocation was rejected because medial-only hard-capacity flags
remained; the runner control now removes those flags and records its exact args.
The first 0.45 mm control was correctly rejected by the diagonal-spacing guard;
its replacement explicitly uses the existing dense-candidate mode with physical
shadows enforced. Neither invalid invocation is a routing experiment success.

V38 first refinement: deferring sampled/corridor overflow works: epoch 1 ends at
73.98 s with 351 history entries, and epoch 2 produces an overflow-free global
allocation at 112.46 s. But 74 exact copper witnesses still trigger another large
regional portfolio, because the existing readiness check ignores exact-oracle
failures once sampled overflow is zero. For straight mode, the existing oracle-first
condition must also own these intermediate epochs. Refine that condition, retain
the full final regional attempt, and rerun. The initial deferred run is stopped
once that gate mismatch is demonstrated; it is neither a pass nor a completed
600-second comparison. No result is promoted solely for zero abstract overflow.

## Experiment V39 — Match direct-chord capacity to straight realization

Failure: the V37 selected paths contain coincident copper on a shared direct/global
segment. Flux already has `single_lane_direct_voronoi_resources`, whose invariant
explicitly says conventional materialization cannot realize multiple packed lanes
on one direct chord. Straight planning was leaving that switch false while the
continuous backend was disabled. This is a representation mismatch, independent
of whether later rip-up can eventually find a better path.

Hypothesis/change: activate the existing capacity-one direct-chord model whenever
straight geometry is selected, including feedback and incremental paths. Preserve
the multi-lane mode for explicit curved routing. This does not alter widths, via
sizes or clearance. No new capacity estimator is introduced. The V38 feedback
schedule remains held constant for this comparison. Test the geometry-policy
binding on a synthetic board and repeat the frontier/lower tier. Expected effect:
shared direct-chord selections become visible as global congestion rather than
unrealizable lane packing. Regression criterion: Tier 0 or explicit curved tests
fail, or the frontier loses more feasible geometry than it gains.

## Experiment V40 — Enforce physical shadows through all straight-routing epochs

V38 and V39 both reach zero sampled overflow but retain hundreds of physical
reports. Code tracing shows the default enclosing feedback policy makes clearance
shadows advisory after the first epoch; `physical_shadow_blockers` are then absent
from hard-capacity rejection even though exact copper failures are recorded.
That policy was intended to leave room for continuous multi-lane legalization.

Run the existing `--feedback-shadow-policy enforced` control with the same V39
straight geometry, 30-iteration ceiling and 600 s diagnostic bound. Unlike the
sampled controls, this retains hard-capacity rip-up and the existing direct-medial
backend. The hypothesis is that actually reserving neighbouring copper will
prevent physical conflicts from being recreated as soon as an exact diagnosis
returns upstream. Risk: conservative shadows or ordering traps can leave opens;
those remain failures, and source clearances are never reduced. The default is
unchanged until the result is measured. This uses existing PathFinder/history and
physical reservation mechanisms; VPR research is recorded under V38.

Diagnostic detailed check: replay snapshot 1191 through the real straight materializer,
not only the per-edge oracle. It emits 527 tracks/0 arcs/47 vias and still fails
internal copper DRC (201 reports). KiCad with the original project rejects it with
193 error reports: 64 clearance, 61 crossings, 45 shorting items, 23 mask bridges,
and nine unconnected items. This validates the physical-failure mechanism and
exposes the internal/external connectivity difference on already illegal copper.
The replay tool now supports a clearly non-authoritative `--detail-preview` mode;
it retains source/export checks and never promotes a tier. V38's reduced oracle
report count is not evidence that final copper is legal.

### V37–V40 final verification and decisions

- V37: keep straight geometry as the requested default. Three Tier-0 runs reproduce
  the same PCB SHA-256 `80a1b05ab922172d5079f0fa95eb4f967a17573e03866b3ffffcd20307534e95`:
  50 connections, 239 tracks, zero arcs, five vias, approximately 498.664 mm of
  track, zero internal errors/opens and zero original-project KiCad errors/opens.
  Most segments are orthogonal; four existing terminal-access segments have other
  straight orientations. No curve or rounded-bend requirement is imposed in this mode.
  The final run takes 9.652 s planning plus 9.826 s detail; runs were not isolated.
- V38: retain geometry-dependent intermediate deferral. Four physical feedback
  epochs finish before the final regional attempt, versus spending the entire
  600 s in first-epoch regional enumeration. Exact probe counts are 733, 561, 642,
  599 with zero internal opens. This supports the scheduling mechanism but shows
  oscillation and no completed Tier 1. The final regional attempt still times out
  at 600 s. Do not interpret abstract zero overflow as copper legality.
- V39: keep the existing capacity-one direct-chord model bound to straight mode,
  because this is the representable copper invariant. It is not a tier-solving
  algorithm: probe counts 733, 560, 614, 633 show no material additional gain over
  V38, and final regional enumeration again reaches the 600 s limit. No further
  capacity heuristic is added. Tier 0 remains identical.
- V40: do not promote the forced-enforcement override. It spends the 600 s control
  budget in hard-capacity rip-up after the initial epoch without a completed
  copper candidate. The hypothesis remains inconclusive; enforcement alone does
  not overcome the current search/order trap within this bound.
- Sampled controls: do not change defaults. The 0.65 mm sampled run stagnates in
  569.878 s with 78 paths, zero unrouted nets, eight sampled and 162 shadow
  overflows. Dense 0.45 mm sampling times out at 600 s. The rejected CLI/spacing
  controls are separately identified as invalid experiments, not successes.

The complete core suite passes 43/43, preserving explicit curve/PH tests and adding
straight full-strength, residual-collision and feedback-deferral checks. Broad
CTest is 127/134 in 22.10 s with the same seven previously documented demo failures.
The known constrained-refine export fixture was independently stopped after
600 s (its dependent DRC consequently also failed) and excluded from that bounded
broad run. Eight Python project-rule/geometry gate tests and four history tests
pass. The replay tool compiles, exports only a diagnostic candidate, and external
KiCad correctly rejects that candidate. `git diff --check` is clean.

All five source fixtures pass hash/strict-import preflight. Tiers 2–4 remain locked
and were not routed: they respectively have 8, 3 and 17 canonical validation errors.
No source placement or fabrication rule was changed. Tier 1 controls use 600 s,
not its full 7200 s manifest planning allowance; no claim of exhaustive failure or
impossibility is made. The benchmark ladder remains Tier 0 qualified, Tier 1 frontier.

Next hypothesis: the two policies now expose the real tradeoff. Advisory physical
feedback repeatedly recreates track/via conflicts; enforcing every physical shadow
traps sequential hard-capacity rip-up. Construct a general micro-board with a pad
escape crossing a neighbouring via shadow, then test a bounded joint escape/via
reroute under frozen surrounding copper. Attribute the seed/portfolio choice that
requires the crossing before adding penalties or widening regional enumeration.
The post-feedback replay's net 13/net 16 via conflicts and net 13/net 7 terminal
crossing are concrete diagnostic witnesses, never implementation keys.

Machine-readable raw history, configuration/binary/source identities, peak emitted
metrics and observed memory are indexed by
`reports/benchmark_history/2026-09-07-straight-routing-summary.json`.
The exact external DRC diagnosis is in
`reports/diagnoses/2026-09-07-straight-tier1.json`.
Changes remain on top of the pre-existing dirty research tree; no broad unrelated
commit was created. The starting touched-file snapshot and a patch relative to it
are in `build/straight-change-baseline`, so this work is separately recoverable
without discarding the existing experiments.


## V41 — Screen alternative KRT inputs for Tiers 2–4 (2026-09-07)

**Failure:** The current locked Tier 2–4 inputs have 8 / 3 / 17 structural
validation blockers. The user requested alternative boards from KRT.

**Hypothesis:** Other real two-layer inputs can preserve increasing routing demand
while passing Flux's strict import and KiCad's pre-routing geometry checks.

**Research and experiment:** Screened 99 KRT catalog candidates with pcbnew 9.0.8,
Flux inspect-kicad and error-level external DRC using original project rules. An
early structural-error-only screen incorrectly shortlisted PicoFX and GB-CART256K;
the full corpus preflight rejected unsupported drills / pad approximations. The
final selection requires strict=true, lossy=false, zero import errors, zero core
errors and zero stripped-board DRC violations beyond expected opens. No router,
source placement or DRC rules were changed.

**Result:** Staged FT2232H JTAG/UART (117 connections), Nintendo Power cartridge
(148) and Nascom EPROM card (415) as proposed levels 2–4. All 3 strict preflights
pass; all 5 active corpus source preflights retain their existing results.
FT2232H and Nascom references are error/open-free. Nintendo Power's original
reference has one starved thermal, two copper-edge errors and one open; its
stripped input is clean. Exact revisions, original projects, full licenses and
source/reference/prepared hashes are preserved.

**Decision:** Keep as a separate all-locked candidate corpus. This is input
qualification, not routed-tier success. Proposed difficulty is based on physical
fanout and connection demand and still requires actual routing measurements.
The existing active tiers are retained.

**Evidence:** `reports/benchmark_history/2026-09-07-krt-tier-alternatives/README.md`
and `summary.json`; fixtures under `tests/fixtures/routing-corpus/alternatives/`.

**Next:** When adopting the candidate ladder, retain the original unsupported
boards as compatibility regressions and continue to require clean final Flux
copper and external KiCad DRC before any tier promotion.

## Experiment V42 — Separate candidate density from copper occupancy

**Failure:** Straight Tier 1's 0.65 mm direct-medial checkpoint has 78 connected
paths but hundreds of exact copper conflicts. Enforced shadows instead trap
hard-capacity repair. Tier 0 was reverified in `build/tier-resolution-tier0-baseline`.
The current 600-second frontier reproduction runs in
`build/tier-resolution-tier1-baseline`.

**Candidate explanations (ranked):** (1) candidate spacing and sparse terminal/via
sites eliminate useful legal packing choices; (2) hard-capacity repair commits a
poor ordering before negotiated history develops; (3) exact physical feedback is
not strong enough to change the chosen homotopy. The earlier 0.45 mm sample control
used broad scheduling and did not isolate ordinary negotiated routing.

**Research:** VPR documents negotiated congestion with separate search/iteration
controls (https://docs.verilogtorouting.org/en/latest/vpr/command_line_usage/).
KiCad's walkaround routing follows obstacle boundaries with H/V/45 tracks rather
than treating the obstacle clearance as a routing-site pitch
(https://github.com/KiCad/kicad-doc/blob/master/src/pcbnew/pcbnew_interactive_router.adoc).
These motivate a candidate-domain control, not a solver port or clearance change.

**Smallest experiment:** Use the existing sampled backend, enforced exact-distance
clearance shadows, legacy single-epoch negotiation, and 30 iterations at 0.65 mm
(control) and 0.25 mm (dense-candidate treatment). Hold source rules, track/via
sizes, seed and scheduling constant. Regional/detail and external KiCad acceptance
remain mandatory. Capture full runner artifacts. Success requires fewer physical
conflicts and eventually zero opens/DRC, not merely zero abstract overflow.
No router default changes until measured. If fine sampling also stalls, inspect
which terminal access or physical reservation first excludes the missing route.

## Experiment V43 — Do not defer sampled copper conflicts to absent lane assignment

**Measured mechanism:** In `populate_route_congestion`, physical shadow witnesses
between two sampled edges are discarded whenever both belong to a corridor with
spare lane capacity. `sampled_planar_edge_capacity` likewise lets multiple nets
own one sampled edge. This is explicitly justified in code by later lateral lane
assignment. Straight materialization does not move these sampled centerlines into
separate lanes. V39 corrected direct medial chords but left this ordinary sampled
case active. The V42 controls consequently omit some real collisions from history.

**Hypothesis:** Make deferred corridor lane assignment an explicit global policy;
straight planners disable it, while curved/direct global callers retain the
existing default. A sampled chord must then reserve one copper path and its true
physical shadows remain reportable even inside a wide corridor. Corridor-wide
capacity estimates remain available for guidance. This is the same representation
consistency requirement as V39, extended to the measured missing case, not a new
packing heuristic. VPR's separate routing resources and KiCad's clearance boundary
routing are the sources recorded under V42.

**Experiment:** Gate capacity inheritance and the shadow exemption together;
record the policy in results/checkpoint identity, propagate straight mode through
incremental routing, test that a physically wide corridor retains independent
capacity-one sampled tracks, and benchmark frontier plus Tier 0. Preserve explicit
curved tests. Snapshot before edits: `build/tier-resolution-v43-baseline`.

### V43 intermediate validation

Tier 0 remains complete with clean external DRC after capacity-one sampled
reservation. The first attempt at a coincident-corridor micro-board found a valid
alternate lane in both modes; it did not reproduce the exemption directly. Retain
it as a two-net straight-copper preservation regression, together with the explicit
sample-capacity check. Do not claim it proves a large-board packing solution.
The frontier's exact probe sequence changes from 733/560/614/633 to
733/520/662/674: its second epoch improves, later epochs still oscillate. No tier
promotion is justified by that partial change.

## Experiment V44 — More frequent physical feedback within the same search budget

**Failure:** Both the original and corrected straight planner spend only four
intermediate epochs on exact copper evidence before final regional enumeration.
Later inner PathFinder iterations achieve zero abstract overflow but cannot
consume new detailed-copper diagnoses until the next outer epoch.

**Hypothesis:** Allocate the same 30 global iterations across 15 physical-feedback
epochs instead of five, so newly measured copper collisions affect search sooner.
This tests feedback frequency with the existing negotiated-history mechanism,
not a larger iteration budget or new penalty. Expected benefit: the minimum
exact conflict count improves across epochs rather than returning to the same
conflict-rich states. Final copper still requires full internal and KiCad DRC.
The runtime bound is 900 seconds to allow the additional exact probes; this is
not a runtime comparison against the 600-second controls. Sources: VPR negotiated
congestion and KiCad physical obstacle handling, recorded under V42.

### V43 final decision — revert

The full core suite exposed a lower-level regression: the previously accepted two-net WorkPlan differential-pair transaction now ends with global overflow 4 and two opens. The new preservation fixture passed, but did not cover this legitimate coarse-corridor allocation. Core tests were 43/44; the broader selected suite was 126/133 with the same seven existing demo failures. Tier 0 remained accepted, while Tier 1 still timed out at 600 seconds. The blanket single-sampled-lane restriction does not earn promotion: straight materialization still has valid uses for coarse corridor sharing. Reverted all V43 production and test changes to the pre-experiment snapshot, preserving the rejected patch under `build/tier-resolution-v43-rejected/`. Do not infer that every sampled resource must be capacity one merely because final copper uses straight sections. V44 runs on the frozen V43 executable and must be interpreted as a rejected-base exploration.

## Experiment V45 — clearance-qualified rip-up cuts

### Failure and evidence
Tier 1 remains blocked at physical realization (zero topological opens does not imply legal copper). First-epoch hard-capacity recovery repeatedly probes soft routes and uses the resulting owner sets as rip-up candidates. `soft_capacity_route_blockers` currently ignores `ClearanceShadowCertificate.available_nm` entirely, whereas hard A* compares that certificate against the chosen trace/via radius plus clearance. Certificates are collected to cover the maximum possible radius; membership alone is not a collision. The helper also collects shadow owners when shadows are advisory. Thus nearby legal copper can enter the cut and inflate the coordinated repair neighborhood. This is a source-level inconsistency, not yet evidence that correcting it completes Tier 1.

### Ranked hypotheses
1. Filter probe shadow owners using actual selected trace widths, via diameter, and active shadow policy: small, directly matches A* legality.
2. Filter shared Voronoi owners by actual capacity overflow: related but deferred to isolate the clearance mechanism.
3. Increase conflict-set search breadth: defer until owner attribution is accurate.

### Research
[Sharon et al., Conflict-Based Search](https://ojs.aaai.org/index.php/AAAI/article/view/8140/7998) derives search constraints from detected pairwise conflicts. Flux is a static copper allocator, not time-expanded MAPF, so no CBS completeness guarantee transfers. The applicable principle is to build repair scope from actual conflicts rather than broad-phase proximity. Integration point: existing bounded non-committing probe and progressive rip-up; no new search family or external code. Cost stays linear in certificate observations on the probe.

### Hypothesis and experiment
Filtering legally separated or policy-inactive shadow observations will reduce false blocker pressure without dropping true trace/via conflicts. Preserve all existing sampled/corridor ownership checks for this experiment. Add diagnostic counts, test the mechanism with existing bottleneck fixtures, then run core regressions, Tier 0 and Tier 1 under the same defaults. Success requires preserved lower-tier correctness and measurable frontier benefit; no DRC rules change.

## Experiment V46 — enforce the requested ordinary straight-track angles

### Failure
An exported Tier-0 copper audit found 239 tracks: 133 horizontal, 102 vertical, and four arbitrary-angle pad-access segments (two reciprocal pairs), despite zero arcs. The existing straight-mode switch disables spline/continuous refinement but does not constrain off-grid terminal adapters. This is a geometry-policy gap relative to the requested horizontal/vertical/45-degree sections, not a KiCad DRC failure.

### Hypothesis / change
Use a bounded exact-clearance adapter for newly generated non-octilinear segments. Preserve existing copper and exact same-net branch/pad/via contacts, then try the two shortest diagonal-plus-axis paths and the two Manhattan paths. Accept only candidates clear against represented copper; keep the final structural/connectivity/DRC gate and reject any unfinished angle conversion. Preserve the spline implementation and its explicit curved mode. Add a synthetic off-grid pad-access test and an exported-copper angle assertion for straight benchmark runs. This is deterministic local geometric legalization; no source board, endpoint, width or clearance changes.

### Research
KiCad's [interactive-router documentation](https://github.com/KiCad/kicad-doc/blob/master/src/pcbnew/pcbnew_interactive_router.adoc) describes ordinary 45-degree routing. Here the four bounded two-segment constructions are elementary octilinear geometry; no third-party implementation is copied.

### V44/V45 final measurements

V44 used 14 completed intermediate oracle probes: 733, 520, 807, 738, 554, 712, 571, 687, 791, 798, 599, 832, 808, 901 violations, all with zero internal oracle opens. It timed out in regional enumeration at 900 seconds. More frequent feedback did not converge and its V43 base was reverted. Do not promote this override.

V45 excluded 117 legally separated shadow observations, passed its focused bottleneck fixture and all 44 core tests, and preserved Tier 0. Tier-1 intermediate copper counts were exactly the baseline 733/560/614/633; the run timed out at 600 seconds. Keep the narrow attribution correction, with no claim of improved tier completion.

### V46 angle validation

The final Tier-0 board has 243 tracks (135 horizontal, 104 vertical, four 45-degree), zero arcs, five vias, 50 connections, 498.861562 mm track length, and zero original-project KiCad errors/opens. Before conversion it had 239 tracks and 498.664 mm, including four arbitrary-angle adapters. Only off-angle carriers are split at existing same-net electrical contacts. The focused test covers a blocked first elbow, a legal alternate, preserved interior via contact and fail-closed rejection when every bounded adapter is blocked. Nine Python rule-context/angle-gate checks pass. Full core validation is being completed.

### Frontier runtime audit

All V42–V46 diagnostic runs used the existing `build/dev` Debug executable; Tier 1's actual manifest allowance is 7,200 seconds, while those runs explicitly used 600/900-second diagnostic caps. A fresh unchanged-parameter Tier-1 run now uses the full manifest allowance. Rebuild the existing Release preset as an independent performance control, with executable hashes recorded separately. A Debug timeout alone cannot establish what the full Release benchmark can achieve. Source inspection also shows that with spline realization disabled, regional template enumeration does not build a replacement centerline candidate; the exact final behavior must be recorded before adding a separate straight regional realizer.

### Alternative ladder adoption

Adopted the already vetted KRT alternatives as active locked Tiers 2–4: FT2232H JTAG/UART, Nintendo Power cartridge and Nascom EPROM/80bus. Tier 0/1 entries are unchanged. The former three fixtures and their original expected source errors remain in `compatibility/manifest.json`. Active preflight is 5/5 with no source errors; compatibility preflight is 3/3 under the previous expected allowances. None of the replacement tiers has been promoted by preflight.

V46 final Debug core validation: **45/45 passed**, including the retained spline/PH tests and WorkPlan regression. Broad selected CTest is **126/133**, with the same seven baseline demo failures; no new failures.

## Experiment V47 — optimized-build completeness control

Rebuilt the existing Release preset with unchanged router code and parameters. Tier 0 again passes external KiCad DRC and the H/V/45 export gate. Tier 1 reproduces the same first four oracle counts (733/560/614/633) in about 31 seconds rather than 155 seconds for the simultaneous Debug full-budget run. This is build-mode performance, not an algorithmic routing improvement. Both jobs retain the configured 7,200-second bound pending a complete result.

The earlier V42 0.25 mm sampled-candidate control was inconclusive after only three Debug iterations. Repeat that existing experiment in Release with the same 30-iteration, enforced-clearance, no-outer-feedback settings and a recorded 900-second bound. Geometry rules remain unchanged. This follow-up is justified by the measured build-speed difference, not by retuning clearance or selecting a lucky seed. It is exploratory and cannot promote a tier without final detailed/export/DRC gates.

## Experiment V48 — bound the measured regional portfolio explosion

The optimized full Tier-1 run reaches the ordinary window-0 bundle context after about 258 seconds. The recorded region contains 11 nets and a 100,000-state template-search bound. A frozen-context replay at 256 states yields 174 access-screened choices under the current whole-channel policy in about 50 ms, with no forced capacity contradiction. These are partial allocation candidates, not legal copper. The production routine evaluates every enumerated choice through full access selection, pairwise template conflicts and lane packing before reducing it to a small retained pool; it has no progress event inside that loop.

Run the existing `--max-regional-bundle-states 256` control on Tier 1 in Release (same 30 routing iterations, 900-second diagnostic bound), while retaining the unchanged full-budget baseline. Hypothesis: the first bounded portion already contains useful discrete assignments, and screening tens of thousands more delays concrete physical failure evidence. This is an isolated existing parameter control, not a promoted default, and it may lose route diversity. Frozen scene and 21 replay-policy records are retained in this run's report directory. A successful search bound must still survive all detailed and external DRC gates.

### First complete regional outcome (V47 full Release)

At 522.7 seconds the first retained Tier-1 candidate completes discrete regional analysis: 44 windows, 44 complete bundles, 116 lane cross-sections, nine geometric lane reorders, 88 avoided lane inversions and no lane-solver exhaustion. This is not a completed routing plan. In straight mode `solve_windows` skips both `realize_bundle_splines` and `build_regional_spline_candidate`, so the selected assignments do not replace the globally conflicting centerlines. The next focused hypothesis is an explicit piecewise-linear regional realization of existing lane/template/via assignments, with preserved electrical contacts and exact whole-board copper acceptance; simply setting completeness flags or copying template centerlines without the physical gate is invalid. Preserve this completed allocation as an oracle for a small synthetic lane-packing micro-board. The full run continues evaluating retained portfolios.

### V47/V48 completed frontier comparison and decision

The full Release benchmark finishes (within its 7,200-second allowance) in **1,115.174 s**, unqualified. The selected topology has 78 paths, zero unrouted nets and zero abstract overflow. All 44 regional bundles are complete, but the physical graph still has 151 conflict edges and no regional replacement candidate is published. The exact copper probe has **560 violations: 436 track clearance, 105 track-via clearance and 19 hole clearance**, with 1,204 provisional segments, 48 vias and zero internal opens. This is not final copper or external KiCad acceptance.

The existing 256-state regional control finishes in **617.164 s** with the same selected abstract/physical verdict, while reducing selected regional search states from 371,615 to 2,529 and generated candidates from 205,287 to 1,609. Keep it as an exploratory control rather than changing the default before the missing straight realization can evaluate portfolio quality. The 0.25 mm Release grid control reaches ten iterations but times out at 900 s; its last completed iteration has six sampled and 231 shadow conflicts. Do not promote the denser grid. The redundant full Debug control was explicitly cancelled after Release reproduced its global checkpoints and oracle sequence; it is not mislabeled as a timeout or a completed baseline.

Final retained verification: Debug and Release each pass **45/45 core tests**; nine Python rule-context/angle checks pass; broad Debug CTest is **126/133**, retaining the same seven documented demo failures. Final Tier 0 has identical ordinary-angle copper in both builds and passes original-project external KiCad DRC. `git diff --check` passes. Focused retained experiment patches, all 19 run records, source-ladder preflights, region replay and the completed frontier diagnosis are in `reports/benchmark_history/2026-09-07-tier-resolution/`. No tier above Tier 0 is promoted.


## Experiment V49 — Straight regional lane realization

### Failure
Tier 1 `bm_svf`: V47 selected 78 paths, zero abstract overflow, 44 complete regional bundles, but no replacement geometry. Exact global copper still had 560 violations (436 track, 105 track/via, 19 hole). Straight mode disabled the only regional geometry producer together with PH interpolation.

### Hypotheses and ranking
1. Missing assignment-to-copper handoff (direct source evidence): first construct integer straight lane targets and retain exact assembled-scene proof.
2. Inadequate endpoint/via packing: inspect the first realized scene before expanding access search.
3. Wrong global corridor choices: use surviving physical conflicts to drive upstream alternatives after geometry exists.

### Research
OpenROAD documents track assignment, initial detailed routing, search/repair, and DRC as distinct stages: https://openroad.readthedocs.io/en/latest/main/src/drt/README.html . This supports separating straight realization from the existing PH stage; no external source code was copied. A deterministic lane-target polyline is the minimum experiment, with linear construction cost plus the existing exact whole-scene oracle. It does not establish that assigned lanes can be joined legally.

### Change
Added an explicit straight realization path using packed cross-section targets, selected terminal/via access, integer diagonal-first doglegs, existing ownership/path splicing, native-scene checks, and an independent whole-path quantized copper proof. PH code stays available and straight transactions do not claim PH certificates. Initial alternative elbow or joint rerouting search is deliberately absent, so failure diagnoses can identify the next mechanism. The existing linear scaffold counters measure proofs/rejections.

### Verification plan
Two-net shared-copper fixture must become legal; static wall and cancellation must reject. Run core and Tier 0, then the V48 bounded Tier-1 control (256 bundle states, 900-second allowance), retaining original project DRC and angle gates. Full default Tier 1 is required before promotion.

### Baseline preservation
Touched pre-experiment files and frozen Release CLI are in `build/tier-resolution-v49-baseline/`. Previous uncommitted work was preserved.

### V49 intermediate evidence
Release core 46/46 and Python corpus/angle tests 9/9 passed. Tier 0 remained qualified: 50 connections, 243 ordinary-angle tracks, 5 vias, zero final internal errors/opens and original-project KiCad DRC errors. The first three bounded Tier-1 portfolios reached straight assembly but failed multilayer-plan validation (net IDs 22/0, 13/3, 21/1); the fourth reached exact copper rejection. These are diagnosed implementation stages, not new tier passes.

## Experiment V50 — Coalesce redundant straight layer adapters

### Failure
V49 first Tier-1 portfolio, window 4, net `kicad-net-22` connection 0: source F.Cu fragment ends at the original F.Cu-to-B.Cu via (66.49,57.54 mm). Its selected replacement already runs on B.Cu. Appending the fragment return adapter and then the original via creates B.Cu→F.Cu→B.Cu at one position; the ordered multilayer planner correctly rejects that sequence. The same failure recurred on two other portfolios.

### Hypothesis and change
A zero-length layer return introduced by assembly is redundant when both surrounding runs use the same layer. Coalesce these adapter excursions in straight transactions before multilayer parsing. Preserve PH assembly and the strict public multilayer validator. Require the whole-board straight oracle to report zero opens as well as legal copper, so an intermediate-layer branch cannot lose its physical via unnoticed. Count collapsed adapters explicitly.

### Minimum experiment
Synthetic F.Cu route followed by B.Cu continuation, with the repaired fragment moving its transition earlier: expect one remaining real via and legal copper. Add a front-only branch at the removed adapter: require rejection for an open. Compare the same bounded Tier-1 configuration after core/Tier-0 verification.

### Research/integration
This is a path-assembly invariant repair supported by the captured route and the existing `plan_multilayer_route` contract, within the track-assignment/detailed-routing separation documented for V49. No new routing algorithm or external code was introduced.

### V50 first portfolio: exact matched reconstruction
All 490 source fragment endpoints matched recorded snapshot 1112. The 38 emitted complete multilayer plans plus the remaining single-layer fragment replacements reproduced the production oracle exactly: 796 violations, 2 opens, 1,077 segments, 94 vias. The matching original allocation was 560 violations, 0 opens, 1,204 segments, 48 vias. Categories after straight realization: 582 track clearance, 127 track/via, 63 hole clearance, 5 hole/hole, 5 track/edge, 5 track/pad, 6 via clearance, 3 via/edge. The first wider scope retry improved to 756 violations / 1 open, still rejected. Reconstruction artifacts and provenance are in `build/tier-resolution-v50-tier1/first-scene/`; this diagnostic cannot qualify a tier.

The two opens were traced to lost same-net tree contacts, not removed vias. Net 21 branch connection 1 ends on connection 0 at (66.49,56.89 mm), F.Cu; the replacement trunk moved 0.65 mm away. Net 2 branch connection 2 ends on connection 0 at (72.34,95.89 mm), F.Cu; the replacement trunk moved 1.066 mm away and changed layer. All original vias on these nets remained. `make_fragments` merges across collinear vertices even when another connection ends there, erasing that electrical anchor before template selection.

## Experiment V51 — Compare retained source topology with exact copper

### Failure / hypothesis
The preferred discrete state nearly doubled physical vias and worsened DRC. The allocator already retains a packed source-topology fallback, but the new straight consumer initially evaluated only the preferred discrete state. Compare two assembled scenes using exact connectivity and copper, retaining the source fallback when it improves the result.

### Change / bounded experiment
Use already retained source-topology/source-via selections from active/reserve pools. Evaluate at most one additional complete scene, rank acceptance, evaluated proof, opens, DRC count, then native conflicts. No new lane assignments or relaxed rules are invented. A synthetic failed outside-board detour now falls back to the legally packed source lanes; its focused regression passes. Diagnostic scope-only replay compares V50/V51 before another full tier run.

### Next independently evidenced hypothesis
Preserve shared same-net connection endpoints during collinear regional-fragment compression. This addresses the earliest stage which loses the two measured branch contacts.

## Experiment V52 — Preserve tree junctions during fragment compression

### Failure
The V50 exact reconstruction isolated opens on nets 21 and 2. Both original tree branch endpoints were existing vertices on another connection's straight run, but `make_fragments` removed those vertices while merging collinear edges. Later detour/layer templates no longer knew the shared points were electrical anchors. No original via was removed on either net.

### Hypothesis and minimal change
Index selected connection endpoints by net/layer/position and forbid collinear merges across those vertices. Keep source-edge identities and fragment endpoint authority intact. Complexity is logarithmic endpoint lookup per candidate merge; no geometry or routing rule is weakened. This experiment covers recorded vertices; arbitrary off-vertex physical junctions remain a separate concern.

### Research
KiCad's optimizer explicitly supports topology and preserved-vertex constraints alongside collinear merging: https://docs.kicad.org/doxygen/classPNS_1_1OPTIMIZER.html . Its topology code separately manages line simplification and joint structure: https://docs.kicad.org/doxygen/pns__topology_8cpp_source.html . Flux's source graph already contains the measured junction vertices, so preserving them during compression is smaller than adding a new geometric repair stage. No KiCad source code was copied.

### Local verification
A three-terminal T fixture retains two trunk fragments around the shared endpoint. Moving both halves to offset lanes preserves the branch and exact copper connectivity. The focused regression passed. Tier 0 was rerun and remains qualified with original-project KiCad DRC and ordinary track-angle audit. Core regression and updated Tier-1 comparison follow.

### V51/V52 verification results
V51 exact-witness-only 15-window replay: 953→892 DRC reports, 1→0 opens, planned via transitions 70→60. This is a scoped comparison, not the production 45-window benchmark. Keep the bounded source-fallback comparison as a partial improvement. V52 Release core passed 48/48 after updating the provenance test's expected fragment counts for the intentionally preserved vertex; stale index/geometry/layer checks remain unchanged. Broader tests remain 126/133 with the same seven prior CLI demo failures. Python rule/angle tests passed 9/9. Typed feedback now retains exact straight-scene witnesses without inventing PH/PNS failures from absent curve certificates.

V52 first production portfolio and its wider scopes: 733/0, 725/0, 644/0 (DRC reports/opens). A second matched reconstruction reproduced the 644-report scene exactly: 505 source fragments, 37 windows, 1,032 segments, 64 vias; categories 498 track clearance, 94 track/via, 39 hole clearance, 4 hole/hole, 7 track/pad, 2 via clearance. This remains illegal copper. The first portfolio's two measured tree-contact opens are gone.

## Experiment V53 — Diagnostic exact whole-net portfolio swaps

### Failure and hypothesis
After correct branch preservation, the first portfolio still has 644 physical reports. Largest paired diagnoses include nets 16/5 on B.Cu and 21/22 on F.Cu. Retained whole-net source alternatives may be physically better than the individually preferred regional detours. Test those alternatives against the same frozen copper before changing production selection.

### Experiment
A diagnostic script invokes the existing exact portfolio replay for the largest remaining pairs, retaining only improvements in (opens, raw DRC reports, vias). Complete connection sets are required for every slot; each accepted update is independently replayed to reproduce its measured oracle. Cap at 16 pair attempts and stop when the five largest remaining groups have no improving retained pair. The script changes no production routing policy and cannot qualify a tier.

### Result
13 pair attempts, 25 whole-net combinations each where available. The diagnostic scene improved 644→419 reports, with zero opens throughout, and 64→49 vias. It then stalled: all five current largest conflict pairs rejected every available improvement. Selected geometry, per-pair cases and executable hash are retained in `build/tier-resolution-v52-tier1/portfolio-sweep/`; the durable summary is `V53-diagnostic-portfolio-sweep.json`.

### Decision
Keep this as oracle evidence, not an active routing heuristic. The available portfolio has a physical local minimum under pair swaps; merely increasing repeated pair attempts cannot solve it. Next investigate new corridor/via alternatives or a larger coordinated repair scope, using exact copper costs and preserved tree anchors. A straight detailed preview is checked separately under original project rules before drawing any external-DRC conclusion.

### V53 detailed and KiCad rejection
The 419-report provisional diagnostic produced 615 segments, zero arcs, 49 vias and 219 internal copper reports after straight detailed materialization; internal opens remained zero. With the byte-identical original project, KiCad rejected it with 209 errors: 65 clearance, 63 shorting items, 53 track crossings, 26 solder-mask bridges, 2 hole/hole, plus 9 unconnected items. The independent angle audit found 5 unresolved non-ordinary segments (189 horizontal, 210 vertical, 211 diagonal45); ordinary-angle conversion was requested but could not legalize every segment amid the conflicts. This preview is explicitly rejected. Compared with the earlier V40 diagnostic (193 external errors and the same 9 unconnected items), provisional report minimization did not translate to improved final copper. Do not promote the diagnostic pair-sweep heuristic; preserve the negative result to avoid repeating the same optimization target.

### V52 completed production comparison
The 256-state Tier-1 run completed in 1,698.735897 seconds, with 12 exact straight scenes over four retained portfolios; none qualified. The first portfolio's 733/0 → 725/0 → 644/0 sequence retained connectivity, while a second portfolio still had one open. Durable scene metrics are in `reports/benchmark_history/2026-09-07-tier-resolution/V52-completed-tier1.json`. The completed benchmark exposed an independent result-retention bug; do not treat final zero regional counters as absence of executed geometry.

## Experiment V54 — Keep evaluated regional assessments over unknown checkpoints

### Failure and hypothesis
The completed V52 run selected an earlier deferred regional result over all four physically examined portfolios. The comparator counts incomplete bundle assignments, so an unexamined empty list erroneously appears better than positive measured failures. This discards the final physical diagnosis and its feedback authority.

### Smallest experiment
Add explicit `regional_evaluated` state to the plan result and CLI report. After completion/connectivity/topology gates, prefer evaluated assessments before comparing regional failure counts. Mark the state only when regional analysis is invoked, including exact promotion. Keep existing copper acceptance unchanged. Extend the existing oracle-first feedback fixture to require a retained evaluated final assessment; it fails on the pre-change executable with the precise deferred-checkpoint assertion. No new routing algorithm or external source code is involved.

### Verification
The pre-change fixture failed at the new assertion; the fixed Release core suite passes 48/48. Broader CLI tests remain 126/133 with the same seven baseline demo failures. Tier 0 qualifies again with a byte-identical final PCB (243 H/V/45 tracks, 5 vias, zero internal opens, zero KiCad errors/unconnected). Keep this correction. The completed V52 Tier-1 board result remains attributed to its original executable; it was not rerun after this selection-only correction.

The default Debug CLI was also rebuilt through V54; its Tier-0 benchmark independently qualifies again (`build/tier-resolution-v54-dev-tier0`). Both normal development and Release executables include the correction.

## Experiment V55 — One early straight physical feedback transaction

Tier 1's 12 measured V52 straight scenes all occur in the final epoch. The straight-only intermediate predicate disables realization, and the exact coarse oracle defers regional analysis while provisional copper is illegal. Therefore none of the newly exposed assembled-copper failures can affect a later global epoch. Ranked explanations: (1) physical feedback arrives too late; (2) retained corridor/via portfolios lack the needed joint alternative; (3) fixed diagonal-first elbows introduce avoidable collisions. Test the scheduling mechanism first using existing interfaces.

The opt-in `--early-straight-feedback` experiment permits one bounded first-epoch straight transaction, one portfolio and zero physical scope retries, then consumes its diagnostics through the ordinary history/rerouting path. All global iteration counts and final legality checks remain intact. The default remains disabled pending evidence. Primary references: OpenROAD detailed routing separates track assignment and search/repair (https://openroad.readthedocs.io/en/latest/main/src/drt/README.html); VPR documents negotiated-congestion routing (https://docs.verilogtorouting.org/en/latest/vpr/command_line_usage/). This experiment reuses Flux's existing mechanisms and copies no source code.

Machine-readable hypothesis and ranking: `reports/benchmark_history/2026-09-08-tier-resolution/V55-experiment.json`.

### V55 deferred and reverted
The scheduling fixture did not preserve the assumed failing state; no scheduling improvement is claimed. During investigation, the repeated external opens exposed a prior canonical-geometry error. The ongoing 2-iteration/32-state control was intentionally stopped, with its exact command and reason retained. All V55 production/harness/test edits were restored from the pre-experiment snapshot; a focused patch is preserved for later work after geometry is corrected.

## Experiment V56 — Normalize KiCad back-side footprint coordinates

### Failure
KiCad's independent 9.0.8 loader confirms that Flux mirrored all eight U1/U2 pad anchors a second time on Tier-1 bm_svf. Each anchor is displaced by 2.54 mm, swapping the apparent physical positions of different nets. This explains why previous Flux-zero-open previews still had missing pad connections in KiCad. The earliest failing representation is canonical import, before any route allocation. Exact pad UUIDs, positions and nets are preserved in `reports/benchmark_history/2026-09-08-tier-resolution/V56-pad-position-diagnosis.json`.

### Hypothesis and research
KiCad board syntax already stores reflected back-side local coordinates, whereas canonical Flux footprints apply a side reflection at placement. Undo the serialized reflection once on import; preserve canonical transform semantics. KiCad's parser also explicitly treats an omitted pad angle as zero in board coordinates: https://docs.kicad.org/doxygen/pcb__io__kicad__sexpr__parser_8cpp_source.html . Local KiCad loader outputs independently verify the positions and orientations.

### Change and verification plan
Normalize back-side pad anchors and local courtyard/circle/keepout geometry at import; transform direct footprint Edge.Cuts without a second reflection. Initialize omitted pad angles using the same absolute-to-local conversion as explicit angles. Test a synthetic front/back fixture at four orthogonal rotations, then compare all active ladder board pad positions/dimensions/angles with KiCad using a new read-only inspector. Preserve original input/project files, rerun Tier 0 and Tier 1, and keep failed previous routing outputs as evidence of the old representation.

### V56 independent geometry result
KiCad 9.0.8 and Flux now agree exactly on 1,163 pads across all five active tier boards, plus all 16 synthetic pad-frame cases. The old 615-track/49-via rejected preview now has 9 internal opens, matching KiCad's 9 unconnected items; the pre-fix executable reported zero on the identical file. This corrects false connectivity evidence, not the tracks themselves. Tier 0 qualifies again with original-project KiCad DRC and ordinary-angle export audit. The new full bounded Tier-1 run is in progress; its first straight scene has 707 reports and zero opens. The inspector explicitly rejects nonzero pad-offset cases rather than claiming parity for unsupported anchor semantics.

## Experiment V57 — Preserve offset-pad hole and copper anchors

The corrected omitted-angle semantics exposed an existing offset-drill test assumption. Independent KiCad geometry shows a second import error: the serialized drill offset moves copper relative to the hole anchor, while the canonical field describes the hole relative to the copper center. Copying the vector directly moves the wrong object. The V56 inspector mismatches all 16 offset-pad micro-cases.

Rebase the imported copper-local position by the rotated serialized offset and negate the canonical drill offset before back-side normalization. This preserves KiCad's actual hole center and annulus center through the existing geometry interfaces. Extend the inspector to compare copper with KiCad ShapePos and drill centers with GetPosition; add plated/non-plated offset cases at four rotations on both sides. KiCad source reference: https://docs.kicad.org/doxygen/classPAD.html (ShapePos and GetPosition). Existing canonical manually constructed pad semantics remain unchanged. The active benchmark boards contain no nonzero offsets, so this correction does not change the geometry of the in-progress V56 Tier-1 run; differential pad output will verify that assertion.

### V56/V57 verification
Release core passes 50/50. Broader CLI tests remain 126/133 with the same seven baseline demo failures. Direct KiCad comparison passes all 1,163 active-tier pads and 32 synthetic frame/offset pads, including drill anchors. The active-board copper-pad geometry is byte-for-byte equivalent between V56 and V57 inspector payloads after excluding newly added drill diagnostic fields. Tier 0 qualifies again. The first V56 reconstructed scene reproduces 707 provisional reports and zero opens exactly; detailed preview has 397 internal reports, and original-project KiCad rejects it with 271 errors (139 clearance, 69 shorts, 58 crossings, 3 solder-mask bridges, 2 hole/hole), but zero unconnected items. All 1,005 tracks are ordinary angles, with zero arcs and 63 vias. This is an import/connectivity correctness improvement, not a tier promotion.

## Experiment V58 — Whole-net single-layer diagnostic alternatives

The known-connected KRT topology keeps V_IN and V_BF on F.Cu without vias, whereas the first reconstructed Flux scene uses six via sites on each. Use that as structural evidence, not track coordinates: for the five most implicated nets in the exact conflict groups, project every complete branch onto each existing copper layer, remove only duplicate same-position layer-transition vertices, and check the whole scene. This is diagnostic only and changes no production search.

Eight non-identical alternatives were checked. Moving V_IN to F.Cu worsened raw reports 707→751 despite removing six vias; moving V_BF to F.Cu improved 707→689, zero opens, 63→57 vias. Other seemingly lower report counts introduced opens and were rejected. The best candidate receives separate detailed/KiCad/angle verification before judging value. The experiment demonstrates that via count alone is not a sufficient layer-assignment objective; geometric path changes are still needed. Artifacts: `reports/benchmark_history/2026-09-08-tier-resolution/V58-single-layer-probe.json`.

### V58 external result and decision
The best single-layer diagnostic reduced detailed internal reports 397→385 and KiCad errors 271→265, with 63→57 vias, zero unconnected items, and ordinary-angle copper throughout. Categories moved rather than disappearing: clearance 139→130, crossings 58→56, hole/hole 2→1, shorts unchanged at 69, solder-mask bridges 3→9. Keep the oracle evidence; do not activate an additional routing heuristic for this small incomplete result. Future layer alternatives need joint corridor changes and physical evaluation, not simply a lower via count.

### Retention issue observed during V56
The regional scope loop in `analyze_regional_conflicts` overwrites its prior result unconditionally. The first corrected allocation's scene sequence is 707→791→808 reports with zero opens, so the broader scene can replace a physically better earlier one. All remain rejected; no false acceptance results. This is a concrete next checkpoint-retention investigation, separate from the corrected import and the unchanged in-progress executable.

The independent KiCad pad/drill comparator is now a CTest regression when the configured Python provides pcbnew; building core tests also builds its inspector. The registered external parity test passes (1/1), so future import changes are checked against KiCad itself as well as synthetic expected coordinates.

### V56 completed benchmark
The full 256-state/1,800-second-bound Tier-1 run completed in 1,591.087 seconds, unqualified. Nine assembled scenes received exact proofs, all with zero opens and nonzero copper errors (707, 791, 808, 729, 786, 798, 1245, 1472, 1439); one additional regional attempt did not assemble. The selected result is regionally evaluated but only 45/47 discrete windows complete, so its zero raw scene counters are unexamined values, not zero-error copper. The selected coarse allocation has 78 paths on 25 nets, zero unrouted nets/abstract overflow, and 613 exact provisional errors. Final copper is not accepted.

The smallest unresolved region is window 25: two source-topology fragments on F.Cu, nets GND/V_MID, one retained template each. Partial access rejects their sole joint via/run-up combination before any spline/straight realization. Its exact JSON, bounds and candidates are preserved in the new frontier report. This is the next concrete micro-case for compatible joint access/corridor generation. The scope loop's unconditional checkpoint replacement and the plan comparator's preference for incomplete-assignment counts also merit separate physical-checkpoint retention work.

The final two import tests were linked against a preserved pre-fix importer object to verify red/green behavior with the final assertions: the frame test fails on absolute pad orientation and the offset test fails on the physical hole anchor. Both pass with the corrected importer. No routing experiment remains running.

## Experiment V59 — Screen legal-angle via approaches before access selection (2026-09-08)

Failure: V56's selected bm_svf window 25 (ordinary context occurrence 7) retains
four via choices per net for GND/V_MID, but all 16 pairs fail. The apparently
remote blocker (choices 4/67) is real: the GND via is only 650 µm from the frozen
V_MID B.Cu approach anchor, below the required 675 µm. Moving GND does not help
its retained arbitrary-angle approach: choices 5/64 have 366.816 µm B.Cu track
separation against a 450 µm requirement.

Hypothesis: screen H/V/45 approach copper before mutual access selection, instead
of screening arbitrary chords that the straight realizer later bends. KiCad's
[trace posture selection](https://docs.kicad.org/doxygen/pns__line__placer_8cpp_source.html)
provides the relevant alternative straight/diagonal-head concept. The experiment
added four constant-size connector postures and retained existing static checks,
ranking and budgets. Curved mode was unchanged.

The isolated two-net replacement reduced exact static-board DRC from two track
clearance errors to zero, with the same two vias. Its 78 board opens reflect
intentionally absent unrelated routes; this was never a routed-board result.
A mirrored synthetic geometry test and a production candidate-generation test
passed. Tier 0 remained externally qualified; the initial core run passed 51/51
and broader tests passed 127/134 with the same seven baseline demo failures.

However, all 48 tested compatible local replacements made the frozen complete
source allocation worse (613 errors before, best 624 afterward, zero opens in
all cases). The complete Tier 1 experiment also reproduced unchanged first
scenes (707, 791, 808) and worsened later scenes (729→757 and 786→823).
This changes where collisions occur without coordinating neighboring routes.

Decision: revert the candidate-generation experiment. Preserve its focused patch,
micro-tests, exact pair replay and bounded frozen-scene sweep under
`reports/benchmark_history/2026-09-08-tier-resolution/` and
`build/tier-resolution-v59/`. The originally launched full benchmark continues
on its recorded executable; its final outcome will be recorded separately.
No failed candidate generator remains enabled merely because its local test passes.

## Experiment V60 — Preserve exact via failures in regional conflict extraction (2026-09-08)

A smaller synthetic case exposed an earlier feedback omission: the exact copper
oracle reports `track_via_clearance` and `via_clearance` as
`via_site_exhausted`, but regional extraction filters out that failure kind.
It consequently reports no regional physical conflicts for this via-only case.

Hypothesis: admit exact via failures through the existing bounded net/layer/region
attribution. A failure must either create repair scope or remain explicitly
unmapped. This follows the separate access/DRC repair responsibilities described
by [OpenROAD's detailed router](https://openroad.readthedocs.io/en/latest/main/src/drt/README.html).
No clearance, physical acceptance rule, or routing budget is relaxed.

The synthetic case now counts both failures: one maps into joint scope, while
one remains unmapped because the localized via-overlap box does not touch the
relevant track centerline. This is an unresolved attribution limitation, not a
successful repair. A nonexistent-layer variant must retain both unmapped failures.

On the saved 613-error Tier 1 allocation, the V59+V60 diagnostic replay expands
27 regions to 39. The control assembled an illegal 971-error scene; the expanded
scope has incomplete assignments and produces no checked scene. Its zero raw
scene counters mean unknown, not legal. Tier 0 passes with both changes. Separate
V60-only tests and benchmark runs follow the V59 reversion so the final decision
is not based on the abandoned candidate-generation experiment.

V59 completed its full 256-state run in 1754.7 seconds without qualifying Tier 1.
The targeted local assignment recovered (45→46 complete assignments of 47), but
assembled scenes worsened 729→757, 786→823 and 798→821. Remaining scenes matched
the control and the selected checkpoint still had no checked physical scene.
Reversion is retained; this is a useful local mechanism for future coordinated
routing, not a successful whole-board routing heuristic.

V60 refinement preserves original attached planar source edges on generated vias.
Track/via diagnoses use the track layer. Via/via diagnoses choose a common attached
layer deterministically, independent of primitive/path enumeration order. The
existing source-pair field identifies exact repair owners; it does not claim that
the attached edges themselves collide. Stale or unavailable-layer provenance still
fails closed. The synthetic via-only case now maps both diagnoses with zero
unmapped pairs; the final test fails against the preserved old regional object.

The V60-only 2-iteration/0-feedback/32-state Tier 1 comparison completed in
422.3 seconds before versus 161.6 seconds afterward (overlapping other work, so
not an isolated speed measurement). Both select the same 491-error, zero-open
coarse copper. Unmapped regional pairs drop 100→0; complete assignments rise
18/19→19/20. One 31-fragment, nine-net region exhausts the 32-state bound before
physical assembly. The selected scene is unchecked, not zero-error copper.
A 256-state follow-up is running and is recorded separately in the benchmark history.

Verification: 51/51 Release core tests; 127/134 broader tests, with the same seven
known baseline demo failures; Debug micro-test passes. Release and Debug Tier 0
both qualify with 243 H/V/45 tracks, zero arcs, five vias and clean KiCad DRC.
The rejected Tier 1 coarse diagnostic preview has 107 KiCad errors (64 crossings,
20 shorts, 23 clearance errors), 66 warnings and zero unconnected items. It also
retains three non-H/V/45 segments that the existing legality-gated converter could
not replace, and therefore cannot satisfy the straight-angle qualification gate.
This preview uses the unmodified source project rules and is explicitly rejected.

Decision: keep the exact via failure/ownership feedback correction. The focused
175-line patch is preserved separately because the surrounding working tree already
contains extensive uncommitted implementation, including the previously untracked
copper-oracle source; committing these whole files would conflate earlier experiments.

The V60 256-state follow-up assembled two exact-checked scenes (817→812
violations, zero opens) but timed out at 600 seconds during another regional
portfolio. This is a partial run, not a completed selected checkpoint. The
second scene was reconstructed from all 496 source fragments and reproduced
812/0 exactly before diagnostic materialization and independent KiCad DRC.
The rejected detailed preview has 745 tracks, 60 vias, zero arcs, 241 KiCad
errors (65 crossings, 78 shorts, 95 clearance, 3 hole-to-hole), zero unconnected
items and two non-H/V/45 segments. Rules are byte-identical to the source project.

The highest-priority recorded physical pair is `Net-(U4A--)` / `Net-(R4-Pad1)`
on F.Cu, with 19 coalesced diagnoses; the worst witness spans approximately
(55.052499, 82.277499)–(55.827501, 83.502501) mm. This identifies a concrete
physical region, not a proof that the frozen pair is unsatisfiable. Trace its
selected lanes/templates/corridors upstream after evaluating the corrected
ownership model with the default full-feedback configuration. Final benchmark
history records the timeout, successful lower-tier checks and all intermediate
scene proofs; no benchmark process remains running.

## Experiment V61 — Retain physically assessed regional checkpoints

### Failure
The normal Tier 0 baseline qualifies: 50 paths, 243 ordinary-angle tracks,
five vias, zero opens, clean internal copper and external KiCad DRC. All five
ladder boards import successfully. Tier 1 remains the frontier. A frozen
`bm_svf` coarse allocation materializes 78 paths with zero opens but 491 exact
errors: 420 track clearance, 47 track/via clearance, 24 hole clearance. The
largest recorded pair is nets 13/14 on F.Cu, 15 coalesced diagnoses and a
675 um maximum deficit. This is a coarse scene, not assembled regional copper.

Regional retries unconditionally replaced an earlier assessed scene. Historical
scope scenes worsened 707→791→808 errors. Unknown/unassembled scenes reported
raw zero counters and could outrank checked scenes through assignment counts.

### Hypothesis and research
Retain the best whole-scene checkpoint separately from the latest exploratory
scope. MAPF-LNS2 incumbent acceptance/rollback provides a relevant search pattern:
<https://ojs.aaai.org/index.php/AAAI/article/view/21266> and
<https://raw.githubusercontent.com/Jiaoyang-Li/MAPF-LNS2/master/src/LNS.cpp>.
Its discrete time agents differ from static copper; only acceptance semantics
are adapted, with no copied code. OpenROAD's separate assignment, detailed
routing and DRC repair phases support keeping physical assessment explicit:
<https://openroad.readthedocs.io/en/latest/main/src/drt/README.html>.

### Change
Add scene evaluation/completeness flags; invalidate assessments when rebuilding
geometry; compare fully assessed checkpoints before assignment counts; retain
geometry and diagnoses atomically while continuing exploration from the newest
scope. Report each attempt and distinguish selected retry from explored retry.
Cancellation remains authoritative even if an older candidate was retained.
Independent review identified stale flags and cancellation hiding; both were
corrected before validation.

### Before and after
All 52 Release core tests pass, including checked-versus-unknown checkpoint
ordering, physical completeness and aborted rebuild invalidation. Tier 0 still
qualifies with the same 243 tracks/five vias and clean KiCad DRC. A matched
2-iteration/0-feedback/32-state Tier 1 probe selects identical coarse copper
(491 errors, zero opens, 19/20 complete assignments). It never assembles a
regional scene; explicit flags now report that evidence as unknown. Before
433.63 s was a Dev build, after 135.12 s was Release: no speedup claim.

The normal 30-iteration baseline emitted coarse scenes 674→613→499→540 errors
(all zero opens), then an assembled straight scene with 1159 errors and zero
opens. Its process ended without harness/final export/DRC artifacts; exact exit
status is unavailable. Preserve this partial run as unknown, not a completed
benchmark or proof of legality. Last telemetry is at 1178.14 s.

### Decision
Keep this correctness change. The bounded board experiment cannot demonstrate
retry improvement because its regional assignment remains incomplete. Preserve
focused patch and experiment metadata under
`reports/benchmark_history/2026-10-01-tier-resolution/` rather than committing
whole files containing extensive prior uncommitted implementation.

## Experiment V62 — Frozen coarse whole-net portfolio probe

### Hypothesis and change
Test conflict-directed replacement of complete whole-net coarse alternatives
on the saved 78-path scene. Diagnostic script enumerates worst participating
pairs, fixes outside nets, checks exact copper at 1 nm, and rejects missing
paths, malformed geometry or increased opens. Bound: 150 probes, three rounds.
No production heuristic is activated.

### Result and interpretation
75 checks in 42.74 s accepted two pair replacements: nets 13/14, 491→478
errors; nets 22/8, 478→468. Final 1267 tracks, 32 vias, zero opens; categories
424 track, 31 track/via, 13 hole clearance. Track errors increased from 420.
Severe conflicts persist. No external KiCad, normal-angle, resource-capacity or
regional native-scene qualification was performed. The replay binary was rebuilt
during this diagnostic run; repeat with immutable binary identity for stronger
comparison. A 4.7% error reduction does not solve the mechanism.

### Decision
Retain as diagnostic evidence only. Existing portfolios provide some improvement
but insufficient diversity/legalisation; do not promote this into active routing
based solely on violation count.

## Experiment V63 — Direct fixed-obstacle conflicts to movable planar geometry

### Failure and hypothesis
The exact copper oracle classifies track/source-via and track/source-hole
collisions as `via_site_exhausted/change_via_site`. Source board geometry is
immutable, and these cases contain no generated via. Global history preparation
skips all planar edges for that action, so the feedback cannot affect the
movable trace. Reclassify only generated planar conflicts with no generated via
to `quantization_clearance/change_corridor`; preserve physical DRC rejection.
This is an isolated correctness hole, not an explanation for every Tier 1 error.

### Research
KiCad's shove implementation walks around immutable solid/hole obstacles:
<https://docs.kicad.org/doxygen/pns__shove_8cpp_source.html>.
Interactive shove and global autorouting differ, but the action must target
movable geometry. Concept adaptation only, no copied source. The smallest
experiment changes diagnosis action, using existing history rather than adding
new search or broad penalties for all dynamic-via collisions.

### Change and verification in progress
A single-layer fixed-source-via microcase checks unchanged `track_via_clearance`
rejection, corridor action and nonzero planar history. A true generated-via
control retains `change_via_site`. Review identified that static obstacle bounds
were missing, so the new planar action would penalize a whole trace. Localizing
the fixed-object witness and adding drilled-hole coverage precede board testing.
Matched V61/V63 feedback-enabled Tier 1 probes and normal Tier 0 are pending.
No tier promotion is justified.

V63 verification update: focused oracle and true-generated-via tests pass; all
52 core tests pass. The synthetic feedback grid was initially invalid at 0.1 mm
(minimum 0.45 mm), so it now explicitly checks valid input at 0.5 mm before
asserting planar history. Static obstacle bounds are applied only to reclassified
cases. Expand the localized overlap using the required clearance, since expanding
by a tiny deficit can otherwise fall back to the full trace union. A near-threshold
via test verifies that mild shortfalls remain local.

Matched Release N2/R1/32-state Tier 1 probes are unchanged: 78 materialized
paths, 1292 tracks, 46 vias, zero malformed paths/opens; 796 errors (570 track,
162 track/via, 58 hole, six via). History has 8579 edges/3983 pressure in both.
Runtime 309.67→308.62 s, not a meaningful speed change. Both regional assessments
remain unknown due to one incomplete assignment. This proves no frontier benefit.
The separate fixed-obstacle microcase proves the correction's general semantics;
identify whether those diagnoses participate on the real board before attributing
its failures to immutable obstacles. Normal full-feedback routing is pending.

Rejected source hypotheses are preserved in `rejected-hypotheses.json`:
(1) direct medial chords do not change endpoints in reconstruction; exact per-net
A* admission checks transformed drills even though polygonal free space omits
them; (2) capacious planar corridor exemption is type-guarded and cannot suppress
via-node shadows. Neither justifies a new routing algorithm. Off-grid via candidate
classification remains an audit question, not a measured current-board cause.

## Experiment V64 — Reject regional vias against frozen foreign copper early

### Failure
The scene-2 exact proof contains 19 coalesced net13/net14 F.Cu diagnoses. Net13
connection 2 has a retained vertical segment from (55.440,81.915) to
(55.440,83.540) mm. The native net14 connection 2 via is at (55.590,82.890) mm,
confirmed from the reconstructed path's coincident B.Cu/F.Cu transition. The
attached runup's endpoint (55.490002,82.790002) mm is not the via site. Center
separation is 150 um; the copper requirement is 675 um; deficit 525 um.
`window-0.json` owns no net13 fragments, so that segment is immutable within
this window and is inserted by `build_regional_frozen_copper_context`.

Access selection checks via-via, terminal-via and attached runup interactions,
but does not test candidate vias against arbitrary retained foreign segments.
The selected alternative via is therefore committed before native DRC sees the
frozen-track collision. This is a concrete early missing constraint; it does not
require rejecting movable lane centrelines.

### Hypothesis and research
Move this exact immutable obstacle check into via access selection. A legal
alternative site should remain selectable; otherwise preserve the blocking
fragment so scope expansion can make the participating routes jointly movable.
TritonRoute-WXL accounts for wide via enclosures when modelling wire pitch and
uses incremental DRC to drive repair:
<https://vlsicad.ucsd.edu/Publications/Journals/j136.pdf>.
IC tracks/cuts differ from continuous PCB copper; use Flux's circular copper,
drill and true source clearance rules. Concept only; no copied source. Existing
native DRC remains the acceptance authority.

### Change
When frozen context is enabled, access selection checks foreign nonowned
fragments sharing the via span. Required distance is the larger of copper
(diameter plus actual track width, halves, plus pairwise clearance) and drill
(drill plus track width, halves, plus hole clearance). Owned fragments remain
available to packing. Integrate into DFS and forward checking so normal site
alternatives are explored. Publish dedicated pruning/rejection counts and record
`frozen_copper_context` in bundle contexts for reproducible replay.

Independent review confirmed the geometric contract but identified evidence loss:
early rejected candidates would never reach native DRC, which previously supplied
scope-expansion witnesses. Preserve bounded, deduplicated frozen conflicts from
partial and late leaves through assignment and native diagnostics. Every selected
owned fragment incident to the via region is retained as a candidate owner;
choosing one lexical owner could point expansion at the wrong source piece.

### Verification pending
The deterministic fixture covers frozen collision/525um deficit, alternate
availability, disabled context, movable owned lane, nonoverlapping layer, exact
clearance boundary, drill-dominant rule, and two incident owners. It also checks
that scope expansion consumes early witnesses and rejects an owned blocker.
Release build, core tests, frozen context before/after replay, Tier 0 and bounded
Tier 1 are requested. The full V63 control continues independently using its
immutable executable. No tier is promoted by this implementation or by replay.

V64 verification update: 53/53 core tests and normal Tier 0 pass, including
external KiCad DRC. After moving disposable renderer artifacts out of host `/tmp`,
134/141 non-frontier CTests pass; the seven failures match existing demo failures.
The checked frozen replay explicitly restores `frozen_copper_context=true` in both
executables. Forward-checking variants reject 3–17 frozen-via states; some bounded
domains become empty and others retain alternative choices. Whole replay runtime
is 0.13→1.97 s; the initial report reversed these timings and was corrected.
No replay constitutes legal copper or whole-board infeasibility evidence.

The N2/R0/32-state board probe retains the same coarse 491 errors/zero opens
(1258 tracks, 35 vias) as V61. New early witnesses trigger scope retries: five
incomplete assignments initially, then two; selected scope retry 1. No complete
native or straight scene is assessed. It prunes 164 frozen-via states and takes
401.93 s versus the V61 Release control's 135.12 s. This is extra work without
proved frontier improvement. A matched N2/R1 comparison and normal V64 run
remain pending; do not compare the R0 run to V63's R1 metrics.

Independent reviewer identified two corrections before retention: the early
witness group can omit incident fragments when partial selection or the unrelated
16-blocker limit truncates it; native-candidate witnesses must be cleared when
promoting a different fallback while retaining separate access-search witnesses.
Regression fixtures and corrections are in progress. Immutable benchmark binaries
preserve the already-running control/comparison configurations.

### Completed normal V63 control
Normal Tier 1 finishes in 2502.44 s routing time (2513.75 s process elapsed),
peak RSS 572412 KiB. The unlocked-board harness itself exits successfully, but
`bm_svf.qualified=false`, `routing_plan_complete=false`, and `detailed=null`.
Global topology has 78 paths, zero unrouted nets and zero resource overflow.
Selected scope retry 1 has 764 straight-copper errors and zero opens; retry 0
has 822 and later retry 2 has 974. V61 correctly retains the better checkpoint.
Native assessment has 439 conflicts, 387 nonprovisional: that latter count does
not mean 387 immutable-source violations; it also includes native/native copper.
Straight categories: 542 track, 142 track/via, 62 hole, 4 via, 4 track/pad,
3 hole/hole, 5 track/edge, 2 via/edge. No detailed export or external KiCad DRC
qualification exists for Tier 1. The coarse oracle has 540 errors, demonstrating
that complete zero-overflow graph allocation still fails physical realization.
Machine-readable result: `normal-V63-summary.json`; physical diagnosis:
`reports/diagnoses/2026-10-01-tier1-V63-normal.json`.

## Experiment V65 — Restore selected checkpoint geometry in telemetry

### Failure
The completed V63 result reports a coarse oracle with 540 errors, 1291 tracks
and 46 vias. Replaying the final active global-route snapshot instead gives
647 errors, 1242 tracks and 47 vias at 1 nm. The replay is explicitly diagnostic
and omits allocation metadata; all original neckdown counters are zero. Source
inspection independently identifies the retained-checkpoint telemetry gap:
`publish_feedback_selection` publishes counters but does not restore path
geometry. Thus an older retained portfolio can have the reported best metrics
while later explored routes remain active in telemetry. Do not use that replay
as a matched physical scene or as evidence against V63 obstacle classification.

### Hypothesis and change
Republish the returned coarse checkpoint using canonical global-route IDs and
remove previously published global paths absent from it. Preserve other route
families and provisional/committed distinction. This is a representation and
diagnosis correction, with no expected routing-quality change. Cover multi-epoch
feedback by comparing the final route event view with returned coarse paths;
require removals only for absent IDs, not for same-ID geometry replacements.
Build/full-core and isolated pre-change red proof are pending.

V65 validation: all 53 core tests pass. The isolated pre-change regional object
links with unchanged headers and also passes the new crossing assertion; that
fixture does not force a discarded/retained path divergence. Record this as
positive protocol coverage, not a pre-change red proof. Do not attribute the
entire 540/647 discrepancy to telemetry without a matched restored replay.

## Experiment V66 — Memoize immutable frozen-via checks within access selection

### Failure and hypothesis
The checked V64 frozen replay costs 1.97 s versus 0.13 s without early geometric
checks. Full-budget V64 continues running but spends a long interval without new
regional telemetry. Source-level repetition is explicit: DFS and forward checks
repeatedly call the same frozen-via predicate for each candidate. CPU profiling
was unavailable (`perf_event_paranoid=4` and attach denied); no source-level hot
function is claimed from sampling. Scope expansion also adds independent work.

### Minimal experiment
Cache both clear and conflicting predicate results by derived unique via ID for
one `select_access_geometry` invocation. Fragments, window ownership, rules and
site/span are unchanged within that invocation; clear the cache on return. Preserve
the first blocker and exact copper/drill thresholds. Three alternating before/after
replays must match every discrete field over all 18 variants, with timing excluded
from equality. Full-core regression precedes any board comparison. Pending.

V66 result: all 53 core tests pass. Three alternating paired frozen replays match
all 18 non-timing records exactly. Process medians are 1.95→0.71 s (63.6% lower);
BEFORE [1.95,1.95,1.95], AFTER [0.70,0.72,0.71]. Retain invocation-local reuse
for its measured performance benefit; it does not reduce physical conflicts.

## Experiment V67 — Match early frozen constraints to the actual transaction

### Failure
V64 early access rejection assumes every fragment foreign to window A remains
fixed at its original coarse centerline. Source investigation shows that straight
assembly replaces all windows' owned fragments before its whole-scene oracle.
A fragment foreign to A may be owned and displaced by B; single-writer ownership
allows this. The actual benchmark source has zero tracks/vias, so those generated
coarse fragments are not immutable original-board copper. Per-window frozen
context construction is used by independent curved native proof, not the straight
assembler. The early gate therefore imposes a stronger boundary assumption than
the assembled straight transaction requires. Header documentation already states
that other windows may eventually move frozen copper.

### Smallest corrective experiment
Require spline realization for the early independent-window frozen-via constraint.
Straight and discrete-only probes retain full existing assembled native/quantized
DRC instead of rejecting against foreign coarse geometry. Prove the distinction
with two owned windows: via in A conflicts with B's coarse segment, but B's legal
ordinary-angle replacement moves away while retaining endpoints. Original geometry
must fail represented copper; final assembled replacement must pass. Independent
curved mode must continue rejecting its frozen boundary. Serialize both realization
modes in contexts to make future replay contracts explicit. Pending fixture/build.

### Local validation
The concrete two-window fixture passes: the original coarse via/foreign segment
scene has copper errors and zero opens; assembling both owned replacements has
zero native/quantized conflicts and zero opens, with B's dogleg present in the
manifest. Source tracks and vias remain empty. All 53 core tests pass and normal
Tier 0 qualifies with external KiCad DRC. The first isolated pre-fix invocation
accidentally selected an unrelated placement test, so it supplies no red proof;
the correct frozen-copper fixture is being rerun. Matched Tier-1 control and
independent review are pending. The original V64 normal experiment is being
stopped because its straight-mode boundary contract was disproven, not being
reported as a completed benchmark or a timeout.

### Matched control and causal proof
V67 N2/R1/32/600 completes in 299.966 seconds versus V63 308.621 seconds,
with identical 78 paths, 1292 tracks, 46 vias, 796 copper errors and zero opens.
No complete native/straight scene was assessed; the frontier remains unqualified.
Final telemetry snapshot 5956 replays exactly those materialized counts at 1 nm,
supporting V65's selected checkpoint publication on a real matched run. This does
not restore allocation certificates or explain every prior discrepancy.

The original V66-source snapshot raced the initial V67 edits and already contained
the new gate. Its isolated PASS was not a valid pre-fix result. Using the actual
pre-gate V65 source gives the intended failure at straight-mode admission; current
source passes. A recoverable V66 baseline was reconstructed from V65 plus the
saved cache hunk, with this provenance explicitly recorded.

### Independent review
Review found ambiguous schema-3 replay defaults and a fixture selected via larger
than its board-rule materialization. Contexts now use schema 4 and require both
mode flags; ambiguous older contexts fail closed. Fixture board/net rules now
match its 700/300 um selected via and assert that equality. Rebuild and normal
Tier-1 verification are pending. Original V64 normal run stopped after 46:32
because its boundary contract was disproven; no completed metrics/qualification.

### Reference oracle and next candidate
Normalized bm_svf reference passes KiCad 9.0.8 error-level DRC with zero opens
under a byte-identical source project: 389 segments, no arcs, 55 vias. Its 64
warnings are library/silkscreen/text issues, not routing legality errors. A first
regex topology inventory overcounted nested segment text and was rejected;
KiCad-native item/net inspection corrected the counts. Frozen 812-error scene
is not equivalent to reference topology, and the stale V63 final snapshot cannot
identify its returned 540-error coarse allocation.

FastRoute's primary source (`route.cpp:routeSegL/newrouteZ_edge`) explicitly scores
alternative bend patterns using congestion before choosing them:
https://raw.githubusercontent.com/The-OpenROAD-Project-Attic/FastRoute/master/third_party/fastroute/src/route.cpp
The rectilinear resource model differs from Flux's octilinear physical copper, so
only the candidate-diversity principle is being tested. Flux's straight builder
uses deterministic diagonal-first elbows and currently assesses only the whole
selected transaction and a source-topology fallback. A bounded diagnostic probes
equal-length angle-legal alternative elbows against the whole frozen copper
scene before any production heuristic is added. It will preserve endpoints,
layers, and opens, with allocation/capacity and KiCad qualification explicitly
outside this diagnostic's authority.

## Experiment V68 — Physically compare retained straight bundle alternatives

### Failure and ranked hypothesis
The completed normal V63 run has 78 paths and zero resource overflow, but its
selected assembled straight scene has 764 copper errors and zero opens. There
are 33 windows and 264 active retained bundle candidates (384,513 generated),
while straight realization physically compares only the chosen transaction and
a source-topology fallback. Curved candidate evaluation is disabled in this
straight-mode benchmark; its zero joint candidate checks are not a physics pass.
The source-level gap is in `realize_regional_straight_candidate`.

The equal-length elbow diagnostic completed 22 checks, improving the frozen
812-error scene to 798 (zero opens; 78 paths/60 vias unchanged). It preserves 74
pre-existing zero/off-angle segments and does not certify ordinary angles or
allocation. This 1.7 percent reduction does not justify an active elbow heuristic.
The KiCad-native reference comparison supplies structural layer/via differences
on the now-exactly-replayed V67 coarse allocation, but length alone does not prove
which topology is needed. These alternatives rank below testing existing unused
physical portfolio diversity.

### Research and smallest experiment
MAPF-LNS2's retained neighborhood repair and rollback principle applies to
small route-choice neighborhoods, while PCB copper has static widths/clearance
and real via interactions absent from its time-expanded agent model. FastRoute
explicitly compares alternative patterns before selecting them. No completeness
guarantee or source code is transferred. References and assumptions are recorded
in `V68-candidate-research.json`.

Add an opt-in, bounded whole-scene comparison of existing retained one-window
bundle alternatives. Every trial assembles all owned replacements and reruns
native and quantized authority. Restore only original discrete readiness for
rechecking a physically rejected candidate; never promote an incomplete bundle
or claim legality without new proof. Preserve the best atomic checkpoint; reject
unknown/materialization/open degradation; cancellation remains fail closed.
Default zero checks preserves the matched control. The microcase must contain a
chosen colliding B route and a retained dogleg that clears A when the entire
transaction is checked. Production activation awaits actual board evidence.

### Implementation and review
V68 is opt-in with `--max-straight-portfolio-checks 0..4096` (default zero).
Only the active shortlist is considered; preparation/deduplication is bounded
by remaining checks, and legal acceptance stops search. Every accepted trial
has complete native/quantized evidence and no open degradation. Budget/counters
reset per invocation and all attempted probes contribute to telemetry. A first
micro dogleg still crossed the long horizontal A track; it was corrected to go
around A's endpoint with legal H/V/45 pieces. Initial core then passes 54/54.

Independent review found lost scaffold counts in discarded trials. Counters now
aggregate probe/rejection deltas separately from retained geometry; a two-rejected
then legal fixture expects four total proofs and three rejections. A purported
CLI zero-value issue was a false positive (`std::optional` bool checks presence,
not its numeric value) and was explicitly retracted without a style-only change.
Latest counter fixture rebuild is pending. Focused patches/source hashes preserve
V68 and reviewed V67, plus separate V64 review corrections; the uncaptured
pre-correction routability-knowledge file is explicitly listed as uncovered.

### Control budget limitation
V67 N2/R1/256 times out at 600 seconds in regional alternative generation before
authoritative final metrics. No scene evaluation or physical feasibility follows
from partial near-result zeros. The normal reviewed V67 control is still running
and has already assessed its first whole scene. V68 will use the matching normal
manifest settings plus 24 physical portfolio checks, after latest core and Tier-0
checks, rather than repeatedly constraining a known incomplete regional search.

### Artifact correction
An initial P24 normal trial reused the pre-counter-fix immutable V68 executable
(SHA `ce8c3131e112624770c36efa61760016d5a579e0deaec24dad4badb6946ad92c`)
and was stopped during runner coordination. It is incomplete, not an independent
completed benchmark. The counter-fixed full comparison uses a new reviewed
immutable executable and separate artifact directory to prevent version ambiguity.


## Experiment V69 — Joint physical selection of retained straight alternatives

### Failure
The completed V63/V67 Tier-1 runs find 78 paths with zero resource overflow but
retain 764 assembled straight-copper errors and zero opens. V68 adds opt-in
one-window physical comparison, but has no completed normal Tier-1 treatment
result. Its greedy acceptance cannot reach a legal two-window combination when
neither constituent move improves the incumbent. Its depth-first window order
can also exhaust a small physical-check budget on a single discrete-ranked window.

### Hypothesis and ranked approaches
1. Compare existing conflict-linked two-window alternatives atomically. This
   directly tests missing coordinated selection without generating more topology.
2. Promote assembled copper feedback earlier in the global loop. Defer until a
   matched physical-selection result isolates whether existing diversity suffices.
3. Generate new layer/via/corridor choices only if retained diversity proves
   inadequate. More candidates alone do not repair the current selection gap.

### Research
MAPF-LNS2 (https://ojs.aaai.org/index.php/AAAI/article/view/21266) replans subsets
of colliding agents. Only conflict-directed neighborhood repair is transferred:
PCB copper is static with finite width and multilayer vias, unlike temporal MAPF.
Two-window enumeration costs a bounded number of whole-scene oracle calls and
provides no completeness guarantee. No external source code is copied.

### Change
Rank windows by observed native/straight copper conflict participation, explore
singles breadth first, and reserve half the existing shared exact-check budget
for pairs linked by observed copper conflicts. Keep rejected single alternatives
for atomic pair trials. Every trial rebuilds the full scene and retains the
existing native, quantized, connectivity, and cancellation gates. Default zero
checks stays unchanged until benchmark evidence supports activation.

### Verification plan
A deterministic two-net fixture must fail with either alternative alone and pass
only with both. Assert budget, aggregate oracle counters, no opens, and zero
native/represented conflicts. Run full core, Tier 0, and matched Tier 1 against
the immutable reviewed V68 control. Record incomplete runs as unknown.

### Before / After / Decision
Pending. Structured history: reports/benchmark_history/2026-10-08-physical-selection.json.
Existing uncommitted work is preserved; baseline source copies and library hashes
are under build/physical-selection-20261008/source-before.

### Review and additional correctness finding
Paired and single trials can name the same assembled selection after an incumbent
update. Whole-scene choice signatures now deduplicate those calls so repeated
choices do not consume the shared budget. The resumable breadth-first cursor
retains its position across the paired phase.

The one-shot straight realizer copied incumbent `straight-scene:` diagnoses into
each new trial and appended new failures without removing the old ones. Native
scene assessment already removes its own old diagnoses. Apply the same lifetime
to straight-scene evidence: clear it before realizing each transaction. A clean
accepted alternative must have no leftover native/straight failure observations.
This is a correctness fix independent of whether coupled selection improves Tier 1.
The first 54/54 core run predates this final assertion and is not its validation.

### Independent review
No material correctness, ownership, or cancellation issue was found. The review
confirmed shared budget validation, copied trial isolation, exact acceptance, and
per-scene evidence refresh. The search remains incomplete: after an improving
single move, an old incumbent is not automatically added to the pair dimensions.
Active shortlist/diagnosis caps also bound which neighborhoods are proposed.
These are search limits, never grounds for declaring a board infeasible.

### Isolated causal proof and correction
A current-ABI library with only the selector restored to its saved V68 body fails
the new fixture: "paired retained moves must find legal copper when neither single
move improves it". The current selector passes. This is a genuine quality red
proof, not just a new-counter assertion.

The supposed stale-diagnosis defect above was disproven. Removing only the extra
erase block still passes; inspection confirms `build_regional_spline_candidate`
already clears both `native-scene:` and `straight-scene:` observations before
assembly. Remove the redundant cleanup. Retain the assertion as positive protocol
coverage, and do not count this as an implementation improvement. The candidate
benchmark with redundant cleanup is being superseded by an exact final-source run.

### Final local validation
Release rebuild passes. Final core suite: 54/54. Additional CTest coverage:
12/12 channel occupancy, CLI configuration, and telemetry targets. Final
Tier 0 with unchanged default zero checks: 50 paths and external KiCad DRC PASS.
Both normal Tier-1 controls use the manifest budgets and 24 physical checks;
final copper metrics are pending. The initial baseline run ended without a
harness result and is preserved as incomplete. A candidate built with the
redundant cleanup was explicitly superseded; neither is a quality measurement.

The repository already contains extensive uncommitted router work. The focused
experiment patch is reports/benchmark_history/2026-10-08-physical-selection.patch
(relative to the saved working-tree baseline, not HEAD); source hashes are in the
companion JSON. This avoids silently committing unrelated prior experiments.

### Completed matched board result and decision
V68 P24 control: 752 assembled errors, 432 native conflicts, zero opens, 72
planned via transitions, 2601.714 s global duration. V69 P24: 751 assembled
errors, 434 native conflicts, zero opens, 68 planned via transitions, 2568.130 s.
Both examine 24 alternatives; V69 performs 12 joint checks. Both retain two
measured improvements. Coarse oracle remains exactly 540 errors, 1291 tracks,
46 vias, 78 paths, zero opens. Tier 0 qualifies; Tier 1 does not. Harness exit
zero means the unlocked probe executed as expected, not a routing qualification.

The best expanded-scope trial improves 948 to 861 errors, but still loses to the
selected 751-error checkpoint. This is a search capability improvement, not a
frontier solution. Retain it opt-in (default zero). Native/quantized tradeoffs and
the one-error selected improvement do not justify default activation. The small
wall-time difference is not a performance claim: runs overlapped on one host.

Next hypothesis: straight assembled failures are first assessed in the final
global-feedback epoch, so the unchanged 540-error coarse allocation cannot react
to them. Test an initial assembled straight assessment before further negotiation,
with ownership expansion deferred until later rather than multiplying full-scope
search on every intermediate epoch.

## Experiment V70 — Assess initial straight copper before global retries

### Failure and hypothesis
V69's measured 752→751 change leaves the exact coarse allocation unchanged at
540 errors. All straight non-final epochs are currently classified intermediate,
which disables assembled realization; oracle-pending coarse errors can also defer
regional analysis entirely. The final assembled lane/via failures therefore arrive
after the global retry budget. The existing `full_strength_first_pass` policy
certifies curved seeds but does not do so for straight copper.

### Change
For a full-strength straight initial non-final epoch, inspect one fresh global
checkpoint with straight realization enabled. Limit this first assessment to its
initial ownership scope (zero expansion retries). Feed diagnoses through the
existing knowledge/affected-net loop. Subsequent intermediate epochs remain cheap;
the final epoch retains configured portfolios and scope expansion. No acceptance
predicate, resource capacity, geometry rule, or penalty constant changes. Expose
straight realization/evaluation/violation fields on feedback candidate telemetry.

### Verification
The crossing microcase must show an assessed initial straight scene before global
retries, with the original global iteration bound preserved. In practice its
two-net window is legally resolved on that first assessment, so the regression
requires immediate assessed completion and no unnecessary feedback cone. The
initial assertion demanding a failure on this fixture was incorrect. Use the
real Tier-1 epoch evidence to verify unresolved-scene feedback instead.
Run core tests, external-DRC Tier 0, and normal Tier 1 with P24 against V69's exact
immutable control. Reject scheduling if overhead does not buy material frontier
improvement. This tests earlier evidence in the existing conflict-directed repair
architecture; it does not assume that stronger local optimization fixes topology.

### Result
Completed and scheduling reverted; see
reports/benchmark_history/2026-10-08-early-physical-feedback.json.

Local validation passes: release build, 54/54 core tests, 12/12 auxiliary CTests,
and Tier 0 with default zero portfolio checks plus external KiCad DRC. Independent
review found no material acceptance or scheduling defect. Existing diagnosis and
history caps remain bounded-search limitations; V69 did not exhaust the reported
history budget. Normal Tier 1 with P24 is running against the saved V69 control.

### Independent geometry probe while V70 runs
A public-API synthetic probe isolates another candidate mechanism without changing
production code. Two nominal routes A=(20,20)→(30,26) and B=(21,20)→(31,26), each
0.10 mm wide with 0.10 mm clearance, become overlapping terminal horizontal legs
when independently converted diagonal-first. Native checks report 2 conflicts,
quantized checks 4 track-clearance violations, and zero opens. Supplying B's
axis-first octilinear centerline (21,20)→(25,20)→(31,26) while retaining A's
orientation yields zero conflicts/violations/opens and an accepted scene.
Artifacts: build/early-physical-feedback-20261008/dogleg-probe/. This proves a
missing geometric choice in the deterministic converter; it does not establish
how many frontier-board conflicts it causes. Do not promote an orientation
heuristic without tracing the real scene and benchmarking it. A separate V69
inventory shows 434 native reports over 53 net pairs (top five: 30.6%), so a
single pair is not a sufficient explanation for the remaining board failure.

The board trace does contain overlapping converter legs (nets 16/4, B.Cu), but
it does not validate that simple remedy. Moving the observed vertical leg by
0.152297 mm is below the 0.45 mm required centreline separation, and the alternate
also intersects the neighboring diagonal. Native DRC objects lack an exact scene
mapping to those telemetry paths, so the reported native diagnosis is spatial
correlation only. Preserve this limitation in
build/early-physical-feedback-20261008/dogleg-board-witness.json; do not count it as
a fixed physical conflict or enable another heuristic on its basis.

### V70 completed comparison and rollback
The initial assessment produced 1015 violations/1 open and grew routing history
from 342 coarse entries to 736; the next epoch consumed those 736 entries. Thus
the intended upstream feedback occurred. Final selected topology nevertheless
worsened to 1025 coarse-oracle violations (1528 tracks, 57 vias) versus V69's
540 (1291 tracks, 46 vias). The selected regional scene was not evaluated at all:
its zero scene counters are unavailable evidence, not legal copper. Other final
scenes were assessed at 1346, 1617 and 1593 violations with zero opens. Tier 1
remains unqualified; no external DRC acceptance exists. Duration was 2236.628 s.

Reject the scheduling change. Restore the exact verified V69 routing algorithm
and test configuration; retain only the three additive feedback telemetry fields.
The V70 tested source, binary, proposed patch, and result remain recoverable. The
microcase's early success does not override the frontier-board regression. Final
release/core/Tier-0 checks passed after rollback (54 core, 12 auxiliary, external
KiCad DRC). Restored test-source timestamps initially reused a stale V70 object;
forcing its recompilation removed that build-artifact failure. Next work requires
better conflict constraints or route generation, not simply earlier scalar
penalties; the isolated dogleg observation is insufficient to justify a new
production heuristic.

## Experiment V71 — One physical assessment after global negotiation

### Failure and hypothesis
V70 assessed a one-iteration seed with 25 sampled overflows, one corridor overflow,
and seven unaccepted lane transitions. Its physical failure was not a stable
starting point for detailed feedback. V69's first complete negotiated checkpoint
arrives later but still defers all assembled straight geometry until the final
epoch. Test exactly one non-final assessment after all coarse topology/overflow
gates pass, preferring the fresh checkpoint then an eligible retained checkpoint.
Use zero scope expansions for this assessment and unchanged final exact gates.

### Research and fit
OpenROAD's global router separately controls whether congested routing guides may
be produced (https://openroad.readthedocs.io/en/latest/main/src/grt/README.html).
Its detailed router separates pin access, track assignment, initial routing and
DRC-guided repair (https://openroad.readthedocs.io/en/latest/main/src/drt/README.html).
CUGR evaluates global quality through detailed results and augments guide choices
(https://github.com/cuhk-eda/cu-gr). These primary references motivate separating
coarse readiness from detailed legality; they do not establish that this schedule
will improve Flux. No external source code is copied. The bounded experiment
changes only when Flux's existing physical feedback runs, with one extra initial
scope rather than full detailed search every epoch.

### Criteria
An ineligible coarse seed must remain deferred. A later negotiated checkpoint may
receive exactly one non-final physical assessment; no repeated full scans, rule
relaxation, global budget increase, or loss of final scope expansion is allowed.
Compare against the saved V69-derived verified checkpoint: 751 assembled errors,
540 coarse errors, zero opens, Tier 0 DRC green. Revert if the final selected result
regresses. Release build, 54 core tests, 12 auxiliary tests and Tier 0 external
KiCad DRC pass. The normal Tier 1 comparison is running.

## Experiment V72 — Separate search effort from physical history

### Failure and hypothesis
A source audit of V71's large pre-copper history increase found that
`diagnose_regional_routability` uses `packing_rejected_states` as global history
severity. That counter increments per rejected discrete candidate state. Budget
exhaustion even generates a capacity diagnosis when no packing state failed.
Partial access blockers similarly use repeated search observations and attribute
them to every net and the first layer of the whole window, despite retaining
choice IDs. Search enumeration effort is not a measurement of physical demand.

### Smallest experiment
Retain the existing search diagnostics and counters, but exclude these broad
search-derived summaries from physical congestion history. Preserve localized
exact/native/quantized conflict feedback. Defer geometric localization of access
blockers to a separate experiment; do not invent a clearance deficit from counts.
The regression must vary rejection/observation counts without changing physical
history, and confirm genuine physical conflict evidence still enters history.

### Validation and decision
Implementation and validation pending. V71 runs from its immutable executable,
so this source experiment cannot change that comparison. No tier is promoted
from reduced history pressure or unevaluated scene counters.

V72 local verification: release build, 54 core tests, 12 auxiliary tests, and
Tier 0 external KiCad DRC pass. An isolated binary using the pre-fix routability
object fails the new search-history regression as expected. The test includes
zero and repeated rejection counts plus retained matching physical evidence.
The first immutable copy was detected as stale before any Tier-1 launch; rebuilding
all targets and checking SHA-256 corrected it. The normal Tier-1 P24 comparison
is running from the corrected immutable binary.

The V71 epoch-2 trace supplies a concrete witness: window 0 has 97 templates,
100,000 bundle states, 11,513 via-pair access prunes, zero packing rejections,
and no complete state. Its stop reason is state_budget_exhausted. The remaining
32/33 assignments complete, but this one prevents assembling the whole copper
scene. V72 directly targets the false pressure inferred from this search result;
it does not claim to solve the bounded assignment itself. Witness JSON is saved
in build/search-evidence-feedback-20261008/v71-search-failure-witness.json.

### V71 completed frontier comparison
Normal Tier-1 P24 completed in 2508.527 s: 710 assembled violations, zero opens,
434 native conflicts, versus V69's 751/0/434. The selected coarse oracle has
527 violations, 1318 tracks and 45 vias (V69: 540, 1291, 46). Selected physical
scope progresses 794 → 776 → 710. Tier 1 remains unqualified. Both runs retain
24 portfolio checks, including 12 joint checks; V71 accepts seven improvements.
Do not interpret overlapping runtimes as a speedup.

The early consultation did not assess copper: 32/33 bundle assignments complete,
one budget-exhausted, straight_scene_evaluated=false. Exactly two broad regional
diagnoses increased history entries 572→574 and pressure 5212→15803. Therefore
the measured improvement supports earlier regional-feasibility feedback, but
not the intended claim of early assembled-copper feedback. V72 tests removal of
the unsupported search-derived pressure before a final retain decision.

V72 research cross-check: OR-Tools distinguishes bounded-search UNKNOWN from
proven INFEASIBLE (https://developers.google.com/optimization/cp/cp_solver).
This supports separating search exhaustion from physical impossibility; it does
not predict whether removing a heuristic penalty improves Flux's route choices.
No solver dependency or external source code was introduced.

## Experiment V73 — Corrected history without early scheduling

Separate the V72 semantic correction from V71's optional early consultation.
Build an isolated immutable control from the verified V69 planner plus the V72
history fix. This resolves whether early scheduling earns its cost once search
effort no longer masquerades as physical pressure. No geometry rules, search
budgets or acceptance gates change. Compare full Tier-1 P24 and preserve Tier 0;
source promotion waits for evidence. The working source remains V72 during the
three-way comparison.

V73 candidate has been copied into the working tree after source-hash checks.
The normal release build produces exactly the control binary hash
`af0a4ed8357e26e0b44899df72fa6945c0d84ca0eac2fd679c78c97215ac2a68`.
The isolated matching 54 core tests, 12 auxiliary checks, and Tier 0 external DRC
pass. Final Tier-1 result remains pending. V71/V72 binaries and sources remain
preserved; this source preparation is not tier promotion.

A read-only width-provenance audit rejected another tempting change: skipping
lane displacement for non-terminal explicit widths is shared by the spline and
straight realizers and protects source-bound resource width authority. The V69
frontier result has zero fixed-resource-width fragments, so that branch cannot
explain this measured failure. Do not remove it based on this board.

### V72 completed comparison and V71 decision
V72 completes the normal P24 run in 2768.837 s with 751 assembled violations,
zero opens, 434 native conflicts, and the baseline coarse oracle (540 violations,
1291 tracks, 46 vias). Scope trials are 793→751→861. Peak RSS is 599136 KiB.
Tier 1 remains unqualified. The early regional call now leaves history unchanged
at 572 entries / 5212 pressure, confirming the intended semantic correction.

The V71 reduction to 710 disappears when search-derived pressure is removed.
That is evidence for an accidental diversification effect, not validation of
search effort as physical capacity. Retain V72's semantic boundary and reject the
extra early scheduling: it produces no improvement against V69 under the corrected
contract. An explicit future diversification mechanism should use distinct route
choices or localized verified conflicts, not search-budget-dependent cost inflation.
V73 verifies the smaller final combination without that extra early consultation.
The runtime overlap prevents a controlled speedup claim.

### Remaining mechanism and bounded next experiment
The early clean checkpoint's one unresolved region exhausts bundle enumeration
before finding a complete access assignment. The enumerator currently orders
fragment domains by their initial sizes, then fragment index; no duplicate DFS
branch was established. Capture that region and compare a conflict-aware variable
order against the same state budget before changing global penalties again.
A second independent problem remains after discrete assignment: assembled tracks
and vias still collide. The synthetic dogleg example demonstrates missing
geometric diversity, but the traced real pair does not admit that simple flip.
Neither finding justifies a broad rewrite or a claim that Tier 1 is solved.

The harness's final console summary now separates checks passing from actual
qualification. Replaying its updated summary on the completed mixed run prints
`2/2 selected corpus checks passed; 1/2 assessed boards qualified`. A real
Tier-0 preflight prints `routing qualification not assessed`. Python compilation
and both reporting checks pass. This changes no routing or JSON acceptance logic;
the already-running V73 process loaded the old wording at startup.

### V73 final result and retained implementation
V73 completes in 2520.765 s. Tier 0 qualifies with external KiCad DRC; Tier 1
remains unqualified at 751 assembled violations, zero opens and 434 native
conflicts. All 78 connection paths exist and global overflow is zero. The coarse
oracle remains 540 violations / 1291 tracks / 46 vias. Scope trials remain
793→751→861; 24 exact portfolio checks include 12 joint checks and two accepted
improvements. This matches the verified V69 geometry outcome. Runtime comparisons
are affected by overlapping runs and are not promoted as performance gains.

Retain coupled route-pair portfolio selection (budget remains opt-in, default 0),
passive scene-evaluation telemetry, and the search-evidence history boundary.
Remove early scheduling V70/V71; preserve their source and benchmark evidence.
Keep the corrected benchmark summary. Final validation: release build matching
immutable V73, 54 core tests, 12 auxiliary checks, Tier-0 external DRC, Python
compilation, preflight reporting and mixed-result reporting all pass. No higher
tier is promoted and no claim of complete board routing is made.

Task-only changes are preserved in
reports/benchmark_history/2026-10-08-routing-fixes-combined.patch; the final
machine-readable result is 2026-10-08-corrected-history-control.json in that
directory. Substantial pre-existing uncommitted work is preserved; no unrelated
changes were staged or committed.


## Tier-1 strategy review — 2026-10-08 (V73 evidence; proposals only)

### Failure and evidence
The selected bm_svf run has 78 connection paths, zero global overflow and zero
opens, but 751 assembled straight-scene violations. Track clearance (536) and
track-via clearance (138) comprise 674/751 reports. The 434 native diagnoses
span 53 net pairs and one 20-net connected graph. These representations overlap;
counts must not be added. The coarse oracle's 540 violations and the assembled
751 concern different geometry, so their difference does not attribute causality.
Scope expansion tries 793, 751, and 861 errors and retains the best. Twenty-four
straight portfolio checks (12 joint) produce only two improvements. Runtime is
2520.765 seconds. Tier 0 remains externally qualified; Tier 1 does not.

### Earliest demonstrated pipeline problem
V73 defers assembled regional assessment until the terminal feedback epoch.
Coarse quantized-copper feedback already operates in earlier epochs. The final
selected regional scene produces localized history and affected-net information,
but retry_available is false: no subsequent global allocation can use it.
The configured global budget was allocated as 1+8+7+7+7 iterations across the
epochs; early global convergence means the actual reported cumulative count is
11. The exhausted resource is the outer feedback-epoch schedule, not necessarily
all 30 permitted inner iterations. Earlier V72 consultation
without an assembled assessment is not a test of moving exact regional feedback
upstream. Its lack of improvement must not be used to validate or reject that
separate experiment. Carry only the physically selected checkpoint's evidence;
rejected topology diagnoses must remain isolated.

### Geometric contract and search limitations
Adjacent-channel lane precedence and section packing already exist. Their proof
covers section width/order, not the swept copper of independently generated
inter-section doglegs, bends, and via runups. Those are checked downstream.
A synthetic staggered two-route example is solved by another octilinear elbow;
the previously traced real-board pair is not, so elbow flipping alone is not a
board-level remedy. Alternate topology templates use their own centerlines,
which require physical validation rather than assuming the selected topology's
lane offsets certify them. Existing exact oracles, portfolio retention, scope
expansion and best-scene comparison should be extended, not duplicated.
Single/pair monotonic moves cannot generally resolve larger coordinated swaps.
The connected 20-net conflict graph should seed bounded spatial neighbourhoods,
not a board-wide exponential solver. Final unresolved-window labels include
physical rejection; do not misclassify them as proved discrete infeasibility.
Spline-named counters are not evidence that the straight run lacks exact checks.

### Ranked experiments
1. Export replayable physical witnesses with exact copper ownership, geometry,
   scene/epoch identity, source choice, rules and frozen boundary. Reproduce a
   small observed conflict before changing its geometry. Deduplicate physical
   events and retain category-specific counts.
2. Reserve an existing global epoch after a real penultimate assembled regional
   assessment. Keep the total global iteration budget fixed and run the final
   assembled proof again. A focused test must show the exact regional conflict
   reaching a later global call; merely consulting the regional allocator is
   insufficient. This tests the confirmed scheduling defect, not a guarantee
   that the current candidate set can resolve every resulting conflict.
3. Extend lane-chain candidates with bend/transition/via-runup alternatives and
   lazy exact quantized compatibility checks. Keep existing precedence. Test
   staggered parallel lanes, via blocking and frozen-neighbour interactions,
   including rotated/mirrored and width/clearance variants.
4. Select compatible physically realized candidates jointly in small conflict
   neighbourhoods. Use lazy forbidden combinations, promote useful reserve
   alternatives, and generate new homotopy/layer/via choices when the pool is
   inadequate. Permit coordinated three-plus changes without requiring every
   intermediate move to improve. Commit only a fully assessed scene, expanding
   the neighbourhood only for witnessed boundary blockers.
5. Separately improve bounded CSP ordering/propagation and candidate economics.
   Compare dynamic remaining-domain and conflict-degree ordering on a captured
   budget trap under identical budgets; never turn search effort into physical
   congestion. Cache geometry and compatibility, and measure physical candidate
   yield rather than raw enumeration. Budget exhaustion is unknown, not proof
   of board or finite-candidate infeasibility.

### Research and integration rationale
CUGR (https://github.com/cuhk-eda/cu-gr) motivates evaluating global decisions
through detailed routability. OpenROAD DRT
(https://openroad.readthedocs.io/en/latest/main/src/drt/README.html) separates track
assignment from detailed routing and repair. MAPF-LNS2
(https://ojs.aaai.org/index.php/AAAI/article/view/21266) motivates conflict-focused
subset replanning; persistent PCB copper, widths, and vias require a different
compatibility model, with no inherited completeness guarantee. CP-SAT status
semantics (https://developers.google.com/optimization/cp/cp_solver) illustrate
why UNKNOWN and INFEASIBLE must remain distinct. These are conceptual integration
points, not recommendations to port an IC or time-dependent MAPF router wholesale.

### Verification and decision
This review changes documentation only and reuses completed V73 evidence. The
machine-readable diagnosis, source artifacts and ranked experiments are preserved
in reports/benchmark_history/2026-10-08-tier1-strategy.json. Implementation should
first pass deterministic physical microcases, then existing regression checks,
then all Tier 0 and Tier 1 with unchanged rules/configuration. Record unique
physical conflicts, opens, candidate-family coverage, checks per improvement,
iterations, runtime, and external DRC categories. No tier promotion until all
required connections and KiCad legality checks pass. Increasing budgets alone,
removing width provenance, or restoring search-derived pressure is not supported.


## V74 — Reserve allocation after assembled straight-copper evidence (completed; opt-in)

### Failure / hypothesis
V73 first supplied assembled regional diagnoses after the last global allocation.
Reserve the penultimate straight-feedback epoch for one negotiated fresh topology,
then consume its localized evidence in the final already-budgeted global epoch.
No copper rules or global iteration budgets change.

### Change
The reserved epoch performs real straight realization instead of a cheap discrete
probe, bypassing the coarse-oracle deferral only for a negotiated topology. Its
checkpoint is paired with its own regional history and overrides ordinary
stagnation diversification for the next retry. The final fresh replan receives
physical assessment even if coarse beam ranking would prune it; the configured
portfolio bound and the previous best physical incumbent remain preserved.
Review caught and removed an earlier retained-topology selection which would have
mixed fresh global diagnostics with an older assembled topology.

### Verification so far
The old crossing fixture always has sampled overflow and correctly does not reach
this gate. A new parallel-terminal fixture has zero sampled/corridor overflow,
four assembled physical violations, and two affected nets. It asserts an actual
penultimate assessment, regional history transfer into the final epoch, subsequent
physical assessment, unchanged three-iteration budget and the no-retry boundary.
Focused and all 54 core tests pass. Tier 0 passes external KiCad DRC against the
immutable V74 executable. An early test invocation raced compilation; only the
subsequent sequential fixture/core runs are authoritative.
Seven broad CTest failures reproduce V73. The candidate-bridge check passes on
both binaries after its missing generated fixture is prepared; it is not a
routing regression. Full Tier-1 assessment is still running.

### Initial real-board evidence
The penultimate bm_svf epoch completes an actual regional pass, leaving 1253
history entries / 8896 pressure; the final global epoch starts with those 1253
entries. Baseline V73 ended the equivalent epoch at 937 / 7117 without assembled
assessment. This verifies a consumed physical feedback path, not yet improved
final routing or Tier-1 qualification. Full artifacts are under
build/assembled-feedback-20261008/; the final benchmark decision is pending.


## V75 — Complementary octilinear elbow candidates (completed; Tier1 unqualified)

### Failure / hypothesis
The existing staggered two-route micro-board is topologically simple but its
fixed diagonal-first elbows overlap (two native/four quantized reports). An
axis-first followed by 45-degree elbow preserves the same endpoints and width
while offering a physically distinct realization. This is a bounded geometric
candidate experiment, not a replacement for global topology or joint repair.

### Change
Record owned fragments which actually need a non-octilinear chord converted.
Intersect those with exact physical conflict participants, then try complementary
elbows through the existing complete native and quantized scene checks. Preserve
accepted elbow policies across subsequent retained-template trials. Geometry and
bundle/pair candidates share the existing max_straight_portfolio_checks budget;
when both families have candidates, reserve half for each, with unused geometry
capacity left to bundle search. Reserve-only templates are not treated as active
bundle candidates. Separate geometry-check/improvement counters expose usage.
No default search budget or copper rules change.

### Micro evidence / verification
The synthetic conflict reaches zero native/quantized errors and zero opens after
two geometry checks; the first variant is rejected, the second is accepted.
Regression coverage includes four reflected/rotated geometries, width variation,
noneligible and unimplicated fragments that sort before useful candidates,
reserve-only availability, and a second conflict requiring a later template
change without undoing the accepted elbow. Endpoints, layers, widths and integer
octilinear segments remain asserted. All 55 core tests pass. Tier 0 passes
external KiCad DRC using the immutable combined V75 binary. Independent review
finds no remaining material defects. Tier-1 results remain pending.

### Measurement
V74 and V75 use separate immutable binaries and the same normal manifest with
P24, 30 global iterations and four feedback retries. Their runs overlap, so raw
runtime will be recorded but not promoted as a controlled performance result.
Artifacts and source snapshots: build/dogleg-alternatives-20261008/.

### V74 completed Tier-1 result
The full normal-budget run remains unqualified: 78 paths, zero unrouted nets,
zero sampled/corridor overflow and zero opens, but 751 assembled violations and
434 native conflicts, identical to V73. Actual global iterations increase 11→13
within the unchanged cap30. The newly feedback-informed topology is worse:
its coarse oracle has856 violations (prior checkpoint540), and its assembled
assessment has1514; retained physical incumbent selection preserves751. Thus
feedback is consumed but does not by itself improve routing. Duration3413.54s
versus2520.76s is observational because runs overlap. Do not promote this as a
Tier-1 quality improvement. Final retention decision awaits V75. Structured
report: reports/benchmark_history/2026-10-08-assembled-feedback.json.

### V75 research note — posture alternatives
Primary implementation reference revisited: KiCad LINE_PLACER::FlipPosture,
https://docs.kicad.org/doxygen/pns__line__placer_8cpp_source.html . KiCad explicitly
switches straight/diagonal trace-head posture. Flux applies the underlying idea
as bounded endpoint-preserving candidates, without copying implementation.
For k eligible fragments, at most the configured portfolio budget is tried;
each trial still costs a full assembled scene validation. The local assumption
is that changing elbow posture can remove an overlap without topology/layer
changes. That holds for the regression fixture; it does not imply dense Tier-1
conflicts can be solved by independent elbow flips. Joint topology-changing
repair remains a separate hypothesis.

### V74 retention decision
Disable the additional reserved assessment by default. It remains reproducible
with RoutingPlannerConfig::enable_assembled_straight_feedback or CLI
--assembled-straight-feedback. The immutable V74/V75 benchmark executables
precede this switch and correspond to enabled behavior. The default-control
regression explicitly checks prior intermediate-probe scheduling.

### V75 completed Tier-1 result and decision
Full normal-budget result remains unqualified:78 paths,0 unrouted,0 overflow,
0 opens,751 assembled violations. Native conflicts434→430, track violations
536→534, hole clearance60→61 and hole-hole2→3; other categories unchanged.
The same total hides changed conflicts, not completion. Geometry trials12,
accepted5, combined portfolio checks24; selected scope retry2 (V74 retry1).
Observed runtime2757.84s; concurrent workloads preclude a speedup claim.
Keep the bounded candidate mechanism because it solves the independent physical
microcases with exact native/quantized proof, but do not promote the existing
portfolio budget above its default0 or mark any tier newly passed. The reserved
assembled-feedback scheduling remains separately opt-in. Current default-off
production passes55 core tests and Tier0 external KiCad; the completed V75
Tier1 run used the prior immutable executable with reserved feedback enabled.
There is no completed full Tier1 run of the final default-off executable.

### Remaining mechanism and next experiment
More legal elbow realizations and a working upstream feedback handoff do not
remove the dense connected physical conflict set. The evidence now favors a
bounded conflict-component topology/layer search with simultaneous changes,
using exact scene validation and retaining the incumbent, over further penalty
or retry tuning. Before another real-board search, construct a three-net
ordering/layer trap that cannot improve via any single accepted move; measure
whether joint candidate enumeration escapes it. Avoid claiming this algorithm
has been implemented in this change.

### Bounded feedback-count pilot
Fixed global budget30, regional bundle-state limit1 and straight portfolio0;
retries4 versus29, each capped600s. This is not a normal tier configuration.
The4-retry arm timed out during final regional search; completed coarse epoch
counts were674,613,499,540. The29-retry arm completed after585.9s, actual global
iterations30; minimum coarse epoch554, last epoch1014, repeated late counts
882–883. Selected retained topology oracle597,0 opens, but assembled straight
scene was NOT evaluated.597 is not a final assembled or external KiCad DRC
count and cannot be compared with the normal benchmark's751. Neither arm
qualifies the board. No retry-budget/default change follows from this pilot.
Artifacts: build/feedback-budget-pilot-20261008/.

## Experiment V76 — Measured regional reuse and bounded coordinated topology

### Failure and hypothesis
The pipeline audit brackets most elapsed planning time inside bundle assignment,
including nested geometry/domain/access work. Tier 1 remains physically illegal
although its coarse topology is complete. At the user's request, implement five
related pipeline changes together: timings, bounded domain reuse, fixed-copper
incompatibility pruning, coordinated three/four-window changes, and certified
copper handoff. This bundled experiment cannot attribute any board improvement
to an individual change without subsequent ablations.

### Change
Added inclusive microsecond timers and invocation/cache/pruning counters. Public
regional analysis aggregates discarded scope attempts; planning also aggregates
all regional calls and exact promotions. Parent and child timings overlap and
must not be summed. Reused immutable-board free-space products across regional
scope retries using independent LRU budgets (sources 8, geometry 8, navigation
32). Active navigation handles own their data independently of cache eviction.
Pruning only proves incompatibility for fixed straight H/V/45-degree templates;
unknown/deformable alternatives remain searchable. Joint portfolio search can
change connected groups of three/four windows under the existing exact-scene
budget, retaining the incumbent. Certified legal scenes carry immutable source,
path and realization identity into detailed routing, which still performs final
validation. Reuse is conservatively restricted to no-refinement straight scenes;
other geometry policies fall back to materialization.

### Verification in progress
Task-only source snapshots: build/pipeline-single-pass-20261008/baseline/.
Pre-pass Tier 0 passed external KiCad validation. An immutable pre-pass Tier 1
control is running with straight geometry and portfolio budget 24. Focused
regressions cover fixed crossings, a three-window trap, immutable copper reuse,
stale source/path identity, requested geometry transformation, and final DRC.
Independent review identified cache-reference lifetime and geometry-policy
issues; both were corrected before final validation. Build/tests and matched
Tier 1 experiment results are pending. No tier promotion or performance claim.

### Focused and lower-tier result
The final immutable experiment executable has SHA-256
597df546808b79867f1c9587c8572d0707c2ce022076c88a23e0e5d111972054.
Five focused tests pass, all 57 core tests pass, and Tier 0 qualifies with 50
paths and external KiCad DRC. The pruning fixture was corrected to identify
fixed alternatives explicitly and exclude an unused crossing alternative from
the clearance-equality subcase. The production pruning rule was not weakened.
Second independent review found no remaining material correctness issue;
cache-eviction lifetime and cumulative promotion accounting lack dedicated
regression fixtures. Matched Tier 1 control/experiment remain in progress.

### Tier 1 completion result
The matched control and experiment completed. Both assembled scenes had 751
violations and 0 scene opens, so qualification remains false. The coarse plan contains 78 paths and no
unrouted connections; the runner reports 78 opens for the unaccepted detailed
output, while the assembled scene has 0 opens. Observed command metadata reports control 43:24.80 wall time and
549740 kB RSS versus experiment 38:44.87 and 613008 kB. The experiment made 4
multi-window checks with 0 improvements; cumulative alternative cache counts
were 20482 evictions, 124867 navigation hits and 8692 misses (candidate cache:
8309 evictions; source 2140/1359, navigation 49336/3499, geometry 0/3499
hits/misses). The coordinated search did not improve the assembled scene;
retain this as evidence and do not promote Tier 1.

### V76 benefit/cost re-analysis
Per-board planning duration decreased from 2591.771 s to 2312.487 s (10.8%
observed), with unchanged selected copper: 751 violations, 0 scene opens,
430 native conflicts. Bundled changes and concurrent workloads prevent causal
attribution or a repeatable speedup claim. Reported experiment wrapper RSS is
613008 kB versus control 549740 kB; the experiment raw time log was not retained,
so preserve its provenance as the benchmark-agent report rather than an
independently recovered artifact measurement.

Cumulative inclusive timings: assignment 2179.939 s (94.27% of planning),
alternative domains 697.461 s, access selection 651.623 s, bundle search
381.728 s, channel construction 0.299 s. These overlap. Lane timing is biased
by per-call microsecond truncation across 27,920,949 invocations; do not call
lane work negligible from the reported 71,462 microseconds. Use nanoseconds or
batch timing and exclusive attribution before a solver rewrite.

Navigation cache hit rate is 93.49% (124867 hits / 8692 misses); caching is a
promising performance contributor, not an isolated measured speedup. Geometry
cache hits are zero in the selected analysis (3499 misses); investigate marginal
reuse and retained bytes before enlarging it. Fixed-copper pruning did zero
checks/rejections: its conservative fixed-template predicate does not cover this
workload. Four >2-window probes made no improvement in the selected result;
this supports keeping the capability bounded, not raising its budget. Certified
scene handoff was unavailable because the scene was illegal, so it cannot explain
a performance improvement on this frontier board.

Highest-value performance experiment: prepared access checks, compatible domain
reuse, and reduced repeated equivalent constraint evaluation, with full context
keys and cached proven failures separated from budget exhaustion. Highest-value
routing experiment: physically realized bend/escape/via candidates with lazy
exact compatibility constraints in a small witnessed conflict neighbourhood;
regenerate topology/layer/via alternatives when the candidate pool cannot solve
it. Existing cross-section packing is not a proof of legal inter-section copper.
Keep global negotiation and exact final DRC. Defer larger general search budgets,
whole-board exact solving, and curved/GPU optimization for this straight case.
OpenROAD's staged detailed-routing architecture supports separating track
assignment from search/repair and DRC, not assuming one certifies the next:
https://openroad.readthedocs.io/en/latest/main/src/drt/README.html .
No new algorithm was implemented during this re-analysis.

## Experiment V77 — Prepared access reuse and physical candidate scheduling

### Failure and hypothesis
V76 still selected 751 copper violations on bm_svf. Candidate generation and
assignment consumed 94.3% of planning; access selection repeated more than five
million times. Native and quantized copper are separate acceptance gates; native
conflict counts are not a lower bound on the quantized objective. The user
requested the four ranked improvements together, so this is a bundled experiment.

### Implementation
Performance samples accumulate nanoseconds before reporting microseconds. Bundle
search exposes time excluding its nested access evaluator. A 4096-entry access
cache and board-rule preparation span one immutable window/template enumeration
and immediate downstream validation; exhausted searches are never cached. Entry
peaks and shallow storage estimates are explicit (not total heap/RSS).

Straight realization is split into native preparation and independent quantized
finalization. Up to four prepared candidates are ordered by native compatibility;
no native-count heuristic prunes a candidate. Bends, runups and via spans retain
the existing materialization/DRC contracts. Each scheduled candidate remains
eligible for full proof within the original shared budget. The original native
and quantized acceptance gates, and open-connection non-regression, are preserved.

A complete small fixed retained domain includes original selections and every
retained alternative. Only untruncated domains within the configured window cap,
complete exact coverage, comparable scenes and no provisional-context conflicts
can report retained_fixed_pool_exhausted. This is not global unroutability. Other
cases report budget_or_scope_unknown. Shortlist/reserve candidates are interleaved
so reserves can supply different retained topology/via choices after rejection.

Single/pair/multi families use deterministic improvement-per-native-primitive-work
scheduling, warmup and periodic exploration. Wall time is observed but cannot
change routing order. The initial geometry family relinquishes its bounded share
after two fruitless probes when portfolio alternatives exist. Counters distinguish
native previews from expensive full-scene checks, including discarded previews.

### Baseline and verification
The preserved baseline executable is identical to V76 SHA-256
597df546808b79867f1c9587c8572d0707c2ce022076c88a23e0e5d111972054.
Snapshots: build/pipeline-physical-pass-20261008/baseline/.
Focused tests cover sub-microsecond accumulation, scheduler exploration, bounded
cache capacity, reuse of access leaves, unknown exhausted access, physical
preview ordering, reserve-only escape, coordinated moves and exhaustive fixed
pool diagnosis. All 60 core tests and Tier 0 including external KiCad DRC passed.
The immutable normal Tier 1 run remains in progress. An independent
review of the new scheduling/certificate block found no material correctness
issue before compilation; final verification remains required.

### Parallel research — erosion and other experimental geometry
The research artifact is reports/diagnoses/2026-10-08-novel-algorithms-analysis.json.
Confined anisotropic erosion is a bounded physical-rejection retry, not a remedy
for the initial assignment bottleneck. Its clearance/topology/exact-copper gates
must remain authoritative. Evaluate it on a fixed conflict-cone replay with
checks 0 versus 32; accepted-template counts alone do not demonstrate a solved
board. Historical XPBD work reduced local conflicts at modest GPU solve time,
but did not complete Tier 1. USPED remains opt-in without established whole-board
benefit and can increase mesh cost. Skeleton-derived capacity is a plausible
upstream infeasibility filter, subject to conservative geometric bounds and
false-prune testing. No erosion/USPED/XPBD defaults were changed in this pass.

### Side discussion — resumable routing controller (proposal, not implemented)
The user proposed selecting small pipeline segments from routing/routability,
conflict and DRC state until completion or timeout. Existing work-plan execution
already commits validated board checkpoints transactionally; persistent routing
sessions and plan_routes retain feedback/checkpoints and propagate cancellation.
They do not provide a shared deadline or resumable per-kernel continuation.

Recommended design: hierarchical lifecycle states (prepare, search, validate,
complete, timed_out, cancelled) plus a deterministic evidence-based action policy.
Each kernel accepts a work slice/deadline and returns candidate deltas, typed
observations, work consumed, and an in-memory continuation. Preserve separate
best search and last validated board checkpoints. Context versions identify
board/rules/topology/geometry dependencies so stale evidence and caches cannot
be consumed after changes. Exact copper and external KiCad gates alone authorize
completion; a fixed-pool exhaustion certificate never means global unroutability.

Start by expressing the existing execution order as one policy, with parity
checks, then make access/bundle search and regional repair resumable. Adapt action
choice from typed failure evidence and deterministic yield estimates, retaining
exploration and cycle detection. Erosion is a possible bounded alternative-
generation action after a witnessed physical rejection/pool failure; XPBD is a
local deformation action where topology remains plausible. Their availability
must depend on assumptions, not just elapsed time or generic stagnation. Evaluate
time-to-valid-route and DRC-versus-time curves at fixed overall budgets. A layer
that repeatedly restarts whole routing sessions would not provide the intended
benefit. This note authorizes no new controller implementation in the current pass.

## Experiment V78 — Resumable evidence-driven routing core

### Request and hypothesis
The user requested the complete state-machine design as the new implementation core in one pass,
including erosion, XPBD and USPED. The existing pipeline can spend a long time inside a stage with
no controller decision boundary. Preserving a live kernel continuation should make progress and
cancellation observable without restarting prepared geometry or search frontiers. Typed physical
feedback should select an appropriately scoped next action rather than replaying a fixed pipeline.

### Implementation
`RoutingEngine` now owns working and selected checkpoints, action budgets, a shared deadline,
versioned external-validation handoff and deterministic action selection. Public `plan_routes`,
persistent sessions, CLI routing and their existing optimizer/viewer callers enter the engine.
`plan_routes_legacy` preserves the original interleaved planner for explicit comparison and existing
specialized oracle-first/assembled policies.

A `RoutingKernel` continuation parks its worker at search checkpoints. Local search stacks, frontier
state and caches remain live across work/time slices. Only completed action transactions update
controller state. Cancellation discards the in-flight transaction. The controller separates global
allocation, quantized probing, regional realization, materialization, confined erosion, XPBD and
USPED-informed replanning. Global feedback uses attributed affected nets and accumulated routing
knowledge; global iteration allowances are shared across epochs. Algorithm ranking uses comparable
within-stage quality changes and deterministic work counts, with warmup and exploration. Wall time
is diagnostic. Geometry and typed regional evidence bind repeated-action detection to a state.

XPBD and confined erosion actions are enabled only for the curved realization kernels that support
them. Straight routing still uses hydraulic global topology candidates. The USPED action explicitly
enables its NavMesh path: merely setting the USPED flag in a straight pipeline would be a no-op.
All physical and external acceptance gates remain separate. A plan awaiting materialization is not
completed copper, and an internally legal board waits for generation-matched external DRC.

### Regression findings during integration
The first integration failed Tier 0. It missed early certification of a usable global seed and the
existing exact coarse-polyline fallback. Restoring both retained the original legality checks.
A subsequent run exposed a second distinction: conservative global overflow is not proof that a
connected seed cannot be physically legalized. The controller permits bounded physical certification
of that connected seed, while missing connections still prevent regional routing. Neither correction
relaxes widths, clearances or DRC.

Independent review caught stale externally rejected checkpoints, rejection bypassing the repair
policy, and deadline/cancellation classification. These were corrected. Historical tests asserting
exact old epoch ordering now call the explicit legacy control; all their assertions remain intact.
New tests exercise engine slice invariance, actual yields, deadline/cancellation, current-generation
external acceptance/rejection, and dispatch/configuration of all supported physical families.

### Verification and decision
Builds and focused tests passed during integration. Earlier intermediate full runs were 60/62 and
63/64, and Tier 0 failed as documented above. Final regression and Tier 0 verification is recorded in
`reports/benchmark_history/2026-10-08-routing-engine.json` when complete. The frontier pilot has a
120-second shared deadline and cannot promote Tier 1 without complete final KiCad validation.
Full design and operating limits are documented in the Resumable routing engine section of design.md.

The final integration run passed 64/64 core tests. Its remaining Tier 0 failure was traced to the
CLI preview adapter, not the engine's copper: engine outcome was awaiting-external-validation and
materialization had zero exact errors, but `--write-incomplete-preview` regenerated the still-
overflowing coarse graph over the legal regional result. The preview branch now requires an
incomplete detailed result and cannot replace an accepted or provisional zone-refill candidate.
The external Tier 0 rerun is preserved separately from those failed intermediate artifacts.

### V77 immutable Tier 1 validation — 2026-10-08

The immutable V77 binary was run against Tier 1 with straight geometry and 24 straight portfolio
checks. Both corpus checks completed: `pt2399_delay` qualified with exact copper and external KiCad
DRC, while `bm_svf` remained unqualified (`unlocked`) with 78 open connections despite zero reported
unrouted paths and zero global overflow. This matches the prior V76 single-pass baseline exactly in
qualified boards (1/2), tracks (243), vias (5), and `bm_svf` open connections (78). The qualified
board duration was 2.074951504 seconds versus 2.106970421 seconds in V76.

The benchmark result and machine-readable comparison are preserved in
`reports/benchmark_history/2026-10-08-v77-immutable-tier1.json`. Wrapper stderr was empty, so
`/usr/bin/time -v` timing/RSS provenance is unavailable; per-board duration comes from the runner's
result JSON.

V77's completed normal Tier 1 result requires a qualification/geometry distinction: qualification
remained 1/2, but bm_svf selected-scene DRC increased from 751 (V76) to 809 (V77), with zero scene
opens in both. The unaccepted-output benchmark status still reports 78 opens. Planning was
2260.710 s versus 2312.487 s; concurrent bundled runs do not establish a causal speedup. The cache
avoided 264,327 access evaluations. Measurement/cache reuse is supported; the physical scheduling
bundle needs isolated refinement and is not a demonstrated quality improvement. Detailed numbers
are in reports/benchmark_history/2026-10-08-v77-immutable-tier1.json.

### V78 final verification and decision — 2026-10-08

The final immutable engine binary passed all 64 core tests and the continuation kernel tests.
Independent review found no remaining material correctness defects. Tier 0 pt2399_delay passed
external KiCad DRC with 254 tracks, three vias, zero opens and zero copper violations; detailed
routing took 2.071 seconds. The earlier baseline used 243 tracks and five vias.

The bm_svf frontier pilot exercised the engine's own 120000 ms deadline, separately from a harness
process timeout. It returned `timed-out`, `routing_plan_complete: false`, exit 3 after 120.09 seconds
(101.25 user seconds, 427180 KB maximum RSS), preserving 78 coarse paths with no coarse unrouted
nets. Global `complete: true` describes topology only. Physical routing and Tier 1 remain unqualified.

Decision: retain the new core as verified orchestration infrastructure, without claiming Tier 1
quality improvement. Confined erosion and XPBD currently require curved realization; USPED enables
its NavMesh integration explicitly. Continuations are in-memory and deadlines are cooperative.
Next experiment: compare fixed-budget action traces and physical outcomes against legacy scheduling
on the frontier, then target its earliest persistent conflict using the saved typed evidence.
Final provenance is recorded in reports/benchmark_history/2026-10-08-routing-engine.json.

### Curved routing default — 2026-10-08

At user request, enabled curved realization by default in the planner, routing engine/session,
integrated optimizer, CLI and corpus runner. Explicit straight mode remains available and disables
arc refinement in the engine. The viewer already displays PH telemetry by default. Updated
geometry-option validation so default curved mode does not invalidate unrelated bare commands.

Release and development CLI builds passed; all 64 core tests passed. CLI smoke checks accepted
default/explicit curved/explicit straight and rejected conflicting straight/arcs options. A curved
Tier 0 corpus check exceeded its 180-second harness limit before final DRC qualification. This is
not an engine-deadline result and not a tier pass; curved performance needs further investigation.
The user's requested default is retained, with explicit straight comparison still available.
Provenance: reports/benchmark_history/2026-10-08-curved-default.json.

### Unravel boundary-assignment search assessment — 2026-10-09

Failure context: extended curved Tier 2 produced 117 coarse paths and zero coarse unrouted nets but 381 overflow, exhausting after 9305.4 seconds without detailed copper. Repeated regional, XPBD and erosion actions failed to improve the selected state. Tier 0's six unrounded bends and external KiCad connectivity discrepancy are separate materialization defects; Unravel is not expected to fix them.

Source: https://blog.autorouting.com/p/the-unravel-algorithm-untwisting and tscircuit/tscircuit-autorouter commit 9336cd949fb478669628d74620bfc4135c9ff087, lib/solvers/UnravelSolver. Local research copy/provenance: build/research-unravel-20261009/. Upstream repository is MIT licensed; attribution/license must be retained if code is reused.

Mechanism: bounded search over shared region-boundary point positions and layer assignments, preserving connection ownership. The inspected implementation scores current estimated failure probability (f=g, h=0), caps the queue at 500 candidates and stops after 200 iterations without improvement or 50000 iterations. It is best-first state optimization rather than an optimal additive-cost A* guarantee. Only better proxy states are committed by the multi-section driver; this does not guarantee physical DRC improvement. Crossing checks use region chords and layer transitions, not full width/clearance-expanded copper.

Existing Flux overlap: solve_lane_order_dynamic_program and solve_prepared_lane_packing already search clearance-aware channel permutations. RegionalRouteTemplate alternatives retain original boundary portals; adjacent_channel_lane_precedence_legal rejects inconsistent channel ordering. The incremental opportunity is coordinated interface reassignment across adjacent windows, with a valid connector topology, not another independent per-channel permutation solver.

Hypothesis: a bounded coupled boundary-assignment search can remove order-induced crossings before expensive physical repair, reducing exact-oracle calls and time to a legal regional realization. Benefit is conditional on a witnessed order inversion/crossing; true capacity deficits and fixed terminal permutations may require another corridor or a legal layer transition.

Integration: generate paired/multi-route alternatives before regional bundle enumeration, or expose a resumable boundary-reorder engine action before XPBD/erosion when ordering evidence exists. Preserve net/connection/run/fragment identity, fixed terminals, coupled-net constraints, clearance widths, legal via spans and frozen exterior interfaces. Regenerate affected topology certificates and canonical state keys; keep existing lane precedence and native/quantized copper checks. Validate affected neighbors as well as the changed region. Parallelize only regions with disjoint interfaces, or revalidate against a versioned snapshot before commit.

Cost: naive pair mutation branching is quadratic in participating boundary ports; layered choices and mutation depth can make the state space exponential. Upstream issue checking compares route pairs inside each region. For Flux, use adjacent swaps, small joint scopes, incremental crossing deltas, canonical duplicate detection, a bounded frontier and explicit work/stagnation limits. The expensive cost is regenerating/validating coupled geometry, so send only a small diverse shortlist to that stage.

Minimum experiment (proposed, not implemented): one two/three-net synthetic case where fixed outer terminals permit an internal boundary reassignment, plus an impossible same-layer case requiring a legal via. Then run a saved Tier 1 conflict component with fixed seeds and equal total work budgets. Measure remaining physical conflicts, accepted realizations, opens, vias, route length, exact-oracle calls, states and runtime. Final acceptance requires KiCad DRC; abstract crossing reduction alone is insufficient.

Decision: promising targeted topology operator, medium integration cost, unmeasured Flux benefit. Reuse Flux's regional portfolio and oracle machinery; do not replace the core or port upstream heuristic probability constants. Next step is a bounded ablation against current regional search, not enabling it globally without evidence.

### Extended curved tier capture outcomes — 2026-10-09

Release runs used 10740-second engine / 10800-second outer per-stage limits, with concurrent execution. Tier0 exhausted after 6451.4 seconds detailed routing; its 142-arc candidate retained six unrounded bends and one external KiCad unconnected item. A one-global-iteration control finished in 688.98 seconds with byte-identical rendered copper. Tier2 exhausted after 9305.4 seconds planning: 117 coarse paths, zero coarse unrouted nets, 381 overflow, no detailed copper. Tier3 exhausted after 422.6 seconds with a 30-iteration budget: one path and 64 unrouted nets. Tier1 and Tier4 ended without final results after interruption; telemetry proves work in progress but does not establish timeout or OOM. Do not classify either as a completed benchmark. No tier qualified.

Actual Tier0 copper and diagnostic images from every tier were viewed. Smooth local curves coexist with sharp corners, long detours and unfinished physical routing. Extended runtime alone did not establish smooth, logical, DRC-clean final copper. Artifacts, visual observations and precise limitations are in reports/benchmark_history/2026-10-08-curved-extended.json and reports/diagnoses/2026-10-08-curved-extended-visual.json. The source binary remained frozen; runner additions preserve extended timeouts and rejected-copper rendering.

### KiCad 10 net-assigned cubic copper feasibility — 2026-10-09

User direction: convert PH routing geometry to KiCad graphical Béziers on copper with net assignments, replacing line/arc decomposition. Official KiCad10 manual confirms electrical copper graphics: https://docs.kicad.org/10.0/en/pcbnew/pcbnew.html#graphical-shapes. Installed KiCad10.0.6 was tested independently; stale S-expression documentation must not be used to dismiss netted graphics.

Capability results: cubic PCB_SHAPE/gr_curve preserves controls/net through save/reload, connects pads, connects through a via-only chain (missingcurve/missingvia negatives each expose one open), detects shorts between distinct-net curves, detects curve-to-curve and curve-to-pad clearance, and plots Gerber. A valid-outline close-gap test reports seven clearance violations, including a curve/curve pair; the otherwise equivalent safe-gap board has zero violations and opens. The early shared-endpoint crossing test was inconclusive and superseded by distinct terminal pairs.

Confirmed rule gap: an explicit 0.25mm minimum track_width rule rejects a 0.1mm ordinary track but not a 0.1mm graphic curve. Flux must enforce width itself; do not count a graphics-only KiCad DRC pass as evidence of minimum track width.

Conversion constraint: Flux PH spans already have six quintic Bezier controls. KiCad gr_curve is cubic (four controls), so generic PH cannot be exported exactly as a single cubic. Use adaptive degree reduction with a certified deviation bound after integer quantization, preserving endpoint contacts and tangent continuity. Reserve fit error in clearance or directly revalidate the fitted cubic envelope.

Integration design: add a canonical netted cubic copper primitive, parser/serializer and strict round-trip coverage; extend connectivity, DRC, routing obstacles, widths/length/count metrics and source preservation. Then fit PH to cubic copper in regional/detailed materialization. Retain PH as optimization representation; publish emitted cubics separately in telemetry. Do not write decorative graphics invisible to internal board validation.

Decision: KiCad capability validated, worth implementing as an electrical output backend. Production exporter unchanged by this feasibility experiment. Raw fixtures, DRC, Gerbers and summary: build/bezier-kicad10-probe/.

## Experiment — KiCad 10 electrical cubic output (2026-10-09)

### Failure and hypothesis

The active PH realization reduced smooth quintics to straight segments and circular arcs.
KiCad 10 can instead retain net-assigned cubic copper graphics, reducing dependence on the
line/arc approximation. A quintic is not generically a cubic: the conversion must certify error
and then revalidate the actual emitted copper, rather than merely relabeling PH controls.

### Change

Added canonical `TrackBezier`, adaptive Hermite cubic degree reduction with degree-elevated
control error bounds after integer quantization, monotone span checks, join-angle checks,
and bounded subdivision. Detailed/regional materialization, checkpoints, deltas, source copper
obstacles, work-plan authority, telemetry, viewer, source-preserving KiCad IO and Python candidate
application now carry the cubic controls. Curved routing remains the default.

Outer adaptive envelopes govern collision checks; contracted inner envelopes plus exact endpoint
caps govern connectivity. Curve widths receive explicit minimum-width checks. Differential skew
uses subdivided chord/control-polygon length intervals, preventing approximate chord lengths from
falsely certifying a pair. Conservative uncertainty can reject a marginal legal interior contact
or skew threshold; no clearance or final DRC gate is relaxed.

### Review and validation

Independent review found mismatched fallback numbering for source graphics, which was corrected
and covered by a silk-before-electrical/no-UUID regression. Added tests cover cubic fit error,
subdivision limits, source-preserving IO, locked deletion rejection, width, dynamic collision
identity, connectivity positive/negative controls, and curved length/skew bounds. Python bridge
checks use the installed KiCad IPC Bézier property names. Final run outcomes are recorded in
`reports/benchmark_history/2026-10-09-bezier-backend.json` after verification.

### Decision

Retain as the KiCad 10 curved output backend, subject to the recorded validation results. This
change does not establish that any previously failing routing tier is solved. KiCad DRC of the
exact exported board remains the qualification oracle.

Final verification: release CLI and viewer build passed; final-source core tests 64/64, focused
cubic tests 3/3, Python bridge tests 7/7. The bounded Tier 0 run emitted 174 Béziers and 56 straight
tracks with three vias, zero internal opens/copper violations, and 16 unresolved bends. Exact
exported preview passed KiCad 10.0.6 DRC with the original project rules (zero errors and opens).
It remains incomplete and is not a tier promotion. Runtime 180.31s, peak RSS 598864KiB.
The trial preceded final source-curve bookkeeping corrections; final regression tests used all
corrections. Both exported fixture and Tier 0 cubic renders were viewed.

## Experiment — LittlePlanets-style height-evolution discovery (2026-10-09)

### Failure and hypothesis

The bounded Tier 0 run reaches connected physical copper but can retain unsmoothed joins, with
multiple geometry preparation/endpoint-binding failures. Existing hydraulic discovery follows a
fixed destination potential and reinforces droplets. Hypothesis: evolving an inverse-clearance
surface through repeated erosion/deposition supplies a useful alternative corridor prior. It is
not assumed to repair invalid polygon offsets or terminal attachment automatically.

### Research and change

Inspected LittlePlanets' active Vulkan height-evolution chain in the local terrain project. Added
an explicit CPU experiment: D8 drainage, topological contributing flow, USPED-style capacity,
mask-aware flux divergence, bounded erosion/deposition, then positive-cost path extraction that
can climb saddles. The immutable legal mask, exact chord certificates, corridor projection, and
final copper checks remain authoritative. Defaults retain GPU droplets; `--terrain-height-evolution`
selects the new backend. Dedicated metrics and field labels identify it. This is not a silent GPU
fallback or a wholesale replacement of NavMesh.

### Before

One-iteration Tier 0 global baseline: 56.68s, peak RSS 428428KiB, 50 paths, zero unrouted nets,
three clearance-shadow conflicts. Hydraulic proposals: 48 attempted, 45 accepted. Minimal fixture
baseline completes in 0.22s but exercises no hydraulic proposals, so it is a CLI regression only.

### Validation and decision

See `reports/benchmark_history/2026-10-09-height-evolution.json` for measured results and decision.
Synthetic tests distinguish real height changes, uphill basin escape, narrow-neck connectivity,
obstacle preservation, and exact polygon-to-corridor projection from visual plausibility.

Final measured result: 46/48 proposals accepted versus 45/48 with droplets, with the same three
residual global clearance-shadow conflicts. The detail run recorded 48 requests, 46 actual solver
invocations, 460 evolution iterations, 553963 changed cells summed across solves, and 39889 path
expansions. At 180.17s it exported 178 Béziers with zero internal opens/copper violations but nine
unsmoothed joints; the engine timed out and the preview remains incomplete. KiCad DRC with the
original project rules passed with zero violations and unconnected items. The earlier Bézier run
had 16 unsmoothed joints, but this comparison is not a controlled ablation.

Release build, core 64/64, and focused terrain tests 2/2 passed. Independent review prompted
cancellation propagation, failure-work accounting, actual-invocation counters, and limiting the
flag to global discovery so confined anisotropic repair retains its directional solver. Final
build/focused tests include cancellation diagnostic fixes. Retain the backend as opt-in: evidence
supports usable corridor discovery, not improved tier completion or a speedup. Next experiment:
compare zero-iteration clearance search against evolved heights under identical budgets, then
consider cached layer fields only if evolution itself earns its cost.

## Experiment — Default terrain discovery with live copper feedback (2026-10-09)

### Failure and requested change

Height evolution previously reconstructed terrain from static free space and accumulated failure
history only. Copper laid or ripped up by a later engine action did not itself change the next
terrain solve. The user requested height evolution as the default and current copper feedback
between state-machine passes. This default change is user-directed, not a benchmark promotion.

### Hypothesis and design

A replaceable net/layer-scoped copper snapshot supplies occupancy pressure separately from
persistent failure history. Rebuilding terrain from this snapshot lets removed copper release its
pressure, while retained copper influences other nets. Changed occupancy must invalidate reusable
hydraulic proposals. Provisional routing evidence remains advisory; it does not alter the immutable
legal mask, canonical source board, exact materialization, or KiCad acceptance criteria.

### Verification

Results will be recorded in `reports/benchmark_history/2026-10-09-live-copper-terrain.json`.

Final release validation: core 64/64, terrain targets 3/3, CLI selection targets 4/4 passed.
Independent review fixes covered cycle detection by content rather than monotonic revision,
conservative arc extents, via layer spans, numeric bounds, and resolved net-class clearance.
Incremental tests verify that both adding and removing copper invalidate old proposals; engine
tests verify that later global actions actually consume the replacement snapshot.

The default-mode Tier 0 run (three global iterations allowed, 180s cooperative deadline) completed
six actions before timing out at 195.61s, peak RSS 548664KiB. Input snapshots progressed from empty
to 575 provisional features, 568 regional scaffold features, then 232 materialized copper features.
The feedback-global action consumed revision 3 (the 232 exact candidate features), reduced global
overflow from three to zero, and produced a new 541-feature provisional snapshot. Its subsequent
quantized copper probe worsened from a score of 8 to 56. This demonstrates working feedback but
also that a better global allocation score need not mean better copper.

The retained earlier best preview contains 178 Béziers, 48 tracks, three vias, zero internal opens
or copper violations, and nine unsmoothed joints. KiCad DRC with unchanged original project rules
reported zero errors and unconnected items. Routing remains incomplete and no tier is promoted.
The selected preview's configured terrain revision is 0 because it predates the exploratory
feedback-global pass; attempt telemetry retains the later revision evidence independently.

Decision: keep the user-requested default and feedback lifecycle. All changed snapshots currently
invalidate global reuse conservatively; local invalidation is a future performance improvement.
Next hypothesis: pressure-derived alternative topology needs physical feasibility ranking so
reduced global overflow does not select a worse copper realization.

Both development and release CLI builds passed after explicit aggregate-default initializer
cleanup, and the three focused terrain targets passed again. The updated development executable
is used by `run_tiers.sh`; the viewer build passed with no work required.

## Sequence audit — short engine actions and speculative preparation (2026-10-09)

Evidence is preserved in `reports/benchmark_history/2026-10-09-pipeline-sequence-analysis.json`.
This is an analysis, not an implemented experiment or a tier promotion. The user confirmed that
each action/sequence should be short and focused, returning control so the engine can select the
next action.

The earlier Bézier Tier 0 run spent 11.321s in global allocation, 0.109s probing quantized copper,
45.267s in regional analysis, 70.897s materializing, then 47.177s in XPBD with unchanged reported
quality. Its zero detailed violation score did not represent successful curved routing: 16
unsmoothed joints remained. The terrain-feedback Tier 0 run similarly spent 46.216s regional and
70.263s materializing. Its subsequent global feedback improved allocation score from three to
zero, while the following physical probe worsened from eight to 56. Action-specific score gains
cannot therefore serve as a common measure of end-to-end progress.

In the sampled ongoing Tier 1 trace, global actions took 156.131s and 147.099s, followed by a
0.725s probe. The regional action started at 303.964s and had not returned by telemetry time
6311.627s. Twenty shove-deadlock observations appeared within it; observed diagnostics explicitly
reported that XPBD was not attempted. Internal fallback/scope exploration can therefore occupy
most of the deadline before the engine can choose a different algorithm. Cooperative kernel
yields currently preserve and resume that same transaction; they are not algorithm-selection
boundaries. Per-worker spline timings overlap and must not be subtracted from wall time as if
they were exclusive serial phases.

Code inspection found forced transitions from quantized probe to regional realization and, in
materializing mode, from completed plan actions to materialization. Regional, XPBD, and confined
erosion actions each call the full regional analyzer with different flags; domain caching is
local to one analyzer invocation. The telemetry `accepted` flag is computed after working-state
replacement and is not itself a rollback gate. Best checkpoints are retained separately. The
existing content-based terrain cycle key should be preserved rather than replaced with an
ever-increasing revision.

Priority: split the regional analyzer into resumable bounded scene/conflict preparation,
candidate generation, lane-search expansion, realization, targeted repair, and validation
operations. Every operation should return its immutable input fingerprint, continuation,
candidate/evidence delta, typed outcome, consumed budget, and readiness/dependency information.
The engine must be free to resume it, change algorithm, widen scope, or replan topology. A yielded
search frontier is useful evidence but is not validated copper. Validate final joins/quantization
as early as the relevant primitives exist, and retain final exact/KiCad validation.

Preemptive work should initially prepare immutable obstacle/domain data and bounded terrain,
USPED, and corridor alternatives for likely next regions. Reuse preparation across engine actions
when its complete dependency fingerprint matches. Changed copper should invalidate affected
domains, not automatically every immutable source artifact. Competing XPBD/topology proposals
may later run on isolated snapshots, with bounded resources and serialized validated commits.
Shared mutable regional LRU caches and GPU command pools must not be reused concurrently without
an explicit ownership/synchronization design. Spatial independence includes clearance halos,
via spans, net connectivity, and shared corridor resources, not just disjoint rectangles.

The earlier four-to-eight regional-worker experiment is negative evidence for simply increasing
threads: active-phase time was 326.343s versus 327.324s, with one large window remaining critical.
VPR's documented parallel connection search preserves net ordering, while its separate partitioned
router processes non-overlapping nets; these are useful distinct models for Flux experiments:
https://docs.verilogtorouting.org/en/latest/vpr/command_line_usage/ . OpenROAD likewise separates
pin access, track assignment, initial routing, repair, and DRC:
https://openroad.readthedocs.io/en/latest/main/src/drt/README.html . Neither source establishes a
speedup for Flux.

First experiment: hold algorithms and routing budgets fixed while exposing bounded regional
decision points and reusable preparation. Measure time to first useful physical diagnosis,
time spent after a repeated no-progress signature, repeated preparation, action duration
distribution, useful/stale speculative results, peak memory, and final legality. Then separately
enable one speculative worker. Compare Tier 0 and Tier 1 under identical configurations and
CPU/GPU budgets; historical runs with different configurations are not causal speedup evidence.

## 2026-10-09 — Bounded regional decisions and preparation reuse

### Failure and hypothesis

The interrupted Tier-1 terrain telemetry run spent more than 100 minutes inside a
single regional action. Numerical kernel yields kept the viewer responsive but did
not let the engine reconsider the algorithm. Earlier sequence analysis also found
no gain from increasing spline worker count. Hypothesis: semantic regional
boundaries, diagnosis-directed dispatch and reuse of immutable geometry can expose
stalls earlier without turning every numerical yield into a new routing attempt.

### Implementation

Regional analysis publishes immutable checkpoints at preparation, conflict/window,
template/spline and evaluated-scene boundaries, with lightweight progress boundaries
inside bounded searches. A managed kernel parks even when its numerical slice is
not exhausted. The engine retains the same frontier while continuing; after a
bounded number of boundaries without improved physical evidence it records the
interrupted attempt and selects a different eligible family. Cancellation joins
that continuation before another action acquires the preparation cache. Unevaluated
snapshots provide diagnoses, never accepted copper.

Common lexicographic quality includes measured connectivity, physical conflicts,
validation and unsmoothed bends; per-stage scalar counts remain diagnostics only.
Typed diagnoses select local XPBD/erosion or upstream topology work. Exploration
follows unsuccessful preferred actions rather than an unconditional expensive
warm-up. Materialization readiness accepts evaluated legal scenes and conflict-free
regional results; final copper validation and external KiCad DRC remain mandatory.

A session-owned, exact-board-bound preparation context retains bounded source,
geometry and navigation caches. One optional CPU worker prewarms at most eight
net/layer source entries alongside initial global routing. Foreground routing only
consumes completed preparation and never waits for speculative completion; a late
job is cancelled and discarded. No speculative copper or congestion history is
committed. Cancellation is cooperative between extraction operations, not a hard
wall-clock bound. Managed regional work uses one spline worker so decision hooks
run on the owning continuation.

### Validation and interpretation

Initial regression tests exposed over-restrictive materialization readiness for
simple conflict-free boards; readiness was corrected before benchmarking. Added
semantic-yield/frontier continuation, decision-budget, optional-worker equivalence,
and evidence/curve-quality checks. Full test and tier results are recorded in the
corresponding benchmark-history entry after the rebuilt run. The stopped old run
is preserved at `build/tier1-terrain-telemetry-20261009T104445Z/` and is not a result
for this implementation.

### Tier-0 control and counter correction

The first optimized control exhausted 192 semantic decisions across six regional
families before a full physical scene was produced. Each action spent about 45 s;
adjacent skipped portfolio entries consumed the same stagnation budget as actual
search. This interrupted the retained frontier just before scene assembly. The
control did verify one speculative preparation was consumed and five subsequent
actions reused preparation. Its binary and telemetry are preserved in
`build/bounded-engine-tiers-20261009T125118Z/`.

Corrected the mechanism: only boundaries with intervening kernel search work count
toward stagnation. All boundaries remain observable and the overall decision limit
still applies. No budget constant was raised. The 64 core regression tests passed
before this small correction; focused rechecks and a fresh optimized tier run follow.

### Explicit search chunks replace numeric work as the stagnation clock

The numeric-work correction recovered Tier-0 planning in 62.49 s, with one regional
attempt and 38 decisions. A subsequent detailed-feedback regional search exposed
why numeric kernel work is still an unsuitable semantic clock: 662 decision points
contained only five numeric-work advances, despite expensive spline candidate work.

The final budget therefore counts **completed spline candidates and bounded lane
search batches** explicitly. Candidate starts and phase announcements have separate
phase types and do not consume the stagnation budget. Removed an observer hook from
a general telemetry publisher so diagnostic publication cannot change scheduling.
The existing engine equivalence test now compares decision counts and copper with
telemetry enabled/disabled, different numerical slices and speculative preparation
enabled/disabled. Budget constants remain unchanged. The intermediate run at
`build/bounded-engine-tiers-20261009T125909Z/` is retained as a numeric-counter control,
not the final benchmark result.

### Final validation and suite launch

Debug and optimized builds passed. The 64-test core regression recheck passed;
after the final typed-boundary correction, all four focused engine tests and the
policy, kernel and preparation targets passed again. The telemetry/speculation/slice
independence assertion passed. Final suite artifacts are in
`build/bounded-engine-tiers-20261009T131008Z/`; its `suite-summary.json` is the live
completion record. Tier 0 and Tier 1 route with 7200-second internal deadlines and
7500-second harness deadlines. The still-locked Tiers 2–4 receive preflight checks.
No tier promotion is claimed from planning success or from the suite merely starting.

## Experiment — 2026-10-09: clearance-constrained PH tightening

### Failure and hypothesis

The viewer showed looping and oscillatory provisional curves. The PH exact-candidate comparator
preferred greater worst clearance before its objective even for already legal routes. Regional
legal-candidate selection preferred fewer control moves/sweeps before length. These mechanisms
can reward bulging away from obstacles and preserve unnecessary interpolation detours. The
baseline Tier-0 detailed run in `build/bounded-engine-tiers-20261009T131008Z` took 502.52 seconds,
reported zero internal copper violations, but was incomplete with seven unsmoothed non-junction
bends. This is not a passing tier or evidence that every provisional curve was legal.

### Research and alternatives

Farouki's [Introduction to PH curves](https://faculty.engineering.ucdavis.edu/farouki/wp-content/uploads/sites/41/2013/02/Introduction-to-PH-curves.pdf)
explains polynomial speed and arc length; Flux already has the exact integral, so chord-sample
length is unnecessary for ranking PH candidates. Elastic-band work by Quinlan and Khatib
([Stanford publication index](https://www-cs-faculty.stanford.edu/group/manips/publications.html))
provides the relevant fixed-topology shortening model. Neither technique solves an incorrect
corridor allocation. Ranked interventions: (1) correct feasible candidate ranking, minimal cost;
(2) bounded anchor deletion inside certified free space, targeted cost; (3) global homotopy search,
more expensive and reserved for conflicts that cannot be removed locally. No external code copied.

### Change and acceptance criteria

All PH proposal families use clearance/shape feasibility before exact arc length; curvature
regularity only breaks feasible length ties. Recovery of illegal inputs remains possible, but
cannot replace a legal input with illegal or longer copper. Tightening removes only interpolation
anchors, is bounded by candidate/removal limits, and obeys route constraints plus continuous
clearance/corridor and shape certificates. Open-route first/last spans are now included in
self-intersection checks. A dedicated engine action runs once per completed curved candidate
before materialization; external KiCad validation remains mandatory.

Success requires certified strict shortening in the synthetic detour, veto of blocked shortcuts,
no growth from excess-clearance rewards, coherent native candidate reconstruction and no
regression in engine sequencing or cancellation. The tier comparison is recorded separately in
`reports/benchmark_history/2026-10-09-ph-tautening.json`; results pending during implementation.

### Verification refinements and obstacle-support audit

Initial full core verification exposed stalled inter-sample collision recovery. The recovery
path needs bounded insertion of an extra interpolation degree of freedom even before that
insertion improves the objective. Restored that behavior only for continuously infeasible inputs,
without worsening shape violations, and retained worst-clearance tie-breaking only among illegal
candidates. The legal-route objective remains exact length then curvature. The inter-sample
rebound and committed-copper retry tests passed after this correction.

Independent review found optional tightening could consume the last action reserved for copper
materialization; scheduling now reserves that action. Cancellation during continuity promotion
now works on a private route copy and retains the caller checkpoint. Focused tests exercise both.
Regional tightening has accepted-shortening, protected-junction veto, and foreign-copper veto
cases, including regenerated manifest/path ownership and a fresh native DRC/connectivity gate.

The user's paired obstacle-support proposal was audited separately: PH recovery inserts one
node at the worst continuous collision, then proposes normal displacements; generic corner
construction has two trimmed guard points, but these are not coordinated entry/exit contacts
around a clearance-expanded obstacle. XPBD subdivision and pairwise route shoves are also not
obstacle contact pairs. Explicit paired-support generation and joint tangent/position optimisation
remain a distinct follow-on experiment; this implementation does not claim to provide it.

### Paired obstacle supports — implementation following the audit

The subsequent user request to fix the missing mechanism is now implemented. A bounded
proposal brackets an obstructed shortcut using signed clearance and the continuous certificate's
collision witness. It inserts approach/departure interpolation points, lifts them together to
clear the obstacle interior, and tries a small coupled contraction portfolio on both sides.
The existing PH reconstruction supplies tangents; this is not a separate exact tangent-graph
solver or a guarantee of a globally shortest route. Full shape, continuous clearance and caller
corridor/topology certificates remain hard acceptance gates. A legal incumbent is replaced only
by a strictly shorter exact PH curve. Failed proposals leave the incumbent intact.

The feature is enabled by default in both infeasible PH recovery and bounded legal post-polish
refinement, including detailed materialization. Regional tightening builds its signed-distance
field from clearance-expanded retained copper, with trace radius, pairwise net rules and board
edges included. Regenerated native copper must pass fresh DRC and actual-scene connectivity
before adoption. Two support points are interpolation stations, not merely the two handles
that every cubic Bézier already has.

Candidate checks are shared with the optimizer's existing budget; selected support pairs are
bounded and failed attempts are counted. Detailed JSON exposes paired attempts, inserted
supports and infeasible-to-legal recoveries separately. Legal shortening is not counted as a
collision recovery. Cancellation retains the last accepted route.

Synthetic circle and box cases shortened from 263.5 µm to 230.2 µm (12.6%) and 217.7 µm
(17.4%), respectively. Their sampled before/after curves were plotted and inspected: both
are smooth single detours without loops. Conservative clearance slack remains. These are
mechanism tests, not board qualification. Tests also cover disabling the mechanism, topology
veto, candidate limits, collision recovery and retained foreign-copper clearance.

The pre-paired length-only Tier-0 control finished in 478.2 s but remained incomplete with
seven unrounded non-junction bends, zero internal copper violations and zero opens. It did
not earn tier promotion. The engine tightening action examined no retained regional PH curves
in this case; some curves are generated later in detailed materialization, which motivated the
bounded legal post-polish integration. Final paired validation is recorded in the structured
benchmark history alongside this control.

### Regression follow-up

The first paired implementation full core run passed 62/64. Fieldless post-polish deletion
changed the interpolation station count and worsened the existing zigzag regularity check;
restricting optimizer obstacle refinement to an actual clearance evaluator restored that test.
The explicit regional tightening API still supports ordinary certified redundant-node deletion.

The other failure was a G2-promotion regression. Disabling paired supports reproduced it
unchanged (1680 versus 1686 candidate checks), so it was not caused by the new obstacle pair.
The shorter optimized G1 station layout failed the bounded scalar-G2 solver. One retry from the
pre-optimization checkpoint restored the G2 micro-test without relaxing curvature tolerance.
That retry must pass the same continuous envelope, shape, topology and native copper transaction
as the first candidate. A speculative vector-derivative solver recovery did not help and was
reverted. A final post-polish callback also allows detailed materialization to veto station
changes that cannot satisfy downstream G2 requirements; it runs only on the final proposal.

### Final paired Tier-0 measurement

The corrected release completed detailed routing in 489.264 s (length-only control: 478.227 s;
runs were concurrent with Tier 1, so this is not an isolated performance comparison). It kept
47 tracks, 3 vias and 199 cubic Béziers, with zero internal copper violations and zero opens.
KiCad 10.0.6 reported zero configured DRC violations and zero unconnected items for the saved
rejected candidate. The curved qualification still failed on the same seven unsmoothed joints.
Paired telemetry recorded 30 attempts and zero inserted supports; engine tightening recorded
one action and zero improvements. Thus the circle/box benefit has not translated to this board.

Both saved copper layers were exported and inspected. They have generally smooth bends without
visible oscillating loops, but retain broad detours and sharp joints. This is not proof of shortest
paths or a passed tier. Preserve the bounded mechanism and focused tests, but investigate corridor
restrictions and unsmoothed materialization failures before attributing any board improvement to
paired supports. Final artifacts: `build/paired-final-tier0-20261009`; structured metrics remain in
`reports/benchmark_history/2026-10-09-ph-tautening.json`.

Tier 1 (`bm_svf`) finished its global stage in 907.216 s with 78 paths, zero unrouted nets
and zero ordinary resource overflow, but 398 clearance-shadow conflicts. The engine timed out with `routing_plan_complete=false`, so the benchmark skipped detailed
materialization. Shadows were advisory with zero shadow-history penalty, not a direct count
threshold in the benchmark gate; the unlocked
probe's successful harness exit does not mean the tier passed. Provisional imagery was inspected
and remains predominantly unsmoothed routing geometry. No final Tier-1 copper/DRC claim is made.
Maximum observed router RSS during the sampled interval was 863312 KiB for Tier 0 and 799740 KiB
for Tier 1. Both benchmark processes finished; the normal release CLI was rebuilt successfully.

## Experiment — Actual-geometry clearance shadows (2026-10-09)

### Failure and hypothesis

The Tier-1 run reported 398 tile-coalesced shadow witnesses from 724 observations. Source tracing
found that crossing-safe capacity canonicalization was also used for geometric certificates.
Rasterizing an unselected opposite diagonal could overwrite the selected diagonal's true
clearance. In a 650 µm cell, a selected diagonal is 459.619 µm from a vertical owner, with a
450 µm required centerline distance, yet the aliased shadow reports a 450 µm deficit.
Separately, candidate qualification used nominal track width instead of selected neck-down width.
These are representation errors, not evidence that fabrication clearance should be reduced.

### Change

Keep shared diagonal identities for capacity, but preserve actual chord identities for clearance
rasterization, selected occupancy, search, conflict attribution, history and incremental rebuilds.
Qualify shadows with selected segment widths; junctions use the widest incident track/via.
Filter local planar escape pressure by the actual clearance threshold rather than counting every
broad-phase certificate (which also covers wider via candidates). Label the viewer metric
“Shadow witnesses” and explain that it is not a KiCad DRC count.

### Verification

Add one-cell diagonal and selected-width micro-tests with true-collision controls, plus frozen
checkpoint rebuilds. Preserve exact copper and benchmark qualification gates. Compare
Tier-0 and Tier-1 against the saved pre-change executable and structured baseline in
`reports/benchmark_history/2026-10-09-clearance-shadow-accuracy.json`.

Review also identified a preexisting false negative: multi-cell direct Voronoi chords looked up
full-edge keys absent from the one-pitch raster. Retain exact owner segments and lazily cache
exact same-layer segment distances for direct edges, with conservative bounding-box rejection.
Adding or removing copper invalidates this cache. Four focused regression cases pass, including
a retained two-point direct chord with a midpoint crossing and a legal offset control. Independent
review found no further actionable issue. Full regression and tier comparison runs are pending.

A viewer audit confirmed the former filled circle radius represented deficit magnitude clamped
to 7–34 pixels, not a physical envelope. The overlay now uses fixed-size hollow diamond witness
markers and explicitly labels their meaning. Telemetry does not carry enough geometry to draw
exact physical shadows; the display no longer suggests otherwise.

The full debug suite exposed a race missed by the initial review: static seed workers share
const empty occupancy, but the lazy cache was mutable and unguarded. A debugger reproduced
a double free in `build_static_hybrid_seed_plan` during
`global_to_detailed_route_is_transactional_and_capacity_gated`. The first tier comparisons were
interrupted and are not final evidence. Synchronize cache publication and preserve safe occupancy
copy/move semantics; rerun the reproducer and full tests before restarting tier comparisons.

The corrected cache uses value ownership with locked copy/move and locked construction/lookup;
completed entries remain immutable between serial copper updates. Independent concurrency review
found no remaining actionable issue. The previously crashing transactional test passes twice in
Debug; focused shadow/kernel/engine-policy tests pass; the complete optimized core suite passes
64/64. Release CLI and viewer build successfully. Corrected 900-second tier comparisons use
`build/shadow-verified-tier0-20261010` and `build/shadow-verified-tier1-20261010`.

Corrected Tier 0: global 65.059 s, detail 519.538 s (prior detail 489.264 s; concurrent
runs prevent isolated timing attribution). The same seven non-junction joints remain unsmoothed,
so curved qualification still fails. Output has 47 straight tracks, 199 added cubic Béziers and
3 vias; internal copper violations, open connections and configured KiCad DRC violations remain
zero. Both exported copper PNGs are byte-identical to the prior paired-support run and were
inspected. Thus this accuracy fix preserves that baseline copper, including its unresolved
sharp joints and detours; it does not promote Tier 0 under curved qualification.

Corrected Tier 1 timed out at 900.209 s: 78 paths, zero unrouted nets and ordinary overflow,
but `routing_plan_complete=false`. Shadow witnesses fell from 398 to 275 (raw observations
724 to 497; coalesced observations 326 to 222). These remain advisory, with zero shadow history
increment. The internal provisional quantized-copper oracle improved from 417 to 310 diagnoses
and 1114 to 799 internal violations, with tracks 1429 to 1329 and vias 76 to 56, but remains
infeasible. Detailed materialization was skipped; there is no final Tier-1 KiCad DRC result.
Provisional imagery was inspected and still contains coarse unsmoothed geometry and crossings.

Sampled peak RSS was 880476 KiB for Tier 0 and 936492 KiB for Tier 1. These runs overlapped;
**do not use their durations as performance evidence**. Following the user instruction on
2026-10-10, all future tier runs must be sequential with recorded telemetry; this is saved in
AGENTS.md. Both current runs recorded diagnostic telemetry and have finished.

Decision: keep the geometry/width accuracy fix and synchronized exact direct-edge cache.
Regression evidence supports the intended mechanism, while Tier-1 physical feasibility remains
unsolved. Investigate the remaining regional/oracle conflicts next; do not interpret the smaller
planning witness count as final copper legality.

## Experiment — Vulkan LittlePlanets height evolution (2026-10-10)

### Hypothesis and scope

The LittlePlanets-style terrain branch currently runs its entire height-evolution loop on the
CPU, even when the rest of discovery uses Vulkan. Move downhill selection, contributing-area
accumulation, capacity, conservative divergence and bounded height updates onto the GPU. Preserve
CPU input validation, deterministic masked path extraction and downstream exact copper gates.
Feedback fields and the walkable mask are uploaded for each solve so laid/ripped-up copper cannot
reuse stale terrain. Retain the CPU implementation as an explicit numerical oracle, not a silent
fallback. Measure field evolution separately from extraction and end-to-end solve time.

### Design and research

Each cell has at most one strictly lower downstream neighbour, forming a forest. Accumulate
integer contributing area by pointer doubling. With downstream operator P, successive passes
apply `(I + P)`, `(I + P^2)`, `(I + P^4)`, etc. to unit cell contributions. Their product includes
every path distance once; integer atomic scatter preserves exact counts on branching forests.
This avoids sorting heights on the CPU or dispatching once per downhill depth. A seeded mechanical
check matched serial accumulation for 399 branching forests. Separate initialization/scatter
passes prevent writes from racing with inputs. Gather divergence from the original eight adjacent
upstream cells, preserving mask boundaries and conservative outgoing/incoming transport.

Vulkan synchronization follows the Khronos compute-to-compute and compute-to-host examples:
https://docs.vulkan.org/guide/latest/synchronization_examples.html
Integer atomic operations follow the GLSL built-in definition:
https://github.khronos.org/Vulkan-Site/glsl/latest/chapters/builtinfunctions.html
Persistent, capacity-grown buffers and independent input/output state remain resident across
iterations, with bounded iteration submissions for cancellation. CPU extraction still permits
uphill/saddle crossings; terrain never changes legal free-space topology.

### Verification plan

Check numerical fields against the CPU oracle, exact integer flow on long/branching forests,
no diagonal corner cutting, conservation, feedback replacement, mask replacement, cancellation,
concurrent calls and buffer reuse. Measure representative raster sizes on the actual Vulkan
device. Run core regressions, then Tier 0 followed by Tier 1, sequentially with diagnostic telemetry.

### Implementation and measured cost

Implemented Vulkan compute with device-local working fields, persistent capacity-grown staging
buffers, and coherent cached readback where supported. An initial host-visible working-buffer
layout was slower than the CPU on large fields and was replaced rather than retained. The
production height-evolution branch now selects GPU execution by default; the CPU remains an
explicit reference. Field evolution and CPU extraction have separate telemetry, alongside actual
dispatch counts and buffer growth/reuse. Cancellation is checked between iterations and while
waiting for the shared GPU context.

Three sequential warm-cache measurements on an NVIDIA GeForce RTX 5080, using 16 iterations:

| Raster | CPU total | GPU total | Total speedup | CPU evolution | GPU evolution |
| --- | ---: | ---: | ---: | ---: | ---: |
| 64² | 5.155 ms | 1.665 ms | 3.10× | 4.780 ms | 1.345 ms |
| 128² | 22.328 ms | 3.237 ms | 6.90× | 20.896 ms | 1.813 ms |
| 256² | 100.459 ms | 8.709 ms | 11.54× | 94.195 ms | 2.462 ms |
| 512² | 484.481 ms | 34.478 ms | 14.05× | 456.745 ms | 4.927 ms |

These are median terrain-solve timings, not whole-board routing speedups. At 512², unchanged CPU
path extraction now dominates elapsed time. The four focused terrain suites and CLI terrain flag
check pass. CPU/GPU field comparisons use FP32 tolerance; integer contributing areas remain exact.
An accidentally broad parallel CTest invocation started overlapping corpus harnesses and was
terminated. Its corpus outputs are excluded from validation and performance comparisons. Final
core and sequential telemetry-enabled tier results are recorded below when completed.

### Regression results

All 64 core cases, four focused terrain suites, and CLI flag checks pass. The regular Release CLI
and viewer builds succeed. Sequential Tier 0 (`build/terrain-gpu-tier0-20261010`) completed planning
in 64.865 s and detail in 485.340 s. All 46 invoked height evolutions ran on GPU (15,180 dispatches;
105,564 µs field time and 4,599 µs extraction in planning). Final copper retains 47 straight tracks,
199 Béziers and 3 vias, zero opens, zero internal violations and zero configured KiCad DRC violations.
Both copper layer PNGs are byte-identical to the prior shadow-accuracy baseline and were inspected.
The same seven unsmoothed non-junction joints still prevent curved qualification; this is preserved
baseline behavior, not a solved tier. Sampled peak detail RSS was 960,296 KiB. Previous baseline tier
runs overlapped, so the duration difference cannot establish a whole-board speedup.

### Follow-up: regional GPU/parallel opportunity

User asked whether regional routing could also benefit from GPU/parallel execution. Existing
`realize_bundle_splines()` already runs up to four independent window workers, and PH refinement
uses a bounded exact-candidate pool. Tier 0 detail reports 382 GPU polish batches/15,876 candidates.
The 135 detailed regional spline timing events sum to 52.058 task-seconds: trajectory polish 14.819,
joint shove 13.648, interpolation 8.987, setup 6.131, native DRC 3.271, adaptive pair 3.051,
quantization 1.326, post-polish 0.352 and XPBD 0.100. These sums are not wall-time fractions or a
complete explanation of detail runtime.

More decisive planning evidence: channel construction (`populate_channels`) consumed 44.309 s
of the 64.865 s planning run. First partition that timing into extraction, preparation and geometry
predicates; reuse/index prepared geometry before adding more compute. Then parallelize conservative
intersection screening or batch it on GPU, with deterministic CPU merging and exact validation.
A hypothetical 4x acceleration of that stage alone predicts about 31.633 s planning; this is an
Amdahl estimate, not a measured result or a whole-board claim. Next consider bounded independent
candidate evaluation within a single window, with immutable snapshots and ordered selection.
Increasing window workers alone does little for one dominant window. Shared preparation leases,
cache ownership, nested worker budgets and generation-bound commit checks need explicit handling.
Do not move branching assignment search, history mutation or native copper publication wholesale
to the GPU. No regional implementation changes were made for this analysis.

Tier 1 GPU-only baseline (`build/terrain-gpu-tier1-20261010`) then ran alone with diagnostic
telemetry and timed out at 900.597 s. It retains 78 paths, zero unrouted nets, 275 shadow witnesses,
and the same infeasible internal copper oracle as the prior run: 310 diagnoses, 799 violations
(70 hole-clearance, 544 track-clearance, 179 track/via-clearance, 6 via-clearance), 1329 tracks,
56 vias. No final detailed output/KiCad DRC is available. All 45 invoked evolutions used GPU
(15,750 dispatches, 178,755 µs field time, 9,778 µs extraction). Peak sampled RSS: 861,028 KiB.
Decision: retain GPU evolution; numerical tests and both tiers preserve baseline behavior. This
accelerates the terrain mechanism but does not solve Tier-1 physical feasibility. The tested binary
was saved as `build/terrain-gpu-20261010/flux-cli-regional-baseline` before regional acceleration.

## Experiment — Bounded regional acceleration (2026-10-10)

### Failure and hypothesis

User requested spatial/preparation reuse, bounded parallel regional alternatives, conservative
GPU batches and one shared CPU worker budget in a single integration. Tier-0 GPU-only planning
spent 44.309/64.865 seconds constructing channels. Inspection found pairwise channel overlap
rebuilding the same PreparedPolygon BVHs for every pair. This is avoidable computation before
routing search. A prepared-cell spatial index should remove most of that cost without changing
first-match channel merging. Independent candidate realization can then use bounded private
snapshots, leaving ordered selection, fallback promotion and exact scene publication on the CPU.

### Change and verification plan

Reuse prepared channel polygons behind a bounded spatial index; retain exact predicates and
original channel order. Reuse board-bound source extraction/chord validation, with a source BVH
for local obstacle extraction and stable source ordering. Existing bounded navigation caches
remain invalidated by complete board equality, including copper changes. Bound GPU chord work
per dispatch and yield between chunks outside the shared device lock. Share CPU leases across
static seed, regional/window/candidate, exact PH and detailed materialization pools. Limit snapshot
concurrency using an explicit estimated scratch budget. Candidate workers return raw realizations;
the parent accounts actual work, discards excess speculation, performs stable reduction and
publishes telemetry/decisions. Validate index parity, CPU budget exhaustion, conservative GPU
chunks, serial/parallel equivalence and cancellation before sequential telemetry Tier 0/Tier 1.

Indexed channel overlap survivors also use bounded CPU batches when at least eight channel
candidates remain; workers only read prepared geometry and write private result slots. The parent
chooses the first exact overlap in original channel order before mutating channel state. Small
queries remain serial. All such workers share the same process-wide budget.

### Local verification

All 65 core cases and 11 focused suites pass. Candidate parity exercises real concurrent shortlist
entries and checks identical selected geometry and physical outcome. Channel parity constructs eight
eligible prior channels and a broad incoming corridor; batched evaluation performs additional exact
checks but preserves channel topology and occupants. Tests also cover copper cache invalidation,
worker-budget exhaustion, conservative multi-chunk GPU screening, and managed suspension/cancellation.
Independent review found no remaining material issues. A serial single-worker overlap fallback now
releases its unused worker lease. The initial candidate test used an invalid zero fallback-wave
configuration; that fixture error was corrected before accepting the result.

### Tier-0 result

Sequential diagnostic-telemetry run: planning 23.794 s (baseline 64.865), channel construction
0.141 s (44.309), detail 418.485 s (485.340). Planning plus detail is 442.279 s vs 550.205 s,
19.6% less elapsed time in this single comparison. Preparation indexed 53,532 cells, performed
567 queries and 577 exact channel comparisons. Planning exercised 4 candidate batches/8 tasks
with 2 workers; one speculative candidate was discarded by the ordered legal-portfolio limit.
Copper stays at 47 tracks, 199 Béziers, 3 vias, zero opens/internal violations/configured KiCad DRC
violations. Both layer PNGs are byte-identical to baseline and visually inspected. The same seven
unsmoothed non-junction joints prevent curved qualification; harness exit 1 is this existing failure,
not a new physical regression. Peak process RSS was 1,169,872 KiB vs baseline sampled 960,296 KiB
(about 22% higher). Retain the speed improvement but expose this memory cost; this is one sequential
run per configuration, not an isolated attribution of every acceleration component.

### Tier-1 result and decision

Tier 1 ran only after Tier 0 finished, with diagnostic telemetry and the same 900-second engine
budget. It timed out at 900.433 s with unchanged feasibility: 78 paths/25 nets, zero unrouted nets,
275 shadow witnesses, and the internal quantized oracle still reports 799 violations (544 track
clearance, 179 track/via clearance, 70 hole clearance, 6 via clearance), 310 diagnoses, 1329 tracks,
56 vias and zero opens. No qualified detailed output or final KiCad DRC exists. Both provisional
layer images were inspected and retain coarse crossings. Peak RSS 897,888 KiB is about 4.3% above
the prior sampled 861,028 KiB. Parallel candidate completion is visible in telemetry; zero final
regional counters reflect cancellation of an uncommitted action, not absence of parallel work.

Retain the implementation: it preserves the tested physical outcomes, substantially removes channel
preparation overhead, and reduces Tier-0 planning+detail elapsed time in this single comparison.
It does not solve the Tier-1 physical frontier. Summed detail candidate phase durations rose from
52.058 to 191.815 seconds; these per-worker phase durations include changed scheduling/speculation
and are neither CPU-time measurements nor elapsed wall time. Avoid claiming isolated candidate/GPU
speedups without ablation. Memory costs are explicit in the structured report:
`reports/benchmark_history/2026-10-10-regional-acceleration.json`.

## Research — tscircuit routing and full autorouter (2026-10-10)

### Scope and evidence

Inspected the user-linked `tscircuit/routing` at
`bc7419513fe21e5501aae732de48dfb12b9a5f8d` (HEAD commit dated 2024-07-19), then the
separate, more relevant `tscircuit/tscircuit-autorouter` at
`a9982c678cc8f130b2d87c3f6e3f126c20386170`. Read source only; no external package scripts,
services, benchmarks or code imports were run. This is a design review, not a measured improvement.

The original library routes 2D points around axis-aligned rectangles. Its granular A* rounds
obstacle occupancy, outputs a fixed width, and has no net/layer/via/physical-rule model. `findRoute`
sorts terminals by distance from the origin and chains adjacent pairs; it is not an improvement on
Flux's existing growing route trees. Mixed-granularity routing attempts pitch multipliers and
repairs endpoint links, but the implementation immediately returns the multiple=1 result even on
failure when that level is eligible. These routines must not replace Flux's exact legality gates.

### Ranked adaptations

1. **Progress-earned regional work allowances — high expected value, medium cost.**
   Modern `applyPipeline9BoundedRegionalRepairs.ts` limits regions, candidate attempts and expanded
   search nodes separately; accepted repairs unlock more work under a hard total ceiling.
   `Pipeline9BoundedRegionalRepairSolver` maintains attempted regions and explicit validation phases.
   Flux already has bounded kernels and no-improvement policy; the experiment is to put work credit
   at individual region/candidate scope and count actual child work, not merely parent decisions.
   Preserve connectivity, hard-rule categories and failure attribution; raw DRC-count reduction
   alone is insufficient. Measure time to first accepted physical improvement, work per improvement,
   retained conflict sets and timeout behavior on Tier 1, with Tier-0 copper as regression.

2. **Guarded partial repair publication — high potential quality value, medium/high cost.**
   Modern `selectIndependentClearanceRepairs.ts` validates simultaneous proposed moves, restores
   blocked vertices, rebuilds the obstacle index and rechecks surviving moves. This matters because
   restoring one neighbor can invalidate a previously safe move. Its removal phase is bounded by
   moved-vertex count, but repeated scene rebuilding can be expensive. Flux already has exact
   transactions; apply the concept inside one failed transaction to salvage an independent subset.
   PH junction/tangent dependencies must form atomic groups; exact whole-scene DRC/connectivity and
   unchanged-boundary checks remain mandatory. Minimum test: two mutually dependent bends plus a
   third blocked proposal; retain the valid pair without accepting a new conflict after rollback.

3. **Conflict-aware candidate scheduling and cheap infeasibility rejection — high potential
   runtime value, medium cost.** Modern `PortfolioSingleIntraNodeSolver` accounts actual work,
   discounts provisional routes with conflicts, expands route-order diversity only after initial
   work, and can reject fixed unrelated terminals that overlap before trying more orderings.
   Flux already scores physical quality; move comparable evidence into live candidate scheduling.
   Use width+clearance/via envelopes rather than their simple equal-width point test. A rejection
   must invalidate only the fixed terminal/layer assignment, not a region whose topology can change.
   Keep exploration allowance so greedy progress does not starve a necessary temporary detour.

4. **Exit-class shortlist generation — medium/high potential, medium cost.** The original
   `findBestBorderTargetPaths.ts` groups reachable border samples and retains representatives;
   `findTwoPointNearBiasRoute.ts` searches endpoint-to-exit and exit-to-exit pieces separately.
   Flux already has terminal portals, adaptive NavMesh and route-class guards. Incremental value is
   deduplicating expensive regional candidates before PH by certified reachable exit/corridor class,
   retaining several representatives when capacity, width, layers or trace order differ. Their
   contiguous-border heuristic is not a homotopy proof. Unbounded recursive Cartesian products
   would reproduce Flux's portfolio cost. Use a lazy bounded pair shortlist and stable selection.

5. **Local outcome caching and reproducible subproblem captures — medium potential, medium cost.**
   Modern `CachedIntraNodeRouteSolver` caches successes and failures using geometry/rules/connection
   metadata; original `logging/log-context.ts` and debug-grid serialization capture nested work.
   Flux already reuses geometry and certificates. Candidate outcome reuse needs exact local copper
   fingerprints, rules, protected junctions, algorithm version, search settings and failure kind.
   Do not treat a budget timeout as proof of infeasibility or use their rounded-coordinate cache
   identity as a Flux legality certificate. Stream bounded parent/child work events on interruption
   so final zero counters no longer hide unfinished regional work.

### Sources

- Original exit grouping: https://github.com/tscircuit/routing/blob/bc7419513fe21e5501aae732de48dfb12b9a5f8d/src/lib/find-best-border-target-paths.ts
- Original recursive search: https://github.com/tscircuit/routing/blob/bc7419513fe21e5501aae732de48dfb12b9a5f8d/src/lib/find-two-point-near-bias-route.ts
- Original representation: https://github.com/tscircuit/routing/blob/bc7419513fe21e5501aae732de48dfb12b9a5f8d/src/lib/types.ts
- Work allowances: https://github.com/tscircuit/tscircuit-autorouter/blob/a9982c678cc8f130b2d87c3f6e3f126c20386170/lib/autorouter-pipelines/AutoroutingPipeline9_PreloadedTraceGraph/applyPipeline9BoundedRegionalRepairs.ts
- Partial repair selection: https://github.com/tscircuit/tscircuit-autorouter/blob/a9982c678cc8f130b2d87c3f6e3f126c20386170/lib/autorouter-pipelines/AutoroutingPipeline9_PreloadedTraceGraph/selectIndependentClearanceRepairs.ts
- Candidate scheduling: https://github.com/tscircuit/tscircuit-autorouter/blob/a9982c678cc8f130b2d87c3f6e3f126c20386170/lib/solvers/HyperHighDensitySolver/PortfolioSingleIntraNodeSolver.ts
- Outcome cache: https://github.com/tscircuit/tscircuit-autorouter/blob/a9982c678cc8f130b2d87c3f6e3f126c20386170/lib/solvers/HighDensitySolver/CachedIntraNodeRouteSolver.ts

### Decision

Prioritize a bounded experiment combining cheap fixed-assignment feasibility rejection with
progress-earned candidate work; evaluate partial-repair salvage separately. Do not replace the
router, weaken clearances, copy rectangular raster legality, or port origin-sorted net chaining.
No routing implementation changed in this review. Original package metadata declares ISC but the
inspected tree has no standalone license file; modern repository includes an MIT license. No source
was copied. Dependency licensing and attribution would need review for any future literal port.

## Experiment — Progress-earned regional search and guarded repair subsets (2026-10-10)

### Failure and hypothesis

Regional-acceleration baseline Tier 1 (`bm_svf`) spent 900.433 s and timed out despite
78 provisional paths covering 25 nets. The retained quantized oracle still reported 799
violations (544 track clearance, 179 track/via, 70 hole clearance, 6 via clearance).
The interrupted action's zero aggregate counters were not evidence of zero work.
Tier 0 had legal copper and zero opens/configured DRC, but still failed curved qualification
with seven unsmoothed non-junction joints; planning/detail elapsed were 23.794/418.485 s.
Baseline: `reports/benchmark_history/2026-10-10-regional-acceleration.json`.

Hypothesis: spending more time on assignments without measured physical progress is less useful
than returning control to the state engine; impossible fixed assignments should be rejected before
PH fitting, and independent valid moves in a failed XPBD proposal should survive rollback.
The user requested these mechanisms and conflict-aware candidate scheduling together. This is one
integration pass, with separate focused mechanism tests and counters; benchmark causality cannot
be assigned to an individual mechanism from this combined experiment alone.

### Implementation

- Session-wide regional child-work allowance with a hard ceiling; only strict improvements in
  evaluated physical evidence earn additional credit. Kernel work, not telemetry announcements,
  consumes allowance. Existing coherent decision points remain the cancellation boundaries.
- Conservative fixed-assignment preflight and deterministic conflict-aware portfolio ordering.
  Movable centreline crossings remain search problems rather than infeasibility certificates.
- After a rejected XPBD proposal, preserve net-atomic interior-control moves in a hybrid scene.
  Differential pairs/buses remain unchanged in this subset fallback. Failed reconstruction and
  exact native-copper conflicts remove groups; every restoration forces reassessment of survivors.
  At most eight subset assessments are allowed. Cached baseline/proposal fitted pieces are reused.
  Strict adaptive improvement, non-regressing topology/continuous clearance, native violation and
  fit-failure bounds, and legal changed routes are required. Final scene/connectivity/KiCad gates
  remain unchanged. This local salvage does not publish an incomplete board as routed.
- CLI and session output include earned/consumed/allowed work and subset assessment/retention counts.

### Validation

Integrated builds, focused regressions, independent review, and sequential telemetry runs
of Tier 0 then Tier 1 are recorded below. Legality and curved qualification are reported separately.


### User-requested extension — AVBD contact blocks

The same implementation pass now also includes the requested AVBD cord/rigid-contact adaptation.
References inspected locally: Fidgets `src/avbd_cord.cpp`, `docs/gpu-rigid-contacts.md`,
`docs/gpu-surface-contact-reduction.md`, and `docs/gpu-mpm-contact-relaxation.md` in
`/home/adam/qtbits/pipe_dreams/fidgets`. These are mathematical/design references, not imported code.
PCB vias are disks, so their rigid block needs shared planar translation, not a six-DOF body pose.

The bounded CPU experiment adds per-owner 2D local block solves, persistent within-solve dual
multipliers, capped adaptive penalties, deterministic contact reduction, owner adjacency and
color barriers. Contacts are reevaluated from current geometry. A common contact weight based
on maximum movable-owner degree under-relaxes crowded contacts; fixed supports have degree zero.
This borrows the simultaneous-response stabilization idea, not Fidgets' dynamic energy guarantee:
a quasistatic routing solve has different objectives and still requires exact copper validation.

AVBD remains a regional geometry proposal, behind the existing PH reconstruction, topology,
quantization, connectivity and exact DRC checks. New via motion must keep all attached endpoint
copies coherent; frozen/out-of-region/protected attachments prohibit motion. Failure or capacity
exhaustion must preserve the input proposal, never silently drop contacts. GPU AVBD kernels are
not part of this CPU block-solver experiment; existing GPU XPBD and terrain stages remain usable.

The four original mechanisms passed 65 core cases and 15 focused suites before this extension.
Final integrated validation and sequential tier evidence are recorded below.

The first AVBD behavioral test exposed nonlinear overshoot: crowded-contact total residual
started at 55 nm, improved to 48.286 nm at iteration five, then ended at 144.081 nm. The initial
implementation therefore failed the improvement test despite producing finite geometry.
Owner updates now use up to 12 augmented-energy backtracking steps, and the solver returns its
best complete physical-residual iterate. This preserves progress across non-monotonic dual
updates. Rigid endpoint gradients are summed before forming each block Hessian; fixed members
pin their entire group, and a common step bound respects every member's corridor. Typed exact
contact identities avoid hash-collision suppression, and all candidate visits have explicit caps.

### Integrated validation and pilot Tier 0 result

All 16 focused suites and all 65 core cases passed. Release and viewer builds passed, including
the viewer configuration's warnings-as-errors build. Independent review found no remaining
material blocker after the contact, via-span, and solver-stability corrections.

Tier 0 ran first with diagnostic telemetry and the existing 900 s routing allowance. Planning
took 23.757 s; detailed routing took 314.862 s, compared with the previous 418.485 s. Both final
copper layer PNGs were inspected and are byte-identical to the baseline: 47 straight tracks,
199 Beziers, three vias, zero opens, zero internal copper violations, and zero configured KiCad
DRC violations/unconnected items. The same seven unsmoothed non-junction bends still fail strict
curved qualification; the harness's generic regression wording describes this existing failure.
Peak harness/child RSS was 728,840 KiB.

The regional allowance exhausted once at a coherent boundary (1,343,897 work units consumed
against 1,000,000 initially allowed, zero credit earned). A child can finish past its allowance
before the next decision point. The 24.8% lower detail time is a combined-feature, single-run
observation; it does not demonstrate AVBD-specific speedup or improved copper geometry.
The subsequent Tier 1 pilot exposed a larger enforcement defect: alternative preparation
consumed 28,170,662 work units against the initial 1,000,000 allowance while still awaiting a
high-level regional decision. The run was interrupted without claiming a final routing result.
Ordinary kernel yields already park every mutable child worker, so exhaustion now creates a
decision at that safe checkpoint, discards the unfinished private transaction and retains the
last evaluated scene. The next kernel slice is also capped to the remaining allowance. A focused
engine regression requires every exhausted yielded step to expose a decision point. Fresh builds,
validation and sequential Tier 0/Tier 1 runs follow this correction.

Independent review of the parked-kernel correction found no production blocker: work is charged
before the decision, all mutable child work is parked before inspection, and cancellation joins
the kernel before destroying its callback targets. The regression also requires an interrupted
regional attempt and exactly one budget-exhaustion event, rather than accepting natural task
completion as evidence. A dedicated injected better-scene/worse-scene retention fixture remains
a possible coverage improvement; the production restoration path retains the existing best-plan
selection and exact acceptance gates.

### Final validation after the parked-kernel correction

The rebuilt 16 focused suites and all 65 core cases passed. Release CLI and viewer CLI/viewer
builds passed. Tier 0 then ran with diagnostic telemetry, followed sequentially by Tier 1.
Tier 0 planning took 23.967 s and detail took 309.873 s (26.0% below the previous 418.485 s
baseline, one combined-feature observation). Final output remains 47 straight tracks, 199 Beziers
and three vias; both inspected copper PNGs are byte-identical to baseline. There are zero opens,
zero internal copper violations, zero configured KiCad DRC violations/unconnected items, and
the same seven unsmoothed joints. Peak RSS was 705,544 KiB. Regional search stopped at exactly
1,000,000 work units, with zero earned credit and one exhaustion. Strict curve qualification
therefore remains failed without a new physical regression.

The fresh Tier 1 telemetry likewise stopped the first regional action at exactly 1,000,000 work
units and selected USPED then feedback-global; it did not repeat the pilot's 28-million-unit
preparation overrun. Final outcome is recorded below.

Tier 1 finished in 396.107 s, versus the previous 900.433 s timeout, at 759,088 KiB peak RSS.
Its expected-failure harness probe returned zero, but explicitly qualified 0/1 boards. All 78
abstract paths remain connected with zero unrouted nets; physical quality did not improve:
275 clearance-shadow conflicts and 799 quantized-copper oracle violations (544 track clearance,
179 track/via clearance, 70 hole clearance, six via clearance). The engine exhausted its action
choices, with exactly 1,000,000 regional work consumed and zero credit. No detailed final copper
or external KiCad DRC result was produced. The 799 count is the internal quantized oracle, not
an external KiCad result. Both non-authoritative preview layers were inspected: angular coarse
polylines and unresolved crossings remain, so there is no evidence of smooth legalized routing.
All retained-plan AVBD counters are zero; the early preparation stop prevents this run from
establishing a real-board AVBD benefit. Local AVBD behavior is covered by the focused tests.

Decision: keep the requested bounded mechanisms and tested CPU AVBD proposal stage. Tier 0
physical copper is unchanged; excess preparation is stopped and Tier 1 reaches a bounded result
sooner, but neither strict Tier 0 curve qualification nor Tier 1 physical routing is solved.
Next hypothesis: preparation consumes the seed allowance before a small physical scene can be
evaluated. Incremental, reusable preparation that yields an evaluable small conflict region
should be investigated before granting more budget or attributing benefit to geometry solvers.
Evidence: `reports/benchmark_history/2026-10-10-progress-policy-avbd.json`.

### Follow-up Tier 1 sequence analysis

The 14 engine attempts used 89.642 s for the first global/feedback seed, 3.898 s for regional
preparation, 301.395 s for later USPED/global retries, and 1.035 s for five quantized probes.
Telemetry confirms regional preparation reached only 9/384 ordinary portfolio checks, with no
evaluated regional scene and no AVBD invocation. The stronger budget guard is correct, but its
fixed seed allowance is consumed before the pipeline can demonstrate local physical progress.
Later physical probes measured 799, 1234, 906, 1092 and 1143 violations. None beat the initial
physical incumbent, so no regional credit was earned. About 76% of total runtime went to the
later global/USPED retries, which do not share that progress-earned allowance.

Resource overflow zero did not imply physical capacity: clearance shadows were advisory and
275 shadow conflicts remained. The retained result contains 310 quantization diagnoses, 215
on B.Cu and 95 on F.Cu. One concrete two-net micro-case has V_BF and Net-(U4B--) sharing the
B.Cu endpoint (47.64, 86.79) mm with a 0.45 mm clearance deficit. This is an overlapping global
path pair before curve fitting, not evidence of a smoothing or AVBD failure. Diagnostic counts
are overlapping evidence records, not independent collision counts.

Recommended next experiment: make preparation resumable and reusable, prepare one bounded
conflict region and evaluate it before enumerating the broad portfolio. Bound setup separately
from improvement-earned repair and shrink the selected region if necessary. Apply physical
progress budgeting to global/USPED retries as well. Compare XPBD and AVBD only after the same
small physical scene reaches both solvers. Improve telemetry naming so recording an oracle
result cannot be mistaken for accepting a better physical route.

## Experiment — Incremental physical feedback and bounded preparation (2026-10-10)

Failure: the bounded Tier-1 run spent 3.898 s on 9/384 portfolio checks, reached no regional
scene or AVBD invocation, then spent 301.395 s on global/USPED retries whose physical probes
all worsened the 799-violation incumbent. The preparation and repair budget were conflated.

Hypothesis: small resumable preparation batches followed by exact physical evaluation allow
local solvers to earn repair work, while separate finite preparation and global retry budgets
prevent broad topology enumeration from consuming the remaining deadline without benefit.
Changed physical inputs must re-enable relevant actions without resetting session budgets.

Implementation: a finite 8M preparation reserve, separate 1M initial earned-repair
budget (30M ceiling), and two initial global retry tokens after physical evidence exists, with
one token per strict physical improvement up to eight. Source/context-sensitive action history
is reconsidered after geometry/conflict/history changes. Worse probed candidates restore the
physical incumbent while retaining transferable failure knowledge. Telemetry separates recording
physical evidence from improving it. Every enabled curved regional action can use XPBD followed by AVBD, including the first
bounded physical prefix and obstacle-only scenes. Exact PH reconstruction and whole-scene
connectivity/DRC remain acceptance gates.

Completed navigation requests (including completed negative results) are memoized with exact
source, endpoint and configuration keys. Live kernels resume in place; interrupted transactions
reuse completed requests but regenerate unfinished searches and cheap enumeration bookkeeping.
The prefix uses at most eight alternative-template checks and eight interpolation nodes per route.
The preparation reserve is never refilled. Two additional non-renewable changed-context retry
opportunities permit relevant actions after genuinely changed physical evidence; A/B/A history
cannot replay the same failed action/context.

Review corrections cover cancellation before broad continuation, whole-native opens/conflicts
in checkpoint ranking, native assessment of merged repairs, stricter plan-only acceptance, and
retention of the best physical scene after interrupted regional work. Prefix limits clamp to
existing user limits instead of rejecting smaller valid configurations.

Memory limitation: the 512-entry/64 MiB bound is for completed route-result payloads. Prepared
geometry/navigation remain bounded by existing entry and per-domain cell limits, not by a deep
aggregate byte limit; shallow cache telemetry is not RSS. A deep ownership estimator and an
aggregate domain-memory cap remain separate follow-up work.

Validation in progress: all 65 core cases verified (63 in the final full run and two updated
contract fixtures passed individually). The new prefix-order/cancellation/whole-scene regression
passes. The attempted black-box completed-navigation cache-hit fixture was invalid: run-up
rejection prevented any navigation request. Its cache-hit assertions were removed rather than
claim coverage; direct request-cache hit/config-invalidation coverage remains a limitation.
Existing prepared-geometry reuse/invalidation tests and independent key/lifetime review passed.
Release/viewer builds and sequential telemetry tiers follow.


### First benchmark and prefix-scope correction

Tier 0 preserved byte-identical final copper (47 tracks, 199 Beziers, three vias), zero opens,
zero internal/KiCad DRC violations and the same seven unsmoothed joints. Planning took 31.828 s
and detail 370.264 s (baseline 23.967/309.873 s); peak RSS 769,232 KiB. The prefix reached a legal
whole native scene at 151,398 preparation units and invoked both XPBD and AVBD eight times.

Tier 1 finished in 284.442 s but still failed. Both global retry tokens were consumed without
earning more; regional preparation exhausted at 8,109,218/8,000,000 units (atomic-operation
overshoot), before any XPBD/AVBD or physical assessment. The eight-template prefix still
prepared 11 windows with 100,000 bundle states and 2,048 access states per window. This is not
a successful first-feedback experiment. A bounded-window and bounded-assignment prefix
correction follows; results above remain retained as the failed first variant.

The corrective prefix selects at most two eligible windows in deterministic overlap/clearance
pressure order and clamps bundle/lane search to 1,024 states and access search to 128 states.
Unselected global routes remain in native assembly and whole-board DRC/connectivity. Failed
prefixes do not manufacture evaluated evidence; partial prefixes cannot use the early return.
Independent review found no false-coverage or retention blocker. All 16 focused suites passed again.

## Experiment — Deadline-driven repair experiments and GPU residency (2026-10-10)

### Failure and evidence

The final bounded-prefix Tier-1 run took 212.412236 s, peaked at 763,216 KiB RSS,
and retained the 799-violation quantized oracle (zero opens). Both selected prefix windows
proved access-assignment wipeouts without reaching a complete candidate; neither exhausted
its access search allowance. The session then exhausted its 1,000,000 repair units after
2,854,021 preparation units, without invoking XPBD/AVBD. A token policy cannot distinguish
necessary setup from an unproductive strategy reliably. The user rejected further credit tuning.
Tier 0 retained zero physical DRC/opens but seven pre-existing unrounded joints; it is not a
strict curved qualification pass.

### Hypothesis and ranking

1. Replace credit permission with deadline-bounded, resumable experiments. Initially explore
   eligible diagnosis strategies, then prioritize measured whole-scene gain per active time,
   with aging to prevent starvation. Retire only an exhausted/stagnant strategy-input pair.
2. Expose each completed regional window and proven access failure as a decision point, so
   the coordinator can switch upstream before wasting continuous optimization on no candidate.
3. Reuse GPU immutable inputs where exact identity is available; avoid repeating transfers.

These are related scheduling/data-lifetime changes, not a replacement of routing algorithms.
Physical copper acceptance and external KiCad validation remain unchanged.

### Research and assumptions

Streeter and Smith, [New Techniques for Algorithm Portfolio Design (UAI 2008)](https://mattstreeter.org/Research/mstreeter_uai_2008.pdf),
separate portfolio scheduling from learning which solvers merit time. This motivates interleaving
bounded strategies and measuring outcome per time. Flux's physical objectives and mutable scene
are different from the paper's solver instances; this implementation claims no transferred
optimality guarantee. Exact state keys delimit comparable trials. The small strategy portfolio
permits deterministic selection; stored samples and parked continuations must remain bounded.

The GPU audit found persistent input buffers already in XPBD and hydraulic Vulkan backends.
Terrain evolution retained allocation capacity but re-uploaded its readonly walkability mask.
A single exact mask entry (dimensions plus normalized contents) can remain in device memory
across terrain dispatches. Height feedback must still upload on every invocation. Independent
pipeline contexts have distinct descriptor/lock lifetimes, so sharing raw buffers across them
without an ownership/synchronization interface is deferred.

### Experiment and verification criteria

Implement resumable experiment selection, per-window decisions and the resident mask. Verify
fair exploration, state-specific retirement/reopening, zero legacy-credit starvation, cancellation,
continuation lifetime, GPU repeated-hit parity, changed-mask invalidation, dimension changes and
fresh height feedback with a resident mask. Run focused/core tests, then Tier 0 and Tier 1
sequentially with diagnostic telemetry. Compare physical results and time; do not promote a tier
from internal connectivity or an expected-failure harness exit code.

### Result

Focused validation passed 17/17 suites and core validation passed 65/65 tests. Release and
viewer builds passed. The first diagnostic Tier-0 run exposed admission starvation: five
eligible strategies could exceed the two-entry parked pool during initial exploration,
evicting unfinished setup before any resume. The run was stopped and retained under
`build/repair-experiments-tier0-20261010`; it is not a completed benchmark. Admission now
restricts a full live pool to its existing continuations until a slot becomes available.
A one-slot end-to-end test requires an actual resume with zero count evictions. Final
validation and sequential tier results follow; no tier promotion is claimed yet.


The corrected one-slot test passed; all 17 focused suites and 65 core cases passed again.
Tier 0 completed global planning in 33.359 s and detailed routing in 529.299 s, versus
31.661/369.819 s previously. Retained copper is byte-identical on both rendered layers:
47 straight tracks, 199 Beziers, three vias, zero opens/internal violations/external KiCad
DRC violations, and the same seven unrounded joints. There is no strict tier promotion.
The live pool recorded 54 suspensions, 24 resumes, zero memory/count evictions, and 28
state invalidations. Admission is repaired, but runtime regressed 43% with no physical gain.
Exact context includes negotiation history: a rejected global retry can change that history
and invalidate regional continuations despite retaining incumbent geometry. A future refinement
must distinguish physical dependencies from global search-history dependencies, preserving
current knowledge when a resumed private transaction commits. Simply dropping fields from
the shared exact key would risk stale search state and is not justified by this result.
The GPU mask cache recorded 17 reused transfers and 29 uploads during the global pass.
Tier 1 subsequently timed out at 900.166 s with peak RSS 1,278,852 KiB. It retained the
same 799 oracle violations (544 track-clearance, 179 track/via, 70 hole-clearance, six
via-clearance), zero opens, 1,329 provisional segments and 56 vias. No final detailed
copper or external KiCad validation was reached. The selected checkpoint has no evaluated
regional scene and zero retained XPBD/AVBD windows; these selected-result counters must
not be confused with a complete census of discarded/private attempts. Intermediate telemetry
reached 305/384 portfolio checks and topology-valid but mutually illegal curve candidates.
The engine recorded 221 suspensions, 168 resumes, 53 input invalidations and zero pool
evictions. Terrain telemetry recorded 18 mask reuse hits and 27 uploads. Both provisional
layer previews were inspected: they remain angular scaffold geometry with obvious conflicts,
not a completed smooth route.

### Interpretation and decision

GPU residency works and avoids measured transfers; parity/invalidation checks pass. Keep it.
Deadline experiments remove credit starvation and continuation admission now retains work,
but this experiment does **not** improve the frontier tier. Keep the explicitly requested
scheduler architecture as an unqualified implementation, with the runtime regression and
failure recorded; do not promote tiers or claim algorithmic routing benefit. The next
experiment should isolate physical preparation dependencies from history-only global changes,
then give feasible regional geometry a physical assessment before another broad replan.
Full metrics, source hashes and validation logs are indexed in
`reports/benchmark_history/2026-10-10-repair-experiments.json`.

## Experiment — Physical continuation validity and cross-pipeline GPU sharing (2026-10-10)

### Failure

The preceding deadline experiment retained unchanged Tier-0 copper at 529.299 s detailed
runtime (43% slower), with 28 continuation invalidations. Tier 1 timed out at 900.166 s
with the same 799 quantized violations and 53 invalidations. Rejected global proposals can
increase negotiation pressure without changing incumbent geometry; full-history dependency
keys nevertheless invalidate parked regional work. GPU contexts already share a Vulkan device
and queue but maintain independent copies of compatible readonly mask inputs.

### Hypothesis and scope

Separate regional physical validity from the search-history snapshot used to prioritize a
bounded experiment. Retain unchanged regional preparation through history-only updates and
merge new knowledge at commit so a resumed private transaction cannot overwrite later feedback.
Geometry changes must still invalidate dependent work. Global retries still depend on history.

Share immutable normalized walkability masks between terrain and hydraulic compute pipelines
through bounded runtime-owned device buffers and consumer leases. These algorithms are alternate
routing methods, not a sequential terrain-to-hydraulic algorithm; do not introduce a new solver
chain or change height/potential semantics merely to reuse storage. Mutable polygon rasterization
must use private buffers, and fresh height/pressure feedback remains required.

### Research and validation

The Khronos [synchronization examples](https://docs.vulkan.org/guide/latest/synchronization_examples.html)
and [synchronization specification](https://docs.vulkan.org/spec/latest/chapters/synchronization.html)
distinguish execution ordering from memory visibility. Use transfer-to-compute visibility,
fenced publication, and lifetime leases; a shared VkDevice alone does not make buffer reuse safe.
Exact input identity, bounded allocation, concurrent consumers, changed inputs, polygon writes,
CPU parity, and cancellation need regression coverage. Then run Tier 0 and Tier 1 sequentially
with diagnostic telemetry against the recorded baseline. Implementation/results pending.

Validation note: the first Tier-0 check exposed exhaustion of the 64 MiB canonical-key store after 23 regional decision points. Full corridor geometry had been duplicated in every diagnosis/history context. Physical geometry is now interned independently with exact string equality, and negotiation keys reference its stable local identifier. Physical and negotiation records share the existing byte bound. The failed check is preserved in `build/history-sharing-tier0-key-bound-20261010`; the corrected run advanced through global/regional planning normally. Focused validation passed 17/17 suites. Core validation passed 65/65 after updating the hydraulic reuse assertion to require 14 private buffers plus one shared mask (zero mask uploads on reuse).

Final Tier-0 measurement for this experiment: global 33.842 s, detailed 507.719 s versus 529.299 s baseline. The selected copper remained 47 straight tracks, 199 Beziers and 3 vias, with zero internal/external DRC violations and zero opens. Both copper PNG hashes match the baseline exactly; both layers were visually inspected. The same seven unrounded non-junction joints still fail strict curve acceptance. Scheduler resumptions increased 24→57, invalidations decreased 28→19, and no pool eviction occurred. Mask uploads decreased 29→20 and reuses increased 17→26 in the selected-plan counters. Cross-consumer reuse is covered by GPU tests but was zero in this terrain-only benchmark configuration. This is a modest runtime improvement and stronger continuation reuse, not a tier promotion.

The intermediate 250.535 s run is deliberately excluded from performance claims: further record-level interning was required to avoid duplicating unchanged corridor records as individual curves changed. Complete physical keys and their individual records are now interned separately under the shared metadata bound; metadata exhaustion has an explicit telemetry event. The interrupted initial Tier-1 attempt is also excluded from comparisons.

Tier-1 final result: timed out after 900.199 s, retaining the same 799 oracle violations (544 track clearance, 179 track/via clearance, 70 hole clearance, 6 via clearance), zero abstract opens, 1,329 provisional tracks and 56 vias. No evaluated regional scene and no final copper/KiCad DRC result were produced. Invalidations fell 53→14 and experiments started fell 61→19; resumptions were 69 and suspensions 84. Peak RSS rose 1,278,852→1,428,236 KiB. Selected-plan mask uploads fell 27→23 and reuse rose 18→22. Both preview layers were inspected: they remain angular provisional scaffolds with visible crossings, not smooth legal copper.

Interpretation: the targeted history/continuation mechanism improved and GPU reuse is covered by parity/lifetime tests, but Tier 1 did not improve physically. Retaining expensive regional preparation trades fewer restarts for more resident state. The next evidence-backed investigation is the expensive regional portfolio/preparation step before the first evaluated scene, and fair bounded scheduling of that retained frontier. Do not claim that cross-pipeline sharing or fewer invalidations solves Tier 1.

Final validation: 17/17 focused suites and 65/65 core tests passed after the final metadata change. Release CLI and viewer-tree CLI/viewer targets built successfully. Complete results, source/binary hashes and limitations are in `reports/benchmark_history/2026-10-10-history-sharing.json`. Decision: keep the targeted continuation and shared-mask changes; continue investigating the pre-physical regional preparation bottleneck separately.

## Tier-2 diagnostic check — 2026-10-10

User-requested check after history-continuation and shared GPU mask changes; no algorithm edits. Ran `ft2232h_jtag` alone with diagnostic telemetry, curved routing, explicit locked-tier execution, 30 global/detail iterations and 900 s routing deadline (960 s harness deadlines). The preliminary manifest-default one-iteration probe was interrupted and excluded.

Result: engine timed out after 900.166 s (904.19 s harness wall), peak RSS 1,120,852 KiB. Retained global checkpoint has 117 paths, zero unrouted nets, total overflow 381, 34 congested resources, five congested corridors and 340 clearance-shadow conflicts. Internal quantized-copper oracle: zero opens but 930 violations (842 track clearance, 45 track/via clearance, 43 hole clearance), 1,672 tracks and 21 vias. No retained evaluated regional scene or accepted final copper; external KiCad DRC was not run. Harness diagnostic exit zero does not qualify the board.

Telemetry reached topology-legal PH candidates whose gate sequences were valid but mutual copper legality failed. Scheduler recorded 12 starts, 14 resumptions, six invalidations and no memory evictions. Selected-plan GPU masks: 10 uploads, 31 reuses, zero cross-consumer reuses. The provisional overview was visually inspected and remains predominantly angular scaffold geometry with overlaps. This check reinforces the distinction between complete abstract connectivity and physical legality; it does not promote Tier 2. Full configuration, metrics, artifact paths and binary hash: `reports/benchmark_history/2026-10-10-tier2-check.json`.

## Clearance-shadow / erosion audit — 2026-10-10 (proposal, not implemented)

Tier-2 evidence: 340 shadows were not truncated (`clearance_shadow_conflict_budget_exhausted=false`) and were not advisory-only. All had owner attribution, but owner-history passes were zero. The 930 quantized-copper violations use different geometry and aggregation; the difference is not a measured shadow false-negative count.

Code findings: `hydraulic_path.cpp::rasterize_polygon_mask` computes point-to-polygon-boundary distance but stores integer cell units; its fallback clearance builder uses a four-neighbour wavefront. `terrain_evolution.cpp` initializes height from inverse clearance plus pressure/success and then evolves that height, so evolved height is not a physical clearance distance. Live copper enters hydraulic discovery via pressure deposits. `global_router.cpp::account_routing_occupancy` deliberately suppresses coincident centreline shadows when integer corridor capacity appears adequate, deferring actual lane legality to regional packing. Exact chord shadow queries scan owner segments, and shadow geometry precedes final curves and quantization.

Ranked proposal: (1) common immutable, rule-aware physical clearance service with exact owner primitives and conservative distance bounds, separately consumed by shadows and erosion; (2) cheap packing certificates before suppressing same-corridor conflicts, plus conflict-combination feedback upstream; (3) integrate dynamic copper updates into hard clearance as well as soft terrain pressure, with actual copper hard and provisional alternatives negotiable; (4) add a spatial index to exact owner-segment queries and use actual selected widths; (5) keep bounded GPU distance/owner tiles resident across consumers. Benchmark shadow-versus-oracle coverage on the same geometry and classify deferred packing separately before assuming missed detections.

Research: Felzenszwalb/Huttenlocher exact sampled Euclidean distance transform (https://cs.brown.edu/people/pfelzens/papers/dt.pdf) and GPU Parallel Banding Algorithm (https://www.comp.nus.edu.sg/~tants/pba.html). Both concern sampled fields, not exact continuous copper legality. Jump flooding (https://www.comp.nus.edu.sg/~tants/jfa.html) is approximate, suitable for guidance/candidate discovery unless supplemented with conservative bounds and exact checks. GPU EDT integration is higher cost than immediate indexed primitive-query reuse; validate benefit before replacing existing geometry machinery.

## Experiment — Indexed physical pixels beneath terrain discovery (2026-10-10)

Hypothesis: preserve exact polygon geometry behind a reusable 0.05 mm pixel index, rather than recomputing every distance by scanning every polygon boundary on each terrain solve. Separate geometry from erosion/history so valid physical records survive preference changes. This is a representation/reuse experiment; it does not claim to resolve corridor packing or Tier-1/Tier-2 legality by itself.

Implementation: integer pixel-to-record array, pooled exact boundary primitives and overlap references, BVH nearest/parity/overlap queries, conservative whole-cell clearance bounds, sparse record materialization and bounded immutable geometry cache. The default terrain wrapper consumes cached physical walkability/clearance while retaining adaptive erosion resolution and final exact geometry gates. Prepared-domain ring ownership is retained; original KiCad item identity and raw per-net primitive filtering are outside this integration. Physical geometry changes rebuild; pressure-only changes reuse. Bounds fall back explicitly to the exhaustive polygon raster, not to assumed free space. GPU height evolution and shared GPU mask leases remain active.

Validation pending: synthetic geometry/multi-boundary/conservative/cap/cancellation tests; production indexed-vs-exhaustive raster/terrain parity; history reuse and changed-geometry invalidation; existing CPU/GPU regression suites; sequential tier checks with telemetry. Baselines are the recorded history-sharing Tier-0/Tier-1 runs and the separate Tier-2 diagnostic check.

Review/validation update: 8/8 focused suites and 65/65 core tests passed. Integration review found no remaining material defect after single-builder synchronization, boundary parity and cancellation fixes. Additional tests cover endpoint/success-history reuse, multiple owners in a pixel, unsampled-cell identity, overflow/byte/reference limits and BVH-versus-exhaustive queries. The non-polygon fallback wavefront and GPU dispatch retain their existing in-flight cancellation latency.

Scope correction: the user correctly identified that this implementation does not integrate the indexed map into clearance-shadow checks. This pass was scoped too narrowly to terrain preparation. Dynamic shadows retain net/layer/selected-radius owner segments in `RoutingOccupancy`; polygon-parity `exact_query` cannot safely replace those checks. A shared candidate-query API must preserve owner identity, layer, width/rules and occupancy invalidation, while retaining exact segment distance. Broad-phase reuse alone would accelerate checks without fixing deferred same-corridor packing. Do not claim this experiment implements the complete shared physical clearance service proposed above.

Terrain-only result observed before the requested follow-up: Tier 0 global 27.298 s, detail 422.361 s, 0 opens and 0 internal copper violations; 47 straight tracks, 199 Beziers, 3 vias, and the same 7 unrounded non-junction joints. Physical-map counters: 20 builds, 26 reuses, 0 fallbacks. Both copper layer images were inspected. Build-directory artifacts subsequently became unavailable for an undetermined reason, so hashes, RSS and external DRC were not recovered. The observations and this preservation limitation are recorded in `reports/benchmark_history/2026-10-10-indexed-clearance.json`; do not promote Tier 0 from this run.

## Follow-up experiment — Explicit packing obligations and indexed owner shadows

User requested closing the missing clearance-shadow integration and accepted explicit packing-unresolved state. Same-corridor integer-capacity compatibility previously suppressed overlaps before selected-width qualification. Hypothesis: preserve bounded attributed obligations and require physical evidence before treating the corridor as clear, while accelerating direct-edge owner enumeration without changing exact clearance rules.

Change: same-corridor overlaps become bounded, deduplicated obligations only after exact selected-width/rule qualification. They carry normalized owner/net/layer geometry, remain separate from violation/overflow counts, and do not charge generic congestion history. Regional work consumes distinct obligation evidence and cannot claim completion from a no-conflicts/discrete label without a complete legal physical scene. Engine diagnosis selects bounded regional proof; measured capacity failures still select upstream replanning. CLI and viewer report pending obligations and truncation explicitly.

Direct-edge shadow checks use an immutable open-segment BVH built through IndexedClearanceMap, with owner references, net/layer tags and selected copper radii. Conservative envelope enumeration retains exact segment-distance and pairwise rule tests; same-net handling and via-layer attribution remain in the authoritative owner records. Snapshot invalidation follows occupancy insertion/removal, with exhaustive fallback when bounded index preparation fails. Sampled shadow construction retains its existing spatial raster. Rich physical records remain CPU-side; this does not claim a new GPU geometry index.

Independent review found no material issue after bounding the obligation dedup set. Root also corrected partial packing metrics so they cannot zero unrelated viewer metrics. Final build/tests and sequential telemetry tier checks pending. Source diagrams: `docs/diagrams/flux-system-flow.mmd`, `flux-engine-decisions.mmd`, and `flux-clearance-and-compute.mmd`.

Combined validation after the packing fixture correction: 12/12 focused suites passed; the core suite passed 65/65. Release targets relinked and the viewer-tree CLI/viewer build completed. The synthetic corridor fixture now carries sampled occupancy evidence, matching the incremental corridor reconstruction contract; neighboring cross-corridor shadows remain ordinary conflicts. Independent source review also checked the system diagrams for conditional CPU/GPU and repair paths. Sequential Tier-0/Tier-1 telemetry checks follow.

Combined Tier-0 result: global 26.983 s, detailed 421.233 s; zero internal and external KiCad DRC violations and zero opens, with the same 47 straight tracks, 199 Beziers, three vias and seven unrounded non-junction joints. Both copper PNGs were viewed and their hashes exactly match the history-sharing baseline. The selected plan has zero source packing obligations, so this board does not exercise that mechanism. Indexed physical preparation recorded 20 builds, 26 reuses and zero fallbacks. This preserves physical correctness but still fails strict curve acceptance; it is not a tier promotion. Artifacts are now preserved under reports/validation/indexed-shadow-20261010/tier0.

Combined Tier-1 result: 900.185 s timeout, 78 abstract paths, zero unrouted nets and zero abstract overflow, but the same 799 quantized-copper oracle violations (544 track clearance, 179 track/via, 70 hole clearance, six via clearance). No retained evaluated regional scene or final external KiCad DRC. Three unresolved packing obligations remain, with no truncation. Indexed preparation: 23 builds, 22 reuses, zero fallbacks. Scheduler: 19 starts, 74 resumptions, 14 invalidations, 89 suspensions; peak RSS 1,341,448 KiB. Both provisional copper layer previews were inspected; they remain angular, overlapping planning geometry. This improves explicitness and preserves correctness, but does not establish physical routing progress.

Concurrent-source limitation: additional legal-scaffold source changes appeared while these benchmarks ran. The binary hash remained fixed. The recorded builds/tests/results validate the binary and source hashes in reports/benchmark_history/2026-10-10-indexed-shadow-packing.json, not the later scaffold edits. They were preserved untouched. The system diagrams mark that additional working-tree path as unbenchmarked in this run.

## Source comparison — FreeRouting physical connection loop (2026-10-10)

Inspected FreeRouting revision `2d2e7423adc65a3f953a186a58207fa04993dba9` using primary source, without running FreeRouting on the Flux corpus. This establishes architectural differences, not comparative pass rates. Source license is GPL-3.0; this analysis transfers concepts and contains no imported implementation.

Evidence: the frozen Flux Tier-1 run retained 78 paths, zero abstract overflow and zero abstract opens but 799 quantized-copper violations after 900.185 s. Three corridor packing obligations remained and no evaluated regional scene was retained. Scheduler suspension/resumption worked (89/74), but that did not produce a better retained physical result. Tier 0 instead already has electrically connected, DRC-clean copper and fails its separate seven-joint curve-completion gate. These are different failure mechanisms.

FreeRouting sources:
- [AutorouteConnectionRouter](https://github.com/freerouting/freerouting/blob/2d2e7423adc65a3f953a186a58207fa04993dba9/src/main/java/app/freerouting/autoroute/pipeline/AutorouteConnectionRouter.java): connected/unconnected same-net item sets, per-connection routing, local changed-area pull-tight, pass-scaled rip-up costs. Strict post-insertion DRC applies when explicitly configured or from pass three; full-board rollback snapshots are conditional. Do not assert that every intermediate FreeRouting board is always DRC-clean.
- [MazeSearchEngine](https://github.com/freerouting/freerouting/blob/2d2e7423adc65a3f953a186a58207fa04993dba9/src/main/java/app/freerouting/autoroute/maze/MazeSearchEngine.java): clearance-compensated widths constrain rooms and doors; shoving, rip-up, and drill transitions participate in search. This is geometric room/door search, not a finer terrain raster.
- [AutorouteEngine](https://github.com/freerouting/freerouting/blob/2d2e7423adc65a3f953a186a58207fa04993dba9/src/main/java/app/freerouting/autoroute/maze/AutorouteEngine.java): locate and insert each physical connection, with explicitly selected ripped connections. Fresh expansion databases are the production default; retained state is optional, so wholesale cache retention is not the defining advantage.
- [AutoroutePassRunner](https://github.com/freerouting/freerouting/blob/2d2e7423adc65a3f953a186a58207fa04993dba9/src/main/java/app/freerouting/autoroute/pipeline/AutoroutePassRunner.java) and [AutorouteBatchLoop](https://github.com/freerouting/freerouting/blob/2d2e7423adc65a3f953a186a58207fa04993dba9/src/main/java/app/freerouting/autoroute/pipeline/AutorouteBatchLoop.java): persistent incomplete-net ordering, deterministic seeded diversification after stagnation, pass-local and global stagnation tracking, best-board recovery.
- [BatchOptimizer](https://github.com/freerouting/freerouting/blob/2d2e7423adc65a3f953a186a58207fa04993dba9/src/main/java/app/freerouting/autoroute/pipeline/BatchOptimizer.java): normal preflight guards bypass whole-board optimization while connections remain incomplete; candidate acceptance protects connectivity and clearance and evaluates score. Local pull-tight still runs during routing.

Ranked interpretation and experiments:
1. Highest confidence: the normal unit of progress should be a physically checked connection or small conflict-component transaction. Flux's tested pipeline can yield repeatedly while retaining a large preparation frontier rather than improving actual copper. Retain a legal partial-board checkpoint and publish completed transactions with their exact blockers; measure time to first legal insertion and legal connected-component growth. Existing short engine actions and indexed queries can support this; a new scheduler is unnecessary for the experiment.
2. High confidence: physical feasibility must constrain the route choices before expensive whole-region fitting. Explicit packing obligations now expose missing proof but do not themselves pack a channel. Keep the new exact owner queries, and use them for constructive search and small blocker repairs. Do not interpret zero abstract overflow versus 799 oracle reports as a measured shadow false-negative rate: their representations and aggregation differ.
3. Source-supported opportunity: route between current connected copper components and their legal access sets. Flux supports same-net branching elsewhere, but the new scaffold search still uses each supplied request's first/last point as its grid search endpoints. Generalize these targets to existing same-net copper, pad access and permitted layers; test a legal branch connection that avoids an unnecessary endpoint detour or via.
4. Source-supported recovery cost: the current scaffold now attributes victims to conflicting copper, but an exchange still rebuilds survivors by searching them again. Preserve unchanged legal survivors and transact only a small causal blocker component. Measure revalidated versus rerouted survivors and time per committed repair. Larger coherent exchanges should be introduced only with a reproducing narrow-channel case.
5. Keep connectivity/DRC completion distinct from curve refinement. Preserve the DRC-clean Tier-0 board while targeting its seven unfinished joints; do not use those joints as evidence that global connectivity search failed. Whole-board curve quality work should not repeatedly replace proven electrical topology.

Concurrent source addition: `construct_copper_scaffold` now runs ahead of ordinary regional realization when enabled/eligible, retains legal subsets, uses exact dynamic checks and only promotes a complete result after the quantized oracle passes. This addresses the principal architectural gap in direction, but was not in the benchmark binary. Remaining source risks observed in `search_route`: fixed snapped endpoint states; rejecting the first assembled goal returns failure rather than generating a different legal attachment/path; bounded lattice/expansion exhaustion is not an infeasibility proof. A robust fix needs legal source/target states and rejection-aware alternatives, not merely replacing `return` with `continue` under the same single-predecessor A* state.

Decision: keep the validated index/obligation integration, with no claim of Tier-1 completion. Prioritize measuring and validating the constructive physical transaction path before adding more preparation portfolios. Next oracle experiment: run FreeRouting on the same board/rules and time budget, then compare connected copper, DRC and bottleneck topology rather than its reported route count alone. Diagram sources include `docs/diagrams/flux-vs-freerouting.mmd`; the legal-scaffold branch is explicitly marked as unbenchmarked in this run.

## Experiment — Component attachment search and retained physical transactions (2026-10-10)

Failure: the constructive scaffold's first assembled goal could fail its whole-path copper check and terminate the connection search. Fixed endpoint access omitted useful same-net component branches. Causal victim repair reconstructed unaffected survivor copper, and checkpoint quality counted stored paths instead of satisfied requests. The earlier 799-violation Tier-1 benchmark predates this scaffold implementation and is not a direct baseline for these changes.

Hypothesis: bounded physical attachment alternatives and source/arrival search labels preserve useful recovery options; component queries let branches terminate on existing copper. Preserve exact-checked connection transactions and exchange only attributed victim geometry so search effort produces retained physical progress.

Change: batched canonical-connectivity component membership, endpoint-layer and component attachment sites (including track-interior projections), bounded attachment rings, source/arrival/via-history labels, fairly bounded rejected-goal attempts, and exact immutable-scene edge reuse. Every assembled path still passes dynamic copper DRC and request connectivity. Causal exchange copies the retained scene, removes explicitly owned victim copper, preserves survivor object IDs/geometry and shared vias, and verifies all prior satisfied requests before committing. Checkpoints and CLI report request/satisfied/missing counts independently of route count; the engine recognizes growth in checked satisfied requests without relaxing full-board acceptance.

Limits: search remains a bounded lattice heuristic, not a completeness proof. Labels preserve drilled-via history; planar histories with the same source, arrival and via history may still coalesce. Search remains limited by the existing expansion allowance. Zone interiors are not sampled; existing component contacts remain available. Curves are refined after constructive topology and remain subject to the existing oracle and curve-quality gates. No FreeRouting code was copied.

Verification: pending focused regressions, core suite, independent review and sequential telemetry tier checks. Mechanism cases include disconnected padded components, layer isolation, already-connected no-op requests, interior branches, stale retained attachments, a rejected two-via drill-spacing assembly, cancellation, and survivor identity preservation.

Review refinements: canonical bounded via-history tries preserve different drilled-site sets without distinguishing their insertion order; failed insertion rolls back newly interned nodes so rejected candidates cannot consume later search capacity. Endpoint membership distinguishes coincident SMT pads on separate layers. The physical scaffold scene must preserve materialization-source obstacles and apply exactly the detailed router's replacement policy, including locked-copper protection and implicit routed-net scope. Regression fixtures cover both source-only obstacles and authorized replacement; final validation follows below.

Final validation: six focused suites passed and the core suite passed 66/66. Viewer and Release CLI builds passed; the viewer's warnings-as-errors caught a signed bit-width conversion, corrected with an explicit bounded conversion. Three scaffold suites passed again after that correction. Independent review found no remaining material correctness issue. No source or frozen-binary drift occurred during benchmarks. Full evidence: `reports/benchmark_history/2026-10-10-component-scaffold.json` and `reports/validation/component-scaffold-20261010/`.

Sequential telemetry Tier 0 (`pt2399_delay`): planning 125.928 s, detail 327.951 s, zero opens, zero internal and external KiCad DRC violations; 47 straight tracks, 199 Beziers, three vias, and the same seven unrounded non-junction joints. Both final copper images were viewed and are byte-identical to the prior indexed-shadow baseline. The planning scaffold retained 49/50 requests, with two attempted but no accepted victim exchanges. Strict curve acceptance still fails. Combined planning+detail time is 453.879 s versus the earlier 448.216 s; this is not an isolated performance comparison because the earlier binary predates the scaffold. Peak RSS 1,470,288 KiB.

Sequential telemetry Tier 1 (`bm_svf`): engine exhausted its action allowance after 843.536 s (900 s deadline), with 78 abstract paths but only 60/78 checked scaffold requests satisfied. One request was satisfied by existing component connectivity, so there are 59 stored paths; retained copper reports 499 tracks and 22 vias. One of 51 victim exchanges succeeded while preserving 438 survivor tracks and 22 vias. Search expanded 6,632,018 states, reported no assembled-goal rejection on this board, and used at most 153,035 via-history nodes without exhausting the node allowance. The synthetic regression, rather than this board, demonstrates rejected-goal recovery. Eighteen requests remain; there is no final whole-board oracle certificate or external KiCad DRC. Peak RSS 928,480 KiB. Both provisional layer images were viewed; they are angular coarse-planning previews, not the checked partial scaffold or final curves. The corpus runner's expected-incomplete probe passes, but the board is unqualified and must not be called routed.

Interpretation and decision: retain the implementation. Focused tests establish endpoint/component and rejected-goal correctness, and the real Tier-1 exchange confirms survivor reuse is exercised. Tier 0 preserves the exact baseline copper; Tier 1 now retains explicit physical progress but remains incomplete. No claim is made that 60 checked requests are directly comparable to the old 799 whole-board oracle reports. Next experiment should diagnose the 18 unresolved requests and why 50 causal exchanges failed, then test a bounded coupled repair or better escape alternatives against a minimal reproducer. Increasing the action limit alone is not evidence of a solution. Rendering/exporting the retained partial scaffold separately from coarse previews remains a visibility limitation.

## Experiment — Physical scaffold feedback and bounded coupled repair (2026-10-11)

Failure: Tier 1 retained 60/78 checked demands, but regional/global feedback still operated on coarse plan geometry. One of 51 single-victim exchanges succeeded. The engine stopped after 16 actions at 843.536 s, before its 900 s deadline. Aggregate telemetry did not explain termination of each missing demand.

Hypothesis: retain a stable physical demand domain across replans, analyze only missing demands against the checked copper scene, and escalate causal repairs to bounded victim pairs. New global paths remain proposals; they do not replace the original connectivity obligations. Carry attributed failed-search evidence into repair selection without treating bounded exhaustion as a measured capacity deficit.

Change: stable original requests, endpoint-scoped proposal admission, regional frozen checked scene, exact reconstruction of all demands before whole-board certification, and physical terrain feedback from retained copper. Shared regional preparation from the coarse scene is excluded. Coupled exchanges preserve unrelated copper and commit only after every prior satisfied request reconnects; eight pair attempts share a 200k expansion allowance. Per-request termination and bounded rollback/owner evidence are exported. Default action safety ceiling is 256 with a separate 16-action physical-stagnation stop; explicit action overrides and the shared deadline remain hard bounds.

Validation pending: synthetic pair-only repair, partial physical-scene handoff, prior unit/regression suites, independent review, and sequential telemetry Tier 0 then Tier 1.

Review refinements: dynamic vias are attributed to their current same-net path consumers after each committed exchange, and identical zero-search reconstruction retains the previous causal failure evidence. Frozen missing-demand repair is limited to the regional-realization action; XPBD and confined erosion retain whole-plan alternatives so a bad retained topology cannot exclude coordinated recovery. Their continuation identities include the actual proposal scene. Explicit saved-zone-refill workflows retain their provisional regional/materialization path and final external refill gate. Materialization-ready candidates keep priority over partial-scaffold repair.

Validation: six focused suites and all 66 core tests pass. The existing WorkPlan zone-refill regression caught incompatible fixed-fill blocker feedback; explicitly deferred refill now bypasses scaffold construction and retains its original external refill boundary. Release CLI and warnings-as-errors CLI/viewer builds pass. Independent review reports no remaining material finding; the complex successive shared-via selection fixture is deferred, while existing shared-transition/exchange cases and the consumer-map implementation were checked. Frozen source/binary identities and sequential telemetry runs are under `reports/validation/physical-repair-20261011/`.

Sequential Tier 0 telemetry check: `pt2399_delay` planning 110.914 s, detail 313.399 s (424.313 s combined versus 453.879 s before), peak RSS 1,382,832 KiB. Checked scaffold reports 50/50 requests rather than 49/50; this does not itself establish the full-board oracle. Final materialized copper remains byte-identical on both viewed layers to the component-scaffold baseline: zero opens, zero internal DRC, zero KiCad violations/unconnected items, 47 straight tracks, 199 Beziers, three vias, and the same seven unrounded non-junction joints. Strict curve acceptance remains failed. No lower-tier connectivity, DRC, or curve-quality regression observed; proceed to Tier 1 sequentially.

### Assessment of six-phase navmesh/erosion proposal (2026-10-11)

The suggested geometry → corridor → density → search → curve → repair sequence matches much of existing Flux. Its main missing bridge is multi-net corridor packing and coupled topology/layer/via assignment before smoothing. Single-route free-space validity is not a certificate that several routed demands fit simultaneously.

Terminology: morphological erosion of free space (equivalently obstacle Minkowski dilation) is a hard geometric operation; LittlePlanets height evolution and USPED are soft guidance. Flux's AVBD-style local block/contact solver belongs to physical relaxation, not Voronoi decomposition. The source paper calls AVBD Augmented Vertex Block Descent: https://graphics.cs.utah.edu/research/projects/avbd/ .

Phase 1: reuse existing offset/prepared geometry and indexed ownership metadata. A valid free-space certificate must specify layer, trace width, pairwise clearance/rule profile, board-edge margin, copper revision, and conservative approximation error. Do not obstruct an entire component body unless its actual rule/keepout requires that restriction. Via copper and drills need separate multilayer legality. Reference: https://doc.cgal.org/Manual/latest/doc_html/cgal_manual/Minkowski_sum_2/Chapter_main.html . Recast itself voxelizes before polygon generation; importing a game navmesh does not automatically provide exact PCB clearance: https://recastnav.com/ .

Phase 2: retain generalized medial/Voronoi routes as clearance-aware alternatives, not mandatory centre lanes. All nets choosing the same centre causes congestion; maximum obstacle clearance also does not prove electrical signal integrity. Generalized visibility/Voronoi tradeoffs are relevant: https://www.cgl.cs.tau.ac.il/projects/the-visibility-voronoi-complex/ .

Phase 3: current USPED is explicitly geometry-only, dimensionless refinement, with unit rainfall unrelated to net demand (include/flux/core/usped.hpp). A useful new experiment would feed actual route demand, available cross-section, and measured physical conflicts into congestion guidance, while retaining the immutable blocked mask and legal terminal access. Raw pin-density repulsion can penalize exactly the escape region a net must use. USPED's original sediment transport/divergence semantics do not themselves establish congestion feasibility: https://ncsu-osgeorel.github.io/erosion-modeling-tutorial/erdep_theory.html .

Phase 4: prepared navmesh/portal search and funneling already exist. Retain endpoint/arrival/layer alternatives and via footprints. Fewer graph nodes may help, but mesh preparation and multi-net allocation remain significant; no near-instant runtime guarantee follows. Add or strengthen explicit lane order/cross-section/transition packing before expensive curve work. A 1.2 mm raw channel with 0.2 mm traces and 0.2 mm trace/sidewall clearances can fit two ideal straight traces, not three, although each independently has a valid route.

Phase 5: funnel/string-pull the selected portals, then constrain curve pieces within certified convex safe regions; do not interpolate portal midpoints. Source: https://digestingduck.blogspot.com/2010/03/simple-stupid-funnel-algorithm.html . Bézier control points being individually in a nonconvex free region does not imply that the curve stays there; a convex-hull containment certificate is sufficient for position, but curvature still needs its own bound: https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node12.html . Ordinary cubic Bézier offsets are generally not exact cubic Béziers; PH has rational-offset advantages, with bounded conversion/error checks still needed for native output: https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node232.html . Flux currently exports net-assigned gr_curve copper in src/kicad_export.cpp; the older PH ADR wording about only segments/arcs is stale.

Phase 6: use exact checks throughout, not only after fitting. XPBD/AVBD contact relaxation generates candidates under bounded work; atomic connectivity/DRC commit remains authoritative. When local geometry stagnates, change lane order, homotopy, layer or via choice and feed the physical witness upstream. Keep safe repair subsets and explicit rollback reasons.

Benefit/cost ranking for incremental experiments: (1) physical failure attribution and early packing rejection, high expected benefit/moderate integration cost; (2) reusable rule-aware clearance certificates and local invalidation, high expected runtime benefit/moderate correctness complexity; (3) demand-aware field guidance, medium potential benefit/medium cost and experimental risk; (4) stronger smoothing, mostly geometry-quality benefit and lower expected effect on unresolved Tier-1 topology. Avoid replacing the existing navmesh or adding another unconstrained smoothing stage.

### Final Tier 1 result and decision (2026-10-11)

Sequential telemetry run of `bm_svf` exhausted after 836.222 s and 18 completed actions: 78 abstract paths, zero abstract overflow, but only 62/78 checked requests (61 stored paths plus one already-connected request), versus 60/78 before. No checked coverage gain followed the first scaffold construction. Sixteen requests remain: twelve report expansion-budget exhaustion and four attachment exhaustion. Neither category proves geometric infeasibility. No final whole-board oracle or KiCad DRC certificate exists.

The retained scaffold pass reports 7,262,443 expansions, 740 tracks, 24 vias, 64 single-victim attempts with zero accepted, and 22 coupled attempts with zero accepted; seven coupled expansion-budget exhaustions occurred. These are latest retained-pass counters, not cumulative run totals. The bounded rollback journal contains 30 incoming-unroutable and two victim-unroutable records. Nine frozen regional dispatches produced no retained coverage improvement. Peak RSS rose from 928,480 to 1,685,768 KiB (about 82%). The two-connection gain is not evidence that pair exchange improved this board; its benefit is established only by the focused synthetic regression.

Both Tier-1 layer previews were viewed, but they depict coarse global paths, not checked retained scaffold copper or final curves. Full-scaffold oracle failures currently discard their diagnostic result, and frozen regional local diagnostics are not installed in legacy full-plan scope counters. These gaps prevent a sound claim about the precise physical reason each regional candidate failed. Source and frozen-binary hashes remained unchanged during both tier runs. Structured results: `reports/benchmark_history/2026-10-11-physical-repair.json`.

Decision: retain the verified stable-demand, retained-scene and bounded transactional repair mechanisms, with no claim of Tier-1 completion or strict Tier-0 curve qualification. Do not increase coupled search budgets based on this result. Next experiment should retain oracle/frozen-regional failure witnesses, distinguish access exhaustion from packing/topology conflict, and reject impossible assignments before expensive searches. Improve causal selection and preparation reuse before adding more concurrent work or larger search allowances.

## Experiment — Physical conflict components and scoped assignment evidence (2026-10-11)

Failure: previous Tier 1 had 78 abstract paths, zero abstract overflow and 62/78 checked physical requests after 836.222 s. The retained pass had 22 coupled attempts with zero accepted; peak RSS was 1,685,768 KiB. Frozen regional work discarded its local diagnostics and preparation, while pair-only exchanges could not rearrange larger blocker components. This run is the immediate baseline; Tier 0 retained zero opens/DRC but seven unrounded joints.

Hypothesis: bounded causal joint jobs, reusable physical preparation and correct failure provenance improve useful repair work without replacing the global architecture. Unfinished curve optimisation must not blacklist a potentially feasible discrete assignment.

Research: conflict-based search separates conflict constraints from low-level paths (Sharon et al., https://ojs.aaai.org/index.php/AAAI/article/view/8140). Adapt the decomposition only: PCB permanent occupancy, rule widths and multilayer vias do not meet discrete time-expanded MAPF assumptions. Existing conservative offsets and physical lane packing remain necessary (https://doc.cgal.org/Manual/latest/doc_html/cgal_manual/Minkowski_sum_2/Chapter_main.html).

Change: bounded fair causal demand batches; dedicated exact-scene regional preparation reuse; one shared-budget speculative preparation worker against immutable retained copper; retained per-region and scaffold-oracle diagnostics; bounded deterministic multi-victim copper transactions; safer assignment no-good provenance. Existing regional joint topology/packing, XPBD/AVBD, GPU screening and exact commit checks are reused. Detailed design: `docs/adr/0170-physical-conflict-components.md`.

Validation and sequential Tier 0/Tier 1 telemetry benchmarks pending. Success requires more checked connectivity without lower-tier DRC/curve regression; report runtime/RSS and accepted repair mechanisms rather than attributing any gain to the entire bundle without evidence.

Regression findings during validation: the first fair-budget implementation reserved for configured attempt slots even when a two-victim frontier had no larger component, starving the previously passing pair repair. Allocation now reserves only for actual larger component cardinalities and reclaims unused demand shares within the same hard total bound. The WorkPlan selected-net regression also exposed a scope error in newly retained oracle feedback: unrelated whole-board opens entered a selected-net action's repair instructions. Active oracle feedback is now filtered to intersect the explicitly selected net scope, while the complete oracle report and whole-board acceptance gate remain unchanged. Neither failure was accepted as a benchmark result; reruns follow.

The feasible three-victim U-detour micro-board exposed an additional discovery gap: initial source/target search attributed the outer blockers, but the middle retained route was hidden behind them. The first generic component implementation therefore attempted only pairs (four total victim participations) and could never exercise its three-victim transaction. Failed repair searches must contribute newly exposed retained-object owners to the bounded causal frontier; merely increasing maximum component cardinality is insufficient. The full core suite still passed after the selected-net fix.

The discovery fix maps newly exposed exact collision objects back to retained request owners, keeps a six-owner frontier bound, and updates later component enumeration. The corrected micro-board now passes: the two-victim bound remains incomplete, while the three-victim transaction reconnects all demands and preserves the unrelated locked copper under internal DRC. Independent review found no material issue in the discovery, bounded scheduling, or transaction changes. Final rebuilt CLI/regressions and tier checks follow.

Final implementation validation: Release CLI build passed; ten focused suites passed; the full core suite passed; warnings-as-errors CLI/viewer build passed. Independent review found no remaining material issue. The frozen binary and 203 implementation-input hashes are recorded in `reports/validation/joint-physical-20261011/source-build.json`; Tier 0 then Tier 1 run sequentially with diagnostic telemetry and a 900 s routing deadline.

The first Tier-0 check was interrupted after revealing a representation-scope regression: retained failed scaffold-oracle diagnoses vetoed the independently produced regional no-conflicts handoff, extending planning well beyond the 110.914 s baseline. That attempt is archived under `reports/validation/joint-physical-20261011/interrupted-oracle-scope/` with its frozen binary, hashes and telemetry; it is not a completed benchmark. Repair guidance now leaves an independently ready materialization candidate to its own evidence and downstream exact checks. The full failed scaffold oracle remains recorded. A new engine integration fixture adds a source-only open terminal on the same net, retains the exact scaffold-oracle rejection, verifies that the separately planned regional candidate still reaches detailed source validation, and verifies that no detailed copper is thereby certified.

A second interrupted Tier-0 check showed that guarding only an already-ready candidate was insufficient: raw scaffold-oracle witnesses had entered earlier independent global history. Final provenance is stricter: only current-scene frozen-region evidence guides incomplete-scaffold repair. Complete-scaffold oracle failures remain fully recorded for diagnostics and certification, with no raw injection into another candidate's history. The integration regression now checks every engine step for such leakage, not just final handoff. The second interrupted binary/telemetry is archived under `interrupted-early-oracle-history/`.

Independent review found the remaining Tier-0 scheduling defect: retaining an oracle or frozen-region journal set `pending.evidence_recorded`, a flag whose existing semantics authorize rollback/comparison of the working plan. These diagnostic records need not publish any working plan. Both new flag assignments were removed; actual evaluated plan/probe checkpoints keep the flag. The integration test now also asserts that failed scaffold certification cannot mark an unrelated plan evaluated. The third interrupted run is archived under `interrupted-diagnostic-rollback/`; earlier evidence-scope fixes were valid but did not account for this separate rollback path.

Viewer follow-up: live retained scaffold checkpoints previously emitted scalar progress only, while generated-copper snapshots filtered out `track_bezier`. The engine now publishes throttled replaceable copper scenes at safe checkpoints, with forced final/cancel publication; telemetry preserves native Beziers. Publication stays non-authoritative and cannot change routing if an observer fails. The latest generated scene replaces the previous scene rather than accumulating by revision.

Follow-up experiment in implementation: initial bounded board/layer map → erosion → connection-demand pressure. Inspection established that terrain field evolution itself is endpoint-independent; only path extraction requires source and target. CPU/GPU field-only APIs now expose that reusable stage. The initial shared-guidance integration retains per-net exact legal masks and reads GPU fields back for immutable host-side sampling; GPU-resident cross-pipeline field sharing is not claimed. Build/reuse cost, pressure work and physical invalidation must be measured before claiming a benefit.

The user also proposed layer bands with power/ground planes at greater algorithmic depth. ADR 0174 records a proposed sparse cross-layer pressure graph while preserving canonical physical stackup order and layer-local clearance. KiCad signal/power layer roles are currently collapsed on import, so automatic plane-role penalties require metadata preservation or explicit policy. A per-layer constant elevation alone does not alter lateral gradients; travel/transition costs must express the preference. Cross-layer coupling is not implemented by the current per-layer guidance work.


## Experiment — Cached terrain reachability before pressure search (2026-10-11)

Failure mechanism: the new shared terrain pressure preparation repeats Dijkstra for endpoints on
an unchanged disconnected raster, consuming the global pressure allowance before later reachable
demands. This is a concrete preparation/search inefficiency, not proof of the Tier-1 copper cause.

Research and alternatives: reviewed CBS/MAPF, time-expanded/SIPP planning and roadmap reachability.
The bounded regional exact-assignment no-good loop already implements static conflict learning;
new temporal collision states would not resolve permanent copper overlaps. Voronoi corridor
capacity and terminal escape preservation already exist. The missing cheap mechanism is cached
shared-map connectivity. Sources, repository symbols, costs and limits are recorded in ADR 0175.

Hypothesis: layer-local component labels can reject impossible *advisory pressure searches* in
constant time and preserve the pressure allowance for reachable demands. Linear label storage and
build work are bounded by the existing map cell cap. Component area is diagnostic only; it is not
physical channel capacity, and sampled disconnection does not reject a real routing connection.

Change in progress: immutable no-corner-cut component labels and cell counts, pressure precheck,
disconnected-pair telemetry and regression coverage. Before/after tier conclusions remain pending.

HC-CBS follow-up: the authors' hierarchy retains subproblem paths/constraints/costs and composes
constraint-tree alternatives. Source inspection found Flux's closest mechanisms in
`select_physical_repair_requests`, `merge_residual_cone_checkpoint`, the regional no-good master
and survivor-preserving scaffold transactions. The missing mechanism is a bounded, scene-scoped
hierarchy of retained alternatives across calls. A static adaptation needs no time axis. Retaining
only one child winner risks losing a needed parent solution; retaining all combinations risks
memory explosion. ADR 0175 records the lazy bounded composition micro-test. Following explicit implementation
authorization, a static HC-inspired finite child-assignment composer now seeds the regional
master, with equality-guarded retained proposals and adaptive stale revalidation/rebuilding.
It is not equivalent to temporal HC-CBS and makes no optimality claim. Existing exact gates
remain authoritative; measured tier validation follows.


Shared-guidance validation finding: the real-pad integration test caught a production integration
error that field-only tests could not. All-owner terrain extraction passed an empty routed net,
but the extraction API unconditionally required a declared net. Consequently every board-wide
raster was unavailable (enabled=true, cells=0, pressure_demands=1, pressure_cells=0 in the
synthetic fixture), even though global corridor allocation succeeded. The API now requires a
declared net only when omitting same-net copper; all-owner maps retain every pad/copper owner.
The regression asserts both all-owner extraction and continued rejection of unknown nets in
net-specific mode. Before this fix, the consolidated suite passed 16/17 tests; rerun pending.

## Experiment — Explicit multilayer cell and via graph (2026-10-11)

Failure: inspection confirms that prepared NavMesh domains and their portals are planar.
Global routing changes layers on a separate sampled grid; regional alternative generation
enumerates via pairs outside planar search. This can separate useful corridor choice from
legal transition placement. It is a representation gap, not yet a measured sole Tier-1 cause.

Hypothesis: a bounded graph referencing immutable cells on every copper layer in a regional
window can choose planar portals and certified via transitions together. Preserve physical
stackup and complete barrel occupancy, including intermediate layers. The initial consumer
retains existing template and physical admission checks. See ADR 0176 for sources, invariants,
verification and the distinction between layered connectivity and unrestricted substrate travel.

Implementation and benchmark verification are in progress. No routing-completeness or runtime
improvement is claimed until the synthetic mechanism and sequential telemetry tiers are checked.

Multilayer implementation: one graph per ordinary regional fragment includes all copper-layer
islands in the window, with exact full-span certified directed via samples. Search requires two
transitions; the chosen target-layer cell corridor is materialized by closing all outside cells
in the existing funnel. Its via pair and layer receive first consideration in the unchanged
physical template pipeline. The graph API supports general transition counts; the initial
consumer remains limited to two-via overpasses. Ordinary alternatives and local bypass waves
remain available. Added default-enabled ablations and graph/HC counters to CLI JSON.

Independent agent review caught a stale-cache index remapping error and unchecked child-cost
addition in HC; both were corrected and stale-cache mapping received a regression fixture.
Memory review also replaced broad multilayer defaults with 65,536 graph nodes and explicit
262,144-state/frontier bounds. A new core fixture requires the regional pipeline to consume
a graph-selected corridor, complementing the module's isolated connectivity tests. Consolidated
release/strict validation and sequential tier measurements remain pending.

### 2026-10-11 — Multilayer/HC integration verification corrections

Initial focused validation passed 17/19 executables. Layered path costs omitted lateral travel
to via sites, and the production fixture prepared/searched graphs but consumed zero selected
corridors (1,760 nodes, 1,614 expansions). Corrections charge complete via-site travel, preserve
open incident endpoint cells during constrained flood fill, and certify via samples before
retained-site limits. A boundary-binding micro-test also rejects escape from a closed cell.

HC integration exposed an independent admission defect: pairwise compatibility did not prove
aggregate channel capacity. HC seed admission now uses the existing exact aggregate via/channel
check before coupled/access checks. The historical forward-checking ablation explicitly turns
HC off; the aggregate-capacity fixture retains HC enabled. The public enumeration API continues
to honor configuration. Consolidated verification and tier benefit remain pending.

The corrected production crossing regression now passes, including an assertion that a
certified multilayer corridor is consumed by template construction. The shared-boundary
binding and closed-cell rejection checks pass in the same fixture. Full focused-suite and
tier validation are still pending; no completion or speedup claim follows from this fixture.

Focused consolidated validation: 19/19 CTest executables passed in 105.56 seconds, including
all 67 core cases, layered navmesh, HC composition, GPU terrain evolution, physical scaffold,
engine policy/preparation, and telemetry viewer/live-client tests. Log:
`reports/validation/joint-physical-20261011/layered-hc-final-tests-v4.log`.
The only subsequent source change replaced two const-value LayerId loop variables with
const references to satisfy warnings-as-errors; strict CLI/viewer verification follows.

### Sequential telemetry benchmark — Tier 0

`pt2399_delay`: planning 111.348 s (baseline 110.914 s), detailed routing 317.100 s
(baseline 313.399 s), peak RSS 1,406,712 KiB (baseline 1,382,832 KiB). Checked scaffold
50/50 connections; 0 opens, 0 internal copper violations. The rejected diagnostic board's
KiCad DRC reports 0 violations and 0 unconnected items. Strict curved qualification remains
failed on the same 7 unrounded non-junction bends; this is not a promoted tier.

Both final F.Cu/B.Cu PNGs are byte-identical to the baseline. Visual inspection found no new
looping or oscillatory curves; straight runs, obstacle detours, and the existing abrupt joints
remain. The saved intermediate spline close-up contains only a straight candidate, so it does
not demonstrate multilayer search quality.

The board terrain producer is now active: 7,668 cells on 2 layers, GPU evolution used,
cache reused, 100 pressure paths, 23,116 pressure-search expansions, no pressure budget
exhaustion. Final regional HC/layered counters are zero but describe the retained checkpoint,
not cumulative work. A read-only audit found cumulative layered stats already collected but
not emitted, while HC lacks a cumulative plan field. A proposed observability patch was deferred after audit: existing cumulative candidate
metrics follow selected-checkpoint ancestry and also omit discarded sibling attempts.
An execution-owned accumulator is required for reliable whole-run totals. No benefit is
attributed from those zero fields; the benchmark source remains unchanged.

Tier 1 follows sequentially as a diagnostic comparison because Tier 0's strict curve failure
and final copper are unchanged from the recorded baseline.

### Remaining Tier-0 mechanism

Failure details identify five generated PH chains, not inherited source copper (the input has
zero tracks/vias): net 3/F.Cu bases 13 and 14; net 17/F.Cu base 32 and B.Cu base 33;
net 2/F.Cu base 59. Four fail NavMesh endpoint/free-space admission on the initial pass;
one fails trajectory. Retry failures include exact offset validation. The two passes withhold
nine corners each, while final junction-aware qualification counts seven sharp joins.
The net-17 endpoint `(34.8, 65.4)` mm is exactly its own via site. The net-2 chain is a closed
three-edge loop. Neither observation alone proves the required fix; next isolate terminal
access-domain handling and whether that cycle carries any mandatory attachment. Do not relax
clearance or the strict curved qualification threshold.

### Sequential telemetry benchmark — Tier 1 and interpretation

`bm_svf`: global 851.584 s versus baseline 836.222 s; runner wall 14:14.72;
peak RSS 1,445,416 KiB versus 1,685,768 KiB (14.3% lower). Abstract routing completes,
but the engine exhausts with only 62/78 checked scaffold connections, 16 missing,
740 published tracks and 24 vias: unchanged physical connectivity. Detailed copper
and a final KiCad DRC certificate were not produced. Runner global success is not tier success.

The retained scaffold reports 8,749,598 expansions, 189 victim exchanges and 128 coupled
exchanges, with no accepted exchange. Selected repair records repeatedly say
“selected global paths contain no regional physical conflicts”, despite missing scaffold
connections. This identifies an integration limit: conflict-triggered regional alternatives
cannot necessarily address a failed physical attachment/search with conflict-free abstract
paths. Next experiment should construct regional requests from failed physical connections
and their actual blocking owners, rather than rely solely on conflicts in selected global paths.

The layered graph is retained as a tested bounded implementation (including a production
corridor-consumption fixture); these benchmarks do not establish a routing benefit.
No tier is promoted. Final regional counters are checkpoint-scoped and cannot establish
whole-run invocation totals. The broader engine-owned telemetry accumulator is deferred.

Release CLI and strict CLI/viewer builds passed. The frozen binary/source manifest verified
without drift after both sequential runs. Machine-readable comparison:
`reports/benchmark_history/2026-10-11-joint-physical.json`.

### Experiment — Failed physical demands as regional repair inputs

Failure: Tier-1 retained scaffold satisfies 62/78 requests; 12 failures exhaust expansion
bounds and four exhaust attachments. Every missing request has 6–32 attributed blocking
objects. Yet physical regional attempts report no conflicts in the selected global paths.
Hypothesis: regional admission depends on intersections of coarse paths, so a physical
connection failure with different blocking geometry never reaches alternative generation.
Construct bounded requests from missing physical demands and their current owner evidence,
retain immutable unrelated copper, and preserve exact admission/connectivity gates.

User-provided KRT oracle: 25/25 nets, 78/78 pairs, 7.30 s, 699,298 expansions, 615 segments,
73 through-vias, 925.353 mm copper, no rule escalation; internal represented DRC zero and
two permitted same-net crossings. This is useful topology evidence, not an independently
rerun KiCad DRC certificate. Structured reference: `reports/benchmark_history/2026-10-11-krt-user-reference.json`.

Primary source inspected: KiCadRoutingTools commit
`0db4b103f95a63995e8e2c2b2c8c0e3077a2388a`, `rust_router/src/router.rs` via expansion.
It explores layer transitions and treats existing same-net holes as zero additional via cost.
Flux already considers transitions at every expanded scaffold cell with a modest four-step
via cost. Fewer final vias alone therefore do not establish excessive via pricing: incomplete
connectivity and bounded search state growth also matter. Evaluate constraints and opportunity
generation before changing a cost constant. No external source code is copied.

Via audit: retained Tier-1 JSON reports 60 planned via transitions but only 24 published
scaffold vias; the regional selected checkpoint reports zero planned via transitions. This
locates an important loss after global allocation. Scaffold considers every layer at every
expanded cell (no sparse via lattice); new-via cost is four orthogonal grid steps. Its exact
via-history identity multiplies search labels while prospective drill-pair spacing is only
checked on assembled goals. A future bounded history range query could reject illegal
new-drill pairs earlier without removing legal alternate histories.

Existing same-net transition reuse currently still pays the new-via cost in scaffold search.
KRT discounts it, but changing Flux to zero also requires correcting the heuristic's layer
transition lower bound. Defer that separate experiment: first measure whether admitting
physical-failure regional requests exposes the already implemented layer alternatives.

### Updated KRT oracle — preserved run

Pinned KRT revision `0db4b103f95a63995e8e2c2b2c8c0e3077a2388a`, native source build,
explicit 0.2 mm tracks/clearance, 0.7 mm vias, 0.3 mm drills, escalation off and strict sizes.
Routed 25/25 nets and 78/78 pad pairs in 4.19 s wall, 724,415 reported search iterations,
70 vias, 352 actual track segments (excluding four copper graphics), 848.751 mm planar
segment length. Peak RSS 312,064 KiB. KRT internal checker: no violations, one permitted
same-net crossing. These costs are not an isolated algorithm ablation against Flux.

KRT wrote copper-to-hole clearance 0.20 mm rather than the original 0.25 mm, despite no
reported escalation. Preserve both projects. A second KiCad DRC using the untouched original
project found zero opens and zero copper/hole-clearance violations. Its 67 other reports
are footprint-library (34), text-height (22), and silk-over-copper (11). Thus the produced
copper passes the original copper rules even though KRT's search configuration was not
strictly identical. Do not conflate this with a completely clean full-board DRC report.

Both copper-layer previews were inspected. KRT uses short opposite-layer bridges and
frequent vias near terminal escapes, rather than requiring long same-layer detours.
Nets with missing Flux requests use 45 of KRT's 70 vias (per-net geometry summary retained);
this is evidence for exposing physical layer-change alternatives, not a target via count.
All output, commands, hashes, DRC, renderings, and per-net copper are preserved under
`reports/validation/krt-tier1-20261011/`.

Frozen Flux CLI also imports the KRT board with original project minima applied, strict/non-lossy
import, zero core validation errors and zero open connections (`flux-inspection.json`).
This rules out a basic format/connectivity-model mismatch as the explanation for this gap.

Implementation: dedicated `RegionalPhysicalRepairRequest` carries current net/connection
identity and owner IDs. Regional windows include the failed demand's actual fragments and
conservative bounds of resolved blocking copper; owners remain immutable board geometry.
No synthetic owner paths, all-net-pair conflicts, or exact-conflict claims are introduced.
Source and resolved owner counts distinguish stale references. All proposed replacements
still pass existing exact scaffold connectivity/DRC admission. Source freeze for focused
regression and build validation; benchmark benefit remains unverified.

Validation correction: release compilation passed. The strengthened engine fixture initially
failed attribution because its zero-expansion scaffold configuration intentionally skipped all
collision probes. The fixture now places the blocker near the source and permits 128 expansions,
so it must collect real owner evidence while remaining unable to finish the 60 mm request.
This changes the test mechanism, not production routing rules or acceptance criteria.

The first positive-budget fixture attempt still missed its obstacle because the global
planner oriented the request from the opposite endpoint. Diagnostics showed 80→20 mm and
128 expansions with no frontier contact. Symmetric endpoint-near blockers now make evidence
collection independent of demand orientation; production attribution is unchanged.

Read-only follow-up: four attachment-exhausted Tier-1 request endpoints are not pads. Their
nearest canonical pad distances are 5.293 mm (net22), 2.719 mm (net16), 2.376 mm and
1.956 mm (net1). `scaffold_requests` copies abstract global paths, including branch endpoints.
`scaffold_access` keeps an endpoint absent from copper as a free-space anchor, or substitutes
anchors from its component when it touches existing copper. `attachment_exhausted` means an
empty legal raster-attachment set before A*, and current data does not distinguish those
cases. Next isolated diagnosis: report endpoint component membership plus source/goal anchor
counts in a relocated-Steiner-branch micro-case. Do not assert this is already the proven
cause of all four failures. Source unchanged during benchmark validation.


### Follow-up — physical-demand scheduler and attachment access

The first dedicated-demand build passed 19 focused tests and strict CLI/viewer builds.
Tier 0 retained 50/50 checked scaffold requests, zero diagnostic copper violations and
zero KiCad opens, but still failed strict curved qualification with the same seven
unrounded non-junction bends as the preceding baseline. It is not a tier pass.
Evidence: `reports/benchmark_history/2026-10-11-physical-demand.json`.

A scheduler audit found two independent liveness gaps: physical regional repair required
at least one accepted scaffold path, and newly attributed blockers did not change the
physical retry dependency. The next experiment admits valid missing demands even with
zero accepted paths, and includes canonical missing-request/blocker identity in its retry
key. Blocker ordering, duplicate IDs and search termination alone do not reset attempts;
termination was explicitly excluded after review identified an A/B retry-cycle risk.
Regression coverage checks empty-scaffold dispatch and causal key stability.

Validation and sequential telemetry benchmarks for this follow-up are recorded separately
under `reports/validation/physical-demand-v2-20261011/`; prior binary snapshots remain intact.


Attachment experiment: disconnected same-net objects previously consumed the candidate
cap before exact component membership filtering. Candidate generation now streams fixed-size
membership batches, includes the endpoint in each oracle call (component IDs are call-local),
and caps accepted same-component anchors. Original object ordering and exact admission remain.
A micro-test places more disconnected pads and tracks than the access cap before a useful
track projection. Per-request primary-search membership/anchor/attachment diagnostics expose
whether remaining Tier-1 failures originate at physical components or abstract branch points.
This does not claim to relocate or solve all orphan branch endpoints.

Independent scheduler review also caught global attempt resets on evidence-only updates.
The follow-up preserves attempts for unrelated requests and reopens only changed demands;
actual retained geometry/domain changes still reset all. This prevents a higher-index changed
demand losing the remaining bounded budget to unrelated lower-index requests.


Read-only Tier-0 follow-up during v2 validation: the previous diagnostic has zero copper
violations/opens/validation errors but seven unrounded non-junction bends. The audit in
`detailed_router.cpp` deliberately rejects these; disabling it would hide qualification
failure. Existing exact fillet fitting is gated off when the normal PH candidate budget is
positive, even when residual raw bends remain. A future isolated experiment should report
bend coordinates/source runs and attempt bounded residual-only exact fillets, preserving
DRC/connectivity and retaining failure if a bend cannot be rounded. This is not enabled in
v2, and the aggregate old telemetry does not prove all seven are repairable this way.


V2 validation: targeted scaffold/engine tests passed, then 19/19 regression CTests passed
in 105.51 s; strict CLI/viewer and release builds passed. The first zero-path fixture stopped
on a preparation checkpoint before admission; its wait condition was corrected to observe
actual request admission (no production change). Tier 0: global 110.29 s, detail 311.73 s,
50/50 scaffold requests, 3 vias, 47 tracks + 199 Beziers, zero internal copper violations,
zero KiCad diagnostic violations/opens, same seven residual bends. Every captured PNG is
byte-identical to the previous physical-demand run. Qualification remains failed, unchanged.
Tier 1 is running sequentially with diagnostic telemetry and the same frozen binary.


V2 Tier 1 completed in 287.73 s (preceding joint-physical run 851.58 s), peak RSS 583,664 KiB
(previously 1,445,416), but retained the same 62/78 checked requests. There are 741 tracks
and 22 vias. This is faster failed routing, not a completed board or demonstrated throughput
improvement. Stored repair records admit batches of 6, 6 and 4 missing demands with 18, 18
and 14 bundle invocations respectively; none yields an accepted candidate. Records coalesce
repeated identical keys, so these counts must not be summed as whole-run totals.

The new access evidence confirms requests 30, 35, 66 and 73 terminate before A*: source is
on physical copper, target is absent from copper, and target legal attachments equal zero.
The remaining twelve failures exhaust expansions; all have nonempty attachment sets.
This disproves anchor-cap starvation as the direct cause of those four failures while still
retaining the independently reproduced cap fix. `failure-analysis.json` preserves per-demand
evidence. Final Tier-1 copper/DRC certification is unavailable because routing is incomplete.
The saved provisional regional image was inspected; it is not legal final copper.

### Next isolated experiment — physical terminal demands

Hypothesis: treating an unmaterialized global branch point as a mandatory physical terminal
creates impossible attachment requests. Regional curve repair cannot fix a blocked required
endpoint. Make physical completion refer to real terminal components and keep coarse branches
as routing proposals, with exact connectivity and DRC admission still authoritative.

Candidate ranking: (1) canonical physical terminal/component demand binding; (2) reconstruct
coarse tree chains to real terminals while preserving route geometry; (3) enlarge raster
attachment rings; (4) lower via cost. The last two cannot fix a required point inside forbidden
copper, and nearest-pad substitution alone could silently omit connectivity. Preserve useful
coarse geometry and passing Tier-0 coverage when testing the first option.

Research: KRT's pinned `docs/routing-architecture.md` describes MST demands between real pads
and subsequent taps from existing connected segments to the target pad (Phase 3). Reference:
https://github.com/drandyhaas/KiCadRoutingTools/blob/0db4b103f95a63995e8e2c2b2c8c0e3077a2388a/docs/routing-architecture.md
This is a concept reference; no external source is copied. KiCad similarly finishes routing on
same-net pads/tracks/vias: https://github.com/KiCad/kicad-doc/blob/master/src/pcbnew/pcbnew_interactive_router.adoc.


Physical-terminal implementation is intentionally net-scoped: only nets with selected
endpoints outside actual same-net pad copper (position/layer membership) or failed global
connections receive rebuilt deterministic pad-spanning demands. Unaffected coarse paths are
kept unchanged. A net with fewer than two usable pads keeps its original demands, avoiding
silent omission of imported/copper-only obligations. Affected-net global geometry is not
reused without exact endpoint association. Tests cover a mixed three-pad tree with one orphan
branch and preservation of a no-pad request. The CLI now separately reports physical opens
and unrouted nets from the retained copper connectivity oracle; canonical request counts are
not directly comparable to the prior abstract-branch demand domain.

Source frozen for independent review, focused validation and sequential telemetry benchmarks
under `reports/validation/physical-terminals-20261011/`.


Physical-terminal result: targeted tests and 19/19 regressions passed (105.94 s), strict builds
passed, but Tier 0 regressed from 50/50 to 46/50 checked requests, with four actual opens across
three nets. Global run took 76.24 s; all four failures exhausted search allowance. Whole-net
canonicalization discarded useful selected geometry and changed route ordering despite fixing
the request semantics. Tier 1 started before the regression stop signal and was interrupted;
it is not a valid benchmark result. Decision: revert the canonicalization implementation and
its experiment-only test; retain v2 scheduler/component-access fixes and diagnostic reporting.
Preserve rejected source and artifacts to avoid repeating this experiment blindly.

Next hypothesis requires explicit correspondence between physical terminal obligations and
retained branch geometry (including producer/dependency ordering), rather than replacing the
entire successful subtree with a fresh MST. The confirmed orphan endpoints remain unresolved.
This follow-up was not promoted or hidden behind relaxed DRC or reduced connectivity scope.


Final retained state: all restored engine/header/test files exactly match the v2 benchmark
source hashes. The only source difference is CLI read-only physical connectivity reporting;
removing those added reporting lines reproduces the v2 CLI source hash. Final rebuild,
19/19 regressions (106.04 s), strict CLI/viewer build and release relink passed. Final binary
SHA-256: b38846f2bc61b4e4efd73195e9000546b926703ca2bcff4961cb6e3047400d85.
No extra tier runs were claimed: sequential v2 runs are the retained routing implementation's
benchmark evidence, with provenance in `reports/validation/physical-demand-final-20261011/`.
Neither tier is promoted: Tier 0 still fails strict curve qualification and Tier 1 remains
physically incomplete. The whole-net terminal rewrite was rejected and is absent from active code.


## Experiment — residual corner repair and narrow branch binding (2026-10-11)

### Failure and hypothesis

The retained physical-demand-v2 baseline leaves Tier 0 physically connected and clean
under diagnostic KiCad DRC, but rejects seven non-junction bends. Independent endpoint
inspection of its exported rejected board identifies all seven as straight/straight
corners, not Bezier tangent mismatches. Preserve the strict curve audit and test a
bounded exact circular-fillet postpass after PH realization. Each accepted edit must
preserve connectivity and pass changed-copper clearance checks. Classification and
attempt/accept/reject counters distinguish repaired corners from unsupported tangencies.

Tier 1 remains 62/78 checked requests. Four zero-attachment targets are noncopper coarse
branch points. Exact layer-aware segment containment in canonical request geometry
identifies producer connections: net-22 connection 0 -> connection 2; net-16 connection
2 -> connection 0; net-1 connection 3 -> connection 0; net-1 connection 10 -> connection 2.
Two producers are satisfied despite their selected branch points being absent from
physical copper; two producers also failed. Deferral alone cannot solve this.

The branch experiment will bind only a failed demand to a real pad reached through its
selected same-net coarse tree, excluding the failed branch itself. No nearest-pad
substitution or whole-net MST reconstruction is permitted. Preserve peer requests and
require exact physical connectivity for admission. This refines the existing KRT/KiCad
terminal-component research above rather than introducing a new routing family.

### Baseline and acceptance

Baseline: Tier 0 50/50 checked requests, zero physical opens, seven residual bends;
Tier 1 62/78, 16 missing requests, 287.73 s global runtime. Sources and independent
bend evidence are preserved under reports/validation/residual-repair-20261011.
Run Tier 0 then Tier 1 sequentially with diagnostic telemetry, preserving all rules.
Reject any lower-tier physical regression. Neither tier is promoted without final
curve qualification and KiCad validation. Implementation and verification pending.

### Implemented scope and review

The branch experiment is deliberately narrower than a general coarse-tree graph walk:
only an exact direct producer with a real-pad endpoint qualifies. This covers the four
observed orphan endpoints. Existing source-component membership excludes redundant loop
attachments. Rebound requests clear stale corridor, medial, width and neck-down metadata;
unchanged requests remain byte-identical. A proof is consumed before kernel dispatch
(one attempt per exact canonical producer proof and immutable physical context). A shared
checkpoint predicate prevents changed-domain updates from dropping any checked peer.
Adopted bindings and attempted proofs are reported separately. The existing geometry-only physical dependency key re-enables a proof after copper or
request geometry changes, while history and diagnostic counter changes do not.

Independent review caught and corrected scaffold topology-lock over-exclusion, source
copper under-protection, and missing quantized arc/segment tangent checking. Residual
fillets now target only audited straight nonjunction corners; preserved/locked copper
and closed loops remain untouched. Each transaction validates changed copper and
per-net connectivity under consistent zone-fill treatment. Closed loops and mixed
curve tangencies remain subject to the unchanged final rejection audit.

Focused tests cover legal oblique fillets with actual tangent checks, blocked fillets,
preserved source copper, unavailable/bypassing producers, no proof, consumed proof,
cleared neck-down metadata, source-component loop exclusion, and checked-peer retention.
Sources frozen for compilation and sequential telemetry benchmarks.

Validation exposed an existing source-preservation bug before benchmarking: the PH chain
collector used only native-curve protection and could collect preserved tracks on an
unrouted net. After committing the unintended curve, dimensions.at(net) threw because
that net had no routing dimensions. A debugger confirmed the materialize_impl call path.
The shared full protected-ID set now feeds branch splitting, primary PH collection and
residual repair. Generated scaffold copper stays eligible. The corrected preservation
fixture routes another net and checks all original source segments remain unchanged.
Independent review approved this narrow correction.

### First frozen benchmark result

Authoritative freshly relinked regressions: 19/19 pass (106.08 s); strict CLI/viewer
and release builds pass. Frozen binary SHA-256:
541e505f84f19d6e01a551b27675d028df2e57d28a9d3ffea44d6383f706ff47.
Tier 0: 50/50 checked requests, global 112.04 s, detail 330.27 s, 0 physical opens,
0 represented copper violations, external diagnostic KiCad DRC 0 violations/0 opens.
Residual repair attempted and accepted six corners; remaining bends fell 7 -> 1.
Output has 47 tracks, 199 Beziers, 6 arcs, 3 vias; peak RSS 1,416,664 KiB.
Not promoted: one strict straight-corner audit failure remains.

Independent inspection locates the remaining corner at (24,29.4) on F.Cu, net
Net-(U2-LPF1-IN). Its three generated segments form a return-to-pad triangle:
(23.5,28.58) -> (24,28.58) -> (24,29.4) -> (23.5,28.58), all width 0.7 mm.
The blanket closed-run exclusion skips the entire pad-anchored chain. A narrow next
experiment is to distinguish an anchored return chain (seam is a physical junction)
from an unsupported closed degree-two loop (seam is itself an audited bend).
Inspecting the exported F.Cu image confirms mostly smooth paths with permitted junctions;
this image is diagnostic rejected copper, not an accepted final artifact.
Tier 1 runs sequentially on the same frozen source before any further edits.

Tier 1 first frozen result: 64/78 checked requests (baseline 62/78), 14 missing,
384.27 s global runtime (baseline 287.73 s), 9,178,226 expansions, 6,654,415 candidate
rejections, 754 published tracks, 22 vias, peak RSS 590,976 KiB. Four proof-backed
endpoint bindings were attempted/adopted. All four zero-attachment failures disappeared;
all remaining failures exhaust expansion budgets. Retained physical connectivity reports
24 opens across 10 nets, distinct from the 14 missing abstract requests. No detailed
output/final KiCad DRC exists because the board is incomplete. Regional candidate
selection still produces no accepted repair.

Interpretation: direct terminal rebinding solves the measured impossible-endpoint
mechanism and gains two checked demands, at ~34% higher runtime. Keep as a bounded
experiment; it does not solve the remaining search/packing problem or certify Tier 1.
The next narrow change targets Tier 0's anchored return-chain guard and is isolated
under reports/validation/anchored-residual-20261011.

### Follow-up — anchored closed-chain eligibility

Only two source files changed from the first frozen run: detailed_router.cpp and the
scaffold-curve tests. Closed runs now remain excluded when their chosen seam is itself
an audited straight nonjunction bend. An audit-exempt physical attachment at the seam
allows the interior corners through the existing exact transaction. Added an anchored
triangle and a floating triangle fixture; source snapshots precede the edit. No audit
tolerance, clearance rule, or connectivity qualification was relaxed. Validation pending.

Anchored follow-up validation: focused mechanism tests and all 19 freshly relinked
regressions pass (107.21 s); strict CLI/viewer and release builds pass. Frozen binary
SHA-256 d22f974845649e531502874537683613dcd0053701accde49f9b605fc08439a4.
Tier 0 retains 50/50 checked requests. Global took 114.08 s; detail 321.39 s.
Seven residual repairs were attempted and accepted, leaving zero straight-corner or
curve-tangent audit failures, zero physical opens and zero represented copper violations.
The diagnostic exported board also has zero KiCad error violations and zero unconnected
items under the corpus rule configuration. Both exported copper layers were visually
inspected; no new looping or clearance artifact was observed. This is still diagnostic
copper: the CLI retains incomplete_global_route because materialization was reached only
through the review-preview fallback. The physical repair hypothesis is supported, but
Tier 0 is not promoted. The remaining certification/scheduling gate is a separate issue;
it must be fixed through exact engine admission rather than by relabeling a preview.
Tier 1 is being rerun sequentially against this same frozen binary.

Follow-up diagnosis corrects the initial handoff suspicion: route-kicad and detail-kicad
are independent runs. In Tier 0, the exact scaffold oracle itself rejects regenerated
scaffold paths with one open, matching retained copper connectivity. The isolated pad is
R1.2, net kicad-net-16 / Net-(J1-Pin_1), at (22.5,75.74) mm; canonical request 1 instead
starts at (22.5,55.42) mm and ends at Q1.2. Satisfying that declared endpoint graph does
not connect the actual R1.2 pad. The earliest problem is canonical terminal coverage,
not merely preview labeling. Next experiment should add or regenerate only missing
physical terminal demands while preserving satisfied peers, and require the existing
whole-board oracle. Do not revive the rejected whole-net MST replacement experiment.

Final Tier 1 rerun confirms the first frozen result: 64/78 checked requests, 14 missing
(all expansion_budget_exhausted), four proof-backed rebindings adopted, 24 physical opens
across 10 nets, 754 tracks, 22 vias. Runtime 385.42 s, peak RSS 589,460 KiB; 9,178,226
expansions and 6,654,415 candidate rejections. All 166 victim exchanges and all regional
repairs still fail to produce an accepted repair. No detailed/final DRC artifact exists.
Source freeze verification reports no drift. Final history:
reports/benchmark_history/2026-10-11-anchored-residual.json.
Decision: retain verified corner/source-protection and narrow terminal-rebinding fixes.
Tier 1 remains unsolved. Main remaining cost is unsuccessful physical search/repair;
next mechanism to address is exact physical-terminal obligation coverage, followed by
conflict/via-aware alternatives for the remaining bounded-search failures.

## Experiment — bounded route-and-displace (2026-10-11)

### Failure and hypothesis

The retained Tier 1 baseline has 64/78 checked requests, 14 expansion-budget failures,
9,178,226 expansions, 166 victim-exchange attempts and zero accepted exchanges. Search
rejects every currently colliding edge. The failed-demand regional path freezes blocker
copper, so existing XPBD/pair-shove machinery cannot move those owners. A passage that
could become legal after a small deformation is therefore invisible except through
complete victim rerouting.

Hypothesis: bounded speculative passage generation followed by an atomic geometric
shove can recover some connections without rebuilding a victim route. Keep this separate
from the real-terminal coverage defect identified on Tier 0.

### Research and design

KiCad distinguishes walk-around from shove routing, where traces/vias may move and pads
remain obstacles: https://github.com/KiCad/kicad-doc/blob/master/src/pcbnew/pcbnew_interactive_router.adoc .
Flux already has pns_push_and_shove with bounded recursive contact propagation, fixed
source endpoints, source protection, validation, connectivity checks and exact DRC.
Reuse that implementation as the initial backend rather than introducing another solver.
Regional XPBD/AVBD remains available elsewhere; direct XPBD conversion is not part of
this first adapter.

Generate a small number of speculative passages with attributed generated blockers
omitted only from the search snapshot. Restore all original copper before invoking PNS.
Keep source/locked copper protected, retain satisfied request connectivity, rebuild
moved path representations, and commit the incoming plus displaced copper together
only after exact validation. Failure, budget exhaustion or cancellation leaves the
published checkpoint untouched. Track attempts, rejection causes, work and accepted
moves. Baseline saved in reports/validation/route-displace-20261011/baseline-source.
Validation and sequential diagnostic Tier 0 / Tier 1 comparison pending.

Implementation scope: one attributed generated planar owner per proposal; up to four
owner proposals and 20,000 aggregate proposal-search expansions per failed request.
PNS bounds are 16 candidate checks, depth four and 2 mm displacement per proposal.
Source copper, vias and other owners remain protected. Multi-segment nominal-width
planar paths are reconstructed from unchanged segments and PNS doglegs; source-ID
ownership is retained for both. The existing victim-exchange fallback remains available.
The first implementation deliberately does not add an XPBD adapter or move native curves.
Independent review identified and corrected lost ownership of unchanged segments; a
final cancellation guard and extended rotated/multi-segment coverage are pending after
the initial focused build. Initial tests cover positive shove, source preservation,
disabled recovery and retained path reconstruction.

Focused validation initially rejected the shove fixture. Inspection showed a useful
mechanism: seeding displacement from 0.2 mm grid steps makes PNS try 0.2/0.4/0.8/1.6 mm,
which skips the narrow legal movement around 0.9 mm. The adapter now seeds displacement
from the colliding pair's projected copper/clearance envelope rather than the grid.
The test retains its original grid and now covers both orientations with a multi-segment
nominal-width owner, an unchanged segment and retained-path reconstruction. Added a final
stop-token check after all expensive validation and immediately before transaction mutation.

Success criteria: the shove-required synthetic passage completes with exact legal copper
and survives retained reconstruction in both orientations; protected copper stays byte
identical; disabled or failed repair leaves prior copper untouched. Benchmark criteria:
record displacement attempts/acceptance/rejection stages and compare checked requests,
physical opens, runtime and expansions with the frozen baseline. No tier promotion is
allowed from per-request completion alone. User explicitly requested this machinery as a
general capability; if Tier 1 does not benefit, report that outcome rather than claiming
performance improvement. Independent v2 review found no remaining material issues.

Validation: revised focused scaffold tests pass (2.21 s); all 19 freshly rebuilt
regression targets pass (106.52 s). Strict release CLI and viewer builds both pass with
warnings treated as errors. Independent review reports no remaining material issues.
Sources remain frozen for sequential Tier 0 then Tier 1 diagnostic telemetry runs.

Tier 0 frozen result: global 112.38 s (baseline114.08), 50/50 checked requests and
one physical scaffold open, zero displacement attempts. Detailed 324.39 s (baseline
321.39), zero opens, zero unrounded bends and zero internal copper violations. Exported
diagnostic KiCad DRC again reports zero errors and zero unconnected items. Qualification
still fails with incomplete_global_route, matching the immediate anchored-residual
baseline. The runner correctly flags this against the older qualified corpus expectation;
it is not a new regression introduced by this patch. Tier 1 continues sequentially.

### Final benchmark and decision

Tier 1: 64/78 checked requests, 14 missing, 24 physical opens across 10 nets, 754 tracks,
22 vias; unchanged connectivity from baseline. Runtime 372.91 s versus 385.42 s and peak
RSS590,856 KiB; do not infer a speedup from this single run, since regional scheduling
and runtime vary. The retained checkpoint records 14 displacement proposals, 13 proposal
search failures exhausting the allowance, one rejected PNS shove, zero accepted shoves,
zero validation/reconstruction rejections and 260,000 extra proposal-search expansions.
Total checkpoint expansions9,438,226 versus9,178,226 baseline. Counters belong to the
retained scaffold checkpoint, not an aggregate over every discarded engine attempt.
All remaining requests terminate with expansion_budget_exhausted. No Tier 1 final copper
or external final DRC exists. Source-freeze verification reports no drift.

Decision: retain the bounded PNS adapter as the user-requested general capability; its
positive, rotated and retained-reconstruction micro-tests pass, but Tier 1 benefit is not
demonstrated. Preserve the unchanged exact acceptance gates. Next evidence-backed extension
is better passage generation or a small coupled-owner neighbourhood: thirteen searches
still cannot produce a candidate with only one owner omitted. Native-curve/multilayer
movement and direct XPBD/AVBD displacement adapters remain outside this first backend.

Authoritative structured comparison: reports/benchmark_history/2026-10-11-route-displace.json.

## 2026-10-11 — Retained blocked passages and coupled displacement

### Failure and hypothesis
The frozen single-owner displacement experiment retained 64/78 Tier-1 scaffold connections,
with 14 missing and no accepted displacement transaction. Thirteen of fourteen retained
attempts exhausted proposal search; removing one owner still leaves coupled passages blocked.
Hypothesis: retain paths through a bounded pool of generated copper, attribute their actual
contacts, and legalize the incoming path and connected owner group together.

### Research and experiment
XPBD's compliant constraint formulation supplies coordinated positional proposals
(https://matthias-research.github.io/pages/publications/XPBD.pdf). KiCad's shove machinery
explicitly assembles obstacle clusters and processes interacting lines
(https://docs.kicad.org/doxygen/classPNS_1_1SHOVE.html). These support bounded coupled
contact handling, but neither approximate residuals nor a local shove certifies a board.
Flux reuses its existing XPBD, AVBD and PNS implementations; it does not port external code.

Candidates are ranked by unmet clearance after estimated displacement space, movement cost,
and deterministic identity. Static/source copper remains hard. A bounded planar generated-owner
bundle is sampled, relaxed with XPBD then AVBD, and offered to local PNS contact handling.
All changes must pass the whole-transaction exact copper and connectivity gate before publication.
The controlled micro-test disables legacy exchange and single-owner repair and requires both
blocking owners to change. Frontier comparison is against `route-displace-20261011`, with
sequential telemetry Tier 0 then Tier 1; inherited qualification failures remain failures.

### Results
Validation and benchmark results follow below.

### Mechanism result before tier evaluation
The first coupled XPBD/AVBD proposal reached the exact gate but failed the two-owner
crossing fixture. Segment-local separation did not choose the distinct escape sides around
the incoming route endpoints. The refinement adds a deterministic owner-normal escape seed,
localized by the incoming tangent interval and a tapered run-up. Both solvers still run;
the lowest clearance-residual seed/XPBD/AVBD proposal is retained for exact validation.
Board-edge constraints account for the actual copper-to-edge rule rather than applying
net clearance a second time. The focused tests now pass (2 targets, 2.97 seconds), including
both owners changing, exact DRC, retained replay, source protection, caps, cancellation,
and nonfatal AVBD refinement failure. This establishes the mechanism, not tier completion.

### Final validation and sequential benchmark comparison

All 20 selected regression targets passed (107.29 s). Strict release CLI and viewer
builds passed. Independent review found no remaining material correctness, ownership,
or budget defects. The production source snapshot and executable were frozen before
running Tier 0 followed by Tier 1, both with telemetry.

Tier 0 matches the immediate baseline: 50/50 scaffold requests, one scaffold physical
open, then zero final opens, zero internal copper violations and zero unrounded joints.
Diagnostic KiCad DRC reports zero errors and zero unconnected items. It remains
unqualified with `incomplete_global_route`; physical cleanliness does not override this
inherited qualification failure. Global/detail runtimes were 113.47/328.40 s.

Tier 1 remains 64/78 requests, 14 missing, 24 physical opens across 10 nets, 754 tracks
and 22 vias. Runtime was 385.82 s versus 372.91 s baseline; peak RSS was 591400 KiB.
Retained-checkpoint expansions increased by 280000 to 9718226. Two blocked candidates
were retained and two groups attempted, but neither reached XPBD under the configured
obstacle-work allowance. Both groups contained three movable owners. No transaction
was accepted. The 16 budget events comprise 14 search caps and two adapter caps.
These counters describe the retained checkpoint, not every discarded engine attempt.
There is no qualified Tier-1 final copper or final KiCad DRC result.

### Bounded diagnostic budget experiment

Replayed the supplied incoming paths for requests 67 and 69 against generated copper
from the retained 64-request telemetry checkpoint, using their attributed owner groups.
This isolates adapter preparation; it is not a full engine rerun or a replay of every
soft-search alternative. The groups contain 201 and 164 particles, below the particle
cap. At 16384 contact/work allowance, both stop at the XPBD obstacle-work guard.
At 32768, 65536 and 131072, preparation admits 687/750 obstacle segments and XPBD runs
32 iterations; AVBD also runs, discovering 117/241 contacts. Request 67 exceeds the allowed deformation envelope; request 69 returns
an approximate proposal whose materialized copper has 27 exact-check violations.
Each larger-cap invocation processes both requests in approximately 0.44 seconds.
The diagnostic does not execute the parent PNS transaction fallback, so it does not
prove that fallback cannot rescue a proposal. It does show that more relaxation budget
alone has not established legal copper. Zero reported violations when no proposal was
returned must not be interpreted as success.

Decision: retain the requested bounded coupled-transaction capability and its positive
regression; retain current safety caps. Tier-1 improvement is not demonstrated. Next
experiment should localize movable owner spans and their obstacle neighborhoods, then
measure whether coordinated motion plus residual PNS produces an exact legal transaction.
Avoid treating whole long owners as uniformly movable or raising caps without physical
progress. Vias, source copper and non-planar owners remain fixed in this implementation.

Structured comparison: `reports/benchmark_history/2026-10-11-joint-passages.json`.
Validation, frozen sources, telemetry and diagnostic sweep:
`reports/validation/joint-passages-20261011/`.

## 2026-10-11 — Compact physical search before path-dependent fallback

### Failure and hypothesis
Tier 1 retained 64/78 requests, with all 14 missing requests exhausting search expansions
and zero assembled-goal rejections. The checkpoint consumed 9718226 expansions. Source
attachments number 25–128 per failing request, and the rich state distinguishes source,
arrival direction and canonical via history at every cell. Hypothesis: this multiplicity
spends bounded search effort on paths with equivalent immediate futures before reaching
useful legal goals. Relaxation cannot compensate for missing candidate paths.

### Research and experiment
Boost's shortest-path documentation describes one distance/predecessor per vertex
(https://www.boost.org/doc/libs/latest/libs/graph/doc/html/graph/algorithms/shortest_paths/dijkstra_shortest_paths.html).
That is applicable to the immutable edge-clearance graph but not a completeness guarantee
for assembled copper with path-dependent drilled-hole constraints. Therefore test a bounded
compact multi-source proposal search, with exact full-path admission and the existing rich
search as fallback. Both passes share the original expansion allowance. Keep soft blocked
passage search unchanged to isolate the experiment. Measure compact successes, fallbacks,
expansions and total physical connectivity, not merely abstract graph coverage.

### Validation plan
Use a frozen retained-scene replay of failed requests for an on/off comparison, followed
by focused regression and independent review. Run the curved Tier 0 and Tier 1 benchmarks
sequentially with telemetry. Retain only evidence-backed gains; never weaken exact gates.

### Isolated retained-scene replay
All 14 failing canonical paths were reconstructed from request evidence and searched
independently against all 776 retained generated tracks/vias plus the original board.
Exchange/displacement recovery was disabled. The first comparison solved 0/14 with
compact search disabled and 12/14 with it enabled, each accepted output passing exact
copper checks. Total expansions fell from 1400000 to 296628; aggregate time was 7.888 s
versus 8.526 s, so this is a reachability gain, not a demonstrated runtime speedup.
The old search reused many edge checks across labels; compact search visits more distinct
physical edges. The 12 successes needed 1152–18001 expansions each. Requests 63 and 69
still exhausted the shared 100000 expansion limit. These independent routes have not yet
been shown to coexist. An accidental repeated harness invocation reproduced all outcomes;
the structured comparison retains only the first run.

Initial focused validation caught an undersized new moat-test budget (1000 versus the
existing equivalent fixture's 100000); the corrected fixture retains the original bound.
Independent review identified missing cancellation termination evidence in compact search;
the returns now record cancellation and a managed-kernel regression exercises this path.
Baseline commit a649f0f and tag codex/tier1-curved-baseline-20261011 were pushed before
this round, preserving the experiment as a separate working-tree delta.

### Full validation and sequential curved tier results
All 20 regression targets passed in 105.98 s. Strict release CLI built successfully;
independent review's cancellation finding is closed. The final executable hash is
f513ffd0bc6bfce46ebca0e03b4cdaf27835008282f559bf3a109e4fbf754daa.
Tier 0 then Tier 1 ran sequentially with diagnostic telemetry and curved-track settings.

Tier 0: global109.75 s, detail323.83 s; 50/50 declared requests, one scaffold physical
open, then zero diagnostic final opens/violations/unsmoothed joints and 199 Beziers.
KiCad diagnostic DRC reports zero violations and zero unconnected items. Both copper
layers were inspected visually: smooth bends without new looping artifacts. Qualification
remains incomplete_global_route, matching the immediate baseline. Compact search did not
run for this tier: the missing physical terminal is not in its declared demand domain.

Tier 1:76/78 checked requests,13 physical opens across7 nets,829 tracks,54 vias.
Baseline:64/78,24 opens across10 nets,754 tracks,22 vias. Global runtime329.07 s versus
385.82 s. Retained-checkpoint expansions1349002 versus9718226; these are not aggregates
over discarded engine attempts. The retained checkpoint records34 compact attempts,
6 compact successes and28 fallbacks; earlier accepted compact work can be carried as
retained paths rather than counted again. No displacement/exchange transaction succeeded.
The coarse result says complete and the frontier runner exits0, but the routing engine
exhausted and the physical scaffold is incomplete: Tier 1 is NOT passed. No final curved
Tier-1 board or final KiCad qualification exists for this round.

Remaining requests60(net12,connection9) and69(net1,connection6) both have physically
absent target endpoints with one legal grid attachment and terminate on expansion caps.
Request69's orphan is exactly on request63's original coarse path, supporting explicit
producer/component attachment rather than retaining a stale literal coordinate. The13
physical opens despite only2 missing declared demands show additional physical coverage
work is required. Next hypothesis: repair physical terminal obligation coverage and
rebind orphan branches using their proven producer relationships, preserving checked peers.

Decision: KEEP compact search. The matched replay and full benchmark support the state
multiplicity hypothesis. Do not increase its default allowance:50000 compact expansions
failed to solve the two remaining isolated baseline requests and only increased cost.

Artifact note: a runner path mistake overwrote the earlier joint-passages local frozen
executable during preparation. Original benchmark outputs were unchanged. Its source
manifest and source snapshot were restored from baseline commit a649f0f; the replacement
binary was isolated and the unavailable original binary is explicitly documented in
joint-passages-20261011/artifact-integrity.json. Git source/tag isolation remained intact.
Structured round history: reports/benchmark_history/2026-10-11-compact-search.json.

## Round 02 — Rebind budget-exhausted orphan branches

### Failure and hypothesis
Round 01 (`6a9ae7f`) retained 76/78 requests and 13 physical opens on Tier 1. Requests 60 and 69 exhausted search toward F.Cu points absent from copper, although each had one legal grid attachment. Exact canonical segment containment identifies satisfied producer requests 52 and 63 respectively. The former zero-attachment-only trigger mistook grid accessibility for meaningful electrical attachment.

### Ranked approaches and research
1. Rebind using exact coarse producer provenance plus verified physical producer connectivity (selected).
2. Append missing physical component obligations without replacing checked canonical demands (next separate experiment).
3. Increase search budget (not selected: it does not repair stale target semantics).
KiCad's [routing documentation](https://docs.kicad.org/7.0/en/pcbnew/pcbnew.html) describes connections to actual pads/tracks/ratsnest anchors. Flux retains its stricter producer proof rather than adopting nearest-pad substitution.

### Change and safety
Expansion-budget failures may now use the existing producer rebind only when the target is absent from copper and the exact same-net, same-layer coarse producer is both retained as satisfied and physically connected under the canonical connectivity query. The replacement remains a real producer pad outside the source component. Existing zero-attachment behavior is unchanged. Focused tests cover stale satisfaction evidence, unchecked producer, physically present target, exact provenance, and loop rejection. Independent review found no material issue.

### Validation
In progress; artifacts isolated in `reports/validation/branch-rebind-round02-20261011`. Tier benchmarks will run sequentially with telemetry and curved geometry. Full physical connectivity and final KiCad DRC remain the goal; request completion alone is not promotion.

Tier 0 completed: global 107.589 s, 50/50 declared requests and one physical open; detailed 324.585 s, zero opens and internal copper violations, 199 Bézier curves and zero unrounded joints. Diagnostic KiCad DRC reports zero violations/unconnected items. Both copper-layer renders were inspected and show baseline-equivalent smooth geometry. Qualification still reports `incomplete_global_route`; this pre-existing mismatch is not a new regression and is not promoted as a passing tier. Peak RSS 1,455,040 KiB.

### Round 02 final result and decision
Tier 1 reached 78/78 declared requests with 10 physical opens across 7 nets (round 01: 76/78, 13 opens). Retained copper contains 853 tracks and 56 vias. The engine timed out at 900.203 s versus 329.070 s previously; peak RSS rose from 997,624 to 1,382,404 KiB. Five terminal rebinds were recorded, including the two newly admitted request/producer pairs 60/52 and 69/63. The final checkpoint has zero new search expansions because it reused checked paths; this is not the total run search cost.

The semantic hypothesis is supported: actual physical opens decreased, not merely request counts. Retain the correction, but performance and completion remain unresolved. All declared requests being satisfied leaves no missing-request basis for targeted scaffold repair, while whole-board certification still rejects the ten physical opens. The engine consequently spends the remaining allowance exploring global/regional alternatives. Next experiment: append residual physical pad-component demands while preserving every checked canonical request and avoiding duplication of pending edges. Do not promote Tier 1 or claim final curved-copper/DRC acceptance.

## Round 03 — Add residual physical component obligations

### Failure
Round 02 (`faba2a1`) completed 78/78 declared Tier-1 requests but left ten real opens across seven nets. The engine spent its remaining 900-second allowance in global/regional search because the missing-request list was empty. Tier 0 exposed the same mismatch at its scaffold checkpoint.

### Hypothesis and experiment
After all existing requests are checked, derive the actual pad-bearing copper components and append deterministic connecting obligations only for residual disconnected components on represented nets. Preserve the canonical request prefix and exact seed reuse; never replace the route tree. Do not append while any request is pending, which prevents duplicate obligations without speculative contraction. Exact physical search and whole-board oracle remain unchanged.

Ranked alternatives: (1) additive residual component requests (selected); (2) add only uncovered pad endpoints (insufficient when all endpoints are covered but physical trees are separate); (3) replace every net with a fresh MST (previously regressed Tier 0 and discards checked work); (4) more global search budget (round 02 evidence shows the wrong representation remains). This adapts canonical connectivity/ratsnest concepts already researched from KiCad documentation in round 02.

### Verification plan
Micro-tests cover two disconnected checked trees, stable prefix/fresh identities, successful physical closure, idempotence, pending-demand preservation, and coincident front/back pads without a via. Engine test uses an additional physical-source terminal missing from the planning board and requires exact scaffold certification. The independent rejected-oracle fixture remains separately covered on an unrepresented net. Sequential Tier 0/Tier 1 telemetry benchmarks follow a strict build and independent review.

Independent review found the initial all-pairs edge table and repeated edge scanning scaled poorly. Replaced it before benchmarking with an on-demand Prim frontier (linear frontier storage, each component pair inspected once) and a shared one-million pad-pair comparison bound. An unfinished net contributes no suffix. A 32-component regression checks the exact 496-comparison boundary and atomic rollback at 495 comparisons.

### Round 03 results
Tier 1 reached a certified physical plan in 105.439 s: 88/88 requests, zero physical opens/unrouted nets, 910 tracks and 67 vias. This closes all ten residual connections and avoids the round-02 900-second exhausted search. Tier 0 reached 51/51 and zero opens in 18.244 s (repeat 18.641 s).

The original Tier-0 runner rejected 51 paths against its fixed expected50 count before detailed validation. Added a separately reported `expanded_physical_demand` structural-count alternative only for detailed-completion tiers with a complete physical plan, all expanded requests satisfied, and zero physical opens/unrouted nets. All final detailed/DRC/KiCad gates are unchanged; ten runner tests and independent review passed. Preserved the first run and its frozen source manifest; supplemental Tier-0 validation uses `residual-demands-round03-final-20261011` with the same CLI hash.

Both boards now fail detailed materialization with generic `invalid_path` before curve completion (Tier1 105.849 s; Tier0 18.490 s). This is a real detailed-output regression, not a qualified pass. No final KiCad DRC was reached. Evidence points to a handoff mismatch: scaffold compact/grid paths carry explicit nominal segment widths, whereas detailed materialization requires any explicit positive width vector to contain a narrower-than-nominal segment. Its generic diagnostic mislabels all materialization failures as an invalid layer transition.

Decision: preserve this experimental checkpoint because exact physical connectivity improved to zero opens, but do not promote either tier or claim preserved curved output. Next round must correct nominal-width handoff semantics and validate actual curves and DRC.

## Round 04 — Normalize explicit nominal width metadata

### Failure and hypothesis
Round 03 certified both scaffolds with zero physical opens, but detailed materialization rejected paths before curve conversion. Search explicitly records the nominal width for every edge; the consumer treated any positive width vector as neck-down metadata and required at least one narrower segment. This is a representation contract defect, not evidence of an impossible via transition.

### Experiment
Interpret aligned nominal widths (including implicit-zero entries), nominal kinds and zero neck-down counters as the existing nominal-width policy. Keep metadata alignment, actual layer transitions, non-nominal width checks, exact copper validation and final DRC gates. Improve the rejection diagnostic to identify net and connection without claiming every failure is a layer transition.

### Verification
A certified multilayer A/B fixture compares explicit and implicit nominal copper. Negative fixtures retain malformed metadata and non-coincident layer-transition rejection. Validation and sequential telemetry tier runs are recorded in `reports/validation/nominal-width-round04-20261011`; results pending.

### Round 04 results and decision
Strict build and four focused suites passed; independent review closed an additional fail-open case for unbacked neck-down metadata. The intended historical red-test probe was inconclusive because an initial test syntax failure delayed linking until after the library rebuilt; do not count it as a red reproduction.

Tier 0 is qualified again: global 18.191 s, detailed 63.604 s, zero opens/internal violations, 394 Béziers, 23 arcs, 3 vias, zero unrounded non-junction bends, and external KiCad DRC passes. Both copper layer images were inspected: smooth bends, with some broad detours left for later optimization.

Tier 1 passes the metadata handoff and preserves zero opens/internal violations, but fails the curved-output gate after 273.286 s: 396 Béziers, 34 arcs, 67 vias, 103 remaining straight bends. Global scaffold takes 104.633 s with 88/88 requests. Residual bend repair accepted all 34 attempted corners, suggesting coverage/budget rather than local infeasibility is now the first mechanism to investigate. Keep this narrow contract fix; do not promote Tier 1.

## Round 05 — Resume residual bend repair in bounded batches

### Failure and evidence
Round 04 (`3353b6f`) produces zero-open, diagnostic KiCad-DRC-clean Tier-1 copper but retains 103 sharp straight bends. Residual repair accepted all 34 attempts; the routine only scans the first 32 chains and charges 128 points before checking corner eligibility. The separate PH budget is also divided into very small per-chain allowances, but that is deferred to keep this experiment isolated.

### Hypothesis and experiment
Retain the existing exact fillet transaction and global arc/candidate ceilings. Replace the one-prefix residual scan with finite batches that resume monotonically within and across chains. Charge corner work only for auditable eligible bends. Use current incident endpoints when fitting adjacent corners to avoid extending earlier trims. Expose scan, batch and exhaustion metrics; keep the final curve audit unchanged. This applies existing proven geometry rather than importing a new algorithm.

### Validation plan
Synthetic cases require resumption within a subdivided chain and across multiple chains under one-corner/one-run batch limits. An explicit total batch cap must leave a truthful incomplete result. Existing blocked-corner, locked/source preservation, pad-rooted loop and floating-loop regressions remain required. Then sequential Tier 0 and Tier 1 telemetry benchmarks, with final copper and external DRC verification.

### Round 05 first results
Tier 0 remains qualified (detailed 63.637 s). Tier 1 now completes curved routing in 313.308 s, with zero opens, zero internal violations, zero external KiCad DRC violations/unconnected items, and zero unrounded non-junction bends. Final copper: 550 straight segments, 137 arcs, 396 Béziers, 67 vias. Strict re-import preserves those counts and zero opens/model errors.

Residual coverage visited 49 chains and 397 points in two batches, attempted137/accepted137/rejected0, and did not exhaust its total budget. This strongly supports the coverage hypothesis: the previously missed 103 bends needed no clearance relaxation or additional physical routing. First-run runner exit1 is its explicit promotion sentinel, not a routing failure. Tier1 is promoted to qualified, Tier2 unlocked, and a fresh sequential Tier0/Tier1 mandatory regression run is in progress in `residual-batches-round05-repeat-20261012`. Original inventory78 remains the structural baseline; physically certified demand expansion is separately checked.

Both Tier1 layer images were inspected. Main routes have rounded turns without the previous oscillating loops; redundant same-net stubs/branches and some small staircase-like detours remain quality/performance opportunities, not permission to relax the DRC or bend gates.

### Round 05 repeated qualified regression
The promoted mandatory Tier0/Tier1 run passed with runner exit0 for both. Detailed runtime: Tier0 63.353 s, Tier1 314.896 s. Both again have zero opens, zero internal copper violations, zero unrounded bends and zero external KiCad DRC violations/unconnected items. Export byte identity versus the first run: {'tier0': {'byte_identical_export': True, 'first_sha256': 'ef790b89654918bfc83a8b6334f6c07fb25f7adcd1a272066d60a55c8f7a66d9', 'repeat_sha256': 'ef790b89654918bfc83a8b6334f6c07fb25f7adcd1a272066d60a55c8f7a66d9'}, 'tier1': {'byte_identical_export': True, 'first_sha256': '68027a741ebb62f73f6ac3e3098a13d922839a4a032a0eec03d07a4439488484', 'repeat_sha256': '68027a741ebb62f73f6ac3e3098a13d922839a4a032a0eec03d07a4439488484'}}. Keep the batch fix and promotion. The Tier1 curved-routing goal is achieved; Tier2 is now the next frontier.

## Round 06 — Tier 2 check after Tier 1 qualification

Ran ft2232h_jtag alone on commit 43012e9 with curved routing, diagnostic telemetry, 30 iterations and a 900 s deadline, matching the earlier Tier 2 check. No implementation changes. The engine timed out after 900.235 s (the global checkpoint reports iteration_limit), with peak RSS 658,016 KiB. Abstract routing has 117 paths and 381 overflow. The retained physical scaffold satisfies 88/117 requests, with 29 missing requests, 43 physical opens across 21 nets, 930 tracks and 13 vias. Detailed curves and final KiCad DRC were not reached. Harness exit 0 means the unlocked diagnostic behaved as expected, not qualification.

Of the missing retained requests, 15 have attachment_exhausted evidence and 14 are marked not_attempted; the initial failing requests have zero legal target attachments. All 32 victim exchanges rolled back as incoming_unroutable. Coupled exchanges (18), displacement attempts (14) and joint groups (8) accepted none; four joint actions exhausted their budget and eight had exact rejections. Eight blocked passages report no available displacement space. Primary hypothesis: dense terminal access/escape topology is blocking physical search, and repeated repair consumes the deadline before a complete scaffold. This is not a curve-fitting failure. These checkpoint counters are not necessarily cumulative run totals.

The diagnostic front-copper preview was inspected; it is an incomplete angular preview, not final legal copper. The earlier 930-violation quantized-global oracle is a different representation from this partial scaffold, so no direct DRC improvement is claimed. Preserve the baseline; Tier 2 remains unlocked and unqualified. Machine-readable evidence: reports/benchmark_history/2026-10-12-tier2-baseline-round06.json.

## Round 07 — Geometry-aware terminal access and preparation reuse

### Failure and hypothesis
Round 06 timed out in scaffold repair with 15 attachment-exhausted missing requests and 32 incoming-unroutable victim exchange rollbacks. Failed terminal evidence shows copper-contained targets with one component anchor but no legal grid attachments. Current access offers pad centers only, then searches a 0.2 mm grid neighborhood. Fine-pitch elongated pads can have a legal outward escape beyond that neighborhood even when nearby grid connections clip neighboring pads. This is a candidate-completeness hypothesis, not proof that all 15 terminals have a legal solution.

### Ranked experiment
Add bounded near-edge principal-axis pad anchors, checked against exact pad containment and physical component membership, after the existing anchor prefix. Reuse the transformed pad-shape array within the access call. Prefer this over a globally finer grid or increasing repair retries: it adds the missing terminal states and avoids repeated geometry preparation without changing clearances. [KiCad PCB Editor documentation](https://docs.kicad.org/8.0/en/pcbnew/pcbnew.html) treats optimized pad exits as a distinct routing concern; no implementation is copied.

### Verification plan
A synthetic rotated/off-grid fine-pitch escape must fail center-only attachment but route with the new anchors under exact DRC. Opposite-side coincident pad components remain separate. Preserve existing branch ordering and run Tier 0, Tier 1, then Tier 2 sequentially with telemetry and the same deadlines.

### Round 07 focused and lower-tier verification
The strengthened rotated/off-grid fine-pitch regression confirms all 25 center-neighborhood attachments are blocked, fails against the pre-change library for absent pad-edge anchors, then passes with an exact-DRC-clean searched route. Strict build, four focused suites, and independent review pass. New edge scanning checks cancellation per pad, prefilters by accepted component/layer pad centers, and retains exact membership validation.

Sequential diagnostic-telemetry regressions both pass final curved-output and external KiCad DRC gates. Tier0: global18.683s, detailed65.415s, zero opens/violations/unrounded bends. Tier1: global96.904s, detailed264.433s versus314.896s previously (~16% faster), zero opens/violations/unrounded bends;526straight tracks,131arcs,360Béziers,57vias versus67vias previously. Physical scaffold90/90 includes residual component demands, compared with88/88 before; both certify zero physical opens. These are single-run performance comparisons, not isolated pad-preparation microbenchmarks.

Both Tier1 copper layers were inspected: rounded main routes, with some redundant same-net branches and small detours remaining. Tier2 validation is running against the unchanged900s deadline. No board-specific rules or clearance relaxation were introduced.

### Round 07 frontier result and decision
Tier 2 timed out after 900.182 s with 85/117 satisfied requests and 46 physical opens, compared with 88/117 and 43 opens previously. Peak RSS was 651,452 KiB. No final curves or external DRC were reached. The retained scaffold records one searched connection, 128 source attachments and 4 target attachments; repair/exchange counters remain zero. New legal access states allow search, but its cost now consumes the deadline before repair. This is a Tier-2 performance regression, not a solved frontier.

Keep the locally proven terminal-access fix and Tier-1 improvement as a recoverable checkpoint; the frontier requires bounded search/preparation work and better attachment ranking. Do not increase the deadline to hide the regression. The user also identified visually poor Tier-1 output despite legal DRC: object-level checks confirm generated bare tails at (55.44,55.59), (58.04,52.99), (59.99,51.69), and (66.49,56.89), none attached to a pad, via or zone. The next separate round will remove unnecessary generated topology before curve fitting, preserving real physical terminal connectivity.

## Round 08 — Remove generated bare tails before curve fitting

### Failure and hypothesis
The user's highlighted Tier-1 front-copper image contains real generated dead tails, despite passing connectivity and DRC. An object-level audit finds no physical pad, via or zone at four tips: V_CV (55.44,55.59), V_SUPPLY (58.04,52.99) and (59.99,51.69), and Net-(U5B-+) (66.49,56.89). Distinct stored route endpoints must not be mistaken for required physical terminals. Curve fitting preserves these stale branch anchors and spends work smoothing unnecessary copper.

### Ranked experiment
Prefer a bounded, physically certified generated-leaf cleanup before curve fitting over post-export deletion or changing PH smoothing. Split real same-net junctions, remove only generated bare leaf chains, and preserve actual pad partitions and protected/source/via attachments. Skip ambiguous or specially constrained nets. KiCad's documented dangling-track cleanup is a relevant established concept: https://docs.kicad.org/8.0/en/pcbnew/pcbnew.html. No source is copied.

### Verification plan
Synthetic required T branches with a redundant spur, interior same-net crossings, source/protected contacts and padless preservation; then Tier 0 followed by Tier 1 with diagnostic telemetry, curved output and external KiCad DRC. Inspect the highlighted copper again. Tier 2's Round-07 pre-materialization timeout is unaffected by this later-stage change and remains recorded separately.

### Round 08 verification before Tier 1
Strict build passes. Four focused suites pass, including required T preservation, opposite-layer isolation, finite-width pad-to-via contact retention, source-curve and padless protection, zero allowance and atomic split-output rollback. The initial new-fixture failure was structural: footprints declared explicit courtyards without geometry. Correcting fixture envelope provenance enabled the intended regression; no router change was needed for that failure. Independent review found no remaining material issues after terminal-probe membership and bounded splitter safeguards.

Tier 0 passes in 65.351 s with zero opens, unrounded bends and external KiCad DRC violations. No dead-tail candidates were removed. Tier 1 is running. New cleanup is bounded to 256 candidate transactions and at most 4096 input/output split tracks, with cancellation checkpoints; padless, source/locked/native/coupled/zone/curved nets are conservatively protected. Normal curve-only splitting retains its prior endpoint-based semantics.

### Round 08 result and performance follow-up
Tier 1 passes with zero opens, zero unrounded bends and clean external KiCad DRC. Cleanup examined 12 chains and removed 11 chains/30 straight segments without exhausting its budget. Final copper has 440 straight segments, 104 arcs, 413 Beziers and 57 vias. Front-copper inspection confirms the highlighted redundant tails/hooks are removed; the fourth original tip coordinate is now a connected junction, so endpoint presence alone is not a dangling-tail test. Conservative cleanup leaves other protected/contact-ambiguous excess copper for later work.

Detailed runtime regressed from 264.433 s to 351.669 s. Global/scaffold time stayed near 98 s; materialization increased from 168.26 s to 253.24 s. Trajectory checks rose from 133607 to 245806, while spline tasks decreased from 183 to 179 and accepted route commits increased from 79 to 96. These counters do not isolate cleanup validation from changed curve-search work. Keep as a correctness checkpoint, explicitly not a performance win. Next experiment will profile and eliminate redundant preparation against immutable curve snapshots while preserving geometry and exact final validation. The separate standalone global benchmark adds about 98 s to harness wall time and is not part of the detailed command's 351.669 s end-to-end measurement.

## Round 09 — Reuse immutable spline navigation preparation

### Failure and hypothesis
Tier 1's detailed command spends 253.24 of 351.67 seconds in materialization after dead-tail cleanup. Each independent spline run currently re-extracts and clearance-expands all board obstacles, even when another run uses the same net, layer and width against the same immutable pass snapshot. Reuse that exact geometry to avoid redundant work without changing route choices or relaxing validation. This is a targeted experiment; the existing telemetry does not prove the fraction of materialization spent in preparation.

### Ranked approaches and research
First: cache extracted/offset regions within each immutable spline pass, keyed by net/layer/width, then discard at the next snapshot. Second: measure candidate predicate cost before changing trajectory search. Third: reduce repeated recovery snapshot payloads (about 95% of current telemetry bytes), separately from routing correctness. GEOS documents preparing geometry for repeated operations (https://libgeos.org/doxygen/classgeos_1_1geom_1_1prep_1_1PreparedGeometry.html); the applicable principle is immutable preparation reuse, not adopting another geometry engine or copying code.

### Experiment and validation
Retain per-run NavMesh corridors/search and stable task-order commits. Prepare shared keys once with independent synchronization. Expose preparation build/hit counts and summed preparation time, cleanup time, and curve realization wall time. Check focused suites and sequential Tier 0/Tier 1 diagnostic runs; compare exported copper byte identity against Round 08 as well as exact and external DRC. Report actual detailed-command time separately from the runner's extra standalone global test.

### Review refinements
Independent review identified and corrected two issues before benchmarking: fallback results now accumulate all attempt timings, and cached geometry retention is bounded to twice the shared worker count rather than the number of board nets. Only keys with repeated consumers are admitted, in stable order; the last consumer removes its map reference. Tests compare exact output across cached parallel and uncached serial execution, assert exact build/hit counts across layers, and verify unique net keys produce no hits. Review has no remaining material blockers.

### Separate Tier 2 performance hypothesis
Round07 telemetry reaches global completion near57.97s and a retained85-connection scaffold near65.72s, then spends the remaining deadline in scaffold work. The retained result reports only55 compact expansions and no repair, but these are checkpoint counters, not a cumulative profiler. Generic kernel work grows beyond104000; it cannot be called an attachment-probe count because nested helpers also checkpoint. `search_route` fully enumerates source attachments and then target attachments before searching. Progressive, deterministic endpoint prefixes are a plausible next experiment, but first add separate attachment/map/stub/search timings and cumulative counters. Do not infer that all834s were spent in one lambda from retained telemetry alone.

### Round 09 measured results and decision
Strict release build, all four focused suites and independent review pass. Sequential diagnostic-telemetry Tier0 and Tier1 runs both pass exact connectivity/copper, bend audit and external KiCad DRC. Both exports are byte-identical to Round08, so the measured experiment does not trade route quality for speed.

Tier0 detailed command:65.351→62.543s (-4.30%), peak RSS624136→613308KiB,55 preparation builds/21 hits. Tier1:351.669→336.386s (-4.35%), peak RSS682780→665992KiB,100 builds/79 hits. These are single-run comparisons under the same configuration, not statistical performance guarantees. Keep the cache: it removes counted duplicate geometry builds, saves elapsed time on both boards and stays within memory limits.

Tier1 stage timing: global35.67s, scaffold61.33s across three actions, materialize238.13s. Cleanup costs only1.29s; realization including residual repair costs236.51s. Summed worker preparation time127.95s and summed main/refit trajectory time79.24s are overlapping worker durations, not serial wall-time components. These do not explain all realization time: per-run NavMesh/corridor certification, exact proposal/merge validation and residual filleting still need separate timings. Do not label the remaining238s as trajectory optimization alone. GPU preconditioner solve time was only42.6ms summed in Round08, so accelerating that kernel alone cannot materially shorten the pipeline.

Remaining performance frontier: profile those unmeasured realization subphases, then eliminate repeated exact geometry work in the measured dominant phase. Separately, instrument Tier2 attachment generation and indexed-versus-full-validation fallbacks before progressive endpoint search. This late-stage cache cannot improve Tier2's pre-materialization timeout, so its preserved Round07 result remains unqualified. Telemetry still writes roughly150MB/run with many unchanged recovery snapshots; reduce that separately with replay/reconnect coverage.

## Round 10 — Batched GPU materialization with exact acceptance

### Failure and hypothesis
Round09 Tier1 completes in 336.39s but materialization costs 238.13s. Preparation and trajectory timing are summed worker durations and do not isolate remaining serial work. GPU scoring currently still packs dense basis coefficients on CPU and builds clearance fields using scalar CPU queries. Its dispatch counter incorrectly counts successful CPU scoring as GPU use. Independent NavMesh preparations also screen repeated immutable boundary geometry separately.

### Coordinated experiment requested by user
Implement GPU-resident boundary reuse and batched advisory signed-distance queries; compact quintic span inputs so GPU generates curve samples/derivatives instead of uploading dense CPU bases; shared bounded cross-route NavMesh chord screening; conservative curve-envelope/collision screening retaining exact CPU checks for uncertain cases. Immutable snapshot identity and bounded memory/work are required. No relaxed rules or approximate final acceptance. Existing Vulkan compute interfaces are extended rather than adding a new GPU dependency.

### Validation
Focused CPU/GPU equivalence and boundary/uncertainty/fallback regressions, deterministic concurrent NavMesh comparisons and changed-domain invalidation. Correct actual-GPU counters and expose call durations, cache uploads/reuses, NavMesh time, per-route time, merge time and residual repair time. Run Tier0 then Tier1 sequentially with diagnostic telemetry, curved output and external KiCad DRC. Compare runtime, memory, geometry quality and output; do not assume approximate scoring preserves byte identity or claim speedup before results.

### Implementation boundaries and research
GPU broad-phase screening follows the separation of cheap conservative candidates from exact narrow-phase geometry described in [GPU Gems 3, collision detection](https://developer.nvidia.com/gpugems/gpugems3/part-v-physics-simulation/chapter-32-broad-phase-collision-detection-cuda). Batching and retaining immutable inputs follow the data-transfer guidance in [NVIDIA's best-practices guide](https://docs.nvidia.com/cuda/cuda-c-best-practices-guide/index.html); Flux continues to use its existing Vulkan backend.

The shared Vulkan context retains at most 16 immutable boundary buffers and 32 MiB of packed geometry, matched by exact content. Boundary-relative coordinates permit reuse across query batches and chord/distance pipelines. Advisory distance requests are limited to 16,777,216 edge tests overall and 4,194,304 per dispatch; overload requests indexed scalar fallback rather than performing an unbounded brute-force CPU scan. NavMesh caches are scoped to immutable materialization passes and bounded to 8 domains / 262,144 chord certificates; ready caller threads service bounded queued batches without creating another worker pool. Compact quintic inputs replace dense sample-by-control basis uploads. Capsule tests cover the full Bézier control hull plus requested clearance, require exact endpoint containment, and retain selected-corridor checks; ambiguous cases retain scalar certificates. Final quantized copper remains subject to exact transactional validation and KiCad DRC.

### Focused finding
The existing nested-bend fixture changed from one first-pass commit plus a committed-context retry to two legal first-pass commits, with zero opens/DRC violations. The capsule bound is tighter than the prior midpoint-distance-minus-hull-diameter fallback. The retry regression now disables the new screening for its stale-context control, and separately asserts that enabled screening completes this same fixture immediately. The focused test passes in both modes; this is evidence of reduced conservative rejection, not permission to relax physical rules.

### Review hardening and frozen validation
Review found an empty-proof-vector out-of-bounds path for malformed PH controls. The certificate now rejects nonfinite/unrepresentable control geometry before either scalar or GPU callbacks and requires proof counts to match the route. A regression uses all-clear callbacks to verify malformed input cannot be accepted. The preliminary Tier1 run was stopped, its artifacts preserved under `pre-malformed-guard`, and Tier0/Tier1 restarted sequentially against a freshly frozen executable. The final guarded build, affected core tests and six dependent suites pass; independent re-review found no remaining blocker. The complete 71-test core suite passed before these final narrow input guards.

### Round 10 measured results and decision
Keep. Final frozen build passes Tier0 and Tier1 sequentially with diagnostic telemetry, exact copper validation, zero opens, zero unrounded non-junction bends and external KiCad DRC zero violations/unconnected items. Tier0 detailed runtime62.543→57.843s (-7.51%), peakRSS613308→574984KiB. Tier1 detailed runtime336.386→310.183s (-7.79%), peakRSS665992→566684KiB. These are single-run comparisons. The separate preliminary global harness command took18.055s/95.687s and is excluded from these detailed-command durations.

Tier1 materialization238.134→213.199s (-10.47%). Final copper442straight/105arcs/407Bezier/57vias, versus440/104/413/57 previously. Both layers of both boards were inspected: rounded main paths and ordinary straight runs, no obvious new looping artifact; small local detours and retained stubs remain. Approximate ranking and tighter conservative capsules change copper, so output is not byte-identical to Round09. No shortest-path optimality claim. Tier0 is byte-identical to the preliminary pre-guard run.

Actual Tier1 GPU work:1491 polish dispatches evaluating120704 candidates;3759 envelope dispatches proving77110/88195 envelopes clear;33576 resident NavMesh buffer reuses versus267 uploads;1344031 duplicate in-flight chord requests coalesced. Clearance batches intentionally use indexed CPU fallback54 times when aggregate work exceeds the bound. No envelope/polish GPU failure fallback occurred. Geometry remains device-local and shared between chord and distance pipelines; exact CPU transaction validation remains mandatory.

New timing separates remaining costs: serial exact merge17.697s, residual bend repair32.934s; summed overlapping worker NavMesh111.411s, preparation129.798s and trajectory61.828s. Total per-route refinement590.936s is also a worker sum, not elapsed time. GPU polish batches cost1.303s; accelerating that kernel alone cannot remove most of the remaining213s materialization wall time. NavMesh screening still evaluates8.58billion chord/boundary work items over33843 bounded dispatches (30.117s summed call time), suggesting spatial pruning and avoiding repeated certification as the next measured optimization targets. Investigate expensive CPU refinement/validation separately rather than attributing all remaining time to GPU dispatch.

Tier2 is not rerun: its recorded Round07 failure occurs before materialization. This change does not qualify Tier2 or claim to fix its search/preparation bottleneck. Full evidence and frozen build identity are in `reports/benchmark_history/2026-10-12-materialization-gpu-round10.json` and the compact Round10 validation manifest.

## Round 11 — Spatially pruned chord screening and prepared NavMesh reuse

### Failure and hypothesis
Round10 Tier1 materialization costs213.199s of310.183s end-to-end. NavMesh workers total111.411s; GPU chord screening performs8,578,197,318 boundary comparisons across33,843 dispatches. Resident buffers remove transfer costs but the kernel still scans every boundary chunk. `NavMeshScreeningContext::prepare` also rebuilds each lattice rather than reusing identical prepared domains. Spatial nearest-boundary pruning and exact-key immutable preparation reuse should reduce redundant work without changing legal topology or weakening exact acceptance.

### Experiment and validation plan
Add bounded reusable spatial screening and bounded prepared-domain reuse; preserve guide/config/geometry identity, conservative lower bounds, ambiguity fallback and deterministic boundary ties. Test near-clearance boundaries, intersections, translated coordinates, concurrent cache hits and changed-domain/config invalidation. Then run Tier0 followed by Tier1 with diagnostic telemetry, curved output and external KiCad DRC. Compare actual boundary work, preparation hits, elapsed time, memory and copper. The broad-phase spatial subdivision principle is described in [NVIDIA GPU Gems](https://developer.nvidia.com/gpugems/gpugems3/part-v-physics-simulation/chapter-32-broad-phase-collision-detection-cuda); repeated immutable predicate preparation is described in [GEOS prepared geometry](https://libgeos.org/doxygen/classgeos_1_1geom_1_1prep_1_1PreparedGeometry.html). No external code is copied.

### Implementation and local validation
GPU chord screening uses a median BVH with8-segment leaves over each bounded1024-boundary chunk. Guarded chord-AABB/node-AABB lower bounds visit nearer children first; leaves retain ordinary segment distance tests and stable original-index ties. Invalid/deep traversal falls back to a linear scan. Exact-content bounded host/device caches retain the indexed representation; mode18 shares it for <=1024-boundary inputs and avoids building an unused BVH for larger inputs. Actual segment-test and pruned-pair counters distinguish avoided work from dispatched chords.

The materialization screening context now coalesces and caches identical region/full-config/topology-validation preparations. A separate bounded geometry cache reuses prepared polygons, cutout BVH and topology validation across different guides. Value-owned public fields still require geometry copies; this experiment avoids rebuilding, not all copying. Retention budgets include copied geometry in complete lattices. Review caught allocation-failure publication paths that could strand waiters; optional cache admission now fails harmlessly and common completion always publishes geometry-registration/build failures. All72core tests, six focused cases and six related suites pass on the frozen final release build. Minor review note: retained static geometry/eviction counts remain available in direct context Stats but are not exported into CLI metrics; hit/build counts are exported.

### Round 11 measured results and decision
Keep. Tier0 and Tier1 pass sequential diagnostic-telemetry curved runs with zero opens, exact copper violations, external KiCad DRC violations/unconnected items and unrounded non-junction bends. Final copper is byte-identical to the previously inspected Round10 output on both boards. Tier0 detailed57.843→55.375s (-4.27%); Tier1 detailed310.183→300.426s (-3.15%). These are single-run measurements. Separate preliminary global harness runs17.558s/96.273s are excluded from detailed-command times.

Tier0 screening:371.94million possible segment pairs→15.69million tested (95.78% pruned), NavMesh summed-worker14.778→10.564s. Tier1:8.583billion possible pairs→222.04million tested (97.41% pruned), screening call time30.117→16.296s and NavMesh summed-worker111.411→73.681s. Tier1 materialization213.199→204.242s (-4.20%). GPU dispatch count remains33859 versus33843; spatial pruning reduces per-dispatch comparisons rather than submission count. Minor chord workload variations reflect bounded cross-worker cache scheduling; byte-identical copper confirms unchanged output.

Static prepared geometry reuse produces168hits/50builds on Tier0 and339hits/5090builds on Tier1. Full-config lattice cache has zero hits on both tiers; its correctness/reuse is demonstrated by microtests, but it must not receive credit for this measured speedup. The large Tier1 region-build count suggests rejecting irrelevant free-space components before preparing their lattices as a next experiment. Full-domain cache retention should also be reconsidered for workloads with no repeat keys rather than increasing its budget.

PeakRSS Tier0 rises574984→582124KiB (+6.97MiB), Tier1 rises566684→587560KiB (+20.39MiB). This is a bounded memory tradeoff, not a memory improvement. Remaining Tier1 serial merge17.529s and residual bend repair33.033s are largely unchanged; navigation obstacle extraction/offset preparation129.275worker-seconds also remains. No claim is made that these optimizations address Tier2's preserved pre-materialization timeout. Frozen source/build identity and structured comparison are recorded in `reports/benchmark_history/2026-10-12-navmesh-pruning-round11.json`.

## Round 12 — Progressive endpoint attachment generation

### Failure and hypothesis
Round11 Tier1 scaffold actions still take61.07s before materialization. `search_route` enumerates/validates up to128 source and128 target attachments before attempting any path. The prior Tier2 timeout also precedes materialization, but existing counters do not isolate attachment preparation from search. Generate a small resumable legal prefix with a bounded raw-probe quota, search it, then widen only when required. Retain the original bounded attachment universe, exact assembled-path validation and shared expansion/goal budgets.

### Research and experiment
The general motivation is to avoid expensive geometric evaluations until useful search feedback is available, as in [Lazy PRM](https://www.kavrakilab.org/publications/bohlin-kavraki2000path-planning-using.html) and [LazySP](https://www.ri.cmu.edu/pub_files/2016/3/paper-lazysp.pdf). This change is deterministic prefix widening of already validated attachment states, not a port of either algorithm and not permission to accept unchecked copper. Measure access/map/preparation/search time independently; keep rich source/arrival/via-history fallback. Synthetic tests must cover early success, later-prefix escape, total budget bounds and cancellation, followed by sequential telemetry Tier0/Tier1 and relevant Tier2 check under the existing timeout.

### Implementation and review
Attachment cursors preserve ring-before-anchor order and reuse the base scene, domain and exact edge cache. Default compact search tries prefixes8/32/128, reserving cumulative25%/50%/100% of its existing budget; provenance-rich search retains the remaining connection allowance. Each compact round restarts with the expanded goal set, so spent expansions are not equivalent to continuing one frontier: this is a bounded heuristic tradeoff to evaluate against the benchmark. Unchanged exhausted searches are skipped. The raw-probe quota bounds each cursor advance and the initial preparation slice; later widening completes its requested prefix across multiple slices. `--eager-endpoint-attachments` retains the previous full-prefix strategy for comparisons.

Independent review identified repeated rejected assemblies consuming the shared goal allowance. A per-search exact assembled-geometry identity now suppresses duplicate checks across compact and rich search; the bounded-goal via-history regression checks this mechanism. Timers are nested: edge/exact validation belong within search or attachment time and must not be added again. `domain_build_microseconds` includes the base domain and collision-index preparation, including the optional original-scene index used for soft passage ranking. Detailed output labels its counters `retained_scaffold_search`, because they describe the retained scaffold snapshot rather than cumulative engine work.

### Focused validation adjustments
The existing via-history fixture needs26 distinct rejected geometries before its legal result. A proposed two-goal test cap was unsupported and failed with two distinct rejections plus one duplicate suppressed. Restoring a diagnostic256 cap established26 rejections/8666expansions; the final regression uses a tight27-check cap, asserts duplicate suppression and bounded total expansions, and passes that mechanism. The existing compact-fallback test now allows up to three compact rounds while still requiring zero compact successes, exact connected copper and the shared connection cap.

### Round12a benchmark and decision
The initial enabled-by-default experiment is rejected as the default. Tier0 passes51/51 with zero opens, internal violations, external KiCad violations and unrounded bends, but detailed runtime rises55.375→58.371s (+5.41%). Its retained searched request uses16 total attachment probes instead of128 legal attachments per side, while expansions rise24→97 and the resulting copper differs. Both final layer images were inspected. Tier1's preliminary global run does not reproduce baseline completion: scaffold stalls at72/78 requests, the last scaffold action ends at189.118s, and the engine enters regional/global recovery. The experiment was interrupted after this confirmed regression rather than labelled a completed timeout. Frozen implementation, telemetry and compact evidence are preserved in `reports/validation/progressive-attachments-round12a-20261012/`.

The final implementation is opt-in using `--progressive-endpoint-attachments`; `--eager-endpoint-attachments` explicitly selects the established default and conflicting flags are rejected. Progressive unit tests explicitly enable it. Shared rejected-assembly deduplication and phase telemetry remain enabled independently. This preserves the requested resumable machinery without silently degrading the previously passing benchmark. Next hypothesis: merely finding a legal route from a small endpoint prefix is insufficient; prefix coverage and route quality must preserve downstream escape/packing options before early success can safely become the default. Tier2 is deferred because the progressive experiment regressed the lower tier.

### Final-default observations
Final-default Tier0 detail55.239s, zero opens/DRC/unrounded bends, byte-identical copper toRound11. Tier1 preliminary global completes90/90 in95.197s (baseline96.273s). Its retained scaffold snapshot records2411 raw probes,1152 attachments per side,39.570ms attachment generation,98.059ms access generation,2.428ms domain/index preparation and2389.658ms search. These are retained snapshot measurements, not all scaffold actions summed. They do not support attachment generation as the dominant cost for this scene.

### Final verification and decision
Keep the feature opt-in. Final release passes72/72 core tests and all seven related suites (eight CTest targets total). Sequential diagnostic-telemetry Tier0/Tier1 curved runs pass51/51 and90/90, with zero physical opens, internal copper violations, external KiCad DRC violations/unconnected items and unrounded non-junction bends. Both final boards are byte-identical toRound11. Detailed times55.239s/299.852s versus55.375s/300.426s are effectively unchanged single-run results; do not claim a speedup. PeakRSS580132/583940KiB. Frozen source, mode-flag smoke checks, final comparison and the rejected progressive-default experiment are recorded in `reports/benchmark_history/2026-10-12-progressive-attachments-round12.json`.

## Round 13 — Incremental copper transactions and reusable geometry certificates

### Failure and hypothesis
Round12's eager-default Tier0/Tier1 remain fully routed, but detailed wall times are55.239s/299.852s. Curved materialization repeatedly validates unchanged copper and rebuilds net-filtered obstacle geometry. Local PH edits also recertify the complete route. The hypothesis is that reusing exact immutable evidence and rejecting candidates that cannot improve the accepted objective removes work without reducing clearance or connectivity requirements.

### Implementation
`CopperTransactionValidator` infers additions, removals and replacements from complete canonical objects against one immutable base. A conservative spatial index selects unchanged copper neighbours; changed-to-changed interactions remain in the exact dynamic DRC set. Curve bounds include integer rounding and the shared conservative Bezier-proxy tolerance. Affected differential-pair copper is retained in full for gap/skew checks, including deletion-only transactions. Connectivity recomputes affected nets and caches their baseline components; acceptance preserves each existing pad component. Changes to static geometry, ownership context or rules, malformed IDs and uncertified baselines fall back to full validation. The three repeated residual-arc, spline-realization and serial-merge paths share this implementation. Final whole-board validation and KiCad DRC remain authoritative.

Per-layer extraction catalogues all obstacle owners independently of the routed net. Net exclusions are applied to that immutable catalogue. Individual solid/filled-region offsets are cached using full contour equality and actual effective clearance, with hash buckets followed by exact equality. Both caches are bounded and scoped to a materialization pass. Callers that do not request caching retain the direct Boost polygon path.

PH certification caches only complete legal span proofs, scoped to one optimizer invocation with an immutable clearance field. Keys include exact hull geometry, corridor identity and proof settings. Local span parameters allow reuse after route reparameterization; logical whole-route hull budgets still apply, and nonlegal/budget cases retain the original whole-route certificate. Exact-wave GPU screening omits already certified spans. Local proposal waves suppress identical/clamped proposals and skip exact assessments when an already legal incumbent is strictly shorter than the candidate under the existing comparator. No heuristic stagnation threshold or relaxed DRC was added.

### Validation
Focused tests compare incremental and full dynamic DRC, moved/deleted copper, pad-component preservation, rule/static invalidation, differential-pair deletion, offset holes and overlapping heterogeneous clearances, and cached/uncached span geometry and hull budgets. Independent review found no material defect; review hardening added proxy-tolerance coverage and filled-hole cases.

### Regression-driven correction
The first regression run passed71/72 core cases and all six related targets, but failed the existing nested-bend first-snapshot case (zero opens/DRC, one unrounded bend). A bounded diagnostic isolated this to duplicate pruning before advisory GPU ranking: disabling either caching or length pruning alone still failed, while disabling early duplicate pruning restored both legal bend commits. Suppression now occurs after the original GPU shortlist is selected and before exact rebuilding/certification. The diagnostic again completes both bends, with40 span-cache hits,41 equivalent candidates skipped and92 non-improving candidates skipped. The existing regression assertion was retained.

### Frozen benchmark results and decision
Keep. Sequential curved Tier0/Tier1 runs with diagnostic telemetry pass51/51 and90/90 physical connections, with zero opens, internal copper violations, external KiCad DRC violations/unconnected items and unrounded non-junction bends. Both layers of both final boards were inspected; rounded runs and straight branches remain, with no new obvious looping artifact. Geometry is not byte-identical toRound12 (Tier0:162tracks/24arcs/396Beziers/3vias; Tier1:442/105/408/57).

Detailed-command wall time fell55.239→44.398s (-19.63%) onTier0 and299.852→238.599s (-20.43%) onTier1. Materialization fell37.810→26.996s and203.981→144.352s respectively. The separate preliminary global harness command took16.835s/93.794s and is excluded from these detailed-command times. These are single-run comparisons. PeakRSS increased580132→635868KiB onTier0 (+9.61%) and583940→646892KiB onTier1 (+10.78%): retained geometry trades memory for reduced repeated work.

Tier1 executed476 incremental transactions, checking3307 changed owners against7661 indexed neighbours;150 affected-net baseline connectivity results were reused and no full-context fallback was needed. Serial exact merge fell17.518→3.256s, residual bend repair32.805→3.952s. PH optimization reused60641 legal span certificates and skipped4386 equivalent plus10531 strictly non-improving candidates. Obstacle expansion hits90353/misses4695; per-layer catalogue hits96/misses4. These counters demonstrate use but do not isolate each optimization's wall-time contribution.

The remaining preparation cost deserves attention: Tier1 summed worker preparation time remains129.670→127.100s despite high offset reuse; final Boolean union/subtraction and output validation still run per request. Summed NavMesh time73.186→75.407s is also effectively unchanged; trajectory59.436→52.678s and realization202.353→142.721s decreased. Worker sums overlap and must not be added to materialization wall time. Next performance work should target the remaining full-domain combination/validation and per-net geometry assembly, rather than assuming cached dilation removed preparation cost.

Tier2 was not rerun: its previously recorded failure precedes curved materialization. This round makes no claim of advancing that search frontier. Frozen source identity, the rejected pre-ranking dedup diagnostic, final verification, timings and copper/DRC identities are recorded in `reports/benchmark_history/2026-10-12-incremental-validation-round13.json`.

KiCad10.0.6 copper-centreline measurement (tracks/arcs plus net-assigned Bezier shapes; excludes via barrels) reports442.065→441.263mm forTier0 and902.747129→902.747594mm forTier1. Thus the changed Tier1 serialization adds only0.000465mm at this measurement precision; it is not a substantial route-length increase. The measurement method and counts are stored in `copper-lengths.json`.

Final post-freeze verification passed all11 CTest targets, including72/72 core cases, against the same CLI SHA-256 as both benchmarks. A settled build reported no work; source hashes showed no drift. Independent review found no material outstanding issue.

## Round 14 — Prepared free-space validation and contour certificates (2026-10-12)

### Failure and hypothesis
Round13 passes Tier0 and Tier1, but Tier1 still spends127.100s of summed worker time in navigation preparation despite90353 obstacle-offset cache hits and4695 misses. Source polygon validation/BVH construction and contour-to-source clearance comparisons repeat after expansion reuse. The raw clearance routine computes full nearest distances against every boundary, even though validation only needs to reject distances below an exact threshold.

### Experiment
Reuse immutable validated/prepared source geometry; spatially prune boundary comparisons using the existing conservative BVH lower bounds and capped exact segment-distance routine. Retain collision-safe clearance certificates only with matching geometry and effective rules. Keep exact containment, hole and overlap validation, Boolean construction, and whole-board/KiCad acceptance. Bound retained cache storage. Report input validation, preparation, Boolean operations and output validation separately; these are summed worker times and overlap in wall time.

### Implementation and verification
Source BVHs and their simple-polygon validation are reused independently of net exclusions and rule expansion. Whole-domain validation uses a content-hash prefilter and full equality; prepared domain views share immutable source objects. Contour certificates record only AABB-near source boundaries and effective clearances, with neighbourhoods recomputed on every query. Hash collisions require full contour/witness equality. Distant changes can reuse a proof; nearby boundaries and changed rules cannot silently retain it. Clearance uses the existing capped exact segment/BVH queries. Boolean construction and exact containment/overlap checks still run.

Each new source/context/certificate store has both an entry-count bound and a 262144-vertex bound. Certificate accounting includes referenced source vertices, preventing certificates from keeping an unbounded set of evicted BVHs alive. Independent review caught the initial design copying full rule vectors into every contour certificate; local shared witnesses replaced that design before testing. Existing obstacle expansion storage is unchanged.

All 11 regression targets passed, including source reuse across net views, locality-aware certificate reuse, nearby/rule invalidation, moved/added/deleted blockers, filled holes, invalid input, output budgets, and concurrent parity. A settled build was a no-op and source hashes matched the frozen CLI. Independent review found no remaining issues. ### Frozen benchmark results and decision
Keep. Sequential curved Tier 0 / Tier 1 runs with diagnostic telemetry pass all 51/51 and 90/90 physical connections. Both have zero opens, internal copper violations, external KiCad violations/unconnected items, and unrounded non-junction bends. Final KiCad boards are byte-identical to Round 13; route lengths, tracks, arcs, Beziers and vias are unchanged. No new visual-geometry claim is needed for this unchanged output.

Detailed-command wall time fell 44.398→39.692 s (-10.60%) for Tier 0 and 238.599→226.621 s (-5.02%) for Tier 1. Materialization fell 26.996→23.996 s and 144.352→133.512 s. Separate preliminary global harness commands took 15.286 s and 92.805 s and are excluded from those detailed-command times. Peak RSS fell 635868→621592 KiB (-2.25%) and 646892→621960 KiB (-3.85%). These are single-run comparisons with the existing desktop viewer left running and no overlapping builds, tests or tier runs.

Summed worker navigation preparation fell 36.458→23.801 s (-34.72%) and 127.100→87.475 s (-31.18%). Tier 1 reused 42186 prepared source polygons (2641 misses) and 6361 contour clearance certificates (584 misses). The new Tier 1 stage timers report 0.015 s input validation, 0.011 s source preparation, 86.834 s offset/Boolean operations, and 0.584 s output validation. They are summed worker durations, not additive wall-time phases. The remaining preparation cost is overwhelmingly Boolean assembly; reuse of Boolean-ready obstacle sets/common unions is the next evidence-backed performance experiment.

Tier 2 was not rerun: the known frontier failure precedes curved materialization. This round optimizes the passing curved pipeline without claiming that the search frontier has advanced. Verification, hashes, telemetry configuration, metrics and the keep decision are recorded in `reports/benchmark_history/2026-10-12-free-space-validation-round14.json`.

## Round 15 — Boolean-ready geometry and reusable spatial unions (2026-10-12)

### Failure and hypothesis
Round 14 leaves Tier 1 preparation dominated by offset/Boolean work: 86.834 s of 87.475 s summed worker time. Cached offsets are reconstructed as Boost sets, and each obstacle eagerly unions against the complete accumulated set. Reusing Boolean-ready operands and eroded outline/cutout geometry removes reconstruction. A deterministic spatial/hash radix hierarchy limits union recomputation to changed groups across net views and geometry/rule updates.

### Change
Obstacle and board-boundary caches retain normalized immutable Boost sets keyed by full geometry and effective clearance. Every Boolean operation uses local copies to avoid Boost's mutable const normalization. Spatial union groups retain immutable operand identities, so hash collisions cannot substitute geometry; owned operands prevent address reuse while a cache key remains live. Net exclusions occur before grouping, never by subtraction from an already merged obstacle set. Cached and uncached paths use the same deterministic grouping.

Each Boolean store has an entry-count limit and a 1048576 source/edge-unit retention limit. Union accounting includes its retained operand sets. Exact output containment, clearance, and final whole-board/KiCad checks remain enabled. Separate conversion, offset preparation, union, subtraction and contour-extraction timers are disjoint subsets of the aggregate Boolean wall timer, which also includes cache/copy overhead.

### Verification
All 11 selected regression targets passed, including cache/exclusion parity, hole survival and closure, changed source/rule invalidation, reusable union subtrees after removing/moving one blocker, concurrent reuse, and cache reset. Independent review found an existing clearance-plus-two overflow path; uniform and heterogeneous values now share a representable guarded-clearance check, including on warm caches. Added extreme-invalid-value regressions pass. The new ring fixture initially consumed all exterior free space as well as its hole; enlarging its board isolates hole closure without changing production semantics.

The final build was a no-op and the frozen source/binary passed verification.

### Frozen benchmark results and decision
Keep. Sequential curved Tier0 and Tier1 runs with diagnostic telemetry pass 51/51 and 90/90 physical connections, respectively, with zero opens, internal copper violations, external KiCad violations/unconnected items, and unrounded non-junction bends. All11 regression targets passed and independent review found no outstanding material issues.

Detailed-command wall time fell39.692→31.430s (-20.82%) and226.621→133.248s (-41.20%). Materialization fell23.996→15.888s and133.512→45.145s. Separate preliminary global commands took15.224s and92.920s; these are excluded from the detailed-command times. PeakRSS rose621592→636288KiB and621960→639648KiB. These are single-run measurements, with the desktop viewer remaining open and no overlapping builds/tests or tier runs.

Summed worker Boolean time fell23.418→0.778s and86.834→2.903s; total navigation preparation fell23.801→1.370s and87.475→3.498s. Tier1 reused81221 union groups (12407 misses). Its individual Boolean timers were0.015s conversion,0.036s offset preparation,2.210s union,0.258s subtraction,0.352s extraction. Worker sums overlap and are not additive wall-time stages.

Tier0 copper is byte-identical. Tier1 copper differs:490 tracks,124 arcs,355 Beziers and57 vias versus442/105/408/57. KiCad centreline length rises902.748→904.299mm (+0.172%). Front/back copper renders were inspected: long runs remain direct with rounded turns, without obvious new loops; short segmented transitions remain in places. Exact checks pass. The cause of the changed decomposition was not isolated; grouping and runtime-sensitive candidate processing changed, so this is not claimed as a byte-preserving optimization on Tier1.

Tier1 global/scaffold actions now consume about87s and materialization45s. Summed NavMesh construction39.628s and trajectory35.754s outweigh Boolean preparation. Tier2 was not rerun because its known search failure precedes materialization. Detailed evidence is in `reports/benchmark_history/2026-10-12-boolean-cache-round15.json`.

## Round 16 — Lazy NavMesh construction for certified corridors (2026-10-12)

### Failure and hypothesis
Round15 passes Tier0/1, but Tier1 records5147 prepared NavMesh builds with zero hits and39.628s of summed navigation work. `refine_spline_run` constructs a full search lattice before selecting the certified source corridor; downstream clearance queries only need prepared boundary geometry. Reordering this decision should eliminate unused lattices without changing acceptance rules.

### Change
Expose geometry-only preparation through the existing bounded, exact-region cache. Certified source corridors use this geometry for exact clearance probes. Construct the full lattice only when no source corridor is available. Full-domain preparation shares the same geometry API, preserving concurrent request coalescing, validation mode isolation, and memory limits.

### Verification
Independent review found no material correctness, clearance, fallback, or concurrency issue. Geometry-only preparation intentionally returns a domain with no lattice: `PreparedNavMeshDomain::ok()` remains the searchable-domain predicate and returns false for this object; consumers use its geometry and diagnostic. Public API comments document the non-searchable contract. All11 regression targets passed, including new point-query parity through holes/outside the board, no-lattice assertions, repeated geometry reuse and full-mesh reuse of that geometry. The final build was a no-op; frozen source hashes have no drift.

### Results and decision
Keep this focused work-elimination change. Sequential curved Tier0 and Tier1 telemetry runs pass51/51 and90/90 physical connections. Both final boards are byte-identical to Round15, with zero opens, internal copper violations, external KiCad violations/unconnected items, and unrounded non-junction bends. Route lengths and copper/via counts are unchanged; no new visual claim is needed for identical copper.

Tier0 full prepared-mesh builds fell218→168; summed navigation work12.403→11.071s, materialization15.888→15.722s and detailed-command wall time31.430→31.178s. Tier1 full builds fell5147→5076; summed navigation work39.628→28.943s (-26.96%), materialization45.145→42.274s (-6.36%). However total detailed-command time rose133.248→135.310s (+1.55%): unchanged global/scaffold actions took91.908s versus87.0s. This single run supports the targeted mechanism but does not demonstrate an end-to-end speedup. Worker timers overlap and are not additive wall-time phases.

PeakRSS was636288→628588KiB and639648→639596KiB. Separate preliminary global commands took15.264s and92.387s, excluded from detailed-command timing. No builds/tests or tier processes overlapped the benchmarks. Tier2 was not rerun because its known frontier failure precedes materialization. Remaining full mesh builds largely occur outside successful certified-corridor selection; this optimization cannot remove those without changing fallback behavior. Global/scaffold work remains the principal wall-time target. Evidence is recorded in `reports/benchmark_history/2026-10-12-lazy-navmesh-round16.json`.

## Round 17 — Tier2 and Tier3 diagnostic attempts (2026-10-12)

Ran the unchanged Round16 router (`4277938`, binary SHA256 `90bf45d8e63a32ce54272494c9f815639d72cf04420ea050df9033d583a35797`) sequentially with diagnostic telemetry, curved mode,30 iterations,900s internal and960s outer limits. Tier3 used the explicit diagnostic locked-tier override; manifest status is unchanged. No builds/tests or routing processes overlapped; frozen hashes showed no drift.

Tier2 `ft2232h_jtag` timed out at900.178s. Retained scaffold85/117,32 missing requests,46 physical opens,21 physically unrouted nets. It reports117 abstract paths/zero abstract unrouted nets but381 overflow, including340 clearance-shadow overflow and6 corridor overflow. Scaffold search consumed768.260s with18279 candidate rejections. Ordinary/coupled victim exchanges accepted0/10 and0/8. PeakRSS643056KiB. This matches Round07 connectivity and remains worse than Round06's88 retained/43 physical opens; the later curve optimizations were not reached.

Tier3 `gb_nintendo_power` timed out at900.111s. Its global checkpoint has only1 path and64 unrouted nets (zero overflow is not evidence of success here). Scaffold33/148,115 missing requests/physical opens,61 physically unrouted nets. Recorded scaffold search329.789s; ordinary/coupled victim exchanges accepted0/15 and0/4. PeakRSS531828KiB. The rest of runtime is not attributed by this one search timer. This establishes a diagnostic baseline, not tier promotion.

Both engine outcomes are `timed-out`; their global result field still says `iteration_limit`, so use the engine outcome and measured runtime when classifying the stop. Neither reached detailed curved materialization, produced accepted final copper, or ran final external KiCad DRC. Harness exit0 means the expected incomplete diagnostic was recorded, not that routing passed. Front/back Tier2 and front Tier3 diagnostic PNGs were inspected; they reconstruct global checkpoint geometry and do not constitute final copper or a complete scaffold rendering.

No source changes. Keep both failure records. Next investigation should target Tier2 attachment/fallback search cost and victim recovery, while Tier3 additionally requires diagnosing initial global access/topology coverage. Evidence: `reports/benchmark_history/2026-10-12-tier23-round17.json`; raw telemetry and previews remain under `reports/validation/tier23-round17-20261012/`.

## Round18 — Item-level scaffold collision pruning (2026-10-12)

### Failure and hypothesis
Tier2 Round17 retained85/117 connections and46 physical opens after900s. Edge validation consumed826.387s, attachment generation61.102s and search768.260s (overlapping timers). Only two unresolved requests were reached; net12 had128 source attachments and zero target attachments.

Inspection found that the spatial index maps a pad to its entire footprint. Near the64-pin IC, every edge check reconstructs all64 pads and invokes the dynamic checker's GPU broadphase threshold, repeatedly. Rank1 is individual-pad pruning; rank2 is fairer bounded repair scheduling only if starvation persists after removing query overhead; rank3 is targeted terminal/via escape generation once evidence is affordable to obtain.

### Research and experiment
KiCad's [PNS node collision query](https://raw.githubusercontent.com/KiCad/kicad-source-mirror/master/pcbnew/router/pns_node.cpp) queries indexed items and refines individual candidates. Apply that concept using Flux's existing exact checker: preserve board/footprint keepouts, rules and transforms while copying only nearby pad owners, including independent drill bounds. No external implementation copied. Test exact indexed/full-board DRC parity before sequential Tier0/1/2 curved benchmarks.

### Results and decision
Keep. All11 regression targets passed and independent review found no material issue. Frozen sequential Tier0/1 curved telemetry runs pass51/51 and90/90 connections with zero opens/internal/external KiCad violations or unconnected items. Both copper boards are byte-identical to Round16. Detail runtimes31.476s and130.463s (baseline31.178s and135.310s); unchanged lengths441.263mm and904.299mm.

Tier2 reaches112/117 scaffold requests (baseline85/117),19 physical opens (baseline46),8 physically unrouted nets,1258 tracks and51 vias. It stops exhausted at442.613s instead of timing out at900.178s. Missing requests43,44,91,95,115 remain. The result's abstract `complete=true` is not physical success: `routing_plan_complete=false`, no materialization or final KiCad DRC. Final retained-checkpoint rebuilding resets search counters, so detailed failure analysis must use earlier telemetry snapshots. PeakRSS1205660KiB versus643056KiB; faster search explores substantially more states, increasing memory. No lower-tier regression. Next experiment must address the remaining physical access/search failures. Evidence: `reports/benchmark_history/2026-10-12-scaffold-pad-index-round18.json`.

## Round19 — Bounded off-grid terminal escape experiment (2026-10-12)

### Failure and hypothesis
Round18's five unresolved requests cluster at U4 fine-pitch banks. Earlier evidence for one request showed zero target lattice attachments despite a valid physical pad. Direct pad-anchor-to-grid chords can encroach on an adjacent pad before clearing the pad bank. An exact-checked off-grid escape stub followed by a lattice attachment may represent legal paths excluded by the current graph. This is a hypothesis about the remaining board, not a claim that Round18 proved attachment exhaustion: regional proposal admission rebuilt its final scaffold with zeroed search counters. New per-pass telemetry preserves search/access outcomes independently of later checkpoint reconstruction.

### Candidate ranking and research
1. Bounded terminal escape stubs: directly addresses the earlier measured zero-attachment failure and the remaining geometric cluster; test on a narrow off-grid microcase before the board.
2. Admit residual physical demands before all original coarse demands complete: fourteen additional physical opens cannot currently enter residual repair while five original requests remain; defer until terminal experiment identifies remaining failures.
3. Skip rich fallback only after compact graph disconnection is proven: useful work reduction, but cannot create missing legal terminal access and must retain history-sensitive fallback after any goal-node hit.

OpenROAD's [detailed-routing documentation](https://openroad.readthedocs.io/en/latest/main/src/drt/README.html) separates pin-access analysis from maze routing and exposes minimum access-point and via-access constraints. This supports explicit exact-checked attachment generation as a distinct stage; no external source code is copied, and IC track/via restrictions are not assumed to match PCB rules.

### Change
Retain existing direct attachments as a stable prefix. On exhaustion, derive a pad-relative outward direction from same-net/layer pad anchors, extend beyond its conservative copper extent, and enumerate a bounded local lattice around that exit. Attachments retain the off-grid waypoint and charge both segment lengths. Compact and history-aware assembly insert both source/target stubs before the unchanged whole-route exact DRC and connectivity gate. Pad shapes are lazy; stub geometry and first-leg exact legality are cached only within the immutable search. Existing attachment/expansion caps and cancellation checkpoints remain authoritative. New outward probe/admission counters are serialized and emitted with immutable per-pass missing-request evidence.

The synthetic rotated-pad test explicitly rejects every old anchor-to-grid chord with a one-cell attachment radius, then requires exact-clean routes for both search modes and source/target directions. Independent review found no material issue. The bounded policy can still withhold stubs if direct attachments consume the full configured cap; this experiment targets missing physical access rather than changing all candidate ordering.

### Rejected eager trial and refinement
The first frozen trial passed Tier0 with byte-identical copper but changed Tier1's scaffold choices:73/78, then77/78, then another77/78 checkpoint. At over180s it entered further global/USPED work, versus the previous complete90-demand result in89s. Stopped this trial early; it is an aborted diagnostic, not a completed benchmark. Direct candidates remained an enumeration prefix but eager A* still ranked new stubs against them. That policy is rejected.

Refine the mechanism to generate stubs only when all direct rings yield zero legal attachments for that endpoint. This preserves the original entire attachment set whenever direct access exists; it still covers the measured fine-pitch zero-access failure and the synthetic regression. It deliberately does not cover non-empty but disconnected direct attachment graphs. Retain the first trial's source/binary, telemetry and abort reason separately; rerun all lower tiers before Tier2.

### Refined verification and results
Keep the zero-direct-attachment fallback. Focused mechanism test and all11 regression targets pass, no-op build settled, independent review found no material issue, frozen source verified unchanged. Sequential curved telemetry Tier0/1 runs pass51/51 and90/90 physical demands; both outputs are byte-identical to Round18 and have zero internal/external KiCad violations, opens, unconnected items and unrounded non-junction bends. Detail runtimes30.374s and120.555s; separate planning runs14.813s and85.306s. These are single-run measurements.

Tier2 reaches113/117 (previous112),18 physical opens (previous19), and exhausts at397.394s (previous442.613s), peakRSS1268940KiB. Request43 is recovered. Remaining44 and91 terminate with zero target/source attachments;95 and115 have120/1 and128/16 attachments but exhaust expansion budgets. Third scaffold pass spends49.450s searching and37.844s in overlapping edge checks, with2800 outward probes/105 admitted attachments and zero goal rejections. No detailed materialization or final KiCad DRC was reached; abstract `complete=true` still does not signify routed copper. Preserve per-pass evidence rather than the zeroed counters from final regional rebuilding. Next hypothesis: a single conservative pad-normal depth may overrun a feasible earlier turn; inspect the local copper and test bounded depth alternatives independently before changing it. Evidence: `reports/benchmark_history/2026-10-12-terminal-stub-round19.json`.

## Round20 — Endpoint-specific causal repair witnesses (2026-10-12)

### Failure and hypothesis
Round19 still has zero-access terminal failures44 and91. Saved `source_boards/generated-copper:latest` contains actual retained track/via geometry; coarse route candidates must not be mistaken for that copper. Adjacent retained exits create a tight pad-bank packing problem. Request95 depends on coarse producer net3:1 (failed request91), so its lack of a terminal-rebind proof is currently expected; do not broaden rebind to an arbitrary nearest terminal.

`search_route` previously unions rejected probes at both ends into `frontier_blockers`, including near endpoints with128 legal attachments. `extend_causal_victims` admits only the first six generated owners in lexical object order, and singleton repair normally tries four. Therefore unrelated successful-end probe failures can consume the repair allowance before a zero-access endpoint's blockers. Hypothesis: isolate the zero-access cut before selecting victims, retaining useful copper and the same exact acceptance gates.

### Change and experiment
Each attachment cursor records its own rejected-object witnesses. Cached stub first-leg results retain their witnesses. On genuine attachment exhaustion, use only the empty cursor(s)' witnesses for causal repair; keep ordinary maze-search evidence and cancellation behavior unchanged. Report the number of irrelevant attachment owners pruned. A symmetric synthetic case has a successfully accessible endpoint beside a noisy obstacle and an inaccessible opposite endpoint; it requires repair evidence to retain only the latter blocker. No routing rules or repair budgets change. Exact saved-copper probing is also being prepared to distinguish overshooting stub geometry from coordinated packing needs; no speculative depth ladder is enabled.

### Research context
This applies the existing conflict-directed repair principle already researched in earlier rounds: select a bounded repair from witnesses of the actual failed constraint. It is a correction to attribution, not a new solver family or imported implementation. First validate the witness contract, then rerun lower tiers and the frontier sequentially with telemetry.

### Result and decision
Revert the endpoint-specific filtering policy. All11 tests and independent review passed; sequential telemetry Tier0/1 remain KiCad-clean with zero opens. Tier0 copper is byte-identical, but Tier1 choices changed (57→63 vias) and detailed runtime increased120.555→199.475s, almost entirely in spline realization39→118s. Tier2 is unchanged at113/117 with18 physical opens and missing44/91/95/115;398.341s versus397.394s. The hypothesis did not yield physical progress. Preserve the rejected patch and frozen source/binary for reproducibility; restore Round19 behavior. Exact saved-copper probes show request43's via blocks the sampled outward exits of44 and91 even after removing45. Shorter stub depths do not restore sampled access.

Next experiment: correct exact-grid attachment admission. Bare endpoints are retained by scaffold access without same-net copper membership, but coincident grid points bypass exact collision checks. The symmetric fixture exposed this when a foreign-copper endpoint admitted one zero-length attachment. Test point-clearance screening independently before attempting joint via packing changes.

## Round21 — Exact-grid endpoint clearance (2026-10-12)

### Failure and hypothesis
The Round20 synthetic fixture revealed that an endpoint inside foreign copper still acquired one attachment when exactly on the scaffold lattice. `scaffold_connection_access` intentionally retains bare endpoints without existing same-net copper; attachment generation skipped collision checks whenever origin equalled the grid point. A fabricated accessible terminal prevents accurate attachment-exhaustion diagnosis and consumes maze budget.

### Change and experiment
Use the existing exact dynamic-copper check for coincident endpoint/grid points as well as nonzero chords. A zero-length query represents the trace-width clearance disk and is not emitted as output copper. Preserve same-net contact exemptions. Test foreign-copper exact-grid endpoints and legal same-net contacts before sequential curved Tier0/1/2 telemetry runs. This is a correctness fix to existing geometry semantics, not a new routing algorithm. Separately probe bounded blocker-owner removals against the prior saved copper to establish which small joint repair actually restores terminal access; diagnostic deletions are never accepted board output.

### Results and decision
Keep. All11 CTest targets pass; the strengthened same-net fixture permits only one exact-grid attachment and also passes. Review found no source issue. Sequential curved telemetry Tier0/1 pass with zero internal/external KiCad violations, opens or unconnected items. Tier0 remains byte-identical (30.498s detail); Tier1 improves from120.555s to103.435s detail and85.306s to61.864s separate planning.

Tier2 improves113→114/117 scaffold requests, recovering95; missing44/91/115 remain. Physical opens stay18, so this is not board completion. Runtime397.394→261.274s, peakRSS1288616KiB. Second/third scaffold searches take23.202/22.784s versus49.450s previously. No materialization or final KiCad DRC is reached. The exact-grid correction removes an invalid access-state mechanism and improves search; it does not solve terminal-bank packing.

Saved-copper owner-removal diagnostics: removing43 alone provides48 sampled stub attachments for91; removing43+45 provides48 for both44 and91. Removing11+19 yields four direct attachments for each, whereas either alone yields none. These are bounded geometric witnesses, not valid transactions: all removed routes must be reconstructed and exactly validated. Next inspect per-pass exchange rollback evidence, which final checkpoint rebuilds currently obscure, before changing repair selection or budgets.

## Round22 — Preserve causal exchange evidence (2026-10-12)

### Failure and experiment
Round21 leaves requests44/91 with terminal-access failures and115 with maze failure. Prior final-result scaffold reconstruction replaces the earlier exchange counters/records; zero final counters do not mean exchanges were never attempted. Emit existing bounded exchange attempts, acceptance, budget-exhaustion and rollback counters at the end of each physical scaffold pass, plus stored per-exchange reasons, victim reconstruction order and blocker IDs. Identifier lists are escaped and length-prefixed. This changes telemetry only; routing policy and acceptance remain Round21's. Rebuild and run focused engine regression, then rerun Tier2 sequentially with diagnostic telemetry. Lower-tier physical validation from Round21 remains the algorithm baseline.

### Results and diagnosis
Telemetry-only engine build/test passes; Tier2 reproduces114/117,18 opens at252.305s, no materialization. The first scaffold pass reports84 total exchange attempts, including50 coupled attempts;64 recorded rollbacks comprise40 incoming-unroutable and24 victim-unroutable. For44, singleton11/19/43/45 fail incoming;11+19 and11+19+43 admit incoming but fail victim reconstruction. The bounded ordering never reaches43+45. For91, singleton43 admits incoming but fails rebuilding43. This disproves the misleading conclusion one would draw from final reset counters that repairs were never attempted.

Offline replay tests43 for91,43+45 for44 in both orders, and11+19 for44/91 in both orders. All incoming requests route; every transaction remains incomplete because a victim loses endpoint access. Baseline saved copper has six static pad-clearance reports; no replay introduces additional reports. Thus skipped-group selection alone is insufficient. The actual mechanism is incoming copper closing a displaced victim's terminal escape.

### Ranked next experiments and research
1. Retry failed-victim transactions with temporary reservation of original legal terminal escape fragments, omitting obstructing vias; remove reservations before rebuilding victims and retain exact transaction acceptance. This directly protects the observed lost access without freezing full victim routes.
2. Conflict-directed constraints on the incoming geometry responsible for victim access failure, with bounded alternate search. More flexible but needs precise geometric constraints and duplicate suppression.
3. Merely enlarge victim/attempt budgets: lower priority because diagnostic replays already show victim failures with the promising groups.

[OpenROAD detailed routing](https://openroad.readthedocs.io/en/latest/main/src/drt/README.html) treats pin access separately from track assignment and search/repair. [Conflict-Based Search](https://ojs.aaai.org/index.php/AAAI/article/view/8140) separates conflict-level constraints from individual path search. These support the separation of concerns; temporary PCB escape reservations are a Flux experiment, not a claimed direct implementation of either source. Time-expanded MAPF guarantees do not transfer to persistent PCB copper.
