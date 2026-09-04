# Flux feature-parity plan for KiCadRoutingTools

## 1. Objective

Bring Flux to behavioral feature parity with the public capabilities of
[KiCadRoutingTools](https://github.com/drandyhaas/KiCadRoutingTools) while preserving Flux's
integrated placement/routing design, physical corridor model, continuous quintic PH routes,
transactional source handling, deterministic search, mandatory GPU acceleration, and exact KiCad
acceptance gates.

This plan is based on KiCadRoutingTools v0.21.5 at commit
`e239e067c204870b4ac570fc9e931a374bdfd5b4` from 2026-09-02. That reference point contains 199
Python modules across its routing, placement, and utility packages, a Rust A* core, 547 Python
tests, and 443 entries in its checked-in stress manifests. Reassess the upstream feature inventory
before declaring parity because the upstream repository remains active.

Parity means that a Flux user can accomplish the same board-design task with equal or stronger
safety and observable evidence. It does not mean copying KiCadRoutingTools' grid model, internal
module layout, command names, heuristics, or source code.

## 2. Definition of parity

A reference capability is complete only when all of the following hold:

1. A public Flux CLI or KiCad workflow exposes the behavior.
2. Its configuration is explicit, validated, serializable, and visible in telemetry.
3. The behavior operates on the canonical immutable source board rather than a parallel PCB model.
4. The result passes canonical validation, represented connectivity, represented copper DRC,
   source-preserving round-trip checks, and external KiCad DRC where applicable.
5. Failure is typed and attributable; unsupported source semantics fail closed.
6. A focused capability fixture passes, all previously qualified routing tiers remain green, and a
   relevant real-board stress sample does not regress.
7. Runtime, peak memory, GPU utilization, queue delay, and exact-check workload are recorded.
8. The implementation remains deterministic for a fixed seed, device, configuration, and source
   revision. GPU reductions use stable keys and deterministic final reduction on the CPU.

Stronger Flux behavior satisfies parity. For example, Flux's global PathFinder plus regional
packing can satisfy a reference A* routing feature; a second KRT-style router is not required.

## 3. Non-negotiable Flux design constraints

- Keep one canonical `Board` and one route acceptance pipeline. Do not build a separate feature
  router whose copper bypasses global/regional feedback.
- Preserve the three representations: topology/resources, continuous PH/corridor geometry, and
  quantized KiCad copper.
- Keep the nested global-routability, regional-realization, and materialization-feedback loops.
- Treat placement, polarity changes, target swaps, fanout, planes, teardrops, and cleanup as complete
  transactions on a board copy.
- Never accept a partial coupled-pair, bus, plane, placement, or shove transaction.
- Never make an inferred bus, electrical constraint, fabrication capability, or schematic mutation
  authoritative. Inference may generate a reviewable source-bound proposal.
- Do not weaken or rewrite KiCad project rules merely to make generated copper pass. Resolve rules
  from the source, report unrepresented semantics, and leave external KiCad DRC authoritative.
- Keep zones under KiCad refill authority. Flux may propose zone outlines, priorities, taps, and
  stitching, but acceptance requires refill, strict re-import, connectivity, and DRC.
- Keep quintic PH chains as routed continuous geometry. Quantized lines/arcs are the KiCad physical
  realization, not the optimization representation.
- GPU support is mandatory for production routing. CPU code remains authoritative for branchy
  orchestration, exact integer predicates, source parsing, transactional commit, and final
  certification.
- Telemetry remains non-authoritative and non-blocking. A future control surface must use a separate
  orchestration boundary rather than sending commands through the observer WebSocket.
- Preserve source bytes and refuse accidental overwrite. Existing copper is immutable unless the
  caller explicitly selects it as a reroute/rip-up scope; KiCad-locked copper is never movable.

## 4. Upstream capability inventory and current Flux gap

The status column is a planning assessment, not a claim of qualification.

| Capability family | Reference behavior | Current Flux position | Parity work |
| --- | --- | --- | --- |
| Single-ended routing | Octilinear multilayer A*, automatic vias, incremental obstacles | Core equivalent is stronger but dense-board completion is incomplete | Qualify outcomes and close remaining global/regional failures |
| Route scope | Net patterns, exclusions, components, classes, placement blocks, preview, forced reroute | Individual-net selection and previews exist; scope vocabulary is incomplete | Add one canonical route-selector language and explicit reroute modes |
| Net ordering | MPS, inside-out, original order, bus middle-out, MPS layer swaps | Constraint priority, portfolio diversity, interference scores, and feedback ordering exist | Add MPS/inside-out evidence as portfolio generators, not a replacement scheduler |
| Rip-up/reroute | Bidirectional probes, blocker attribution, min-cut ranking, N+1 escalation, ripped-corridor avoidance | PathFinder history, hard-capacity rip-up, blocker memory, and LNS exist | Add explicit frontier/min-cut evidence and protected/pre-existing route policy |
| Proximity guidance | Stub, BGA, track, via, vertical-alignment, and guide-corridor costs | Physical history and terminal reservation cover part of this | Add typed future-demand and user-guidance fields with residual-demand accounting |
| Differential pairs | Coupled bare-pad, multipoint, hybrid escape, tangent-aware routing, swaps, return vias | Declared pairs, atomic capacity, synchronized vias/fanout, PH geometry, and skew checks are partial | Complete pair topology, return path, target/polarity proposal, and qualification |
| Buses | Detection, middle-out routing, neighbor attraction, parallel packing | Explicit ordered buses and shared packing exist; auto-detection is deliberately absent | Add non-authoritative bus proposal generation and complete joint realization |
| BGA/QFN fanout | Channel, dogbone, under-pad, layer balancing, pair fanout, adjacent-net joins | Generic terminal access exists; specialized package fanout is absent | Add package-aware terminal-escape template generators feeding regional routing |
| Power traces | Per-net widths, fabrication tiers, neck-downs and stepped transitions | Explicit net-class widths and bounded neck-downs exist | Add current-policy sidecars, complete transition geometry, and fab validation |
| Power/ground planes | Plane creation, multi-net Voronoi zones, region repair, pad taps, thermal arrays | Saved zones/refill are understood; plane synthesis and repair are absent | Add proposal/refill/repair workflow without implementing a competing fill engine |
| Return/stitch vias | Signal return vias, split-ground domains, area stitching, edge fences | Same-net via reuse exists; return-domain and stitching policy are absent | Add ground-domain analysis and joint via-site allocation |
| Length/time matching | Groups, DDR discovery, trombone meanders, via barrels, layer delay, AC-coupled XNets | Differential skew tuning is narrow and post-route | Add explicit match groups and PH-native length/time tuning |
| Impedance | Stackup-aware microstrip, stripline, coplanar, pair widths and reports | Absent | Add advisory electrical model plus source-rule-bound geometry validation |
| Placement | Seed refinement, quench, portfolios, router-in-loop repair, block moves | Integrated analytical/local placement and route acceptance are strong but not workflow-complete | Add design intent, portfolio UX, health grading, and package-specific repair |
| Design intent | Design brief, edge-facing connectors, floorplan rules, locks, groups, health | Source-bound groups/regions/relative constraints exist | Add a strict design-intent schema and deterministic brief-to-policy compiler |
| Swaps/schematic sync | Hungarian target assignment and opt-in P/N/pad swaps | Absent | Add proposal-only logical swaps with schematic consistency and human approval gates |
| Cleanup | Gap snapping, redundant-copper removal, restore accounting, KiCad reconnect | Exact validation and transactional fallback exist; cleanup vocabulary is incomplete | Add connectivity-preserving copper hygiene and DRC-oracle repair |
| Verification tools | DRC, connectivity, pads, orphan stubs, cycles, odd copper, capacity, channels, pockets | Core validation plus analysis PNGs exist | Expose focused diagnostic subcommands from shared authoritative predicates |
| Teardrops | Add settings to routed pads and vias | Absent | Add source-preserving opt-in policy and external refill/DRC validation |
| GUI/plugin | Full KiCad tabs, selection, settings, cancellation, live apply | File workflow, live route application, telemetry viewer, and preferences exist | Add a thin KiCad orchestration UI while keeping routing out of the plugin process |
| Rendering/replay | Static board images, routing movies, plan recording/replay | Live/history telemetry and PNG diagnostics exist | Add deterministic static export and trace-to-video tooling |
| Plans/AI adapters | JSON routing plans, headless replay, optional AI assistance | No stable workflow-plan schema | Add deterministic plan schema first; AI remains an optional untrusted author |
| Stress/release | Feature tests, replayable chains, A/B waves, large real-board corpus | Progressive tier corpus exists | Add feature-parity fixtures and scalable A/B qualification without running all boards per edit |

## 5. Target architecture

```text
KiCad board + project rules + reviewed source-bound policies
                         |
                         v
                 canonical Board IR
                         |
          +--------------+---------------+
          |                              |
          v                              v
  intent/electrical analysis       physical free-space preparation
          |                              |
          +--------------+---------------+
                         v
              route/fanout/plane work plan
                         |
                         v
              GLOBAL NEGOTIATION LOOP
       portfolios + PathFinder + physical feedback
                         |
                         v
              REGIONAL REALIZATION LOOP
      terminal/lane/via/layer state <-> PH/PNS/XPBD
                         |
                         v
               MATERIALIZATION LOOP
         quantize -> exact local check -> PH feedback
                         |
                         v
      optional tuning / planes / cleanup transactions
                         |
                         v
       validation + connectivity + analytic copper DRC
                         |
                         v
  source-preserving export -> KiCad refill -> external DRC
```

New capabilities must enter through work-plan inputs, route portfolios, regional templates,
continuous constraints, or post-route transactions. None may write copper around the established
acceptance pipeline.

## 6. Performance and GPU policy

### 6.1 Shared GPU runtime

Complete and use one Vulkan runtime for hydraulic routing, SDF/clearance fields, global batch
search, PH scoring, XPBD, broadphase, plane analysis, fanout evaluation, and rendering.

- Select one physical device deliberately and expose its identity in every run artifact.
- Keep one logical device, allocator, descriptor/pipeline cache, queue family set, and timeline.
- Use capacity-grown persistent buffers and a bounded in-flight ring.
- Cache immutable board geometry, layer masks, SDFs, boundary BVHs, pad fields, and rule tables by
  source revision plus policy identity.
- Keep intermediate products device-local across clearance, hydraulic, global, regional, XPBD, and
  broadphase stages.
- Batch independent nets, candidate portfolios, fanout terminals, via sites, tuning patterns, and
  placement proposals before dispatch.
- Read back compact scores, predecessor paths, conflict pairs, diagnostic tiles, and top-K indices;
  do not read complete fields unless telemetry or an analysis artifact explicitly requests them.
- Use fences or timeline semaphores rather than queue-idle waits. Overlap preparation, compute,
  compact readback, and CPU exact certification.
- A missing/failed GPU is a clear production preflight failure. A CPU reference backend may exist
  only for focused verification and differential testing.

### 6.2 Suitable GPU work

- polygon/grid/SDF preparation and obstacle-distance transforms;
- hydraulic and anisotropic routing fields per copper layer;
- batched reverse cost-to-go fields and bucketed global searches;
- occupancy histograms, critical-cut demand, history deposition, and route-priority reductions;
- terminal, via, fanout, layer, lane, and regional-template approximate scoring;
- MPS crossing matrices and route-interference matrices;
- PH position/tangent/curvature/offset sampling and quantized candidate scoring;
- XPBD constraint projection and route-to-route broadphase;
- segment/circle/simple-arc DRC broadphase and guarded simple narrow-phase;
- plane masks, connected components, Voronoi ownership, resistance/current approximations, and via
  coverage fields;
- meander/serpentine pattern generation and approximate clearance/skew scoring;
- placement density, congestion, HPWL, plane-fragility, and batched screening objectives;
- telemetry overlay reduction, static board rasterization, and replay frames.

### 6.3 Work that remains CPU or KiCad authoritative

- S-expression import/export and source-revision binding;
- rule precedence and source-policy validation;
- branch-and-bound regional state orchestration and stable tie reduction;
- exact integer polygon, rational PH-offset, connectivity, and complex-pad predicates;
- final whole-board transaction comparison;
- source-preserving change-set construction;
- KiCad zone refill and external KiCad DRC.

### 6.4 Performance gates

Every milestone reports, before and after:

- wall time and CPU time by stage;
- peak RSS and peak device memory;
- GPU busy time, dispatch count, batch occupancy, cache hit rate, and readback bytes;
- search expansions/states and exact predicate counts;
- candidates generated, GPU-rejected, CPU-shortlisted, exact-rejected, and accepted;
- telemetry queue drops and observer overhead;
- route-quality feasibility tuple, copper length, via count, width loss, and DRC results.

A feature may not become default if it regresses the qualified-tier geometric mean runtime by more
than 10%, peak memory by more than 15%, or route feasibility on any qualified board unless a
documented benchmark decision explicitly accepts the trade. GPU batching must show either a
measured wall-time improvement or a demonstrated removal of a scaling bottleneck.

## 7. Implementation milestones

Milestones are dependency-ordered. Work within a milestone may proceed in parallel only when it
does not create two board models or two acceptance paths.

### Milestone 0 — Freeze the parity contract and benchmark baseline

Priority: P0.

1. Add a machine-readable parity manifest keyed to the pinned upstream commit. Give every advertised
   capability a stable ID, Flux status, reference evidence, focused fixture, stress sample, public
   command, and acceptance criteria.
2. Add `flux-cli capabilities --json` so scripts and the KiCad adapter can fail before running an
   unsupported plan.
3. Record the current Flux tier results, performance counters, GPU identity, source revisions,
   route configuration, analysis PNGs, and telemetry traces.
4. Select the smallest legally reusable KRT fixtures for each feature. Copy only boards whose
   licenses permit it, retain provenance, and prefer the already imported routing corpus.
5. Establish three suites:
   - per-change risk suite: focused invariant tests plus all qualified tiers;
   - frontier suite: qualified tiers plus the one permanently unlocked tier;
   - release suite: parity fixtures plus a representative real-board stress wave.
6. Record equivalent KiCadRoutingTools outcomes on the comparison fixtures. Compare connectivity,
   DRC, route scope, and feature behavior, not byte-identical copper.

Exit gate: every parity item is present in the manifest with `present`, `partial`, `missing`, or
`intentional-stronger-alternative`, and the baseline can be replayed from one command.

### Milestone 1 — Unified workflow, configuration, and route selection

Priority: P0.

1. Define one versioned `FluxWorkPlan` schema for route, fanout, pair, bus, plane, tuning, cleanup,
   validation, and placement steps. Bind it to the source revision and policy identities.
2. Add selectors for exact net names, glob inclusion/exclusion, net classes, footprints, declared
   placement groups, and touching/internal group scope.
3. Add explicit modes for preview, new routing, force-reroute selected nets, and allowlisted rip-up
   of pre-existing nets. Never infer permission from a wildcard; locked copper remains immutable.
4. Make every CLI option serialize to the work plan and round-trip without semantic drift.
5. Add plan dry-run, capability preflight, deterministic replay, stop-token cancellation, and a
   structured final summary.
6. Preserve telemetry defaults and expose active step, selector expansion, policy identity, and
   exact acceptance state.

GPU work: compile expanded route scopes into device-resident net/resource masks and reuse them
across all plan steps.

Exit gate: the same saved plan produces the same ordered proposals and acceptance tuple through CLI
and KiCad orchestration, with no board mutation in preview mode.

### Milestone 2 — Complete rule, stackup, and geometry fidelity

Priority: P0.

1. Extend strict import and canonical policy to cover every geometry needed by the selected parity
   corpus: Edge.Cuts lines/arcs and cutouts, rotated and supported custom pads, rule areas, plated
   holes, enabled copper layers, via spans, existing zones, copper text/graphics that affect
   routing, and locked objects.
2. Capture per-net/per-layer width, clearance, via style, hole-to-hole, copper-to-edge, differential
   pair, length/skew, and stackup dielectric/copper-thickness data through official KiCad data or a
   strict source-bound snapshot.
3. Represent unsupported region-dependent or pairwise rules explicitly. Do not replace them with a
   global minimum while claiming exactness.
4. Add fabrication-capability sidecars as reviewed constraints. A standard/advanced tier may select
   allowable geometry but may not rewrite project DRC floors silently.
5. Keep resolved routing rules cached on both host and device by immutable rule identity.

GPU work: prepare device-resident obstacle primitives, per-layer masks, SDFs, clearance matrices,
and boundary indices once per source/policy revision.

Exit gate: every selected feature fixture imports strictly or stops with an attributed unsupported
semantic; represented rule queries agree with KiCad on a focused matrix.

### Milestone 3 — Core routing-strategy parity

Priority: P0.

1. Add MPS evidence from component-boundary ordering and MST-segment crossings. Use it to seed route
   portfolios and crossing pressure; do not replace staged scarcity priority or PathFinder.
2. Add inside-out BGA ordering as another deterministic portfolio.
3. Retain original/source order and explicit user order as reproducible portfolios.
4. Add MPS-derived layer/stub alternatives to regional template enumeration.
5. Implement direction-aware future-terminal demand, routed-track spreading, cross-layer vertical
   alignment, and BGA-edge pressure as typed fields. Residual capacity and critical cuts determine
   whether a proximity term attracts, repels, or remains neutral.
6. Add explicit per-layer costs and straightness/turn preferences to portfolio objectives while PH
   length, curvature, and minimum-radius terms remain the final geometric objectives.
7. Import user-drawn guide polylines as soft corridor priors and user-layer closed polygons as hard
   reviewed keepout proposals. Guidance can never make an otherwise legal topology unavailable.
8. Preserve longer coarse paths and river alternatives as late congestion fallbacks.

GPU work: crossing/interference matrices, terminal/BGA proximity fields, guide distance fields,
critical-cut demand, portfolio scoring, and compatible batched global searches.

Exit gate: focused boards demonstrate each strategy changes the intended evidence, all strategies
retain exact legality, and at least one hard crossing fixture improves without regressing the
frontier tier.

### Milestone 4 — Conflict-directed rip-up and recovery parity

Priority: P0.

1. Add cheap forward/reverse feasibility probes whose blocked frontier is attributed to exact route
   owners and immutable obstacles.
2. Preserve validator-named blockers ahead of inferred blockers.
3. Add bounded blocker rankings equivalent in outcome to count, endpoint proximity,
   both-directions, and soft min-cut evidence.
4. Integrate that evidence with Flux's existing history, hard-capacity repair, rip-up memory, and
   large-neighborhood destruction ladder.
5. Escalate from direct conflicts through adjacent users, portal occupants, regional windows, and
   upstream/downstream routes. Keep differential pairs and buses atomic.
6. Store former route corridors as expiring avoidance history so a victim does not immediately
   reclaim the same topology.
7. Add protected-net policy for tuned, paired, plane, and user-protected routes. Explicit exact-net
   selection is required to remove protection; it never overrides a KiCad lock.
8. Restore a rejected destruction transaction completely, including copper, via reservations,
   tuning metadata, and pressure ownership.

GPU work: frontier extraction, owner intersection histograms, soft min-cut probes, former-corridor
distance fields, and blocker score reduction.

Exit gate: N+1 blocker fixtures recover routes without losing restored copper, and repeated retries
demonstrably explore different corridors rather than oscillating.

### Milestone 5 — Package-aware BGA/QFN fanout

Priority: P0 because fanout quality controls every later global route.

1. Analyze package-local pad geometry without relying on reference prefixes. Detect BGA lattice,
   pitch, holes, population, perimeter/interior balls, QFN/QFP edges, exposed pads, and rotation.
   Use actual reviewed body/courtyard geometry for package exclusion rather than inventing a BGA
   rectangle when exact source geometry exists.
2. Generate bounded BGA channel, dogbone, under-pad, and direct same-net terminal templates.
3. Generate QFN/QFP perpendicular, two-leg, under-pad, via-in-pad-authorized, and interior-pad
   templates.
4. Treat fanout as a regional multi-terminal packing problem: jointly select escape direction,
   layer, lane, via site/span, adjacent same-net join, and differential-pair adjacency.
5. Balance layers only after scarcity, critical cuts, return path, pair order, and terminal survival.
6. Reuse compatible existing same-net vias and plated through-hole pads.
7. Add exposed-pad thermal-via-array proposals with explicit fabrication policy.
8. Feed accepted fanout reservations into the global portfolio rather than committing independent
   stubs before routing.
9. Add bounded decoupling-cap repair proposals driven by foreign-via overlap, same-net electrical
   proximity, placement groups, and reviewed placement envelopes. Accept only through the complete
   integrated route gate.

GPU work: one thread per terminal/template/site for static legality and score terms, batched
terminal contention matrices, via interaction bitsets, escape wavefronts, and layer-balance
reductions. CPU performs joint assignment and exact certification of top-K bundles.

Exit gate: generic rotated BGA and QFN fixtures complete fanout with no opens/shorts, dense under-pad
fixtures retain legal escapes, and a fanout-plus-route board passes external KiCad DRC.

### Milestone 6 — Differential-pair and bus parity

Priority: P1.

1. Complete pair terminal classification for existing stubs, bare pads, different-layer launches,
   three-plus-terminal chains, electrically short legs, and hybrid coupled-middle/single-ended legs.
2. Carry endpoint pose and preferred tangent into corridor search and PH interpolation. Use a
   curvature-aware admissible lower bound where useful; do not add a separate Dubins-grid copper
   representation.
3. Jointly select pair escape, center topology, polarity/order, layer sequence, synchronized via
   sites, approach/exit run-ups, and return-via sites.
4. Enforce gap, skew, minimum radius, U-turn/inflection policy, phase continuity, and return-path
   constraints throughout regional and materialization loops.
5. Add split-ground-domain analysis. A signal/pair may receive return vias only on its resolved
   domain; ambiguous domains must be reported or explicitly selected.
6. Add bounded target-assignment proposals using crossing, length, and pin-function policy. Logical
   P/N or target swaps require an explicit source-bound permission, a reversible schematic change
   set, and human approval before becoming authoritative.
7. Add bus-candidate detection as a proposal generator using terminal clustering, naming, and
   crossing evidence. It emits a reviewable explicit ordered-bus sidecar; it never silently creates
   an authoritative bus.
8. Complete multi-channel bus ordering, middle-out escape, neighbor/river cohesion, synchronized
   transitions, and joint exact acceptance.

GPU work: pose/template expansion, tangent/curvature scoring, pair/bus envelope collision matrices,
target-assignment cost matrices, return-via site scoring, and joint quantized shortlist assessment.
Hungarian or branch-and-bound assignment remains CPU-side unless measured matrix size justifies a
device implementation.

Exit gate: bare-pad, multipoint, hybrid, multilayer, swapped-policy, return-domain, and ordered-bus
fixtures all pass connectivity, pair/bus constraints, and external DRC atomically.

### Milestone 7 — Power traces, planes, repair, and via stitching

Priority: P1.

1. Add reviewed power-policy sidecars describing source/sink/pass-through/shunt roles, expected
   current, allowed temperature rise, preferred plane/trace treatment, and net-specific widths.
2. Implement continuous or bounded stepped power neck-down transitions with exact current-capacity
   and fabrication-policy reporting. A narrow escape remains explicit width state.
3. Generate proposed single-net and multi-net plane zone outlines. Multi-net ownership may use
   Voronoi/weighted-distance evidence, but KiCad refill determines physical copper.
4. Model predicted fill only for search and scoring. After each zone proposal, refill a separate
   candidate in KiCad, strictly re-import actual fill, and invalidate stale predictions.
5. Build an actual-fill connectivity graph containing pads, through holes, filled components,
   islands, necks, and layer transitions.
6. Repair disconnected regions through jointly selected pad taps, region joins, layer changes, and
   blocker-aware local rerouting. Do not rip protected plane or tuned nets without authority.
7. Generate exposed-pad thermal arrays, area stitching lattices, and board-edge fences. Count an
   existing same-net via/PTH as coverage and enforce fill-thickness, edge, drill, and clearance
   constraints.
8. Estimate plane resistance, current density, bottleneck width, thermal-via coverage, and maximum
   supported current with explicit model assumptions.
9. Integrate single-ended and pair return-via requirements with plane-domain connectivity.

GPU work: zone masks, weighted Voronoi ownership, connected-component labeling, distance transforms,
fill-neck detection, resistor-network/PDE approximations, stitching coverage, and candidate-site
scoring. CPU/KiCad retain exact zone syntax, refill, connectivity, and DRC authority.

Exit gate: plane creation, shared-layer planes, region repair, pad welding, thermal arrays, area
stitching, edge fencing, and split-ground return paths pass refill, re-import, connectivity, and
external DRC fixtures.

### Milestone 8 — Impedance, length, and time matching

Priority: P1.

1. Parse stackup geometry and dielectric properties with provenance. Report assumptions whenever
   source data is absent; assumptions cannot satisfy a hard electrical constraint silently.
2. Add microstrip, stripline, and coplanar single/differential impedance calculators with a clear
   applicability envelope and comparison against KiCad-defined impedance profiles where available.
3. Resolve per-layer track width and pair gap from an explicit target, then pass that geometry into
   global capacity, fanout, via allocation, PH offsets, and DRC.
4. Add explicit source-bound length/time match groups. DDR naming may produce a review proposal, not
   an accepted electrical classification.
5. Measure horizontal geometry, native arcs, PH analytic length, via barrels, pad-to-route joins,
   and layer-dependent propagation delay consistently.
6. Generate smooth PH-native trombone/serpentine alternatives over certified free runs. Tune
   multiple nets/pairs jointly so added copper does not consume another member's only tuning area.
7. Support pair-to-pair, intra-pair, multi-run, multilayer, and AC-coupled extended-net matching.
8. Protect accepted tuned routes and retain their electrical declaration through reroute.
9. Add impedance, physical length, propagation time, skew, tuning reserve, and residual-error reports.

GPU work: free-run discovery, thousands of tuning-pattern candidates, analytic PH evaluation,
SDF/route clearance, candidate conflict bitsets, combination scoring, and top-K reduction. CPU
exact-checks selected patterns and the final coupled transaction.

Exit gate: impedance-controlled, DDR group, multilayer delay, intra-pair, and AC-coupled fixtures
meet their declared tolerances after KiCad materialization and DRC.

### Milestone 9 — Placement intent and parity workflows

Priority: P1.

1. Define a strict versioned design-brief schema for board purpose, user-facing connectors, required
   edges/bands, fixed parts, access directions, assembly notes, and explicitly unknown facts.
2. Compile the brief into existing source-bound placement groups, regions, relative constraints,
   move priorities, and new connector-access policy. Compilation is deterministic and reports
   declared, inferred, unknown, and withheld fields separately.
3. Add a floorplan grader for overlap, containment, permitted side/orientation, edge intent,
   connector access, net affinity, decoupling proximity, terminal escape lanes, via slots, corridor
   cuts, and plane fragility.
4. Add intent-driven seed proposals and a bounded deterministic placement portfolio containing the
   immutable input, analytical seeds, route-pressure repairs, swaps, group moves, and controlled
   perturbations.
5. Keep feasibility-first candidate comparison and the exact routed Pareto archive. Proxy scores
   only screen candidates; they never accept placement.
6. Add violation-directed minimal-move reconstruction for illegal source placements and an explicit
   lock advisor whose output remains a review proposal.
7. Integrate BGA/QFN fanout feasibility and decoupling-cap cleanup into placement evaluation.
8. Produce placement heatmaps and before/after renderings with route-pressure attribution.

GPU work: density/electrostatic fields, overlap/containment broadphase, HPWL and crossing matrices,
escape-lane/via-slot fields, plane-fragility estimates, and concurrent proxy evaluation of candidate
waves. CPU owns constraints, proposal construction, stable reduction, and full reroute acceptance.

Exit gate: seeded, perturbed-recovery, connector-edge, group/relative, BGA-decoupling, and illegal-
placement repair fixtures improve or preserve every hard term and pass a fresh complete route.

### Milestone 10 — Cleanup, verification, and KiCad-oracle repair

Priority: P1.

1. Expose diagnostic commands for connectivity, DRC, pad geometry, orphan stubs, redundant cycles,
   overlapping vias, malformed/very short copper, non-preferred angles, capacity, channels,
   terminal pockets, impedance, and per-net forensics.
2. Make every diagnostic consume the same prepared geometry, connectivity, and rules as routing.
   There must not be a lenient checker and a stricter hidden acceptance path.
3. Add connectivity-preserving simplification, gap snapping, redundant-loop removal, duplicate-via
   removal, dead-tail removal, and PH/arc refitting as board-wide proposed transactions.
4. Compute before/after connected components and prove that cleanup creates no new open, orphan,
   short, rule violation, or protected-copper mutation.
5. Parse KiCad DRC unconnected-item evidence into exact source-bound repair requests. Route only the
   reported missing links, then rerun complete connectivity and DRC; KiCad evidence cannot exempt
   other rules.
6. Add opt-in teardrop policy for pads and vias using native KiCad semantics where possible. Refill
   and DRC the complete candidate.
7. Add explicit cleanup proposals for movable copper text or routing-relevant graphics. Preserve
   content, layer, orientation, source identity, and clearance, and never move ordinary annotation
   merely because it is visually inconvenient.
8. Produce stable machine-readable diagnostics and corresponding viewer overlays/analysis PNGs.

GPU work: shared physical broadphase, connectivity edge generation, cycle candidates, orphan/gap
proximity, cleanup candidate scoring, and diagnostic overlay rasterization. CPU exact predicates and
KiCad remain authoritative.

Exit gate: deliberately damaged fixtures are detected and repaired only when the complete exact
transaction is cleaner; clean fixtures remain byte-stable under check-only workflows.

### Milestone 11 — KiCad UX, plans, rendering, and optional AI adapters

Priority: P2 after core feature behavior is qualified.

1. Add a thin KiCad 9/10 orchestration plugin with Route, Advanced, Differential/Bus, Fanout, Planes,
   Placement, Validation, and Plan tabs.
2. Read the live selection, expand it through the canonical selector, save a source snapshot, launch
   the external Flux process, attach the viewer, support cancellation, and apply only an exactly
   certified routing transaction as one unsaved undo step.
3. Keep placement, logical swaps, zone proposals, and other high-impact changes preview-only until
   their dedicated live transaction boundaries are proven.
4. Persist settings through the existing platform-standard viewer/plugin configuration path. The
   plan file remains the authoritative portable configuration.
5. Retain the telemetry WebSocket as observer-only. Process launch/cancel/apply uses a separate local
   orchestration protocol with source revision and capability checks.
6. Add deterministic static PCB/copper/placement PNG export and trace-to-MP4/GIF rendering. Reuse
   recorded snapshots; do not rerun routing to create a movie.
7. Expose route plans, failure diagnoses, board review, and source evidence as documented JSON APIs.
8. Allow optional external AI tools to author or explain a `FluxWorkPlan`. Validate the result with
   the same schema and capabilities as any human-authored plan; the AI receives no routing authority
   and is never required to use Flux.

GPU work: viewer/static rasterization, video frame generation, large-overlay filtering, and optional
visual heatmaps. UI orchestration itself remains CPU/event-loop work.

Exit gate: CLI and plugin execute the same serialized work plan, cancellation leaves the source
untouched, settings round-trip, replay is deterministic, and telemetry on/off does not change the
routing result.

### Milestone 12 — Parity qualification and release gate

Priority: P0 for declaring completion; performed incrementally throughout all milestones.

1. Run each focused parity fixture when its capability changes.
2. Run all permanently qualified tiers and the currently unlocked frontier tier after any routing,
   geometry, rules, GPU, or acceptance change. Permanently unlock the next tier only after its
   predecessor remains qualified.
3. Always write source/copper/spline/PH/via/conflict/erosion analysis PNGs and a diagnostic telemetry
   trace for tier runs, including failed and near-result candidates.
4. Run representative KRT stress boards by feature class rather than all 443 boards per edit:
   - simple two-layer and four-layer routing;
   - dense BGA/QFN escape;
   - differential pairs and buses;
   - planes, power, and return paths;
   - length/time/impedance;
   - placement recovery;
   - complex outlines, keepouts, and existing copper.
5. Run a full corpus-scale A/B wave only for milestone promotion and release. Freeze the executable,
   config, source corpus, GPU driver/device, and grader for the complete wave.
6. Compare Flux against both its previous qualified release and pinned KiCadRoutingTools on common
   source boards. Report completion, DRC, opens/shorts, length/skew, vias, copper length, runtime,
   peak host/device memory, and unsupported semantics.
7. Repeat representative runs to measure variance. A randomized portfolio carries an explicit seed;
   acceptance and reduction remain deterministic.
8. Preserve every source board byte and make artifact directories write-once.

Exit gate: every parity-manifest item is qualified or explicitly documented as a stronger Flux
alternative; every routing tier is qualified; no selected stress category has an unexplained
feasibility regression; the performance gates pass; and all final boards pass external KiCad DRC.

## 8. Cross-cutting implementation requirements

### Diagnostics and telemetry

Every new capability emits semantic events at decision boundaries, not hot-loop probes. Required
information includes:

- work-plan step and source-policy identity;
- selector expansion and protected/locked exclusions;
- fanout/package classification and terminal alternatives;
- ordering/portfolio evidence and selected topology;
- rip-up blocker evidence, destruction radius, and restored objects;
- pair/bus pose, order, via, gap, skew, and return-domain decisions;
- plane ownership, refill revision, island/neck connectivity, and stitching coverage;
- impedance/length/time target, measured value, reserve, and residual;
- placement intent terms and exact rejection reason;
- GPU batch sizes, queue delay, cache hits, shortlist rates, and fallback/withholding reasons;
- exact validation state and final external KiCad DRC status.

Normal telemetry coalesces state. Diagnostic traces retain bounded candidate summaries. Trace mode is
reserved for targeted algorithm work. Queue overflow may drop observer data but can never delay or
change routing.

### Testing discipline

- Keep compiled unit tests limited to risky invariants: source binding, exact geometry boundaries,
  deterministic GPU/CPU differential checks, transaction atomicity, coupled groups, and
  serialization compatibility.
- Remove redundant tests once their behavior is permanently covered by a stronger invariant or
  board-level qualification.
- Use board fixtures for behavior involving routing, fanout, planes, tuning, placement, export, or
  KiCad DRC.
- A passed test becomes part of the qualified baseline; do not repeatedly add narrow tests for the
  same mechanism.
- Every GPU kernel has a small CPU reference comparison over adversarial fixed cases. Production
  routing still requires the GPU.

### Source and licensing discipline

- Treat KiCadRoutingTools as a behavioral and algorithmic reference. Reimplement Flux-native C++/GPU
  mechanisms around the canonical IR.
- Record the upstream commit and referenced files in research/ADR notes.
- Preserve attribution and licenses for any imported board fixture, data file, or directly adapted
  material.
- Do not mix AGPL slicer code or other incompatible source into Flux. Conceptual use of published
  algorithms must be documented independently.

## 9. Delivery order

The dependency-respecting route to useful parity is:

```text
M0 parity contract/baseline
  -> M1 work plans and route scope
  -> M2 rules/geometry fidelity
  -> M3 global strategy + M4 conflict recovery
  -> M5 fanout
  -> M6 pairs/buses
  -> M7 power/planes
  -> M8 impedance/length/time
  -> M9 placement intent
  -> M10 cleanup/verification
  -> M11 KiCad UX/rendering/optional AI
  -> M12 full parity qualification
```

M12 runs continuously rather than only at the end. When a milestone exposes a lower-level routing
failure, repair the shared global/regional/detail mechanism first and resume the feature milestone;
do not add a feature-specific legality bypass.

## 10. Completion definition

Feature parity is complete when:

- the parity manifest covers the pinned upstream public feature set;
- every feature has a Flux command/work-plan representation and observable result;
- all explicitly supported boards retain strict import and source-preserving export;
- all route, fanout, pair, bus, plane, tuning, cleanup, and placement transactions pass their hard
  constraints and final KiCad validation;
- every progressive tier is permanently qualified;
- the selected real-board stress wave shows no unexplained regression from the previous Flux
  release and records a comparison with the pinned reference;
- production routing fails clearly without a supported GPU and uses the shared runtime when enabled;
- telemetry, replay, plugin orchestration, and optional AI authorship cannot affect acceptance;
- documentation distinguishes implemented, qualified, experimental, proposed, and unsupported
  behavior without overstating any result.
