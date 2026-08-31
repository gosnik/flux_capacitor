# Future implementation plan: routing-friendly RCQ placement

## Purpose

Implement an early, bounded placement phase for resistors, capacitors, and transistors (RCQ) that
improves downstream routability without weakening Flux's exact placement, routing, connectivity,
or DRC acceptance boundaries.

This is an extension of the existing placement stack rather than a separate placer. Flux already
has deterministic analytical placement, exact courtyard and policy legalization, route-pressure
attribution, incremental candidate screening, full-route acceptance, and staged
resistor/capacitor/transistor move authorization. The missing capability is component-role-aware
placement backed by a fast, orientation-sensitive routability estimate.

The phase belongs after canonical import and policy validation, but before the production routing
transaction. "Early" does not mean routing-blind: the inner loop uses a cheap placement-time route
model, while shortlisted placements still require the authoritative global/regional/detailed
routing pipeline.

## Design principles

- Preserve the immutable source `Board`; every placement remains a proposal on a board copy.
- Treat locks, board geometry, placement envelopes, keepouts, groups, regions, relative placement,
  permitted sides, and permitted rotations as hard constraints.
- Never infer hard electrical or mechanical intent solely from reference prefixes, proximity, or
  connectivity.
- Use reference prefixes only to suggest component classification and move priority.
- Distinguish component type from component role. For example, a decoupling capacitor, AC-coupling
  capacitor, and bulk capacitor require different placement objectives.
- Use physical routing demand derived from resolved width and clearance rather than unit-demand
  proxies.
- Use smooth or weighted objectives to generate candidates, but use feasibility-first
  lexicographic comparison to select them.
- Require a fresh authoritative route and exact acceptance before a placement can become a normal
  candidate.
- Keep KiCad placement output file-based and reviewable until whole-footprint live updates are
  demonstrated to preserve schematic linkage safely.

## Existing foundation

The implementation should reuse these current boundaries:

- `place_global` for deterministic analytical placement and exact legalization;
- `optimize_integrated_placement_routing` for priority-scoped placement proposals, incremental
  screening, fresh-route acceptance, and Pareto retention;
- `validate_board` and the placement-policy sidecars for exact mechanical and designer-policy
  gates;
- global-router pad-access, obstacle-attribution, sampled/corridor/Voronoi capacity, and routing
  rule resolution;
- detailed routing, connectivity analysis, represented copper DRC, source-bound export, and
  external KiCad DRC; and
- the existing resistor, capacitor, transistor, other, IC, MCU, SoC, and connector move-priority
  policy.

The existing prefix classifier changes search authorization only. It must not be treated as proof
of electrical role.

## Required design intent

### Physical identity additions

Extend the canonical import identity needed to bind schematic intent to physical pads:

- retain each pad number or name separately from its stable object ID;
- retain footprint value and library identifier where available;
- retain stable schematic-path or symbol-instance identity where available;
- retain provenance for every imported or externally supplied field; and
- bump the board schema and extend validation, equality, export round trips, and fixtures.

These additions must not silently make schematic metadata authoritative for physical geometry.

### Source-bound placement-intent policy

Introduce a strict, versioned `PlacementIntentPolicy`, initially loaded from a source-revision-bound
sidecar. It should be normalized into stable footprint and pad IDs before optimization and contain:

- component class and functional role;
- pad roles such as input, output, gate/base, source/emitter, drain/collector, power, and return;
- ordered path relationships such as `driver -> series resistor -> receiver`;
- pad-to-pad or footprint-to-region proximity limits;
- loop, branch-stub, and maximum connection-length limits;
- same-side, polarity, and allowed-orientation requirements;
- required or preferred escape directions;
- reserved local via-site or routing-channel requirements;
- electrical grouping and anchor relationships;
- hard versus soft status; and
- explicit provenance and review status.

The first version should reject missing, duplicate, stale, ambiguous, or unsupported identities.
Unknown roles may use the generic routing objective, but they must not receive inferred hard
constraints.

### KiCad integration boundary

For KiCad 10, do not depend on schematic IPC for this data. Use a reviewed sidecar first, with an
optional later `.kicad_sch` reader for Reference, Value, Footprint, symbol properties, hierarchy,
and pin identity. A future KiCad version with suitable schematic IPC may replace the file adapter,
but not the normalized core policy.

Flux-specific schematic properties may eventually generate sidecar suggestions. They still need
source binding and explicit review before becoming hard intent.

## Role-aware placement behaviour

### Resistors

Generate and score candidates that can:

- place a series resistor on the geometric path between its annotated endpoints;
- preserve source-to-sink pad order;
- align the two-pad axis with the likely entry and exit directions;
- minimize source and receiver branch stubs;
- place pull-up, pull-down, shunt, and feedback parts near their annotated controlling pin;
- preserve reviewed divider and feedback-loop relationships; and
- avoid packing passive rows into walls across scarce routing channels.

### Capacitors

Generate and score candidates that can:

- place a decoupler on the same side as its load when required;
- minimize the complete power-pin to capacitor to return-via loop;
- reserve a legal return-via site and pad escape;
- distinguish local decoupling from bulk storage;
- place coupling and filter capacitors in their ordered signal path;
- preserve reviewed polarity and orientation; and
- avoid moving a capacitor closer geometrically while making its physical escape or return path
  worse.

### Transistors

Generate and score candidates that can:

- enumerate every explicitly permitted discrete rotation;
- point gate/base access toward the control network;
- point source/emitter and drain/collector access toward their respective current-path regions;
- score the control loop and power loop separately;
- preserve pin ordering and avoid forced local crossings;
- observe thermal, copper-area, creepage, and assembly policies when supplied; and
- retain the later move priority appropriate for a more disruptive component.

Side changes should remain disabled in the first implementation. They enlarge the search space and
introduce assembly, thermal, via, and review consequences that are not needed to prove the core
placement model.

## Placement-time routability estimator

Add a bounded `PlacementRoutabilityEstimator` that is cheaper than a full routing transaction and
shares the authoritative geometry and rule models wherever practical.

It should compute:

- orientation-sensitive legal pad portals and escape deficits;
- coarse connection demand over spatial bins or physical corridors;
- total and peak capacity deficit in physical units;
- same-layer pin-pair or topology-edge crossing pressure;
- net-separation pressure where a crossing model is unavailable;
- obstacle and keepout shadows;
- expected layer transitions and via demand;
- role-specific path, stub, and loop metrics; and
- affected-net deltas for a local move.

Routing demand must use each net's resolved track width and clearance envelope. Differential pairs,
routing buses, and other explicitly coupled nets must remain coupled in the estimate.

The estimator must be deterministic and bounded by explicit limits on cells, connections,
candidate states, expansions, and affected-net recomputation. It may reuse immutable free-space
indices and cache unaffected net demand, but every cache entry needs board/configuration identity
and geometric invalidation equivalent to the routing checkpoint boundary.

This estimator is advisory. It cannot report a placement as routed, connected, or DRC-clean.

## Objective and acceptance

### Candidate-generation objective

Analytical or local search may combine differentiable or weighted terms for:

- wirelength;
- density and placement halos;
- crossing or net-separation pressure;
- escape deficit;
- coarse physical overflow;
- role-specific path and loop quality; and
- source anchoring.

Weights are configuration, not acceptance semantics. Normalize dimensions before combining them,
and retain each raw dimension in telemetry.

### Placement comparison

Compare legalized early-placement candidates lexicographically by:

1. hard placement and reviewed hard-intent violations;
2. failed pad escapes and estimated unroutable connections;
3. total and peak physical routing overflow;
4. same-layer crossings or net-separation deficit;
5. soft role-specific loop and stub penalties;
6. estimated via and layer-transition cost;
7. width-aware wirelength; and
8. transform count, rotation change, and displacement from source.

Do not permit a lower soft cost to compensate for a worse hard or feasibility dimension.

### Authoritative acceptance

Only a bounded shortlist proceeds to fresh whole-board global and regional routing. A placement may
be accepted by the integrated optimizer only after:

- exact structural and placement validation;
- a fresh complete routing plan;
- detailed copper materialization;
- zero open connections;
- zero represented copper DRC violations; and
- objective improvement under the existing integrated score.

Export additionally requires source-bound round-trip verification and workflow-level external
KiCad DRC. No incomplete placement checkpoint may be exported as a normal candidate.

## Search and legalization

### Placement atoms

Extend placement atoms from translation-only states to bounded `(x, y, rotation, side)` states.
The first version fixes the side and enumerates only the explicitly allowed rotations.

Rigid placement groups and the fixed-point closure of relative constraints remain atomic. One
locked member immobilizes the complete component. An atomic action waits for its highest
move-priority tier.

### Candidate sites

Add topology-derived candidates alongside the existing deterministic grid rings:

- inline positions between annotated endpoints;
- positions orbiting a specific anchor pad or footprint edge;
- IC-perimeter sites facing the associated pin;
- sites adjacent to a feasible via region;
- channel-preserving row positions;
- mirrored or rotated equivalents; and
- legal swaps or local reorderings among comparable small components.

Candidate generation must use actual transformed pad geometry, not footprint anchors alone.

### Global and detailed phases

Use the existing analytical placer to generate one or more coarse RCQ seeds, augmented with
crossing, escape, role, and orientation terms. Then perform bounded detailed improvement using
coordinate moves, rotations, swaps, reordering, and simultaneous site assignment.

The current small simultaneous legalizer can remain the exact solver for bounded local windows.
Larger populations should be partitioned into deterministic independent or weakly coupled windows
rather than increasing one exponential assignment search without limit.

Exact legalization must remain shape-aware and enforce board/cutout containment, placement
regions, courtyards, footprint keepouts, component-owned keepouts, relative constraints, locks,
side policy, and orientation policy.

## Pipeline integration

The intended pipeline is:

```text
KiCad import
  -> structural and policy validation
  -> placement-intent normalization
  -> freeze anchors and non-RCQ footprints
  -> role-aware analytical placement seed
  -> orientation-aware exact legalization
  -> coarse routability evaluation
  -> bounded detailed RCQ refinement
  -> top-K fresh global/regional routes
  -> detailed copper, connectivity, and represented DRC
  -> source-bound, externally checked KiCad candidate
```

The unchanged source placement must always participate as a candidate. If no certified improvement
exists, retain it without claiming that an exhausted partial search proved global optimality.

## Existing routing and zone policy

For an already routed or partially routed source:

- locked tracks, arcs, vias, zones, and keepouts remain immutable physical obstacles;
- a footprint move that disconnects a pad from locked copper is illegal;
- unlocked copper on affected nets is removed and replanned;
- pad nets, obstacle-attributed nets, coupled nets, and policy closures expand the affected cone;
- retained paths and copper undergo the existing checkpoint revalidation; and
- invalid reuse falls back to fresh routing.

Integrated placement on boards with refillable zones needs an explicit transactional extension.
Moving a footprint can change pad connections, thermal relief, clearance, and pour topology. Until
placement plus zone refill can be refilled, strictly re-imported, connectivity-checked, and
externally DRC-checked as one transaction, such results must remain withheld or unsupported rather
than treating saved fills as authoritative final geometry.

## Observability

Add placement-specific telemetry and CLI JSON for:

- loaded component roles and review provenance;
- active move-priority tier and eligible footprints;
- every proposed position/orientation and its generator;
- before/after raw estimator dimensions;
- hard and soft intent satisfaction;
- pad escape counts and rejected portals;
- affected nets and cache reuse/invalidation;
- shortlist and fresh-route decisions;
- accepted/rejected move reasons; and
- budget exhaustion distinct from infeasibility or local optimality.

Every accepted transform should be explainable in terms such as `reduced failed escape`,
`removed crossing`, `reduced peak channel deficit`, `satisfied decoupling loop`, or
`reduced termination stub`.

## Implementation phases

### Phase 1 - Identity and intent contract

- Extend pad and footprint identity in the canonical IR and KiCad importer.
- Define the versioned placement-intent policy and sidecar grammar.
- Add strict source binding, stable-ID/pad binding, provenance, and validation.
- Add empty templates and reviewed examples; do not infer hard roles.
- Add round-trip and stale-policy tests.

#### Gate

- A hierarchical or duplicated-reference fixture binds every intent to exactly one footprint and
  pad, or fails closed.
- Board import/export and existing fixtures retain deterministic identity.
- Unknown or stale intent cannot influence placement.

### Phase 2 - Fixed-side role-aware candidate generation

- Scope the early placement to reviewed R/C/Q footprints.
- Implement ordered series-path, decoupler-loop, and transistor pin-direction candidates.
- Enumerate allowed discrete rotations without side changes.
- Extend exact legalization and simultaneous assignment to position/orientation states.
- Preserve the source placement as a fallback.

#### Gate

- Series-resistor and decoupler fixtures select the expected pad ordering and orientation.
- Locks, groups, regions, relative constraints, and orientation policy remain exact.
- Candidate generation is deterministic and bounded.

### Phase 3 - Fast routability objective

- Implement orientation-sensitive pad escape analysis.
- Add width/clearance-aware coarse demand and physical capacity.
- Add net-crossing or net-separation pressure and estimated via demand.
- Add incremental affected-net updates and deterministic cache invalidation.
- Integrate raw metrics into global and local RCQ placement search.

#### Gate

- A known HPWL-improving but channel-blocking move is rejected.
- A known route-improving move reduces escape or physical-deficit metrics.
- Incremental and fresh estimator results match exactly for the affected fixture set.

### Phase 4 - Integrated routed acceptance

- Shortlist early placements by the placement-time objective.
- Run fresh global/regional routing for the shortlist.
- Admit only exact detailed, connected, represented-DRC-clean improvements.
- Preserve existing placement priority, Pareto, cancellation, and budget semantics.
- Emit candidate-only KiCad output with explicit placement-review reasons.

#### Gate

- Every accepted fixture has zero opens and zero represented copper violations.
- External KiCad DRC passes for exported candidates.
- Worker-count changes do not change selected placement or output bytes.
- No estimator-only result can be exported as a normal candidate.

### Phase 5 - Scaling and richer intent

- Partition large passive populations into deterministic local windows.
- Add independent-set matching, swaps, row reordering, and bounded joint assignment.
- Add power-stage, feedback-divider, filter, and thermal/creepage policies.
- Evaluate multi-start analytical seeds and bounded parameter portfolios.
- Add schematic-derived suggestions while retaining reviewed-policy authority.
- Add transactional placement plus zone-refill qualification.

#### Gate

- Runtime and memory stay within explicit board-scale budgets.
- Larger fixtures improve routed completion or physical congestion without regressing hard intent.
- Suggested schematic intent is visibly provisional until reviewed.

## Verification fixtures

The existing 4-through-20-component placement/routing suite remains a regression baseline, but its
simple two-pad footprints do not establish role-aware quality. Add project-owned fixtures for:

- a driver, series resistor, and receiver;
- a source or load termination;
- an RC input filter;
- a fixed IC with several decouplers and constrained return-via sites;
- a BJT or MOSFET switch with a gate/base resistor and pull-down;
- a passive row blocking a narrow routing channel;
- a transistor placement with one route-friendly and one crossing-heavy orientation;
- a rigid or relatively constrained RCQ cluster;
- a partially routed board containing locked copper; and
- a zone-bearing board requiring refill after placement.

Each fixture should encode expected invariants rather than only a desired final coordinate. Check
pad order, maximum distances, loop/stub metrics, escape availability, exact placement legality,
routing completion, overflow, vias, routed length, transform cost, and deterministic provenance.

Run ablations against:

- unchanged source placement;
- the existing HPWL/density global placer;
- role-aware placement without routability terms;
- routability-aware placement without role terms; and
- the combined implementation.

## Completion criteria

The RCQ phase is production-ready only when it can, under explicit budgets:

- consume a reviewed, source-bound placement-intent policy;
- preserve every unauthorized footprint and locked object exactly;
- produce only exactly legal placements;
- improve or retain the feasibility-first routing score;
- satisfy every reviewed hard electrical placement constraint;
- complete detailed routing with zero opens and represented copper violations for accepted output;
- pass source-bound reproduction and external KiCad DRC;
- return deterministic results independent of worker completion order; and
- report cancellation, evaluation limits, and local-search exhaustion truthfully.

The first useful milestone is deliberately narrower: fixed-side resistor and capacitor placement
with explicit series/decoupling intent, discrete legal rotations, escape/congestion/crossing
scoring, exact legalization, and existing full-router acceptance. Transistor-specific power and
thermal intent, automatic schematic suggestions, scalable large-population refinement, and zone
refill should follow after that boundary is measured and stable.

## Research references

- [KiCad IPC API guidance for add-on developers](https://dev-docs.kicad.org/en/apis-and-binding/ipc-api/for-addon-developers/)
- [KiCad schematic S-expression format](https://dev-docs.kicad.org/en/file-formats/sexpr-schematic/)
- [Net Separation-Oriented PCB Placement via Margin Maximization](https://arxiv.org/abs/2210.14259)
- [Cypress: VLSI-Inspired PCB Placement with GPU Acceleration](https://doi.org/10.1145/3698364.3705346)
- [Cypress implementation](https://github.com/NVlabs/Cypress)

