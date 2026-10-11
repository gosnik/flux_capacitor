# Static conflict search, reachability and breathing space

Status: shared-map connected-component experiment in implementation. A bounded static HC-CBS-inspired composition integration is also in implementation after explicit
user approval. No time-expanded copper router is enabled.

## Findings and ranking

| Concept | Fit to Flux | Benefit and cost | Decision |
| --- | --- | --- | --- |
| Cached flood-fill components | Repeated pressure searches use the same immutable layer raster | Linear build/storage; constant-time disconnected-pair check; directly avoids repeated fruitless Dijkstra work | Implement in shared terrain preparation |
| CBS-style conflict branching | Small connected groups of physically conflicting traces | Can reconsider which exact route/lane/via assignment yields; exponential conflict tree requires strict regional bounds | Retain existing exact-assignment no-good mechanism; investigate new branching only against a measured unresolved conflict |
| Reachable area / breathing space | Preserving future pad escapes and scarce channels | Useful diagnosis and eventual ordering signal; area alone does not measure capacity or alternative exits | Record component cell counts; do not introduce an unvalidated area penalty |
| Time-expanded graphs / SIPP | Moving obstacles with predicted occupancy intervals | Adds state and reservations; temporal separation cannot legalize overlapping permanent copper | Do not add time to final-copper path states |
| Voronoi area control | Demand-to-capacity allocation, not competitive territory ownership | Existing shared Voronoi corridors already model resources; a new exclusive territory partition could block valid interleaving | Use existing corridor width/demand evidence; no arbitrary net-owned territory |

## Source interpretation

[CBS (Sharon et al., AAAI 2012)](https://ojs.aaai.org/index.php/AAAI/article/view/8140/7998)
uses a high-level conflict constraint tree with low-level single-agent searches. Its temporal
collision rules and optimality guarantees do not transfer automatically to permanent PCB traces.
A static adaptation must express incompatible copper envelopes or exact assignment combinations,
retain other routes, and replan the implicated choice. It must not turn an unsuccessful bounded
continuous fit into proof that an entire corridor or topology is impossible.

[SIPP (Phillips and Likhachev, 2011)](https://www.cs.cmu.edu/~maxim/files/sipp_icra11.pdf)
compresses collision-free time intervals for moving-obstacle planning. Copper that is placed
remains occupied in the final board. A wait action cannot make two final trace envelopes overlap
legally. Transaction epochs and immutable snapshots are appropriate for coordinating speculative
workers and rip-up/reroute, but are not another geometric axis or a new DRC exemption.

[LaValle, roadmap connectivity](https://lavalle.pl/planning/node265.html)
explains preserving free-space connectivity with a graph decomposition. Flux's sampled map has a
weaker guarantee: connectivity is exact only for the chosen raster and its adjacency. Narrow legal
continuous channels can disappear in a coarse shared map; same-net pad/plane ownership also
changes each connection's actual free space. Shared component labels may therefore skip a
*pressure-field search*, never certify that a real connection cannot route.

## Repository evidence

- `src/regional_router.cpp`: `RegionalNoGoodClauses`, exact template/via/lane identities and
  conflict-directed master reruns already provide bounded static conflict learning.
- `regional_assignment_is_no_good_certificate`: monotonic preflight proof is distinguished from
  unknown/budget-limited PH or native realization failure.
- `route_prepared_navmesh_flood_fill`: existing static reachability with dynamically closed cells,
  not time-indexed movement.
- `src/global_router.cpp`: terminal escape reservations, local planar escape pressure, shared
  Voronoi capacity and critical-cut costs already preserve parts of future route freedom.
- `construct_copper_scaffold`: exact survivor-preserving victim exchange and transactional commit
  already supply coherent multi-route repair.
- New `BoardTerrainGuidanceCache`: geometry is reused, but disconnected pressure endpoints
  previously repeated bounded Dijkstra searches on an unchanged mask.

## Smallest experiment and acceptance

Compute component labels and cell counts once per immutable layer map, using exactly the same
8-neighbour adjacency and no diagonal corner cutting as pressure search. Before allocating a
pressure search, compare the bound access cells' component IDs. Different labels skip only that
advisory demand search, preserve its disconnected diagnosis and leave the work allowance for
other demands. Changed geometry rebuilds labels; pressure-only refresh reuses them.

Synthetic test: a large disconnected demand precedes a reachable demand under one shared work
budget. The disconnected pair should use zero search expansions, and the reachable demand should
still deposit pressure. Also test diagonal-only adjacency, a one-cell bridge, immutable cache
reuse and cancellation. Measure component memory/build time, skipped searches, pressure search
expansions and total preparation time. Full routing tier results remain authoritative for any
claim of improved routing quality.

Component area is measured in raster cells, not geometric packing capacity. A large chamber with
one narrow exit can be more constrained than a small chamber with two exits. Any later breathing
space heuristic should consider width-qualified exits, alternative layer/via access and remaining
demand before influencing candidate ordering. Hard rejection requires a sound exact certificate.

## HC-CBS and hierarchical composition

[Lee et al., Parallel Hierarchical Composition CBS (2021)](https://parasollab.web.illinois.edu/papers/Lee-phccs-2021/)
and the [authors' algorithm overview](https://parasollab.web.illinois.edu/research/multi-agent-path-finding/)
extend CBS by solving subsets, then composing them while retaining paths, constraints and costs.
PHC-CBS uses structured layers of parallel jobs; DPHC-CBS schedules compositions as work completes.
This is more than sorting conflicts or increasing a region's net count: it requires retaining
subproblem search states so larger problems can start from combinations of smaller ones.

For Flux, the plausible adaptation is a bounded hierarchy of permanent-copper assignments:
solve strongly interacting small groups first, retain several valid local alternatives, compose
groups, and resolve newly exposed cross-group copper conflicts. Cache keys must include the
physical obstacle snapshot, boundary attachments, rules, net/layer scope and assignment identity.
A no-good from one scene cannot be imported as a universal constraint in a changed scene.

The existing causal request groups, survivor-preserving transactions, exact regional assignment
clauses and shared CPU worker budget provide useful foundations. They do not yet constitute
HC-CBS: there is no preserved hierarchy of constraint-tree frontiers whose combinations seed
parent searches. Reusing only the single winning child route can lose the alternative required by
its sibling. Conversely, retaining all child frontier combinations can explode memory. A bounded
lazy composition would be a heuristic adaptation without the paper's optimality/completeness
claims.

Expected value is highest when repeated small local repairs succeed individually but conflict
when combined. Costs include retained alternatives, boundary consistency, state invalidation and
cross-group exact checks. Independent subproblems may share the existing CPU allowance, while
parent admission remains deterministic and rechecks all changed copper against current survivors.
A useful micro-test has two solvable two-net groups whose initially preferred local solutions
conflict, but a retained child alternative permits a legal four-net composition. Measure reused
child work, combinations expanded, peak memory and final connectivity/DRC before extending it to
Tier 1. The user subsequently authorized adapting and applying this mechanism with adaptive stale-state
handling. Implementation uses a bounded finite-assignment composition portfolio ahead of the
existing exact regional gates; validation and benchmark results are recorded separately.


Implementation boundary: the static HC-inspired composer constructs bounded two-fragment child
assignments and retains several alternatives per child. It orders interactions and lazily
composes finite child choices, then submits seeds to the existing current-scene regional gates.
The cache stores equality-guarded scene and candidate identities with a conservative deep
retention estimate. Changed copper triggers compatibility revalidation; changed endpoint/rule
identity invalidates proposals. A poor stale-validation yield selects rebuilding. No temporal
constraint tree, optimality guarantee, or stale hard no-good reuse is claimed.

Production ablation: `--no-hc-composition`. The default is enabled and can be made explicit with
`--hc-composition`. JSON `regional_hc_*` counters include generated/admitted seeds, work bounds,
compatibility checks and stale/cache outcomes. Downstream exact copper admission is unchanged.
