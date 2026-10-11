# ADR 0170: Bounded physical conflict components

## Decision

Retain the existing global/navmesh planner, regional discrete master, XPBD/AVBD geometry,
exact copper predicates and final KiCad validation. Make their handoff use canonical physical
demands and checked copper, not a new independent routing representation.

A frozen regional job selects a bounded causal component of unresolved demands. Shared blocker
nets and same-net connections establish adjacency, not infeasibility. Least-attempted demands
receive priority; completed work on one component does not retire untouched components. Each
job remains a resumable engine kernel. The full original demand domain is reconstructed before
any whole-board acceptance; a legal subset cannot certify completion.

A separate prepared-geometry lease belongs to the retained physical scene. It is returned only
after its kernel stops, and is reused only under the exact physical dependency key. Global
history changes do not invalidate this geometry. Existing regional preparation owns its bounded
navigation/clearance caches and existing compute facilities; this change does not introduce
another GPU allocator. One optional worker, leased from the shared CPU budget, can prepare an
immutable retained-copper snapshot during global replanning. Foreground admission checks the
snapshot key and never waits for unfinished speculation.

Causal copper exchange considers bounded multi-victim components and deterministic routing
orders. Unaffected objects stay in the transaction; every previously satisfied request must
remain connected before commit. The shared expansion budget and attempt/cardinality bounds
apply to the component alternatives together. Failed rebuilt-scene searches expose additional blocker owners through retained object identity,
allowing a hidden middle blocker to join the bounded causal frontier. Search exhaustion remains
unknown, never a proof that a corridor is impossible.

Regional discrete incompatibilities must carry evidence appropriate to their scope. Definitive
packing/endpoint failures can reject an assignment; an unfinished PH search or an unsuccessful
continuous deformation cannot prove that every realisation of its topology is impossible.

The engine retains bounded regional failure summaries, selected request indices and exact oracle
results even when no whole-board plan is promoted. Current-scene frozen-region witnesses feed diagnosis-driven selection while the scaffold is
incomplete. Complete-scaffold oracle reports remain diagnostic records: their raw witnesses
must not enter the history of a different global/regional candidate without an explicit geometry
mapping. Stale regional witnesses also remain diagnostic records only. Exact checks and KiCad DRC retain
acceptance authority. No new clearance relaxation is introduced.

## Research and limits

Conflict-based search separates high-level conflict constraints from low-level paths:
https://ojs.aaai.org/index.php/AAAI/article/view/8140 . Flux uses that decomposition principle,
not the paper's time-expanded MAPF model or optimality guarantee. PCB routes occupy space
permanently, vias span layers, and route widths/rules differ.

Conservative obstacle offsets remain the geometric clearance foundation:
https://doc.cgal.org/Manual/latest/doc_html/cgal_manual/Minkowski_sum_2/Chapter_main.html .
Terrain evolution ranks alternatives; it does not certify packing. Existing corridor capacity,
lane-order and via-footprint checks precede expensive geometry. Unknown packing stays unresolved.

This is bounded heuristic joint repair, not a complete multi-commodity solver. Benchmark evidence
must establish whether larger causal components and better reuse offset their search cost.
