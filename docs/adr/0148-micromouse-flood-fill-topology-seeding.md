# ADR 0148: Micromouse-style flood-fill topology seeding

## Status

Accepted after ADR 0147.

## Context

Flux already prepares immutable clearance-expanded rectilinear NavMesh cells and searches their
adjacency before funneling a centerline. The cell graph is also a natural micromouse maze: absent
neighbors are closed walls, free neighbors are passages, and integer distance labels provide a
cheap deterministic reachability field. Using that field explicitly supplies fast multi-goal
terminal/corridor seeding without adding continuous geometry to PathFinder.

## Decision

`route_prepared_navmesh_flood_fill` performs a bounded reverse breadth-first flood from a sorted,
deduplicated set of legal goal cells. Each free cell is enqueued at most once. Search stops when
the start cell is reached or the configured expansion bound is exhausted.

The route is reconstructed from the start by choosing the first legal neighbor in fixed cardinal
order whose label is exactly one lower. It must terminate at a bound goal cell. Existing guard-cell
construction, portal creation, funnel/string pulling, source-boundary certificates, and fallback
visibility pulling then operate on that selected corridor unchanged.

`route_prepared_navmesh` delegates its single-goal request to the flood implementation. Therefore
global static B-spline seeds, regional alternative templates, same-layer detours, and detailed
spline corridors all receive the same deterministic topology seed. The API also accepts several
goals for later terminal-fanout and via-portal allocation.

The result reports whether flood fill was used and how many cells were visited. Global route JSON
aggregates flood routes and visited cells alongside existing NavMesh metrics.

The flood has no routing authority. It cannot cross a missing cell, consume capacity, override
PathFinder history, select a regional lane order, authorize a via, or accept copper. PathFinder,
regional physical packing, B-spline/PNS optimization, quantization, connectivity, represented DRC,
and external KiCad DRC retain their existing roles.

## Consequences

The previous unit-cost priority-queue cell search is replaced by linear-time reverse BFS on the
same immutable graph. Multi-source seeding no longer requires repeated endpoint searches. Stable
goal sorting and neighbor ordering make reconstructed corridors independent of queue allocation or
worker timing.

A bounded failure is distinguishable from a disconnected completed flood. Dynamic obstacle or
capacity changes create a new prepared-domain identity and hence a new flood; incremental label
repair is deliberately deferred until profiling demonstrates that full affected-domain flooding is
materially expensive.
