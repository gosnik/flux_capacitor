# Layered cell and via navmesh

Status: implemented; focused tests and sequential Tier-0/Tier-1 benchmarks completed; no measured connectivity improvement.

## Failure and hypothesis

Flux prepares a polygonal cell graph separately for each copper layer. The global router
chooses layers on a sampled grid, while regional alternatives enumerate via pairs around a
planar path. This separates the choice of a useful corridor from the choice of where to enter
it. A planar dead end therefore does not expose its reachable alternate-layer corridors in
the same search state.

The experiment adds a bounded derived graph over the existing immutable layer domains.
It does not replace the canonical board, exact clearance geometry, or physical admission.
The hypothesis is that selecting a corridor and its transitions together yields useful
regional alternatives with fewer unsuccessful via-pair searches.

## Representation and invariants

* A node identifies a physical copper layer, prepared domain, and polygonal cell.
* A planar edge carries the existing shared-face portal and aperture.
* A transition carries a via-zone identity, selected site, entry and exit layers, and the
  entire physical copper span occupied by its barrel and annuli.
* Copper layer ordering comes from the actual stackup. Algorithmic preferences must not
  reorder it or imply that dielectric material is routable copper.
* A via zone is a candidate placement region. A certified sample site does not certify
  every point in the region. Moving a via requires exact revalidation.
* Every transition must pass the current net's diameter, drill, clearance, board-edge,
  keepout, and intermediate-layer checks. Through-via occupancy includes all physical
  layers even when its electrical entry and exit are inner layers.
* Shared cells reduce repeated geometry work. A prepared graph is valid only for its
  physical scene, net and rule context; current copper admission remains mandatory.
* Cell, edge, transition and search bounds and cancellation return an incomplete/unknown
  result. They do not prove that the board is infeasible.

This is a layered volumetric connectivity representation: the planar free spaces are linked
only by manufacturable vertical transitions. It is not an unrestricted tetrahedral mesh in
which a trace can travel diagonally through the substrate.

## Integration and verification

The first consumer is regional layer-alternative generation, which already materializes
planar runs and explicit via choices. The graph must influence selected alternatives rather
than exist solely as exported diagnostic data. Existing access, packing, curve and exact
copper gates remain in force. The module may represent more general multilayer paths than
the initial consumer can materialize; report that limit explicitly.

Deterministic fixtures cover a required layer change, an obstructed intermediate layer,
invalid layer/span identities, same-layer portals, cancellation and exhausted bounds.
Run the lower-tier benchmark before the frontier tier, sequentially with telemetry, and
record actual layer alternatives selected, rejections, expansions, runtime and final DRC.
No tier is promoted from graph connectivity alone.

## Research

[LaValle's cell decomposition framework](https://lavalle.pl/planning/node273.html) motivates
reducing connected free-space cells to a searchable adjacency graph. Completeness depends on
the decomposition and endpoint binding; Flux's bounded approximate cells do not inherit an
exact planner's completeness guarantee.

[KiCad's routing documentation](https://github.com/KiCad/kicad-doc/blob/master/src/pcbnew/pcbnew_routing.adoc)
distinguishes through, blind/buried and micro vias. The graph must preserve the supported
physical via model rather than invent transitions from visual layer adjacency.

## Implemented boundary

`PreparedLayeredNavMeshDomain` references immutable islands for every supplied physical copper
layer. A layer with no cells remains in the physical span model. Directed or bidirectional
via-zone samples connect explicit layer/domain/cell nodes. Preparation and search have separate
caps; the default preparation permits 65,536 nodes and 262,144 planar edges, and search permits
262,144 transition states and frontier entries. Cancellation uses both explicit stop tokens
and the active routing-kernel deadline.

`build_alternative_layer_templates` prepares one graph across all copper layers inside the
regional search bounds. It considers at most eight sites per endpoint/layer (also limited by
the existing per-region site allowance), 1,024 sample inspections, and 128 zones. Samples
pass both exact full-span via checks and the existing via-radius domain filters before
consuming the retained-site allowance. Certification is reused only within that immutable
scene and net context. It requires exactly two directed
transitions and uses the selected middle-layer corridor with all other cells closed during
funnel realization. The selected via pair and layer are considered first in the existing
portfolio; ordinary alternatives remain available. Local conflict-bracketing bypass waves
continue to use their existing narrow search.

This consumer materializes two-via overpasses. The graph API supports general transition
counts, but the global sampled-grid allocator and arbitrary multilayer template materialization
have not been replaced. No continuous via-placement region is certified from one sample.

`--no-layered-navmesh` provides the production ablation; `--layered-navmesh` explicitly enables
the default. JSON fields prefixed `regional_layered_navmesh_` expose prepared nodes, planar
edges, via zones, certified transitions, rejected spans, preparation/search bound hits, search
expansions, and proposals consumed by regional template construction.

### Integration defects caught by focused tests

The initial transition cost charged only the via penalty. A site incident to several cells
could therefore act as an artificial shortcut, making an unobstructed same-layer route switch
layers and return. Transition costs now include both cell-centre-to-site legs. Goal selection
also includes the final centre-to-endpoint leg.

Constrained planar realization originally bound a boundary endpoint to the first incident
cell before checking dynamic walls. It now selects an open incident cell; an endpoint strictly
inside a closed cell cannot use static-domain fallback to escape that wall. The regression
covers both cases. These corrections preserve the selected corridor without weakening copper
clearance or using a failed graph search as proof of infeasibility.
