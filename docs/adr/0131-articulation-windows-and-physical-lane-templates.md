# ADR 0131: Articulation windows and physical lane templates

## Status

Accepted for v0.118.

## Context

A net-connected residual group can be large because weak chains join several dense local
conflicts. Net count alone does not identify the state that must be solved atomically. At the same
time, scalar lane counts cannot prove that heterogeneous traces fit in a corridor in a realizable
order.

## Decision

Run deterministic Tarjan biconnected decomposition over the route-fragment conflict graph. Build
one initial regional window per block, expose articulation fragments, expand the physical conflict
bounds by the configured margin and largest governed copper envelope, and bind quantized fixed
portals where selected routes enter or leave. Oversize, uncuttable, and split coupled atoms remain
explicit failures.

Promote selected topology-preserved geometric corridors into regional channels. Pack a proposed
order from actual track widths, outer clearances, and adjacent maximum pairwise clearances. Center
successful lanes and retain signed offsets as evidence. Derive the preferred physical order from
where selected source centre-lines intersect each canonicalized channel wall; use midpoint
projection for a parallel/non-intersecting fragment and stable ID only for coincident ties. Try
those orders before deterministic permutation search and minimize physical-order inversions under
the hard state budget. Preserve coupled-member relative order as a separate hard constraint.
Enroll a fragment only when its represented segment overlaps the retained corridor cells;
same-layer connection identity alone is not channel topology evidence.

Represent the selected terminal access and layer sequence as bounded route templates. Existing
selected via sites are the first choices; nearby sites are admitted only after adding a complete
via object to a board probe and passing structural and represented copper checks. Missing topology
never creates an inferred channel or layer jump.

A complete lane assignment initializes cubic B-splines with fixed endpoints and lane-displaced
interior controls. Joint bounded sweeps retain selected-corridor topology and impose route-to-route
clearance barriers. Quantized legal centerlines form a separate candidate path set. That set may
enter detailed routing with planning-only conflicts cleared only on a private copy; full-board
connectivity and copper DRC accept or discard the transaction atomically.

## Consequences

- Weak conflict chains become independently observable physical routing windows.
- A corridor's capacity is expressed in nanometres and occupant order, not only lane count.
- Stable route identity cannot accidentally dictate the physical left/right lane topology.
- Terminal and via decisions become explicit regional state before B-spline optimization.
- Candidate paths never overwrite coarse selected paths and cannot bypass exact transaction gates.
