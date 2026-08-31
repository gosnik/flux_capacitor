# ADR 0130: Regional routing plan boundary and segment conflict evidence

## Status

Accepted for v0.118.

## Context

The global router can find a connected, sampled-capacity-compatible route set while exact
clearance envelopes still overlap. Net-level residual groups and pitch-derived repair tiles lose
the spatial structure required to separate one large, weakly connected conflict component into
small physical routing windows. Continuing to add scalar PathFinder history cannot express lane
order, coordinated terminal escapes, or movable via choices.

## Decision

Introduce `RoutingPlanResult` as the first-class boundary between coarse global routing and
regional physical packing. It retains the selected `GlobalRouteResult`, exposes the current path
set separately, and cannot report completion while either coarse topology/capacity or regional
physical conflicts remain incomplete.

Split selected planar paths into deterministic route fragments identified by net, connection,
same-layer run, and fragment ordinal. Attribute every exact clearance-shadow observation to two
selected fragments. Coalesce only spatially touching observations for the same ordered fragment
pair and layer, retain peak deficit and observed overlap length, and build deterministic connected
components over fragments rather than nets. Unattributed evidence is counted and fails closed.

This milestone detects and publishes regional work; it does not yet mutate routes. The existing
global router remains the topology planner. Subsequent regional windows, route templates, lane
assignment, via choices, spline optimization, and exact transactions must be implemented behind
this boundary.

## Consequences

- Physical collision evidence becomes regional input instead of another unitless capacity.
- Stable fragment identities permit articulation analysis and frozen entry/exit boundaries.
- The canonical board, global selected paths, and quantized copper remain separate states.
- No conflict is considered repaired until a complete materialized board passes connectivity and
  authoritative copper DRC.
- Board schema remains 18 because all new state is derived routing evidence.
