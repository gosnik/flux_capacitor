# ADR 0147: Adaptive whole-window pair-shove transactions

## Status

Accepted after ADR 0146.

## Context

The regional elastic-band pass first moves one cubic B-spline control neighborhood at a time and
accepts a lower per-route sampled legality score. This is useful local control, but a prior
experimental pair shove showed why a sampled aggregate is insufficient: hundreds of locally
accepted moves reduced that score while adaptive final validation completed fewer ESC windows.

The retained ADR 0146 shortlist evaluates complete discrete states, yet many selected states still
contain a small number of mutual spline conflicts. Moving both participants is the natural PNS
operation, provided the complete adaptive window—not either route in isolation—decides acceptance.

## Decision

After ordinary per-route trajectory optimization, run an additional bounded pair transaction only
when prepared route samples show mutual pressure. The default budget is 16 candidate transactions
per regional assignment.

For the deepest adaptive interior collision:

1. derive and normalize its obstacle normal;
2. rank other same-layer, different-net splines by adaptive distance to the collision point;
3. move both local cubic control neighborhoods in opposite directions, splitting the required
   displacement when both are movable and assigning it to the owned route when the boundary route
   is frozen;
4. use four deterministic displacement halvings inside the shared transaction budget; and
5. accept only the best proposal whose complete adaptively tessellated window improves
   lexicographically.

The complete-window objective compares topology-illegal routes, mutual-illegal routes, topology
sample violations, mutual sample violations, and aggregate clearance deficit, in that order. The
original complete set of spline controls is the fallback. A rejected proposal leaves neither
participant changed.

Final regional mutual validation prepares every route with the same 0.02 mm adaptive tolerance,
depth 10, and 512-sample bound before comparing route envelopes. It no longer checks one dense
candidate curve against a coarser neighboring curve.

No pair transaction changes route topology, selected lane order, via choice, endpoint, width,
layer, or frozen exterior state. Exact candidate construction, connectivity, canonical copper
DRC, and external KiCad DRC remain mandatory and atomic.

## Consequences

On the isolated one-pass ESC checkpoint, 208 bounded pair proposals produce 17 accepted
transactions across 11 windows. Complete spline windows improve from 220 to 222 and retained
mutual-clearance route failures fall from 51 to 50; windows 17 and 78 leave the unresolved set.
Runtime changes from 288.87 to 290.26 seconds and peak RSS from 803,860 to 813,212 KiB. The board
still has 20 failed spline windows and 1,317 coarse overflow, so this does not satisfy Gate B/C or
authorize export.

This remains block-coordinate elastic PNS rather than a general simultaneous nonlinear optimizer.
Fixed endpoints, insufficient corridor width, wrong terminal/via topology, or several-way packing
may still require another discrete regional state or placement repair.
