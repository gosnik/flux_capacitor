# ADR 0153: Exact regional handoff may discharge coarse physical overflow

## Status

Accepted.

## Decision

Once every requested connection has connected coarse topology, a complete atomic regional
B-spline candidate may pass the planner boundary despite bounded sampled or clearance-shadow
overflow in the selected PathFinder checkpoint. The candidate replaces the governed fragments as
one transaction and proceeds to ordinary detailed materialization.

This does not redefine coarse overflow as legal copper. Missing regional assignments, unmapped
conflicts, incomplete topology, failed spline realization, structural failure, disconnected
copper, or exact DRC failure still rejects the complete transaction. The original global paths
remain fallback evidence and the immutable source board is never modified.

## Consequences

PathFinder remains the global reachability and negotiation engine, while the regional solver can
resolve bounded sub-grid packing that the sampled model cannot express. The four-component risk
fixture reaches exact materialization with a complete regional candidate and is correctly rejected
for three represented copper violations instead of being accepted or stopped solely by sampled
overflow.
