# ADR 0160: Atomic coupled WorkPlan routing

## Status

Accepted.

## Context

`FluxWorkPlan` represented differential-pair and ordered-bus step kinds, selector expansion already
rejected a partial declared group, and the global/regional/detailed router already treated declared
groups atomically. The execution adapter nevertheless accepted only a generic `route` step. A saved
plan therefore could not express the coupled operation directly, and force-reroute authority was
calculated only from generic route steps.

The interleaved routing planner can also retain a zero-overflow coarse checkpoint while a bounded
regional bundle is unavailable. Rejecting that checkpoint without attempting the detailed router's
conventional polyline transaction made the workflow stricter than the underlying transactional
router and prevented a simple coupled route from reaching exact validation.

## Decision

Compile `differential-pair` and `bus` steps through the same versioned `autoroute_v1` adapter as a
generic route while retaining the step kind in the compiled result. A differential-pair step may
select only nets belonging to declared pairs; a bus step may select only members of declared
ordered buses. Existing selector expansion continues to reject every partially selected pair or
bus. The global router's atomic group scheduling, polarity or declared bus order, coupled regional
planning, and complete detailed transaction remain authoritative.

An exact, non-wildcard coupled selector may grant force-reroute authority to its complete selected
group. Locked copper, unrelated copper, source zones, and partially authorized groups remain
immutable. In allowlisted-ripup mode, a coupled step does not absorb unrelated allowlisted nets;
the detailed replacement scope is intersected with the step's active group before routing.

Execution accepts route, differential-pair, bus, and validation steps. If the interleaved planner
returns complete coarse topology with zero aggregate overflow and no unaccepted selected medial
transition but no usable regional candidate, execution may attempt the conventional coarse
polyline detailed transaction. A retained checkpoint can receive the global `complete` stop tag
only on that private copy and only after the same global completion tuple has been checked. The
fallback is observable in the deterministic execution summary. It commits only if structural
validation, selected-net connectivity, represented copper DRC, and the ordinary final whole-board
acceptance all pass.

## Consequences

Saved plans can now route declared pairs and buses as explicit atomic steps without weakening
copper authority or acceptance. Focused coverage exercises exact authority, rejects an ordinary net
inside a pair step, executes already-connected no-op steps, and routes open two-member pair and bus
fixtures through global, regional/detailed fallback, connectivity, and DRC gates.

This does not claim package-aware fanout, complete pair return-path policy, arbitrary pair/bus
topology completion, or parity qualification of the corresponding manifest capabilities. Those
remain later milestones and public execution remains capability-gated.
