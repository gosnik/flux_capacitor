# ADR 0145: Bounded conflict-cone joint via and lane portfolio

## Status

Accepted for the first regional joint-repair wave after ADR 0144.

## Context

After coherent via stacks, access-aware partial pruning, and dogleg run-ups, the isolated ESC has
one discrete regional failure. Window 109 has complete ordinary route templates, but every complete
tuple either contains an incompatible via assignment or fails physical cross-section packing.
Increasing the ordinary endpoint stencil globally can find another topology, but it doubles runtime
and produces substantially worse continuous geometry in unrelated windows.

PathFinder should remain responsible for coarse reachability. This residual failure needs the first
bounded form of the planned regional joint solver: topology alternatives, via sites, and lane order
must be evaluated as one local state without reopening the complete board.

## Decision

Flux reserves a separate joint-repair budget after the ordinary owner-scoped topology portfolio is
complete. The default bounds are:

- eight regional windows sharing one global topology-check budget;
- eight blocker-attributed fragments in that window;
- 128 additional exact-prevalidated topology checks;
- eight total templates per repaired fragment, including its ordinary templates; and
- four local via-site pitch rings.

Before spending that budget, Flux reconstructs each eligible window's ordinary template tuple
frontier and proves whether any complete tuple has all of:

- coupled-group legality;
- a complete terminal/via access assignment;
- a channel for every selected run; and
- at least one physically fitting lane order in every used channel.

An ordinary portfolio with no complete state and an attributed via-pair prune is always the first
repair candidate. The remaining bounded slots admit windows that already fit discretely but carry
physical conflict pressure. Candidates are ranked by proven discrete failure, via-pair prune count,
physical overlap, and stable window ID. This is derived evidence over the bounded ordinary
portfolio, not a heuristic based on board-specific window numbers.

Within a selected conflict cone, blocker observations are mapped back from stable via-choice IDs to
the non-global route templates that own them. A fitting pressure window without such attributed
owners ranks its owned fragments by observed overlap plus clearance deficit. Only those ranked
owned fragments receive expanded endpoint/layer alternatives. Each generated alternative still requires
legal run-ups, a legal via span, a prepared NavMesh route, topology guard cells, and represented
static clearance before it enters the portfolio.

The 128 checks are not multiplied by the eight-window bound. A proven discrete failure reserves at
least one third of the global budget; the remaining checks are divided fairly among the remaining
candidates. Exhaustion and accepted-template counts remain global and observable.

The ordinary deterministic bundle solver then evaluates the expanded templates atomically. Its
state contains the complete per-fragment template tuple, fixed topology-specific via sites,
terminal choices, and channel occupants. Access-CSP pruning and physical lane-order packing are
therefore part of the same bounded decision. For the expanded window, the objective first counts
physical envelope collisions between topology-specific alternative centrelines, their original-
layer via run-ups, and the other selected runs. It then compares residual source conflicts,
relocated vias, via count, topology cost, and stable template IDs. Ordinary windows keep their
established comparator. No partial repair is committed.

The additional wave has independent counters and an independent exhaustion flag. It does not
increase `max_alternative_template_checks`, alter the global route, relax the all-window spline
gate, or authorize copper. B-spline realization, exact connectivity, canonical DRC, and external
KiCad DRC remain subsequent mandatory gates.

## Consequences

Dense regional failures can receive additional topology diversity without changing sparse or
already feasible windows. The total template and state limits remain explicit, and telemetry can
distinguish an attempted, expanded, completed, or exhausted joint repair.

On the isolated ESC, eight via-pressure-ranked windows use 126/128 checks and accept 96 new
templates. The continuous shortlist recovers window 75 at rank one, raising complete spline
windows from 222 to 223 and reducing retained topology/mutual failures from 6/50 to 5/48. Runtime
is 294.49 seconds at 811,104 KiB. An overlap-first ordering is rejected: it falls back to 222
complete windows despite reaching 6/46 failures and takes 303.87 seconds.

This milestone jointly allocates a bounded discrete portfolio; it is not yet the complete regional
route-bundle search described in the long-term plan. Via coordinates remain members of validated
finite alternatives, and lane order is solved per channel. Later work may add continuous lateral
via regions and synchronized multi-channel ordering. ADR 0146 now carries a bounded shortlist of
these complete discrete states through the existing joint B-spline/PNS feasibility boundary; it
does not turn this finite portfolio into an unbounded continuous/discrete optimizer.
