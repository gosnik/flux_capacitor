# ADR 0146: Joint-window continuous-feasibility shortlist

## Status

Accepted after the first ADR 0145 conflict-cone portfolio.

## Context

ADR 0145 makes the formerly blocked ESC window 109 discretely feasible. Its ordinary bundle
solver can find many complete template/via/lane tuples, but selecting only the best scalar
discrete objective still leaves the window's B-spline transaction illegal. A topology centreline
collision estimate improves the selected state, but it cannot predict the complete elastic-band
result: the actual routed objects are corridor-constrained cubic B-splines with shared clearance
barriers.

Committing the single best discrete tuple before continuous realization therefore discards useful
joint states too early. Conversely, running continuous optimization for the complete tuple
frontier would be unbounded and would duplicate work in ordinary windows that are not the active
conflict cone.

## Decision

For the ADR 0145 expanded joint window and at most seven additional pressure-ranked windows, retain
a bounded pool of the 32 best complete bundle selections already produced by their ordinary
exact-prevalidated portfolios. Ranking keeps the expanded window first, then compares selected
residual conflicts, observed overlap, and stable window ID. A window must have more than one
complete state and nonzero selected residual pressure. Keep the scalar-best state first, then
select at most three more states by maximizing
their minimum template-ID Hamming distance from the retained set, using the existing deterministic
discrete objective as the tie-break. This avoids spending every continuous trial on states that
differ only by a nearby via coordinate. Each selection contains the complete atomic
state needed by realization:

- one route template per fragment;
- the jointly selected terminal/via access assignment;
- via-region ordering and alternative switches;
- every packed channel cross section and lane order; and
- its physical-template and residual-conflict counts.

Realize each retained state independently with the existing topology-guarded cubic B-spline,
joint elastic-band shove, and per-route trajectory optimizer. Select the first complete
spline-legal window in shortlist order. If none is legal, retain only the least-illegal diagnostic
state, comparing missing/short realizations, total illegal owned routes, topology failures,
mutual-clearance failures, physical template conflicts, residual conflicts, and stable shortlist
rank in that order. Discard all other
trials as a single transaction; no fragment, via, lane, control point, or route from a failed state
may leak into the retained assignment. Retaining a better failed diagnostic state does not make
the window complete and cannot authorize copper.

The eight-window bound includes the ADR 0145 window rather than adding ordinary repairs on top of
it. Windows outside the selected set discard their stored pool and retain the original single-state
cost. The shortlist is a continuous feasibility gate, not a quality vote. Exact whole-board candidate
construction, connectivity, canonical DRC, and external KiCad DRC remain authoritative. Ordinary
windows retain one selected bundle and incur no shortlist work.

Telemetry and CLI diagnostics expose the bounded candidate count, legal count, selected rank, and
windows recovered by a nonzero rank. Established topology/mutual rejection counters are recomputed
from the retained state so discarded trial failures cannot make the board appear worse. Search-
effort counters may include every bounded trial.

## Consequences

The joint solver can now use the actual B-spline/PNS feasibility boundary when a few discrete
states are geometrically close, while retaining deterministic bounded execution. The maximum
continuous portfolio work is four realizations for each of eight selected windows by default;
every other window still receives one realization.

This is not a general route-bundle A* search, continuous via-region optimization, or synchronized
ordering across several channels. It does not relax any all-window or exact-copper acceptance
gate. A window for which all four states fail remains unresolved and supplies attributed evidence
for the next regional topology or placement-repair wave.
