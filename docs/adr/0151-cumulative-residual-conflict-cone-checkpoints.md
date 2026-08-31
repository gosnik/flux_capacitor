# ADR 0151: Cumulative residual conflict-cone checkpoints

## Status

Accepted after ADR 0150.

## Context

Articulation-aware decomposition deliberately keeps weakly connected physical conflicts in small
windows. A route fragment shared by several windows can therefore be movable in one assignment and
frozen in another. The first residual-cone implementation constructed bounded merged windows but
discarded every retry unless it covered all failed windows and immediately produced a complete
board. That prevented independently legal shared-fragment repairs from becoming the next planning
checkpoint.

## Decision

Retain the original biconnected conflict evidence, but permit a complete bounded regional retry to
merge non-overlapping failed windows with the windows owning their articulation fragments. A cone
may contain at most the configured eight windows and remains subject to the existing fragment/net,
template, bundle, spline, PNS, and exact-geometry budgets.

Compare the complete retry against the ordinary result using the existing feasibility-first tuple:
candidate availability, unresolved assignments, discrete failures, combined topology/mutual
failures, topology failures, and mutual failures. Retain the retry only on a strict lexicographic
improvement. A partially improved regional checkpoint remains diagnostic planning evidence;
`spline_candidate_available` and the exact whole-board transaction still require every retained
assignment to complete.

The risk invariant is structural rather than a fixed post-repair window count: biconnected blocks
and articulation fragments remain explicit, and a weak conflict chain may not collapse into one
whole-component repair window.

## Consequences

With ADR 0150 active, the isolated ESC builds six cones from 12 source windows and selects the
merged checkpoint. Unresolved windows fall from 15 to 10, retained mutual-route failures from 24
to 20, and selected residual conflicts from 45 to 43. The result has 183 complete assignments out
of 193 repair windows. Runtime rises from 4:58.31 to 9:02.42 and peak RSS is 625,940 KiB. Ten
windows still withhold candidate copper, so materialization and external KiCad DRC remain closed.

