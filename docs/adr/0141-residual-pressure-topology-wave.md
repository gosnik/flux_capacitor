# ADR 0141: Residual-pressure topology wave

## Status

Accepted as an incomplete regional-routing milestone for v0.126.

## Context

The packing-first topology wave spent only 82 of its 128 complete endpoint-pair checks making four
otherwise infeasible windows fit. Continuous realization then received 373 same-layer conflict
edges. Elastic spline shoving improved local geometry but could not compensate for those discrete
layer choices.

## Decision

- Preserve packing-failure windows as the first and mandatory topology wave.
- Spend remaining alternative checks one per eligible non-packing owner window, ordered by
  observed physical overlap length, peak deficit, and stable window ID.
- Within a window, try the highest overlap/deficit owned fragment first.
- Prevalidate endpoint run-ups, full via spans, NavMesh topology, and corridor cells exactly as in
  the packing wave; a residual candidate receives no weaker legality.
- Retain generated choices for the existing joint bundle objective. Same-layer residual conflict
  count remains the first objective, followed by relocated vias, via count, topology cost, and
  stable template IDs.
- Keep alternative ownership, via CSP, lane packing, spline realization, exact transaction, and
  whole-board acceptance unchanged.
- Publish packing-priority and residual-priority window counts separately.

## Consequences

With the original 128-check cap, the isolated ESC run spends all 46 remaining checks across the
highest-pressure residual windows. It accepts 111 templates, reduces selected same-layer conflicts
from 373 to 337, and improves continuous realization from 146 to 161 complete windows. The 81
failed windows contain 32 topology and 141 mutual-clearance route failures. Runtime rises from
334.85 to 410.38 seconds and peak RSS is 291,744 KiB. A conflict-count-first ordering is rejected:
although it reaches 333 discrete conflicts, it regresses the authoritative continuous result to
84 failed windows. With the bounded cap raised to 320, 238 residual owner windows receive a first
chance and 24 otherwise unused checks go to additional fragments in multi-conflict windows. The
solver accepts 253 templates, selects 166, and reduces same-layer residual conflicts to 190. It
completes 199 of 242 windows; the remaining 43 contain 23 topology and 83 mutual-clearance route
failures. Runtime is 561.43 seconds and peak RSS is 769,368 KiB. This is a substantial feasibility
gain but an explicit memory/performance regression, not an exact routed-board claim.

A half-cell route-specific capsule now preserves each already selected, exact-static-checked global
fragment where merged channel cells do not reach its clipped endpoints. Endpoint topology failures
fall from 17 to zero, total topology failures to three, and complete windows rise from 199 to 213.
The remaining 29 windows contain 64 mutual-clearance route failures. Explicit cell-pitch
approach/departure controls are rejected because they regress to 44 failed windows. Sharing one
immutable offset-region polygon between search-specific NavMeshes is routing-byte-identical and
lowers peak RSS from 772,968 to 735,236 KiB.
