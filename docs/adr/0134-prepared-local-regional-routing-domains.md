# ADR 0134: Prepared local regional-routing domains

## Status

Accepted for v0.120.

## Context

The first ESC run through the regional planner exposed two scale errors rather than an exact-DRC
error. Static-seed NavMesh construction revalidated a complete polygonal free-space domain for
each cell chord, and regional alternative templates built full-board NavMeshes for what should
have been local conflict-window decisions. The selected-via probe wave also shared its bound with
alternative-template search and could exhaust the latter before it evaluated one state.

The global plan represents a lattice path as one edge per sampled step. Treating every such edge
as an independent regional conflict node inflated the ESC checkpoint to 2,678 fragments and 442
windows even when many steps formed one straight physical run.

## Decision

- Validate each NavMesh free-space domain once, retain immutable prepared outline/cutout polygons,
  and batch cell, endpoint-link, funnel, and string-pull chord certificates.
- Use cached polygon bounds and prepared boundary BVHs for clearance queries. Prepared BVH
  traversal uses allocation-free local scratch in the ordinary balanced-tree case.
- Permit a NavMesh configuration to carry an optional search AABB. The AABB bounds only the cell
  lattice; original free-space contours and chord certificates remain authoritative.
- Key regional alternative domains by net, layer, envelope, refill policy, and exact window bounds.
  If conservative polygon offset fails, use the unoffset union only with the requested minimum
  boundary-clearance certificate on every accepted point, cell, and chord.
- Bound selected-global-via probes independently from alternative-template checks. Neither phase
  may consume the other's deterministic budget.
- Coalesce consecutive same-layer, same-width, forward-collinear sampled edges into one regional
  fragment. Preserve bends, reversals, layer transitions, width transitions, terminals, stable
  run ordering, and exact reconstruction of a replacement into the original path.
- Keep the default alternative wave at 128 exact checks. A bounded incomplete wave publishes its
  evidence but cannot authorize copper.

## Consequences

On the isolated ESC one-pass diagnostic, prepared geometry allowed the route to reach regional
analysis in about one minute rather than spending the entire four-minute cap in seed construction.
Window-bounded domains and collinear coalescing reduced the observed conflict representation from
2,678 fragments/442 windows to 1,372 fragments/204 windows. The bounded v0.120 checkpoint completed
in 190.46 seconds with 344,116 KiB peak RSS, evaluated 128 alternative templates, accepted 27, and
mapped every clearance-shadow and capacity conflict to regional evidence.

This is not route completion. Only 14 of 202 eligible ESC windows produced complete bundle
assignments in that checkpoint, so no regional spline candidate was eligible for an exact copper
transaction. Connectivity, canonical copper DRC, and external KiCad DRC remain mandatory, and no
partial bundle or shove is committed.
