# ADR 0142: Feasibility-first follow-up and global run-up guard

## Status

Accepted as an incomplete regional-routing milestone for v0.127.

## Context

After all eligible owner windows received one alternative check, 43 windows still failed. Seventeen
topology rejections occurred at fixed endpoints because merged corridor cells did not necessarily
reach a clipped selected-global fragment endpoint. The final 24 checks were also being spent on
dense multi-conflict windows even though the route comparator values clearing a complete window
before lowering aggregate conflicts.

## Decision

- Add a half-NavMesh-cell capsule around each already selected, exact-static-checked global
  fragment. This is route-specific endpoint/run-up topology, not a new graph path or copper
  authorization.
- Preserve degree-plus-one lane-target control multiplicity and use eight repeated controls only
  at alternate NavMesh funnel corners.
- Rank residual follow-up checks by no retained alternative, single-conflict window, physical
  overlap, and stable ID before returning to dense windows.
- Keep the first packing wave and one-check all-window residual wave unchanged.
- Share immutable offset-region polygons by stable pointer between search-bounds-specific prepared
  NavMesh entries. The geometry cache owns every pointed-to region for the complete alternative
  generation lifetime.
- Publish endpoint and interior topology failures separately.
- Raise the bounded complete endpoint-pair cap to 384 so the feasibility-first follow-up can cover
  the remaining simple owners before revisiting dense windows.

## Consequences

At 320 checks, the run-up guard removes all 17 endpoint topology failures and the feasibility-first
follow-up reaches 216 of 242 complete windows. The remaining 26 all have nonzero discrete residual
conflicts; one has an interior topology failure and there are 65 mutually illegal route
realizations. Selected residual conflicts rise from 190 to 201, which is accepted because three
additional complete windows win the feasibility-first comparison. Runtime is 568.93 seconds and
peak RSS is 704,096 KiB. Sharing immutable region geometry is routing-byte-identical and accounts
for part of that memory reduction. At 384 checks, 297 templates are retained and 173 selected;
residual conflicts fall to 188 and 218 windows realize. The remaining 24 contain one interior
topology and 61 mutual-clearance route failures. Runtime is 592.38 seconds and peak RSS is 829,260
KiB. The marginal two-window gain ends portfolio bound growth; the next state is a bounded
same-layer NavMesh detour around a single physical conflict. No complete spline or copper
transaction is authorized.
