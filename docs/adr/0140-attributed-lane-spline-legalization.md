# ADR 0140: Attributed lane targets and joint spline legalization

## Status

Accepted as an incomplete regional-routing milestone for v0.125.

## Context

The packing-first topology wave gave every isolated ESC window a discrete bundle, but continuous
realization rejected 100 windows. Route-level attribution found no short spline, 35 topology-guard
failures, and 177 route-to-route clearance failures. The spline seed also treated a packed lane's
signed cross-section offset as though it were measured along the source fragment normal. A merged
channel can contain cross-sections from several source corridors, so applying all of them to every
occupant is equally invalid.

## Decision

- Carry a packed offset as a world-space displacement along its canonical physical cross-section.
- Canonicalize cross-section wall order so oppositely directed source paths cannot reverse lane
  identity.
- Preserve multiple lane targets only when the physical cross-section intersects the governed
  source fragment; merged-channel evidence that does not intersect that fragment remains capacity
  evidence and is not a spline constraint.
- Seed the cubic B-spline through the ordered attributed lane targets, using bounded repeated local
  controls to reduce overshoot without changing the route into a polyline.
- Before ordinary per-route trajectory optimization, run a bounded joint elastic legalization
  sweep. Select the worst interior clearance collision, calculate its obstacle normal, apply the
  existing cubic-basis control shove, and accept it only when a whole-route lexicographic score of
  topology violations, mutual-clearance violations, and total deficit improves.
- Keep endpoints, endpoint tangents, frozen boundary routes, corridor displacement bounds, and the
  complete exact transaction gate unchanged. Failed windows publish no partial replacement.
- Publish lane-target, multi-target-fragment, joint-shove attempt/acceptance, topology-rejection,
  and mutual-clearance-rejection metrics.

## Consequences

The isolated collision-normal experiment accepts 668 of 4,528 bounded shoves, but improves only
one window (97 failures to 96). This proves that local spline control is active while also showing
that shove pressure alone cannot repair missing or misattributed corridor geometry. An unrestricted
merged-channel waypoint experiment is explicitly rejected: it worsens the result to 143 failed
windows because unrelated cross-sections pull occupants outside their selected topology. The
fragment-intersection attribution rule prevents that invalid handoff. Exact ESC completion remains
open. The guarded one-pass checkpoint carries 205 attributed lane targets and no multi-target
fragment, accepts 686 of 4,493 joint shoves, and completes 146 of 242 windows. It leaves 34
topology and 165 mutual-clearance route rejections across 96 windows in 334.85 seconds at 291,076
KiB peak RSS. Whole-board connectivity, represented DRC, KiCad export, refill, and external DRC
remain mandatory before any candidate can be accepted.
