# ADR 0137: Width-aware regional spline transactions

## Status

Accepted as an incomplete regional-routing milestone for v0.123.

## Context

Regional conflict windows were previously rejected when any participating global-path edge used
an explicit terminal, direct-medial, or sampled-corridor neck-down. The selected width is aligned
with global polyline edges, whereas regional B-spline tessellation changes the edge count. The
isolated ESC checkpoint happens to use no explicit-width fragment; its 55 ineligible windows had a
separate sampled-bound/fragment-anchor cause addressed by ADR 0138.

The three width policies do not have identical authority. A terminal policy authorizes a width for
a bounded physical access length. A direct-medial or sampled-corridor policy additionally depends
on the exact graph resource that certified the narrow traversal. Preserving only the numeric width
would therefore be insufficient for the latter cases.

## Decision

- Every regional fragment retains its resolved physical width, explicit-width flag, and exact
  `GlobalRouteSegmentWidthKind`. Fragment coalescing stops at every width or kind change.
- Explicit-width fragments receive only the selected global topology. Regional search cannot move
  them to another layer, and a via region touching one cannot select a relocated site.
- A terminal-neck-down fragment may be lane shifted and continuously optimized inside its selected
  topology. Its endpoints remain fixed. B-spline tessellation emits the selected positive width
  and `terminal_neck_down` kind on every resulting edge.
- A direct-medial or sampled-corridor neck-down is a fixed spline obstacle. It participates in
  physical packing and mutual-clearance evaluation but emits no replacement. A candidate must
  preserve the complete ordered set of its kind, layer, endpoints, and selected width.
- Before a regional candidate is published, Flux validates width-vector alignment, nominal-zero
  encoding, positive explicit widths, same-layer explicit edges, configured policy widths, and
  maximum terminal/medial/corridor physical lengths. It recomputes all per-path segment counts and
  rounded lengths from the candidate geometry.
- Any failure withholds the complete regional spline candidate. Detailed connectivity, represented
  copper DRC, and the immutable-source transaction remain authoritative.

## Consequences

An explicit width no longer makes its entire physical conflict window ineligible. Terminal access
geometry can participate in the actual continuous solver, and neighboring routes can deform around
fixed resource-certified narrow runs. The implementation deliberately does not move a
medial/corridor neck-down off its source resource, infer a new width certificate, add taper
geometry, or authorize coupled pair/bus width changes.

A focused two-layer crossing fixture now retains an explicit terminal width through regional
B-spline tessellation, recomputes its evidence, and completes whole-board connectivity and
represented DRC. ESC completion and external KiCad DRC remain separate gates.
