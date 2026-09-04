# ADR 0159: Exact oval and offset pad-drill geometry

## Status

Accepted.

## Context

The canonical `Pad` stored one drill diameter. Import consequently collapsed an oval drill to its
larger dimension and ignored KiCad's pad-local drill offset. This produced false
`invalid_pad_dimensions` errors for legal USB-C shield holes, represented non-plated oval holes as
overly large circles, and exposed a plated pad's copper anchor rather than its physical hole as a
reusable layer-transition portal.

## Decision

Board schema 19 stores independent pad-drill width and height plus a pad-local offset. A circular
drill has equal dimensions; an oval drill is a capsule aligned with the pad-local axes before the
pad and footprint transforms. Canonical validation requires either two zero dimensions or two
positive dimensions whose complete axis-aligned local bounds fit the pad bounds.

Non-plated drilled pads become exact circle or capsule obstacles on every declared copper layer.
Their world centre includes the rotated pad-local offset. Plated pads retain their outer copper as
the routing obstacle, but global transition discovery and detailed same-net reuse use the physical
drill centre. Telemetry retains both dimensions and offset while keeping the legacy maximum
`drill_nm` field for old trace readers; the viewer renders the actual rotated circle/capsule hole.

## Consequences

Legal oval shield holes no longer fail structural validation, offset holes reserve the correct
free space, and a route cannot place or reuse a layer transition at the wrong pad coordinate. This
does not yet qualify KiCad's complete annular-ring, hole-to-hole, slot fabrication, or drill-offset
rule matrix; those rules remain explicit Milestone 2 work and final KiCad DRC remains authoritative.
