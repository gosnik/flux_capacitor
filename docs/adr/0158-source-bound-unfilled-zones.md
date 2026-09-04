# ADR 0158: Source-bound unfilled KiCad zones

## Status

Accepted.

## Context

KiCad 10 permits a zone definition to exist without any saved `filled_polygon` geometry. Flux
previously discarded such a zone and marked the import lossy. Treating the zone as absent would be
unsafe: a later KiCad refill can create copper that was invisible to routing and DRC. Inventing a
fill from the outline would also cross the established authority boundary because KiCad owns zone
fill policy.

## Decision

Board schema 19 preserves an unfilled source zone as an immutable `CopperZone` with no physical
fill and `requires_refill == true`. The marker retains stable identity, net identity when present,
name, and lock state. It does not pretend that the source outline is filled copper. A missing net is
represented by the canonical no-net identity, matching KiCad's legal netless zone syntax.

The two zone states are exclusive:

- a saved physical snapshot has one or more fills and does not require refill;
- an unfilled source definition has no fills and requires refill.

Canonical validation accepts the second state with an observable warning and rejects an empty
unmarked zone or a zone that simultaneously carries fills and the refill marker.

Normal detailed routing fails closed when any source zone requires refill. The existing explicit
refill workflow may route without saved fills, but its result remains provisional even after local
connectivity and represented-copper DRC pass. Acceptance still requires source-preserving export,
KiCad zone refill, strict re-import of the resulting saved fill, connectivity analysis, and final
KiCad DRC. Global planning and telemetry may observe the marker, but it never becomes copper or an
authoritative clearance object.

## Consequences

Original KiCad 10 boards with unfilled zones can cross the strict source boundary without format
normalization, while Flux cannot accidentally certify copper against an unknown future fill. This
does not implement zone synthesis, refill, thermal policy, island removal, or plane repair; those
remain later work-plan transactions owned by KiCad at the physical-fill boundary.
