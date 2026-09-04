# ADR 0164: Atomic WorkPlan zone-refill publication

## Status

Accepted.

## Context

ADR 0158 made saved fill copper immutable by default and allowed an explicit routing mode to
produce only a provisional candidate with fill clearance deferred to KiCad. ADR 0161 withheld a
normal WorkPlan output until external KiCad DRC, but WorkPlan execution had no transaction that
could advance a provisional candidate through refill. Merely treating that candidate as ordinary
export would publish copper that had never been checked against the regenerated zones.

## Decision

A route step may explicitly set `refillable_zone_fills`. It must be the final plan step and cannot
request differential-pair skew tuning. The compiler enables the established refillable global and
regional obstacle policy, defers saved-fill clearance in detailed routing, and withholds detailed
passes that do not carry that provisional state. A detailed result is eligible only when it carries
the dedicated `zone_refill_required` stop reason, a candidate, structural validity, zero opens, and
zero non-deferred copper violations. The core execution result exposes that state as
`candidate_ready_for_zone_refill`, never as `candidate_ready_for_export`.

The non-preview CLI publication boundary exports the provisional board into a private sibling
directory with matching source `.kicad_pro` and `.kicad_dru` files. It resolves an explicit
`FLUX_KICAD_PYTHON` interpreter or `python3` on `PATH`, and an explicit
`FLUX_KICAD_REFILL_HELPER` or the configured Flux `refill_zones.py`. Explicit invalid paths are
authoritative and fail closed. The shared adapter launches the helper through a shell-free argument
vector, captures bounded JSON diagnostics, observes cancellation, enforces a fifteen-minute
deadline, and requires a new regular non-symlink output no larger than 512 MiB.

The refilled file is not yet publishable. Flux strict-imports it, requires every refill marker to
be resolved, reruns canonical structural, connectivity, and analytic copper checks, and requires
exact internal acceptance. It then runs the
ADR 0161 external KiCad JSON DRC using a separately staged copy of the source rule context. Only
after all gates pass is the refilled file atomically renamed to the requested output. Refill,
import, internal acceptance, DRC, cancellation, tooling, or rename failure leaves that output
absent and removes the entire staging tree.

`zone_refill_required`, process run/pass/exit status, interpreter identity, and diagnostic text are
serialized independently of external DRC state. The authoritative candidate is replaced with the
strictly re-imported board only after internal acceptance. Preview mode never launches refill or
authorizes publication.

## Consequences

WorkPlan and the existing file workflow now share the same provisional-to-refilled trust boundary:
saved fill can be ignored only temporarily, while regenerated zone copper participates in both
Flux and KiCad acceptance before a final filename exists. The process adapter is independently
tested for successful, failed, and unavailable helpers; the core executor is tested to withhold a
provisional candidate from ordinary export.

This does not make Flux the owner of zone filling, authorize live application of refill-derived
copper, synthesize planes, or qualify the still-partial WorkPlan capability surface. Those remain
separate roadmap gates.
