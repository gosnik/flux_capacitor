# ADR 0165: Non-authoritative WorkPlan incomplete preview

## Status

Accepted.

## Context

Flux retains useful global paths when a route step cannot connect every selected net. The direct
`detail-kicad --write-incomplete-preview` workflow can materialize those paths for inspection, but
the saved WorkPlan executor discarded them with its failed private checkpoint. That made a failed
plan correctly non-publishable but deprived the always-on telemetry/viewer workflow of the closest
physical realization.

An incomplete visualization must not become another success state. In particular, it must not
commit into a dependent step, enter zone refill, run publication DRC, or be confused with exact
copper merely because some selected nets have tracks.

## Decision

A route-like WorkPlan step may explicitly set `retain_incomplete_diagnostic_preview`. The compiler
accepts it only in `preview` mode and rejects any combination with `refillable_zone_fills`.

After ordinary global negotiation, feedback, and bounded fallback have selected their final
authoritative checkpoint, the executor may materialize its non-empty retained path set on a private
board copy. For this diagnostic transaction only, a copy of the global result has its completion
tag, unrouted list, and aggregate overflow cleared so the conventional detailed materializer can
realize the paths that actually exist. The authoritative global result is unchanged. Topology-
locked materialization and differential-pair skew tuning are disabled for this attempt because
their complete-bundle preconditions do not hold.

If materialization yields a board, the executor stores it separately from the current accepted
checkpoint. The route step remains failed, dependent steps remain skipped, `internally_complete`
and `whole_board_accepted` remain false, and both export and refill readiness remain false. At the
end of a failed preview plan, that separate board may be returned as `candidate` only with
`candidate_is_incomplete_diagnostic_preview=true`. The step reports whether the attempt produced a
candidate and how many retained paths it contains. A diagnostic telemetry event identifies the
step and preserves the ordinary global route events for the viewer.

No output filename is accepted for preview WorkPlans, so the transaction cannot create a KiCad
artifact through `execute-work-plan-kicad`. Cancellation continues to take precedence and cannot
publish a preview.

## Consequences

Failed routing is now physically inspectable without weakening any acceptance or publication
gate. A caller can distinguish the last accepted checkpoint from review-only partial copper using
one explicit result flag, while the failure metrics and missing nets remain authoritative.

This is not loss-tolerant KiCad import, a route completion fallback, or permission to fabricate the
candidate. It does not infer the option for ordinary preview plans and does not make incomplete
copper available to mutating WorkPlan modes.
