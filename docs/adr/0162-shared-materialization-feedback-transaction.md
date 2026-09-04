# ADR 0162: Shared materialization-feedback transaction

## Status

Accepted.

## Context

Detailed KiCad copper exposes geometric conflicts that global resources and regional continuous
geometry cannot always predict. The direct `detail-kicad` workflow already converted attributed
materialization failures into routability history and incrementally replanned the affected nets.
WorkPlan execution stopped before that loop, so an otherwise identical saved configuration could
produce a different proposal sequence and acceptance result.

The loop must also distinguish the global planning board from the materialization source. An
authorized reroute may omit selected unlocked copper during search, but that copper remains in the
source checkpoint until a complete replacement transaction passes exact acceptance.

## Decision

Provide one core `refine_materialization_with_routability_feedback` transaction used by direct
KiCad detail routing and WorkPlan execution. It receives the planning board, immutable
materialization source, selected routing checkpoint, rejected detailed result, shared planner and
detailed configuration, bounded pass count, stop token, and observer telemetry sink.

Each pass accumulates exact detailed diagnoses and clean-copper evidence as cost-only routability
knowledge, reroutes the cumulative attributed net set incrementally, repeats regional realization,
and materializes a private candidate. Only a candidate that improves the existing lexicographic
exact-quality tuple may replace the retained checkpoint. Cancellation, an incomplete candidate, or
an exhausted budget never commits partial copper.

Expose `max_final_routability_feedback_passes` as a strictly positive bounded WorkPlan route option;
its default of three matches the direct CLI. Record attempted/selected state, pass and improvement
counts, and consumed diagnosis count in every WorkPlan step result. The telemetry events remain
non-authoritative.

## Consequences

Saved plans and direct detail routing now share the downstream-failure learning algorithm rather
than merely sharing their initial router configuration. Explicit replacement authority remains
intact because incremental global search uses the stripped planning board while exact
materialization uses the unchanged source checkpoint and configured replacement scope.

This does not make a failed detailed candidate legal, weaken the final DRC gates, or guarantee that
bounded feedback will find a solution. Post-negotiation sampled-conflict repair and diagnostic
incomplete preview remain separate orchestration modes until they receive an equivalent explicit
WorkPlan contract.
