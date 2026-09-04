# ADR 0166: Atomic WorkPlan integrated placement

## Status

Accepted.

## Context

`FluxWorkPlan` already represented placement steps, but execution rejected them because no saved
configuration boundary connected the plan to Flux's existing integrated placement/routing
optimizer. Treating placement as an isolated transform pass would violate Flux's central
invariant: a placement is useful only if its affected nets can be physically routed and the
complete candidate remains legal.

A saved placement step also needs stricter authority than a direct optimizer invocation. Its
footprint selector must bound every movable object, its routing projection must include every net
touching those objects, and existing copper may be removed only under the plan's ordinary
force-reroute or allowlisted authority.

## Decision

A `placement` step compiles through the same `autoroute_v1` route configuration as route steps and
adds a bounded integrated-optimizer configuration. The typed options cover placement passes,
candidate and worker budgets, shortlist size, move step, feedback margin, global/routing-guided
placement, swaps, rigid-group and relative-closure moves, incremental verification, route-quality
refinement, orientation/side repair, assignment legalization, tentative envelopes, and suggested
move-priority policy. Placement-only options are rejected on other step kinds.
Route-transaction wrappers that the integrated optimizer does not execute—outer materialization
feedback, sampled-cone repair, retained planner portfolios, refill, and incomplete-preview
retention—are rejected on placement steps rather than accepted without effect.

Placement selectors must use `group_scope: touching`. The compiler therefore carries both the
stable selected footprint IDs and every touching net into one transaction. Differential-pair and
ordered-bus membership remains atomic. Preview and new-routing modes receive an explicit empty
existing-copper replacement scope; force-reroute and allowlisted modes receive only the exact
authority already derived by the canonical selector expansion.

The executor locks every unselected footprint on a private board copy before invoking
`optimize_integrated_placement_routing`. It restores the original lock values on the returned
candidate and independently rejects any changed unselected transform. Selected touching nets are
the only global routing scope. The step commits to the next private checkpoint only when the
integrated detailed route is complete and canonical structural validation, selected-net
connectivity, and whole-board represented copper DRC all pass. Failure or cancellation retains the
previous checkpoint and skips dependants.

A reviewed `placement-move-priorities` artifact is retained from the existing one-read,
SHA-256-checked policy boundary and passed directly to the optimizer. Prefix-derived suggested
priorities and tentative placement envelopes are allowed only in preview mode and retain their
mandatory human-review evidence.

Step summaries expose selected footprints and nets, candidate/full-route evaluations, accepted
moves, unlocked priority tiers, human-review state, and the optimizer stop reason. Existing
placement proposal telemetry remains non-authoritative; the WorkPlan step completion event adds
only bounded aggregate evidence.

## Consequences

Saved placement now uses Flux's actual router-in-the-loop optimizer and exact transaction gates
instead of a parallel transform path. Selection cannot silently move a neighboring component or
rip unrelated copper, and a proxy-improved but physically unroutable placement cannot advance the
plan.

This milestone does not claim the later design-brief, package-aware repair, placement-portfolio UX,
or live KiCad placement-application work. The parity manifest therefore remains partial and the
CLI capability preflight continues to withhold plans that claim the broader unqualified placement
capability.
