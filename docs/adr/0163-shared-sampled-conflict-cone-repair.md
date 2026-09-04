# ADR 0163: Shared sampled-conflict cone repair

## Status

Accepted.

## Context

The direct KiCad CLI exposed `--repair-sampled-conflicts`, but its implementation was a private CLI
function with a fixed two-cone budget. Saved WorkPlans could not request or reproduce the same
proposal sequence, and the algorithm had no reusable stop-token boundary.

This mode is distinct from the final materialization-feedback loop. It acts on a connected global
checkpoint whose sampled, shared-medial, or clearance-shadow resources still overflow, before
regional realization and exact copper construction.

## Decision

Move the algorithm into the shared routing-feedback core. Group sampled edges, shared-medial
resources, and clearance-shadow conflicts into deterministic four-grid-step spatial tiles. Retain
tiles with at least two nets, order them by descending pressure and stable net/tile identity, and
incrementally reroute the complete attributed net set for each bounded cone. Repair enables atomic
net trees, diversified terminal access, and immediate hard-capacity pressure while disabling
selective partial rerouting inside the private transaction. The feasibility-first global checkpoint
comparator alone may accept an updated route.

Provide a shared `plan_routes_with_sampled_conflict_repair` wrapper so direct `route-kicad`, direct
`detail-kicad`, and WorkPlan execution perform identical global repair and regional analysis.
Expose the WorkPlan options `post_negotiation_sampled_conflict_repair` and
`max_post_negotiation_sampled_conflict_repair_cones`; a cone budget without the enabled mode fails
closed. The direct CLI retains its existing two-cone default.

When this explicit mode is selected, do not also run the ordinary final-materialization feedback or
automatic sampled fallback in the same WorkPlan step. This preserves one reproducible retry
sequence. Report whether the mode was enabled and its attempted cones, accepted checkpoints, and
attributed nets in the deterministic step result.

## Consequences

Sampled-conflict repair is no longer a CLI-only semantic, its resource budget is explicit in saved
plans, cancellation reaches both the initial route and every repair cone, and failed cones leave the
retained checkpoint unchanged. The algorithm still does not legalize overflow or bypass regional,
connectivity, copper DRC, source-preserving export, or external KiCad DRC gates.

This bounded spatial repair is not the complete frontier/min-cut or large-neighborhood recovery
milestone. It remains one deterministic post-negotiation operator within that future hierarchy.
