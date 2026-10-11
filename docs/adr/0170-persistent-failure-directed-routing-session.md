# ADR 0170: Persistent Failure-Directed Routing Session

## Status

Accepted.

## Context

Flux carries global resource state, regional continuous state, and quantized copper. Previously,
an intermediate discrete regional probe could terminate planning before PH realization, while a
later detailed failure caused every feedback pass to rerun global routing. Affected-net sets grew
monotonically and failures from discarded topology portfolios entered shared history.

## Decision

Use one bounded persistent coordinator for planning, regional realization, detailed copper, and
exact-feedback repair.

1. A complete discrete-only regional probe is provisional and must be promoted immediately with
   full PH realization before it may terminate planning.
2. Regional diagnoses remain private to the global topology that produced them. Only the selected
   portfolio contributes downstream history to the next epoch.
3. `increase_local_clearance_margin` first re-enters local/regional realization. Terminal, lane,
   via, layer, resource-demand, and regional-corridor changes re-enter the regional exact master.
   `replan_global_topology` re-enters PathFinder.
4. Repetition of the same stable failure escalates one loop. It does not weaken legality.
5. The affected cone is reconstructed from the current failure set on every pass.
6. The newest exploratory routing state becomes the next working checkpoint, but only a strict
   exact-quality improvement or complete exact transaction becomes externally selectable.
7. Incremental materialization accepts verified regional PH and ordered multilayer-via plans and
   reuses only clean unaffected copper before whole-board validation, connectivity, and DRC.

## Consequences

Quantization, curvature, shove, lane, terminal, and via failures do not pay for an unnecessary
global reroute on their first occurrence. The planner receives exact physical evidence earlier,
and retained homotopies are not penalized by another portfolio's failure. Memory remains bounded
by existing portfolio, diagnosis, feedback-pass, regional-state, and detailed-candidate limits.

The coordinator remains heuristic and bounded. It does not make regional geometry authoritative,
persist mutable state into the canonical board, relax copper rules, or replace final external
KiCad DRC.
