# Rip-up budget experiment

## Hypothesis

Tier-0 `pt2399_delay` may fail because the corpus's default one-iteration planning
budget prevents exact physical conflicts from driving another allocation round.
Compare budgets 1, 4 and 8, sequentially, with the same executable, source board,
50,000 expansion limit, two static-seed workers, diversification and diagnostic
telemetry. Keep the existing 600-second planning timeout and all DRC rules.

## Proven configuration coupling

`apps/flux-cli/main.cpp` caps planner feedback retries at `max_iterations - 1`.
The planner normally permits four retries. Thus these trials allow respectively
zero, three and four retries, not zero, three and seven.

`src/regional_router.cpp` additionally derives `quantized_oracle_owns_shadows`
from whether any feedback retries are allowed. One iteration enforces shadow
negotiation; larger budgets make shadows advisory and use exact copper feedback.
Intermediate epochs disable spline realization and may later promote a candidate
to a full regional solve; the sole epoch in a one-iteration run is already final.

Consequently this sweep measures the existing CLI budget's combined behavior,
not the isolated causal effect of adding otherwise identical rip-up rounds.
The first observed conflict set is not identical across these modes.

## Baseline identity

- Git HEAD: `25e3b884be9b359d9c5bf0cc4fbb6cc836fd4f82`; dirty working tree.
- Executable SHA-256: `0974ae0806c623ec999d91265bc4e4c9d5c5d74a5e243f925d35cbdcbb68eaef`.
- No router changes, source-board changes or manifest edits during the sweep.
- `verify_routing_corpus.py --iteration-budget` overrides planning and detailed
  budgets in memory and records the override in each immutable run manifest.
- Artifact roots: `build/ripup-budget-{1,4,8}`.

## Results

One iteration: 169.455 seconds; incomplete plan; 50 paths; zero unrouted nets;
16 reported global overflows; 76 native-scene conflicts; three scaffold
rejections; zero global/regional feedback passes. The provisional copper oracle
reported 32 DRC violations and zero opens. These are not final KiCad DRC results.

Four iterations: planning timed out after 600 seconds, without final result JSON.
Telemetry records one completed epoch (48 oracle violations, zero opens) and
the start of a second epoch with 21 physical-history entries and pressure 128.
Missing final conflict counts are unknown, not zero. No final board or KiCad DRC.

Eight iterations: also timed out after 600 seconds without final result JSON.
One completed epoch had the same 48 oracle violations, zero opens and 21 history
entries. The second epoch started with a two-iteration PathFinder budget, versus
one in the four-iteration trial. Neither larger budget published a completed
second epoch before termination. No final board or KiCad DRC was produced.

All three run manifests record the same executable hash. Machine-readable
results, original run metadata and epoch evidence are retained in
`reports/benchmark_history/2026-09-05-ripup-budget-sweep.json`.

Verification: Python compilation, positive-budget preflight, zero-budget rejection,
summary assertions for incomplete/missing results, and `git diff --check` passed.
No C++ algorithm changed in this experiment. No tier was promoted.

## Interpretation

The missing feedback opportunity is real, and increasing the budget activates it.
Whether additional rounds improve final copper remains unproven. The first
intermediate regional round is expensive enough to consume most of the practical
time budget. Merely raising a constant is not yet a demonstrated fix.

Next controlled experiment: separate physical-feedback count, global iteration
budget and physical-conflict policy; hold the policy fixed. Profile and bound
intermediate regional work before escalating search breadth. Do not promote a
tier or change production defaults on the evidence above.
