# Oracle-first feedback: bounded regional scheduling experiment

## Failure and candidates

The earlier Tier-0 budget sweep completed only one physical-feedback epoch in
600 seconds at both budgets 4 and 8. Its first provisional copper oracle already
reported 48 violations with zero opens, before expensive regional work. Exact
provisional-copper feedback therefore existed before that work began.

Candidate interventions, ranked:

1. Defer intermediate regional work when exact provisional-copper failures already
   identify an upstream feedback opportunity. Small, opt-in scheduling experiment.
2. Cache complete regional search products. Larger invalidation risk: these depend
   on selected routes, neighboring copper, history and scope, not just geometry.
3. Increase the overall timeout or search budget. Does not address delayed feedback.

This is not evidence that a colliding straight seed has no legal curved realization.
The final epoch must still attempt regional realization even if the oracle fails.

## Research

[VPR's primary router documentation](https://docs.verilogtorouting.org/en/latest/vpr/command_line_usage/)
separates iteration limits from present and accumulated congestion controls.
Its FPGA capacity model is not a continuous PCB copper model; only the experimental
principle of separating search budget from conflict policy applies here. No port
or new optimality claim is involved.

## Controls

- `--max-router-iterations N`: total interleaved PathFinder budget.
- `--physical-feedback-retries R`: explicit outer retry count, including zero.
  Requires an explicit total budget and `0 <= R < N`, with `R <= 64`.
  Without this option, legacy automatic retry-count selection remains unchanged.
- `--feedback-shadow-policy enforced|advisory`: fixes physical-conflict policy
  independently of retry count. Unspecified preserves the legacy policy.
- `--oracle-first-feedback`: opt-in bypass of intermediate regional search when
  the enabled exact provisional oracle is infeasible. Its diagnoses still enter
  history. A feasible oracle still allows early regional certification, and the
  final epoch always retains the full regional acceptance gates.

The same options are available on the corpus runner (using `--iteration-budget`
for total iterations) and are recorded in its run metadata. The algorithm remains
disabled by default until quality and regression evidence justify promotion.

## Micro-tests

- A two-terminal clean board still receives exact early regional promotion and
  legal full/incremental detailed materialization, with enforced shadows despite
  having feedback enabled.
- An impossible crossing with exact oracle violations bypasses intermediate
  regional work, starts feedback, and still consults regional realization in the
  final epoch. It is not accepted merely because expensive work was deferred.

## Benchmark hypothesis

At fixed four global iterations, three physical retries and advisory shadows,
oracle-first scheduling should reach more feedback epochs within 600 seconds.
Success requires improved physical completion, not merely more epochs. A timeout
or missing detailed output is not a DRC pass. Keep all clearance rules unchanged.

## Result

Both fixed-policy runs timed out at 600 seconds without final plan JSON or final
KiCad DRC. Enabled: epochs completed at 105.281, 213.688 and 318.918 seconds;
oracle violations 48, 38, 50 respectively, each with zero opens. Final epoch began
at 319.393 seconds and did not finish. Disabled: only one completed epoch, with
48 violations, followed by the start of another. The disabled run overlapped an
independently started Tier-0 process, so no isolated wall-time speedup is claimed.
Both run manifests record executable SHA-256
`fd2094c09e6b90e6f53a76d4137d28949775a7281da41ceb810d2b731cf5eb04`.

The scheduling mechanism is exercised and more feedback is observed, but final
copper benefit is unproven. Retain disabled by default. Full core CTest passed
in 134.29 seconds; all nine native-arc/multilayer export/DRC checks passed in
26.66 seconds. Structured results:
`reports/benchmark_history/2026-09-05-oracle-first-feedback.json`.

The user next requested the opposite scheduling experiment: activate compatible
routing/repair stages at full strength in the first epoch rather than defer them.
