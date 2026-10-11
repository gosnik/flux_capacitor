# Default physics/rip-up policy

User-directed policy change: make the kitchen-sink pipeline the default, including
physics push/shove, then rip-up/reroute after failed repair, repeating within bounds.

Evidence: prior N6/R0 frozen-context trials timed out at 900 s in local scope
rebuilding. Existing full-strength mode enabled physical repair only in epoch zero;
later intermediate epochs reverted to cheap discrete probes. The actual local
via gap is documented in `2026-09-06-candidate-via-context.md`.

Change: default full-strength planner/CLI policy; full physics on every epoch;
frozen copper and candidate-via context enabled; intermediate local scope retries
deferred in favour of the existing typed affected-net global replan. The final
epoch can still run configured scope repair. Retain existing congestion history,
incremental cone/full-global selection, checkpoint rollback and exact acceptance.
No new solver, DRC relaxation or unbounded loop. Default retry upper bounds clamp
to short iteration budgets; explicit CLI invalid R>=N remains rejected.
`--no-full-strength-first-pass` restores legacy scheduling; explicit oracle/physics
disable choices also override the default.

Tests: default activation, short-budget clamping, and a forced physical-window
failure across three epochs must show physical consultation on every epoch plus
nonempty affected-net feedback. Preserve explicit legacy scheduling tests.

Benchmark build note: earlier recent runs used build/dev, CMAKE_BUILD_TYPE=Debug,
without compiler optimisation. Subsequent Release runs must be identified as
such; their runtime cannot be attributed solely to the routing-policy change.
Tier acceptance still requires an accepted board and final KiCad DRC.

Follow-up: `diagnose_regional_routability` previously summarized failed windows
without their retained exact outside-copper blocker pairs. An unowned blocker
could therefore be missing from the upstream affected-net set. It now emits the
candidate/blocker net pair with the actual layer and measured deficit, requesting
a corridor change. Complete assignments do not emit these stale failures.
This uses existing spatial coalescing and severity quanta, not raw-nanometre costs.

Final verification: 45 selected checks passed, including two KiCad DRC fixtures.
The policy-only N6 control timed out at 600 s despite completing three feedback
epochs and running XPBD after upstream rerouting. The first epoch reported
32 oracle violations/0 opens; later epochs reported 0 violations/9 opens.
This is a search/connection-recovery problem, not legal completion.
The normal N1 probe produced an accepted internal native scene in 435.384 s
(50 paths, 0 unrouted nets, 3 complete windows, 0 native scene conflicts), but
its separate N30 detailed export still needs verification. No tier is promoted.

Verification runs share CPU/GPU resources. Timings are recorded but are not
controlled algorithm-only performance comparisons. The final detail check is
subject to a 600-second audit cap without changing the corpus's 2400-second
detail limit; exceeding the audit cap means partial verification, not proof of
failure at the full corpus budget.
