# Tier-0 local/native scene mismatch

Baseline: `build/full-strength-first-epoch-diagnosis`, pt2399_delay, N=6/R=0,
enforced shadows, full-strength first pass. Runtime 518.570 seconds. All 50
global paths exist, zero unrouted nets, but the plan is incomplete. Full native
assembly reports 39 conflicts, six classified fixed; local native checking
reported zero copper violations and six legal windows/checks. No final board
or KiCad DRC was produced. These counters describe different stages, not a pass.

Window 3 owns net23 connection1/run0/fragment1 and net4
connection0/run0/fragment6 on F.Cu, around x29.5–30.5/y43–44 mm. The selected
templates [5,9] are locally legal in every spline check. The net4 owned stub
ends at (30.5,43.5); its untouched horizontal continuation crosses net23's
replacement bulge near (31.38,43.5). Local native assessment clones only the
source board and local splines, whereas assembly includes the global routes.
Thus a first-locally-legal early exit discards opportunities to select another
template before discovering the outside-copper conflict. Preview geometry is
diagnostic, not final accepted copper. B.Cu two-via alternatives exist, but
their eventual full-scene legality is not established.

Ranked explanations/experiments:

1. Missing untouched copper in local validation (selected): retain unowned
   global fragments in the exact native scene, excluding fragment IDs rather
   than entire nets. Expected effect: unsafe candidates rejected before early
   exit, more alternatives evaluated. Conservative because neighbouring windows
   may subsequently move their copper. Opt-in, no clearance changes.
2. Conflict-directed post-assembly candidate reselection: less conservative,
   but requires coordination across retained window portfolios. Next if freezing
   prevents necessary simultaneous movements.
3. Missing proposed-via geometry in local validation: known separate coverage
   gap, not necessary to explain this planar crossing; defer to an isolated test.

Research: KiCad's NODE branches query both branch and root spatial indices,
excluding overridden items (`NODE::Branch`, `OBSTACLE_VISITOR::visit`,
`NODE::QueryColliding`):
https://raw.githubusercontent.com/KiCad/kicad-source-mirror/master/pcbnew/router/pns_node.cpp
The relevant invariant is validation against unchanged context. GPL source was
inspected for the concept only; no implementation was copied. Flux uses a
temporary Board and existing exact predicates, not KiCad's index implementation.
Scene construction is linear in fragments plus ordered-set membership cost;
exact checking cost is unchanged algorithmically but has a larger input.

Experiment flag: `--frozen-copper-context`. Baseline defaults remain unchanged.
Focused regression: local owned stub plus same-net untouched continuation,
crossing candidate, expanded joint ownership, another copper layer, missing
window, and immutable source. Final scene validation remains mandatory. This
experiment does not claim coverage of proposed vias or inter-window motion.

## Outcome

Enabled run: `build/frozen-copper-context-tier0`, 431.656 seconds. Still 50 paths,
zero unrouted nets, incomplete plan. Candidate evaluations increased 4 → 21;
local native checks report 87 violations, three legal candidates, zero fit
failures. Window 3 remains unresolved with all eight finally retained states
failing native clearance (including B.Cu alternatives). Assembly is withheld;
its zero conflicts must not be compared as a legality improvement. No final
board/DRC. Physical-scope retries decreased 1 → 0; the unchanged provisional
oracle reports 32 violations and zero opens.

The test confirms a missing-context acceptance gap, but does not solve routing.
Keep disabled by default as a diagnostic experiment. Core plus nine arc/multilayer
checks passed (173.18 seconds, including two KiCad DRC checks); the focused
transactional regression passed separately. Machine-readable comparison:
`reports/benchmark_history/2026-09-06-frozen-context-comparison.json`.

Next: preserve outside-fragment identities in native rejection evidence before
scope expansion. Current generic window diagnoses omit those blockers, and the
scope wrapper only retries assembled-native/linear-scaffold conflicts. Do not
infer the exact remaining blocker from a final telemetry snapshot: repair-cone
alternatives may still be present there. No further scope heuristic is enabled
without establishing those exact pairs.
