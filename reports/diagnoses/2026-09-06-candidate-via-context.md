# Candidate-via context experiment

## Reproduction

Tier 0 pt2399_delay, N6/R0, enforced shadows, full-strength first pass,
exact frozen scope expansion, diagnostic timeout 900 s. Reporting-only binary
6b30e95efcbc6060922dcb996f60c40aa749b51793a01169204c1d1f131f598b
finished in 718.651 s, incomplete. Last scene has two track_via_clearance
reports, each with 125000 nm deficit. Native GND B.Cu track object 649 spans
(28,44)–(34.5,44) mm; new +5V vias are objects 799 and 800. These are
last-assessed-scene IDs, not canonical board objects or final KiCad DRC.

Local assessment checks native planar pieces but omits selected candidate vias.
Consequently local mutual legality does not imply legality after via insertion.
The final invalidation uses net identity, marking an unrelated +5V/+9V window
failed as well. Its location must not be mistaken for the physical collision.

## Ranked hypotheses

1. Include selected candidate vias, with real copper/drill/span and reuse rules,
   in local native assessment. Smallest direct test of the missing context.
2. Attribute full-scene invalidation to exact owning fragments, not entire nets.
   Improves repair targeting but alone cannot make the colliding vias legal.
3. Generate/optimise via sites against changing neighbouring native copper.
   Larger search change, appropriate if rejecting unsafe candidates exhausts
   the existing portfolio.

## Research

KiCad's router treats vias as collision-tested copper objects, including in
shove operations: https://docs.kicad.org/7.0/en/pcbnew/pcbnew.html and
https://gitlab.com/kicad/code/kicad/-/blob/master/pcbnew/router/pns_via.cpp .
The applicable idea is candidate-scene validation, not a wholesale router port.
Flux already has exact copper/hole predicates; reuse these, no external code.

## Minimum experiment

Materialize resolved entry/exit vias into the local native scene before accepting
a candidate. Attribute via conflicts to all attached candidate fragments, so
existing rejection and frozen-fragment feedback can act on them. Preserve
source immutability and existing same-net via/plated-pad reuse. Keep this within
the opt-in frozen-context experiment. Do not change retry budgets or clearance.

Micro-test: a trace 0.5 mm from a proposed via centre fits the neighbouring
0.25 mm trace but not the full via copper plus 0.2 mm clearance. Also verify
shared-site deduplication, layer span, source preservation, and reuse.
Success: no locally accepted candidate hides this via collision; benchmark
success still requires complete output and final KiCad DRC. An earlier rejection
alone is not a routing-quality improvement.

## Rejection-only result

Timed out at 900 s during the first enlarged-scope retry; no final JSON or DRC.
The local check reports exact via/blocker pairs (including +5V/GND and REF/+5V)
and changes selected scope feedback, but a larger search has not completed.
10/10 selected regressions pass (181.52 s, two KiCad DRCs), as does the expanded
reuse/provenance micro-test. Keep default off. History:
`reports/benchmark_history/2026-09-06-candidate-via-context.json`.

## Refinement: guide existing continuous optimisation

The existing regional clearance evaluator and its GPU broad-phase model contain
planar routes only. Thus trajectory optimisation cannot see the via envelope
that the new final local check rejects. Existing regression
`constrained_ph_rebound_avoids_inter_sample_obstacle` demonstrates that Flux can
bend a straight spline around a circular obstacle when given that evaluator.
Hypothesis: include fixed candidate via envelopes in this existing evaluator and
its broad phase. No new optimiser, retries or clearance relaxation. Final native
and KiCad gates remain authoritative, including hole rules not represented by
this copper-clearance optimisation objective. Test clearance radius, layer/net
filtering, an inter-sample via rebound, and rerun the same frontier configuration.

## Refined result and decision

Also timed out at 900 s. The first regional pass ends at 433.404 s; the enlarged
retry prepares 3 windows / 50 templates at 497.552 s. Retained six-route window
profiles at 732.932 s and 838.697 s still have topology=0, shape=0, mutual=1,
quantization=1. GND's y=44 mm candidate is now correctly reported mutually
illegal, but its retained geometry remains straight. At 885.978 s another REF
candidate still intersects outside +5V fragments. These are diagnostic candidate
observations, not final copper metrics; no final planning JSON or KiCad DRC exists.

The isolated 125000 nm via deficit is solved by the ordinary regional two-iteration,
64-check optimisation budget in 44 checks without exhaustion. The larger micro-test
also passes shape/corridor/continuous certification and exact native arc/segment
copper checks. CPU/GPU disk proximity checks and all 10 selected regressions pass
(183.53 s, including two KiCad DRC fixtures). Thus a generic missing circular-obstacle
solver or merely too small a per-attempt budget is not established as the remaining
cause. Guidance closes a representation gap, not the benchmark frontier.

Decision: retain the tested context/guidance work only in the existing default-off
experiment. No tier promotion, relaxed rules, timeout extension beyond 900 s, or
new optimiser. History: `reports/benchmark_history/2026-09-06-candidate-via-guidance.json`.
Next: reproduce the real six-fragment window's ownership, fixed endpoints and
NavMesh guard constraints in a regional micro-case. Determine whether legal
deformation is forbidden by the chosen corridor/ownership or missed by its search.
If this local topology is trapped, feed the conflict into broader connection/corridor
selection rather than layering on more local penalties. Whole-net assembly
invalidation remains a separate, demonstrated repair-targeting defect.
