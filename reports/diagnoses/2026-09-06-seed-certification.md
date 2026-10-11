# Seed certification before global negotiation

Frontier: Tier 0 pt2399_delay. Prior N1 planning produced an accepted native scene
with 50 paths, zero unrouted nets, three complete regional assignments and zero
native-scene conflicts after two local scope retries (435.384 s). N6/R4 default
physics runs instead timed out; feedback traded 32 oracle violations/0 opens for
0 violations/9 opens. Normal N30 detail was cancelled after the audit cap with
58 violations/10 opens. Final KiCad legality of the N1 scene is not yet known.

Earliest measured divergence is scheduling, before new physics: full-strength
mode consumes N-R global iterations before its first physical assessment, and
disables local scope retries until the final epoch. Thus N1 receives a physical
certification opportunity that N30 does not. More global budget can bury a useful
seed under a proxy-ranked topology. This is a representation/checkpoint selection
failure hypothesis, not proof that the N1 copper is finally legal.

Ranked candidates:
1. Certify the initial global seed with the full physical scope budget before
   spending the remaining global budget; retain full physics on later epochs.
2. Roll back incomplete hard-capacity reroutes more aggressively to preserve
   connection coverage (addresses the later 9-open trap, not lost seed opportunity).
3. Preserve a larger global portfolio (more compute; may retain similar topologies).

Minimum experiment: use the existing seed-first budget distribution for full
strength too; allow its configured scope retries on the first and final epochs.
No new routing algorithm, clearance changes, or total-iteration increase. Test
that larger budgets certify the same seed early, and that failed seeds still get
bounded repeated physics/rerouting. Run N1 full export/DRC as a control first.

Additional seed dependence: the default shadow policy was advisory whenever
retries were available, enforced otherwise. For a valid seed-first comparison,
the default first epoch must use the same enforced policy as N1. Explicit policy
overrides remain untouched; later default epochs retain oracle-owned shadows.

Research: VPR documents distinct first-iteration congestion costs and retaining
the best legal implementation when reconverging
(https://docs.verilogtorouting.org/en/latest/vpr/command_line_usage/).
OpenROAD separates global route guides from detailed search/repair and DRC
(https://openroad.readthedocs.io/en/latest/main/src/drt/README.html).
Flux inference: a proxy-ranked checkpoint is not a physical incumbent; certify
the seed before allowing additional negotiation to hide it. Integration reuses
existing scheduling and exact gates, with one early regional certification cost.
