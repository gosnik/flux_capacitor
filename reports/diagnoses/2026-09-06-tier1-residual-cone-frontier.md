# Tier-1 residual allocation frontier

The frozen V15 `bm_svf` control timed out at 1,800 seconds. Its first three copper
checkpoints remained 764, 1,297 and 2,216 violations with zero opens. The fourth
ordinary spline batch finished at 1,067.534 seconds. Tier 1 is not passed.

Residual cone 0 retained 268 templates, including 212 using vias, but exhausted
100,000 bundle states without a complete access/lane-feasible selection. It
therefore invoked no spline realization. Eight recorded same-layer detour
attempts, involving nets 9, 11, 22 and 26, all returned zero candidates. These
are the observed generator participants, not a proven minimal unsatisfiable net
set. The trace does not publish the cone bounds or a decisive constraint core.

The entire cone portfolio stage cost 178.032 seconds; this includes alternative
generation, preliminary screening, enumeration and leaf checking. Existing
telemetry cannot attribute that duration to the DFS alone. A `state_budget_exhausted`
result likewise does not distinguish zero raw leaves from leaves rejected by
the later access/lane solvers. The next experiment adds those measurements.

Cone 1 found a retained pool of 584 complete discrete states and entered physical
realization. Observed active candidates had 19–22 native-illegal routes out of
22–24. Its first reserve wave began at 1,653.729 seconds; five reserve candidates
finished before timeout. Cone 2 was not reached. No final KiCad DRC was run.

The code-supported hypothesis is delayed propagation: the primary DFS previously
checked via identity and aggregate width, then applied access and adjacent-run
layer constraints only to complete choices. A contradictory early escape can
consume the search budget while unrelated fragments vary underneath it.

The ranked approaches are (1) exact forward checking in the existing DFS,
(2) maintaining arc consistency across all domains, and (3) a bounded SAT/CP-SAT
master for the residual region. Raising the state limit lacks evidence and
increases the already substantial runtime.

[Haralick and Elliott](https://www.ijcai.org/Proceedings/79-1/Papers/004.pdf)
provide the forward-checking basis. [Wood and Rutenbar](https://users.ece.cmu.edu/~rutenbar/pdf/rutenbar-satisfpga97.pdf)
motivate simultaneous routing constraints, with fixed FPGA resources as an
important difference from Flux's continuous geometry. The smallest experiment
keeps the current search order and introduces cached pairwise access proofs,
forward domain checks, and full-leaf coupling/access checks. Exhausted access
queries remain unknown. Independent generation and the exact downstream lane,
spline, quantization and DRC stages retain their existing authority.

Success requires a useful discrete selection within the same state budget, plus
lower-tier correctness. A synthetic escape-order trap and a movable-via case
test the mechanism separately from the full board.
