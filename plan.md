# Flux Autorouter — Iterative Research, Integration, and Benchmark Improvement Instructions

You are working on **Flux**, an experimental KiCad PCB autorouter.

Your objective is not merely to improve routing quality incrementally. Your objective is to **systematically advance Flux through its complete routing benchmark suite until every test-board tier passes**, while preserving or improving performance on all previously passing tiers.

Flux currently:

* completes **Tier 0**
* does **not complete Tier 1**
* has additional increasingly difficult tiers after Tier 1
* uses test boards drawn from KiCad routing/router projects and related routing benchmarks

Flux already contains substantial routing machinery. Do not assume that replacing the architecture wholesale is appropriate.

The current broad routing architecture includes:

1. immutable canonical KiCad board representation
2. clearance-expanded geometry
3. obstacles, board edges, pads and keepouts
4. spatial indices
5. NavMesh / free-space representation
6. terminal and via candidate generation
7. global route allocation
8. route portfolios
9. PathFinder-style iterative congestion negotiation
10. graph topology and layer assignment
11. corridor/resource capacity and history
12. clearance-shadow/resource conflicts
13. via alternatives
14. physical copper conflict extraction
15. conflict graphs
16. regional joint routing
17. corridor alternatives
18. funnel / continuous geometry
19. continuous route geometry
20. B-spline-based route representation
21. final quantisation to legal KiCad copper
22. KiCad DRC verification

A recurring historical failure mode has been:

> Flux can often find combinatorial paths for all nets, but cannot convert those paths into mutually legal physical copper.

Therefore always distinguish between:

* topological routability
* resource/capacity feasibility
* geometric feasibility
* layer/via feasibility
* detailed copper legality
* final KiCad DRC correctness

Do not consider a board routed merely because every net has a path.

---

# Primary Goal

Iteratively:

**diagnose → research → hypothesise → implement → verify → benchmark → analyse → refine**

until:

* Tier 0 passes
* Tier 1 passes
* Tier 2 passes
* …
* every available tier passes

A higher tier must never be considered solved by breaking lower tiers.

The benchmark suite is the authority.

---

# Core Working Principle

Do not make speculative large rewrites.

Use an **evidence-driven experimental loop**.

At any point, you should be able to answer:

1. What specifically prevents the current board from completing?
2. What evidence demonstrates that?
3. What algorithmic family might address that failure?
4. Why should that family help this particular failure?
5. What is the smallest useful experiment?
6. What changed in the benchmark results?
7. Did the change solve the intended mechanism, or merely move the failure elsewhere?

Treat router development as applied algorithm research rather than ordinary feature development.

---

# Autonomous Iteration Loop

Repeat the following loop continuously.

## Phase 1 — Establish the Current Baseline

Run the complete benchmark suite that is currently practical.

At minimum record for every board:

* pass/fail
* routed nets
* unrouted nets
* open connections
* track count
* via count
* route length
* runtime
* memory consumption where available
* global-routing overflows
* resource congestion
* corridor congestion
* physical conflicts
* conflict graph size
* shorts
* track crossings
* clearance violations
* hole-clearance violations
* via-related violations
* malformed geometry
* KiCad DRC violation counts by category

Store benchmark output in a structured format suitable for comparison between runs.

Never rely on console impressions.

Create or maintain a benchmark history.

---

## Phase 2 — Select the Frontier Board

Work primarily on the **first failing tier**.

Within that tier, select the smallest or clearest representative failing board unless another board exposes the underlying issue substantially better.

The normal progression should be:

Tier 0 remains green
↓
solve Tier 1
↓
Tier 0 + Tier 1 remain green
↓
solve Tier 2
↓
etc.

Do not prematurely optimise later tiers while an earlier tier still exposes fundamental failures.

---

# Phase 3 — Reproduce and Characterise the Failure

Before changing code, reproduce the failure.

Instrument Flux where required.

Determine which stage first becomes infeasible or loses useful information.

Classify the failure into one or more categories such as:

### Global topology failure

Examples:

* no path exists in the routing graph
* poor terminal escape choices
* poor via candidates
* wrong layer topology
* disconnected NavMesh
* insufficient route alternatives

### Allocation / congestion failure

Examples:

* PathFinder oscillation
* persistent resource overflow
* poor history-cost behaviour
* bottlenecks assigned to too many nets
* route ordering sensitivity
* starvation of difficult nets
* locally optimal but globally impossible allocations

### Detailed geometry failure

Examples:

* centreline path exists but copper does not fit
* clearance envelopes overlap
* funnel paths collapse
* spline smoothing creates illegal copper
* neck-down opportunities are missed
* corner geometry is unnecessarily expensive

### Via/layer failure

Examples:

* legal path requires different via location
* via clearance blocks neighbouring nets
* layer assignment becomes trapped
* excessive vias cause local congestion
* stacked decisions made independently should be solved jointly

### Local interaction failure

Examples:

* two or more nets require coordinated movement
* sequential routing cannot solve a channel
* push-and-shove cannot escape a local minimum
* route repair repeatedly moves the same conflict elsewhere

### Search failure

Examples:

* search horizon too local
* alternative corridors pruned too early
* insufficient route diversity
* poor cost heuristic
* high-quality solution exists but search never explores it

### Physical legalisation failure

Examples:

* abstract allocation says capacity is available but true copper does not fit
* resource capacity model does not accurately represent physical packing
* path quantisation introduces crossings
* smoothing/legalisation invalidates formerly valid topology

### Rip-up/reroute failure

Examples:

* same nets repeatedly reclaim the same resources
* congestion history is ineffective
* victim selection is poor
* routing order dominates the result
* router lacks sufficient perturbation to escape a fixed point

Do not treat DRC violations as a single undifferentiated metric.

Identify their physical causes.

A single collision may produce several KiCad DRC reports.

---

# Phase 4 — Produce a Concrete Failure Report

Before researching solutions, create a short machine-readable and human-readable diagnosis containing:

* board
* tier
* failing stage
* smallest set of implicated nets
* implicated physical region
* relevant layers
* relevant corridors/resources
* whether the failure is topological or geometric
* whether an alternative route appears to exist
* dominant DRC categories
* suspected root mechanism
* supporting metrics

Where useful, produce temporary diagnostic geometry or logs.

Prefer precise statements such as:

> Nets A, B and C repeatedly share corridor R17. The global allocator reports zero NavMesh overflow, but clearance-expanded copper envelopes overlap by 0.31 mm after via insertion. All PathFinder iterations converge to the same three corridor choices.

over statements such as:

> Routing seems congested.

---

# Phase 5 — Search the Internet for Relevant Techniques

Once the failure mechanism is understood, research algorithms specifically relevant to it.

Internet research is part of the development process.

Search broadly across:

* PCB autorouting literature
* VLSI routing
* FPGA routing
* IC detailed routing
* global routing
* maze routing
* negotiated congestion routing
* PathFinder
* rip-up and reroute
* push-and-shove routing
* track assignment
* channel routing
* escape routing
* bus routing
* detailed routing
* geometric shortest-path algorithms
* computational geometry
* robotics path planning
* multi-agent path finding
* MAPF
* CBS / Conflict-Based Search
* ECBS
* prioritized planning
* dynamic obstacle planning
* motion planning
* visibility graphs
* navigation meshes
* funnel algorithms
* homotopy-aware planning
* continuous optimisation
* spline optimisation
* trajectory optimisation
* sequential convex optimisation
* packing
* graph colouring
* min-cost flow
* multicommodity flow
* constraint programming
* SAT/SMT techniques
* ILP/MILP approaches
* simulated annealing
* large-neighbourhood search
* tabu search
* evolutionary optimisation
* local repair methods
* Steiner trees
* topology optimisation

Also inspect implementations and papers from relevant open-source projects.

Potentially useful sources include, but are not limited to:

* KiCad interactive router
* FreeRouting
* TopoR
* pcb-rnd
* OpenROAD
* TritonRoute
* FastRoute
* CUGR
* FPGA routing research
* VPR
* academic PCB autorouters
* modern VLSI detailed routers
* multi-agent path-planning libraries
* game pathfinding literature

Do not search for generic “PCB routing algorithm” results only.

Search based on the observed failure.

For example:

> negotiated congestion routing narrow channel physical packing

> detailed routing local rip-up reroute conflict graph

> PCB routing simultaneous routing multiple nets bottleneck

> VLSI detailed routing track assignment minimum conflict

> MAPF continuous agents different radii conflict based search

> PathFinder oscillation congestion history adaptive penalty

> push shove routing local minimum rip up reroute

> topology preserving path smoothing obstacles

Read primary papers, technical documentation and source code where practical.

---

# Phase 6 — Maintain a Research Notebook

For every promising technique record:

* algorithm / technique name
* source
* problem it solves
* assumptions
* complexity
* how closely those assumptions match Flux
* likely integration point
* expected benefit
* likely downside
* minimum experiment needed to test it
* result after testing

Do not repeatedly rediscover the same approaches.

Also record approaches that were tested and rejected.

A failed experiment is useful information.

---

# Phase 7 — Rank Candidate Approaches

Do not immediately implement the first interesting algorithm.

Rank candidates.

Prefer changes that:

1. directly address the measured failure
2. fit Flux's existing architecture
3. can be experimentally isolated
4. preserve useful route diversity
5. produce measurable intermediate metrics
6. are reversible
7. generalise beyond one benchmark board

Avoid board-specific hacks.

A heuristic may depend on general geometric or routing properties, but never on:

* board filename
* exact benchmark identity
* specific net names
* hard-coded coordinates

---

# Phase 8 — Design the Smallest Useful Experiment

For each selected technique formulate a hypothesis.

Example:

> Hypothesis: persistent Tier-1 failure occurs because corridor capacity is represented topologically rather than using clearance-aware packing. Adding geometry-derived edge capacity will prevent the global allocator from committing incompatible nets to the same bottleneck.

Or:

> Hypothesis: sequential local repair cannot resolve three-net swaps. Jointly rerouting the connected component of the physical conflict graph will allow topology changes unavailable through pairwise push-shove.

Then define:

* code change
* expected metric change
* expected board behaviour
* success criterion
* regression criterion

Implement the minimum needed to test the hypothesis.

Do not bundle several unrelated algorithms into one experiment unless absolutely necessary.

---

# Phase 9 — Integrate Cleanly

New algorithms should normally fit into Flux through explicit interfaces.

Avoid accumulating one-off conditions.

Where practical separate:

* candidate generation
* scoring
* search
* conflict detection
* negotiation
* legalisation

from orchestration.

Algorithms should expose diagnostic information.

For example:

* alternatives considered
* alternatives rejected
* cost terms
* expanded nodes
* conflict count
* rip-up reason
* congestion penalties
* capacity estimates
* convergence behaviour

This instrumentation is part of the implementation, not temporary debugging clutter.

---

# Phase 10 — Verify at Multiple Levels

Every algorithm change must be tested at several levels.

## Unit / local algorithm tests

Add deterministic tests for the new mechanism.

Examples:

* two-net channel swap
* three-net narrow passage
* via blocking channel
* forced layer transition
* local bottleneck
* U-shaped obstacle
* conflicting spline smoothing
* insufficient physical width despite topological connectivity
* route ordering trap

Use synthetic micro-boards when useful.

Micro-tests should reproduce the algorithmic mechanism independently of a large benchmark board.

## Regression tests

Run existing tests.

No previously passing test may silently regress.

## Tier benchmark

Run:

* the frontier board
* its entire tier
* all lower tiers

If promising, run every tier.

## KiCad validation

Final copper must be verified using KiCad DRC.

Internal Flux validity is necessary but not sufficient.

---

# Phase 11 — Analyse the Result

Compare before/after metrics.

Do not ask merely:

> Did it pass?

Ask:

* Did global congestion decrease?
* Did physical conflicts decrease?
* Did via count explode?
* Did runtime increase significantly?
* Did the same conflicts remain?
* Did conflicts move?
* Did route diversity improve?
* Did the number of reroutes increase?
* Did search actually explore the intended alternatives?
* Did topology improve while geometry worsen?
* Did DRC categories change?
* Did an earlier stage become more accurate?
* Did the router converge or oscillate?

Determine whether the hypothesis was:

* strongly supported
* partially supported
* disproven
* inconclusive

Record the result.

---

# Phase 12 — Keep, Refine or Revert

If the experiment produces clear general improvement:

* keep it
* add regression coverage
* commit it as a coherent change

If it shows promise but remains incomplete:

* refine it

If it does not help:

* revert or disable it
* record why
* move to the next hypothesis

Do not leave layers of failed heuristics active “just in case”.

Complexity must earn its place through benchmark results.

---

# Phase 13 — Escape Local Optima

If repeated modifications only reduce violation counts without completing the tier, reconsider the level at which the problem is being solved.

A common autorouter failure is attempting to repair a global topological mistake using increasingly sophisticated local geometry.

Watch specifically for:

* repeated conflicts between the same nets
* oscillation between equivalent states
* congestion moving between adjacent resources
* local repairs recreating previous conflicts
* heavy sensitivity to routing order
* excessive via growth
* route-length growth without conflict reduction
* physical legaliser fighting global allocation

When observed, move the feedback upstream.

For example:

physical copper conflict
→ identify participating nets
→ identify their route portfolios
→ penalise responsible corridor combination
→ regenerate alternatives
→ rerun regional/global allocation

Do not force the final geometry stage to solve mistakes that should have been reconsidered earlier.

---

# Feedback Between Routing Representations

Flux has several representations of the same routing problem.

Treat them as a feedback system rather than a one-way pipeline.

Useful feedback may include:

### Detailed geometry → global routing

Physical conflicts should modify:

* corridor cost
* corridor capacity
* route portfolio ranking
* congestion history
* layer preference
* via-placement preference

### Detailed routing → terminal analysis

Repeated failed pad escapes may trigger regeneration of terminal candidates.

### Local conflict graph → joint routing scope

Connected physical conflicts may define the region and net set for simultaneous rerouting.

### Legalisation → continuous geometry

If quantisation repeatedly invalidates a spline/path, feed legalisation constraints into the continuous optimiser.

### Continuous geometry → global allocation

If a nominal corridor cannot physically fit the route width/clearance combination, reduce its effective capacity rather than repeatedly rediscovering the failure downstream.

The router should learn from failed physical realisations.

---

# Algorithm Families Worth Explicitly Investigating

These are research directions, not mandatory implementation requirements.

## Clearance-aware corridor capacity

Replace simplistic occupancy with estimates based on:

* available corridor width
* required trace width
* clearance
* via diameter
* via clearance
* local curvature
* layer availability

Investigate interval/strip packing rather than grid occupancy.

---

## Homotopy-aware route portfolios

Preserve fundamentally different path classes instead of numerous small variations of the same path.

A portfolio of 10 nearly identical routes is much less useful than 3 topologically distinct routes.

Investigate:

* homotopy classes
* obstacle signatures
* corridor sequences
* portal sequences
* visibility graph alternatives
* Yen/Eppstein k-shortest paths with diversity penalties

---

## Conflict-directed rerouting

Instead of random or net-order rip-up:

1. construct physical conflict graph
2. find connected components
3. rank responsible nets/resources
4. reroute the component jointly or in coordinated sequence

Investigate ideas from:

* Conflict-Based Search
* large-neighbourhood search
* VLSI detailed-routing repair

---

## Large-neighbourhood search

When locally trapped:

* rip up a coherent subset of routes
* temporarily reserve useful structure outside the region
* regenerate alternatives inside the region
* solve that region again

Vary neighbourhood size adaptively.

---

## Adaptive negotiated congestion

Investigate stronger PathFinder variants:

* historical congestion
* present congestion
* adaptive penalty growth
* overflow-sensitive cost scaling
* resource-specific learning rates
* deterministic tie-breaking
* randomized diversification
* congestion decay
* oscillation detection

---

## Multi-commodity-flow approximations

Use relaxed global flow to estimate which bottlenecks fundamentally cannot support the proposed routing arrangement.

This need not become the final router.

It may provide better:

* corridor pressure
* capacity estimation
* ordering
* decomposition

---

## Track assignment

Inside narrow channels, convert continuous free-space routing into a track-assignment problem where appropriate.

Determine:

* ordering of traces
* feasible lanes
* layer assignment
* via insertion

before producing final curved geometry.

---

## Simultaneous local routing

Some regions should route several nets together rather than sequentially.

Potential formulations:

* joint A*
* CBS-like search
* min-cost flow
* ILP
* bounded combinatorial enumeration
* dynamic programming

Use expensive methods only on small conflict regions.

---

## Push-and-shove plus topology change

Push-and-shove is useful when a nearby valid deformation exists.

It is insufficient when the required solution changes topology.

Detect when shove displacement ceases to improve feasibility and trigger:

* rip-up
* alternative corridor selection
* via relocation
* layer reassignment

rather than endlessly pushing.

---

## Difficulty-aware net ordering

Investigate routing priority based on:

* escape freedom
* route entropy
* number of route portfolios
* bottleneck dependence
* pad density
* clearance width
* required layer changes
* expected via scarcity

The most constrained nets may deserve early reservation.

Also investigate dynamically changing priority during negotiated routing.

---

## Via-aware planning

Do not treat vias as simple point transitions.

Model their true:

* copper diameter
* drill
* annular ring
* clearance shadow
* layer interaction

Generate via candidates according to regional packing feasibility.

---

## Continuous optimisation

Once topology is fixed, investigate optimisation objectives combining:

* path length
* curvature
* clearance
* obstacle distance
* neighbouring copper distance
* via placement
* manufacturability

Possible techniques include:

* elastic bands
* CHOMP-like optimisation
* sequential convex approximation
* constrained B-spline optimisation

But do not use continuous optimisation to compensate for an incorrect topology.

---

# Benchmark Promotion Rules

A tier is considered passed only when:

* all required connections are routed
* there are zero opens
* no forbidden shorts exist
* no crossings exist where prohibited
* clearance rules pass
* hole clearance passes
* via legality passes
* board-edge constraints pass
* KiCad DRC passes according to the benchmark's expected rule set

Once a tier passes, add it to the mandatory regression set.

Do not manually promote a tier based on visual inspection.

---

# Performance Discipline

Correctness comes first, but track computational cost.

For each meaningful improvement record:

* runtime
* route-search expansions
* reroute iterations
* peak conflict count
* peak memory where available

An algorithm that solves a formerly impossible board at higher runtime may initially be acceptable.

After correctness is established, optimise expensive stages.

Use hierarchical computation:

cheap global approximation
→ targeted expensive regional solve

rather than applying exponential algorithms to the entire board.

---

# Source-Code Investigation

When an algorithm from another project appears relevant, do not copy it blindly.

Inspect:

* data representation
* invariants
* cost model
* conflict representation
* update strategy
* convergence criteria
* assumptions about Manhattan routing
* assumptions about grids
* assumptions about fixed track widths
* licence compatibility

Translate the underlying idea into Flux's continuous/corridor architecture where appropriate.

Prefer algorithmic concepts over cargo-cult ports.

---

# Git Discipline

Keep experiments recoverable.

Before substantial changes:

* ensure the baseline state is known
* preserve benchmark results

For successful changes:

* create focused commits
* include the hypothesis or reason in the commit message
* include relevant benchmark improvement where practical

Do not combine unrelated router experiments into a single enormous change.

Never discard useful existing work merely to simplify an experiment.

---

# Research and Experiment Log

Maintain a persistent development document, for example:

`docs/router_research_log.md`

Each experiment should contain roughly:

## Experiment N — <name>

### Failure

Board/tier and observed failure.

### Hypothesis

What mechanism is believed to be responsible.

### Research

Relevant algorithms/papers/projects and why they apply.

### Change

What was implemented.

### Before

Metrics.

### After

Metrics.

### Interpretation

What changed and why.

### Decision

Keep / refine / revert.

### Next hypothesis

What should be investigated next.

---

# Structured Benchmark History

Also preserve benchmark data in machine-readable form.

For example:

`reports/benchmark_history/`

Each run should identify:

* git revision
* date
* configuration
* board
* tier
* router parameters
* outcome
* metrics
* DRC summary

This will prevent optimisation based on memory or anecdote.

---

# Important Anti-Patterns

Do **not**:

* optimise only for violation count
* accept combinatorial connectivity as successful routing
* introduce benchmark-board-specific hacks
* endlessly tweak constants without diagnosing the failure
* add random penalties without measuring their effect
* route higher tiers while lower tiers regress
* add heuristics that cannot be inspected
* hide routing failures with relaxed DRC rules
* reduce legal clearance merely to make benchmarks pass
* discard difficult nets
* silently omit vias or connections
* declare success from an incomplete preview
* rewrite major subsystems without evidence that the architecture is the blocker
* assume grids are inherently required
* assume gridless routing is inherently superior
* assume B-splines solve routing topology
* assume PathFinder alone will solve physical copper packing

---

# When Progress Stalls

If several consecutive experiments fail to materially improve the frontier tier:

1. stop tuning
2. rerun the baseline
3. inspect traces/logs
4. identify the earliest stage where the eventual conflict becomes inevitable
5. search literature specifically for that mechanism
6. construct a minimal synthetic board reproducing it
7. solve the synthetic case
8. integrate the solution
9. rerun the real tier

If necessary, temporarily use an exact/expensive solver on the small failing region to determine whether:

* a solution exists
* Flux's heuristic is failing to find it

This distinction is extremely valuable.

---

# Differential/Oracle Testing

Where practical, compare Flux against another router.

For a board Flux cannot solve:

* route it with KiCad interactive router, FreeRouting, another available autorouter, or an existing known-good solution
* examine topology rather than merely track coordinates
* identify:

  * different bottleneck usage
  * different layer allocation
  * different net ordering
  * different via placement
  * different escape direction
  * different corridor choice

Do not simply copy the resulting tracks.

Use the external result as an oracle demonstrating what structural decision Flux is missing.

---

# Automatic Parameter Exploration

If an algorithm is sound but parameter-sensitive, perform bounded automated sweeps.

Examples:

* congestion penalty
* history multiplier
* alternative-route count
* regional repair size
* rip-up threshold
* route diversity weighting
* via penalty

Evaluate parameter combinations against multiple boards.

Do not hand-tune a value solely for one benchmark.

Promote parameters that generalise.

---

# Randomisation

Use controlled randomisation when useful for escaping deterministic local minima.

Always record the seed.

If different seeds yield dramatically different results, investigate why.

Successful stochastic solutions can reveal missing deterministic heuristics.

A benchmark should preferably become reliably solvable rather than pass only with a lucky seed.

---

# Stop Condition

Continue this process without asking for routine permission after each experiment.

The task is complete when:

1. every available routing tier passes
2. all lower-tier regressions remain green
3. final output is accepted by KiCad DRC
4. successful algorithms have regression tests
5. benchmark history records the progression
6. abandoned experiments are documented sufficiently to avoid repeating them
7. the resulting implementation remains general rather than benchmark-specific

If complete success is not yet achievable, continue advancing the frontier and leave the repository in the best verified state reached.

Do not stop merely because one attempted algorithm failed.

Choose the next evidence-backed hypothesis and continue.

---

# Immediate Starting Procedure

Begin now with the following sequence:

1. inspect the Flux repository and identify the benchmark/tier runner
2. determine exactly which boards comprise Tier 0, Tier 1 and subsequent tiers
3. run Tier 0 to confirm the known baseline
4. run Tier 1
5. identify the first failing Tier-1 board
6. collect detailed routing and KiCad DRC metrics
7. locate the smallest physical conflict set responsible for failure
8. trace that conflict backwards through:

   * quantised copper
   * continuous geometry
   * regional routing
   * corridor choice
   * route portfolio
   * global allocation
9. identify the earliest decision that makes the conflict likely or inevitable
10. formulate 2–5 candidate algorithmic explanations
11. research those mechanisms on the internet
12. rank the approaches
13. implement the smallest high-value experiment
14. add a focused regression/micro-test
15. rerun the failing board
16. rerun all of Tier 1
17. rerun Tier 0
18. retain or revert based on evidence
19. record the result
20. repeat

As soon as Tier 1 passes reliably, move the frontier to Tier 2 and repeat the same process.

The benchmark ladder—not the elegance of any particular algorithm—is the final measure of progress.

