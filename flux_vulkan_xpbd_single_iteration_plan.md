# Flux Single-Iteration Development Plan: Vulkan XPBD Routing Relaxation

## Background

This development iteration grew from four related design discussions:

1. using gaming-style terrain erosion as a routing feedback model;
2. combining that learned terrain with a physics-style geometry solver;
3. identifying practical GPU PBD/XPBD references; and
4. mapping the resulting solver onto Flux's existing Vulkan compute module.

The central proposal is not to replace Flux's global router with a terrain
simulation or a general-purpose physics engine. It is to introduce a
hierarchical feedback system in which:

- PathFinder chooses topology and negotiates discrete resources;
- an XPBD-like constrained-rod solver jointly settles traces and vias into
  physically legal local geometry; and
- a persistent erosion, pressure, and history field learns from successful
  geometry and unresolved physical stress.

### Gaming terrain erosion as a routing model

The useful part of terrain erosion is its spatial dynamics, rather than a
literal hydraulic simulation. The analogy is:

| Terrain erosion | PCB routing analogue |
|---|---|
| Height | Routing cost |
| Water | Route demand |
| Flow direction | Preferred route direction |
| Sediment | Congestion or conflict history |
| Erosion | Making successful corridors cheaper |
| Deposition | Making overloaded or infeasible regions more expensive |
| Drainage basin | Natural routing channel |
| Ridge | Routing barrier |
| Stream merging | Shared global corridor demand |
| River avulsion | Nets moving to a different corridor family |

PathFinder already raises the cost of congested resources over successive
rip-up/reroute passes. A terrain model extends that idea with **spatial
diffusion and physical feedback**. Instead of penalizing only the exact graph
edge on which a failure occurred, Flux can deposit pressure at the physical
conflict, spread it into the surrounding free space, and guide later searches
away from the whole problematic region.

For each layer, define a routing-height field:

\[
H(x,y,l) = C_{obstacle} + C_{clearance} + C_{via} + C_{geometry}
         + C_{congestion}.
\]

Route demand tends to move down the field gradient:

\[
\vec v = -\nabla H.
\]

Successful, physically realizable flow erodes a shallow valley, while
overflow and physical conflicts deposit cost:

\[
H_{t+1} = H_t
          - \alpha F_{successful}
          + \beta C_{overflow}
          + \gamma C_{physical}
          + \delta C_{via}
          + \epsilon C_{crossing}.
\]

A diffusion pass then spreads local evidence:

\[
H' = H + \lambda \nabla^2 H.
\]

This is especially relevant to Flux's current failure mode: it can find
combinatorially complete routes, yet small geometric variations reproduce the
same clearance, crossing, via, or quantization failure. A diffused physical
penalty says that the surrounding region is difficult, rather than merely
blacklisting one sampled resource.

The field should ideally be anisotropic. Copper has direction and corridor
grain, so a successful horizontal routing channel should encourage compatible
horizontal traces more than perpendicular crossings. This can be represented
as a directional field \(H(x,y,l,\theta)\), or more practically as a local cost
tensor:

\[
\mathbf C(x,y) =
\begin{bmatrix}
c_{xx} & c_{xy} \\
c_{xy} & c_{yy}
\end{bmatrix},
\qquad
C_{terrain}(d) = d^T \mathbf C d.
\]

Flow accumulation can also make the corridor graph demand-aware. Aggregate
terminal demand forms candidate routing valleys or a global routing skeleton;
those valleys inform NavMesh/corridor capacity and PathFinder allocation rather
than replacing obstacle-derived free-space geometry.

The terrain must not create infinitely attractive early highways. Erosion is
therefore capacity-aware and saturating:

\[
E = \alpha F \left(1-\frac{u}{C}\right)^p.
\]

An empty useful corridor receives strong erosion, a partly occupied corridor
receives less, a corridor near capacity receives none, and an overloaded
corridor receives deposition. When a routing basin remains stuck, Flux can
apply temporary regional "uplift" so several affected nets explore a different
corridor family together. This is the routing analogue of river avulsion and a
possible escape from stable PathFinder local equilibria.

Most importantly, the learned terrain should persist across attempts on the
same board. Repeated clearance conflicts, via collisions, crossings, or
quantization failures build hills; corridors that consistently become clean
KiCad copper build shallow valleys. The slow feedback loop is:

```text
route -> physicalize -> measure -> reshape terrain -> route again
```

### Combining terrain with constrained physics

A conventional rigid-body engine such as Bullet is not the right primary
abstraction. PCB traces are deformable constrained curves, closer to elastic
rods than rigid bodies or ideal strings. Position-Based Dynamics (PBD), and
especially Extended Position-Based Dynamics (XPBD), is attractive because it
solves directly for a stable constraint-satisfying equilibrium.

The division of responsibilities is:

```text
PathFinder / global routing
    chooses topology, layers, corridors, and resource allocation

XPBD regional relaxation
    moves control points, traces, and optionally vias into legal geometry

Erosion/history field
    learns the global landscape from constraint stress and DRC results
```

Represent a trace using control points \(p_0,p_1,\ldots,p_n\), with pad
endpoints fixed. Useful energies or their equivalent positional constraints
include:

Length or tension:

\[
E_L = k_L \sum_i (|p_{i+1}-p_i|-L_i)^2.
\]

Bending and smoothness:

\[
E_B = k_B \sum_i |p_{i-1}-2p_i+p_{i+1}|^2.
\]

Obstacle and trace clearance:

\[
E_C(d) =
\begin{cases}
k_c(c-d)^2, & d<c,\\
0, & d\ge c.
\end{cases}
\]

The terrain becomes an external potential:

\[
E_T = k_T H(p), \qquad F_T=-\nabla H.
\]

The global planner therefore chooses an approximate valley while the physics
solver determines the precise curve that settles within it.

XPBD is preferred over Newtonian simulation because meaningful mass, velocity,
momentum, and physical time are unnecessary. The solver repeatedly projects
endpoint, clearance, curvature, corridor, and via constraints. XPBD compliance
also makes stiffness more predictable as iteration count changes. PCB traces
should have strong stretch resistance, moderate bending preference, no
self-intersection, hard endpoint and clearance constraints, and optional
directional preferences.

The important feature is **joint relaxation**. When several tracks conflict in
a local window, all affected curves move together under obstacle clearance,
track-to-track clearance, bend, endpoint, corridor, and length constraints.
This generalizes sequential push-and-shove and fits Flux's existing regional
joint-routing stage.

Vias can eventually become movable particles with limited XY freedom,
layer-transition constraints, keepout radii, via-to-via repulsion, attached
trace tension, and a preferred region. This allows via placement, escape
direction, corridor packing, and local spacing to be optimized together.

The same constraint framework naturally supports:

- weak bundle attraction and cross-corridor ordering constraints;
- coupled elastic rods for differential-pair spacing and tangent matching;
- bus ordering, spacing, length, and shared-corridor preferences; and
- capacity-derived pressure that pushes routes outward before hard overflow.

KiCad DRC remains the final source of truth. A DRC collision at \((x,y)\) can
produce a local repulsive deposit:

\[
H(x,y) \mathrel{+}= K e^{-r^2/\sigma^2}.
\]

Repeated failures strengthen the field and resolved failures decay. XPBD
constraint multipliers and residuals also expose *why* a region failed: lateral
pressure, clearance pressure, via pressure, or curvature stress. That is much
richer feedback than a binary route failure or simple resource overflow.

The combined architecture operates at three timescales:

| Timescale | Mechanism | Responsibility |
|---|---|---|
| Fast | XPBD relaxation | Local trace and via geometry |
| Medium | PathFinder/regional rerouting | Topology and resource allocation |
| Slow | Erosion/history field | Learned board-wide routing landscape |

Physics must remain inside a bounded deformation envelope. It is good at
deciding that a route should move slightly sideways, but not at discovering
that it should pass around the opposite side of a component or change layers.
If relaxation stalls or constraint pressure exceeds a threshold, Flux reports
the stressed region to PathFinder for a topology change.

### GPU PBD/XPBD implementation references

The most relevant references for the implementation are:

1. **NVIDIA Warp** — a practical GPU simulation framework with an XPBD
   integrator, GPU hash grids, mesh/SDF queries, particle repulsion examples,
   and repeated GPU execution patterns. It is useful for prototyping and GPU
   organization, not necessarily as a Flux production dependency.
   [Warp documentation](https://nvidia.github.io/warp/modules/sim.html) and
   [source](https://github.com/NVIDIA/warp).
2. **Velvet** — an open-source C++17/CUDA XPBD cloth solver with stretch,
   bending, attachment, SDF collision, particle collision, and spatial-hash
   neighbour finding. It is a useful source-level reference for parallel XPBD
   organization. [Velvet source](https://github.com/vitalight/Velvet).
3. **Macklin, Müller, and Chentanez, “XPBD: Position-Based Simulation of
   Compliant Constrained Dynamics” (2016)** — the canonical compliance and
   multiplier formulation. [XPBD paper](https://doi.org/10.1145/2994258.2994272).
4. **Interactive Computer Graphics PositionBasedDynamics** — useful for
   distance, bend/twist, stretch/shear, Cosserat rod, point-edge, edge-edge,
   SDF, and XPBD constraint formulations. It is primarily a mathematics and
   constraint reference. [PositionBasedDynamics](https://github.com/InteractiveComputerGraphics/PositionBasedDynamics).
5. **NVIDIA FleX publications** — historically useful for the unified GPU
   particle model, although the core implementation is closed source. Velvet
   provides a more directly inspectable implementation reference.

The general XPBD update is:

\[
\Delta\lambda =
\frac{-C(x)-\tilde{\alpha}\lambda}
{\nabla C M^{-1}\nabla C^T+\tilde{\alpha}},
\]

followed by positional corrections derived from \(\Delta\lambda\). For Flux,
pad attachment and clearances are effectively rigid, length and bend are stiff,
and terrain, corridor, and bundle attraction are soft.

GPU constraint conflicts require deliberate scheduling because multiple
constraints may update the same particle. Suitable strategies are:

- **Jacobi accumulation:** solve constraints independently, accumulate
  corrections, then apply them in a separate pass. This is highly parallel but
  converges more slowly.
- **Graph colouring:** constraints of one colour share no particles, allowing
  parallel Gauss-Seidel-like passes separated by barriers. Trace chains make
  stretch and bend constraints particularly easy to colour.
- **Hybrid solving:** use coloured XPBD for static stretch/bend constraints,
  fully parallel evaluation for fixed geometry and terrain, and
  Jacobi/atomics for dynamic trace collisions.

GPU spatial hashing avoids \(O(N^2)\) segment testing. Segments are assigned to
cells, sorted or binned by cell ID, indexed with a cell-offset table, and tested
only against nearby cells. Flux's problem is simpler than most general XPBD
engines because copper is two-dimensional per layer, with occasional vias
coupling layers.

### Mapping the solver onto Flux's Vulkan compute module

Flux should retain its Vulkan backend rather than adding CUDA or Warp as a
production dependency. Vulkan compute already provides storage buffers,
compute shaders, atomics, scan/sort infrastructure, image-backed signed-distance
fields, indirect dispatch, synchronization, and reusable command buffers.

A practical GPU pass layout is:

```text
1. Build or update trace particles
2. Spatially bin segments
3. Generate broadphase candidate pairs
4. Evaluate obstacle/SDF constraints
5. Evaluate trace-to-trace clearance constraints
6. Project stretch constraints
7. Project bend/curvature constraints
8. Project via constraints
9. Sample terrain/pressure potentials
10. Repeat XPBD projection passes
11. Reduce residuals and spatial stress
```

Structural trace constraints should use pre-coloured passes where practical.
Dynamic collision constraints should use parallel correction accumulation or
atomics followed by an apply pass. The repeated solver sequence can be recorded
in reusable Vulkan command buffers, avoiding CPU round trips between XPBD
iterations.

Static obstacles are well suited to a per-layer signed-distance texture
\(D_l(x,y)\). A shader samples distance and gradient; if clearance is violated,
it projects the particle outward. The same resident texture/image approach can
hold:

- hard-geometry SDFs;
- learned routing terrain;
- current congestion pressure; and
- persistent congestion history.

Trace-to-trace interaction should retain exact segment geometry. A Vulkan
broadphase assigns segments to cells, builds cell offsets, queries neighbouring
cells, and performs analytic 2D segment-distance tests on candidate pairs.

Full geometry need not return to the CPU during each iteration. The GPU should
return compact decision data such as maximum and total violation, worst regions,
per-corridor and per-net stress, and unresolved collision counts. Constraint
stress—particularly a value proportional to \(|\Delta\lambda|\)—should be a
first-class output accumulated into a spatial pressure image and diffused by a
subsequent compute pass.

This closes the intended GPU-resident loop:

```text
PathFinder -> Vulkan XPBD -> stress field -> erosion/diffusion -> PathFinder
```

The desired result is therefore a small, PCB-specific Vulkan solver for 2D
elastic traces, vias, SDF obstacles, exact segment clearance, and soft potential
fields—not a port of a generic physics engine.

---

## Objective

Implement and evaluate a single experimental routing iteration that
combines Flux's existing global routing architecture with a
Vulkan-compute XPBD-style local geometry solver and a persistent
stress/terrain feedback field.

The purpose of this iteration is **not** to replace PathFinder or
redesign the complete router. It is to test whether physics-based joint
relaxation can convert globally valid routes into mutually legal copper
more effectively, while producing a useful physical-congestion signal
for subsequent routing decisions.

### Success hypothesis

Flux currently separates:

1.  global topological/resource planning,
2.  continuous corridor/B-spline geometry,
3.  quantized KiCad copper.

The hypothesis for this iteration is:

> PathFinder should continue deciding topology and resource allocation,
> while a GPU XPBD solver jointly relaxes local trace geometry subject
> to PCB constraints. Constraint stress is then fed back into a spatial
> terrain/history field so subsequent routing decisions avoid physically
> difficult regions.

The target feedback loop is:

``` text
PathFinder / corridor allocation
            |
            v
Initial continuous routes
            |
            v
Vulkan XPBD relaxation
            |
            +----> legal geometry ----> accept
            |
            +----> residual stress/conflicts
                         |
                         v
                 stress/terrain field
                         |
                         v
              regional/global reroute
```

This iteration should be evaluated against the existing tiered routing
tests, with particular attention to boards where Flux can find
connectivity but fails to produce mutually legal copper.

------------------------------------------------------------------------

## 1. Scope

### In scope

Implement a minimal but useful GPU relaxation system supporting:

-   fixed pad/terminal endpoints,
-   movable trace control points,
-   trace length/stretch constraints,
-   bend/smoothness constraints,
-   static obstacle clearance,
-   trace-to-trace clearance,
-   fixed or optionally movable vias,
-   corridor containment or attraction,
-   terrain/stress-field sampling,
-   XPBD constraint residual/stress reporting,
-   GPU spatial broadphase for trace interactions,
-   local/regional invocation from the existing routing pipeline,
-   feedback of unresolved physical stress into routing cost.

### Explicitly out of scope

Do not attempt in this iteration to:

-   replace PathFinder,
-   replace the NavMesh/corridor representation,
-   build a general-purpose physics engine,
-   simulate real mass, velocity, momentum, or time,
-   solve topology changes purely through physics,
-   implement every PCB rule,
-   optimize differential pairs or buses beyond what falls naturally out
    of the generic constraints,
-   move the complete routing pipeline onto the GPU,
-   replace KiCad DRC as final ground truth.

The solver is an **equilibrium constraint optimizer**, not a
physical-time simulator.

------------------------------------------------------------------------

## 2. Architectural Integration

Add the solver primarily behind Flux's existing regional joint-routing /
continuous-geometry stage.

``` text
KiCad board + rules
        |
        v
Canonical board
        |
        v
Clearance-expanded free space
        |
        v
NavMesh / corridors
        |
        v
Terminal/via candidates
        |
        v
Global allocation / PathFinder
        |
        v
Initial continuous route geometry
        |
        v
+----------------------------------+
| Vulkan XPBD regional relaxation  |
|                                  |
| endpoint constraints             |
| obstacle clearance               |
| trace clearance                  |
| stretch                          |
| bend/curvature                   |
| corridor attraction              |
| terrain potential                |
+----------------+-----------------+
                 |
          +------+------+
          |             |
       converged      stressed
          |             |
          v             v
    quantization    stress extraction
          |             |
          v             v
      KiCad DRC     terrain/history
          |             |
          +------> reroute decision
```

Physics is allowed to change geometry inside the selected
topological/corridor solution.

Physics is **not** allowed to silently make major topological decisions
such as choosing the opposite side of a component or changing layers
arbitrarily. Failure to relax is instead converted into information for
PathFinder/regional rerouting.

------------------------------------------------------------------------

## 3. GPU Data Model

Keep the implementation native to the existing Vulkan compute module.

Prefer structure-of-arrays GPU storage for frequently accessed data.

Conceptually the solver requires:

``` cpp
positions[N]          // vec2
previous[N]           // vec2, if required
inverse_mass[N]       // float
particle_flags[N]     // endpoint, via, movable, etc.

segment_p0[M]         // uint
segment_p1[M]         // uint
segment_net[M]        // uint
segment_layer[M]      // uint
segment_width[M]      // float

constraint_type[C]
constraint_indices[C]
constraint_target[C]
constraint_compliance[C]
constraint_lambda[C]
```

The exact representation should fit the conventions of the existing
Vulkan compute code rather than introducing a separate abstraction
unnecessarily.

### Particle representation

Represent each continuous trace as a chain of control particles:

``` text
Pad                                      Pad
 O--o--o--o--o--o--o--o--o--o--o--o--O
```

Endpoints attached to pads are immovable.

Intermediate points are movable.

Vias can initially be fixed. If the basic solver works, enable limited
XY freedom for vias as a stretch goal within this same iteration.

------------------------------------------------------------------------

## 4. Constraint Model

Use XPBD-style positional constraints rather than force integration.

A generic XPBD update can follow:

\[ `\Delta`{=tex}`\lambda `{=tex}=
`\frac{-C(x)-\tilde{\alpha}\lambda}`{=tex} {`\nabla `{=tex}C
M^{-1}`\nabla `{=tex}C^T+`\tilde{\alpha}`{=tex}} \]

with the corresponding positional correction applied to participating
particles.

Constraint compliance should distinguish hard PCB legality from soft
optimization preferences.

### Hard or near-hard constraints

-   pad attachment,
-   board/keepout clearance,
-   copper-to-copper clearance,
-   via clearance,
-   corridor hard boundaries where applicable.

### Stiff constraints

-   excessive trace stretching,
-   minimum bend/curvature behaviour,
-   local geometry preservation where useful.

### Soft constraints

-   corridor-centre attraction,
-   routing terrain attraction,
-   bundle/lane preference,
-   route-shortening preference.

The important design rule is:

> Soft objectives must never overpower electrical/physical legality
> constraints.

------------------------------------------------------------------------

## 5. Static Obstacle Clearance

Use the existing geometry where practical, but evaluate whether a
per-layer GPU signed-distance representation provides a simpler and
faster relaxation constraint.

Conceptually:

\[ D_l(x,y) \]

returns distance to forbidden geometry on layer (l).

The local surface normal can be approximated from:

\[ n = `\frac{\nabla D}{|\nabla D|}`{=tex} \]

A particle or trace sample violating required clearance is projected
away from the obstacle.

Potential GPU resources:

``` text
layer SDF             hard static geometry
terrain field         learned routing preference
pressure field        current physical congestion
history field         persistent routing difficulty
```

Do not depend exclusively on sampled SDF values for final correctness.
Exact geometry/KiCad DRC remains authoritative.

------------------------------------------------------------------------

## 6. Trace-to-Trace Collision and Clearance

Avoid all-pairs segment testing.

Implement or reuse a GPU broadphase based on spatial cells/hash bins.

Suggested pipeline:

``` text
segments
   |
   v
calculate occupied cell IDs
   |
   v
sort/group by cell
   |
   v
construct cell ranges
   |
   v
query neighboring cells
   |
   v
candidate segment pairs
   |
   v
exact 2D segment clearance
```

Reuse existing Vulkan sort/scan infrastructure if Flux already has
suitable primitives.

Only compare segments where:

-   layers overlap electrically/physically,
-   nets differ where required,
-   spatial cells overlap or neighbor one another,
-   the pair is not otherwise exempt by PCB rules.

For candidate pairs, use exact 2D segment distance where possible rather
than approximating the entire problem as particles.

------------------------------------------------------------------------

## 7. GPU Solver Strategy

Use a hybrid strategy rather than forcing every constraint into one
synchronization model.

### Structural constraints

Stretch and bend constraints along a trace have predictable topology and
can be pre-coloured.

For example:

``` text
stretch:

R  B  R  B  R
o--o--o--o--o--o
```

Execute:

``` text
dispatch colour 0
barrier
dispatch colour 1
barrier
```

This preserves much of the convergence behaviour of Gauss-Seidel while
exposing substantial GPU parallelism.

### Dynamic collision constraints

Trace-to-trace constraints are generated dynamically and are less
convenient to colour.

Use a Jacobi-style approach:

``` text
parallel collision solve
          |
          v
per-particle correction accumulation
          |
          v
apply accumulated corrections
```

Use atomics or another suitable reduction strategy already supported
efficiently by the Vulkan backend.

### Iteration sequence

A first implementation can use:

``` text
for each XPBD iteration:

    structural stretch constraints
    barrier

    bend/curvature constraints
    barrier

    obstacle constraints
    barrier

    trace-trace clearance constraints
    barrier

    apply accumulated collision corrections
    barrier

    corridor/terrain soft constraints
    barrier
```

Keep the entire iteration loop GPU-resident where practical.

Avoid CPU readback between solver iterations.

------------------------------------------------------------------------

## 8. Terrain and Erosion Feedback

Introduce a spatial field representing learned physical routing
difficulty.

This does not need to implement a literal hydraulic erosion simulator.

The useful concepts to borrow are:

-   deposition,
-   erosion,
-   diffusion,
-   flow accumulation,
-   saturation,
-   persistent terrain memory.

Define a routing terrain approximately as:

\[ H = H\_{`\text{base}`{=tex}} + H\_{`\text{history}`{=tex}} +
H\_{`\text{physical stress}`{=tex}} -
H\_{`\text{successful corridor}`{=tex}} \]

Regions repeatedly producing conflicts become hills.

Regions repeatedly accepting clean copper can become shallow valleys.

### Stress deposition

Use XPBD constraint activity as a physical-congestion measurement.

For a constraint, use a quantity based on:

\[ \|`\Delta`{=tex}`\lambda`{=tex}\| \]

or another normalized residual/stress metric.

Accumulate this spatially:

``` text
XPBD constraints
      |
      v
stress samples
      |
      v
GPU pressure image
      |
      v
diffusion
      |
      v
terrain/history update
```

The exact physical interpretation of lambda is less important than
whether the resulting normalized signal correlates with routing failure.

### Diffusion

Diffuse deposited pressure into neighboring free space rather than
penalizing only the exact collision coordinate.

Conceptually:

\[ H' = H + `\beta `{=tex}`\nabla`{=tex}\^2 H \]

This should discourage Flux from repeatedly discovering geometrically
slightly different versions of the same physical conflict.

### Saturation

Do not allow successful routes to create infinitely attractive
corridors.

Successful-corridor preference should weaken as utilization approaches
estimated capacity.

Overloaded regions should switch from erosion/attraction to
deposition/repulsion.

------------------------------------------------------------------------

## 9. Solver Failure as a Routing Signal

A major goal of this iteration is to improve the information passed back
to the global router.

Do not reduce solver failure to a boolean.

Produce metrics such as:

``` text
region:
    maximum clearance residual
    total clearance residual
    trace-trace stress
    obstacle stress
    bend stress
    via stress
    number of unresolved pairs

net:
    accumulated stress
    worst stress location

corridor:
    accumulated physical pressure
    peak pressure
```

This allows the routing system to distinguish:

``` text
topologically possible and physically comfortable

topologically possible but physically stressed

locally impossible under current allocation
```

PathFinder or regional rerouting can then penalize the actual source of
physical difficulty.

------------------------------------------------------------------------

## 10. Local/Regional Invocation

Do not initially relax the entire PCB after every routing action.

Use the existing physical conflict graph/local-window machinery to
select regions containing interacting routes.

A window should include:

-   conflicting traces,
-   traces immediately constraining them,
-   nearby movable vias,
-   static obstacle geometry,
-   sufficient surrounding corridor space to permit movement.

Then jointly relax all affected traces.

Example:

``` text
         regional window
+--------------------------------+
|                                |
| A --------------------\        |
|                       \        |
| B ---------------------X----   |
|                       /        |
| C --------------------/        |
|                                |
+--------------------------------+
```

All affected traces should be allowed to move together rather than
sequentially shoving one route at a time.

This is one of the main behaviours the experiment is intended to test.

------------------------------------------------------------------------

## 11. Topology Escape

Physics should have a bounded deformation envelope.

If the solver cannot remove violations without leaving the assigned
corridor/topological solution:

``` text
XPBD iterations
      |
      v
residual below threshold?
     / \
   yes  no
   |     |
accept   identify stressed resources
         |
         v
deposit terrain/history cost
         |
         v
regional PathFinder reroute
```

Do not increase XPBD iterations indefinitely.

A persistent high residual is useful information: it indicates that
geometry optimization is no longer the correct level at which to solve
the problem.

------------------------------------------------------------------------

## 12. KiCad DRC Feedback

Continue to use KiCad DRC as ground truth.

After relaxed routes are converted into legal KiCad primitives:

1.  run DRC,
2.  associate violations with Flux geometry/resources,
3.  classify the violation,
4.  deposit additional physical pressure around relevant locations,
5.  feed this into the next routing attempt.

A useful first feedback kernel is a local Gaussian-like penalty:

\[ H(x,y) `\mathrel{+}`{=tex}= K e^{-r^2/`\sigma`{=tex}\^2} \]

The precise kernel is less important initially than proving that
repeated DRC failures cause subsequent routes to migrate away from
problematic regions.

------------------------------------------------------------------------

## 13. Implementation Stages

### Stage A --- Solver skeleton

Implement:

-   GPU particle/control-point buffers,
-   fixed endpoint constraints,
-   stretch constraints,
-   bend/smoothing constraints,
-   solver iteration dispatch,
-   residual collection.

Test using synthetic traces with no obstacles.

Acceptance:

-   endpoints remain fixed,
-   disturbed traces converge to smooth stable geometry,
-   repeated execution is deterministic enough for router testing,
-   no CPU synchronization is required within an iteration sequence.

### Stage B --- Static geometry

Add:

-   board obstacles,
-   clearance-expanded geometry or SDF representation,
-   obstacle projection.

Synthetic tests should include:

-   trace beside rectangular obstacle,
-   trace through obstacle,
-   narrow passage,
-   concave obstacle,
-   pad escape.

Acceptance:

-   traces consistently move out of illegal static geometry when a
    solution exists.

### Stage C --- Trace-to-trace clearance

Add:

-   spatial broadphase,
-   candidate pair generation,
-   exact 2D segment clearance,
-   Jacobi correction accumulation.

Tests:

``` text
two crossing traces
parallel traces too close
three traces competing for a neck
multiple traces requiring joint displacement
```

Acceptance:

-   solver can separate locally feasible configurations without topology
    changes.

### Stage D --- Regional Flux integration

Feed an existing Flux regional conflict window into the solver.

Translate current B-spline/polyline geometry into solver control points
and return relaxed continuous geometry.

Acceptance:

-   at least one existing physical conflict currently requiring rip-up
    can be removed through joint relaxation.

### Stage E --- Stress field

Accumulate:

-   constraint residual,
-   delta-lambda-derived stress,
-   unresolved collision pressure.

Rasterize/accumulate into a GPU pressure field.

Add a diffusion compute pass.

Acceptance:

-   visually/numerically confirm that difficult routing necks produce
    concentrated pressure,
-   nearby cells receive lower diffused pressure,
-   uncongested regions remain low pressure.

### Stage F --- Routing feedback

Expose the pressure/history field to the existing route-cost machinery.

Initially keep the coupling weak.

Measure whether subsequent PathFinder/regional-routing attempts avoid
repeatedly stressed geometry.

------------------------------------------------------------------------

## 14. Benchmark Plan

Use the existing tiered Flux boards.

Do not optimize against only one board.

Record the existing baseline before enabling XPBD.

For every board capture:

  Metric                        Baseline   XPBD   XPBD + feedback
  --------------------------- ---------- ------ -----------------
  Global unrouted nets
  PathFinder overflows
  Clearance violations
  Hole-clearance violations
  Shorts
  Track crossings
  Via count
  Track length
  Rip-up/reroute operations
  Regional solver calls
  XPBD iterations
  Peak XPBD residual
  Routing runtime
  GPU time
  Final DRC-clean

The key metric is not simply whether XPBD makes geometry prettier.

The experiment succeeds if it materially improves:

\[ `\text{globally connected}`{=tex}
`\quad`{=tex}`\rightarrow`{=tex}`\quad`{=tex}
`\text{mutually legal copper}`{=tex} \]

------------------------------------------------------------------------

## 15. Instrumentation

Add enough instrumentation to understand why the experiment succeeds or
fails.

Per solver invocation record:

``` text
region ID
affected nets
particle count
segment count
candidate collision pairs
actual constraints
iterations executed

initial:
    violations
    maximum residual
    total residual

final:
    violations
    maximum residual
    total residual

timing:
    broadphase
    constraint generation
    XPBD solve
    stress reduction
    readback

result:
    converged
    stalled
    topology_escape_required
```

Also record per-iteration residual reduction for selected diagnostic
runs.

This should answer whether failures arise from:

-   bad topology,
-   insufficient free space,
-   solver convergence,
-   inadequate collision detection,
-   poor constraint weighting/compliance,
-   bad control-point discretization.

------------------------------------------------------------------------

## 16. Performance Rules

The first implementation should favor correctness and observability over
premature shader optimization, but maintain several architectural rules
from the start:

1.  Keep solver state resident on the GPU.
2.  Avoid CPU readback inside XPBD iterations.
3.  Batch regional work where practical.
4.  Use spatial broadphase rather than all-pairs collision.
5.  Reuse Vulkan buffers rather than reallocating each iteration.
6.  Reuse command structures where practical.
7.  Keep static board fields resident across routing attempts.
8.  Return compact residual/stress summaries to the CPU rather than
    complete intermediate state.
9.  Preserve a CPU/debug path or diagnostic dump sufficient to reproduce
    solver failures.
10. Ensure the feature can be disabled so benchmark comparisons remain
    straightforward.

------------------------------------------------------------------------

## 17. Initial Constraint Tuning

Start conservatively.

Priority should be:

``` text
1. electrical/physical legality
2. endpoint correctness
3. topology/corridor preservation
4. reasonable curvature
5. short routes
6. aesthetically attractive routing
```

Do not tune for visually smooth routes until clearance behaviour is
robust.

Start with hard/near-hard clearance constraints and comparatively weak
terrain attraction.

Terrain/history feedback should initially have a low gain so that a
noisy stress measurement cannot destabilize PathFinder.

Increase feedback strength only after demonstrating that high-stress
regions correlate with actual DRC/routing failures.

------------------------------------------------------------------------

## 18. Experimental Questions

This iteration should explicitly answer the following questions.

### Q1 --- Can XPBD resolve conflicts that the current continuous geometry stage cannot?

If no, determine whether the problem is:

-   topology,
-   control-point resolution,
-   collision formulation,
-   solver convergence,
-   insufficient movable neighborhood.

### Q2 --- Does joint movement outperform sequential local repair?

Compare cases where several routes share the same constrained region.

### Q3 --- Does XPBD stress predict real physical routing difficulty?

Compare solver stress against:

-   KiCad DRC violations,
-   PathFinder overflow,
-   repeated rip-up regions,
-   eventual routing failure.

### Q4 --- Does spatial diffusion improve feedback?

Compare:

``` text
exact-resource penalty
vs.
spatially diffused physical-pressure penalty
```

### Q5 --- Does feedback reduce repeated failures?

Measure whether the same physical locations repeatedly generate
conflicts across rerouting iterations.

### Q6 --- Is GPU execution cheap enough to invoke frequently?

Measure solver time independently from the rest of Flux.

------------------------------------------------------------------------

## 19. Success Criteria for This Iteration

The experiment should be considered successful if all of the following
are demonstrated:

1.  Vulkan XPBD relaxation operates entirely within the existing GPU
    compute architecture.
2.  Multiple interacting traces can be jointly displaced while
    respecting fixed endpoints.
3.  Static and trace-to-trace clearance constraints converge reliably on
    synthetic cases.
4.  At least one real Flux routing conflict is resolved without changing
    global topology.
5.  Solver residual/stress identifies physically difficult regions.
6.  Stress can be converted into a spatial pressure/terrain field.
7.  Feeding that field back into routing changes subsequent route
    selection in the expected direction.
8.  Tier benchmark results show either:
    -   fewer physical DRC violations, or
    -   fewer repeated local conflicts/reroutes, without materially
        degrading easier tiers.

A full Tier 1 pass is desirable but is **not required to validate this
iteration**. The purpose is to establish whether the mechanism provides
useful geometry repair and feedback.

------------------------------------------------------------------------

## 20. Failure Criteria

Do not continue expanding the physics system if testing shows that:

-   most failures require topology changes rather than local geometry
    movement,
-   XPBD stress does not correlate with actual routing difficulty,
-   the solver frequently converges to geometry that fails exact DRC,
-   local relaxation simply moves conflicts to neighboring regions,
-   terrain feedback destabilizes otherwise successful PathFinder
    allocation,
-   GPU synchronization/broadphase costs overwhelm any reduction in
    rerouting work.

If these occur, retain any useful stress/congestion measurement work and
remove the physics solver from the critical routing path.

------------------------------------------------------------------------

## 21. Recommended Deliverable

The output of this development iteration should be an experimental Flux
feature behind a configuration flag, for example:

``` text
routing.physical_relaxation.enabled
routing.physical_relaxation.iterations
routing.physical_relaxation.clearance_compliance
routing.physical_relaxation.bend_compliance

routing.physical_feedback.enabled
routing.physical_feedback.gain
routing.physical_feedback.diffusion
routing.physical_feedback.decay
```

Alongside the implementation, produce a benchmark report comparing:

``` text
baseline
vs.
XPBD relaxation only
vs.
XPBD relaxation + terrain feedback
```

The decision at the end of the iteration should be evidence-based:

> **Does GPU joint physical relaxation plus learned spatial stress make
> Flux materially better at turning valid global paths into legal PCB
> copper?**

If yes, the next iteration can expand the model to movable vias,
anisotropic terrain, bundle ordering, differential-pair constraints,
capacity-aware erosion, and more aggressive regional topology escape.

------------------------------------------------------------------------

## Reference Implementations and Literature

Useful implementation and mathematical references for the development
work:

-   **Macklin, Müller & Chentanez --- XPBD: Position-Based Simulation of
    Compliant Constrained Dynamics**\
    Canonical XPBD constraint formulation and compliance model.\
    https://doi.org/10.1145/2994258.2994272

-   **NVIDIA Warp**\
    GPU simulation framework with XPBD integration, spatial hashing,
    particle interaction, SDF-style geometry queries, and GPU-oriented
    solver examples.\
    https://github.com/NVIDIA/warp

-   **Velvet**\
    Open-source C++17/CUDA XPBD cloth solver useful as a reference for
    parallel constraint solving, spatial hashing, collision handling,
    stretch and bend constraints.\
    https://github.com/vitalight/Velvet

-   **InteractiveComputerGraphics/PositionBasedDynamics**\
    Reference implementation for PBD/XPBD constraints, including
    distance, collision and elastic-rod/Cosserat-rod formulations useful
    for trace modelling.\
    https://github.com/InteractiveComputerGraphics/PositionBasedDynamics

-   **NVIDIA FleX / PBD literature**\
    Useful architectural reference for large-scale GPU unified
    particle/constraint simulation. Use primarily for solver
    architecture concepts rather than as a Flux dependency.

These references should be treated as sources for algorithms and
implementation patterns. The Flux implementation should remain a small,
PCB-specific Vulkan compute solver rather than importing a
general-purpose physics framework.
