# Terrain height evolution for corridor discovery

Status: default global hydraulic discovery backend; comparison backend selected with
`--no-terrain-height-evolution`.

The existing hydraulic family reinforces source-to-target droplets on a fixed destination potential.
This experiment instead uses LittlePlanets-style iterative height evolution. The initial surface is
inverse legal-space clearance, with bounded routing pressure and successful-route priors. Each
iteration recomputes steepest-descent D8 drainage, contributing flow, transport capacity
`A^1.6 * sin(slope)^1.3`, flux divergence, and bounded erosion/deposition. The CPU reference uses
height-sorted topological accumulation and conservative edge flux rather than spherical central
finite differences. Neither material factors nor terrain-specific smoothing are copied blindly.

The legal-space mask is immutable. Diagonal moves cannot cut obstacle corners. After evolution,
positive-cost graph search may climb saddles and leave local basins; drainage sinks do not disconnect
otherwise connected free space. The output remains an advisory corridor. Polygon chord checks,
coarse corridor/topology projection, PH/Bézier materialization, connectivity, and copper DRC remain
unchanged. This replaces static first-pass discovery within the hydraulic candidate family; confined anisotropic
repair keeps its existing directional solver. It does not replace the entire NavMesh
representation or the free-space offsetter.

The production implementation evolves fields on Vulkan. The CPU reference remains explicitly
selectable through the library; a failed GPU request never silently falls back to CPU. Vulkan
startup checks and other compute stages remain required. The user selected height evolution as
the default; this does not by itself establish a speedup or tier promotion. The droplet backend
remains available for comparison. Fields reuse viewer
height/erosion telemetry, with a backend label; dedicated counters distinguish requested proposals, actual solver invocations,
accepted corridors, GPU dispatches, field/extraction time, iteration work, changed cells, and
path-search expansions. Changed-cell counts
are sums over solves, not unique board cells. A corridor acceptance is not a completed board.

Work is bounded by raster size and evolution iterations. Standalone tests exercise height changes,
immutable obstacles, narrow necks, saddle escape, disconnection, deterministic execution, cancellation,
and invalid parameters. Integration checks certify polygon discovery and corridor projection. Baseline
and experiment results are retained under `build/height-evolution-validation/` and summarized in
`reports/benchmark_history/2026-10-09-height-evolution.json`.

Reference implementation inspected: `/home/adam/qtbits/LittlePlanets/terrain/src/compute/terrain_compute.cpp`
and `terrain/shaders/terrain_{transport_capacity,flux_divergence,height_apply}.comp`. This is an
adaptation of the algorithmic structure, not a claim of numerical equivalence with planet generation.

## GPU field evolution

Downhill selection, contributing area, transport capacity, conservative divergence and height
updates run on-device. Integer pointer doubling accumulates contributing area in logarithmically
many passes without CPU sorting or floating-point atomics. Separate compute passes and memory
barriers protect immutable inputs; divergence gathers incoming transport over the original D8
edges. A mutex protects the persistent Vulkan context across concurrent seed requests. Buffers
grow with capacity and are reused, with each solve replacing the mask and initial feedback-derived
height. Cancellation is checked between bounded iteration submissions.

The CPU validates input and extracts the path using the evolved height field. GPU fields use
32-bit floating point; the double-precision CPU implementation remains the numerical reference.
Tests compare field error, exact integer flow on controlled forests, legal masks, conservation,
feedback changes, cancellation and concurrent buffer reuse. Reports distinguish field time from
extraction and total time, including dispatch and transfer overhead. Zero iterations preserve the
original field and do not claim GPU execution. No free-space or copper legality rule is relaxed.

## Live copper feedback between engine actions

Completed planning actions replace a provisional copper snapshot; materialization replaces it with
physical candidate copper. The snapshot is distinct from persistent failure history and from the
canonical board. Foreign-net copper contributes layer-scoped terrain pressure, including segments,
arcs, Béziers and the copper-layer span of vias. Own-net copper is excluded. The next global action
recomputes terrain from the current snapshot, so removed routes do not leave stale occupancy
pressure or persisted height changes. Historical conflict evidence can still remain intentionally.

Snapshot changes conservatively invalidate all incremental global route reuse, ensuring that
no retained proposal silently uses old occupancy. Regional invalidation can later reduce this cost. Exact free-space preparation remains reusable
because advisory pressure does not change the legal mask. Engine attempt telemetry records the
input snapshot revision, feature count and whether it is provisional. Replacement happens at
completed action boundaries, not at cooperative yields or individual intra-pass PathFinder rip-ups.
Such rip-ups are reflected by the next completed routing checkpoint.

Pressure envelopes use the maximum resolved board clearance, including assigned net classes;
provisional widths/vias use resolved net rules. Curves use bounding envelopes. This is a bounded
advisory approximation, not an exact dynamic-copper obstacle representation.
Final legality still requires detailed copper checks and KiCad DRC. The default backend change
was explicitly requested; comparative completion and runtime benefits remain to be measured.
