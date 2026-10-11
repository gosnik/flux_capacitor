# GPU NavMesh Preparation and Adaptive Subdivision Plan

## Summary

Flux's NavMesh can be accelerated further with Vulkan compute, but it should not move the entire
NavMesh pipeline or any authoritative geometric decision to the GPU.

Flux already accelerates two important NavMesh operations:

- Candidate-cell clearance broad phase. Each rectangular candidate is represented by its four
  sides and two diagonals, and the resulting chords are passed to the conservative GPU chord
  classifier.
- NavMesh search. Large regular lattices use the GPU raster wavefront, while sufficiently large
  adaptive meshes use the GPU graph wavefront.

The remaining CPU work includes candidate-queue processing, adaptive child generation, exact
point/polygon and chord predicates, retained-cell compaction, and construction of mixed-resolution
shared-face adjacency.

The recommended experiment is therefore a resident, breadth-first GPU subdivision pipeline with
CPU exact verification. The first optimisation should be eliminating repeated boundary packing,
upload, dispatch, and readback—not merely moving the inexpensive rectangle-splitting arithmetic
onto the GPU.

## Current Pipeline

NavMesh preparation currently performs the following work:

1. Prepare the exact free-space outline and cutout polygons.
2. Build a CPU cutout BVH.
3. Enumerate the coarse rectilinear lattice.
4. Process candidates in batches of 2,048 cells.
5. Generate six certification chords per cell.
6. Run conservative GPU chord classification when the workload is large enough.
7. Run exact CPU geometry for ambiguous cases.
8. Retain legal cells, reject terminal leaves, or generate four children.
9. Build mixed-resolution adjacency with coordinate-keyed CPU maps and sorted face lists.
10. Search the resulting lattice or graph, using GPU wavefronts above existing thresholds.
11. Reconstruct portals and run the funnel and topology validation on the CPU.

The broad-phase result has three states:

- `definitely_clear`: may bypass a complete CPU boundary scan
- `definitely_blocked`: not authoritative by itself
- `ambiguous`: requires exact CPU geometry

Only conservative clearance can bypass exact work. GPU floating-point geometry must never become
the authority for free space, connectivity, topology, or legal copper.

## Likely Bottlenecks

Before implementation, measure these costs independently:

- polygon preparation and free-space validation
- cutout BVH construction
- coarse lattice enumeration
- topology-guide filtering
- boundary-segment packing
- boundary upload
- chord generation
- GPU classification and synchronization
- exact chord fallback
- exact point/polygon fallback
- child generation and queue management
- retained-cell compaction
- face-key construction and sorting
- adjacency joining
- raster/graph wavefront search
- funnel reconstruction
- topology-signature validation

Also record:

- candidates per refinement level
- accepted, rejected, split, and uncertain cells
- fraction of chords requiring exact fallback
- number and size of GPU dispatches
- bytes uploaded and downloaded
- refined parent and leaf counts
- maximum realized depth
- final cell and adjacency counts
- CPU and GPU wall time per prepared domain

Without this profile, full GPU subdivision may optimise work that is already insignificant. The
default maximum refinement depth is only two, and the split operation itself consists mostly of a
few integer calculations.

## Proposed GPU Pipeline

### 1. Persistent prepared boundary data

Create a GPU representation owned by or cached for a `PreparedNavMeshDomain`:

- outline boundary segments
- cutout boundary segments
- conservative segment bounds or spatial bins
- domain-relative coordinate origin and scale

Upload this data once and reuse it across all refinement levels and route queries that share the
prepared domain. Preserve the process-wide Vulkan runtime and existing CPU fallback behaviour.

### 2. Breadth-first refinement

Replace the serial deque as an optional acceleration path with two resident candidate buffers:

1. Generate all level-zero candidates.
2. Classify the active level in parallel.
3. Assign each candidate one of four provisional outcomes:
   - retain
   - reject
   - split
   - uncertain
4. Use prefix sums and stream compaction to allocate deterministic output positions.
5. Append retained leaves to the leaf buffer.
6. Emit four children for every split candidate into the next-level buffer.
7. Return uncertain candidates for exact CPU classification, or classify them on the CPU between
   bounded level dispatches.
8. Repeat until the configured maximum depth, minimum pitch, or cell budget is reached.

Prefix-sum allocation avoids unordered atomic append results and allows stable, reproducible cell
ordering. A deterministic order should be defined using the parent order followed by a fixed child
quadrant order.

### 3. Conservative geometry contract

GPU classification must remain advisory. It may prove that a candidate is sufficiently far from
all boundaries, but it must not prove ambiguous boundary contact safe.

Use one of these coordinate strategies:

- exact integer nanometre arithmetic where Vulkan `shaderInt64` is supported, or
- domain-relative fixed-point/integer coordinates whose range and error bound are proven before
  dispatch.

Floating-point calculations require an explicit numerical guard. Any value within the guard band,
any possible intersection, overflow, unsupported coordinate range, or non-finite result must be
classified as uncertain and sent to the CPU.

The CPU remains authoritative for:

- point-in-polygon classification
- boundary-touch semantics
- segment/polygon intersection
- ambiguous clearance
- shared-face and portal validity
- topology signatures
- funnel output certification
- continuous geometry
- quantised copper legality
- KiCad DRC

### 4. Mixed-resolution adjacency

After subdivision is validated, investigate adjacency as a separate experiment.

Generate the four faces of every retained leaf with keys containing:

- axis
- fixed coordinate
- side/orientation
- interval start and end
- stable cell identifier

Sort or radix-sort faces by key, then match opposite-facing interval overlaps in parallel. A Morton
code or equivalent hierarchical leaf key can provide deterministic spatial ordering and simplify
parent/child relationships.

GPU output should be treated as adjacency candidates. CPU code should validate exact positive
overlap, portal orientation, aperture, duplicate removal, and final deterministic edge order before
the graph becomes authoritative.

Do not combine adjacency acceleration with the initial subdivision experiment. Each change should
have separately measurable behaviour and an independent CPU comparison.

### 5. Keep downstream serial work on the CPU

The funnel, directed-gate reconstruction, and topology-signature validation operate on relatively
small ordered corridors and contain sequential dependencies. They are unlikely to repay dispatch
and synchronization overhead. Keep these stages on the CPU unless profiling later demonstrates a
large batchable workload.

## Smallest Useful Experiment

### Hypothesis

NavMesh preparation time on large domains is dominated by repeated boundary preparation/transfer,
candidate classification, exact fallback, or mixed-resolution adjacency—not by the arithmetic that
creates four child rectangles. Keeping boundary data resident and classifying a complete refinement
level per dispatch will reduce preparation time without changing cells, topology, or legal copper.

### Change

1. Add the per-stage timings and counters listed above.
2. Add an optional resident GPU boundary-classification context.
3. Batch candidates by refinement level instead of arbitrary deque chunks for the GPU path.
4. Reuse uploaded boundaries across every level.
5. Preserve the existing exact CPU fallback and existing CPU-only implementation as the oracle.
6. Add a configuration or test hook that runs the same domain through both paths.

Do not implement GPU adjacency in this first experiment.

### Correctness criteria

For every test domain, CPU and GPU-assisted preparation must produce identical:

- accepted leaf rectangles
- cell refinement levels
- deterministic cell identifiers
- rejected and refined counts
- cell-budget status
- shared-face portals and apertures
- adjacency order
- selected route connectivity
- topology signatures
- final quantised copper and DRC results

If provisional GPU ordering differs internally, canonicalise before public identifiers or adjacency
are assigned.

### Performance criteria

Retain the experiment only if it shows a repeatable improvement on representative large NavMeshes
without a meaningful regression on small regional meshes. Record both NavMesh-stage and end-to-end
router time; an isolated kernel speedup is insufficient if transfer or synchronization cost removes
the gain.

Candidate promotion targets, to be validated rather than assumed:

- at least 20% lower NavMesh preparation time on large frontier-board domains, or
- at least 10% lower end-to-end time where NavMesh preparation is material
- no material slowdown on lower-tier boards
- byte-identical authoritative results
- no increase in CPU fallback caused by a weaker numerical guard

Choose the GPU threshold from a multi-board sweep rather than tuning it for one benchmark.

## Follow-up Experiments

If resident level batching succeeds:

1. Add GPU prefix-sum/compaction and resident child emission.
2. Compare GPU-generated leaves against the CPU oracle over randomized synthetic domains.
3. Add keyed GPU face generation and sorting.
4. Add GPU adjacency candidate generation with CPU exact verification.
5. Batch preparation of independent regional NavMeshes when individual meshes are too small to
   saturate the GPU.
6. Investigate asynchronous overlap between GPU classification and unrelated CPU exact work.

Batching independent domains may provide more benefit than accelerating a single small mesh,
especially with the default two-level refinement bound.

## Tests

Add deterministic tests covering:

- no-cutout rectangular free space
- boundary cells split at every permitted level
- a narrow passage recovered only through refinement
- a cutout corner touching the numerical guard band
- mixed coarse/fine shared-face adjacency
- topology-guide filtering at a refined-cell boundary
- exact coordinate-range limit and forced CPU fallback
- maximum-cell budget exhaustion
- empty and fully blocked domains
- repeated runs producing identical cell and portal order
- GPU-disabled, GPU-unavailable, and GPU-error fallbacks

Add randomized differential tests using seeded outlines and cutouts. Compare the GPU-assisted result
against the existing CPU implementation, and minimize any mismatch into a permanent regression
case.

## Benchmark Procedure

For each retained implementation step:

1. Run focused unit and differential tests.
2. Run the first failing Tier-1 board with CPU-only and GPU-assisted preparation.
3. Compare stage timings, candidate counts, uncertainty rate, routes, and DRC categories.
4. Run all of Tier 1.
5. Run Tier 0 as the mandatory regression set.
6. If promising, run every available tier and a bounded threshold/batch-size sweep.
7. Store structured results in the benchmark history and document the experiment in the router
   research log.

## Decision

GPU acceleration is technically appropriate for NavMesh classification, compaction, child
generation, spatial ordering, and large graph traversal. It is inappropriate as the final authority
for exact free-space topology or copper legality.

The immediate priority is instrumentation followed by resident boundary classification. Full GPU
subdivision should be implemented only if profiling shows that CPU queue/compaction work remains
material after transfer and synchronization overhead is removed. GPU adjacency should remain a
separate, later experiment.

## References

- Mark Harris, Shubhabrata Sengupta, and John D. Owens, [Parallel Prefix Sum (Scan) with
  CUDA](https://developer.nvidia.com/gpugems/gpugems3/part-vi-gpu-computing/chapter-39-parallel-prefix-sum-scan-cuda).
  Scan and stream compaction provide the basic deterministic allocation mechanism for retained
  cells and generated children.
- Tero Karras, [Maximizing Parallelism in the Construction of BVHs, Octrees, and k-d
  Trees](https://research.nvidia.com/publication/2012-06_maximizing-parallelism-construction-bvhs-octrees-and-k-d-trees).
  Morton ordering, radix-tree construction, compaction, and prefix allocation are relevant to an
  adaptive rectilinear hierarchy.
- Duane Merrill, Michael Garland, and Andrew Grimshaw, [Scalable GPU Graph
  Traversal](https://research.nvidia.com/publication/2012-02_scalable-gpu-graph-traversal).
  GPU graph traversal benefits from parallel frontier management and prefix-sum-based work
  allocation; Flux already applies this family to sufficiently large NavMesh graphs.
- Khronos Group, [Vulkan indirect dispatch
  command](https://registry.khronos.org/VulkanSC/specs/1.0-extensions/man/html/VkDispatchIndirectCommand.html).
  Indirect dispatch can support GPU-generated next-level workloads without requiring a CPU
  readback between every refinement level.
