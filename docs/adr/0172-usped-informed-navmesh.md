# ADR 0172 — Experimental USPED-informed NavMesh refinement

## Status

Implemented, opt-in; no routing-tier promotion claimed.

## Reference and scope

Reviewed `~/qtbits/LittlePlanets/terrain/README.md`,
`src/terrain/slhydrology.cpp` (`SLHydrology::USPED`), and the GPU
`terrain_transport_capacity.comp` and `terrain_flux_divergence.comp` passes.
The reference is MIT licensed (Michael Tuck, 2024). This implementation independently expresses
the algorithm; no reference source is copied. The underlying model is described at
https://grass.osgeo.org/grass80/manuals/addons/r.divergence.html .

Unlike the reference's landscape evolution, Flux never applies a height delta to obstacle geometry.
This is a geometry-only initial prepass, not per-net routing, demand estimation, or copper packing.

## Mapping and computation

- Sample the prepared polygonal domain at coarse NavMesh cell centres, separately for each domain.
- Outside the board, pad/obstacle/keepout cutouts and insufficient-clearance centres are masked
  mountains at height 1. They receive no rain and exchange no flux.
- Legal centres have height `1 / (1 + residual_clearance / cell_pitch)`: boundary slopes descend
  into open-space valleys. Obstacles absent from the input domain cannot be inferred by the field.
- Unit rainfall at free cells follows deterministic steepest D8 descent, without diagonal corner
  cutting. Descending height order accumulates flow. Flats remain explicit sinks; no sink filling
  cuts through an obstacle and no artificial terminal/outlet is invented.
- Compute dimensionless rill transport capacity `A^1.6 * sin(slope)^1.3`, with material factors 1.
- Compute conservative outgoing-minus-incoming capacity on D8 edges. Positive divergence denotes
  erosion, negative divergence deposition. This differs deliberately from LittlePlanets' central
  differences: no flux stencil crosses a masked boundary, and total divergence sums to zero.
- Rank by normalized `log(accumulation)` plus normalized `log1p(abs(divergence))`. Unit rain alone
  has zero priority, so an entirely flat field does not arbitrarily select cells. Accumulation
  matters independently: zero divergence does not imply low flow or low routing demand.

## Integration and bounds

`NavMeshConfig::usped` controls the prepass; global seeding forwards
`GlobalRouterConfig::global_navmesh_usped`. KiCad routing commands accept `--usped-navmesh`.
The default remains disabled for controlled comparison.

Defaults cap the raster at 262,144 cells and select the highest-ranked 20 percent of positive-priority free coarse
cells, capped at 256 parents. Selected already-certified cells receive one extra subdivision,
subject to the existing depth/minimum-pitch bounds. Every child still passes ordinary geometry
certification. Normal clearance-boundary refinement is unchanged; low-flow regions are not removed.
This first version uses the existing coarse raster, not a new fine-resolution terrain allocation.

If the raster budget is exceeded, skip only the prepass. If extra subdivision exhausts the mesh
budget, rebuild the baseline decomposition. Both cases retain explicit diagnostics. Prepared-domain
configuration matching and global checkpoint provenance include the USPED configuration.

Prepared domains expose height, accumulation, capacity, divergence, priority, sink count, and actual
extra refined parents. Global JSON exposes enablement and route-summed USPED parent counts (not a
unique-domain count). Heatmap viewer integration is not included.

CPU cost is O(N log N) for sorting plus O(N) field work, in addition to polygon queries and mesh
certification. This is a deterministic CPU reference, not yet a Vulkan implementation; bitwise
equivalence across different floating-point libraries is not promised.

## Evaluation and limitations

Tests cover mountain exclusion, separated basins, diagonal masks, flat sinks, signed/conservative
divergence, deterministic repeated results, malformed/over-budget inputs, actual mesh subdivision,
certified routing, config mismatch rejection, and baseline fallback under a tight cell budget.

No evidence yet shows that the extra cells improve full-board realization or runtime. The prior
Tier-0 failure occurred before native fitting, with no proven causal link to mesh resolution.
Do not interpret this feature as fixing that failure or replacing clearance-aware capacity.

The next controlled board comparison should hold maximum mesh cells and all downstream settings
fixed, compare disabled/enabled, and record prepass cost, passages retained, realized candidates,
whole-board physical legality and external KiCad DRC. An equal hard cap alone does not guarantee
equal actual cell counts; report both. GPU work is justified only after usefulness is established.
