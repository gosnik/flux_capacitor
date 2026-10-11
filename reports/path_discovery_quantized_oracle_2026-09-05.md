# Flux path discovery and the quantized-copper oracle

Date: 2026-09-05

## Current discovery stack

Flux does not have one path finder. It carries several complementary views of free space:

1. **Clearance-expanded polygonal free space** is extracted per net and copper layer. Pads, holes,
   keepouts, existing copper, board edges, and cutouts remain exact source obstacles. Same-net
   conductive copper may be omitted or reused where policy permits.
2. **Adaptive NavMesh cells** decompose that polygonal domain. The ordinary flood search selects
   connected cells and the funnel shortens the portal sequence. Two boundary-focused refinement
   levels recover narrow clearance regions without imposing the finest pitch on the whole board.
   Directed portal gates form a homotopy certificate carried into continuous realization.
3. **Hydraulic erosion** runs on every copper layer as an additional GPU topology generator. Its
   walkable mask covers the polygonal free space, obstacles form impassable terrain, terminal
   attraction creates a sink, and droplets expose riverbeds. Anisotropy, downstream failure
   pressure, and clean-route priors bias discovery. The field is advisory and is reprojected into
   exact NavMesh/free-space topology before use.
4. **The hybrid global graph** combines a sampled planar lattice, direct medial/Voronoi edges,
   terminal access portals, corridor capacity, and layer transitions. PathFinder negotiates
   occupancy, physical-width resources, future terminal demand, critical cuts, and typed exact
   failure history. When the quantized oracle owns physical feedback, clearance shadows are
   advisory broad-phase witnesses rather than another negotiated capacity dimension.
5. **Static route portfolios** now retain shortest, edge-diversified, RRT-Connect, per-layer
   hydraulic, and explicit perimeter-biased families. The perimeter carrier is not replaced by an
   endpoint-only shortest NavMesh query during PH preparation, so its longer homotopy survives as
   a fallback.
6. **Regional realization** converts residual physical conflicts into joint lane, escape, layer,
   and via decisions, then uses quintic PH curves, XPBD/PNS, quantization, exact copper checks, and
   finally KiCad DRC.

Each representation has a legitimate role. NavMesh is strongest at obstacle-side topology,
erosion at broad layer-specific discovery and learned route priors, PathFinder at shared-resource
allocation, and regional PH at continuous geometry. The failure was not lack of discovery
algorithms; it was the absence of an early common physical question.

## Differential evidence from KiCadRoutingTools

KRT routed the exact stripped `bm_svf` board in 7.30 seconds with 615 segments, 73 vias, 925.353 mm
of copper, zero opens, and no clearance/routing errors under the original KiCad project rules.
Flux's preceding 7,200-second run found paths for every connection but retained 19 sampled
overflows, three corridor overflows, and 324 clearance shadows before timing out in regional
realization.

KRT gains leverage from using one conservative 0.1 mm copper occupancy representation for both
search and commit. A route that exists combinatorially but cannot place its track width, clearance,
or via disc never becomes a deceptively successful global state. Its full-grid A* also naturally
retains long perimeter routes, and layer changes are ordinary search transitions. Failed forward
and backward frontiers identify a small blocker set for bounded rip-up.

Flux should not replace its richer discovery stack with KRT's grid. Instead, the fine quantized
space should act as a critic between global allocation and expensive continuous realization:

```text
NavMesh / hydraulic / RRT / perimeter discovery
                    |
                    v
      PathFinder topology and allocation
                    |
                    v
   0.1 mm quantized-copper feasibility oracle
          |                         |
        clean                    conflict/open
          |                         |
          v                         v
 regional PH realization     attributed history + affected nets
                                    |
                                    +------> next global epoch
```

This preserves Flux's curved-copper objective while acquiring KRT's most useful invariant: a
selected topology receives a physical verdict in the same integer track/via representation that
will eventually be exported.

## Implemented oracle

`probe_quantized_copper_feasibility()` now:

- snaps every interior global path point to a deterministic 0.1 mm lattice while preserving exact
  pad endpoints;
- uses resolved per-net widths, clearances, via diameters, drills, and explicit neck-down widths;
- collapses any same-site multi-layer transition chain into one complete physical via span;
- reuses existing same-net vias and plated through-hole component pads when they cover the span;
- builds a provisional board from integer `TrackSegment` and `Via` primitives;
- invokes exact dynamic-copper DRC and exact connectivity;
- maps each violation back to the implicated route net(s), layer, and localized overlap bounds;
- retains connection-run identity and classifies exact run/run conflicts as lane/template
  no-goods, exact via conflicts as via-site no-goods, and malformed transitions as layer-sequence
  no-goods;
- emits bounded `quantization` diagnoses and diagnostic telemetry;
- feeds those diagnoses into routability history and the next incremental affected-net cone;
- uses oracle opens and violation count as feasibility-first tie-breaks between otherwise
  comparable retained global portfolios;
- remains non-authoritative: it cannot accept final copper or bypass regional, connectivity, Flux
  DRC, or external KiCad DRC.

## Clearance-shadow ownership

When the default multi-epoch planner enables the oracle, raster clearance shadows now have one
bounded role: discover approximate physical contact and define telemetry/regional repair scope.
Adjacent raster samples are collapsed to the strongest witness per net pair, resource kind,
layer, and two-cell physical region. The coalesced witnesses remain in the physical conflict graph
and residual groups.

They no longer contribute to `total_overflow`, reject hard-capacity transitions, add present-cost
pressure, or accumulate candidate/owner edge and node history. Exact integer-copper diagnoses
provide the corrective feedback instead, retaining the responsible connection runs and selecting
lane/template, layer-sequence, or via-site relaxation. This removes the observed amplification in
which one physical overlap produced dozens of scalar raster penalties. A deliberately single-pass
legacy planner retains the former shadow behavior because it has no subsequent epoch capable of
consuming oracle no-goods.

The first bounded `bm_svf` measurement used the selected all-net checkpoint from one PathFinder
pass. Abstract routing reported all 25 routed nets, but the oracle found **769 exact dynamic-copper
violations and 11 opens**. The checkpoint contained **zero vias**. Oracle materialization, exact
DRC, connectivity, and feedback took about **0.63 seconds**; the expensive work remained global
portfolio/search construction. This is direct evidence that the new stage is both cheap enough to
run between global epochs and more physically discriminating than the prior sampled checkpoint.

## Via and long-route policy

Search now prices an ordinary via at three planar steps rather than eight. This is a search
preference only: exact via-site geometry, clearance shadows, capacity, regional site assignment,
and DRC remain unchanged. Existing same-net vertical features retain their cheaper reuse path.
The implicit via lattice now offers every global node as a candidate instead of spacing candidates
far enough apart that all could hypothetically be occupied. Candidate density is no longer
confused with simultaneous via legality: exact selected-via envelopes still negotiate and pass
DRC. An explicit via-site pitch remains available as a performance/sparsity control.

One static seed slot is now an explicit perimeter-biased route. Interior depth adds a non-negative
cost during that search, producing a genuinely different long homotopy without authorizing new
edges. This candidate is protected from ordinary duplicate pruning and from the endpoint-only
NavMesh shortcut that would otherwise collapse it back to the shortest class.

## Next leverage point

The implemented oracle evaluates and attributes a selected allocation. The next bounded extension
should reuse the same representation for **conflict-cone search**, not run another board-wide
continuous portfolio:

1. retain the exact failed forward/backward frontier around an oracle conflict;
2. use NavMesh gates and hydraulic/perimeter carriers as allowed homotopy columns;
3. run fine-grid multilayer search only for the implicated connection and its small victim set;
4. keep exact-size via occupancy in that local map;
5. commit a complete local exchange atomically;
6. pass the clean polyline's topology/layers/vias to PH for curve optimization.

This is the route to KRT-like performance without sacrificing Flux's topology certificates,
continuous curved geometry, or transactional validation model.
