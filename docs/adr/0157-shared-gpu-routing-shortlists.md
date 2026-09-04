# ADR 0157: Shared GPU routing shortlists and reductions

## Status

Accepted.

## Context

Regional realization repeatedly spent CPU time evaluating large collections of deliberately
non-authoritative alternatives before exact geometry rejected most of them. The same pattern
occurred in global reverse searches, neck-down product states, arc candidates, differential-pair
tuning assignments, local terminal fanout, RRT nearest-node queries, and PathFinder reductions.
Moving the final decision to approximate GPU arithmetic would weaken Flux's transactional and
deterministic acceptance boundary.

## Decision

Use the process-wide Vulkan runtime for the following bounded advisory work:

- candidate-major PH tessellation followed by KiCad-coordinate quantization and mutual copper
  envelope scoring;
- regional template compatibility filtering, with exact via identity and aggregate channel-width
  checks on the returned states;
- dense subset lane-order DP for at most twelve occupants;
- Boolean Voronoi cross-section orientation enumeration followed by exact junction/miter checks;
- job-major reverse graph fields for global meeting search and explicit
  `(node, width-mode, narrow-distance)` product graphs;
- guarded segment/capsule/circle narrow-phase classification after the shared broad phase;
- circular-arc candidate ranking before exact native-arc board transactions;
- compatibility-bitset filtering of differential-pair skew assignments;
- exact-edge-prepared terminal fanout wavefronts and large-tree RRT nearest-node queries; and
- stable-order negotiated-resource overflow/history reductions.

The shared routing shader uses 64-bit integer cross products for simple-copper intersection
classification. Vulkan `shaderInt64` is therefore mandatory. Distance estimates retain an explicit
numerical guard. A GPU result may reject work only when its contract is conservative; otherwise it
only ranks a bounded shortlist. CPU analytical geometry, canonical validation, connectivity, and
KiCad DRC remain authoritative.

GPU CSP ordering is normalized back to the established deterministic variable order before
downstream portfolio bounds are applied. GPU lane DP is a fast seed: the existing Pareto/exhaustive
CPU tail remains available unless the accelerated seed proves the zero-inversion optimum. Failed
dispatches fall back to the corresponding CPU implementation without changing legality.

## Consequences

The routing workers submit larger coherent batches and copy back compact scores, states, conflict
flags, or distance fields. No new authoritative board or routing representation is introduced.
GPU availability is now part of the executable platform contract, while exact acceptance remains
portable C++ and deterministic.
