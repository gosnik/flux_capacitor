# ADR 0133: Weak-occupancy seeding and regional placement feedback

## Status

Accepted for v0.119.

## Context

Several independently shortest empty-board trees often choose the same central corridors. Later
negotiated history can separate them, but that is an unnecessarily poor dense-board starting
allocation. A failed regional bundle also contains more actionable placement geometry than a
whole-net congestion count.

## Decision

Retain the deterministic parallel empty-occupancy searches, then assemble one additional portfolio
without running more A*. Walk nets in their stable priority order. For each ordinary net, choose
the complete retained tree with the least provisional pressure over sampled nodes/edges,
clearance shadows, scan corridors, and shared physical resources; use path cost and candidate index
as stable later ties. Preserve one common candidate index for coupled members. Add the chosen tree
to occupancy before considering the next net.

Compare this mixed portfolio with every uniform-index portfolio using the existing feasibility-first
checkpoint comparator. It has no special acceptance preference and cannot make iteration zero
worse.

When integrated placement sees unresolved regional windows, propose bounded moves away from their
physical centers for connected terminal footprints. If a conflict edge attributes a footprint or
owned pad/keepout as an obstacle, also feed that edge's local bounds into the same proposal path.
All moves remain subject to placement priority, hard constraints, fresh routing, exact connectivity,
and copper DRC.

## Consequences

- Dense routing starts from one occupancy-aware combination of already paid-for alternatives.
- Determinism and the static-search parallel boundary are preserved.
- Regional failures can request placement relief using local physical evidence.
- This is weak sequential portfolio assembly, not a global bundle search over every net tree.
