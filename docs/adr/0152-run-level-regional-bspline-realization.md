# ADR 0152: Run-level regional B-spline realization

## Status

Accepted after ADR 0151.

## Context

Regional realization built one cubic B-spline per straight route fragment. Every coarse corner and
articulation point was consequently a fixed spline endpoint. Two unlike nets sharing a sampled
junction could not be separated by elastic optimization even when several adjacent fragments were
owned by the same residual cone.

## Decision

Before regional spline optimization, merge contiguous owned fragments when they belong to the
same net, connection, run, routing layer, physical width/clearance state, and selected-global
topology and have no via boundary. The merged route unions the source topology cells and links,
keeps only its outer endpoints fixed, and represents each internal source vertex with local cubic
controls. Collision-normal elastic optimization and adaptive whole-window legality operate on the
merged curve.

A realization retains its final replacement endpoint so candidate reconstruction can replace the
complete contiguous source span once. Alternate-layer templates, moved-via boundaries, fixed
resource-width fragments, non-owned fragments, and incompatible widths remain separate. Candidate
availability, quantized reconstruction, connectivity, copper DRC, and transactional fallback are
unchanged. JSON reports merged runs and source-fragment counts.

## Consequences

On the isolated ESC, the selected residual checkpoint contains one eligible two-fragment merged
run. The complete-window count remains 183/193, while retained mutual-route failures improve from
20 to 19 with no topology failure. Runtime is 9:08.02 and peak RSS is 637,956 KiB. The ten
unresolved windows still withhold candidate copper.

Expanding cones with uncongested predecessor/successor guard fragments was measured and rejected:
17 guards created ten merged runs but regressed unresolved windows from ten to eleven and added a
topology failure. The implementation therefore merges only fragments already owned by the selected
regional transaction.

