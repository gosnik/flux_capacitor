# ADR 0138: Fragment-anchored windows and bounded via CSP

## Status

Accepted as an incomplete regional-routing milestone for v0.123.

## Context

A regional window was originally expanded only from its coalesced sampled conflict bounds. A
capacity or clearance-shadow sample can be displaced from the continuous fragment later selected
by attribution. Fifty-five isolated ESC windows therefore failed portal clipping even though the
physical fragment identity was known. Two later windows also exhausted 512 complete-tuple via
assignments, and the four closest legal choices often occupied one local clearance basin.

## Decision

- Build the initial sampled window as before. For each attributed fragment that does not intersect
  it, add the quantized nearest point on that segment to the window's anchor bounds. Apply the
  configured margin once to the union, clamp it to board bounds, then run the existing portal clip.
- Report expansion count and peak physical displacement. A fragment that still cannot be clipped
  remains an explicit `uncuttable_fragment` failure.
- Assign via variables in existing scarcity/contention order with deterministic depth-first
  backtracking. Reject terminal/via and assigned-via conflicts on the partial state and forward
  check that every remaining variable retains at least one compatible candidate.
- Count each attempted partial assignment against the unchanged positive state bound. Exhaustion
  remains distinct from an exhaustively proven infeasible portfolio.
- Keep the selected-global site first, but reject nearby alternative sites until retained choices
  are separated by at least via diameter plus resolved clearance. The per-region site bound does
  not increase.
- Connectivity, via span, represented clearance, B-spline realization, and whole-board DRC remain
  exact transaction gates.

## Consequences

On the one-pass isolated ESC checkpoint, 57 physical anchors with a peak 18.68 mm expansion make
all 242 windows eligible. Forward checking reduces total access states from 1,227 to 448 and
removes both 512-state exhaustions. Spatial diversification resolves one formerly infeasible via
window; 238 windows complete and four remain as physical lane-packing failures. The run remains
incomplete and no candidate is exported.
