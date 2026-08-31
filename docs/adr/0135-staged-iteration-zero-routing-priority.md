# ADR 0135: Staged iteration-zero routing priority

## Status

Accepted for v0.121.

## Context

The mandatory static seed portfolio deliberately evaluates each complete net task against empty
occupancy. That makes portfolio construction parallel and deterministic, but an independently good
set of trees tends to select the same central channels. Flux v0.119 added one weak-occupancy
portfolio which chooses among those already prepared trees in stable net-priority order. It does
not search again, but its order gives early flexible nets the same opportunity to claim scarce
resources as terminal-constrained or high-interference nets.

Changing normal PathFinder legality or its later negotiated scheduling is neither necessary nor
desirable. The missing decision is limited to the order in which the iteration-zero provisional
occupancy is assembled.

## Decision

- After terminal-access filtering and the parallel static portfolio have completed, derive one
  immutable evidence record for each ordinary net or declared differential-pair/routing-bus atom.
- Reuse existing data only. Record minimum legal terminal portals; viable complete trees;
  corridor-bearing candidates; distinct coarse topology and layer sequences; overlap of the
  primary candidate with other primary candidates; resolved physical envelope; and primary route
  extent.
- Treat shared sampled nodes/edges in primary candidates as attributed interference/scarcity
  evidence. This is a seed-order signal, not a new capacity or legality model.
- Derive deterministic board-relative thresholds from integer quartiles. Correlated portal,
  corridor, topology, and layer observations contribute one constrained-alternative signal rather
  than several votes for the same scarcity.
- Assign each atom to exactly one wave: short non-contentious, constrained/high-interference,
  ordinary, or highly flexible. Preserve explicit board/caller priority as the first key, then wave
  order, wave-specific severity, and stable group ID. Preserve pair polarity and declared bus order
  inside each atom.
- Apply this order only while choosing complete static trees for the weak-occupancy iteration-zero
  portfolio. Static route searches, the empty portfolios, PathFinder present/history negotiation,
  rip-up, detailed routing, connectivity, and DRC are unchanged.
- During qualification, also assemble the preceding stable weak-occupancy order. The staged
  portfolio is eligible only when the ordinary feasibility-first comparator does not rank it below
  that baseline and it creates no additional clearance-shadow conflicts. The baseline remains a
  selectable checkpoint until the owned fixture/ESC gate permits its deletion.
- The v0.122 qualification amendment does not promote a changed seed on a board with at most 20
  atomic routing groups unless that staged seed is already complete. Identical small-board seeds
  and larger diagnostic boards retain the ordinary comparison. This preserves the owned-fixture
  fallback while end-to-end negotiated parity remains unproved.
- Publish ordered evidence, wave counts, baseline/staged unrouted counts, aggregate overflow,
  clearance-shadow counts, qualification, and final selected-policy identity.

## Consequences

The classification and reduction are deterministic across static-seed worker counts, and coupled
members cannot be separated by the new order. The fourth iteration-zero portfolio temporarily adds
one occupancy-accounting pass but no route search. A regressing staged order cannot silently replace
the previous seed.

On the current isolated ESC one-pass diagnostic, the tuned classifier produced 2 trivial, 41
constrained, 22 ordinary, and 16 flexible atomic groups. The staged portfolio passed the bounded
qualification gate and was selected. The run completed in 123.08 seconds at 354,044 KiB peak RSS;
219 of 232 eligible regional windows formed complete assignments and 13 remained unresolved. The
regional search rejected 11 packing states, down from 17 in the immediately preceding 13-window
checkpoint, but accepted only 3 alternative templates. No regional spline candidate, detailed
copper, or external-DRC claim follows from this result.

v0.122 also adds primary via-transition count, distinct sampled via sites, complete via-assignment
signatures, and primary-site contention to the immutable evidence. Mandatory scarce transitions
contribute one constrained signal and prohibit a via-scarce route from entering the trivial wave.
