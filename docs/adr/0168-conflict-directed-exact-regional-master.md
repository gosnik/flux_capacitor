# ADR 0168: Conflict-directed exact regional master

## Status

Accepted.

## Context

Regional routing already retained a finite portfolio of route templates, terminal accesses, layer
sequences, via sites, and packed cross sections. Its continuous shortlist could nevertheless spend
every PH/XPBD/PNS attempt on geometrically different realizations of the same discrete decision.
When exact continuous or quantized clearance rejected them all, the later residual-cone retry could
repeat the same discrete selection. That rejection proves only that the realized deformation
failed, however; it does not prove every deformation in that topology is impossible.

Putting PH controls directly into a mixed-integer model would make the state large, numerically
fragile, and duplicate Flux's authoritative geometry predicates. Conversely, adding only more
PathFinder history would ask a global scalar-resource solver to infer a local lane, via, terminal,
and topology rearrangement indirectly.

## Decision

Use the existing bounded deterministic regional CSP as an exact discrete master. It retains one
template per fragment and its established hard checks for coupled layer sequences, terminal and run
continuity, via-site/span and run-up interaction, physical channel packing, and lane order. Adjacent
selected channels additionally reject a lane-precedence reversal relative to their source geometric
ordering.

Keep quintic PH geometry outside the master. Continuous topology, shape, rational-offset envelope,
quantized mutual-clearance, connectivity, and DRC checks supply candidate-specific feedback. They
drive conflict-core extraction and column generation but do not by themselves create a hard
no-good. A hard no-good is emitted only by a monotonic preflight certificate over fixed endpoint
envelopes or packing, keyed by physical decisions rather than retry-local IDs:

- fragment template geometry, topology signature, route layer, layer sequence, and terminal
  portal/access geometry;
- via net, coordinate, and complete layer span;
- cross-section walls, layer, ordered occupants, and packed centre offsets.

Extract a bounded conflict core from failed PH realizations. Expand each implicated fragment to its
complete in-window `(net, connection, run)` so a layer excursion can leave and return through an
explicit via sequence. Generate new homotopy/corridor/layer/via columns only for those core runs;
all other window fragments remain fixed master variables. A discrete failure without a realized PH
candidate uses the window's owned fragments as its bounded core and does not invent a no-good.
Budget exhaustion, truncated sampling, and failed continuous realizations remain unknown for
no-good learning and are counted as withheld evidence.

Run a bounded number of master/separation rounds. A successful window replaces only itself and only
when the complete regional checkpoint strictly improves under the established feasibility-first
comparison. Exhaustion proceeds to the broader residual conflict-cone and global routability
feedback loops.

## Consequences

The router can deliberately leave a locally optimal topology without globally rerouting unrelated
nets, while preserving the possibility that a later deformation succeeds. Stable preflight
no-goods survive local ID renumbering, and counters expose cores, generated columns, master
reruns, clauses, rejected states, withheld unknown evidence, recovered windows, lane-order
rejections, and exhaustion through CLI JSON and telemetry.

This is not a claim that the internal CSP or PH geometry is canonical copper. It cannot commit a
partial window, accept a temporary violation, or weaken any exact predicate. Whole-board structural
validation, connectivity, represented copper DRC, source-preserving KiCad round-trip, and external
KiCad DRC remain authoritative.
