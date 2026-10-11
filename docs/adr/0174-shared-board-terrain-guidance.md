# Shared board terrain guidance and layer coupling

Status: per-layer shared guidance in implementation; cross-layer coupling proposed and not enabled.

## Purpose

Run a bounded board-wide map → erosion → demand-pressure preparation before per-connection
hydraulic discovery. Reuse target-independent terrain between candidates and refresh pressure
when routing demand changes. Exact net-specific clearance, physical copper admission and KiCad
DRC remain authoritative.

The terrain field stage does not depend on endpoints. The extracted CPU and GPU field APIs
therefore avoid both invented endpoints and unnecessary route extraction. The first integration
reads an evolved field back once for immutable host-side sampling; it does not claim that the
shared field remains resident across GPU pipelines.

## Layer bands

The proposed extension is a graph of two-dimensional layer bands, with sparse via transitions.
It is not an isotropic three-dimensional clearance transform. Lateral obstacle clearance and
erosion remain within a layer. Routing pressure may redistribute between bands only at
admissible via transitions, accounting for their capacity and cost.

Keep canonical `Layer::stack_order` and physical `BoardStackup` unchanged. Algorithmic depth is
an independent routing-cost coordinate. In the current low-height preference model, literally
lowering a plane would attract routes; discouraging it requires a positive cost or access barrier.
Reordering physical layers would corrupt via-span,
intermediate-layer copper and stackup semantics. A constant per-layer height offset alone is
also insufficient: local erosion depends on gradients, so a preference must enter travel and
transition costs explicitly.

A proposed lateral cost is physical distance times a positive layer-role multiplier, plus
bounded terrain and negotiated congestion costs. A proposed vertical edge cost includes via
construction and span cost, local via crowding and role preference. Existing through vias can
span several bands; every affected layer and drill clearance must be checked. No diffusion
through arbitrary overlapping XY cells is allowed to imply a usable layer transition.

Plane protection is net-aware. Signal travel through plane space should be discouraged or
prohibited according to explicit policy; same-net ground/power attachment to its plane should
remain attractive. Existing foreign-net plane copper is an obstacle, not merely high cost.
Do not infer plane roles from net names. Prefer explicit imported design intent, with zone
ownership/coverage as supporting evidence and a neutral policy when intent is unknown.
Currently `import_layers` collapses KiCad signal/power types to `LayerKind::copper`; the canonical
model does not retain a semantic plane role. Automatic role-based preference therefore first
requires preserving that metadata or accepting an explicit per-layer policy. Fill coverage alone
does not establish that a layer is reserved for power or ground.

## Smallest cross-layer experiment

Use a synthetic multilayer bottleneck with a free alternative signal layer and a designated
plane layer. Compare independent fields with coupled pressure. Require fewer physical conflicts
without illegal via transitions, unbounded via growth, or signal cuts through reserved planes.
Include a same-net plane attachment and a via blocked on an intermediate layer. Then run tiers
sequentially with telemetry, preserving the per-layer ablation switch.

Track preparation time, shared field reuse, pressure iterations/residual, per-layer demand,
vertical transfer, rejected via transitions, via count, physical conflicts and final DRC.
Cross-layer pressure is a routing preference estimate, not a connectivity certificate.

## Sources

- [KiCad PCB Editor](https://docs.kicad.org/master/en/pcbnew/pcbnew.pdf): physical stackup,
  layer roles (advisory) and via/layer routing semantics.
- [VPR routing resource graph](https://vtr-verilog-to-routing.readthedocs.io/en/latest/api/vpr/rr_graph/):
  explicit routing resources and switch connectivity; relevant architectural analogy for sparse
  inter-band transitions, not a PCB legality model.
