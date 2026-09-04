# flux_capacitor

The native C++/Dear ImGui application for live router inspection and JSONL record/replay is
documented in [docs/router-telemetry-viewer.md](docs/router-telemetry-viewer.md). It visualizes
routes, rip-up activity, Pathfinder metrics, congestion and clearance conflicts, resource
occupancy, routing priority, and via-site decisions from `flux.telemetry.v1`.

The v0.128 router-side observer exposes global routing milestones, staged-priority evidence,
regional residual groups, attributed physical conflicts, bounded via candidates, and selected via
assignments through a bounded, read-only Boost.Beast server at
`ws://127.0.0.1:8765/telemetry` by default for KiCad routing and integrated-optimization commands;
`--no-telemetry` is the explicit headless opt-out. A preallocated non-blocking MPSC ring
isolates workers from the observer, while stable-ID coalescing and recovery snapshots let the
viewer recover current state. A separately cached immutable source snapshot supplies the viewer
with board outline/cutouts, layers/nets, footprint courtyards and pads, existing tracks/native arcs,
vias, zones, and keepouts without retransmitting the board in every dynamic snapshot. A stable
path-plus-revision identity lets the same observer retain multiple KiCad source scenes.
Selected global polylines are overlaid as provisional routes even when regional/detailed routing is
incomplete. Complete local quintic PH tessellations, selected regional vias, and their exact-static-
checked straight or dogleg run-ups are separate provisional overlays, so a via relocation is not
displayed as an unconnected marker. Detailed-copper, placement, and validation events remain
incremental follow-up coverage.
The activity table is newest-first, and compact regional `SearchSummary` events keep long PH
window realization observable after the global pass has finished. Chronological trace storage and
replay semantics are unchanged. Search-only bursts do not trigger unchanged recovery snapshots.

Connected coarse topology may proceed to exact detailed materialization when every regional window
has produced one complete atomic PH candidate, even if sampled PathFinder/shadow overflow
remains. Exact connectivity and copper DRC still reject the complete transaction. See
[`ADR 0153`](docs/adr/0153-exact-regional-handoff.md) and
[`ADR 0154`](docs/adr/0154-default-websocket-telemetry.md). Dense-board observability and the
current Freerouting control are recorded in
[`ADR 0155`](docs/adr/0155-dense-board-observability-and-external-control.md).

The active continuous route representation is now a tangent-continuous chain of quintic
Pythagorean-hodograph Bezier segments. PH Hermite interpolation, centripetal/chord parameter
selection, optional bounded C2 joining, polynomial speed and arc length, analytical curvature and
inflection isolation, exact rational offsets, and bounded rational-offset intersections replace
the former cubic B-spline implementation. XPBD and PNS move interpolation nodes and rebuild the PH
segments; telemetry publishes flattened six-control degree-five spans. Detailed routing uses a
resident Vulkan XPBD whole-chain precondition and candidate-major GPU polishing: every bounded
node-move wave is sampled/ranked on the device, while only a stable four-candidate shortlist pays
the recursive CPU shape/corridor certificate. Fine-grained certificates remain serial because
their recursive long tails measured worse when synchronized per node. Complete same-layer route
chains instead run as bounded parallel tasks against one immutable board snapshot; compact copper
deltas are reduced in stable route order and exact-rechecked before commit. CPU exact geometry,
connectivity, DRC, and commit remain authoritative, and unavailable compute restores the complete
CPU wave. KiCad has no electrical Bézier-track primitive, so the PH Bézier remains Flux geometry
and accepted routes are adaptively emitted as native connected track arcs/segments. See
[`ADR 0156`](docs/adr/0156-quintic-ph-continuous-routing.md).
Residual retry now solves only transitive fragment-owner cones, reuses unaffected assignments,
and deterministically remaps derived channel/terminal/via identities before atomic checkpoint
replacement. Telemetry and CLI JSON distinguish reused, recomputed, and withheld assignments.
Prepared output validation and NavMesh routing now share cached polygon bounds, boundary indices,
inline immutable cell collections, and a cutout AABB hierarchy. Regional alternatives use
fail-closed local domains, bounded worker caches, clearance-capped distance queries, and indexed
lane-order screening. The completed isolated-ESC qualification takes 18:08.16 at 264,772 KiB peak
RSS and reports 288 reused plus 15 recomputed cone assignments with none withheld. It is still an
incomplete diagnostic checkpoint: 22 nets are unrouted, sampled overflow is 1,114, and 4,756
clearance-shadow conflicts remain, so no detailed copper or KiCad candidate is produced.

`flux_capacitor` is an experimental C++ engine for integrated PCB component placement
and track routing. Placement and routing share one board model so that routing pressure
can improve placement and bounded placement changes can resolve routing failures.

The current executable baseline provides:

- a versioned C++ board IR with integer-nanometre coordinates, stable typed IDs, and
  board-fixed or footprint-owned layer-aware keepouts, plus per-footprint allowed absolute
  rotations and front/back placement permissions;
- board/footprint/pad/track/via/filled-zone/layer/net models plus canonical
  multiple-net-class membership and inferred differential-pair identity with deterministic,
  per-property priority resolution; saved zone regions retain simple outer contours and exact
  hole contours;
- deterministic transforms and polygon/segment predicates over exact outer-board-minus-cutout
  material;
- structural and placement validation with machine-readable issue codes;
- multilayer connectivity analysis spanning pads, tracks, vias, and immutable physical
  filled-zone polygons;
- explicit pad geometry and board fallback plus per-net-class clearance, track, via,
  drill, and board-edge rules;
- shape-aware internal copper DRC for rotated circle, rectangle, oval, and
  round-rectangle pads, filled copper zones, and independent track/via/pad/pour keepout
  rules, with conservative fallbacks for explicitly lossy shapes;
- deterministic multilayer A* global routing with negotiated edge/node congestion,
  exact board-edge, indexed shape-aware foreign-copper, filled-zone, and track/via keepout
  obstacles,
  orthogonal off-grid pin access, vias, incremental multi-pin trees, capacity overflow,
  stagnation limits, deterministic per-resource demand/contributing-net evidence, and
  affected-cone checkpoint reuse with automatic geometric invalidation; planned width,
  via diameter, and foreign-net spacing use each net's resolved routing rules; conservative
  slicer-inspired scanline decomposition groups distinct free lanes into capacity-bearing
  corridor resources whose present/history congestion participates in PathFinder allocation;
  between unsuccessful passes, deterministic pressure-aware reroute ordering promotes unrouted
  and repeatedly congested nets inside their existing priority tier while keeping declared
  differential pairs and routing buses contiguous and ordered; an opt-in hybrid graph also lets
  PathFinder traverse independently rechecked capacity-bearing Voronoi chords directly instead of
  visiting every intermediate grid sample; explicitly bounded terminal neck-down access may
  recover an ordinary-net pin escape only after nominal width fails, every narrow segment passes
  exact represented clearance, and its grid portal remains legal at nominal width; an independent
  direct-medial policy can allocate a short exact chord at configured narrower demand only when
  nominal demand cannot fit and nominal-width disks remain legal at both transition portals; an
  opt-in stateful sampled-corridor policy constructs a second conservative narrow-width corridor
  graph and lets bounded A* width-mode runs cross multi-edge bottlenecks while charging their own
  physical corridor capacity; when shared medial capacity is enabled, every selected ordinary-net
  width contributes the same `width + 2 * clearance` envelope to PathFinder cost, overflow,
  packing, and checkpoint reuse; an opt-in late constructive phase can reject every transition
  that would exceed sampled, corridor, or selected-width shared physical capacity while retaining
  the best earlier negotiated checkpoint if no strict ordering completes; strict failures also
  retain directed occupying-net evidence, and an independently bounded singleton repair can rip
  complete same-pass blockers, retry first, and defer every victim for a fresh route;
- opt-in transactional KiCad zone-refill routing that treats saved pours as planning-time
  refillable geometry, emits only a provisional result from C++, refills a separate candidate
  through KiCad, and publishes it only after strict re-import and whole-board KiCad DRC;
- deterministic detailed materialization that simplifies global paths into tracks and
  spanning vias, transactionally reuses validated unaffected-net copper with stable IDs,
  falls back to full rematerialization when reuse is invalid, and rejects structurally
  invalid, disconnected, or DRC-illegal results, and materializes per-net resolved
  track/via dimensions unless an explicit run-wide override is requested; deterministic
  45-degree corner pull-tight shortens physical copper and falls back to the legal orthogonal
  route whenever exact checks reject a shortcut; bounded continuous centerline projection then
  shortens conventional polylines inside per-vertex trust regions, using deterministic halving
  to approach obstacle clearance, a run-wide exact-candidate budget, quantized local DRC probes,
  whole-board connectivity/DRC acceptance, and discrete fallback; opt-in strict-medial
  materialization can replace topology-complete resources with checked packed lane chords,
  accepted same-layer
  route-selected junction bridges whose shifted resource lines can meet at checked polyline
  miters, exact terminal joins, and multi-layer runs; complete differential pairs and
  ordered buses can select spacing-checked, order-preserving via arrays as one group transaction;
  mixed-width terminal and direct-medial paths can undergo bounded continuous refinement inside
  equal-width planar runs while every width/layer transition remains fixed, with exact local checks
  and transactional discrete fallback; detailed CLI results retain the governing global status,
  failing-net connectivity details, a copper-violation histogram, and one representative violation
  without exporting an illegal candidate;
  bounded group fanout synthesis preserves legal direct joins or packs ordered staircases at pad
  and layer-run boundaries, can translate a via array longitudinally to create escape run-up, and
  atomically falls back when exact clearance, gap, skew, connectivity, or board DRC rejects every
  candidate; edge-aligned terminal width metadata is simplified only inside equal-width runs and
  materialized as distinct straight KiCad segments; equal-width orthogonal corners can also
  receive deterministic 45-degree pull-tight without moving a width boundary; optional native
  curved fitting likewise operates only within equal-width same-layer runs and inherits that local
  width, while topology-locked replacement remains withheld for that path;
  opt-in post-allocation cubic B-spline routing preserves endpoint tangents, adaptively tessellates
  and fits native lines/arcs, builds exact collision envelopes, invokes bounded recursive PNS
  push-shove for unlocked straight blockers, updates the control corridor, and refits using local
  basis-weighted control displacements; all budgets and whole-transaction polyline fallback remain
  explicit; an independent opt-in
  post-route skew tuner adds ordinary rectangular polyline length to the shorter member of a
  freshly routed point-to-point constrained differential pair, packs one or more deterministic
  periods within explicit amplitude/spacing/candidate bounds, certifies every replacement chord
  against extracted polygonal free space, protects source and reused copper, and commits only after
  exact whole-board acceptance; an independent opt-in batch allocator retains several individually
  legal regions per pair and uses bounded exact conflict search to maximize simultaneously repaired
  pairs instead of committing by greedy pair order;
- deterministic orientation-aware analytical placement using weighted net centroids,
  mixed-size density repulsion, source anchoring, and exact courtyard/board/keepout
  legalization while preserving locked footprints; explicit rigid groups and intersecting
  relative-constraint graphs are projected onto common translations and legalized atomically,
  with one lock immobilizing the complete policy component; an opt-in routing-informed pass feeds
  bounded PathFinder congestion, width-reduction, and blocker escape targets into analytical
  attraction before exact legalization; an independent bounded simultaneous legalizer can assign
  several singleton or policy-component atoms to analytical/routing-guided sites together, making
  occupied-site exchanges legal without exposing an overlapping intermediate placement;
- a bounded deterministic integrated optimizer that tries legal unlocked-footprint moves,
  opt-in discrete rotations and side changes constrained by each footprint's hard orientation
  policy, opt-in atomic position swaps between route-ranked unlocked footprint pairs, and bounded
  rigid translations of explicit designer-declared placement groups; an additional opt-in
  neighborhood translates the complete fixed-point closure of intersecting relative constraints
  and rigid groups as one atomic route-guided component; it can
  evaluate a legalized global placement seed, globally reroutes every proposal,
  runs detailed connectivity and DRC, and accepts only lexicographic feasibility improvements;
  it derives bounded move targets from congestion hot spots, blocking-object geometry, and
  other-pin centroids when the evaluation budget is tight; local candidates use incremental
  global/detailed screening but require a full-board reroute and fresh copper materialization
  before acceptance;
  an explicit routed-quality mode continues beyond first feasibility, reduces actual global
  route cost, and stops only at a bounded local optimum or a truthful pass/evaluation limit;
  equally feasible candidates prefer lower normalized width-loss cost, then actual materialized
  route length, before abstract global search cost;
  every fresh fully routed, connected, and DRC-clean candidate also participates in a bounded
  deterministic Pareto archive across width loss, physical length, route cost, side/rotation
  changes, and placement displacement, without changing the active lexicographic search path;
- optional source-bound placement-move priority sidecars classify every footprint and stage
  cumulative move authorization from resistors and capacitors through transistors, other parts,
  miscellaneous ICs, MCUs, and SoCs, with connectors and pin headers unlocked only as a last
  resort; atomic transactions wait for their highest member tier, while locks and every exact
  placement/routing acceptance gate remain stronger than priority;
- bounded KiCad S-expression parsing and strict `.kicad_pcb` import for layers (including
  `power`-typed inner copper), nets, one outer outline with disjoint board/footprint-owned
  `Edge.Cuts` cutouts, footprints, courtyard provenance, pads, segments,
  track arcs, vias, simple or exactly unfractured saved copper-fill regions, and polygonal
  board/footprint rule areas with all five KiCad keepout categories preserved;
- source-preserving KiCad candidate export with content-derived stale-snapshot checks,
  immutable filled-zone preservation, deterministic UUID mapping, strict semantic round-trip
  verification, and refusal to overwrite an existing artifact;
- deterministic source-bound Specctra DSN export and routing-only SES import for black-box
  freerouting comparison, with exact source/DSN byte provenance, complete placement and protected
  copper verification, 100 nm session-grid checking, normalized deterministic route IDs, full
  connectivity/represented-DRC acceptance, and fail-closed unsupported geometry;
- deterministic source-bound orientation-constraint sidecars with strict stable-ID/reference
  validation, current-only template generation, scoped repair/refinement commands, and safe
  handoff of supported physical changes back to KiCad;
- deterministic source-bound placement-group sidecars with strict stable-ID/reference membership,
  empty non-inferential templates, disjoint group validation, and optimizer-only KiCad policy
  preservation;
- deterministic source-bound placement-region sidecars with explicit world polygons, stable
  footprint membership, full transformed-envelope containment, empty non-inferential templates,
  and hard enforcement across global and integrated placement;
- deterministic source-bound relative-placement sidecars that capture nominal anchor vectors from
  the immutable source and enforce inclusive reviewed X/Y deviation windows across all placement
  paths without inferring relationships from connectivity or proximity; version 2 additionally
  captures nominal radial distance and relative rotation and can enforce optional inclusive
  distance and circular microdegree windows;
- exact source representation of circular KiCad courtyards with conservative circumscribed search
  proxies, plus complete source-bound placement-envelope sidecars whose pad-derived suggestions
  remain unusable until every missing footprint is explicitly reviewed and approved; an
  independent opt-in may instead derive tentative pad/Fab/Silk bounds for proposal generation,
  while marking every resulting placement for mandatory post-process human review;
- a read-only `kicad-python` IPC bridge for explicit project net classes and effective per-net
  constituents, including differential-pair width/gap defaults, with deterministic source-bound
  snapshots consumed by every KiCad routing and optimization command;
- deterministic C++ source/candidate change sets that require byte-for-byte reproduction through
  the source-preserving exporter, inventory every live footprint and routing UUID, and describe
  exact footprint-transform previews plus track, arc, and via create/update/remove operations;
- a fail-closed `kicad-python` candidate bridge whose default mode only previews an exact live-board
  match and whose opt-in routing apply performs a second in-commit stale check, validates every
  returned object, and pushes one unsaved undo step; its offline validation mode exercises the
  current C++ change-set contract without requiring a running editor; live footprint apply remains
  withheld while current upstream whole-footprint updates can damage schematic linkage;
- a minimal routing-only KiCad workflow that packages measured free-space/PathFinder policies as
  the bounded `autoroute_v1` preset, requires a new candidate path, previews a source-bound change
  set, runs external KiCad CLI DRC when available, and permits live routing application only after
  that independent DRC passes;
- a bounded integrated `place_route_v1` KiCad workflow that runs analytical/routing-guided global
  placement and detailed routing together, defaults to the reviewable resistor-first/connector-last
  move policy, verifies a source-bound candidate and external KiCad DRC, reports exact human-review
  reasons, and withholds every placement result from live IPC application;
- deterministic source-bound differential-pair sidecars for explicit minimum physical gap and
  maximum routed skew constraints, applied as hard detailed-routing and optimization acceptance
  gates without pretending preferred KiCad net-class geometry is itself a DRC limit;
- a feasibility-first structured objective vector;
- a deterministic CLI demonstration and dependency-free CTest suite.

Safe live footprint transforms, transactional zone refill/editing and ambiguous fill contours,
non-courtyard circular graphics, custom rules, gridless/shove detailed routing, scalable density
models, spatially incremental DRC, and richer PCB electrical constraints are the next milestones.
See
[AGENTS.md](AGENTS.md) for the engineering contract and research-backed roadmap.

Architectural decisions are recorded in [`docs/adr`](docs/adr), beginning with the
canonical board core and negotiated-congestion routing baseline.

## Build and test

Requirements: CMake 3.25+, Ninja, a C++20 compiler, Boost 1.74+ headers, Vulkan headers/loader,
`glslc`, and a Vulkan 1.1 compute-capable GPU with a working driver. GPU acceleration is a
mandatory Flux runtime requirement: configuration fails without the Vulkan toolchain and routing
fails closed if a compute device cannot execute the startup probe. The deterministic CPU hydraulic
implementation is retained only as a test/oracle backend, not as a production fallback.
Boost.Polygon supplies the permissively licensed integer segment-site Voronoi builder; all
resulting graph chords are quantized and rechecked by Flux before use.

```sh
cmake --preset dev
cmake --build --preset dev
ctest --preset dev
```

An AddressSanitizer/UndefinedBehaviorSanitizer preset is also available on supported
GCC/Clang systems:

```sh
cmake --preset asan
cmake --build --preset asan
ctest --preset asan
```

## CLI baseline

The shortest usable path creates a new routed candidate, verifies its exact source-bound change
set, and runs KiCad CLI DRC when `kicad-cli` is installed:

```sh
python3 plugins/kicad/route_board.py \
  --board board.kicad_pcb \
  --output board.flux-routed.kicad_pcb \
  --flux-cli ./build/dev/flux-cli
```

Add `--capture-netclasses` while the matching saved board is open in KiCad to capture authoritative
project rules through the read-only IPC bridge. Alternatively pass an existing source-bound
snapshot with `--netclasses`. Preview is offline and non-mutating. After reviewing it,
`--apply-routing` applies tracks/arcs/vias to the matching open board as one unsaved undo step and
requires an external KiCad DRC pass. The workflow never saves KiCad and refuses existing output
files.

For an explicit provisional zone-refill transaction with the crossing-safe diagonal graph and
native cubic B-spline/PNS line-arc fitting, use:

```sh
python3 plugins/kicad/route_board.py \
  --board board.kicad_pcb \
  --output board.flux-curved.kicad_pcb \
  --flux-cli ./build/dev/flux-cli \
  --refillable-zones \
  --curved-traces
```

This path always refills zones in a separate file, strictly re-imports it, runs KiCad CLI DRC, and
requires human review. It cannot apply routing directly to the open editor.

For a congested refillable board, add `--repair-sampled-conflicts`. After the negotiated pass it
preserves every unaffected route and visits at most two layer-local conflict tiles in stable
pressure order, with at most 12 hard-capacity passes per cone. Every sampled, shared-physical, and
clearance-shadow participant in the selected cone is invalidated; terminal portals are diversified
and the complete participating nets are rerouted with hard capacity. A non-improving repair is
discarded transactionally.

To inspect an unsuccessful run in KiCad, add `--keep-incomplete-preview` and choose a new output
name. The wrapper retains the materialized layout, reports `ok: false` plus a `review_only`
`incomplete_preview`, and returns Flux's nonzero status. Zone refill, external approval, and live
application are deliberately withheld. The lower-level equivalent is `detail-kicad BOARD
--write-incomplete-preview OUTPUT`.

For integrated placement plus routing, use the same bridge in candidate-only mode:

```sh
python3 plugins/kicad/route_board.py \
  --board board.kicad_pcb \
  --output board.flux-place-routed.kicad_pcb \
  --flux-cli ./build/dev/flux-cli \
  --integrated-placement
```

`place_route_v1` defaults to two placement passes, 64 routed candidate evaluations, routing-guided
global placement, and suggested staged move priorities. Pass a reviewed source-bound policy with
`--placement-move-priorities` to replace the prefix-derived classification. Use
`--tentative-envelopes` only when provisional pad/Fab/Silk bounds and mandatory mechanical review
are acceptable. The output remains a new file-level candidate even after external KiCad DRC;
`--apply-routing` is deliberately rejected in this mode because the live bridge does not mutate
footprints. See [`ADR 0112`](docs/adr/0112-bounded-kicad-place-route-workflow.md).

```sh
./build/dev/flux-cli version
./build/dev/flux-cli demo
./build/dev/flux-cli demo-invalid
./build/dev/flux-cli route-demo
./build/dev/flux-cli route-demo --strict-medial
./build/dev/flux-cli route-demo --strict-medial --direct-medial
./build/dev/flux-cli free-space-demo
./build/dev/flux-cli incremental-route-demo
./build/dev/flux-cli detail-demo
./build/dev/flux-cli detail-demo --strict-medial
./build/dev/flux-cli incremental-detail-demo
./build/dev/flux-cli optimize-demo
./build/dev/flux-cli orientation-optimize-demo
./build/dev/flux-cli group-optimize-demo
./build/dev/flux-cli constraint-global-optimize-demo
./build/dev/flux-cli routing-guided-global-optimize-demo
./build/dev/flux-cli refine-demo
./build/dev/flux-cli global-optimize-demo
./build/dev/flux-cli inspect-kicad board.kicad_pcb
./build/dev/flux-cli inspect-free-space-kicad board.kicad_pcb NET_NAME F.Cu --shared-domain
./build/dev/flux-cli constraints-template-kicad board.kicad_pcb
./build/dev/flux-cli envelopes-template-kicad board.kicad_pcb
./build/dev/flux-cli diffpairs-template-kicad board.kicad_pcb
./build/dev/flux-cli buses-template-kicad board.kicad_pcb
./build/dev/flux-cli placement-move-priorities-template-kicad board.kicad_pcb > board.flux_move_priorities
./build/dev/flux-cli autoroute-kicad board.kicad_pcb board.flux-routed.kicad_pcb
./build/dev/flux-cli route-kicad board.kicad_pcb
./build/dev/flux-cli route-kicad board.kicad_pcb --multisource-steiner
./build/dev/flux-cli route-kicad board.kicad_pcb --atomic-net-trees
./build/dev/flux-cli route-kicad board.kicad_pcb --atomic-net-trees --selective-congestion-reroute
./build/dev/flux-cli route-kicad board.kicad_pcb --atomic-net-trees --selective-congestion-reroute --selective-full-reroute-interval 3
./build/dev/flux-cli route-kicad board.kicad_pcb --max-expansions 10000 --expansion-retry-heuristic-weight-per-mille 10000
./build/dev/flux-cli route-kicad board.kicad_pcb --max-expansions 10000 --expansion-retry-heuristic-weight-per-mille 10000 --reuse-weighted-search-frontier
./build/dev/flux-cli route-kicad board.kicad_pcb --present-congestion-multiplier-per-mille 1300 --max-present-congestion-scale-per-mille 10000 --present-congestion-escalation-start-iteration 4
./build/dev/flux-cli route-kicad board.kicad_pcb --strict-medial --direct-medial
./build/dev/flux-cli route-kicad board.kicad_pcb --terminal-neck-down-width-nm 150000 --max-terminal-neck-down-length-nm 3000000
./build/dev/flux-cli route-kicad board.kicad_pcb --direct-medial --medial-neck-down-width-nm 150000 --max-medial-neck-down-length-nm 3000000
./build/dev/flux-cli route-kicad board.kicad_pcb --corridor-neck-down-width-nm 150000 --max-corridor-neck-down-length-nm 3000000 --max-corridor-neck-down-states 5000000
./build/dev/flux-cli route-kicad board.kicad_pcb --corridor-neck-down-width-nm 150000 --max-corridor-neck-down-length-nm 3000000 --max-corridor-neck-down-states 5000000 --width-reduction-cost-per-grid-step 0
./build/dev/flux-cli route-kicad board.kicad_pcb --max-voronoi-resource-segment-length-nm 1000000
./build/dev/flux-cli detail-kicad board.kicad_pcb
./build/dev/flux-cli detail-kicad board.kicad_pcb --arcs board.curved.kicad_pcb
./build/dev/flux-cli detail-kicad board.kicad_pcb --netclasses board.flux_netclasses
./build/dev/flux-cli detail-kicad board.kicad_pcb --netclasses board.flux_netclasses --diffpairs board.flux_diffpairs --buses board.flux_buses
./build/dev/flux-cli detail-kicad board.kicad_pcb --diffpairs board.flux_diffpairs --tune-differential-pair-skew
./build/dev/flux-cli detail-kicad board.kicad_pcb --diffpairs board.flux_diffpairs --batch-assign-differential-pair-skew
./build/dev/flux-cli optimize-kicad board.kicad_pcb
./build/dev/flux-cli optimize-kicad board.kicad_pcb --placement-move-priorities board.flux_move_priorities
./build/dev/flux-cli optimize-kicad board.kicad_pcb --suggested-placement-move-priorities
./build/dev/flux-cli optimize-kicad board.kicad_pcb --envelopes board.flux_envelopes
./build/dev/flux-cli optimize-kicad board.kicad_pcb board.optimized.kicad_pcb
./build/dev/flux-cli global-optimize-kicad board.kicad_pcb
./build/dev/flux-cli global-optimize-kicad board.kicad_pcb --assignment-global-placement
./build/dev/flux-cli global-optimize-kicad board.kicad_pcb --tentative-envelopes
./build/dev/flux-cli global-optimize-kicad board.kicad_pcb board.optimized.kicad_pcb
./build/dev/flux-cli constrained-optimize-kicad board.kicad_pcb board.constraints
./build/dev/flux-cli constrained-refine-kicad board.kicad_pcb board.constraints board.refined.kicad_pcb
```

`demo` evaluates a small legal routed board and prints a machine-readable objective.
`demo-invalid` deliberately overlaps two courtyards and exits with status 2.
`route-demo` runs the planning-stage global router on the unrouted demo board.
`free-space-demo` runs the net/layer polygonal extraction, conservative integer offset, and
exact-rechecked Voronoi graph pipeline and reports regions, graph topology, withheld curves, and
peak width-dependent lane capacity. Its diagnostics distinguish unsupported obstacles from
identified conservative envelopes used for curved pads, tracks, and vias.
`incremental-route-demo` moves one terminal, reroutes its affected net, and reuses the
compatible path for a second net from the prior global-route checkpoint.
`detail-demo` materializes the global plan as checked physical copper. `--strict-medial` also
requires audited medial transitions and attempts topology-locked packed-lane copper;
`--direct-medial` opts capacity-certified Voronoi chords into PathFinder adjacency and may be
combined with strict mode. JSON distinguishes accepted lane paths, per-path evidence fallback,
and whole-candidate transactional fallback.
`incremental-detail-demo` preserves the stable track object for an unaffected net while
transactionally replacing the changed net and checking the combined board.
`optimize-demo` demonstrates blocking-object geometry and net context producing a useful
placement proposal under a one-move evaluation budget, followed by rerouting and legality
checks.
`orientation-optimize-demo` explicitly enables a 180-degree alternative on a footprint whose
hard allowed set is `{0, 180}`, and shows an offset surface terminal rotating across an
otherwise impassable routing barrier. Rotation and side search are disabled by default, and
run-wide alternatives cannot bypass per-footprint constraints. Side changes remap front/back
pad and owned-keepout layers internally, but source-preserving KiCad export currently rejects
them because a safe flip must rewrite every layer-bearing footprint construct.
`constraint-global-optimize-demo` projects a three-footprint component formed by an intersecting
rigid group and relative constraint onto one analytical translation, legalizes it atomically, and
accepts the seed only after it repairs two blocked routes. JSON reports component counts and one
transaction containing both policy IDs and both companion transforms.
`routing-guided-global-optimize-demo` starts with two routes competing for one sampled lane. HPWL
and density alone have no reason to move the self-contained two-pad footprint; bounded PathFinder
pressure supplies explicit analytical targets, the global placer moves it onto a free lane, and the
seed is accepted only after a fresh route, detailed connectivity, and copper DRC. KiCad users opt in
with `global-optimize-kicad --routing-guided-global-placement`.
`--assignment-global-placement` additionally enables a bounded simultaneous legalization
neighborhood. It can exchange occupied sites or cycle several policy components without exposing
an illegal sequential intermediate placement; JSON reports whether the neighborhood was attempted,
withheld, accepted, or exhausted and the exact candidate/state counts. The resulting board remains
only a global proposal until the integrated routing and DRC gates pass.
`refine-demo` starts from a fully routable placement, moves one unlocked footprint only after a
complete route-cost comparison, and then performs another bounded pass to prove that the
connected, DRC-clean checkpoint is a local optimum. Ordinary repair commands still stop at first
feasibility unless `optimize_routed_quality` is explicitly enabled. Results distinguish
`feasible` copper from optimizer `complete`, so a legal checkpoint remains visible when a pass or
evaluation budget ends before convergence.
`global-optimize-demo` starts with a deterministic analytical placement proposal, exactly
legalizes it, and accepts it only after the same global-routing, detailed-connectivity,
and copper-DRC pipeline improves the feasibility-first score. The corresponding KiCad
command is intentionally separate: ordinary `optimize-kicad` remains an incremental
local-repair workflow unless global placement is explicitly requested.
All KiCad routing and optimization commands accept positive `--max-router-iterations N` and
`--max-expansions N` limits. `--pin-escape-radius N` selects the bounded neighboring-grid search
(zero deliberately restores nearest-node-only access), while positive `--max-pin-access-nodes N`
bounds alternatives per terminal. Results expose total/adaptive access nodes and terminals that
needed the fallback. They make large-board verification effort reproducible; exhausting a limit
remains an incomplete result and never authorizes materialization or export.
`--terminal-neck-down-width-nm N` and `--max-terminal-neck-down-length-nm N` must be supplied
together. Nominal access remains preferred; the explicit smaller width applies only to a bounded
ordinary-net terminal polyline whose nominal-width portal is independently legal. Pair/bus members
are excluded, and detailed output retains the width transition as separately checked straight
segments. This does not authorize arbitrary corridor neck-downs or override unrepresented custom
fabrication rules.

`--medial-neck-down-width-nm N` and `--max-medial-neck-down-length-nm N` must be supplied together
with `--direct-medial`. They authorize an ordinary, uncoupled net to cross only a certified direct
medial chord whose nominal width-plus-clearance demand does not fit, whose narrower demand does fit,
and whose physical length is within the bound. Both snapped portals must independently accept the
nominal width. The selected demand participates in PathFinder cost, overflow, lane packing, route
metadata, incremental reuse, and transactional detailed DRC. Medial junctions, pairs, and buses do
not yet receive variable-width synthesis.

`--corridor-neck-down-width-nm N` and `--max-corridor-neck-down-length-nm N` must be supplied
together. The router constructs a separate shared corridor decomposition using that width and the
largest eligible clearance, then searches `(node, width mode, narrow-run length)` state. A run may
enter only when nominal copper fails, every selected sampled edge passes exact checking at the
narrow width, and the entry portal accepts nominal copper; it must finish within the physical
length bound at another nominal-width portal. Narrow-corridor occupancy/history and capacity are
reported separately. Positive `--max-corridor-neck-down-states N` bounds the expanded graph and
fails closed before search. Differential-pair and bus members, narrow vias, tapers, and arbitrary
width selection remain excluded. KiCad commands automatically enable the shared medial-capacity
model with this policy so overlapping resources observe the selected width. See
[`ADR 0048`](docs/adr/0048-stateful-sampled-corridor-neck-downs.md).

Every explicit-width segment also reports normalized equivalent width loss and contributes the
configured `--width-reduction-cost-per-grid-step N` to global route cost. The default is `100`;
zero keeps width evidence and exact legality checks but disables the preference. Routed-quality
placement compares this cost after all hard feasibility fields and before physical copper length,
then uses narrow segment geometry to propose bounded perpendicular moves for connected unlocked
footprints. Accepted moves still require complete global routing, detailed connectivity, and
copper DRC. This is an optimization preference for routing margin, not an inferred manufacturing
rule; see [`ADR 0050`](docs/adr/0050-width-reduction-cost-and-placement-feedback.md).
`inspect-kicad` reports import loss, unsupported constructs, canonical object counts,
and validation results. Routing, detailed-routing, and optimization commands deliberately
refuse lossy imports. Giving `optimize-kicad` a new output path writes a separately
validated artifact; it never overwrites the source or an existing destination. Export
currently supports footprint position/rotation edits and top-level straight tracks, three-point
circular track arcs, and vias. Optimizer-only orientation constraints can annotate the canonical source and must remain
unchanged in a candidate; they are intentionally excluded from KiCad byte/semantic comparisons
because the file format cannot store them. Side changes and other design changes fail closed
instead of being silently omitted.
`inspect-free-space-kicad` is read-only and may inspect a lossy-but-error-free import. It reports
source strictness separately from whether the selected net/layer can complete conservative
offset and exact-rechecked skeleton construction. `--shared-domain` retains same-net pads and
copper and uses the guarded zero-distance union used by shared allocation resources. Neither mode
authorizes routing or export.

Board schema 7 introduced immutable physical copper-fill polygons. Board schema 9 extends each
fill to an outer contour with exact hole contours and cleanly removes KiCad's documented
oppositely traversed zero-width fracture bridges. The regions participate in validation,
connectivity, internal DRC, global-routing obstacles and attribution, while KiCad remains
responsible for refilling editable zone definitions. Unfilled or ambiguous self-touching fill
geometry is explicitly lossy, and candidate-side zone edits fail closed. Schema 9 also retains
explicit, pad-approximated, or absent placement-envelope provenance; all placement engines fail
closed unless every movable footprint has an explicit source envelope.
Board schema 10 adds explicit ordered physical routing buses. They are never inferred from KiCad
logical bus names or net-name patterns. `buses-template-kicad` emits an empty source-bound sidecar;
users add stable IDs, ordered current net names, and an optional minimum edge gap, then pass it as
`--buses SIDECAR`. Complete bus occupancy aggregates heterogeneous widths and adjacent effective
gaps into shared PathFinder demand and advisory ordered offsets; see
[`ADR 0030`](docs/adr/0030-explicit-ordered-routing-buses.md).
Board schema 11 adds native three-point circular track arcs. Import and source-preserving export
round-trip them without tessellation; connectivity, length, obstacle indexing, internal DRC, and
pair skew/gap checks use analytic arc geometry. `detail-kicad --arcs [OUTPUT]` enables a bounded
post-allocation cubic B-spline pipeline after conventional polyline generation. Adaptive
tessellation is fitted back to native lines/arcs, collision envelopes drive bounded PNS or local
basis-weighted control shoves, and the updated corridor is refitted. Every proposal is quantized,
the complete board is revalidated, and any illegal curved candidate is discarded and retried as
the already legal polyline. Circular graphical primitives remain an explicit import-loss boundary;
see [`ADR 0117`](docs/adr/0117-cubic-bspline-fitting-and-bounded-pns.md).
Board schema 12 preserves exact center/edge geometry for circular KiCad courtyards while using a
separately identified circumscribed polygon only as a conservative placement-search proxy.
`envelopes-template-kicad` emits stable-ID/reference/revision-bound pad-envelope suggestions for
every footprint lacking a source courtyard. Every entry starts `approved no`; `--envelopes`
rejects incomplete, stale, malformed, overlapping, out-of-board, or unreviewed policy. Approved
polygons become optimizer-only authoritative envelopes and may not be changed by export; see
[`ADR 0035`](docs/adr/0035-exact-circular-courtyards-and-reviewed-placement-envelopes.md).
For proposal-first workflows, `global-optimize-kicad --tentative-envelopes` derives conservative
axis-aligned bounds from rotation-independent pad extents and supported Fab/Silk geometry. This is
explicit provisional provenance, not an authoritative courtyard. Result JSON reports the count
and sets `human_placement_review_required: true`; footprint transforms remain preview-only and
must be checked against exact bodies, connectors, assembly, thermal, service, and safety needs
before acceptance. See
[`ADR 0085`](docs/adr/0085-post-process-reviewed-tentative-placement-envelopes.md).
`placement-move-priorities-template-kicad` emits a complete revision/ID/reference-bound move policy.
Its default order is `resistor`, `capacitor`, `transistor`, `other`, `misc_ic`, `mcu`, `soc`, then
`connector`. Prefixes `R`, `C`, `Q`, and `J`/`P` receive the obvious suggestions; `U` defaults only
to `misc_ic`, because a reference designator cannot reliably distinguish an ordinary IC, MCU, and
SoC. Review and override those entries before depending on the escalation order. Passing the file
as `--placement-move-priorities` makes every placement mechanism search cumulative tiers and stop
at the first tier with an improving exactly accepted move. JSON reports tier unlocks, escalation,
per-tier proposals/evaluations/acceptance, and the highest class required by each accepted atomic
move. See
[`ADR 0086`](docs/adr/0086-source-bound-staged-placement-move-priorities.md).
For a proposal-only run, `--suggested-placement-move-priorities` applies that complete
source-bound prefix suggestion in memory and reports `placement_move_priority_review_required:
true`. It cannot be combined with a sidecar and does not establish reviewed MCU or SoC
classification; see
[`ADR 0093`](docs/adr/0093-unique-net-footprint-pressure-and-suggested-priorities.md).
The v0.27 global router keeps a legal nearest grid access unchanged, but if that snap point is
blocked it searches deterministic bounded Chebyshev rings and publishes exact-checked nodes from
the first successful ring. Orthogonal and direct access polylines retain the exact pad endpoint;
static geometry is checked immediately and final mutual clearance/connectivity still require
detailed whole-board acceptance. The bounded Teensy smoke route reduced failed escapes from 43 to
4 and unrouted nets from 66 to 44; see
[`ADR 0036`](docs/adr/0036-bounded-adaptive-exact-pin-access.md).
The v0.28 detailed router partitions topology-complete medial sequences into same-layer packed
runs and joins ordinary signals at exact ordered via anchors selected by the global path. v0.30
extends this to complete differential-pair and ordered-bus groups with matching layer-span
sequences: it deterministically chooses a legal via array from global anchors or adjacent packed
endpoints, preserves bus order, invalidates partial incremental reuse, and accepts the group only
through whole-board connectivity and DRC. Incomplete coupled junction evidence still falls back.
See [`ADR 0037`](docs/adr/0037-uncoupled-multilayer-medial-materialization.md) and
[`ADR 0039`](docs/adr/0039-synchronized-coupled-multilayer-via-arrays.md).
Coupled packed runs now retain bounded access geometry and jointly synthesize ordered fanouts at
physical terminals and internal via boundaries. Direct geometry is preferred; invalid local joins
may trim checked dead tails, alternate cross-first/major-first staircase order, or translate the
complete via array along its longitudinal escape axis. Candidate enumeration is bounded and the
whole group must pass exact pair/bus spacing, skew, obstacle, connectivity, and copper checks; see
[`ADR 0041`](docs/adr/0041-bounded-coupled-run-boundary-fanout.md).
Board schema 6 introduced multiple net classes per net. Each routing property inherits from the
highest-priority assigned class that defines that property, with board rules as the fallback;
unlike-net clearance uses the larger resolved value. These rules drive global obstacle spacing,
detailed copper dimensions, integrated routing feedback, and internal DRC. A standalone
`.kicad_pcb` does not carry the complete modern KiCad project net-class policy, so board-only
import reports zero classes instead of inventing defaults. `plugins/kicad/export_netclasses.py`
uses official `kicad-python` IPC calls to read live project definitions and effective constituent
assignments, verifies the active saved board path through the C++ importer, and emits a strict
source-bound snapshot. Pass that file as `--netclasses SNAPSHOT` to any KiCad route, detail,
optimize, global-optimize, or constrained optimize/refine command. Canonical annotations are
preserved across board export and may not be silently changed.

Board schema 8 infers KiCad-compatible differential pairs from complementary uppercase `P`/`N`
or `+`/`-` suffixes, including trailing underscores and digits. Net-class snapshot version 2
imports preferred pair track width, trace gap, and via gap independently; detailed routing uses
the pair width and global planning reserves the preferred gap. These are routing defaults, not
invented hard limits. `diffpairs-template-kicad` emits a complete source-bound sidecar with `none`
for both hard fields; explicitly set `minimum_gap_nm` and/or `maximum_skew_nm`, then pass it as
`--diffpairs SIDECAR`. Internal DRC measures physical track-edge gap and KiCad-style skew as the
full absolute routed centerline-length difference. Pair annotations must remain unchanged during export and
are excluded only from byte/semantic fields that `.kicad_pcb` cannot store.

Slicer-derived computational geometry is a core Flux design input. PrusaSlicer/Slic3r,
CuraEngine, and the OrcaSlicer/BambuStudio lineage are clean-room behavioral references for
obstacle detours, polygon Boolean operations, clearance offsets, medial/skeletal corridor
discovery, adaptive lane packing, path ordering, simplification, and spatial indices. Flux adapts
those techniques to build net- and layer-specific centreline free space by expanding each obstacle
by `selected trace width / 2 + resolved clearance`, with explicit conservative provenance and
outer/hole topology. Slicer-derived geometry proposes physical corridors and capacities;
PathFinder negotiates net allocation; detailed routing constructs the polyline and may then apply
bounded continuous or native-arc optimization. Quantized whole-board connectivity and copper DRC
remain authoritative. Curves, splines, tapers, and unconstrained variable-width geometry do not
enter the combinatorial graph; only explicit, bounded width modes whose state is carried through
materialization and exact acceptance are eligible. See
[`ADR 0022`](docs/adr/0022-slicer-derived-corridors-pathfinder-and-continuous-traces.md).

v0.21 advances the clean-room slicer-derived routing geometry boundary. Exact obstacle tests
scan-convert conservative free lanes into horizontal/vertical cross-sections; each corridor
resource continues only while its contiguous lane interval is unchanged, so physical bottlenecks
split into explicit neck-down resources with reported lane count, cross-section span, and sampled
medial centerline. An explicit PathFinder term softly prefers lanes nearer that centerline.
PathFinder negotiates edge/node conflicts and aggregate corridor load with present and historical
costs, while CLI results expose corridor demand and capacity evidence. Detailed routing starts
from polylines and applies bounded continuous centerline projection only as an exactly checked
post-allocation optimization. v0.33 checks each quantized two-segment move against represented
physical clearance and deterministically halves blocked proposals, allowing a polyline to shorten
toward an obstacle without relying on whole-route rejection. Attempted, accepted, rejected, and
budget-withheld proposals are observable. v0.25 can then replace eligible corners with native
circular fillets without adding arc state to PathFinder. Exact circular Minkowski offsets,
junction-aware
adaptive neck-down, coupled parallel bus/pair curvature, and curvature-aware continuous
optimization remain later milestones. See
[`ADR 0022`](docs/adr/0022-slicer-derived-corridors-pathfinder-and-continuous-traces.md).
The exact continuous line search is specified by
[`ADR 0042`](docs/adr/0042-bounded-exact-continuous-backtracking.md).
PathFinder reroute ordering uses prior-pass physical congestion without splitting coupled groups;
see [`ADR 0043`](docs/adr/0043-pressure-aware-group-preserving-reroute-order.md).
The independent Boost.Polygon segment-Voronoi service now provides a pitch-independent advisory
graph with exact-rechecked chords, endpoint degree/junction topology, local boundary clearance,
and width-dependent lane capacity. v0.35 can opt accepted unique chords into the PathFinder search
as direct physical-length transitions after separately certifying their grid-quantized portals.
Their aggregate congestion remains width based, multiple packed nets do not create a false
single-lane edge conflict, and congested medial segments feed placement repair targets. An
exhausted hybrid connection receives one reported, separately bounded sampled-graph retry so the
opt-in adjacency does not reduce baseline feasibility. Terminal
portals, vias, ordinary fallback edges, and some junction access remain sampled, so this is a
hybrid rather than a fully gridless production graph. See
[`ADR 0024`](docs/adr/0024-integer-segment-voronoi-free-space-skeleton.md) and
[`ADR 0044`](docs/adr/0044-direct-medial-pathfinder-transitions.md).
v0.36 adds an independently bounded Arachne-inspired width transition at ordinary-net terminals.
It tries nominal geometry first, exact-checks a configured narrow access, requires a nominal-width
portal, and carries the selected width through equal-width polyline simplification and transactional
detailed DRC. Coupled pair/bus access, tapered copper, and width changes inside arbitrary corridors
remain future work; see
[`ADR 0045`](docs/adr/0045-bounded-terminal-neck-down-access.md).
v0.37 extends explicit width state to bounded exact direct-medial bottlenecks. PathFinder charges
the narrower physical demand only on a selected chord that cannot carry nominal demand, while both
transition portals remain nominal-width legal. The detailed route emits the mixed-width polyline
and accepts it only through whole-board connectivity and copper DRC. Coupled nets, sampled-corridor
width variation, tapers, and junction-aware transitions remain future work; see
[`ADR 0046`](docs/adr/0046-bounded-direct-medial-neck-downs.md).
v0.38 lets the continuous centerline optimizer operate on these mixed-width polylines without
changing width policy. Only vertices between equal-width, same-layer segments may move; every
terminal, via, and width transition stays fixed, and the complete result still requires exact
connectivity and DRC or falls back to the discrete mixed-width route. Width-aware pull-tight,
topology-locked replacement, arcs, and tapered transitions remain future work; see
[`ADR 0047`](docs/adr/0047-width-aware-continuous-refinement.md).
v0.39 adds bounded width state to the sampled production graph. A second conservative
narrow-width decomposition supplies realistic cross-section capacity, while PathFinder allocates
multi-edge narrow runs subject to exact entry/exit gates and a physical length limit. Selected
width provenance survives deterministic detailed materialization and exact incremental reuse.
Coupled transitions, narrow vias, and tapers remain future work; see
[`ADR 0048`](docs/adr/0048-stateful-sampled-corridor-neck-downs.md).
v0.40 unifies explicit-width demand with the optional shared medial allocator. Each mapped
resource/net allocation retains its effective physical envelope; PathFinder cost, overflow,
history, advisory packing, reporting, and incremental reuse consume the same value. A nominal
sampled use conservatively dominates a narrow sampled use on the same resource, while exact
direct chords retain their certified override. Flood labels wider than a locally exact-legal
sampled edge are withheld rather than creating false adjacent-channel overflow. Coupled width
changes and longitudinal resource splitting remain future work; see
[`ADR 0049`](docs/adr/0049-width-aware-shared-resource-demand.md).
v0.41 makes authorized width reduction an observable routed-quality objective. A normalized
equivalent-loss metric and configurable grid-step cost are reconstructed identically for fresh
and reused paths. Integrated placement ranks this preference behind every hard feasibility gate
but ahead of physical length, and explicit-width path geometry supplies bounded spatial proposals
with retained net provenance. Zero cost restores ordinary length ordering without suppressing
mixed-width evidence; see
[`ADR 0050`](docs/adr/0050-width-reduction-cost-and-placement-feedback.md).
v0.42 extends deterministic 45-degree pull-tight to mixed-width polylines. Only same-layer corners
whose adjacent resolved widths agree are eligible; the point and width vectors change together,
and every terminal, via, and unequal-width boundary remains fixed. Accepted path/corner metrics
are explicit, while any complete-board validation, connectivity, or DRC failure retries the
unchanged orthogonal mixed-width polyline; see
[`ADR 0051`](docs/adr/0051-width-aware-45-degree-pull-tight.md).
v0.43 extends the optional post-polyline fillet pass to mixed-width paths. A corner is eligible
only when its incident resolved widths agree, and the native circular arc inherits that local
width while every unequal-width transition remains fixed. Checkpoint reuse recognizes only
currently authorized nominal/selected arc widths, and any illegal curved board still falls back
atomically to the legal mixed-width polyline; see
[`ADR 0052`](docs/adr/0052-width-aware-post-polyline-circular-fillets.md).
v0.44 adds an opt-in atomic pairwise-swap neighborhood to integrated placement/routing. Partners
follow the existing deterministic routing-pressure order and a per-footprint bound; both
positions change on one board copy, both routing cones are unioned for incremental screening, and
the accepted candidate still requires a fresh full route, detailed connectivity, and copper DRC.
Move provenance records both footprint IDs and both position changes; see
[`ADR 0053`](docs/adr/0053-atomic-route-guided-footprint-swaps.md).
v0.45 adds first-class source-bound placement groups and opt-in atomic rigid translation. An empty
template refuses to infer functional grouping; edited sidecars bind every member by stable ID and
reference, reject overlap, and become hard source-relative optimizer policy. Route-ranked common
translations union every member's affected nets, structurally pre-screen illegal targets, and
still require a fresh full route, connectivity, and copper DRC. Accepted moves report the group ID
and every companion transform; see
[`ADR 0054`](docs/adr/0054-source-bound-placement-groups-and-atomic-translation.md).
v0.46 adds first-class source-bound placement regions. Empty templates infer no partitions;
designer declarations bind stable footprint IDs and current references to simple world polygons.
Canonical validation requires every member's complete transformed placement envelope to remain
inside its region, and the global legalizer uses the same gate before accepting a site. Local
moves, rotations, side changes, swaps, rigid groups, and global seeds all fail before routing when
they cross a region boundary. Candidate export preserves the policy exactly; see
[`ADR 0055`](docs/adr/0055-source-bound-placement-regions.md).
v0.47 adds source-bound relative-placement tolerance pairs between rigid groups and unrestricted
movement. The sidecar binds two stable footprint IDs and references, captures their nominal vector
from the immutable source, and permits only inclusive reviewed X/Y deviations. Canonical validation
and the global legalizer reject separation before routing; multiple pairs may form a constraint
graph, while duplicate unordered pairs fail closed. Candidate export retains policy exactly; see
[`ADR 0056`](docs/adr/0056-source-bound-relative-placement-tolerances.md).
v0.48 retains bounded exact Pareto checkpoints alongside the active integrated-optimizer result.
Only fresh full-route, detailed, connectivity, and DRC-clean boards qualify; incremental screens
cannot enter the archive. Dominance spans width loss, materialized length, route cost, side and
rotation changes, and displacement. Deterministic bound pruning and discarded-candidate metrics
keep memory and provenance explicit; see
[`ADR 0057`](docs/adr/0057-bounded-exact-pareto-checkpoint-archive.md).
v0.49 adds opt-in atomic translations of complete relative-constraint components. A least
fixed-point closure follows both endpoints of every touching relative constraint and every member
of every touching rigid group. Route-guided common translations preserve all internal offsets,
union every member's routing cone, and require structural pre-screening plus a fresh full route,
connectivity, and copper DRC. Locked members immobilize the closure, and accepted moves report all
relation IDs, group IDs, and companion transforms; see
[`ADR 0058`](docs/adr/0058-atomic-relative-constraint-closure-translations.md).
v0.50 makes global analytical placement aware of the same explicit-policy components. Per-member
attraction and density proposals are projected onto one bounded common translation; one locked
member propagates immobility. Components are ordered deterministically by total courtyard area and
stable ID, and each ring proposal moves every member on one board copy before canonical validation.
An accepted global seed reports component metrics and one provenance-rich transaction, and still
requires fresh global routing, detailed connectivity, and copper DRC; see
[`ADR 0059`](docs/adr/0059-constraint-component-global-placement.md).
v0.51 feeds current global-router evidence into an opt-in analytical placement pass. Congested
sampled or medial resources, selected width reductions, and clearance-aware obstacle escapes become
bounded, deduplicated footprint-anchor guides with positive weight. Constraint projection and exact
legalization remain authoritative, while a guided seed still needs a fresh full route, detailed
connectivity, copper DRC, and feasibility-first improvement. Metrics expose guide and guided-
footprint counts; see
[`ADR 0060`](docs/adr/0060-routing-pressure-guided-global-placement.md).
v0.52/schema 16 extends source-bound relative placement with optional radial-distance and circular
relative-rotation tolerances. Version-2 sidecars still bind stable endpoint IDs/references and
capture nominal values from the immutable source; version 1 remains compatible with neither new
gate enabled. Exact inclusive integer-nanometre and wraparound microdegree checks are conjunctive
with X/Y limits across every placement and export path; see
[`ADR 0061`](docs/adr/0061-source-bound-relative-distance-and-rotation.md).
v0.53 adds opt-in bounded simultaneous component-site legalization. Rigid groups and intersecting
relative-placement closures become indivisible atoms; deterministic analytical targets, occupied
source sites, and bounded ring candidates form a small assignment neighborhood. Exact partial
geometry prunes assigned atoms while canonical whole-board validation accepts complete leaves.
The existing greedy result remains the incumbent, search-state exhaustion is observable, and only
the candidate with lower analytical-target distance/source displacement is selected before the
ordinary fresh global-route, detailed-connectivity, and copper-DRC gate; see
[`ADR 0062`](docs/adr/0062-bounded-simultaneous-component-site-legalization.md).
v0.54 corrects spatial differential-pair skew to KiCad's full member-length difference rather than
the former half-difference interpretation; see
[`ADR 0063`](docs/adr/0063-kicad-full-difference-skew-semantics.md). v0.55 adds an opt-in bounded
post-route tuner for hard spatial pair skew. Its v0.55 baseline considers newly materialized
unlocked axis-aligned segments on the shorter member and tries deterministic rectangular polyline
detours, and accepts one only after fresh validation, connectivity, and whole-board copper DRC.
Metrics report pair/candidate attempts, tuned or withheld pairs, added centerline length, and budget
exhaustion. Curves remain an independent later refinement and do not enter PathFinder; see
[`ADR 0064`](docs/adr/0064-bounded-transactional-polyline-skew-tuning.md).
v0.56 extends that transaction to bounded multi-period accordion patterns. It computes physical
longitudinal capacity from trace pitch, distributes the required half-length correction evenly
within a per-period amplitude limit, and tries stable same-side and alternating layouts. Every
candidate chord receives a polygonal free-space union-obstacle clearance certificate before the
existing exact whole-board gate. Metrics expose assessed or withheld regions, free-space
rejections, maximum period capacity, and materialized periods; see
[`ADR 0065`](docs/adr/0065-free-space-certified-multi-period-skew-tuning.md).
v0.57 adds opt-in conflict-aware batch assignment across those certified pair candidates. Candidate
generation remains local and bounded, while a stable exact branch search selects at most one region
per pair, rejects combined copper conflicts, maximizes repaired pair count, and atomically commits
the best checked incumbent. Geometry, per-pair retention, and assignment states have independent
bounds and observable exhaustion; see
[`ADR 0066`](docs/adr/0066-conflict-aware-batch-skew-tuning-region-assignment.md).
v0.58 extends the same certified pipeline to non-axis-aligned straight copper. Each meander is
constructed in a canonical tangent/normal frame, independently quantized to integer nanometres,
and rejected on collapsed or unrepresentable geometry. Reported added length is measured from the
accepted quantized net rather than inferred from ideal amplitude; polygonal free-space chords and
canonical validation, connectivity, full skew, and whole-board DRC remain authoritative in both
greedy and batch modes. Separate metrics expose assessed/materialized any-direction regions and
quantization withholding; see
[`ADR 0067`](docs/adr/0067-quantized-any-direction-skew-tuning.md).
v0.59/schema 17 represents the board as one exact outer polygon minus ordered disjoint cutouts.
Board- and transformed footprint-owned `Edge.Cuts` contribute to one endpoint-connected contour
assembly; enclosed contours become physical voids, while disjoint, touching, nested, or
overlapping ambiguity fails closed. Shared geometry, validation, placement legalization, copper
DRC, PathFinder cells/edges, slicer-style integer offsets, and Voronoi boundaries all use the same
holed domain. Cutout edges carry board-edge clearance, inward free-space offset expands holes, and
source-preserving export requires exact semantic equality. Footprints contributing board edges are
conservatively locked because candidate export cannot yet move the board-area model; see
[`ADR 0068`](docs/adr/0068-exact-board-cutouts-and-holed-free-space.md).
v0.60/schema 17 adds a source-bound live KiCad preview and routing transaction boundary. The C++
engine accepts only strict source and candidate boards when re-export from the exact source
reproduces the candidate bytes, then emits deterministic full source inventories, footprint
transforms, and exact track/arc/via operations. The thin Python adapter requires the active board
path and every supported live object to match, repeats the check inside one commit, verifies KiCad's
returned objects, and never saves. Placement transforms remain visible in preview but cause live
apply to fail closed because current upstream whole-footprint updates can damage schematic linkage;
the verified candidate artifact remains the placement handoff. See
[`ADR 0069`](docs/adr/0069-source-bound-kicad-ipc-change-sets.md).
v0.61/schema 17 adds a deterministic Specctra DSN/SES comparison boundary. DSN export writes
ordered copper layers, simple board/fill/keepout geometry, footprint images and placement,
side-correct padstacks, resolved net classes, and protected straight source copper without
overwriting an artifact. SES import first reproduces the exact DSN bytes from the strict source,
then requires unchanged complete placement and protected routing before normalizing new straight
tracks/vias and passing structural, connectivity, copper-DRC, and source-preserving KiCad export
gates. Current freerouting v2.3.0 established resolution 10 as the usable 100 nm session contract;
resolution 1000 failed its expansion-room construction on the same fixture. Arcs, cutouts, holed
fills, unsupported pads, non-routing keepout semantics, coupled constraints, and placement exchange
remain explicit blockers. See
[`ADR 0070`](docs/adr/0070-source-bound-specctra-dsn-ses-interoperability.md).
v0.62/schema 17 adds opt-in congestion-aware multi-source growth for multi-terminal nets. At each
step one existing bounded PathFinder search starts from every remaining exact-checked terminal
portal and ends at any node in the current routed tree, so KiCad pad order no longer fixes the
Steiner topology. Original terminal indices remain stable connection identities. An incomplete
multi-source net receives one transactional fixed-order retry against identical prior occupancy;
a complete retry wins, while two incomplete attempts retain the branch-richer result. Only the
first branch materializes the root terminal stub, and later branches join existing tree copper.
Metrics expose selected/reordered connections, deferred inaccessible terminals, fallbacks, and
complete-net recoveries. The policy remains off by default and does not bypass global completion,
detailed connectivity, or DRC gates; see
[`ADR 0071`](docs/adr/0071-multisource-steiner-tree-growth.md).
v0.63/schema 17 adds opt-in atomic capacity ownership for complete net trees. Each net routes from
a checkpoint of current PathFinder occupancy and committed result state. If any terminal remains
unconnected after the configured topology fallbacks, every speculative branch of that net is
rolled back before the next net routes; complete trees retain identical behavior. Unrouted-net,
obstacle, and fallback evidence remains observable, while committed paths, cost, multi-source
connections, and present edge/node/corridor demand describe only complete trees. Rollback metrics
report discarded trees, paths, and search cost. The policy does not invent speculative history and
remains off by default; see
[`ADR 0072`](docs/adr/0072-atomic-complete-net-capacity-ownership.md).
v0.64/schema 17 distinguishes an A* invocation that consumes its expansion ceiling with a live
frontier from one that exhausts the reachable graph. An opt-in weighted-A* recovery retries only
the former once under the identical expansion ceiling and physical/negotiated costs, changing
queue order from `g + h` to `g + weight * h`; the returned route still reports its unweighted
actual `g` cost. Metrics expose all invocations, valid-state expansions, limited searches, retries,
and recoveries, including work from discarded topology attempts and atomically rolled-back nets.
The ordinary minimum-cost search always wins when it completes within budget. Retry weights are
bounded to `(1, 10]`, expressed in per-mille, and disabled by default; see
[`ADR 0073`](docs/adr/0073-selective-weighted-astar-recovery.md).
v0.65/schema 17 adds opt-in bounded PathFinder present-cost escalation. A configurable warm-up
leaves early passes unchanged, then scales sampled, corridor, and shared-Voronoi present
congestion penalties from their immutable base values while history, physical costs, and exact
legality remain unchanged. The scale is deterministic, saturating, capped, and disabled by a
default 1.0 multiplier. Results distinguish executed passes from the selected best pass and expose
the selected scale plus all three effective costs. Feasibility-first checkpoint selection can
therefore retain an earlier, more connected route when later pressure is counterproductive; see
[`ADR 0074`](docs/adr/0074-bounded-pathfinder-present-cost-escalation.md).
v0.66/schema 17 adds opt-in bounded live-frontier continuation for expansion-limited searches and
stable `unrouted_net_ids` JSON observability. After ordinary A* reaches its ceiling, the router
retains valid labels and parents, re-keys the live OPEN set with the configured weighted heuristic,
and explores one additional bounded phase. It still runs the established from-scratch weighted
retry and always selects that route when available; continuation is used only when restart fails.
Metrics distinguish initialized searches, total expansions, continuations, continuation-only
recoveries, retained states, and second-phase limits. Default routing remains unchanged, and the
policy makes no optimality claim; see
[`ADR 0075`](docs/adr/0075-bounded-live-frontier-weighted-continuation.md).
v0.67/schema 17 adds an opt-in dimensioned checkpoint objective for shared-medial routing. The
default continues to rank passes by unrouted nets, aggregate overflow, and actual cost. With
`--physical-overflow-objective`, a shared-capacity route instead ranks exact total and peak
Voronoi overflow in nanometres before separate sampled edge/node conflicts, corridor-lane excess,
congested physical-resource count, and cost. Route and integrated JSON publish the configured
policy plus every split dimension. The option requires `--strict-medial` or `--direct-medial`,
does not raise sampled capacity, and does not relax the zero-aggregate-overflow materialization
gate; see
[`ADR 0076`](docs/adr/0076-dimensioned-physical-overflow-checkpoints.md).
v0.68/schema 17 adds opt-in magnitude-aware history for shared physical resources. With
`--physical-overflow-history`, an overfull resource adds
`ceil(excess_nm / minimum_allocatable_envelope_nm)` history units instead of one. The quantum is
the same conservative minimum width-plus-clearance envelope already used to derive medial lane
capacity, so heterogeneous physical demand is never added directly to abstract route cost. JSON
publishes the policy, quantum, total increment, and peak increment for the selected pass. The
default remains unchanged, shared allocation is required, and all zero-overflow detailed-routing
gates remain authoritative; see
[`ADR 0077`](docs/adr/0077-quantized-physical-overflow-history.md).
v0.69/schema 17 adds opt-in least-flexibility-first scheduling for heterogeneous physical routing
rules. With `--physical-envelope-ordering`, equally prioritized groups with the same completion
status are ordered by decreasing allocator-consistent nominal occupied demand before dynamic
PathFinder pressure. Singletons use resolved `track_width + 2 * clearance`; complete buses and
differential pairs reuse their physical internal gaps and external margins. Declared group order
remains atomic, explicit priority and unrouted recovery remain stronger, and the default order is
unchanged. Route and integrated JSON publish the configured policy plus peak single-net and group envelopes; see
[`ADR 0078`](docs/adr/0078-physical-envelope-reroute-ordering.md).
v0.70/schema 17 adds opt-in selective conflict-cone rerouting. With
`--atomic-net-trees --selective-congestion-reroute`, each incomplete pass retains revalidated
complete net trees that touch no overfull sampled, corridor, controlled-width-corridor, or shared
physical resource, then reroutes only unrouted and conflicted nets. Affected differential pairs
and routing buses expand atomically to the complete declared group. Caller-visible incremental
reuse provenance remains separate from internal pass retention, ordinary-corridor overflow now
invalidates checkpoint reuse correctly, and route/integrated JSON reports bounded cumulative
retention, conflict, pass, and group-expansion observations. The default whole-pass route remains
unchanged, and zero-overflow detailed materialization, connectivity, and DRC gates remain
authoritative; see
[`ADR 0079`](docs/adr/0079-selective-conflict-cone-rerouting.md).
v0.71/schema 17 makes `--physical-overflow-objective` consistent across routing and integrated
placement. Integrated candidate acceptance now preserves total and peak physical excess,
sampled edge/node overflow, corridor-lane overflow, and congested physical-resource count as
separate ordered dimensions instead of collapsing them back into aggregate overflow. Physical-
objective footprint ordering consumes the same resource families and routing-guided moves gain
ordinary plus controlled-width corridor targets. Accepted moves publish separate sampled,
corridor, and physical pressure counts, and a pure C++ feedback API exposes deterministic
deduplicated obstacle/net provenance for future KiCad UI explanation. Aggregate mode retains its
established ordering, and aggregate zero overflow, detailed connectivity, and exact DRC remain
mandatory; see
[`ADR 0080`](docs/adr/0080-dimension-consistent-integrated-routing-feedback.md).
v0.72/schema 17 adds stable, saturating, unit-separated per-net and deduplicated group PathFinder
pressure diagnostics. Route and integrated JSON report run-wide peak sampled and corridor excess,
total and peak physical excess in nanometres, and congested physical-resource participation. A
pure comparator supports bounded experiments, but production reroute scheduling deliberately keeps
ADR 0043's legacy resource-participation order: the equal-budget Teensy experiment remained
connected but regressed most congestion and effort dimensions, so dimensional serial scheduling
was rejected rather than enabled. The compiled unit inventory is also consolidated from 122 passed
feature cases to six high-risk invariants; CLI, import/export, external DRC, and board acceptance
coverage remain at their integration layers. See
[`ADR 0081`](docs/adr/0081-dimensioned-pathfinder-reroute-pressure.md) and
[`ADR 0082`](docs/adr/0082-risk-focused-minimal-unit-suite.md).
v0.73/schema 17 adds opt-in longitudinal ownership for ordinary shared medial resources. A
positive `--max-voronoi-resource-segment-length-nm N` divides a long skeleton edge into the
fewest deterministic local intervals bounded by `N`, while preserving width-valued capacity,
stable topology, the hard resource budget, and all detailed connectivity/DRC gates. Exact direct
medial chords remain atomic because their snapped-chord certificate and PathFinder transition are
atomic. Route and integrated JSON report the configured limit, divided source count, additional
interval count, and retained atomic direct-resource count. The equal-budget Teensy experiment
reduced total physical overflow 13.5% but worsened aggregate overflow, peak physical overflow, and
search effort, so the zero setting remains the default; see
[`ADR 0083`](docs/adr/0083-longitudinally-segmented-medial-capacity.md).
v0.74/schema 17 adds the routing-only `autoroute-kicad` MVP. Its versioned `autoroute_v1` preset
combines shared free-space capacity, direct medial chords, dimensioned physical checkpoint
selection/history, atomic complete-net ownership, selective conflict rerouting, physical-envelope
ordering, and bounded weighted-frontier retry without enabling neck-downs, arcs, or placement.
The Python launcher adds deterministic change-set preview, optional live net-class capture,
independent KiCad CLI DRC, and explicit unsaved routing apply. A distinct new output path and every
existing detailed connectivity/DRC/export gate remain mandatory; see
[`ADR 0084`](docs/adr/0084-minimal-usable-kicad-autoroute-workflow.md).
v0.75/schema 18 adds explicitly provisional placement geometry for post-process-reviewed
integrated placement. `--tentative-envelopes` bounds missing courtyards from pads plus supported
Fab/Silk lines, rectangles, polygons, arcs, and circles with a review margin. Default placement
still rejects provisional provenance; explicit opt-in preserves all exact placement/routing/DRC
gates and reports mandatory human review. Tentative polygons are not serialized into KiCad, and
live footprint application remains withheld; see
[`ADR 0085`](docs/adr/0085-post-process-reviewed-tentative-placement-envelopes.md).
v0.76/schema 18 adds optional source-bound staged placement-move priorities. A deterministic
template completely classifies the current source and supplies the connector-last default order.
Integrated local, orientation, swap, group, relative-component, and global-placement proposals
unlock classes cumulatively, require every member of an atomic transaction to be eligible, and
restart from the first tier after acceptance. Existing workflows remain unrestricted when no
policy is supplied, and locks plus exact legalization/routing/connectivity/DRC remain authoritative;
see [`ADR 0086`](docs/adr/0086-source-bound-staged-placement-move-priorities.md).
v0.77/schema 18 removes two practical proposal-first bottlenecks without weakening final
acceptance. Integrated candidate validation now reuses fully source-certified saved-zone topology
only while outline, cutouts, and complete zones remain exactly unchanged, and reports every reuse.
Explicit tentative-envelope global placement may begin from a narrow whitelist of repairable
overlap, board-crossing, pad-crossing, and footprint-keepout defects, but every published proposal
still passes complete canonical validation. An exact AABB broad phase rejects distant polygon
pairs before the unchanged collision predicate. On the read-only Teensy target the formerly
60-second-timeout candidate loop completes 200 evaluations in 12.7 seconds; the global legalizer
then reaches routing of its legal seed, where corridor construction against 127 saved-fill holes
is the next measured scalability boundary. No complete Teensy route is claimed; see
[`ADR 0087`](docs/adr/0087-source-certified-static-zone-validation-reuse.md) and
[`ADR 0088`](docs/adr/0088-tentative-source-global-legalization.md).
v0.78/schema 18 adds an exact spatial broad phase for routing against saved copper-fill holes.
Inclusive integer hole bounds reject impossible point queries, while a conservative
segment-AABB/hole-AABB distance lower bound with the two-nanometre guard withholds only provably
distant contour calculations. Every possible contact still uses the original polygon predicate;
materialization and copper DRC remain unchanged. Route and integrated JSON expose broad-phase
rejections and exact checks. The read-only Teensy one-expansion result remained identical while
runtime fell from 2.86 to 1.17 seconds, and the integrated tentative-placement probe now completes
all 200 evaluations in 35.0 seconds, accepts a globally legalized zero-placement-violation seed,
and reaches routing. Its artificial one-expansion ceiling leaves all 110 routed nets incomplete,
so no complete route is claimed; see
[`ADR 0089`](docs/adr/0089-indexed-saved-zone-hole-routing-queries.md).
v0.79/schema 18 adds explicit positive placement-pass and complete candidate-evaluation limits to
placement-capable KiCad commands and echoes their resolved values in JSON. Failed terminal escapes
now retain stable footprint/pad/net IDs plus exact positions, and integrated placement ranks that
evidence together with every unrouted net instead of losing unattributed failures. On the legal
tentative Teensy seed, a 100,000-expansion comparison remains at eight unrouted nets with four
failed escapes. The already explicit 0.15 mm / 3 mm terminal neck-down policy removes those four
failures and leaves four unrouted nets, but still has negotiated overflow; detailed copper and
export remain withheld. See
[`ADR 0090`](docs/adr/0090-bounded-integrated-evaluation-and-pin-escape-feedback.md).
v0.80/schema 18 preserves exact selected-checkpoint connection failures: attempted and connected
terminal IDs/positions, failure reason, multisource/direct-fallback provenance, and bounded search
work. Pin access now rejects a legal-but-isolated portal and continues its deterministic ring
search, including only explicitly configured nominal or narrow egress transitions. Exact pin or
no-access failures also receive a bounded routing-pitch placement stencil before broad congestion
targets.
On the legal tentative Teensy seed this resolves three remaining U20 nets; extending the explicit
0.15 mm terminal access bound from 3 mm to 6 mm produces an all-net abstract plan. Twelve
PathFinder passes reduce it to 523 aggregate conflicts and 49.9 mm physical overflow, so detailed
copper and export remain correctly withheld. See
[`ADR 0091`](docs/adr/0091-egress-certified-pin-access-and-connection-failure-evidence.md).
v0.81/schema 18 adds opt-in periodic full rerouting within selective negotiation. With
`--atomic-net-trees --selective-congestion-reroute --selective-full-reroute-interval N`, every
Nth pass transition discards active whole-net reuse while preserving PathFinder history, stable
pressure ordering, and the best feasibility-first checkpoint. Route and integrated JSON report
the configured interval, refresh count, and deterministic selected-checkpoint per-net pressure
without mixing lane counts and nanometres. On the same twelve-pass read-only Teensy experiment,
an interval of three improves aggregate overflow from 523 to 492 and total physical overflow from
49.86 mm to 47.87 mm; a no-retention ablation reaches 478 aggregate overflow but 48.94 mm physical
overflow. All remain abstract over-capacity plans, so copper and export stay withheld. See
[`ADR 0092`](docs/adr/0092-periodic-full-reroute-refresh.md).
v0.82/schema 18 corrects integrated footprint pressure so a congested net contributes once per
footprint regardless of repeated same-net pads, and publishes the stable ranked footprint report
in integrated JSON. `--suggested-placement-move-priorities` can explicitly apply the existing
complete prefix-derived template in memory for a preview run. It remains absent by default,
requires post-process classification review, keeps every U-prefixed part in `misc_ic`, and cannot
be combined with a reviewed sidecar. On Teensy it proves resistor-first evaluation—16 local
candidate evaluations stay in the resistor tier—but accepts no local move and leaves the route
unchanged, while increasing runtime substantially. A routing-pitch displacement experiment was
also rejected after producing identical geometry at twice the runtime. See
[`ADR 0093`](docs/adr/0093-unique-net-footprint-pressure-and-suggested-priorities.md).
v0.83/schema 18 turns explicitly selected narrow-route geometry into bounded placement evidence.
Every non-nominal terminal, sampled-corridor, or direct-medial segment is rechecked at nominal
width against represented static geometry. Exact object blockers are reported separately from
search pressure; a movable owning footprint may receive a perpendicular displacement proposal,
while board boundaries and immutable board copper remain diagnostic-only. Every proposal still
passes placement policy, fresh global routing, detailed connectivity, and copper DRC before
acceptance. On the read-only Teensy experiment, enabling the existing 0.15 mm sampled-corridor
neck-down policy reduces physical overflow from 53.62 mm to 35.94 mm without losing any
connection. The 126 nominal-width blocker records produce no eligible independent footprint move,
so no local component move is fabricated and detailed copper remains withheld. See
[`ADR 0094`](docs/adr/0094-route-selected-neck-down-obstacle-placement-feedback.md).
v0.84/schema 18 adds opt-in bounded multi-ring terminal access diversity. The default still stops
at the first egress-certified Chebyshev ring; `--diversify-pin-access-rings` retains later
exact-legal access/egress portals until `--max-pin-access-nodes N`, preserving stable
ring/distance/layer/coordinate order. Expert KiCad workflows can also set the already supported
router lattice through `--grid-pitch-nm N`; resolved copper spacing and the grid-size budget
remain hard gates. Planar routing now defaults to 0.5 mm, while an omitted via-site pitch is
independently rounded up to the smallest rule-safe multiple of that lattice. `autoroute_v1`
rejects both controls. Checkpoints are invalidated when
either lattice pitch or access-diversity policy changes. On Teensy, the established 1 mm grid,
three-ring radius, 64-node cap, and twelve passes retain every connection while reducing physical
overflow from v0.83's 35.94 mm to 8.65 mm and aggregate overflow from 482 to 181. Detailed copper
is still withheld because neither overflow is zero. See
[`ADR 0095`](docs/adr/0095-bounded-multi-ring-pin-access-diversity.md).
v0.85/schema 18 carries the source-object provenance of each conservative shared-channel width
into routing and integrated placement. JSON reports the exact congested resource chord, physical
demand/capacity/overflow, routed nets, and represented pad, keepout, fill, or copper object that
bounds it. Eligible owners receive a bounded perpendicular placement proposal, and all direct
channel-owner proposals in the active source-bound priority wave are screened before generic
neighborhoods. Exact placement policy, fresh full routing, detailed connectivity, copper DRC, and
human review remain mandatory. On the unchanged Teensy route, attribution preserves the exact
v0.84 objective vector and identifies source owners for 14 of 20 congested resources; the other
six remain correctly boundary-only or unmatched. Eight 5 mm owner moves are screened under a
ten-candidate repair budget, but none passes full routed acceptance. Repeating the same bounded
experiment at 1 mm also accepts no move and retains the identical checkpoint while taking much
longer. The prohibition on detailed copper/export therefore remains unchanged. See
[`ADR 0096`](docs/adr/0096-source-attributed-physical-channel-placement-repair.md).
v0.86/schema 18 adds an opt-in one-based hard-capacity start pass. Earlier passes retain ordinary
PathFinder negotiation; later passes constructively reject sampled edge/node, selected corridor,
and selected-width shared physical overuse while preserving the best feasibility-first checkpoint
across both phases. Direct medial edges keep only their sampled-portal exemption, the explicit
strict-pass budget is not truncated by soft stagnation, and JSON separates all three rejection
categories. On Teensy, six strict passes reject more than one million sampled transitions and
prove the policy active, but no strict ordering connects every net. The v0.85 soft checkpoint
therefore remains the project best; detailed copper and export are still withheld. See
[`ADR 0097`](docs/adr/0097-bounded-late-hard-capacity-negotiation.md).
v0.87/schema 18 makes constructive failure actionable. Each failed net reports the nets actually
occupying its rejected sampled, selected-corridor, and over-capacity physical transitions; the next
strict schedule uses that directed evidence inside the existing stronger priority/group tiers.
Optional positive attempt/victim bounds enable same-pass singleton rip-up with complete occupancy,
selected-demand, path, and cost removal, retry-first ordering, and mandatory victim rerouting.
Reused and coupled nets remain ineligible, while JSON retains every pass, blocker pair, victim,
retry, and end-of-pass result. On Teensy, bounded two-victim repair completes 70 immediate retries
and reduces the best strict deficit from 24 to 22 nets, but still cannot produce materializable
copper. See
[`ADR 0098`](docs/adr/0098-strict-capacity-blocker-guidance-and-bounded-ripup.md).
v0.88/schema 18 optionally retains exact blocker hyperedges from rejected strict transitions and,
under a per-attempt state bound, selects a jointly sufficient eligible victim set by deterministic
coverage/evidence scoring before falling back to v0.87's individual ranking. JSON exposes the
configured/evaluated state bounds, joint attempt count, and withheld evidence. On Teensy, 82 joint
two-net attempts evaluate 3,352 states; 50 immediately recover and 43 remain recovered at pass end.
The run is slightly faster and displaces fewer nets than greedy two-victim repair, but its best
strict deficit remains 22, so detailed copper and export remain prohibited. See
[`ADR 0099`](docs/adr/0099-bounded-joint-hard-capacity-conflict-set-repair.md).
v0.89/schema 18 can transactionally restore an exact pre-rip-up strict-routing state when the
failed net's immediate retry is still incomplete. Unsuccessful victims are removed from the
deferred queue, the failed net remains explicitly unrouted, and successful retries retain v0.88's
mandatory victim rerouting. Attempt and aggregate JSON expose every rollback. On Teensy, 33 failed
retries roll back exactly and the run remains slightly faster, but peak memory grows by about
85 MiB and the best strict deficit remains 22 nets. The policy therefore improves failure
isolation, not route completeness; detailed copper and export remain prohibited. See
[`ADR 0100`](docs/adr/0100-transactional-failed-strict-retry-rollback.md).
v0.90/schema 18 can requeue evidence-selected strict-repair victims at the stable pass tail while
still retrying the failed net immediately. This turns a successful repair into a meaningful
bounded order change rather than immediately recreating the same local conflict; rollback removes
the queued victims when the retry itself fails. On Teensy, the winning 16-attempt/two-victim run
reduces the best strict deficit from 22 to 18 nets. Larger attempt/victim/pass bounds and disabling
rollback all regress, so the measured winner remains bounded and explicit. It still cannot unlock
detailed copper or export. See
[`ADR 0101`](docs/adr/0101-bounded-strict-repair-victim-tail-deferral.md).
v0.91/schema 18 repairs selective same-run checkpoint identity and can invalidate a bounded,
evidence-ranked blocker cone around each failed strict net. Compatible routes outside that cone
remain allocated while the failed nets and their strongest directed blockers are reconsidered;
periodic full refresh remains explicit. On Teensy, a two-blocker cone retains 272 net and 513 path
observations, reduces strict pass sizes as low as 49 of 110 nets, and improves the best strict
deficit from 18 to 17. Retaining that narrow cone through five consecutive focused strict passes
reduces the best deficit again to 16, with later pass sizes as low as 37 nets. One blocker ties
v0.90 and three blockers regress, so the measured policy remains narrow. No pass completes every
net, and detailed copper and export remain withheld. See
[`ADR 0102`](docs/adr/0102-bounded-hard-capacity-blocker-cone-reuse.md).
v0.92/schema 18 can protect each successfully recovered strict-repair target from becoming another
repair's victim during the remainder of the same pass. Protection clears between passes and owns
no capacity exemption; alternate blockers remain eligible. On Teensy, all 72 successful retries
remain routed at pass end with the measured 48-attempt bound, reducing the strict deficit from 16
to 11 nets. A 64-attempt ceiling regresses to 16, and twelve strict passes do not improve the
result, so the winner remains explicitly bounded. No strict pass completes every net; detailed
copper and export remain withheld. See
[`ADR 0103`](docs/adr/0103-pass-local-recovered-repair-protection.md).
v0.93/schema 18 closes two terminal-portal identity errors. Ordinary grid transitions now charge
and, in the constructive phase, reject an occupied initial portal while retaining its actual
blocking nets; certified direct medial edges keep their separate physical-resource semantics. Once
the first path chooses one of a terminal's diversified portals, only that physically materialized
portal remains in the net tree—later branches cannot terminate on unselected phantom access copper.
A requested later hard-capacity phase also runs before soft stagnation may stop the search. The
corrected Teensy replay invalidates an apparent zero-overflow result and truthfully retains all 430
paths with 80 aggregate overflow units rather than passing detailed routing. KiCad empty-name
net-zero pads also remain inventoried without being counted as one open electrical connection. See
[`ADR 0104`](docs/adr/0104-complete-sampled-portal-capacity-accounting.md).
v0.94/schema 18 closes the corresponding integrated-feedback gap. When feasibility-first
selection retains an all-path soft checkpoint, placement repair now still consumes persistent
strict failed nets, directed blocker pressure, and exact attempted terminals from the best
constructive pass. Those terminals receive the bounded fine-move stencil, matching moves expand
their incremental affected cone, and minimum strict unrouted-net count now precedes every soft
overflow dimension in integrated acceptance. The existing staged priority policy still controls
which component class may move. Blocker counts remain advisory discrete pressure: they own no
geometry, capacity, or acceptance. See
[`ADR 0105`](docs/adr/0105-strict-capacity-placement-feedback.md).
On the read-only Teensy placement this rejects a soft-only move that worsens the closest strict
allocation from five to eight nets, then accepts a resistor-tier R74 move that reduces the strict
deficit from five to four while also reducing aggregate and physical overflow. Detailed copper and
export remain withheld until the remaining strict deficit reaches zero.
Long priority waves can resume after a separately certified deterministic prefix with
`--candidate-evaluation-offset N`. The skipped count is explicit and does not consume candidate or
validation metrics. An offset-only run cannot claim a local optimum, and every evaluated
improvement still requires fresh full-route acceptance; see
[`ADR 0106`](docs/adr/0106-bounded-placement-candidate-continuation.md).

v0.95/schema 18 closes an artifact-level gap exposed by the project-owned board progression. A
complete direct-medial PathFinder plan may assign several nets to a physically capacious shared
resource, yet conventional fallback copper cannot place them all on the resource centreline when
the route lacks complete packed-lane topology. `autoroute_v1` now treats that plan transactionally:
if its final detailed candidate fails represented copper DRC after using direct edges, the workflow
discards it and makes one fresh bounded sampled-only route from the immutable source. The retry
keeps the production search budget, weighted recovery, atomic net ownership, selective rerouting,
and present-cost schedule, while capacity-one sampled resources replace shared-medial allocation.
JSON reports the attempt, acceptance, discarded edge count, and discarded violation count. Every
4- through 20-component suite board now exports connected, internally clean copper and passes an
independent KiCad 9 DRC; see
[`ADR 0107`](docs/adr/0107-transactional-sampled-autoroute-recovery.md).

v0.96/schema 18 distinguishes a false skeleton-adjacency break from a real lane-order conflict at
direct-edge grid portals. Each global path now records the exact selected portal between consecutive
resources when one incident direct chord and the other allocated resource provably meet at that
path node. Packed-lane construction maps the portal back to both unsnapped chord ends and proposes
a direct bridge or a quantized shifted-line intersection no farther than one grid pitch from the
portal. Static obstacle checks, mutual spacing between the actual trimmed/extended transition
polylines, connectivity, and whole-board DRC remain hard gates. On the five-board progression this
recovers 83 exact portal transitions and accepts 44 physical bridge proposals, while correctly
rejecting branch permutations whose lanes would cross. `autoroute_v1` now attempts topology-locked
physical lanes before its v0.95 sampled recovery; see
[`ADR 0108`](docs/adr/0108-exact-direct-portal-lane-bridges.md).
v0.97/schema 18 closes the loop between that physical evidence and PathFinder. An exact-rejected
selected portal bridge now adds bounded history only to its net on the incident direct resource at
that snapped portal. Selective reuse invalidates the affected net and its complete pair/bus group;
other nets retain the resource's certified capacity. Zero-overflow checkpoints compare remaining
portal conflicts before route cost, while exact detailed connectivity and copper DRC remain the
acceptance authority. On the owned 4–20 component progression this reduces primary hybrid copper
violations from 18/73/83/222/255 to 16/63/65/196/210. Every final artifact still uses the
transactional sampled recovery and passes internal connectivity/DRC; see
[`ADR 0109`](docs/adr/0109-negotiated-direct-portal-lane-conflicts.md).
v0.98/schema 18 extends that negotiation from direct portals to every exact-rejected selected
medial junction. Bounded history is attached independently to the affected net/resource pair,
the exact selected sampled edge, and the resource transition, while a component-wide binary
orientation search prevents locally accepted resource flips from contradicting another selected
junction. Direct-edge endpoint conversion now retains an exact incident flood resource instead of
reporting a false topology gap. Detailed materialization trims unused packed-lane chord tails and
atomically checks all ordinary nets sharing a packed run; an incompatible via-to-lane permutation
is rejected and those paths fall back rather than crossing. The four-component production case
now has four topology-complete global paths and reduces its discarded hybrid candidate from 16 to
8 copper violations before the unchanged sampled transaction produces connected, DRC-clean
copper. See [`ADR 0110`](docs/adr/0110-negotiated-junctions-and-shared-run-fanout.md).
v0.99/schema 18 makes that ordinary-net fanout fallback more selective without weakening exact
acceptance. A failed shared packed run now reports its physical resource and per-net anchor/lane
geometry, audits endpoint ordering, and searches deterministic member subsets under the existing
candidate budget. The largest visited subset that is exact-clear against the source and itself
stays topology locked; only the complement falls back conventionally. If the assembled board then
fails copper DRC, a separate bounded transaction maps violations to materialized topology nets and
tries increasing-size conventional exclusions from the immutable source. A retry is accepted only
when it retains topology copper and passes complete structural, connectivity, and DRC gates;
otherwise the original all-topology conventional fallback remains authoritative. On the retired
four-net synthetic fixture, the strict detailed scenario retains two of four shared-resource paths, reduces the first assembled
rejection to three violations, explores two topology-net exclusions, and then correctly uses clean
conventional copper because neither retry passes. The production preset still finishes through its
sampled transaction. See
[`ADR 0111`](docs/adr/0111-bounded-exact-topology-subset-repair.md).
v0.100/schema 18 adds the missing application-level integrated workflow without widening the live
mutation boundary. `route_board.py --integrated-placement` invokes the canonical routing-guided
global placement plus detailed-routing engine, uses the complete suggested resistor-first through
connector-last policy unless a reviewed source-bound priority sidecar is provided, and enforces
positive placement/evaluation bounds. It accepts only a complete feasible exported candidate,
reproduces its exact source-bound change set, and runs external KiCad DRC when available. Review
reasons separately expose footprint transforms, tentative placement envelopes, and suggested
classification; the result remains candidate-only and rejects `--apply-routing`. On the retired
four-net synthetic fixture,
the bounded workflow accepts two placement moves, exports two footprint transforms, and passes
KiCad DRC with zero violations and zero unconnected items. The same one-pass/eight-evaluation
integrated engine run completes every owned 4-, 8-, 12-, 16-, and 20-component board with two
accepted moves, zero open connections, and zero represented copper violations; only the basic case
is kept in routine wrapper/artifact qualification because larger debug runs are intentionally slow.
See
[`ADR 0112`](docs/adr/0112-bounded-kicad-place-route-workflow.md).
v0.101/schema 18 repairs the application boundary after the canonical board schema advanced beyond
the bridge's stale schema-17 validator. The Python bridge now accepts only change-set version 1 and
board schema 18, and `apply_candidate.py --validate-only` regenerates a real change set with the
current C++ executable, applies the complete strict Python contract, verifies that the saved source
bytes remain unchanged, and exits before importing `kicad-python` or connecting to KiCad. A
functional CTest uses the actual routed fixture artifact so future C++/Python schema drift cannot
hide behind a hand-written fixture. Both routing-only and integrated `route_board.py` previews use
that same strict validator before reporting a candidate. This preflight neither weakens the live
editor's stale-object checks nor authorizes footprint mutation. See
[`ADR 0113`](docs/adr/0113-current-schema-kicad-candidate-preflight.md).
v0.102/schema 18 reconciles saved-source connectivity and adds bounded expert routing scope/access
without changing `autoroute_v1`. KiCad pad angles are localized at import and transformed once;
connectivity now follows physical copper overlap, so the clean ESC baseline agrees with KiCad at
zero opens and its stripped signal trial agrees at 66. Complete source nets are preserved by
default, while repeated `--route-net NAME` options select an exact atomic reroute scope. Optional
terminal-local fine escape, a separately pitched/phased via sublattice, and a non-crossing guarded
45-degree sampled graph expose more legal access without relaxing resolved spacing. The ESC trial's
failed pin escapes fall from 27 to zero, but 33 nets remain globally unrouted, so Flux still
withholds a candidate. See
[`ADR 0114`](docs/adr/0114-source-connectivity-and-bounded-sampled-access.md).
v0.104/schema 18 adds an opt-in crossing-safe diagonal sampled graph and obstacle-aware adaptive
native arc fitting. Both one-cell diagonal orientations may be searched, but their crossing chords
share one canonical cell-capacity/history resource, so the extra geometric choice cannot create an
unreported unlike-net crossing. After legal polyline materialization, the arc pass tries the maximum
configured radius and deterministically halves it when exact checks reject the changed arc or its
trimmed tracks; explicit backtracking and candidate budgets bound the work, and whole-board
connectivity/DRC can still discard the complete curved transaction. Refillable-zone routing may use
the same arc transaction while remaining provisional and requiring KiCad refill, strict re-import,
external DRC, and human review. See
[`ADR 0116`](docs/adr/0116-crossing-safe-diagonal-discovery-and-adaptive-curved-traces.md).
v0.105/schema 18 replaces independent-corner fillets with an equal-width, same-layer cubic
B-spline transaction. It preserves endpoint tangents, adaptively tessellates and fits native
KiCad lines/arcs, turns exact DRC attribution into a collision envelope, runs a bounded recursive
PNS engine over movable straight copper, rebuilds the control corridor, and refits. If PNS cannot
move a hard obstacle, normalized local basis influence drives a corridor-clamped control-point
shove. Pads, vias, arcs, locked copper, constraints, connectivity, candidate budgets, and final DRC
remain hard gates; failure retains the source polyline. See
[`ADR 0117`](docs/adr/0117-cubic-bspline-fitting-and-bounded-pns.md).
v0.108/schema 18 adds ordered named-net scheduling and a bounded post-negotiation conflict-cone
repair. The repair preserves every unaffected checkpoint path, expands legal terminal portals for
sampled/shared conflict participants, and reroutes only that cone under hard capacity; it never
publishes a non-improving result. Hard sampled/corridor capacity no longer requires a shared
Voronoi graph, while physical hard capacity remains active whenever that graph is configured.
Refillable conventional materialization may also constrain direct-medial transitions to one net so
the allocator cannot authorize packed lanes that this workflow cannot emit. Exact detailed copper
DRC remains authoritative.

v0.109/schema 18 adds fail-closed visualization of partial work. `detail-kicad
--write-incomplete-preview OUTPUT` writes the best materializable layout for inspection even when
global allocation, connectivity, or copper DRC is incomplete; JSON marks it
`artifact_preview_only` and the command still fails, so it cannot be mistaken for a publishable
board. Width-aware physical shadows around sampled tracks, pad escapes, and vias are now enabled
by default and negotiated as attributed edge/node conflicts by v0.112. See
[`ADR 0121`](docs/adr/0121-fail-closed-incomplete-layout-previews-and-sampled-clearance-shadows.md).

For converted or otherwise non-strict source files, `detail-kicad BOARD
--diagnostic-lossy-preview OUTPUT` additionally permits routing a lossy canonical projection and
implies incomplete-preview export. Import/structural/DRC findings are still measured, the source
snapshot and supported-change checks remain enforced, export diagnostics identify the artifact as
lossy and invalid, and the command always retains its nonzero, non-qualified result unless the
ordinary acceptance gates independently pass. This mode is only for inspecting proposed copper;
it must not be used as manufacturing or live-apply authority.

v0.110/schema 18 keeps B-spline optimization in the core negotiated route plan. Every bounded,
exact-reprojected spline corridor now guides subsequent PathFinder passes and weighted retries with
an explicit departure cost; multi-source growth uses the bounded complete-net union. Congestion may
force a departure, but negotiated routing no longer forgets the spline after iteration zero. Native
KiCad arcs remain only the final representation of a legal optimized trace. Guided searches and
corridor departures are observable. See
[`ADR 0122`](docs/adr/0122-persistent-bspline-corridors-through-negotiated-allocation.md).

v0.111/schema 18 makes curved routing a topology-constrained trajectory pipeline rather than a
cosmetic fillet. Clearance-expanded free space is decomposed into bounded polygonal cells, A*
selects a cell corridor, and the funnel algorithm string-pulls its portals. The centerline seeds an
open clamped cubic B-spline whose movable controls are optimized for length, analytical curvature
and bend radius, clearance, pad-entry tangent, curvature continuity, elastic displacement, and
control complexity. The selected cell corridor is a hard homotopy guard. The detailed router uses
that optimized spline for adaptive native line/arc materialization and retains whole-board
validation/connectivity/DRC as the transaction boundary. The public elastic deformation operation
supports localized endpoint changes and basis-influenced obstacle shoves; callers replan the
corridor only when local recovery fails. NavMesh, funnel, optimizer, and rejection metrics are
reported separately. See [`ADR 0123`](docs/adr/0123-navmesh-funnel-bspline-elastic-routing.md).

v0.112/schema 18 promotes selected track/via clearance-shadow intersections into bounded,
net-attributed negotiated resources. They contribute present/history cost, hard-pass blockers,
rip-up evidence, route pressure, terminal-portal history, and connected residual conflict cones.
A bounded topology-preserved same-layer cone is optimized atomically as multiple cubic B-splines
inside retained NavMesh cells with mutual copper-envelope barriers, fixed endpoints,
length/curvature/bend-radius objectives, and bounded native-curve/PNS refinement. Only a complete
validation-, connectivity-, and analytic-DRC-clean board is committed; multilayer joint cone state
fails closed. See
[`ADR 0124`](docs/adr/0124-geometric-corridor-conflict-handoff-and-via-pns.md).

v0.113/schema 18 removes repeated construction from routing hot paths without changing topology or
acceptance. Prepared polygons retain bounds, boundary AABBs and a deterministic BVH; obstacle
queries use flat lattice bins with thread-local generation-stamp deduplication and cached per-net
rules. Fixed-parameter B-spline optimization reuses knot-derived position and derivative
coefficients. Endpoint-independent NavMesh domains retain checked cells and flat coordinate tables
for alternative routes, while selected-corridor membership uses a guard-cell mask. See
[`ADR 0125`](docs/adr/0125-prepared-routing-geometry-and-evaluators.md).

v0.114/schema 18 parallelizes static portfolio planning at the complete-net boundary. Alternative
trees stay sequential within a net; stable task slots reduce in original order before the existing
atomic pair/bus compatibility gate, then per-net NavMesh, B-spline and optional PNS refinement runs
in parallel. Obstacle geometry and resolved rules are frozen and shared, while query caches,
counters, prepared navigation products and diagnostics remain worker-local and are absorbed in
ordered reduction. Configured global safety budgets are partitioned deterministically, and worker
count is observable and runtime-only. See
[`ADR 0126`](docs/adr/0126-deterministic-parallel-static-seed-portfolios.md).

v0.115/schema 18 parallelizes integrated placement-candidate screening within one deterministic
priority-tier/obstacle-phase wave. Proposals receive fixed budget indices before bounded batched
workers evaluate them against the same immutable accepted checkpoint. Only compact scores survive
incremental batches; a feasibility-first, stable-order shortlist is reconstructed and freshly
routed before any move can be accepted. The default is six workers, nested static-seed pools are
suppressed during screening, and exact validation/connectivity/copper DRC remain authoritative.
See
[`ADR 0127`](docs/adr/0127-deterministic-parallel-integrated-candidate-waves.md).

v0.116/schema 18 makes negotiated clearance shadows actionable for both routes. Every certificate
retains the selected edge, terminal portal, or per-layer via node that cast it. Candidate-side
history runs first; after ordinary stagnation, a bounded fallback also charges coalesced owner-side
history while retaining the earlier best checkpoint. Unit pressure improves the 20-component
benchmark from 59 conflicts to 43 with a 20-pass budget and the isolated ESC baseline from 319 to
274 (12 sampled conflicts plus 262 shadows) in eight fallback passes. Expert runs may opt into
clearance-deficit weighting with `--clearance-shadow-cost-quantum-nm`; 50,000 nm reaches 17
conflicts on the smaller fixture, but worsened the isolated ESC checkpoint to 398 (100,000 nm
yielded 379), so zero remains the default. JSON reports attribution, severity, and fallback
execution; localized multilayer conflict-cone execution remains required. See
[`ADR 0128`](docs/adr/0128-owner-attributed-deficit-weighted-clearance-history.md).

v0.117/schema 18 makes the opt-in post-negotiation hard repair consume layer-local, four-pitch
conflict cones containing sampled, shared-medial, and attributed shadow participants. At most two
stable pressure-ordered cones and 12 hard passes per cone are evaluated transactionally;
clearance-shadow-only participants are no longer preserved as immutable blockers and a net's
distant conflicts no longer force one board-wide repair component. JSON exposes cone attempts plus
peak residual-group cardinality/overflow and topology/single-layer eligibility. Multilayer or
incomplete-corridor groups still fail closed before the same-layer joint spline/PNS transaction.
On the 20-component pressure fixture, one of two spatial transactions is accepted and reduces 43
conflicts to 22 with zero sampled overflow. See
[`ADR 0129`](docs/adr/0129-attributed-spatial-conflict-cone-repair.md).

v0.118/schema 18 introduces a first-class regional routing-plan boundary after coarse PathFinder
selection. Selected planar paths are split into stable net/connection/run fragments; exact
clearance-shadow observations are attributed and spatially coalesced into weighted fragment-level
conflict edges, then grouped deterministically without collapsing the evidence to whole nets.
Sampled, scan-corridor, and shared physical excess are projected onto the same graph while keeping
their discrete and nanometre units separate; attributed physical obstacles survive the handoff.
Tarjan biconnected blocks and articulation fragments define bounded physical windows with frozen
portals and explicit oversize/uncuttable/coupled-atom failures. Topology-preserved geometric
corridors become ordered cross-sections packed from heterogeneous widths and adjacent clearances.
Bounded templates retain selected terminal access and exact-checked alternative via sites.
Complete assignments initialize cubic B-splines, optimize controls jointly inside selected
corridors with route-clearance barriers, and retain quantized legal centerlines as a candidate path
set distinct from the coarse plan. A regional candidate may clear planning-only conflicts only on
a private detailed-routing copy; whole-board connectivity and copper DRC accept or discard it
atomically. Unmapped evidence remains explicit and fails closed. See
[`ADR 0130`](docs/adr/0130-regional-routing-plan-boundary.md) and
[`ADR 0131`](docs/adr/0131-articulation-windows-and-physical-lane-templates.md).

v0.119/schema 18 removes zero sampled overflow as a prerequisite for bounded regional repair:
complete global topology plus fully attributed conflicts may enter a private exact transaction.
Oversized biconnected groups split only across conflict-free geometric gaps, coupled pair/bus
partitions merge atomically, and alternative templates reuse cached clearance-offset NavMesh
domains. A template can now select terminal run-ups, relocated entry/exit via sites, an alternate
copper layer, and funnel topology as one state. Competing terminal tracks and via envelopes are
checked jointly before B-spline realization; topology endpoint links are preserved during elastic
optimization. The entire candidate still commits only after whole-board connectivity and copper
DRC. See [`ADR 0132`](docs/adr/0132-bounded-multilayer-regional-template-transaction.md).

Regional lane allocation now treats stable fragment IDs as identities rather than physical lane
positions. Each channel wall derives its preferred occupant order from source-route intersections
along a canonicalized cross-section axis, tries that order before bounded permutation search, and
retains coupled-member order as an independent hard constraint. The selected reference/final
inversions remain observable in the regional result and CLI summary. On the isolated ESC one-pass
checkpoint this avoided 53 lane inversions and reduced continuous-realization failures from 24 to
22 without changing the global feasibility tuple.

Regional corridors now enroll only route fragments that physically traverse their retained cells.
Via relocation is also an attached-geometry decision: candidate sites are checked together with
their planar run-ups, selected B-spline endpoints move with the via, fixed alternate-layer
templates retain their certified sites, and inconsistent shared-via template states are pruned
before access validation. Complete windows publish diagnostic B-spline tessellations even when a
different window is unresolved, while whole-board candidate construction remains all-or-nothing.

The same release adds one weak-occupancy iteration-zero portfolio without another route-search
wave. Existing complete per-net trees are selected in stable priority order using provisional
sampled, clearance-shadow, corridor, and shared-resource pressure, then compared by the unchanged
feasibility-first checkpoint comparator. Unresolved regional windows and attributed blockers also
produce bounded placement targets. See
[`ADR 0133`](docs/adr/0133-weak-occupancy-seeding-and-regional-placement-feedback.md).

v0.120/schema 18 prepares and validates each free-space domain once, batches NavMesh cell/chord
certificates, and uses allocation-free prepared-boundary traversal for ordinary BVH queries.
Regional alternative NavMeshes are bounded by their conflict windows; a failed conservative
offset may fall back only to an unoffset domain carrying the requested minimum-clearance
certificate on every point, cell, and chord. Selected-via probes and alternative templates own
independent deterministic budgets. Consecutive same-layer, same-width, forward-collinear sampled
steps become one regional physical run while bends, reversals, width changes, terminals, and layer
transitions remain fixed. On the one-pass isolated ESC diagnostic this reduced 2,678 fragments/442
windows to 1,372 fragments/204 windows and completed a 128-check regional wave in 190.46 seconds;
27 alternate templates were accepted, but only 14 of 202 eligible windows formed complete bundles,
so no copper transaction was authorized. See
[`ADR 0134`](docs/adr/0134-prepared-local-regional-routing-domains.md).

v0.121/schema 18 stages the weak-occupancy iteration-zero seed by deterministic atomic routing
groups. Already prepared terminal and static-portfolio evidence supplies portal count, complete
tree/corridor/topology/layer alternatives, primary-route interference, physical envelope, and
route extent. Explicit net priority remains the first key; within each priority stratum, short
non-contentious groups precede constrained/high-interference groups, ordinary groups, and finally
high-flexibility groups. Differential pairs and buses remain contiguous atoms. Empty portfolios
and normal negotiated PathFinder passes are unchanged. Until this policy is qualified across the
owned fixtures, Flux also assembles the preceding weak-occupancy order and permits the staged
portfolio to win only when it is no worse under the feasibility-first checkpoint comparator and
does not add clearance-shadow conflicts. All evidence, wave membership, baseline/staged metrics,
qualification, and final selection are emitted by the CLI. On the current one-pass ESC checkpoint
the tuned classifier produced 2 trivial, 41 constrained, 22 ordinary, and 16 flexible groups;
219 of 232 regional windows formed bundles, leaving 13 unresolved, so no spline/copper transaction
was authorized. See [`ADR 0135`](docs/adr/0135-staged-iteration-zero-routing-priority.md).

v0.122/schema 18 promotes vias to bounded regional assignment variables. Exact-legal candidate
sites are grouped by stable transition region and scored from physical via-envelope conflicts,
terminal-run-up conflicts, overlapping layer spans, competing regions, and retained alternative
count. The joint access search assigns the fewest-choice/highest-contention variables first and
retains the selected candidate IDs, region order, switches, exhaustion, and rejection attribution.
Uncontested selected-global sites explicitly defer alternative expansion; already contended sites
retain bounded relocation choices. Static seed evidence also reports primary via transitions,
distinct sites/assignment signatures, and primary-site contention so a short route with one
practical transition is classified as constrained. Exact connectivity, span, clearance, coupled
policy, B-spline realization, and whole-board DRC remain unchanged hard gates. The isolated ESC
checkpoint exposes useful via decisions but remains incomplete and slower than v0.121, so this
stage is not performance-qualified. See
[`ADR 0136`](docs/adr/0136-bounded-scarcity-aware-via-assignment.md).

v0.123/schema 18 carries explicit width state through regional physical packing. A width-change
vertex is a fixed fragment boundary. Terminal neck-down fragments may become genuine B-spline
geometry, but every tessellated edge retains the selected width and policy kind and the candidate
is withheld unless its recomputed run length remains inside the configured bound. Direct-medial
and sampled-corridor neck-downs retain their exact source resource edges and behave as fixed copper
envelopes while neighboring nominal routes are repacked; their adjacent vias and layers cannot be
changed. This replaces the previous whole-window veto without turning an explicit width into
permission to abandon its routing certificate. See
[`ADR 0137`](docs/adr/0137-width-aware-regional-spline-transactions.md).

The same v0.123 regional stage now derives window bounds from both conflict samples and the nearest
points of every attributed physical fragment. This prevents a displaced sampled shadow/capacity
observation from making an otherwise bounded window uncuttable. Via portfolios are assigned by
scarcity-ordered backtracking with forward checking, and their fixed four choices must represent
spatially distinct clearance basins. On the isolated ESC checkpoint all 242 windows become
eligible, both access-search exhaustions disappear, and 238 windows complete. Four physical
lane-packing windows remain, so Flux still withholds the complete spline/copper transaction. See
[`ADR 0138`](docs/adr/0138-fragment-anchored-windows-and-via-csp.md).

v0.124/schema 18 adds a packing-first regional topology wave. Endpoint run-ups and full via spans
are proven before pair search, inward collinear via sites are retained, and alternatives belong
only to their owner window. Packing failures receive the first bounded wave; proper interior
crossings are a fallback. The isolated ESC checkpoint now finds fitting discrete bundles for all
242 windows, but 100 fail continuous B-spline realization, so no copper transaction is exposed.
See [`ADR 0139`](docs/adr/0139-packing-first-owner-scoped-topology-alternatives.md).

v0.125/schema 18 attributes continuous failure separately from discrete packing. Packed lane
offsets now retain their physical cross-section direction, reversed source-route directions share
one canonical lane axis, and only cross-sections that intersect a governed source fragment may
become ordered B-spline lane targets. A bounded joint elastic pass applies the existing cubic-basis
collision-normal shove and accepts only whole-route lexicographic legality improvement before the
ordinary trajectory optimizer. The complete window and whole-board exact transaction gates remain
unchanged; see
[`ADR 0140`](docs/adr/0140-attributed-lane-spline-legalization.md).
On the isolated ESC checkpoint, 205 cross-sections are attributable to their fragments, 686 of
4,493 elastic shoves improve their whole-route score, and 146 of 242 windows realize legally. The
remaining 96 windows contain 34 topology and 165 mutual-clearance route failures, so Flux still
withholds copper.

v0.126/schema 18 spends topology checks left after mandatory packing repair on one exact-prechecked
alternative per highest-overlap owner window. At the 128-check measurement this reduces selected
same-layer conflicts from 373 to 337 and failed spline windows from 96 to 81. Conflict-count-first
priority is explicitly rejected because its slightly lower discrete count produces a worse
continuous result. The bounded check cap is 320 so the same one-choice wave can cover every
eligible ESC owner window; see
[`ADR 0141`](docs/adr/0141-residual-pressure-topology-wave.md).
The 320-check checkpoint uses 24 remaining checks on second fragments in multi-conflict windows,
selects 166 alternatives, and reaches 199 complete spline windows with 190 selected residual
conflicts. Forty-three windows remain, containing 23 topology and 83 mutual-clearance route
failures. Runtime is 561.43 seconds and peak RSS is 769,368 KiB; no copper is authorized.
Selected-global splines also retain a narrow half-cell guard around their exact-static-checked
source fragment because merged channel cells need not reach clipped endpoints. This removes all 17
endpoint topology failures and reaches 213 complete windows; 29 remain, with three interior
topology and 64 mutual-clearance route failures. Immutable offset polygons are shared between
search-specific NavMeshes, reducing a routing-byte-identical checkpoint from 772,968 to 735,236
KiB peak RSS.

v0.127/schema 18 gives residual follow-up checks to no-alternative single-conflict owners before
dense windows and uses eight local controls only at alternate NavMesh corners. At the 320-check
measurement, complete windows rise from 213 to 216; all 26 remaining windows have nonzero discrete
residual conflicts, endpoint topology failures remain zero, one interior-topology failure remains,
and 65 route realizations fail mutual clearance. The bounded portfolio is 384 checks for the next
measured wave; see
[`ADR 0142`](docs/adr/0142-feasibility-first-followup-and-global-runup-guard.md).
At 384 checks, 218 windows realize and 24 remain with one interior-topology and 61 mutual-clearance
route failures. Runtime is 592.38 seconds and peak RSS is 829,260 KiB. The two-window marginal gain
ends portfolio growth; v0.127 now evaluates bounded same-layer NavMesh detours for simple residual
conflicts instead.

The post-v0.128 regional access correction coalesces consecutive same-position layer hops into one
physical via span, checks the terminal/via CSP while enumerating partial template tuples, and
attributes the exact blocker pairs that prune the frontier. A selected via alternative now owns an
exact-static-checked straight or orthogonal-dogleg run-up; its vertices seed the early cubic
B-spline and are exported as provisional WebSocket routes. Terminal provenance survives leading or
trailing via stacks, so a pad via cannot move without explicit pad-fanout geometry. On the isolated
ESC checkpoint the dogleg transaction reduces seven discrete access blockers to window 109 alone;
220 windows complete spline realization while 21 remain continuous failures. Generic wider and
just-clear endpoint stencils were measured and removed. The remaining step is a bounded joint
lane/via-site assignment for the attributed conflict cone, not more independent candidates. See
[`ADR 0144`](docs/adr/0144-coherent-regional-via-access.md).

ADR 0145 adds the first bounded conflict-cone joint portfolio. Flux proves that an ordinary window
has no complete terminal/via/lane state, ranks attributed via-pair blockers, and shares an
independent 128-check wave across eight conflict windows without multiplying the budget. On the
isolated ESC it uses 126 checks and accepts 96 alternatives: window 109 becomes discretely feasible
and the continuous shortlist recovers window 75. The current checkpoint reaches 223/242 complete
regional windows with 5 topology and 48 mutual failures. The board remains withheld; topology-
specific centreline/run-up envelope conflicts remain part of the joint tuple comparator. See
[`ADR 0145`](docs/adr/0145-bounded-conflict-cone-joint-portfolio.md).

ADR 0146 retains at most four complete atomic states for the expanded conflict window and up to
seven additional pressure-ranked windows with existing exact-prevalidated alternatives, then
passes them through the topology-guarded B-spline and elastic-PNS realization. The first state is
scalar-best; the remaining states maximize bounded template diversity so nearby via-only variants
do not consume the complete budget. The first completely legal state wins. If none is legal, only
the least-illegal diagnostic state is retained; other windows still evaluate one selection.
Selected-state rejection counters are
recomputed after the trial reduction so diagnostic failures cannot masquerade as retained board
state. See [`ADR 0146`](docs/adr/0146-joint-window-spline-shortlist.md).

ADR 0147 adds bounded two-spline elastic/PNS transactions after per-route optimization. Both local
control neighborhoods move in opposite collision-normal directions, and the change survives only
when the complete adaptively tessellated window improves; rejection restores both splines. The
first isolated ESC measurement clears two more regional windows (222/242 complete), but 20 failed
windows and coarse overflow still withhold candidate copper. See
[`ADR 0147`](docs/adr/0147-adaptive-whole-window-pair-shove.md).

ADR 0148 makes the prepared rectilinear free-space cells an explicit micromouse-style maze.
Routing floods backward from one or more sorted goals with bounded breadth-first integer labels,
then reconstructs one deterministic decreasing-distance corridor before the existing funnel and
B-spline stages. The flood remains advisory to PathFinder, regional packing, connectivity, and
DRC. Route JSON reports the number of flood-seeded NavMesh routes and visited cells.
[`ADR 0148`](docs/adr/0148-micromouse-flood-fill-topology-seeding.md).

ADR 0149 adds a separate bounded local-layer-overpass portfolio for single-conflict regional
windows. Via pairs bracket the projected physical conflict, preserve original-layer run-ups, and
route only the middle section through the prepared NavMesh/flood/funnel/B-spline pipeline. The
established endpoint portfolio always runs first under its unchanged budget. On the isolated ESC,
20 checks accept 12 overpasses and improve complete regional windows from 272/284 to 273/284;
11 continuous windows still withhold candidate copper. See
[`ADR 0149`](docs/adr/0149-bounded-local-layer-overpass.md).

ADR 0150 adds one bounded quarter-pitch terminal escape only when distinct nets share a nearest
coarse portal. It preserves the nearest exact-legal portal and coarse PathFinder lattice while
making terminal contention visible in the seed portfolio. On the isolated ESC, 58 generated
alternatives reduce global overflow from 940 to 543 and regional windows from 284 to 199. See
[`ADR 0150`](docs/adr/0150-contention-triggered-terminal-fanout.md).

ADR 0151 allows a bounded residual articulation-cone retry to become the next regional diagnostic
checkpoint on a strict feasibility-first improvement, without requiring every independent failed
window to be covered in the same pass. Exact candidate availability remains all-or-nothing. Six
ESC cones reduce continuous failures from 15 to 10; candidate copper is still withheld. See
[`ADR 0151`](docs/adr/0151-cumulative-residual-conflict-cone-checkpoints.md).

ADR 0152 realizes contiguous compatible owned fragments of one selected global run as a single
cubic B-spline, keeping only the outer endpoints fixed. This removes artificial control locks at
internal sampled corners without broadening the topology corridor or candidate gates. The current
ESC checkpoint merges one two-fragment run and reduces retained mutual-route failures from 20 to
19, but still has ten unresolved windows. See
[`ADR 0152`](docs/adr/0152-run-level-regional-bspline-realization.md).

v0.107/schema 18 replaces optional early guidance with a mandatory exact static route portfolio.
Bounded edge-penalty reruns retain topology alternatives; spline tessellation forms advisory
corridors that must reproject through the hybrid graph; complete multi-terminal trees and atomic
pair/bus groups become an iteration-zero checkpoint using canonical occupancy accounting. Bounded
PNS can retain an unlocked straight-track local-split delta for detailed transactional acceptance.
PathFinder remains polyline/resource based, and native arcs remain a post-polyline detailed pass.
See [`ADR 0119`](docs/adr/0119-mandatory-static-seed-portfolio-and-iteration-zero.md).
In strict-medial mode, finite parabolic point/segment bisectors can contribute only a straight
endpoint-chord surrogate that independently passes whole-chord free-space and clearance checks.
Its capacity is recomputed from the chord and its provenance remains observable; ordinary routing
is unchanged. See
[`ADR 0038`](docs/adr/0038-checked-curved-bisector-chord-topology-completion.md).
The integer-miter offset service erodes board boundaries, dilates obstacle polygons, subtracts
their union, preserves disconnected outer/hole regions, and independently verifies requested
clearance after rounding. Net/layer extraction preserves filled-zone hole topology, represents
circles with bounded circumscribed polygons, and conservatively encloses oval/rounded pads and
existing segment/via copper. Custom, trapezoid, and unknown pads remain fail-closed; see
[`ADR 0025`](docs/adr/0025-conservative-integer-miter-free-space-offsets.md).
The global router can opt into a bounded net/layer adapter that publishes positive-capacity,
exact-rechecked Voronoi edges plus accepted/rejected-domain and conservative-provenance metrics.
It can also rasterize those edges into a bounded per-net medial distance field that contributes an
explicit non-negative PathFinder centering cost. Guidance alone owns no capacity or conflicts; see
[`ADR 0026`](docs/adr/0026-bounded-voronoi-corridor-evidence.md).
v0.22 also constructs an opt-in common-domain medial graph whose channel width is shared across
nets. PathFinder sums conservative heterogeneous width-plus-spacing demand in nanometres, applies
present/history costs, reserves both resources at nearest-cell transitions, and reports exact
physical overflow. Sampled edges and nodes still ensure every accepted plan has distinct
materializable centerlines and via conflicts; see
[`ADR 0027`](docs/adr/0027-shared-voronoi-pathfinder-capacity-resources.md).
Declared differential pairs now aggregate as two resolved widths, one effective internal gap, and
two external clearance margins when they share a medial resource. After the first member is
allocated, an explicit non-negative PathFinder term encourages its complement to use the same
channel while sampled capacity keeps their centerlines distinct; see
[`ADR 0028`](docs/adr/0028-differential-pair-medial-resource-coupling.md).
Each used uncongested medial resource also publishes deterministic Arachne-style cross-section
evidence: heterogeneous single-net envelopes are packed around the medial axis, while complete
pairs remain adjacent at their resolved centre spacing. Signed offsets and shifted straight
segments are audited with exact static-board obstacle predicates, but remain advisory because
junction continuity and mutual proposal interactions are not proven by per-chord evidence; see
[`ADR 0029`](docs/adr/0029-advisory-medial-cross-section-packing.md).
Shared medial endpoints now produce bounded route-selected lane-bridge proposals. Unused incident
alternatives cannot reject an allocated topology. A bounded deterministic cross-section
orientation search preserves multi-net lane order, and nonparallel shifted resource lines may meet
at a quantized, length-bounded miter checked against static geometry and the other selected
transition polylines. The detailed router trims to the miter instead of retaining
projected-endpoint spikes. These bridges still do not replace a coherent detailed route; see
[`ADR 0031`](docs/adr/0031-bounded-medial-lane-junction-proposals.md) and
[`ADR 0040`](docs/adr/0040-route-selected-mitered-medial-transitions.md).
Every routed connection also publishes its compressed medial-resource sequence, exact-adjacency
breaks, terminal-access unmapped nodes, and interior mapping gaps. `--strict-medial` rejects grid
edges that would jump between non-adjacent retained resources; it may expose straight-subset
disconnection rather than silently using an unproven link. See
[`ADR 0032`](docs/adr/0032-medial-path-topology-audit-and-strict-mode.md).
Topology-complete sequences can now be materialized from exact-static-checked lane chords and
accepted same-layer junction bridges. Ordinary multi-layer signals retain ordered global-route
via anchors between packed runs; complete pair/bus groups with matching transition sequences may
instead select jointly spaced via arrays from global anchors or packed endpoints. Terminal joins
and vias are added, then the entire quantized board must pass structural validation, connectivity,
and copper DRC. Incomplete evidence falls back per path or complete coupled group; a rejected
combined lane candidate is discarded and the conventional polyline is retried from the immutable
source. See
[`ADR 0033`](docs/adr/0033-transactional-topology-locked-lane-materialization.md) and
[`ADR 0037`](docs/adr/0037-uncoupled-multilayer-medial-materialization.md), and
[`ADR 0039`](docs/adr/0039-synchronized-coupled-multilayer-via-arrays.md).

Automated routing verification now uses a pinned, licensed subset of the KiCadRoutingTools
real-board stress corpus. Qualification is monotonic: every qualified tier remains in routine
CTest, and the next tier is permanently unlocked as a bounded capability probe. The current level-0
PT2399 board completes 50 connections as exact KiCad-DRC-clean copper and emits per-layer PNG review
artifacts; the analog `bm_svf` module is the
always-run level-1 probe and currently seeds all 78 connections before bounded congestion stops it.
Higher boards remain locked until the preceding tier qualifies. Each
stripped input travels with its project, routed human reference, exact upstream revision/hash, and
license. See
[`tests/fixtures/routing-corpus/README.md`](tests/fixtures/routing-corpus/README.md).
Run the complete qualified-plus-frontier baseline with `./run_tiers.sh`. Every invocation writes a
new artifact root containing an immutable `run-metadata.json`, complete global/detailed result JSON
with stage counters and durations, diagnostic telemetry, mandatory analysis PNGs, and a final
`run-result.json` inventory with content hashes. The metadata records the exact Flux executable,
capability contract, corpus manifest, board revisions, normalized route arguments, KiCad/Inkscape
versions, Vulkan override, and the GPU selected by Flux's shared compute runtime. Use
`FLUX_ARTIFACTS=/new/path ./run_tiers.sh` to choose a write-once destination. A fast
`./run_tiers.sh --preflight-only` validates all active source identities and the complete run
contract without launching a route.

The dependency-ordered verification boundaries are executable through `run_suite.sh`:

- `./run_suite.sh per-change` runs the three high-risk routing invariants, observer/manifest/GPU
  contract tests, and every permanently qualified tier;
- `./run_suite.sh frontier` runs the same risk checks followed by all qualified tiers and the one
  unlocked tier;
- `./run_suite.sh release` additionally validates the complete parity-fixture and progressive
  corpus inventory. Locked boards remain strict-import gates until their preceding tier qualifies;
  milestone promotion adds the representative routed feature waves.

Append `--preflight-only` to any suite command to validate its identities and selection without
launching tier routing. `tests/suites.json` is the machine-readable membership contract.
Pinned reference-router comparisons are defined by
`tests/fixtures/parity/reference_cases.json` and replayed with
`tests/python/record_reference_parity.py`; the checked summary under
`reports/reference_comparison/` compares connectivity, DRC, route scope, and feature behavior
rather than copper bytes. Current KiCad 10 reference fixtures remain honest Flux import blockers.

The first versioned workflow boundary is available as `FluxWorkPlan` JSON. Create a write-once,
source-revision-bound empty template with `flux-cli work-plan-template-kicad BOARD PLAN`, inspect it
with `flux-cli inspect-work-plan-kicad BOARD PLAN`, and perform a non-mutating capability preflight
with `flux-cli dry-run-work-plan-kicad BOARD PLAN`. The schema carries ordered route, fanout,
differential-pair, bus, plane, tuning, cleanup, validation, and placement step kinds; explicit net,
glob, class, footprint, and placement-group selectors; protected routing modes; immutable policy
identities; typed scalar options; dependencies; and default-on telemetry. Empty templates infer no
work. Execution/replay remains withheld until option semantics and selector expansion share the
existing authoritative CLI/configuration path.
Routine manual routing and application validation now uses the external Teensy CNC controller as a
strictly read-only target. It is never an automated test dependency, generated outputs stay under
`build/verification/teensy_cnc/`, and its machinery-safety/`DO NOT FABRICATE` rules remain binding.
See [`docs/verification/teensy-cnc-controller.md`](docs/verification/teensy-cnc-controller.md).

`constraints-template-kicad` emits a deterministic current-only sidecar bound to the imported
board fingerprint. Edit `allowed_rotations` with absolute degree values and `allowed_sides` with
`front`/`back`, retaining the current value in each domain. `constrained-optimize-kicad` uses the
sidecar for repair; `constrained-refine-kicad` continues routed-quality improvement with a larger
bounded budget. Unknown fields, stale fingerprints, duplicate normalized angles, unknown stable
IDs, reference mismatches, and policies excluding the current orientation fail closed. Only
footprints named by the sidecar receive orientation proposals, although ordinary legal
translations remain part of the integrated search.
