# Router telemetry viewer

`flux-viewer` is a native C++ desktop application using Dear ImGui, SDL2, and OpenGL. It does not
build or run through WebAssembly. Its default source is the read-only
`ws://127.0.0.1:8765/telemetry` endpoint exposed by `flux-cli --telemetry`.

The UI provides four diagnostic areas plus dashboard-style global scaling:

- a pan/zoom PCB canvas with layer selection, source-board outline/cutouts, footprint courtyard and
  pad geometry, existing copper/zones/keepouts, and independently switchable route, conflict,
occupancy, priority, and via overlays;
- dashboard-style UI scaling from 0.75x through 3x with Ctrl/Cmd+wheel, Ctrl/Cmd `+` or `-`, and
  Ctrl/Cmd `0` to reset;
- live Pathfinder feasibility metrics in a label/value grid, with routed, unrouted, and total nets
  on separate rows, plus iteration history;
- priority and via-site evidence tables; and
- a semantic activity timeline with trace playback controls.

The activity table presents the newest observation first. Drag the horizontal divider above it to
resize the activity pane without changing the PCB canvas or telemetry source. The underlying event
stream and trace remain chronological, so replay position, seeking, and deterministic
reconstruction are unchanged.

Viewer preferences are restored from the platform configuration directory (`$XDG_CONFIG_HOME` or
`~/.config` on Linux). UI scale, activity-pane height, replay speed, endpoint/detail/rate, selected
layer, every overlay checkbox, and the last trace/record directories are saved independently of
the ImGui window-layout file.

The Overlays panel retains the received event deltas in chronological memory and exposes them with
an oldest-to-newest timeline slider. Sparse reduced checkpoints make backward and forward seeks
bounded without copying the complete PCB geometry for every event. Disable `Follow live / replay`
to inspect an earlier overlay while new events continue accumulating; returning to the current
source position resumes following. Timeline state is viewer-only and cannot affect the router.

Committed, current, provisional, ripped-up, and removed routes have distinct rendering. Via
candidates, selected vias, contested sites, and rejected sites also remain separate. Congestion
resources show demand/capacity utilization, while clearance-shadow conflicts retain their own
physical-deficit overlay rather than being presented as ordinary congestion.

Regional quintic PH chains are rendered as smooth magenta curves by default. The PH curve and its
Bezier control polygons have separate overlay switches; enabling the control polygons adds the six
ordered controls for every degree-five span without joining unrelated segment control nets.

Diagnostic routing also publishes the latest bounded direct-polygon hydraulic discovery terrain
independently for each copper layer: complete eroded height, free-space coverage, accumulated flow,
erosion, downstream failure pressure, and accumulated clean-copper success prior. Discovery uses a
separate obstacle-clearance field, while the published origin and
fine pitch are independent of the later coarse NavMesh projection. Enable
`Hydraulic erosion fields` to render the target valley through uphill terrain while leaving pads,
vias, keepouts, and board exterior unpainted. High-flow riverbeds brighten the terrain, downstream
pressure is tinted red/magenta, and the bounded success prior adds green. Use the
ordinary layer controls to isolate F.Cu, inner layers, or B.Cu. The extracted riverbed has a
separate green route-family overlay. Each event retains its net/candidate
attribution in timeline history, while recovery snapshots coalesce to the newest field per layer
so dense-board telemetry remains bounded.

## Build

The viewer is enabled by `FLUX_BUILD_VIEWER`. CMake fetches a pinned official Dear ImGui release by
default. An existing upstream checkout can be used for offline builds without making it a runtime
dependency:

```sh
cmake --preset viewer
cmake --build --preset viewer

# Or use an existing Dear ImGui checkout:
cmake -S . -B build/viewer -G Ninja \
  -DFLUX_BUILD_VIEWER=ON \
  -DFLUX_VIEWER_IMGUI_SOURCE_DIR=/path/to/imgui
cmake --build build/viewer --target flux-viewer
```

SDL2 and OpenGL development packages are required. `FLUX_BUILD_VIEWER=OFF` keeps headless/core-only
configuration independent of those packages.

## Live use

Start a route with the local read-only telemetry server and connect from another terminal while it
is running:

```sh
# Terminal 1
build/dev/flux-cli route-kicad board.kicad_pcb \
  --telemetry --telemetry-listen 127.0.0.1:8765 \
  --telemetry-level normal --telemetry-record route.jsonl

# Terminal 2
build/viewer/flux-viewer --connect ws://127.0.0.1:8765/telemetry
```

`normal` is the viewer and producer default. It reports routing starts/completions, net outcomes,
accepted route and via changes, rip-up activity, completed Pathfinder passes, and checkpoint or
placement milestones. It does not request individual A* node expansions. Select `diagnostic` for
bounded priority, via-candidate, resource, and conflict evidence, or `trace` explicitly for
expensive search debugging, then reconnect to renegotiate the subscription.

The live connection remains active while the producer is absent. The viewer retries automatically,
detects when Flux stops, reconnects when another Flux instance starts on the endpoint, and resets
the displayed live state before consuming the new instance's snapshot and events. Use Disconnect
or `--no-connect` when automatic connection is not wanted.

Each KiCad CLI run first publishes an immutable source scene. The viewer shows its path, revision,
schema and object counts plus represented physical-stackup layer/dielectric counts and board
thickness, fits the canvas to the board outline, and renders retained native arcs as curves. Flux
caches this source snapshot separately from current routing state, so a late or
reconnected viewer receives both while normal periodic snapshots remain compact. When a workflow
publishes several boards, path-plus-imported-revision identities retain every immutable scene
instead of replacing the previously published source. Current fitting
evidence is the geometry retained by the canonical board—footprint courtyards and pads—not
invented Fab/Silk artwork.

The selected final PathFinder polylines are published as provisional routes. They remain visible
when the run stops with residual regional conflicts, making incomplete layouts directly
inspectable rather than leaving only conflict markers and source geometry.

Long regional passes also publish compact `SearchSummary` activity at regional start and before a
complete window enters PH realization. These events keep dense-board progress visible without
emitting hot-loop probes or triggering full snapshots.

`RoutingStarted` and the snapshot label `Routing finished` refer to the complete telemetry
session. Nested PathFinder calls used by feedback and conflict-cone repair appear as
`GlobalRoutingStarted`/`GlobalRoutingFinished` stage summaries. Their completion refreshes metrics
but does not mark the viewer inactive while regional PH, XPBD, detailed routing, or DRC work is
still running.

For a complete regional window, the viewer also receives its flattened quintic PH Bezier controls,
degree, adaptively tessellated collision/materialization centreline, and the exact-static-checked
straight or dogleg run-up attached to every selected relocated via. The viewer evaluates the
degree-five Bezier spans from the controls for display; the transmitted adaptive samples remain
the diagnostic physical-check representation. Run-up routes use
stable `regional-via-runup:*` IDs and remain provisional planning geometry. This keeps the
displayed via connected to the corridor geometry that selected it; it does not imply that the
complete board has passed connectivity or DRC.

The WebSocket thread decodes frames away from the ImGui thread and hands over a bounded queue. A
slow viewer can drop its own pending observations but cannot push back into the router.

## Record and replay

The live endpoint and `.jsonl` traces both carry `flux.telemetry.v1` records. `flux-viewer` accepts
the core producer's event and recovery-snapshot records directly. The extended schema uses the same
records for bounded geometry and evidence arrays; it does not introduce a replay-only frame model.

Open a producer trace with:

```sh
build/viewer/flux-viewer --replay route.jsonl
```

A multi-stage or multi-board telemetry directory may also be opened directly:

```sh
build/viewer/flux-viewer --replay build/tiers-check/telemetry
```

Regular `*.jsonl` files are loaded by modification time and then filename, with each process-local
sequence/time range rebased into one monotonic replay timeline. Invalid, unreadable, empty, or
trace-free directories are reported in the status bar rather than terminating the viewer.

The Source and Capture panels use a built-in filesystem browser. The trace browser can select one
JSONL file or the currently displayed directory; the recording browser selects a destination file
without requiring a manually entered full path.

The replay timeline can play at recorded time, step one event, jump to the next meaningful
milestone, or seek by event index. When its slider has keyboard focus, Left and Right move exactly
one event. The overlay timeline can independently inspect any retained
route, PH curve, via, conflict, resource, or source-scene state. Seeking rebuilds display state
only. It never calls or influences the router.
The viewer can also record incoming live events using the Capture panel or `--record`; each decoded
event is written as one versioned JSONL record.

## Extended event fields

Core fields (`type`, `protocol`, `kind`, `sequence`, iteration counters, stable IDs, and `detail`)
remain accepted. Snapshot or incremental records may additionally contain these bounded arrays:

- `source_boards`: path/revision/schema, optional physical thickness, fixed-point stackup material
  records, stable layers/nets, and generic integer primitives for outline/cutouts, footprints,
  pads, existing segments/arcs/vias, zones, and keepouts;

- `routes`: stable route/net IDs, revision, state, width, cost, geometry kind, spline degree,
  `{x_nm,y_nm,layer}` physical-check points, and optional degree-five PH `control_points`; these include
  coarse `global-route:*` polylines and bounded `regional-spline:*` PH controls/tessellations from
  complete local windows, plus selected `regional-via-runup:*` access geometry, even when another
  window keeps the board incomplete;
- `vias`: stable IDs, position/span, score, clearance margin, contention, alternatives, state, and
  rejection reason;
- `conflicts`: kind, resource/group/net IDs, location, clearance deficit, pressure, and history;
- `resources`: sampled/corridor/physical/via-site span, demand, capacity, width, and participating
  nets;
- `hydraulic_fields`: stable net/layer identity, fine polygon-domain origin and pitch, bounded
  sampling stride, original maxima, and normalized 16-bit row-major height, free-space coverage,
  flow, erosion, downstream-pressure, and success-prior values; and
- `priorities`: the staged class, individual scarcity/interference/width terms, alternatives,
  final priority, and stable tie-break.

Snapshot records replace those bounded collections. Incremental records merge or remove the named
stable entities. A rip-up event without repeated geometry changes the retained route's state, which
lets replay show what was removed without requiring a second representation.
