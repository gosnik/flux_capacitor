# ADR 0143: Non-authoritative Boost.Beast router telemetry

## Status

Accepted for the v0.128 global-router telemetry foundation.

## Context

Flux needs live inspection and record/replay of long routing runs without debug-log parsing. The
observer must expose PathFinder progress, routing priority, via allocation, conflicts, residual
groups, detailed geometry, and placement proposals over time. None of that copied state may become
another routing representation or affect deterministic decisions.

Socket I/O, serialization, disk writes, and a slow viewer are unbounded relative to a routing
worker. Performing them on a worker, or allowing backpressure into the router, would violate that
boundary. The repository also already has a `flux.telemetry.v1` event model and native viewer, so a
second wire format would create ambiguous IDs and incompatible recovery snapshots.

The local BlueOcean test-rig WebSocket transport demonstrates a useful Beast execution shape: one
I/O context and thread, strand-serialized socket state, one asynchronous write in flight, a bounded
outbound queue, and explicit close/error handling. Its source is proprietary and its CAN payload is
unrelated to Flux.

## Decision

Flux uses a nullable `RouterTelemetrySink` at routing boundaries. Producers call only non-blocking
`try_emit` with compact semantic events. The fixed-capacity queue is a preallocated
sequence-stamped MPSC ring: producers reserve cells atomically and drop an observation only when
the ring is full. They never take a consumer lock. Dropped events and transport failures are
counters, never routing errors.

A separate consumer owns aggregation, coalescing, JSON serialization, optional JSONL recording,
and publication. Events and snapshots use the existing `flux.telemetry.v1` model with stable string
Flux IDs. Periodic snapshots let observers recover after event loss.

One separately published immutable source-scene snapshot contains the strict canonical board's
path/revision/schema, layers, nets, and stable-ID drawable primitives. The Beast server caches it
separately from the latest dynamic snapshot. A new observer receives source then dynamic state;
periodic snapshots omit the source board, and replay retains the last scene until another source
snapshot explicitly replaces it. This keeps source geometry available without repeatedly
serializing it.

The live interface is a clean-room Boost.Asio/Boost.Beast implementation with:

- a dedicated `io_context`, strand, acceptor, and I/O thread;
- a read-only `/telemetry` endpoint requiring subprotocol `flux.telemetry.v1`;
- an initial latest snapshot for each accepted session;
- at most one asynchronous write in flight per session;
- bounded router, transport, and per-session queues; and
- failure isolation from routing stop tokens, checkpoints, and results.

Inbound frames may describe observer subscription preferences but cannot submit commands or mutate
authoritative state. ADR 0154 subsequently makes the local endpoint
`ws://127.0.0.1:8765/telemetry` default-on for KiCad routing and integrated optimization, with an
explicit `--no-telemetry` opt-out. Integrated candidate routing shares the same non-blocking sink;
placement-specific events remain incomplete.

The BlueOcean code is an architectural reference only. Flux does not copy its implementation,
names, framing, records, or CAN-specific protocol.

## Current boundary

v0.128 emits global routing start/finish, staged priority records, PathFinder iteration summaries,
best-checkpoint changes, regional residual-group updates, attributed physical conflicts, bounded
via candidates, and selected regional via assignments. It provides bounded queue/drop behavior,
stable-ID replacement coalescing, JSONL recording, dynamic recovery snapshots, and the Beast
WebSocket server. The CLI also publishes outline/cutouts, footprint courtyards and pads, existing
segments/arcs/vias, zones, and keepouts from the immutable source board; the native viewer renders
those beneath dynamic routes. Regional diagnostic events are derived after bounded analysis and
cannot feed the solver. The selected final global polylines are also published as provisional route
geometry so incomplete allocations remain visible. Completed regional windows additionally publish
their adaptively tessellated B-spline planning geometry under separate stable IDs. Selected
regional vias also publish their attached straight or dogleg run-ups as provisional routes, so the
viewer can display coherent local access rather than an isolated marker. Neither observation
relaxes the all-windows candidate gate. Detailed copper transactions, placement proposals, and
validation milestones remain later increments behind the same non-authoritative interface.

## Consequences

Telemetry disabled/enabled routing results must remain identical. A disconnected or non-reading
client can lose observations but cannot reduce throughput through backpressure. A trace may be
incomplete when drops are nonzero, so snapshots and counters are part of the protocol. The viewer
and recorder share one schema and do not need adapters for live versus replay data.

Boost.Beast and Boost.Asio remain implementation dependencies of the telemetry-enabled core build;
Boost.System uses target-local header-only error codes. The native viewer adds optional SDL2,
OpenGL, and Dear ImGui build dependencies but is not required by the router process.
