# ADR 0154: WebSocket telemetry is enabled by default

## Status

Accepted.

## Decision

KiCad routing and integrated-optimization CLI workflows construct the non-authoritative telemetry
service by default. `RouterTelemetryConfig` defaults to `127.0.0.1:8765`; the read-only endpoint
remains `/telemetry` with subprotocol `flux.telemetry.v1`. `--no-telemetry` is the explicit opt-out,
and `--telemetry` remains an idempotent compatibility option.

Integrated optimizer candidate evaluations pass the same bounded MPSC sink through full and
incremental global/regional routing. They do not consume telemetry state. Placement-specific event
coverage can be added independently.

## Failure and performance boundary

Routing workers only perform non-blocking event insertion. Bind failure, a missing observer,
backpressure, disconnect, serialization failure, queue overflow, or recorder failure updates
diagnostic counters and cannot fail, stop, or alter routing. Opting out constructs no service,
socket, queue, or telemetry thread.

Focused verification observes 21 produced and 21 consumed events with zero transport failures for
a default CLI route. The identical command with `--no-telemetry` exposes no telemetry statistics;
the core, live-client, and viewer risk checks pass.
