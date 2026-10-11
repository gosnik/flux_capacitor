# ADR 0169: Confined anisotropic conflict erosion

## Status

Accepted.

## Context

The ordinary hydraulic topology family is isotropic and layer-local. It can discover broad free
space, but a residual physical conflict often needs a more specific decision: preserve progress
along an implicated route run while moving to one of the two sides of an attributed blocker. A
global anisotropic metric would bias unrelated nets and could reduce useful route diversity.

The conflict-directed exact regional master already provides the necessary scope and trigger. It
runs only after continuous PH or quantized copper rejects a complete discrete state, and it
identifies the failing window and complete route runs before requesting new topology columns.

## Decision

Add an optional integer anisotropic metric to the shared hydraulic solver. An axis in raster space
defines longitudinal motion. The configured transverse ratio raises reverse-potential step cost
and increases the bank-erosion divisor in proportion to the integer cross product between that
axis and each of the eight raster directions. A ratio of 1000 preserves the prior isotropic result
byte-for-byte. CPU and Vulkan implementations use the same bounded integer arithmetic.

Use that metric as a production regional feature only inside the conflict-directed master. For an
attributed same-layer blocker set:

- confine polygon rasterization and NavMesh projection to the existing regional search bounds;
- deposit bounded pressure over the blockers without changing polygon-derived walkability;
- run two deterministic solves with success priors displaced to opposite route normals;
- prevent visibility pulling from silently crossing a pressure-barrier side selected by the river;
- reject a proposal unless its centerline clears every attributed route envelope;
- publish distinct exact-certified topology columns to the existing joint lane/layer/via master.

The feature has independent bounds for checks and candidates per fragment. CLI JSON and telemetry
report checks, accepted and rejected proposals, CPU/GPU execution, and exact-certified river
chords.

## Consequences

Residual conflicts gain a directional same-layer topology mechanism without perturbing ordinary
global allocation or the canonical isotropic free-space representation. The two signed priors
avoid encoding a preferred obstacle side, while deterministic seeds keep results repeatable.

Pressure and erosion remain advisory. They cannot open an illegal raster cell, authorize a route,
or relax clearance. Prepared-polygon chord certification, topology signatures, PH geometry,
continuous and quantized copper clearance, connectivity, internal DRC, source-preserving export,
and external KiCad DRC remain mandatory.
