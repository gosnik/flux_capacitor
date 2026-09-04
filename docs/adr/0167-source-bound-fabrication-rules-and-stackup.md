# ADR 0167: Source-bound fabrication rules and physical stackup

## Status

Accepted.

## Context

Flux previously imported net-class routing dimensions but did not retain the project-wide KiCad
fabrication floors or the physical board stackup. A generated track could therefore satisfy its
net class while being narrower than the project minimum, and via search could reserve a copper-legal
site whose drill was mechanically too close to another hole. Later impedance, propagation-delay,
return-path, and time-matching work also needs authoritative copper/dielectric thickness and
material properties rather than a guessed two-layer FR4 model.

The maintained `kicad-python` package exposes live net classes and effective membership, but the
installed generated IPC `BoardDesignRules` message contains no board-wide minimum constraints and
provides no rule-query command. The matching saved `.kicad_pro` contains those official project
settings. KiCad's board S-expression directly contains the physical stackup. The upstream
[KiCad stackup parser](https://gitlab.com/kicad/code/kicad/-/blob/master/pcbnew/pcb_io/kicad_sexpr/pcb_io_kicad_sexpr_parser.cpp)
also shows that a dielectric item may contain multiple `addsublayer` parameter sets and that newer
frequency-dependent material fields exist.

## Decision

Board schema 20 added project fabrication floors to `BoardDesignRules`. Net-class snapshot version
3 binds two independent immutable sources:

- the exact saved board revision reported by Flux's strict importer;
- a SHA-256 revision of the matching saved project file.

The live adapter continues to use official IPC for net classes and effective membership. It reads
the project file once under regular-file, 16 MiB, size, timestamp, and content bounds and captures
exactly:

- minimum clearance;
- minimum track width;
- minimum via diameter;
- minimum via annular width;
- minimum through-hole drill;
- copper-to-edge clearance;
- hole-to-copper clearance;
- hole-to-hole clearance.

Every value is converted from decimal millimetres to integer nanometres with deterministic half-up
rounding. Missing, extra, negative, malformed, stale, or partially applied fields reject the whole
snapshot. The resolver applies track, clearance, drill, diameter, and annular values as floors after
normal net-class inheritance; it does not rewrite source classes. Exact circular/oval drill shapes
make hole-to-hole spacing mechanical and independent of net/layer identity. Whole-board and dynamic
DRC report the violation, while global and regional via selection reject such sites early. Same-net
reuse exempts only an existing hole at the exact selected coordinate; it does not exempt a nearby
same-net drill.

Board schema 21 additionally stores:

- optional physical board thickness from the saved board;
- the ordered physical stackup;
- linked copper/mask/paste/silkscreen board layers;
- core/prepreg dielectric layers and bounded `addsublayer` records;
- integer-nanometre thickness and its lock state;
- material and color strings;
- relative permittivity and loss tangent as exact source decimal values scaled by `1e9`;
- copper finish, dielectric-constraint, edge-plating, castellated-pad, and edge-connector state.

The import must contain every enabled copper layer exactly once in front-to-back order. After
removing technical surface layers, copper and dielectric entries must alternate. Copper and every
dielectric sublayer require positive physical thickness. Canonical validation repeats these
invariants for programmatically constructed boards. Source-preserving export treats board thickness
and stackup as immutable structure and re-imports them during its semantic round-trip gate.

Unknown stackup fields and currently unrepresented `spec_frequency`/`dielectric_model` semantics
produce attributed strict-import errors. They are not silently discarded or replaced by a constant
dielectric model. The source telemetry snapshot carries the same optional thickness and fixed-point
stackup records; live and replay viewers show compact physical-layer/dielectric counts.

## Consequences

Generated ordinary copper now observes the captured project floors, and mechanical drill conflicts
influence both exact acceptance and via planning. The selected two- and four-layer parity, Teensy,
and ESC sources retain calculation-ready stackup geometry without introducing a second electrical
board model.

This does not complete KiCad rule parity. The captured project hole-to-copper value is retained but
is not yet an authoritative analytic copper/hole predicate. Region-dependent/custom rules,
pairwise clearances, per-layer rule specializations, length/skew constraints, blind/buried/microvia
fabrication policy, and frequency-dependent dielectric models remain explicit Milestone 2 work.
No impedance or propagation-delay result may claim a hard electrical constraint from absent or
unsupported material data.
