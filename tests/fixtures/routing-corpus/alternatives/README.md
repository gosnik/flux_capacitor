# Proposed alternatives for Tiers 2–4

These are staged candidates from the [KiCadRoutingTools stress corpus](https://github.com/drandyhaas/KiCadRoutingTools/tree/ec2bf0e1021c0ed95c45b14664d02c56777a0974/tests/stress),
selected on 2026-09-07 after screening 99 boards with Flux and KiCad 9.0.8.
They have now been adopted in the active ladder in `../manifest.json`. All remain locked;
source qualification does not establish routing completion or measured difficulty.

| Proposed tier | Candidate | KRT set | Footprints | Routable nets | Required connections | Main challenge |
| ---: | --- | --- | ---: | ---: | ---: | --- |
| 2 | `ft2232h_jtag` | set5 | 41 | 34 | 117 | Compact 64-pin IC fanout, USB and dual JTAG/UART |
| 3 | `gb_nintendo_power` | set24 | 22 | 65 | 148 | 80-pad IC, flash-memory buses and card-edge fanout |
| 4 | `nascom_eprom_80bus` | set23 | 79 | 116 | 415 | Large parallel bus and repeated through-hole EPROM sockets |

For comparison, active Tier 1 has 25 routable nets and 78 required connections.
All candidates use two copper layers. Tier 4 is a substantial demand increase;
this proposed ordering must be checked by actual routing benchmarks.
The manifest's `nets` inventory also includes single-pad nets and differs from
routable nets in this table. Required connections account for pads already touching.

## Qualification evidence

All three pass strict, lossless Flux import with zero import errors and zero
core structural errors. Their prepared inputs have zero tracks, arcs, vias and
copper zones. Error-level KiCad DRC under each unchanged source project reports
only the expected 117 / 148 / 415 unconnected items, respectively. No exclusions
were added; source projects contain no DRC exclusions. Rules, pad geometry and
placements were not altered to make these candidates qualify.

The FT2232H and Nascom routed references have zero error-level DRC violations
and zero opens. **The Nintendo Power reference is not a clean routing oracle:**
it contains one starved thermal, two copper-edge errors and one open. Those
findings concern existing copper removed from the benchmark input; the stripped
input has no corresponding placement/outline blocker. They remain recorded,
and final Flux copper must still have zero errors and opens.

The default qualification uses straight track sections (`max_track_arcs: 0`).
The optional curved-mode gates are preserved. No routing attempt, tier pass or
promotion is claimed by this candidate screen.

Run the existing source preflight from the repository root:

```sh
python3 tests/python/verify_routing_corpus.py \
  --flux-cli build/dev/flux-cli \
  --corpus tests/fixtures/routing-corpus/alternatives \
  --level all --preflight-only \
  --artifacts build/krt-alternatives-review
```

Use a new artifact destination each time. Full evidence, rejected candidates and
DRC categories are in
[the screening report](../../../../reports/benchmark_history/2026-09-07-krt-tier-alternatives/README.md).

## Provenance and attribution

- **FT2232H JTAG/UART:** Kim, Jeongu / `thecasterian/FT2232-JTAG-UART`,
  revision `503ddfb760f8fc75196cef801b0203f5e2b00d07`, BSD-3-Clause.
- **Nintendo Power cartridge:** RWeick / `RWeick/DMG-A20-01-Nintendo-Power-Cartridge`,
  revision `85b24c7d2eff4973f8333ba6e65c3ce02eb74a1c`, GPL-3.0.
  Upstream acknowledges MSinger for board scans and pin-function testing.
- **Nascom EPROM card:** `codesqueak/EPROM_Card`,
  revision `dfb8a674e2897479aab17d956dff47495f1089c6`, CERN-OHL-S-2.0
  for original work in that project.

`manifest.json` records exact source paths, pinned download URLs, license texts
and SHA-256 hashes. Original PCB/project files are preserved in `source/`;
normalized routed boards and identical projects are in `reference/`. Full
upstream licenses are in `licenses/`. The modifications here were made by the
Flux project on 2026-09-07 for routing benchmarks: KiCad format normalization,
removal of tracks/vias/copper pours and non-routing board graphics, and KRT's
Edge.Cuts reconstruction. These fixtures are not fabrication-ready designs.

Preparation used `tests/stress/prep_set2.py` from KRT revision
`ec2bf0e1021c0ed95c45b14664d02c56777a0974`. To reproduce on KiCad 9, use its
`py_router/swig_compat.py` import and replace
`GetBoardPolygonOutlines(outlines, True)` with
`GetBoardPolygonOutlines(outlines, None, True)`, then invoke one process per board:

```sh
/usr/bin/python3 /path/to/compatible-prep_set2.py \
  source/BOARD.kicad_pcb reference/BOARD.kicad_pcb BOARD.kicad_pcb
```

Run from this directory with the source project beside the source PCB. The
script also preserves sibling `.kicad_pro` and `.kicad_dru` files; these three
pinned upstream repositories have no custom-rule sidecars.
