# Independent KiCad pad-geometry comparison

Build `flux-inspect-pad-geometry` explicitly. It strictly imports a board with its
sibling project rules and prints the canonical copper-pad positions, dimensions
and angles. It writes no PCB files and cannot qualify routing.

Compare it with KiCad's independent loader using the Python environment that
provides `pcbnew`:

```sh
cmake --build build/release --target flux-inspect-pad-geometry
/usr/bin/python3 tests/python/verify_kicad_pad_geometry.py \
  --inspector build/release/flux-inspect-pad-geometry \
  --report build/pad-geometry-parity.json \
  tests/fixtures/pad-coordinate-frames.kicad_pcb \
  tests/fixtures/routing-corpus/bm_svf.kicad_pcb
```

The comparison requires identical pad identities and exact integer positions,
dimensions and angles (modulo a full turn). Pads without copper layers are omitted
by both loaders. Offset-pad copper centers are compared with KiCad ShapePos;
physical hole centers are independently compared with KiCad GetPosition.
