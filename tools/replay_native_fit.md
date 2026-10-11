# Native-fit replay

Build the diagnostic-only target:

```sh
cmake --build build/dev --target flux-replay-native-fit -j 4
```

Replay the first recorded regional PH candidate for a window and candidate rank:

```sh
build/dev/flux-replay-native-fit TELEMETRY.jsonl WINDOW RANK
```

The JSON result reports per-route span/piece counts and total, tessellation, lattice-filter,
line/arc-fit and tangent-biarc timings. The tool reconstructs geometry from telemetry's quantized
Bezier controls, so its output is diagnostic performance evidence only. It cannot certify routing
legality or replace the in-router native materialization, whole-board DRC, or KiCad DRC.
