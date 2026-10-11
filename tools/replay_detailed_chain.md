# Detailed-chain replay

Build the diagnostic target explicitly:

```sh
cmake --build build/release --target flux-replay-detailed-chain -j 2
build/release/flux-replay-detailed-chain SOURCE.kicad_pcb TELEMETRY.jsonl CHAIN.json
```

The chain JSON supplies `net`, `layer`, `width_nm`, exact `track_ids`, and ordered
`points_nm` objects with integer `x` and `y` fields. Each consecutive point pair
must match one removed recorded track. See the diagnostic example in
`reports/diagnoses/2026-09-06-tier0-staircase-chain.json`.

The tool strictly imports the original board, finds the last generated-copper
snapshot, verifies its source revision, freezes the surrounding recorded copper,
and reruns only the selected chain through ordinary detailed routing. It rejects
missing/mismatched track IDs and unsupported generated primitives. It writes no
board or source files. Exit 0 means internal detailed completion, 3 means detailed
rejection, and 1 means invalid replay input.

Output is diagnostic JSON (Boost property-tree scalar values are strings).
The `seed_probes` section samples the exact clearance-offset domain at original
chain nodes and at ordinary/run-aligned corner-seed points. It reports signed
clearance, worst position and shape failures for a bounded guard portfolio.
These samples locate interpolation overshoot; they are not continuous legality
certificates and cannot qualify a candidate. The ordinary detailed-router replay
still runs all production continuous/topology/native-copper checks afterward.
This is not qualification: surrounding *final* copper may differ from the board
seen by concurrent PH tasks during the original run. Source-bound full-board
rerouting and KiCad DRC remain required. The chain data never influences normal
router policy and contains no executable board-specific heuristic.
