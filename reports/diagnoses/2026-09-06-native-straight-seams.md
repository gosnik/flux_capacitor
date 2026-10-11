# Native straight adapters block complete curve realization

Tier 0 pt2399_delay, N1 seed-first detail: 0 opens, 0 internal copper violations,
84 added tracks, 261 added arcs, 5 vias. All 54 board-level PH tasks committed,
but final curve audit rejects six degree-two straight/straight bends.
Replay: `tools/diagnose_unrounded_copper.py` against the control detail telemetry.

Exact net/layer/point (mm): +9V/F.Cu (37,58.5); REF/B.Cu (32.1,45.6);
GND/B.Cu (26,24.5), (28,44); +5V/F.Cu (28.5,41.5), (31,44).
Every pair contains at least one `regional-native-track` straight piece. Five are
native/ordinary boundaries; REF is a native/native right-angle adapter. Existing
native ownership protects all these segments from the board-level chain collector.
The excluded bends are never offered to PH realization; this is not a solver
budget problem. Final feedback currently misclassifies this as a netless topology
failure, so global rerouting cannot target the six specific seam defects.

Candidate ranking:
1. Let unlocked native straight adapters participate in full-board degree-two PH
   realization, while native arcs remain immutable and exact merge checks remain.
   Straight runs with no bend remain untouched. Verify arc-boundary behavior.
2. Add a separate constrained local seam-repair pass with pinned arc tangents;
   more machinery, useful if (1) cannot maintain good boundary joins.
3. Relax the final bend audit: rejected; violates the core flowing-trace objective.

Minimum experiment is (1) with a synthetic native two-segment bend. Keep the
existing native-arc byte-for-byte test and final connectivity/copper validation.
Do not infer KiCad correctness from the internal zero-violation result.
