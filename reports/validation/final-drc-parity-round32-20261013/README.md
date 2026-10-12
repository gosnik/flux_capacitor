# Round32 — final electrical DRC parity

## Round32 — Explicit final electrical-clearance tolerance parity (2026-10-13)

### Failure and hypothesis
Round31 closes all Tier2 physical obligations, but the final whole-board DRC rejects six immutable input pad pairs and discards105 accepted PH runs (163 arcs/842 Beziers), falling back to straight copper. A representative USB-C pair has199872nm gap against200000nm nominal clearance. Analytic pad geometry confirms the128nm shortfall; it is not tessellation.

### Research and experiment
KiCad10.0.6 [copper clearance provider](https://gitlab.com/kicad/code/kicad/-/raw/10.0.6/pcbnew/drc/drc_test_provider_copper_clearance.cpp) applies the electrical DRC epsilon; [advanced configuration](https://docs.kicad.org/doxygen/group__advanced__config.html) documents default0.0005mm. Introduce an explicit, zero-default whole-board CopperDrcConfig. KiCad CLI final materialization opts into500nm, reported in result JSON. Planning, obstacle expansion, local transactions, dynamic checks, holes, board edges and fabrication minima remain strict. No source-object exemptions are added. Test threshold boundaries and overlap rejection; retain external KiCad validation as the final oracle.

Success requires retaining lawful curved copper without lower-tier regressions. Independent curve-envelope and unrounded-joint failures may remain after this earlier gate is corrected.

### Results and decision
All11 regression targets pass after correcting test qualification; independent review passes after adding contact rejection when epsilon exceeds the rule gap. Frozen source remained unchanged across sequential telemetry Tier0/1/2. Tier0 and Tier1 remain byte-identical and clean under KiCad,29.732s and107.985s detailed.

Tier2 global remains complete131/131, zero physical opens,166.074s. Detailed output now retains421 Beziers and163 arcs with889 tracks and75vias; zero internal copper violations and zero opens. Diagnostic KiCad export reports zero error-severity violations and zero opens. It is not yet promoted: one non-junction straight bend remains (164 residual repair attempts,163 accepted,1 rejected), producing invalid_path. Detailed runtime225.253s. Keep final-validation parity; next isolate the single rejected local corner without weakening curvature or copper validation.
