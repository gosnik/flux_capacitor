# Round33 — measured pad-core quantization guard

## Round33 — Geometry-dependent pad quantization bound (2026-10-13)

### Failure
Round32 retains lawful curved Tier2 copper but one residual GND corner cannot pass the strict transaction DRC. All nine radii report324998nm arc-centreline distance versus325000nm required at an axis-aligned rectangular pad. Connectivity and fitted tangents are intact.

### Earliest cause and experiment
The pad-arc distance kernel unconditionally subtracts2nm for vertex quantization, including exactly represented integer pad cores. This is a conservative lower bound, not evidence of actual2nm intrusion. Replace the constant with an upward-rounded bound on the maximum actual vertex rounding displacement. Point/segment/convex polygon distance is Lipschitz under that Hausdorff displacement; subtracting the bound remains conservative and becomes zero when no quantization occurs. Rounded-core radius is unchanged. No clearance tolerance is added, no corner is exempted, and every local transaction still passes strict DRC and pad preservation. Verify exact axis-aligned contact/boundary and rotated fractional geometry before benchmark promotion.

### Results and decision
All11 regression targets and independent implementation review pass. Sequential telemetry Tier0/1 remain byte-identical with zero KiCad DRC and opens (30.879s/106.809s detailed). Frozen source hash verification reports no drift.

Tier2 now passes:131/131 scaffold requests, zero physical opens, zero internal copper violations, zero unrounded bends,421 Beziers/164 arcs/889 tracks/75 vias. All164 residual bend repairs succeed. Authoritative exported KiCad error-severity DRC reports zero violations and opens. Planning161.160s; the separate end-to-end detailed command224.166s, including its own planning. PeakRSS725652KiB. Final copper layers were visually inspected (B.Cu is byte-identical to the previously inspected Round32 image).

The original harness exit1 is solely its intentional promotion guard; independent review confirms a final, non-preview, exported candidate passed all qualification gates. Promote Tier2 to qualified with30 iterations and unlock Tier3. The original exit/result remain unchanged as evidence.

Fresh sequential Tier3 remains51/148 with97 physical opens,279 tracks/20 vias,140.707s, peakRSS559696KiB, iteration limit. No final curved materialization. KRT's matched Tier3 run leaves83 opens in162.172s. Keep this general geometry correction and the Tier2 promotion; Tier3 is the next frontier.

Manifest preflight passes after promotion. Independent Tier3 scaffold export confirms97 opens and only the28 inherited footprint/silkscreen warnings; no added clearance/hole violations.
