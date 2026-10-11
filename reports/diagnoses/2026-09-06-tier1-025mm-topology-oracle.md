# Tier 1: legal topology at Flux's track width

KiCadRoutingTools revision `ec2bf0e1021c0ed95c45b14664d02c56777a0974`
routes `bm_svf` with 0.25 mm tracks, 0.20 mm clearance, 0.70 mm vias and
0.30 mm drills. Escalation and DRC-setting repairs were disabled. All 601
output tracks and 68 vias have these dimensions. The whole router process took
8.61 seconds and peaked at 307,984 KiB RSS; its abbreviated internal `total_time`
field is not the end-to-end timing metric.

An independent KiCad 9.0.8 error-severity DRC run reports zero violations and zero
unconnected items using a byte-identical copy of the original fixture project.
The reference reports all 78 required pad-pair connections and 25 nets connected.
Artifacts and hashes are recorded in
`reports/benchmark_history/2026-09-06-krt-bm-svf-025mm-oracle.json`.

This removes the narrower-track caveat from the earlier 0.20 mm reference. It
does not promote Flux, establish a smooth-curve solution, or provide a precisely
matched runtime comparison. Reference vias are larger than Flux's current
0.60 mm effective default, satisfying the project's 0.70 mm minimum. Flux's
project-rule ingestion remains a separate correctness audit.

## Small conflict's structural difference

Flux's ordinary window 0 forces two fragments into F.Cu channel 61: net 25
(`VACTROL_POS`) and net 26 (`Net-(R17-Pad2)`). Their aggregate envelopes require
1.100 mm versus a recorded 1.092 mm channel. Both recorded domains are singletons;
search cannot create an absent alternative.

The reference makes different decisions:

- Net 25 uses both copper layers, with vias at (69.2, 56.8), (66.4, 56.8) and
  (63.7, 55.7) mm. Near the conflict it uses B.Cu horizontally, changes to F.Cu,
  and approaches its pad from the west.
- Net 26 stays entirely on F.Cu with no vias. Its pad at (67.306, 56.642) mm
  escapes southeast toward (68.2, 57.5) mm, avoiding Flux's forced northern
  shared-channel arrangement.

These coordinates describe the diagnosis only. They must not become algorithm
conditions or canned output. The evidence favours terminal/corridor/layer
alternatives with sufficient rerouting scope, not relaxing channel capacity or
trying harder to smooth the same forced fragments. The V17 propagation experiment
is still being evaluated separately; it changes search ordering, not topology.
