# Round27 — Per-net residual demand admission

Round26 ends with one unresolved explicit request but14 physical opens across5 nets. The residual generator and engine require whole-scaffold completion, delaying actionable real connectivity work on otherwise completed nets. Hypothesis: admit residual demands per completed net, retaining existing prefix and blocking duplicate work on nets with unresolved obligations. Preserve all whole-board completion/DRC gates.

Request115 offline replay has128 source/16 target attachments, exhausts50k expansions in2.330s and introduces no new DRC. Increasing its budget alone was previously unhelpful; defer that independent mechanism.

### Results and decision
All11 regression targets pass, final no-op build and independent review pass. Source manifest unchanged throughout sequential telemetry benchmarks. Tier0 passes51/51, zero opens/internal/external DRC, byte-identical copper,30.432s detailed. Tier1 passes91/91, zero opens/internal/external DRC and unrounded non-junction bends,106.411s detailed; primitive counts unchanged499tracks/125arcs/359Beziers/57vias. Its file ordering changes, but measured geometry signatures match the prior run.

Tier2 improves physical opens14→1, scaffold117/118→130/131,309.399s versus278.111s, peakRSS1274020KiB. Thirteen residual obligations are ultimately appended and completed; net8/request115 remains. Keep the change: the observed mechanism was incomplete-net starvation of physical demand generation. This is not Tier2 completion: no final curved materialization or external final-board promotion.

At the user's explicit request, fresh Tier3 comparison runs on the same frozen build:140.902s, iteration limit,51/148 scaffold demands,97 physical opens, peakRSS559280KiB. No final materialization. KRT matched dimensions gives10 opens onTier2 in14.766s and83 onTier3 in162.172s; both incomplete under independent KiCad validation. Detailed KRT configuration and comparison are recorded separately.

Next hypothesis: remaining /TCK_TXD_A is searched at0.25mm nominal width, while the known-good reference uses0.20mm with one0.60/0.30 via. Legal attachment counts do not prove a reachable free-space corridor, and final-reset via counters cannot exclude via-state growth. Test width feasibility with a controlled offline replay before implementing local width alternatives; do not simply increase global budgets or lower clearances.

Independent post-run provisional scaffold export confirms KiCad unconnected items1/97 onTier2/3 and no clearance/hole violations. Tier2 has21 track_dangling warnings plus41 inherited footprint warnings; Tier3 has only22 footprint/6silk warnings. This is straight scaffold evidence, not final curved certification.
