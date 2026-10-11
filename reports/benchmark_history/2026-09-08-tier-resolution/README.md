# Current state — V60

Tier 0 qualifies in Release and Debug with exact copper and KiCad DRC. Tier 1
remains the frontier; Tiers 2–4 remain locked. Splines are preserved and ordinary
H/V/45 straight output remains the default and a mandatory qualification gate.

Retained: exact via failures now reach regional conflict extraction with their
original attached route-branch identities. The controlled Tier 1 run reduces
unmapped regional pairs from 100 to zero; 19/20 assignments complete at 32 states.
All 51 core tests pass. The broader suite passes 127/134 with the same seven known
baseline demo failures.

Reverted: V59 legal-angle via approaches recovered one local assignment but
worsened several whole-board scenes. Its full 1754.7-second run did not qualify.

The V60 256-state follow-up assembled scenes with 817 then 812 exact violations,
zero opens, and timed out at 600 seconds. The independently reproduced second
scene's rejected detailed preview has 241 KiCad errors and zero unconnected items;
two non-H/V/45 segments also fail the angle gate. No tier was promoted.

- [Experiment decisions](V60-experiment.json)
- [Structured run comparisons](V59-V60-benchmarks.json)
- [Current physical failure and next hypotheses](frontier-diagnosis.json)
- [Exact and external scene verification](V60-frontier-external-verification.json)
- [Focused retained patch](V60-exact-via-owner-feedback.patch)
- [Reverted experiment](V59-experiment.json)

No benchmark processes remain running. Full raw artifacts are preserved under
`build/tier-resolution-v59/` and `build/tier-resolution-v60/`.

## Previous experiments and import correction

# Tier-resolution continuation — V55–V58

Tier 0 remains qualified in Release and Debug. Tier 1 remains the frontier; the full V56 run completed in 1,591.087 seconds without qualifying. Tiers 2–4 keep their existing clean KRT inputs.

The principal correction is canonical KiCad geometry. Back-side pads were mirrored twice, displacing eight Tier-1 pads by 2.54 mm. Missing pad angles were incorrectly relative to the footprint. Drill offsets moved the wrong anchor. These conversions now match KiCad's independent loader. No benchmark input, project rule or legal clearance changed.

- [Pad-position failure evidence](V56-pad-position-diagnosis.json)
- [Full pad/drill parity](V57-pad-geometry-parity.json): 1,163 active-tier pads plus 32 synthetic cases
- [Connectivity parity](V56-open-detection-parity.json): old invalid preview 0→9 internal opens (KiCad 9); known-connected KRT reference 8→0 opens
- [Rejected corrected preview](V56-diagnostic-external-verification.json): 271 KiCad errors, zero opens, ordinary angles only
- [Layer-assignment probe](V58-external-verification.json): 265 KiCad errors, zero opens; diagnostic only
- [Structured continuation summary](summary.json)

Verification: 50/50 core tests; broader CLI suite 126/133 with the same seven baseline demo failures. Both development and Release CLIs rebuilt. The scheduling experiment V55 was reverted when the earlier import error was discovered; its partial control and patch remain available. Successful import changes are preserved as a focused patch against the existing dirty baseline.

The completed run checked nine assembled straight scenes, all with zero opens and nonzero copper errors. Final selection retained a 45/47 discrete assignment; its unchecked scene counters are reported as unknown in [the structured result](V56-completed-tier1.json). The [smallest unresolved region](frontier-diagnosis.json) contains two nets and a rejected via/run-up combination.
