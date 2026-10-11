# Extended curved tier run — completed captures, two interrupted runs

Immutable Release binary: `dc7760fff714dfa6357bff063cc86bd76702fc7f1d1cbe3a49f010d6890e11c8`. Each stage has a 10,740-second engine deadline and 10,800-second outer limit. Runs are concurrent; timings are not isolated performance comparisons.

| Tier | Board | Status | Copper / validation |
|---|---|---|---|
| 0 | pt2399_delay | Exhausted after 6451.4 s detailed stage | Actual rejected candidate: 142 arcs, 6 unrounded bends, external KiCad 1 unconnected item, 0 other violations |
| 1 | bm_svf | Interrupted without final result | Intermediate candidates only; no timeout or OOM attribution established |
| 2 | ft2232h_jtag | Exhausted after 9305.4 s planning | 117 coarse paths, 0 coarse unrouted nets, 381 overflow; no detailed copper |
| 3 | gb_nintendo_power | Exhausted after 422.6 s planning, 30-iteration budget | 1 coarse path, 64 unrouted nets; no detailed copper |
| 4 | nascom_eprom_80bus | Interrupted without final result | Intermediate candidates only; last regional summary 4463 conflicts |

## Visual review

Actual Tier 0 front/back copper has many smooth runs but still includes sharp corners and a large U-shaped detour around an IC pad. It does not satisfy the requested all-smooth, logical, legal result. Its PNGs are byte-identical to a separate 688.98-second control limited to one global iteration. Both fail with six unrounded bends and one external unconnected item.

Other reviewed images are explicitly non-authoritative telemetry reconstructions. Tier 1 and Tier 2 show angular scaffolding, apparent intersections and long detours. Tier 3 has too little routing to assess curves. Tier 4 has board-wide coarse scaffolding and isolated local spline candidates. Visual crossings alone are not counted as authoritative copper shorts.

## Supported pipeline findings

- A pending regional action resumes across cooperative slices; the scheduler chooses its next algorithm only after the action returns. Large transactions can consume most of a deadline.
- Detailed action scoring and best-partial ordering omit the unrounded-bend count, although final acceptance rejects those bends. The control consequently reports zero materialization quality cost while remaining incomplete.
- XPBD is enabled in the frozen CLI and was exercised. An earlier suspected default mismatch was disproven by checking the frozen source and the completed action trace.
- The external +5V disconnect is not simple endpoint serialization drift: pcbnew loads the reported arc start exactly at U2 pad 1 centre. The cause of KiCad's connectivity result remains unresolved.

## Artifacts

- Gallery: `build/tiers-curved-extended-release-20261008/index.html`
- Structured benchmark: `reports/benchmark_history/2026-10-08-curved-extended.json`
- Visual evidence: `reports/diagnoses/2026-10-08-curved-extended-visual.json`
- Actual Tier 0 candidate and DRC: `build/tiers-curved-extended-release-20261008/pt2399_delay/tier-0-pt2399_delay/analysis/detailed-rejected/`

No benchmark tier has been promoted.
