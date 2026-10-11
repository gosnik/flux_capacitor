# Real-board routing corpus

This directory replaces Flux's generated 4--20 component routing boards with a deliberately small
progression derived from the open-source
[KiCadRoutingTools stress corpus](https://github.com/drandyhaas/KiCadRoutingTools/tree/main/tests/stress)
and its [real-board routing discussion](https://github.com/drandyhaas/KiCadRoutingTools/discussions/117).

The important policy is monotonic progressive activation. A qualified tier always remains in the
routine run, and permanently unlocks the next tier as the single capability probe. Higher
levels remain available for focused inspection, but are not silently added to every build merely
because their files exist.

| Level | Board | Upstream stress set | Current gate | Purpose |
| ---: | --- | --- | --- | --- |
| 0 | `pt2399_delay` | set13 | mandatory, passing | Requalified under original project rules; 50 connections, zero errors/opens; straight tracks |
| 1 | `bm_svf` | set15 | unlocked | Always-run analog module probe; 78 seeded connections with residual overflow |
| 2 | `ft2232h_jtag` | set5 | locked | 117 connections; compact 64-pin USB/JTAG fanout |
| 3 | `gb_nintendo_power` | set24 | locked | 148 connections; 80-pad IC and parallel memory buses |
| 4 | `nascom_eprom_80bus` | set23 | locked | 415 connections; repeated DIP sockets and card-edge bus |

`manifest.json` is authoritative for source repository, exact commit, original path, license,
source hash, prepared-board hash, routed-reference hash, project hash, inventory, and current gate.
The board beside each project is the stripped routing input. `reference/` contains the normalized
human-routed board so later quality grading can compare against ground truth rather than assuming
zero source DRC findings. Full upstream license texts are under `licenses/`.

The 2026-09-06 rule-context audit found that the final DRC candidate lacked its
sibling project and was checked under KiCad defaults. The unchanged Tier-0 copper
has 199 track-width and 9 drill-spacing errors under the original project, with
zero opens. The manifest keeps Tier 0's mandatory `qualified` gate so the corrected
test fails visibly; it has not been downgraded to hide the regression. Tier 0 must
be genuinely recovered before promoting Tier 1. See
`reports/diagnoses/2026-09-06-tier0-project-rule-context.json` at the repository root.

Tier 0 was genuinely restored on 2026-09-07 after source-rule import, source-hole
preservation, committed-context retries and run-aligned PH corner seeds. The
curved-mode corpus invocation at that point passed: 82 tracks, 152 arcs, four vias, zero
unfinished bends and clean original-project KiCad DRC. An unspecified CLI pitch
is raised to the shared validator's source-rule floor; explicit values remain
validated. See `reports/benchmark_history/2026-09-07-default-rule-pitch.json`.
Tier 1 remains the frontier; its latest 600-second diagnostic probe timed out in
regional repair, and its 78 selected paths are not legal final copper.

## Preparation

The inputs were prepared from pinned upstream revisions with KiCadRoutingTools commit
`661c88b3c3b0ebc9ade65eab63d30b1cde217fb4`, using `tests/stress/prep_set2.py` and pcbnew 9.0.8.
The only compatibility adjustment supplied a null error-handler argument to the KiCad 9
`GetBoardPolygonOutlines` overload. Preparation normalized the source, removed tracks, vias and
copper zones, retained rule areas, rebuilt Edge.Cuts, removed non-routing graphics, and preserved
the sibling project settings. This is recorded in the manifest rather than treated as upstream
source identity.

## Running the progression

The ordinary test runs every qualified tier plus the first unlocked tier:

```sh
ctest --test-dir build/dev -R flux_cli_routing_corpus_unlocked --output-on-failure
```

Inspect progressively larger capability levels without routing every board:

```sh
python3 tests/python/verify_routing_corpus.py \
  --flux-cli build/dev/flux-cli \
  --corpus tests/fixtures/routing-corpus \
  --level 2
```

`--level all` verifies provenance and strict import for the whole curated set. Locked tiers are not
routed unless they become unlocked. A qualified board must keep completing; the unlocked board
always receives a bounded routing attempt. If that attempt satisfies its qualification target, the
harness fails with an explicit promotion request: change it from `unlocked` to `qualified` and
change the next locked tier to `unlocked` in the same revision. This makes unlocking permanent
without allowing a test process to mutate policy. The source and reference files are hashed before
and after every run.

Every tier now requires `detailed_complete`; a zero-overflow global plan alone cannot trigger
promotion. `detailed_complete` is the final-copper qualification. Such a tier must produce an
exported KiCad
board with zero structural errors, open connections, and represented copper violations. Only after
those exact gates and an error-level external KiCad DRC pass does the harness render one
2400-pixel PNG per enabled copper layer (with `Edge.Cuts` as context) under the configured artifact
directory. Incomplete previews are never labelled or rendered as final copper. CTest writes these review artifacts below
`build/release/tests/routing-corpus-artifacts/`.

Final DRC now requires a byte-identical project and any custom-rule sidecar under
the candidate's basename. Their hashes are recorded in `kicad-drc.context.json`.
They are also published beside accepted copper. Rejected copper and its rule
context are retained under `analysis/kicad-drc-rejected`, not as final output.

Every tier run records diagnostic telemetry and writes bounded, non-authoritative analysis PNGs
under `artifacts/tier-<level>-<board>/analysis`. The images are emitted for successful,
incomplete, and timed-out routing attempts. `--failed-route-png-dir` remains available only to
override that analysis root, for example:

```sh
python3 tests/python/verify_routing_corpus.py \
  --flux-cli build/dev/flux-cli \
  --corpus tests/fixtures/routing-corpus \
  --board bm_svf \
  --failed-route-png-dir build/failed-route-previews \
  --failed-route-png-mode both \
  --max-failed-route-png-results 8
```

The harness ranks completed regional candidates by topology
and mutual-clearance failures, and writes layer copper, whole provisional-board, full-context
quintic PH Bezier controls, and automatically cropped control close-ups. A JSON manifest records the candidate/window identity and explicitly marks
the output as diagnostic rather than accepted copper. `copper` and `spline` modes may be selected
individually. Rendering also runs after a board timeout, using every complete telemetry event that
was flushed before cancellation.

Useful KiCadRoutingTools ideas retained in Flux's harness are immutable pinned sources, paired
human references, manifest provenance, bounded per-board execution, deterministic replay inputs,
and progressive corpus qualification. Their large queue/agent orchestration is intentionally not
copied into the routine test path.

The current default is straight-section routing. Tier 0 has been rechecked repeatedly:
243 horizontal/vertical/45-degree tracks, zero arcs, five vias, all 50 connections and clean
original-project KiCad DRC. The runner independently audits exported segment angles. The arc-count and rounded-bend requirements are retained in `curved_expected`
and selected by `./run_tiers.sh --track-geometry curved`. Fabrication, connectivity and
external DRC requirements are identical in both modes. Tier 1 remains unqualified.
See `reports/benchmark_history/2026-09-07-tier-resolution/summary.json` for the current
experiments and `reports/benchmark_history/2026-09-07-straight-routing-summary.json` for
the earlier straight-mode baseline and rejected controls.

## Adopted alternative boards

A separately staged [alternative Tier 2–4 corpus](alternatives/README.md) contains
FT2232H JTAG/UART, Nintendo Power cartridge and Nascom EPROM inputs with clean
strict-import and stripped-board DRC checks. Selection evidence and reference
caveats are documented there. These boards now replace the source-blocked levels 2–4 in the active manifest;
all remain locked and unqualified. The former Zigbee, CH32V003 and USB-GPIB
inputs remain in `compatibility/manifest.json` as source-compatibility regressions.
The existing Tier-0 and Tier-1 entries are byte-for-byte equivalent JSON objects.
New boards carry their KRT preparation revision per entry; initial provenance
above continues to describe levels 0–1 and the compatibility corpus.
