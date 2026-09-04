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
| 0 | `pt2399_delay` | set13 | qualified | Exact final copper; 50 connections, KiCad DRC clean, layer PNGs |
| 1 | `bm_svf` | set15 | unlocked | Always-run analog module probe; 78 seeded connections with residual overflow |
| 2 | `zigbee_sensor` | set9 | locked | RF-oriented layout and circular courtyard proxies |
| 3 | `ch32v003_usb` | set6 | locked | USB routing plus unsupported/non-copper pad geometry |
| 4 | `usbgpib` | set5 | locked | Fine-pitch VQFN and dense connector fanout |

`manifest.json` is authoritative for source repository, exact commit, original path, license,
source hash, prepared-board hash, routed-reference hash, project hash, inventory, and current gate.
The board beside each project is the stripped routing input. `reference/` contains the normalized
human-routed board so later quality grading can compare against ground truth rather than assuming
zero source DRC findings. Full upstream license texts are under `licenses/`.

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

`detailed_complete` is the final-copper qualification. Such a tier must produce an exported KiCad
board with zero structural errors, open connections, and represented copper violations. Only after
those exact gates and an error-level external KiCad DRC pass does the harness render one
2400-pixel PNG per enabled copper layer (with `Edge.Cuts` as context) under the configured artifact
directory. Incomplete previews are never labelled or rendered as final copper. CTest writes these review artifacts below
`build/release/tests/routing-corpus-artifacts/`.

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
