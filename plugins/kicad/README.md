# KiCad IPC bridge

## Minimal autoroute workflow

`route_board.py` is the end-to-end routing-only entry point. It runs the fixed bounded
`autoroute_v1` preset, writes only to a new candidate path, verifies the source-bound change set,
and invokes `kicad-cli pcb drc` when available:

```sh
python3 plugins/kicad/route_board.py \
  --board board.kicad_pcb \
  --output board.flux-routed.kicad_pcb \
  --flux-cli ./build/dev/flux-cli
```

Pass `--netclasses board.flux_netclasses` for an existing authoritative snapshot, or use
`--capture-netclasses` with the matching saved board open and the KiCad API server enabled. Add
`--diffpairs` and `--buses` only for reviewed source-bound sidecars. The JSON result contains the
engine decision, candidate path, exact operation counts, and external DRC status.

Preview is non-mutating. `--apply-routing` delegates to the live candidate bridge only after an
external DRC pass and creates one unsaved undo transaction. It never saves the board, and any
footprint transform remains withheld. `--skip-external-drc` is provided for offline environments
but disables live application.

The lower-level adapters remain available for explicit workflows.

### Refillable-zone routing

For boards whose saved pours disconnect otherwise usable multilayer free space, opt into the
file-level refill transaction:

```sh
python3 plugins/kicad/route_board.py \
  --board board.kicad_pcb \
  --output board.flux-refilled.kicad_pcb \
  --flux-cli ./build/release/flux-cli \
  --netclasses board.flux_netclasses \
  --refillable-zones \
  --repair-sampled-conflicts
```

Flux routes incomplete nets with saved fill polygons absent from planning clearance, but the C++
candidate remains `zone_refill_required`. The adapter verifies its source-bound routing changes,
refills all zones into a temporary file with KiCad, strictly re-imports it, and runs whole-board
`kicad-cli pcb drc`. The requested output does not exist unless every gate passes. Repeat
`--route-net NAME` to run a deliberately partial experiment; whole-board external DRC still applies,
so a board with other open nets will correctly publish nothing. This mode is review-only and is
incompatible with `--apply-routing`, integrated placement, and `--skip-external-drc`.
`--repair-sampled-conflicts` retains the best negotiated checkpoint, removes only nets owning a
reported sampled or shared-physical conflict, broadens their legal terminal portals, and reroutes
that bounded cone with hard capacity. The original checkpoint remains selected unless the repair
is objectively better.

## Integrated placement and routing workflow

Use `place_route_v1` when placement should respond to routing pressure before the candidate is
reviewed:

```sh
python3 plugins/kicad/route_board.py \
  --board board.kicad_pcb \
  --output board.flux-place-routed.kicad_pcb \
  --flux-cli ./build/dev/flux-cli \
  --integrated-placement
```

This mode runs routing-guided global placement and the full detailed-routing acceptance pipeline.
It defaults to two placement passes and 64 routed candidate evaluations. In the absence of
`--placement-move-priorities`, it uses the complete suggested resistor/capacitor/transistor/other/
miscellaneous-IC/MCU/SoC/connector staged order and reports that prefix-derived classification as a
review reason. A reviewed source-bound priority sidecar replaces the suggestion. The ordinary
`--netclasses`, `--diffpairs`, and `--buses` policies remain available; placement envelopes, rigid
groups, regions, and relative constraints have corresponding options.

Add `--tentative-envelopes` only when provisional pad/Fab/Silk bounds are acceptable. The result
will explicitly require mechanical review. The workflow verifies the exact source-bound change
set and external KiCad DRC just like `autoroute_v1`, but every integrated result remains a new
candidate file. `--apply-routing` is rejected because applying only its routing would separate the
accepted copper from the placement on which it was validated, while live footprint mutation is
still withheld.

Integrated placement proposals may use tentative geometry when source courtyards are missing:

```sh
./build/dev/flux-cli global-optimize-kicad board.kicad_pcb \
  --tentative-envelopes board.flux-placement-review.kicad_pcb
```

The candidate JSON must report `human_placement_review_required: true`. Tentative bounds combine
pads and supported Fab/Silk graphics; they are not exact component bodies. The live bridge refuses
all footprint transforms, so review and acceptance occur through the new candidate file only.

To move small components before mechanically disruptive parts, generate and review a source-bound
priority file, then pass it to an integrated placement command:

```sh
./build/dev/flux-cli placement-move-priorities-template-kicad board.kicad_pcb \
  > board.flux_move_priorities
./build/dev/flux-cli global-optimize-kicad board.kicad_pcb \
  --tentative-envelopes \
  --placement-move-priorities board.flux_move_priorities \
  board.flux-placement-review.kicad_pcb
```

The default policy unlocks resistors first and connectors/pin headers last. Review `U*` entries and
mark actual MCUs and SoCs explicitly; reference prefixes cannot distinguish those classes. The
policy changes search order only. Candidate placement remains preview-only and subject to the same
post-process mechanical review and exact routing/DRC gates.

`export_netclasses.py` is the first thin live-KiCad adapter. Snapshot version 3 uses the official
`kicad-python` IPC binding to read explicit project net classes and each board net's effective
constituent classes. Because the installed official IPC messages do not expose board-wide minimum
constraints, it separately reads the matching saved `.kicad_pro` once, captures the eight supported
fabrication-rule fields, and binds their SHA-256 project revision alongside the board revision. It
then emits one deterministic source-bound snapshot for the C++ engine. It never edits or saves the
live board or project.

With the board open and the KiCad API server enabled:

```sh
python3 plugins/kicad/export_netclasses.py \
  --board board.kicad_pcb --flux-cli ./build/dev/flux-cli \
  > board.flux_netclasses
./build/dev/flux-cli detail-kicad board.kicad_pcb --netclasses board.flux_netclasses
```

KiCad 10 requires a running GUI instance. The bridge requires the maintained `kicad-python`
package and deliberately imports it only at the IPC boundary. The snapshot consumer verifies the
exact board and project revisions, complete named-net coverage, class definitions, memberships,
the complete supported board-rule set, and canonical rule validity before routing.

Save the live board before capture. KiCad 10's IPC document descriptor exposes the active board
path but not a dirty-state flag. The bridge checks that the requested and active paths match, asks
the C++ importer to fingerprint the saved bytes, and rejects a file that changes during capture;
it does not silently save or overwrite the live document.

`apply_candidate.py` previews or applies a candidate that was already written by Flux's
source-preserving exporter. The C++ engine first reproduces that candidate byte-for-byte from the
exact source and emits a versioned change set with complete source inventories. Preview is the
default and does not begin a commit:

```sh
python3 plugins/kicad/apply_candidate.py \
  --board board.kicad_pcb --candidate board.flux-candidate.kicad_pcb \
  --flux-cli ./build/dev/flux-cli
```

For offline contract verification without a running KiCad instance, add `--validate-only`. This
still regenerates the exact change set from the source and candidate and validates the current
change-set version, board schema, inventories, operations, and source revision, but performs no
live-board comparison or mutation.

After reviewing the reported operations, straight tracks, circular track arcs, and vias can be
applied to the open board as one unsaved undo step:

```sh
python3 plugins/kicad/apply_candidate.py \
  --board board.kicad_pcb --candidate board.flux-candidate.kicad_pcb \
  --flux-cli ./build/dev/flux-cli --apply-routing
```

The adapter requires exact live footprint and routing membership/state, repeats that comparison
inside the commit, checks KiCad's returned routing objects, and drops the open commit on any
mismatch. It never calls a save API. If the candidate moves or rotates a footprint, preview still
reports the transform but `--apply-routing` refuses the complete operation. Current upstream
reports show whole-footprint IPC updates can damage schematic linkage, so placement must continue
through the verified candidate file until that API path is demonstrably safe.
