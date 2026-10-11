# Experiment: exact outside-fragment feedback

## Failure and baseline

Tier 0, pt2399_delay, frozen-context trial: 431.656 seconds, 50 paths,
zero unrouted nets, 21 native candidate checks, 87 local native violation
reports, three legal candidates, unresolved window 3, no assembly/final DRC.
Baseline is preserved in `2026-09-06-frozen-context-comparison.json`.

Native local assessment discarded the outside object's identity. Scope retry
only consumed full-assembly/linear-scaffold diagnoses. Rejecting an unsafe
candidate earlier thus prevented the full-assembly witness needed to enlarge
its repair region. Nearest-net mapping also cannot reliably identify an
original F.Cu fragment whose candidate has moved to B.Cu.

## Ranked hypotheses

1. Preserve exact candidate/blocker fragment IDs and couple their repair scope
   after the retained candidate fails. Selected minimum experiment.
2. Regenerate path templates against frozen geometry. May help routing around
   a fixed blocker but cannot coordinate necessary movement of that blocker.
3. Global portfolio feedback for these same pairs. Appropriate if bounded
   regional coupling proves infeasible, but a larger intervention.

## Research and integration

MA-CBS couples interacting agents after repeated conflicts:
https://ojs.aaai.org/index.php/SOCS/article/download/18244/18035/21760
Flux can reuse the concept of selective coupling, not its optimality result:
PCB copper is persistent spatial occupancy, not point agents moving in time,
and Flux's bounded regional solver is not complete. Joint search can grow
exponentially with group size. Existing fragment/net/window/search bounds must
remain authoritative. No external source code was copied.

The experiment records exact provenance when adding frozen tracks, then pairs
native owned fragments with frozen blockers from exact copper violations.
Only the retained unresolved assignment supplies scope witnesses; failed
alternatives do not all become constraints. Pair/layer deduplication prevents
identical retries. Exact IDs seed the existing physical conflict graph instead
of a nearest-net lookup, including across candidate layer changes. Scope bounds
cover both implicated original fragments, conservatively increasing work.

Flag: `--expand-frozen-copper-scope` (implies frozen context), default off.
Per-candidate telemetry and unresolved-window JSON preserve pairs, layer, code
and clearance deficit. Final native checks and KiCad acceptance are unchanged.

## Verification plan

Test provenance, fragment-level ownership, layer-change identity, deduplication,
and exclusion of complete assignments/already-owned blockers. Run core and
arc/multilayer export/DRC checks, then the same N6/R0/enforced/full-strength
Tier-0 trial with scope expansion. Success requires better complete copper,
not just more retries or suppressed assembly. Record timeout as inconclusive
routing quality, retaining any exact conflict evidence as diagnostic progress.

## Outcome

The normal 600-second trial timed out during the second scope retry. No final
planning JSON, accepted board or KiCad DRC was produced. All 10 selected checks
passed beforehand (173.94 seconds, including the core suite and two KiCad DRCs).

Pair-level evidence now identifies:

- F.Cu candidate `kicad-net-23:1:0:1` against untouched
  `kicad-net-4:0:0:8` and `:9`, clearance deficits 318916 and 450000 nm.
- B.Cu alternatives against GND `kicad-net-3:5:0:4`, which was outside
  the original two-net window; recorded crossing deficit 450000 nm.
- After coupling that GND fragment, some candidates switch it to F.Cu and
  collide with further +5V continuations. This is evidence of another ownership
  boundary, not proof that no joint solution exists.

At 437.168 seconds the first regional analysis ended with 3/4 complete bundles,
21 evaluated candidates, then exact scope grew 4 → 5 pairs. At 571.961 seconds
the first retry still had 3/4 complete bundles; generated discrete candidates
grew 228 → 654. It then grew scope to eight pairs. All pairs mapped, none were
lost. The second retry was cut off at the normal limit. Structured result:
`reports/benchmark_history/2026-09-06-frozen-scope-feedback.json`.

The feedback mechanism works; copper improvement is not established. Retain
diagnostics and leave scope expansion off by default. A single identical-binary
900-second diagnostic trial was run to finish the existing bounded retry,
not to qualify the unchanged normal 600-second benchmark. The corpus runner's
new `--planning-timeout-seconds` override is explicit in metadata, validated
to 1..3600, and does not edit the corpus manifest.

The extended trial finished in **730.098 seconds**, still incomplete: 50 paths,
zero unrouted nets, total global overflow 19 (unchanged from both controls),
two scope retries. The last regional analysis generated 3940 discrete states,
evaluated 10 spline candidates, and found 4 locally legal candidates. The sole
reported unresolved window moved from the REF/+5V/GND region to window 3,
`kicad-net-17` (+9V) / `kicad-net-4` (+5V). Its retained templates [0,7]
are both F.Cu and locally native-legal.

Last assembled-native counters: 2 conflicts, both classified fixed, versus 39
in the earlier full-strength-only control. These are **last assessed scene**
counters, not proof that the final retained state was reassembled: the manifest
is withdrawn after a failed local bundle and the native counters can remain.
The existing output omitted detailed native-scene diagnoses, so this artifact
does not establish the exact two remaining object pairs or DRC categories.
Do not attribute them to vias merely because proposed vias remain a known
local-check coverage gap. No accepted board or final KiCad DRC was produced.

Interpretation: exact fragment feedback and bounded coupling are partially
supported: the original region is no longer the unresolved frontier. The tier
and its normal runtime budget still fail. Keep scope expansion opt-in; retain
the provenance instrumentation. Next, classify the remaining assembled-native
violations before another routing-policy change. As a post-benchmark
observability fix, JSON now exports `regional_last_native_scene_diagnoses`
(including materialized object IDs) and `regional_native_scene_manifest_available`.
This changes reporting only, not acceptance or search. Comparison:
`reports/benchmark_history/2026-09-06-frozen-scope-feedback-final.json`.

## Differential topology evidence

Read the preserved reference with KiCad's PCB parser (no source changes):
`tests/fixtures/routing-corpus/reference/pt2399_delay.kicad_pcb`, manifest hash
`0ffb5710e07ee6f65bb6951a9ca1ebf9d52bb701852b1e9f83696991043d7eb6`.
This inspection is topology evidence, not a fresh reference-board DRC run.

REF runs on F.Cu from (30,41) to (30,45.04), then (29.54,45.5).
The +5V branch from (27.46,41) escapes left to (25.33,43.13), continues to
(25.33,46.191737), then reaches the lower terminal at (31.25,51).
GND crosses B.Cu at y=43.5 from x=27 to x=35.794887. Thus the reference
avoids the problematic F.Cu crossing by changing +5V's broader corridor,
not by moving REF onto the occupied back-layer channel. No tracks are copied
and no coordinates or net names enter router heuristics.

Whole-connection/corridor alternatives or candidate generation against actual
outside copper remain promising ways to avoid expensive downstream expansion.
The extended run found locally legal candidates in the original region, so
the reference does NOT establish that local joint repair is impossible. The
immediate remaining failure is the separate assembled-native boundary above.
