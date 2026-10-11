# KiCadRoutingTools versus Flux on `bm_svf`

Date: 2026-09-05

## Reproducible inputs

**2026-09-06 comparison caveat:** this is not a matched-width Flux/KRT experiment.
Current corpus invocations do not pass a `--netclasses` snapshot. Flux's selected
traces use its 0.25 mm default, whereas this KRT run requested the project's
0.2 mm track width. The sibling project also has a 0.7 mm minimum via diameter,
while its default netclass requests 0.6 mm; KRT explicitly requested 0.7 mm.
The CLI's project-rule snapshot is optional and the corpus currently only hashes
the project and supplies it for final external DRC. Therefore the KRT result
proves routability under those recorded project/KRT dimensions, not under every
effective Flux default. Keep it as a topology oracle; do not claim a controlled
speed comparison or silently change benchmark dimensions to match it.

- Board: `tests/fixtures/routing-corpus/bm_svf.kicad_pcb`
- Board SHA-256: `308ce5513cb6b2b33820648a0062a54f548f6b3c13d856cdcd9b7c3ac8a20d2a`
- Flux revision: `25e3b884be9b359d9c5bf0cc4fbb6cc836fd4f82` with a dirty working tree
- KiCadRoutingTools revision: `ec2bf0e1021c0ed95c45b14664d02c56777a0974`
- KiCadRoutingTools version: `0.22.0`
- KiCad Python used by KRT: `9.0.8+dfsg-1`
- External oracle: KiCad `10.0.4` AppImage

The KRT strict run used the original board and project and explicitly requested:

```text
track width       0.2 mm
clearance         0.2 mm
via diameter      0.7 mm
via drill         0.3 mm
escalation        off
fab tier          standard
strict sizes      enabled
DRC-rule rewrite  disabled
```

It also enabled `KICAD_ROUTE_TRACE=1`, verbose diagnostics, A* statistics, memory
diagnostics and raw/simplified debug paths.

## Results

| Metric | Flux | KiCadRoutingTools strict |
| --- | ---: | ---: |
| Outcome | timeout, no committed board | complete routed board |
| Wall time | 7,200 s timeout; last event at 7,169.08 s | 7.30 s |
| Required pad-pair connections | 78 | 78 |
| Connected pad pairs | no accepted exact transaction | 78/78 |
| Path-search expansions | 9,801,533 across 30 PF completions | 699,298 A* expansions |
| Final segments | none committed | 615 |
| Final vias | none committed | 73 through-vias |
| Final route length | unavailable | 925.353 mm |
| Rule escalation | none accepted | none |
| Internal copper DRC | no complete candidate | zero violations; two permitted same-net crossings |
| KiCad 10 opens | no output | zero |
| KiCad 10 routing violations | no output | one 0.0452 mm dangling-track warning; no clearance, size, hole or crossing error |
| Peak/observed RSS | at least 596,264 KiB near timeout | 316,020 KiB maximum |
| Diagnostic trace | 831,964,957 bytes, 8,494 semantic events plus snapshots | 79,915 bytes, 69 copper events |

The source board itself has 74 KiCad unconnected-item reports and 67 unrelated warnings. The KRT
strict output removes all 74 opens. Its 68 remaining reports are the same 67 source warnings plus
one generated dangling-track warning. KRT's exact checker reports all 25 routed nets connected and
zero represented DRC violations.

The ordinary KRT default was also measured. It completed, but used 22 rule/fabrication escalations
on six nets and rewrote the output project floors. It is retained as a diagnostic run, but is not
the fair Flux comparison. The strict run demonstrates that the board is genuinely routable without
those relaxations.

## Trace reconstruction

KRT performed the following high-level sequence:

1. Parsed the two-layer board and created a 0.1 mm grid.
2. Detected 84 endpoint crossing conflicts and produced eight MPS ordering rounds.
3. Built the base obstacle map in 0.02 s.
4. Built all 25 per-net obstacle caches in 0.04 s; the estimated cache was 1.0 MiB.
5. Routed a main connection for every net sequentially. Short/easy routes completed first in tens
   of A* expansions. Hard routes first used 5,000-expansion forward and backward probes, then a
   full weighted A* only when both probes remained viable.
6. Routed 54 remaining multi-point/MST tap connections in boxed-in-risk order.
7. Used localized rip-up only when a particular tap failed. The trace contains eight rip events,
   one restore, 58 route commits, one reconciliation and one final cleanup event.
8. Closed 12 small same-net stub gaps, pruned 70 redundant loop segments, collapsed five redundant
   segments, and simplified 28 staircase spans before writing copper.

The useful successful exchanges were small and causal:

- `Net-(U4A--)` could not complete two taps, ripped `Net-(R4-Pad1)`, and then completed.
- Re-routing `Net-(R4-Pad1)` exposed one trapped pad; it ripped `Net-(U4B--)`, completed, and then
  successfully re-routed `Net-(U4B--)`.
- A previously unsafe restore of `V_SUPPLY` was deferred and reconstructed after the local chain.

KRT recorded only six history updates: two contests, two rip deposits and two failed-frontier
deposits over 2,801 grid cells. It did not need a board-wide negotiated-congestion fixed point.

## What the successful copper proves

The Tier-1 failure is not caused by an unroutable placement, insufficient board area, or too few
copper layers. A strict two-layer route exists. Its topology uses several long perimeter detours
and 73 vias, so route candidates that appear expensive by length or via count are essential.

The KRT solution is not a substitute for Flux's PH output: it is visibly octilinear and contains
615 segments. It is, however, a very strong topology/layer/via oracle. It proves that Flux should
be able to establish a complete conservative quantized-copper solution before asking PH geometry
to improve that solution.

## Earliest divergence in Flux

Flux completed 30 PathFinder passes and 9.80 million reported search expansions. Its retained
all-net checkpoint still carried 19 sampled overflows, three corridor overflows and 324 clearance
shadows. It then constructed a 542-assignment regional portfolio, ran repeated eight-candidate
conflict waves, constructed a second 410-assignment portfolio, and timed out while starting
assignment 56. The trace contains 155 `RouteRippedUp` events, but none produced a complete exact
transaction.

The first decisive divergence is therefore before PH curve quality. Flux accepts globally
connected but physically incompatible allocations into an expensive regional realization space.
KRT never creates that semantic gap: the same clearance-inflated grid cell and via legality used by
search is updated immediately after each committed route.

The second divergence is failure attribution. KRT asks which committed copper blocks one failed
frontier and alters one or a few victims. Flux repeatedly tests complete regional assignments and
adds no-goods at a much coarser state granularity. Many subsequent candidates therefore preserve
the same responsible run/layer/via relation.

The third divergence is portfolio scheduling. KRT's complete route uses unattractive long detours
that remain reachable because full-grid A* can explore them. Flux's corridor and regional beams
spend substantial time realizing many similar candidates while longer same-board/perimeter
homotopies can be absent or ranked too late.

## Highest-value Flux experiment

Add a strict, fast, KRT-like quantized-copper feasibility oracle before regional PH realization:

1. Search the complete two-layer fine occupancy domain, including long perimeter detours.
2. Treat layer transitions and exact-size through-vias as normal search states.
3. Commit only conservative clearance-envelope-legal grid copper into one incrementally updated
   occupancy map.
4. On failure, retain forward/backward frontiers, attribute exact copper blockers, and perform a
   bounded N+1 victim exchange with fresh attribution after each attempt.
5. Preserve the complete legal polyline's homotopy, layer sequence, vias and terminal access as a
   regional route template.
6. Run PH fitting and trajectory optimization only after this oracle has established mutually legal
   quantized copper; PH remains the final routed geometry and exact KiCad checks remain authoritative.

This is not a request to replace Flux with a grid router. It inserts a cheap physical feasibility
master between global allocation and expensive continuous geometry. It should sharply reduce the
number of PH/XPBD candidates while retaining Flux's continuous curved-copper objective.

## Secondary actions

- Keep full-board/perimeter A* fallbacks outside the ordinary corridor beam.
- Learn no-good clauses from the responsible run/template/layer/via pair, not only the complete
  regional assignment.
- Gate XPBD to topology-clean candidates with a small bounded geometric residual.
- Rank candidates globally by conservative quantized-copper legality before spending exact PH work.
- Reuse one incremental obstacle/certificate cache across local victim exchanges.
- Reduce always-on telemetry snapshots. The Flux trace was over 10,000 times larger than KRT's
  copper-event trace and repeatedly serialized complete candidate geometry.

## Artifacts

- Strict KRT run: `build/reference-benchmarks/krt-bm_svf-20260905-strict-traced/`
- Default KRT run: `build/reference-benchmarks/krt-bm_svf-20260905-default-traced/`
- Flux comparison run: `build/tiers-check-20260905-ripup-sequencing-tier1/`
