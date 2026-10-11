# Regional bundle replay

`flux-replay-regional-bundle BOARD.kicad_pcb CONTEXT.json` compares the original
bounded template DFS and forward checking using the same derived domains,
channels, access candidates and source board. It writes one diagnostic JSON
record per policy: control, access forward checking, then access plus resource
forward checking. None of these runs generates or authorizes copper.

Build with `cmake --build build/release --target flux-replay-regional-bundle`.
The context is the parsed JSON in the `message` field of a diagnostic event whose
`active_group` is `regional-bundle-context:PHASE:WINDOW`. Select the desired
occurrence explicitly when a trace contains repeated routing epochs. Preserve
the context alongside its original trace and executable hash.

To read a context directly from telemetry, pass the exact group and its one-based
occurrence after the trace path:

```sh
build/release/flux-replay-regional-bundle BOARD.kicad_pcb TELEMETRY.jsonl regional-bundle-context:ordinary:0 1
```

Schemas 2 and 3 check the source revision, all eight project minimum constraints and
each net's resolved routing dimensions. External rule overlays must match the
recording; the tool imports sibling project minimum rules and does not accept
custom overlay flags. Schema-1 recordings predate the project-rule audit and
must use their frozen pre-audit executable: silently replaying them with newly
loaded fabrication minima would change the experiment. Schema 2 contains only
channel widths and therefore always replays the legacy whole-channel demand.
Schema 3 also records template centrelines and channel wall endpoints. It runs
all three policies with whole-channel/local longitudinal demand and with/without
exact shared-copper identities, plus V35's local centerline-cut policies (eighteen records),
reporting the number of demand scopes and forced capacity contradictions.
Those contradictions describe the recorded resource model, not a physical
unsatisfiability proof. Full lane packing, spline realization and final DRC are deliberately
outside this diagnostic replay.

The control's choice count includes leaves that downstream access or coupling may
reject. Forward checking screens these leaves, retaining budget-exhausted queries
as unknown. These two counts are therefore not comparable physical success rates;
inspect the rejection counters and the recorded downstream portfolio as well.

The finite-support integration remains experimental and disabled by default:
use `--local-channel-occupancy` with `route-kicad` or `detail-kicad` for a controlled
run. It also regenerates local sections for governed fragments whose old channel
membership had only remote samples. V31's frozen executable used infinite strips;
V32/V33 use finite segment-envelope contact. Preserve the executable hash when
comparing these schema-3 recordings. V34 adds the independent opt-in
`--shared-channel-copper`: exactly identical same-net/layer/dimension polylines
share capacity and lane targets while retaining all branch owners. Adjoining bends,
partial overlaps and independent same-net branches remain separate.

With sharing, the forced-capacity diagnostic uses only domains with one copper
identity, an explicitly incomplete lower bound. Contacts report the first template
of each domain, not proof that every alternative has those contacts. Older schema-3
recordings lack coupled-fragment metadata; `coupling_metadata_recorded` makes that
replay limitation explicit. Current recordings include it.

`--centerline-channel-cuts` implies local occupancy in the routing CLI. It replaces
envelope-touch demand with finite centerline contact, retaining all dimensions and
all downstream physical checks. It is an optimistic allocation estimate, not a
clearance certificate, and remains disabled by default. Replay accepts a trailing
`--max-states N` for bounded comparisons and reports both recorded and override
budgets. A count at an exhausted budget is a lower bound on enumerated choices,
not an exhaustive routability result.

For full routing, `--max-regional-bundle-states N` overrides the existing DFS
state bound (default 100,000; CLI range 1–1,000,000). The benchmark runner accepts
the same flag and preserves it in both planning and detailed command metadata.
The lower bound may reduce portfolio diversity as well as runtime; keep its
results separate from default qualification runs.
