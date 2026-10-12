# Round31 — terminal-pocket evidence after forward failure

## Isolated feasibility
The saved Round27 Tier2 board can be completed with unchanged forward search: remove owner97, route the final incoming demand, rebuild97, and validate against original incumbent pad groups. This takes19,751 expansions, leaves zero physical opens and adds no internal exact DRC. Four larger groups containing97 also succeed; a group that satisfies only explicit endpoints but loses a pad remains rejected.

`replay_forward_cut.cpp` and `forward-results.json` were built against the locally frozen Round30 implementation with its rejected direction flag explicitly disabled. This reproduces forward search behavior; the diagnostic manually selects victims and does not demonstrate automatic repair.

## Production experiment
Preserve every successful forward search. After an eligible forward failure, probe the opposite endpoint using an independent bounded search. Discard every diagnostic route; only an uncancelled, exhausted sampled frontier contributes priority blocker evidence. Keep original forward blockers, endpoint evidence, termination, repair bounds and exact acceptance. Each scaffold invocation permits at most8 probes of2,000 expansions. Dedicated counters include diagnostic time and expansions.

All source/build inputs are frozen before sequential curved Tier0/1/2 runs. Promotion requires final physical connectivity and external KiCad DRC. Results below.

### Results and decision
All 11 regression targets and independent review pass. Frozen source remained unchanged throughout sequential curved telemetry runs. Tier0 and Tier1 retain byte-identical copper, zero opens and zero internal/external KiCad DRC; detailed runtimes30.641s and105.105s.

Tier2 completes131/131 physical scaffold requests with zero opens in159.621s, versus130/131 with one open in309.399s. Three diagnostic probes cost50.694ms; two exhausted pockets supplied actionable blocker evidence. Keep the general evidence-only repair change.

Final curved materialization remains incomplete: six inherited input pad-clearance reports trigger fallback to straight copper. Independent provisional scaffold export has zero KiCad opens or clearance/hole violations,41 inherited footprint warnings and21 dangling-track warnings. This is not final curved Tier2 promotion. Detailed materialization takes222.119s. Next investigate final validation parity: a representative input pad gap is199872nm against200000nm nominal clearance, within KiCad's500nm electrical DRC epsilon. Routing clearances must remain unchanged.
