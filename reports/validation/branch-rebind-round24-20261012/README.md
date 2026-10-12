# Round24 branch-terminal rebind replay

This offline proof experiment replays each incoming request (44 and 91), then victim 11, then
the observed failure of victim 19.  It creates a retained scaffold record over every canonical
request from the saved global result, marks only request 19 missing, and passes the real failed
attachment evidence to `derive_scaffold_terminal_rebinds`.

The utility only retries request 19 if the existing proof routine returns a pad-backed terminal.
It reports that proof, the original and proven endpoints, Flux DRC delta, and the
`CopperTransactionValidator` result for preservation of pre-existing pad components.  It never
chooses a nearest pad or synthesises an endpoint.

## Production validation

11 regression targets pass; independent review found no material correctness defect. Frozen CLI hash and all226 source inputs are in `source-build.json`. Tiers ran sequentially with diagnostic telemetry. Tier0 andTier1 pass final curved copper and external KiCad DRC. Tier2 improves18→15 physical opens at114/117 scaffold requests,313.552s; it remains incomplete and has no final KiCad validation. See `comparison.json`, `summary.json`, `exchange-evidence.json`, and the benchmark history. Raw telemetry, binary, source snapshot and full harness artifacts remain alongside these compact reports.
