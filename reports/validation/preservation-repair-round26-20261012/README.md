# Round26 — Atomic pad-component preservation repair

Measured failure: rebuilding a coupled route group can satisfy every explicit demand while losing an incidental same-net pad attachment. Round25 early-checkpoint replay showed one direct supplemental demand restores the original ground component. This experiment generates only demands for components newly split by the transaction, routes a bounded suffix within the same shared budget, and atomically accepts the full domain/copper or rolls back.

### Verification and result
All 11 regression targets pass, including retained-checkpoint replay of the appended demand. Independent review found no material issue after cancellation checks were added through supplemental staging and immediately before commit. Frozen source remained unchanged. Sequential curved telemetry runs preserve byte-identical Tier0/1 copper, both with zero opens and internal/external KiCad DRC: detailed runtime29.948s/107.060s versus29.571s/104.561s.

Tier2 improves114/117→117/118 scaffold requests (one preservation demand appended), remaining request115 only; physical opens15→14. Runtime278.111s versus313.552s Round24/316.035s Round25, peakRSS1210528KiB. First pass accepts one supplemental demand, one transaction rebind and one coupled exchange; later passes retain it. No final curved materialization or external KiCad validation is reached, so Tier2 remains incomplete. Keep the general atomic preservation repair. Next diagnose request115 and why outstanding real pad components do not become actionable residual demands.
