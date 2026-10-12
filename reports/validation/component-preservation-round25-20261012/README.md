# Round25 — Attribute preservation rejections

Diagnostic-only telemetry distinguishes the first disconnected checked request from a real pad-component split when an exchange rolls back. No routing policy or validation gate is changed. The saved-copper replay validates against the full original saved copper, not the post-rip-up intermediate scene, and lists any split pad groups. Both Round21 saved-copper scenarios preserve original groups, so production rejection depends on earlier state or another checked obligation.

## Results

Tier2 reproduces114/117 and15 physical opens in316.035s. Early snapshot1630 reproduces44/11+19 splitting one real ground pad. A single direct residual reconnection restores original components with zero new DRC. Request92 instead closes victim13 source access (0 legal source attachments,128 target attachments). See early-component-results.json, supplement-results.json and branch92-results.json. All are offline diagnostics, not final Tier2 completion.
