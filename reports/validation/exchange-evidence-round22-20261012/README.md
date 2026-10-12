# Round22 saved-copper exchange replay

`replay_joint_repair.cpp` is an offline diagnostic.  It imports the Tier2 fixture, finds the last
`generated-copper:latest` scene in the saved Round21 telemetry, and gets request 11, 19, 43, 44,
45, 91, and 115 from `result.copper_scaffold.request_evidence` in the saved global result.

The request identities are parsed as canonical scaffold paths and checked against their saved net
and connection index.  This avoids treating a coarse route candidate as physical request data.
If any identity or generated-copper snapshot is missing or malformed, the utility exits with a
JSON error instead of choosing substitute endpoints.

For each scenario, it removes only the selected generated-owner primitives, tries the incoming
request first, then rebuilds victims in the specified order.  Victim exchange, joint passage,
displacement, and coupled exchange are all disabled.  Each request has a 50,000 expansion bound
and a 25,000 compact-search expansion bound.

The single JSON document reports the baseline exact Flux DRC categories/object IDs and each
scenario's new exact DRC violations relative to that baseline.  It also reports per-request
source/target attachment counts and, when a failed victim cites a primitive built for the incoming
request, that incoming track/via's ID, net, layers, and geometry.  This distinguishes an incoming
via or track sealing a victim escape from inherited saved-copper violations.  It does not run
KiCad DRC; this is a pre-KiCad causal-repair probe.

Suggested invocation after the coordinating tier run has ended:

```sh
# Compile using the same include/link flags as the existing diagnostic utilities, then run:
./replay_joint_repair \
  tests/fixtures/routing-corpus/ft2232h_jtag.kicad_pcb \
  reports/validation/grid-contact-round21-20261012/tier2/tier-2-ft2232h_jtag/global-result.json \
  reports/validation/grid-contact-round21-20261012/tier2/telemetry/ft2232h_jtag-global.jsonl
```
