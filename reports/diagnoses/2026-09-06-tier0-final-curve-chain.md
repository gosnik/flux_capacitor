# Tier 0: one remaining curve chain

V26 has connected, physically legal copper under the original KiCad project, but
is not a completed Flux benchmark route. Five unfinished corners remain in net
17 on F.Cu, within x=15.9–21.3 mm, y=51.5–61.8 mm. The source polyline is legal;
global allocation and diagnostic quantization have zero overflow/errors/opens.

Replaying only this chain against immutable final neighbouring curves gives the
same failure: NavMesh succeeds, but all 24 curve seeds fail exact certification.
Optimization reduces continuous defects from 377 to 140, retaining seven sampled
clearance failures, no shape/radius defects and no certificate budget exhaustion.
The committed-context retry already recovered the other six-corner chain, so
further identical retries do not address this remaining mechanism.

The earliest demonstrated failure is the polyline/corridor-to-curve handoff,
not global connectivity. Check one-sided corridor room and seed geometry before
requiring a new topology. A valid alternative curve at the fixed minimum radius
has not yet been demonstrated. Exact chain and metrics are in the sibling JSON
and `2026-09-06-tier0-net17-chain.json`; none is a benchmark-specific routing rule.
