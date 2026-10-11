# Straight Tier 1: allocation remains physically conflicting

`bm_svf` has all 78 paths but illegal copper before spline fitting. At snapshot
613, net 1 connection 6 segment 4 and net 16 connection 3 segment 5 both occupy
F.Cu from (50.240, 78.340) to (49.590, 78.990) mm. They require 0.450 mm
centreline separation. The smallest conflict set therefore has two nets.

Native-coordinate replay reports 733 errors: 632 track/track, 71 track/via and
30 hole clearances. These coalesce into 202 diagnoses, with zero opens and no
malformed primitives. The same counts at 10 um and 1 nm rule out final snapping
as the dominant cause. A 100 um replay adds false pad/edge reports.

The selected paths collide before regional routing, continuous geometry or final
quantization. The first straight run spends its 600 s bound in regional portfolio
construction after one coarse epoch. Deferring that work reaches a second global
allocation in 112 s with zero abstract overflow, but replay still finds 561 DRC
reports and 191 diagnoses. Via count rises from 18 to 47. This moves the topology
and reduces some track conflicts while increasing track/via reports; it is not a pass.

The next experiments return exact copper failures upstream as well as sampled
overflow, and bind the existing capacity-one direct-chord model to straight mode.
The original-project external-router reference proves this board is feasible,
not that Flux's current corridor selections have a legal straight realization.

See the sibling JSON and replay JSONL files for exact identities and metrics,
and `docs/router_research_log.md` experiments V37–V39 for research and decisions.

A subsequent **diagnostic** replay of snapshot 1191 through actual straight detailed
materialization emits 527 tracks, zero arcs and 47 vias. Internal checks report 201
copper violations, zero structural errors and zero opens. External KiCad DRC with
the byte-identical source project rejects this exact PCB: 64 clearance reports,
61 crossings, 45 shorting-item reports, 23 solder-mask bridges and nine unconnected
items. Reports overlap physically and are not independent collision counts.
The internal zero-open count must not override those nine external unconnected
reports. This preview is in `build/straight-detail-replay`, never a qualified board.
