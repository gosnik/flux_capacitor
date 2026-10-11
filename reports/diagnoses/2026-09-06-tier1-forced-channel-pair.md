# Tier-1 forced channel pair

Ordinary window 0 has no discrete solution in either replay policy: both stop
after 20 states without exhausting the search budget. Channel 61 has a maximum
recorded width of 1.092 mm. Its two forced occupants require 1.100 mm under the
current aggregate width/clearance model. Neither fragment has a second template.

The implicated fragments are `kicad-net-25/0/2/0` and `kicad-net-26/0/0/17` on
F.Cu, within x=65.84–66.49 mm, y=54.29–56.24 mm. The replay context is occurrence
1 of `regional-bundle-context:ordinary:0` in the V16b Tier-1 trace.

This identifies the smallest pair responsible for this aggregate inequality.
It does not prove the physical region or the entire board unsatisfiable. The
fragments are not parallel strips over their full lengths, so a cross-section
occupancy experiment may reveal an overly conservative model. The independent
physical copper checks remain mandatory; an 8 µm aggregate deficit is no reason
to reduce clearance.

If this remains relevant after residual repair, test capacity-conflict-directed
column generation first, then validate longitudinal occupancy on a synthetic
fixture. Extra DFS states cannot change this forced-domain contradiction.
