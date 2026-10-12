# Round23 terminal-escape reservation replay

This offline utility reuses the Round22 saved-copper parser.  For each existing exchange scenario,
it removes the victim owners, selects at most two deterministic pad-touching tracks per victim
terminal (eight tracks total), and adds copies with private reservation IDs before routing the
incoming request.  It then removes every reservation before rebuilding victims in their original
order.

Each reservation outcome includes a same-run no-reservation control using the Round22 ordering
and bounds.  Candidate pads with unsupported geometry fail the utility instead of falling back to
an inferred terminal contact.

Only track endpoints physically touching matching same-net pad copper are eligible.  Vias are
never reserved.  The output records selected original IDs and geometry, incoming and victim
results, reservation removal/leakage, Flux DRC changes relative to saved copper, and connectivity.
