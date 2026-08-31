# ADR 0144: Coherent regional via access and bounded partial-frontier pruning

## Status

Accepted for the regional multilayer implementation after v0.128.

## Context

The isolated ESC exposed regional assignments that appeared combinatorially valid but could not be
realized as connected physical copper. Three representation gaps were responsible:

- consecutive same-coordinate layer hops were treated as several independent vias even though
  detailed materialization creates one physical through via;
- complete template tuples were enumerated before their terminal/via access CSP was checked; and
- a selected via could move while its adjacent planar connection remained anchored to the old
  coordinate, or could be displayed as an isolated marker with no observable run-up geometry.

Generic history pressure and larger bundle limits cannot repair these inconsistencies. They enlarge
search around a state whose physical variables do not match the eventual copper transaction.

## Decision

A maximal consecutive same-coordinate layer-transition stack is one regional via variable. Its
span includes every copper layer between the first and last hop and its adjacent planar anchors are
the first non-transition points on either side. Via diameter, clearance, contention, assignment,
relocation, and telemetry use that one physical object.

Each selected-topology via choice owns its local planar run-ups. Candidate generation checks the
direct, X-first, and Y-first polylines against represented board geometry and retains the shortest
legal deterministic option for each attached layer. Via position and run-up geometry are selected
atomically. The regional B-spline seed includes those polylines and adds topology links for every
run-up segment. A failed local or whole-board check rejects the complete transaction.

Terminal provenance is defined by the first and last non-zero planar edge of a connection, not by
the raw point index. Leading or trailing stacked layer transitions therefore do not disguise a pad
via as an interior via. A selected-global terminal via cannot relocate unless a route template owns
explicit pad fanout geometry for the new site.

The deterministic template frontier invokes the bounded terminal/via access solver on partial
states. A state is pruned only when its current access variables have no compatible assignment;
adding later variables cannot restore a candidate already removed from every current domain.
Failure evidence distinguishes terminal/terminal, via/via, and terminal/via incompatibility and
retains a bounded stable-ID blocker set for diagnostics.

Alternative endpoint discovery remains on the bounded local stencil. A measured wider stencil and
a selected-blocker-derived just-clear proposal did not solve the complete physical problem at an
acceptable cost and are not retained. Exact static geometry, NavMesh topology, span, access-CSP,
spline, connectivity, and DRC gates remain authoritative.

Selected via run-ups are emitted after analysis as provisional `regional-via-runup:*` route events.
This is observer geometry only; it lets the WebSocket viewer draw the local connection associated
with a selected via but cannot change or authorize routing state.

## Consequences

The bundle state count now measures physically coherent partial states instead of permutations that
are known to have no access assignment. Blocker attribution identifies the exact competing via or
terminal identities without parsing logs. Dogleg run-ups can make a bounded via relocation useful
to the continuous solver while preserving the global corridor as the controlling topology.

The router may report fewer apparently complete windows after terminal provenance is corrected.
That is intentional: losing a pad fanout is not a routing success even when later connectivity
would reject it. Wide, generic via-site portfolios remain inappropriate because they increase
runtime and can select geometrically poor corridors; a remaining dense conflict must advance to
conflict-directed regional assignment rather than an unbounded radius increase.

The retained terminal-safe ESC checkpoint has all 320 global paths, 220 complete regional windows,
21 continuous failures, and one discrete failure at window 109. Its 3,259-state frontier prunes
1,580 access-infeasible prefixes. This is not a routed-board claim; exact candidate construction and
KiCad DRC remain withheld.
