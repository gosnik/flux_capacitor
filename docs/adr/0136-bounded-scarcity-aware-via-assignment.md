# ADR 0136: Bounded scarcity-aware via assignment

## Status

Accepted as an incomplete regional-routing milestone for v0.122.

## Context

Flux already represented a global layer transition as coincident points on two copper layers and
could generate several regional relocation sites when selected vias collided. The bounded access
solver checked via/via and terminal/via geometry jointly, but its variables were ordered by net and
region ID. Via-site scarcity was not first-class evidence, static seed priority could classify a
short via-dependent route as flexible, and callers could not distinguish one legal transition from
a region with several alternatives.

Independent locally convenient sites are particularly harmful near terminal fanout and narrow
multilayer channels: several valid paths can consume the same small transition region and become
collectively unrealizable only after global routing.

## Decision

- Keep every via candidate as derived regional state with stable ID, net, region, physical site,
  legal layer span, diameter, drill, and selected-global provenance.
- For a selected global transition whose planned copper envelope conflicts with another planned
  transition, retain a bounded portfolio containing the selected site and exact-static-legal
  relocation sites. For an uncontested transition, retain the selected site and explicitly mark
  alternative expansion deferred. This bounds the initial milestone while preserving a distinct
  state for later lazy expansion.
- Alternative-layer templates retain their exact-checked entry and exit via sites together with
  the run-up and NavMesh topology that made those sites legal. A via site is never detached from
  its route template.
- Build immutable `RegionalViaRegionEvidence` after template generation. Publish candidate IDs,
  selected-global count, span, overlapping-span regions, physically conflicting candidate pairs,
  terminal-escape conflicts, competing regions, contention status, and deferred-expansion status.
- Resolve pairwise routing clearance once per net pair while constructing evidence and while
  searching one access assignment.
- Order joint access variables by fewest retained candidates, greatest competing-region count,
  greatest candidate and terminal conflict pressure, longest layer span, then stable net/region
  identity. This reserves scarce choices for constrained transitions before flexible variables.
- Search the complete bounded assignment with the existing best-first objective: physical conflict
  count, relocated-via count, displacement, and stable choice IDs. Expand only variables involved
  in the current conflict. Retain selected choice IDs, region order, alternative switches, search
  states, and exhaustion in the bundle result.
- Continue treating differential-pair and routing-bus layer sequences atomically. This milestone
  does not authorize a partial via array.
- Extend iteration-zero evidence with primary via-transition count, distinct sampled via sites,
  distinct complete assignment signatures, and contention at primary sites. A via-bearing route
  with no meaningful alternative becomes constrained even if its planar extent is short.
- Keep every selected site provisional. Existing structural validation, layer-span policy,
  represented clearance, connectivity, coupled constraints, B-spline realization, copper DRC, and
  source-preserving export remain authoritative.

This milestone does not yet create a PathFinder capacity resource for a via region, jointly reserve
sites across separate regional windows, relocate arbitrary interior vias through route-run PNS, or
prove an uncontested site's deferred alternatives. Those remain required before deleting the
existing selected-global fallback.

## Consequences

The router can now attribute a via failure to a bounded physical region and can switch among
already validated sites without rediscovering global topology. Search ordering uses the same
scarcity evidence that callers can inspect, and the risk fixture proves that a route forced through
a sparse via sublattice enters the constrained seed wave. Regional risk coverage also verifies
bounded candidate portfolios and unique stable assignment order.

The current isolated zero-track/zero-via ESC command remains incomplete. In the v0.122 Release
measurement it took 225.86 seconds at 291,672 KiB peak RSS, published 228 via regions and 397 sites,
classified 57 regions as contested, selected nine alternative sites, rejected two via-pair states,
and exhausted two bounded access searches. It produced 182 complete assignments among 242 windows,
with five no-fitting bundles and 55 ineligible windows; no alternate-layer template or complete
spline/copper transaction was accepted. These results fail the runtime and route-completion gates.
They justify keeping the fallback and prioritizing window eligibility, lazy candidate expansion,
and spatially indexed contention rather than increasing the assignment bound.
