# ADR 0139: Packing-first owner-scoped topology alternatives

## Status

Accepted as an incomplete regional-routing milestone for v0.124.

## Context

Four isolated ESC windows had no fitting lane state. Alternative generation spent bounded pair
checks before proving endpoint run-ups and via spans, then allowed articulation alternatives to
spread into unrelated owner windows. Inclusive intersection priority also admitted endpoint and
collinear contact.

## Decision

- Add two inward collinear sites to the bounded endpoint stencil.
- Prove each endpoint's source-layer run-up and complete via span before pair search.
- Attribute domain, NavMesh, run-up, via-span, and topology-cell rejection separately.
- Offer alternatives only in a fragment's owner window.
- Give physical packing failures the first bounded wave. Use proper interior crossings only when
  there is no packing failure; endpoint and collinear contact do not qualify.
- Preserve all existing template, site, bundle, lane, and exact transaction bounds.

## Consequences

The one-pass isolated ESC checkpoint accepts 81 alternate-layer templates from 82 complete pair
checks and gives all 242 windows fitting discrete topology/lane/via bundles. Runtime is 345.42
seconds with 291,432 KiB peak RSS. Continuous B-spline realization rejects 100 windows, so no
candidate is materialized. The next stage must separately attribute topology containment,
route-to-route clearance, short tessellation, and control-optimization failure.
