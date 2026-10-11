# ADR 0171: Fair G2 PH curves with native-copper realization authority

## Status

Experimental; Tier-0 qualification remains required.

## Context

Flowing curved traces are a product requirement, but Flux does not save PH Bézier curves as
electrical copper. KiCad's routed-copper primitives are straight track segments and circular track
arcs. The previous regional path paid for continuous PH-envelope and rational degree-nine offset
intersection work before discovering whether the much simpler emitted arc/segment scene was legal.

The previous optional join promotion also compared parameter-scaled second-derivative vectors and
called that C2. This was neither invariant under span parameterization nor the geometric fairness
condition wanted at a visible trace join. PH Hermite interpolation can additionally return several
valid algebraic branches whose length and endpoint constraints are similar but whose curvature
profiles look very different.

## Decision

Keep quintic PH chains as Flux's continuous design representation and make them visibly fair before
materialization:

1. Rank valid PH Hermite branches with soft penalties for total curvature variation and for more
   than one interior curvature extremum. Cusp, loop, inflection, corridor, and topology predicates
   remain hard gates.
2. Define join quality geometrically. A multi-span route is G2 when the scalar signed curvature on
   the two sides of every join differs by no more than the configured inverse-length tolerance.
3. After trajectory optimization, make one bounded promotion attempt. A damped finite-difference
   Newton solve changes only internal tangent magnitudes, preserving interpolation points and G1
   tangent directions. Retain the promoted route only if its shape, exact corridor envelope, and
   route class still pass; otherwise keep the already legal G1 PH route.
4. At the final regional-window boundary, immediately adaptively fit every accepted PH route to the
   native track arcs and segments Flux would emit. Add all fitted pieces to one disposable board
   scene and run exact represented-copper DRC jointly.
5. Treat that native scene as the ordinary regional physical-legality authority. Run continuous PH
   envelope and rational-offset intersection only to diagnose/attribute a native rejection, or in
   the explicitly disabled compatibility path.

The final detailed transaction locates each certified regional centerline interval and consumes
its native pieces byte-for-byte on insertion, including reversed path orientation. It reconstructs the rest of
the complete board, protects native arcs from a second spline pass, validates topology
and connectivity, checks represented copper, writes the candidate board atomically, and requires
external KiCad DRC. Regional acceptance is therefore necessary but never sufficient.

Straight native adapters are legal seeds, not always finished curves. Separately certified
fragments can meet at a degree-two bend. Unlocked straight adapters therefore participate in
board-complete PH chain realization; unchanged native arcs remain fixed. Every resulting delta
must pass the same structural, copper and connectivity checks before replacing the inserted seed.

## Consequences

The curve objective and the manufacturing representation now have separate jobs. PH geometry
chooses a smooth, low-variation shape; native KiCad copper decides whether that shape can actually
be published. The common legal case avoids rational degree-nine pair isolation, while rejected
windows retain the stronger continuous diagnostics needed to direct repair upstream.

G2 promotion is deliberately opportunistic. Failure to converge does not destroy a legal,
tangent-continuous flowing route, and no fairness score can authorize copper that fails topology,
corridor, represented-copper, connectivity, or KiCad DRC.

Telemetry reports G2 promotion attempts/promotions/fallbacks and native windows, pieces,
violations, and legal windows. Each regional realization also records whether continuous-envelope
or rational-offset diagnostics were actually evaluated.

## Review corrections

Arc navigation obstacles cover the actual sweep using conservative chord envelopes with a
5 micrometre sagitta bound, not the filled supporting circle. Regional assembly publishes an
ownership-resolved native manifest. Detailed routing verifies width provenance, fit policy,
integer primitive continuity, and aggregate arc limits, and protects those primitives from
subsequent splitting, smoothing and shove-delta replacement.

The assembled-scene assessment includes all provisional routes and physical via spans. Fixed
native/static/via conflicts invalidate the regional selection and identify windows for existing
topology/via reconsideration. Conflicts with still-provisional straight tracks are attributed
upstream without treating those deformable tracks as immutable obstacles. Final detailed DRC
and external KiCad DRC remain mandatory; this assessment does not authorize publication.

Native quantization, rather than diagnostic tessellation chords, supplies the quantized legality
gate in native-authority mode. A shared native fitting policy checks actual quantized join angles
(default 0.035 radians) and allows eight bounded positional-refinement attempts. This is bounded
angular smoothness, not a claim of exact G2 circular-arc copper. If a positional fit fails that
angle bound, a local equal-distance Hermite biarc fit matches the PH endpoint tangents directly.
Integer tangent checks, positional probes (including interval interiors), piece budgets and
represented-copper DRC still gate acceptance. Fit failure remains explicit and is counted
separately from copper violations.

G2 promotion uses local two-span Jacobian probes and a tridiagonal solve, with a 128-join bound,
24-iteration cap and cancellation. A singular pivot falls back instead of amplifying numerical
noise. Promotion timing is reported, and a promoted curve must complete its native transaction
before displacing the optimized G1 fallback. Endpoint extrema are excluded from the fairness
penalty's interior-extremum count.

## References

- R. T. Farouki and C. A. Neff, “Hermite interpolation by Pythagorean-hodograph quintics,”
  *Mathematics of Computation* 64 (1995), 1589–1609.
- R. T. Farouki and T. Sakkalis, “Pythagorean hodographs,” *IBM Journal of Research and
  Development* 34 (1990), 736–752.
- KiCad Developer Documentation, [S-expression PCB format](https://dev-docs.kicad.org/en/file-formats/sexpr-pcb/).
