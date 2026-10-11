# ADR 0156: Quintic PH continuous routing

## Status

Accepted.

## Context

The cubic B-spline route foundation allowed local deformation, but it did not provide polynomial
speed, exact arc length, rational offsets, or an algebraic curvature-extrema boundary. Arbitrary
control motion also made loops and small oscillations easy to create, and sampled clearance was a
poor match for continuous copper envelopes.

## Decision

Replace the active B-spline implementation with chains of planar quintic
Pythagorean-hodograph (PH) Bézier segments. For a quadratic complex Bernstein preimage `w(t)`, each
segment satisfies `r'(t) = w(t)^2`. The implementation stores the three preimage coefficients, six
degree-five Bézier controls, five degree-four speed coefficients, and six exact arc-length
coefficients.

Each immutable span also prepares a monomial cache exactly once when its interpolation nodes
change: quadratic `w`, quartic `w^2`, quintic position, quartic speed, quintic arc length, and the
quadratic curvature numerator. Horner evaluation consumes this cache in trajectory and contact
hot paths. Exact positional bounds come from the real unit-interval roots of the two quartic
hodograph components; cusp, inflection, and curvature-extremum parameters are isolated and cached
at the same boundary. Bernstein coefficients remain available for convex-hull subdivision and
rational products, but derivative/regularity queries do not reconstruct the full quintic.

The preparation order is fixed: `w` coefficients, `w^2`/hodograph coefficients, integrated curve
position, speed, integrated arc length, curvature numerator, and finally bounds/root extrema. A
segment revision publishes that complete cache atomically. DRC and contact code may read the cache
but must not lazily reconstruct one of these products per query. Rational signed offsets similarly
prepare their homogeneous numerator/weight power coefficients once per segment revision and offset
distance.

First-order Hermite interpolation enumerates the formal square-root and quadratic branches. Branch
selection is deterministic and penalizes self-intersections, unnecessary inflections, absolute
rotation, excess length, curvature variation, excess curvature extrema, and control-hull reversal.
A looping algebraic solution is never accepted merely because it satisfies the endpoint equations.
Route nodes use bounded centripetal/chord
parameter selection. Adjacent segments use one shared nodal derivative scaled by each non-uniform
parameter interval, so the chain is C1 with respect to the route parameter. Optional geometric-G2
promotion uses a damped bounded Newton solve that relaxes only internal tangent magnitudes while
keeping interpolation points and tangent directions fixed. It fails closed when its explicit
scalar-curvature mismatch tolerance is not reached.

Interpolation nodes are the only continuous deformation variables. XPBD, elastic-band repair, and
PNS feedback may move those nodes inside their selected corridor, after which all affected PH
segments are rebuilt. They may not move PH Bézier controls independently, because that would
destroy the hodograph property.

The sampled global lattice defaults to 0.5 mm so the selected corridor does not inherit avoidable
1 mm staircases. Via-site sampling is independent: an omitted via pitch is rounded upward to the
smallest exact lattice multiple that preserves resolved via spacing. The NavMesh guard cells are
constructed in free space already offset by the complete copper envelope. Adaptive convex-hull
containment in those certified cells proves continuous clearance; point-to-boundary distance is
not reduced by the copper envelope again.

The signed offset is represented exactly as a homogeneous rational degree-nine Bézier curve.
Offset evaluation is rational, and bounded homogeneous subdivision followed by a two-parameter
Newton solve finds transverse rational-offset intersections. Curvature extrema are isolated from
the degree-five numerator of the curvature derivative; inflections are isolated from the quadratic
curvature numerator. Final regional candidates are first fitted to the exact native circular arcs
and straight segments that Flux would emit. DRC over that native copper scene is the ordinary
regional authority; rational offsets are evaluated only as attribution diagnostics after native
rejection, or when the native-materialization authority is explicitly disabled. Final detailed
transaction and external KiCad DRC remain the board-level authorities.

GPU advisory evaluation receives flattened six-control PH Bézier spans and their quintic Bernstein
basis. Telemetry publishes the same representation with geometry kind `quintic_ph`, degree five,
and six controls per segment, allowing the viewer to draw the actual continuous route. Old cubic
B-spline geometry and its source implementation are removed rather than maintained as a routing
fallback. Transactional fallback remains the source-derived polyline, not the obsolete curve type.
Independent local-corner fillets are not a qualifying curved-route fallback: every non-junction
direction change must belong to an accepted whole-chain PH realization or the curved transaction
fails.

Detailed routing also uses the resident Vulkan XPBD solver as a whole-chain elastic preconditioner.
It moves only intermediate interpolation nodes, keeps endpoints fixed, applies stretch/bending and
corridor constraints for the complete chain in one dispatch, and never commits its output directly.
The ordinary long-double CPU assessment must show a feasibility-first improvement before the
preconditioned nodes become the starting point for exact refinement. GPU availability, fallback,
acceptance, and solve timing are observable; backend selection cannot weaken deterministic CPU
acceptance or final KiCad DRC.

CPU polishing does not exact-certify every generated move blindly. Each node's complete bounded
proposal wave is represented as a candidate-major PH Bézier/basis batch, evaluated in one Vulkan
dispatch, and deterministically ranked using sampled represented legality, length, and turning.
The best four proposals retain their stable order identity and pass the unchanged CPU
shape/continuous-envelope comparator. Vulkan failure restores exact evaluation of the complete
wave. Fine-grained exact-candidate threading was measured and rejected as the default: recursive
certificate cost is strongly imbalanced, so synchronized four-candidate waves spent most of their
time waiting for one long-tail task. The exact shortlist therefore remains serial.

Parallel CPU work is scheduled one complete same-layer degree-two route chain per task. Every task
reads the same immutable board-complete snapshot and owns its Board scratch state, prepared
NavMesh, PH route, exact counters, and bounded PNS proposal. It returns only a compact copper delta
in its fixed task-index slot. Stable route-order reduction checks that changed source objects still
match the snapshot, applies the delta to a disposable current-board transaction, and repeats
canonical validation, touched-copper DRC, and connectivity before commit. A conflicting stale
delta is discarded; worker completion order is never observable by the routing decision.

KiCad does not define an electrical Bézier track primitive. Its native routed copper consists of
straight segments, vias, and circular track arcs; graphical Béziers cannot substitute because they
do not carry routed-net connectivity. The quintic PH Bézier is consequently retained as Flux's
continuous trajectory and adaptively fitted into connected native track arcs/segments at the KiCad
materialization boundary. Legal committed curved routes remain visible in incomplete previews even
when another route cannot yet satisfy the whole-board curved-geometry gate.

The continuous NavMesh certificate covers subdivided Bézier hulls over the union of certified guard
cells. A later sampled cell-mask lookup cannot reject a hull already proven inside that union,
although ordered obstacle winding remains mandatory. PH branch ranking rejects cusp singularities
before secondary quality terms. Materialization also prefers one tolerance-bounded line over an
equally accurate near-collinear circle; this avoids remote-centre KiCad arcs while preserving native
arcs for spans with meaningful curvature. Completely straight PH solutions remain successful
whole-chain realizations.

## Consequences

The global PathFinder state remains polyline/resource based. NavMesh topology selects the homotopy
class; PH geometry realizes it continuously; KiCad tracks and arcs remain the quantized physical
form. Exact endpoint, topology, continuous-envelope, quantization, connectivity, and DRC gates are
unchanged in authority.

PH interpolation can have several mathematically valid branches, so branch ranking and shape
certification are hard gates. A bounded enter/leave inflection pair is permitted for a genuine
obstacle-avoidance bulge; extra sign changes are treated as a wiggle. Curvature-variation and
excess-extremum terms rank fair spiral-like branches without overriding corridor or DRC gates.
Rational offset intersection remains available for rejected-candidate diagnosis, while the emitted
native arcs and segments decide ordinary regional copper legality.

## References

- R. T. Farouki and C. A. Neff, “Hermite interpolation by Pythagorean-hodograph quintics,”
  *Mathematics of Computation* 64 (1995), 1589–1609.
- B. Dong and R. T. Farouki, “Algorithm 952: PHquintic,” *ACM Transactions on Mathematical
  Software* 41(4) (2015), article 28, DOI 10.1145/2699467.
- R. T. Farouki et al., “Path planning with Pythagorean-hodograph curves for unmanned or autonomous
  vehicles,” *Proceedings of the IMechE, Part G* (2017).

## KiCad 10 cubic copper backend (2026-10-09)

The active curved pipeline now degree-reduces PH quintics into piecewise cubic Béziers,
represented canonically as `TrackBezier` (schema 22) and exported as net-assigned copper
`gr_curve` objects. PH remains the continuous routing representation. The previous line/arc
fitter remains available for explicit diagnostic uses; it is no longer the active PH output path.

Each candidate cubic matches the quintic's endpoint derivatives before integer quantization.
Degree elevation back to quintic bounds the maximum positional error by the largest control-point
difference. Failed bounds trigger exact subdivision; exhausted work budgets reject the fit.
Accepted spans also require monotone chord projection and bounded join angles.

Planning and DRC use adaptive, certified outer segment envelopes. Connectivity uses contracted
inner envelopes plus exact endpoint caps so an obstacle approximation cannot invent contact.
Export preserves the four integer controls, layer, width, net and lock state. Final KiCad DRC
remains required. KiCad 10 does not enforce track-width rules on graphic copper in the tested
configuration, so Flux separately enforces the canonical minimum width. Marginal contacts may
be conservatively rejected by internal connectivity; a preview is never a completion certificate.
