#pragma once

#include "flux/core/board.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stop_token>
#include <vector>

namespace flux {

struct SplineVector {
  long double x{0.0L};
  long double y{0.0L};
  bool operator==(const SplineVector&) const = default;
};

// Planar quintic Pythagorean-hodograph segment.  The quadratic complex
// preimage w(t) is represented by three (real,imaginary) Bernstein
// coefficients and r'(t)=w(t)^2.  Speed and arc length are exact polynomials.
struct QuinticPHSegment {
  std::array<SplineVector, 3U> preimage;
  std::array<SplineVector, 6U> bezier;
  std::array<long double, 5U> speed{};
  std::array<long double, 6U> arc_length{};
  // One immutable prepared-geometry cache, rebuilt only when an interpolation node changes:
  // quadratic w -> quartic w^2/hodograph -> quintic position -> quartic speed -> exact quintic
  // arc length -> curvature numerator -> roots/bounds below. Hot DRC/contact evaluation is Horner
  // only. Bernstein state above remains authoritative for convex-hull subdivision, rational
  // products, and KiCad realization.
  std::array<SplineVector, 3U> preimage_power{};
  std::array<SplineVector, 5U> hodograph_power{};
  std::array<SplineVector, 6U> position_power{};
  std::array<long double, 5U> speed_power{};
  std::array<long double, 6U> arc_length_power{};
  std::array<long double, 3U> curvature_numerator_power{};
  SplineVector bounds_minimum{};
  SplineVector bounds_maximum{};
  std::array<long double, 2U> cusp_parameters{};
  std::size_t cusp_count{0U};
  std::array<long double, 2U> inflection_parameters{};
  std::size_t inflection_count{0U};
  std::array<long double, 7U> curvature_extremum_parameters{};
  std::size_t curvature_extremum_count{0U};
  bool regular{false};
  long double route_parameter_start{0.0L};
  long double route_parameter_end{1.0L};
  std::size_t interpolation_branch{0U};
  bool operator==(const QuinticPHSegment&) const = default;
};

enum class PHJoinContinuity { tangent, curvature };

struct PreparedSplineCorridor;

struct QuinticPHRoute {
  // These interpolation nodes, not arbitrary Bézier controls, are the
  // optimizer/XPBD degrees of freedom.  The PH segments are regenerated after
  // every accepted deformation so the hodograph invariant cannot be lost.
  std::vector<SplineVector> control_points;
  std::vector<SplineVector> corridor_anchors;
  std::vector<long double> node_parameters;
  std::vector<SplineVector> node_derivatives;
  std::vector<QuinticPHSegment> segments;
  // Immutable prepared corridor geometry is shared by every control-point candidate. A route
  // optimization wave may clone thousands of candidates; copying the complete NavMesh guard
  // union into each one would dominate both memory traffic and exact-certificate runtime.
  std::shared_ptr<const PreparedSplineCorridor> prepared_corridor;
  long double parameter_exponent{0.5L};
  long double tangent_scale{1.0L};
  PHJoinContinuity requested_continuity{PHJoinContinuity::tangent};
  bool tangent_continuous{false};
  bool curvature_continuous{false};
  // Optional per-node Hermite direction constraints (zero means automatic).
  // Shoulder nodes of a corner blend retain the incident run direction so a
  // rounded corner cannot bow the intervening straight span across a boundary.
  // Magnitudes remain derived from the local parameterization after deformation.
  std::vector<SplineVector> node_tangent_directions;
  bool operator==(const QuinticPHRoute& other) const;
};

struct SplineSample {
  long double parameter{0.0L};
  SplineVector position;
};

struct SplineDifferential {
  SplineVector position;
  SplineVector tangent;
  SplineVector second_derivative;
  long double curvature_per_nm{0.0L};
  long double bend_radius_nm{0.0L};
};

struct PHCurvatureExtremum {
  long double parameter{0.0L};
  long double curvature_per_nm{0.0L};
};

struct PHInflection {
  long double parameter{0.0L};
};

// Exact homogeneous degree-nine Bézier representation of a signed PH offset.
struct PHRationalOffset {
  std::array<long double, 10U> weight{};
  std::array<long double, 10U> weighted_x{};
  std::array<long double, 10U> weighted_y{};
  std::array<long double, 10U> weight_power{};
  std::array<long double, 10U> weighted_x_power{};
  std::array<long double, 10U> weighted_y_power{};
  long double distance_nm{0.0L};
  bool regular{false};
};

struct PHOffsetIntersection {
  long double first_parameter{0.0L};
  long double second_parameter{0.0L};
  SplineVector position;
  long double residual_nm{std::numeric_limits<long double>::infinity()};
};

struct PHOffsetIntersectionStats {
  std::size_t subdivision_nodes{0U};
  std::size_t isolated_leaf_pairs{0U};
  std::size_t maximum_depth_reached{0U};
  bool node_budget_exhausted{false};
};

// Outward-rounded convex-hull bounds for a fixed-depth subdivision of a rational PH offset.
// Positive homogeneous weights make each box conservative: absence of an overlapping box pair
// proves that two offsets cannot intersect. An overlap remains advisory and requires the exact
// long-double intersection routine below.
struct PHOffsetSubdivisionBounds {
  std::int64_t minimum_x_nm{0};
  std::int64_t minimum_y_nm{0};
  std::int64_t maximum_x_nm{0};
  std::int64_t maximum_y_nm{0};
};

// Prepared conservative subdivision of a polynomial quintic PH centreline. Bounds use
// outward-rounded doubles so the common separation proof is inexpensive and vectorizable; the
// long-double Bezier controls remain attached for fail-closed refinement. Rational degree-nine
// offsets are needed only when this hierarchy cannot prove separation or penetration.
struct PHBezierCertificateNode {
  std::array<SplineVector, 6U> bezier;
  std::array<double, 4U> outward_bounds{}; // minimum x/y, maximum x/y
  long double first_parameter{0.0L};
  long double last_parameter{1.0L};
  std::size_t first_child{std::numeric_limits<std::size_t>::max()};
  std::size_t second_child{std::numeric_limits<std::size_t>::max()};
  std::size_t depth{0U};
};

struct PreparedPHSegmentCertificate {
  std::uint64_t geometry_fingerprint{0U};
  std::vector<PHBezierCertificateNode> nodes;
};

enum class PHCenterlineClearanceVerdict { separated, colliding, ambiguous };

struct PHCenterlineClearanceStats {
  std::size_t node_pairs{0U};
  std::size_t simd_batches{0U};
  std::size_t simd_lanes{0U};
  std::size_t double_separation_proofs{0U};
  std::size_t long_double_collision_proofs{0U};
  std::size_t multiprecision_comparisons{0U};
  std::size_t ambiguous_leaf_pairs{0U};
  bool node_budget_exhausted{false};
};

struct PHInterpolationConfig {
  long double parameter_exponent{0.5L}; // centripetal; 1 selects chord length
  long double tangent_scale{1.0L};
  PHJoinContinuity continuity{PHJoinContinuity::tangent};
  long double curvature_tolerance_per_nm{1.0e-10L};
  std::size_t maximum_continuity_iterations{24U};
};

class PreparedQuinticPHEvaluator {
public:
  PreparedQuinticPHEvaluator() = default;
  PreparedQuinticPHEvaluator(const QuinticPHRoute& route, std::size_t sample_count);
  [[nodiscard]] bool valid() const;
  [[nodiscard]] bool update(const QuinticPHRoute& route);
  [[nodiscard]] const std::vector<SplineDifferential>& samples() const { return samples_; }

private:
  std::size_t segment_count_{0U};
  std::vector<long double> parameters_;
  std::vector<SplineDifferential> samples_;
};

struct SplineClearanceProbe {
  long double clearance_nm{0.0L};
  SplineVector obstacle_normal;
  bool represented_legal{false};
};
using SplineClearanceEvaluator = std::function<SplineClearanceProbe(SplineVector)>;
// Optional bulk form used exclusively to populate advisory scoring fields.  It must preserve the
// input order; malformed or incomplete output is ignored in favour of the scalar evaluator.
// Shape and continuous-envelope certificates always retain the scalar long-double predicate.
using SplineClearanceBatchEvaluator =
    std::function<std::vector<SplineClearanceProbe>(const std::vector<SplineVector>&)>;
struct SplineEnvelopeChord {
  SplineVector first;
  SplineVector second;
  Nanometres radius{0};
};
// A clear result must include exact endpoint containment and a conservative capsule-vs-boundary
// proof. It may skip redundant scalar clearance probes; corridor/topology and final certificates
// remain CPU-authoritative, while ambiguous spans keep the ordinary exact path.
struct SplineEnvelopeScreenResult {
  std::vector<std::uint8_t> definitely_clear;
  bool used_gpu{false};
  bool cpu_fallback{false};
  std::size_t dispatches{0U};
  std::uint64_t elapsed_us{0U};
};
using SplineEnvelopeScreenEvaluator =
    std::function<SplineEnvelopeScreenResult(const std::vector<SplineEnvelopeChord>&)>;
using SplineRouteConstraintEvaluator = std::function<bool(const QuinticPHRoute&)>;

struct SplineContinuousEnvelopeConfig {
  Nanometres hull_tolerance{Nanometres::from_mm(0.02)};
  std::size_t maximum_subdivision_depth{14U};
  std::size_t maximum_hulls{4'096U};
  // Set only when every convex corridor cell is itself certified inside free space at the full
  // copper envelope. In that case Bezier-hull containment is the continuous clearance proof and
  // applying point-to-boundary distance minus hull diameter would count clearance twice.
  bool corridor_certifies_clearance{false};
};

struct SplineContinuousEnvelopeCertificate {
  bool legal{false};
  bool budget_exhausted{false};
  std::size_t hulls_checked{0U};
  std::size_t corridor_violations{0U};
  std::size_t clearance_violations{0U};
  long double worst_clearance_nm{std::numeric_limits<long double>::infinity()};
  long double worst_parameter{0.5L};
  SplineVector worst_normal;
  [[nodiscard]] std::size_t violations() const {
    return corridor_violations + clearance_violations + (budget_exhausted ? 1U : 0U);
  }
};

struct SplineTrajectoryWeights {
  long double length{1.0L};
  long double curvature{0.25L};
  long double bend_radius{4.0L};
  long double clearance{8.0L};
  long double tangent{1.0L};
  long double curvature_change{0.5L};
  long double elastic_anchor{0.2L};
  long double control_complexity{0.05L};
};

struct SplineShapeCertificationConfig {
  Nanometres minimum_bend_radius{Nanometres{0}};
  Nanometres tessellation_tolerance{Nanometres::from_mm(0.02)};
  Nanometres station_regression_tolerance{Nanometres{1'000}};
  long double maximum_extra_turn_radians{1.0L};
  // A chain with many intentional corridor corners accumulates small Hermite join rotations.
  // Preserve the strict base bound for a straight/simple route while allowing this much
  // additional absolute rotation for each non-collinear anchor join.
  long double maximum_extra_turn_per_anchor_corner_radians{0.25L};
  std::size_t maximum_tessellation_depth{10U};
  std::size_t maximum_samples{128U};
};

struct SplineShapeCertificate {
  bool legal{false};
  bool sample_budget_exhausted{false};
  std::size_t samples{0U};
  std::size_t self_intersections{0U};
  std::size_t cusps{0U};
  std::size_t control_station_reversals{0U};
  std::size_t station_reversals{0U};
  std::size_t bend_radius_violations{0U};
  std::size_t excess_turn_violations{0U};
  std::size_t unintended_inflections{0U};
  long double minimum_tangent_speed_nm{0.0L};
  long double minimum_bend_radius_nm{std::numeric_limits<long double>::infinity()};
  long double worst_bend_parameter{0.5L};
  long double total_turn_radians{0.0L};
  long double anchor_turn_radians{0.0L};
  [[nodiscard]] std::size_t violations() const {
    return self_intersections + cusps + control_station_reversals + station_reversals +
           bend_radius_violations + excess_turn_violations + unintended_inflections +
           (sample_budget_exhausted ? 1U : 0U);
  }
};

struct SplineTrajectoryOptimizationConfig {
  SplineTrajectoryWeights weights;
  Nanometres minimum_clearance{Nanometres{0}};
  Nanometres minimum_bend_radius{Nanometres{0}};
  Nanometres control_step{Nanometres::from_mm(0.1)};
  Nanometres gradient_probe_step{Nanometres::from_mm(0.02)};
  Nanometres maximum_corridor_displacement{Nanometres::from_mm(2.0)};
  Nanometres clearance_influence_distance{Nanometres::from_mm(0.2)};
  long double second_difference_preconditioner{8.0L};
  long double third_difference_preconditioner{2.0L};
  std::size_t samples{48U};
  std::size_t maximum_iterations{24U};
  std::size_t maximum_candidate_checks{4096U};
  std::size_t maximum_adaptive_control_insertions{2U};
  // When a straightened span cuts an obstacle, replace the broad detour with two
  // interpolation stations on the approach/departure sides of the blocked interval.
  // This is deliberately bounded: the signed-distance field is sampled only while
  // producing a small set of hard-certified candidates.
  bool enable_paired_obstacle_supports{true};
  std::size_t maximum_paired_obstacle_support_pairs{1U};
  std::size_t maximum_paired_obstacle_probes{32U};
  // Optional final gate for the bounded post-polish tautening proposal.  It is deliberately not
  // part of the ordinary candidate constraint: callers can preserve a downstream realization
  // invariant without charging every coordinate-descent proposal for that expensive check.
  SplineRouteConstraintEvaluator postpolish_route_constraint;
  std::size_t maximum_multistart_attempts{4U};
  // Optional advisory whole-chain XPBD preconditioner. The returned interpolation nodes are
  // accepted only through the ordinary CPU shape/corridor/clearance comparator and exact gates.
  bool prefer_gpu_preconditioner{false};
  std::size_t gpu_precondition_iterations{32U};
  bool prefer_gpu_polishing{false};
  std::size_t gpu_polish_shortlist{4U};
  std::size_t exact_candidate_workers{1U};
  std::optional<SplineVector> preferred_start_tangent;
  std::optional<SplineVector> preferred_end_tangent;
  SplineShapeCertificationConfig shape;
  SplineContinuousEnvelopeConfig continuous_envelope;
  // Reuse complete legal span proofs only within this immutable-field optimization call.
  bool reuse_span_certificates{true};
};

struct SplineTrajectoryOptimizationResult {
  std::size_t span_certificate_cache_hits{0U};
  std::size_t span_certificate_cache_misses{0U};
  std::size_t equivalent_candidates_skipped{0U};
  std::size_t non_improving_candidates_skipped{0U};
  bool accepted{false};
  bool legal{false};
  bool budget_exhausted{false};
  std::size_t iterations{0};
  std::size_t candidate_checks{0};
  std::size_t accepted_control_moves{0};
  std::size_t elastic_predeformed_controls{0};
  std::size_t adaptive_controls_inserted{0};
  std::size_t paired_obstacle_support_attempts{0};
  std::size_t paired_obstacle_supports_inserted{0};
  std::size_t paired_obstacle_support_recoveries{0};
  std::size_t multistart_attempts{0};
  std::size_t multistart_recoveries{0};
  std::size_t initial_violations{0};
  std::size_t final_violations{0};
  std::size_t initial_shape_violations{0};
  std::size_t final_shape_violations{0};
  std::size_t final_self_intersections{0};
  std::size_t final_cusps{0};
  std::size_t final_control_station_reversals{0};
  std::size_t final_station_reversals{0};
  std::size_t final_excess_turn_violations{0};
  std::size_t final_unintended_inflections{0};
  std::size_t initial_bend_radius_violations{0};
  std::size_t final_bend_radius_violations{0};
  std::size_t initial_sampled_clearance_violations{0};
  std::size_t final_sampled_clearance_violations{0};
  std::size_t initial_continuous_violations{0};
  std::size_t final_continuous_violations{0};
  std::size_t final_continuous_corridor_violations{0};
  std::size_t final_continuous_clearance_violations{0};
  std::size_t continuous_hulls_checked{0};
  bool continuous_budget_exhausted{false};
  bool gpu_precondition_attempted{false};
  bool gpu_precondition_used_gpu{false};
  bool gpu_precondition_cpu_fallback{false};
  bool gpu_precondition_accepted{false};
  bool gpu_precondition_legal{false};
  std::uint64_t gpu_precondition_solve_us{0U};
  std::size_t gpu_polish_batches{0U};
  std::size_t gpu_polish_dispatches{0U};
  std::size_t gpu_polish_cpu_fallbacks{0U};
  std::uint64_t gpu_polish_solve_us{0U};
  std::size_t gpu_polish_candidates{0U};
  std::size_t gpu_polish_exact_shortlist_checks{0U};
  std::size_t parallel_exact_waves{0U};
  std::size_t parallel_exact_checks{0U};
  long double initial_arc_length_nm{0.0L};
  long double final_arc_length_nm{0.0L};
  long double initial_regularity{0.0L};
  long double final_regularity{0.0L};
  long double initial_objective{0.0L};
  long double final_objective{0.0L};
};

enum class SplineFittedPieceKind { segment, circular_arc, cubic_bezier };
struct SplineFittedPiece {
  SplineFittedPieceKind kind{SplineFittedPieceKind::segment};
  Point start;
  Point middle;
  Point end;
  long double first_parameter{0.0L};
  long double last_parameter{0.0L};
  Point control1{};
  Point control2{};

  bool operator==(const SplineFittedPiece&) const = default;
};

[[nodiscard]] std::vector<long double>
select_quintic_ph_parameters(const std::vector<SplineVector>& points, long double exponent = 0.5L);
[[nodiscard]] std::optional<QuinticPHSegment>
interpolate_quintic_ph_segment(SplineVector start, SplineVector end, SplineVector start_derivative,
                               SplineVector end_derivative);
[[nodiscard]] std::optional<QuinticPHRoute>
make_quintic_ph_route(const std::vector<Point>& polyline, const PHInterpolationConfig& config = {});
[[nodiscard]] std::optional<QuinticPHRoute>
make_topology_preserving_quintic_ph_route(const std::vector<Point>& polyline,
                                          Nanometres corner_guard,
                                          const PHInterpolationConfig& config = {});
[[nodiscard]] std::optional<QuinticPHRoute>
make_corner_rounded_quintic_ph_route(const std::vector<Point>& polyline, Nanometres corner_guard,
                                     const PHInterpolationConfig& config = {});
[[nodiscard]] std::optional<QuinticPHRoute>
make_run_aligned_quintic_ph_route(const std::vector<Point>& polyline, Nanometres corner_guard,
                                 const PHInterpolationConfig& config = {});
[[nodiscard]] bool rebuild_quintic_ph_route(QuinticPHRoute& route);
// Rebuilds the bounded PH support of one changed interpolation node. Falls back to a complete
// rebuild when the existing prepared chain is structurally unavailable.
[[nodiscard]] bool rebuild_quintic_ph_route_around_control(QuinticPHRoute& route,
                                                           std::size_t control_index);
[[nodiscard]] long double quintic_ph_max_curvature_jump(const QuinticPHRoute& route);
[[nodiscard]] bool enforce_quintic_ph_continuity(QuinticPHRoute& route, PHJoinContinuity continuity,
                                                 long double curvature_tolerance_per_nm = 1.0e-10L,
                                                 std::size_t maximum_iterations = 24U,
                                                 std::stop_token stop_token = {});
[[nodiscard]] bool insert_quintic_ph_interpolation_node(QuinticPHRoute& route,
                                                        long double parameter);
// Attaches an independently certified convex corridor to an already prepared PH chain. This does
// not change interpolation geometry and therefore deliberately preserves the segment caches.
[[nodiscard]] bool set_quintic_ph_corridor(QuinticPHRoute& route,
                                           const std::vector<Polygon>& convex_regions);

[[nodiscard]] SplineVector evaluate_quintic_ph(const QuinticPHRoute& route, long double parameter);
[[nodiscard]] SplineDifferential evaluate_quintic_ph_differential(const QuinticPHRoute& route,
                                                                  long double parameter);
[[nodiscard]] std::vector<long double> quintic_ph_bezier_basis(const QuinticPHRoute& route,
                                                               long double parameter);
[[nodiscard]] std::vector<SplineVector> quintic_ph_bezier_controls(const QuinticPHRoute& route);
[[nodiscard]] long double quintic_ph_arc_length(const QuinticPHSegment& segment,
                                                long double parameter = 1.0L);
[[nodiscard]] std::vector<PHCurvatureExtremum>
quintic_ph_curvature_extrema(const QuinticPHSegment& segment);
[[nodiscard]] std::vector<PHInflection> quintic_ph_inflections(const QuinticPHSegment& segment);

[[nodiscard]] PHRationalOffset make_quintic_ph_offset(const QuinticPHSegment& segment,
                                                      long double signed_distance_nm);
[[nodiscard]] SplineVector evaluate_quintic_ph_offset(const PHRationalOffset& offset,
                                                      long double parameter);
[[nodiscard]] std::vector<PHOffsetIntersection> intersect_quintic_ph_offsets(
    const PHRationalOffset& first, const PHRationalOffset& second,
    Nanometres tolerance = Nanometres{10}, std::size_t maximum_subdivision_depth = 18U,
    std::size_t maximum_intersections = 64U, std::size_t maximum_subdivision_nodes = 1'000'000U,
    PHOffsetIntersectionStats* stats = nullptr);
[[nodiscard]] std::vector<PHOffsetSubdivisionBounds>
subdivide_quintic_ph_offset_bounds(const PHRationalOffset& offset,
                                   std::size_t subdivision_depth = 3U,
                                   Nanometres outward_guard = Nanometres{10});
[[nodiscard]] PreparedPHSegmentCertificate
prepare_quintic_ph_segment_certificate(const QuinticPHSegment& segment,
                                       std::size_t subdivision_depth = 6U);
[[nodiscard]] PHCenterlineClearanceVerdict certify_quintic_ph_segment_clearance(
    const PreparedPHSegmentCertificate& first, const PreparedPHSegmentCertificate& second,
    long double required_separation_nm, Nanometres numerical_guard = Nanometres{10},
    std::size_t maximum_node_pairs = 16'384U,
    PHCenterlineClearanceStats* stats = nullptr);

[[nodiscard]] std::vector<SplineSample> adaptive_tessellate_quintic_ph(const QuinticPHRoute& route,
                                                                       Nanometres tolerance,
                                                                       std::size_t maximum_depth,
                                                                       std::size_t maximum_samples);
[[nodiscard]] std::vector<SplineFittedPiece>
fit_spline_samples_to_lines_and_arcs(const std::vector<SplineSample>& samples, Nanometres tolerance,
                                     std::size_t maximum_arc_samples, std::size_t maximum_pieces);
// Checks integer native geometry, not the PH design curve. Invalid/disconnected primitives
// return infinity; a single valid primitive has no internal join and returns zero.
[[nodiscard]] long double native_spline_max_join_angle(const std::vector<SplineFittedPiece>& pieces);
struct NativeSplineFitConfig {
  Nanometres tessellation_tolerance{Nanometres::from_mm(0.02)};
  Nanometres fit_tolerance{Nanometres::from_mm(0.02)};
  std::size_t maximum_depth{10U};
  std::size_t maximum_samples{4'096U};
  std::size_t maximum_arc_samples{24U};
  std::size_t maximum_pieces{4'096U};
  long double maximum_join_angle_radians{0.035L};
  std::size_t maximum_refinement_attempts{8U};
  bool operator==(const NativeSplineFitConfig&) const = default;
};
struct NativeSplineFitStats {
  std::size_t attempts{0U};
  std::size_t peak_samples{0U};
  std::uint64_t tessellation_us{0U};
  std::uint64_t lattice_filter_us{0U};
  std::uint64_t line_arc_fit_us{0U};
  std::uint64_t tangent_biarc_fit_us{0U};
};
// Bounded refinement of the positional fit until the emitted arc/line joins meet the angular
// policy. Failure is explicit: never label a polygonal kink as a smooth native realization.
[[nodiscard]] std::vector<SplineFittedPiece>
fit_quintic_ph_to_native_copper(const QuinticPHRoute& route, const NativeSplineFitConfig& config,
                                std::stop_token stop_token = {},
                                NativeSplineFitStats* stats = nullptr);
[[nodiscard]] std::vector<SplineFittedPiece>
fit_quintic_ph_to_cubic_copper(const QuinticPHRoute& route, const NativeSplineFitConfig& config,
                               std::stop_token stop_token = {});
[[nodiscard]] SplineShapeCertificate
certify_quintic_ph_shape(const QuinticPHRoute& route,
                         const SplineShapeCertificationConfig& config = {});
[[nodiscard]] SplineContinuousEnvelopeCertificate
certify_quintic_ph_continuous_envelope(const QuinticPHRoute& route, Nanometres minimum_clearance,
                                       const SplineClearanceEvaluator& clearance_evaluator,
                                       const SplineContinuousEnvelopeConfig& config = {},
                                       const SplineEnvelopeScreenEvaluator& envelope_screen_evaluator = {});

[[nodiscard]] std::size_t
shove_quintic_ph_interpolation_corridor(QuinticPHRoute& route, long double parameter,
                                        SplineVector normal, Nanometres distance,
                                        Nanometres maximum_corridor_displacement);
[[nodiscard]] long double quintic_ph_curvature_objective(const QuinticPHRoute& route,
                                                         std::size_t samples = 32U);
[[nodiscard]] SplineTrajectoryOptimizationResult
optimize_quintic_ph_trajectory(QuinticPHRoute& route,
                               const SplineTrajectoryOptimizationConfig& config,
                               const SplineClearanceEvaluator& clearance_evaluator,
                               const SplineRouteConstraintEvaluator& route_constraint = {},
                               const SplineClearanceBatchEvaluator& clearance_batch_evaluator = {},
                               const SplineEnvelopeScreenEvaluator& envelope_screen_evaluator = {});
struct PHRouteTighteningStatistics {
  std::size_t candidate_checks{0U};
  std::size_t paired_attempts{0U};
  std::size_t paired_supports_inserted{0U};
};

// Removes redundant interpolation anchors only when rebuilding preserves the complete hard
// route certificate and strictly shortens the exact PH arc length.
[[nodiscard]] bool tauten_quintic_ph_route(
    QuinticPHRoute& route, const SplineTrajectoryOptimizationConfig& config,
    const SplineClearanceEvaluator& clearance_evaluator,
    const SplineRouteConstraintEvaluator& route_constraint = {},
    std::size_t maximum_removals = 16U,
    PHRouteTighteningStatistics* statistics = nullptr);
[[nodiscard]] SplineTrajectoryOptimizationResult
deform_quintic_ph_elastic(QuinticPHRoute& route, SplineVector new_start, SplineVector new_end,
                          const SplineTrajectoryOptimizationConfig& config,
                          const SplineClearanceEvaluator& clearance_evaluator);
[[nodiscard]] SplineTrajectoryOptimizationResult
deform_quintic_ph_elastic_at(QuinticPHRoute& route, long double collision_parameter,
                             SplineVector obstacle_normal, Nanometres displacement,
                             const SplineTrajectoryOptimizationConfig& config,
                             const SplineClearanceEvaluator& clearance_evaluator);

} // namespace flux
