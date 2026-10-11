#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace flux {

// Shared advisory compute boundary for the data-parallel parts of routing. GPU results may
// shortlist work and prove conservative broad-phase separation; topology, exact geometry, DRC,
// and transactional commit remain CPU-authoritative.
struct RoutingComputePoint {
  std::int64_t x_nm{0};
  std::int64_t y_nm{0};
  std::int64_t radius_nm{0};
  std::uint32_t layer{0};
  std::uint32_t owner{std::numeric_limits<std::uint32_t>::max()};
};

struct RoutingComputeSegment {
  std::int64_t first_x_nm{0};
  std::int64_t first_y_nm{0};
  std::int64_t second_x_nm{0};
  std::int64_t second_y_nm{0};
  std::int64_t radius_nm{0};
  std::uint32_t layer{0};
  std::uint32_t owner{std::numeric_limits<std::uint32_t>::max()};
};

struct RoutingComputeProximity {
  std::int64_t conservative_margin_nm{std::numeric_limits<std::int64_t>::max()};
  std::size_t closest_segment{std::numeric_limits<std::size_t>::max()};
  std::size_t tested_segments{0};
  bool possible_conflict{false};
};

struct RoutingComputeProximityResult {
  std::vector<RoutingComputeProximity> queries;
  bool used_gpu{false};
  bool cpu_fallback{false};
  std::string diagnostic;

  [[nodiscard]] bool ok() const { return diagnostic.empty(); }
};

// Advisory signed distance to a closed even/odd boundary contour.  A negative value denotes
// outside.  It is never a legality certificate: callers must retain their exact polygon test.
struct RoutingComputeDomainPointResult {
  std::vector<long double> signed_distances_nm;
  bool used_gpu{false};
  bool cpu_fallback{false};
  std::size_t dispatches{0U};
  std::size_t resident_geometry_uploads{0U};
  std::size_t resident_geometry_reuses{0U};
  std::uint64_t elapsed_us{0U};
  std::string diagnostic;
  [[nodiscard]] bool ok() const { return diagnostic.empty(); }
};

[[nodiscard]] RoutingComputeDomainPointResult
probe_routing_domain_points(const std::vector<std::pair<std::int64_t, std::int64_t>>& points,
                            const std::vector<RoutingComputeSegment>& boundary_segments,
                            bool prefer_gpu = true);

[[nodiscard]] RoutingComputeProximityResult
probe_routing_proximity(const std::vector<RoutingComputePoint>& queries,
                        const std::vector<RoutingComputeSegment>& segments,
                        std::int64_t numerical_guard_nm = 1'024, bool prefer_gpu = true);

struct RoutingComputeSplineBatchInput {
  std::vector<std::pair<std::int64_t, std::int64_t>> control_points_nm;
  // Candidate-major controls and candidate/sample-major basis coefficients. Each basis row has
  // one coefficient per control of that candidate. A candidate count of one preserves the
  // original single-curve interface.
  std::vector<long double> basis_coefficients;
  // Samples per candidate; returned positions are candidate-major.
  std::size_t sample_count{0U};
  std::size_t candidate_count{1U};
  // Optional compact candidate-major quintic Bézier representation.  Six controls per span and
  // one [begin,end] parameter interval per span; dense basis_coefficients remains authoritative
  // when this is empty.
  std::vector<std::pair<std::int64_t, std::int64_t>> quintic_bezier_controls_nm;
  std::vector<std::pair<float, float>> quintic_span_intervals;
  std::size_t quintic_spans_per_candidate{0U};
};

struct RoutingComputeSplineBatchResult {
  std::vector<std::pair<long double, long double>> positions_nm;
  bool used_gpu{false};
  bool cpu_fallback{false};
  std::string diagnostic;

  [[nodiscard]] bool ok() const { return diagnostic.empty(); }
};

// Advisory batched spline evaluation for clearance/conflict probes. Authoritative spline shape,
// corridor and DRC certification continues to use the long-double CPU evaluator.
[[nodiscard]] RoutingComputeSplineBatchResult
evaluate_routing_spline_batch(const RoutingComputeSplineBatchInput& input, bool prefer_gpu = true);

struct RoutingComputeSplineScoreInput : RoutingComputeSplineBatchInput {
  std::size_t field_width{0U};
  std::size_t field_height{0U};
  std::int64_t field_origin_x_nm{0};
  std::int64_t field_origin_y_nm{0};
  std::int64_t field_pitch_nm{0};
  // Signed approximate clearance in nm and a conservative selected-corridor mask.
  std::vector<float> signed_clearance_nm;
  std::vector<std::uint8_t> corridor_mask;
  std::array<float, 5U> objective_weights{}; // length, curvature, curvature-change, bend, clearance
  std::int64_t minimum_clearance_nm{0};
  std::int64_t minimum_bend_radius_nm{0};
  std::int64_t clearance_influence_nm{0};
  std::size_t top_k{1U};
};

struct RoutingComputeSplineCandidateScore {
  std::size_t candidate{0U};
  std::uint32_t violations{0U};
  std::uint32_t approximate_self_intersections{0U};
  long double objective{0.0L};
};

struct RoutingComputeSplineScoreResult {
  std::vector<RoutingComputeSplineCandidateScore> shortlist;
  std::size_t assessed_candidates{0U};
  bool used_gpu{false};
  bool cpu_fallback{false};
  std::string diagnostic;
  std::size_t dispatches{0U};
  std::uint64_t elapsed_us{0U};
  [[nodiscard]] bool ok() const { return diagnostic.empty() && !shortlist.empty(); }
};

// Approximate end-to-end PH candidate ranking. The GPU returns only a deterministic top-K; every
// selected candidate remains subject to the exact continuous CPU certificate.
[[nodiscard]] RoutingComputeSplineScoreResult
score_routing_spline_candidates(const RoutingComputeSplineScoreInput& input,
                                bool prefer_gpu = true);

struct RoutingComputeSegmentPairResult {
  std::vector<std::uint8_t> possible_conflicts;
  bool used_gpu{false};
  bool cpu_fallback{false};
  std::string diagnostic;

  [[nodiscard]] bool ok() const { return diagnostic.empty(); }
};

enum class RoutingComputeChordClass : std::uint8_t {
  definitely_clear,
  definitely_blocked,
  ambiguous
};

struct RoutingComputeChordAssessment {
  RoutingComputeChordClass classification{RoutingComputeChordClass::ambiguous};
  std::int64_t conservative_clearance_nm{0};
  std::size_t closest_boundary{std::numeric_limits<std::size_t>::max()};
};

struct RoutingComputeChordBatchResult {
  std::vector<RoutingComputeChordAssessment> chords;
  bool used_gpu{false};
  bool cpu_fallback{false};
  std::string diagnostic;
  // Number of bounded GPU launches and pairwise chord/boundary checks completed by them.
  // These remain zero for CPU-only classification and are advisory telemetry only.
  std::size_t gpu_batch_count{0U};
  std::size_t gpu_work_items{0U};
  std::size_t resident_boundary_uploads{0U};
  std::size_t resident_boundary_reuses{0U};

  [[nodiscard]] bool ok() const { return diagnostic.empty(); }
};

// Conservative resident-boundary broad phase. Only definitely_clear chords may bypass a full
// boundary scan; blocked and ambiguous results still use the caller's exact topology predicate.
[[nodiscard]] RoutingComputeChordBatchResult
classify_routing_chords(const std::vector<RoutingComputeSegment>& chords,
                        const std::vector<RoutingComputeSegment>& boundary_segments,
                        std::int64_t required_clearance_nm, std::int64_t numerical_guard_nm = 2'048,
                        bool prefer_gpu = true);

struct RoutingComputePrimitiveBounds {
  std::int64_t minimum_x_nm{0};
  std::int64_t minimum_y_nm{0};
  std::int64_t maximum_x_nm{0};
  std::int64_t maximum_y_nm{0};
  std::uint32_t layer{0U};
  std::uint32_t owner{std::numeric_limits<std::uint32_t>::max()};
};

struct RoutingComputeBroadphaseResult {
  std::vector<std::pair<std::uint32_t, std::uint32_t>> candidate_pairs;
  std::size_t occupied_bins{0U};
  bool used_gpu{false};
  bool cpu_fallback{false};
  bool budget_exhausted{false};
  std::string diagnostic;
  [[nodiscard]] bool ok() const { return diagnostic.empty(); }
};

// Shared deterministic uniform-grid broad phase. It is deliberately shape-agnostic: consumers
// run exact track/arc/via/pad predicates only for the returned candidates.
[[nodiscard]] RoutingComputeBroadphaseResult
discover_routing_broadphase_pairs(const std::vector<RoutingComputePrimitiveBounds>& primitives,
                                  std::int64_t cell_size_nm, std::size_t maximum_pairs = 1'048'576U,
                                  bool prefer_gpu = true);

enum class RoutingComputeViaRejection : std::uint32_t {
  none = 0U,
  invalid_layer_span = 1U << 0U,
  obstacle_conflict = 1U << 1U,
  runup_conflict = 1U << 2U,
};

struct RoutingComputeViaCandidate {
  std::int64_t x_nm{0};
  std::int64_t y_nm{0};
  std::int64_t radius_nm{0};
  std::uint32_t owner{std::numeric_limits<std::uint32_t>::max()};
  // Copper-layer indices 0..63 traversed by the via barrel.
  std::uint64_t layer_mask{0U};
  std::array<std::pair<std::int64_t, std::int64_t>, 2U> runup_origins_nm{};
  std::array<std::uint32_t, 2U> runup_layers{};
  std::size_t runup_count{0U};
  std::int64_t runup_radius_nm{0};
  // Advisory pressure already known to the regional solver. It never authorizes geometry.
  std::uint32_t corridor_occupancy_pressure{0U};
};

struct RoutingComputeViaCandidateAssessment {
  std::size_t candidate{0U};
  std::uint32_t rejection_bits{0U};
  std::uint32_t approximate_conflicts{0U};
  std::int64_t conservative_margin_nm{std::numeric_limits<std::int64_t>::max()};
  std::uint64_t runup_length_nm{0U};
  std::uint32_t corridor_occupancy_pressure{0U};
  long double score{0.0L};
};

struct RoutingComputeViaCandidateInput {
  std::vector<RoutingComputeViaCandidate> candidates;
  // Includes immutable copper obstacles and already selected vias. Same-owner objects are ignored.
  std::vector<RoutingComputeSegment> obstacles;
  std::int64_t clearance_nm{0};
  std::int64_t numerical_guard_nm{1'024};
  std::size_t top_k{256U};
};

struct RoutingComputeViaCandidateResult {
  std::vector<RoutingComputeViaCandidateAssessment> shortlist;
  // One entry per input candidate; non-zero means the approximate GPU/CPU filter rejected it.
  std::vector<std::uint32_t> rejection_bitmap;
  std::size_t assessed_candidates{0U};
  bool used_gpu{false};
  bool cpu_fallback{false};
  std::string diagnostic;
  [[nodiscard]] bool ok() const { return diagnostic.empty(); }
};

// Bounded advisory regional filtering. This reduces thousands of locally generated sites to a
// stable shortlist. The CPU regional state search and its exact via/run-up predicates remain the
// only authority for selection and commit.
[[nodiscard]] RoutingComputeViaCandidateResult
filter_routing_via_candidates(const RoutingComputeViaCandidateInput& input, bool prefer_gpu = true);

// Candidate-major quantized copper bundle. Every candidate has the same route/point layout;
// coordinates are continuous proposals and are quantized by the compute evaluator before its
// segment envelopes are compared. This is a shortlist only: the regional router repeats its
// exact integer copper, topology and corridor checks for every retained candidate.
struct RoutingComputeQuantizedRoute {
  std::uint32_t point_begin{0U};
  std::uint32_t point_count{0U};
  std::uint32_t layer{0U};
  std::uint32_t owner{std::numeric_limits<std::uint32_t>::max()};
  std::int64_t radius_nm{0};
};

struct RoutingComputeQuantizedCandidateInput {
  std::size_t candidate_count{0U};
  std::size_t points_per_candidate{0U};
  std::vector<std::pair<long double, long double>> candidate_points_nm;
  std::vector<RoutingComputeQuantizedRoute> routes;
  std::int64_t quantization_nm{1};
  std::int64_t clearance_nm{0};
  std::int64_t numerical_guard_nm{1'024};
  std::size_t top_k{8U};
};

struct RoutingComputeQuantizedCandidateScore {
  std::size_t candidate{0U};
  std::uint32_t clearance_violations{0U};
  std::uint32_t crossings{0U};
  std::uint64_t total_deficit_nm{0U};
  std::uint64_t length_nm{0U};
};

struct RoutingComputeQuantizedCandidateResult {
  std::vector<RoutingComputeQuantizedCandidateScore> shortlist;
  std::size_t assessed_candidates{0U};
  bool used_gpu{false};
  bool cpu_fallback{false};
  std::string diagnostic;
  [[nodiscard]] bool ok() const { return diagnostic.empty() && !shortlist.empty(); }
};

[[nodiscard]] RoutingComputeQuantizedCandidateResult score_quantized_routing_candidates(
    const RoutingComputeQuantizedCandidateInput& input, bool prefer_gpu = true);

struct RoutingComputeArcCandidate {
  std::array<std::pair<std::int64_t, std::int64_t>, 5U> points_nm{};
  std::int64_t radius_nm{0};
  std::int64_t copper_radius_nm{0};
  std::uint32_t layer{0U};
  std::uint32_t owner{std::numeric_limits<std::uint32_t>::max()};
};

struct RoutingComputeArcCandidateScore {
  std::size_t candidate{0U};
  std::uint32_t approximate_conflicts{0U};
  long double tangent_error{0.0L};
  std::uint64_t approximate_length_nm{0U};
};

struct RoutingComputeArcCandidateResult {
  std::vector<RoutingComputeArcCandidateScore> shortlist;
  bool used_gpu{false};
  bool cpu_fallback{false};
  std::string diagnostic;
  [[nodiscard]] bool ok() const { return diagnostic.empty() && !shortlist.empty(); }
};

// Approximate batched arc ranking. Points are previous/start/middle/end/next; exact circular
// geometry, quantization and DRC remain the detailed router's acceptance gates.
[[nodiscard]] RoutingComputeArcCandidateResult score_routing_arc_candidates(
    const std::vector<RoutingComputeArcCandidate>& candidates,
    const std::vector<RoutingComputeSegment>& obstacles, std::int64_t clearance_nm,
    std::size_t top_k = 8U, bool prefer_gpu = true);

// Dense compatibility evaluator for bounded regional template/CSP states. Options are global
// indices, variable_domains contains the allowed option indices for each variable, and every
// incompatibility is an unordered pair of option indices. The GPU ranks complete assignments;
// aggregate channel capacity and all exact geometry remain CPU checks on the returned shortlist.
struct RoutingComputeAssignmentInput {
  std::vector<std::vector<std::uint32_t>> variable_domains;
  std::vector<std::pair<std::uint32_t, std::uint32_t>> incompatible_options;
  std::vector<std::uint32_t> option_costs;
  std::size_t maximum_states{100'000U};
  std::size_t top_k{512U};
};

struct RoutingComputeAssignmentCandidate {
  std::vector<std::uint32_t> options;
  std::uint64_t score{0U};
  std::size_t dense_state{0U};
};

struct RoutingComputeAssignmentResult {
  std::vector<RoutingComputeAssignmentCandidate> shortlist;
  std::size_t assessed_states{0U};
  bool state_space_truncated{false};
  bool used_gpu{false};
  bool cpu_fallback{false};
  std::string diagnostic;
  [[nodiscard]] bool ok() const { return diagnostic.empty(); }
};

[[nodiscard]] RoutingComputeAssignmentResult
evaluate_routing_assignments(const RoutingComputeAssignmentInput& input, bool prefer_gpu = true);

struct RoutingComputeLaneOrderInput {
  std::vector<std::uint32_t> widths_nm;
  std::vector<std::uint32_t> clearances_nm;
  // reference_position[i] is the desired geometric order of occupant i.
  std::vector<std::uint32_t> reference_position;
  // UINT32_MAX for an uncoupled member, otherwise its required contiguous coupled rank.
  std::vector<std::uint32_t> coupled_rank;
  std::uint32_t usable_width_nm{0U};
};

struct RoutingComputeLaneOrderResult {
  std::vector<std::uint32_t> order;
  std::uint64_t packed_width_nm{0U};
  std::uint32_t inversions{0U};
  std::size_t assessed_states{0U};
  bool physically_fits{false};
  bool used_gpu{false};
  bool cpu_fallback{false};
  std::string diagnostic;
  [[nodiscard]] bool ok() const { return diagnostic.empty(); }
};

[[nodiscard]] RoutingComputeLaneOrderResult
solve_routing_lane_order(const RoutingComputeLaneOrderInput& input, bool prefer_gpu = true);

// Boolean orientation clauses reject a state when (state & mask) == value. This represents
// unary/pairwise resource and junction compatibility; callers exact-check the deterministic
// shortlist for higher-order geometry.
struct RoutingComputeOrientationInput {
  std::size_t bit_count{0U};
  std::vector<std::pair<std::uint32_t, std::uint32_t>> forbidden_clauses;
  std::vector<std::uint32_t> bit_flip_costs;
  std::size_t top_k{32U};
};

struct RoutingComputeOrientationResult {
  std::vector<std::uint32_t> states;
  std::size_t assessed_states{0U};
  bool used_gpu{false};
  bool cpu_fallback{false};
  std::string diagnostic;
  [[nodiscard]] bool ok() const { return diagnostic.empty(); }
};

[[nodiscard]] RoutingComputeOrientationResult
evaluate_routing_orientations(const RoutingComputeOrientationInput& input, bool prefer_gpu = true);

struct RoutingComputeNegotiatedResource {
  std::uint32_t demand{0U};
  std::uint32_t capacity{0U};
  std::uint32_t history{0U};
  std::uint32_t history_scale{1U};
};

struct RoutingComputeNegotiatedResourceResult {
  std::vector<std::uint32_t> overflow;
  std::vector<std::uint32_t> updated_history;
  std::uint64_t total_overflow{0U};
  std::uint32_t peak_overflow{0U};
  bool used_gpu{false};
  bool cpu_fallback{false};
  std::string diagnostic;
  [[nodiscard]] bool ok() const { return diagnostic.empty(); }
};

// Device reduction shared by sampled edges/nodes, corridors and compact physical resources.
// Stable CPU maps retain identity and commit the returned per-resource increments in order.
[[nodiscard]] RoutingComputeNegotiatedResourceResult reduce_negotiated_routing_resources(
    const std::vector<RoutingComputeNegotiatedResource>& resources, bool prefer_gpu = true);

enum class RoutingComputeCopperKind : std::uint32_t { segment = 0U, circle = 1U };

struct RoutingComputeCopperPrimitive {
  RoutingComputeCopperKind kind{RoutingComputeCopperKind::segment};
  std::int64_t first_x_nm{0};
  std::int64_t first_y_nm{0};
  std::int64_t second_x_nm{0};
  std::int64_t second_y_nm{0};
  std::int64_t radius_nm{0};
  std::uint32_t layer{0U};
  std::uint32_t owner{std::numeric_limits<std::uint32_t>::max()};
};

struct RoutingComputeCopperPairAssessment {
  std::uint8_t possible_conflict{0U};
  std::int64_t approximate_deficit_nm{0};
  long double normal_x{0.0L};
  long double normal_y{0.0L};
};

struct RoutingComputeCopperNarrowphaseResult {
  std::vector<RoutingComputeCopperPairAssessment> pairs;
  bool used_gpu{false};
  bool cpu_fallback{false};
  std::string diagnostic;
  [[nodiscard]] bool ok() const { return diagnostic.empty(); }
};

// Fixed-coordinate, guarded simple-copper classifier. A clear result may prune a pair; every
// reported/ambiguous conflict is still evaluated by the caller's exact analytic predicate.
[[nodiscard]] RoutingComputeCopperNarrowphaseResult assess_simple_copper_pairs(
    const std::vector<RoutingComputeCopperPrimitive>& primitives,
    const std::vector<std::pair<std::uint32_t, std::uint32_t>>& pairs,
    std::int64_t clearance_nm, std::int64_t numerical_guard_nm = 2'048,
    bool prefer_gpu = true);

[[nodiscard]] RoutingComputeSegmentPairResult discover_routing_segment_conflicts(
    const std::vector<RoutingComputeSegment>& segments,
    const std::vector<std::pair<std::uint32_t, std::uint32_t>>& pairs,
    std::int64_t numerical_guard_nm = 1'024, bool prefer_gpu = true);

struct RoutingComputeWavefrontInput {
  std::size_t width{0};
  std::size_t height{0};
  std::vector<std::uint8_t> traversable;
  std::vector<std::size_t> sources;
  std::size_t maximum_passes{0};
};

struct RoutingComputeWavefrontResult {
  std::vector<std::uint32_t> distance;
  std::size_t passes{0};
  bool used_gpu{false};
  bool cpu_fallback{false};
  bool budget_exhausted{false};
  std::string diagnostic;

  [[nodiscard]] bool ok() const { return diagnostic.empty() && !distance.empty(); }
};

// Multi-source micromouse-style flood fill. The same kernel also constructs per-layer clearance
// fields by making obstacle/boundary cells the sources and the complete raster traversable.
[[nodiscard]] RoutingComputeWavefrontResult
compute_routing_wavefront(const RoutingComputeWavefrontInput& input, bool prefer_gpu = true);

struct RoutingComputeGraphEdge {
  std::uint32_t node{0U};
  std::uint32_t cost{1U};
};

struct RoutingComputeGraphWavefrontInput {
  // CSR adjacency. offsets.size() == node_count + 1 and its final entry equals edges.size().
  std::size_t node_count{0U};
  std::vector<std::uint32_t> offsets;
  std::vector<RoutingComputeGraphEdge> edges;
  std::vector<std::size_t> sources;
  std::size_t maximum_passes{0U};
};

struct RoutingComputeBatchedGraphWavefrontInput {
  std::size_t node_count{0U};
  std::vector<std::uint32_t> offsets;
  std::vector<RoutingComputeGraphEdge> edges;
  // One independently relaxed reverse field per source group.
  std::vector<std::vector<std::size_t>> source_groups;
  std::size_t maximum_passes{0U};
};

struct RoutingComputeBatchedGraphWavefrontResult {
  std::vector<std::vector<std::uint32_t>> distance;
  std::size_t passes{0U};
  bool used_gpu{false};
  bool cpu_fallback{false};
  bool budget_exhausted{false};
  std::string diagnostic;
  [[nodiscard]] bool ok() const { return diagnostic.empty() && !distance.empty(); }
};

// Multi-source weighted relaxation over an arbitrary sparse routing graph. This is the GPU path
// used by adaptive NavMesh leaves; exact portal construction and route validation stay on CPU.
[[nodiscard]] RoutingComputeWavefrontResult
compute_routing_graph_wavefront(const RoutingComputeGraphWavefrontInput& input,
                                bool prefer_gpu = true);

// Shared job-major weighted frontier used for global reverse cost-to-go fields and explicitly
// expanded (node,width-mode,narrow-distance) corridor product graphs.
[[nodiscard]] RoutingComputeBatchedGraphWavefrontResult compute_batched_routing_graph_wavefront(
    const RoutingComputeBatchedGraphWavefrontInput& input, bool prefer_gpu = true);

struct RoutingComputeClearanceFieldResult {
  // Positive values are legal-cell distance to the nearest obstacle and negative values are
  // obstacle depth to the nearest legal cell; the interface lies between +1 and -1. Distances are
  // in raster cells.
  std::vector<std::int32_t> signed_distance;
  bool used_gpu{false};
  bool cpu_fallback{false};
  std::string diagnostic;

  [[nodiscard]] bool ok() const { return diagnostic.empty() && !signed_distance.empty(); }
};

[[nodiscard]] RoutingComputeClearanceFieldResult
compute_routing_clearance_field(std::size_t width, std::size_t height,
                                const std::vector<std::uint8_t>& legal_cells,
                                std::size_t maximum_passes, bool prefer_gpu = true);

struct RoutingComputeLinearCandidate {
  std::array<std::uint32_t, 8U> terms{};
};

struct RoutingComputeLinearScores {
  std::vector<std::uint64_t> scores;
  bool used_gpu{false};
  bool cpu_fallback{false};
  std::string diagnostic;

  [[nodiscard]] bool ok() const { return diagnostic.empty(); }
};

// Shared fixed-order dot product for coarse portfolio and placement feasibility probes. It does
// not replace the feasibility-first comparators; it supplies a cheap stable pre-score.
[[nodiscard]] RoutingComputeLinearScores
score_routing_compute_candidates(const std::vector<RoutingComputeLinearCandidate>& candidates,
                                 const std::array<std::uint32_t, 8U>& weights,
                                 bool prefer_gpu = true);

// Bounded PCB-specific XPBD relaxation input. Coordinates remain in nanometres at the public
// boundary, but backends translate them to a local origin before using floating point. A particle
// with inverse_mass == 0 is fixed (pad endpoint or fixed via). Corridor anchors preserve the
// selected regional homotopy; they are a deformation envelope, not a new topology search.
enum RoutingXpbdParticleFlag : std::uint32_t {
  routing_xpbd_endpoint = 1U << 0U,
  routing_xpbd_via = 1U << 1U,
};

struct RoutingXpbdParticle {
  std::int64_t x_nm{0};
  std::int64_t y_nm{0};
  std::int64_t radius_nm{0};
  long double inverse_mass{1.0L};
  std::uint32_t route{0U};
  std::uint32_t layer{0U};
  std::uint32_t flags{0U};
  std::int64_t corridor_anchor_x_nm{0};
  std::int64_t corridor_anchor_y_nm{0};
  std::int64_t corridor_radius_nm{0};
  // Point-owner clearance (not an expanded radius). AVBD uses this for via disks;
  // ordinary XPBD segment contacts continue to use RoutingXpbdSegment::clearance_nm.
  std::int64_t clearance_nm{0};
};

struct RoutingXpbdSegment {
  std::uint32_t first_particle{0U};
  std::uint32_t second_particle{0U};
  std::uint32_t route{0U};
  std::uint32_t layer{0U};
  std::int64_t radius_nm{0};
  std::int64_t rest_length_nm{0};
  std::int64_t clearance_nm{0};
};

struct RoutingXpbdObstacleSegment {
  std::int64_t first_x_nm{0};
  std::int64_t first_y_nm{0};
  std::int64_t second_x_nm{0};
  std::int64_t second_y_nm{0};
  std::int64_t radius_nm{0};
  std::uint32_t layer{0U};
  // Apply to every route when unset, or to one route-specific free-space extraction.
  std::uint32_t route{std::numeric_limits<std::uint32_t>::max()};
  // Optional unit-independent vector pointing away from forbidden polygon material. A zero vector
  // preserves the symmetric segment-clearance behavior used for tracks and open line barriers.
  // Polygonal board obstacles populate this so a trace inside or crossing a filled obstacle is
  // projected toward free space instead of becoming trapped on the wrong side of its boundary.
  std::int64_t outward_normal_x{0};
  std::int64_t outward_normal_y{0};
};

struct RoutingXpbdTerrainField {
  std::size_t width{0U};
  std::size_t height{0U};
  // Consecutive layer-major slices. Particle layer codes index this dimension directly.
  // A value of one preserves the original single-layer API.
  std::size_t layer_count{1U};
  std::int64_t origin_x_nm{0};
  std::int64_t origin_y_nm{0};
  std::int64_t pitch_nm{0};
  // Layer-major, then row-major non-negative routing pressure. The solver follows the negative
  // local gradient only in the particle's own copper-layer slice.
  std::vector<float> pressure;
};

struct RoutingXpbdConfig {
  std::size_t iterations{32U};
  long double stretch_compliance{1.0e-5L};
  long double bend_compliance{5.0e-4L};
  long double clearance_compliance{1.0e-7L};
  long double corridor_compliance{1.0e-5L};
  long double terrain_gain{0.02L};
  // Maximum one-iteration particle displacement protects topology and numerical stability.
  std::int64_t maximum_step_nm{100'000};
  std::int64_t numerical_guard_nm{1'024};
  std::size_t maximum_candidate_pairs{262'144U};
  std::size_t pressure_width{0U};
  std::size_t pressure_height{0U};
  std::int64_t pressure_origin_x_nm{0};
  std::int64_t pressure_origin_y_nm{0};
  std::int64_t pressure_pitch_nm{0};
  std::size_t pressure_diffusion_passes{2U};
  long double pressure_diffusion{0.2L};
  // Diagnostic-only GPU residual reduction is recorded into a device buffer and read once after
  // the complete iteration sequence. Normal routing leaves this off to avoid doubling constraint
  // evaluation merely for telemetry.
  bool record_iteration_metrics{false};
  bool prefer_gpu{true};

  bool operator==(const RoutingXpbdConfig&) const = default;
};

struct RoutingXpbdIterationMetrics {
  std::size_t violations{0U};
  long double maximum_residual_nm{0.0L};
  long double total_residual_nm{0.0L};
};

struct RoutingXpbdConstraintCounts {
  std::size_t stretch{0U};
  std::size_t bend{0U};
  std::size_t obstacle{0U};
  std::size_t trace_clearance{0U};
  std::size_t corridor{0U};

  [[nodiscard]] std::size_t total() const {
    return stretch + bend + obstacle + trace_clearance + corridor;
  }

  bool operator==(const RoutingXpbdConstraintCounts&) const = default;
};

// Host-observed phase timings. `solve_us` includes the submitted Vulkan iteration sequence when
// the GPU backend is selected; no timing query or readback is inserted between solver iterations.
struct RoutingXpbdTimings {
  std::uint64_t broadphase_us{0U};
  std::uint64_t constraint_generation_us{0U};
  std::uint64_t solve_us{0U};
  std::uint64_t stress_reduction_us{0U};
  std::uint64_t readback_us{0U};

  bool operator==(const RoutingXpbdTimings&) const = default;
};

// Attributed solver activity. `via_nm` is an overlapping attribution for stress observed on a
// via particle and is therefore not added again by total_nm(). The other terms partition the
// PCB-specific constraint families used by the regional solver.
struct RoutingXpbdStressBreakdown {
  long double obstacle_nm{0.0L};
  long double trace_clearance_nm{0.0L};
  long double stretch_bend_nm{0.0L};
  long double corridor_nm{0.0L};
  long double via_nm{0.0L};

  [[nodiscard]] long double total_nm() const {
    return obstacle_nm + trace_clearance_nm + stretch_bend_nm + corridor_nm;
  }

  RoutingXpbdStressBreakdown& operator+=(const RoutingXpbdStressBreakdown& other) {
    obstacle_nm += other.obstacle_nm;
    trace_clearance_nm += other.trace_clearance_nm;
    stretch_bend_nm += other.stretch_bend_nm;
    corridor_nm += other.corridor_nm;
    via_nm += other.via_nm;
    return *this;
  }

  bool operator==(const RoutingXpbdStressBreakdown&) const = default;
};

struct RoutingXpbdResult {
  std::vector<std::pair<std::int64_t, std::int64_t>> positions_nm;
  std::vector<long double> particle_stress_nm;
  std::vector<RoutingXpbdStressBreakdown> particle_stress_by_type;
  RoutingXpbdStressBreakdown stress;
  std::vector<float> pressure_field;
  std::size_t pressure_width{0U};
  std::size_t pressure_height{0U};
  std::size_t pressure_layer_count{0U};
  std::vector<RoutingXpbdIterationMetrics> iteration_metrics;
  RoutingXpbdIterationMetrics initial;
  RoutingXpbdIterationMetrics final;
  RoutingXpbdConstraintCounts constraints;
  RoutingXpbdTimings timings;
  std::size_t candidate_pairs{0U};
  std::size_t unresolved_trace_pairs{0U};
  std::size_t broadphase_cells{0U};
  std::size_t broadphase_pair_emissions{0U};
  std::size_t resident_input_buffer_reuses{0U};
  std::size_t iterations_executed{0U};
  bool converged{false};
  bool stalled{false};
  bool topology_escape_required{false};
  bool pair_budget_exhausted{false};
  bool used_gpu_broadphase{false};
  bool used_gpu_pressure_field{false};
  bool broadphase_fallback{false};
  bool used_gpu{false};
  bool cpu_fallback{false};
  std::string diagnostic;

  [[nodiscard]] bool ok() const { return diagnostic.empty() && !positions_nm.empty(); }
};

// Jointly relax a complete regional route bundle. The returned positions are diagnostic geometry
// until the caller re-fits its quintic PH curves and passes the existing topology, exact copper and
// DRC transaction. GPU and CPU implementations intentionally share this compact public model.
[[nodiscard]] RoutingXpbdResult
relax_routing_xpbd(const std::vector<RoutingXpbdParticle>& particles,
                   const std::vector<RoutingXpbdSegment>& segments,
                   const std::vector<RoutingXpbdObstacleSegment>& obstacles = {},
                   const RoutingXpbdTerrainField& terrain = {},
                   const RoutingXpbdConfig& config = {});

[[nodiscard]] bool routing_compute_gpu_available();

struct RoutingComputeDeviceInfo {
  bool available{false};
  std::string device_name;
  std::string device_type;
  std::uint32_t vendor_id{0U};
  std::uint32_t device_id{0U};
  std::uint32_t api_version{0U};
  std::uint32_t driver_version{0U};
  std::string diagnostic;
};

// Reports the exact process-wide Vulkan device selected by the same runtime used for production
// routing. This is observational metadata for run manifests; it does not participate in device
// selection or routing decisions.
[[nodiscard]] RoutingComputeDeviceInfo routing_compute_device_info();

} // namespace flux
