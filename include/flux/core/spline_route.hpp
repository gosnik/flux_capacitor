#pragma once

#include "flux/core/board.hpp"

#include <cstddef>
#include <functional>
#include <optional>
#include <vector>

namespace flux {

struct SplineVector {
  long double x{0.0L};
  long double y{0.0L};

  bool operator==(const SplineVector&) const = default;
};

struct CubicBSplineRoute {
  std::vector<SplineVector> control_points;
  std::vector<SplineVector> corridor_anchors;
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

// Fixed-parameter evaluator for an immutable spline topology. Knot and basis/derivative
// coefficients are prepared once; update recomputes only control-dependent sample values and
// reuses its result storage after each accepted or probed control move.
class PreparedCubicBSplineEvaluator {
public:
  PreparedCubicBSplineEvaluator() = default;
  PreparedCubicBSplineEvaluator(std::size_t control_count, std::size_t sample_count);

  [[nodiscard]] bool valid() const;
  [[nodiscard]] bool update(const CubicBSplineRoute& route);
  [[nodiscard]] const std::vector<SplineDifferential>& samples() const { return samples_; }

private:
  std::size_t control_count_{0};
  std::vector<std::vector<long double>> position_coefficients_;
  std::vector<std::vector<long double>> tangent_coefficients_;
  std::vector<std::vector<long double>> second_coefficients_;
  std::vector<SplineDifferential> samples_;
};

struct SplineClearanceProbe {
  long double clearance_nm{0.0L};
  SplineVector obstacle_normal;
  bool represented_legal{false};
};

using SplineClearanceEvaluator = std::function<SplineClearanceProbe(SplineVector)>;

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

struct SplineTrajectoryOptimizationConfig {
  SplineTrajectoryWeights weights;
  Nanometres minimum_clearance{Nanometres{0}};
  Nanometres minimum_bend_radius{Nanometres{0}};
  Nanometres control_step{Nanometres::from_mm(0.1)};
  Nanometres gradient_probe_step{Nanometres::from_mm(0.02)};
  Nanometres maximum_corridor_displacement{Nanometres::from_mm(2.0)};
  std::size_t samples{48U};
  std::size_t maximum_iterations{24U};
  std::size_t maximum_candidate_checks{4096U};
  std::optional<SplineVector> preferred_start_tangent;
  std::optional<SplineVector> preferred_end_tangent;
};

struct SplineTrajectoryOptimizationResult {
  bool accepted{false};
  bool legal{false};
  bool budget_exhausted{false};
  std::size_t iterations{0};
  std::size_t candidate_checks{0};
  std::size_t accepted_control_moves{0};
  std::size_t elastic_predeformed_controls{0};
  std::size_t initial_violations{0};
  std::size_t final_violations{0};
  long double initial_objective{0.0L};
  long double final_objective{0.0L};
};

enum class SplineFittedPieceKind { segment, circular_arc };

struct SplineFittedPiece {
  SplineFittedPieceKind kind{SplineFittedPieceKind::segment};
  Point start;
  Point middle;
  Point end;
  long double first_parameter{0.0L};
  long double last_parameter{0.0L};
};

[[nodiscard]] std::optional<CubicBSplineRoute>
make_cubic_bspline_route(const std::vector<Point>& polyline);

[[nodiscard]] SplineVector evaluate_cubic_bspline(const CubicBSplineRoute& route,
                                                  long double parameter);

[[nodiscard]] SplineDifferential evaluate_cubic_bspline_differential(const CubicBSplineRoute& route,
                                                                     long double parameter);

[[nodiscard]] std::vector<long double> cubic_bspline_basis(const CubicBSplineRoute& route,
                                                           long double parameter);

[[nodiscard]] std::vector<SplineSample>
adaptive_tessellate_cubic_bspline(const CubicBSplineRoute& route, Nanometres tolerance,
                                  std::size_t maximum_depth, std::size_t maximum_samples);

[[nodiscard]] std::vector<SplineFittedPiece>
fit_spline_samples_to_lines_and_arcs(const std::vector<SplineSample>& samples, Nanometres tolerance,
                                     std::size_t maximum_arc_samples, std::size_t maximum_pieces);

// Applies C_i' = C_i + alpha_i * distance * normal, where alpha_i is the normalized cubic basis
// influence at the collision parameter. The first two and last two controls remain fixed so pad
// and via endpoint tangents are preserved. Every movable control is clamped to its source corridor.
[[nodiscard]] std::size_t
shove_cubic_bspline_control_corridor(CubicBSplineRoute& route, long double parameter,
                                     SplineVector normal, Nanometres distance,
                                     Nanometres maximum_corridor_displacement);

[[nodiscard]] long double cubic_bspline_curvature_objective(const CubicBSplineRoute& route,
                                                            std::size_t samples = 32U);

// Continuously optimizes movable controls while keeping endpoint/tangent controls fixed. Candidate
// moves are corridor-clamped and accepted lexicographically by represented violations then the
// weighted trajectory objective. Analytical spline derivatives provide tangent and curvature.
[[nodiscard]] SplineTrajectoryOptimizationResult
optimize_cubic_bspline_trajectory(CubicBSplineRoute& route,
                                  const SplineTrajectoryOptimizationConfig& config,
                                  const SplineClearanceEvaluator& clearance_evaluator);

// Applies endpoint geometry changes, propagates the displacement locally through cubic basis-like
// falloff, then invokes the same bounded constrained optimizer. Corridor replanning remains the
// caller's fallback when this local elastic deformation is not legal.
[[nodiscard]] SplineTrajectoryOptimizationResult
deform_cubic_bspline_elastic(CubicBSplineRoute& route, SplineVector new_start, SplineVector new_end,
                             const SplineTrajectoryOptimizationConfig& config,
                             const SplineClearanceEvaluator& clearance_evaluator);

// Applies one collision-normal elastic displacement only to controls whose cubic basis influences
// the affected parameter, then runs bounded local trajectory optimization. Interactive callers use
// DRC/collision-envelope attribution to select affected splines and replan their corridor only when
// this operation returns a non-accepted result.
[[nodiscard]] SplineTrajectoryOptimizationResult
deform_cubic_bspline_elastic_at(CubicBSplineRoute& route, long double collision_parameter,
                                SplineVector obstacle_normal, Nanometres displacement,
                                const SplineTrajectoryOptimizationConfig& config,
                                const SplineClearanceEvaluator& clearance_evaluator);

} // namespace flux
