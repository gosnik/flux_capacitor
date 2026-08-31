#include "flux/core/spline_route.hpp"

#include "flux/core/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <numeric>
#include <optional>
#include <utility>
#include <vector>

namespace flux {
namespace {

[[nodiscard]] std::vector<long double> knots(std::size_t control_count) {
  constexpr std::size_t degree = 3U;
  std::vector<long double> result(control_count + degree + 1U, 0.0L);
  if (control_count < degree + 1U) {
    return result;
  }
  const auto denominator = static_cast<long double>(control_count - degree);
  for (std::size_t index = degree + 1U; index < control_count; ++index) {
    result[index] = static_cast<long double>(index - degree) / denominator;
  }
  std::fill(result.begin() + static_cast<std::ptrdiff_t>(control_count), result.end(), 1.0L);
  return result;
}

[[nodiscard]] std::vector<long double> basis_for_knots(std::size_t control_count,
                                                       std::size_t degree,
                                                       const std::vector<long double>& knot,
                                                       long double parameter) {
  if (control_count < degree + 1U || knot.size() != control_count + degree + 1U) {
    return {};
  }
  const auto u = std::clamp(parameter, 0.0L, 1.0L);
  std::vector<long double> basis(control_count, 0.0L);
  if (u == 1.0L) {
    basis.back() = 1.0L;
    return basis;
  }
  for (std::size_t index = 0U; index < control_count; ++index) {
    basis[index] = knot[index] <= u && u < knot[index + 1U] ? 1.0L : 0.0L;
  }
  for (std::size_t order = 1U; order <= degree; ++order) {
    std::vector<long double> next(control_count, 0.0L);
    for (std::size_t index = 0U; index < control_count; ++index) {
      const auto left_denominator = knot[index + order] - knot[index];
      const auto right_denominator = knot[index + order + 1U] - knot[index + 1U];
      if (left_denominator > 0.0L) {
        next[index] += (u - knot[index]) / left_denominator * basis[index];
      }
      if (right_denominator > 0.0L && index + 1U < control_count) {
        next[index] += (knot[index + order + 1U] - u) / right_denominator * basis[index + 1U];
      }
    }
    basis = std::move(next);
  }
  return basis;
}

[[nodiscard]] SplineVector evaluate_controls(const std::vector<SplineVector>& controls,
                                             std::size_t degree,
                                             const std::vector<long double>& knot,
                                             long double parameter) {
  const auto basis = basis_for_knots(controls.size(), degree, knot, parameter);
  SplineVector result;
  for (std::size_t index = 0U; index < basis.size(); ++index) {
    result.x += basis[index] * controls[index].x;
    result.y += basis[index] * controls[index].y;
  }
  return result;
}

[[nodiscard]] SplineVector normalized(SplineVector vector) {
  const auto length = std::hypot(vector.x, vector.y);
  return length > 0.0L ? SplineVector{vector.x / length, vector.y / length} : SplineVector{};
}

[[nodiscard]] long double tangent_penalty(SplineVector tangent,
                                          const std::optional<SplineVector>& preferred) {
  if (!preferred) {
    return 0.0L;
  }
  const auto first = normalized(tangent);
  const auto second = normalized(*preferred);
  return 1.0L - std::clamp(first.x * second.x + first.y * second.y, -1.0L, 1.0L);
}

[[nodiscard]] bool forward_collinear(Point first, Point middle, Point last) {
  const auto first_x = static_cast<long double>(middle.x.value() - first.x.value());
  const auto first_y = static_cast<long double>(middle.y.value() - first.y.value());
  const auto second_x = static_cast<long double>(last.x.value() - middle.x.value());
  const auto second_y = static_cast<long double>(last.y.value() - middle.y.value());
  return first_x * second_y == first_y * second_x &&
         first_x * second_x + first_y * second_y >= 0.0L;
}

// A cubic control polygon should describe changes of direction, not the sampling density of its
// source polyline. In particular, regional routing used to feed repeated topology-lock controls
// and every sampled grid step into this constructor. Those redundant controls gave the optimizer
// enough independent degrees of freedom to create visible ripples along otherwise simple runs.
[[nodiscard]] std::vector<Point> canonical_spline_polyline(const std::vector<Point>& polyline) {
  std::vector<Point> result;
  result.reserve(polyline.size());
  for (const auto point : polyline) {
    if (!result.empty() && result.back() == point) {
      continue;
    }
    if (result.size() >= 2U &&
        forward_collinear(result[result.size() - 2U], result.back(), point)) {
      result.back() = point;
      continue;
    }
    result.push_back(point);
  }
  return result;
}

struct SplineShapeAssessment {
  long double total_turn_radians{0.0L};
  std::size_t inflections{0U};
};

[[nodiscard]] SplineShapeAssessment
assess_direction_sequence(const std::vector<SplineVector>& directions) {
  SplineShapeAssessment result;
  int prior_sign = 0;
  std::optional<SplineVector> prior;
  constexpr long double meaningful_turn = 1.0e-7L;
  for (const auto direction : directions) {
    const auto unit = normalized(direction);
    if (!(std::hypotl(unit.x, unit.y) > 0.0L)) {
      continue;
    }
    if (prior) {
      const auto cross = prior->x * unit.y - prior->y * unit.x;
      const auto dot = std::clamp(prior->x * unit.x + prior->y * unit.y, -1.0L, 1.0L);
      const auto turn = std::atan2(cross, dot);
      if (std::abs(turn) > meaningful_turn) {
        result.total_turn_radians += std::abs(turn);
        const auto sign = turn > 0.0L ? 1 : -1;
        if (prior_sign != 0 && sign != prior_sign) {
          ++result.inflections;
        }
        prior_sign = sign;
      }
    }
    prior = unit;
  }
  return result;
}

[[nodiscard]] SplineShapeAssessment assess_anchor_shape(const CubicBSplineRoute& route) {
  std::vector<SplineVector> directions;
  directions.reserve(route.corridor_anchors.size());
  for (std::size_t index = 1U; index < route.corridor_anchors.size(); ++index) {
    directions.push_back(
        SplineVector{route.corridor_anchors[index].x - route.corridor_anchors[index - 1U].x,
                     route.corridor_anchors[index].y - route.corridor_anchors[index - 1U].y});
  }
  return assess_direction_sequence(directions);
}

[[nodiscard]] SplineShapeAssessment
assess_sample_shape(const std::vector<SplineDifferential>& samples) {
  std::vector<SplineVector> directions;
  directions.reserve(samples.size());
  for (const auto& sample : samples) {
    directions.push_back(sample.tangent);
  }
  return assess_direction_sequence(directions);
}

struct TrajectoryAssessment {
  long double objective{std::numeric_limits<long double>::infinity()};
  std::size_t violations{std::numeric_limits<std::size_t>::max()};
  std::size_t unnecessary_inflections{std::numeric_limits<std::size_t>::max()};
  long double excess_turn_radians{std::numeric_limits<long double>::infinity()};
};

[[nodiscard]] TrajectoryAssessment
assess_trajectory(const CubicBSplineRoute& route, const SplineTrajectoryOptimizationConfig& config,
                  const SplineClearanceEvaluator& clearance_evaluator,
                  PreparedCubicBSplineEvaluator& evaluator) {
  TrajectoryAssessment result;
  if (route.control_points.size() < 4U || config.samples < 3U) {
    return result;
  }
  result.objective = 0.0L;
  result.violations = 0U;
  if (!evaluator.update(route)) {
    return result;
  }
  const auto& samples = evaluator.samples();
  const auto anchor_shape = assess_anchor_shape(route);
  const auto sample_shape = assess_sample_shape(samples);
  result.unnecessary_inflections = sample_shape.inflections > anchor_shape.inflections
                                       ? sample_shape.inflections - anchor_shape.inflections
                                       : 0U;
  // Numerical derivative noise below this threshold is not a physical turn. Above it, total
  // turning is a useful variation bound: a rounded corner retains the source turn while a ripple
  // necessarily adds turning even when its length happens to improve.
  constexpr long double turn_tolerance = 1.0e-5L;
  result.excess_turn_radians = std::max(0.0L, sample_shape.total_turn_radians -
                                                  anchor_shape.total_turn_radians - turn_tolerance);
  constexpr long double nm_per_mm = 1'000'000.0L;
  long double prior_curvature = samples.front().curvature_per_nm * nm_per_mm;
  for (std::size_t index = 0U; index < samples.size(); ++index) {
    const auto& sample = samples[index];
    if (index > 0U) {
      result.objective += config.weights.length *
                          std::hypot(sample.position.x - samples[index - 1U].position.x,
                                     sample.position.y - samples[index - 1U].position.y) /
                          nm_per_mm;
    }
    const auto curvature = sample.curvature_per_nm * nm_per_mm;
    result.objective += config.weights.curvature * curvature * curvature;
    if (index > 0U) {
      const auto change = curvature - prior_curvature;
      result.objective += config.weights.curvature_change * change * change;
    }
    prior_curvature = curvature;
    if (config.minimum_bend_radius.value() > 0 && sample.bend_radius_nm > 0.0L &&
        sample.bend_radius_nm < static_cast<long double>(config.minimum_bend_radius.value())) {
      const auto deficit =
          (static_cast<long double>(config.minimum_bend_radius.value()) - sample.bend_radius_nm) /
          nm_per_mm;
      result.objective += config.weights.bend_radius * deficit * deficit;
      ++result.violations;
    }
    if (clearance_evaluator) {
      const auto probe = clearance_evaluator(sample.position);
      const auto deficit =
          static_cast<long double>(config.minimum_clearance.value()) - probe.clearance_nm;
      if (!probe.represented_legal || deficit > 0.0L) {
        result.objective +=
            config.weights.clearance * std::pow(std::max(0.0L, deficit) / nm_per_mm, 2.0L);
        ++result.violations;
      } else {
        // A reciprocal barrier supplies the elastic obstacle-repulsion force even after the hard
        // minimum has been met. It approaches zero in open space without redefining legality.
        const auto scale = static_cast<long double>(
            std::max<std::int64_t>(100'000, config.minimum_clearance.value()));
        const auto safe_clearance = std::max(1.0L, probe.clearance_nm);
        result.objective +=
            config.weights.clearance * scale * scale / (safe_clearance * safe_clearance);
      }
    }
  }
  result.objective += config.weights.tangent *
                      (tangent_penalty(samples.front().tangent, config.preferred_start_tangent) +
                       tangent_penalty(samples.back().tangent, config.preferred_end_tangent));
  for (std::size_t index = 2U; index + 2U < route.control_points.size(); ++index) {
    const auto anchor = index < route.corridor_anchors.size() ? route.corridor_anchors[index]
                                                              : route.control_points[index];
    const auto displacement = std::hypot(route.control_points[index].x - anchor.x,
                                         route.control_points[index].y - anchor.y) /
                              nm_per_mm;
    result.objective += config.weights.elastic_anchor * displacement * displacement;
    const auto before = route.control_points[index - 1U];
    const auto after = route.control_points[index + 1U];
    const auto second_x = after.x - 2.0L * route.control_points[index].x + before.x;
    const auto second_y = after.y - 2.0L * route.control_points[index].y + before.y;
    result.objective += config.weights.control_complexity *
                        (second_x * second_x + second_y * second_y) / (nm_per_mm * nm_per_mm);
  }
  return result;
}

void clamp_control_to_corridor(CubicBSplineRoute& route, std::size_t index,
                               Nanometres maximum_displacement) {
  if (index >= route.corridor_anchors.size() || maximum_displacement.value() <= 0) {
    return;
  }
  const auto anchor = route.corridor_anchors[index];
  const auto dx = route.control_points[index].x - anchor.x;
  const auto dy = route.control_points[index].y - anchor.y;
  const auto distance = std::hypot(dx, dy);
  const auto maximum = static_cast<long double>(maximum_displacement.value());
  if (distance > maximum) {
    route.control_points[index] =
        SplineVector{anchor.x + dx * maximum / distance, anchor.y + dy * maximum / distance};
  }
}

[[nodiscard]] SplineVector quantized_vector(Point point) {
  return {static_cast<long double>(point.x.value()), static_cast<long double>(point.y.value())};
}

[[nodiscard]] Point quantize(SplineVector point) {
  constexpr auto low = static_cast<long double>(std::numeric_limits<std::int64_t>::min());
  constexpr auto high = static_cast<long double>(std::numeric_limits<std::int64_t>::max());
  return Point{Nanometres{static_cast<std::int64_t>(std::llround(std::clamp(point.x, low, high)))},
               Nanometres{static_cast<std::int64_t>(std::llround(std::clamp(point.y, low, high)))}};
}

[[nodiscard]] long double point_chord_distance(SplineVector point, SplineVector start,
                                               SplineVector end) {
  const auto dx = end.x - start.x;
  const auto dy = end.y - start.y;
  const auto length_squared = dx * dx + dy * dy;
  if (!(length_squared > 0.0L)) {
    return std::hypot(point.x - start.x, point.y - start.y);
  }
  const auto parameter = std::clamp(
      ((point.x - start.x) * dx + (point.y - start.y) * dy) / length_squared, 0.0L, 1.0L);
  return std::hypot(point.x - (start.x + parameter * dx), point.y - (start.y + parameter * dy));
}

[[nodiscard]] bool consistent_turn(const std::vector<SplineSample>& samples, std::size_t first,
                                   std::size_t last) {
  int sign = 0;
  for (auto index = first + 1U; index < last; ++index) {
    const auto& a = samples[index - 1U].position;
    const auto& b = samples[index].position;
    const auto& c = samples[index + 1U].position;
    const auto cross = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
    const auto current = cross > 1.0L ? 1 : cross < -1.0L ? -1 : 0;
    if (current == 0) {
      continue;
    }
    if (sign != 0 && current != sign) {
      return false;
    }
    sign = current;
  }
  return sign != 0;
}

} // namespace

PreparedCubicBSplineEvaluator::PreparedCubicBSplineEvaluator(std::size_t control_count,
                                                             std::size_t sample_count)
    : control_count_(control_count) {
  constexpr std::size_t degree = 3U;
  if (control_count < degree + 1U || sample_count < 2U) {
    control_count_ = 0U;
    return;
  }
  const auto original_knots = knots(control_count);
  const std::vector<long double> first_knots(original_knots.begin() + 1, original_knots.end() - 1);
  const std::vector<long double> second_knots(first_knots.begin() + 1, first_knots.end() - 1);
  position_coefficients_.reserve(sample_count);
  tangent_coefficients_.reserve(sample_count);
  second_coefficients_.reserve(sample_count);
  samples_.resize(sample_count);
  for (std::size_t sample = 0U; sample < sample_count; ++sample) {
    const auto parameter =
        static_cast<long double>(sample) / static_cast<long double>(sample_count - 1U);
    position_coefficients_.push_back(
        basis_for_knots(control_count, degree, original_knots, parameter));
    std::vector<long double> tangent(control_count, 0.0L);
    const auto first_basis =
        basis_for_knots(control_count - 1U, degree - 1U, first_knots, parameter);
    std::vector<long double> first_scales(control_count - 1U, 0.0L);
    for (std::size_t index = 0U; index + 1U < control_count; ++index) {
      const auto denominator = original_knots[index + 4U] - original_knots[index + 1U];
      first_scales[index] = denominator > 0.0L ? 3.0L / denominator : 0.0L;
      tangent[index] -= first_basis[index] * first_scales[index];
      tangent[index + 1U] += first_basis[index] * first_scales[index];
    }
    tangent_coefficients_.push_back(std::move(tangent));
    std::vector<long double> second(control_count, 0.0L);
    const auto second_basis =
        basis_for_knots(control_count - 2U, degree - 2U, second_knots, parameter);
    for (std::size_t index = 0U; index + 2U < control_count; ++index) {
      const auto denominator = first_knots[index + 3U] - first_knots[index + 1U];
      const auto scale = denominator > 0.0L ? 2.0L / denominator : 0.0L;
      const auto weighted = second_basis[index] * scale;
      second[index] += weighted * first_scales[index];
      second[index + 1U] -= weighted * (first_scales[index] + first_scales[index + 1U]);
      second[index + 2U] += weighted * first_scales[index + 1U];
    }
    second_coefficients_.push_back(std::move(second));
  }
}

bool PreparedCubicBSplineEvaluator::valid() const {
  return control_count_ >= 4U && !samples_.empty() &&
         position_coefficients_.size() == samples_.size() &&
         tangent_coefficients_.size() == samples_.size() &&
         second_coefficients_.size() == samples_.size();
}

bool PreparedCubicBSplineEvaluator::update(const CubicBSplineRoute& route) {
  if (!valid() || route.control_points.size() != control_count_) {
    return false;
  }
  for (std::size_t sample = 0U; sample < samples_.size(); ++sample) {
    SplineDifferential differential;
    for (std::size_t control = 0U; control < control_count_; ++control) {
      const auto& point = route.control_points[control];
      differential.position.x += position_coefficients_[sample][control] * point.x;
      differential.position.y += position_coefficients_[sample][control] * point.y;
      differential.tangent.x += tangent_coefficients_[sample][control] * point.x;
      differential.tangent.y += tangent_coefficients_[sample][control] * point.y;
      differential.second_derivative.x += second_coefficients_[sample][control] * point.x;
      differential.second_derivative.y += second_coefficients_[sample][control] * point.y;
    }
    const auto speed_squared = differential.tangent.x * differential.tangent.x +
                               differential.tangent.y * differential.tangent.y;
    if (speed_squared > 0.0L) {
      const auto numerator = std::abs(differential.tangent.x * differential.second_derivative.y -
                                      differential.tangent.y * differential.second_derivative.x);
      differential.curvature_per_nm = numerator / std::pow(speed_squared, 1.5L);
      differential.bend_radius_nm = differential.curvature_per_nm > 0.0L
                                        ? 1.0L / differential.curvature_per_nm
                                        : std::numeric_limits<long double>::infinity();
    }
    samples_[sample] = differential;
  }
  return true;
}

std::optional<CubicBSplineRoute> make_cubic_bspline_route(const std::vector<Point>& polyline) {
  auto canonical = canonical_spline_polyline(polyline);
  if (canonical.size() < 2U || canonical.front() == canonical.back()) {
    return std::nullopt;
  }
  if (canonical.size() == 2U) {
    canonical.insert(
        canonical.begin() + 1U,
        Point{Nanometres{std::midpoint(canonical.front().x.value(), canonical.back().x.value())},
              Nanometres{std::midpoint(canonical.front().y.value(), canonical.back().y.value())}});
  }
  CubicBSplineRoute route;
  route.control_points.reserve(canonical.size() + 2U);
  const auto first = quantized_vector(canonical.front());
  const auto second = quantized_vector(canonical[1U]);
  route.control_points.push_back(first);
  route.control_points.push_back(
      SplineVector{first.x + (second.x - first.x) * 0.5L, first.y + (second.y - first.y) * 0.5L});
  for (std::size_t index = 1U; index + 1U < canonical.size(); ++index) {
    route.control_points.push_back(quantized_vector(canonical[index]));
  }
  const auto previous = quantized_vector(canonical[canonical.size() - 2U]);
  const auto last = quantized_vector(canonical.back());
  route.control_points.push_back(
      SplineVector{last.x + (previous.x - last.x) * 0.5L, last.y + (previous.y - last.y) * 0.5L});
  route.control_points.push_back(last);
  route.corridor_anchors = route.control_points;
  return route;
}

std::vector<long double> cubic_bspline_basis(const CubicBSplineRoute& route,
                                             long double parameter) {
  constexpr std::size_t degree = 3U;
  const auto count = route.control_points.size();
  if (count < degree + 1U) {
    return {};
  }
  const auto knot = knots(count);
  const auto u = std::clamp(parameter, 0.0L, 1.0L);
  std::vector<long double> basis(count, 0.0L);
  if (u == 1.0L) {
    basis.back() = 1.0L;
    return basis;
  }
  for (std::size_t index = 0U; index < count; ++index) {
    basis[index] = knot[index] <= u && u < knot[index + 1U] ? 1.0L : 0.0L;
  }
  for (std::size_t order = 1U; order <= degree; ++order) {
    std::vector<long double> next(count, 0.0L);
    for (std::size_t index = 0U; index < count; ++index) {
      const auto left_denominator = knot[index + order] - knot[index];
      const auto right_denominator = knot[index + order + 1U] - knot[index + 1U];
      if (left_denominator > 0.0L) {
        next[index] += (u - knot[index]) / left_denominator * basis[index];
      }
      if (right_denominator > 0.0L && index + 1U < count) {
        next[index] += (knot[index + order + 1U] - u) / right_denominator * basis[index + 1U];
      }
    }
    basis = std::move(next);
  }
  return basis;
}

SplineVector evaluate_cubic_bspline(const CubicBSplineRoute& route, long double parameter) {
  const auto basis = cubic_bspline_basis(route, parameter);
  SplineVector result;
  for (std::size_t index = 0U; index < basis.size(); ++index) {
    result.x += basis[index] * route.control_points[index].x;
    result.y += basis[index] * route.control_points[index].y;
  }
  return result;
}

SplineDifferential evaluate_cubic_bspline_differential(const CubicBSplineRoute& route,
                                                       long double parameter) {
  SplineDifferential result;
  if (route.control_points.size() < 4U) {
    return result;
  }
  const auto original_knots = knots(route.control_points.size());
  result.position = evaluate_controls(route.control_points, 3U, original_knots, parameter);
  std::vector<SplineVector> first_controls;
  first_controls.reserve(route.control_points.size() - 1U);
  for (std::size_t index = 0U; index + 1U < route.control_points.size(); ++index) {
    const auto denominator = original_knots[index + 4U] - original_knots[index + 1U];
    const auto scale = denominator > 0.0L ? 3.0L / denominator : 0.0L;
    first_controls.push_back(
        SplineVector{scale * (route.control_points[index + 1U].x - route.control_points[index].x),
                     scale * (route.control_points[index + 1U].y - route.control_points[index].y)});
  }
  std::vector<long double> first_knots(original_knots.begin() + 1, original_knots.end() - 1);
  result.tangent = evaluate_controls(first_controls, 2U, first_knots, parameter);
  std::vector<SplineVector> second_controls;
  second_controls.reserve(first_controls.size() - 1U);
  for (std::size_t index = 0U; index + 1U < first_controls.size(); ++index) {
    const auto denominator = first_knots[index + 3U] - first_knots[index + 1U];
    const auto scale = denominator > 0.0L ? 2.0L / denominator : 0.0L;
    second_controls.push_back(
        SplineVector{scale * (first_controls[index + 1U].x - first_controls[index].x),
                     scale * (first_controls[index + 1U].y - first_controls[index].y)});
  }
  std::vector<long double> second_knots(first_knots.begin() + 1, first_knots.end() - 1);
  result.second_derivative = evaluate_controls(second_controls, 1U, second_knots, parameter);
  const auto speed_squared =
      result.tangent.x * result.tangent.x + result.tangent.y * result.tangent.y;
  if (speed_squared > 0.0L) {
    const auto numerator = std::abs(result.tangent.x * result.second_derivative.y -
                                    result.tangent.y * result.second_derivative.x);
    result.curvature_per_nm = numerator / std::pow(speed_squared, 1.5L);
    result.bend_radius_nm = result.curvature_per_nm > 0.0L
                                ? 1.0L / result.curvature_per_nm
                                : std::numeric_limits<long double>::infinity();
  }
  return result;
}

std::vector<SplineSample> adaptive_tessellate_cubic_bspline(const CubicBSplineRoute& route,
                                                            Nanometres tolerance,
                                                            std::size_t maximum_depth,
                                                            std::size_t maximum_samples) {
  std::vector<SplineSample> result;
  if (route.control_points.size() < 4U || tolerance.value() <= 0 || maximum_samples < 2U) {
    return result;
  }
  const auto knot = knots(route.control_points.size());
  const auto append_interval = [&](auto&& self, long double first_parameter, SplineVector first,
                                   long double last_parameter, SplineVector last,
                                   std::size_t depth) -> void {
    if (result.size() >= maximum_samples) {
      return;
    }
    const auto span = last_parameter - first_parameter;
    const auto one_third_parameter = first_parameter + span / 3.0L;
    const auto middle_parameter = first_parameter + span / 2.0L;
    const auto two_thirds_parameter = first_parameter + span * 2.0L / 3.0L;
    const auto one_third = evaluate_cubic_bspline(route, one_third_parameter);
    const auto middle = evaluate_cubic_bspline(route, middle_parameter);
    const auto two_thirds = evaluate_cubic_bspline(route, two_thirds_parameter);
    const auto flatness = std::max({point_chord_distance(one_third, first, last),
                                    point_chord_distance(middle, first, last),
                                    point_chord_distance(two_thirds, first, last)});
    if (depth >= maximum_depth || flatness <= static_cast<long double>(tolerance.value()) ||
        result.size() + 1U >= maximum_samples) {
      result.push_back(SplineSample{last_parameter, last});
      return;
    }
    self(self, first_parameter, first, middle_parameter, middle, depth + 1U);
    self(self, middle_parameter, middle, last_parameter, last, depth + 1U);
  };

  result.push_back(SplineSample{0.0L, evaluate_cubic_bspline(route, 0.0L)});
  for (std::size_t index = 3U; index < route.control_points.size(); ++index) {
    const auto first_parameter = knot[index];
    const auto last_parameter = knot[index + 1U];
    if (!(last_parameter > first_parameter)) {
      continue;
    }
    append_interval(append_interval, first_parameter,
                    evaluate_cubic_bspline(route, first_parameter), last_parameter,
                    evaluate_cubic_bspline(route, last_parameter), 0U);
    if (result.size() >= maximum_samples) {
      break;
    }
  }
  if (result.empty() || result.back().parameter != 1.0L) {
    if (result.size() < maximum_samples) {
      result.push_back(SplineSample{1.0L, evaluate_cubic_bspline(route, 1.0L)});
    } else {
      result.back() = SplineSample{1.0L, evaluate_cubic_bspline(route, 1.0L)};
    }
  }
  std::vector<SplineSample> deduplicated;
  deduplicated.reserve(result.size());
  for (const auto& sample : result) {
    if (!deduplicated.empty() &&
        quantize(deduplicated.back().position) == quantize(sample.position)) {
      deduplicated.back() = sample;
    } else {
      deduplicated.push_back(sample);
    }
  }
  return deduplicated;
}

std::vector<SplineFittedPiece>
fit_spline_samples_to_lines_and_arcs(const std::vector<SplineSample>& samples, Nanometres tolerance,
                                     std::size_t maximum_arc_samples, std::size_t maximum_pieces) {
  std::vector<SplineFittedPiece> result;
  if (samples.size() < 2U || tolerance.value() < 0 || maximum_arc_samples < 3U ||
      maximum_pieces == 0U) {
    return result;
  }
  for (std::size_t first = 0U; first + 1U < samples.size() && result.size() < maximum_pieces;) {
    std::optional<SplineFittedPiece> selected;
    const auto furthest = std::min(samples.size() - 1U, first + maximum_arc_samples - 1U);
    for (auto last = furthest; last >= first + 2U; --last) {
      if (!consistent_turn(samples, first, last)) {
        if (last == first + 2U) {
          break;
        }
        continue;
      }
      const auto middle_index = first + (last - first) / 2U;
      const auto start = quantize(samples[first].position);
      const auto middle = quantize(samples[middle_index].position);
      const auto end = quantize(samples[last].position);
      if (start == middle || middle == end || start == end ||
          !geometry::circular_arc_parameters(start, middle, end)) {
        if (last == first + 2U) {
          break;
        }
        continue;
      }
      bool fits = true;
      for (auto index = first + 1U; index < last; ++index) {
        if (geometry::point_circular_arc_distance_nm(quantize(samples[index].position), start,
                                                     middle, end) >
            static_cast<long double>(tolerance.value())) {
          fits = false;
          break;
        }
      }
      if (fits) {
        selected = SplineFittedPiece{SplineFittedPieceKind::circular_arc,
                                     start,
                                     middle,
                                     end,
                                     samples[first].parameter,
                                     samples[last].parameter};
        break;
      }
      if (last == first + 2U) {
        break;
      }
    }
    if (selected) {
      result.push_back(*selected);
      const auto last_parameter = selected->last_parameter;
      const auto found = std::find_if(
          samples.begin() + static_cast<std::ptrdiff_t>(first + 1U), samples.end(),
          [last_parameter](const auto& sample) { return sample.parameter == last_parameter; });
      first = static_cast<std::size_t>(found - samples.begin());
      continue;
    }
    const auto start = quantize(samples[first].position);
    const auto end = quantize(samples[first + 1U].position);
    if (start != end) {
      result.push_back(SplineFittedPiece{SplineFittedPieceKind::segment, start, start, end,
                                         samples[first].parameter, samples[first + 1U].parameter});
    }
    ++first;
  }
  if (result.empty() || result.back().end != quantize(samples.back().position)) {
    return {};
  }
  return result;
}

std::size_t shove_cubic_bspline_control_corridor(CubicBSplineRoute& route, long double parameter,
                                                 SplineVector normal, Nanometres distance,
                                                 Nanometres maximum_corridor_displacement) {
  if (route.control_points.size() < 5U ||
      route.corridor_anchors.size() != route.control_points.size() || distance.value() <= 0 ||
      maximum_corridor_displacement.value() <= 0) {
    return 0U;
  }
  const auto normal_length = std::hypot(normal.x, normal.y);
  if (!(normal_length > 0.0L)) {
    return 0U;
  }
  normal.x /= normal_length;
  normal.y /= normal_length;
  const auto basis = cubic_bspline_basis(route, parameter);
  long double squared_influence = 0.0L;
  for (std::size_t index = 2U; index + 2U < basis.size(); ++index) {
    squared_influence += basis[index] * basis[index];
  }
  if (!(squared_influence > 0.0L)) {
    return 0U;
  }
  std::size_t moved = 0U;
  for (std::size_t index = 2U; index + 2U < basis.size(); ++index) {
    if (!(basis[index] > 0.0L)) {
      continue;
    }
    const auto alpha = basis[index] / squared_influence;
    auto candidate = route.control_points[index];
    candidate.x += alpha * static_cast<long double>(distance.value()) * normal.x;
    candidate.y += alpha * static_cast<long double>(distance.value()) * normal.y;
    const auto& anchor = route.corridor_anchors[index];
    const auto dx = candidate.x - anchor.x;
    const auto dy = candidate.y - anchor.y;
    const auto displacement = std::hypot(dx, dy);
    const auto maximum = static_cast<long double>(maximum_corridor_displacement.value());
    if (displacement > maximum) {
      candidate.x = anchor.x + dx * maximum / displacement;
      candidate.y = anchor.y + dy * maximum / displacement;
    }
    if (candidate != route.control_points[index]) {
      route.control_points[index] = candidate;
      ++moved;
    }
  }
  return moved;
}

long double cubic_bspline_curvature_objective(const CubicBSplineRoute& route, std::size_t samples) {
  if (samples < 3U || route.control_points.size() < 4U) {
    return std::numeric_limits<long double>::infinity();
  }
  long double objective = 0.0L;
  auto previous = evaluate_cubic_bspline(route, 0.0L);
  auto current = evaluate_cubic_bspline(route, 1.0L / static_cast<long double>(samples - 1U));
  for (std::size_t index = 2U; index < samples; ++index) {
    const auto next = evaluate_cubic_bspline(route, static_cast<long double>(index) /
                                                        static_cast<long double>(samples - 1U));
    const auto first_x = current.x - previous.x;
    const auto first_y = current.y - previous.y;
    const auto second_x = next.x - current.x;
    const auto second_y = next.y - current.y;
    const auto first_length = std::hypot(first_x, first_y);
    const auto second_length = std::hypot(second_x, second_y);
    if (first_length > 0.0L && second_length > 0.0L) {
      const auto cross = first_x * second_y - first_y * second_x;
      objective += cross * cross / (first_length * second_length);
    }
    previous = current;
    current = next;
  }
  return objective;
}

SplineTrajectoryOptimizationResult
optimize_cubic_bspline_trajectory(CubicBSplineRoute& route,
                                  const SplineTrajectoryOptimizationConfig& config,
                                  const SplineClearanceEvaluator& clearance_evaluator) {
  SplineTrajectoryOptimizationResult result;
  if (route.control_points.size() < 5U ||
      route.corridor_anchors.size() != route.control_points.size() || config.samples < 3U ||
      config.maximum_iterations == 0U || config.maximum_candidate_checks == 0U ||
      config.control_step.value() <= 0 || config.gradient_probe_step.value() <= 0 ||
      config.maximum_corridor_displacement.value() <= 0) {
    return result;
  }
  PreparedCubicBSplineEvaluator evaluator(route.control_points.size(), config.samples);
  if (!evaluator.valid()) {
    return result;
  }
  auto best = assess_trajectory(route, config, clearance_evaluator, evaluator);
  result.initial_objective = best.objective;
  result.initial_violations = best.violations;
  const auto better = [](const TrajectoryAssessment& lhs, const TrajectoryAssessment& rhs) {
    if (lhs.violations != rhs.violations) {
      return lhs.violations < rhs.violations;
    }
    if (lhs.unnecessary_inflections != rhs.unnecessary_inflections) {
      return lhs.unnecessary_inflections < rhs.unnecessary_inflections;
    }
    if (lhs.excess_turn_radians + 1.0e-9L < rhs.excess_turn_radians) {
      return true;
    }
    if (rhs.excess_turn_radians + 1.0e-9L < lhs.excess_turn_radians) {
      return false;
    }
    return lhs.objective + 1.0e-12L < rhs.objective;
  };
  for (std::size_t iteration = 0U; iteration < config.maximum_iterations; ++iteration) {
    bool improved = false;
    for (std::size_t index = 2U; index + 2U < route.control_points.size(); ++index) {
      if (result.candidate_checks + 4U > config.maximum_candidate_checks) {
        result.budget_exhausted = true;
        break;
      }
      const auto original = route.control_points[index];
      const auto probe = static_cast<long double>(config.gradient_probe_step.value());
      route.control_points[index].x = original.x + probe;
      clamp_control_to_corridor(route, index, config.maximum_corridor_displacement);
      const auto positive_x = assess_trajectory(route, config, clearance_evaluator, evaluator);
      route.control_points[index] = original;
      route.control_points[index].x = original.x - probe;
      clamp_control_to_corridor(route, index, config.maximum_corridor_displacement);
      const auto negative_x = assess_trajectory(route, config, clearance_evaluator, evaluator);
      route.control_points[index] = original;
      route.control_points[index].y = original.y + probe;
      clamp_control_to_corridor(route, index, config.maximum_corridor_displacement);
      const auto positive_y = assess_trajectory(route, config, clearance_evaluator, evaluator);
      route.control_points[index] = original;
      route.control_points[index].y = original.y - probe;
      clamp_control_to_corridor(route, index, config.maximum_corridor_displacement);
      const auto negative_y = assess_trajectory(route, config, clearance_evaluator, evaluator);
      route.control_points[index] = original;
      result.candidate_checks += 4U;
      const auto scalar = [](const TrajectoryAssessment& value) {
        return static_cast<long double>(value.violations) * 1.0e12L +
               static_cast<long double>(value.unnecessary_inflections) * 1.0e9L +
               value.excess_turn_radians * 1.0e6L + value.objective;
      };
      auto gradient = SplineVector{(scalar(positive_x) - scalar(negative_x)) / (2.0L * probe),
                                   (scalar(positive_y) - scalar(negative_y)) / (2.0L * probe)};
      const auto gradient_length = std::hypot(gradient.x, gradient.y);
      if (!(gradient_length > 0.0L) || !std::isfinite(gradient_length)) {
        continue;
      }
      gradient.x /= gradient_length;
      gradient.y /= gradient_length;
      auto step = static_cast<long double>(config.control_step.value());
      for (std::size_t backtrack = 0U; backtrack < 8U; ++backtrack) {
        if (result.candidate_checks >= config.maximum_candidate_checks) {
          result.budget_exhausted = true;
          break;
        }
        route.control_points[index] =
            SplineVector{original.x - step * gradient.x, original.y - step * gradient.y};
        clamp_control_to_corridor(route, index, config.maximum_corridor_displacement);
        const auto candidate = assess_trajectory(route, config, clearance_evaluator, evaluator);
        ++result.candidate_checks;
        if (better(candidate, best)) {
          best = candidate;
          improved = true;
          ++result.accepted_control_moves;
          break;
        }
        route.control_points[index] = original;
        step *= 0.5L;
      }
      if (result.budget_exhausted) {
        break;
      }
    }
    result.iterations = iteration + 1U;
    if (result.budget_exhausted || !improved) {
      break;
    }
  }
  result.final_objective = best.objective;
  result.final_violations = best.violations;
  result.legal = best.violations == 0U;
  result.accepted = result.legal;
  return result;
}

SplineTrajectoryOptimizationResult
deform_cubic_bspline_elastic(CubicBSplineRoute& route, SplineVector new_start, SplineVector new_end,
                             const SplineTrajectoryOptimizationConfig& config,
                             const SplineClearanceEvaluator& clearance_evaluator) {
  if (route.control_points.size() < 5U) {
    return {};
  }
  const auto start_delta = SplineVector{new_start.x - route.control_points.front().x,
                                        new_start.y - route.control_points.front().y};
  const auto end_delta = SplineVector{new_end.x - route.control_points.back().x,
                                      new_end.y - route.control_points.back().y};
  const auto denominator = static_cast<long double>(route.control_points.size() - 1U);
  std::size_t predeformed_controls = 0U;
  for (std::size_t index = 0U; index < route.control_points.size(); ++index) {
    const auto parameter = static_cast<long double>(index) / denominator;
    const auto start_influence = std::pow(1.0L - parameter, 3.0L);
    const auto end_influence = std::pow(parameter, 3.0L);
    const auto dx = start_influence * start_delta.x + end_influence * end_delta.x;
    const auto dy = start_influence * start_delta.y + end_influence * end_delta.y;
    predeformed_controls += dx != 0.0L || dy != 0.0L ? 1U : 0U;
    route.control_points[index].x += dx;
    route.control_points[index].y += dy;
    if (index < route.corridor_anchors.size()) {
      route.corridor_anchors[index].x +=
          start_influence * start_delta.x + end_influence * end_delta.x;
      route.corridor_anchors[index].y +=
          start_influence * start_delta.y + end_influence * end_delta.y;
    }
  }
  route.control_points.front() = new_start;
  route.control_points.back() = new_end;
  route.corridor_anchors.front() = new_start;
  route.corridor_anchors.back() = new_end;
  auto result = optimize_cubic_bspline_trajectory(route, config, clearance_evaluator);
  result.elastic_predeformed_controls = predeformed_controls;
  return result;
}

SplineTrajectoryOptimizationResult
deform_cubic_bspline_elastic_at(CubicBSplineRoute& route, long double collision_parameter,
                                SplineVector obstacle_normal, Nanometres displacement,
                                const SplineTrajectoryOptimizationConfig& config,
                                const SplineClearanceEvaluator& clearance_evaluator) {
  const auto moved =
      shove_cubic_bspline_control_corridor(route, collision_parameter, obstacle_normal,
                                           displacement, config.maximum_corridor_displacement);
  if (moved == 0U) {
    return {};
  }
  auto result = optimize_cubic_bspline_trajectory(route, config, clearance_evaluator);
  result.elastic_predeformed_controls = moved;
  return result;
}

} // namespace flux
