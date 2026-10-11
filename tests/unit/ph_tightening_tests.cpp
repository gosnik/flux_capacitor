#include "flux/core/spline_route.hpp"
#include "flux/core/routing_kernel.hpp"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <utility>

namespace {

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

} // namespace

int main() {
  const auto detour = [] {
    flux::QuinticPHRoute route;
    route.control_points = {{0.0L, 0.0L}, {100'000.0L, 80'000.0L}, {200'000.0L, 0.0L}};
    route.corridor_anchors = route.control_points;
    require(flux::rebuild_quintic_ph_route(route), "fixture PH route must rebuild");
    return route;
  };
  auto route = detour();
  const auto endpoints = std::pair{route.control_points.front(), route.control_points.back()};
  flux::SplineTrajectoryOptimizationConfig config;
  config.minimum_clearance = flux::Nanometres{0};
  require(flux::tauten_quintic_ph_route(route, config, {}, [](const auto&) { return true; }),
          "a redundant detour anchor must be removed when the shortened PH route is certified");
  require(route.control_points.size() == 2U && route.control_points.front() == endpoints.first &&
              route.control_points.back() == endpoints.second,
          "tautening must preserve endpoints while removing only interior anchors");

  const auto exact_length = [](const flux::QuinticPHRoute& candidate) {
    long double length = 0.0L;
    for (const auto& segment : candidate.segments) length += flux::quintic_ph_arc_length(segment);
    return length;
  };
  auto optimized = detour();
  const auto original_length = exact_length(optimized);
  // The entire curve is legal, but points higher above the axis have more clearance.
  // The old worst-clearance ranking rewarded moving farther away and growing the route.
  const auto legal_clearance = [](flux::SplineVector point) {
    return flux::SplineClearanceProbe{100'000.0L + std::abs(point.y), {}, true};
  };
  config.maximum_iterations = 4U;
  config.maximum_candidate_checks = 64U;
  const auto optimized_result = flux::optimize_quintic_ph_trajectory(
      optimized, config, legal_clearance);
  require(optimized_result.legal && exact_length(optimized) <= original_length + 1.0e-6L,
          "legal optimization must not grow a curve merely to gain extra clearance");
  auto bounded = detour();
  require(!flux::tauten_quintic_ph_route(bounded, config, {}, {}, 0U) &&
              bounded.control_points.size() == 3U,
          "zero removal budget must preserve the route");

  auto constrained = detour();
  require(!flux::tauten_quintic_ph_route(
              constrained, config, {}, [](const flux::QuinticPHRoute& candidate) {
                return candidate.control_points.size() >= 3U;
              }) && constrained.control_points.size() == 3U,
          "a route constraint must veto an otherwise shorter anchor removal");

  auto bend_limited = detour();
  auto bend_config = config;
  bend_config.minimum_bend_radius = flux::Nanometres::from_mm(100.0);
  require(!flux::tauten_quintic_ph_route(bend_limited, bend_config, {}) &&
              bend_limited.control_points.size() == 3U,
          "legal-only tightening must not treat an uncertified bend-limited input as legal");

  auto blocked = detour();
  const auto obstacle = [](flux::SplineVector point) {
    const auto radial = flux::SplineVector{point.x - 100'000.0L, point.y};
    const auto distance = std::hypot(radial.x, radial.y);
    const auto clearance = distance - 30'000.0L;
    const auto normal = distance > 0.0L
                            ? flux::SplineVector{radial.x / distance, radial.y / distance}
                            : flux::SplineVector{};
    return flux::SplineClearanceProbe{clearance, normal, clearance >= 0.0L};
  };
  require(flux::certify_quintic_ph_continuous_envelope(
              blocked, config.minimum_clearance, obstacle, config.continuous_envelope).legal,
          "obstacle fixture must start from a certified legal detour");
  const auto broad_hump_length = exact_length(blocked);
  require(flux::tauten_quintic_ph_route(blocked, config, obstacle) &&
              blocked.control_points.size() == 4U &&
              flux::certify_quintic_ph_continuous_envelope(
                  blocked, config.minimum_clearance, obstacle, config.continuous_envelope).legal &&
              exact_length(blocked) + 1.0e-6L < broad_hump_length,
          "a blocked shortcut must replace a broad hump with a shorter, legal paired skirt");
  auto pair_disabled = detour();
  auto disabled_config = config;
  disabled_config.enable_paired_obstacle_supports = false;
  require(!flux::tauten_quintic_ph_route(pair_disabled, disabled_config, obstacle) &&
              pair_disabled.control_points.size() == 3U,
          "disabling paired obstacle supports must preserve the blocked broad hump");
  auto pair_vetoed = detour();
  require(!flux::tauten_quintic_ph_route(
              pair_vetoed, config, obstacle, [](const flux::QuinticPHRoute& candidate) {
                return candidate.control_points.size() <= 3U;
              }) && pair_vetoed.control_points.size() == 3U,
          "topology constraints must veto paired support insertion");
  auto pair_budgeted = detour();
  auto pair_budget_config = config;
  pair_budget_config.maximum_candidate_checks = 1U;
  flux::PHRouteTighteningStatistics budget_statistics;
  require(!flux::tauten_quintic_ph_route(pair_budgeted, pair_budget_config, obstacle, {}, 16U,
                                       &budget_statistics) &&
              pair_budgeted.control_points.size() == 3U &&
              budget_statistics.candidate_checks == 1U,
          "the paired-support recovery must respect the tautening candidate budget");

  const auto box_obstacle = [](flux::SplineVector point) {
    constexpr long double left = 80'000.0L;
    constexpr long double right = 120'000.0L;
    constexpr long double bottom = -20'000.0L;
    constexpr long double top = 20'000.0L;
    const auto outside_x = std::max({left - point.x, 0.0L, point.x - right});
    const auto outside_y = std::max({bottom - point.y, 0.0L, point.y - top});
    const auto outside = std::hypot(outside_x, outside_y);
    if (outside > 0.0L) {
      const auto nearest = flux::SplineVector{std::clamp(point.x, left, right),
                                               std::clamp(point.y, bottom, top)};
      const auto delta = flux::SplineVector{point.x - nearest.x, point.y - nearest.y};
      return flux::SplineClearanceProbe{outside, {delta.x / outside, delta.y / outside}, true};
    }
    const auto to_left = point.x - left;
    const auto to_right = right - point.x;
    const auto to_bottom = point.y - bottom;
    const auto to_top = top - point.y;
    const auto nearest = std::min({to_left, to_right, to_bottom, to_top});
    const auto normal = nearest == to_left ? flux::SplineVector{-1.0L, 0.0L} :
                        nearest == to_right ? flux::SplineVector{1.0L, 0.0L} :
                        nearest == to_bottom ? flux::SplineVector{0.0L, -1.0L} :
                                                flux::SplineVector{0.0L, 1.0L};
    return flux::SplineClearanceProbe{-nearest, normal, false};
  };
  auto box_skirt = detour();
  const auto box_hump_length = exact_length(box_skirt);
  require(flux::tauten_quintic_ph_route(box_skirt, config, box_obstacle) &&
              box_skirt.control_points.size() == 4U &&
              flux::certify_quintic_ph_continuous_envelope(
                  box_skirt, config.minimum_clearance, box_obstacle, config.continuous_envelope).legal &&
              exact_length(box_skirt) + 1.0e-6L < box_hump_length,
          "a signed-distance box must produce a legal, shorter paired skirt");
  if (std::getenv("FLUX_WRITE_PAIRED_SUPPORT_CASES")) {
    std::ofstream report{"build/tautening-20261009/paired-support-cases.json"};
    const auto emit_route = [&](const flux::QuinticPHRoute& candidate) {
      report << "{\"length_nm\":" << static_cast<double>(exact_length(candidate))
             << ",\"controls\":[";
      for (std::size_t i = 0U; i < candidate.control_points.size(); ++i) {
        if (i) report << ',';
        const auto point = candidate.control_points[i];
        report << '[' << static_cast<double>(point.x) << ',' << static_cast<double>(point.y) << ']';
      }
      report << "],\"samples\":[";
      for (std::size_t i = 0U; i < 33U; ++i) {
        if (i) report << ',';
        const auto point = flux::evaluate_quintic_ph(candidate, static_cast<long double>(i) / 32.0L);
        report << '[' << static_cast<double>(point.x) << ',' << static_cast<double>(point.y) << ']';
      }
      report << "]}";
    };
    report << "{\"units\":\"nm\",\"circle\":{\"center\":[100000,0],\"radius\":30000,"
           << "\"before\":";
    emit_route(detour());
    report << ",\"after\":";
    emit_route(blocked);
    report << "},\"box\":{\"minimum\":[80000,-20000],\"maximum\":[120000,20000],"
           << "\"before\":";
    emit_route(detour());
    report << ",\"after\":";
    emit_route(box_skirt);
    report << "}}";
  }
  flux::QuinticPHRoute colliding;
  colliding.control_points = {{0.0L, 0.0L}, {200'000.0L, 0.0L}};
  colliding.corridor_anchors = colliding.control_points;
  require(flux::rebuild_quintic_ph_route(colliding), "collision recovery fixture must rebuild");
  auto recovery_config = config;
  recovery_config.maximum_iterations = 0U;
  recovery_config.maximum_adaptive_control_insertions = 2U;
  const auto recovery = flux::optimize_quintic_ph_trajectory(colliding, recovery_config, obstacle);
  require(recovery.legal && colliding.control_points.size() == 4U &&
              recovery.paired_obstacle_support_attempts == 1U &&
              recovery.paired_obstacle_supports_inserted == 2U &&
              recovery.paired_obstacle_support_recoveries == 1U,
          "infeasible optimisation must recover through the bounded paired-support path");
  auto legal_optimizer = detour();
  auto legal_optimizer_config = config;
  legal_optimizer_config.maximum_iterations = 0U;
  const auto legal_optimizer_before = exact_length(legal_optimizer);
  const auto legal_optimised = flux::optimize_quintic_ph_trajectory(
      legal_optimizer, legal_optimizer_config, obstacle);
  require(legal_optimised.legal && legal_optimizer.control_points.size() == 4U &&
              legal_optimised.paired_obstacle_supports_inserted == 2U &&
              legal_optimised.paired_obstacle_support_recoveries == 0U &&
              legal_optimised.candidate_checks > 0U &&
              legal_optimised.candidate_checks <= legal_optimizer_config.maximum_candidate_checks &&
              exact_length(legal_optimizer) + 1.0e-6L < legal_optimizer_before,
          "legal optimiser integration must replace a broad obstacle hump with paired supports");
  auto postpolish_veto = detour();
  auto postpolish_veto_config = legal_optimizer_config;
  postpolish_veto_config.postpolish_route_constraint =
      [](const flux::QuinticPHRoute& candidate) { return candidate.control_points.size() == 3U; };
  const auto vetoed = flux::optimize_quintic_ph_trajectory(
      postpolish_veto, postpolish_veto_config, obstacle);
  require(vetoed.legal && postpolish_veto.control_points.size() == 3U &&
              vetoed.paired_obstacle_supports_inserted == 0U &&
              vetoed.paired_obstacle_support_recoveries == 0U &&
              vetoed.candidate_checks > 0U,
          "the post-polish realization gate must retain a legal G1 station layout when its "
          "paired shortcut is unsuitable downstream");
  auto promotion = detour();
  promotion.control_points.insert(promotion.control_points.end() - 1,
                                  {150'000.0L, 20'000.0L});
  promotion.corridor_anchors = promotion.control_points;
  require(flux::rebuild_quintic_ph_route(promotion) &&
              flux::quintic_ph_max_curvature_jump(promotion) > 0.0L,
          "continuity cancellation fixture must require real promotion work");
  const auto original = promotion;
  flux::RoutingKernel kernel([&](std::stop_token token) {
    static_cast<void>(flux::enforce_quintic_ph_continuity(
        promotion, flux::PHJoinContinuity::curvature, 0.0L, 24U, token));
  });
  require(kernel.advance({1U, std::chrono::milliseconds{1000}}) ==
              flux::RoutingKernelStatus::yielded,
          "continuity promotion must yield after starting its private transaction");
  kernel.request_stop();
  static_cast<void>(kernel.advance());
  require(promotion.requested_continuity == original.requested_continuity &&
              promotion.control_points == original.control_points &&
              promotion.node_parameters == original.node_parameters &&
              promotion.node_derivatives == original.node_derivatives &&
              exact_length(promotion) == exact_length(original),
          "cancelling continuity promotion must preserve the complete caller checkpoint");
  return 0;
}
