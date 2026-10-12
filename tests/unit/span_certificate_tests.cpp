#include "flux/core/spline_route.hpp"

#include <cmath>
#include <stdexcept>

namespace {

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

[[nodiscard]] flux::QuinticPHRoute multi_span_route() {
  flux::QuinticPHRoute route;
  route.control_points = {{0.0L, 0.0L}, {50'000.0L, 4'000.0L},
                          {100'000.0L, -4'000.0L}, {150'000.0L, 8'000.0L},
                          {200'000.0L, -8'000.0L}, {250'000.0L, 4'000.0L},
                          {300'000.0L, 0.0L}};
  route.corridor_anchors = route.control_points;
  require(flux::rebuild_quintic_ph_route(route), "multi-span PH fixture must rebuild");
  require(route.segments.size() >= 6U, "fixture must retain independent PH spans");
  return route;
}

[[nodiscard]] flux::SplineClearanceEvaluator clear_space() {
  return [](flux::SplineVector) {
    return flux::SplineClearanceProbe{1'000'000.0L, {}, true};
  };
}

[[nodiscard]] flux::SplineTrajectoryOptimizationConfig base_config() {
  flux::SplineTrajectoryOptimizationConfig config;
  config.minimum_clearance = flux::Nanometres{0};
  config.maximum_iterations = 1U;
  config.maximum_candidate_checks = 32U;
  config.maximum_adaptive_control_insertions = 0U;
  config.enable_paired_obstacle_supports = false;
  // Every generated local proposal is geometrically identical, so the optimizer must prune it
  // before exact PH rebuilding. The immutable spans still exercise the certificate cache.
  config.control_step = flux::Nanometres{0};
  return config;
}

} // namespace

int main() {
  const auto clearance = clear_space();
  const auto config = base_config();

  auto cached_route = multi_span_route();
  const auto cached = flux::optimize_quintic_ph_trajectory(cached_route, config, clearance);
  auto uncached_route = multi_span_route();
  auto uncached_config = config;
  uncached_config.reuse_span_certificates = false;
  const auto uncached = flux::optimize_quintic_ph_trajectory(
      uncached_route, uncached_config, clearance);
  require(cached.legal && uncached.legal && cached_route.control_points == uncached_route.control_points &&
              cached_route.segments == uncached_route.segments,
          "span-certificate reuse must preserve deterministic legal multi-span geometry");
  require(cached.span_certificate_cache_hits > 0U && cached.span_certificate_cache_misses > 0U &&
              uncached.span_certificate_cache_hits == 0U && uncached.span_certificate_cache_misses == 0U,
          "enabled reuse must hit immutable span certificates while disabled reuse remains uncached");
  require(cached.equivalent_candidates_skipped > 0U &&
              cached.candidate_checks <= config.maximum_candidate_checks,
          "identical local proposals must be pruned before spending the bounded exact candidate budget");

  auto moving_config = base_config();
  moving_config.control_step = flux::Nanometres{1'000};
  moving_config.maximum_candidate_checks = 128U;
  const auto preserve_span_count = [](const flux::QuinticPHRoute& candidate) {
    return candidate.control_points.size() == 7U;
  };
  auto moving_cached_route = multi_span_route();
  const auto moving_cached = flux::optimize_quintic_ph_trajectory(
      moving_cached_route, moving_config, clearance, preserve_span_count);
  auto moving_uncached_route = multi_span_route();
  auto moving_uncached_config = moving_config;
  moving_uncached_config.reuse_span_certificates = false;
  const auto moving_uncached = flux::optimize_quintic_ph_trajectory(
      moving_uncached_route, moving_uncached_config, clearance, preserve_span_count);
  require(moving_cached.legal && moving_uncached.legal &&
              moving_cached_route.control_points == moving_uncached_route.control_points &&
              moving_cached_route.segments == moving_uncached_route.segments &&
              moving_cached.span_certificate_cache_hits > 0U &&
              moving_cached.non_improving_candidates_skipped > 0U,
          "local nonzero control edits must reuse unchanged neighbor spans without changing legal output");

  const auto now_blocked = [](flux::SplineVector point) {
    const auto dx = point.x - 150'000.0L;
    const auto dy = point.y;
    const auto distance = std::hypotl(dx, dy);
    const auto clearance_nm = distance - 35'000.0L;
    const auto normal = distance > 0.0L ? flux::SplineVector{dx / distance, dy / distance}
                                        : flux::SplineVector{};
    return flux::SplineClearanceProbe{clearance_nm, normal, clearance_nm >= 0.0L};
  };
  auto changed_clearance_route = multi_span_route();
  auto changed_clearance_config = config;
  changed_clearance_config.maximum_iterations = 0U;
  const auto changed_clearance = flux::optimize_quintic_ph_trajectory(
      changed_clearance_route, changed_clearance_config, now_blocked);
  require(!changed_clearance.legal &&
              changed_clearance.span_certificate_cache_misses >= changed_clearance_route.segments.size(),
          "a separate invocation with the same geometry but a blocked field must reject rather than reuse evidence");

  auto tiny_cached_route = multi_span_route();
  auto tiny_config = config;
  tiny_config.maximum_iterations = 0U;
  tiny_config.continuous_envelope.maximum_hulls = 1U;
  const auto tiny_cached = flux::optimize_quintic_ph_trajectory(tiny_cached_route, tiny_config, clearance);
  auto tiny_uncached_route = multi_span_route();
  auto tiny_uncached_config = tiny_config;
  tiny_uncached_config.reuse_span_certificates = false;
  const auto tiny_uncached = flux::optimize_quintic_ph_trajectory(
      tiny_uncached_route, tiny_uncached_config, clearance);
  require(!tiny_cached.legal && !tiny_uncached.legal && tiny_cached.continuous_budget_exhausted &&
              tiny_uncached.continuous_budget_exhausted &&
              tiny_cached_route.control_points == tiny_uncached_route.control_points,
          "a tiny whole-route hull allowance must remain fail-closed with or without cached spans");
  return 0;
}
