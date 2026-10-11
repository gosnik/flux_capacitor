#include "flux/core/routing_avbd.hpp"

#include <cmath>
#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void require(bool value, const char* message) { if (!value) throw std::runtime_error(message); }

flux::RoutingXpbdConfig xpbd() {
  flux::RoutingXpbdConfig config; config.prefer_gpu = false; config.maximum_step_nm = 100'000;
  config.numerical_guard_nm = 10; return config;
}
flux::RoutingAvbdConfig avbd() {
  flux::RoutingAvbdConfig config; config.iterations = 8; config.maximum_contacts = 128;
  config.maximum_particles = 32; config.max_workers = 2; return config;
}
} // namespace

int main() {
  try {
    // Two crowded routes produce an augmented contact constraint; the bounded solve remains
    // finite and applies dual/penalty state instead of reporting a synthetic legal result.
    std::vector<flux::RoutingXpbdParticle> crowded{
        {-100, 0, 20, 0, 0, 0, flux::routing_xpbd_endpoint, -100, 0, 1000},
        {100, 0, 20, 0, 0, 0, flux::routing_xpbd_endpoint, 100, 0, 1000},
        {-100, 5, 20, 1, 1, 0, 0, -100, 5, 1000}, {100, 5, 20, 1, 1, 0, 0, 100, 5, 1000}};
    std::vector<flux::RoutingXpbdSegment> tracks{{0, 1, 0, 0, 20, 200, 10}, {2, 3, 1, 0, 20, 200, 10}};
    auto crowded_result = flux::relax_routing_avbd(crowded, tracks, {}, xpbd(), {}, avbd());
    require(crowded_result.ok() && crowded_result.contacts != 0 && crowded_result.duals != 0,
            "crowded AVBD contact did not use augmented duals");
    require(std::isfinite(static_cast<double>(crowded_result.relaxation.final.total_residual_nm)),
            "crowded AVBD result is not finite");
    if (!(crowded_result.relaxation.final.total_residual_nm < crowded_result.relaxation.initial.total_residual_nm)) {
      std::cerr << "Residual " << crowded_result.relaxation.initial.total_residual_nm << " -> "
                << crowded_result.relaxation.final.total_residual_nm << '\n';
      for (const auto& point : crowded_result.relaxation.positions_nm)
        std::cerr << point.first << ',' << point.second << ' ';
      std::cerr << '\n';
      for (const auto& metric : crowded_result.relaxation.iteration_metrics)
        std::cerr << metric.total_residual_nm << ' ';
      std::cerr << '\n';
    }
    require(crowded_result.relaxation.final.total_residual_nm < crowded_result.relaxation.initial.total_residual_nm,
            "crowded AVBD contacts did not reduce the measured residual");
    auto serial = avbd(); serial.max_workers = 1;
    const auto serial_result = flux::relax_routing_avbd(crowded, tracks, {}, xpbd(), {}, serial);
    require(serial_result.relaxation.positions_nm == crowded_result.relaxation.positions_nm,
            "independent color blocks differ between serial and parallel execution");

    // Exact duplicate obstacle records collapse, while the fixed endpoint remains anchored.
    const flux::RoutingXpbdObstacleSegment wall{-10, -50, -10, 50, 20, 0};
    auto obstacle_result = flux::relax_routing_avbd(crowded, tracks, {wall, wall}, xpbd(), {}, avbd());
    require(obstacle_result.duplicate_reductions != 0, "duplicate obstacle contact was retained");
    require(obstacle_result.relaxation.positions_nm[0] == std::pair<std::int64_t, std::int64_t>{-100, 0},
            "fixed anchor moved during AVBD");

    // A via's layer copies belong to one owner and must receive exactly one translation.
    auto via_particles = crowded;
    via_particles[2].flags = flux::routing_xpbd_via; via_particles[3].flags = flux::routing_xpbd_via;
    const auto via_result = flux::relax_routing_avbd(via_particles, tracks, {}, xpbd(), {{{2, 3}}}, avbd());
    const auto d0 = via_result.relaxation.positions_nm[2].first - via_particles[2].x_nm;
    const auto d1 = via_result.relaxation.positions_nm[3].first - via_particles[3].x_nm;
    const auto y0 = via_result.relaxation.positions_nm[2].second - via_particles[2].y_nm;
    const auto y1 = via_result.relaxation.positions_nm[3].second - via_particles[3].y_nm;
    require(d0 == d1 && y0 == y1, "rigid via copies did not share a translation");

    auto three_routes = crowded;
    three_routes.push_back({-100, 9, 20, 1, 2, 0, 0, -100, 9, 1000});
    three_routes.push_back({100, 9, 20, 1, 2, 0, 0, 100, 9, 1000});
    auto three_tracks = tracks; three_tracks.push_back({4, 5, 2, 0, 20, 200, 10});
    auto overflow = avbd(); overflow.maximum_contacts = 1;
    const auto bounded = flux::relax_routing_avbd(three_routes, three_tracks, {}, xpbd(), {}, overflow);
    require(!bounded.ok() && bounded.relaxation.diagnostic == "AVBD contact budget exhausted",
            "AVBD contact cap was not fail-closed");

    require(!flux::relax_routing_avbd(crowded, tracks, {}, xpbd(), {{{2, 2}}}, avbd()).ok(),
            "duplicate rigid members were accepted");
    require(!flux::relax_routing_avbd(crowded, tracks, {}, xpbd(), {{{2, 3}}, {{2}}}, avbd()).ok(),
            "overlapping rigid groups were accepted");

    // Only the middle layer intersects a through-via. All layer copies still translate together.
    std::vector<flux::RoutingXpbdParticle> span{
        {-1000, 0, 10, 0, 0, 1}, {1000, 0, 10, 0, 0, 1},
        {0, 5, 40, 1, 1, 0, flux::routing_xpbd_via, 0, 5, 1000, 20},
        {0, 5, 40, 1, 1, 1, flux::routing_xpbd_via, 0, 5, 1000, 20},
        {0, 5, 40, 1, 1, 2, flux::routing_xpbd_via, 0, 5, 1000, 20}};
    const std::vector<flux::RoutingXpbdSegment> span_tracks{{0, 1, 0, 1, 10, 2000, 10}};
    const std::vector<flux::RoutingAvbdRigidGroup> span_group{{{2, 3, 4}}};
    const auto span_result = flux::relax_routing_avbd(span, span_tracks, {}, xpbd(), span_group, avbd());
    require(span_result.ok() && span_result.relaxation.initial.violations == 1 &&
            span_result.relaxation.final.total_residual_nm < span_result.relaxation.initial.total_residual_nm,
            "intermediate-layer via contact was ignored");
    require(span_result.relaxation.positions_nm[2] == span_result.relaxation.positions_nm[3] &&
            span_result.relaxation.positions_nm[3] == span_result.relaxation.positions_nm[4] &&
            span_result.moved_rigid_owners == 1,
            "via span did not receive one shared rigid translation");
    auto pinned_span = span; pinned_span[4].inverse_mass = 0;
    const auto pinned = flux::relax_routing_avbd(pinned_span, span_tracks, {}, xpbd(), span_group, avbd());
    require(pinned.ok() && pinned.moved_rigid_owners == 0 && pinned.penalty_updates > 0,
            "fixed via copy did not pin the owner or stalled penalties did not adapt");
    for (std::size_t i = 2; i < span.size(); ++i)
      require(pinned.relaxation.positions_nm[i] == std::pair<std::int64_t, std::int64_t>{0, 5},
              "fixed attachment allowed another layer copy to move");
    auto narrow_span = span; narrow_span[3].corridor_radius_nm = 5;
    const auto bounded_span = flux::relax_routing_avbd(narrow_span, span_tracks, {}, xpbd(), span_group, avbd());
    require(bounded_span.ok() && std::hypot(static_cast<double>(bounded_span.relaxation.positions_nm[3].first),
            static_cast<double>(bounded_span.relaxation.positions_nm[3].second - 5)) <= 5.0,
            "shared rigid step left a member's corridor");

    // Two spatially separate interacting pairs create actual concurrent same-color work.
    auto independent = crowded; auto independent_tracks = tracks;
    for (auto particle : crowded) {
      particle.x_nm += 1'000'000; particle.corridor_anchor_x_nm += 1'000'000;
      particle.route += 2; independent.push_back(particle);
    }
    for (auto segment : tracks) {
      segment.first_particle += 4; segment.second_particle += 4; segment.route += 2;
      independent_tracks.push_back(segment);
    }
    const auto serial_pairs = flux::relax_routing_avbd(independent, independent_tracks, {}, xpbd(), {}, serial);
    const auto parallel_pairs = flux::relax_routing_avbd(independent, independent_tracks, {}, xpbd(), {}, avbd());
    require(serial_pairs.ok() && parallel_pairs.ok() &&
            serial_pairs.relaxation.positions_nm == parallel_pairs.relaxation.positions_nm,
            "concurrent independent blocks changed the deterministic solution");

    // Huge valid dimensions must not wrap a positive clearance into an absent contact.
    auto enormous_span = pinned_span;
    enormous_span[3].radius_nm = std::numeric_limits<std::int64_t>::max();
    enormous_span[3].clearance_nm = std::numeric_limits<std::int64_t>::max();
    auto enormous_config = xpbd();
    enormous_config.maximum_step_nm = std::numeric_limits<std::int64_t>::max();
    enormous_config.numerical_guard_nm = std::numeric_limits<std::int64_t>::max();
    const auto enormous = flux::relax_routing_avbd(enormous_span, span_tracks, {}, enormous_config, span_group, avbd());
    require(!enormous.ok() || (enormous.relaxation.initial.violations > 0 &&
            std::isfinite(enormous.relaxation.initial.total_residual_nm)),
            "extreme dimensions wrapped away a real contact");

    auto bad_guard = xpbd(); bad_guard.numerical_guard_nm = -1;
    require(!flux::relax_routing_avbd(crowded, tracks, {}, bad_guard, {}, avbd()).ok(),
            "negative guard was accepted");
    auto excessive_budget = avbd(); excessive_budget.maximum_contacts = std::numeric_limits<std::size_t>::max();
    require(!flux::relax_routing_avbd(crowded, tracks, {}, xpbd(), {}, excessive_budget).ok(),
            "overflowing contact budget was accepted");
    std::cout << "routing AVBD tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n'; return 1;
  }
}
