#include "scaffold_joint_displacement.hpp"

#include "flux/core/board_free_space.hpp"
#include "flux/core/connectivity.hpp"
#include "flux/core/routing_rules.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <utility>

namespace flux {
namespace {

struct SampledPath {
  GlobalRoutePath path;
  std::vector<Nanometres> widths;
  std::vector<GlobalRouteSegmentWidthKind> width_kinds;
  std::vector<std::size_t> particle_indices;
};

[[nodiscard]] bool cancelled(const std::stop_token stop) { return stop.stop_requested(); }

[[nodiscard]] bool valid_config(const ScaffoldJointDisplacementConfig& config) {
  return config.maximum_particles > 1U && config.maximum_iterations > 0U &&
         config.maximum_contacts > 0U && config.maximum_movement_nm > 0 &&
         config.maximum_particle_spacing_nm > 0 &&
         config.maximum_contacts <= std::numeric_limits<std::size_t>::max() / 16U;
}

[[nodiscard]] Nanometres edge_width(const Board& board, const GlobalRoutePath& path,
                                    const std::size_t edge) {
  const auto nominal = resolve_net_routing_rules(board, path.net).track_width;
  if (edge < path.segment_widths.size() && path.segment_widths[edge].value() > 0)
    return path.segment_widths[edge];
  return nominal;
}

[[nodiscard]] GlobalRouteSegmentWidthKind edge_kind(const GlobalRoutePath& path,
                                                     const std::size_t edge) {
  return edge < path.segment_width_kinds.size() ? path.segment_width_kinds[edge]
                                                : GlobalRouteSegmentWidthKind::nominal;
}

[[nodiscard]] std::int64_t rounded_coordinate(const long double value) {
  constexpr auto low = static_cast<long double>(std::numeric_limits<std::int64_t>::min());
  constexpr auto high = static_cast<long double>(std::numeric_limits<std::int64_t>::max());
  if (!std::isfinite(value) || value < low || value > high) return 0;
  return static_cast<std::int64_t>(std::llround(value));
}

// A perpendicular crossing has no usable pair-separation normal: the ordinary
// segment contact picks the incoming route's normal and merely makes the long
// owner slide along itself.  Seed each unpinned owner toward the nearer side of
// the incoming route's normal interval, then let the common XPBD/AVBD bundle
// settle all contacts.  This is a deterministic coupled deformation, not a
// per-owner reroute, and the corridor envelope still bounds every movement.
void seed_owner_escapes(std::vector<RoutingXpbdParticle>& particles,
                        const std::vector<SampledPath>& paths,
                        const Board& board,
                        const ScaffoldJointDisplacementConfig& config) {
  if (paths.size() < 2U || paths.front().particle_indices.size() < 2U) return;
  const auto incoming_rules = resolve_net_routing_rules(board, paths.front().path.net);
  for (std::size_t owner = 1U; owner < paths.size(); ++owner) {
    const auto& route = paths[owner];
    if (route.particle_indices.size() < 3U) continue;
    const auto first = route.particle_indices.front();
    const auto last = route.particle_indices.back();
    const auto dx = static_cast<long double>(particles[last].x_nm - particles[first].x_nm);
    const auto dy = static_cast<long double>(particles[last].y_nm - particles[first].y_nm);
    const auto length = std::hypotl(dx, dy);
    if (!(length > 1.0L)) continue;
    const auto normal_x = -dy / length;
    const auto normal_y = dx / length;
    long double incoming_min = std::numeric_limits<long double>::infinity();
    long double incoming_max = -std::numeric_limits<long double>::infinity();
    long double incoming_tangent_min = std::numeric_limits<long double>::infinity();
    long double incoming_tangent_max = -std::numeric_limits<long double>::infinity();
    const auto tangent_x = dx / length;
    const auto tangent_y = dy / length;
    for (const auto index : paths.front().particle_indices) {
      const auto projection = normal_x * particles[index].x_nm + normal_y * particles[index].y_nm;
      incoming_min = std::min(incoming_min, projection);
      incoming_max = std::max(incoming_max, projection);
      const auto tangent_projection = tangent_x * particles[index].x_nm + tangent_y * particles[index].y_nm;
      incoming_tangent_min = std::min(incoming_tangent_min, tangent_projection);
      incoming_tangent_max = std::max(incoming_tangent_max, tangent_projection);
    }
    long double owner_projection = 0.0L;
    for (const auto index : route.particle_indices)
      owner_projection += normal_x * particles[index].x_nm + normal_y * particles[index].y_nm;
    owner_projection /= static_cast<long double>(route.particle_indices.size());
    const auto rules = resolve_net_routing_rules(board, route.path.net);
    const auto required = static_cast<long double>(incoming_rules.track_width.value() +
                                                    rules.track_width.value()) / 2.0L +
        static_cast<long double>(std::max(incoming_rules.clearance.value(), rules.clearance.value()) +
                                 std::max<std::int64_t>(0, config.xpbd.numerical_guard_nm));
    const auto below = incoming_min - required;
    const auto above = incoming_max + required;
    if (owner_projection <= below || owner_projection >= above) continue;
    const auto target = std::abs(owner_projection - below) <= std::abs(owner_projection - above)
        ? below : above;
    auto displacement = target - owner_projection;
    displacement = std::clamp(displacement,
                              -static_cast<long double>(config.maximum_movement_nm),
                              static_cast<long double>(config.maximum_movement_nm));
    // Keep the deformation local to the crossing.  The full translation is
    // applied across the incoming route's tangent span plus the clearance
    // envelope, then fades to zero over at most twice that displacement.
    // This preserves remote owner geometry and fixed-scene attachments.
    const auto inner_min = incoming_tangent_min - required;
    const auto inner_max = incoming_tangent_max + required;
    const auto run_up = std::max(1.0L, 2.0L * std::abs(displacement));
    for (const auto index : route.particle_indices) {
      if (!(particles[index].inverse_mass > 0.0L)) continue;
      const auto tangent_projection = tangent_x * particles[index].corridor_anchor_x_nm +
                                      tangent_y * particles[index].corridor_anchor_y_nm;
      const auto outside = tangent_projection < inner_min ? inner_min - tangent_projection :
                           tangent_projection > inner_max ? tangent_projection - inner_max : 0.0L;
      const auto weight = std::clamp(1.0L - outside / run_up, 0.0L, 1.0L);
      if (!(weight > 0.0L)) continue;
      particles[index].x_nm = rounded_coordinate(static_cast<long double>(particles[index].x_nm) +
                                                  normal_x * displacement * weight);
      particles[index].y_nm = rounded_coordinate(static_cast<long double>(particles[index].y_nm) +
                                                  normal_y * displacement * weight);
    }
  }
}

[[nodiscard]] std::optional<SampledPath> sample_path(const Board& board,
                                                      const GlobalRoutePath& original,
                                                      const ScaffoldJointDisplacementConfig& config,
                                                      std::size_t& remaining_particles, bool& budget_exhausted) {
  if (original.points.size() < 2U) return std::nullopt;
  SampledPath sampled;
  sampled.path = original;
  sampled.path.points.clear();
  // Resampling preserves the selected planar order but invalidates every
  // point-aligned coarse certificate.  Do not let a later stage mistake the
  // diagnostic polyline for its original NavMesh proof.
  sampled.path.medial_resource_sequence.clear();
  sampled.path.medial_transition_portals.clear();
  sampled.path.medial_transition_grid_edges.clear();
  sampled.path.medial_topology_breaks = 0U;
  sampled.path.medial_direct_portal_transitions = 0U;
  sampled.path.medial_unmapped_nodes = 0U;
  sampled.path.medial_interior_unmapped_nodes = 0U;
  sampled.path.direct_medial_edges = 0U;
  sampled.path.direct_medial_grid_steps_bypassed = 0U;
  sampled.path.segment_widths.clear();
  sampled.path.segment_width_kinds.clear();
  sampled.path.geometric_corridors.clear();
  sampled.path.terminal_neck_down_segments = 0U;
  sampled.path.terminal_neck_down_length_nm = 0U;
  sampled.path.medial_neck_down_segments = 0U;
  sampled.path.medial_neck_down_length_nm = 0U;
  sampled.path.corridor_neck_down_segments = 0U;
  sampled.path.corridor_neck_down_length_nm = 0U;
  sampled.path.legal_scaffold = false;
  const auto append = [&](const GlobalRoutePoint point, const Nanometres width,
                          const GlobalRouteSegmentWidthKind kind) -> bool {
    if (sampled.path.points.size() >= remaining_particles) { budget_exhausted = true; return false; }
    sampled.path.points.push_back(point);
    sampled.widths.push_back(width);
    sampled.width_kinds.push_back(kind);
    return true;
  };
  for (std::size_t edge = 0U; edge + 1U < original.points.size(); ++edge) {
    const auto& first = original.points[edge];
    const auto& second = original.points[edge + 1U];
    // This adapter owns a planar retained-passage experiment.  A via needs a
    // multi-layer rigid representation and must stay with the regional solver.
    if (first.layer != second.layer) return std::nullopt;
    const auto width = edge_width(board, original, edge);
    const auto kind = edge_kind(original, edge);
    if (width.value() <= 0) return std::nullopt;
    const auto dx = static_cast<long double>(second.position.x.value() - first.position.x.value());
    const auto dy = static_cast<long double>(second.position.y.value() - first.position.y.value());
    const auto distance = std::hypotl(dx, dy);
    const auto portions = std::max<std::size_t>(1U, static_cast<std::size_t>(std::ceil(
        distance / static_cast<long double>(config.maximum_particle_spacing_nm))));
    if (edge == 0U && !append(first, width, kind)) return std::nullopt;
    for (std::size_t part = 1U; part <= portions; ++part) {
      const auto fraction = static_cast<long double>(part) / static_cast<long double>(portions);
      const auto point = GlobalRoutePoint{{Nanometres{rounded_coordinate(
                                      static_cast<long double>(first.position.x.value()) + dx * fraction)},
                                           Nanometres{rounded_coordinate(
                                      static_cast<long double>(first.position.y.value()) + dy * fraction)}},
                                          first.layer};
      if (!append(point, width, kind)) return std::nullopt;
    }
  }
  if (sampled.path.points.size() < 2U) return std::nullopt;
  for (std::size_t index = 1U; index < sampled.widths.size(); ++index) {
    sampled.path.segment_widths.push_back(sampled.widths[index]);
    sampled.path.segment_width_kinds.push_back(sampled.width_kinds[index]);
  }
  remaining_particles -= sampled.path.points.size();
  return sampled;
}

[[nodiscard]] bool add_polygon_obstacles(const Polygon& polygon, const std::uint32_t layer,
                                         const std::uint32_t route,
                                         std::vector<RoutingXpbdObstacleSegment>& obstacles,
                                         const std::size_t maximum_segments,
                                         const bool forbidden_inside = true,
                                         const std::int64_t shift_toward_forbidden_nm = 0) {
  if (polygon.vertices.size() < 2U) return true;
  if (obstacles.size() > maximum_segments ||
      polygon.vertices.size() > maximum_segments - obstacles.size()) return false;
  long double twice_area = 0.0L;
  for (std::size_t index = 0U; index < polygon.vertices.size(); ++index) {
    const auto& first = polygon.vertices[index];
    const auto& second = polygon.vertices[(index + 1U) % polygon.vertices.size()];
    twice_area += static_cast<long double>(first.x.value()) * second.y.value() -
                  static_cast<long double>(second.x.value()) * first.y.value();
  }
  const auto counter_clockwise = twice_area > 0.0L;
  for (std::size_t index = 0U; index < polygon.vertices.size(); ++index) {
    const auto& first = polygon.vertices[index];
    const auto& second = polygon.vertices[(index + 1U) % polygon.vertices.size()];
    const auto dx = second.x.value() - first.x.value();
    const auto dy = second.y.value() - first.y.value();
    auto outward_x = counter_clockwise ? dy : -dy;
    auto outward_y = counter_clockwise ? -dx : dx;
    if (!forbidden_inside) { outward_x = -outward_x; outward_y = -outward_y; }
    const auto normal_length = std::hypotl(static_cast<long double>(outward_x),
                                           static_cast<long double>(outward_y));
    const auto shift_x = normal_length > 0.0L
        ? rounded_coordinate(-static_cast<long double>(outward_x) * shift_toward_forbidden_nm / normal_length) : 0;
    const auto shift_y = normal_length > 0.0L
        ? rounded_coordinate(-static_cast<long double>(outward_y) * shift_toward_forbidden_nm / normal_length) : 0;
    obstacles.push_back({first.x.value() + shift_x, first.y.value() + shift_y,
                         second.x.value() + shift_x, second.y.value() + shift_y, 0,
                         layer, route, outward_x, outward_y});
  }
  return true;
}

} // namespace

ScaffoldJointDisplacementResult solve_scaffold_joint_displacement(
    const Board& scene, const std::vector<GlobalRoutePath>& paths,
    const ScaffoldJointDisplacementConfig& input_config, const std::stop_token stop_token) {
  ScaffoldJointDisplacementResult result;
  result.residual_pns_requested = input_config.enable_residual_pns;
  if (!valid_config(input_config)) { result.diagnostic = "invalid joint displacement bounds"; return result; }
  if (paths.size() > 32U) { result.diagnostic = "joint owner budget exceeded"; return result; }
  if (paths.size() < 2U) { result.diagnostic = "joint displacement requires incoming path and an owner"; return result; }
  if (cancelled(stop_token)) { result.cancelled = true; result.diagnostic = "joint displacement cancelled"; return result; }

  std::size_t remaining = input_config.maximum_particles;
  std::vector<SampledPath> sampled;
  sampled.reserve(paths.size());
  for (const auto& path : paths) {
    if (cancelled(stop_token)) { result.cancelled = true; result.diagnostic = "joint displacement cancelled"; return result; }
    auto candidate = sample_path(scene, path, input_config, remaining, result.budget_exhausted);
    if (!candidate) {
      result.diagnostic = "path is non-planar, invalid, or exceeds particle budget"; return result; }
    sampled.push_back(std::move(*candidate));
  }

  std::map<LayerId, std::uint32_t> layer_codes;
  for (const auto& path : sampled) for (const auto& point : path.path.points)
    layer_codes.try_emplace(point.layer, static_cast<std::uint32_t>(layer_codes.size()));
  std::vector<RoutingXpbdParticle> particles;
  std::vector<RoutingXpbdSegment> segments;
  particles.reserve(input_config.maximum_particles);
  for (std::size_t route = 0U; route < sampled.size(); ++route) {
    auto& path = sampled[route];
    const auto rules = resolve_net_routing_rules(scene, path.path.net);
    const auto radius = rules.track_width.value() / 2;
    std::vector<CopperConnectivityProbe> probes;
    probes.reserve(path.path.points.size());
    for (const auto& point : path.path.points)
      probes.push_back({point.position, {point.layer}});
    const auto memberships = copper_probe_component_memberships(scene, path.path.net, probes);
    if (memberships.size() != path.path.points.size()) {
      result.diagnostic = "unable to classify joint attachment points";
      return result;
    }
    for (std::size_t index = 0U; index < path.path.points.size(); ++index) {
      const auto& point = path.path.points[index];
      const bool pin = index == 0U || index + 1U == path.path.points.size() ||
                       memberships[index].contained_by_copper;
      const auto particle_index = particles.size();
      path.particle_indices.push_back(particle_index);
      particles.push_back({point.position.x.value(), point.position.y.value(), radius,
                           pin ? 0.0L : 1.0L, static_cast<std::uint32_t>(route),
                           layer_codes.at(point.layer),
                           pin ? routing_xpbd_endpoint : 0U,
                           point.position.x.value(), point.position.y.value(),
                           input_config.maximum_movement_nm, rules.clearance.value()});
      if (pin) ++result.pinned_particles;
      if (index == 0U) continue;
      const auto& previous = path.path.points[index - 1U];
      const auto dx = static_cast<long double>(point.position.x.value() - previous.position.x.value());
      const auto dy = static_cast<long double>(point.position.y.value() - previous.position.y.value());
      const auto width = path.widths[index];
      segments.push_back({static_cast<std::uint32_t>(particle_index - 1U),
                          static_cast<std::uint32_t>(particle_index), static_cast<std::uint32_t>(route),
                          layer_codes.at(point.layer), width.value() / 2,
                          rounded_coordinate(std::hypotl(dx, dy)), rules.clearance.value()});
    }
  }
  if (particles.size() > input_config.maximum_particles || segments.empty()) {
    result.diagnostic = "joint displacement has no bounded planar segments"; return result;
  }
  result.particles = particles.size(); result.segments = segments.size();
  seed_owner_escapes(particles, sampled, scene, input_config);

  std::vector<RoutingXpbdObstacleSegment> obstacles;
  const auto maximum_obstacle_segments = input_config.maximum_contacts * 16U;
  for (std::size_t route = 0U; route < sampled.size(); ++route) {
    if (cancelled(stop_token)) { result.cancelled = true; result.diagnostic = "joint displacement cancelled"; return result; }
    std::set<LayerId> layers;
    for (const auto& point : sampled[route].path.points) layers.insert(point.layer);
    for (const auto& layer : layers) {
      const auto extracted = extract_polygonal_board_free_space(
          scene, {layer, sampled[route].path.net, true, true, true});
      if (!extracted.complete()) { result.diagnostic = "fixed-scene obstacle extraction incomplete"; return result; }
      const auto code = layer_codes.at(layer);
      // XPBD's obstacle rule adds segment clearance. Shift the board-edge
      // constraint into forbidden material by its excess over the actual
      // copper-to-edge rule, so it enforces radius + edge clearance rather
      // than radius + generic net clearance.
      const auto edge_shift = resolve_net_routing_rules(
          scene, sampled[route].path.net).clearance.value() -
          scene.rules.copper_to_edge_clearance.value();
      if (!add_polygon_obstacles(extracted.domain.outline, code, static_cast<std::uint32_t>(route), obstacles,
                                 maximum_obstacle_segments, false, edge_shift)) {
        result.budget_exhausted = true;
        result.diagnostic = "joint displacement obstacle contact budget exhausted"; return result;
      }
      for (const auto& cutout : extracted.domain.cutouts)
        if (!add_polygon_obstacles(cutout, code, static_cast<std::uint32_t>(route), obstacles,
                                   maximum_obstacle_segments, true, edge_shift)) {
          result.budget_exhausted = true; result.diagnostic = "joint displacement obstacle contact budget exhausted"; return result;
        }
      for (const auto& obstacle : extracted.domain.obstacles)
        if (!add_polygon_obstacles(obstacle, code, static_cast<std::uint32_t>(route), obstacles, maximum_obstacle_segments)) {
          result.budget_exhausted = true; result.diagnostic = "joint displacement obstacle contact budget exhausted"; return result;
        }
      for (const auto& filled : extracted.domain.filled_obstacles) {
        if (!add_polygon_obstacles(filled.area, code, static_cast<std::uint32_t>(route), obstacles, maximum_obstacle_segments)) {
          result.budget_exhausted = true; result.diagnostic = "joint displacement obstacle contact budget exhausted"; return result;
        }
        for (const auto& hole : filled.holes)
          if (!add_polygon_obstacles(hole, code, static_cast<std::uint32_t>(route), obstacles, maximum_obstacle_segments, false)) {
            result.budget_exhausted = true; result.diagnostic = "joint displacement obstacle contact budget exhausted"; return result;
          }
      }
    }
  }
  // XPBD evaluates every retained obstacle edge against every matching route/layer
  // segment on every iteration.  The raw edge cap alone therefore does not bound
  // work.  Cull edges that cannot be reached from immutable anchors inside the
  // maximum deformation envelope, then charge each survivor its *full* matching
  // segment count (XPBD has no obstacle broad phase).
  struct ExpandedSegmentBounds {
    std::int64_t minimum_x{0};
    std::int64_t minimum_y{0};
    std::int64_t maximum_x{0};
    std::int64_t maximum_y{0};
  };
  std::vector<ExpandedSegmentBounds> segment_bounds;
  segment_bounds.reserve(segments.size());
  const auto saturating_add = [](const std::int64_t value, const std::int64_t amount) {
    return amount > 0 && value > std::numeric_limits<std::int64_t>::max() - amount
        ? std::numeric_limits<std::int64_t>::max() : value + amount;
  };
  const auto saturating_subtract = [](const std::int64_t value, const std::int64_t amount) {
    return amount > 0 && value < std::numeric_limits<std::int64_t>::min() + amount
        ? std::numeric_limits<std::int64_t>::min() : value - amount;
  };
  for (const auto& segment : segments) {
    const auto& first = particles[segment.first_particle];
    const auto& second = particles[segment.second_particle];
    auto expansion = std::max<std::int64_t>(0, input_config.maximum_movement_nm);
    for (const auto term : {std::max<std::int64_t>(0, segment.radius_nm),
                            std::max<std::int64_t>(0, segment.clearance_nm),
                            std::max<std::int64_t>(0, input_config.xpbd.numerical_guard_nm)})
      expansion = saturating_add(expansion, term);
    segment_bounds.push_back({
        saturating_subtract(std::min(first.corridor_anchor_x_nm, second.corridor_anchor_x_nm), expansion),
        saturating_subtract(std::min(first.corridor_anchor_y_nm, second.corridor_anchor_y_nm), expansion),
        saturating_add(std::max(first.corridor_anchor_x_nm, second.corridor_anchor_x_nm), expansion),
        saturating_add(std::max(first.corridor_anchor_y_nm, second.corridor_anchor_y_nm), expansion)});
  }
  const auto overlaps = [](const ExpandedSegmentBounds& bounds,
                           const RoutingXpbdObstacleSegment& obstacle) {
    const auto minimum_x = std::min(obstacle.first_x_nm, obstacle.second_x_nm);
    const auto minimum_y = std::min(obstacle.first_y_nm, obstacle.second_y_nm);
    const auto maximum_x = std::max(obstacle.first_x_nm, obstacle.second_x_nm);
    const auto maximum_y = std::max(obstacle.first_y_nm, obstacle.second_y_nm);
    return bounds.minimum_x <= maximum_x && minimum_x <= bounds.maximum_x &&
           bounds.minimum_y <= maximum_y && minimum_y <= bounds.maximum_y;
  };
  std::vector<RoutingXpbdObstacleSegment> culled_obstacles;
  culled_obstacles.reserve(obstacles.size());
  std::size_t obstacle_work = 0U;
  for (const auto& obstacle : obstacles) {
    std::size_t matching_segments = 0U;
    bool relevant = false;
    for (std::size_t index = 0U; index < segments.size(); ++index) {
      const auto& segment = segments[index];
      if (segment.layer != obstacle.layer ||
          (obstacle.route != std::numeric_limits<std::uint32_t>::max() &&
           segment.route != obstacle.route)) continue;
      ++matching_segments;
      relevant = relevant || overlaps(segment_bounds[index], obstacle);
    }
    if (!relevant) continue;
    if (matching_segments > input_config.maximum_contacts - obstacle_work) {
      result.budget_exhausted = true;
      result.diagnostic = "joint XPBD obstacle work budget exhausted";
      return result;
    }
    obstacle_work += matching_segments;
    culled_obstacles.push_back(obstacle);
  }
  obstacles = std::move(culled_obstacles);
  result.obstacle_segments = obstacles.size();
  auto xpbd_config = input_config.xpbd;
  xpbd_config.iterations = input_config.maximum_iterations;
  xpbd_config.maximum_step_nm = std::min(xpbd_config.maximum_step_nm, input_config.maximum_movement_nm);
  xpbd_config.maximum_candidate_pairs = input_config.maximum_contacts;
  xpbd_config.prefer_gpu = false;
  xpbd_config.record_iteration_metrics = true;
  std::vector<std::pair<std::int64_t, std::int64_t>> seeded_positions;
  seeded_positions.reserve(particles.size());
  for (const auto& particle : particles)
    seeded_positions.emplace_back(particle.x_nm, particle.y_nm);
  const auto xpbd = relax_routing_xpbd(particles, segments, obstacles, {}, xpbd_config);
  result.xpbd_iterations = xpbd.iterations_executed;
  result.xpbd_candidate_pairs = xpbd.candidate_pairs;
  result.xpbd_unresolved_trace_pairs = xpbd.unresolved_trace_pairs;
  if (xpbd.pair_budget_exhausted) { result.budget_exhausted = true;
    result.diagnostic = "joint XPBD contact budget exhausted"; return result; }
  if (!xpbd.ok() || xpbd.positions_nm.size() != particles.size()) {
    result.diagnostic = "XPBD failed: " + xpbd.diagnostic; return result;
  }
  if (cancelled(stop_token)) { result.cancelled = true; result.diagnostic = "joint displacement cancelled"; return result; }

  auto avbd_particles = particles;
  for (std::size_t index = 0U; index < avbd_particles.size(); ++index) {
    avbd_particles[index].x_nm = xpbd.positions_nm[index].first;
    avbd_particles[index].y_nm = xpbd.positions_nm[index].second;
  }
  auto avbd_config = input_config.avbd;
  avbd_config.iterations = std::min(avbd_config.iterations, input_config.maximum_iterations);
  avbd_config.maximum_particles = input_config.maximum_particles;
  avbd_config.maximum_contacts = input_config.maximum_contacts;
  avbd_config.max_workers = 1U;
  const auto avbd = relax_routing_avbd(avbd_particles, segments, obstacles, xpbd_config, {}, avbd_config, stop_token);
  result.used_avbd = true;
  result.avbd_contacts = avbd.contacts;
  result.avbd_residual_contacts = avbd.relaxation.final.violations;
  if (avbd.cancelled || cancelled(stop_token)) { result.cancelled = true; result.diagnostic = "joint displacement cancelled"; return result; }
  // XPBD remains a useful coupled proposal if AVBD rejects its own bounded
  // refinement.  The parent can still send that proposal through PNS and the
  // exact atomic gate; the diagnostic keeps AVBD's failure observable.
  const bool avbd_usable = avbd.ok() && avbd.relaxation.positions_nm.size() == particles.size();
  const auto* proposal_positions = &xpbd.positions_nm;
  auto best_residual = xpbd.final.total_residual_nm;
  if (xpbd.initial.total_residual_nm < best_residual) {
    proposal_positions = &seeded_positions;
    best_residual = xpbd.initial.total_residual_nm;
  }
  if (avbd_usable && avbd.relaxation.final.total_residual_nm < best_residual) {
    proposal_positions = &avbd.relaxation.positions_nm;
    best_residual = avbd.relaxation.final.total_residual_nm;
  }
  std::vector<GlobalRoutePath> repaired;
  repaired.reserve(sampled.size());
  for (auto& path : sampled) {
    for (std::size_t index = 0U; index < path.path.points.size(); ++index) {
      const auto particle = path.particle_indices[index];
      const auto dx = static_cast<long double>(proposal_positions->at(particle).first) -
                      static_cast<long double>(particles[particle].corridor_anchor_x_nm);
      const auto dy = static_cast<long double>(proposal_positions->at(particle).second) -
                      static_cast<long double>(particles[particle].corridor_anchor_y_nm);
      if (std::hypotl(dx, dy) > static_cast<long double>(input_config.maximum_movement_nm)) {
        result.diagnostic = "joint displacement exceeded topology deformation envelope";
        return result;
      }
      path.path.points[index].position = {Nanometres{proposal_positions->at(particle).first},
                                          Nanometres{proposal_positions->at(particle).second}};
    }
    repaired.push_back(std::move(path.path));
  }
  result.paths = std::move(repaired);
  if (!avbd_usable)
    result.refinement_diagnostic = "AVBD failed; returning XPBD proposal: " + avbd.relaxation.diagnostic;
  return result;
}

} // namespace flux
