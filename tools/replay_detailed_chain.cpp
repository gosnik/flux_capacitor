// Replay one recorded failed chain against immutable surrounding candidate copper.
// This is diagnostic input, never an authoritative board export or qualification path.
#include "flux/core/detailed_router.hpp"
#include "flux/core/board_free_space.hpp"
#include "flux/core/free_space_offset.hpp"
#include "flux/core/geometry.hpp"
#include "flux/core/routing_rules.hpp"
#include "flux/core/spline_route.hpp"
#include "flux/io/kicad_import.hpp"
#include "flux/telemetry/event.hpp"
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <fstream>
#include <cmath>
#include <iostream>
#include <set>
#include <stdexcept>

// Sampled evidence for locating the representation that first loses a legal source
// chain. These probes deliberately do not replace the production continuous certificate.
boost::property_tree::ptree probe_chain_seeds(const flux::Board& board,
                                             const flux::GlobalRoutePath& path,
                                             flux::Nanometres width) {
  using namespace flux;
  boost::property_tree::ptree output;
  const auto extracted = extract_polygonal_board_free_space(
      board, {path.points.front().layer, path.net, true, true, true});
  if (!extracted.complete()) {
    output.put("diagnostic", extracted.diagnostic);
    return output;
  }
  const auto radius = (width.value() + 1) / 2;
  FreeSpaceOffsetConfig offset_config{
      Nanometres{radius + board.rules.copper_to_edge_clearance.value()}, 1000000U};
  for (const auto& owner : extracted.obstacle_nets)
    offset_config.obstacle_clearances.push_back(Nanometres{
        radius + (owner ? resolve_pairwise_clearance(board, path.net, *owner).value() : 0)});
  for (const auto& owner : extracted.filled_obstacle_nets)
    offset_config.filled_obstacle_clearances.push_back(Nanometres{
        radius + (owner ? resolve_pairwise_clearance(board, path.net, *owner).value() : 0)});
  const auto offset = offset_polygonal_free_space(extracted.domain, offset_config);
  output.put("offset_diagnostic", offset.diagnostic);
  if (!offset.ok() || offset.regions.empty())
    return output;
  struct Region {
    geometry::PreparedPolygon outline;
    std::vector<geometry::PreparedPolygon> holes;
  };
  std::vector<Region> regions;
  for (const auto& region : offset.regions) {
    Region prepared{geometry::prepare_polygon(region.area), {}};
    for (const auto& hole : region.holes)
      prepared.holes.push_back(geometry::prepare_polygon(hole));
    regions.push_back(std::move(prepared));
  }
  const auto query = [&](Point point) {
    auto best = -std::numeric_limits<long double>::infinity();
    for (const auto& region : regions) {
      bool inside = geometry::locate_point(region.outline, point) != geometry::PointLocation::outside;
      auto distance = geometry::point_polygon_boundary_distance_nm(point, region.outline);
      for (const auto& hole : region.holes) {
        distance = std::min(distance, geometry::point_polygon_boundary_distance_nm(point, hole));
        if (geometry::locate_point(hole, point) != geometry::PointLocation::outside)
          inside = false;
      }
      best = std::max(best, inside ? distance : -distance);
    }
    return best;
  };
  std::vector<Point> points;
  boost::property_tree::ptree source_points;
  for (const auto& node : path.points) {
    points.push_back(node.position);
    boost::property_tree::ptree item;
    item.put("x_nm", node.position.x.value());
    item.put("y_nm", node.position.y.value());
    item.put("signed_clearance_nm", query(node.position));
    source_points.push_back({"", std::move(item)});
  }
  output.add_child("source_points", source_points);
  boost::property_tree::ptree seeds;
  for (const auto guard : {125000LL, 250000LL, 500000LL, 1000000LL}) {
    for (const auto aligned : {false, true}) {
    boost::property_tree::ptree item;
    item.put("guard_nm", guard);
    item.put("run_aligned", aligned);
    const auto route = aligned ? make_run_aligned_quintic_ph_route(points, Nanometres{guard})
                               : make_corner_rounded_quintic_ph_route(points, Nanometres{guard});
    item.put("available", route.has_value());
    if (route) {
      SplineShapeCertificationConfig shape_config;
      shape_config.minimum_bend_radius = Nanometres{250000};
      const auto shape = certify_quintic_ph_shape(*route, shape_config);
      item.put("shape_violations", shape.violations());
      std::size_t violations = 0U;
      long double worst = std::numeric_limits<long double>::infinity();
      Point worst_point;
      for (const auto& segment : route->segments) {
        for (unsigned sample = 0U; sample <= 64U; ++sample) {
          const auto t = segment.route_parameter_start +
              (segment.route_parameter_end - segment.route_parameter_start) * sample / 64.0L;
          const auto position = evaluate_quintic_ph(*route, t);
          const Point point{Nanometres{static_cast<std::int64_t>(std::llround(position.x))},
                            Nanometres{static_cast<std::int64_t>(std::llround(position.y))}};
          const auto distance = query(point);
          violations += distance < 0.0L ? 1U : 0U;
          if (distance < worst) {
            worst = distance;
            worst_point = point;
          }
        }
      }
      item.put("sampled_outside", violations);
      item.put("worst_signed_clearance_nm", worst);
      item.put("worst_x_nm", worst_point.x.value());
      item.put("worst_y_nm", worst_point.y.value());
    }
    seeds.push_back({"", std::move(item)});
    }
  }
  output.add_child("source_corner_seeds", seeds);
  return output;
}

int main(int argc, char** argv) {
  try {
    if (argc != 4)
      throw std::runtime_error(
          "usage: flux-replay-detailed-chain SOURCE.kicad_pcb TELEMETRY.jsonl CHAIN.json");
    auto imported = flux::io::import_kicad_pcb_file(argv[1]);
    if (!imported.strict())
      throw std::runtime_error("source board must import strictly");
    auto source = std::move(*imported.board);
    boost::property_tree::ptree chain;
    boost::property_tree::read_json(argv[3], chain);
    std::set<std::string> removed;
    for (const auto& item : chain.get_child("track_ids"))
      removed.insert(item.second.get_value<std::string>());
    if (removed.empty())
      throw std::runtime_error("chain needs exact recorded track IDs");
    std::ifstream stream(argv[2]);
    if (!stream)
      throw std::runtime_error("cannot read telemetry");
    std::shared_ptr<const flux::telemetry::SourceBoard> generated;
    std::string line;
    while (std::getline(stream, line)) {
      const auto decoded = flux::telemetry::decode_events(line);
      if (!decoded)
        throw std::runtime_error("malformed telemetry: " + decoded.error);
      for (const auto& event : decoded.events)
        for (const auto& board : event.source_boards)
          if (board && board->id.starts_with("generated-copper:"))
            generated = board;
    }
    if (!generated)
      throw std::runtime_error("no generated copper snapshot");
    if (generated->revision != source.source_revision)
      throw std::runtime_error("recorded copper does not belong to the imported source revision");
    const auto point = [](const auto& p) {
      return flux::Point{flux::Nanometres{p.x_nm}, flux::Nanometres{p.y_nm}};
    };
    std::set<std::string> found;
    for (const auto& p : generated->primitives) {
      if (removed.contains(p.id)) {
        if (p.kind != "track" || p.net != chain.get<std::string>("net") ||
            p.width_nm != chain.get<std::uint64_t>("width_nm") ||
            p.layers != std::vector<std::string>{chain.get<std::string>("layer")})
          throw std::runtime_error("selected track identity does not match chain");
        found.insert(p.id);
        continue;
      }
      if (p.kind == "track" && p.points.size() == 2U && p.layers.size() == 1U) {
        source.tracks.push_back({flux::ObjectId{p.id}, flux::NetId{p.net},
                                 flux::LayerId{p.layers.front()}, point(p.points[0]),
                                 point(p.points[1]),
                                 flux::Nanometres{static_cast<std::int64_t>(p.width_nm)}, true});
      } else if (p.kind == "track_arc" && p.points.size() == 3U && p.layers.size() == 1U) {
        source.track_arcs.push_back(
            {flux::ObjectId{p.id}, flux::NetId{p.net}, flux::LayerId{p.layers.front()},
             point(p.points[0]), point(p.points[1]), point(p.points[2]),
             flux::Nanometres{static_cast<std::int64_t>(p.width_nm)}, true});
      } else if (p.kind == "via" && p.points.size() == 1U) {
        std::vector<flux::LayerId> layers;
        for (const auto& layer : p.layers)
          layers.push_back(flux::LayerId{layer});
        source.vias.push_back({flux::ObjectId{p.id}, flux::NetId{p.net}, point(p.points[0]),
                               flux::Nanometres{static_cast<std::int64_t>(p.diameter_nm)},
                               flux::Nanometres{static_cast<std::int64_t>(p.drill_nm)},
                               std::move(layers), true});
      } else {
        throw std::runtime_error("unsupported generated copper primitive: " + p.kind);
      }
    }
    if (found != removed)
      throw std::runtime_error("some selected track IDs are absent from recorded copper");
    flux::GlobalRouteResult global;
    global.stop_reason = flux::GlobalRouterStopReason::complete;
    flux::GlobalRoutePath path;
    path.net = flux::NetId{chain.get<std::string>("net")};
    const flux::LayerId layer{chain.get<std::string>("layer")};
    for (const auto& entry : chain.get_child("points_nm")) {
      const auto& p = entry.second;
      path.points.push_back(
          {{flux::Nanometres{p.get<std::int64_t>("x")}, flux::Nanometres{p.get<std::int64_t>("y")}},
           layer});
    }
    if (path.points.size() != removed.size() + 1U)
      throw std::runtime_error("chain must describe every selected straight segment");
    // Refuse an accidental changed seed: each removed primitive must match one chain edge.
    std::set<std::string> matched;
    for (std::size_t i = 1U; i < path.points.size(); ++i) {
      bool edge_found = false;
      for (const auto& p : generated->primitives) {
        if (!removed.contains(p.id) || matched.contains(p.id) || p.points.size() != 2U)
          continue;
        const auto a = point(p.points[0]);
        const auto b = point(p.points[1]);
        if ((a == path.points[i - 1U].position && b == path.points[i].position) ||
            (b == path.points[i - 1U].position && a == path.points[i].position)) {
          matched.insert(p.id);
          edge_found = true;
          break;
        }
      }
      if (!edge_found)
        throw std::runtime_error("chain edge differs from removed recorded copper");
    }
    const auto seed_probes = probe_chain_seeds(
        source, path, flux::Nanometres{chain.get<std::int64_t>("width_nm")});
    global.paths.push_back(std::move(path));
    flux::DetailedRouterConfig config;
    config.track_width = flux::Nanometres{chain.get<std::int64_t>("width_nm")};
    config.pull_tight_45_degrees = false;
    config.continuous_refinement = false;
    config.circular_arc_refinement = true;
    const auto result = flux::materialize_detailed_route(source, global, config);
    boost::property_tree::ptree output;
    output.put("authoritative", false);
    output.add_child("seed_probes", seed_probes);
    output.put("complete", result.complete());
    output.put("diagnostic", result.diagnostic);
    output.put("open_connections", result.connectivity.open_connections);
    output.put("copper_ok", result.copper_drc.ok());
    output.put("unrounded_bends", result.unrounded_non_junction_bends);
    output.put("fallbacks", result.spline_navmesh_certificate_fallbacks);
    output.put("navmesh_diagnostic", result.spline_navmesh_last_diagnostic);
    output.put("topology_diagnostic", result.spline_navmesh_last_topology_diagnostic);
    output.put("seed_candidates", result.spline_seed_candidates);
    output.put("seed_exact_legal", result.spline_seed_exact_legal);
    output.put("run_aligned_seed_candidates", result.spline_run_aligned_seed_candidates);
    output.put("run_aligned_seed_exact_legal", result.spline_run_aligned_seed_exact_legal);
    output.put("trajectory_attempts", result.spline_trajectory_attempts);
    output.put("trajectory_accepted", result.spline_trajectory_accepted);
    output.put("initial_shape", result.spline_trajectory_initial_shape_violations);
    output.put("final_shape", result.spline_trajectory_final_shape_violations);
    output.put("initial_continuous", result.spline_trajectory_initial_continuous_violations);
    output.put("final_continuous", result.spline_trajectory_final_continuous_violations);
    output.put("final_bend_radius", result.spline_trajectory_final_bend_radius_violations);
    output.put("continuous_budget_exhaustions", result.spline_continuous_budget_exhaustions);
    output.put("trajectory_candidate_checks", result.spline_trajectory_candidate_checks);
    output.put("fitted_arcs", result.fitted_circular_arcs);
    boost::property_tree::ptree failures;
    for (const auto& failure : result.spline_route_failure_details) {
      boost::property_tree::ptree item;
      item.put_value(failure);
      failures.push_back({"", std::move(item)});
    }
    output.add_child("failures", failures);
    boost::property_tree::write_json(std::cout, output);
    return result.complete() ? 0 : 3;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
