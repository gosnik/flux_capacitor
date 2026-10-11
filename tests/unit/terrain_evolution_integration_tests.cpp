#include "flux/core/navmesh_route.hpp"
#include "flux/core/global_router.hpp"
#include "flux/core/board_free_space.hpp"
#include <cstdlib>
#include <thread>
#include <chrono>
#include <iostream>

namespace {
flux::Point point(double x, double y) {
  return {flux::Nanometres::from_mm(x), flux::Nanometres::from_mm(y)};
}
flux::Polygon rectangle(double x, double y, double xx, double yy) {
  return {{point(x,y), point(xx,y), point(xx,yy), point(x,yy)}};
}
void require(bool ok, const std::string& message) {
  if (!ok) { std::cerr << message << '\n'; std::exit(1); }
}
void board_guidance_reaches_global_routing() {
  const flux::LayerId front{"F.Cu"};
  const flux::NetId net{"terrain-signal"};
  flux::Board board;
  board.source_revision = "board-guidance-integration";
  board.outline = rectangle(0, 0, 20, 12);
  board.layers = {{front, "F.Cu", flux::LayerKind::copper, 0U}};
  board.nets = {{net, "SIGNAL", 0U}};
  for (const auto x : {4.0, 16.0}) {
    flux::Footprint footprint;
    footprint.id = flux::ObjectId{x == 4.0 ? "U1" : "U2"};
    footprint.reference = x == 4.0 ? "U1" : "U2";
    footprint.position = point(x, 6);
    footprint.courtyard = rectangle(-1, -1, 1, 1);
    footprint.pads = {{flux::ObjectId{x == 4.0 ? "U1-pad" : "U2-pad"},
                       net, flux::Point{}, {front}}};
    board.footprints.push_back(std::move(footprint));
  }
  flux::GlobalRouterConfig config;
  config.grid_pitch = flux::Nanometres::from_mm(1);
  config.max_iterations = 1U;
  config.static_seed_hydraulic_flow.prefer_gpu = false;
  config.board_terrain_guidance.prefer_gpu = false;
  const auto physical = flux::extract_polygonal_board_free_space(
      board, {front, flux::NetId{}, true, false, true});
  require(physical.complete() && physical.obstacle_ids.size() == 2U,
          "all-owner terrain extraction must include both pads without an invented routed net");
  require(!flux::extract_polygonal_board_free_space(
              board, {front, flux::NetId{}, true, true, true}).complete(),
          "net-specific extraction must still reject an undeclared routed net");
  const auto first = flux::route_global(board, config);
  require(first.board_terrain_guidance_stats.enabled &&
              first.board_terrain_guidance_stats.cells > 0U &&
              first.board_terrain_guidance_stats.pressure_demands == 1U &&
              first.board_terrain_guidance_stats.pressure_cells > 0U,
          "real pad demands must reach board-wide terrain pressure: " +
              first.board_terrain_guidance_stats.diagnostic + "; " + first.diagnostic);
  const auto reused = flux::route_global(board, config);
  require(reused.board_terrain_guidance_stats.cache_reused,
          "repeated global preparation must reuse endpoint-independent erosion");
  config.board_terrain_guidance.enabled = false;
  const auto disabled = flux::route_global(board, config);
  require(!disabled.board_terrain_guidance_stats.enabled &&
              disabled.board_terrain_guidance_stats.cells == 0U,
          "ablation must bypass board guidance");
}
}
int main() {
  board_guidance_reaches_global_routing();
  flux::OffsetFreeSpaceRegion region;
  region.area = rectangle(0.5, 0.5, 99.5, 79.5);
  region.holes.push_back(rectangle(29.5, 39.5, 70.5, 40.7));
  flux::NavMeshConfig nav;
  nav.cell_pitch = flux::Nanometres::from_mm(5);
  nav.maximum_cells = 100'000;
  const auto prepared = flux::prepare_navmesh_domain(region, nav);
  require(prepared.ok(), "test domain must be valid");
  flux::NavMeshHydraulicConfig config;
  config.flow.evolve_terrain_height = true;
  config.flow.prefer_gpu = false;
  const auto route = flux::route_prepared_navmesh_hydraulic(prepared, point(20,40), point(80,60), nav, config);
  require(route.ok(), "height-evolution corridor must survive exact projection: " + route.diagnostic);
  require(route.used_height_evolution && !route.used_hydraulic_gpu && !route.hydraulic_cpu_fallback &&
          route.height_evolution_iterations > 0 && route.height_evolution_changed_cells > 0 &&
          route.height_evolution_expanded_nodes > 0 && route.hydraulic_exact_certified_chords > 0,
          "actual height evolution, path search and exact certification must be exercised");
  require(flux::validate_navmesh_topology_signature(route.topology_signature, route.centerline).preserved,
          "terrain corridor must preserve its certified topology");
  config.flow.terrain_evolution.maximum_cells = 1;
  const auto limited = flux::route_prepared_navmesh_hydraulic(prepared, point(20,40), point(80,60), nav, config);
  require(!limited.ok() && limited.used_height_evolution, "budget exhaustion must not silently return droplets");
  flux::HydraulicPathInput disconnected;
  disconnected.width=5; disconnected.height=3; disconnected.source=0; disconnected.target=4;
  disconnected.walkable = {1,1,0,1,1, 1,1,0,1,1, 1,1,0,1,1};
  disconnected.obstacle_clearance = {1,5,0,5,1, 1,5,0,5,1, 1,5,0,5,1};
  flux::HydraulicPathConfig mode;
  mode.evolve_terrain_height=true; mode.prefer_gpu=false;
  const auto failed = flux::solve_hydraulic_erosion_path(disconnected, mode);
  require(!failed.ok() && failed.height_evolution_invoked && failed.height_evolution_iterations==10 &&
          failed.height_evolution_changed_cells>0, "failed extraction must retain completed evolution work");
  std::stop_source already;
  already.request_stop();
  const auto not_started = flux::route_prepared_navmesh_hydraulic(prepared, point(20,40), point(80,60),
                                                                 nav, config, already.get_token());
  require(!not_started.ok() && !not_started.height_evolution_invoked, "pre-cancellation must not invoke evolution");
  flux::HydraulicPathInput large;
  large.width=512; large.height=512; large.source=0; large.target=512*512-1;
  large.walkable.assign(512*512,1); large.obstacle_clearance.assign(512*512,2);
  mode.terrain_evolution.iterations=256;
  std::stop_source running;
  std::jthread cancel([&] { std::this_thread::sleep_for(std::chrono::milliseconds(2)); running.request_stop(); });
  const auto interrupted = flux::solve_hydraulic_erosion_path(large, mode, running.get_token());
  require(!interrupted.ok() && interrupted.height_evolution_iterations < 256,
          "production wrapper must forward cancellation during an active terrain solve");
  std::cout << "height-evolution polygon discovery and exact projection passed\n";
}
