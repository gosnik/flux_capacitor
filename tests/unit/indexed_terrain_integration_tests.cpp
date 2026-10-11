#include "flux/core/hydraulic_path.hpp"
#include <cstdlib>
#include <iostream>
#include <thread>
#include <chrono>
#include <barrier>
#include <future>
#include <array>

namespace {
void require(bool value, const char* message) {
  if (!value) { std::cerr << message << '\n'; std::exit(1); }
}
flux::HydraulicPathInput input() {
  flux::HydraulicPathInput value;
  value.width = 20; value.height = 20; value.source = 21; value.target = 378;
  value.polygon_raster = flux::HydraulicPolygonRaster{0,0,100'000,10'000,
      {{0,0,2'000'000,0,0}, {2'000'000,0,2'000'000,2'000'000,0},
       {2'000'000,2'000'000,0,2'000'000,0}, {0,2'000'000,0,0,0},
       {800'000,500'000,1'200'000,500'000,1},
       {1'200'000,500'000,1'200'000,1'500'000,1},
       {1'200'000,1'500'000,800'000,1'500'000,1},
       {800'000,1'500'000,800'000,500'000,1}}};
  return value;
}
}
int main() {
  auto request = input();
  flux::HydraulicPathConfig config;
  config.evolve_terrain_height = true; config.prefer_gpu = false;
  auto oracle_config = config; oracle_config.use_indexed_clearance_map = false;
  const auto oracle = flux::solve_hydraulic_erosion_path(request, oracle_config);
  const auto first = flux::solve_hydraulic_erosion_path(request, config);
  require(oracle.ok() && first.ok(), "indexed terrain and exhaustive oracle must route");
  require(first.indexed_clearance_map && first.indexed_clearance_map_builds == 1 &&
      first.indexed_clearance_map_fallbacks == 0, "production terrain must use indexed physical map");
  require(first.walkable == oracle.walkable && first.obstacle_clearance == oracle.obstacle_clearance &&
      first.riverbed == oracle.riverbed && first.terrain_height == oracle.terrain_height,
      "indexed geometry must preserve exact polygon raster and terrain results");
  request.downstream_pressure.assign(400, 100);
  const auto history = flux::solve_hydraulic_erosion_path(request, config);
  require(history.ok() && history.indexed_clearance_map == first.indexed_clearance_map &&
      history.indexed_clearance_map_reuses == 1 && history.walkable == first.walkable &&
      history.obstacle_clearance == first.obstacle_clearance,
      "pressure changes must reuse immutable physical records without changing clearance");
  auto preferences = request;
  preferences.source = 22; preferences.target = 377;
  preferences.success_prior.assign(400, 3);
  const auto preferred = flux::solve_hydraulic_erosion_path(preferences, config);
  require(preferred.ok() && preferred.indexed_clearance_map == first.indexed_clearance_map &&
      preferred.indexed_clearance_map_reuses == 1 &&
      preferred.obstacle_clearance == first.obstacle_clearance,
      "endpoint and success history changes must preserve the physical snapshot");
  auto changed = request;
  changed.polygon_raster->boundary_segments[4].first_x_nm = 700'000;
  changed.polygon_raster->boundary_segments[6].second_x_nm = 700'000;
  changed.polygon_raster->boundary_segments[7].first_x_nm = 700'000;
  changed.polygon_raster->boundary_segments[7].second_x_nm = 700'000;
  const auto rebuilt = flux::solve_hydraulic_erosion_path(changed, config);
  require(rebuilt.indexed_clearance_map && rebuilt.indexed_clearance_map != first.indexed_clearance_map &&
      rebuilt.indexed_clearance_map_builds == 1,
      "changed physical geometry must not reuse old records");
  const auto restored = flux::solve_hydraulic_erosion_path(request, config);
  require(restored.indexed_clearance_map == first.indexed_clearance_map,
      "restoring geometry must reuse the still-valid immutable snapshot");
  auto edge = input(); edge.polygon_raster->minimum_boundary_clearance_nm = 0;
  // Place several cell centres exactly on a hole boundary; preserve the reference half-open rule.
  for (std::size_t i = 4; i < 8; ++i) {
    edge.polygon_raster->boundary_segments[i].first_x_nm += 50'000;
    edge.polygon_raster->boundary_segments[i].second_x_nm += 50'000;
    edge.polygon_raster->boundary_segments[i].first_y_nm += 50'000;
    edge.polygon_raster->boundary_segments[i].second_y_nm += 50'000;
  }
  const auto edge_index = flux::solve_hydraulic_erosion_path(edge, config);
  const auto edge_oracle = flux::solve_hydraulic_erosion_path(edge, oracle_config);
  require(edge_index.walkable == edge_oracle.walkable &&
      edge_index.obstacle_clearance == edge_oracle.obstacle_clearance,
      "zero-clearance boundary-centre parity must match exact polygon reference");
  flux::HydraulicPathInput large;
  large.width = 256; large.height = 256; large.source = 0; large.target = 65535;
  large.walkable.assign(65536, 1); large.obstacle_clearance.assign(65536, 2);
  auto normal = config; normal.evolve_terrain_height = false;
  std::stop_source active;
  std::jthread cancel([&] { std::this_thread::sleep_for(std::chrono::milliseconds(5)); active.request_stop(); });
  require(!flux::solve_hydraulic_erosion_path(large, normal, active.get_token()).ok(),
      "normal CPU hydraulic solve must honor active cancellation");
  auto concurrent_input = input(); concurrent_input.polygon_raster->origin_x_nm = 7'000'000;
  std::barrier start(4);
  std::array<std::future<flux::HydraulicPathResult>, 4> futures;
  for (auto& future : futures) future = std::async(std::launch::async, [&] {
    start.arrive_and_wait(); return flux::solve_hydraulic_erosion_path(concurrent_input, config);
  });
  std::array<flux::HydraulicPathResult, 4> concurrent;
  std::size_t builds = 0;
  for (std::size_t i = 0; i < concurrent.size(); ++i) {
    concurrent[i] = futures[i].get(); builds += concurrent[i].indexed_clearance_map_builds;
    require(concurrent[i].ok() && concurrent[i].indexed_clearance_map == concurrent[0].indexed_clearance_map,
        "concurrent requests must share the same immutable map lease");
  }
  require(builds == 1, "simultaneous identical cache misses must build only one map");
  std::stop_source stop; stop.request_stop();
  require(!flux::solve_hydraulic_erosion_path(request, config, stop.get_token()).ok(),
      "cancellation must not publish a partial physical map");
  std::cout << "indexed terrain parity and history/geometry cache invalidation passed\n";
}
