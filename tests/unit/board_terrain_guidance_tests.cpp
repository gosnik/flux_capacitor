#include "flux/core/board_terrain_guidance.hpp"

#include <cstdlib>
#include <iostream>
#include <limits>

namespace {
void require(bool value, const char* message) {
  if (!value) { std::cerr << message << '\n'; std::exit(1); }
}
flux::Point point(std::int64_t x, std::int64_t y) { return {flux::Nanometres{x}, flux::Nanometres{y}}; }
flux::BoardTerrainGuidanceLayerInput field(flux::LayerId layer) {
  flux::BoardTerrainGuidanceLayerInput result;
  result.layer = std::move(layer); result.origin = point(0, 0); result.cell_pitch = flux::Nanometres{1};
  result.width = 5; result.height = 5; result.walkable.assign(25, 1U); result.clearance.assign(25, 8U);
  result.walkable[12] = 0U; result.clearance[12] = 0U;
  return result;
}
}
int main() {
  flux::BoardTerrainGuidanceConfig config;
  config.prefer_gpu = false; config.maximum_cells = 64; config.evolution.maximum_cells = 64;
  config.evolution.iterations = 1;
  flux::BoardTerrainGuidanceCache cache;
  const auto first = cache.prepare({field(flux::LayerId{"F.Cu"}), field(flux::LayerId{"B.Cu"})}, config, 7U);
  require(first->stats().diagnostic.empty() && first->layers().size() == 2U, "bounded layer fields build");
  require(first->sample_cost(flux::LayerId{"F.Cu"}, point(2, 2)) == 0U,
          "blocked guidance cell must not be made free or charged as a route");
  require(first->sample_cost(flux::LayerId{"other"}, point(1, 1)) == 0U, "guidance is layer isolated");
  const auto reused = cache.prepare({field(flux::LayerId{"F.Cu"}), field(flux::LayerId{"B.Cu"})}, config, 7U);
  require(reused->stats().cache_reused, "identical semantic physical input reuses geometry field");
  auto changed = field(flux::LayerId{"F.Cu"}); changed.clearance[0] = 3U;
  const auto rebuilt = cache.prepare({changed}, config, 7U);
  require(!rebuilt->stats().cache_reused, "same revision with changed physical input cannot alias cache");
  const auto neutral = cache.refresh_pressure(first, {});
  const auto pressured = cache.refresh_pressure(first, {{flux::LayerId{"F.Cu"}, {0U, 4U, 24U}, 100U}});
  require(pressured->stats().pressure_cells > 0U &&
              pressured->sample_cost(flux::LayerId{"F.Cu"}, point(1, 0)) >
                  neutral->sample_cost(flux::LayerId{"F.Cu"}, point(1, 0)),
          "actual multi-terminal demand contributes bounded soft pressure");
  const auto ordinary = cache.refresh_pressure(first, {{flux::LayerId{"F.Cu"}, {0U, 4U}, 1U}});
  require(ordinary->sample_cost(flux::LayerId{"F.Cu"}, point(1, 0)) >
              neutral->sample_cost(flux::LayerId{"F.Cu"}, point(1, 0)),
          "one ordinary unresolved connection has a non-zero bounded pressure response");
  require(pressured->layers()[0].pressure[12] == 0U, "pressure never changes blocked cells");
  const auto refreshed_again = cache.refresh_pressure(pressured, {{flux::LayerId{"F.Cu"}, {0U, 4U, 24U}, 100U}});
  require(refreshed_again->layers()[0].pressure == pressured->layers()[0].pressure &&
              refreshed_again->stats().pressure_paths == pressured->stats().pressure_paths &&
              refreshed_again->stats().pressure_search_expansions == pressured->stats().pressure_search_expansions,
          "refreshing a pressure snapshot must start with a fresh bounded allowance");
  flux::BoardTerrainGuidanceConfig limited = config; limited.maximum_cells = 4;
  const auto rejected = cache.prepare({field(flux::LayerId{"F.Cu"})}, limited, 9U);
  require(!rejected->stats().diagnostic.empty(), "cell limit is observable");
  auto islands = field(flux::LayerId{"F.Cu"});
  for (std::size_t y = 0; y < 5; ++y) islands.walkable[y * 5 + 2] = 0U;
  auto one_search = config;
  one_search.maximum_pressure_paths = 1U;
  const auto split = cache.prepare({islands}, one_search, 10U);
  require(split->stats().component_count == 2U && split->reachable_component_cells(flux::LayerId{"F.Cu"}, point(0, 0)) == 10U,
          "immutable component labels retain disconnected island areas");
  const auto mixed = cache.refresh_pressure(split, {{flux::LayerId{"F.Cu"}, {0U, 4U}, 1U},
                                                     {flux::LayerId{"F.Cu"}, {0U, 5U}, 1U}});
  require(mixed->stats().disconnected_pressure_pairs == 1U && mixed->stats().pressure_pairs_inspected == 2U &&
              mixed->stats().pressure_paths == 1U && mixed->stats().pressure_cells > 0U &&
              mixed->stats().pressure_search_expansions > 0U,
          "disconnected pressure pairs skip Dijkstra while later connected pairs still run");
  islands.walkable[12] = 1U;
  const auto bridged = cache.prepare({islands}, config, 10U);
  require(!bridged->stats().cache_reused && bridged->stats().component_count == 1U,
          "one-cell bridge rebuilds immutable component backing");
  auto pressure_limited = config;
  pressure_limited.maximum_pressure_expansions = 1U;
  const auto bounded_geometry = cache.prepare({field(flux::LayerId{"F.Cu"})}, pressure_limited, 10U);
  const auto bounded_pressure = cache.refresh_pressure(bounded_geometry, {{flux::LayerId{"F.Cu"}, {0U, 24U}, 1U}});
  require(bounded_pressure->stats().pressure_budget_exhausted &&
              bounded_pressure->stats().pressure_search_expansions == 1U &&
              bounded_pressure->stats().pressure_cells == 0U,
          "exhausted pressure search reports partial work without depositing an unfinished path");
  auto extreme = field(flux::LayerId{"F.Cu"});
  extreme.origin = point(std::numeric_limits<std::int64_t>::min(), 0);
  const auto extreme_field = cache.prepare({extreme}, config, 11U);
  require(extreme_field->sample_cost(flux::LayerId{"F.Cu"},
              point(std::numeric_limits<std::int64_t>::max(), 0)) == 0U,
          "out-of-range coordinate differences must not overflow");
  require(extreme_field->reachable_component_cells(flux::LayerId{"F.Cu"},
              point(std::numeric_limits<std::int64_t>::max(), 0)) == 0U,
          "component area query must safely handle extreme coordinate differences");
  const auto duplicate = cache.prepare({field(flux::LayerId{"F.Cu"}), field(flux::LayerId{"F.Cu"})}, config, 12U);
  require(!duplicate->stats().diagnostic.empty(), "duplicate layer identity is ambiguous and must be rejected");
  std::cout << "board terrain guidance passed\n";
}
