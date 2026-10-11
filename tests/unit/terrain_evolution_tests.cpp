#include "flux/core/terrain_evolution.hpp"

#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <limits>
#include <numeric>
#include <iostream>
#include <stop_token>
#include <vector>

namespace {
void require(bool value, const char* message) {
  if (!value) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

std::size_t at(std::size_t width, std::size_t x, std::size_t y) { return y * width + x; }
} // namespace

int main() {
  constexpr std::size_t width = 7U;
  constexpr std::size_t height = 5U;
  std::vector<std::uint8_t> open(width * height, 1U);
  std::vector<std::uint32_t> clearance(width * height, 1U);
  for (std::size_t y = 0; y < height; ++y)
    clearance[at(width, 3U, y)] = 8U;

  flux::TerrainEvolutionConfig zero_iterations;
  zero_iterations.iterations = 0U;
  const auto baseline = flux::solve_terrain_evolution(width, height, open, clearance, 0U,
                                                       width * height - 1U, zero_iterations);
  require(baseline.ok(), "zero-iteration terrain still extracts a connected path");
  require(baseline.height == baseline.initial_height, "zero iterations preserve initial terrain");

  std::vector<std::uint32_t> pressure(width * height, 0U), success_prior(width * height, 0U);
  pressure[at(width, 3U, 2U)] = 11U;
  success_prior[at(width, 1U, 1U)] = 7U;
  const auto fields = flux::evolve_terrain_fields(width, height, open, clearance, {}, {}, pressure,
                                                   success_prior);
  const auto endpoint_a = flux::solve_terrain_evolution(width, height, open, clearance, 0U,
                                                         width * height - 1U, {}, {}, pressure,
                                                         success_prior);
  const auto endpoint_b = flux::solve_terrain_evolution(width, height, open, clearance, width - 1U,
                                                         width * (height - 1U), {}, {}, pressure,
                                                         success_prior);
  require(fields.ok() && endpoint_a.ok() && endpoint_b.ok() &&
              fields.initial_height == endpoint_a.initial_height && fields.height == endpoint_a.height &&
              fields.flow == endpoint_a.flow && fields.divergence == endpoint_a.divergence &&
              fields.height == endpoint_b.height && fields.flow == endpoint_b.flow,
          "target-independent field evolution must match solves with different endpoints");

  const auto evolved = flux::solve_terrain_evolution(width, height, open, clearance, 0U,
                                                      width * height - 1U);
  require(evolved.ok() && evolved.iterations_completed == 10U,
          "bounded evolution returns a terrain corridor");

  bool changed = false;
  for (std::size_t cell = 0; cell < evolved.height.size(); ++cell)
    changed = changed || evolved.height[cell] != evolved.initial_height[cell];
  require(changed, "erosion/deposition must change terrain after a nonzero iteration count");

  // A one-cell neck remains traversable and diagonal moves cannot cut around its blocked walls.
  std::vector<std::uint8_t> neck(5U * 5U, 1U);
  std::vector<std::uint32_t> neck_clearance(neck.size(), 1U);
  for (std::size_t x = 0U; x < 5U; ++x)
    if (x != 2U)
      neck[at(5U, x, 2U)] = 0U;
  const auto narrow = flux::solve_terrain_evolution(5U, 5U, neck, neck_clearance,
                                                     at(5U, 2U, 0U), at(5U, 2U, 4U));
  require(narrow.ok() && std::find(narrow.path.begin(), narrow.path.end(), at(5U, 2U, 2U)) !=
                             narrow.path.end(),
          "narrow passage remains connected through its only legal neck");

  // U-shaped terrain has a local downhill basin; extraction must nevertheless climb out.
  std::vector<std::uint8_t> u(7U * 7U, 1U);
  std::vector<std::uint32_t> u_clearance(u.size(), 1U);
  for (std::size_t y = 1U; y < 6U; ++y) {
    u[at(7U, 2U, y)] = 0U;
    u[at(7U, 4U, y)] = 0U;
  }
  u[at(7U, 3U, 5U)] = 0U;
  for (std::size_t y = 1U; y < 5U; ++y) u_clearance[at(7U, 3U, y)] = 20U;
  const auto saddle = flux::solve_terrain_evolution(7U, 7U, u, u_clearance, at(7U, 3U, 4U),
                                                     at(7U, 6U, 3U));
  require(saddle.ok(), "weighted graph extraction escapes a terrain basin via a saddle");
  bool climbs = false;
  for (std::size_t i=1; i<saddle.path.size(); ++i)
    climbs = climbs || saddle.height[saddle.path[i]] > saddle.height[saddle.path[i-1]] + 1e-9;
  require(climbs, "basin fixture must actually require an uphill transition");
  for (std::size_t cell = 0U; cell < u.size(); ++cell)
    if (u[cell] == 0U)
      require(saddle.height[cell] == saddle.initial_height[cell],
              "immutable obstacle mask never evolves");

  std::vector<std::uint8_t> split(3U * 3U, 1U);
  for (std::size_t y = 0U; y < 3U; ++y)
    split[at(3U, 1U, y)] = 0U;
  const auto disconnected = flux::solve_terrain_evolution(3U, 3U, split,
                                                           std::vector<std::uint32_t>(9U, 1U),
                                                           0U, 2U);
  require(!disconnected.ok() && !disconnected.diagnostic.empty(), "disconnected inputs fail closed");

  const auto repeat = flux::solve_terrain_evolution(width, height, open, clearance, 0U, width*height-1U);
  require(repeat.path == evolved.path && repeat.height == evolved.height,
          "identical terrain solves must be deterministic");
  require(std::abs(std::accumulate(evolved.divergence.begin(), evolved.divergence.end(), 0.0)) < 1e-9,
          "masked edge flux must conserve outgoing/incoming transport");
  flux::TerrainEvolutionConfig invalid;
  invalid.erosion_rate = std::numeric_limits<double>::infinity();
  require(!flux::solve_terrain_evolution(width,height,open,clearance,0U,width*height-1U,invalid).ok(),
          "nonfinite evolution parameters must fail closed");
  require(!flux::evolve_terrain_fields(width, height, open, clearance, invalid).ok(),
          "field evolution must reject invalid parameters without endpoint extraction");
  invalid = {}; invalid.iterations = 257;
  require(!flux::solve_terrain_evolution(width,height,open,clearance,0U,width*height-1U,invalid).ok(),
          "unbounded iteration requests must fail closed");
  const std::vector<std::uint8_t> diagonal_only{1,0,0,1};
  require(!flux::solve_terrain_evolution(2,2,diagonal_only,{1,0,0,1},0,3).ok(),
          "diagonal corner cutting cannot invent a passage");
  const auto invalid_source = flux::solve_terrain_evolution(
      width, height, open, clearance, width * height, width * height - 1U);
  auto blocked_endpoint_mask = open;
  blocked_endpoint_mask.back() = 0U;
  const auto blocked_endpoint = flux::solve_terrain_evolution(
      width, height, blocked_endpoint_mask, clearance, 0U, width * height - 1U);
  require(!invalid_source.ok() && !blocked_endpoint.ok() &&
              invalid_source.iterations_completed == 0U && blocked_endpoint.iterations_completed == 0U &&
              invalid_source.height.empty() && blocked_endpoint.height.empty() &&
              invalid_source.evolution_time_us == 0U && blocked_endpoint.evolution_time_us == 0U,
          "invalid source or blocked endpoint must fail before CPU terrain evolution");
  std::stop_source stop;
  stop.request_stop();
  const auto cancelled = flux::solve_terrain_evolution(width, height, open, clearance, 0U,
                                                        width * height - 1U, {}, stop.get_token());
  require(!cancelled.ok() && !cancelled.diagnostic.empty(), "cancellation fails closed");
  const auto cancelled_fields = flux::evolve_terrain_fields(width, height, open, clearance, {},
                                                             stop.get_token());
  require(!cancelled_fields.ok() && !cancelled_fields.diagnostic.empty(),
          "field evolution cancellation fails closed without fabricating endpoints");
  flux::TerrainEvolutionConfig field_bounds;
  field_bounds.maximum_cells = width * height - 1U;
  require(!flux::evolve_terrain_fields(width, height, open, clearance, field_bounds).ok(),
          "field evolution enforces its independent bounded work budget");
}
