#include "flux/core/board_free_space.hpp"
#include "flux/core/free_space_offset.hpp"

#include <cstdlib>
#include <iostream>

namespace {
using flux::LayerId;
using flux::Nanometres;
using flux::NetId;
using flux::ObjectId;
using flux::Point;
using flux::Polygon;

[[noreturn]] void fail(const char* message) { std::cerr << message << '\n'; std::exit(1); }
void require(bool value, const char* message) { if (!value) fail(message); }
Point point(std::int64_t x, std::int64_t y) { return {Nanometres{x}, Nanometres{y}}; }
Polygon rectangle(std::int64_t x0, std::int64_t y0, std::int64_t x1, std::int64_t y1) {
  return {{point(x0, y0), point(x1, y0), point(x1, y1), point(x0, y1)}};
}

} // namespace

int main() {
  flux::Board board;
  board.source_revision = "intentionally-shared-revision";
  board.outline = rectangle(0, 0, 10000, 10000);
  board.layers = {{LayerId{"F.Cu"}, "F.Cu", flux::LayerKind::copper, 0U}};
  board.nets = {{NetId{"a"}, "a"}, {NetId{"b"}, "b"}};
  board.tracks = {
      {ObjectId{"a-track"}, NetId{"a"}, LayerId{"F.Cu"}, point(1000, 1000), point(2000, 1000), Nanometres{200}, false},
      {ObjectId{"b-track"}, NetId{"b"}, LayerId{"F.Cu"}, point(4000, 1000), point(5000, 1000), Nanometres{200}, false}};

  flux::BoardFreeSpaceExtractionCache extraction_cache{2U};
  const flux::BoardFreeSpaceExtractionConfig for_a{LayerId{"F.Cu"}, NetId{"a"}, true, true, true};
  const flux::BoardFreeSpaceExtractionConfig for_b{LayerId{"F.Cu"}, NetId{"b"}, true, true, true};
  const auto uncached_a = flux::extract_polygonal_board_free_space(board, for_a);
  const auto cached_a = flux::extract_polygonal_board_free_space(board, for_a, &extraction_cache);
  const auto cached_b = flux::extract_polygonal_board_free_space(board, for_b, &extraction_cache);
  require(uncached_a.domain.obstacles == cached_a.domain.obstacles &&
              cached_a.obstacle_nets == uncached_a.obstacle_nets,
          "cached extraction must preserve the exact net-filtered result");
  require(cached_a.domain.obstacles.size() == 1U && cached_b.domain.obstacles.size() == 1U &&
              cached_a.obstacle_nets.front() == NetId{"b"} && cached_b.obstacle_nets.front() == NetId{"a"},
          "one all-owner layer catalogue must apply same-net exclusions per request");
  require(extraction_cache.stats().hits == 1U && extraction_cache.stats().misses == 1U,
          "second net view must reuse the layer catalogue");
  auto changed = board;
  changed.tracks.back().start.x = changed.tracks.back().start.x + Nanometres{100};
  (void)flux::extract_polygonal_board_free_space(changed, for_a, &extraction_cache);
  require(extraction_cache.stats().misses == 2U,
          "equal revision text with different immutable board content must miss");

  flux::FreeSpaceDomain domain;
  domain.outline = rectangle(0, 0, 10000, 10000);
  domain.obstacles = {rectangle(4000, 4000, 5000, 5000)};
  flux::FreeSpaceOffsetConfig offset{Nanometres{250}, 1000U};
  flux::FreeSpaceOffsetCache offset_cache{2U};
  const auto first = flux::offset_polygonal_free_space(domain, offset, &offset_cache);
  const auto second = flux::offset_polygonal_free_space(domain, offset, &offset_cache);
  require(first.ok() && second.ok() && first.regions == second.regions,
          "cached obstacle expansion must preserve complete free-space geometry");
  require(offset_cache.stats().hits == 1U && offset_cache.stats().misses == 1U,
          "identical obstacle and exact clearance must reuse its expansion");
  offset.obstacle_clearances = {Nanometres{300}};
  const auto different_rule = flux::offset_polygonal_free_space(domain, offset, &offset_cache);
  require(different_rule.ok() && offset_cache.stats().misses == 2U,
          "effective heterogeneous clearance is part of the expansion identity");
  // Boolean reconstruction must preserve islands and holes, including holes closed by dilation.
  domain.obstacles.push_back(rectangle(4700, 4500, 6500, 6000));
  domain.filled_obstacles.push_back({LayerId{"F.Cu"}, rectangle(2000, 2000, 8000, 8000),
                                     {rectangle(3000, 3000, 7000, 7000)}});
  flux::FreeSpaceOffsetCache complex_cache{16U};
  for (const auto clearance : {Nanometres{250}, Nanometres{2200}}) {
    flux::FreeSpaceOffsetConfig complex{Nanometres{100}, 1000U};
    complex.obstacle_clearances = {Nanometres{150}, Nanometres{300}};
    complex.filled_obstacle_clearances = {clearance};
    const auto reference = flux::offset_polygonal_free_space(domain, complex);
    (void)flux::offset_polygonal_free_space(domain, complex, &complex_cache);
    const auto reused = flux::offset_polygonal_free_space(domain, complex, &complex_cache);
    require(reference.ok() == reused.ok() && reference.regions == reused.regions &&
                reference.diagnostic == reused.diagnostic,
            "cached filled holes and overlapping heterogeneous obstacles must match uncached Boolean geometry");
  }
  require(complex_cache.stats().hits >= 6U, "complex obstacle checks must exercise cached offsets");
  require(offset_cache.stats().entries <= 2U, "offset cache must retain its capacity bound");

}
