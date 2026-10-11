#include "flux/core/regional_channel_spatial_index.hpp"

#include <cstdlib>
#include <iostream>

namespace {
using namespace flux;

[[noreturn]] void fail(const char* message) { std::cerr << message << '\n'; std::exit(1); }
void require(bool value, const char* message) { if (!value) fail(message); }

geometry::PreparedPolygon rectangle(std::int64_t min_x, std::int64_t min_y,
                                    std::int64_t max_x, std::int64_t max_y) {
  return geometry::prepare_polygon(Polygon{{{Nanometres{min_x}, Nanometres{min_y}},
                                             {Nanometres{max_x}, Nanometres{min_y}},
                                             {Nanometres{max_x}, Nanometres{max_y}},
                                             {Nanometres{min_x}, Nanometres{max_y}}}});
}

void indexed_candidates_preserve_first_match_order() {
  detail::RegionalChannelSpatialIndex index;
  index.append(1U, rectangle(0, 0, 2'000'000, 2'000'000));
  index.append(0U, rectangle(1'000'000, 0, 3'000'000, 2'000'000));
  const auto candidates = index.query({rectangle(2'000'000, 500'000, 2'500'000, 1'500'000)}, 2U);
  require(candidates == std::vector<std::size_t>({0U, 1U}),
          "spatial candidates must retain channel-vector order for first-match merging");
}

void indexed_candidates_cover_touching_containment_and_negative_cells() {
  detail::RegionalChannelSpatialIndex index;
  index.append(0U, rectangle(-3'000'000, -3'000'000, -1'000'000, -1'000'000));
  index.append(1U, rectangle(0, 0, 2'000'000, 2'000'000));
  require(index.query({rectangle(-2'000'000, -2'000'000, -1'500'000, -1'500'000)}, 2U) ==
              std::vector<std::size_t>({0U}),
          "negative-coordinate containment must remain indexed");
  require(index.query({rectangle(2'000'000, 500'000, 2'500'000, 1'500'000)}, 2U) ==
              std::vector<std::size_t>({1U}),
          "touching bounds must remain exact-overlap candidates");
}

void oversized_cells_are_conservative_candidates() {
  detail::RegionalChannelSpatialIndex index;
  index.append(0U, rectangle(0, 0, 20'000'000, 20'000'000));
  require(index.query({rectangle(100'000'000, 100'000'000, 101'000'000, 101'000'000)}, 1U) ==
              std::vector<std::size_t>({0U}),
          "oversized cells must use conservative fallback rather than disappear");
}
} // namespace

int main() {
  indexed_candidates_preserve_first_match_order();
  indexed_candidates_cover_touching_containment_and_negative_cells();
  oversized_cells_are_conservative_candidates();
}
