#include "flux/core/indexed_clearance_map.hpp"

#include <cmath>
#include <algorithm>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using flux::IndexedClearanceMapConfig;
using flux::IndexedClearancePrimitive;
[[noreturn]] void fail(const std::string& s) { throw std::runtime_error(s); }
void require(bool v, const std::string& s) {
  if (!v)
    fail(s);
}
std::vector<IndexedClearancePrimitive> square(std::int64_t lo, std::int64_t hi) {
  return {{lo, lo, hi, lo, 0}, {hi, lo, hi, hi, 0}, {hi, hi, lo, hi, 0}, {lo, hi, lo, lo, 0}};
}
void exact_distance_and_parity() {
  IndexedClearanceMapConfig c;
  c.width = 20;
  c.height = 20;
  c.pitch_nm = 10;
  auto r = flux::build_indexed_clearance_map(c, square(20, 180));
  require(r.map && r.status == flux::IndexedClearanceBuildStatus::ok, "build");
  auto in = r.map->exact_query(100, 100), out = r.map->exact_query(190, 100);
  require(in.inside && !out.inside, "parity");
  require(std::abs(in.boundary_distance_nm - 80.0) < 1e-9, "exact distance");
}
void nested_and_disjoint_parity() {
  auto p = square(10, 190);
  auto hole = square(60, 140);
  p.insert(p.end(), hole.begin(), hole.end());
  IndexedClearanceMapConfig c;
  c.width = 20;
  c.height = 20;
  c.pitch_nm = 10;
  auto r = flux::build_indexed_clearance_map(c, p);
  require(r.map->exact_query(30, 30).inside, "outer");
  require(!r.map->exact_query(100, 100).inside, "nested hole");
  require(!r.map->exact_query(0, 0).inside, "outside");
}
void thin_diagonal_and_multiple() {
  IndexedClearanceMapConfig c;
  c.width = 2;
  c.height = 2;
  c.pitch_nm = 100;
  c.sampled_cells = {0};
  std::vector<IndexedClearancePrimitive> p = {{0, 0, 100, 100, 1}, {0, 100, 100, 0, 2}};
  auto r = flux::build_indexed_clearance_map(c, p);
  const auto* cell = r.map->record_for_cell(0);
  require(cell && cell->boundary_touched, "diagonal touched");
  require(cell->overlap_count == 2 && r.map->overlap_primitive_indices().size() == 2,
          "both owners must survive in one pixel");
  require(r.map->record_for_cell(1) == nullptr, "unsampled cells have no free-space record");
  require(cell->whole_cell_distance_lower_bound_nm == 0.0, "conservative bound");
}
void limits_and_cancel() {
  IndexedClearanceMapConfig c;
  c.width = 3;
  c.height = 3;
  c.maximum_cells = 4;
  auto x = flux::build_indexed_clearance_map(c, {});
  require(x.status == flux::IndexedClearanceBuildStatus::limit_exceeded, "cell cap");
  c.width = 1;
  c.height = 1;
  c.maximum_cells = 4;
  c.cancellation_requested = [] { return true; };
  x = flux::build_indexed_clearance_map(c, {});
  require(x.status == flux::IndexedClearanceBuildStatus::cancelled, "cancel");
  c.cancellation_requested = {};
  c.origin_x_nm = std::numeric_limits<std::int64_t>::max();
  x = flux::build_indexed_clearance_map(c, {});
  require(x.status == flux::IndexedClearanceBuildStatus::invalid_input, "coordinate overflow");
  c.origin_x_nm = 0;
  c.maximum_references = 1;
  x = flux::build_indexed_clearance_map(c, square(0, 100));
  require(x.status == flux::IndexedClearanceBuildStatus::limit_exceeded, "reference cap");
  c.maximum_references = 64;
  c.maximum_bytes = 1;
  x = flux::build_indexed_clearance_map(c, square(0, 100));
  require(x.status == flux::IndexedClearanceBuildStatus::limit_exceeded, "byte cap");
}
void bvh_matches_exhaustive() {
  auto primitives = square(0, 1000);
  for (int i = 0; i < 6; ++i) {
    auto ring = square(50 + i * 130, 100 + i * 130);
    primitives.insert(primitives.end(), ring.begin(), ring.end());
  }
  IndexedClearanceMapConfig c;
  c.width = 20; c.height = 20; c.pitch_nm = 50;
  const auto built = flux::build_indexed_clearance_map(c, primitives);
  require(built.map != nullptr, "multi-node BVH build");
  for (int y = 5; y < 1000; y += 37) for (int x = 7; x < 1000; x += 31) {
    long double nearest = std::numeric_limits<long double>::infinity();
    bool inside = false;
    for (const auto& p : primitives) {
      const long double dx = p.end_x_nm - p.start_x_nm, dy = p.end_y_nm - p.start_y_nm;
      const auto t = std::clamp(((x - p.start_x_nm) * dx + (y - p.start_y_nm) * dy) /
                                  (dx * dx + dy * dy), 0.0L, 1.0L);
      nearest = std::min(nearest, std::hypotl(x - p.start_x_nm - t * dx, y - p.start_y_nm - t * dy));
      if ((p.start_y_nm > y) != (p.end_y_nm > y)) {
        const auto cross = p.start_x_nm + (y - p.start_y_nm) * dx / dy;
        if (cross > x) inside = !inside;
      }
    }
    const auto query = built.map->exact_query(x, y);
    require(query.inside == inside && std::abs(query.boundary_distance_nm - nearest) < 1e-9L,
            "BVH must match exhaustive distance and parity across multiple obstacles");
  }
}
void copper_envelope_candidates() {
  IndexedClearanceMapConfig c;
  c.width = c.height = 1; c.materialize_cells = false;
  std::vector<IndexedClearancePrimitive> primitives;
  for (std::uint32_t i = 0; i < 24; ++i) {
    const auto x = static_cast<std::int64_t>(i) * 100;
    primitives.push_back({x, 0, x, i % 3 == 0 ? 0 : 1000, i, i % 4, i % 2,
                          static_cast<long double>(i % 5) * 25});
  }
  const auto built = flux::build_indexed_clearance_map(c, primitives);
  require(built.map && built.map->cell_record_indices().empty(),
          "copper index must not allocate or classify polygon cells");
  for (std::uint32_t layer = 0; layer < 2; ++layer) for (int x = -100; x < 2500; x += 31) {
    std::vector<std::uint32_t> expected;
    for (std::uint32_t i = 0; i < primitives.size(); ++i) {
      const auto& p = primitives[i]; const auto radius = p.copper_radius_nm + 20 + 35;
      if (p.layer_id == layer && !(x + 75 + radius < p.start_x_nm || x - radius > p.end_x_nm ||
          50 + radius < p.start_y_nm || -50 - radius > p.end_y_nm)) expected.push_back(i);
    }
    require(built.map->segment_candidates(x, -50, x + 75, 50, 20, 35, layer) == expected,
            "expanded layered candidate query must match exhaustive envelope screening");
  }
  require(built.map->primitives()[5].owner_id == 5 && built.map->primitives()[5].net_id == 1,
          "candidate IDs must preserve owner and net membership metadata");
  primitives[0].copper_radius_nm = -1;
  require(flux::build_indexed_clearance_map(c, primitives).status ==
              flux::IndexedClearanceBuildStatus::invalid_input, "invalid radius rejected");
}
} // namespace
int main() {
  const std::vector<std::function<void()>> tests = {exact_distance_and_parity,
                                                    nested_and_disjoint_parity,
                                                    thin_diagonal_and_multiple, limits_and_cancel,
                                                    bvh_matches_exhaustive, copper_envelope_candidates};
  for (const auto& t : tests)
    t();
}
