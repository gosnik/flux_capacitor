#include "flux/core/board_free_space.hpp"
#include "flux/core/free_space_offset.hpp"

#include <atomic>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <vector>

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

bool same_offset_result(const flux::FreeSpaceOffsetResult& lhs,
                        const flux::FreeSpaceOffsetResult& rhs) {
  return lhs.regions == rhs.regions && lhs.output_vertices == rhs.output_vertices &&
         lhs.diagnostic == rhs.diagnostic;
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
  const auto changed_uncached = flux::extract_polygonal_board_free_space(changed, for_a);
  const auto changed_cached = flux::extract_polygonal_board_free_space(changed, for_a, &extraction_cache);
  require(changed_cached.domain.obstacles == changed_uncached.domain.obstacles &&
              changed_cached.obstacle_ids == changed_uncached.obstacle_ids &&
              changed_cached.obstacle_nets == changed_uncached.obstacle_nets,
          "moved copper with an equal revision must not reuse stale extracted blockers");
  auto added_blocker = changed;
  added_blocker.tracks.push_back(
      {ObjectId{"added-track"}, NetId{"b"}, LayerId{"F.Cu"}, point(6500, 1000),
       point(7500, 1000), Nanometres{200}, false});
  const auto added_uncached = flux::extract_polygonal_board_free_space(added_blocker, for_a);
  const auto added_cached = flux::extract_polygonal_board_free_space(added_blocker, for_a, &extraction_cache);
  require(added_cached.domain.obstacles == added_uncached.domain.obstacles &&
              added_cached.obstacle_ids == added_uncached.obstacle_ids,
          "added copper must be reflected in cached extraction");
  added_blocker.tracks.pop_back();
  const auto removed_cached = flux::extract_polygonal_board_free_space(added_blocker, for_a, &extraction_cache);
  require(removed_cached.domain.obstacles == changed_uncached.domain.obstacles &&
              removed_cached.obstacle_ids == changed_uncached.obstacle_ids,
          "removed copper must not survive in a cached extraction");

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
  const auto repeated_context_stats = offset_cache.stats();
  require(repeated_context_stats.input_validation_hits > 0U &&
              repeated_context_stats.prepared_domain_hits > 0U &&
              repeated_context_stats.clearance_certificate_hits > 0U,
          "a repeated positive-clearance request must reuse validation, prepared-domain, and clearance certificates");
  const auto stats_before_rule_change = offset_cache.stats();
  offset.obstacle_clearances = {Nanometres{300}};
  const auto different_rule = flux::offset_polygonal_free_space(domain, offset, &offset_cache);
  const auto different_rule_reference = flux::offset_polygonal_free_space(domain, offset);
  require(same_offset_result(different_rule, different_rule_reference),
          "changed obstacle clearance must match uncached free-space geometry");
  const auto stats_after_rule_change = offset_cache.stats();
  require(stats_after_rule_change.input_validation_hits > stats_before_rule_change.input_validation_hits &&
              stats_after_rule_change.prepared_domain_hits > stats_before_rule_change.prepared_domain_hits &&
              stats_after_rule_change.clearance_certificate_misses >
                  stats_before_rule_change.clearance_certificate_misses,
          "a rule change must retain domain preparation while invalidating its clearance certificate");
  auto moved_blocker = domain;
  moved_blocker.obstacles.front() = rectangle(4500, 4000, 5500, 5000);
  const auto moved_reference = flux::offset_polygonal_free_space(moved_blocker, offset);
  const auto moved_cached = flux::offset_polygonal_free_space(moved_blocker, offset, &offset_cache);
  require(same_offset_result(moved_cached, moved_reference),
          "moved obstacle expansion must not reuse stale cached geometry");
  auto added_blocker_domain = moved_blocker;
  added_blocker_domain.obstacles.push_back(rectangle(6500, 4000, 7000, 5000));
  flux::FreeSpaceOffsetConfig added_rule{Nanometres{250}, 1000U};
  added_rule.obstacle_clearances = {Nanometres{300}, Nanometres{125}};
  const auto added_reference = flux::offset_polygonal_free_space(added_blocker_domain, added_rule);
  const auto added_cached_offset =
      flux::offset_polygonal_free_space(added_blocker_domain, added_rule, &offset_cache);
  require(same_offset_result(added_cached_offset, added_reference),
          "added obstacle and its rule must match uncached free-space geometry");
  added_blocker_domain.obstacles.pop_back();
  const auto removed_reference = flux::offset_polygonal_free_space(moved_blocker, offset);
  const auto removed_cached_offset =
      flux::offset_polygonal_free_space(added_blocker_domain, offset, &offset_cache);
  require(same_offset_result(removed_cached_offset, removed_reference),
          "removed obstacle must not remain in cached free-space geometry");
  auto changed_cutout = moved_blocker;
  changed_cutout.cutouts.push_back(rectangle(1000, 6000, 2500, 8000));
  const auto stats_before_context_change = offset_cache.stats();
  const auto cutout_reference = flux::offset_polygonal_free_space(changed_cutout, offset);
  const auto cutout_cached = flux::offset_polygonal_free_space(changed_cutout, offset, &offset_cache);
  require(same_offset_result(cutout_cached, cutout_reference),
          "cutout changes must be combined with cached obstacle expansions per request");
  const auto stats_after_context_change = offset_cache.stats();
  require(stats_after_context_change.input_validation_misses >
              stats_before_context_change.input_validation_misses &&
              stats_after_context_change.prepared_domain_misses >
                  stats_before_context_change.prepared_domain_misses &&
              stats_after_context_change.clearance_certificate_misses >
                  stats_before_context_change.clearance_certificate_misses,
          "changed cutout geometry must invalidate validation, preparation, and clearance context");
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
  flux::FreeSpaceOffsetConfig filled_hole_rule{Nanometres{100}, 1000U};
  filled_hole_rule.obstacle_clearances = {Nanometres{150}, Nanometres{300}};
  filled_hole_rule.filled_obstacle_clearances = {Nanometres{250}};
  auto changed_filled_hole = domain;
  changed_filled_hole.filled_obstacles.front().holes.front() = rectangle(3200, 3000, 7000, 7000);
  const auto filled_hole_reference =
      flux::offset_polygonal_free_space(changed_filled_hole, filled_hole_rule);
  const auto filled_hole_cached =
      flux::offset_polygonal_free_space(changed_filled_hole, filled_hole_rule, &complex_cache);
  require(same_offset_result(filled_hole_cached, filled_hole_reference),
          "changed filled-obstacle holes must not reuse stale cached expansion geometry");
  require(complex_cache.stats().retained_vertices <= 3U * 262'144U,
          "the combined validation, prepared-source, and certificate stores must remain bounded");
  require(offset_cache.stats().entries <= 2U, "offset cache must retain its capacity bound");

  // A net/domain view may exclude a remote blocker while retaining the same source polygons.
  flux::FreeSpaceDomain source_domain;
  source_domain.outline = rectangle(0, 0, 20000, 20000);
  source_domain.obstacles = {rectangle(8000, 8000, 9000, 9000),
                             rectangle(15000, 8000, 16000, 9000)};
  flux::FreeSpaceOffsetConfig local_rule{Nanometres{200}, 1000U};
  flux::FreeSpaceOffsetCache source_view_cache{16U};
  (void)flux::offset_polygonal_free_space(source_domain, local_rule, &source_view_cache);
  auto local_view = source_domain;
  local_view.obstacles.pop_back();
  const auto source_stats_before_view = source_view_cache.stats();
  const auto local_view_reference = flux::offset_polygonal_free_space(local_view, local_rule);
  const auto local_view_cached =
      flux::offset_polygonal_free_space(local_view, local_rule, &source_view_cache);
  require(same_offset_result(local_view_cached, local_view_reference) &&
              source_view_cache.stats().prepared_source_hits > source_stats_before_view.prepared_source_hits,
          "a domain view excluding a far blocker must reuse its prepared shared sources");

  // The local obstacle hole is unchanged when a remote blocker is introduced. Its certificate
  // should therefore remain reusable even though the domain-level preparation changes.
  flux::FreeSpaceOffsetCache locality_cache{16U};
  (void)flux::offset_polygonal_free_space(local_view, local_rule, &locality_cache);
  const auto locality_stats_before_distant_change = locality_cache.stats();
  const auto distant_reference = flux::offset_polygonal_free_space(source_domain, local_rule);
  const auto distant_cached =
      flux::offset_polygonal_free_space(source_domain, local_rule, &locality_cache);
  require(same_offset_result(distant_cached, distant_reference) &&
              locality_cache.stats().clearance_certificate_hits >
                  locality_stats_before_distant_change.clearance_certificate_hits,
          "an unchanged local hole must reuse its clearance certificate across a distant mutation");

  auto nearby_boundary = local_view;
  nearby_boundary.obstacles.front() = rectangle(8100, 8000, 9000, 9000);
  const auto locality_stats_before_nearby_change = locality_cache.stats();
  const auto nearby_reference = flux::offset_polygonal_free_space(nearby_boundary, local_rule);
  const auto nearby_cached =
      flux::offset_polygonal_free_space(nearby_boundary, local_rule, &locality_cache);
  require(same_offset_result(nearby_cached, nearby_reference) &&
              locality_cache.stats().clearance_certificate_misses >
                  locality_stats_before_nearby_change.clearance_certificate_misses,
          "a nearby boundary change must invalidate its local clearance certificate");
  auto local_rule_change = local_rule;
  local_rule_change.obstacle_clearances = {Nanometres{250}};
  const auto locality_stats_before_rule_change = locality_cache.stats();
  const auto rule_reference = flux::offset_polygonal_free_space(local_view, local_rule_change);
  const auto rule_cached =
      flux::offset_polygonal_free_space(local_view, local_rule_change, &locality_cache);
  require(same_offset_result(rule_cached, rule_reference) &&
              locality_cache.stats().clearance_certificate_misses >
                  locality_stats_before_rule_change.clearance_certificate_misses,
          "an effective obstacle-rule change must invalidate its clearance certificate");

  flux::FreeSpaceOffsetCache rejection_cache{4U};
  flux::FreeSpaceOffsetConfig rejection_rule{Nanometres{250}, 1000U};
  const auto valid_reference = flux::offset_polygonal_free_space(domain, rejection_rule);
  const auto valid_cached = flux::offset_polygonal_free_space(domain, rejection_rule, &rejection_cache);
  require(same_offset_result(valid_cached, valid_reference),
          "valid input must establish a cache entry before rejection checks");
  auto invalid_domain = domain;
  invalid_domain.obstacles.push_back(
      Polygon{{point(7000, 7000), point(8000, 8000), point(7000, 8000), point(8000, 7000)}});
  const auto invalid_reference = flux::offset_polygonal_free_space(invalid_domain, rejection_rule);
  const auto invalid_cached =
      flux::offset_polygonal_free_space(invalid_domain, rejection_rule, &rejection_cache);
  require(!invalid_reference.ok() && same_offset_result(invalid_cached, invalid_reference),
          "invalid geometry after a valid cache entry must retain uncached rejection semantics");
  const auto valid_after_rejection =
      flux::offset_polygonal_free_space(domain, rejection_rule, &rejection_cache);
  require(same_offset_result(valid_after_rejection, valid_reference),
          "invalid geometry must not poison a previously valid expansion");

  flux::FreeSpaceOffsetCache budget_cache{4U};
  const auto budget_reference = flux::offset_polygonal_free_space(domain, rejection_rule);
  require(budget_reference.ok() && budget_reference.output_vertices > 0U,
          "budget test requires a non-empty valid free-space result");
  (void)flux::offset_polygonal_free_space(domain, rejection_rule, &budget_cache);
  auto insufficient_budget = rejection_rule;
  insufficient_budget.maximum_output_vertices = budget_reference.output_vertices - 1U;
  const auto budget_failure_reference =
      flux::offset_polygonal_free_space(domain, insufficient_budget);
  const auto budget_failure_cached =
      flux::offset_polygonal_free_space(domain, insufficient_budget, &budget_cache);
  require(!budget_failure_reference.ok() &&
              same_offset_result(budget_failure_cached, budget_failure_reference),
          "a smaller output budget must preserve cached and uncached failure semantics");
  const auto budget_recovered = flux::offset_polygonal_free_space(domain, rejection_rule, &budget_cache);
  require(same_offset_result(budget_recovered, budget_reference),
          "budget rejection must not invalidate a reusable expansion");

  flux::FreeSpaceOffsetCache concurrent_cache{8U};
  const auto concurrent_reference = flux::offset_polygonal_free_space(domain, rejection_rule);
  std::atomic<bool> concurrent_parity{true};
  std::vector<std::thread> workers;
  for (std::size_t index = 0U; index < 4U; ++index) {
    workers.emplace_back([&] {
      for (std::size_t attempt = 0U; attempt < 3U; ++attempt) {
        const auto result = flux::offset_polygonal_free_space(domain, rejection_rule, &concurrent_cache);
        if (!same_offset_result(result, concurrent_reference)) concurrent_parity.store(false);
      }
    });
  }
  for (auto& worker : workers) worker.join();
  require(concurrent_parity.load() && concurrent_cache.stats().entries <= 8U,
          "bounded concurrent expansion reuse must preserve uncached geometry");

}
