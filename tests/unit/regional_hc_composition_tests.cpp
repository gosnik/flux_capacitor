#include "flux/core/regional_hc_composition.hpp"

#include <cstdlib>
#include <iostream>

namespace {
using namespace flux;
[[noreturn]] void fail(const char* message) {
  std::cerr << message << '\n';
  std::exit(1);
}
void require(bool value, const char* message) {
  if (!value)
    fail(message);
}

RegionalHcSemanticContext context(std::string scene = "scene") {
  return {nullptr,  std::move(scene), "rules",   "endpoints",
          "access", "compatibility",  "options", {"pair-ab-geometry", "pair-cd-geometry"}};
}
std::vector<RegionalHcChildGroup> groups() {
  // Each alternative is a locally legal assignment for two regional variables (A/B and C/D).
  return {{"pair-a-b", {{"a0-b0-preferred", 1U}, {"a1-b0-alternative", 2U}}},
          {"pair-c-d", {{"c0-d0-preferred", 1U}, {"c1-d0-alternative", 2U}}}};
}
RegionalHcCompatibility conflict_preferred = [](const auto& a, const auto& b) {
  return !(a.identity == "a0-b0-preferred" && b.identity == "c0-d0-preferred");
};
} // namespace

int main() {
  RegionalHcCompositionBounds bounds;
  bounds.max_nodes = 32U;
  bounds.max_frontier = 32U;
  bounds.max_retained_per_group = 2U;
  bounds.max_results = 4U;
  RegionalHcCompositionCache cache;
  const auto first =
      compose_regional_hc_assignments(context(), groups(), bounds, conflict_preferred, &cache);
  require(
      !first.assignments.empty() && first.assignments.front()[0] == 0U &&
          first.assignments.front()[1] == 1U,
      "two locally valid two-variable children must retain and compose a compatible alternative");
  const auto unchanged =
      compose_regional_hc_assignments(context(), groups(), bounds, conflict_preferred, &cache);
  require(unchanged.stats.cache_disposition == RegionalHcCacheDisposition::exact_reused &&
              unchanged.assignments == first.assignments,
          "an exact semantic context must reuse stable child assignments deterministically");
  const auto stale = compose_regional_hc_assignments(
      context("changed-copper"), groups(), bounds, [](const auto&, const auto&) { return false; },
      &cache);
  require(stale.stats.cache_disposition == RegionalHcCacheDisposition::stale_rebuilt &&
              stale.stats.stale_candidates_rejected != 0U,
          "changed copper must revalidate stale compatibility rather than trust it");
  auto changed_endpoints = context("changed-copper");
  changed_endpoints.terminal_geometry = "changed-endpoints";
  const auto invalidated = compose_regional_hc_assignments(changed_endpoints, groups(), bounds,
                                                           conflict_preferred, &cache);
  require(invalidated.stats.cache_disposition == RegionalHcCacheDisposition::invalidated,
          "endpoint or rule changes must invalidate retained static composition states");
  auto board_scene = std::make_shared<Board>();
  board_scene->rules.default_clearance = Nanometres{100};
  auto board_context = context("board-rules");
  board_context.board_scene = board_scene;
  static_cast<void>(
      compose_regional_hc_assignments(board_context, groups(), bounds, conflict_preferred, &cache));
  auto changed_rules = board_context;
  auto mutable_changed_board = std::make_shared<Board>(*board_scene);
  mutable_changed_board->rules.default_clearance = Nanometres{101};
  changed_rules.board_scene = std::move(mutable_changed_board);
  const auto board_invalidated =
      compose_regional_hc_assignments(changed_rules, groups(), bounds, conflict_preferred, &cache);
  require(board_invalidated.stats.cache_disposition == RegionalHcCacheDisposition::invalidated,
          "changed canonical Board rules must invalidate stale composition state");
  RegionalHcCompositionCache tiny_cache;
  auto oversized = context("oversized");
  auto oversized_board = std::make_shared<Board>();
  oversized_board->layers.push_back({LayerId{std::string(4096, 'L')}, std::string(4096, 'N')});
  oversized.board_scene = oversized_board;
  auto tiny_bounds = bounds;
  tiny_bounds.max_cache_bytes = 128U;
  static_cast<void>(compose_regional_hc_assignments(oversized, groups(), tiny_bounds,
                                                    conflict_preferred, &tiny_cache));
  const auto not_retained = compose_regional_hc_assignments(oversized, groups(), tiny_bounds,
                                                            conflict_preferred, &tiny_cache);
  require(not_retained.stats.cache_hits == 0U,
          "oversized nested board identity must not enter the bounded retained cache");
  auto tiny = bounds;
  tiny.max_nodes = 1U;
  const auto unknown =
      compose_regional_hc_assignments(context("bounded"), groups(), tiny, conflict_preferred);
  require(unknown.outcome == RegionalHcCompositionOutcome::unknown && unknown.stats.node_bound_hit,
          "a hard composition bound must report unknown, never infeasible");
  const auto repeat = compose_regional_hc_assignments(context("deterministic"), groups(), bounds,
                                                      conflict_preferred);
  const auto repeat_again = compose_regional_hc_assignments(context("deterministic"), groups(),
                                                            bounds, conflict_preferred);
  require(repeat.assignments == repeat_again.assignments,
          "composition ordering must be deterministic");
  const std::vector<RegionalHcChildGroup> unsorted{
      {"left", {{"slow", 9U}, {"dup", 8U}, {"dup", 1U}}}, {"right", {{"right", 1U}}}};
  const auto mapped = compose_regional_hc_assignments(
      context("mapping"), unsorted, bounds, [](const auto&, const auto&) { return true; });
  require(!mapped.assignments.empty() && mapped.assignments.front()[0] == 2U,
          "sorted/deduplicated alternatives must return their original input index");
  RegionalHcCompositionCache mapping_cache;
  auto single_result = bounds;
  single_result.max_results = 1U;
  const auto compatible = [](const auto&, const auto&) { return true; };
  static_cast<void>(compose_regional_hc_assignments(context("mapping-before"), unsorted,
                                                    single_result, compatible, &mapping_cache));
  const auto salvaged = compose_regional_hc_assignments(context("mapping-after"), unsorted,
                                                        single_result, compatible, &mapping_cache);
  require(salvaged.stats.cache_disposition == RegionalHcCacheDisposition::stale_revalidated &&
              salvaged.assignments.size() == 1U && salvaged.assignments.front()[0] == 2U,
          "successful stale-cache early return must preserve original alternative indices");
}
