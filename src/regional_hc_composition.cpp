#include "flux/core/regional_hc_composition.hpp"
#include "flux/core/routing_kernel.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <queue>
#include <tuple>
#include <type_traits>

namespace flux {

namespace {
[[nodiscard]] std::size_t scene_add(std::size_t total, std::size_t add, std::size_t limit) {
  return total > limit || add > limit - total ? limit + 1U : total + add;
}
[[nodiscard]] std::size_t scene_string(const std::string& value) {
  return sizeof(std::string) + value.capacity();
}
[[nodiscard]] std::size_t scene_polygon(const Polygon& value, std::size_t limit) {
  return scene_add(sizeof(Polygon), value.vertices.capacity() * sizeof(Point), limit);
}
template <class Id> [[nodiscard]] std::size_t scene_id(const Id& id) {
  return scene_string(id.value());
}
[[nodiscard]] std::size_t scene_keepout(const Keepout& value, std::size_t limit) {
  auto total = scene_add(sizeof(Keepout), scene_id(value.id), limit);
  total = scene_add(total, scene_polygon(value.area, limit), limit);
  total = scene_add(total, value.layers.capacity() * sizeof(LayerId), limit);
  for (const auto& layer : value.layers)
    total = scene_add(total, scene_id(layer), limit);
  return scene_add(total, scene_string(value.name), limit);
}
} // namespace

std::size_t estimate_regional_hc_scene_bytes(const Board& board, const std::size_t limit) {
  auto total = scene_add(sizeof(Board), scene_string(board.source_revision), limit);
  total = scene_add(total, scene_polygon(board.outline, limit), limit);
  for (const auto& value : board.cutouts)
    total = scene_add(total, scene_polygon(value, limit), limit);
  total = scene_add(total, board.cutouts.capacity() * sizeof(Polygon), limit);
  for (const auto& value : board.layers)
    total = scene_add(total, scene_id(value.id) + scene_string(value.name), limit);
  total = scene_add(total, board.layers.capacity() * sizeof(Layer), limit);
  for (const auto& value : board.nets) {
    total = scene_add(total,
                      scene_id(value.id) + scene_string(value.name) +
                          value.classes.capacity() * sizeof(NetClassId),
                      limit);
    for (const auto& klass : value.classes)
      total = scene_add(total, scene_id(klass), limit);
  }
  total = scene_add(total, board.nets.capacity() * sizeof(Net), limit);
  for (const auto& value : board.net_classes)
    total = scene_add(total, scene_id(value.id) + scene_string(value.name), limit);
  total = scene_add(total, board.net_classes.capacity() * sizeof(NetClass), limit);
  for (const auto& value : board.differential_pairs)
    total = scene_add(total,
                      scene_id(value.id) + scene_string(value.name) + scene_id(value.positive_net) +
                          scene_id(value.negative_net),
                      limit);
  for (const auto& value : board.routing_buses) {
    total = scene_add(total,
                      scene_id(value.id) + scene_string(value.name) +
                          value.ordered_nets.capacity() * sizeof(NetId),
                      limit);
    for (const auto& net : value.ordered_nets)
      total = scene_add(total, scene_id(net), limit);
  }
  for (const auto& value : board.placement_groups) {
    total = scene_add(total,
                      scene_id(value.id) + scene_string(value.name) +
                          value.footprints.capacity() * sizeof(ObjectId),
                      limit);
    for (const auto& footprint : value.footprints)
      total = scene_add(total, scene_id(footprint), limit);
  }
  for (const auto& value : board.placement_regions) {
    total =
        scene_add(total,
                  scene_id(value.id) + scene_string(value.name) + scene_polygon(value.area, limit) +
                      value.footprints.capacity() * sizeof(ObjectId),
                  limit);
    for (const auto& footprint : value.footprints)
      total = scene_add(total, scene_id(footprint), limit);
  }
  for (const auto& value : board.relative_placement_constraints)
    total = scene_add(total,
                      scene_id(value.id) + scene_string(value.name) +
                          scene_id(value.first_footprint) + scene_id(value.second_footprint),
                      limit);
  if (board.stackup) {
    total = scene_add(total,
                      scene_string(board.stackup->copper_finish) +
                          board.stackup->layers.capacity() * sizeof(StackupLayer),
                      limit);
    for (const auto& layer : board.stackup->layers) {
      total = scene_add(total,
                        scene_string(layer.name) + scene_string(layer.type) +
                            layer.sublayers.capacity() * sizeof(StackupSublayer),
                        limit);
      if (layer.board_layer)
        total = scene_add(total, scene_id(*layer.board_layer), limit);
      for (const auto& sublayer : layer.sublayers)
        total =
            scene_add(total, scene_string(sublayer.material) + scene_string(sublayer.color), limit);
    }
  }
  for (const auto& value : board.footprints) {
    total = scene_add(
        total,
        scene_id(value.id) + scene_string(value.reference) + scene_polygon(value.courtyard, limit) +
            value.courtyard_circles.capacity() * sizeof(CourtyardCircle) +
            value.orientation_constraints.allowed_rotations.capacity() * sizeof(Microdegrees),
        limit);
    for (const auto& pad : value.pads) {
      total = scene_add(
          total, scene_id(pad.id) + scene_id(pad.net) + pad.layers.capacity() * sizeof(LayerId),
          limit);
      for (const auto& layer : pad.layers)
        total = scene_add(total, scene_id(layer), limit);
    }
    total = scene_add(total, value.pads.capacity() * sizeof(Pad), limit);
    for (const auto& keepout : value.keepouts)
      total = scene_add(total, scene_keepout(keepout, limit), limit);
    total = scene_add(total, value.keepouts.capacity() * sizeof(Keepout), limit);
  }
  total = scene_add(total, board.footprints.capacity() * sizeof(Footprint), limit);
  const auto copper = [&](const auto& values) {
    for (const auto& value : values)
      total =
          scene_add(total, scene_id(value.id) + scene_id(value.net) + scene_id(value.layer), limit);
    total = scene_add(
        total, values.capacity() * sizeof(typename std::decay_t<decltype(values)>::value_type),
        limit);
  };
  copper(board.tracks);
  copper(board.track_arcs);
  copper(board.track_beziers);
  for (const auto& via : board.vias) {
    total = scene_add(
        total, scene_id(via.id) + scene_id(via.net) + via.layers.capacity() * sizeof(LayerId),
        limit);
    for (const auto& layer : via.layers)
      total = scene_add(total, scene_id(layer), limit);
  }
  total = scene_add(total, board.vias.capacity() * sizeof(Via), limit);
  for (const auto& zone : board.zones) {
    total =
        scene_add(total, scene_id(zone.id) + scene_id(zone.net) + scene_string(zone.name), limit);
    for (const auto& fill : zone.fills) {
      total = scene_add(total, scene_id(fill.layer) + scene_polygon(fill.area, limit), limit);
      for (const auto& hole : fill.holes)
        total = scene_add(total, scene_polygon(hole, limit), limit);
      total = scene_add(total, fill.holes.capacity() * sizeof(Polygon), limit);
    }
    total = scene_add(total, zone.fills.capacity() * sizeof(CopperZoneFill), limit);
  }
  total = scene_add(total, board.zones.capacity() * sizeof(CopperZone), limit);
  for (const auto& value : board.keepouts)
    total = scene_add(total, scene_keepout(value, limit), limit);
  total = scene_add(total, board.keepouts.capacity() * sizeof(Keepout), limit);
  // Less geometry-dense metadata still owns vector/string allocations; charge each retained
  // element by its full object size so the estimate remains conservative for cache admission.
  total = scene_add(total,
                    board.differential_pairs.capacity() * sizeof(DifferentialPair) +
                        board.routing_buses.capacity() * sizeof(RoutingBus) +
                        board.placement_groups.capacity() * sizeof(PlacementGroup) +
                        board.placement_regions.capacity() * sizeof(PlacementRegion) +
                        board.relative_placement_constraints.capacity() *
                            sizeof(RelativePlacementConstraint),
                    limit);
  return total;
}

struct RegionalHcCompositionCache::Entry {
  std::shared_ptr<const RegionalHcSemanticContext> context;
  std::vector<std::vector<std::string>> assignments;
  std::size_t bytes{0U};
};

RegionalHcCompositionCache::RegionalHcCompositionCache() = default;
RegionalHcCompositionCache::~RegionalHcCompositionCache() = default;

bool RegionalHcSemanticContext::operator==(const RegionalHcSemanticContext& other) const {
  const auto board_equal = board_scene == other.board_scene ||
                           (board_scene && other.board_scene && *board_scene == *other.board_scene);
  return board_equal && board_copper == other.board_copper && rules == other.rules &&
         terminal_geometry == other.terminal_geometry &&
         access_constraints == other.access_constraints &&
         compatibility_constraints == other.compatibility_constraints && options == other.options &&
         candidate_geometry == other.candidate_geometry;
}

bool RegionalHcSemanticContext::same_candidate_geometry(
    const RegionalHcSemanticContext& other) const {
  return candidate_geometry == other.candidate_geometry;
}

bool RegionalHcSemanticContext::same_endpoints_and_rules(
    const RegionalHcSemanticContext& other) const {
  const auto board_rules_equal =
      static_cast<bool>(board_scene) == static_cast<bool>(other.board_scene) &&
      (!board_scene || (board_scene->rules == other.board_scene->rules &&
                        board_scene->net_classes == other.board_scene->net_classes &&
                        board_scene->nets == other.board_scene->nets &&
                        board_scene->differential_pairs == other.board_scene->differential_pairs &&
                        board_scene->routing_buses == other.board_scene->routing_buses));
  return board_rules_equal && rules == other.rules &&
         terminal_geometry == other.terminal_geometry &&
         access_constraints == other.access_constraints &&
         compatibility_constraints == other.compatibility_constraints && options == other.options;
}

namespace {

using IndexRow = std::vector<std::size_t>;

[[nodiscard]] std::vector<RegionalHcChildGroup>
normalized_groups(const std::vector<RegionalHcChildGroup>& groups,
                  const RegionalHcCompositionBounds& bounds, RegionalHcCompositionStats& stats,
                  std::vector<std::vector<std::size_t>>& original_indices) {
  auto copy = groups;
  original_indices.resize(copy.size());
  for (std::size_t group_index = 0; group_index < copy.size(); ++group_index) {
    auto& group = copy[group_index];
    std::vector<std::size_t> ordering(group.alternatives.size());
    for (std::size_t index = 0; index < ordering.size(); ++index)
      ordering[index] = index;
    std::stable_sort(ordering.begin(), ordering.end(), [&](const auto lhs, const auto rhs) {
      const auto& a = group.alternatives[lhs];
      const auto& b = group.alternatives[rhs];
      return std::tie(a.cost, a.identity, lhs) < std::tie(b.cost, b.identity, rhs);
    });
    std::vector<RegionalHcAlternative> ordered;
    ordered.reserve(group.alternatives.size());
    auto& indices = original_indices[group_index];
    for (const auto index : ordering) {
      if (!ordered.empty() && ordered.back().identity == group.alternatives[index].identity)
        continue;
      ordered.push_back(group.alternatives[index]);
      indices.push_back(index);
    }
    group.alternatives = std::move(ordered);
    if (group.alternatives.size() > bounds.max_retained_per_group) {
      stats.child_bound_hit = true;
      group.alternatives.resize(bounds.max_retained_per_group);
    }
    if (indices.size() > bounds.max_retained_per_group)
      indices.resize(bounds.max_retained_per_group);
    stats.retained_children += group.alternatives.size();
  }
  return copy;
}

[[nodiscard]] std::vector<std::size_t>
interaction_order(const std::vector<RegionalHcChildGroup>& groups,
                  const RegionalHcCompatibility& compatible, RegionalHcCompositionStats& stats,
                  const RegionalHcCompositionBounds& bounds, const std::stop_token stop_token) {
  const auto count = groups.size();
  std::vector<std::vector<std::size_t>> strength(count, std::vector<std::size_t>(count));
  for (std::size_t lhs = 0; lhs < count; ++lhs)
    for (std::size_t rhs = lhs + 1; rhs < count; ++rhs) {
      std::size_t conflicts = 0U;
      for (const auto& a : groups[lhs].alternatives)
        for (const auto& b : groups[rhs].alternatives) {
          routing_kernel_checkpoint();
          if (stop_token.stop_requested()) {
            stats.cancelled = true;
            return {};
          }
          if (stats.compatibility_checks >= bounds.max_compatibility_checks) {
            stats.compatibility_bound_hit = true;
            return {};
          }
          ++stats.compatibility_checks;
          if (!compatible(a, b))
            ++conflicts;
        }
      if (conflicts != 0U)
        ++stats.interaction_edges;
      strength[lhs][rhs] = strength[rhs][lhs] = conflicts;
    }
  std::vector<std::size_t> order;
  std::vector<bool> chosen(count, false);
  for (std::size_t step = 0; step < count; ++step) {
    std::size_t best = count;
    std::size_t best_strength = 0U;
    for (std::size_t candidate = 0; candidate < count; ++candidate)
      if (!chosen[candidate]) {
        std::size_t score = 0U;
        if (order.empty())
          for (const auto value : strength[candidate])
            score += value;
        else
          for (const auto prior : order)
            score += strength[candidate][prior];
        if (best == count || score > best_strength ||
            (score == best_strength && candidate < best)) {
          best = candidate;
          best_strength = score;
        }
      }
    chosen[best] = true;
    order.push_back(best);
  }
  return order;
}

[[nodiscard]] bool
rows_compatible(const IndexRow& row, const std::vector<RegionalHcChildGroup>& groups,
                const RegionalHcCompatibility& compatible, RegionalHcCompositionStats& stats,
                const RegionalHcCompositionBounds& bounds, const std::stop_token stop_token) {
  for (std::size_t lhs = 0; lhs < row.size(); ++lhs)
    if (row[lhs] != std::numeric_limits<std::size_t>::max())
      for (std::size_t rhs = lhs + 1; rhs < row.size(); ++rhs)
        if (row[rhs] != std::numeric_limits<std::size_t>::max()) {
          routing_kernel_checkpoint();
          if (stop_token.stop_requested()) {
            stats.cancelled = true;
            return false;
          }
          if (stats.compatibility_checks >= bounds.max_compatibility_checks) {
            stats.compatibility_bound_hit = true;
            return false;
          }
          ++stats.compatibility_checks;
          if (!compatible(groups[lhs].alternatives[row[lhs]], groups[rhs].alternatives[row[rhs]])) {
            ++stats.compatibility_rejections;
            return false;
          }
        }
  return true;
}

[[nodiscard]] std::size_t context_bytes(const RegionalHcSemanticContext& context) {
  std::size_t total =
      sizeof(RegionalHcSemanticContext) +
      (context.board_scene ? estimate_regional_hc_scene_bytes(
                                 *context.board_scene, std::numeric_limits<std::size_t>::max() / 2U)
                           : 0U) +
      context.board_copper.capacity() + context.rules.capacity() +
      context.terminal_geometry.capacity() + context.access_constraints.capacity() +
      context.compatibility_constraints.capacity() + context.options.capacity() +
      context.candidate_geometry.capacity() * sizeof(std::string);
  for (const auto& value : context.candidate_geometry)
    total += value.capacity();
  return total;
}

[[nodiscard]] std::size_t cached_bytes(const RegionalHcSemanticContext& context,
                                       const std::vector<std::vector<std::string>>& rows) {
  auto total = context_bytes(context);
  total += rows.capacity() * sizeof(std::vector<std::string>);
  for (const auto& row : rows) {
    total += row.capacity() * sizeof(std::string);
    for (const auto& value : row)
      total += value.capacity();
  }
  total += sizeof(std::shared_ptr<const RegionalHcSemanticContext>) +
           sizeof(std::vector<std::vector<std::string>>) + sizeof(std::size_t) + 4U * sizeof(void*);
  return total;
}

[[nodiscard]] std::vector<IndexRow>
external_rows(const std::vector<IndexRow>& rows,
              const std::vector<std::vector<std::size_t>>& originals) {
  auto output = rows;
  for (auto& row : output)
    for (std::size_t group = 0; group < row.size(); ++group)
      row[group] = originals[group][row[group]];
  return output;
}

[[nodiscard]] std::vector<IndexRow>
resolve_cached_rows(const std::vector<std::vector<std::string>>& cached,
                    const std::vector<RegionalHcChildGroup>& groups) {
  std::vector<IndexRow> rows;
  for (const auto& saved : cached) {
    if (saved.size() != groups.size())
      continue;
    IndexRow row(groups.size(), std::numeric_limits<std::size_t>::max());
    bool found = true;
    for (std::size_t group = 0; group < groups.size(); ++group) {
      const auto it = std::find_if(
          groups[group].alternatives.begin(), groups[group].alternatives.end(),
          [&](const auto& alternative) { return alternative.identity == saved[group]; });
      if (it == groups[group].alternatives.end()) {
        found = false;
        break;
      }
      row[group] = static_cast<std::size_t>(it - groups[group].alternatives.begin());
    }
    if (found)
      rows.push_back(std::move(row));
  }
  return rows;
}

} // namespace

RegionalHcCompositionResult compose_regional_hc_assignments(
    const RegionalHcSemanticContext& context, const std::vector<RegionalHcChildGroup>& input_groups,
    const RegionalHcCompositionBounds& bounds, const RegionalHcCompatibility& compatible,
    RegionalHcCompositionCache* cache, const std::stop_token stop_token) {
  RegionalHcCompositionResult result;
  if (stop_token.stop_requested()) {
    result.stats.cancelled = true;
    return result;
  }
  if (!compatible || input_groups.empty() || bounds.max_nodes == 0U || bounds.max_frontier == 0U ||
      bounds.max_retained_per_group == 0U || bounds.max_results == 0U)
    return result;
  std::size_t input_count = 0U;
  std::size_t input_bytes = 0U;
  for (const auto& group : input_groups)
    for (const auto& alternative : group.alternatives) {
      ++input_count;
      input_bytes += alternative.identity.size();
    }
  if (input_groups.size() > bounds.max_input_groups ||
      input_count > bounds.max_input_alternatives ||
      input_bytes > bounds.max_candidate_identity_bytes ||
      context_bytes(context) > bounds.max_cache_bytes) {
    result.stats.memory_bound_hit = true;
    return result;
  }
  std::vector<std::vector<std::size_t>> original_indices;
  auto groups = normalized_groups(input_groups, bounds, result.stats, original_indices);
  const auto expose_rows = [&]() {
    result.assignments = external_rows(result.assignments, original_indices);
  };
  if (std::any_of(groups.begin(), groups.end(),
                  [](const auto& group) { return group.alternatives.empty(); })) {
    result.outcome = RegionalHcCompositionOutcome::infeasible;
    return result;
  }

  std::vector<IndexRow> cached_rows;
  bool stale = false;
  if (cache != nullptr) {
    std::lock_guard lock(cache->mutex_);
    const auto exact = std::find_if(cache->entries_.begin(), cache->entries_.end(),
                                    [&](const auto& entry) { return *entry.context == context; });
    if (exact != cache->entries_.end()) {
      cached_rows = resolve_cached_rows(exact->assignments, groups);
      result.stats.cache_hits += cached_rows.size();
      result.stats.cache_disposition = RegionalHcCacheDisposition::exact_reused;
    } else {
      const auto salvage =
          std::find_if(cache->entries_.begin(), cache->entries_.end(), [&](const auto& entry) {
            return entry.context->same_candidate_geometry(context) &&
                   entry.context->same_endpoints_and_rules(context);
          });
      if (salvage != cache->entries_.end()) {
        cached_rows = resolve_cached_rows(salvage->assignments, groups);
        stale = true;
        result.stats.cache_disposition = RegionalHcCacheDisposition::stale_revalidated;
      } else {
        result.stats.cache_misses = 1U;
        result.stats.cache_disposition = RegionalHcCacheDisposition::invalidated;
      }
    }
  }
  for (const auto& row : cached_rows) {
    if (stale)
      ++result.stats.stale_candidates_seen;
    if (result.assignments.size() >= bounds.max_results)
      break;
    if (rows_compatible(row, groups, compatible, result.stats, bounds, stop_token)) {
      if (std::find(result.assignments.begin(), result.assignments.end(), row) ==
          result.assignments.end())
        result.assignments.push_back(row);
      if (stale)
        ++result.stats.stale_candidates_revalidated;
    } else if (stale) {
      ++result.stats.stale_candidates_rejected;
    }
  }
  // Cached rows are always revalidated against the current callback. Downstream exact packing
  // and copper gates still run for every invocation.
  if (result.stats.cancelled || result.stats.compatibility_bound_hit) {
    expose_rows();
    return result;
  }
  if (!stale && !cached_rows.empty() && result.assignments.size() >= bounds.max_results) {
    result.outcome = RegionalHcCompositionOutcome::complete;
    expose_rows();
    return result;
  }
  // Rebuild after stale evidence whose revalidation is poor. Good stale rows seed the result,
  // but the fresh bounded search still fills its portfolio. This decision depends on observed
  // stale rejection work, not a hash or fixed cache age.
  const bool rebuild =
      !stale || result.stats.stale_candidates_seen == 0U ||
      result.stats.stale_candidates_rejected * 2U >= result.stats.stale_candidates_seen;
  if (stale)
    result.stats.cache_disposition = rebuild ? RegionalHcCacheDisposition::stale_rebuilt
                                             : RegionalHcCacheDisposition::stale_revalidated;
  if (!rebuild && result.assignments.size() >= bounds.max_results) {
    result.outcome = RegionalHcCompositionOutcome::complete;
    expose_rows();
    return result;
  }

  const auto order = interaction_order(groups, compatible, result.stats, bounds, stop_token);
  if (result.stats.cancelled || result.stats.compatibility_bound_hit) {
    expose_rows();
    return result;
  }
  struct State {
    std::uint64_t cost;
    std::size_t depth;
    IndexRow row;
  };
  const auto less = [](const State& lhs, const State& rhs) {
    return std::tie(lhs.cost, lhs.row) > std::tie(rhs.cost, rhs.row);
  };
  std::priority_queue<State, std::vector<State>, decltype(less)> frontier(less);
  IndexRow empty(groups.size(), std::numeric_limits<std::size_t>::max());
  frontier.push({0U, 0U, empty});
  while (!frontier.empty() && result.assignments.size() < bounds.max_results) {
    routing_kernel_checkpoint();
    if (result.stats.nodes_expanded >= bounds.max_nodes) {
      result.stats.node_bound_hit = true;
      break;
    }
    auto state = frontier.top();
    frontier.pop();
    ++result.stats.nodes_expanded;
    if (state.depth == order.size()) {
      if (std::find(result.assignments.begin(), result.assignments.end(), state.row) ==
          result.assignments.end())
        result.assignments.push_back(std::move(state.row));
      continue;
    }
    const auto group_index = order[state.depth];
    for (std::size_t alternative = 0; alternative < groups[group_index].alternatives.size();
         ++alternative) {
      auto next = state;
      next.depth++;
      next.row[group_index] = alternative;
      if (!rows_compatible(next.row, groups, compatible, result.stats, bounds, stop_token))
        continue;
      if (result.stats.cancelled || result.stats.compatibility_bound_hit)
        break;
      if (frontier.size() >= bounds.max_frontier) {
        result.stats.frontier_bound_hit = true;
        break;
      }
      if (next.cost > std::numeric_limits<std::uint64_t>::max() -
                          groups[group_index].alternatives[alternative].cost) {
        result.stats.memory_bound_hit = true;
        continue;
      }
      next.cost += groups[group_index].alternatives[alternative].cost;
      frontier.push(std::move(next));
      result.stats.frontier_peak = std::max(result.stats.frontier_peak, frontier.size());
    }
    if (result.stats.frontier_bound_hit || result.stats.cancelled ||
        result.stats.compatibility_bound_hit)
      break;
  }
  const bool bounded = result.stats.node_bound_hit || result.stats.frontier_bound_hit ||
                       result.stats.memory_bound_hit || result.stats.child_bound_hit ||
                       result.stats.compatibility_bound_hit || result.stats.cancelled;
  result.outcome = bounded ? RegionalHcCompositionOutcome::unknown
                           : (result.assignments.empty() ? RegionalHcCompositionOutcome::infeasible
                                                         : RegionalHcCompositionOutcome::complete);
  if (cache != nullptr && !result.assignments.empty()) {
    std::vector<std::vector<std::string>> saved;
    for (const auto& row : result.assignments) {
      std::vector<std::string> identities;
      for (std::size_t group = 0; group < row.size(); ++group)
        identities.push_back(groups[group].alternatives[row[group]].identity);
      saved.push_back(std::move(identities));
    }
    std::lock_guard lock(cache->mutex_);
    const auto bytes = cached_bytes(context, saved);
    if (bytes <= bounds.max_cache_bytes) {
      cache->entries_.erase(
          std::remove_if(cache->entries_.begin(), cache->entries_.end(),
                         [&](const auto& entry) { return *entry.context == context; }),
          cache->entries_.end());
      cache->entries_.push_back(
          {std::make_shared<RegionalHcSemanticContext>(context), std::move(saved), bytes});
      std::size_t total = 0U;
      for (const auto& entry : cache->entries_)
        total += entry.bytes;
      while (!cache->entries_.empty() && (cache->entries_.size() > bounds.max_cache_entries ||
                                          total > bounds.max_cache_bytes)) {
        total -= cache->entries_.front().bytes;
        cache->entries_.erase(cache->entries_.begin());
      }
    }
  }
  expose_rows();
  return result;
}

} // namespace flux
