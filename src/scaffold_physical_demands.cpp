#include "scaffold_physical_demands.hpp"

#include "flux/core/connectivity.hpp"
#include "flux/core/copper_geometry.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <tuple>

namespace flux {
namespace {

struct PadKey {
  std::string id;
  std::int64_t x{0};
  std::int64_t y{0};

  auto tie() const { return std::tie(id, x, y); }
  bool operator<(const PadKey& other) const { return tie() < other.tie(); }
};

[[nodiscard]] std::vector<LayerId> legal_layers(const CopperPadShape& pad,
                                                 const std::set<LayerId>& copper_layers) {
  std::vector<LayerId> result;
  for (const auto& layer : pad.layers)
    if (copper_layers.contains(layer)) result.push_back(layer);
  std::sort(result.begin(), result.end(), [](const auto& lhs, const auto& rhs) {
    return lhs.value() < rhs.value();
  });
  result.erase(std::unique(result.begin(), result.end()), result.end());
  return result;
}

[[nodiscard]] PadKey key_for(const CopperPadShape& pad) {
  return PadKey{pad.id.value(), pad.position.x.value(), pad.position.y.value()};
}

[[nodiscard]] std::pair<LayerId, LayerId> pair_layers(const CopperPadShape& lhs,
                                                        const CopperPadShape& rhs,
                                                        const std::set<LayerId>& copper_layers) {
  const auto left = legal_layers(lhs, copper_layers);
  const auto right = legal_layers(rhs, copper_layers);
  for (const auto& layer : left)
    if (std::find(right.begin(), right.end(), layer) != right.end()) return {layer, layer};
  return {left.front(), right.front()};
}

[[nodiscard]] long double squared_distance(const CopperPadShape& lhs, const CopperPadShape& rhs) {
  const auto dx = static_cast<long double>(lhs.position.x.value()) - rhs.position.x.value();
  const auto dy = static_cast<long double>(lhs.position.y.value()) - rhs.position.y.value();
  return dx * dx + dy * dy;
}

struct Component {
  std::size_t id{0U};
  std::vector<const CopperPadShape*> pads;
  PadKey representative;
};

struct Edge {
  std::size_t left{0U};
  std::size_t right{0U};
  const CopperPadShape* left_pad{nullptr};
  const CopperPadShape* right_pad{nullptr};
  long double distance{0.0L};
  PadKey first;
  PadKey second;
};

[[nodiscard]] bool edge_less(const Edge& lhs, const Edge& rhs) {
  return std::tie(lhs.distance, lhs.first, lhs.second, lhs.left, lhs.right) <
         std::tie(rhs.distance, rhs.first, rhs.second, rhs.left, rhs.right);
}

using PadComponentKey = std::pair<NetId, ObjectId>;

[[nodiscard]] std::optional<std::map<PadComponentKey, std::size_t>>
pad_component_map(const ConnectivityReport& connectivity) {
  std::map<PadComponentKey, std::size_t> result;
  for (const auto& item : connectivity.pad_components)
    if (!result.emplace(PadComponentKey{item.net, item.pad}, item.component).second) return std::nullopt;
  return result;
}

} // namespace

std::vector<GlobalRoutePath>
append_residual_physical_demands(const CopperScaffoldResult& retained,
                                 std::size_t maximum_pad_pair_checks) {
  auto result = retained.requests;
  if (!retained.complete || !retained.missing_indices.empty()) return result;

  std::set<NetId> represented;
  std::map<NetId, std::size_t> next_index;
  for (const auto& request : result) {
    represented.insert(request.net);
    const auto next = request.connection_index == std::numeric_limits<std::size_t>::max()
                          ? request.connection_index
                          : request.connection_index + 1U;
    auto& current = next_index[request.net];
    current = std::max(current, next);
  }
  if (represented.empty()) return result;

  const auto pads = copper_pad_shapes(retained.copper);
  const auto connectivity = analyze_connectivity(retained.copper);
  std::map<std::pair<NetId, ObjectId>, std::size_t> pad_components;
  for (const auto& item : connectivity.pad_components)
    pad_components.emplace(std::pair{item.net, item.pad}, item.component);

  std::map<NetId, std::string> named_nets;
  for (const auto& net : retained.copper.nets) named_nets.emplace(net.id, net.name);
  std::set<LayerId> copper_layers;
  for (const auto& layer : retained.copper.layers)
    if (layer.kind == LayerKind::copper) copper_layers.insert(layer.id);
  if (copper_layers.empty()) return result;

  std::size_t pair_checks = 0U;
  for (const auto& net : represented) {
    const auto named = named_nets.find(net);
    if (named == named_nets.end() || named->second.empty()) continue;
    if (next_index[net] == std::numeric_limits<std::size_t>::max()) continue;

    std::map<std::size_t, std::vector<const CopperPadShape*>> groups;
    bool complete_metadata = true;
    for (const auto& pad : pads) {
      if (pad.net != net) continue;
      const auto component = pad_components.find(std::pair{net, pad.id});
      if (legal_layers(pad, copper_layers).empty() || component == pad_components.end()) {
        complete_metadata = false;
        break;
      }
      groups[component->second].push_back(&pad);
    }
    if (!complete_metadata || groups.size() < 2U) continue;

    std::vector<Component> components;
    components.reserve(groups.size());
    for (auto& [id, group] : groups) {
      std::sort(group.begin(), group.end(), [](const auto* lhs, const auto* rhs) {
        return key_for(*lhs) < key_for(*rhs);
      });
      const auto representative = key_for(*group.front());
      components.push_back({id, std::move(group), representative});
    }
    std::sort(components.begin(), components.end(), [](const auto& lhs, const auto& rhs) {
      return lhs.representative < rhs.representative;
    });

    // Prim's frontier stores only the best attachment for each unselected component.
    // Each component pair is inspected once, avoiding a quadratic edge table and cubic scans.
    std::vector<bool> selected(components.size(), false);
    std::vector<std::optional<Edge>> frontier(components.size());
    const auto update_frontier = [&](std::size_t from) {
      for (std::size_t to = 0U; to < components.size(); ++to) {
        if (selected[to]) continue;
        const auto left = std::min(from, to), right = std::max(from, to);
        for (const auto* lhs : components[left].pads) for (const auto* rhs : components[right].pads) {
          if (pair_checks >= maximum_pad_pair_checks) return false;
          ++pair_checks;
          const auto lhs_key = key_for(*lhs), rhs_key = key_for(*rhs);
          Edge candidate{left, right, lhs, rhs, squared_distance(*lhs, *rhs),
                         std::min(lhs_key, rhs_key), std::max(lhs_key, rhs_key)};
          if (!frontier[to] || edge_less(candidate, *frontier[to])) frontier[to] = std::move(candidate);
        }
      }
      return true;
    };
    selected.front() = true;
    if (!update_frontier(0U)) return result;
    const auto edge_count = components.size() - 1U;
    if (next_index[net] > std::numeric_limits<std::size_t>::max() - edge_count) continue;
    std::vector<GlobalRoutePath> appended;
    appended.reserve(edge_count);
    for (std::size_t count = 1U; count < components.size(); ++count) {
      std::optional<Edge> best;
      for (std::size_t index = 0U; index < frontier.size(); ++index) {
        if (selected[index] || !frontier[index]) continue;
        if (!best || edge_less(*frontier[index], *best)) best = frontier[index];
      }
      if (!best) { complete_metadata = false; break; }
      const auto next = selected[best->left] ? best->right : best->left;
      selected[next] = true;
      if (!update_frontier(next)) return result;
      const auto [first, second] = pair_layers(*best->left_pad, *best->right_pad, copper_layers);
      GlobalRoutePath request;
      request.net = net;
      request.connection_index = next_index[net]++;
      request.points = {{best->left_pad->position, first}, {best->right_pad->position, second}};
      appended.push_back(std::move(request));
    }
    if (complete_metadata) result.insert(result.end(), appended.begin(), appended.end());
  }
  return result;
}

std::optional<std::vector<GlobalRoutePath>>
derive_pad_preservation_demands(const Board& incumbent, const Board& candidate,
                                const std::vector<GlobalRoutePath>& existing_requests,
                                std::size_t maximum_demands,
                                std::size_t maximum_pad_pair_checks) {
  const auto incumbent_components = pad_component_map(analyze_connectivity(incumbent));
  const auto candidate_components = pad_component_map(analyze_connectivity(candidate));
  if (!incumbent_components || !candidate_components) return std::nullopt;

  std::set<LayerId> candidate_copper_layers;
  for (const auto& layer : candidate.layers)
    if (layer.kind == LayerKind::copper) candidate_copper_layers.insert(layer.id);
  if (candidate_copper_layers.empty()) return std::nullopt;

  const auto incumbent_pads = copper_pad_shapes(incumbent);
  const auto candidate_pads = copper_pad_shapes(candidate);
  std::set<PadComponentKey> incumbent_pad_keys;
  for (const auto& pad : incumbent_pads)
    if (!incumbent_pad_keys.emplace(pad.net, pad.id).second) return std::nullopt;
  std::map<PadComponentKey, const CopperPadShape*> candidate_pad_by_key;
  for (const auto& pad : candidate_pads) {
    const PadComponentKey key{pad.net, pad.id};
    if (!candidate_pad_by_key.emplace(key, &pad).second) return std::nullopt;
  }

  // The key keeps each preservation obligation within a formerly connected component. Candidate
  // components are deliberately only labels for a partition of that old group.
  std::map<std::pair<NetId, std::size_t>, std::map<std::size_t, std::vector<const CopperPadShape*>>>
      partitions;
  for (const auto& old_pad : incumbent_pads) {
    const PadComponentKey key{old_pad.net, old_pad.id};
    const auto old_component = incumbent_components->find(key);
    const auto new_component = candidate_components->find(key);
    const auto candidate_pad = candidate_pad_by_key.find(key);
    if (old_component == incumbent_components->end() || new_component == candidate_components->end() ||
        candidate_pad == candidate_pad_by_key.end() ||
        legal_layers(*candidate_pad->second, candidate_copper_layers).empty())
      return std::nullopt;
    partitions[{old_pad.net, old_component->second}][new_component->second].push_back(candidate_pad->second);
  }

  // Reject report entries without a real incumbent pad witness as well as any candidate pad whose
  // identity or net metadata conflicts with the preserved domain.
  for (const auto& [key, ignored] : *incumbent_components)
    if (!incumbent_pad_keys.contains(key)) return std::nullopt;

  std::map<NetId, std::size_t> next_index;
  std::set<NetId> saturated_indices;
  for (const auto& request : existing_requests) {
    if (request.connection_index == std::numeric_limits<std::size_t>::max()) {
      saturated_indices.insert(request.net);
      continue;
    }
    auto& next = next_index[request.net];
    next = std::max(next, request.connection_index + 1U);
  }

  std::vector<GlobalRoutePath> demands;
  std::size_t pair_checks = 0U;
  for (auto& [old_group, groups] : partitions) {
    if (groups.size() < 2U) continue;
    std::vector<Component> components;
    components.reserve(groups.size());
    for (auto& [id, pads] : groups) {
      std::sort(pads.begin(), pads.end(), [](const auto* lhs, const auto* rhs) {
        return key_for(*lhs) < key_for(*rhs);
      });
      const auto representative = key_for(*pads.front());
      components.push_back({id, std::move(pads), representative});
    }
    std::sort(components.begin(), components.end(), [](const auto& lhs, const auto& rhs) {
      return lhs.representative < rhs.representative;
    });
    const auto edge_count = components.size() - 1U;
    if (demands.size() > maximum_demands || edge_count > maximum_demands - demands.size()) return std::nullopt;
    if (saturated_indices.contains(old_group.first)) return std::nullopt;
    auto& next = next_index[old_group.first];
    if (next > std::numeric_limits<std::size_t>::max() - edge_count) return std::nullopt;

    std::vector<bool> selected(components.size(), false);
    std::vector<std::optional<Edge>> frontier(components.size());
    const auto update_frontier = [&](std::size_t from) {
      for (std::size_t to = 0U; to < components.size(); ++to) {
        if (selected[to]) continue;
        const auto left = std::min(from, to), right = std::max(from, to);
        for (const auto* lhs : components[left].pads) for (const auto* rhs : components[right].pads) {
          if (pair_checks >= maximum_pad_pair_checks) return false;
          ++pair_checks;
          const auto lhs_key = key_for(*lhs), rhs_key = key_for(*rhs);
          Edge edge{left, right, lhs, rhs, squared_distance(*lhs, *rhs),
                    std::min(lhs_key, rhs_key), std::max(lhs_key, rhs_key)};
          if (!frontier[to] || edge_less(edge, *frontier[to])) frontier[to] = std::move(edge);
        }
      }
      return true;
    };
    selected.front() = true;
    if (!update_frontier(0U)) return std::nullopt;
    for (std::size_t count = 1U; count < components.size(); ++count) {
      std::optional<Edge> best;
      for (std::size_t index = 0U; index < frontier.size(); ++index)
        if (!selected[index] && frontier[index] && (!best || edge_less(*frontier[index], *best)))
          best = frontier[index];
      if (!best) return std::nullopt;
      const auto next_component = selected[best->left] ? best->right : best->left;
      selected[next_component] = true;
      if (!update_frontier(next_component)) return std::nullopt;
      const auto [first, second] = pair_layers(*best->left_pad, *best->right_pad, candidate_copper_layers);
      GlobalRoutePath demand;
      demand.net = old_group.first;
      demand.connection_index = next++;
      demand.points = {{best->left_pad->position, first}, {best->right_pad->position, second}};
      demands.push_back(std::move(demand));
    }
  }
  return demands;
}

} // namespace flux
