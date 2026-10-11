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

} // namespace flux
