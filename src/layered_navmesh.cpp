#include "flux/core/layered_navmesh.hpp"
#include "flux/core/routing_kernel.hpp"

#include <algorithm>
#include <limits>
#include <optional>
#include <queue>
#include <set>
#include <utility>

namespace flux {
namespace {
[[nodiscard]] const PreparedNavMeshDomain& planar_domain(const PreparedLayeredNavMeshDomain& graph,
                                                         LayeredNavMeshNode node) {
  return *graph.layers[node.layer_index].domains[node.domain_index];
}
[[nodiscard]] long double node_distance(const PreparedLayeredNavMeshDomain& graph,
                                        LayeredNavMeshNode a, LayeredNavMeshNode b) {
  return geometry::point_distance_nm(planar_domain(graph, a).cells[a.cell].center,
                                     planar_domain(graph, b).cells[b.cell].center);
}
// A via edge is a vertical transition at one exact certified site. Its graph cost must include
// the planar movement from each selected cell centre to that site; charging only the via itself
// would turn every zone into an unphysical zero-length teleport between distant cell centres.
[[nodiscard]] long double transition_distance(const PreparedLayeredNavMeshDomain& graph,
                                              const LayeredNavMeshEdge& edge) {
  const auto& first = planar_domain(graph, edge.from).cells[edge.from.cell].center;
  const auto& second = planar_domain(graph, edge.to).cells[edge.to.cell].center;
  return geometry::point_distance_nm(first, edge.transition.site) +
         static_cast<long double>(edge.transition.cost.value()) +
         geometry::point_distance_nm(edge.transition.site, second);
}
[[nodiscard]] std::optional<std::size_t> layer_index(const PreparedLayeredNavMeshDomain& graph,
                                                     const LayerId& layer) {
  const auto found = std::find_if(graph.layers.begin(), graph.layers.end(),
                                  [&](const auto& candidate) { return candidate.layer == layer; });
  return found == graph.layers.end()
             ? std::nullopt
             : std::optional<std::size_t>{static_cast<std::size_t>(found - graph.layers.begin())};
}
[[nodiscard]] std::vector<std::size_t> containing_nodes(const PreparedLayeredNavMeshDomain& graph,
                                                        std::size_t layer, Point point) {
  std::vector<std::size_t> result;
  for (std::size_t island = 0U; island < graph.layers[layer].domains.size(); ++island) {
    const auto& cells = graph.layers[layer].domains[island]->cells;
    for (std::size_t cell = 0U; cell < cells.size(); ++cell) {
      routing_kernel_checkpoint();
      if (cells[cell].contains(point))
        result.push_back(graph.node_by_layer_domain_cell[layer][island][cell]);
    }
  }
  return result;
}
[[nodiscard]] bool interrupted(PreparedLayeredNavMeshDomain& result, std::stop_token token) {
  if (!token.stop_requested())
    return false;
  result.cancelled = true;
  result.diagnostic = "layered NavMesh preparation cancelled";
  return true;
}
} // namespace

PreparedLayeredNavMeshDomain prepare_layered_navmesh_domain(
    std::vector<LayeredNavMeshLayer> layers, std::vector<LayeredNavMeshViaZone> via_zones,
    const LayeredNavMeshViaCertification& certify_via,
    const LayeredNavMeshPreparationConfig& config, std::stop_token stop_token) {
  PreparedLayeredNavMeshDomain result;
  if (layers.empty() || via_zones.size() > config.maximum_via_zones) {
    result.preparation_bound_exhausted = via_zones.size() > config.maximum_via_zones;
    result.diagnostic = result.preparation_bound_exhausted
                            ? "layered NavMesh via-zone bound exhausted"
                            : "layered NavMesh has no copper layers";
    return result;
  }
  std::sort(layers.begin(), layers.end(), [](const auto& a, const auto& b) {
    return a.stack_order != b.stack_order ? a.stack_order < b.stack_order : a.layer < b.layer;
  });
  std::set<LayerId> seen_layers;
  for (std::size_t layer = 0U; layer < layers.size(); ++layer) {
    if (layers[layer].layer.empty() ||
        (layer && (layers[layer - 1U].stack_order == layers[layer].stack_order ||
                   layers[layer - 1U].layer == layers[layer].layer)) ||
        !seen_layers.insert(layers[layer].layer).second) {
      result.diagnostic = "layered NavMesh requires unique physical copper stack layers";
      return result;
    }
    for (const auto& island : layers[layer].domains)
      if (!island || !island->ok()) {
        result.diagnostic = "layered NavMesh received an invalid prepared domain";
        return result;
      }
  }
  result.layers = std::move(layers);
  result.node_by_layer_domain_cell.resize(result.layers.size());
  for (std::size_t layer = 0U; layer < result.layers.size(); ++layer) {
    if (interrupted(result, stop_token))
      return result;
    auto& map = result.node_by_layer_domain_cell[layer];
    map.resize(result.layers[layer].domains.size());
    for (std::size_t island = 0U; island < result.layers[layer].domains.size(); ++island) {
      const auto& cells = result.layers[layer].domains[island]->cells;
      if (cells.size() > config.maximum_nodes - result.nodes.size()) {
        result.preparation_bound_exhausted = true;
        result.diagnostic = "layered NavMesh node bound exhausted";
        return result;
      }
      map[island].resize(cells.size());
      for (std::size_t cell = 0U; cell < cells.size(); ++cell) {
        routing_kernel_checkpoint();
        map[island][cell] = result.nodes.size();
        result.nodes.push_back({layer, island, cell});
      }
    }
  }
  if (result.nodes.empty()) {
    result.diagnostic = "layered NavMesh has no navigable copper cells";
    return result;
  }
  result.adjacency.resize(result.nodes.size());
  std::size_t planar_edges = 0U;
  for (std::size_t layer = 0U; layer < result.layers.size(); ++layer)
    for (std::size_t island = 0U; island < result.layers[layer].domains.size(); ++island) {
      if (interrupted(result, stop_token))
        return result;
      const auto& planar = *result.layers[layer].domains[island];
      if (planar.adjacency_by_cell.size() > planar.cells.size()) {
        result.diagnostic = "layered NavMesh planar adjacency is malformed";
        return result;
      }
      for (std::size_t cell = 0U; cell < planar.adjacency_by_cell.size(); ++cell)
        for (const auto& adjacent : planar.adjacency_by_cell[cell]) {
          routing_kernel_checkpoint();
          if (adjacent.cell >= planar.cells.size())
            continue;
          if (planar_edges++ == config.maximum_planar_edges) {
            result.preparation_bound_exhausted = true;
            result.diagnostic = "layered NavMesh planar-edge bound exhausted";
            return result;
          }
          const auto from = result.node_by_layer_domain_cell[layer][island][cell],
                     to = result.node_by_layer_domain_cell[layer][island][adjacent.cell];
          result.adjacency[from].push_back({LayeredNavMeshEdgeKind::planar,
                                            result.nodes[from],
                                            result.nodes[to],
                                            adjacent.portal,
                                            {}});
        }
    }
  std::size_t transition_edges = 0U;
  for (const auto& zone : via_zones) {
    if (interrupted(result, stop_token))
      return result;
    const auto electrical_first = layer_index(result, zone.electrical_first_layer),
               electrical_last = layer_index(result, zone.electrical_last_layer),
               physical_first = layer_index(result, zone.physical_first_layer),
               physical_last = layer_index(result, zone.physical_last_layer);
    if (!electrical_first || !electrical_last || !physical_first || !physical_last ||
        *electrical_first == *electrical_last || zone.cost.value() < 0)
      continue;
    const auto low = std::min(*physical_first, *physical_last),
               high = std::max(*physical_first, *physical_last);
    if (*electrical_first < low || *electrical_first > high || *electrical_last < low ||
        *electrical_last > high)
      continue;
    LayeredNavMeshViaTransition transition{
        zone.id, zone.site, zone.electrical_first_layer, zone.electrical_last_layer, {}, zone.cost};
    for (std::size_t layer = low; layer <= high; ++layer)
      transition.span.push_back(result.layers[layer].layer);
    bool physical_site = true;
    for (std::size_t layer = low; layer <= high; ++layer)
      if (containing_nodes(result, layer, zone.site).empty()) {
        physical_site = false;
        break;
      }
    if (!physical_site || !certify_via || !certify_via(transition))
      continue;
    const auto entry_nodes = containing_nodes(result, *electrical_first, zone.site);
    const auto exit_nodes = containing_nodes(result, *electrical_last, zone.site);
    const auto directional_edges =
        entry_nodes.size() * exit_nodes.size() * (zone.bidirectional ? 2U : 1U);
    if (directional_edges > config.maximum_transitions - transition_edges) {
      result.preparation_bound_exhausted = true;
      result.diagnostic = "layered NavMesh transition-edge bound exhausted";
      return result;
    }
    transition_edges += directional_edges;
    result.transitions.push_back(transition);
    for (const auto from : entry_nodes)
      for (const auto to : exit_nodes) {
        result.adjacency[from].push_back({LayeredNavMeshEdgeKind::transition,
                                          result.nodes[from],
                                          result.nodes[to],
                                          {},
                                          transition});
        if (zone.bidirectional)
          result.adjacency[to].push_back({LayeredNavMeshEdgeKind::transition,
                                          result.nodes[to],
                                          result.nodes[from],
                                          {},
                                          transition});
      }
  }
  return result;
}

LayeredNavMeshRoute route_prepared_layered_navmesh(const PreparedLayeredNavMeshDomain& domain,
                                                   Point start, LayerId start_layer, Point goal,
                                                   LayerId goal_layer,
                                                   const LayeredNavMeshConfig& config,
                                                   std::stop_token token) {
  LayeredNavMeshRoute result;
  result.start_layer = std::move(start_layer);
  result.goal_layer = std::move(goal_layer);
  if (!domain.ok() || !config.maximum_search_expansions || !config.maximum_search_states ||
      !config.maximum_frontier_entries || config.minimum_transitions > config.maximum_transitions) {
    result.search_budget_exhausted = !config.maximum_search_expansions;
    result.diagnostic = domain.ok() ? "layered NavMesh search budget exhausted" : domain.diagnostic;
    return result;
  }
  const auto start_index = layer_index(domain, result.start_layer),
             goal_index = layer_index(domain, result.goal_layer);
  if (!start_index || !goal_index) {
    result.diagnostic = "layered NavMesh route references an unknown copper layer";
    return result;
  }
  const auto starts = containing_nodes(domain, *start_index, start),
             goals = containing_nodes(domain, *goal_index, goal);
  if (starts.empty() || goals.empty()) {
    result.diagnostic = "layered NavMesh endpoint is outside represented copper free space";
    return result;
  }
  const auto states_per_node = config.maximum_transitions + 1U;
  if (states_per_node == 0U ||
      domain.nodes.size() > std::numeric_limits<std::size_t>::max() / states_per_node ||
      domain.nodes.size() * states_per_node > config.maximum_search_states) {
    result.search_budget_exhausted = true;
    result.diagnostic = "layered NavMesh transition-state bound exhausted";
    return result;
  }
  const auto state_id = [states_per_node](std::size_t node, std::size_t transitions) {
    return node * states_per_node + transitions;
  };
  const auto infinity = std::numeric_limits<long double>::infinity();
  std::vector<long double> distances(domain.nodes.size() * states_per_node, infinity);
  std::vector<std::optional<std::pair<std::size_t, LayeredNavMeshEdge>>> previous(distances.size());
  std::vector<bool> is_goal(domain.nodes.size());
  for (const auto node : goals)
    is_goal[node] = true;
  using Queue = std::pair<long double, std::size_t>;
  std::priority_queue<Queue, std::vector<Queue>, std::greater<>> queue;
  for (const auto node : starts) {
    const auto state = state_id(node, 0U);
    distances[state] = geometry::point_distance_nm(
        start, planar_domain(domain, domain.nodes[node]).cells[domain.nodes[node].cell].center);
    if (queue.size() == config.maximum_frontier_entries) {
      result.search_budget_exhausted = true;
      result.diagnostic = "layered NavMesh frontier bound exhausted";
      return result;
    }
    queue.emplace(distances[state], state);
  }
  std::optional<std::size_t> reached;
  auto best_goal_cost = std::numeric_limits<long double>::infinity();
  while (!queue.empty()) {
    routing_kernel_checkpoint();
    if (token.stop_requested()) {
      result.cancelled = true;
      result.diagnostic = "layered NavMesh search cancelled";
      return result;
    }
    const auto [cost, current_state] = queue.top();
    queue.pop();
    if (cost != distances[current_state])
      continue;
    if (cost >= best_goal_cost)
      break;
    const auto current = current_state / states_per_node;
    const auto transition_count = current_state % states_per_node;
    if (++result.search_expansions > config.maximum_search_expansions) {
      result.search_budget_exhausted = true;
      result.diagnostic = "layered NavMesh search budget exhausted";
      return result;
    }
    if (is_goal[current] && transition_count >= config.minimum_transitions) {
      const auto& center =
          planar_domain(domain, domain.nodes[current]).cells[domain.nodes[current].cell].center;
      const auto goal_cost = cost + geometry::point_distance_nm(center, goal);
      if (goal_cost < best_goal_cost) {
        best_goal_cost = goal_cost;
        reached = current_state;
      }
      continue;
    }
    for (const auto& edge : domain.adjacency[current]) {
      const auto next =
          domain.node_by_layer_domain_cell[edge.to.layer_index][edge.to.domain_index][edge.to.cell];
      const auto next_transition_count =
          transition_count + (edge.kind == LayeredNavMeshEdgeKind::transition ? 1U : 0U);
      if (next_transition_count > config.maximum_transitions)
        continue;
      const auto next_state = state_id(next, next_transition_count);
      const auto next_cost = cost + (edge.kind == LayeredNavMeshEdgeKind::planar
                                         ? node_distance(domain, edge.from, edge.to)
                                         : transition_distance(domain, edge));
      if (next_cost < distances[next_state]) {
        distances[next_state] = next_cost;
        previous[next_state] = std::pair{current_state, edge};
        if (queue.size() == config.maximum_frontier_entries) {
          result.search_budget_exhausted = true;
          result.diagnostic = "layered NavMesh frontier bound exhausted";
          return result;
        }
        queue.emplace(next_cost, next_state);
      }
    }
  }
  if (!reached) {
    result.diagnostic = "layered NavMesh corridor is disconnected";
    return result;
  }
  std::vector<LayeredNavMeshEdge> reverse_edges;
  std::vector<LayeredNavMeshNode> reverse_nodes{domain.nodes[*reached / states_per_node]};
  for (auto current = *reached; previous[current]; current = previous[current]->first) {
    reverse_edges.push_back(previous[current]->second);
    reverse_nodes.push_back(domain.nodes[previous[current]->first / states_per_node]);
  }
  result.nodes.assign(reverse_nodes.rbegin(), reverse_nodes.rend());
  result.edges.assign(reverse_edges.rbegin(), reverse_edges.rend());
  return result;
}

std::vector<LayeredNavMeshPlanarRun>
layered_navmesh_planar_runs(const PreparedLayeredNavMeshDomain& domain,
                            const LayeredNavMeshRoute& route) {
  std::vector<LayeredNavMeshPlanarRun> runs;
  for (const auto& edge : route.edges) {
    if (edge.kind != LayeredNavMeshEdgeKind::planar)
      continue;
    const auto& layer = domain.layers[edge.from.layer_index].layer;
    if (runs.empty() || runs.back().layer != layer ||
        runs.back().domain_index != edge.from.domain_index) {
      runs.push_back({layer, edge.from.domain_index, {edge.from.cell}, {}});
    }
    runs.back().cells.push_back(edge.to.cell);
    runs.back().portals.push_back(edge.portal);
  }
  return runs;
}
} // namespace flux
