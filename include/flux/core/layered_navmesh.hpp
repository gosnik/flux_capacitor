#pragma once

#include "flux/core/navmesh_route.hpp"

#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <stop_token>
#include <string>
#include <vector>

namespace flux {

// A layered domain contains copper layers only, in physical stack order.  A caller constructs
// it from every copper layer in the regional window; dielectric layers never become graph nodes.
struct LayeredNavMeshLayer {
  LayerId layer;
  std::size_t stack_order{0U};
  // Each disconnected offset free-space island retains its own prepared domain. An empty list
  // still represents this physical copper layer in a barrel span, but offers no graph nodes.
  std::vector<std::shared_ptr<const PreparedNavMeshDomain>> domains;
};

// A via zone is a proposed physical site, not permission to use a via.  Preparation expands its
// endpoint layers to the complete configured copper span and asks the caller to certify that
// exact span.  Regional routing supplies its existing via_site_legal/vias_fit_span authority.
struct LayeredNavMeshViaZone {
  std::size_t id{0U};
  Point site;
  // Electrical endpoints are the copper layers connected by the proposed route.
  LayerId electrical_first_layer;
  LayerId electrical_last_layer;
  // Physical barrel endpoints are supplied by the board's supported via model.  They may be
  // wider than the electrical span (for example a through via used to reach an inner layer).
  LayerId physical_first_layer;
  LayerId physical_last_layer;
  Nanometres cost{Nanometres{0}};
  // Directed zones let a constrained regional overpass require entry then exit. Ordinary via
  // candidates remain bidirectional by default.
  bool bidirectional{true};
};

struct LayeredNavMeshViaTransition {
  std::size_t zone_id{0U};
  Point site;
  LayerId electrical_first_layer;
  LayerId electrical_last_layer;
  // Always physical stack order, including every configured copper layer between endpoints.
  std::vector<LayerId> span;
  Nanometres cost{Nanometres{0}};
};

using LayeredNavMeshViaCertification = std::function<bool(const LayeredNavMeshViaTransition&)>;

struct LayeredNavMeshNode {
  std::size_t layer_index{0U};
  std::size_t domain_index{0U};
  std::size_t cell{0U};

  bool operator==(const LayeredNavMeshNode&) const = default;
};

enum class LayeredNavMeshEdgeKind { planar, transition };

// These edge records deliberately retain the original planar portal or certified via span. They
// are the handoff for RegionalRouteTemplate construction, not a second geometry authority.
struct LayeredNavMeshEdge {
  LayeredNavMeshEdgeKind kind{LayeredNavMeshEdgeKind::planar};
  LayeredNavMeshNode from;
  LayeredNavMeshNode to;
  NavMeshPortal portal{};
  LayeredNavMeshViaTransition transition{};
};

struct LayeredNavMeshConfig {
  std::size_t maximum_search_expansions{500'000U};
  std::size_t minimum_transitions{0U};
  std::size_t maximum_transitions{8U};
  std::size_t maximum_search_states{262'144U};
  std::size_t maximum_frontier_entries{262'144U};
};

struct LayeredNavMeshPlanarRun {
  LayerId layer;
  std::size_t domain_index{0U};
  std::vector<std::size_t> cells;
  std::vector<NavMeshPortal> portals;
};

struct LayeredNavMeshPreparationConfig {
  // Conservative defaults keep the derived graph below a large transient allocation. Callers
  // that have measured headroom may raise these explicitly.
  std::size_t maximum_nodes{65'536U};
  std::size_t maximum_planar_edges{262'144U};
  std::size_t maximum_via_zones{16'384U};
  std::size_t maximum_transitions{65'536U};
};

struct LayeredNavMeshRoute {
  LayerId start_layer;
  LayerId goal_layer;
  std::vector<LayeredNavMeshNode> nodes;
  std::vector<LayeredNavMeshEdge> edges;
  std::size_t search_expansions{0U};
  bool search_budget_exhausted{false};
  bool cancelled{false};
  std::string diagnostic;

  [[nodiscard]] bool ok() const { return diagnostic.empty() && !nodes.empty(); }
};

// Immutable graph whose vertices refer back to immutable PreparedNavMeshDomain cells.  It has
// bounded planar adjacencies plus only pre-certified full-span via transitions.
struct PreparedLayeredNavMeshDomain {
  std::vector<LayeredNavMeshLayer> layers;
  std::vector<LayeredNavMeshViaTransition> transitions;
  std::vector<LayeredNavMeshNode> nodes;
  std::vector<std::vector<LayeredNavMeshEdge>> adjacency;
  std::vector<std::vector<std::vector<std::size_t>>> node_by_layer_domain_cell;
  bool preparation_bound_exhausted{false};
  bool cancelled{false};
  std::string diagnostic;

  [[nodiscard]] bool ok() const { return diagnostic.empty() && !nodes.empty(); }
};

[[nodiscard]] PreparedLayeredNavMeshDomain prepare_layered_navmesh_domain(
    std::vector<LayeredNavMeshLayer> layers, std::vector<LayeredNavMeshViaZone> via_zones,
    const LayeredNavMeshViaCertification& certify_via,
    const LayeredNavMeshPreparationConfig& config = {}, std::stop_token stop_token = {});

[[nodiscard]] LayeredNavMeshRoute route_prepared_layered_navmesh(
    const PreparedLayeredNavMeshDomain& domain, Point start, LayerId start_layer, Point goal,
    LayerId goal_layer, const LayeredNavMeshConfig& config = {}, std::stop_token stop_token = {});

// Converts explicit selected planar graph edges into domain-local cell corridors. A regional
// consumer closes every other cell when calling route_prepared_navmesh_flood_fill(), so funnel
// realization cannot silently replace the graph-selected corridor.
[[nodiscard]] std::vector<LayeredNavMeshPlanarRun>
layered_navmesh_planar_runs(const PreparedLayeredNavMeshDomain& domain,
                            const LayeredNavMeshRoute& route);

} // namespace flux
