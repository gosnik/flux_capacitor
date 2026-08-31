#include "flux/core/global_router.hpp"

#include "flux/core/board_free_space.hpp"
#include "flux/core/connectivity.hpp"
#include "flux/core/copper_geometry.hpp"
#include "flux/core/free_space_offset.hpp"
#include "flux/core/free_space_skeleton.hpp"
#include "flux/core/geometry.hpp"
#include "flux/core/keepout_geometry.hpp"
#include "flux/core/navmesh_route.hpp"
#include "flux/core/pns_router.hpp"
#include "flux/core/router_telemetry.hpp"
#include "flux/core/routing_rules.hpp"
#include "flux/core/spline_route.hpp"
#include "flux/core/validation.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <queue>
#include <set>
#include <stdexcept>
#include <stop_token>
#include <string>
#include <string_view>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

namespace flux {
namespace {

struct Grid {
  std::int64_t origin_x{0};
  std::int64_t origin_y{0};
  std::int64_t pitch{0};
  std::size_t via_stride{1};
  std::size_t via_phase_x{0};
  std::size_t via_phase_y{0};
  bool diagonal_edges{false};
  bool crossing_safe_diagonal_edges{false};
  std::size_t width{0};
  std::size_t height{0};
  std::vector<LayerId> layers;
  std::vector<bool> valid_cells;

  [[nodiscard]] std::size_t layer_size() const { return width * height; }

  [[nodiscard]] std::size_t node(std::size_t x, std::size_t y, std::size_t layer) const {
    return layer * layer_size() + y * width + x;
  }

  [[nodiscard]] std::tuple<std::size_t, std::size_t, std::size_t>
  decode(std::size_t node_index) const {
    const auto size = layer_size();
    if (size == 0U) {
      throw std::logic_error("cannot decode a node in an empty routing grid");
    }
    const auto layer = node_index / size;
    const auto within_layer = node_index % size;
    return {within_layer % width, within_layer / width, layer};
  }

  [[nodiscard]] bool valid(std::size_t x, std::size_t y) const {
    return x < width && y < height && valid_cells[y * width + x];
  }

  [[nodiscard]] bool via_site(std::size_t x, std::size_t y) const {
    return via_stride != 0U && x % via_stride == via_phase_x && y % via_stride == via_phase_y;
  }

  [[nodiscard]] Point point(std::size_t x, std::size_t y) const {
    return Point{Nanometres{origin_x + static_cast<std::int64_t>(x) * pitch},
                 Nanometres{origin_y + static_cast<std::int64_t>(y) * pitch}};
  }
};

struct Edge {
  std::size_t first{0};
  std::size_t second{0};

  Edge(std::size_t lhs, std::size_t rhs) : first(std::min(lhs, rhs)), second(std::max(lhs, rhs)) {}

  auto operator<=>(const Edge&) const = default;
};

// Both diagonals through one sampled cell intersect physically. When the crossing-safe topology
// is enabled, charge either chord to the same canonical cell resource while retaining the actual
// chord for exact obstacle checks and final geometry.
[[nodiscard]] Edge sampled_capacity_edge(const Grid& grid, const Edge& edge) {
  if (!grid.crossing_safe_diagonal_edges) {
    return edge;
  }
  const auto [first_x, first_y, first_layer] = grid.decode(edge.first);
  const auto [second_x, second_y, second_layer] = grid.decode(edge.second);
  if (first_layer != second_layer ||
      std::max(first_x, second_x) - std::min(first_x, second_x) != 1U ||
      std::max(first_y, second_y) - std::min(first_y, second_y) != 1U) {
    return edge;
  }
  const auto cell_x = std::min(first_x, second_x);
  const auto cell_y = std::min(first_y, second_y);
  return Edge{grid.node(cell_x, cell_y, first_layer),
              grid.node(cell_x + 1U, cell_y + 1U, first_layer)};
}

enum class ObstacleShape { segment, circular_arc, point, pad, zone, keepout };

struct PolygonBounds {
  Point minimum;
  Point maximum;

  [[nodiscard]] bool contains(Point point) const {
    return minimum.x <= point.x && point.x <= maximum.x && minimum.y <= point.y &&
           point.y <= maximum.y;
  }
};

struct IndexedCopperZoneFill {
  CopperZoneFill geometry;
  geometry::PreparedPolygon prepared_area;
  std::vector<geometry::PreparedPolygon> prepared_holes;
};

[[nodiscard]] IndexedCopperZoneFill index_zone_fill(const CopperZoneFill& fill) {
  IndexedCopperZoneFill result;
  result.geometry = fill;
  result.prepared_area = geometry::prepare_polygon(fill.area);
  result.prepared_holes.reserve(fill.holes.size());
  for (const auto& hole : fill.holes) {
    result.prepared_holes.push_back(geometry::prepare_polygon(hole));
  }
  return result;
}

struct IndexedObstacle {
  ObjectId id;
  NetId net;
  std::size_t layer{0};
  ObstacleShape shape{ObstacleShape::point};
  Point start;
  Point end;
  long double radius_nm{0.0L};
  std::optional<CopperPadShape> pad_geometry;
  std::optional<WorldKeepout> keepout_geometry;
  std::optional<IndexedCopperZoneFill> zone_geometry{};
  std::optional<TrackArc> arc_geometry{};
};

struct EdgeBlockage {
  bool board_edge{false};
  std::vector<ObjectId> obstacles;

  [[nodiscard]] bool blocked() const { return board_edge || !obstacles.empty(); }
};

struct ClearanceShadowCertificate {
  long double available_nm{0.0L};
  // The occupied resource that cast this shadow. Charging the shadow owner on this resource makes
  // negotiated history effective for both participants instead of attaching both penalties to the
  // later candidate's resource.
  std::optional<Edge> owner_edge;
  std::optional<std::size_t> owner_node;
};

struct RoutingOccupancy {
  std::map<Edge, std::set<NetId>> edges;
  std::map<std::size_t, std::set<NetId>> nodes;
  // Conservative physical-clearance shadows are separate from PathFinder capacity accounting:
  // they prohibit a later sampled centreline without pretending the shadow itself consumes a
  // routed lane or contributes overflow.
  // Certificates retain the minimum centreline distance remaining after subtracting the selected
  // occupant's copper radius and the actual occupied resource that cast the shadow. Search compares
  // the distance with the candidate radius plus resolved clearance; negotiated history charges each
  // participant on its own resource.
  std::map<Edge, std::map<NetId, ClearanceShadowCertificate>> clearance_edges;
  std::map<std::size_t, std::map<NetId, ClearanceShadowCertificate>> clearance_nodes;
  std::map<std::size_t, std::set<NetId>> corridors;
  std::map<std::size_t, std::set<NetId>> corridor_neck_downs;
  std::map<Edge, std::set<NetId>> corridor_neck_down_edges;
  std::map<std::size_t, std::set<NetId>> voronoi_resources;
  std::map<std::size_t, std::map<NetId, std::uint64_t>> voronoi_demand_nm_by_net;
  std::map<std::size_t, std::set<NetId>> direct_medial_neck_downs;
};

// A hard rip-up temporarily removes selected copper so a blocked net can be retried.  Retaining
// the former sampled edges as a soft, pass-local cost prevents that retry from simply taking over
// the victim's old route and recreating the same deadlock.  The owner set deliberately exempts a
// victim rebuilding its own path.  This is advisory only: ordinary capacity, clearance, and exact
// detailed validation remain authoritative.
struct FormerRouteMemory {
  std::map<Edge, std::set<NetId>> edges;
  std::size_t ripped_paths{0U};
};

// Return whether one ordinary sampled edge has an immediately adjacent, parallel edge occupied
// by another net on the same layer. This is intentionally a local routing-style cue rather than
// a new resource: the caller must still prove that its own corridor lane has capacity and pass
// all normal clearance, sampled-capacity, and exact-DRC gates.
[[nodiscard]] bool has_parallel_river_neighbor(const Grid& grid, const RoutingOccupancy& occupancy,
                                               const NetId& net, const Edge& edge) {
  const auto [first_x, first_y, first_layer] = grid.decode(edge.first);
  const auto [second_x, second_y, second_layer] = grid.decode(edge.second);
  if (first_layer != second_layer || (first_x == second_x && first_y == second_y)) {
    return false;
  }
  const auto signed_delta = [](std::size_t first, std::size_t second) {
    return second > first ? 1 : second < first ? -1 : 0;
  };
  const auto delta_x = signed_delta(first_x, second_x);
  const auto delta_y = signed_delta(first_y, second_y);
  if (std::abs(delta_x) > 1 || std::abs(delta_y) > 1) {
    return false;
  }
  const auto occupied_by_other_net = [&occupancy, &net](const Edge& candidate) {
    const auto found = occupancy.edges.find(candidate);
    return found != occupancy.edges.end() &&
           std::any_of(found->second.begin(), found->second.end(),
                       [&net](const NetId& occupant) { return occupant != net; });
  };
  const auto probe_offset = [&](int offset_x, int offset_y) {
    const auto shifted_first_x = static_cast<std::int64_t>(first_x) + offset_x;
    const auto shifted_first_y = static_cast<std::int64_t>(first_y) + offset_y;
    const auto shifted_second_x = static_cast<std::int64_t>(second_x) + offset_x;
    const auto shifted_second_y = static_cast<std::int64_t>(second_y) + offset_y;
    if (shifted_first_x < 0 || shifted_first_y < 0 || shifted_second_x < 0 ||
        shifted_second_y < 0 || shifted_first_x >= static_cast<std::int64_t>(grid.width) ||
        shifted_first_y >= static_cast<std::int64_t>(grid.height) ||
        shifted_second_x >= static_cast<std::int64_t>(grid.width) ||
        shifted_second_y >= static_cast<std::int64_t>(grid.height)) {
      return false;
    }
    const auto shifted_first_x_index = static_cast<std::size_t>(shifted_first_x);
    const auto shifted_first_y_index = static_cast<std::size_t>(shifted_first_y);
    const auto shifted_second_x_index = static_cast<std::size_t>(shifted_second_x);
    const auto shifted_second_y_index = static_cast<std::size_t>(shifted_second_y);
    if (!grid.valid(shifted_first_x_index, shifted_first_y_index) ||
        !grid.valid(shifted_second_x_index, shifted_second_y_index)) {
      return false;
    }
    return occupied_by_other_net(sampled_capacity_edge(
        grid, Edge{grid.node(shifted_first_x_index, shifted_first_y_index, first_layer),
                   grid.node(shifted_second_x_index, shifted_second_y_index, first_layer)}));
  };
  // Rotate the one-grid routing direction by 90 degrees in both directions. For a diagonal,
  // this selects the two immediately adjacent diagonal lanes rather than crossing diagonals.
  return probe_offset(-delta_y, delta_x) || probe_offset(delta_y, -delta_x);
}

struct RoutingHistory {
  struct ClearanceEdgeKey {
    NetId net;
    Edge edge;

    auto operator<=>(const ClearanceEdgeKey&) const = default;
  };

  struct ClearanceNodeKey {
    NetId net;
    std::size_t node{0};

    auto operator<=>(const ClearanceNodeKey&) const = default;
  };

  struct DirectPortalKey {
    NetId net;
    std::size_t resource_id{0};
    std::size_t portal_node{0};

    auto operator<=>(const DirectPortalKey&) const = default;
  };

  struct ExactJunctionKey {
    NetId net;
    Edge resources;

    auto operator<=>(const ExactJunctionKey&) const = default;
  };

  struct SelectedJunctionEdgeKey {
    NetId net;
    Edge selected_sampled_edge;

    auto operator<=>(const SelectedJunctionEdgeKey&) const = default;
  };

  struct LaneConflictResourceKey {
    NetId net;
    std::size_t resource_id{0};

    auto operator<=>(const LaneConflictResourceKey&) const = default;
  };

  std::map<Edge, std::size_t> edges;
  std::map<std::size_t, std::size_t> nodes;
  std::map<ClearanceEdgeKey, std::size_t> clearance_edges;
  std::map<ClearanceNodeKey, std::size_t> clearance_nodes;
  std::map<ClearanceEdgeKey, std::size_t> routability_edges;
  std::map<std::size_t, std::size_t> corridors;
  std::map<std::size_t, std::size_t> corridor_neck_downs;
  std::map<std::size_t, std::size_t> voronoi_resources;
  std::map<DirectPortalKey, std::size_t> direct_portals;
  std::map<ExactJunctionKey, std::size_t> exact_junctions;
  std::map<SelectedJunctionEdgeKey, std::size_t> selected_junction_edges;
  std::map<LaneConflictResourceKey, std::size_t> lane_conflict_resources;
};

struct AccessKey {
  NetId net;
  std::size_t layer{0};
  Point start;
  Point end;
  Nanometres width;

  auto operator<=>(const AccessKey&) const = default;
};

struct EdgeWidthKey {
  NetId net;
  Edge edge;
  Nanometres width;

  auto operator<=>(const EdgeWidthKey&) const = default;
};

struct TerminalAccessPath {
  std::vector<Point> points;
  Nanometres width;
  bool neck_down{false};
  // Other same-net terminals physically traversed before this access reaches its coarse portal.
  // They are part of the same local fanout transaction and become connected with this path.
  std::vector<std::size_t> joined_terminals;
};

struct Terminal {
  ObjectId footprint;
  ObjectId pad;
  Point exact_position;
  std::vector<std::size_t> routing_layers;
  std::vector<std::size_t> grid_nodes;
  std::map<std::size_t, TerminalAccessPath> access_paths;
};

struct NetTask {
  NetId net;
  std::uint32_t priority{0};
  Nanometres nominal_track_width;
  Nanometres via_diameter;
  Nanometres physical_clearance;
  bool coupled_member{false};
  std::vector<Terminal> terminals;
};

struct SearchResult {
  std::vector<std::size_t> nodes;
  std::vector<Nanometres> segment_widths;
  std::vector<GlobalRouteSegmentWidthKind> segment_width_kinds;
  std::uint64_t cost{0};
  bool cancelled{false};
  bool expansion_limit_reached{false};
  bool encountered_expansion_limit{false};
  bool weighted_frontier_continuation_attempted{false};
  bool weighted_frontier_continuation_recovered{false};
  std::size_t expansions{0};
  std::size_t weighted_frontier_states_reused{0};
  std::size_t weighted_frontier_continuation_limits{0};
  std::size_t topology_rejected_edges{0};
  std::size_t hard_sampled_capacity_rejected_transitions{0};
  std::size_t hard_corridor_capacity_rejected_transitions{0};
  std::size_t hard_voronoi_capacity_rejected_transitions{0};
  std::map<NetId, std::size_t> hard_capacity_blocking_nets;
  std::map<std::vector<NetId>, std::size_t> hard_capacity_blocker_sets;
  std::size_t hard_capacity_conflict_sets_withheld{0};
  std::size_t residual_corridor_demand_penalty_events{0};
  std::uint64_t residual_corridor_demand_penalty{0};
  std::size_t critical_cut_penalty_events{0};
  std::uint64_t critical_cut_penalty{0};
  std::size_t river_following_edges{0};
  std::uint64_t river_following_cost_discount{0};
  std::size_t former_route_memory_penalty_events{0};
  std::uint64_t former_route_memory_penalty{0};
  std::size_t terminal_escape_reservation_penalty_events{0};
  std::uint64_t terminal_escape_reservation_penalty{0};
};

// A terminal is represented by the exact local access polylines already proven when its coarse
// portals were discovered.  These are future-demand envelopes, not occupancy: a candidate route
// receives only a soft cost when it would take the last access alternative which remains for a
// different, still-unrouted terminal.
struct TerminalEscapeReservationKey {
  NetId net;
  ObjectId footprint;
  ObjectId pad;

  auto operator<=>(const TerminalEscapeReservationKey&) const = default;
};

struct TerminalEscapeReservation {
  struct Candidate {
    TerminalEscapeReservationKey terminal;
    std::size_t layer{0U};
    Nanometres width{Nanometres{0}};
    long double collision_distance_nm{0.0L};
    std::vector<std::pair<Point, Point>> segments;
    std::set<Edge> blocking_edges;
    std::set<std::size_t> blocking_nodes;
  };

  std::vector<Candidate> candidates;
  std::map<TerminalEscapeReservationKey, std::vector<std::size_t>> candidates_by_terminal;
  std::map<Edge, std::vector<std::size_t>> candidates_by_edge;
  std::map<std::size_t, std::vector<std::size_t>> candidates_by_node;
  std::size_t resources{0U};
  std::size_t withheld_candidates{0U};
};

struct TerminalEscapeReservationAvailability {
  std::map<TerminalEscapeReservationKey, std::vector<std::size_t>> viable_by_terminal;
};

enum class SearchCostPolicyKind { negotiated, static_seed, corridor_reprojection };

// One graph search owns topology and legality. Policies change only non-negative costs; they can
// never add a transition that ordinary negotiated routing could not exact-check.
struct SearchCostPolicy {
  SearchCostPolicyKind kind{SearchCostPolicyKind::negotiated};
  std::set<Edge> penalized_edges;
  std::set<std::size_t> corridor_nodes;
  std::uint64_t alternative_edge_penalty{0};
  std::uint64_t corridor_departure_penalty{0};
  // The maps are owned by the routing pass.  They contain only corridors that every complete
  // static seed tree for a remaining net traverses, so optional alternatives are not treated as
  // reserved future capacity.
  const std::map<std::size_t, std::size_t>* residual_corridor_demand{nullptr};
  const std::map<std::size_t, std::size_t>* residual_corridor_neck_down_demand{nullptr};
  std::uint64_t residual_corridor_demand_cost{0};
  const std::map<std::size_t, std::size_t>* critical_corridor_cut_demand{nullptr};
  const std::map<std::size_t, std::size_t>* critical_corridor_neck_down_cut_demand{nullptr};
  const std::set<std::size_t>* current_reserved_corridors{nullptr};
  const std::set<std::size_t>* current_reserved_corridor_neck_downs{nullptr};
  std::uint64_t critical_cut_cost{0};
  const FormerRouteMemory* former_route_memory{nullptr};
  std::uint64_t former_route_memory_cost{0};
  const TerminalEscapeReservation* terminal_escape_reservation{nullptr};
  const TerminalEscapeReservationAvailability* terminal_escape_availability{nullptr};
  std::uint64_t terminal_escape_reservation_cost{0};
};

struct SearchEffort {
  std::size_t invocations{0};
  std::size_t expansions{0};
  std::size_t expansion_limited{0};
  std::size_t weighted_retries{0};
  std::size_t weighted_recoveries{0};
  std::size_t weighted_frontier_continuations{0};
  std::size_t weighted_frontier_recoveries{0};
  std::size_t weighted_frontier_states_reused{0};
  std::size_t weighted_frontier_continuation_limits{0};
};

struct QueueEntry {
  std::uint64_t estimate{0};
  std::uint64_t cost{0};
  std::size_t state{0};

  bool operator>(const QueueEntry& rhs) const {
    return std::tie(estimate, cost, state) > std::tie(rhs.estimate, rhs.cost, rhs.state);
  }
};

struct CorridorDescriptor {
  std::size_t id{0};
  std::size_t layer{0};
  RoutingCorridorOrientation orientation{RoutingCorridorOrientation::horizontal};
  Point first_lane_start;
  Point last_lane_end;
  std::size_t capacity{0};
  bool neck_down{false};
  std::size_t first_lane{0};
  std::size_t last_lane{0};
};

struct CorridorGraph {
  std::vector<CorridorDescriptor> corridors;
  std::size_t vertical_stride{0};
  std::vector<std::size_t> horizontal_corridor_by_node;
  std::vector<std::size_t> vertical_corridor_by_node;

  [[nodiscard]] std::optional<std::size_t> corridor_for(const Edge& edge) const {
    constexpr auto none = std::numeric_limits<std::size_t>::max();
    const auto difference = edge.second - edge.first;
    if (difference == 1U && edge.first < horizontal_corridor_by_node.size() &&
        horizontal_corridor_by_node[edge.first] != none) {
      return horizontal_corridor_by_node[edge.first];
    }
    if (difference == vertical_stride && edge.first < vertical_corridor_by_node.size() &&
        vertical_corridor_by_node[edge.first] != none) {
      return vertical_corridor_by_node[edge.first];
    }
    return std::nullopt;
  }

  [[nodiscard]] std::pair<std::uint64_t, std::uint64_t> centering_ratio(const Grid& grid,
                                                                        const Edge& edge) const {
    const auto corridor_id = corridor_for(edge);
    if (!corridor_id) {
      return {0U, 1U};
    }
    const auto& corridor = corridors[*corridor_id];
    if (corridor.capacity <= 1U) {
      return {0U, 1U};
    }
    const auto [x, y, unused_layer] = grid.decode(edge.first);
    static_cast<void>(unused_layer);
    const auto lane = corridor.orientation == RoutingCorridorOrientation::horizontal ? y : x;
    const auto doubled_lane = lane * 2U;
    const auto lane_sum = corridor.first_lane + corridor.last_lane;
    const auto doubled_offset =
        doubled_lane > lane_sum ? doubled_lane - lane_sum : lane_sum - doubled_lane;
    return {static_cast<std::uint64_t>(doubled_offset),
            static_cast<std::uint64_t>(2U * (corridor.capacity - 1U))};
  }
};

[[nodiscard]] bool
river_following_lane_available(const Grid& grid, const RoutingOccupancy& occupancy,
                               const NetId& net, const Edge& edge, const CorridorGraph& corridors,
                               const CorridorGraph& corridor_neck_downs, bool narrow) {
  if (!has_parallel_river_neighbor(grid, occupancy, net, edge)) {
    return false;
  }
  const auto& selected_corridors = narrow ? corridor_neck_downs : corridors;
  const auto corridor_id = selected_corridors.corridor_for(edge);
  if (!corridor_id) {
    return false;
  }
  const auto& corridor = selected_corridors.corridors[*corridor_id];
  if (corridor.capacity < 2U) {
    return false;
  }
  const auto& selected_occupancy = narrow ? occupancy.corridor_neck_downs : occupancy.corridors;
  const auto found = selected_occupancy.find(*corridor_id);
  const auto occupied =
      found == selected_occupancy.end() ? 0U : found->second.size() - found->second.count(net);
  return occupied < corridor.capacity;
}

struct VoronoiCorridorEvidenceBundle {
  std::vector<RoutingVoronoiCorridorEvidence> edges;
  std::size_t domains_attempted{0};
  std::size_t domains_accepted{0};
  std::size_t domains_rejected{0};
  std::size_t conservative_obstacles{0};
  std::size_t accepted_curved_edge_chords{0};
  std::size_t withheld_curved_edges{0};
  bool cancelled{false};
};

struct VoronoiPathFinderGuidance {
  std::map<NetId, std::vector<std::uint32_t>> distance_steps_by_net;
  std::size_t seed_nodes{0};

  [[nodiscard]] std::uint64_t edge_cost(const NetId& net, const Edge& edge,
                                        std::uint64_t unit_cost) const {
    const auto found = distance_steps_by_net.find(net);
    if (found == distance_steps_by_net.end() || edge.second >= found->second.size()) {
      return 0U;
    }
    const auto distance = std::min(found->second[edge.first], found->second[edge.second]);
    if (distance == std::numeric_limits<std::uint32_t>::max() || distance == 0U ||
        unit_cost == 0U) {
      return 0U;
    }
    return unit_cost >
                   std::numeric_limits<std::uint64_t>::max() / static_cast<std::uint64_t>(distance)
               ? std::numeric_limits<std::uint64_t>::max()
               : static_cast<std::uint64_t>(distance) * unit_cost;
  }
};

struct GlobalSeedCandidate {
  NetId net;
  std::size_t connection_index{0};
  std::size_t tree_candidate_index{0};
  std::size_t terminal_index{0};
  std::vector<std::size_t> connected_terminals;
  std::vector<std::size_t> starts;
  std::set<std::size_t> goals;
  SearchResult route;
  std::vector<std::size_t> shove_proposal_ids;
  std::set<std::size_t> spline_corridor_nodes;
  std::vector<RoutingGeometricCorridorRun> geometric_corridors;
  bool spline_refined{false};
};

struct StaticSeedStats {
  std::size_t connections{0};
  std::size_t tasks_dispatched{0};
  std::size_t workers_used{0};
  std::size_t searches{0};
  std::size_t candidates{0};
  std::size_t search_expansions{0};
  std::size_t penalty_reruns{0};
  std::size_t deduplicated_candidates{0};
  std::size_t via_bearing_candidates{0};
  std::size_t direct_medial_candidates{0};
  std::size_t terminals_without_seed{0};
  std::size_t distinct_layer_sequences{0};
  std::size_t distinct_topology_signatures{0};
  std::size_t corridor_nodes{0};
  std::size_t spline_samples{0};
  std::size_t navmesh_domains{0};
  std::size_t navmesh_routes{0};
  std::size_t navmesh_cells{0};
  std::size_t navmesh_search_expansions{0};
  std::size_t flood_fill_routes{0};
  std::size_t flood_fill_cells_visited{0};
  std::size_t navmesh_failures{0};
  std::size_t navmesh_certificate_domains{0};
  std::size_t funnel_points{0};
  std::size_t trajectory_attempts{0};
  std::size_t trajectory_accepted{0};
  std::size_t trajectory_control_moves{0};
  std::size_t trajectory_candidate_checks{0};
  std::size_t trajectory_rejected{0};
  std::size_t collision_envelopes{0};
  std::size_t control_shoves{0};
  std::size_t refits{0};
  std::size_t refined_candidates{0};
  std::size_t reprojection_searches{0};
  std::size_t reprojections_accepted{0};
  std::size_t reprojections_reverted{0};
  std::size_t corridor_departures{0};
  std::size_t complete_trees{0};
  std::size_t partial_trees{0};
  std::size_t alternative_trees{0};
  std::size_t tree_joins{0};
  std::size_t growth_reorderings{0};
  std::size_t joint_groups{0};
  std::size_t joint_group_candidates{0};
  std::size_t joint_groups_withheld{0};
  std::size_t pns_proposal_attempts{0};
  std::size_t pns_eligible_collisions{0};
  std::size_t pns_ineligible_collisions{0};
  std::size_t pns_candidate_checks{0};
  std::size_t pns_local_splits{0};
  bool budget_exhausted{false};
};

struct StaticSeedPlan {
  std::vector<GlobalSeedCandidate> candidates;
  std::vector<GlobalRoutingShoveProposal> shove_proposals;
  std::optional<Board> planning_board;
  StaticSeedStats stats;
};

// A capacity resource is claimed once per net, even when a multi-terminal tree uses several
// edges in it.  This mirrors RoutingOccupancy's set<NetId> representation.
struct ResidualCorridorDemand {
  std::set<std::size_t> corridors;
  std::set<std::size_t> neck_downs;
};

[[nodiscard]] bool critical_cut(const CorridorGraph& graph, std::size_t corridor_id,
                                std::size_t future_demand, const GlobalRouterConfig& config) {
  return config.critical_cut_cost > 0U &&
         future_demand >= config.critical_cut_minimum_future_demand &&
         corridor_id < graph.corridors.size() &&
         graph.corridors[corridor_id].capacity <= future_demand + 1U;
}

[[nodiscard]] std::uint64_t critical_cut_penalty(std::size_t future_demand, std::size_t capacity,
                                                 std::uint64_t critical_cut_cost) {
  if (future_demand == 0U || critical_cut_cost == 0U) {
    return 0U;
  }
  // A cut with seven reserved lanes and one spare lane is seven times more sensitive than one
  // with the same demand and seven spare lanes. This preserves scarce strategic slack without
  // converting it into a premature hard prohibition.
  const auto spare_lanes = capacity > future_demand ? capacity - future_demand : 0U;
  const auto demand = static_cast<std::uint64_t>(future_demand);
  const auto product = demand > std::numeric_limits<std::uint64_t>::max() / critical_cut_cost
                           ? std::numeric_limits<std::uint64_t>::max()
                           : demand * critical_cut_cost;
  return product / static_cast<std::uint64_t>(std::max<std::size_t>(1U, spare_lanes));
}

struct SharedVoronoiResource {
  std::size_t id{0};
  std::size_t layer{0};
  Point start;
  Point end;
  std::uint64_t capacity_nm{0};
  bool conservative_domain{false};
  bool curved_bisector_chord{false};
  std::vector<ObjectId> limiting_obstacles;
  std::optional<Edge> direct_edge;
  std::uint64_t direct_length_nm{0};
};

[[nodiscard]] std::vector<ObjectId>
limiting_free_space_obstacles(const BoardFreeSpaceExtractionResult& extracted, Point start,
                              Point end, Nanometres certified_clearance) {
  std::vector<ObjectId> result;
  if (extracted.obstacle_ids.size() != extracted.domain.obstacles.size() ||
      extracted.filled_obstacle_ids.size() != extracted.domain.filled_obstacles.size() ||
      certified_clearance.value() < 0) {
    return result;
  }
  // The skeleton certificate is floored after integer quantization. Retain the established
  // two-nanometre guard so provenance is not lost at a rounded equality.
  constexpr long double rounding_guard_nm = 2.0L;
  const auto limit = static_cast<long double>(certified_clearance.value()) + rounding_guard_nm;
  for (std::size_t index = 0U; index < extracted.domain.obstacles.size(); ++index) {
    if (geometry::segment_polygon_copper_distance_nm(start, end,
                                                     extracted.domain.obstacles[index]) <= limit) {
      result.push_back(extracted.obstacle_ids[index]);
    }
  }
  for (std::size_t index = 0U; index < extracted.domain.filled_obstacles.size(); ++index) {
    if (geometry::segment_filled_region_copper_distance_nm(
            start, end, extracted.domain.filled_obstacles[index]) <= limit) {
      result.push_back(extracted.filled_obstacle_ids[index]);
    }
  }
  std::sort(result.begin(), result.end());
  result.erase(std::unique(result.begin(), result.end()), result.end());
  return result;
}

struct DifferentialPairCrossSection {
  NetId positive_net;
  NetId negative_net;
  std::uint64_t positive_width_nm{0};
  std::uint64_t negative_width_nm{0};
  std::uint64_t gap_nm{0};
  std::uint64_t boundary_clearance_nm{0};
  std::uint64_t demand_nm{0};
};

struct RoutingBusCrossSection {
  RoutingBusId id;
  std::vector<NetId> ordered_nets;
  std::vector<std::uint64_t> widths_nm;
  std::vector<std::uint64_t> gaps_nm;
  std::uint64_t boundary_clearance_nm{0};
  std::uint64_t demand_nm{0};
};

struct SharedVoronoiAllocationGraph {
  std::vector<SharedVoronoiResource> resources;
  std::vector<std::size_t> resource_by_node;
  std::map<NetId, std::uint64_t> demand_nm_by_net;
  std::map<NetId, std::uint64_t> track_width_nm_by_net;
  std::map<NetId, std::uint64_t> clearance_nm_by_net;
  std::map<NetId, std::uint64_t> medial_neck_down_demand_nm_by_net;
  std::map<NetId, Nanometres> medial_neck_down_width_by_net;
  std::map<NetId, NetId> differential_complement_by_net;
  std::map<std::pair<NetId, NetId>, std::uint64_t> differential_pair_demand_nm;
  std::map<std::pair<NetId, NetId>, DifferentialPairCrossSection> differential_pair_cross_sections;
  std::map<NetId, RoutingBusId> routing_bus_by_net;
  std::map<RoutingBusId, RoutingBusCrossSection> routing_bus_cross_sections;
  std::set<Edge> resource_adjacency;
  std::map<Edge, std::size_t> direct_resource_by_edge;
  std::vector<std::vector<std::size_t>> direct_neighbors_by_node;
  std::size_t withheld_curved_edges{0};
  std::size_t withheld_direct_edges{0};
  std::size_t segmented_source_resources{0};
  std::size_t additional_segment_resources{0};
  std::size_t direct_resources_retained_atomic{0};
  std::size_t nodes_per_layer{0};
  std::uint64_t minimum_demand_nm{0};
  std::uint64_t max_medial_neck_down_length_nm{0};

  [[nodiscard]] std::vector<std::size_t> resources_for(const Edge& edge,
                                                       bool use_direct = true) const {
    if (use_direct) {
      if (const auto direct = direct_resource_by_edge.find(edge);
          direct != direct_resource_by_edge.end()) {
        return {direct->second};
      }
    }
    constexpr auto none = std::numeric_limits<std::size_t>::max();
    if (edge.second >= resource_by_node.size() || nodes_per_layer == 0U ||
        edge.first / nodes_per_layer != edge.second / nodes_per_layer) {
      return {};
    }
    const auto first = resource_by_node[edge.first];
    const auto second = resource_by_node[edge.second];
    if (first == none && second == none) {
      return {};
    }
    if (first == none) {
      return {second};
    }
    if (second == none || first == second) {
      return {first};
    }
    return first < second ? std::vector<std::size_t>{first, second}
                          : std::vector<std::size_t>{second, first};
  }

  [[nodiscard]] std::optional<std::size_t> direct_resource(const Edge& edge) const {
    const auto found = direct_resource_by_edge.find(edge);
    return found == direct_resource_by_edge.end() ? std::nullopt
                                                  : std::optional<std::size_t>{found->second};
  }

  [[nodiscard]] const std::vector<std::size_t>& direct_neighbors(std::size_t node) const {
    static const std::vector<std::size_t> empty;
    return node < direct_neighbors_by_node.size() ? direct_neighbors_by_node[node] : empty;
  }

  [[nodiscard]] std::size_t direct_lane_capacity(std::size_t resource) const {
    if (resource >= resources.size() || minimum_demand_nm == 0U) {
      return 1U;
    }
    const auto lanes =
        std::max<std::uint64_t>(1U, resources[resource].capacity_nm / minimum_demand_nm);
    return lanes > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max())
               ? std::numeric_limits<std::size_t>::max()
               : static_cast<std::size_t>(lanes);
  }

  [[nodiscard]] bool topologically_adjacent(std::size_t first, std::size_t second) const {
    return first == second || resource_adjacency.contains(Edge{first, second});
  }

  [[nodiscard]] bool resources_join_at_direct_portal(std::size_t first, std::size_t second,
                                                     std::size_t portal_node) const {
    if (first >= resources.size() || second >= resources.size() ||
        resources[first].layer != resources[second].layer ||
        portal_node >= resource_by_node.size()) {
      return false;
    }
    const auto incident = [this, portal_node](std::size_t resource) {
      const auto& direct = resources[resource].direct_edge;
      return direct && (direct->first == portal_node || direct->second == portal_node);
    };
    const auto mapped = resource_by_node[portal_node];
    return (incident(first) && (incident(second) || mapped == second)) ||
           (incident(second) && mapped == first);
  }

  [[nodiscard]] bool edge_topologically_supported(const Edge& edge, bool use_direct = true) const {
    constexpr auto none = std::numeric_limits<std::size_t>::max();
    if (const auto direct = use_direct ? direct_resource(edge) : std::nullopt) {
      for (const auto node : {edge.first, edge.second}) {
        if (node < resource_by_node.size() && resource_by_node[node] != none &&
            !topologically_adjacent(resource_by_node[node], *direct)) {
          return false;
        }
      }
      return true;
    }
    const auto resources_for_edge = resources_for(edge, use_direct);
    return resources_for_edge.size() < 2U ||
           topologically_adjacent(resources_for_edge[0], resources_for_edge[1]);
  }

  [[nodiscard]] std::uint64_t net_demand(const NetId& net) const {
    const auto found = demand_nm_by_net.find(net);
    return found == demand_nm_by_net.end() ? 0U : found->second;
  }

  [[nodiscard]] std::uint64_t demand_for_width(const NetId& net, Nanometres width) const {
    if (width.value() <= 0) {
      return net_demand(net);
    }
    const auto clearance = clearance_nm_by_net.find(net);
    if (clearance == clearance_nm_by_net.end()) {
      return net_demand(net);
    }
    const auto selected_width = static_cast<std::uint64_t>(width.value());
    const auto doubled_clearance =
        clearance->second > std::numeric_limits<std::uint64_t>::max() / 2U
            ? std::numeric_limits<std::uint64_t>::max()
            : clearance->second * 2U;
    return doubled_clearance > std::numeric_limits<std::uint64_t>::max() - selected_width
               ? std::numeric_limits<std::uint64_t>::max()
               : selected_width + doubled_clearance;
  }

  [[nodiscard]] std::optional<Nanometres> medial_neck_down_width(std::size_t resource,
                                                                 const NetId& net) const {
    if (resource >= resources.size() || !resources[resource].direct_edge ||
        resources[resource].direct_length_nm == 0U || max_medial_neck_down_length_nm == 0U ||
        resources[resource].direct_length_nm > max_medial_neck_down_length_nm) {
      return std::nullopt;
    }
    const auto nominal = demand_nm_by_net.find(net);
    const auto narrow = medial_neck_down_demand_nm_by_net.find(net);
    const auto width = medial_neck_down_width_by_net.find(net);
    if (nominal == demand_nm_by_net.end() || narrow == medial_neck_down_demand_nm_by_net.end() ||
        width == medial_neck_down_width_by_net.end() ||
        nominal->second <= resources[resource].capacity_nm ||
        narrow->second > resources[resource].capacity_nm) {
      return std::nullopt;
    }
    return width->second;
  }

  [[nodiscard]] std::optional<NetId> differential_complement(const NetId& net) const {
    const auto found = differential_complement_by_net.find(net);
    return found == differential_complement_by_net.end() ? std::nullopt
                                                         : std::optional<NetId>{found->second};
  }

  [[nodiscard]] const RoutingBusCrossSection* routing_bus(const NetId& net) const {
    const auto membership = routing_bus_by_net.find(net);
    if (membership == routing_bus_by_net.end()) {
      return nullptr;
    }
    const auto bus = routing_bus_cross_sections.find(membership->second);
    return bus == routing_bus_cross_sections.end() ? nullptr : &bus->second;
  }

  [[nodiscard]] static bool contains_complete_bus(const RoutingBusCrossSection& bus,
                                                  const std::set<NetId>& nets) {
    return std::all_of(bus.ordered_nets.begin(), bus.ordered_nets.end(),
                       [&nets](const NetId& net) { return nets.contains(net); });
  }

  [[nodiscard]] std::uint64_t
  occupied_demand(const std::set<NetId>& nets,
                  const std::map<NetId, std::uint64_t>& selected_demand_nm_by_net,
                  const NetId* excluded = nullptr) const {
    std::uint64_t result = 0U;
    std::set<NetId> accounted;
    for (const auto& net : nets) {
      if ((excluded != nullptr && net == *excluded) || accounted.contains(net)) {
        continue;
      }
      const auto selected = selected_demand_nm_by_net.find(net);
      auto demand =
          selected == selected_demand_nm_by_net.end() ? net_demand(net) : selected->second;
      if (const auto* bus = routing_bus(net); bus != nullptr && contains_complete_bus(*bus, nets)) {
        demand = bus->demand_nm;
        accounted.insert(bus->ordered_nets.begin(), bus->ordered_nets.end());
      } else if (const auto complement = differential_complement(net);
                 complement && nets.contains(*complement) &&
                 (excluded == nullptr || *complement != *excluded)) {
        const auto key =
            net < *complement ? std::pair{net, *complement} : std::pair{*complement, net};
        if (const auto found = differential_pair_demand_nm.find(key);
            found != differential_pair_demand_nm.end()) {
          demand = found->second;
          accounted.insert(*complement);
        }
      }
      if (demand > std::numeric_limits<std::uint64_t>::max() - result) {
        return std::numeric_limits<std::uint64_t>::max();
      }
      result += demand;
    }
    return result;
  }

  [[nodiscard]] bool contains_complete_differential_pair(const std::set<NetId>& nets) const {
    return std::any_of(nets.begin(), nets.end(), [this, &nets](const NetId& net) {
      const auto complement = differential_complement(net);
      return complement && nets.contains(*complement);
    });
  }

  [[nodiscard]] bool contains_complete_routing_bus(const std::set<NetId>& nets) const {
    return std::any_of(
        routing_bus_cross_sections.begin(), routing_bus_cross_sections.end(),
        [&nets](const auto& entry) { return contains_complete_bus(entry.second, nets); });
  }

  [[nodiscard]] std::uint64_t required_lane_gap(const NetId& first, const NetId& second) const {
    const auto first_clearance =
        clearance_nm_by_net.contains(first) ? clearance_nm_by_net.at(first) : std::uint64_t{0};
    const auto second_clearance =
        clearance_nm_by_net.contains(second) ? clearance_nm_by_net.at(second) : std::uint64_t{0};
    auto gap = std::max(first_clearance, second_clearance);
    if (const auto complement = differential_complement(first);
        complement && *complement == second) {
      const auto key = first < second ? std::pair{first, second} : std::pair{second, first};
      if (const auto found = differential_pair_cross_sections.find(key);
          found != differential_pair_cross_sections.end()) {
        gap = std::max(gap, found->second.gap_nm);
      }
    }
    const auto* first_bus = routing_bus(first);
    const auto* second_bus = routing_bus(second);
    if (first_bus != nullptr && second_bus != nullptr && first_bus->id == second_bus->id) {
      const auto first_index =
          std::find(first_bus->ordered_nets.begin(), first_bus->ordered_nets.end(), first);
      const auto second_index =
          std::find(first_bus->ordered_nets.begin(), first_bus->ordered_nets.end(), second);
      if (first_index != first_bus->ordered_nets.end() &&
          second_index != first_bus->ordered_nets.end()) {
        const auto first_offset =
            static_cast<std::size_t>(first_index - first_bus->ordered_nets.begin());
        const auto second_offset =
            static_cast<std::size_t>(second_index - first_bus->ordered_nets.begin());
        if (first_offset + 1U == second_offset) {
          gap = std::max(gap, first_bus->gaps_nm[first_offset]);
        } else if (second_offset + 1U == first_offset) {
          gap = std::max(gap, first_bus->gaps_nm[second_offset]);
        }
      }
    }
    return gap;
  }
};

[[nodiscard]] std::uint64_t saturating_add(std::uint64_t lhs, std::uint64_t rhs) {
  return rhs > std::numeric_limits<std::uint64_t>::max() - lhs
             ? std::numeric_limits<std::uint64_t>::max()
             : lhs + rhs;
}

[[nodiscard]] std::size_t saturating_count_add(std::size_t lhs, std::size_t rhs) {
  return std::numeric_limits<std::size_t>::max() - lhs < rhs
             ? std::numeric_limits<std::size_t>::max()
             : lhs + rhs;
}

[[nodiscard]] std::uint64_t saturating_multiply(std::uint64_t lhs, std::uint64_t rhs) {
  if (lhs == 0U || rhs == 0U) {
    return 0U;
  }
  return rhs > std::numeric_limits<std::uint64_t>::max() / lhs
             ? std::numeric_limits<std::uint64_t>::max()
             : lhs * rhs;
}

[[nodiscard]] std::size_t voronoi_history_increment(std::uint64_t overflow_nm,
                                                    std::uint64_t quantum_nm,
                                                    bool scale_by_overflow) {
  if (overflow_nm == 0U) {
    return 0U;
  }
  if (!scale_by_overflow || quantum_nm == 0U) {
    return 1U;
  }
  auto increment = overflow_nm / quantum_nm;
  if (overflow_nm % quantum_nm != 0U) {
    increment = saturating_add(increment, 1U);
  }
  return increment > std::numeric_limits<std::size_t>::max()
             ? std::numeric_limits<std::size_t>::max()
             : static_cast<std::size_t>(increment);
}

struct WidthReductionPenalty {
  std::uint64_t equivalent_length_nm{0};
  std::uint64_t cost{0};
};

[[nodiscard]] WidthReductionPenalty width_reduction_penalty(Point first, Point second,
                                                            Nanometres nominal_width,
                                                            Nanometres selected_width,
                                                            Nanometres grid_pitch,
                                                            std::uint64_t cost_per_grid_step) {
  if (nominal_width.value() <= 0 || selected_width.value() <= 0 ||
      selected_width >= nominal_width || grid_pitch.value() <= 0) {
    return {};
  }
  const auto length = geometry::point_distance_nm(first, second);
  const auto normalized = length *
                          static_cast<long double>(nominal_width.value() - selected_width.value()) /
                          static_cast<long double>(nominal_width.value());
  if (!(normalized > 0.0L)) {
    return {};
  }
  const auto maximum = static_cast<long double>(std::numeric_limits<std::uint64_t>::max());
  const auto equivalent_length = normalized >= maximum
                                     ? std::numeric_limits<std::uint64_t>::max()
                                     : static_cast<std::uint64_t>(std::ceil(normalized));
  const auto pitch = static_cast<std::uint64_t>(grid_pitch.value());
  auto steps = equivalent_length / pitch;
  if (equivalent_length % pitch != 0U) {
    steps = saturating_add(steps, 1U);
  }
  return WidthReductionPenalty{equivalent_length, saturating_multiply(steps, cost_per_grid_step)};
}

[[nodiscard]] WidthReductionPenalty path_width_reduction_penalty(
    const GlobalRoutePath& path, Nanometres nominal_width, const GlobalRouterConfig& config,
    std::optional<GlobalRouteSegmentWidthKind> selected_kind = std::nullopt) {
  WidthReductionPenalty result;
  if (path.segment_widths.size() + 1U != path.points.size() ||
      path.segment_width_kinds.size() != path.segment_widths.size()) {
    return result;
  }
  for (std::size_t index = 0U; index < path.segment_widths.size(); ++index) {
    if (selected_kind && path.segment_width_kinds[index] != *selected_kind) {
      continue;
    }
    const auto penalty = width_reduction_penalty(
        path.points[index].position, path.points[index + 1U].position, nominal_width,
        path.segment_widths[index], config.grid_pitch, config.width_reduction_cost_per_grid_step);
    result.equivalent_length_nm =
        saturating_add(result.equivalent_length_nm, penalty.equivalent_length_nm);
    result.cost = saturating_add(result.cost, penalty.cost);
  }
  return result;
}

struct PlannedVoronoiAllocation {
  std::uint64_t occupied_nm{0};
  std::uint64_t planned_nm{0};
};

[[nodiscard]] std::optional<PlannedVoronoiAllocation> planned_voronoi_allocation(
    const SharedVoronoiAllocationGraph& graph, const RoutingOccupancy& occupancy, const NetId& net,
    std::size_t resource_id, Nanometres selected_width, bool direct_resource) {
  const auto candidate_demand = graph.demand_for_width(net, selected_width);
  const auto& resource = graph.resources[resource_id];
  if (!direct_resource && candidate_demand > resource.capacity_nm) {
    return std::nullopt;
  }
  const auto found = occupancy.voronoi_resources.find(resource_id);
  const auto demand_found = occupancy.voronoi_demand_nm_by_net.find(resource_id);
  const std::set<NetId> empty;
  const std::map<NetId, std::uint64_t> empty_demands;
  const auto& nets = found == occupancy.voronoi_resources.end() ? empty : found->second;
  const auto& demands = demand_found == occupancy.voronoi_demand_nm_by_net.end()
                            ? empty_demands
                            : demand_found->second;
  auto existing_nets = nets;
  existing_nets.erase(net);
  auto existing_demands = demands;
  existing_demands.erase(net);
  const auto occupied = graph.occupied_demand(existing_nets, existing_demands);
  auto planned_nets = std::move(existing_nets);
  planned_nets.insert(net);
  auto planned_demands = std::move(existing_demands);
  auto planned_net_demand = candidate_demand;
  const bool direct_medial_neck_down =
      direct_resource && graph.medial_neck_down_width(resource_id, net) == selected_width;
  const auto existing_direct = occupancy.direct_medial_neck_downs.find(resource_id);
  const bool existing_direct_medial_neck_down =
      existing_direct != occupancy.direct_medial_neck_downs.end() &&
      existing_direct->second.contains(net);
  if (const auto existing = demands.find(net);
      existing != demands.end() && !direct_medial_neck_down && !existing_direct_medial_neck_down) {
    planned_net_demand = std::max(planned_net_demand, existing->second);
  }
  planned_demands[net] = planned_net_demand;
  return PlannedVoronoiAllocation{occupied, graph.occupied_demand(planned_nets, planned_demands)};
}

[[nodiscard]] bool voronoi_allocation_edge_over_capacity(const SharedVoronoiAllocationGraph& graph,
                                                         const RoutingOccupancy& occupancy,
                                                         const NetId& net, const Edge& edge,
                                                         Nanometres selected_width,
                                                         bool use_direct_edges,
                                                         std::set<NetId>* blocking_nets = nullptr) {
  bool over_capacity = false;
  for (const auto resource_id : graph.resources_for(edge, use_direct_edges)) {
    const bool direct_resource = graph.direct_resource(edge) == resource_id;
    const auto allocation = planned_voronoi_allocation(graph, occupancy, net, resource_id,
                                                       selected_width, direct_resource);
    if (allocation && allocation->planned_nm > graph.resources[resource_id].capacity_nm) {
      over_capacity = true;
      if (blocking_nets != nullptr) {
        if (const auto found = occupancy.voronoi_resources.find(resource_id);
            found != occupancy.voronoi_resources.end()) {
          blocking_nets->insert(found->second.begin(), found->second.end());
          blocking_nets->erase(net);
        }
      }
    }
  }
  return over_capacity;
}

[[nodiscard]] std::uint64_t voronoi_allocation_edge_cost(
    const SharedVoronoiAllocationGraph& graph, const RoutingOccupancy& occupancy,
    const RoutingHistory& history, const NetId& net, bool complement_allocated,
    bool routing_bus_member_allocated, const Edge& edge, Nanometres selected_width,
    const GlobalRouterConfig& config, bool use_direct_edges = true) {
  std::uint64_t result = 0U;
  if (config.negotiate_voronoi_lane_conflicts) {
    const RoutingHistory::SelectedJunctionEdgeKey key{net, edge};
    const auto junction_history = history.selected_junction_edges.contains(key)
                                      ? history.selected_junction_edges.at(key)
                                      : 0U;
    result =
        saturating_add(result, saturating_multiply(static_cast<std::uint64_t>(junction_history),
                                                   config.voronoi_lane_conflict_history_cost));
  }
  const auto selected_resources = graph.resources_for(edge, use_direct_edges);
  if (config.negotiate_voronoi_lane_conflicts && selected_resources.size() == 2U) {
    const RoutingHistory::ExactJunctionKey key{
        net, Edge{selected_resources.front(), selected_resources.back()}};
    const auto junction_history =
        history.exact_junctions.contains(key) ? history.exact_junctions.at(key) : 0U;
    result =
        saturating_add(result, saturating_multiply(static_cast<std::uint64_t>(junction_history),
                                                   config.voronoi_lane_conflict_history_cost));
  }
  for (const auto resource_id : selected_resources) {
    const auto& resource = graph.resources[resource_id];
    const bool direct_resource = graph.direct_resource(edge) == resource_id;
    const auto allocation = planned_voronoi_allocation(graph, occupancy, net, resource_id,
                                                       selected_width, direct_resource);
    if (!allocation) {
      continue;
    }
    const auto found = occupancy.voronoi_resources.find(resource_id);
    const std::set<NetId> empty;
    const auto& nets = found == occupancy.voronoi_resources.end() ? empty : found->second;
    result = saturating_add(
        result, saturating_multiply(allocation->occupied_nm, config.voronoi_resource_present_cost) /
                    resource.capacity_nm);
    if (allocation->planned_nm > resource.capacity_nm) {
      const auto overflow = allocation->planned_nm - resource.capacity_nm;
      result = saturating_add(result, config.voronoi_resource_present_cost);
      result = saturating_add(result,
                              saturating_multiply(overflow, config.voronoi_resource_present_cost) /
                                  resource.capacity_nm);
    }
    const auto resource_history = history.voronoi_resources.contains(resource_id)
                                      ? history.voronoi_resources.at(resource_id)
                                      : 0U;
    result =
        saturating_add(result, saturating_multiply(static_cast<std::uint64_t>(resource_history),
                                                   config.voronoi_resource_history_cost));
    if (config.negotiate_voronoi_lane_conflicts) {
      const RoutingHistory::LaneConflictResourceKey key{net, resource_id};
      const auto conflict_history = history.lane_conflict_resources.contains(key)
                                        ? history.lane_conflict_resources.at(key)
                                        : 0U;
      result =
          saturating_add(result, saturating_multiply(static_cast<std::uint64_t>(conflict_history),
                                                     config.voronoi_lane_conflict_history_cost));
    }
    if (direct_resource && config.negotiate_voronoi_lane_conflicts) {
      for (const auto portal_node : {edge.first, edge.second}) {
        const RoutingHistory::DirectPortalKey key{net, resource_id, portal_node};
        const auto portal_history =
            history.direct_portals.contains(key) ? history.direct_portals.at(key) : 0U;
        result =
            saturating_add(result, saturating_multiply(static_cast<std::uint64_t>(portal_history),
                                                       config.voronoi_lane_conflict_history_cost));
      }
    }
    if (complement_allocated) {
      const auto complement = graph.differential_complement(net);
      if (complement && !nets.contains(*complement)) {
        result = saturating_add(result, config.differential_pair_resource_separation_cost);
      }
    }
    if (routing_bus_member_allocated) {
      if (const auto* bus = graph.routing_bus(net);
          bus != nullptr && std::none_of(bus->ordered_nets.begin(), bus->ordered_nets.end(),
                                         [&nets, &net](const NetId& member) {
                                           return member != net && nets.contains(member);
                                         })) {
        result = saturating_add(result, config.routing_bus_resource_separation_cost);
      }
    }
  }
  return result;
}

class RoutingObstacleIndex {
public:
  RoutingObstacleIndex(const Board& board, const Grid& grid, bool include_existing_copper,
                       bool include_saved_zone_fills)
      : board_(board), grid_(grid), mutable_geometry_(std::make_shared<ImmutableGeometry>()),
        geometry_(mutable_geometry_) {
    mutable_geometry_->rules_by_net.emplace(NetId{}, resolve_net_routing_rules(board, NetId{}));
    for (const auto& net : board.nets) {
      mutable_geometry_->rules_by_net.emplace(net.id, resolve_net_routing_rules(board, net.id));
    }
    const auto account_rules = [this](const ResolvedNetRoutingRules& rules, bool pair_member) {
      mutable_geometry_->maximum_track_radius_nm =
          std::max(mutable_geometry_->maximum_track_radius_nm,
                   static_cast<long double>(rules.track_width.value()) / 2.0L);
      mutable_geometry_->maximum_planned_radius_nm =
          std::max(mutable_geometry_->maximum_planned_radius_nm,
                   std::max(static_cast<long double>(rules.track_width.value()) / 2.0L,
                            static_cast<long double>(rules.via_diameter.value()) / 2.0L));
      mutable_geometry_->maximum_clearance_nm =
          std::max(mutable_geometry_->maximum_clearance_nm,
                   static_cast<long double>(
                       (pair_member ? std::max(rules.clearance, rules.differential_pair_gap)
                                    : rules.clearance)
                           .value()));
    };
    if (board.nets.empty()) {
      account_rules(rules_for(NetId{}), false);
    } else {
      for (const auto& net : board.nets) {
        const bool pair_member =
            std::any_of(board.differential_pairs.begin(), board.differential_pairs.end(),
                        [&net](const DifferentialPair& pair) {
                          return pair.positive_net == net.id || pair.negative_net == net.id;
                        });
        account_rules(rules_for(net.id), pair_member);
      }
    }
    mutable_geometry_->bins.resize(grid_.layers.size() * grid_.layer_size());
    if (include_existing_copper) {
      for (const auto& track : board.tracks) {
        const auto layer = layer_index(track.layer);
        if (layer) {
          add_obstacle(IndexedObstacle{
              track.id, track.net, *layer, ObstacleShape::segment, track.start, track.end,
              static_cast<long double>(track.width.value()) / 2.0L, std::nullopt, std::nullopt});
        }
      }
      for (const auto& arc : board.track_arcs) {
        const auto layer = layer_index(arc.layer);
        const auto critical = geometry::circular_arc_extrema(arc.start, arc.middle, arc.end);
        if (layer && !critical.empty()) {
          auto min_point = critical.front();
          auto max_point = critical.front();
          for (const auto point : critical) {
            min_point.x = std::min(min_point.x, point.x);
            min_point.y = std::min(min_point.y, point.y);
            max_point.x = std::max(max_point.x, point.x);
            max_point.y = std::max(max_point.y, point.y);
          }
          add_obstacle(IndexedObstacle{arc.id, arc.net, *layer, ObstacleShape::circular_arc,
                                       min_point, max_point,
                                       static_cast<long double>(arc.width.value()) / 2.0L,
                                       std::nullopt, std::nullopt, std::nullopt, arc});
        }
      }
      for (const auto& via : board.vias) {
        for (const auto& layer_id : via.layers) {
          const auto layer = layer_index(layer_id);
          if (layer) {
            add_obstacle(IndexedObstacle{
                via.id, via.net, *layer, ObstacleShape::point, via.position, via.position,
                static_cast<long double>(via.diameter.value()) / 2.0L, std::nullopt, std::nullopt});
          }
        }
      }
    }
    for (const auto& pad : copper_pad_shapes(board)) {
      for (const auto& layer_id : pad.layers) {
        const auto layer = layer_index(layer_id);
        if (layer) {
          add_obstacle(IndexedObstacle{pad.id, pad.net, *layer, ObstacleShape::pad, pad.position,
                                       pad.position, pad.conservative_radius_nm, pad,
                                       std::nullopt});
        }
      }
    }
    if (include_saved_zone_fills) {
      for (const auto& zone : board.zones) {
        for (const auto& fill : zone.fills) {
          const auto layer = layer_index(fill.layer);
          if (!layer || fill.area.vertices.empty()) {
            continue;
          }
          auto min_x = fill.area.vertices.front().x;
          auto max_x = min_x;
          auto min_y = fill.area.vertices.front().y;
          auto max_y = min_y;
          for (const auto point : fill.area.vertices) {
            min_x = std::min(min_x, point.x);
            max_x = std::max(max_x, point.x);
            min_y = std::min(min_y, point.y);
            max_y = std::max(max_y, point.y);
          }
          add_obstacle(IndexedObstacle{zone.id, zone.net, *layer, ObstacleShape::zone,
                                       Point{min_x, min_y}, Point{max_x, max_y}, 0.0L, std::nullopt,
                                       std::nullopt, index_zone_fill(fill)});
        }
      }
    }
    for (const auto& keepout : world_keepouts(board)) {
      if (!keepout.prohibit.tracks && !keepout.prohibit.vias) {
        continue;
      }
      auto min_x = keepout.area.vertices.front().x;
      auto max_x = min_x;
      auto min_y = keepout.area.vertices.front().y;
      auto max_y = min_y;
      for (const auto point : keepout.area.vertices) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
      }
      for (const auto& layer_id : keepout.layers) {
        const auto layer = layer_index(layer_id);
        if (layer) {
          add_obstacle(IndexedObstacle{keepout.id, NetId{""}, *layer, ObstacleShape::keepout,
                                       Point{min_x, min_y}, Point{max_x, max_y}, 0.0L, std::nullopt,
                                       keepout});
        }
      }
    }
    mutable_geometry_.reset();
  }

  [[nodiscard]] long double maximum_planned_radius_nm() const {
    return geometry_->maximum_planned_radius_nm;
  }

  [[nodiscard]] long double maximum_clearance_nm() const { return geometry_->maximum_clearance_nm; }

  [[nodiscard]] bool valid() const { return geometry_->valid; }
  [[nodiscard]] const std::string& diagnostic() const { return geometry_->diagnostic; }

  // Static portfolio workers share only this frozen geometry. Query caches and counters start
  // empty so no worker mutates state observed by another worker.
  [[nodiscard]] RoutingObstacleIndex fork_worker() const {
    return RoutingObstacleIndex(board_, grid_, geometry_);
  }

  // Reduction occurs in stable NetTask order. Besides retaining diagnostic accounting, absorbing
  // the worker caches makes later negotiated routing reuse exactly the static probes already made.
  void absorb_worker(RoutingObstacleIndex&& worker) {
    if (geometry_.get() != worker.geometry_.get()) {
      throw std::logic_error("cannot merge routing obstacle caches from different geometry");
    }
    zone_hole_broad_phase_rejections_ = saturating_count_add(
        zone_hole_broad_phase_rejections_, worker.zone_hole_broad_phase_rejections_);
    zone_hole_exact_checks_ =
        saturating_count_add(zone_hole_exact_checks_, worker.zone_hole_exact_checks_);
    for (auto& [key, blockage] : worker.edge_cache_) {
      edge_cache_.try_emplace(std::move(key), std::move(blockage));
    }
    for (auto& [key, blockage] : worker.access_cache_) {
      access_cache_.try_emplace(std::move(key), std::move(blockage));
    }
  }

  [[nodiscard]] std::pair<std::size_t, std::size_t> query_cache_entry_counts() const {
    return {edge_cache_.size(), access_cache_.size()};
  }

  void release_query_caches() {
    edge_cache_.clear();
    access_cache_.clear();
  }

  [[nodiscard]] bool edge_blocked(const NetId& net, const Edge& edge) {
    return edge_blocked(net, edge, rules_for(net).track_width);
  }

  [[nodiscard]] bool edge_blocked(const NetId& net, const Edge& edge, Nanometres width) {
    const auto key = EdgeWidthKey{net, edge, width};
    if (const auto found = edge_cache_.find(key); found != edge_cache_.end()) {
      return found->second.blocked();
    }
    const auto [first_x, first_y, first_layer] = grid_.decode(edge.first);
    const auto [second_x, second_y, second_layer] = grid_.decode(edge.second);
    const auto& rules = rules_for(net);
    const auto clearance = static_cast<long double>(rules.clearance.value());
    EdgeBlockage blockage;
    if (first_layer == second_layer) {
      blockage = segment_blockage(net, {first_layer}, grid_.point(first_x, first_y),
                                  grid_.point(second_x, second_y),
                                  static_cast<long double>(width.value()) / 2.0L, clearance, false);
    } else {
      const auto position = grid_.point(first_x, first_y);
      blockage = segment_blockage(net, {first_layer, second_layer}, position, position,
                                  static_cast<long double>(rules.via_diameter.value()) / 2.0L,
                                  clearance, true);
    }
    const auto [inserted, success] = edge_cache_.emplace(key, std::move(blockage));
    static_cast<void>(success);
    return inserted->second.blocked();
  }

  [[nodiscard]] std::vector<ObjectId> edge_blocking_objects(const NetId& net, const Edge& edge,
                                                            Nanometres width) {
    static_cast<void>(edge_blocked(net, edge, width));
    const auto found = edge_cache_.find(EdgeWidthKey{net, edge, width});
    return found == edge_cache_.end() ? std::vector<ObjectId>{} : found->second.obstacles;
  }

  [[nodiscard]] bool pin_access_blocked(const NetId& net, std::size_t layer, Point exact,
                                        Point snapped) {
    return pin_access_blocked(net, layer, exact, snapped, rules_for(net).track_width);
  }

  [[nodiscard]] bool pin_access_blocked(const NetId& net, std::size_t layer, Point exact,
                                        Point snapped, Nanometres width) {
    AccessKey key{net, layer, exact, snapped, width};
    if (const auto found = access_cache_.find(key); found != access_cache_.end()) {
      return found->second.blocked();
    }
    const auto& rules = rules_for(net);
    auto blockage = segment_blockage(net, {layer}, exact, snapped,
                                     static_cast<long double>(width.value()) / 2.0L,
                                     static_cast<long double>(rules.clearance.value()), false);
    const auto [inserted, success] = access_cache_.emplace(std::move(key), std::move(blockage));
    static_cast<void>(success);
    return inserted->second.blocked();
  }

  [[nodiscard]] std::vector<ObjectId> pin_access_blocking_objects(const NetId& net,
                                                                  std::size_t layer, Point exact,
                                                                  Point snapped, Nanometres width) {
    static_cast<void>(pin_access_blocked(net, layer, exact, snapped, width));
    const auto found = access_cache_.find(AccessKey{net, layer, exact, snapped, width});
    return found == access_cache_.end() ? std::vector<ObjectId>{} : found->second.obstacles;
  }

  [[nodiscard]] bool corridor_edge_free(const Edge& edge) const {
    const auto [first_x, first_y, first_layer] = grid_.decode(edge.first);
    const auto [second_x, second_y, second_layer] = grid_.decode(edge.second);
    if (first_layer != second_layer) {
      return false;
    }
    return !segment_blockage(NetId{}, {first_layer}, grid_.point(first_x, first_y),
                             grid_.point(second_x, second_y), geometry_->maximum_track_radius_nm,
                             geometry_->maximum_clearance_nm, false, true)
                .blocked();
  }

  [[nodiscard]] bool corridor_edge_free(const Edge& edge, Nanometres width,
                                        Nanometres clearance) const {
    const auto [first_x, first_y, first_layer] = grid_.decode(edge.first);
    const auto [second_x, second_y, second_layer] = grid_.decode(edge.second);
    if (first_layer != second_layer || width.value() <= 0 || clearance.value() < 0) {
      return false;
    }
    return !segment_blockage(NetId{}, {first_layer}, grid_.point(first_x, first_y),
                             grid_.point(second_x, second_y),
                             static_cast<long double>(width.value()) / 2.0L,
                             static_cast<long double>(clearance.value()), false, true)
                .blocked();
  }

  [[nodiscard]] bool static_segment_free(const NetId& net, std::size_t layer, Point start,
                                         Point end, long double radius_nm,
                                         long double clearance_nm) const {
    return !segment_blockage(net, {layer}, start, end, radius_nm, clearance_nm, false).blocked();
  }

  void populate_feedback(GlobalRouteResult& result, std::size_t failed_pin_escapes,
                         const std::vector<PinEscapeFailure>& pin_escape_failures,
                         std::size_t pin_access_nodes, std::size_t adaptive_pin_access_nodes,
                         std::size_t terminals_with_adaptive_pin_access,
                         std::size_t isolated_pin_access_nodes_rejected,
                         std::size_t terminals_with_isolated_pin_access_rejections,
                         std::size_t terminal_neck_down_access_nodes,
                         std::size_t terminals_with_neck_down_access,
                         std::size_t coupled_terminals_ineligible_for_neck_down) const {
    std::map<std::pair<NetId, ObjectId>, std::size_t> impacts;
    const auto account = [&](const NetId& net, const EdgeBlockage& blockage) {
      if (!blockage.blocked()) {
        return;
      }
      ++result.blocked_search_edges;
      if (blockage.board_edge) {
        ++result.board_edge_rejections;
      }
      for (const auto& obstacle : blockage.obstacles) {
        ++impacts[{net, obstacle}];
      }
    };
    for (const auto& [key, blockage] : edge_cache_) {
      account(key.net, blockage);
    }
    for (const auto& [key, blockage] : access_cache_) {
      account(key.net, blockage);
    }
    result.failed_pin_escapes = failed_pin_escapes;
    result.pin_escape_failures = pin_escape_failures;
    result.zone_hole_broad_phase_rejections = zone_hole_broad_phase_rejections_;
    result.zone_hole_exact_checks = zone_hole_exact_checks_;
    result.pin_access_nodes = pin_access_nodes;
    result.adaptive_pin_access_nodes = adaptive_pin_access_nodes;
    result.terminals_with_adaptive_pin_access = terminals_with_adaptive_pin_access;
    result.isolated_pin_access_nodes_rejected = isolated_pin_access_nodes_rejected;
    result.terminals_with_isolated_pin_access_rejections =
        terminals_with_isolated_pin_access_rejections;
    result.terminal_neck_down_access_nodes = terminal_neck_down_access_nodes;
    result.terminals_with_neck_down_access = terminals_with_neck_down_access;
    result.coupled_terminals_ineligible_for_neck_down = coupled_terminals_ineligible_for_neck_down;
    result.obstacle_impacts.reserve(impacts.size());
    for (const auto& [key, count] : impacts) {
      result.obstacle_impacts.push_back(RoutingObstacleImpact{key.first, key.second, count});
    }
  }

private:
  struct ImmutableGeometry {
    std::map<NetId, ResolvedNetRoutingRules> rules_by_net;
    long double maximum_planned_radius_nm{0.0L};
    long double maximum_track_radius_nm{0.0L};
    long double maximum_clearance_nm{0.0L};
    bool valid{true};
    std::string diagnostic;
    std::size_t spatial_reference_count{0};
    std::vector<IndexedObstacle> obstacles;
    std::vector<std::vector<std::size_t>> bins;
  };

  RoutingObstacleIndex(const Board& board, const Grid& grid,
                       std::shared_ptr<const ImmutableGeometry> geometry)
      : board_(board), grid_(grid), geometry_(std::move(geometry)) {}

  [[nodiscard]] const ResolvedNetRoutingRules& rules_for(const NetId& net) const {
    const auto found = geometry_->rules_by_net.find(net);
    return found == geometry_->rules_by_net.end() ? geometry_->rules_by_net.at(NetId{})
                                                  : found->second;
  }

  [[nodiscard]] std::optional<std::size_t> layer_index(const LayerId& id) const {
    const auto found = std::find(grid_.layers.begin(), grid_.layers.end(), id);
    return found == grid_.layers.end()
               ? std::nullopt
               : std::optional<std::size_t>{static_cast<std::size_t>(found - grid_.layers.begin())};
  }

  [[nodiscard]] std::int64_t bin_coordinate(long double coordinate, std::int64_t origin) const {
    return static_cast<std::int64_t>(std::floor((coordinate - static_cast<long double>(origin)) /
                                                static_cast<long double>(grid_.pitch)));
  }

  [[nodiscard]] std::size_t bin_key(std::size_t x, std::size_t y, std::size_t layer) const {
    return layer * grid_.layer_size() + y * grid_.width + x;
  }

  void add_obstacle(IndexedObstacle obstacle) {
    if (!mutable_geometry_ || !mutable_geometry_->valid) {
      return;
    }
    const auto expansion = obstacle.radius_nm + mutable_geometry_->maximum_planned_radius_nm +
                           mutable_geometry_->maximum_clearance_nm;
    const auto min_x_value =
        static_cast<long double>(std::min(obstacle.start.x.value(), obstacle.end.x.value())) -
        expansion;
    const auto max_x_value =
        static_cast<long double>(std::max(obstacle.start.x.value(), obstacle.end.x.value())) +
        expansion;
    const auto min_y_value =
        static_cast<long double>(std::min(obstacle.start.y.value(), obstacle.end.y.value())) -
        expansion;
    const auto max_y_value =
        static_cast<long double>(std::max(obstacle.start.y.value(), obstacle.end.y.value())) +
        expansion;
    const auto max_x_index = static_cast<std::int64_t>(grid_.width - 1U);
    const auto max_y_index = static_cast<std::int64_t>(grid_.height - 1U);
    const auto min_x =
        std::clamp(bin_coordinate(min_x_value, grid_.origin_x), std::int64_t{0}, max_x_index);
    const auto max_x =
        std::clamp(bin_coordinate(max_x_value, grid_.origin_x), std::int64_t{0}, max_x_index);
    const auto min_y =
        std::clamp(bin_coordinate(min_y_value, grid_.origin_y), std::int64_t{0}, max_y_index);
    const auto max_y =
        std::clamp(bin_coordinate(max_y_value, grid_.origin_y), std::int64_t{0}, max_y_index);
    const auto width = static_cast<std::size_t>(max_x - min_x + 1);
    const auto height = static_cast<std::size_t>(max_y - min_y + 1);
    constexpr std::size_t maximum_spatial_references = 20'000'000;
    if (width > maximum_spatial_references / height ||
        mutable_geometry_->spatial_reference_count > maximum_spatial_references - width * height) {
      mutable_geometry_->valid = false;
      mutable_geometry_->diagnostic = "routing obstacle spatial index exceeds its safety limit";
      return;
    }
    const auto obstacle_index = mutable_geometry_->obstacles.size();
    mutable_geometry_->obstacles.push_back(std::move(obstacle));
    for (auto y = min_y; y <= max_y; ++y) {
      for (auto x = min_x; x <= max_x; ++x) {
        mutable_geometry_
            ->bins[bin_key(static_cast<std::size_t>(x), static_cast<std::size_t>(y),
                           mutable_geometry_->obstacles.back().layer)]
            .push_back(obstacle_index);
      }
    }
    mutable_geometry_->spatial_reference_count += width * height;
  }

  [[nodiscard]] std::vector<std::size_t> candidate_obstacles(const std::vector<std::size_t>& layers,
                                                             Point start, Point end) const {
    const auto max_x_index = static_cast<std::int64_t>(grid_.width - 1U);
    const auto max_y_index = static_cast<std::int64_t>(grid_.height - 1U);
    const auto min_x = std::clamp(
        bin_coordinate(static_cast<long double>(std::min(start.x.value(), end.x.value())),
                       grid_.origin_x),
        std::int64_t{0}, max_x_index);
    const auto max_x = std::clamp(
        bin_coordinate(static_cast<long double>(std::max(start.x.value(), end.x.value())),
                       grid_.origin_x),
        std::int64_t{0}, max_x_index);
    const auto min_y = std::clamp(
        bin_coordinate(static_cast<long double>(std::min(start.y.value(), end.y.value())),
                       grid_.origin_y),
        std::int64_t{0}, max_y_index);
    const auto max_y = std::clamp(
        bin_coordinate(static_cast<long double>(std::max(start.y.value(), end.y.value())),
                       grid_.origin_y),
        std::int64_t{0}, max_y_index);
    thread_local std::vector<std::uint32_t> generation_stamps;
    thread_local std::uint32_t generation = 0U;
    if (++generation == 0U) {
      std::fill(generation_stamps.begin(), generation_stamps.end(), 0U);
      generation = 1U;
    }
    if (generation_stamps.size() < geometry_->obstacles.size()) {
      generation_stamps.resize(geometry_->obstacles.size(), 0U);
    }
    std::vector<std::size_t> result;
    for (const auto layer : layers) {
      for (auto y = min_y; y <= max_y; ++y) {
        for (auto x = min_x; x <= max_x; ++x) {
          const auto key = bin_key(static_cast<std::size_t>(x), static_cast<std::size_t>(y), layer);
          if (key >= geometry_->bins.size()) {
            continue;
          }
          for (const auto obstacle : geometry_->bins[key]) {
            if (generation_stamps[obstacle] != generation) {
              generation_stamps[obstacle] = generation;
              result.push_back(obstacle);
            }
          }
        }
      }
    }
    return result;
  }

  struct IndexedZonePointLocation {
    geometry::PointLocation location{geometry::PointLocation::outside};
    std::optional<std::size_t> containing_hole;
  };

  void account_zone_hole_broad_phase_rejection() const {
    zone_hole_broad_phase_rejections_ = saturating_count_add(zone_hole_broad_phase_rejections_, 1U);
  }

  void account_zone_hole_exact_check() const {
    zone_hole_exact_checks_ = saturating_count_add(zone_hole_exact_checks_, 1U);
  }

  [[nodiscard]] IndexedZonePointLocation
  locate_point_in_indexed_zone(const IndexedCopperZoneFill& zone, Point point) const {
    const auto outer = geometry::locate_point(zone.prepared_area, point);
    if (outer != geometry::PointLocation::inside) {
      return IndexedZonePointLocation{outer, std::nullopt};
    }
    for (std::size_t index = 0U; index < zone.geometry.holes.size(); ++index) {
      if (!zone.prepared_holes[index].bounds.contains(point)) {
        account_zone_hole_broad_phase_rejection();
        continue;
      }
      account_zone_hole_exact_check();
      const auto hole = geometry::locate_point(zone.prepared_holes[index], point);
      if (hole == geometry::PointLocation::boundary) {
        return IndexedZonePointLocation{geometry::PointLocation::boundary, std::nullopt};
      }
      if (hole == geometry::PointLocation::inside) {
        return IndexedZonePointLocation{geometry::PointLocation::outside, index};
      }
    }
    return IndexedZonePointLocation{geometry::PointLocation::inside, std::nullopt};
  }

  [[nodiscard]] static long double
  segment_bounds_distance_lower_bound(Point start, Point end, const PolygonBounds& bounds) {
    const auto segment_min_x = std::min(start.x.value(), end.x.value());
    const auto segment_max_x = std::max(start.x.value(), end.x.value());
    const auto segment_min_y = std::min(start.y.value(), end.y.value());
    const auto segment_max_y = std::max(start.y.value(), end.y.value());
    const auto x_separation = segment_max_x < bounds.minimum.x.value()
                                  ? static_cast<long double>(bounds.minimum.x.value()) -
                                        static_cast<long double>(segment_max_x)
                              : bounds.maximum.x.value() < segment_min_x
                                  ? static_cast<long double>(segment_min_x) -
                                        static_cast<long double>(bounds.maximum.x.value())
                                  : 0.0L;
    const auto y_separation = segment_max_y < bounds.minimum.y.value()
                                  ? static_cast<long double>(bounds.minimum.y.value()) -
                                        static_cast<long double>(segment_max_y)
                              : bounds.maximum.y.value() < segment_min_y
                                  ? static_cast<long double>(segment_min_y) -
                                        static_cast<long double>(bounds.maximum.y.value())
                                  : 0.0L;
    return std::hypot(x_separation, y_separation);
  }

  [[nodiscard]] long double
  segment_indexed_zone_copper_distance_nm(Point start, Point end,
                                          const IndexedCopperZoneFill& zone) const {
    const auto start_location = locate_point_in_indexed_zone(zone, start);
    const auto end_location = locate_point_in_indexed_zone(zone, end);
    if (start_location.location != geometry::PointLocation::outside ||
        end_location.location != geometry::PointLocation::outside) {
      return 0.0L;
    }

    auto distance = geometry::segment_polygon_boundary_distance_nm(start, end, zone.prepared_area);
    if (distance <= 0.0L) {
      return 0.0L;
    }
    std::set<std::size_t> checked_holes;
    const auto exact_check = [&](std::size_t index) {
      if (!checked_holes.insert(index).second) {
        return;
      }
      account_zone_hole_exact_check();
      distance = std::min(distance, geometry::segment_polygon_boundary_distance_nm(
                                        start, end, zone.prepared_holes[index]));
    };
    if (start_location.containing_hole) {
      exact_check(*start_location.containing_hole);
    }
    if (end_location.containing_hole) {
      exact_check(*end_location.containing_hole);
    }
    if (distance <= 0.0L) {
      return 0.0L;
    }

    // The AABB-to-AABB distance is a lower bound on segment-to-contour distance. Preserve the
    // established two-nanometre conservative rounding guard before withholding an exact check.
    constexpr long double rounding_guard_nm = 2.0L;
    for (std::size_t index = 0U; index < zone.geometry.holes.size(); ++index) {
      if (checked_holes.contains(index)) {
        continue;
      }
      const auto& prepared_bounds = zone.prepared_holes[index].bounds;
      const PolygonBounds hole_bounds{prepared_bounds.minimum, prepared_bounds.maximum};
      if (segment_bounds_distance_lower_bound(start, end, hole_bounds) >
          distance + rounding_guard_nm) {
        account_zone_hole_broad_phase_rejection();
        continue;
      }
      exact_check(index);
      if (distance <= 0.0L) {
        return 0.0L;
      }
    }
    return distance;
  }

  [[nodiscard]] EdgeBlockage segment_blockage(const NetId& net,
                                              const std::vector<std::size_t>& layers, Point start,
                                              Point end, long double planned_radius_nm,
                                              long double planned_clearance_nm, bool planned_via,
                                              bool treat_all_copper_as_foreign = false) const {
    EdgeBlockage result;
    const auto edge_clearance =
        static_cast<long double>(board_.rules.copper_to_edge_clearance.value());
    if (!geometry::segment_inside_board_area(start, end, board_) ||
        geometry::segment_board_boundary_distance_nm(start, end, board_) <
            planned_radius_nm + edge_clearance) {
      result.board_edge = true;
    }
    for (const auto index : candidate_obstacles(layers, start, end)) {
      const auto& obstacle = geometry_->obstacles[index];
      if (std::find(layers.begin(), layers.end(), obstacle.layer) == layers.end()) {
        continue;
      }
      if (obstacle.shape == ObstacleShape::keepout) {
        const auto& keepout = *obstacle.keepout_geometry;
        const bool prohibited = planned_via ? keepout.prohibit.vias : keepout.prohibit.tracks;
        if (prohibited &&
            segment_copper_overlaps_keepout(start, end, planned_radius_nm, keepout.area)) {
          result.obstacles.push_back(obstacle.id);
        }
        continue;
      }
      if (!treat_all_copper_as_foreign && obstacle.net == net) {
        continue;
      }
      const auto is_pad = obstacle.shape == ObstacleShape::pad;
      const auto is_zone = obstacle.shape == ObstacleShape::zone;
      const auto pairwise_clearance =
          std::max(planned_clearance_nm,
                   static_cast<long double>(rules_for(obstacle.net).clearance.value()));
      auto planned_pair_clearance = pairwise_clearance;
      const auto is_complement =
          std::any_of(board_.differential_pairs.begin(), board_.differential_pairs.end(),
                      [&net, &obstacle](const DifferentialPair& pair) {
                        return (pair.positive_net == net && pair.negative_net == obstacle.net) ||
                               (pair.negative_net == net && pair.positive_net == obstacle.net);
                      });
      if (is_complement) {
        planned_pair_clearance = std::max(
            {planned_pair_clearance,
             static_cast<long double>(rules_for(net).differential_pair_gap.value()),
             static_cast<long double>(rules_for(obstacle.net).differential_pair_gap.value())});
      }
      const auto required =
          planned_radius_nm + planned_pair_clearance + (is_pad ? 0.0L : obstacle.radius_nm);
      const auto actual =
          is_pad    ? segment_to_pad_copper_distance_nm(start, end, *obstacle.pad_geometry)
          : is_zone ? segment_indexed_zone_copper_distance_nm(start, end, *obstacle.zone_geometry)
          : obstacle.shape == ObstacleShape::segment
              ? geometry::segment_distance_nm(start, end, obstacle.start, obstacle.end)
          : obstacle.shape == ObstacleShape::circular_arc
              ? geometry::segment_circular_arc_distance_nm(start, end, obstacle.arc_geometry->start,
                                                           obstacle.arc_geometry->middle,
                                                           obstacle.arc_geometry->end)
              : geometry::point_segment_distance_nm(obstacle.start, start, end);
      if (actual < required) {
        result.obstacles.push_back(obstacle.id);
      }
    }
    std::sort(result.obstacles.begin(), result.obstacles.end());
    result.obstacles.erase(std::unique(result.obstacles.begin(), result.obstacles.end()),
                           result.obstacles.end());
    return result;
  }

  const Board& board_;
  const Grid& grid_;
  std::shared_ptr<ImmutableGeometry> mutable_geometry_;
  std::shared_ptr<const ImmutableGeometry> geometry_;
  mutable std::size_t zone_hole_broad_phase_rejections_{0};
  mutable std::size_t zone_hole_exact_checks_{0};
  std::map<EdgeWidthKey, EdgeBlockage> edge_cache_;
  std::map<AccessKey, EdgeBlockage> access_cache_;
};

[[nodiscard]] std::optional<Grid> make_grid(const Board& board, const GlobalRouterConfig& config,
                                            std::string& diagnostic) {
  const bool neck_down_width_enabled = config.terminal_neck_down_width.value() > 0;
  const bool neck_down_length_enabled = config.max_terminal_neck_down_length.value() > 0;
  const bool medial_neck_down_width_enabled = config.medial_neck_down_width.value() > 0;
  const bool medial_neck_down_length_enabled = config.max_medial_neck_down_length.value() > 0;
  const bool corridor_neck_down_width_enabled = config.corridor_neck_down_width.value() > 0;
  const bool corridor_neck_down_length_enabled = config.max_corridor_neck_down_length.value() > 0;
  const bool local_pin_escape_enabled = config.local_pin_escape_pitch.value() > 0;
  const bool explicit_via_pitch = config.via_site_pitch.value() > 0;
  const bool invalid_via_pitch =
      config.via_site_pitch.value() < 0 ||
      (!explicit_via_pitch && (config.via_site_phase_x != 0U || config.via_site_phase_y != 0U)) ||
      (explicit_via_pitch &&
       (config.grid_pitch.value() <= 0 || config.via_site_pitch < config.grid_pitch ||
        config.via_site_pitch.value() % config.grid_pitch.value() != 0 ||
        config.via_site_phase_x >=
            static_cast<std::size_t>(config.via_site_pitch.value() / config.grid_pitch.value()) ||
        config.via_site_phase_y >=
            static_cast<std::size_t>(config.via_site_pitch.value() / config.grid_pitch.value())));
  const bool invalid_local_pin_escape =
      config.local_pin_escape_pitch.value() < 0 ||
      ((local_pin_escape_enabled || config.allocate_contended_terminal_fanout) &&
       config.max_local_pin_escape_expansions_per_terminal == 0U) ||
      (local_pin_escape_enabled &&
       (config.max_local_pin_escape_expansions_per_terminal == 0U ||
        config.local_pin_escape_pitch > config.grid_pitch ||
        config.grid_pitch.value() % config.local_pin_escape_pitch.value() != 0));
  const bool invalid_retry_weight = config.expansion_retry_heuristic_weight_per_mille != 0U &&
                                    (config.expansion_retry_heuristic_weight_per_mille <= 1'000U ||
                                     config.expansion_retry_heuristic_weight_per_mille > 10'000U);
  const bool invalid_frontier_reuse = config.reuse_weighted_search_frontier &&
                                      config.expansion_retry_heuristic_weight_per_mille == 0U;
  const bool invalid_checkpoint_objective =
      config.checkpoint_objective ==
          GlobalRouteCheckpointObjective::dimensioned_physical_overflow &&
      !config.use_voronoi_allocation_resources;
  const bool invalid_physical_history =
      config.scale_voronoi_history_by_overflow && !config.use_voronoi_allocation_resources;
  const bool invalid_physical_envelope_ordering =
      config.order_routing_groups_by_physical_envelope &&
      (!config.reorder_nets_by_pathfinder_pressure || !config.use_voronoi_allocation_resources);
  const bool invalid_lane_conflict_negotiation =
      config.negotiate_voronoi_lane_conflicts &&
      (!config.use_voronoi_allocation_resources ||
       config.voronoi_lane_conflict_history_cost == 0U ||
       config.max_voronoi_lane_conflict_history_entries == 0U);
  const bool invalid_voronoi_segmentation =
      config.max_voronoi_resource_segment_length.value() < 0 ||
      (config.max_voronoi_resource_segment_length.value() > 0 &&
       !config.use_voronoi_allocation_resources);
  const bool invalid_selective_reroute =
      config.selective_congestion_reroute && !config.atomic_complete_net_routing;
  const bool invalid_selective_refresh =
      config.selective_full_reroute_interval > 0U && !config.selective_congestion_reroute;
  const bool invalid_selective_hard_blockers =
      config.max_hard_capacity_blocker_reuse_invalidations_per_failed_net > 0U &&
      (!config.selective_congestion_reroute || config.hard_capacity_start_iteration == 0U);
  const bool invalid_search_diversification =
      config.enable_search_diversification &&
      (!config.atomic_complete_net_routing || config.global_route_beam_width < 2U ||
       config.global_route_beam_width > 32U || config.max_large_neighborhood_attempts == 0U ||
       config.large_neighborhood_stagnation_iterations == 0U ||
       config.destroy_route_percent == 0U || config.destroy_route_percent > 100U ||
       config.max_routing_order_variants == 0U || config.max_routing_order_variants > 16U);
  const bool invalid_hard_capacity = config.hard_capacity_start_iteration > config.max_iterations;
  const bool hard_ripup_attempts_enabled = config.max_hard_capacity_ripup_attempts_per_pass > 0U;
  const bool hard_ripup_nets_enabled = config.max_hard_capacity_ripup_nets_per_attempt > 0U;
  const bool invalid_hard_ripup =
      hard_ripup_attempts_enabled != hard_ripup_nets_enabled ||
      (hard_ripup_attempts_enabled &&
       (config.hard_capacity_start_iteration == 0U || !config.atomic_complete_net_routing)) ||
      (config.max_hard_capacity_conflict_set_states_per_attempt > 0U &&
       !hard_ripup_attempts_enabled) ||
      (config.rollback_failed_hard_capacity_ripup && !hard_ripup_attempts_enabled) ||
      (config.defer_hard_capacity_ripup_victims_to_pass_tail && !hard_ripup_attempts_enabled) ||
      (config.protect_recovered_hard_capacity_ripup_nets && !hard_ripup_attempts_enabled);
  const bool invalid_present_escalation =
      config.present_congestion_multiplier_per_mille < 1'000U ||
      config.max_present_congestion_scale_per_mille < 1'000U ||
      config.present_congestion_escalation_start_iteration < 2U ||
      config.max_present_congestion_scale_per_mille <
          config.present_congestion_multiplier_per_mille;
  const bool invalid_diagonal_policy =
      config.allow_diagonal_grid_edges && config.allow_crossing_safe_diagonal_grid_edges;
  const bool invalid_static_seed = config.max_static_seed_candidates_per_connection == 0U ||
                                   config.max_static_seed_searches == 0U ||
                                   config.max_static_seed_reprojection_searches == 0U ||
                                   config.static_seed_alternative_edge_penalty == 0U ||
                                   config.static_seed_corridor_departure_penalty == 0U ||
                                   config.max_static_seed_corridor_cells == 0U;
  const bool invalid_spline_refinement =
      config.global_spline_tessellation_tolerance.value() <= 0 ||
      config.global_spline_control_corridor.value() <= 0 ||
      config.global_spline_shove_step.value() <= 0 ||
      config.global_spline_max_tessellation_depth == 0U ||
      config.max_global_spline_samples_per_seed < 3U ||
      config.max_global_spline_shove_iterations == 0U ||
      config.global_navmesh_cell_pitch.value() < 0 ||
      config.max_global_navmesh_cells_per_seed == 0U ||
      config.max_global_navmesh_search_expansions_per_seed == 0U ||
      config.global_spline_minimum_bend_radius.value() < 0 ||
      config.global_spline_trajectory_samples < 3U ||
      config.max_global_spline_trajectory_iterations == 0U ||
      config.max_global_spline_trajectory_candidate_checks < 4U ||
      config.max_global_pns_propagation_depth == 0U ||
      config.max_global_pns_candidate_checks == 0U ||
      config.maximum_global_pns_displacement < config.global_spline_shove_step;
  if (config.grid_pitch.value() <= 0 || config.edge_capacity != 1U || config.max_iterations == 0U ||
      config.max_expansions_per_connection == 0U || invalid_retry_weight ||
      invalid_frontier_reuse || invalid_present_escalation || invalid_local_pin_escape ||
      invalid_via_pitch || invalid_diagonal_policy || invalid_static_seed ||
      invalid_spline_refinement || config.max_pin_access_nodes_per_terminal == 0U ||
      config.max_free_space_edges == 0U || config.clearance_shadow_cost_quantum.value() < 0 ||
      config.terminal_neck_down_width.value() < 0 ||
      config.max_terminal_neck_down_length.value() < 0 ||
      config.medial_neck_down_width.value() < 0 || config.max_medial_neck_down_length.value() < 0 ||
      config.corridor_neck_down_width.value() < 0 ||
      config.max_corridor_neck_down_length.value() < 0 ||
      config.max_corridor_neck_down_states == 0U ||
      neck_down_width_enabled != neck_down_length_enabled ||
      medial_neck_down_width_enabled != medial_neck_down_length_enabled ||
      corridor_neck_down_width_enabled != corridor_neck_down_length_enabled) {
    diagnostic = "router values must be positive and edge capacity must remain one until lanes "
                 "are materialized explicitly; terminal neck-down width and length must be "
                 "positive together; medial neck-down width and length must also be positive "
                 "together; corridor neck-down width and length must also be positive together "
                 "with a positive state budget; a nonzero retry heuristic weight must be in "
                 "(1000, 10000] per mille; present congestion multiplier/cap must be at least "
                 "1000 per mille, the cap must not be smaller than the multiplier, and escalation "
                 "must start no earlier than iteration two; local pin-escape pitch must be a "
                 "positive exact divisor of the global pitch with a positive expansion bound; "
                 "an explicit via-site pitch must be an exact positive multiple of the planar "
                 "grid pitch and its X/Y phases must lie within that stride; checkerboard and "
                 "crossing-safe diagonal policies are mutually exclusive; enabled early "
                 "spline/PNS guidance requires positive costs, geometry bounds, and effort "
                 "budgets";
    return std::nullopt;
  }
  if (local_pin_escape_enabled) {
    const auto stride = static_cast<std::uint64_t>(config.grid_pitch.value() /
                                                   config.local_pin_escape_pitch.value());
    const auto radius =
        (static_cast<std::uint64_t>(config.max_pin_escape_grid_radius) + 1U) * stride;
    const auto side = radius * 2U + 1U;
    constexpr std::uint64_t max_local_pin_escape_states = 5'000'000U;
    if (side == 0U || side > max_local_pin_escape_states / side) {
      diagnostic = "local pin-escape mesh exceeds its bounded state domain";
      return std::nullopt;
    }
  }
  if (invalid_voronoi_segmentation) {
    diagnostic = "longitudinal Voronoi resource segmentation requires shared Voronoi allocation "
                 "and a non-negative segment length";
    return std::nullopt;
  }
  if (invalid_checkpoint_objective) {
    diagnostic = "dimensioned physical-overflow objective requires shared Voronoi allocation";
    return std::nullopt;
  }
  if (invalid_physical_history) {
    diagnostic = "physical-overflow history requires shared Voronoi allocation resources";
    return std::nullopt;
  }
  if (invalid_physical_envelope_ordering) {
    diagnostic = "physical-envelope ordering requires pressure-aware shared Voronoi reroute "
                 "ordering";
    return std::nullopt;
  }
  if (invalid_lane_conflict_negotiation) {
    diagnostic = "Voronoi lane-conflict negotiation requires shared allocation, a positive "
                 "history cost, and a positive history-entry budget";
    return std::nullopt;
  }
  if (invalid_selective_reroute) {
    diagnostic = "selective congestion rerouting requires atomic complete-net routing";
    return std::nullopt;
  }
  if (invalid_selective_refresh) {
    diagnostic = "a selective full-reroute interval requires selective congestion rerouting";
    return std::nullopt;
  }
  if (invalid_selective_hard_blockers) {
    diagnostic = "selective hard-capacity blocker invalidation requires selective congestion "
                 "rerouting and a positive hard-capacity start iteration";
    return std::nullopt;
  }
  if (invalid_search_diversification) {
    diagnostic = "search diversification requires atomic complete-net routing, beam width 2-32, "
                 "positive neighborhood/stagnation/order bounds, and a destruction percentage "
                 "in [1, 100]";
    return std::nullopt;
  }
  if (invalid_hard_capacity) {
    diagnostic = "hard capacity must start within the configured routing pass bound";
    return std::nullopt;
  }
  if (invalid_hard_ripup) {
    diagnostic = "hard-capacity rip-up attempt/victim bounds must be positive together and "
                 "require hard capacity plus atomic complete-net routing; a positive joint "
                 "conflict-set state bound additionally requires enabled rip-up";
    return std::nullopt;
  }
  if (config.require_voronoi_topological_transitions && !config.use_voronoi_allocation_resources) {
    diagnostic = "strict Voronoi topology requires shared Voronoi allocation resources";
    return std::nullopt;
  }
  if (config.use_direct_voronoi_resource_edges && !config.use_voronoi_allocation_resources) {
    diagnostic = "direct Voronoi edges require shared Voronoi allocation resources";
    return std::nullopt;
  }
  if (medial_neck_down_width_enabled &&
      (!config.use_voronoi_allocation_resources || !config.use_direct_voronoi_resource_edges)) {
    diagnostic = "medial neck-down requires shared Voronoi allocation and direct medial edges";
    return std::nullopt;
  }
  if (corridor_neck_down_width_enabled && !config.use_free_space_corridors) {
    diagnostic = "corridor neck-down requires sampled free-space corridors";
    return std::nullopt;
  }
  long double minimum_planar_pitch = 0.0L;
  long double minimum_via_pitch = 0.0L;
  const auto account_rules = [&minimum_planar_pitch, &minimum_via_pitch](
                                 const ResolvedNetRoutingRules& rules, bool pair_member) {
    const auto clearance = static_cast<long double>(
        (pair_member ? std::max(rules.clearance, rules.differential_pair_gap) : rules.clearance)
            .value());
    minimum_planar_pitch = std::max(
        minimum_planar_pitch, static_cast<long double>(rules.track_width.value()) + clearance);
    minimum_via_pitch = std::max(minimum_via_pitch,
                                 static_cast<long double>(rules.via_diameter.value()) + clearance);
  };
  if (board.nets.empty()) {
    account_rules(resolve_net_routing_rules(board, NetId{}), false);
  } else {
    const std::set<NetId> selected(config.selected_nets.begin(), config.selected_nets.end());
    std::map<NetId, std::size_t> open_connections;
    if (selected.empty()) {
      for (const auto& connectivity : analyze_connectivity(board).nets) {
        open_connections.emplace(connectivity.net, connectivity.open_connections);
      }
    }
    for (const auto& net : board.nets) {
      const auto open = open_connections.find(net.id);
      const bool planned = !selected.empty() ? selected.contains(net.id)
                                             : open != open_connections.end() && open->second > 0U;
      if (!planned || net.name.empty()) {
        continue;
      }
      const bool pair_member =
          std::any_of(board.differential_pairs.begin(), board.differential_pairs.end(),
                      [&net](const DifferentialPair& pair) {
                        return pair.positive_net == net.id || pair.negative_net == net.id;
                      });
      account_rules(resolve_net_routing_rules(board, net.id), pair_member);
    }
  }
  const auto resolved_via_pitch =
      explicit_via_pitch ? config.via_site_pitch.value() : config.grid_pitch.value();
  const auto required_planar_pitch =
      config.allow_diagonal_grid_edges || config.allow_crossing_safe_diagonal_grid_edges
          ? minimum_planar_pitch * std::sqrt(2.0L)
          : minimum_planar_pitch;
  if (static_cast<long double>(config.grid_pitch.value()) < required_planar_pitch ||
      static_cast<long double>(resolved_via_pitch) < minimum_via_pitch) {
    diagnostic = "planar grid or via-site pitch cannot preserve resolved per-net copper spacing";
    return std::nullopt;
  }

  const auto [min_x_it, max_x_it] =
      std::minmax_element(board.outline.vertices.begin(), board.outline.vertices.end(),
                          [](Point lhs, Point rhs) { return lhs.x < rhs.x; });
  const auto [min_y_it, max_y_it] =
      std::minmax_element(board.outline.vertices.begin(), board.outline.vertices.end(),
                          [](Point lhs, Point rhs) { return lhs.y < rhs.y; });

  Grid grid;
  grid.origin_x = min_x_it->x.value();
  grid.origin_y = min_y_it->y.value();
  grid.pitch = config.grid_pitch.value();
  grid.via_stride = static_cast<std::size_t>(resolved_via_pitch / grid.pitch);
  grid.via_phase_x = config.via_site_phase_x;
  grid.via_phase_y = config.via_site_phase_y;
  grid.diagonal_edges = config.allow_diagonal_grid_edges;
  grid.crossing_safe_diagonal_edges = config.allow_crossing_safe_diagonal_grid_edges;
  const auto span_x = max_x_it->x.value() - grid.origin_x;
  const auto span_y = max_y_it->y.value() - grid.origin_y;
  grid.width = static_cast<std::size_t>(span_x / grid.pitch) + 1U;
  grid.height = static_cast<std::size_t>(span_y / grid.pitch) + 1U;

  constexpr std::size_t max_grid_nodes = 20'000'000;
  if (grid.width == 0U || grid.height == 0U || grid.width > max_grid_nodes / grid.height) {
    diagnostic = "routing grid is too large; increase grid pitch";
    return std::nullopt;
  }

  std::vector<std::pair<std::uint32_t, LayerId>> copper_layers;
  for (const auto& layer : board.layers) {
    if (layer.kind == LayerKind::copper) {
      copper_layers.emplace_back(layer.stack_order, layer.id);
    }
  }
  std::sort(copper_layers.begin(), copper_layers.end(), [](const auto& lhs, const auto& rhs) {
    return std::tie(lhs.first, lhs.second) < std::tie(rhs.first, rhs.second);
  });
  for (const auto& [order, id] : copper_layers) {
    static_cast<void>(order);
    grid.layers.push_back(id);
  }
  if (grid.layers.empty() || grid.layer_size() > max_grid_nodes / grid.layers.size()) {
    diagnostic = "routing grid requires at least one copper layer and bounded size";
    return std::nullopt;
  }

  if (corridor_neck_down_width_enabled) {
    const auto node_count = grid.layer_size() * grid.layers.size();
    const auto max_steps = static_cast<std::size_t>(config.max_corridor_neck_down_length.value() /
                                                    config.grid_pitch.value());
    if (max_steps == 0U || node_count > config.max_corridor_neck_down_states ||
        max_steps > (config.max_corridor_neck_down_states - node_count) / node_count) {
      diagnostic = "corridor neck-down expanded search exceeds its state budget or cannot span "
                   "one grid edge";
      return std::nullopt;
    }
  }

  grid.valid_cells.resize(grid.layer_size(), false);
  for (std::size_t y = 0; y < grid.height; ++y) {
    for (std::size_t x = 0; x < grid.width; ++x) {
      grid.valid_cells[y * grid.width + x] =
          geometry::locate_point_in_board_area(board, grid.point(x, y)) !=
          geometry::PointLocation::outside;
    }
  }
  return grid;
}

[[nodiscard]] std::optional<CorridorGraph>
build_corridor_graph(const Grid& grid, const GlobalRouterConfig& config,
                     const RoutingObstacleIndex& obstacles, std::stop_token stop_token,
                     std::string& diagnostic,
                     std::optional<Nanometres> explicit_width = std::nullopt,
                     Nanometres explicit_clearance = Nanometres{0}) {
  CorridorGraph result;
  if (!config.use_free_space_corridors) {
    return result;
  }
  const auto horizontal_per_layer = grid.width > 0U ? grid.height * (grid.width - 1U) : 0U;
  const auto vertical_per_layer = grid.height > 0U ? grid.width * (grid.height - 1U) : 0U;
  if (horizontal_per_layer > config.max_free_space_edges ||
      vertical_per_layer > config.max_free_space_edges - horizontal_per_layer) {
    diagnostic = "free-space corridor extraction exceeds its edge budget; increase grid pitch";
    return std::nullopt;
  }
  const auto edges_per_layer = horizontal_per_layer + vertical_per_layer;
  if (edges_per_layer == 0U) {
    return result;
  }
  if (grid.layers.size() > config.max_free_space_edges / edges_per_layer) {
    diagnostic = "free-space corridor extraction exceeds its edge budget; increase grid pitch";
    return std::nullopt;
  }
  constexpr auto no_corridor = std::numeric_limits<std::size_t>::max();
  const auto node_count = grid.layer_size() * grid.layers.size();
  result.vertical_stride = grid.width;
  result.horizontal_corridor_by_node.assign(node_count, no_corridor);
  result.vertical_corridor_by_node.assign(node_count, no_corridor);

  // Each run is a cross-section of parallel materializable lanes. Keeping the same
  // cross-section active along the routing direction creates a rectangular skeletal segment;
  // any boundary/capacity change terminates it so PathFinder sees the physical bottleneck.
  using Run = std::pair<std::size_t, std::size_t>;
  using ActiveRun = std::pair<std::size_t, std::size_t>;
  const auto add_corridor = [&](std::size_t layer, RoutingCorridorOrientation orientation,
                                Point first, Point last, std::size_t capacity,
                                std::size_t first_lane, std::size_t last_lane) {
    const auto id = result.corridors.size();
    result.corridors.push_back(CorridorDescriptor{id, layer, orientation, first, last, capacity,
                                                  false, first_lane, last_lane});
    return id;
  };
  const auto edge_free = [&](const Edge& edge) {
    return explicit_width ? obstacles.corridor_edge_free(edge, *explicit_width, explicit_clearance)
                          : obstacles.corridor_edge_free(edge);
  };

  for (std::size_t layer = 0; layer < grid.layers.size(); ++layer) {
    std::map<Run, ActiveRun> active_horizontal;
    for (std::size_t x = 0; x + 1U < grid.width; ++x) {
      if (stop_token.stop_requested()) {
        diagnostic = "global routing cancelled during free-space corridor extraction";
        return std::nullopt;
      }
      std::vector<Run> runs;
      std::vector<bool> free_edges(grid.height, false);
      for (std::size_t y = 0; y < grid.height; ++y) {
        free_edges[y] = grid.valid(x, y) && grid.valid(x + 1U, y) &&
                        edge_free(Edge{grid.node(x, y, layer), grid.node(x + 1U, y, layer)});
      }
      for (std::size_t y = 0; y < grid.height;) {
        if (!free_edges[y]) {
          ++y;
          continue;
        }
        const auto start = y;
        while (y < grid.height && free_edges[y]) {
          ++y;
        }
        runs.emplace_back(start, y - 1U);
      }
      for (const auto& run : runs) {
        auto id = std::numeric_limits<std::size_t>::max();
        if (const auto found = active_horizontal.find(run);
            found != active_horizontal.end() && found->second.second + 1U == x) {
          id = found->second.first;
          found->second.second = x;
          auto& corridor = result.corridors[id];
          corridor.last_lane_end = grid.point(x + 1U, run.second);
        } else {
          id = add_corridor(layer, RoutingCorridorOrientation::horizontal, grid.point(x, run.first),
                            grid.point(x + 1U, run.second), run.second - run.first + 1U, run.first,
                            run.second);
          active_horizontal[run] = {id, x};
        }
        for (auto y = run.first; y <= run.second; ++y) {
          result.horizontal_corridor_by_node[grid.node(x, y, layer)] = id;
        }
      }
    }

    std::map<Run, ActiveRun> active_vertical;
    for (std::size_t y = 0; y + 1U < grid.height; ++y) {
      if (stop_token.stop_requested()) {
        diagnostic = "global routing cancelled during free-space corridor extraction";
        return std::nullopt;
      }
      std::vector<Run> runs;
      std::vector<bool> free_edges(grid.width, false);
      for (std::size_t x = 0; x < grid.width; ++x) {
        free_edges[x] = grid.valid(x, y) && grid.valid(x, y + 1U) &&
                        edge_free(Edge{grid.node(x, y, layer), grid.node(x, y + 1U, layer)});
      }
      for (std::size_t x = 0; x < grid.width;) {
        if (!free_edges[x]) {
          ++x;
          continue;
        }
        const auto start = x;
        while (x < grid.width && free_edges[x]) {
          ++x;
        }
        runs.emplace_back(start, x - 1U);
      }
      for (const auto& run : runs) {
        auto id = std::numeric_limits<std::size_t>::max();
        if (const auto found = active_vertical.find(run);
            found != active_vertical.end() && found->second.second + 1U == y) {
          id = found->second.first;
          found->second.second = y;
          auto& corridor = result.corridors[id];
          corridor.last_lane_end = grid.point(run.second, y + 1U);
        } else {
          id = add_corridor(layer, RoutingCorridorOrientation::vertical, grid.point(run.first, y),
                            grid.point(run.second, y + 1U), run.second - run.first + 1U, run.first,
                            run.second);
          active_vertical[run] = {id, y};
        }
        for (auto x = run.first; x <= run.second; ++x) {
          result.vertical_corridor_by_node[grid.node(x, y, layer)] = id;
        }
      }
    }
  }

  const auto mark_neck_down = [&result](std::size_t first, std::size_t second) {
    constexpr auto none = std::numeric_limits<std::size_t>::max();
    if (first == none || second == none || first == second) {
      return;
    }
    if (result.corridors[first].capacity < result.corridors[second].capacity) {
      result.corridors[first].neck_down = true;
    } else if (result.corridors[second].capacity < result.corridors[first].capacity) {
      result.corridors[second].neck_down = true;
    }
  };
  for (std::size_t layer = 0; layer < grid.layers.size(); ++layer) {
    for (std::size_t y = 0; y < grid.height; ++y) {
      for (std::size_t x = 1; x + 1U < grid.width; ++x) {
        mark_neck_down(result.horizontal_corridor_by_node[grid.node(x - 1U, y, layer)],
                       result.horizontal_corridor_by_node[grid.node(x, y, layer)]);
      }
    }
    for (std::size_t x = 0; x < grid.width; ++x) {
      for (std::size_t y = 1; y + 1U < grid.height; ++y) {
        mark_neck_down(result.vertical_corridor_by_node[grid.node(x, y - 1U, layer)],
                       result.vertical_corridor_by_node[grid.node(x, y, layer)]);
      }
    }
  }
  return result;
}

[[nodiscard]] VoronoiCorridorEvidenceBundle
build_voronoi_corridor_evidence(const Board& board, const Grid& grid,
                                const std::vector<NetTask>& tasks, const GlobalRouterConfig& config,
                                std::stop_token stop_token) {
  VoronoiCorridorEvidenceBundle result;
  if (!config.emit_voronoi_corridor_evidence && !config.use_voronoi_pathfinder_guidance) {
    return result;
  }

  for (const auto& task : tasks) {
    const auto rules = resolve_net_routing_rules(board, task.net);
    const auto center_clearance = Nanometres{
        rules.clearance.value() + rules.track_width.value() / static_cast<std::int64_t>(2)};
    for (const auto& layer : grid.layers) {
      if (stop_token.stop_requested()) {
        result.cancelled = true;
        return result;
      }
      ++result.domains_attempted;
      const auto extracted = extract_polygonal_board_free_space(
          board,
          BoardFreeSpaceExtractionConfig{layer, task.net, config.respect_existing_copper, true,
                                         !config.treat_saved_zone_fills_as_refillable});
      result.conservative_obstacles += extracted.conservative_obstacles.size();
      if (!extracted.complete()) {
        ++result.domains_rejected;
        continue;
      }
      const auto offset = offset_polygonal_free_space(
          extracted.domain, FreeSpaceOffsetConfig{center_clearance, 1'000'000U});
      if (!offset.ok()) {
        ++result.domains_rejected;
        continue;
      }

      std::vector<RoutingVoronoiCorridorEvidence> domain_edges;
      std::size_t domain_curved_edge_chords = 0U;
      bool domain_ok = true;
      for (const auto& region : offset.regions) {
        FreeSpaceDomain region_domain;
        region_domain.outline = region.area;
        region_domain.cutouts = region.holes;
        const auto skeleton = build_free_space_skeleton(
            region_domain, FreeSpaceSkeletonConfig{Nanometres{}, 200'000U,
                                                   config.require_voronoi_topological_transitions});
        result.withheld_curved_edges += skeleton.skipped_curved_edges;
        if (!skeleton.ok() ||
            skeleton.edges.size() > config.max_voronoi_evidence_edges - result.edges.size() ||
            domain_edges.size() >
                config.max_voronoi_evidence_edges - result.edges.size() - skeleton.edges.size()) {
          domain_ok = false;
          break;
        }
        for (const auto& edge : skeleton.edges) {
          const auto capacity = skeleton_lane_capacity(edge, rules.track_width, rules.clearance);
          if (capacity == 0U) {
            continue;
          }
          domain_edges.push_back(RoutingVoronoiCorridorEvidence{
              0U, task.net, layer, edge.start, edge.end, edge.minimum_boundary_clearance, capacity,
              !extracted.conservative_obstacles.empty(), edge.curved_bisector_chord});
          domain_curved_edge_chords += edge.curved_bisector_chord ? 1U : 0U;
        }
      }
      if (!domain_ok) {
        ++result.domains_rejected;
        continue;
      }
      for (auto& edge : domain_edges) {
        edge.id = result.edges.size();
        result.edges.push_back(std::move(edge));
      }
      result.accepted_curved_edge_chords += domain_curved_edge_chords;
      ++result.domains_accepted;
    }
  }
  return result;
}

[[nodiscard]] std::optional<std::pair<std::size_t, std::size_t>> snap_to_grid(const Grid& grid,
                                                                              Point point) {
  const auto relative_x = point.x.value() - grid.origin_x;
  const auto relative_y = point.y.value() - grid.origin_y;
  auto x = static_cast<std::int64_t>(
      std::llround(static_cast<long double>(relative_x) / static_cast<long double>(grid.pitch)));
  auto y = static_cast<std::int64_t>(
      std::llround(static_cast<long double>(relative_y) / static_cast<long double>(grid.pitch)));
  x = std::clamp<std::int64_t>(x, 0, static_cast<std::int64_t>(grid.width - 1U));
  y = std::clamp<std::int64_t>(y, 0, static_cast<std::int64_t>(grid.height - 1U));
  if (grid.valid(static_cast<std::size_t>(x), static_cast<std::size_t>(y))) {
    return std::pair{static_cast<std::size_t>(x), static_cast<std::size_t>(y)};
  }

  const auto max_radius = std::max(grid.width, grid.height);
  for (std::size_t radius = 1; radius < max_radius; ++radius) {
    for (std::int64_t dy = -static_cast<std::int64_t>(radius);
         dy <= static_cast<std::int64_t>(radius); ++dy) {
      for (std::int64_t dx = -static_cast<std::int64_t>(radius);
           dx <= static_cast<std::int64_t>(radius); ++dx) {
        if (static_cast<std::size_t>(std::max(std::abs(dx), std::abs(dy))) != radius) {
          continue;
        }
        const auto candidate_x = x + dx;
        const auto candidate_y = y + dy;
        if (candidate_x >= 0 && candidate_y >= 0 &&
            candidate_x < static_cast<std::int64_t>(grid.width) &&
            candidate_y < static_cast<std::int64_t>(grid.height) &&
            grid.valid(static_cast<std::size_t>(candidate_x),
                       static_cast<std::size_t>(candidate_y))) {
          return std::pair{static_cast<std::size_t>(candidate_x),
                           static_cast<std::size_t>(candidate_y)};
        }
      }
    }
  }
  return std::nullopt;
}

[[nodiscard]] std::vector<NetTask> make_tasks(const Board& board, const Grid& grid,
                                              const GlobalRouterConfig& config) {
  std::vector<NetTask> tasks;
  const std::set<NetId> selected(config.selected_nets.begin(), config.selected_nets.end());
  const bool scoped = !selected.empty();
  std::map<NetId, std::size_t> source_open_connections;
  for (const auto& net : analyze_connectivity(board).nets) {
    source_open_connections.emplace(net.net, net.open_connections);
  }
  for (const auto& net : board.nets) {
    if (scoped && !selected.contains(net.id)) {
      continue;
    }
    const auto source_connectivity = source_open_connections.find(net.id);
    if (net.name.empty() || (!scoped && source_connectivity != source_open_connections.end() &&
                             source_connectivity->second == 0U)) {
      continue;
    }
    const bool differential_pair_member =
        std::any_of(board.differential_pairs.begin(), board.differential_pairs.end(),
                    [&net](const DifferentialPair& pair) {
                      return pair.positive_net == net.id || pair.negative_net == net.id;
                    });
    const bool coupled_member =
        differential_pair_member ||
        std::any_of(board.routing_buses.begin(), board.routing_buses.end(),
                    [&net](const RoutingBus& bus) {
                      return std::find(bus.ordered_nets.begin(), bus.ordered_nets.end(), net.id) !=
                             bus.ordered_nets.end();
                    });
    const auto explicit_priority =
        std::find(config.priority_nets.begin(), config.priority_nets.end(), net.id);
    const auto priority =
        explicit_priority == config.priority_nets.end()
            ? net.priority
            : std::numeric_limits<std::uint32_t>::max() -
                  static_cast<std::uint32_t>(explicit_priority - config.priority_nets.begin());
    const auto rules = resolve_net_routing_rules(board, net.id);
    NetTask task{net.id,
                 priority,
                 rules.track_width,
                 rules.via_diameter,
                 differential_pair_member ? std::max(rules.clearance, rules.differential_pair_gap)
                                          : rules.clearance,
                 coupled_member,
                 {}};
    for (const auto& footprint : board.footprints) {
      for (const auto& pad : footprint.pads) {
        if (pad.net != net.id) {
          continue;
        }
        Terminal terminal;
        terminal.footprint = footprint.id;
        terminal.pad = pad.id;
        terminal.exact_position = geometry::transform_point(pad.local_position, footprint.position,
                                                            footprint.rotation, footprint.side);
        const auto snapped = snap_to_grid(grid, terminal.exact_position);
        if (!snapped) {
          continue;
        }
        for (const auto& pad_layer : pad.layers) {
          const auto found = std::find(grid.layers.begin(), grid.layers.end(), pad_layer);
          if (found != grid.layers.end()) {
            const auto layer = static_cast<std::size_t>(found - grid.layers.begin());
            terminal.routing_layers.push_back(layer);
            terminal.grid_nodes.push_back(grid.node(snapped->first, snapped->second, layer));
          }
        }
        std::sort(terminal.routing_layers.begin(), terminal.routing_layers.end());
        terminal.routing_layers.erase(
            std::unique(terminal.routing_layers.begin(), terminal.routing_layers.end()),
            terminal.routing_layers.end());
        std::sort(terminal.grid_nodes.begin(), terminal.grid_nodes.end());
        terminal.grid_nodes.erase(
            std::unique(terminal.grid_nodes.begin(), terminal.grid_nodes.end()),
            terminal.grid_nodes.end());
        if (!terminal.grid_nodes.empty()) {
          task.terminals.push_back(std::move(terminal));
        }
      }
    }
    if (task.terminals.size() > 1U) {
      tasks.push_back(std::move(task));
    }
  }
  std::sort(tasks.begin(), tasks.end(), [](const auto& lhs, const auto& rhs) {
    if (lhs.priority != rhs.priority) {
      return lhs.priority > rhs.priority;
    }
    return lhs.net < rhs.net;
  });
  return tasks;
}

struct PinAccessStats {
  std::size_t failed{0};
  std::vector<PinEscapeFailure> failures;
  std::size_t nodes{0};
  std::size_t adaptive_nodes{0};
  std::size_t terminals_with_adaptive_access{0};
  std::size_t diversified_nodes{0};
  std::size_t terminals_with_diversified_access{0};
  std::size_t additional_successful_rings{0};
  std::size_t isolated_nodes_rejected{0};
  std::size_t terminals_with_isolated_rejections{0};
  std::size_t neck_down_nodes{0};
  std::size_t terminals_with_neck_down_access{0};
  std::size_t coupled_terminals_ineligible_for_neck_down{0};
  std::size_t local_paths{0};
  std::size_t local_expansions{0};
  std::size_t terminals_with_local_access{0};
  std::size_t same_net_fanout_paths{0};
  std::size_t terminals_with_same_net_fanout{0};
  std::size_t contended_terminals{0};
  std::size_t contention_triggered_local_paths{0};
};

[[nodiscard]] std::vector<std::size_t> sampled_planar_neighbors(const Grid& grid,
                                                                std::size_t node) {
  const auto [x, y, layer] = grid.decode(node);
  std::vector<std::size_t> result;
  result.reserve(grid.diagonal_edges || grid.crossing_safe_diagonal_edges ? 8U : 4U);
  const auto add = [&](std::int64_t dx, std::int64_t dy) {
    const auto next_x = static_cast<std::int64_t>(x) + dx;
    const auto next_y = static_cast<std::int64_t>(y) + dy;
    if (next_x < 0 || next_y < 0 || next_x >= static_cast<std::int64_t>(grid.width) ||
        next_y >= static_cast<std::int64_t>(grid.height) ||
        !grid.valid(static_cast<std::size_t>(next_x), static_cast<std::size_t>(next_y))) {
      return;
    }
    if (dx != 0 && dy != 0) {
      if (!grid.diagonal_edges && !grid.crossing_safe_diagonal_edges) {
        return;
      }
      if (!grid.crossing_safe_diagonal_edges) {
        const auto cell_x = std::min(static_cast<std::int64_t>(x), next_x);
        const auto cell_y = std::min(static_cast<std::int64_t>(y), next_y);
        const bool backslash = (cell_x + cell_y) % 2 == 0;
        if ((backslash && dx != dy) || (!backslash && dx == dy)) {
          return;
        }
      }
    }
    result.push_back(
        grid.node(static_cast<std::size_t>(next_x), static_cast<std::size_t>(next_y), layer));
  };
  add(-1, 0);
  add(1, 0);
  add(0, -1);
  add(0, 1);
  add(-1, -1);
  add(1, -1);
  add(1, 1);
  add(-1, 1);
  return result;
}

struct SameNetFanoutSearch {
  std::vector<Point> points;
  std::size_t expansions{0};
};

// Search a small exact-checked fine mesh between two terminals of one net. This represents local
// fanout topology that need not independently touch the coarse allocation lattice. Every accepted
// chord still passes the same authoritative obstacle predicate used by ordinary terminal access.
[[nodiscard]] std::optional<SameNetFanoutSearch>
search_same_net_terminal_fanout(const NetId& net, std::size_t layer, Point start, Point goal,
                                Nanometres width, std::int64_t fine_pitch,
                                std::int64_t detour_margin, std::size_t maximum_expansions,
                                RoutingObstacleIndex& obstacles) {
  if (fine_pitch <= 0 || detour_margin < 0 || maximum_expansions == 0U || start == goal) {
    return std::nullopt;
  }
  const auto floor_coordinate = [fine_pitch](std::int64_t value) {
    return static_cast<std::int64_t>(
        std::floor(static_cast<long double>(value) / static_cast<long double>(fine_pitch)));
  };
  const auto ceil_coordinate = [fine_pitch](std::int64_t value) {
    return static_cast<std::int64_t>(
        std::ceil(static_cast<long double>(value) / static_cast<long double>(fine_pitch)));
  };
  const auto minimum_x =
      floor_coordinate(std::min(start.x.value(), goal.x.value()) - detour_margin);
  const auto minimum_y =
      floor_coordinate(std::min(start.y.value(), goal.y.value()) - detour_margin);
  const auto maximum_x = ceil_coordinate(std::max(start.x.value(), goal.x.value()) + detour_margin);
  const auto maximum_y = ceil_coordinate(std::max(start.y.value(), goal.y.value()) + detour_margin);
  if (maximum_x < minimum_x || maximum_y < minimum_y) {
    return std::nullopt;
  }
  const auto width_states = static_cast<std::uint64_t>(maximum_x - minimum_x) + 1U;
  const auto height_states = static_cast<std::uint64_t>(maximum_y - minimum_y) + 1U;
  constexpr std::uint64_t maximum_states = 5'000'000U;
  if (width_states == 0U || height_states == 0U || width_states > maximum_states / height_states) {
    return std::nullopt;
  }
  const auto state_count = width_states * height_states;
  if (state_count > maximum_states || state_count > std::numeric_limits<std::size_t>::max()) {
    return std::nullopt;
  }
  const auto side = static_cast<std::size_t>(width_states);
  const auto state = [side, minimum_x, minimum_y](std::int64_t x, std::int64_t y) {
    return static_cast<std::size_t>(y - minimum_y) * side + static_cast<std::size_t>(x - minimum_x);
  };
  const auto coordinate = [side, minimum_x, minimum_y](std::size_t index) {
    return std::pair{minimum_x + static_cast<std::int64_t>(index % side),
                     minimum_y + static_cast<std::int64_t>(index / side)};
  };
  const auto point = [fine_pitch](std::int64_t x, std::int64_t y) {
    return Point{Nanometres{x * fine_pitch}, Nanometres{y * fine_pitch}};
  };
  const auto in_bounds = [minimum_x, minimum_y, maximum_x, maximum_y](std::int64_t x,
                                                                      std::int64_t y) {
    return x >= minimum_x && y >= minimum_y && x <= maximum_x && y <= maximum_y;
  };
  const auto rounded_coordinate = [fine_pitch](Nanometres value) {
    return static_cast<std::int64_t>(std::llround(static_cast<long double>(value.value()) /
                                                  static_cast<long double>(fine_pitch)));
  };

  struct QueueEntry {
    std::uint64_t cost{0U};
    std::size_t state{0U};
    auto operator<=>(const QueueEntry&) const = default;
  };
  constexpr auto infinity = std::numeric_limits<std::uint64_t>::max();
  constexpr auto no_parent = std::numeric_limits<std::size_t>::max();
  std::vector<std::uint64_t> distances(static_cast<std::size_t>(state_count), infinity);
  std::vector<std::size_t> parents(static_cast<std::size_t>(state_count), no_parent);
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open;
  const auto start_x = rounded_coordinate(start.x);
  const auto start_y = rounded_coordinate(start.y);
  for (std::int64_t dy = -1; dy <= 1; ++dy) {
    for (std::int64_t dx = -1; dx <= 1; ++dx) {
      const auto x = start_x + dx;
      const auto y = start_y + dy;
      if (!in_bounds(x, y)) {
        continue;
      }
      const auto seed = point(x, y);
      if (obstacles.pin_access_blocked(net, layer, start, seed, width)) {
        continue;
      }
      const auto index = state(x, y);
      const auto cost =
          static_cast<std::uint64_t>(std::ceil(geometry::point_distance_nm(start, seed)));
      if (cost < distances[index]) {
        distances[index] = cost;
        parents[index] = index;
        open.push(QueueEntry{cost, index});
      }
    }
  }

  std::set<std::size_t> goals;
  const auto goal_x = rounded_coordinate(goal.x);
  const auto goal_y = rounded_coordinate(goal.y);
  for (std::int64_t dy = -1; dy <= 1; ++dy) {
    for (std::int64_t dx = -1; dx <= 1; ++dx) {
      const auto x = goal_x + dx;
      const auto y = goal_y + dy;
      if (!in_bounds(x, y)) {
        continue;
      }
      const auto endpoint = point(x, y);
      if (!obstacles.pin_access_blocked(net, layer, endpoint, goal, width)) {
        goals.insert(state(x, y));
      }
    }
  }
  if (open.empty() || goals.empty()) {
    return std::nullopt;
  }

  static constexpr std::array<std::pair<std::int64_t, std::int64_t>, 8> offsets{
      {{-1, 0}, {0, -1}, {1, 0}, {0, 1}, {-1, -1}, {1, -1}, {1, 1}, {-1, 1}}};
  std::optional<std::size_t> reached;
  std::size_t expansions = 0U;
  while (!open.empty() && expansions < maximum_expansions) {
    const auto current = open.top();
    open.pop();
    if (current.cost != distances[current.state]) {
      continue;
    }
    ++expansions;
    if (goals.contains(current.state)) {
      reached = current.state;
      break;
    }
    const auto [x, y] = coordinate(current.state);
    const auto current_point = point(x, y);
    for (const auto [dx, dy] : offsets) {
      const auto next_x = x + dx;
      const auto next_y = y + dy;
      if (!in_bounds(next_x, next_y)) {
        continue;
      }
      const auto next_point = point(next_x, next_y);
      if (obstacles.pin_access_blocked(net, layer, current_point, next_point, width)) {
        continue;
      }
      const auto next = state(next_x, next_y);
      const auto next_cost =
          saturating_add(current.cost, geometry::segment_length_nm(current_point, next_point));
      if (next_cost < distances[next]) {
        distances[next] = next_cost;
        parents[next] = current.state;
        open.push(QueueEntry{next_cost, next});
      }
    }
  }
  if (!reached) {
    return SameNetFanoutSearch{{}, expansions};
  }
  std::vector<Point> points;
  auto current = *reached;
  while (true) {
    const auto [x, y] = coordinate(current);
    points.push_back(point(x, y));
    if (parents[current] == current) {
      break;
    }
    if (parents[current] == no_parent) {
      return SameNetFanoutSearch{{}, expansions};
    }
    current = parents[current];
  }
  std::reverse(points.begin(), points.end());
  points.insert(points.begin(), start);
  points.push_back(goal);
  points.erase(std::unique(points.begin(), points.end()), points.end());
  return SameNetFanoutSearch{std::move(points), expansions};
}

[[nodiscard]] PinAccessStats
filter_pin_access(std::vector<NetTask>& tasks, const Grid& grid, RoutingObstacleIndex& obstacles,
                  const CorridorGraph& corridor_neck_downs,
                  const SharedVoronoiAllocationGraph& voronoi_allocation,
                  const GlobalRouterConfig& config) {
  PinAccessStats stats;
  std::map<std::size_t, std::set<NetId>> nearest_portal_nets;
  for (const auto& task : tasks) {
    for (const auto& terminal : task.terminals) {
      for (const auto node : terminal.grid_nodes) {
        nearest_portal_nets[node].insert(task.net);
      }
    }
  }
  for (auto& task : tasks) {
    const bool neck_down_configured = config.terminal_neck_down_width.value() > 0 &&
                                      config.max_terminal_neck_down_length.value() > 0;
    const bool neck_down_eligible = neck_down_configured && !task.coupled_member &&
                                    config.terminal_neck_down_width < task.nominal_track_width;
    const bool corridor_neck_down_eligible =
        config.corridor_neck_down_width.value() > 0 && !task.coupled_member &&
        config.corridor_neck_down_width < task.nominal_track_width &&
        !corridor_neck_downs.corridors.empty();
    if (neck_down_configured && task.coupled_member &&
        config.terminal_neck_down_width < task.nominal_track_width) {
      stats.coupled_terminals_ineligible_for_neck_down += task.terminals.size();
    }
    for (auto& terminal : task.terminals) {
      const std::set<std::size_t> nearest_nodes(terminal.grid_nodes.begin(),
                                                terminal.grid_nodes.end());
      const bool terminal_portal_contended =
          std::any_of(nearest_nodes.begin(), nearest_nodes.end(), [&](const auto node) {
            const auto found = nearest_portal_nets.find(node);
            return found != nearest_portal_nets.end() && found->second.size() > 1U;
          });
      if (terminal_portal_contended) {
        ++stats.contended_terminals;
      }
      struct CandidateNode {
        std::size_t node{0};
        std::size_t radius{0};
        std::size_t manhattan{0};
        std::size_t layer{0};
        std::size_t x{0};
        std::size_t y{0};
      };
      std::vector<CandidateNode> candidates;
      for (const auto nearest : nearest_nodes) {
        const auto [base_x, base_y, layer] = grid.decode(nearest);
        for (std::size_t radius = 0; radius <= config.max_pin_escape_grid_radius; ++radius) {
          for (std::int64_t dy = -static_cast<std::int64_t>(radius);
               dy <= static_cast<std::int64_t>(radius); ++dy) {
            for (std::int64_t dx = -static_cast<std::int64_t>(radius);
                 dx <= static_cast<std::int64_t>(radius); ++dx) {
              if (static_cast<std::size_t>(std::max(std::abs(dx), std::abs(dy))) != radius) {
                continue;
              }
              const auto x = static_cast<std::int64_t>(base_x) + dx;
              const auto y = static_cast<std::int64_t>(base_y) + dy;
              if (x < 0 || y < 0 || x >= static_cast<std::int64_t>(grid.width) ||
                  y >= static_cast<std::int64_t>(grid.height) ||
                  !grid.valid(static_cast<std::size_t>(x), static_cast<std::size_t>(y))) {
                continue;
              }
              candidates.push_back(CandidateNode{
                  grid.node(static_cast<std::size_t>(x), static_cast<std::size_t>(y), layer),
                  radius, static_cast<std::size_t>(std::abs(dx) + std::abs(dy)), layer,
                  static_cast<std::size_t>(x), static_cast<std::size_t>(y)});
            }
          }
        }
      }
      std::sort(candidates.begin(), candidates.end(), [](const auto& lhs, const auto& rhs) {
        return std::tie(lhs.radius, lhs.manhattan, lhs.layer, lhs.y, lhs.x) <
               std::tie(rhs.radius, rhs.manhattan, rhs.layer, rhs.y, rhs.x);
      });
      candidates.erase(
          std::unique(candidates.begin(), candidates.end(),
                      [](const auto& lhs, const auto& rhs) { return lhs.node == rhs.node; }),
          candidates.end());
      std::vector<std::size_t> accessible_nodes;
      bool terminal_has_adaptive_access = false;
      bool terminal_has_diversified_access = false;
      bool terminal_has_isolated_rejection = false;
      bool terminal_has_neck_down_access = false;
      bool terminal_has_local_access = false;
      std::optional<std::size_t> first_successful_radius;
      std::optional<std::size_t> last_successful_radius;
      const auto portal_has_egress = [&](std::size_t node) {
        if (std::any_of(task.terminals.begin(), task.terminals.end(), [&](const Terminal& other) {
              return &other != &terminal &&
                     std::find(other.grid_nodes.begin(), other.grid_nodes.end(), node) !=
                         other.grid_nodes.end();
            })) {
          return true;
        }
        const auto [x, y, layer] = grid.decode(node);
        auto adjacent = sampled_planar_neighbors(grid, node);
        if (grid.via_site(x, y)) {
          if (layer > 0U) {
            adjacent.push_back(grid.node(x, y, layer - 1U));
          }
          if (layer + 1U < grid.layers.size()) {
            adjacent.push_back(grid.node(x, y, layer + 1U));
          }
        }
        if (config.use_direct_voronoi_resource_edges) {
          const auto& direct = voronoi_allocation.direct_neighbors(node);
          adjacent.insert(adjacent.end(), direct.begin(), direct.end());
        }
        std::sort(adjacent.begin(), adjacent.end());
        adjacent.erase(std::unique(adjacent.begin(), adjacent.end()), adjacent.end());
        for (const auto next : adjacent) {
          const Edge edge{node, next};
          if (!obstacles.edge_blocked(task.net, edge, task.nominal_track_width)) {
            return true;
          }
          const auto [next_x, next_y, next_layer] = grid.decode(next);
          static_cast<void>(next_x);
          static_cast<void>(next_y);
          if (layer == next_layer && corridor_neck_down_eligible &&
              corridor_neck_downs.corridor_for(edge) &&
              !obstacles.edge_blocked(task.net, edge, config.corridor_neck_down_width)) {
            return true;
          }
          if (config.use_direct_voronoi_resource_edges) {
            const auto resource = voronoi_allocation.direct_resource(edge);
            const auto width = resource
                                   ? voronoi_allocation.medial_neck_down_width(*resource, task.net)
                                   : std::nullopt;
            if (width && !obstacles.edge_blocked(task.net, edge, *width)) {
              return true;
            }
          }
        }
        return false;
      };
      for (const auto& candidate : candidates) {
        if (!config.diversify_pin_access_rings && first_successful_radius &&
            candidate.radius > *first_successful_radius) {
          break;
        }
        if (accessible_nodes.size() >= config.max_pin_access_nodes_per_terminal) {
          break;
        }
        const auto node = candidate.node;
        const auto layer = candidate.layer;
        const auto snapped = grid.point(candidate.x, candidate.y);
        const std::array access_candidates{
            std::vector<Point>{terminal.exact_position, Point{snapped.x, terminal.exact_position.y},
                               snapped},
            std::vector<Point>{terminal.exact_position, Point{terminal.exact_position.x, snapped.y},
                               snapped},
            std::vector<Point>{terminal.exact_position, snapped}};
        bool isolated_candidate_counted = false;
        for (auto path : access_candidates) {
          path.erase(std::unique(path.begin(), path.end()), path.end());
          const auto blocked_at_width = [&](Nanometres width) {
            bool blocked =
                path.size() == 1U &&
                obstacles.pin_access_blocked(task.net, layer, path.front(), path.front(), width);
            for (std::size_t i = 1; i < path.size() && !blocked; ++i) {
              blocked = obstacles.pin_access_blocked(task.net, layer, path[i - 1U], path[i], width);
            }
            return blocked;
          };
          bool blocked = blocked_at_width(task.nominal_track_width);
          bool neck_down = false;
          Nanometres access_width = task.nominal_track_width;
          if (blocked && neck_down_eligible && path.size() > 1U) {
            long double length_nm = 0.0L;
            for (std::size_t i = 1U; i < path.size(); ++i) {
              length_nm += geometry::point_distance_nm(path[i - 1U], path[i]);
            }
            if (length_nm > 0.0L &&
                length_nm <=
                    static_cast<long double>(config.max_terminal_neck_down_length.value()) &&
                !obstacles.pin_access_blocked(task.net, layer, snapped, snapped,
                                              task.nominal_track_width) &&
                !blocked_at_width(config.terminal_neck_down_width)) {
              blocked = false;
              neck_down = true;
              access_width = config.terminal_neck_down_width;
            }
          }
          if (!blocked) {
            if (!portal_has_egress(node)) {
              if (!isolated_candidate_counted) {
                ++stats.isolated_nodes_rejected;
                terminal_has_isolated_rejection = true;
                isolated_candidate_counted = true;
              }
              break;
            }
            accessible_nodes.push_back(node);
            terminal.access_paths.emplace(
                node, TerminalAccessPath{std::move(path), access_width, neck_down, {}});
            if (!first_successful_radius) {
              first_successful_radius = candidate.radius;
              last_successful_radius = candidate.radius;
            } else if (candidate.radius > *first_successful_radius) {
              ++stats.diversified_nodes;
              terminal_has_diversified_access = true;
              if (!last_successful_radius || candidate.radius > *last_successful_radius) {
                ++stats.additional_successful_rings;
                last_successful_radius = candidate.radius;
              }
            }
            if (!nearest_nodes.contains(node)) {
              ++stats.adaptive_nodes;
              terminal_has_adaptive_access = true;
            }
            if (neck_down) {
              ++stats.neck_down_nodes;
              terminal_has_neck_down_access = true;
            }
            break;
          }
        }
      }
      auto local_escape_pitch = config.local_pin_escape_pitch.value();
      bool contention_triggered_local_escape = false;
      if (local_escape_pitch == 0 && config.allocate_contended_terminal_fanout &&
          terminal_portal_contended) {
        if (grid.pitch % 4 == 0) {
          local_escape_pitch = grid.pitch / 4;
        } else if (grid.pitch % 2 == 0) {
          local_escape_pitch = grid.pitch / 2;
        }
        contention_triggered_local_escape = local_escape_pitch > 0;
      }
      if (accessible_nodes.size() < config.max_pin_access_nodes_per_terminal &&
          local_escape_pitch > 0) {
        struct LocalQueueEntry {
          std::uint64_t cost{0};
          std::size_t state{0};

          auto operator<=>(const LocalQueueEntry&) const = default;
        };
        const auto search_local_escape =
            [&](std::size_t layer, Nanometres width,
                bool neck_down) -> std::optional<std::pair<std::size_t, TerminalAccessPath>> {
          const auto fine_pitch = local_escape_pitch;
          const auto stride = grid.pitch / fine_pitch;
          const auto radius_steps =
              static_cast<std::int64_t>(config.max_pin_escape_grid_radius + 1U) * stride;
          const auto center_x = static_cast<std::int64_t>(std::llround(
              static_cast<long double>(terminal.exact_position.x.value() - grid.origin_x) /
              static_cast<long double>(fine_pitch)));
          const auto center_y = static_cast<std::int64_t>(std::llround(
              static_cast<long double>(terminal.exact_position.y.value() - grid.origin_y) /
              static_cast<long double>(fine_pitch)));
          const auto minimum_x = center_x - radius_steps;
          const auto minimum_y = center_y - radius_steps;
          const auto side = static_cast<std::size_t>(2 * radius_steps + 1);
          if (side == 0U || side > std::numeric_limits<std::size_t>::max() / side) {
            return std::nullopt;
          }
          const auto state_count = side * side;
          const auto state = [side, minimum_x, minimum_y](std::int64_t x, std::int64_t y) {
            return static_cast<std::size_t>(y - minimum_y) * side +
                   static_cast<std::size_t>(x - minimum_x);
          };
          const auto coordinate = [side, minimum_x, minimum_y](std::size_t index) {
            return std::pair{minimum_x + static_cast<std::int64_t>(index % side),
                             minimum_y + static_cast<std::int64_t>(index / side)};
          };
          const auto point = [&](std::int64_t x, std::int64_t y) {
            return Point{Nanometres{grid.origin_x + x * fine_pitch},
                         Nanometres{grid.origin_y + y * fine_pitch}};
          };
          const auto in_bounds = [&](std::int64_t x, std::int64_t y) {
            return x >= minimum_x && y >= minimum_y &&
                   x < minimum_x + static_cast<std::int64_t>(side) &&
                   y < minimum_y + static_cast<std::int64_t>(side);
          };

          std::map<std::size_t, std::size_t> goals;
          const auto minimum_goal_radius =
              accessible_nodes.empty()
                  ? 0U
                  : std::max<std::size_t>(1U, config.max_pin_escape_grid_radius / 2U);
          for (const auto& candidate : candidates) {
            if (candidate.layer != layer || candidate.radius < minimum_goal_radius ||
                std::find(accessible_nodes.begin(), accessible_nodes.end(), candidate.node) !=
                    accessible_nodes.end() ||
                !portal_has_egress(candidate.node)) {
              continue;
            }
            const auto portal = grid.point(candidate.x, candidate.y);
            const auto x = (portal.x.value() - grid.origin_x) / fine_pitch;
            const auto y = (portal.y.value() - grid.origin_y) / fine_pitch;
            if (!in_bounds(x, y) ||
                (neck_down && obstacles.pin_access_blocked(task.net, layer, portal, portal,
                                                           task.nominal_track_width))) {
              continue;
            }
            goals.try_emplace(state(x, y), candidate.node);
          }
          if (goals.empty()) {
            return std::nullopt;
          }

          constexpr auto infinity = std::numeric_limits<std::uint64_t>::max();
          constexpr auto no_parent = std::numeric_limits<std::size_t>::max();
          std::vector<std::uint64_t> distance(state_count, infinity);
          std::vector<std::size_t> parent(state_count, no_parent);
          std::priority_queue<LocalQueueEntry, std::vector<LocalQueueEntry>,
                              std::greater<LocalQueueEntry>>
              open;
          for (std::int64_t dy = -1; dy <= 1; ++dy) {
            for (std::int64_t dx = -1; dx <= 1; ++dx) {
              const auto x = center_x + dx;
              const auto y = center_y + dy;
              if (!in_bounds(x, y)) {
                continue;
              }
              const auto seed_point = point(x, y);
              if (obstacles.pin_access_blocked(task.net, layer, terminal.exact_position, seed_point,
                                               width)) {
                continue;
              }
              const auto index = state(x, y);
              const auto cost = static_cast<std::uint64_t>(
                  std::ceil(geometry::point_distance_nm(terminal.exact_position, seed_point)));
              if (cost < distance[index]) {
                distance[index] = cost;
                parent[index] = index;
                open.push(LocalQueueEntry{cost, index});
              }
            }
          }

          static constexpr std::array<std::pair<std::int64_t, std::int64_t>, 8> offsets{
              {{-1, 0}, {0, -1}, {1, 0}, {0, 1}, {-1, -1}, {1, -1}, {1, 1}, {-1, 1}}};
          std::optional<std::size_t> reached;
          std::size_t expansions = 0U;
          while (!open.empty() &&
                 expansions < config.max_local_pin_escape_expansions_per_terminal) {
            const auto current = open.top();
            open.pop();
            if (current.cost != distance[current.state]) {
              continue;
            }
            ++expansions;
            if (goals.contains(current.state)) {
              reached = current.state;
              break;
            }
            const auto [x, y] = coordinate(current.state);
            const auto current_point = point(x, y);
            for (const auto [dx, dy] : offsets) {
              const auto next_x = x + dx;
              const auto next_y = y + dy;
              if (!in_bounds(next_x, next_y)) {
                continue;
              }
              const auto next_point = point(next_x, next_y);
              if (obstacles.pin_access_blocked(task.net, layer, current_point, next_point, width)) {
                continue;
              }
              const auto next = state(next_x, next_y);
              const auto step = geometry::segment_length_nm(current_point, next_point);
              const auto next_cost = saturating_add(current.cost, step);
              if (next_cost < distance[next]) {
                distance[next] = next_cost;
                parent[next] = current.state;
                open.push(LocalQueueEntry{next_cost, next});
              }
            }
          }
          stats.local_expansions = saturating_count_add(stats.local_expansions, expansions);
          if (!reached) {
            return std::nullopt;
          }
          if (neck_down && distance[*reached] > static_cast<std::uint64_t>(
                                                    config.max_terminal_neck_down_length.value())) {
            return std::nullopt;
          }
          std::vector<Point> path;
          auto current = *reached;
          while (true) {
            const auto [x, y] = coordinate(current);
            path.push_back(point(x, y));
            if (parent[current] == current) {
              break;
            }
            if (parent[current] == no_parent) {
              return std::nullopt;
            }
            current = parent[current];
          }
          std::reverse(path.begin(), path.end());
          if (path.empty() || path.front() != terminal.exact_position) {
            path.insert(path.begin(), terminal.exact_position);
          }
          path.erase(std::unique(path.begin(), path.end()), path.end());
          return std::pair{goals.at(*reached),
                           TerminalAccessPath{std::move(path), width, neck_down, {}}};
        };

        std::set<std::size_t> candidate_layers;
        for (const auto& candidate : candidates) {
          candidate_layers.insert(candidate.layer);
        }
        for (const auto layer : candidate_layers) {
          auto found = search_local_escape(layer, task.nominal_track_width, false);
          if (!found && neck_down_eligible) {
            found = search_local_escape(layer, config.terminal_neck_down_width, true);
          }
          if (!found) {
            continue;
          }
          accessible_nodes.push_back(found->first);
          if (found->second.neck_down) {
            ++stats.neck_down_nodes;
            terminal_has_neck_down_access = true;
          }
          terminal.access_paths.emplace(found->first, std::move(found->second));
          ++stats.local_paths;
          if (contention_triggered_local_escape) {
            ++stats.contention_triggered_local_paths;
          }
          terminal_has_local_access = true;
          if (accessible_nodes.size() >= config.max_pin_access_nodes_per_terminal) {
            break;
          }
        }
      }
      terminal.grid_nodes = std::move(accessible_nodes);
      stats.nodes += terminal.grid_nodes.size();
      if (terminal_has_adaptive_access) {
        ++stats.terminals_with_adaptive_access;
      }
      if (terminal_has_diversified_access) {
        ++stats.terminals_with_diversified_access;
      }
      if (terminal_has_isolated_rejection) {
        ++stats.terminals_with_isolated_rejections;
      }
      if (terminal_has_neck_down_access) {
        ++stats.terminals_with_neck_down_access;
      }
      if (terminal_has_local_access) {
        ++stats.terminals_with_local_access;
      }
    }

    // Some dense fanouts contain a legal same-net local tree but no independent coarse portal for
    // every pad. Grow the portal-connected terminal component over a bounded fine mesh before
    // declaring those pads inaccessible. This is joint terminal topology, not another global
    // allocation grid: the resulting path is attached atomically to an already certified portal.
    auto fanout_pitch = config.local_pin_escape_pitch.value();
    if (fanout_pitch == 0) {
      if (grid.pitch % 4 == 0) {
        fanout_pitch = grid.pitch / 4;
      } else if (grid.pitch % 2 == 0) {
        fanout_pitch = grid.pitch / 2;
      }
    }
    const auto maximum_terminal_span =
        static_cast<long double>(config.max_pin_escape_grid_radius + 1U) *
        static_cast<long double>(grid.pitch);
    bool added_fanout = fanout_pitch > 0;
    while (added_fanout) {
      added_fanout = false;
      for (std::size_t source_index = 0U; source_index < task.terminals.size(); ++source_index) {
        auto& source = task.terminals[source_index];
        if (!source.grid_nodes.empty()) {
          continue;
        }
        std::vector<std::size_t> targets;
        for (std::size_t target_index = 0U; target_index < task.terminals.size(); ++target_index) {
          if (target_index == source_index || task.terminals[target_index].grid_nodes.empty() ||
              geometry::point_distance_nm(source.exact_position,
                                          task.terminals[target_index].exact_position) >
                  maximum_terminal_span) {
            continue;
          }
          targets.push_back(target_index);
        }
        std::sort(targets.begin(), targets.end(), [&](std::size_t lhs, std::size_t rhs) {
          const auto lhs_distance = geometry::point_distance_nm(source.exact_position,
                                                                task.terminals[lhs].exact_position);
          const auto rhs_distance = geometry::point_distance_nm(source.exact_position,
                                                                task.terminals[rhs].exact_position);
          return lhs_distance != rhs_distance ? lhs_distance < rhs_distance : lhs < rhs;
        });
        if (targets.size() > config.max_pin_access_nodes_per_terminal) {
          targets.resize(config.max_pin_access_nodes_per_terminal);
        }
        for (const auto target_index : targets) {
          const auto& target = task.terminals[target_index];
          bool connected_to_target = false;
          for (const auto layer : source.routing_layers) {
            const auto target_supports_layer = std::binary_search(
                target.routing_layers.begin(), target.routing_layers.end(), layer);
            if (!target_supports_layer) {
              continue;
            }
            auto local = search_same_net_terminal_fanout(
                task.net, layer, source.exact_position, target.exact_position,
                task.nominal_track_width, fanout_pitch, grid.pitch,
                config.max_local_pin_escape_expansions_per_terminal, obstacles);
            if (!local) {
              continue;
            }
            stats.local_expansions =
                saturating_count_add(stats.local_expansions, local->expansions);
            if (local->points.empty()) {
              continue;
            }
            for (const auto& [portal, target_access] : target.access_paths) {
              const auto [portal_x, portal_y, portal_layer] = grid.decode(portal);
              static_cast<void>(portal_x);
              static_cast<void>(portal_y);
              if (portal_layer != layer || target_access.neck_down ||
                  source.grid_nodes.size() >= config.max_pin_access_nodes_per_terminal) {
                continue;
              }
              auto points = local->points;
              if (!target_access.points.empty()) {
                points.insert(points.end(), std::next(target_access.points.begin()),
                              target_access.points.end());
              }
              points.erase(std::unique(points.begin(), points.end()), points.end());
              auto joined = target_access.joined_terminals;
              joined.push_back(target_index);
              std::sort(joined.begin(), joined.end());
              joined.erase(std::unique(joined.begin(), joined.end()), joined.end());
              source.grid_nodes.push_back(portal);
              source.access_paths.emplace(portal, TerminalAccessPath{std::move(points),
                                                                     task.nominal_track_width,
                                                                     false, std::move(joined)});
            }
            if (!source.grid_nodes.empty()) {
              std::sort(source.grid_nodes.begin(), source.grid_nodes.end());
              source.grid_nodes.erase(
                  std::unique(source.grid_nodes.begin(), source.grid_nodes.end()),
                  source.grid_nodes.end());
              stats.nodes = saturating_count_add(stats.nodes, source.grid_nodes.size());
              ++stats.same_net_fanout_paths;
              ++stats.terminals_with_same_net_fanout;
              added_fanout = true;
              connected_to_target = true;
              break;
            }
          }
          if (connected_to_target) {
            break;
          }
        }
      }
    }
    for (const auto& terminal : task.terminals) {
      if (!terminal.grid_nodes.empty()) {
        continue;
      }
      ++stats.failed;
      PinEscapeFailure failure{terminal.footprint,
                               terminal.pad,
                               task.net,
                               terminal.exact_position,
                               {},
                               {},
                               false,
                               false};
      for (const auto layer : terminal.routing_layers) {
        const auto nominal = obstacles.pin_access_blocking_objects(
            task.net, layer, terminal.exact_position, terminal.exact_position,
            task.nominal_track_width);
        failure.nominal_center_blockers.insert(failure.nominal_center_blockers.end(),
                                               nominal.begin(), nominal.end());
        failure.nominal_center_board_blocked =
            failure.nominal_center_board_blocked ||
            (obstacles.pin_access_blocked(task.net, layer, terminal.exact_position,
                                          terminal.exact_position, task.nominal_track_width) &&
             nominal.empty());
        if (neck_down_eligible) {
          const auto narrow = obstacles.pin_access_blocking_objects(
              task.net, layer, terminal.exact_position, terminal.exact_position,
              config.terminal_neck_down_width);
          failure.neck_down_center_blockers.insert(failure.neck_down_center_blockers.end(),
                                                   narrow.begin(), narrow.end());
          failure.neck_down_center_board_blocked =
              failure.neck_down_center_board_blocked ||
              (obstacles.pin_access_blocked(task.net, layer, terminal.exact_position,
                                            terminal.exact_position,
                                            config.terminal_neck_down_width) &&
               narrow.empty());
        }
      }
      std::sort(failure.nominal_center_blockers.begin(), failure.nominal_center_blockers.end());
      failure.nominal_center_blockers.erase(std::unique(failure.nominal_center_blockers.begin(),
                                                        failure.nominal_center_blockers.end()),
                                            failure.nominal_center_blockers.end());
      std::sort(failure.neck_down_center_blockers.begin(), failure.neck_down_center_blockers.end());
      failure.neck_down_center_blockers.erase(std::unique(failure.neck_down_center_blockers.begin(),
                                                          failure.neck_down_center_blockers.end()),
                                              failure.neck_down_center_blockers.end());
      stats.failures.push_back(std::move(failure));
    }
  }
  return stats;
}

[[nodiscard]] SearchResult
search_path(const Grid& grid, const std::vector<std::size_t>& starts,
            const std::set<std::size_t>& goals, const RoutingOccupancy& occupancy,
            const RoutingHistory& history, const CorridorGraph& corridors,
            const CorridorGraph& corridor_neck_downs,
            const VoronoiPathFinderGuidance& voronoi_guidance,
            const SharedVoronoiAllocationGraph& voronoi_allocation, RoutingObstacleIndex& obstacles,
            const NetTask& task, const GlobalRouterConfig& config, bool use_direct_edges,
            bool enforce_hard_capacity, std::uint32_t heuristic_weight_per_mille,
            const SearchCostPolicy& cost_policy, std::stop_token stop_token);

[[nodiscard]] StaticSeedPlan
build_static_hybrid_seed_plan(const Board& board, const Grid& grid,
                              const std::vector<NetTask>& tasks, const CorridorGraph& corridors,
                              const CorridorGraph& corridor_neck_downs,
                              const SharedVoronoiAllocationGraph& voronoi_allocation,
                              RoutingObstacleIndex& obstacles, const GlobalRouterConfig& config,
                              const RoutingHistory& initial_history, std::stop_token stop_token) {
  StaticSeedPlan plan;
  auto& stats = plan.stats;
  const auto node_count = grid.layer_size() * grid.layers.size();
  if (node_count == 0U) {
    stats.budget_exhausted = true;
    return plan;
  }

  const RoutingOccupancy empty_occupancy;
  const VoronoiPathFinderGuidance empty_guidance;

  struct NetSeedTaskResult {
    StaticSeedPlan plan;
    RoutingObstacleIndex obstacles;
    std::exception_ptr failure;

    explicit NetSeedTaskResult(RoutingObstacleIndex worker_obstacles)
        : obstacles(std::move(worker_obstacles)) {}
  };
  std::vector<std::unique_ptr<NetSeedTaskResult>> task_results(tasks.size());
  std::atomic_size_t next_task{0U};
  const auto hardware_workers = std::max(1U, std::thread::hardware_concurrency());
  const auto requested_workers =
      config.max_static_seed_workers == 0U ? hardware_workers : config.max_static_seed_workers;
  const auto worker_count = std::min<std::size_t>(tasks.size(), requested_workers);
  stats.tasks_dispatched = tasks.size();
  stats.workers_used = worker_count;
  const auto task_budget = [](std::size_t total, std::size_t index, std::size_t count) {
    return count == 0U ? 0U : total / count + (index < total % count ? 1U : 0U);
  };
  const auto run_task = [&]() {
    while (!stop_token.stop_requested()) {
      const auto task_index = next_task.fetch_add(1U, std::memory_order_relaxed);
      if (task_index >= tasks.size()) {
        return;
      }
      auto result = std::make_unique<NetSeedTaskResult>(obstacles.fork_worker());
      try {
        const auto& task = tasks[task_index];
        auto task_config = config;
        task_config.max_static_seed_searches =
            task_budget(config.max_static_seed_searches, task_index, tasks.size());
        auto& local_plan = result->plan;
        auto& local_stats = local_plan.stats;
        if (task.terminals.size() >= 2U && !task.terminals.front().grid_nodes.empty()) {
          local_stats.connections = task.terminals.size() - 1U;
          std::set<Edge> first_tree_edges;
          std::vector<std::vector<std::vector<std::size_t>>> retained_trees;
          for (std::size_t tree_candidate = 0U;
               tree_candidate < task_config.max_static_seed_candidates_per_connection;
               ++tree_candidate) {
            std::set<std::size_t> tree(task.terminals.front().grid_nodes.begin(),
                                       task.terminals.front().grid_nodes.end());
            std::vector<std::size_t> remaining;
            for (std::size_t terminal_index = 1U; terminal_index < task.terminals.size();
                 ++terminal_index) {
              remaining.push_back(terminal_index);
            }
            std::vector<GlobalSeedCandidate> branches;
            std::vector<std::vector<std::size_t>> topology;
            bool complete_tree = true;
            while (!remaining.empty()) {
              const auto fixed_terminal = *std::min_element(remaining.begin(), remaining.end());
              std::sort(remaining.begin(), remaining.end(), [&](const auto lhs, const auto rhs) {
                const auto& first = task.terminals[lhs];
                const auto& second = task.terminals[rhs];
                return std::tuple{first.grid_nodes.size(), first.footprint, first.pad, lhs} <
                       std::tuple{second.grid_nodes.size(), second.footprint, second.pad, rhs};
              });
              const auto terminal_index = remaining.front();
              remaining.erase(remaining.begin());
              local_stats.growth_reorderings = saturating_count_add(
                  local_stats.growth_reorderings, terminal_index == fixed_terminal ? 0U : 1U);
              const auto& terminal = task.terminals[terminal_index];
              if (terminal.grid_nodes.empty() || tree.empty() ||
                  local_stats.searches >= task_config.max_static_seed_searches) {
                local_stats.budget_exhausted =
                    local_stats.searches >= task_config.max_static_seed_searches;
                complete_tree = false;
                break;
              }
              const std::set<std::size_t> goals = tree;
              SearchCostPolicy policy;
              policy.kind = SearchCostPolicyKind::static_seed;
              policy.penalized_edges = tree_candidate == 0U ? std::set<Edge>{} : first_tree_edges;
              policy.alternative_edge_penalty = task_config.static_seed_alternative_edge_penalty;
              local_stats.penalty_reruns =
                  saturating_count_add(local_stats.penalty_reruns, tree_candidate == 0U ? 0U : 1U);
              ++local_stats.searches;
              auto route = search_path(grid, terminal.grid_nodes, goals, empty_occupancy,
                                       initial_history, corridors, corridor_neck_downs,
                                       empty_guidance, voronoi_allocation, result->obstacles, task,
                                       task_config, task_config.use_direct_voronoi_resource_edges,
                                       false, 1'000U, policy, stop_token);
              local_stats.search_expansions =
                  saturating_count_add(local_stats.search_expansions, route.expansions);
              if (route.cancelled) {
                break;
              }
              if (route.nodes.empty() && task_config.use_direct_voronoi_resource_edges) {
                if (local_stats.searches >= task_config.max_static_seed_searches) {
                  local_stats.budget_exhausted = true;
                  complete_tree = false;
                  break;
                }
                ++local_stats.searches;
                route = search_path(grid, terminal.grid_nodes, goals, empty_occupancy,
                                    initial_history, corridors, corridor_neck_downs, empty_guidance,
                                    voronoi_allocation, result->obstacles, task, task_config, false,
                                    false, 1'000U, policy, stop_token);
                local_stats.search_expansions =
                    saturating_count_add(local_stats.search_expansions, route.expansions);
              }
              if (route.nodes.empty()) {
                complete_tree = false;
                break;
              }
              topology.push_back(route.nodes);
              const bool joins_existing_tree = !branches.empty();
              branches.push_back(GlobalSeedCandidate{task.net,
                                                     terminal_index - 1U,
                                                     tree_candidate,
                                                     terminal_index,
                                                     branches.empty() ? std::vector<std::size_t>{0U}
                                                                      : std::vector<std::size_t>{},
                                                     terminal.grid_nodes,
                                                     goals,
                                                     std::move(route),
                                                     {},
                                                     {},
                                                     {},
                                                     false});
              branches.back().goals = {branches.back().route.nodes.back()};
              local_stats.tree_joins =
                  saturating_count_add(local_stats.tree_joins, joins_existing_tree ? 1U : 0U);
              if (branches.size() == 1U) {
                tree.clear();
              }
              tree.insert(branches.back().route.nodes.begin(), branches.back().route.nodes.end());
            }
            if (!complete_tree || branches.size() + 1U != task.terminals.size()) {
              ++local_stats.partial_trees;
              break;
            }
            if (std::find(retained_trees.begin(), retained_trees.end(), topology) !=
                retained_trees.end()) {
              ++local_stats.deduplicated_candidates;
              break;
            }
            ++local_stats.complete_trees;
            local_stats.alternative_trees =
                saturating_count_add(local_stats.alternative_trees, tree_candidate == 0U ? 0U : 1U);
            retained_trees.push_back(std::move(topology));
            if (tree_candidate == 0U) {
              for (const auto& branch : branches) {
                for (std::size_t index = 1U; index < branch.route.nodes.size(); ++index) {
                  first_tree_edges.emplace(branch.route.nodes[index - 1U],
                                           branch.route.nodes[index]);
                }
              }
            }
            local_stats.candidates = saturating_count_add(local_stats.candidates, branches.size());
            local_plan.candidates.insert(local_plan.candidates.end(),
                                         std::make_move_iterator(branches.begin()),
                                         std::make_move_iterator(branches.end()));
          }
        }
      } catch (...) {
        result->failure = std::current_exception();
      }
      task_results[task_index] = std::move(result);
    }
  };
  std::vector<std::jthread> workers;
  workers.reserve(worker_count);
  for (std::size_t index = 0U; index < worker_count; ++index) {
    workers.emplace_back(run_task);
  }
  workers.clear();

  // Stable slots decouple completion order from all externally visible candidate/metric order.
  for (auto& result : task_results) {
    if (!result) {
      continue;
    }
    if (result->failure) {
      std::rethrow_exception(result->failure);
    }
    const auto& local = result->plan.stats;
    stats.connections = saturating_count_add(stats.connections, local.connections);
    stats.searches = saturating_count_add(stats.searches, local.searches);
    stats.search_expansions =
        saturating_count_add(stats.search_expansions, local.search_expansions);
    stats.penalty_reruns = saturating_count_add(stats.penalty_reruns, local.penalty_reruns);
    stats.deduplicated_candidates =
        saturating_count_add(stats.deduplicated_candidates, local.deduplicated_candidates);
    stats.complete_trees = saturating_count_add(stats.complete_trees, local.complete_trees);
    stats.partial_trees = saturating_count_add(stats.partial_trees, local.partial_trees);
    stats.alternative_trees =
        saturating_count_add(stats.alternative_trees, local.alternative_trees);
    stats.tree_joins = saturating_count_add(stats.tree_joins, local.tree_joins);
    stats.growth_reorderings =
        saturating_count_add(stats.growth_reorderings, local.growth_reorderings);
    stats.budget_exhausted = stats.budget_exhausted || local.budget_exhausted;
    plan.candidates.insert(plan.candidates.end(),
                           std::make_move_iterator(result->plan.candidates.begin()),
                           std::make_move_iterator(result->plan.candidates.end()));
  }
  stats.candidates = plan.candidates.size();

  std::map<std::string, std::vector<NetId>> coupled_groups;
  std::map<std::string, std::uint64_t> coupled_group_demands;
  for (const auto& [pair, cross_section] : voronoi_allocation.differential_pair_cross_sections) {
    static_cast<void>(pair);
    const auto key =
        "pair:" + cross_section.positive_net.value() + ":" + cross_section.negative_net.value();
    coupled_groups.emplace(
        key, std::vector<NetId>{cross_section.positive_net, cross_section.negative_net});
    coupled_group_demands.emplace(key, cross_section.demand_nm);
  }
  for (const auto& [id, cross_section] : voronoi_allocation.routing_bus_cross_sections) {
    const auto key = "bus:" + id.value();
    coupled_groups.emplace(key, cross_section.ordered_nets);
    coupled_group_demands.emplace(key, cross_section.demand_nm);
  }
  std::set<NetId> coupled_nets;
  std::set<std::pair<NetId, std::size_t>> accepted_group_trees;
  const auto layer_sequence = [&](const GlobalSeedCandidate& candidate) {
    std::vector<std::size_t> result;
    for (const auto node : candidate.route.nodes) {
      const auto [x, y, layer] = grid.decode(node);
      static_cast<void>(x);
      static_cast<void>(y);
      if (result.empty() || result.back() != layer) {
        result.push_back(layer);
      }
    }
    return result;
  };
  const auto topology_signature = [&](const GlobalSeedCandidate& candidate) {
    std::vector<std::size_t> result;
    constexpr auto layer_tag = std::numeric_limits<std::size_t>::max();
    const auto resource_tag = voronoi_allocation.resources.size() + corridors.corridors.size() + 1U;
    const auto corridor_tag = resource_tag + voronoi_allocation.resources.size() + 1U;
    for (std::size_t index = 1U; index < candidate.route.nodes.size(); ++index) {
      const Edge edge{candidate.route.nodes[index - 1U], candidate.route.nodes[index]};
      auto resources = voronoi_allocation.resources_for(edge);
      if (!resources.empty()) {
        for (const auto resource : resources) {
          const auto signature = resource_tag + resource;
          if (result.empty() || result.back() != signature) {
            result.push_back(signature);
          }
        }
        continue;
      }
      if (const auto corridor = corridors.corridor_for(edge)) {
        const auto signature = corridor_tag + *corridor;
        if (result.empty() || result.back() != signature) {
          result.push_back(signature);
        }
        continue;
      }
      const auto [x, y, layer] = grid.decode(edge.first);
      static_cast<void>(x);
      static_cast<void>(y);
      const auto signature = layer_tag - layer;
      if (result.empty() || result.back() != signature) {
        result.push_back(signature);
      }
    }
    return result;
  };
  for (const auto& [group_id, members] : coupled_groups) {
    ++stats.joint_groups;
    coupled_nets.insert(members.begin(), members.end());
    std::size_t accepted_candidates = 0U;
    for (std::size_t tree_candidate = 0U;
         tree_candidate < config.max_static_seed_candidates_per_connection; ++tree_candidate) {
      std::vector<std::vector<std::vector<std::size_t>>> reference_sequences;
      std::vector<std::vector<std::vector<std::size_t>>> reference_topologies;
      bool complete = true;
      for (const auto& member : members) {
        const auto task = std::find_if(tasks.begin(), tasks.end(),
                                       [&](const auto& value) { return value.net == member; });
        if (task == tasks.end() || task->terminals.size() < 2U) {
          complete = false;
          break;
        }
        std::vector<const GlobalSeedCandidate*> branches(task->terminals.size() - 1U, nullptr);
        for (const auto& candidate : plan.candidates) {
          if (candidate.net == member && candidate.tree_candidate_index == tree_candidate &&
              candidate.connection_index < branches.size()) {
            branches[candidate.connection_index] = &candidate;
          }
        }
        if (std::any_of(branches.begin(), branches.end(),
                        [](const auto* branch) { return branch == nullptr; })) {
          complete = false;
          break;
        }
        std::vector<std::vector<std::size_t>> sequences;
        std::vector<std::vector<std::size_t>> topologies;
        for (const auto* branch : branches) {
          sequences.push_back(layer_sequence(*branch));
          topologies.push_back(topology_signature(*branch));
          for (std::size_t index = 1U; index < branch->route.nodes.size(); ++index) {
            const auto resources = voronoi_allocation.resources_for(
                Edge{branch->route.nodes[index - 1U], branch->route.nodes[index]});
            if (std::any_of(resources.begin(), resources.end(), [&](const auto resource) {
                  return voronoi_allocation.resources[resource].capacity_nm <
                         coupled_group_demands.at(group_id);
                })) {
              complete = false;
            }
          }
        }
        if (!complete) {
          break;
        }
        if (reference_sequences.empty()) {
          reference_sequences.push_back(std::move(sequences));
          reference_topologies.push_back(std::move(topologies));
        } else if (reference_sequences.front() != sequences ||
                   reference_topologies.front() != topologies) {
          complete = false;
          break;
        }
      }
      if (!complete) {
        continue;
      }
      ++accepted_candidates;
      ++stats.joint_group_candidates;
      for (const auto& member : members) {
        accepted_group_trees.emplace(member, tree_candidate);
      }
    }
    if (accepted_candidates == 0U) {
      ++stats.joint_groups_withheld;
    }
  }
  std::erase_if(plan.candidates, [&](const auto& candidate) {
    return coupled_nets.contains(candidate.net) &&
           !accepted_group_trees.contains({candidate.net, candidate.tree_candidate_index});
  });
  stats.candidates = plan.candidates.size();

  const auto quantize = [](SplineVector point) {
    return Point{Nanometres{static_cast<std::int64_t>(std::llround(point.x))},
                 Nanometres{static_cast<std::int64_t>(std::llround(point.y))}};
  };
  const auto simplify = [](const std::vector<Point>& points) {
    std::vector<Point> result;
    for (const auto point : points) {
      while (result.size() >= 2U) {
        const auto& first = result[result.size() - 2U];
        const auto& middle = result.back();
        const auto first_x = static_cast<long double>(middle.x.value() - first.x.value());
        const auto first_y = static_cast<long double>(middle.y.value() - first.y.value());
        const auto second_x = static_cast<long double>(point.x.value() - middle.x.value());
        const auto second_y = static_cast<long double>(point.y.value() - middle.y.value());
        if (first_x * second_y - first_y * second_x != 0.0L ||
            first_x * second_x + first_y * second_y <= 0.0L) {
          break;
        }
        result.pop_back();
      }
      if (result.empty() || result.back() != point) {
        result.push_back(point);
      }
    }
    return result;
  };

  struct NavigationDomain {
    std::vector<OffsetFreeSpaceRegion> regions;
    std::vector<std::optional<PreparedNavMeshDomain>> prepared_regions;
    Nanometres center_clearance;
    bool complete{false};
    bool regions_are_offset{true};
  };

  struct NetRefinementTaskResult {
    StaticSeedPlan plan;
    RoutingObstacleIndex obstacles;
    std::map<std::pair<NetId, std::size_t>, NavigationDomain> navigation_domain_products;
    std::vector<std::string> diagnostics;
    std::exception_ptr failure;

    explicit NetRefinementTaskResult(RoutingObstacleIndex worker_obstacles)
        : obstacles(std::move(worker_obstacles)) {}
  };
  std::vector<std::vector<GlobalSeedCandidate>> refinement_inputs(tasks.size());
  for (auto& candidate : plan.candidates) {
    const auto task = std::find_if(tasks.begin(), tasks.end(),
                                   [&](const auto& value) { return value.net == candidate.net; });
    if (task != tasks.end()) {
      refinement_inputs[static_cast<std::size_t>(task - tasks.begin())].push_back(
          std::move(candidate));
    }
  }
  plan.candidates.clear();
  std::vector<std::unique_ptr<NetRefinementTaskResult>> refinement_results(tasks.size());
  std::atomic_size_t next_refinement_task{0U};
  const auto run_refinement_task = [&]() {
    while (!stop_token.stop_requested()) {
      const auto task_index = next_refinement_task.fetch_add(1U, std::memory_order_relaxed);
      if (task_index >= tasks.size()) {
        return;
      }
      auto result = std::make_unique<NetRefinementTaskResult>(
          task_results[task_index] ? std::move(task_results[task_index]->obstacles)
                                   : obstacles.fork_worker());
      try {
        auto& plan = result->plan;
        auto& stats = plan.stats;
        auto& obstacles = result->obstacles;
        plan.candidates = std::move(refinement_inputs[task_index]);
        const auto corridor_cell_budget =
            task_budget(config.max_static_seed_corridor_cells, task_index, tasks.size());
        const auto reprojection_budget =
            task_budget(config.max_static_seed_reprojection_searches, task_index, tasks.size());
        auto& navigation_domains = result->navigation_domain_products;
        const auto navigation_domain = [&](const NetTask& task,
                                           std::size_t layer) -> NavigationDomain& {
          const auto key = std::pair{task.net, layer};
          if (const auto found = navigation_domains.find(key); found != navigation_domains.end()) {
            return found->second;
          }
          NavigationDomain domain;
          const auto rules = resolve_net_routing_rules(board, task.net);
          domain.center_clearance =
              Nanometres{rules.clearance.value() +
                         task.nominal_track_width.value() / static_cast<std::int64_t>(2)};
          ++stats.navmesh_domains;
          const auto extracted = extract_polygonal_board_free_space(
              board, BoardFreeSpaceExtractionConfig{grid.layers[layer], task.net,
                                                    config.respect_existing_copper, true,
                                                    !config.treat_saved_zone_fills_as_refillable});
          if (extracted.complete()) {
            auto offset = offset_polygonal_free_space(
                extracted.domain, FreeSpaceOffsetConfig{domain.center_clearance, 1'000'000U});
            if (offset.ok()) {
              domain.regions = std::move(offset.regions);
              domain.complete = !domain.regions.empty();
            } else {
              auto unioned = offset_polygonal_free_space(
                  extracted.domain, FreeSpaceOffsetConfig{Nanometres{0}, 1'000'000U});
              if (unioned.ok()) {
                domain.regions = std::move(unioned.regions);
                domain.complete = !domain.regions.empty();
                domain.regions_are_offset = false;
                ++stats.navmesh_certificate_domains;
              }
            }
          }
          return navigation_domains.emplace(key, std::move(domain)).first->second;
        };

        for (auto& candidate : plan.candidates) {
          if (stop_token.stop_requested()) {
            break;
          }
          const auto task = tasks.begin() + static_cast<std::ptrdiff_t>(task_index);
          if (candidate.route.nodes.size() < 2U) {
            continue;
          }
          std::set<std::size_t> corridor(candidate.route.nodes.begin(),
                                         candidate.route.nodes.end());
          std::vector<RoutingGeometricCorridorRun> geometric_corridors;
          bool sampled_spline = false;
          bool all_runs_refined = true;
          std::optional<std::tuple<std::size_t, Point, Point, std::vector<ObjectId>>> pns_collision;
          for (std::size_t first = 0U; first + 1U < candidate.route.nodes.size();) {
            const auto [unused_x, unused_y, layer] = grid.decode(candidate.route.nodes[first]);
            static_cast<void>(unused_x);
            static_cast<void>(unused_y);
            std::size_t last = first + 1U;
            while (last + 1U < candidate.route.nodes.size()) {
              const auto [next_x, next_y, next_layer] =
                  grid.decode(candidate.route.nodes[last + 1U]);
              static_cast<void>(next_x);
              static_cast<void>(next_y);
              if (next_layer != layer ||
                  (candidate.route.segment_widths.size() + 1U == candidate.route.nodes.size() &&
                   candidate.route.segment_widths[last] != candidate.route.segment_widths[first])) {
                break;
              }
              ++last;
            }
            std::vector<Point> points;
            for (auto index = first; index <= last; ++index) {
              const auto [x, y, unused_layer] = grid.decode(candidate.route.nodes[index]);
              static_cast<void>(unused_layer);
              points.push_back(grid.point(x, y));
            }
            points = simplify(points);
            std::optional<NavMeshRouteResult> navmesh_route;
            const OffsetFreeSpaceRegion* selected_region = nullptr;
            const PreparedNavMeshDomain* selected_prepared_region = nullptr;
            bool selected_region_is_offset = true;
            Nanometres center_clearance{0};
            if (config.use_navmesh_funnel_spline_seeds) {
              auto& domain = navigation_domain(*task, layer);
              center_clearance = domain.center_clearance;
              if (domain.complete) {
                const auto navmesh_pitch = config.global_navmesh_cell_pitch.value() > 0
                                               ? config.global_navmesh_cell_pitch
                                               : Nanometres{grid.pitch};
                domain.prepared_regions.resize(domain.regions.size());
                for (std::size_t region_index = 0U; region_index < domain.regions.size();
                     ++region_index) {
                  const auto& region = domain.regions[region_index];
                  const NavMeshConfig navmesh_config{
                      navmesh_pitch,
                      config.max_global_navmesh_cells_per_seed,
                      config.max_global_navmesh_search_expansions_per_seed,
                      {},
                      Nanometres{0},
                      static_cast<std::size_t>(
                          std::max<std::int64_t>(1, (config.global_spline_control_corridor.value() +
                                                     navmesh_pitch.value() - 1) /
                                                        navmesh_pitch.value())),
                      domain.regions_are_offset ? Nanometres{0} : domain.center_clearance,
                      std::nullopt};
                  if (!domain.prepared_regions[region_index]) {
                    domain.prepared_regions[region_index] =
                        prepare_navmesh_domain(region, navmesh_config);
                  }
                  auto routed =
                      route_prepared_navmesh(*domain.prepared_regions[region_index], points.front(),
                                             points.back(), navmesh_config);
                  stats.navmesh_cells =
                      saturating_count_add(stats.navmesh_cells, routed.cells.size());
                  stats.navmesh_search_expansions = saturating_count_add(
                      stats.navmesh_search_expansions, routed.search_expansions);
                  if (routed.ok()) {
                    if (routed.used_flood_fill) {
                      stats.flood_fill_routes = saturating_count_add(stats.flood_fill_routes, 1U);
                      stats.flood_fill_cells_visited = saturating_count_add(
                          stats.flood_fill_cells_visited, routed.flood_fill_cells_visited);
                    }
                    navmesh_route = std::move(routed);
                    selected_region = &region;
                    selected_prepared_region = &*domain.prepared_regions[region_index];
                    selected_region_is_offset = domain.regions_are_offset;
                    break;
                  }
                }
              }
              if (!navmesh_route) {
                ++stats.navmesh_failures;
                if (result->diagnostics.size() < config.max_static_seed_candidates_per_connection) {
                  result->diagnostics.push_back("net " + task->net.value() + " layer " +
                                                grid.layers[layer].value() +
                                                ": no legal prepared NavMesh route");
                }
                all_runs_refined = false;
                first = last;
                continue;
              }
              ++stats.navmesh_routes;
              stats.funnel_points =
                  saturating_count_add(stats.funnel_points, navmesh_route->centerline.size());
              points = navmesh_route->centerline;
            }
            if (points.size() == 2U) {
              points.insert(
                  points.begin() + 1,
                  Point{Nanometres{points.front().x.value() +
                                   (points.back().x.value() - points.front().x.value()) / 2},
                        Nanometres{points.front().y.value() +
                                   (points.back().y.value() - points.front().y.value()) / 2}});
            }
            if (auto spline = make_cubic_bspline_route(points)) {
              if (navmesh_route && selected_region != nullptr &&
                  selected_prepared_region != nullptr) {
                const auto point_bounds_distance = [](Point point,
                                                      const geometry::AxisAlignedBounds& bounds) {
                  const auto dx =
                      point.x < bounds.minimum.x
                          ? static_cast<long double>(bounds.minimum.x.value() - point.x.value())
                      : point.x > bounds.maximum.x
                          ? static_cast<long double>(point.x.value() - bounds.maximum.x.value())
                          : 0.0L;
                  const auto dy =
                      point.y < bounds.minimum.y
                          ? static_cast<long double>(bounds.minimum.y.value() - point.y.value())
                      : point.y > bounds.maximum.y
                          ? static_cast<long double>(point.y.value() - bounds.maximum.y.value())
                          : 0.0L;
                  return std::hypot(dx, dy);
                };
                const auto boundary_distance = [&](Point point) {
                  auto distance = geometry::point_polygon_boundary_distance_nm(
                      point, selected_prepared_region->prepared_outline);
                  for (const auto& hole : selected_prepared_region->prepared_cutouts) {
                    if (point_bounds_distance(point, hole.bounds) >= distance) {
                      continue;
                    }
                    distance = std::min(distance,
                                        geometry::point_polygon_boundary_distance_nm(point, hole));
                  }
                  return distance;
                };
                const auto clearance_evaluator = [&](SplineVector position) {
                  const auto point = quantize(position);
                  const auto in_region =
                      geometry::locate_point(selected_prepared_region->prepared_outline, point) !=
                          geometry::PointLocation::outside &&
                      std::none_of(selected_prepared_region->prepared_cutouts.begin(),
                                   selected_prepared_region->prepared_cutouts.end(),
                                   [&](const auto& hole) {
                                     return hole.bounds.contains(point) &&
                                            geometry::locate_point(hole, point) !=
                                                geometry::PointLocation::outside;
                                   });
                  const auto in_corridor = navmesh_corridor_contains(*navmesh_route, point);
                  const auto probe = std::max<std::int64_t>(1, grid.pitch / 100);
                  const auto distance = boundary_distance(point);
                  const auto dx =
                      boundary_distance(Point{Nanometres{point.x.value() + probe}, point.y}) -
                      boundary_distance(Point{Nanometres{point.x.value() - probe}, point.y});
                  const auto dy =
                      boundary_distance(Point{point.x, Nanometres{point.y.value() + probe}}) -
                      boundary_distance(Point{point.x, Nanometres{point.y.value() - probe}});
                  const auto length = std::hypot(dx, dy);
                  return SplineClearanceProbe{
                      (selected_region_is_offset
                           ? static_cast<long double>(center_clearance.value())
                           : 0.0L) +
                          (in_region ? distance : -distance),
                      length > 0.0L ? SplineVector{dx / length, dy / length} : SplineVector{},
                      in_region && in_corridor};
                };
                SplineTrajectoryOptimizationConfig trajectory_config;
                trajectory_config.minimum_clearance = center_clearance;
                trajectory_config.minimum_bend_radius = config.global_spline_minimum_bend_radius;
                trajectory_config.control_step = config.global_spline_shove_step;
                trajectory_config.gradient_probe_step = Nanometres{std::max<std::int64_t>(
                    1, config.global_spline_shove_step.value() / static_cast<std::int64_t>(5))};
                trajectory_config.maximum_corridor_displacement =
                    config.global_spline_control_corridor;
                trajectory_config.samples = config.global_spline_trajectory_samples;
                trajectory_config.maximum_iterations =
                    config.max_global_spline_trajectory_iterations;
                trajectory_config.maximum_candidate_checks =
                    config.max_global_spline_trajectory_candidate_checks;
                trajectory_config.preferred_start_tangent = SplineVector{
                    static_cast<long double>(points[1U].x.value() - points[0U].x.value()),
                    static_cast<long double>(points[1U].y.value() - points[0U].y.value())};
                trajectory_config.preferred_end_tangent =
                    SplineVector{static_cast<long double>(points.back().x.value() -
                                                          points[points.size() - 2U].x.value()),
                                 static_cast<long double>(points.back().y.value() -
                                                          points[points.size() - 2U].y.value())};
                ++stats.trajectory_attempts;
                const auto optimized = optimize_cubic_bspline_trajectory(*spline, trajectory_config,
                                                                         clearance_evaluator);
                stats.trajectory_control_moves = saturating_count_add(
                    stats.trajectory_control_moves, optimized.accepted_control_moves);
                stats.trajectory_candidate_checks = saturating_count_add(
                    stats.trajectory_candidate_checks, optimized.candidate_checks);
                if (!optimized.accepted) {
                  ++stats.trajectory_rejected;
                  all_runs_refined = false;
                  first = last;
                  continue;
                }
                ++stats.trajectory_accepted;
              }
              auto samples = adaptive_tessellate_cubic_bspline(
                  *spline, config.global_spline_tessellation_tolerance,
                  config.global_spline_max_tessellation_depth,
                  config.max_global_spline_samples_per_seed);
              if (navmesh_route && selected_region != nullptr) {
                FreeSpaceDomain represented_region;
                represented_region.outline = selected_region->area;
                represented_region.cutouts = selected_region->holes;
                std::vector<std::pair<Point, Point>> sample_chords;
                sample_chords.reserve(samples.size() > 1U ? samples.size() - 1U : 0U);
                for (std::size_t index = 1U; index < samples.size(); ++index) {
                  sample_chords.emplace_back(quantize(samples[index - 1U].position),
                                             quantize(samples[index].position));
                }
                const auto certificates =
                    assess_free_space_chords(represented_region, sample_chords);
                bool topology_legal = samples.size() >= 2U;
                for (std::size_t index = 0U; topology_legal && index < samples.size(); ++index) {
                  const auto point = quantize(samples[index].position);
                  topology_legal = navmesh_corridor_contains(*navmesh_route, point);
                  if (topology_legal && index > 0U) {
                    topology_legal = certificates[index - 1U].has_value();
                  }
                }
                if (!topology_legal) {
                  ++stats.trajectory_rejected;
                  all_runs_refined = false;
                  first = last;
                  continue;
                }

                long double minimum_residual_clearance =
                    std::numeric_limits<long double>::infinity();
                for (std::size_t index = 1U; index < samples.size(); ++index) {
                  const auto& certificate = certificates[index - 1U];
                  if (!certificate) {
                    minimum_residual_clearance = -1.0L;
                    break;
                  }
                  minimum_residual_clearance = std::min(
                      minimum_residual_clearance, static_cast<long double>(certificate->value()));
                }
                if (!(minimum_residual_clearance >= 0.0L) ||
                    !std::isfinite(minimum_residual_clearance)) {
                  ++stats.trajectory_rejected;
                  all_runs_refined = false;
                  first = last;
                  continue;
                }
                const auto required_envelope = Nanometres{std::max<std::int64_t>(
                    1, center_clearance.value() > std::numeric_limits<std::int64_t>::max() / 2
                           ? std::numeric_limits<std::int64_t>::max()
                           : center_clearance.value() * 2)};
                const auto physical_width =
                    (selected_region_is_offset ? static_cast<long double>(required_envelope.value())
                                               : 0.0L) +
                    2.0L * minimum_residual_clearance;
                const auto bounded_width =
                    Nanometres{static_cast<std::int64_t>(std::llround(std::clamp(
                        physical_width, 1.0L,
                        static_cast<long double>(std::numeric_limits<std::int64_t>::max()))))};
                RoutingGeometricCorridorRun run;
                run.layer = grid.layers[layer];
                run.portals = navmesh_route->portals;
                run.lane_centerline.reserve(samples.size());
                for (const auto& sample : samples) {
                  const auto point = quantize(sample.position);
                  if (run.lane_centerline.empty() || run.lane_centerline.back() != point) {
                    run.lane_centerline.push_back(point);
                  }
                }
                run.topology_cells.reserve(navmesh_route->topology_guard_cells.size());
                for (const auto cell : navmesh_route->topology_guard_cells) {
                  if (cell < navmesh_route->cells.size()) {
                    run.topology_cells.push_back(navmesh_route->cells[cell].polygon());
                  }
                }
                run.required_envelope_width = required_envelope;
                run.minimum_physical_cross_section_width = bounded_width;
                run.lane_capacity = static_cast<std::size_t>(
                    std::max<std::int64_t>(1, bounded_width.value() / required_envelope.value()));
                run.selected_path_preserves_topology = true;
                geometric_corridors.push_back(std::move(run));
              }
              stats.spline_samples = saturating_count_add(stats.spline_samples, samples.size());
              sampled_spline = sampled_spline || samples.size() >= 2U;
              if (!pns_collision) {
                for (std::size_t index = 1U; index < samples.size(); ++index) {
                  const auto start = quantize(samples[index - 1U].position);
                  const auto end = quantize(samples[index].position);
                  if (start == end) {
                    continue;
                  }
                  auto blockers = obstacles.pin_access_blocking_objects(
                      candidate.net, layer, start, end, task->nominal_track_width);
                  if (!blockers.empty()) {
                    ++stats.collision_envelopes;
                    const bool movable =
                        std::all_of(blockers.begin(), blockers.end(), [&](const auto& id) {
                          const auto selected = [&](const NetId& net) {
                            return !coupled_nets.contains(net) &&
                                   (config.selected_nets.empty() ||
                                    std::find(config.selected_nets.begin(),
                                              config.selected_nets.end(),
                                              net) != config.selected_nets.end());
                          };
                          const auto track = std::find_if(
                              board.tracks.begin(), board.tracks.end(), [&](const auto& item) {
                                return item.id == id && !item.locked &&
                                       item.layer == grid.layers[layer] && selected(item.net);
                              });
                          if (track != board.tracks.end()) {
                            return true;
                          }
                          return std::any_of(
                              board.vias.begin(), board.vias.end(), [&](const auto& via) {
                                return via.id == id && !via.locked && selected(via.net) &&
                                       std::find(via.layers.begin(), via.layers.end(),
                                                 grid.layers[layer]) != via.layers.end();
                              });
                        });
                    if (movable) {
                      ++stats.pns_eligible_collisions;
                    } else {
                      ++stats.pns_ineligible_collisions;
                    }
                    if (movable && plan.shove_proposals.empty() &&
                        candidate.tree_candidate_index == 0U) {
                      pns_collision.emplace(layer, start, end, std::move(blockers));
                    } else {
                      const auto parameter =
                          (samples[index - 1U].parameter + samples[index].parameter) / 2.0L;
                      const auto tangent_x =
                          samples[index].position.x - samples[index - 1U].position.x;
                      const auto tangent_y =
                          samples[index].position.y - samples[index - 1U].position.y;
                      const auto blocked_chords = [&](const auto& candidate_samples) {
                        std::size_t count = 0U;
                        for (std::size_t chord = 1U; chord < candidate_samples.size(); ++chord) {
                          const auto first_point = quantize(candidate_samples[chord - 1U].position);
                          const auto last_point = quantize(candidate_samples[chord].position);
                          if (!obstacles
                                   .pin_access_blocking_objects(candidate.net, layer, first_point,
                                                                last_point,
                                                                task->nominal_track_width)
                                   .empty()) {
                            count = saturating_count_add(count, 1U);
                          }
                        }
                        return count;
                      };
                      auto best_samples = samples;
                      auto best_blocked = blocked_chords(samples);
                      for (const auto direction : {1.0L, -1.0L}) {
                        auto shoved = *spline;
                        const auto moved = shove_cubic_bspline_control_corridor(
                            shoved, parameter,
                            SplineVector{direction * -tangent_y, direction * tangent_x},
                            config.global_spline_shove_step, config.global_spline_control_corridor);
                        if (moved == 0U) {
                          continue;
                        }
                        auto refitted = adaptive_tessellate_cubic_bspline(
                            shoved, config.global_spline_tessellation_tolerance,
                            config.global_spline_max_tessellation_depth,
                            config.max_global_spline_samples_per_seed);
                        stats.spline_samples =
                            saturating_count_add(stats.spline_samples, refitted.size());
                        const auto blocked = blocked_chords(refitted);
                        if (blocked < best_blocked) {
                          best_blocked = blocked;
                          best_samples = std::move(refitted);
                          stats.control_shoves = saturating_count_add(stats.control_shoves, moved);
                          ++stats.refits;
                        }
                      }
                      samples = std::move(best_samples);
                    }
                    break;
                  }
                }
              }
              const auto radius = static_cast<std::int64_t>(std::max<std::int64_t>(
                  1,
                  (config.global_spline_control_corridor.value() + grid.pitch - 1) / grid.pitch));
              for (const auto& sample : samples) {
                const auto snapped = snap_to_grid(grid, quantize(sample.position));
                if (!snapped) {
                  continue;
                }
                const auto center_x = static_cast<std::int64_t>(snapped->first);
                const auto center_y = static_cast<std::int64_t>(snapped->second);
                for (auto y = center_y - radius; y <= center_y + radius; ++y) {
                  for (auto x = center_x - radius; x <= center_x + radius; ++x) {
                    if (x < 0 || y < 0 || x >= static_cast<std::int64_t>(grid.width) ||
                        y >= static_cast<std::int64_t>(grid.height) ||
                        !grid.valid(static_cast<std::size_t>(x), static_cast<std::size_t>(y))) {
                      continue;
                    }
                    const auto point =
                        grid.point(static_cast<std::size_t>(x), static_cast<std::size_t>(y));
                    if (!obstacles.pin_access_blocked(candidate.net, layer, point, point)) {
                      corridor.insert(grid.node(static_cast<std::size_t>(x),
                                                static_cast<std::size_t>(y), layer));
                    }
                  }
                }
              }
            }
            first = last;
          }
          if (!sampled_spline || !all_runs_refined) {
            continue;
          }
          std::optional<Board> proposed_board;
          std::optional<GlobalRoutingShoveProposal> proposed_shove;
          if (pns_collision) {
            const auto& [layer, start, end, blockers] = *pns_collision;
            std::vector<std::string> eligible;
            std::vector<NetId> affected_nets;
            for (const auto& blocker : blockers) {
              const auto track =
                  std::find_if(board.tracks.begin(), board.tracks.end(), [&](const auto& t) {
                    return t.id == blocker && !t.locked && t.layer == grid.layers[layer];
                  });
              if (track != board.tracks.end()) {
                eligible.push_back(track->id.value());
                affected_nets.push_back(track->net);
                continue;
              }
              const auto via =
                  std::find_if(board.vias.begin(), board.vias.end(), [&](const auto& item) {
                    return item.id == blocker && !item.locked &&
                           std::find(item.layers.begin(), item.layers.end(), grid.layers[layer]) !=
                               item.layers.end();
                  });
              if (via == board.vias.end()) {
                eligible.clear();
                break;
              }
              eligible.push_back(via->id.value());
              affected_nets.push_back(via->net);
            }
            std::sort(affected_nets.begin(), affected_nets.end());
            affected_nets.erase(std::unique(affected_nets.begin(), affected_nets.end()),
                                affected_nets.end());
            if (!eligible.empty()) {
              ++stats.pns_proposal_attempts;
              Board probe = board;
              const auto intruder_id = ObjectId{"flux-global-pns-intruder"};
              const bool id_available =
                  std::none_of(probe.tracks.begin(), probe.tracks.end(),
                               [&](const auto& track) { return track.id == intruder_id; });
              if (id_available) {
                probe.tracks.push_back(TrackSegment{intruder_id, candidate.net, grid.layers[layer],
                                                    start, end, task->nominal_track_width, false});
                const auto dx = static_cast<long double>(end.x.value() - start.x.value());
                const auto dy = static_cast<long double>(end.y.value() - start.y.value());
                const PnsShoveRequest request{
                    {intruder_id.value()},
                    eligible,
                    {},
                    Point{Nanometres{start.x.value() + (end.x.value() - start.x.value()) / 2},
                          Nanometres{start.y.value() + (end.y.value() - start.y.value()) / 2}},
                    -dy,
                    dx,
                    config.global_spline_shove_step,
                    {}};
                const PnsShoveConfig pns_config{
                    config.max_global_pns_propagation_depth, config.max_global_pns_candidate_checks,
                    config.global_spline_shove_step, config.maximum_global_pns_displacement};
                auto shoved = pns_push_and_shove(probe, request, pns_config);
                stats.pns_candidate_checks =
                    saturating_count_add(stats.pns_candidate_checks, shoved.candidate_checks);
                stats.pns_local_splits =
                    saturating_count_add(stats.pns_local_splits, shoved.local_splits);
                if (shoved.accepted()) {
                  std::erase_if(shoved.candidate->tracks,
                                [&](const auto& track) { return track.id == intruder_id; });
                  GlobalRoutingShoveProposal proposal;
                  proposal.id = 0U;
                  proposal.intruding_net = candidate.net;
                  proposal.affected_nets = std::move(affected_nets);
                  proposal.displacement_cost_nm =
                      static_cast<std::uint64_t>(config.global_spline_shove_step.value());
                  proposal.propagation_depth = shoved.propagation_depth;
                  std::set<ObjectId> source_track_ids;
                  for (const auto& track : board.tracks) {
                    source_track_ids.insert(track.id);
                  }
                  std::set<ObjectId> source_via_ids;
                  for (const auto& via : board.vias) {
                    source_via_ids.insert(via.id);
                  }
                  const std::set<std::string> moved_ids(shoved.moved_object_ids.begin(),
                                                        shoved.moved_object_ids.end());
                  for (const auto& track : board.tracks) {
                    if (moved_ids.contains(track.id.value())) {
                      proposal.replaced_tracks.push_back(track.id);
                    }
                  }
                  for (const auto& via : board.vias) {
                    if (moved_ids.contains(via.id.value())) {
                      proposal.replaced_vias.push_back(via.id);
                    }
                  }
                  for (const auto& track : shoved.candidate->tracks) {
                    if (!source_track_ids.contains(track.id) ||
                        moved_ids.contains(track.id.value())) {
                      proposal.replacement_tracks.push_back(track);
                    }
                  }
                  for (const auto& via : shoved.candidate->vias) {
                    if (source_via_ids.contains(via.id) && moved_ids.contains(via.id.value())) {
                      proposal.replacement_vias.push_back(via);
                    }
                  }
                  proposed_board = std::move(*shoved.candidate);
                  proposed_shove = std::move(proposal);
                }
              }
            }
          }
          if (corridor.size() >
              corridor_cell_budget - std::min(corridor_cell_budget, stats.corridor_nodes)) {
            stats.budget_exhausted = true;
            ++stats.reprojections_reverted;
            continue;
          }
          stats.corridor_nodes = saturating_count_add(stats.corridor_nodes, corridor.size());
          if (stats.reprojection_searches >= reprojection_budget) {
            stats.budget_exhausted = true;
            ++stats.reprojections_reverted;
            continue;
          }
          SearchCostPolicy policy;
          policy.kind = SearchCostPolicyKind::corridor_reprojection;
          policy.corridor_nodes = std::move(corridor);
          policy.corridor_departure_penalty = config.static_seed_corridor_departure_penalty;
          ++stats.reprojection_searches;
          std::optional<RoutingObstacleIndex> proposed_obstacles;
          auto* selected_obstacles = &obstacles;
          if (proposed_board) {
            proposed_obstacles.emplace(*proposed_board, grid, config.respect_existing_copper,
                                       !config.treat_saved_zone_fills_as_refillable);
            if (proposed_obstacles->valid()) {
              selected_obstacles = &*proposed_obstacles;
            } else {
              proposed_board.reset();
              proposed_shove.reset();
            }
          }
          auto refined = search_path(
              grid, candidate.starts, candidate.goals, empty_occupancy, initial_history, corridors,
              corridor_neck_downs, empty_guidance, voronoi_allocation, *selected_obstacles, *task,
              config, config.use_direct_voronoi_resource_edges, false, 1'000U, policy, stop_token);
          stats.search_expansions =
              saturating_count_add(stats.search_expansions, refined.expansions);
          if (refined.cancelled) {
            break;
          }
          if (refined.nodes.empty()) {
            ++stats.reprojections_reverted;
            continue;
          }
          stats.corridor_departures = saturating_count_add(
              stats.corridor_departures,
              static_cast<std::size_t>(
                  std::count_if(refined.nodes.begin(), refined.nodes.end(), [&](const auto node) {
                    return !policy.corridor_nodes.contains(node);
                  })));
          candidate.route = std::move(refined);
          candidate.spline_corridor_nodes = policy.corridor_nodes;
          const auto preserves_topology =
              std::all_of(candidate.route.nodes.begin(), candidate.route.nodes.end(),
                          [&](const auto node) { return policy.corridor_nodes.contains(node); });
          for (auto& geometric : geometric_corridors) {
            geometric.selected_path_preserves_topology = preserves_topology;
          }
          candidate.geometric_corridors = std::move(geometric_corridors);
          candidate.spline_refined = true;
          if (proposed_shove && proposed_board) {
            candidate.shove_proposal_ids.push_back(proposed_shove->id);
            plan.shove_proposals.push_back(std::move(*proposed_shove));
            plan.planning_board = std::move(*proposed_board);
          }
          ++stats.refined_candidates;
          ++stats.reprojections_accepted;
        }
      } catch (...) {
        result->failure = std::current_exception();
      }
      refinement_results[task_index] = std::move(result);
    }
  };
  workers.clear();
  next_refinement_task.store(0U, std::memory_order_relaxed);
  for (std::size_t index = 0U; index < worker_count; ++index) {
    workers.emplace_back(run_refinement_task);
  }
  workers.clear();

  for (auto& result : refinement_results) {
    if (!result) {
      continue;
    }
    if (result->failure) {
      std::rethrow_exception(result->failure);
    }
    const auto& local = result->plan.stats;
    stats.search_expansions =
        saturating_count_add(stats.search_expansions, local.search_expansions);
    stats.corridor_nodes = saturating_count_add(stats.corridor_nodes, local.corridor_nodes);
    stats.spline_samples = saturating_count_add(stats.spline_samples, local.spline_samples);
    stats.navmesh_domains = saturating_count_add(stats.navmesh_domains, local.navmesh_domains);
    stats.navmesh_routes = saturating_count_add(stats.navmesh_routes, local.navmesh_routes);
    stats.navmesh_cells = saturating_count_add(stats.navmesh_cells, local.navmesh_cells);
    stats.navmesh_search_expansions =
        saturating_count_add(stats.navmesh_search_expansions, local.navmesh_search_expansions);
    stats.flood_fill_routes =
        saturating_count_add(stats.flood_fill_routes, local.flood_fill_routes);
    stats.flood_fill_cells_visited =
        saturating_count_add(stats.flood_fill_cells_visited, local.flood_fill_cells_visited);
    stats.navmesh_failures = saturating_count_add(stats.navmesh_failures, local.navmesh_failures);
    stats.navmesh_certificate_domains =
        saturating_count_add(stats.navmesh_certificate_domains, local.navmesh_certificate_domains);
    stats.funnel_points = saturating_count_add(stats.funnel_points, local.funnel_points);
    stats.trajectory_attempts =
        saturating_count_add(stats.trajectory_attempts, local.trajectory_attempts);
    stats.trajectory_accepted =
        saturating_count_add(stats.trajectory_accepted, local.trajectory_accepted);
    stats.trajectory_control_moves =
        saturating_count_add(stats.trajectory_control_moves, local.trajectory_control_moves);
    stats.trajectory_candidate_checks =
        saturating_count_add(stats.trajectory_candidate_checks, local.trajectory_candidate_checks);
    stats.trajectory_rejected =
        saturating_count_add(stats.trajectory_rejected, local.trajectory_rejected);
    stats.collision_envelopes =
        saturating_count_add(stats.collision_envelopes, local.collision_envelopes);
    stats.control_shoves = saturating_count_add(stats.control_shoves, local.control_shoves);
    stats.refits = saturating_count_add(stats.refits, local.refits);
    stats.refined_candidates =
        saturating_count_add(stats.refined_candidates, local.refined_candidates);
    stats.reprojection_searches =
        saturating_count_add(stats.reprojection_searches, local.reprojection_searches);
    stats.reprojections_accepted =
        saturating_count_add(stats.reprojections_accepted, local.reprojections_accepted);
    stats.reprojections_reverted =
        saturating_count_add(stats.reprojections_reverted, local.reprojections_reverted);
    stats.corridor_departures =
        saturating_count_add(stats.corridor_departures, local.corridor_departures);
    stats.pns_proposal_attempts =
        saturating_count_add(stats.pns_proposal_attempts, local.pns_proposal_attempts);
    stats.pns_eligible_collisions =
        saturating_count_add(stats.pns_eligible_collisions, local.pns_eligible_collisions);
    stats.pns_ineligible_collisions =
        saturating_count_add(stats.pns_ineligible_collisions, local.pns_ineligible_collisions);
    stats.pns_candidate_checks =
        saturating_count_add(stats.pns_candidate_checks, local.pns_candidate_checks);
    stats.pns_local_splits = saturating_count_add(stats.pns_local_splits, local.pns_local_splits);
    stats.budget_exhausted = stats.budget_exhausted || local.budget_exhausted;

    const auto proposal_offset = plan.shove_proposals.size();
    for (auto& candidate : result->plan.candidates) {
      for (auto& proposal_id : candidate.shove_proposal_ids) {
        proposal_id = saturating_count_add(proposal_offset, proposal_id);
      }
    }
    for (std::size_t index = 0U; index < result->plan.shove_proposals.size(); ++index) {
      result->plan.shove_proposals[index].id = proposal_offset + index;
    }
    plan.candidates.insert(plan.candidates.end(),
                           std::make_move_iterator(result->plan.candidates.begin()),
                           std::make_move_iterator(result->plan.candidates.end()));
    plan.shove_proposals.insert(plan.shove_proposals.end(),
                                std::make_move_iterator(result->plan.shove_proposals.begin()),
                                std::make_move_iterator(result->plan.shove_proposals.end()));
    if (!plan.planning_board && result->plan.planning_board) {
      plan.planning_board = std::move(result->plan.planning_board);
    }
    obstacles.absorb_worker(std::move(result->obstacles));
  }
  std::set<std::vector<std::size_t>> layer_sequences;
  std::set<std::vector<std::size_t>> topology_signatures;
  std::set<std::pair<NetId, std::size_t>> covered_connections;
  for (const auto& candidate : plan.candidates) {
    covered_connections.emplace(candidate.net, candidate.connection_index);
    const auto sequence = layer_sequence(candidate);
    layer_sequences.insert(sequence);
    topology_signatures.insert(topology_signature(candidate));
    if (sequence.size() > 1U) {
      ++stats.via_bearing_candidates;
    }
    bool direct = false;
    for (std::size_t index = 1U; index < candidate.route.nodes.size(); ++index) {
      if (voronoi_allocation.direct_resource(
              Edge{candidate.route.nodes[index - 1U], candidate.route.nodes[index]})) {
        direct = true;
        break;
      }
    }
    stats.direct_medial_candidates =
        saturating_count_add(stats.direct_medial_candidates, direct ? 1U : 0U);
  }
  stats.distinct_layer_sequences = layer_sequences.size();
  stats.distinct_topology_signatures = topology_signatures.size();
  for (const auto& task : tasks) {
    for (std::size_t connection = 0U; connection + 1U < task.terminals.size(); ++connection) {
      if (!covered_connections.contains({task.net, connection})) {
        ++stats.terminals_without_seed;
      }
    }
  }
  return plan;
}
[[nodiscard]] std::vector<std::size_t>
neighbors(const Grid& grid, const SharedVoronoiAllocationGraph& voronoi_allocation,
          std::size_t node, bool use_direct_edges) {
  const auto [x, y, layer] = grid.decode(node);
  auto result = sampled_planar_neighbors(grid, node);
  if (grid.via_site(x, y)) {
    if (layer > 0U) {
      result.push_back(grid.node(x, y, layer - 1U));
    }
    if (layer + 1U < grid.layers.size()) {
      result.push_back(grid.node(x, y, layer + 1U));
    }
  }
  if (use_direct_edges) {
    const auto& direct = voronoi_allocation.direct_neighbors(node);
    result.insert(result.end(), direct.begin(), direct.end());
  }
  std::sort(result.begin(), result.end());
  result.erase(std::unique(result.begin(), result.end()), result.end());
  return result;
}

[[nodiscard]] TerminalEscapeReservation
build_terminal_escape_reservation(const Grid& grid, const NetTask& current_task,
                                  const std::deque<NetTask>& future_tasks,
                                  std::size_t maximum_resources) {
  TerminalEscapeReservation result;
  if (future_tasks.empty() || maximum_resources == 0U || grid.pitch <= 0) {
    return result;
  }

  // Use the largest route/via envelope that any imminent task may introduce.  This makes the
  // reservation conservative while still leaving all graph transitions legal and available.
  long double maximum_competing_radius_nm =
      std::max(current_task.nominal_track_width.value(), current_task.via_diameter.value()) / 2.0L +
      static_cast<long double>(current_task.physical_clearance.value());
  for (const auto& task : future_tasks) {
    maximum_competing_radius_nm =
        std::max(maximum_competing_radius_nm,
                 std::max(task.nominal_track_width.value(), task.via_diameter.value()) / 2.0L +
                     static_cast<long double>(task.physical_clearance.value()));
  }
  const auto clamped_index = [](long double coordinate, std::int64_t origin, std::int64_t pitch,
                                std::size_t extent, bool upper) {
    const auto unbounded =
        upper
            ? static_cast<std::int64_t>(std::ceil((coordinate - static_cast<long double>(origin)) /
                                                  static_cast<long double>(pitch)))
            : static_cast<std::int64_t>(std::floor((coordinate - static_cast<long double>(origin)) /
                                                   static_cast<long double>(pitch)));
    if (unbounded < 0) {
      return std::size_t{0};
    }
    return std::min(static_cast<std::size_t>(unbounded), extent - 1U);
  };

  for (const auto& task : future_tasks) {
    for (const auto& terminal : task.terminals) {
      const TerminalEscapeReservationKey key{task.net, terminal.footprint, terminal.pad};
      for (const auto& [portal, access] : terminal.access_paths) {
        if (access.points.empty()) {
          continue;
        }
        const auto [unused_x, unused_y, layer] = grid.decode(portal);
        static_cast<void>(unused_x);
        static_cast<void>(unused_y);
        TerminalEscapeReservation::Candidate candidate;
        candidate.terminal = key;
        candidate.layer = layer;
        candidate.width = access.width;
        candidate.segments.reserve(std::max<std::size_t>(access.points.size() - 1U, 1U));
        if (access.points.size() == 1U) {
          candidate.segments.emplace_back(access.points.front(), access.points.front());
        } else {
          for (std::size_t index = 1U; index < access.points.size(); ++index) {
            candidate.segments.emplace_back(access.points[index - 1U], access.points[index]);
          }
        }

        const auto collision_distance_nm =
            maximum_competing_radius_nm + static_cast<long double>(access.width.value()) / 2.0L;
        candidate.collision_distance_nm = collision_distance_nm;
        for (const auto& [start, end] : candidate.segments) {
          const auto minimum_x = std::min(start.x.value(), end.x.value());
          const auto maximum_x = std::max(start.x.value(), end.x.value());
          const auto minimum_y = std::min(start.y.value(), end.y.value());
          const auto maximum_y = std::max(start.y.value(), end.y.value());
          const auto lower_x =
              clamped_index(static_cast<long double>(minimum_x) - collision_distance_nm,
                            grid.origin_x, grid.pitch, grid.width, false);
          const auto upper_x =
              clamped_index(static_cast<long double>(maximum_x) + collision_distance_nm,
                            grid.origin_x, grid.pitch, grid.width, true);
          const auto lower_y =
              clamped_index(static_cast<long double>(minimum_y) - collision_distance_nm,
                            grid.origin_y, grid.pitch, grid.height, false);
          const auto upper_y =
              clamped_index(static_cast<long double>(maximum_y) + collision_distance_nm,
                            grid.origin_y, grid.pitch, grid.height, true);
          for (std::size_t y = lower_y; y <= upper_y; ++y) {
            for (std::size_t x = lower_x; x <= upper_x; ++x) {
              if (!grid.valid(x, y)) {
                continue;
              }
              const auto node = grid.node(x, y, layer);
              const auto point = grid.point(x, y);
              if (geometry::point_segment_distance_nm(point, start, end) <= collision_distance_nm) {
                candidate.blocking_nodes.insert(node);
              }
              for (const auto next : sampled_planar_neighbors(grid, node)) {
                if (next <= node) {
                  continue;
                }
                const auto [next_x, next_y, next_layer] = grid.decode(next);
                if (next_layer != layer) {
                  continue;
                }
                if (geometry::segment_distance_nm(point, grid.point(next_x, next_y), start, end) <=
                    collision_distance_nm) {
                  candidate.blocking_edges.insert(sampled_capacity_edge(grid, Edge{node, next}));
                }
              }
            }
          }
        }
        const auto candidate_resources =
            saturating_count_add(candidate.blocking_edges.size(), candidate.blocking_nodes.size());
        if (candidate_resources > maximum_resources - result.resources) {
          result.withheld_candidates = saturating_count_add(result.withheld_candidates, 1U);
          continue;
        }
        const auto candidate_id = result.candidates.size();
        for (const auto& edge : candidate.blocking_edges) {
          result.candidates_by_edge[edge].push_back(candidate_id);
        }
        for (const auto node : candidate.blocking_nodes) {
          result.candidates_by_node[node].push_back(candidate_id);
        }
        result.resources = saturating_count_add(result.resources, candidate_resources);
        result.candidates_by_terminal[key].push_back(candidate_id);
        result.candidates.push_back(std::move(candidate));
      }
    }
  }
  return result;
}

[[nodiscard]] bool
terminal_escape_candidate_blocked(const TerminalEscapeReservation::Candidate& candidate,
                                  const RoutingOccupancy& occupancy) {
  const auto occupied_by_another_net = [&candidate](const auto& occupants) {
    return std::any_of(occupants.begin(), occupants.end(), [&candidate](const NetId& occupant) {
      return occupant != candidate.terminal.net;
    });
  };
  for (const auto& edge : candidate.blocking_edges) {
    if (const auto found = occupancy.edges.find(edge);
        found != occupancy.edges.end() && occupied_by_another_net(found->second)) {
      return true;
    }
  }
  for (const auto node : candidate.blocking_nodes) {
    if (const auto found = occupancy.nodes.find(node);
        found != occupancy.nodes.end() && occupied_by_another_net(found->second)) {
      return true;
    }
  }
  return false;
}

[[nodiscard]] TerminalEscapeReservationAvailability
terminal_escape_reservation_availability(const TerminalEscapeReservation& reservation,
                                         const RoutingOccupancy& occupancy) {
  TerminalEscapeReservationAvailability result;
  for (const auto& [terminal, candidate_ids] : reservation.candidates_by_terminal) {
    auto& viable = result.viable_by_terminal[terminal];
    for (const auto candidate_id : candidate_ids) {
      if (!terminal_escape_candidate_blocked(reservation.candidates[candidate_id], occupancy)) {
        viable.push_back(candidate_id);
      }
    }
  }
  return result;
}

[[nodiscard]] std::uint64_t terminal_escape_reservation_penalty(
    const Grid& grid, const TerminalEscapeReservation& reservation,
    const TerminalEscapeReservationAvailability& availability, const NetId& net, const Edge& edge,
    bool is_direct, std::uint64_t reservation_cost, std::size_t* protected_terminals) {
  if (reservation_cost == 0U) {
    return 0U;
  }
  std::set<std::size_t> impacted;
  const auto add = [&impacted](const auto& index, const auto& resource) {
    if (const auto found = index.find(resource); found != index.end()) {
      impacted.insert(found->second.begin(), found->second.end());
    }
  };
  add(reservation.candidates_by_edge, sampled_capacity_edge(grid, edge));
  add(reservation.candidates_by_node, edge.first);
  add(reservation.candidates_by_node, edge.second);
  if (is_direct) {
    const auto [first_x, first_y, unused_first_layer] = grid.decode(edge.first);
    const auto [second_x, second_y, unused_second_layer] = grid.decode(edge.second);
    static_cast<void>(unused_second_layer);
    const auto start = grid.point(first_x, first_y);
    const auto end = grid.point(second_x, second_y);
    for (std::size_t candidate_id = 0U; candidate_id < reservation.candidates.size();
         ++candidate_id) {
      const auto& candidate = reservation.candidates[candidate_id];
      if (candidate.layer != unused_first_layer) {
        continue;
      }
      const auto threshold = candidate.collision_distance_nm;
      if (std::any_of(candidate.segments.begin(), candidate.segments.end(),
                      [&start, &end, threshold](const auto& segment) {
                        return geometry::segment_distance_nm(start, end, segment.first,
                                                             segment.second) <= threshold;
                      })) {
        impacted.insert(candidate_id);
      }
    }
  }
  std::map<TerminalEscapeReservationKey, std::size_t> impacted_viable_by_terminal;
  for (const auto candidate_id : impacted) {
    const auto& candidate = reservation.candidates[candidate_id];
    if (candidate.terminal.net == net) {
      continue;
    }
    const auto available = availability.viable_by_terminal.find(candidate.terminal);
    if (available == availability.viable_by_terminal.end() ||
        !std::binary_search(available->second.begin(), available->second.end(), candidate_id)) {
      continue;
    }
    impacted_viable_by_terminal[candidate.terminal] =
        saturating_count_add(impacted_viable_by_terminal[candidate.terminal], 1U);
  }
  std::size_t count = 0U;
  for (const auto& [terminal, impacted_count] : impacted_viable_by_terminal) {
    const auto available = availability.viable_by_terminal.find(terminal);
    if (available != availability.viable_by_terminal.end() &&
        impacted_count == available->second.size()) {
      count = saturating_count_add(count, 1U);
    }
  }
  if (protected_terminals != nullptr) {
    *protected_terminals = count;
  }
  return saturating_multiply(static_cast<std::uint64_t>(count), reservation_cost);
}

struct PreparedRoutabilityHistory {
  RoutingHistory history;
  std::size_t entries{0U};
  std::size_t edges{0U};
  std::uint64_t pressure{0U};
  bool budget_exhausted{false};
};

[[nodiscard]] bool bounds_overlap(const geometry::AxisAlignedBounds& lhs,
                                  const geometry::AxisAlignedBounds& rhs) {
  return lhs.minimum.x <= rhs.maximum.x && rhs.minimum.x <= lhs.maximum.x &&
         lhs.minimum.y <= rhs.maximum.y && rhs.minimum.y <= lhs.maximum.y;
}

// Physical feedback is prepared once per global pass. Conservative AABB rasterization is
// intentional: it changes only non-negative search cost, never graph legality or capacity.
[[nodiscard]] PreparedRoutabilityHistory
prepare_routability_history(const Grid& grid,
                            const SharedVoronoiAllocationGraph& voronoi_allocation,
                            const GlobalRouterConfig& config) {
  PreparedRoutabilityHistory prepared;
  prepared.entries = config.routability_history.size();
  if (config.routability_history.empty() || config.routability_history_cost == 0U ||
      config.max_routability_history_edges == 0U) {
    prepared.budget_exhausted = !config.routability_history.empty() &&
                                config.routability_history_cost > 0U &&
                                config.max_routability_history_edges == 0U;
    return prepared;
  }
  for (const auto& entry : config.routability_history) {
    prepared.pressure = saturating_add(prepared.pressure, entry.pressure);
    const auto clamped_index = [](std::int64_t coordinate, std::int64_t origin, std::int64_t pitch,
                                  std::size_t extent) {
      const auto relative = coordinate - origin;
      auto index = relative / pitch;
      if (relative < 0 && relative % pitch != 0) {
        --index;
      }
      index = std::clamp<std::int64_t>(index - 1, 0, static_cast<std::int64_t>(extent - 1U));
      return static_cast<std::size_t>(index);
    };
    const auto x_begin = clamped_index(entry.resource.bounds.minimum.x.value(), grid.origin_x,
                                       grid.pitch, grid.width);
    const auto y_begin = clamped_index(entry.resource.bounds.minimum.y.value(), grid.origin_y,
                                       grid.pitch, grid.height);
    const auto x_end =
        std::min(grid.width - 1U, clamped_index(entry.resource.bounds.maximum.x.value(),
                                                grid.origin_x, grid.pitch, grid.width) +
                                      2U);
    const auto y_end =
        std::min(grid.height - 1U, clamped_index(entry.resource.bounds.maximum.y.value(),
                                                 grid.origin_y, grid.pitch, grid.height) +
                                       2U);
    const auto layer_matches = [&](std::size_t layer) {
      return entry.resource.layer.value().empty() || grid.layers[layer] == entry.resource.layer;
    };
    for (std::size_t layer = 0U; layer < grid.layers.size(); ++layer) {
      if (!layer_matches(layer)) {
        continue;
      }
      for (std::size_t y = y_begin; y <= y_end; ++y) {
        for (std::size_t x = x_begin; x <= x_end; ++x) {
          if (!grid.valid(x, y)) {
            continue;
          }
          const auto node = grid.node(x, y, layer);
          for (const auto next : neighbors(grid, voronoi_allocation, node,
                                           config.use_direct_voronoi_resource_edges)) {
            if (node >= next) {
              continue;
            }
            const auto [next_x, next_y, next_layer] = grid.decode(next);
            // A diagnosed via-site conflict should move the layer transition, not repel the
            // complete planar run from an otherwise useful corridor. Restrict that feedback to
            // via edges; other diagnosis classes retain their conservative all-edge pressure.
            if (entry.kind == RoutabilityFailureKind::via_site_exhausted && layer == next_layer) {
              continue;
            }
            const geometry::AxisAlignedBounds edge_bounds{
                {std::min(grid.point(x, y).x, grid.point(next_x, next_y).x),
                 std::min(grid.point(x, y).y, grid.point(next_x, next_y).y)},
                {std::max(grid.point(x, y).x, grid.point(next_x, next_y).x),
                 std::max(grid.point(x, y).y, grid.point(next_x, next_y).y)}};
            if (!bounds_overlap(edge_bounds, entry.resource.bounds)) {
              continue;
            }
            for (const auto& net : entry.resource.nets) {
              const RoutingHistory::ClearanceEdgeKey key{net, Edge{node, next}};
              auto found = prepared.history.routability_edges.find(key);
              if (found == prepared.history.routability_edges.end() &&
                  prepared.history.routability_edges.size() >=
                      config.max_routability_history_edges) {
                prepared.budget_exhausted = true;
                prepared.edges = prepared.history.routability_edges.size();
                return prepared;
              }
              auto& value = found == prepared.history.routability_edges.end()
                                ? prepared.history.routability_edges.emplace(key, 0U).first->second
                                : found->second;
              const auto bounded_pressure =
                  entry.pressure >
                          static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max())
                      ? std::numeric_limits<std::size_t>::max()
                      : static_cast<std::size_t>(entry.pressure);
              value = saturating_count_add(value, bounded_pressure);
              prepared.edges = prepared.history.routability_edges.size();
            }
          }
        }
      }
    }
  }
  return prepared;
}

[[nodiscard]] std::optional<VoronoiPathFinderGuidance>
build_voronoi_pathfinder_guidance(const Grid& grid, const VoronoiCorridorEvidenceBundle& evidence,
                                  const GlobalRouterConfig& config, std::string& diagnostic) {
  VoronoiPathFinderGuidance result;
  if (!config.use_voronoi_pathfinder_guidance || evidence.edges.empty()) {
    return result;
  }
  std::set<NetId> nets;
  for (const auto& edge : evidence.edges) {
    nets.insert(edge.net);
  }
  const auto node_count = grid.layer_size() * grid.layers.size();
  if (node_count == 0U || nets.size() > config.max_voronoi_guidance_cells / node_count) {
    diagnostic = "Voronoi PathFinder guidance exceeds its net/grid cell budget";
    return std::nullopt;
  }

  const auto coordinate = [&grid](Point point) {
    const auto x =
        std::clamp<std::int64_t>(static_cast<std::int64_t>(std::llround(
                                     static_cast<long double>(point.x.value() - grid.origin_x) /
                                     static_cast<long double>(grid.pitch))),
                                 0, static_cast<std::int64_t>(grid.width - 1U));
    const auto y =
        std::clamp<std::int64_t>(static_cast<std::int64_t>(std::llround(
                                     static_cast<long double>(point.y.value() - grid.origin_y) /
                                     static_cast<long double>(grid.pitch))),
                                 0, static_cast<std::int64_t>(grid.height - 1U));
    return std::pair{x, y};
  };

  for (const auto& net : nets) {
    auto& distances = result.distance_steps_by_net[net];
    distances.assign(node_count, std::numeric_limits<std::uint32_t>::max());
    std::queue<std::size_t> open;
    const auto seed = [&](std::int64_t x, std::int64_t y, std::size_t layer) {
      const auto grid_x = static_cast<std::size_t>(x);
      const auto grid_y = static_cast<std::size_t>(y);
      if (!grid.valid(grid_x, grid_y)) {
        return;
      }
      const auto node = grid.node(grid_x, grid_y, layer);
      if (distances[node] != 0U) {
        distances[node] = 0U;
        open.push(node);
        ++result.seed_nodes;
      }
    };
    for (const auto& edge : evidence.edges) {
      if (edge.net != net) {
        continue;
      }
      const auto layer = std::find(grid.layers.begin(), grid.layers.end(), edge.layer);
      if (layer == grid.layers.end()) {
        continue;
      }
      const auto layer_index = static_cast<std::size_t>(layer - grid.layers.begin());
      auto [x, y] = coordinate(edge.start);
      const auto [target_x, target_y] = coordinate(edge.end);
      const auto delta_x = std::abs(target_x - x);
      const auto step_x = x < target_x ? 1 : -1;
      const auto delta_y = -std::abs(target_y - y);
      const auto step_y = y < target_y ? 1 : -1;
      auto error = delta_x + delta_y;
      while (true) {
        seed(x, y, layer_index);
        if (x == target_x && y == target_y) {
          break;
        }
        const auto doubled_error = 2 * error;
        if (doubled_error >= delta_y) {
          error += delta_y;
          x += step_x;
        }
        if (doubled_error <= delta_x) {
          error += delta_x;
          y += step_y;
        }
      }
    }

    while (!open.empty()) {
      const auto current = open.front();
      open.pop();
      const auto [x, y, layer] = grid.decode(current);
      const auto visit = [&](std::size_t next_x, std::size_t next_y) {
        const auto next = grid.node(next_x, next_y, layer);
        if (distances[next] == std::numeric_limits<std::uint32_t>::max()) {
          distances[next] = distances[current] + 1U;
          open.push(next);
        }
      };
      if (x > 0U && grid.valid(x - 1U, y)) {
        visit(x - 1U, y);
      }
      if (x + 1U < grid.width && grid.valid(x + 1U, y)) {
        visit(x + 1U, y);
      }
      if (y > 0U && grid.valid(x, y - 1U)) {
        visit(x, y - 1U);
      }
      if (y + 1U < grid.height && grid.valid(x, y + 1U)) {
        visit(x, y + 1U);
      }
    }
  }
  return result;
}

[[nodiscard]] std::optional<SharedVoronoiAllocationGraph> build_shared_voronoi_allocation_graph(
    const Board& board, const Grid& grid, const std::vector<NetTask>& tasks,
    const GlobalRouterConfig& config, const RoutingObstacleIndex& obstacles,
    std::stop_token stop_token, std::string& diagnostic) {
  SharedVoronoiAllocationGraph result;
  if (!config.use_voronoi_allocation_resources) {
    return result;
  }
  if (config.max_voronoi_allocation_resources == 0U || tasks.empty()) {
    diagnostic = "shared Voronoi allocation requires a positive resource budget and routed nets";
    return std::nullopt;
  }

  std::uint64_t minimum_demand = std::numeric_limits<std::uint64_t>::max();
  std::set<NetId> routed_nets;
  for (const auto& task : tasks) {
    const auto rules = resolve_net_routing_rules(board, task.net);
    const auto spacing = static_cast<std::uint64_t>(rules.clearance.value());
    const auto width = static_cast<std::uint64_t>(rules.track_width.value());
    const auto demand = saturating_add(width, saturating_multiply(2U, spacing));
    result.demand_nm_by_net.emplace(task.net, demand);
    result.track_width_nm_by_net.emplace(task.net, width);
    result.clearance_nm_by_net.emplace(task.net, spacing);
    routed_nets.insert(task.net);
    minimum_demand = std::min(minimum_demand, demand);
  }
  for (const auto& pair : board.differential_pairs) {
    if (!routed_nets.contains(pair.positive_net) || !routed_nets.contains(pair.negative_net)) {
      continue;
    }
    const auto positive = resolve_net_routing_rules(board, pair.positive_net);
    const auto negative = resolve_net_routing_rules(board, pair.negative_net);
    auto gap = std::max(positive.differential_pair_gap, negative.differential_pair_gap);
    if (pair.minimum_gap) {
      gap = std::max(gap, *pair.minimum_gap);
    }
    const auto boundary_clearance = std::max(positive.clearance, negative.clearance);
    auto demand = saturating_add(static_cast<std::uint64_t>(positive.track_width.value()),
                                 static_cast<std::uint64_t>(negative.track_width.value()));
    demand = saturating_add(demand, static_cast<std::uint64_t>(gap.value()));
    demand = saturating_add(
        demand, saturating_multiply(2U, static_cast<std::uint64_t>(boundary_clearance.value())));
    const auto key = pair.positive_net < pair.negative_net
                         ? std::pair{pair.positive_net, pair.negative_net}
                         : std::pair{pair.negative_net, pair.positive_net};
    result.differential_complement_by_net[pair.positive_net] = pair.negative_net;
    result.differential_complement_by_net[pair.negative_net] = pair.positive_net;
    result.differential_pair_demand_nm.emplace(key, demand);
    result.differential_pair_cross_sections.emplace(
        key, DifferentialPairCrossSection{pair.positive_net, pair.negative_net,
                                          static_cast<std::uint64_t>(positive.track_width.value()),
                                          static_cast<std::uint64_t>(negative.track_width.value()),
                                          static_cast<std::uint64_t>(gap.value()),
                                          static_cast<std::uint64_t>(boundary_clearance.value()),
                                          demand});
  }
  for (const auto& bus : board.routing_buses) {
    if (!std::all_of(bus.ordered_nets.begin(), bus.ordered_nets.end(),
                     [&routed_nets](const NetId& net) { return routed_nets.contains(net); })) {
      continue;
    }
    RoutingBusCrossSection section;
    section.id = bus.id;
    section.ordered_nets = bus.ordered_nets;
    section.widths_nm.reserve(bus.ordered_nets.size());
    if (bus.ordered_nets.size() > 1U) {
      section.gaps_nm.reserve(bus.ordered_nets.size() - 1U);
    }
    std::vector<std::uint64_t> clearances;
    clearances.reserve(bus.ordered_nets.size());
    for (const auto& net : bus.ordered_nets) {
      const auto rules = resolve_net_routing_rules(board, net);
      const auto width = static_cast<std::uint64_t>(rules.track_width.value());
      const auto clearance = static_cast<std::uint64_t>(rules.clearance.value());
      section.widths_nm.push_back(width);
      clearances.push_back(clearance);
      section.boundary_clearance_nm = std::max(section.boundary_clearance_nm, clearance);
      section.demand_nm = saturating_add(section.demand_nm, width);
      result.routing_bus_by_net.emplace(net, bus.id);
    }
    const auto explicit_gap =
        bus.minimum_gap ? static_cast<std::uint64_t>(bus.minimum_gap->value()) : 0U;
    for (std::size_t index = 1U; index < clearances.size(); ++index) {
      const auto gap = std::max({explicit_gap, clearances[index - 1U], clearances[index]});
      section.gaps_nm.push_back(gap);
      section.demand_nm = saturating_add(section.demand_nm, gap);
    }
    section.demand_nm =
        saturating_add(section.demand_nm, saturating_multiply(2U, section.boundary_clearance_nm));
    result.routing_bus_cross_sections.emplace(bus.id, std::move(section));
  }

  result.max_medial_neck_down_length_nm =
      static_cast<std::uint64_t>(config.max_medial_neck_down_length.value());
  if (config.medial_neck_down_width.value() > 0) {
    for (const auto& task : tasks) {
      if (task.coupled_member || config.medial_neck_down_width >= task.nominal_track_width) {
        continue;
      }
      const auto clearance = result.clearance_nm_by_net.at(task.net);
      const auto demand =
          saturating_add(static_cast<std::uint64_t>(config.medial_neck_down_width.value()),
                         saturating_multiply(2U, clearance));
      result.medial_neck_down_demand_nm_by_net.emplace(task.net, demand);
      result.medial_neck_down_width_by_net.emplace(task.net, config.medial_neck_down_width);
      minimum_demand = std::min(minimum_demand, demand);
    }
  }
  if (config.corridor_neck_down_width.value() > 0) {
    for (const auto& task : tasks) {
      if (task.coupled_member || config.corridor_neck_down_width >= task.nominal_track_width) {
        continue;
      }
      const auto demand = result.demand_for_width(task.net, config.corridor_neck_down_width);
      minimum_demand = std::min(minimum_demand, demand);
    }
  }

  result.minimum_demand_nm = minimum_demand;
  const auto coordinate = [&grid](Point point) {
    const auto x =
        std::clamp<std::int64_t>(static_cast<std::int64_t>(std::llround(
                                     static_cast<long double>(point.x.value() - grid.origin_x) /
                                     static_cast<long double>(grid.pitch))),
                                 0, static_cast<std::int64_t>(grid.width - 1U));
    const auto y =
        std::clamp<std::int64_t>(static_cast<std::int64_t>(std::llround(
                                     static_cast<long double>(point.y.value() - grid.origin_y) /
                                     static_cast<long double>(grid.pitch))),
                                 0, static_cast<std::int64_t>(grid.height - 1U));
    return std::pair{x, y};
  };

  for (std::size_t layer_index = 0; layer_index < grid.layers.size(); ++layer_index) {
    if (stop_token.stop_requested()) {
      diagnostic = "global routing cancelled during shared Voronoi allocation construction";
      return std::nullopt;
    }
    const auto extracted = extract_polygonal_board_free_space(
        board, BoardFreeSpaceExtractionConfig{grid.layers[layer_index], tasks.front().net,
                                              config.respect_existing_copper, false,
                                              !config.treat_saved_zone_fills_as_refillable});
    if (!extracted.complete()) {
      diagnostic = "shared Voronoi allocation cannot represent every physical obstacle";
      return std::nullopt;
    }
    const auto offset = offset_polygonal_free_space(
        extracted.domain, FreeSpaceOffsetConfig{Nanometres{0}, 1'000'000U});
    if (!offset.ok()) {
      diagnostic = "shared Voronoi allocation free-space union failed on " +
                   grid.layers[layer_index].value() + ": " + offset.diagnostic;
      return std::nullopt;
    }
    for (const auto& region : offset.regions) {
      FreeSpaceDomain region_domain;
      region_domain.outline = region.area;
      region_domain.cutouts = region.holes;
      const auto skeleton = build_free_space_skeleton(
          region_domain, FreeSpaceSkeletonConfig{Nanometres{}, 200'000U,
                                                 config.require_voronoi_topological_transitions});
      result.withheld_curved_edges += skeleton.skipped_curved_edges;
      if (!skeleton.ok()) {
        diagnostic = "shared Voronoi allocation skeleton failed: " + skeleton.diagnostic;
        return std::nullopt;
      }
      std::vector<std::optional<Edge>> direct_edges(skeleton.edges.size());
      std::vector<std::optional<Nanometres>> snapped_clearances(skeleton.edges.size());
      if (config.use_direct_voronoi_resource_edges) {
        std::vector<std::pair<Point, Point>> snapped_chords;
        std::vector<std::size_t> snapped_indices;
        snapped_chords.reserve(skeleton.edges.size());
        snapped_indices.reserve(skeleton.edges.size());
        for (std::size_t index = 0U; index < skeleton.edges.size(); ++index) {
          const auto& edge = skeleton.edges[index];
          const auto source_clearance = static_cast<std::uint64_t>(
              std::max<std::int64_t>(0, edge.minimum_boundary_clearance.value()));
          if (saturating_multiply(2U, source_clearance) < minimum_demand) {
            continue;
          }
          const auto [start_x, start_y] = coordinate(edge.start);
          const auto [end_x, end_y] = coordinate(edge.end);
          if (!grid.valid(static_cast<std::size_t>(start_x), static_cast<std::size_t>(start_y)) ||
              !grid.valid(static_cast<std::size_t>(end_x), static_cast<std::size_t>(end_y))) {
            ++result.withheld_direct_edges;
            continue;
          }
          const auto start_node = grid.node(static_cast<std::size_t>(start_x),
                                            static_cast<std::size_t>(start_y), layer_index);
          const auto end_node = grid.node(static_cast<std::size_t>(end_x),
                                          static_cast<std::size_t>(end_y), layer_index);
          const auto portal_steps =
              static_cast<std::size_t>(std::abs(end_x - start_x) + std::abs(end_y - start_y));
          if (start_node == end_node || portal_steps <= 1U) {
            ++result.withheld_direct_edges;
            continue;
          }
          direct_edges[index].emplace(start_node, end_node);
          snapped_chords.emplace_back(
              grid.point(static_cast<std::size_t>(start_x), static_cast<std::size_t>(start_y)),
              grid.point(static_cast<std::size_t>(end_x), static_cast<std::size_t>(end_y)));
          snapped_indices.push_back(index);
        }
        const auto assessments = assess_free_space_chords(region_domain, snapped_chords);
        for (std::size_t candidate = 0U; candidate < assessments.size(); ++candidate) {
          const auto index = snapped_indices[candidate];
          if (!assessments[candidate]) {
            direct_edges[index].reset();
            ++result.withheld_direct_edges;
          } else {
            snapped_clearances[index] = assessments[candidate];
          }
        }
      }
      for (std::size_t index = 0U; index < skeleton.edges.size(); ++index) {
        const auto& edge = skeleton.edges[index];
        const auto clearance = static_cast<std::uint64_t>(
            std::max<std::int64_t>(0, edge.minimum_boundary_clearance.value()));
        auto capacity = saturating_multiply(2U, clearance);
        if (capacity < minimum_demand) {
          continue;
        }
        if (direct_edges[index]) {
          if (!snapped_clearances[index]) {
            direct_edges[index].reset();
            ++result.withheld_direct_edges;
          } else {
            const auto snapped_capacity = saturating_multiply(
                2U, static_cast<std::uint64_t>(snapped_clearances[index]->value()));
            if (snapped_capacity < minimum_demand) {
              direct_edges[index].reset();
              ++result.withheld_direct_edges;
            } else {
              capacity = std::min(capacity, snapped_capacity);
            }
          }
        }
        auto limiting_obstacles = limiting_free_space_obstacles(
            extracted, edge.start, edge.end, Nanometres{static_cast<std::int64_t>(clearance)});
        if (direct_edges[index] && snapped_clearances[index]) {
          const auto [start_x, start_y, start_layer] = grid.decode(direct_edges[index]->first);
          const auto [end_x, end_y, end_layer] = grid.decode(direct_edges[index]->second);
          static_cast<void>(start_layer);
          static_cast<void>(end_layer);
          auto snapped_obstacles =
              limiting_free_space_obstacles(extracted, grid.point(start_x, start_y),
                                            grid.point(end_x, end_y), *snapped_clearances[index]);
          limiting_obstacles.insert(limiting_obstacles.end(), snapped_obstacles.begin(),
                                    snapped_obstacles.end());
          std::sort(limiting_obstacles.begin(), limiting_obstacles.end());
          limiting_obstacles.erase(
              std::unique(limiting_obstacles.begin(), limiting_obstacles.end()),
              limiting_obstacles.end());
        }
        std::uint64_t direct_length_nm = 0U;
        if (direct_edges[index]) {
          const auto [start_x, start_y, start_layer] = grid.decode(direct_edges[index]->first);
          const auto [end_x, end_y, end_layer] = grid.decode(direct_edges[index]->second);
          static_cast<void>(start_layer);
          static_cast<void>(end_layer);
          const auto length =
              geometry::point_distance_nm(grid.point(start_x, start_y), grid.point(end_x, end_y));
          direct_length_nm = static_cast<std::uint64_t>(std::ceil(length));
        }
        std::vector<Point> interval_points{edge.start, edge.end};
        const auto segment_limit = config.max_voronoi_resource_segment_length.value();
        if (segment_limit > 0 && !direct_edges[index]) {
          const auto source_length = geometry::point_distance_nm(edge.start, edge.end);
          const auto required_intervals = std::max<long double>(
              1.0L, std::ceil(source_length / static_cast<long double>(segment_limit)));
          const auto remaining_budget =
              config.max_voronoi_allocation_resources - result.resources.size();
          if (required_intervals > static_cast<long double>(remaining_budget)) {
            diagnostic = "longitudinal shared Voronoi allocation exceeds its resource budget";
            return std::nullopt;
          }
          auto interval_count = static_cast<std::size_t>(required_intervals);
          const auto interpolate = [&edge](std::size_t part, std::size_t count) {
            const auto ratio = static_cast<long double>(part) / static_cast<long double>(count);
            return Point{Nanometres{static_cast<std::int64_t>(std::llround(
                             static_cast<long double>(edge.start.x.value()) +
                             ratio * (static_cast<long double>(edge.end.x.value()) -
                                      static_cast<long double>(edge.start.x.value()))))},
                         Nanometres{static_cast<std::int64_t>(std::llround(
                             static_cast<long double>(edge.start.y.value()) +
                             ratio * (static_cast<long double>(edge.end.y.value()) -
                                      static_cast<long double>(edge.start.y.value()))))}};
          };
          while (true) {
            std::vector<Point> candidate;
            candidate.reserve(interval_count + 1U);
            for (std::size_t part = 0U; part <= interval_count; ++part) {
              const auto point = interpolate(part, interval_count);
              if (candidate.empty() || candidate.back() != point) {
                candidate.push_back(point);
              }
            }
            const auto within_limit =
                candidate.size() >= 2U &&
                std::adjacent_find(candidate.begin(), candidate.end(),
                                   [segment_limit](Point first, Point second) {
                                     return geometry::point_distance_nm(first, second) >
                                            static_cast<long double>(segment_limit);
                                   }) == candidate.end();
            if (within_limit) {
              interval_points = std::move(candidate);
              break;
            }
            if (interval_count >= remaining_budget) {
              diagnostic = "quantized longitudinal Voronoi intervals exceed their resource budget";
              return std::nullopt;
            }
            ++interval_count;
          }
        }
        const auto child_count = interval_points.size() > 1U ? interval_points.size() - 1U : 0U;
        if (child_count > 1U) {
          ++result.segmented_source_resources;
          result.additional_segment_resources =
              saturating_count_add(result.additional_segment_resources, child_count - 1U);
        }
        for (std::size_t child = 0U; child < child_count; ++child) {
          if (result.resources.size() >= config.max_voronoi_allocation_resources) {
            diagnostic = "longitudinal shared Voronoi allocation exceeds its resource budget";
            return std::nullopt;
          }
          const auto id = result.resources.size();
          result.resources.push_back(SharedVoronoiResource{
              id, layer_index, interval_points[child], interval_points[child + 1U], capacity,
              !extracted.conservative_obstacles.empty(), edge.curved_bisector_chord,
              limiting_obstacles, direct_edges[index], direct_length_nm});
        }
      }
    }
  }
  if (result.resources.empty()) {
    diagnostic = "shared Voronoi allocation found no positive-capacity straight resources";
    return std::nullopt;
  }

  const auto node_count = grid.layer_size() * grid.layers.size();
  result.direct_neighbors_by_node.resize(node_count);
  for (auto& resource : result.resources) {
    if (!resource.direct_edge) {
      continue;
    }
    const auto [inserted, unique] =
        result.direct_resource_by_edge.emplace(*resource.direct_edge, resource.id);
    static_cast<void>(inserted);
    if (!unique) {
      resource.direct_edge.reset();
      ++result.withheld_direct_edges;
      continue;
    }
    result.direct_neighbors_by_node[resource.direct_edge->first].push_back(
        resource.direct_edge->second);
    result.direct_neighbors_by_node[resource.direct_edge->second].push_back(
        resource.direct_edge->first);
  }
  for (auto& adjacent : result.direct_neighbors_by_node) {
    std::sort(adjacent.begin(), adjacent.end());
    adjacent.erase(std::unique(adjacent.begin(), adjacent.end()), adjacent.end());
  }

  std::map<std::pair<std::size_t, Point>, std::vector<std::size_t>> resources_by_endpoint;
  for (const auto& resource : result.resources) {
    resources_by_endpoint[{resource.layer, resource.start}].push_back(resource.id);
    resources_by_endpoint[{resource.layer, resource.end}].push_back(resource.id);
  }
  for (auto& [endpoint, resource_ids] : resources_by_endpoint) {
    static_cast<void>(endpoint);
    std::sort(resource_ids.begin(), resource_ids.end());
    resource_ids.erase(std::unique(resource_ids.begin(), resource_ids.end()), resource_ids.end());
    for (std::size_t first = 0U; first + 1U < resource_ids.size(); ++first) {
      for (std::size_t second = first + 1U; second < resource_ids.size(); ++second) {
        result.resource_adjacency.insert(Edge{resource_ids[first], resource_ids[second]});
      }
    }
  }

  constexpr auto none = std::numeric_limits<std::size_t>::max();
  result.nodes_per_layer = grid.layer_size();
  result.resource_by_node.assign(node_count, none);
  std::vector<std::uint32_t> distances(node_count, std::numeric_limits<std::uint32_t>::max());
  std::map<std::size_t, std::size_t> seeds;
  for (const auto& resource : result.resources) {
    auto [x, y] = coordinate(resource.start);
    const auto [target_x, target_y] = coordinate(resource.end);
    const auto delta_x = std::abs(target_x - x);
    const auto step_x = x < target_x ? 1 : -1;
    const auto delta_y = -std::abs(target_y - y);
    const auto step_y = y < target_y ? 1 : -1;
    auto error = delta_x + delta_y;
    while (true) {
      const auto grid_x = static_cast<std::size_t>(x);
      const auto grid_y = static_cast<std::size_t>(y);
      if (grid.valid(grid_x, grid_y)) {
        const auto node = grid.node(grid_x, grid_y, resource.layer);
        const auto found = seeds.find(node);
        if (found == seeds.end() || resource.id < found->second) {
          seeds[node] = resource.id;
        }
      }
      if (x == target_x && y == target_y) {
        break;
      }
      const auto doubled_error = 2 * error;
      if (doubled_error >= delta_y) {
        error += delta_y;
        x += step_x;
      }
      if (doubled_error <= delta_x) {
        error += delta_x;
        y += step_y;
      }
    }
  }

  std::queue<std::size_t> open;
  for (const auto& [node, resource] : seeds) {
    distances[node] = 0U;
    result.resource_by_node[node] = resource;
    open.push(node);
  }
  while (!open.empty()) {
    const auto current = open.front();
    open.pop();
    const auto [x, y, layer] = grid.decode(current);
    const auto visit = [&](std::size_t next_x, std::size_t next_y) {
      const auto next = grid.node(next_x, next_y, layer);
      const Edge edge{current, next};
      if (!obstacles.corridor_edge_free(edge)) {
        return;
      }
      const auto candidate_distance = distances[current] + 1U;
      const auto candidate_resource = result.resource_by_node[current];
      if (candidate_distance < distances[next] ||
          (candidate_distance == distances[next] &&
           candidate_resource < result.resource_by_node[next])) {
        distances[next] = candidate_distance;
        result.resource_by_node[next] = candidate_resource;
        open.push(next);
      }
    };
    if (x > 0U && grid.valid(x - 1U, y)) {
      visit(x - 1U, y);
    }
    if (x + 1U < grid.width && grid.valid(x + 1U, y)) {
      visit(x + 1U, y);
    }
    if (y > 0U && grid.valid(x, y - 1U)) {
      visit(x, y - 1U);
    }
    if (y + 1U < grid.height && grid.valid(x, y + 1U)) {
      visit(x, y + 1U);
    }
  }
  if (config.require_voronoi_topological_transitions && !result.direct_resource_by_edge.empty()) {
    const auto compatible = [&result, none](const Edge& edge, std::size_t resource) {
      for (const auto node : {edge.first, edge.second}) {
        if (node < result.resource_by_node.size() && result.resource_by_node[node] != none &&
            result.resource_by_node[node] != resource &&
            !result.resource_adjacency.contains(Edge{result.resource_by_node[node], resource})) {
          return false;
        }
      }
      return true;
    };
    for (auto entry = result.direct_resource_by_edge.begin();
         entry != result.direct_resource_by_edge.end();) {
      if (compatible(entry->first, entry->second)) {
        ++entry;
        continue;
      }
      result.resources[entry->second].direct_edge.reset();
      entry = result.direct_resource_by_edge.erase(entry);
      ++result.withheld_direct_edges;
    }
    for (auto& adjacent : result.direct_neighbors_by_node) {
      adjacent.clear();
    }
    for (const auto& [edge, resource] : result.direct_resource_by_edge) {
      static_cast<void>(resource);
      result.direct_neighbors_by_node[edge.first].push_back(edge.second);
      result.direct_neighbors_by_node[edge.second].push_back(edge.first);
    }
    for (auto& adjacent : result.direct_neighbors_by_node) {
      std::sort(adjacent.begin(), adjacent.end());
      adjacent.erase(std::unique(adjacent.begin(), adjacent.end()), adjacent.end());
    }
  }
  if (config.max_voronoi_resource_segment_length.value() > 0) {
    result.direct_resources_retained_atomic = static_cast<std::size_t>(std::count_if(
        result.resources.begin(), result.resources.end(),
        [](const SharedVoronoiResource& resource) { return resource.direct_edge.has_value(); }));
  }
  return result;
}

[[nodiscard]] std::uint64_t heuristic(const Grid& grid, std::size_t node,
                                      const std::set<std::size_t>& goals,
                                      const GlobalRouterConfig& config, bool use_direct_edges) {
  const auto [x, y, layer] = grid.decode(node);
  auto best = std::numeric_limits<std::uint64_t>::max();
  for (const auto goal : goals) {
    const auto [goal_x, goal_y, goal_layer] = grid.decode(goal);
    const auto delta_x = static_cast<long double>(x > goal_x ? x - goal_x : goal_x - x);
    const auto delta_y = static_cast<long double>(y > goal_y ? y - goal_y : goal_y - y);
    const auto xy_distance =
        use_direct_edges ? static_cast<std::uint64_t>(std::floor(std::hypotl(delta_x, delta_y)))
                         : static_cast<std::uint64_t>(delta_x + delta_y);
    const auto layer_distance =
        static_cast<std::uint64_t>(layer > goal_layer ? layer - goal_layer : goal_layer - layer);
    const auto estimate = saturating_add(saturating_multiply(xy_distance, config.step_cost),
                                         saturating_multiply(layer_distance, config.via_cost));
    best = std::min(best, estimate);
  }
  return best;
}

[[nodiscard]] std::uint64_t weighted_heuristic(std::uint64_t value,
                                               std::uint32_t weight_per_mille) {
  constexpr std::uint64_t scale = 1'000U;
  const auto whole = saturating_multiply(value / scale, weight_per_mille);
  const auto remainder = saturating_multiply(value % scale, weight_per_mille) / scale;
  return saturating_add(whole, remainder);
}

[[nodiscard]] std::uint64_t scale_per_mille(std::uint64_t value,
                                            std::uint64_t scale_per_mille_value) {
  constexpr std::uint64_t scale = 1'000U;
  const auto whole = saturating_multiply(value / scale, scale_per_mille_value);
  const auto value_remainder = value % scale;
  const auto remainder_whole = saturating_multiply(value_remainder, scale_per_mille_value / scale);
  const auto remainder_fraction = (value_remainder * (scale_per_mille_value % scale)) / scale;
  return saturating_add(whole, saturating_add(remainder_whole, remainder_fraction));
}

[[nodiscard]] std::uint64_t direct_edge_base_cost(const Grid& grid, const Edge& edge,
                                                  const GlobalRouterConfig& config) {
  const auto [first_x, first_y, first_layer] = grid.decode(edge.first);
  const auto [second_x, second_y, second_layer] = grid.decode(edge.second);
  if (first_layer != second_layer) {
    return config.via_cost;
  }
  const auto distance =
      geometry::point_distance_nm(grid.point(first_x, first_y), grid.point(second_x, second_y));
  const auto floating_steps =
      std::max<long double>(1.0L, std::ceil(distance / static_cast<long double>(grid.pitch)));
  const auto steps =
      floating_steps >= static_cast<long double>(std::numeric_limits<std::uint64_t>::max())
          ? std::numeric_limits<std::uint64_t>::max()
          : static_cast<std::uint64_t>(floating_steps);
  return saturating_multiply(steps, config.step_cost);
}

struct BidirectionalMeetingSearch {
  std::vector<std::size_t> nodes;
  std::size_t expansions{0U};
};

// This deliberately solves only topology for the meeting window. Each half is subsequently run
// through ordinary costed A* and therefore retains all width, capacity, clearance-shadow, and
// hard-capacity gates. The bounded flood fill prevents a constrained terminal from committing the
// whole connection to the first one-sided route topology it discovers.
[[nodiscard]] BidirectionalMeetingSearch find_bidirectional_meeting_nodes(
    const Grid& grid, const std::vector<std::size_t>& starts, const std::set<std::size_t>& goals,
    const SharedVoronoiAllocationGraph& voronoi_allocation, RoutingObstacleIndex& obstacles,
    const NetTask& task, const GlobalRouterConfig& config, bool use_direct_edges,
    std::stop_token stop_token) {
  BidirectionalMeetingSearch result;
  const auto node_count = grid.layer_size() * grid.layers.size();
  if (starts.empty() || goals.empty() || node_count == 0U ||
      config.max_bidirectional_meeting_nodes == 0U ||
      config.max_bidirectional_meeting_expansions == 0U) {
    return result;
  }
  constexpr auto unseen = std::numeric_limits<std::uint32_t>::max();
  std::vector<std::uint32_t> from_start(node_count, unseen);
  std::vector<std::uint32_t> from_goal(node_count, unseen);
  std::deque<std::size_t> start_frontier;
  std::deque<std::size_t> goal_frontier;
  for (const auto node : starts) {
    if (node < node_count && from_start[node] == unseen) {
      from_start[node] = 0U;
      start_frontier.push_back(node);
    }
  }
  for (const auto node : goals) {
    if (node < node_count && from_goal[node] == unseen) {
      from_goal[node] = 0U;
      goal_frontier.push_back(node);
    }
  }
  if (start_frontier.empty() || goal_frontier.empty()) {
    return result;
  }
  std::map<std::size_t, std::uint64_t> meetings;
  const auto record_meeting = [&](std::size_t node) {
    if (from_start[node] == unseen || from_goal[node] == unseen) {
      return;
    }
    meetings.try_emplace(node, static_cast<std::uint64_t>(from_start[node]) + from_goal[node]);
  };
  for (const auto node : starts) {
    record_meeting(node);
  }
  bool expand_start = true;
  while (!start_frontier.empty() && !goal_frontier.empty() &&
         result.expansions < config.max_bidirectional_meeting_expansions &&
         meetings.size() < config.max_bidirectional_meeting_nodes && !stop_token.stop_requested()) {
    auto& frontier = expand_start ? start_frontier : goal_frontier;
    auto& this_distance = expand_start ? from_start : from_goal;
    auto& other_distance = expand_start ? from_goal : from_start;
    const auto node = frontier.front();
    frontier.pop_front();
    const auto [unused_x, unused_y, layer] = grid.decode(node);
    static_cast<void>(unused_x);
    static_cast<void>(unused_y);
    for (const auto next : neighbors(grid, voronoi_allocation, node, use_direct_edges)) {
      const auto [next_x, next_y, next_layer] = grid.decode(next);
      static_cast<void>(next_x);
      static_cast<void>(next_y);
      const Edge edge{node, next};
      const bool is_via = layer != next_layer;
      if (!is_via && config.require_voronoi_topological_transitions &&
          !voronoi_allocation.edge_topologically_supported(edge, use_direct_edges)) {
        continue;
      }
      if (obstacles.edge_blocked(task.net, edge)) {
        continue;
      }
      if (this_distance[next] == unseen) {
        this_distance[next] = this_distance[node] + 1U;
        frontier.push_back(next);
      }
      if (other_distance[next] != unseen) {
        record_meeting(next);
      }
      if (meetings.size() >= config.max_bidirectional_meeting_nodes) {
        break;
      }
    }
    ++result.expansions;
    expand_start = !expand_start;
  }
  std::vector<std::pair<std::uint64_t, std::size_t>> ordered;
  ordered.reserve(meetings.size());
  for (const auto& [node, distance] : meetings) {
    ordered.emplace_back(distance, node);
  }
  std::sort(ordered.begin(), ordered.end());
  result.nodes.reserve(ordered.size());
  for (const auto& [unused_distance, node] : ordered) {
    static_cast<void>(unused_distance);
    result.nodes.push_back(node);
  }
  return result;
}

[[nodiscard]] SearchResult
search_path(const Grid& grid, const std::vector<std::size_t>& starts,
            const std::set<std::size_t>& goals, const RoutingOccupancy& occupancy,
            const RoutingHistory& history, const CorridorGraph& corridors,
            const CorridorGraph& corridor_neck_downs,
            const VoronoiPathFinderGuidance& voronoi_guidance,
            const SharedVoronoiAllocationGraph& voronoi_allocation, RoutingObstacleIndex& obstacles,
            const NetTask& task, const GlobalRouterConfig& config, bool use_direct_edges,
            bool enforce_hard_capacity, std::uint32_t heuristic_weight_per_mille,
            const SearchCostPolicy& cost_policy, std::stop_token stop_token) {
  if (starts.empty() || goals.empty()) {
    return {};
  }
  const auto& net = task.net;
  const auto node_count = grid.layer_size() * grid.layers.size();
  const bool corridor_neck_down_eligible =
      config.corridor_neck_down_width.value() > 0 && !task.coupled_member &&
      config.corridor_neck_down_width < task.nominal_track_width &&
      !corridor_neck_downs.corridors.empty();
  const auto max_narrow_steps =
      corridor_neck_down_eligible
          ? static_cast<std::size_t>(config.max_corridor_neck_down_length.value() / grid.pitch)
          : 0U;
  const auto state_count = node_count + node_count * max_narrow_steps;
  const auto infinity = std::numeric_limits<std::uint64_t>::max();
  const auto complement = voronoi_allocation.differential_complement(net);
  const bool complement_allocated =
      complement &&
      std::any_of(occupancy.voronoi_resources.begin(), occupancy.voronoi_resources.end(),
                  [&complement](const auto& entry) { return entry.second.contains(*complement); });
  const auto* routing_bus = voronoi_allocation.routing_bus(net);
  const bool routing_bus_member_allocated =
      routing_bus != nullptr &&
      std::any_of(occupancy.voronoi_resources.begin(), occupancy.voronoi_resources.end(),
                  [&routing_bus, &net](const auto& entry) {
                    return std::any_of(routing_bus->ordered_nets.begin(),
                                       routing_bus->ordered_nets.end(),
                                       [&entry, &net](const NetId& member) {
                                         return member != net && entry.second.contains(member);
                                       });
                  });
  const auto narrow_state = [node_count, max_narrow_steps](std::size_t node, std::size_t steps) {
    return node_count + node * max_narrow_steps + steps - 1U;
  };
  const auto decode_state = [node_count, max_narrow_steps](std::size_t state) {
    if (state < node_count) {
      return std::tuple{state, false, std::size_t{0}};
    }
    const auto relative = state - node_count;
    return std::tuple{relative / max_narrow_steps, true, relative % max_narrow_steps + 1U};
  };
  std::vector<std::uint64_t> distances(state_count, infinity);
  std::vector<std::size_t> parent(state_count, state_count);
  std::vector<GlobalRouteSegmentWidthKind> parent_width_kind(state_count,
                                                             GlobalRouteSegmentWidthKind::nominal);
  std::vector<std::uint64_t> parent_river_following_discount(state_count, 0U);
  std::vector<std::uint64_t> parent_former_route_memory_penalty(state_count, 0U);
  std::vector<std::uint64_t> parent_terminal_escape_reservation_penalty(state_count, 0U);
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<>> open;

  for (const auto start : starts) {
    distances[start] = 0;
    open.push(QueueEntry{weighted_heuristic(heuristic(grid, start, goals, config, use_direct_edges),
                                            heuristic_weight_per_mille),
                         0, start});
  }

  std::size_t expansions = 0;
  std::size_t phase_expansions = 0;
  std::size_t topology_rejected_edges = 0U;
  std::size_t hard_sampled_capacity_rejected_transitions = 0U;
  std::size_t hard_corridor_capacity_rejected_transitions = 0U;
  std::size_t hard_voronoi_capacity_rejected_transitions = 0U;
  std::map<NetId, std::size_t> hard_capacity_blocking_nets;
  std::map<std::vector<NetId>, std::size_t> hard_capacity_blocker_sets;
  std::size_t hard_capacity_conflict_sets_withheld = 0U;
  bool encountered_expansion_limit = false;
  bool weighted_frontier_continuation_attempted = false;
  std::size_t weighted_frontier_states_reused = 0U;
  std::size_t weighted_frontier_continuation_limits = 0U;
  auto active_heuristic_weight_per_mille = heuristic_weight_per_mille;
  while (true) {
    while (!open.empty() && phase_expansions < config.max_expansions_per_connection) {
      if (stop_token.stop_requested()) {
        SearchResult cancelled;
        cancelled.cancelled = true;
        cancelled.expansions = expansions;
        cancelled.encountered_expansion_limit = encountered_expansion_limit;
        cancelled.weighted_frontier_continuation_attempted =
            weighted_frontier_continuation_attempted;
        cancelled.weighted_frontier_states_reused = weighted_frontier_states_reused;
        cancelled.weighted_frontier_continuation_limits = weighted_frontier_continuation_limits;
        cancelled.topology_rejected_edges = topology_rejected_edges;
        cancelled.hard_sampled_capacity_rejected_transitions =
            hard_sampled_capacity_rejected_transitions;
        cancelled.hard_corridor_capacity_rejected_transitions =
            hard_corridor_capacity_rejected_transitions;
        cancelled.hard_voronoi_capacity_rejected_transitions =
            hard_voronoi_capacity_rejected_transitions;
        cancelled.hard_capacity_blocking_nets = hard_capacity_blocking_nets;
        cancelled.hard_capacity_blocker_sets = hard_capacity_blocker_sets;
        cancelled.hard_capacity_conflict_sets_withheld = hard_capacity_conflict_sets_withheld;
        return cancelled;
      }
      const auto current = open.top();
      open.pop();
      if (current.cost != distances[current.state]) {
        continue;
      }
      const auto [current_node, current_narrow, current_narrow_steps] = decode_state(current.state);
      const auto [x, y, layer] = grid.decode(current_node);
      const bool current_is_search_start = parent[current.state] == state_count;
      const auto current_node_occupants = occupancy.nodes.find(current_node);
      const auto current_start_occupied =
          !current_is_search_start || current_node_occupants == occupancy.nodes.end()
              ? 0U
              : current_node_occupants->second.size() - current_node_occupants->second.count(net);
      const bool goal_exit_legal =
          !current_narrow ||
          !obstacles.pin_access_blocked(net, layer, grid.point(x, y), grid.point(x, y));
      const bool goal_start_capacity_legal = !enforce_hard_capacity || !current_is_search_start ||
                                             current_start_occupied < config.edge_capacity;
      if (goals.contains(current_node) && goal_exit_legal && goal_start_capacity_legal) {
        SearchResult result;
        result.cost = current.cost;
        result.expansions = expansions;
        result.encountered_expansion_limit = encountered_expansion_limit;
        result.weighted_frontier_continuation_attempted = weighted_frontier_continuation_attempted;
        result.weighted_frontier_continuation_recovered = weighted_frontier_continuation_attempted;
        result.weighted_frontier_states_reused = weighted_frontier_states_reused;
        result.weighted_frontier_continuation_limits = weighted_frontier_continuation_limits;
        result.topology_rejected_edges = topology_rejected_edges;
        result.hard_sampled_capacity_rejected_transitions =
            hard_sampled_capacity_rejected_transitions;
        result.hard_corridor_capacity_rejected_transitions =
            hard_corridor_capacity_rejected_transitions;
        result.hard_voronoi_capacity_rejected_transitions =
            hard_voronoi_capacity_rejected_transitions;
        result.hard_capacity_blocking_nets = hard_capacity_blocking_nets;
        result.hard_capacity_blocker_sets = hard_capacity_blocker_sets;
        result.hard_capacity_conflict_sets_withheld = hard_capacity_conflict_sets_withheld;
        std::vector<std::size_t> states;
        for (auto state = current.state; state != state_count; state = parent[state]) {
          states.push_back(state);
        }
        std::reverse(states.begin(), states.end());
        result.nodes.reserve(states.size());
        if (states.size() > 1U) {
          result.segment_widths.reserve(states.size() - 1U);
          result.segment_width_kinds.reserve(states.size() - 1U);
        }
        for (std::size_t index = 0U; index < states.size(); ++index) {
          const auto [node, narrow, steps] = decode_state(states[index]);
          static_cast<void>(narrow);
          static_cast<void>(steps);
          result.nodes.push_back(node);
          if (index == 0U) {
            continue;
          }
          const auto kind = parent_width_kind[states[index]];
          result.segment_width_kinds.push_back(kind);
          if (parent_river_following_discount[states[index]] > 0U) {
            result.river_following_edges = saturating_count_add(result.river_following_edges, 1U);
            result.river_following_cost_discount =
                saturating_add(result.river_following_cost_discount,
                               parent_river_following_discount[states[index]]);
          }
          if (parent_former_route_memory_penalty[states[index]] > 0U) {
            result.former_route_memory_penalty_events =
                saturating_count_add(result.former_route_memory_penalty_events, 1U);
            result.former_route_memory_penalty =
                saturating_add(result.former_route_memory_penalty,
                               parent_former_route_memory_penalty[states[index]]);
          }
          if (parent_terminal_escape_reservation_penalty[states[index]] > 0U) {
            result.terminal_escape_reservation_penalty_events =
                saturating_count_add(result.terminal_escape_reservation_penalty_events, 1U);
            result.terminal_escape_reservation_penalty =
                saturating_add(result.terminal_escape_reservation_penalty,
                               parent_terminal_escape_reservation_penalty[states[index]]);
          }
          if (kind == GlobalRouteSegmentWidthKind::corridor_neck_down) {
            result.segment_widths.push_back(config.corridor_neck_down_width);
          } else if (kind == GlobalRouteSegmentWidthKind::medial_neck_down) {
            const auto resource = voronoi_allocation.direct_resource(
                Edge{result.nodes[index - 1U], result.nodes[index]});
            result.segment_widths.push_back(
                resource ? voronoi_allocation.medial_neck_down_width(*resource, net)
                               .value_or(Nanometres{0})
                         : Nanometres{0});
          } else {
            result.segment_widths.push_back(Nanometres{0});
          }
        }
        if (cost_policy.residual_corridor_demand_cost > 0U) {
          std::set<std::pair<bool, std::size_t>> charged_resources;
          for (std::size_t index = 1U; index < result.nodes.size(); ++index) {
            const auto narrow = result.segment_width_kinds[index - 1U] ==
                                GlobalRouteSegmentWidthKind::corridor_neck_down;
            const auto& selected_corridors = narrow ? corridor_neck_downs : corridors;
            const auto* future_demand = narrow ? cost_policy.residual_corridor_neck_down_demand
                                               : cost_policy.residual_corridor_demand;
            if (future_demand == nullptr) {
              continue;
            }
            const auto corridor_id = selected_corridors.corridor_for(
                Edge{result.nodes[index - 1U], result.nodes[index]});
            if (!corridor_id || !charged_resources.emplace(narrow, *corridor_id).second) {
              continue;
            }
            const auto future = future_demand->find(*corridor_id);
            if (future == future_demand->end()) {
              continue;
            }
            const auto& selected_occupancy =
                narrow ? occupancy.corridor_neck_downs : occupancy.corridors;
            const auto occupied = [&] {
              const auto found = selected_occupancy.find(*corridor_id);
              return found == selected_occupancy.end()
                         ? 0U
                         : found->second.size() - found->second.count(net);
            }();
            const bool current_net_already_claims = [&] {
              const auto found = selected_occupancy.find(*corridor_id);
              return found != selected_occupancy.end() && found->second.contains(net);
            }();
            const auto capacity = selected_corridors.corridors[*corridor_id].capacity;
            const auto projected_demand = occupied + (current_net_already_claims ? 0U : 1U);
            const auto residual_capacity =
                capacity > projected_demand ? capacity - projected_demand : 0U;
            if (future->second > residual_capacity) {
              result.residual_corridor_demand_penalty_events =
                  saturating_count_add(result.residual_corridor_demand_penalty_events, 1U);
              result.residual_corridor_demand_penalty =
                  saturating_add(result.residual_corridor_demand_penalty,
                                 saturating_multiply(
                                     static_cast<std::uint64_t>(future->second - residual_capacity),
                                     cost_policy.residual_corridor_demand_cost));
            }
          }
        }
        if (cost_policy.critical_cut_cost > 0U) {
          std::set<std::pair<bool, std::size_t>> charged_cuts;
          for (std::size_t index = 1U; index < result.nodes.size(); ++index) {
            const auto narrow = result.segment_width_kinds[index - 1U] ==
                                GlobalRouteSegmentWidthKind::corridor_neck_down;
            const auto& selected_corridors = narrow ? corridor_neck_downs : corridors;
            const auto* cut_demand = narrow ? cost_policy.critical_corridor_neck_down_cut_demand
                                            : cost_policy.critical_corridor_cut_demand;
            const auto* current_reservation = narrow
                                                  ? cost_policy.current_reserved_corridor_neck_downs
                                                  : cost_policy.current_reserved_corridors;
            if (cut_demand == nullptr) {
              continue;
            }
            const auto corridor_id = selected_corridors.corridor_for(
                Edge{result.nodes[index - 1U], result.nodes[index]});
            if (!corridor_id || !charged_cuts.emplace(narrow, *corridor_id).second ||
                (current_reservation != nullptr && current_reservation->contains(*corridor_id))) {
              continue;
            }
            if (const auto cut = cut_demand->find(*corridor_id); cut != cut_demand->end()) {
              result.critical_cut_penalty_events =
                  saturating_count_add(result.critical_cut_penalty_events, 1U);
              result.critical_cut_penalty = saturating_add(
                  result.critical_cut_penalty,
                  critical_cut_penalty(cut->second,
                                       selected_corridors.corridors[*corridor_id].capacity,
                                       cost_policy.critical_cut_cost));
            }
          }
        }
        return result;
      }
      ++expansions;
      ++phase_expansions;

      for (const auto next : neighbors(grid, voronoi_allocation, current_node, use_direct_edges)) {
        const auto [next_x, next_y, next_layer] = grid.decode(next);
        const Edge edge{current_node, next};
        const bool is_via = layer != next_layer;
        const bool is_direct =
            use_direct_edges && voronoi_allocation.direct_resource(edge).has_value();
        if (!is_via && config.require_voronoi_topological_transitions &&
            !voronoi_allocation.edge_topologically_supported(edge, use_direct_edges)) {
          ++topology_rejected_edges;
          continue;
        }
        const auto direct_resource =
            is_direct ? voronoi_allocation.direct_resource(edge) : std::nullopt;
        const auto medial_neck_down =
            direct_resource ? voronoi_allocation.medial_neck_down_width(*direct_resource, net)
                            : std::nullopt;
        const bool nominal_blocked = obstacles.edge_blocked(net, edge);

        struct Transition {
          std::size_t state{0};
          GlobalRouteSegmentWidthKind width_kind{GlobalRouteSegmentWidthKind::nominal};
        };
        std::vector<Transition> transitions;
        transitions.reserve(2U);
        const bool nominal_exit_at_current =
            !current_narrow ||
            !obstacles.pin_access_blocked(net, layer, grid.point(x, y), grid.point(x, y));
        if (medial_neck_down) {
          if (nominal_exit_at_current &&
              !obstacles.pin_access_blocked(net, next_layer, grid.point(next_x, next_y),
                                            grid.point(next_x, next_y)) &&
              !obstacles.edge_blocked(net, edge, *medial_neck_down)) {
            transitions.push_back(Transition{next, GlobalRouteSegmentWidthKind::medial_neck_down});
          }
        } else if (!nominal_blocked && nominal_exit_at_current) {
          transitions.push_back(Transition{next, GlobalRouteSegmentWidthKind::nominal});
        }
        if (!is_via && !is_direct && corridor_neck_down_eligible &&
            corridor_neck_downs.corridor_for(edge) && (current_narrow || nominal_blocked) &&
            (current_narrow || nominal_exit_at_current) &&
            current_narrow_steps < max_narrow_steps &&
            !obstacles.edge_blocked(net, edge, config.corridor_neck_down_width)) {
          const auto steps = current_narrow ? current_narrow_steps + 1U : 1U;
          transitions.push_back(Transition{narrow_state(next, steps),
                                           GlobalRouteSegmentWidthKind::corridor_neck_down});
        }
        if (transitions.empty()) {
          continue;
        }
        const auto sampled_edge = is_direct ? edge : sampled_capacity_edge(grid, edge);
        const bool owns_sampled_capacity =
            !is_direct || config.single_lane_direct_voronoi_resources;
        const auto edge_occupants = occupancy.edges.find(sampled_edge);
        const auto edge_occupied =
            !owns_sampled_capacity || edge_occupants == occupancy.edges.end()
                ? 0U
                : edge_occupants->second.size() - edge_occupants->second.count(net);
        const auto node_occupants = occupancy.nodes.find(next);
        const auto node_occupied =
            node_occupants == occupancy.nodes.end()
                ? 0U
                : node_occupants->second.size() - node_occupants->second.count(net);
        const auto edge_history = owns_sampled_capacity && history.edges.contains(sampled_edge)
                                      ? history.edges.at(sampled_edge)
                                      : 0U;
        const auto node_history = history.nodes.contains(next) ? history.nodes.at(next) : 0U;
        const auto current_start_history =
            current_is_search_start && history.nodes.contains(current_node)
                ? history.nodes.at(current_node)
                : 0U;
        for (const auto& transition : transitions) {
          const auto selected_width =
              transition.width_kind == GlobalRouteSegmentWidthKind::corridor_neck_down
                  ? config.corridor_neck_down_width
              : transition.width_kind == GlobalRouteSegmentWidthKind::medial_neck_down
                  ? medial_neck_down.value_or(Nanometres{0})
                  : Nanometres{0};
          const auto candidate_radius =
              is_via ? static_cast<long double>(task.via_diameter.value()) / 2.0L
                     : static_cast<long double>(
                           (selected_width.value() > 0 ? selected_width : task.nominal_track_width)
                               .value()) /
                           2.0L;
          const auto required_from_occupied_copper =
              candidate_radius + static_cast<long double>(task.physical_clearance.value());
          const auto clearance_blockers = [&net, required_from_occupied_copper,
                                           &config](const auto& resources, const auto& key) {
            std::map<NetId, std::size_t> blockers;
            const auto found = resources.find(key);
            if (found == resources.end()) {
              return blockers;
            }
            for (const auto& [occupant, certificate] : found->second) {
              if (occupant != net && certificate.available_nm < required_from_occupied_copper) {
                const auto deficit = required_from_occupied_copper - certificate.available_nm;
                const auto quantum = config.clearance_shadow_cost_quantum.value();
                blockers.emplace(
                    occupant,
                    quantum > 0 ? static_cast<std::size_t>(std::max(
                                      1.0L, std::ceil(deficit / static_cast<long double>(quantum))))
                                : 1U);
              }
            }
            return blockers;
          };
          std::map<NetId, std::size_t> physical_shadow_blockers;
          if (config.reserve_sampled_route_clearance) {
            const auto merge_blockers = [&physical_shadow_blockers](const auto& blockers) {
              for (const auto& [blocker, pressure] : blockers) {
                auto& retained = physical_shadow_blockers[blocker];
                retained = std::max(retained, pressure);
              }
            };
            merge_blockers(clearance_blockers(occupancy.clearance_edges, sampled_edge));
            merge_blockers(clearance_blockers(occupancy.clearance_nodes, next));
            if (is_via || current_is_search_start) {
              merge_blockers(clearance_blockers(occupancy.clearance_nodes, current_node));
            }
          }
          if (enforce_hard_capacity) {
            bool hard_rejected = false;
            std::set<NetId> transition_blockers;
            if (!physical_shadow_blockers.empty()) {
              hard_rejected = true;
              for (const auto& [blocker, pressure] : physical_shadow_blockers) {
                static_cast<void>(pressure);
                transition_blockers.insert(blocker);
              }
            }
            const bool sampled_start_over_capacity = owns_sampled_capacity &&
                                                     current_is_search_start &&
                                                     current_start_occupied >= config.edge_capacity;
            if ((owns_sampled_capacity && (edge_occupied >= config.edge_capacity ||
                                           node_occupied >= config.edge_capacity)) ||
                sampled_start_over_capacity) {
              ++hard_sampled_capacity_rejected_transitions;
              hard_rejected = true;
              if (edge_occupied >= config.edge_capacity &&
                  edge_occupants != occupancy.edges.end()) {
                transition_blockers.insert(edge_occupants->second.begin(),
                                           edge_occupants->second.end());
              }
              if (node_occupied >= config.edge_capacity &&
                  node_occupants != occupancy.nodes.end()) {
                transition_blockers.insert(node_occupants->second.begin(),
                                           node_occupants->second.end());
              }
              if (sampled_start_over_capacity && current_node_occupants != occupancy.nodes.end()) {
                transition_blockers.insert(current_node_occupants->second.begin(),
                                           current_node_occupants->second.end());
              }
            }
            if (!is_direct) {
              const bool narrow =
                  transition.width_kind == GlobalRouteSegmentWidthKind::corridor_neck_down;
              const auto& selected_corridors = narrow ? corridor_neck_downs : corridors;
              if (const auto corridor_id = selected_corridors.corridor_for(edge)) {
                const auto& selected_occupancy =
                    narrow ? occupancy.corridor_neck_downs : occupancy.corridors;
                const auto found = selected_occupancy.find(*corridor_id);
                const auto occupied = found == selected_occupancy.end()
                                          ? 0U
                                          : found->second.size() - found->second.count(net);
                if (occupied >= selected_corridors.corridors[*corridor_id].capacity) {
                  ++hard_corridor_capacity_rejected_transitions;
                  hard_rejected = true;
                  if (found != selected_occupancy.end()) {
                    transition_blockers.insert(found->second.begin(), found->second.end());
                  }
                }
              }
            }
            if (!is_via && voronoi_allocation_edge_over_capacity(
                               voronoi_allocation, occupancy, net, edge, selected_width,
                               use_direct_edges, &transition_blockers)) {
              ++hard_voronoi_capacity_rejected_transitions;
              hard_rejected = true;
            }
            if (hard_rejected) {
              transition_blockers.erase(net);
              for (const auto& blocker : transition_blockers) {
                hard_capacity_blocking_nets[blocker] =
                    saturating_count_add(hard_capacity_blocking_nets[blocker], 1U);
              }
              if (!transition_blockers.empty() &&
                  config.max_hard_capacity_conflict_set_states_per_attempt > 0U) {
                std::vector<NetId> blocker_set(transition_blockers.begin(),
                                               transition_blockers.end());
                if (const auto found = hard_capacity_blocker_sets.find(blocker_set);
                    found != hard_capacity_blocker_sets.end()) {
                  found->second = saturating_count_add(found->second, 1U);
                } else if (hard_capacity_blocker_sets.size() <
                           config.max_hard_capacity_conflict_set_states_per_attempt) {
                  hard_capacity_blocker_sets.emplace(std::move(blocker_set), 1U);
                } else {
                  hard_capacity_conflict_sets_withheld =
                      saturating_count_add(hard_capacity_conflict_sets_withheld, 1U);
                }
              }
              continue;
            }
          }
          auto edge_cost = is_via ? config.via_cost : direct_edge_base_cost(grid, edge, config);
          std::uint64_t river_following_discount = 0U;
          std::uint64_t former_route_memory_penalty = 0U;
          std::uint64_t terminal_escape_penalty = 0U;
          if (cost_policy.kind != SearchCostPolicyKind::static_seed && !is_via &&
              cost_policy.former_route_memory != nullptr &&
              cost_policy.former_route_memory_cost > 0U) {
            if (const auto former = cost_policy.former_route_memory->edges.find(sampled_edge);
                former != cost_policy.former_route_memory->edges.end() &&
                std::any_of(former->second.begin(), former->second.end(),
                            [&net](const NetId& owner) { return owner != net; })) {
              former_route_memory_penalty = cost_policy.former_route_memory_cost;
              edge_cost = saturating_add(edge_cost, former_route_memory_penalty);
            }
          }
          if (cost_policy.kind != SearchCostPolicyKind::static_seed &&
              cost_policy.terminal_escape_reservation != nullptr &&
              cost_policy.terminal_escape_availability != nullptr &&
              cost_policy.terminal_escape_reservation_cost > 0U) {
            terminal_escape_penalty = terminal_escape_reservation_penalty(
                grid, *cost_policy.terminal_escape_reservation,
                *cost_policy.terminal_escape_availability, net, edge, is_direct,
                cost_policy.terminal_escape_reservation_cost, nullptr);
            edge_cost = saturating_add(edge_cost, terminal_escape_penalty);
          }
          if (const auto found =
                  history.routability_edges.find(RoutingHistory::ClearanceEdgeKey{net, edge});
              found != history.routability_edges.end()) {
            edge_cost = saturating_add(
                edge_cost, saturating_multiply(static_cast<std::uint64_t>(found->second),
                                               config.routability_history_cost));
          }
          if (!physical_shadow_blockers.empty()) {
            std::size_t physical_shadow_pressure = 0U;
            for (const auto& [blocker, pressure] : physical_shadow_blockers) {
              static_cast<void>(blocker);
              physical_shadow_pressure = saturating_count_add(physical_shadow_pressure, pressure);
            }
            edge_cost = saturating_add(
                edge_cost, saturating_multiply(static_cast<std::uint64_t>(physical_shadow_pressure),
                                               config.present_congestion_cost));
          }
          if (cost_policy.penalized_edges.contains(edge)) {
            edge_cost = saturating_add(edge_cost, cost_policy.alternative_edge_penalty);
          }
          if (cost_policy.kind == SearchCostPolicyKind::corridor_reprojection &&
              (!cost_policy.corridor_nodes.contains(current_node) ||
               !cost_policy.corridor_nodes.contains(next))) {
            edge_cost = saturating_add(edge_cost, cost_policy.corridor_departure_penalty);
          }
          if (edge_occupied >= config.edge_capacity) {
            edge_cost = saturating_add(
                edge_cost, saturating_multiply(static_cast<std::uint64_t>(
                                                   edge_occupied - config.edge_capacity + 1U),
                                               config.present_congestion_cost));
          }
          if (node_occupied >= config.edge_capacity) {
            edge_cost = saturating_add(
                edge_cost, saturating_multiply(static_cast<std::uint64_t>(
                                                   node_occupied - config.edge_capacity + 1U),
                                               config.present_congestion_cost));
          }
          if (owns_sampled_capacity && current_is_search_start &&
              current_start_occupied >= config.edge_capacity) {
            edge_cost = saturating_add(
                edge_cost,
                saturating_multiply(
                    static_cast<std::uint64_t>(current_start_occupied - config.edge_capacity + 1U),
                    config.present_congestion_cost));
          }
          edge_cost = saturating_add(edge_cost,
                                     saturating_multiply(static_cast<std::uint64_t>(edge_history),
                                                         config.history_congestion_cost));
          edge_cost = saturating_add(edge_cost,
                                     saturating_multiply(static_cast<std::uint64_t>(node_history),
                                                         config.history_congestion_cost));
          if (const auto found =
                  history.clearance_edges.find(RoutingHistory::ClearanceEdgeKey{net, sampled_edge});
              found != history.clearance_edges.end()) {
            edge_cost = saturating_add(
                edge_cost, saturating_multiply(static_cast<std::uint64_t>(found->second),
                                               config.history_congestion_cost));
          }
          if (const auto found =
                  history.clearance_nodes.find(RoutingHistory::ClearanceNodeKey{net, next});
              found != history.clearance_nodes.end()) {
            edge_cost = saturating_add(
                edge_cost, saturating_multiply(static_cast<std::uint64_t>(found->second),
                                               config.history_congestion_cost));
          }
          if (current_is_search_start) {
            if (const auto found = history.clearance_nodes.find(
                    RoutingHistory::ClearanceNodeKey{net, current_node});
                found != history.clearance_nodes.end()) {
              edge_cost = saturating_add(
                  edge_cost, saturating_multiply(static_cast<std::uint64_t>(found->second),
                                                 config.history_congestion_cost));
            }
          }
          if (owns_sampled_capacity && current_is_search_start) {
            edge_cost = saturating_add(
                edge_cost, saturating_multiply(static_cast<std::uint64_t>(current_start_history),
                                               config.history_congestion_cost));
          }
          if (!is_direct) {
            const bool narrow =
                transition.width_kind == GlobalRouteSegmentWidthKind::corridor_neck_down;
            const auto& selected_corridors = narrow ? corridor_neck_downs : corridors;
            if (const auto corridor_id = selected_corridors.corridor_for(edge)) {
              const auto& corridor = selected_corridors.corridors[*corridor_id];
              const auto& selected_occupancy =
                  narrow ? occupancy.corridor_neck_downs : occupancy.corridors;
              const auto found = selected_occupancy.find(*corridor_id);
              const auto occupied = found == selected_occupancy.end()
                                        ? 0U
                                        : found->second.size() - found->second.count(net);
              const auto& selected_history =
                  narrow ? history.corridor_neck_downs : history.corridors;
              const auto history_found = selected_history.find(*corridor_id);
              const auto corridor_history =
                  history_found == selected_history.end() ? 0U : history_found->second;
              const auto corridor_present_penalty =
                  saturating_multiply(static_cast<std::uint64_t>(occupied),
                                      config.corridor_present_cost) /
                  static_cast<std::uint64_t>(corridor.capacity);
              edge_cost = saturating_add(edge_cost, corridor_present_penalty);
              if (cost_policy.kind == SearchCostPolicyKind::negotiated &&
                  config.river_following_cost > 0U && physical_shadow_blockers.empty() &&
                  river_following_lane_available(grid, occupancy, net, edge, corridors,
                                                 corridor_neck_downs, narrow)) {
                // Preserve the physical base edge cost. The river preference may only offset
                // the ordinary cost of sharing a still-capacious corridor, never congestion
                // overflow, clearance history, or the exact DRC gate.
                river_following_discount =
                    std::min(config.river_following_cost, corridor_present_penalty);
              }
              if (occupied >= corridor.capacity) {
                edge_cost = saturating_add(
                    edge_cost, saturating_multiply(
                                   static_cast<std::uint64_t>(occupied - corridor.capacity + 1U),
                                   config.present_congestion_cost));
              }
              edge_cost = saturating_add(
                  edge_cost, saturating_multiply(static_cast<std::uint64_t>(corridor_history),
                                                 config.corridor_history_cost));
              // Corridor occupancy is a net set, so cost a cut only when this search enters it;
              // a second branch of the same net does not consume another lane.
              bool entering = current_is_search_start;
              if (!entering) {
                const auto [previous_node, unused_narrow, unused_narrow_steps] =
                    decode_state(parent[current.state]);
                static_cast<void>(unused_narrow);
                static_cast<void>(unused_narrow_steps);
                const auto previous =
                    selected_corridors.corridor_for(Edge{previous_node, current_node});
                entering = !previous || *previous != *corridor_id;
              }
              const auto* future_demand = narrow ? cost_policy.residual_corridor_neck_down_demand
                                                 : cost_policy.residual_corridor_demand;
              if (cost_policy.residual_corridor_demand_cost > 0U && future_demand != nullptr) {
                if (const auto future = future_demand->find(*corridor_id);
                    future != future_demand->end()) {
                  if (entering) {
                    const bool current_net_already_claims =
                        found != selected_occupancy.end() && found->second.contains(net);
                    const auto projected_demand = occupied + (current_net_already_claims ? 0U : 1U);
                    const auto residual_capacity = corridor.capacity > projected_demand
                                                       ? corridor.capacity - projected_demand
                                                       : 0U;
                    if (future->second > residual_capacity) {
                      edge_cost = saturating_add(
                          edge_cost,
                          saturating_multiply(
                              static_cast<std::uint64_t>(future->second - residual_capacity),
                              cost_policy.residual_corridor_demand_cost));
                    }
                  }
                }
              }
              const auto* critical_cut_demand =
                  narrow ? cost_policy.critical_corridor_neck_down_cut_demand
                         : cost_policy.critical_corridor_cut_demand;
              const auto* current_reservation =
                  narrow ? cost_policy.current_reserved_corridor_neck_downs
                         : cost_policy.current_reserved_corridors;
              if (entering && cost_policy.critical_cut_cost > 0U &&
                  critical_cut_demand != nullptr &&
                  (current_reservation == nullptr ||
                   !current_reservation->contains(*corridor_id))) {
                if (const auto cut = critical_cut_demand->find(*corridor_id);
                    cut != critical_cut_demand->end()) {
                  edge_cost = saturating_add(edge_cost,
                                             critical_cut_penalty(cut->second, corridor.capacity,
                                                                  cost_policy.critical_cut_cost));
                }
              }
              const auto [centering_numerator, centering_denominator] =
                  selected_corridors.centering_ratio(grid, edge);
              edge_cost =
                  saturating_add(edge_cost, saturating_multiply(centering_numerator,
                                                                config.corridor_centering_cost) /
                                                centering_denominator);
            }
          }
          if (!is_via) {
            edge_cost = saturating_add(
                edge_cost, width_reduction_penalty(grid.point(x, y), grid.point(next_x, next_y),
                                                   task.nominal_track_width, selected_width,
                                                   Nanometres{grid.pitch},
                                                   config.width_reduction_cost_per_grid_step)
                               .cost);
            if (cost_policy.kind == SearchCostPolicyKind::negotiated) {
              edge_cost = saturating_add(
                  edge_cost, voronoi_guidance.edge_cost(net, edge, config.voronoi_centering_cost));
            }
            edge_cost = saturating_add(
                edge_cost,
                voronoi_allocation_edge_cost(voronoi_allocation, occupancy, history, net,
                                             complement_allocated, routing_bus_member_allocated,
                                             edge, selected_width, config, use_direct_edges));
          }
          if (river_following_discount > 0U) {
            edge_cost -= river_following_discount;
          }
          const auto candidate = saturating_add(current.cost, edge_cost);
          if (candidate < distances[transition.state]) {
            distances[transition.state] = candidate;
            parent[transition.state] = current.state;
            parent_width_kind[transition.state] = transition.width_kind;
            parent_river_following_discount[transition.state] = river_following_discount;
            parent_former_route_memory_penalty[transition.state] = former_route_memory_penalty;
            parent_terminal_escape_reservation_penalty[transition.state] = terminal_escape_penalty;
            const auto estimate = saturating_add(
                candidate,
                weighted_heuristic(heuristic(grid, next, goals, config, use_direct_edges),
                                   active_heuristic_weight_per_mille));
            open.push(QueueEntry{estimate, candidate, transition.state});
          }
        }
        static_cast<void>(x);
        static_cast<void>(y);
        static_cast<void>(next_x);
        static_cast<void>(next_y);
      }
    }

    if (open.empty()) {
      SearchResult result;
      result.expansions = expansions;
      result.encountered_expansion_limit = encountered_expansion_limit;
      result.weighted_frontier_continuation_attempted = weighted_frontier_continuation_attempted;
      result.weighted_frontier_states_reused = weighted_frontier_states_reused;
      result.weighted_frontier_continuation_limits = weighted_frontier_continuation_limits;
      result.topology_rejected_edges = topology_rejected_edges;
      result.hard_sampled_capacity_rejected_transitions =
          hard_sampled_capacity_rejected_transitions;
      result.hard_corridor_capacity_rejected_transitions =
          hard_corridor_capacity_rejected_transitions;
      result.hard_voronoi_capacity_rejected_transitions =
          hard_voronoi_capacity_rejected_transitions;
      result.hard_capacity_blocking_nets = hard_capacity_blocking_nets;
      result.hard_capacity_blocker_sets = hard_capacity_blocker_sets;
      result.hard_capacity_conflict_sets_withheld = hard_capacity_conflict_sets_withheld;
      return result;
    }

    if (!weighted_frontier_continuation_attempted && heuristic_weight_per_mille == 1'000U &&
        config.reuse_weighted_search_frontier &&
        config.expansion_retry_heuristic_weight_per_mille > 1'000U) {
      std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<>> rekeyed;
      while (!open.empty()) {
        const auto entry = open.top();
        open.pop();
        if (entry.cost != distances[entry.state]) {
          continue;
        }
        const auto [node, narrow, narrow_steps] = decode_state(entry.state);
        static_cast<void>(narrow);
        static_cast<void>(narrow_steps);
        const auto estimate = saturating_add(
            entry.cost, weighted_heuristic(heuristic(grid, node, goals, config, use_direct_edges),
                                           config.expansion_retry_heuristic_weight_per_mille));
        rekeyed.push(QueueEntry{estimate, entry.cost, entry.state});
        ++weighted_frontier_states_reused;
      }
      if (rekeyed.empty()) {
        SearchResult result;
        result.expansions = expansions;
        result.topology_rejected_edges = topology_rejected_edges;
        result.hard_sampled_capacity_rejected_transitions =
            hard_sampled_capacity_rejected_transitions;
        result.hard_corridor_capacity_rejected_transitions =
            hard_corridor_capacity_rejected_transitions;
        result.hard_voronoi_capacity_rejected_transitions =
            hard_voronoi_capacity_rejected_transitions;
        result.hard_capacity_blocking_nets = hard_capacity_blocking_nets;
        result.hard_capacity_blocker_sets = hard_capacity_blocker_sets;
        result.hard_capacity_conflict_sets_withheld = hard_capacity_conflict_sets_withheld;
        return result;
      }
      encountered_expansion_limit = true;
      weighted_frontier_continuation_attempted = true;
      active_heuristic_weight_per_mille = config.expansion_retry_heuristic_weight_per_mille;
      phase_expansions = 0U;
      open = std::move(rekeyed);
      continue;
    }

    encountered_expansion_limit = true;
    if (weighted_frontier_continuation_attempted) {
      ++weighted_frontier_continuation_limits;
    }
    SearchResult result;
    result.expansions = expansions;
    result.expansion_limit_reached = true;
    result.encountered_expansion_limit = true;
    result.weighted_frontier_continuation_attempted = weighted_frontier_continuation_attempted;
    result.weighted_frontier_states_reused = weighted_frontier_states_reused;
    result.weighted_frontier_continuation_limits = weighted_frontier_continuation_limits;
    result.topology_rejected_edges = topology_rejected_edges;
    result.hard_sampled_capacity_rejected_transitions = hard_sampled_capacity_rejected_transitions;
    result.hard_corridor_capacity_rejected_transitions =
        hard_corridor_capacity_rejected_transitions;
    result.hard_voronoi_capacity_rejected_transitions = hard_voronoi_capacity_rejected_transitions;
    result.hard_capacity_blocking_nets = hard_capacity_blocking_nets;
    result.hard_capacity_blocker_sets = hard_capacity_blocker_sets;
    result.hard_capacity_conflict_sets_withheld = hard_capacity_conflict_sets_withheld;
    return result;
  }
}

template <typename Key>
void erase_net_from_set_map(const NetId& net, std::map<Key, std::set<NetId>>& resources) {
  for (auto resource = resources.begin(); resource != resources.end();) {
    resource->second.erase(net);
    resource = resource->second.empty() ? resources.erase(resource) : std::next(resource);
  }
}

template <typename Key, typename Certificate>
void erase_net_from_clearance_map(const NetId& net,
                                  std::map<Key, std::map<NetId, Certificate>>& resources) {
  for (auto resource = resources.begin(); resource != resources.end();) {
    resource->second.erase(net);
    resource = resource->second.empty() ? resources.erase(resource) : std::next(resource);
  }
}

void remove_net_occupancy(const NetId& net, RoutingOccupancy& occupancy) {
  erase_net_from_set_map(net, occupancy.edges);
  erase_net_from_set_map(net, occupancy.nodes);
  erase_net_from_clearance_map(net, occupancy.clearance_edges);
  erase_net_from_clearance_map(net, occupancy.clearance_nodes);
  erase_net_from_set_map(net, occupancy.corridors);
  erase_net_from_set_map(net, occupancy.corridor_neck_downs);
  erase_net_from_set_map(net, occupancy.corridor_neck_down_edges);
  erase_net_from_set_map(net, occupancy.voronoi_resources);
  erase_net_from_set_map(net, occupancy.direct_medial_neck_downs);
  for (auto resource = occupancy.voronoi_demand_nm_by_net.begin();
       resource != occupancy.voronoi_demand_nm_by_net.end();) {
    resource->second.erase(net);
    resource = resource->second.empty() ? occupancy.voronoi_demand_nm_by_net.erase(resource)
                                        : std::next(resource);
  }
}

[[nodiscard]] std::pair<std::size_t, std::size_t>
clearance_grid_range(std::int64_t minimum, std::int64_t maximum, std::int64_t origin,
                     std::int64_t pitch, std::size_t count) {
  if (count == 0U || pitch <= 0) {
    return {0U, 0U};
  }
  const auto first_value =
      std::floor(static_cast<long double>(minimum - origin) / static_cast<long double>(pitch));
  const auto last_value =
      std::ceil(static_cast<long double>(maximum - origin) / static_cast<long double>(pitch));
  const auto clamp_index = [count](long double value) {
    if (value <= 0.0L) {
      return std::size_t{0};
    }
    if (value >= static_cast<long double>(count - 1U)) {
      return count - 1U;
    }
    return static_cast<std::size_t>(value);
  };
  return {clamp_index(first_value), clamp_index(last_value)};
}

void add_clearance_segment_occupancy(const NetId& net, std::size_t layer, Point start, Point end,
                                     long double occupied_radius_nm,
                                     long double maximum_other_radius_nm,
                                     long double maximum_clearance_nm,
                                     std::optional<Edge> owner_edge,
                                     std::optional<std::size_t> owner_node, const Grid& grid,
                                     RoutingOccupancy& occupancy) {
  if (layer >= grid.layers.size() || grid.width == 0U || grid.height == 0U) {
    return;
  }
  const auto required = occupied_radius_nm + maximum_other_radius_nm + maximum_clearance_nm;
  const auto expansion = static_cast<std::int64_t>(std::ceil(required));
  const auto [first_x, last_x] = clearance_grid_range(
      std::min(start.x.value(), end.x.value()) - expansion,
      std::max(start.x.value(), end.x.value()) + expansion, grid.origin_x, grid.pitch, grid.width);
  const auto [first_y, last_y] = clearance_grid_range(
      std::min(start.y.value(), end.y.value()) - expansion,
      std::max(start.y.value(), end.y.value()) + expansion, grid.origin_y, grid.pitch, grid.height);
  const bool diagonals = grid.diagonal_edges || grid.crossing_safe_diagonal_edges;
  constexpr std::array<std::pair<int, int>, 4U> candidate_directions{
      std::pair{1, 0}, std::pair{0, 1}, std::pair{1, 1}, std::pair{-1, 1}};
  for (auto y = first_y; y <= last_y; ++y) {
    for (auto x = first_x; x <= last_x; ++x) {
      if (!grid.valid(x, y)) {
        continue;
      }
      const auto node = grid.node(x, y, layer);
      const auto point = grid.point(x, y);
      const auto node_distance = geometry::point_segment_distance_nm(point, start, end);
      if (node_distance < required) {
        const auto available = node_distance - occupied_radius_nm;
        auto& certificates = occupancy.clearance_nodes[node];
        const auto [certificate, inserted] = certificates.try_emplace(
            net, ClearanceShadowCertificate{available, owner_edge, owner_node});
        if (!inserted && available < certificate->second.available_nm) {
          certificate->second = ClearanceShadowCertificate{available, owner_edge, owner_node};
        }
      }
      for (std::size_t direction = 0U; direction < candidate_directions.size(); ++direction) {
        if (direction >= 2U && !diagonals) {
          continue;
        }
        const auto next_x_signed =
            static_cast<std::int64_t>(x) + candidate_directions[direction].first;
        const auto next_y_signed =
            static_cast<std::int64_t>(y) + candidate_directions[direction].second;
        if (next_x_signed < 0 || next_y_signed < 0 ||
            next_x_signed >= static_cast<std::int64_t>(grid.width) ||
            next_y_signed >= static_cast<std::int64_t>(grid.height)) {
          continue;
        }
        const auto next_x = static_cast<std::size_t>(next_x_signed);
        const auto next_y = static_cast<std::size_t>(next_y_signed);
        if (!grid.valid(next_x, next_y)) {
          continue;
        }
        const auto next = grid.node(next_x, next_y, layer);
        const auto edge_distance =
            geometry::segment_distance_nm(point, grid.point(next_x, next_y), start, end);
        if (edge_distance < required) {
          const auto available = edge_distance - occupied_radius_nm;
          auto& certificates =
              occupancy.clearance_edges[sampled_capacity_edge(grid, Edge{node, next})];
          const auto [certificate, inserted] = certificates.try_emplace(
              net, ClearanceShadowCertificate{available, owner_edge, owner_node});
          if (!inserted && available < certificate->second.available_nm) {
            certificate->second = ClearanceShadowCertificate{available, owner_edge, owner_node};
          }
        }
      }
    }
  }
}

[[nodiscard]] std::optional<std::size_t> exact_grid_node(const Grid& grid, Point point,
                                                         std::size_t layer) {
  if (layer >= grid.layers.size()) {
    return std::nullopt;
  }
  const auto relative_x = point.x.value() - grid.origin_x;
  const auto relative_y = point.y.value() - grid.origin_y;
  if (relative_x < 0 || relative_y < 0 || relative_x % grid.pitch != 0 ||
      relative_y % grid.pitch != 0) {
    return std::nullopt;
  }
  const auto x = static_cast<std::size_t>(relative_x / grid.pitch);
  const auto y = static_cast<std::size_t>(relative_y / grid.pitch);
  return grid.valid(x, y) ? std::optional{grid.node(x, y, layer)} : std::nullopt;
}

void remember_ripped_path(const GlobalRoutePath& path, const Grid& grid,
                          FormerRouteMemory& memory) {
  if (path.points.size() < 2U) {
    return;
  }
  bool remembered = false;
  for (std::size_t index = 1U; index < path.points.size(); ++index) {
    const auto& first = path.points[index - 1U];
    const auto& second = path.points[index];
    if (first.layer != second.layer) {
      continue;
    }
    const auto layer = std::find(grid.layers.begin(), grid.layers.end(), first.layer);
    if (layer == grid.layers.end()) {
      continue;
    }
    const auto first_node = exact_grid_node(grid, first.position,
                                            static_cast<std::size_t>(layer - grid.layers.begin()));
    const auto second_node = exact_grid_node(grid, second.position,
                                             static_cast<std::size_t>(layer - grid.layers.begin()));
    if (!first_node || !second_node || *first_node == *second_node) {
      continue;
    }
    const auto [first_x, first_y, first_layer] = grid.decode(*first_node);
    const auto [second_x, second_y, second_layer] = grid.decode(*second_node);
    const auto adjacent_or_same = [](std::size_t lhs, std::size_t rhs) {
      return lhs >= rhs ? lhs - rhs <= 1U : rhs - lhs <= 1U;
    };
    if (first_layer != second_layer || !adjacent_or_same(first_x, second_x) ||
        !adjacent_or_same(first_y, second_y)) {
      // Pad stubs and direct medial chords are not sampled graph resources.  Keeping them out
      // of this tiny memory avoids inventing a second occupancy representation.
      continue;
    }
    memory.edges[sampled_capacity_edge(grid, Edge{*first_node, *second_node})].insert(path.net);
    remembered = true;
  }
  if (remembered) {
    memory.ripped_paths = saturating_count_add(memory.ripped_paths, 1U);
  }
}

void add_path_clearance_occupancy(const GlobalRoutePath& path, Nanometres nominal_width,
                                  Nanometres via_diameter,
                                  const RoutingObstacleIndex& obstacle_index, const Grid& grid,
                                  RoutingOccupancy& occupancy) {
  if (path.points.size() < 2U) {
    return;
  }
  const auto maximum_other_radius = obstacle_index.maximum_planned_radius_nm();
  const auto maximum_clearance = obstacle_index.maximum_clearance_nm();
  std::vector<std::optional<std::size_t>> exact_nodes(path.points.size());
  for (std::size_t index = 0U; index < path.points.size(); ++index) {
    const auto found_layer =
        std::find(grid.layers.begin(), grid.layers.end(), path.points[index].layer);
    if (found_layer != grid.layers.end()) {
      exact_nodes[index] =
          exact_grid_node(grid, path.points[index].position,
                          static_cast<std::size_t>(found_layer - grid.layers.begin()));
    }
  }
  auto previous_nodes = exact_nodes;
  for (std::size_t index = 1U; index < path.points.size(); ++index) {
    if (path.points[index - 1U].layer != path.points[index].layer) {
      continue;
    }
    if (!previous_nodes[index]) {
      previous_nodes[index] = previous_nodes[index - 1U];
    }
  }
  auto next_nodes = exact_nodes;
  for (std::size_t index = path.points.size() - 1U; index > 0U; --index) {
    if (path.points[index - 1U].layer != path.points[index].layer) {
      continue;
    }
    if (!next_nodes[index - 1U]) {
      next_nodes[index - 1U] = next_nodes[index];
    }
  }
  for (std::size_t index = 1U; index < path.points.size(); ++index) {
    const auto& first = path.points[index - 1U];
    const auto& second = path.points[index];
    if (first.layer == second.layer) {
      const auto found_layer = std::find(grid.layers.begin(), grid.layers.end(), first.layer);
      if (found_layer == grid.layers.end()) {
        continue;
      }
      const auto selected_width = path.segment_widths.size() + 1U == path.points.size() &&
                                          path.segment_widths[index - 1U].value() > 0
                                      ? path.segment_widths[index - 1U]
                                      : nominal_width;
      std::optional<Edge> owner_edge;
      std::optional<std::size_t> owner_node;
      if (exact_nodes[index - 1U] && exact_nodes[index] &&
          exact_nodes[index - 1U] != exact_nodes[index]) {
        owner_edge =
            sampled_capacity_edge(grid, Edge{*exact_nodes[index - 1U], *exact_nodes[index]});
      } else {
        // A terminal-access polyline is not necessarily on the global lattice. Attribute every
        // part of that stub to its selected portal so history can choose a different escape.
        owner_node = next_nodes[index].has_value() ? next_nodes[index] : previous_nodes[index - 1U];
      }
      add_clearance_segment_occupancy(
          path.net, static_cast<std::size_t>(found_layer - grid.layers.begin()), first.position,
          second.position, static_cast<long double>(selected_width.value()) / 2.0L,
          maximum_other_radius, maximum_clearance, owner_edge, owner_node, grid, occupancy);
      continue;
    }
    // Detailed materialization uses a through via for a global layer transition. Reserve its
    // represented barrel envelope on every routable copper layer.
    for (std::size_t layer = 0U; layer < grid.layers.size(); ++layer) {
      const auto owner_node = exact_grid_node(grid, second.position, layer);
      add_clearance_segment_occupancy(path.net, layer, second.position, second.position,
                                      static_cast<long double>(via_diameter.value()) / 2.0L,
                                      maximum_other_radius, maximum_clearance, std::nullopt,
                                      owner_node, grid, occupancy);
    }
  }
}

void add_path_occupancy(const NetId& net, const std::vector<std::size_t>& nodes, const Grid& grid,
                        const CorridorGraph& corridors, const CorridorGraph& corridor_neck_downs,
                        const std::vector<Nanometres>& segment_widths,
                        const std::vector<GlobalRouteSegmentWidthKind>& segment_width_kinds,
                        const SharedVoronoiAllocationGraph& voronoi_allocation,
                        RoutingOccupancy& occupancy) {
  for (std::size_t index = 0U; index < nodes.size(); ++index) {
    const bool follows_direct =
        index + 1U < nodes.size() &&
        voronoi_allocation.direct_resource(Edge{nodes[index], nodes[index + 1U]}).has_value();
    const bool precedes_direct =
        index > 0U &&
        voronoi_allocation.direct_resource(Edge{nodes[index - 1U], nodes[index]}).has_value();
    if (!follows_direct && !precedes_direct) {
      occupancy.nodes[nodes[index]].insert(net);
    }
  }
  for (std::size_t i = 1; i < nodes.size(); ++i) {
    const Edge edge{nodes[i - 1U], nodes[i]};
    const auto sampled_edge =
        voronoi_allocation.direct_resource(edge) ? edge : sampled_capacity_edge(grid, edge);
    occupancy.edges[sampled_edge].insert(net);
    const bool corridor_neck_down =
        segment_width_kinds.size() + 1U == nodes.size() &&
        segment_width_kinds[i - 1U] == GlobalRouteSegmentWidthKind::corridor_neck_down;
    if (corridor_neck_down) {
      occupancy.corridor_neck_down_edges[edge].insert(net);
      if (const auto corridor = corridor_neck_downs.corridor_for(edge)) {
        occupancy.corridor_neck_downs[*corridor].insert(net);
      }
    } else if (const auto corridor = corridors.corridor_for(edge)) {
      occupancy.corridors[*corridor].insert(net);
    }
    const auto selected_width =
        segment_widths.size() + 1U == nodes.size() ? segment_widths[i - 1U] : Nanometres{0};
    const auto selected_demand = voronoi_allocation.demand_for_width(net, selected_width);
    for (const auto resource : voronoi_allocation.resources_for(edge)) {
      const bool direct_resource = voronoi_allocation.direct_resource(edge) == resource;
      if (!direct_resource &&
          selected_demand > voronoi_allocation.resources[resource].capacity_nm) {
        continue;
      }
      occupancy.voronoi_resources[resource].insert(net);
      auto& demand = occupancy.voronoi_demand_nm_by_net[resource][net];
      const bool direct_medial_neck_down =
          direct_resource && segment_width_kinds.size() + 1U == nodes.size() &&
          segment_width_kinds[i - 1U] == GlobalRouteSegmentWidthKind::medial_neck_down;
      if (direct_medial_neck_down) {
        occupancy.direct_medial_neck_downs[resource].insert(net);
        demand = selected_demand;
      } else if (!occupancy.direct_medial_neck_downs[resource].contains(net)) {
        demand = std::max(demand, selected_demand);
      }
    }
  }
}

[[nodiscard]] GlobalRoutePoint route_point(const Grid& grid, std::size_t node) {
  const auto [x, y, layer] = grid.decode(node);
  return GlobalRoutePoint{grid.point(x, y), grid.layers[layer]};
}

void populate_resource_usage(GlobalRouteResult& result, const Grid& grid,
                             const SharedVoronoiAllocationGraph& voronoi_allocation,
                             const RoutingOccupancy& occupancy, std::size_t sampled_capacity,
                             bool single_lane_direct_resources) {
  result.resource_usage.clear();
  result.congested_resources = 0;
  result.peak_resource_demand = 0;
  result.resource_usage.reserve(occupancy.edges.size() + occupancy.nodes.size());
  const auto append = [&](RoutingResourceKind kind, GlobalRoutePoint first, GlobalRoutePoint second,
                          const std::set<NetId>& nets, std::size_t capacity) {
    RoutingResourceUsage usage{kind,        std::move(first), std::move(second),
                               nets.size(), capacity,         {nets.begin(), nets.end()}};
    result.peak_resource_demand = std::max(result.peak_resource_demand, usage.demand);
    if (usage.congested()) {
      ++result.congested_resources;
    }
    result.resource_usage.push_back(std::move(usage));
  };
  for (const auto& [edge, nets] : occupancy.edges) {
    auto first = route_point(grid, edge.first);
    auto second = route_point(grid, edge.second);
    if (const auto direct = voronoi_allocation.direct_resource(edge)) {
      append(RoutingResourceKind::medial_edge, std::move(first), std::move(second), nets,
             single_lane_direct_resources ? sampled_capacity
                                          : voronoi_allocation.direct_lane_capacity(*direct));
    } else {
      const auto kind = first.layer == second.layer ? RoutingResourceKind::planar_edge
                                                    : RoutingResourceKind::via_edge;
      append(kind, std::move(first), std::move(second), nets, sampled_capacity);
    }
  }
  for (const auto& [node, nets] : occupancy.nodes) {
    auto point = route_point(grid, node);
    append(RoutingResourceKind::node, point, point, nets, sampled_capacity);
  }
}

void populate_medial_path_summary(GlobalRouteResult& result, const Board& board,
                                  const GlobalRouterConfig& config) {
  result.medial_topology_breaks = 0U;
  result.medial_direct_portal_transitions = 0U;
  result.medial_unmapped_nodes = 0U;
  result.medial_interior_unmapped_nodes = 0U;
  result.topologically_complete_medial_paths = 0U;
  result.used_direct_voronoi_edges = 0U;
  result.direct_voronoi_grid_steps_bypassed = 0U;
  result.used_terminal_neck_down_segments = 0U;
  result.total_terminal_neck_down_length_nm = 0U;
  result.used_medial_neck_down_segments = 0U;
  result.total_medial_neck_down_length_nm = 0U;
  result.used_corridor_neck_down_segments = 0U;
  result.total_corridor_neck_down_length_nm = 0U;
  result.total_equivalent_width_reduction_length_nm = 0U;
  result.total_width_reduction_cost = 0U;
  for (const auto& path : result.paths) {
    result.medial_topology_breaks += path.medial_topology_breaks;
    result.medial_direct_portal_transitions += path.medial_direct_portal_transitions;
    result.medial_unmapped_nodes += path.medial_unmapped_nodes;
    result.medial_interior_unmapped_nodes += path.medial_interior_unmapped_nodes;
    if (path.medial_topology_complete()) {
      ++result.topologically_complete_medial_paths;
    }
    result.used_direct_voronoi_edges += path.direct_medial_edges;
    result.direct_voronoi_grid_steps_bypassed += path.direct_medial_grid_steps_bypassed;
    result.used_terminal_neck_down_segments += path.terminal_neck_down_segments;
    result.total_terminal_neck_down_length_nm = saturating_add(
        result.total_terminal_neck_down_length_nm, path.terminal_neck_down_length_nm);
    result.used_medial_neck_down_segments += path.medial_neck_down_segments;
    result.total_medial_neck_down_length_nm =
        saturating_add(result.total_medial_neck_down_length_nm, path.medial_neck_down_length_nm);
    result.used_corridor_neck_down_segments += path.corridor_neck_down_segments;
    result.total_corridor_neck_down_length_nm = saturating_add(
        result.total_corridor_neck_down_length_nm, path.corridor_neck_down_length_nm);
    const auto penalty = path_width_reduction_penalty(
        path, resolve_net_routing_rules(board, path.net).track_width, config);
    result.total_equivalent_width_reduction_length_nm = saturating_add(
        result.total_equivalent_width_reduction_length_nm, penalty.equivalent_length_nm);
    result.total_width_reduction_cost =
        saturating_add(result.total_width_reduction_cost, penalty.cost);
  }
}

void populate_voronoi_resource_usage(GlobalRouteResult& result, const Grid& grid,
                                     const SharedVoronoiAllocationGraph& graph,
                                     const RoutingOccupancy& occupancy,
                                     const RoutingObstacleIndex& obstacles,
                                     const GlobalRouterConfig& config) {
  result.voronoi_resource_usage.clear();
  result.physical_obstacle_impacts.clear();
  result.voronoi_lane_assignments.clear();
  result.voronoi_lane_transitions.clear();
  result.voronoi_resource_usage.reserve(graph.resources.size());
  result.congested_voronoi_resources = 0U;
  result.width_reduced_voronoi_allocations = 0U;
  result.total_voronoi_demand_reduction_nm = 0U;
  result.voronoi_allocation_curved_edge_chords = static_cast<std::size_t>(
      std::count_if(graph.resources.begin(), graph.resources.end(),
                    [](const auto& resource) { return resource.curved_bisector_chord; }));
  result.voronoi_allocation_withheld_curved_edges = graph.withheld_curved_edges;
  result.total_voronoi_overflow_nm = 0U;
  result.peak_voronoi_overflow_nm = 0U;
  result.peak_voronoi_demand_nm = 0U;
  result.coupled_differential_pair_resources = 0U;
  result.coupled_routing_bus_resources = 0U;
  result.packed_voronoi_resources = 0U;
  result.static_clearance_checked_voronoi_lanes = 0U;
  result.accepted_voronoi_lane_transitions = 0U;
  result.unaccepted_selected_voronoi_lane_transitions = 0U;
  result.direct_portal_lane_transitions = 0U;
  result.accepted_direct_portal_lane_transitions = 0U;
  result.unaccepted_direct_portal_lane_transitions = 0U;
  result.unselected_voronoi_lane_transitions = 0U;
  result.reoriented_voronoi_resources = 0U;
  result.voronoi_lane_orientation_states = 0U;
  result.solved_voronoi_lane_orientation_components = 0U;
  result.withheld_voronoi_lane_orientation_components = 0U;
  result.largest_voronoi_lane_orientation_component_resources = 0U;
  result.solved_voronoi_lane_orientation_junctions = 0U;
  result.withheld_voronoi_lane_orientation_junctions = 0U;
  result.mitered_voronoi_lane_transitions = 0U;
  result.withheld_voronoi_lane_transitions = 0U;
  result.withheld_voronoi_transition_clearance_checks = 0U;
  for (const auto& resource : graph.resources) {
    const auto found = occupancy.voronoi_resources.find(resource.id);
    const auto demand_found = occupancy.voronoi_demand_nm_by_net.find(resource.id);
    const std::set<NetId> empty;
    const std::map<NetId, std::uint64_t> empty_demands;
    const auto& nets = found == occupancy.voronoi_resources.end() ? empty : found->second;
    const auto& selected_demands = demand_found == occupancy.voronoi_demand_nm_by_net.end()
                                       ? empty_demands
                                       : demand_found->second;
    const auto demand = graph.occupied_demand(nets, selected_demands);
    for (const auto& net : nets) {
      const auto selected = selected_demands.find(net);
      const auto nominal = graph.net_demand(net);
      if (selected != selected_demands.end() && selected->second < nominal) {
        ++result.width_reduced_voronoi_allocations;
        result.total_voronoi_demand_reduction_nm =
            saturating_add(result.total_voronoi_demand_reduction_nm, nominal - selected->second);
      }
    }
    RoutingVoronoiResourceUsage usage{resource.id,
                                      grid.layers[resource.layer],
                                      resource.start,
                                      resource.end,
                                      Nanometres{static_cast<std::int64_t>(resource.capacity_nm)},
                                      demand,
                                      resource.capacity_nm,
                                      resource.conservative_domain,
                                      {nets.begin(), nets.end()},
                                      resource.curved_bisector_chord};
    result.peak_voronoi_demand_nm = std::max(result.peak_voronoi_demand_nm, demand);
    if (usage.congested()) {
      ++result.congested_voronoi_resources;
      const auto overflow = usage.demand_nm - usage.capacity_nm;
      result.total_voronoi_overflow_nm = saturating_add(result.total_voronoi_overflow_nm, overflow);
      result.peak_voronoi_overflow_nm = std::max(result.peak_voronoi_overflow_nm, overflow);
      for (const auto& obstacle : resource.limiting_obstacles) {
        result.physical_obstacle_impacts.push_back(
            RoutingPhysicalObstacleImpact{resource.id, obstacle, grid.layers[resource.layer],
                                          resource.start, resource.end, overflow, usage.nets});
      }
    }
    if (graph.contains_complete_differential_pair(nets)) {
      ++result.coupled_differential_pair_resources;
    }
    if (graph.contains_complete_routing_bus(nets)) {
      ++result.coupled_routing_bus_resources;
    }
    result.voronoi_resource_usage.push_back(std::move(usage));

    if (nets.empty() || demand > resource.capacity_nm ||
        demand > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) {
      continue;
    }
    const auto delta_x = static_cast<long double>(resource.end.x.value()) -
                         static_cast<long double>(resource.start.x.value());
    const auto delta_y = static_cast<long double>(resource.end.y.value()) -
                         static_cast<long double>(resource.start.y.value());
    const auto length = std::hypotl(delta_x, delta_y);
    if (!(length > 0.0L)) {
      continue;
    }

    struct PackedMember {
      NetId net;
      std::uint64_t width_nm{0};
      long double centre_within_group_nm{0.0L};
      bool differential_pair_member{false};
      std::optional<RoutingBusId> routing_bus;
    };
    struct PackedGroup {
      std::uint64_t span_nm{0};
      std::vector<PackedMember> members;
    };
    std::vector<PackedGroup> groups;
    std::set<NetId> accounted;
    bool complete = true;
    for (const auto& net : nets) {
      if (accounted.contains(net)) {
        continue;
      }
      if (const auto* bus = graph.routing_bus(net);
          bus != nullptr && SharedVoronoiAllocationGraph::contains_complete_bus(*bus, nets)) {
        PackedGroup group;
        group.span_nm = bus->demand_nm;
        auto centre_cursor = static_cast<long double>(bus->boundary_clearance_nm);
        for (std::size_t index = 0; index < bus->ordered_nets.size(); ++index) {
          const auto width = bus->widths_nm[index];
          group.members.push_back(
              PackedMember{bus->ordered_nets[index], width,
                           centre_cursor + static_cast<long double>(width) / 2.0L, false, bus->id});
          centre_cursor += static_cast<long double>(width);
          if (index < bus->gaps_nm.size()) {
            centre_cursor += static_cast<long double>(bus->gaps_nm[index]);
          }
        }
        groups.push_back(std::move(group));
        accounted.insert(bus->ordered_nets.begin(), bus->ordered_nets.end());
        continue;
      }
      if (const auto complement = graph.differential_complement(net);
          complement && nets.contains(*complement)) {
        const auto key =
            net < *complement ? std::pair{net, *complement} : std::pair{*complement, net};
        const auto pair = graph.differential_pair_cross_sections.find(key);
        if (pair == graph.differential_pair_cross_sections.end()) {
          complete = false;
          break;
        }
        const auto& section = pair->second;
        const auto positive_centre = static_cast<long double>(section.boundary_clearance_nm) +
                                     static_cast<long double>(section.positive_width_nm) / 2.0L;
        const auto negative_centre = static_cast<long double>(section.boundary_clearance_nm) +
                                     static_cast<long double>(section.positive_width_nm) +
                                     static_cast<long double>(section.gap_nm) +
                                     static_cast<long double>(section.negative_width_nm) / 2.0L;
        groups.push_back(PackedGroup{
            section.demand_nm,
            {{section.positive_net, section.positive_width_nm, positive_centre, true, std::nullopt},
             {section.negative_net, section.negative_width_nm, negative_centre, true,
              std::nullopt}}});
        accounted.insert(section.positive_net);
        accounted.insert(section.negative_net);
        continue;
      }
      const auto width = graph.track_width_nm_by_net.find(net);
      const auto clearance = graph.clearance_nm_by_net.find(net);
      if (width == graph.track_width_nm_by_net.end() ||
          clearance == graph.clearance_nm_by_net.end()) {
        complete = false;
        break;
      }
      const auto selected = selected_demands.find(net);
      const auto span =
          selected == selected_demands.end() ? graph.net_demand(net) : selected->second;
      const auto clearance_envelope = saturating_multiply(2U, clearance->second);
      if (span < clearance_envelope) {
        complete = false;
        break;
      }
      const auto selected_width = span - clearance_envelope;
      groups.push_back(PackedGroup{span,
                                   {{net, selected_width,
                                     static_cast<long double>(clearance->second) +
                                         static_cast<long double>(selected_width) / 2.0L,
                                     false, std::nullopt}}});
      accounted.insert(net);
    }
    if (!complete || groups.empty()) {
      continue;
    }

    const auto normal_x = -delta_y / length;
    const auto normal_y = delta_x / length;
    auto cursor = -static_cast<long double>(demand) / 2.0L;
    const auto assignment_start = result.voronoi_lane_assignments.size();
    const auto shifted_point = [&](Point point, std::int64_t offset) -> std::optional<Point> {
      const auto x =
          static_cast<long double>(point.x.value()) + normal_x * static_cast<long double>(offset);
      const auto y =
          static_cast<long double>(point.y.value()) + normal_y * static_cast<long double>(offset);
      constexpr auto low = static_cast<long double>(std::numeric_limits<std::int64_t>::min());
      constexpr auto high = static_cast<long double>(std::numeric_limits<std::int64_t>::max());
      if (x < low || x > high || y < low || y > high) {
        return std::nullopt;
      }
      return Point{Nanometres{static_cast<std::int64_t>(std::llround(x))},
                   Nanometres{static_cast<std::int64_t>(std::llround(y))}};
    };
    for (const auto& group : groups) {
      for (const auto& member : group.members) {
        const auto signed_offset =
            static_cast<std::int64_t>(std::llround(cursor + member.centre_within_group_nm));
        const auto shifted_start = shifted_point(resource.start, signed_offset);
        const auto shifted_end = shifted_point(resource.end, signed_offset);
        if (!shifted_start || !shifted_end ||
            member.width_nm >
                static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) {
          complete = false;
          break;
        }
        const auto static_clearance_checked = obstacles.static_segment_free(
            member.net, resource.layer, *shifted_start, *shifted_end,
            static_cast<long double>(member.width_nm) / 2.0L,
            static_cast<long double>(graph.clearance_nm_by_net.at(member.net)));
        result.voronoi_lane_assignments.push_back(RoutingVoronoiLaneAssignment{
            resource.id, member.net, signed_offset,
            Nanometres{static_cast<std::int64_t>(member.width_nm)}, *shifted_start, *shifted_end,
            member.differential_pair_member, member.routing_bus, static_clearance_checked});
        if (static_clearance_checked) {
          ++result.static_clearance_checked_voronoi_lanes;
        }
      }
      if (!complete) {
        break;
      }
      cursor += static_cast<long double>(group.span_nm);
    }
    if (!complete) {
      result.voronoi_lane_assignments.resize(assignment_start);
      continue;
    }
    ++result.packed_voronoi_resources;
  }

  std::map<std::pair<std::size_t, Point>, std::vector<std::size_t>> incident_resources;
  for (const auto& resource : graph.resources) {
    incident_resources[{resource.layer, resource.start}].push_back(resource.id);
    incident_resources[{resource.layer, resource.end}].push_back(resource.id);
  }
  std::map<std::size_t, std::map<NetId, RoutingVoronoiLaneAssignment*>> lanes_by_resource;
  for (auto& lane : result.voronoi_lane_assignments) {
    lanes_by_resource[lane.resource_id].emplace(lane.net, &lane);
  }
  using SelectedTransition = std::tuple<std::size_t, std::size_t, NetId, std::optional<Point>>;
  std::set<SelectedTransition> selected_transitions;
  for (const auto& path : result.paths) {
    for (std::size_t index = 1U; index < path.medial_resource_sequence.size(); ++index) {
      const auto first = path.medial_resource_sequence[index - 1U];
      const auto second = path.medial_resource_sequence[index];
      if (first == second || first >= graph.resources.size() || second >= graph.resources.size() ||
          graph.resources[first].layer != graph.resources[second].layer) {
        continue;
      }
      const auto ordered = std::minmax(first, second);
      const auto portal =
          path.medial_transition_portals.size() + 1U == path.medial_resource_sequence.size()
              ? path.medial_transition_portals[index - 1U]
              : std::nullopt;
      selected_transitions.emplace(ordered.first, ordered.second, path.net, portal);
    }
  }

  const auto junction_uses_resource_start = [&graph, &grid](std::size_t resource_id,
                                                            Point junction) -> std::optional<bool> {
    if (resource_id >= graph.resources.size()) {
      return std::nullopt;
    }
    const auto& resource = graph.resources[resource_id];
    if (resource.start == junction) {
      return true;
    }
    if (resource.end == junction) {
      return false;
    }
    if (!resource.direct_edge) {
      return std::nullopt;
    }
    const auto [first_x, first_y, first_layer] = grid.decode(resource.direct_edge->first);
    const auto [second_x, second_y, second_layer] = grid.decode(resource.direct_edge->second);
    if (first_layer != resource.layer || second_layer != resource.layer) {
      return std::nullopt;
    }
    const auto first_portal = grid.point(first_x, first_y);
    const auto second_portal = grid.point(second_x, second_y);
    if (junction != first_portal && junction != second_portal) {
      return std::nullopt;
    }
    const auto forward = geometry::point_distance_nm(resource.start, first_portal) +
                         geometry::point_distance_nm(resource.end, second_portal);
    const auto reverse = geometry::point_distance_nm(resource.start, second_portal) +
                         geometry::point_distance_nm(resource.end, first_portal);
    const bool first_portal_is_start = forward <= reverse;
    return junction == first_portal ? first_portal_is_start : !first_portal_is_start;
  };

  struct SelectedBridge {
    std::size_t first_resource{0};
    std::size_t second_resource{0};
    NetId net;
    bool direct_portal{false};
  };
  std::map<std::pair<std::size_t, Point>, std::vector<SelectedBridge>> selected_by_junction;
  for (const auto& [first, second, net, portal] : selected_transitions) {
    const auto& first_resource = graph.resources[first];
    const auto& second_resource = graph.resources[second];
    auto junction = portal;
    if (!junction) {
      for (const auto first_endpoint : {first_resource.start, first_resource.end}) {
        if (first_endpoint == second_resource.start || first_endpoint == second_resource.end) {
          junction = first_endpoint;
          break;
        }
      }
    }
    if (junction && lanes_by_resource.contains(first) && lanes_by_resource.contains(second) &&
        lanes_by_resource.at(first).contains(net) && lanes_by_resource.at(second).contains(net)) {
      selected_by_junction[{first_resource.layer, *junction}].push_back(
          SelectedBridge{first, second, net, portal.has_value()});
      if (portal) {
        incident_resources[{first_resource.layer, *junction}].push_back(first);
        incident_resources[{first_resource.layer, *junction}].push_back(second);
      }
    }
  }

  const auto oriented_lane_points =
      [&graph](std::size_t resource_id, const RoutingVoronoiLaneAssignment& lane,
               bool flipped) -> std::optional<std::pair<Point, Point>> {
    const auto& resource = graph.resources[resource_id];
    const auto delta_x = static_cast<long double>(resource.end.x.value()) -
                         static_cast<long double>(resource.start.x.value());
    const auto delta_y = static_cast<long double>(resource.end.y.value()) -
                         static_cast<long double>(resource.start.y.value());
    const auto length = std::hypotl(delta_x, delta_y);
    if (!(length > 0.0L) ||
        (flipped && lane.signed_offset_nm == std::numeric_limits<std::int64_t>::min())) {
      return std::nullopt;
    }
    const auto offset = flipped ? -lane.signed_offset_nm : lane.signed_offset_nm;
    const auto normal_x = -delta_y / length;
    const auto normal_y = delta_x / length;
    const auto shifted = [normal_x, normal_y, offset](Point point) -> std::optional<Point> {
      const auto x =
          static_cast<long double>(point.x.value()) + normal_x * static_cast<long double>(offset);
      const auto y =
          static_cast<long double>(point.y.value()) + normal_y * static_cast<long double>(offset);
      constexpr auto low = static_cast<long double>(std::numeric_limits<std::int64_t>::min());
      constexpr auto high = static_cast<long double>(std::numeric_limits<std::int64_t>::max());
      if (x < low || x > high || y < low || y > high) {
        return std::nullopt;
      }
      return Point{Nanometres{static_cast<std::int64_t>(std::llround(x))},
                   Nanometres{static_cast<std::int64_t>(std::llround(y))}};
    };
    const auto start = shifted(resource.start);
    const auto end = shifted(resource.end);
    return start && end ? std::optional<std::pair<Point, Point>>{{*start, *end}} : std::nullopt;
  };

  const auto transition_miter =
      [&graph](std::size_t first_resource_id, const std::pair<Point, Point>& first_lane_points,
               std::size_t second_resource_id, const std::pair<Point, Point>& second_lane_points,
               Point first_endpoint, Point second_endpoint) -> std::optional<Point> {
    const auto first_dx = static_cast<long double>(first_lane_points.second.x.value()) -
                          static_cast<long double>(first_lane_points.first.x.value());
    const auto first_dy = static_cast<long double>(first_lane_points.second.y.value()) -
                          static_cast<long double>(first_lane_points.first.y.value());
    const auto second_dx = static_cast<long double>(second_lane_points.second.x.value()) -
                           static_cast<long double>(second_lane_points.first.x.value());
    const auto second_dy = static_cast<long double>(second_lane_points.second.y.value()) -
                           static_cast<long double>(second_lane_points.first.y.value());
    const auto denominator = first_dx * second_dy - first_dy * second_dx;
    const auto scale = std::max(std::hypotl(first_dx, first_dy), std::hypotl(second_dx, second_dy));
    if (!(scale > 0.0L) ||
        std::abs(denominator) <= std::numeric_limits<long double>::epsilon() * scale * scale) {
      return std::nullopt;
    }
    const auto relative_x = static_cast<long double>(second_lane_points.first.x.value()) -
                            static_cast<long double>(first_lane_points.first.x.value());
    const auto relative_y = static_cast<long double>(second_lane_points.first.y.value()) -
                            static_cast<long double>(first_lane_points.first.y.value());
    const auto parameter = (relative_x * second_dy - relative_y * second_dx) / denominator;
    const auto x =
        static_cast<long double>(first_lane_points.first.x.value()) + parameter * first_dx;
    const auto y =
        static_cast<long double>(first_lane_points.first.y.value()) + parameter * first_dy;
    constexpr auto low = static_cast<long double>(std::numeric_limits<std::int64_t>::min());
    constexpr auto high = static_cast<long double>(std::numeric_limits<std::int64_t>::max());
    if (!std::isfinite(x) || !std::isfinite(y) || x < low || x > high || y < low || y > high) {
      return std::nullopt;
    }
    const Point miter{Nanometres{static_cast<std::int64_t>(std::llround(x))},
                      Nanometres{static_cast<std::int64_t>(std::llround(y))}};
    if (miter == first_endpoint || miter == second_endpoint) {
      return std::nullopt;
    }
    const auto first_limit = geometry::point_distance_nm(graph.resources[first_resource_id].start,
                                                         graph.resources[first_resource_id].end);
    const auto second_limit = geometry::point_distance_nm(graph.resources[second_resource_id].start,
                                                          graph.resources[second_resource_id].end);
    if (geometry::point_distance_nm(first_endpoint, miter) > first_limit ||
        geometry::point_distance_nm(second_endpoint, miter) > second_limit) {
      return std::nullopt;
    }
    return miter;
  };

  const auto bridge_static_free = [&obstacles](const NetId& net, std::size_t layer, Point start,
                                               Point end, const std::optional<Point>& miter,
                                               long double radius, long double clearance) {
    return miter ? obstacles.static_segment_free(net, layer, start, *miter, radius, clearance) &&
                       obstacles.static_segment_free(net, layer, *miter, end, radius, clearance)
                 : obstacles.static_segment_free(net, layer, start, end, radius, clearance);
  };

  const auto bridge_distance = [](Point first_start, Point first_end,
                                  const std::optional<Point>& first_miter, Point second_start,
                                  Point second_end, const std::optional<Point>& second_miter) {
    const std::array first_segments =
        first_miter ? std::array<std::pair<Point, Point>, 2>{std::pair{first_start, *first_miter},
                                                             std::pair{*first_miter, first_end}}
                    : std::array<std::pair<Point, Point>, 2>{std::pair{first_start, first_end},
                                                             std::pair{first_end, first_end}};
    const std::array second_segments =
        second_miter
            ? std::array<std::pair<Point, Point>, 2>{std::pair{second_start, *second_miter},
                                                     std::pair{*second_miter, second_end}}
            : std::array<std::pair<Point, Point>, 2>{std::pair{second_start, second_end},
                                                     std::pair{second_end, second_end}};
    auto distance = std::numeric_limits<long double>::infinity();
    const auto first_count = first_miter ? 2U : 1U;
    const auto second_count = second_miter ? 2U : 1U;
    for (std::size_t first = 0U; first < first_count; ++first) {
      for (std::size_t second = 0U; second < second_count; ++second) {
        distance =
            std::min(distance, geometry::segment_distance_nm(
                                   first_segments[first].first, first_segments[first].second,
                                   second_segments[second].first, second_segments[second].second));
      }
    }
    return distance;
  };

  std::vector<decltype(selected_by_junction)::value_type*> orientation_junctions;
  orientation_junctions.reserve(selected_by_junction.size());
  for (auto& junction : selected_by_junction) {
    std::set<NetId> nets;
    for (const auto& bridge : junction.second) {
      nets.insert(bridge.net);
    }
    if (nets.size() > 1U) {
      orientation_junctions.push_back(&junction);
    }
  }
  std::sort(orientation_junctions.begin(), orientation_junctions.end(),
            [](const auto* lhs, const auto* rhs) {
              return lhs->second.size() != rhs->second.size()
                         ? lhs->second.size() > rhs->second.size()
                         : lhs->first < rhs->first;
            });

  std::map<std::size_t, bool> resource_orientation;
  std::size_t orientation_states = 0U;
  std::vector<std::set<std::size_t>> orientation_resources(orientation_junctions.size());
  std::map<std::size_t, std::vector<std::size_t>> orientation_junctions_by_resource;
  std::vector<bool> orientation_junction_eligible(orientation_junctions.size(), true);
  for (std::size_t junction_index = 0U; junction_index < orientation_junctions.size();
       ++junction_index) {
    const auto* selected_junction = orientation_junctions[junction_index];
    std::set<std::size_t> resource_set;
    for (const auto& bridge : selected_junction->second) {
      resource_set.insert(bridge.first_resource);
      resource_set.insert(bridge.second_resource);
    }
    orientation_resources[junction_index] = resource_set;
    if (resource_set.size() > config.max_voronoi_lane_orientation_resources_per_junction ||
        resource_set.size() >= std::numeric_limits<std::size_t>::digits) {
      orientation_junction_eligible[junction_index] = false;
      ++result.withheld_voronoi_lane_orientation_junctions;
      continue;
    }
    for (const auto resource : resource_set) {
      orientation_junctions_by_resource[resource].push_back(junction_index);
    }
  }

  const auto junction_valid = [&](const auto* selected_junction, const auto& orientation) {
    const auto layer = selected_junction->first.first;
    const auto junction = selected_junction->first.second;
    const auto& bridges = selected_junction->second;
    struct ProposedBridge {
      NetId net;
      Point start;
      Point end;
      Nanometres width;
      std::optional<Point> miter;
    };
    std::vector<ProposedBridge> proposals;
    proposals.reserve(bridges.size());
    for (const auto& bridge : bridges) {
      const auto* first_lane = lanes_by_resource.at(bridge.first_resource).at(bridge.net);
      const auto* second_lane = lanes_by_resource.at(bridge.second_resource).at(bridge.net);
      const auto first_points = oriented_lane_points(bridge.first_resource, *first_lane,
                                                     orientation(bridge.first_resource));
      const auto second_points = oriented_lane_points(bridge.second_resource, *second_lane,
                                                      orientation(bridge.second_resource));
      if (!first_points || !second_points) {
        return false;
      }
      const auto first_uses_start = junction_uses_resource_start(bridge.first_resource, junction);
      const auto second_uses_start = junction_uses_resource_start(bridge.second_resource, junction);
      if (!first_uses_start || !second_uses_start) {
        return false;
      }
      const auto first_endpoint = *first_uses_start ? first_points->first : first_points->second;
      const auto second_endpoint =
          *second_uses_start ? second_points->first : second_points->second;
      const auto first_far_endpoint =
          *first_uses_start ? first_points->second : first_points->first;
      const auto second_far_endpoint =
          *second_uses_start ? second_points->second : second_points->first;
      auto miter = transition_miter(bridge.first_resource, *first_points, bridge.second_resource,
                                    *second_points, first_endpoint, second_endpoint);
      if (bridge.direct_portal && miter &&
          geometry::point_distance_nm(*miter, junction) > static_cast<long double>(grid.pitch)) {
        miter.reset();
      }
      if (!bridge_static_free(bridge.net, layer, first_endpoint, second_endpoint, miter,
                              static_cast<long double>(first_lane->track_width.value()) / 2.0L,
                              static_cast<long double>(graph.clearance_nm_by_net.at(bridge.net)))) {
        return false;
      }
      proposals.push_back(ProposedBridge{bridge.net, first_far_endpoint, second_far_endpoint,
                                         first_lane->track_width, miter});
    }
    for (std::size_t first = 0U; first < proposals.size(); ++first) {
      for (std::size_t second = first + 1U; second < proposals.size(); ++second) {
        if (proposals[first].net == proposals[second].net) {
          continue;
        }
        const auto required = static_cast<long double>(proposals[first].width.value()) / 2.0L +
                              static_cast<long double>(graph.required_lane_gap(
                                  proposals[first].net, proposals[second].net)) +
                              static_cast<long double>(proposals[second].width.value()) / 2.0L;
        if (bridge_distance(proposals[first].start, proposals[first].end, proposals[first].miter,
                            proposals[second].start, proposals[second].end,
                            proposals[second].miter) < required) {
          return false;
        }
      }
    }
    return true;
  };

  std::vector<bool> orientation_junction_visited(orientation_junctions.size(), false);
  for (std::size_t seed = 0U; seed < orientation_junctions.size(); ++seed) {
    if (!orientation_junction_eligible[seed] || orientation_junction_visited[seed]) {
      continue;
    }
    std::vector<std::size_t> component;
    std::set<std::size_t> component_resources;
    std::deque<std::size_t> pending{seed};
    orientation_junction_visited[seed] = true;
    while (!pending.empty()) {
      const auto junction_index = pending.front();
      pending.pop_front();
      component.push_back(junction_index);
      for (const auto resource : orientation_resources[junction_index]) {
        component_resources.insert(resource);
        for (const auto adjacent : orientation_junctions_by_resource.at(resource)) {
          if (orientation_junction_eligible[adjacent] && !orientation_junction_visited[adjacent]) {
            orientation_junction_visited[adjacent] = true;
            pending.push_back(adjacent);
          }
        }
      }
    }
    std::sort(component.begin(), component.end());
    result.largest_voronoi_lane_orientation_component_resources = std::max(
        result.largest_voronoi_lane_orientation_component_resources, component_resources.size());
    if (component_resources.size() >= std::numeric_limits<std::size_t>::digits) {
      ++result.withheld_voronoi_lane_orientation_components;
      result.withheld_voronoi_lane_orientation_junctions = saturating_count_add(
          result.withheld_voronoi_lane_orientation_junctions, component.size());
      continue;
    }
    const std::vector<std::size_t> component_resource_vector(component_resources.begin(),
                                                             component_resources.end());
    const auto states = std::size_t{1} << component_resource_vector.size();
    if (states > config.max_voronoi_lane_orientation_states -
                     std::min(orientation_states, config.max_voronoi_lane_orientation_states)) {
      ++result.withheld_voronoi_lane_orientation_components;
      result.withheld_voronoi_lane_orientation_junctions = saturating_count_add(
          result.withheld_voronoi_lane_orientation_junctions, component.size());
      continue;
    }
    orientation_states += states;
    bool solved = false;
    for (std::size_t state = 0U; state < states && !solved; ++state) {
      const auto orientation = [&](std::size_t resource) {
        const auto found = std::lower_bound(component_resource_vector.begin(),
                                            component_resource_vector.end(), resource);
        const auto bit = static_cast<std::size_t>(found - component_resource_vector.begin());
        return ((state >> bit) & 1U) != 0U;
      };
      bool valid = true;
      for (const auto resource : component_resource_vector) {
        for (const auto& [net, lane] : lanes_by_resource.at(resource)) {
          const auto points = oriented_lane_points(resource, *lane, orientation(resource));
          if (!points || !obstacles.static_segment_free(
                             net, graph.resources[resource].layer, points->first, points->second,
                             static_cast<long double>(lane->track_width.value()) / 2.0L,
                             static_cast<long double>(graph.clearance_nm_by_net.at(net)))) {
            valid = false;
            break;
          }
        }
        if (!valid) {
          break;
        }
      }
      for (const auto junction_index : component) {
        if (valid && !junction_valid(orientation_junctions[junction_index], orientation)) {
          valid = false;
        }
      }
      if (valid) {
        for (std::size_t bit = 0U; bit < component_resource_vector.size(); ++bit) {
          resource_orientation.emplace(component_resource_vector[bit], ((state >> bit) & 1U) != 0U);
        }
        solved = true;
        ++result.solved_voronoi_lane_orientation_components;
        result.solved_voronoi_lane_orientation_junctions = saturating_count_add(
            result.solved_voronoi_lane_orientation_junctions, component.size());
      }
    }
    if (!solved) {
      ++result.withheld_voronoi_lane_orientation_components;
      result.withheld_voronoi_lane_orientation_junctions = saturating_count_add(
          result.withheld_voronoi_lane_orientation_junctions, component.size());
    }
  }
  result.voronoi_lane_orientation_states = orientation_states;

  for (auto& lane : result.voronoi_lane_assignments) {
    if (!resource_orientation.contains(lane.resource_id) ||
        !resource_orientation.at(lane.resource_id)) {
      continue;
    }
    const auto points = oriented_lane_points(lane.resource_id, lane, true);
    if (!points) {
      continue;
    }
    lane.signed_offset_nm = -lane.signed_offset_nm;
    lane.shifted_start = points->first;
    lane.shifted_end = points->second;
    lane.static_clearance_checked = obstacles.static_segment_free(
        lane.net, graph.resources[lane.resource_id].layer, lane.shifted_start, lane.shifted_end,
        static_cast<long double>(lane.track_width.value()) / 2.0L,
        static_cast<long double>(graph.clearance_nm_by_net.at(lane.net)));
  }
  result.reoriented_voronoi_resources = static_cast<std::size_t>(
      std::count_if(resource_orientation.begin(), resource_orientation.end(),
                    [](const auto& orientation) { return orientation.second; }));
  result.static_clearance_checked_voronoi_lanes = static_cast<std::size_t>(
      std::count_if(result.voronoi_lane_assignments.begin(), result.voronoi_lane_assignments.end(),
                    [](const auto& lane) { return lane.static_clearance_checked; }));

  std::size_t clearance_checks = 0U;
  bool transition_budget_exhausted = false;
  for (auto& [junction_key, resource_ids] : incident_resources) {
    std::sort(resource_ids.begin(), resource_ids.end());
    resource_ids.erase(std::unique(resource_ids.begin(), resource_ids.end()), resource_ids.end());
    if (resource_ids.size() < 2U) {
      continue;
    }
    const auto& junction = junction_key.second;
    const auto junction_transition_begin = result.voronoi_lane_transitions.size();
    for (std::size_t first_index = 0U; first_index + 1U < resource_ids.size(); ++first_index) {
      const auto first_resource_id = resource_ids[first_index];
      const auto first_lanes = lanes_by_resource.find(first_resource_id);
      if (first_lanes == lanes_by_resource.end()) {
        continue;
      }
      for (std::size_t second_index = first_index + 1U; second_index < resource_ids.size();
           ++second_index) {
        const auto second_resource_id = resource_ids[second_index];
        const auto second_lanes = lanes_by_resource.find(second_resource_id);
        if (second_lanes == lanes_by_resource.end()) {
          continue;
        }
        for (const auto& [net, first_lane] : first_lanes->second) {
          const auto second_lane = second_lanes->second.find(net);
          if (second_lane == second_lanes->second.end()) {
            continue;
          }
          const auto direct_portal_selected = selected_transitions.contains(SelectedTransition{
              first_resource_id, second_resource_id, net, std::optional<Point>{junction}});
          const auto exact_junction_selected = selected_transitions.contains(
              SelectedTransition{first_resource_id, second_resource_id, net, std::nullopt});
          if (!direct_portal_selected && !exact_junction_selected) {
            ++result.unselected_voronoi_lane_transitions;
            continue;
          }
          if (result.voronoi_lane_transitions.size() >= config.max_voronoi_lane_transitions) {
            result.withheld_voronoi_lane_transitions = 1U;
            transition_budget_exhausted = true;
            break;
          }
          const auto first_uses_start = junction_uses_resource_start(first_resource_id, junction);
          const auto second_uses_start = junction_uses_resource_start(second_resource_id, junction);
          if (!first_uses_start || !second_uses_start) {
            ++result.withheld_voronoi_lane_transitions;
            continue;
          }
          const auto first_endpoint =
              *first_uses_start ? first_lane->shifted_start : first_lane->shifted_end;
          const auto second_endpoint = *second_uses_start ? second_lane->second->shifted_start
                                                          : second_lane->second->shifted_end;
          const auto first_points = oriented_lane_points(first_resource_id, *first_lane, false);
          const auto second_points =
              oriented_lane_points(second_resource_id, *second_lane->second, false);
          auto miter = first_points && second_points
                           ? transition_miter(first_resource_id, *first_points, second_resource_id,
                                              *second_points, first_endpoint, second_endpoint)
                           : std::nullopt;
          if (direct_portal_selected && miter &&
              geometry::point_distance_nm(*miter, junction) >
                  static_cast<long double>(grid.pitch)) {
            miter.reset();
          }

          std::size_t required_checks = 0U;
          for (std::size_t previous = junction_transition_begin;
               previous < result.voronoi_lane_transitions.size(); ++previous) {
            if (result.voronoi_lane_transitions[previous].net != net) {
              ++required_checks;
            }
          }
          if (required_checks >
              config.max_voronoi_transition_clearance_checks -
                  std::min(clearance_checks, config.max_voronoi_transition_clearance_checks)) {
            result.withheld_voronoi_transition_clearance_checks = 1U;
            result.withheld_voronoi_lane_transitions = 1U;
            transition_budget_exhausted = true;
            break;
          }

          const auto width = first_lane->track_width;
          const auto static_clearance_checked =
              bridge_static_free(net, junction_key.first, first_endpoint, second_endpoint, miter,
                                 static_cast<long double>(width.value()) / 2.0L,
                                 static_cast<long double>(graph.clearance_nm_by_net.at(net)));
          RoutingVoronoiLaneTransition transition{first_resource_id,
                                                  second_resource_id,
                                                  net,
                                                  grid.layers[junction_key.first],
                                                  junction,
                                                  first_endpoint,
                                                  second_endpoint,
                                                  width,
                                                  first_lane->differential_pair_member,
                                                  first_lane->routing_bus,
                                                  static_clearance_checked,
                                                  true,
                                                  miter,
                                                  direct_portal_selected};
          if (direct_portal_selected) {
            ++result.direct_portal_lane_transitions;
          }
          if (miter) {
            ++result.mitered_voronoi_lane_transitions;
          }
          for (std::size_t previous = junction_transition_begin;
               previous < result.voronoi_lane_transitions.size(); ++previous) {
            auto& other = result.voronoi_lane_transitions[previous];
            if (other.net == transition.net) {
              continue;
            }
            ++clearance_checks;
            const auto gap = graph.required_lane_gap(other.net, transition.net);
            const auto required_distance =
                static_cast<long double>(other.track_width.value()) / 2.0L +
                static_cast<long double>(gap) +
                static_cast<long double>(transition.track_width.value()) / 2.0L;
            const auto* other_first_lane =
                lanes_by_resource.at(other.first_resource_id).at(other.net);
            const auto* other_second_lane =
                lanes_by_resource.at(other.second_resource_id).at(other.net);
            const auto other_first_far =
                other.first_lane_endpoint == other_first_lane->shifted_start
                    ? other_first_lane->shifted_end
                    : other_first_lane->shifted_start;
            const auto other_second_far =
                other.second_lane_endpoint == other_second_lane->shifted_start
                    ? other_second_lane->shifted_end
                    : other_second_lane->shifted_start;
            const auto transition_first_far =
                transition.first_lane_endpoint == first_lane->shifted_start
                    ? first_lane->shifted_end
                    : first_lane->shifted_start;
            const auto transition_second_far =
                transition.second_lane_endpoint == second_lane->second->shifted_start
                    ? second_lane->second->shifted_end
                    : second_lane->second->shifted_start;
            const auto actual_distance =
                bridge_distance(other_first_far, other_second_far, other.miter,
                                transition_first_far, transition_second_far, transition.miter);
            if (actual_distance < required_distance) {
              other.mutual_clearance_checked = false;
              transition.mutual_clearance_checked = false;
            }
          }
          result.voronoi_lane_transitions.push_back(std::move(transition));
        }
        if (transition_budget_exhausted) {
          break;
        }
      }
      if (transition_budget_exhausted) {
        break;
      }
    }
    if (transition_budget_exhausted) {
      break;
    }
  }
  result.accepted_voronoi_lane_transitions = static_cast<std::size_t>(std::count_if(
      result.voronoi_lane_transitions.begin(), result.voronoi_lane_transitions.end(),
      [](const auto& transition) {
        return transition.static_clearance_checked && transition.mutual_clearance_checked;
      }));
  result.unaccepted_selected_voronoi_lane_transitions = static_cast<std::size_t>(std::count_if(
      result.voronoi_lane_transitions.begin(), result.voronoi_lane_transitions.end(),
      [](const auto& transition) {
        return !transition.static_clearance_checked || !transition.mutual_clearance_checked;
      }));
  result.accepted_direct_portal_lane_transitions = static_cast<std::size_t>(std::count_if(
      result.voronoi_lane_transitions.begin(), result.voronoi_lane_transitions.end(),
      [](const auto& transition) {
        return transition.direct_portal_bridge && transition.static_clearance_checked &&
               transition.mutual_clearance_checked;
      }));
  result.unaccepted_direct_portal_lane_transitions = static_cast<std::size_t>(std::count_if(
      result.voronoi_lane_transitions.begin(), result.voronoi_lane_transitions.end(),
      [](const auto& transition) {
        return transition.direct_portal_bridge &&
               (!transition.static_clearance_checked || !transition.mutual_clearance_checked);
      }));
}

void populate_corridor_usage(GlobalRouteResult& result, const Grid& grid,
                             const CorridorGraph& graph, const RoutingOccupancy& occupancy,
                             bool explicit_neck_down = false) {
  auto& output = explicit_neck_down ? result.corridor_neck_down_usage : result.corridor_usage;
  output.clear();
  output.reserve(graph.corridors.size());
  auto& congested =
      explicit_neck_down ? result.congested_corridor_neck_downs : result.congested_corridors;
  auto& peak =
      explicit_neck_down ? result.peak_corridor_neck_down_demand : result.peak_corridor_demand;
  congested = 0U;
  peak = 0U;
  if (!explicit_neck_down) {
    result.neck_down_corridors = 0U;
  }
  const auto& selected_occupancy =
      explicit_neck_down ? occupancy.corridor_neck_downs : occupancy.corridors;
  for (const auto& corridor : graph.corridors) {
    const auto found = selected_occupancy.find(corridor.id);
    const std::set<NetId> empty;
    const auto& nets = found == selected_occupancy.end() ? empty : found->second;
    const auto midpoint = [](Nanometres first, Nanometres last) {
      return Nanometres{first.value() + (last.value() - first.value()) / 2};
    };
    const auto medial_start =
        corridor.orientation == RoutingCorridorOrientation::horizontal
            ? Point{corridor.first_lane_start.x,
                    midpoint(corridor.first_lane_start.y, corridor.last_lane_end.y)}
            : Point{midpoint(corridor.first_lane_start.x, corridor.last_lane_end.x),
                    corridor.first_lane_start.y};
    const auto medial_end = corridor.orientation == RoutingCorridorOrientation::horizontal
                                ? Point{corridor.last_lane_end.x, medial_start.y}
                                : Point{medial_start.x, corridor.last_lane_end.y};
    RoutingCorridorUsage usage{
        corridor.id,
        grid.layers[corridor.layer],
        corridor.orientation,
        corridor.first_lane_start,
        corridor.last_lane_end,
        medial_start,
        medial_end,
        nets.size(),
        corridor.capacity,
        Nanometres{static_cast<std::int64_t>(
            corridor.capacity > 1U ? (corridor.capacity - 1U) * static_cast<std::size_t>(grid.pitch)
                                   : 0U)},
        corridor.neck_down,
        {nets.begin(), nets.end()}};
    peak = std::max(peak, usage.demand);
    if (usage.congested()) {
      ++congested;
    }
    if (!explicit_neck_down && usage.neck_down) {
      ++result.neck_down_corridors;
    }
    output.push_back(std::move(usage));
  }
}

[[nodiscard]] GlobalRoutePath convert_path(const Grid& grid, const NetTask& task,
                                           std::size_t source_terminal_index,
                                           const std::vector<std::size_t>& connected_terminals,
                                           const SearchResult& search,
                                           const SharedVoronoiAllocationGraph& graph) {
  const auto& nodes = search.nodes;
  GlobalRoutePath path;
  path.net = task.net;
  path.connection_index = source_terminal_index - 1U;
  if (nodes.empty()) {
    return path;
  }
  path.points.reserve(nodes.size() + 2U);
  path.segment_widths.reserve(nodes.size() + 1U);
  path.segment_width_kinds.reserve(nodes.size() + 1U);
  const auto append_point = [&path](GlobalRoutePoint point, Nanometres segment_width,
                                    GlobalRouteSegmentWidthKind width_kind) {
    if (!path.points.empty() && path.points.back() == point) {
      return;
    }
    if (!path.points.empty()) {
      path.segment_widths.push_back(segment_width);
      path.segment_width_kinds.push_back(width_kind);
    }
    path.points.push_back(std::move(point));
  };
  const auto [first_x, first_y, first_layer] = grid.decode(nodes.front());
  static_cast<void>(first_x);
  static_cast<void>(first_y);
  const auto& source_terminal = task.terminals[source_terminal_index];
  bool source_access_joins_tree = false;
  if (const auto access = source_terminal.access_paths.find(nodes.front());
      access != source_terminal.access_paths.end() && access->second.points.size() > 1U) {
    source_access_joins_tree =
        std::any_of(access->second.joined_terminals.begin(), access->second.joined_terminals.end(),
                    [&connected_terminals](const auto terminal) {
                      return std::find(connected_terminals.begin(), connected_terminals.end(),
                                       terminal) != connected_terminals.end();
                    });
    const auto width = access->second.neck_down ? access->second.width : Nanometres{0};
    for (std::size_t i = 0; i + 1U < access->second.points.size(); ++i) {
      append_point(GlobalRoutePoint{access->second.points[i], grid.layers[first_layer]}, width,
                   access->second.neck_down ? GlobalRouteSegmentWidthKind::terminal_neck_down
                                            : GlobalRouteSegmentWidthKind::nominal);
    }
  }
  for (std::size_t node_index = 0U; node_index < nodes.size(); ++node_index) {
    const auto node = nodes[node_index];
    const auto [x, y, layer] = grid.decode(node);
    auto width = Nanometres{0};
    auto width_kind = GlobalRouteSegmentWidthKind::nominal;
    if (node == nodes.front()) {
      if (const auto access = source_terminal.access_paths.find(node);
          access != source_terminal.access_paths.end() && access->second.neck_down) {
        width = access->second.width;
        width_kind = GlobalRouteSegmentWidthKind::terminal_neck_down;
      }
    } else if (search.segment_widths.size() + 1U == nodes.size() &&
               search.segment_width_kinds.size() == search.segment_widths.size()) {
      width = search.segment_widths[node_index - 1U];
      width_kind = search.segment_width_kinds[node_index - 1U];
    }
    append_point(GlobalRoutePoint{grid.point(x, y), grid.layers[layer]}, width, width_kind);
  }
  constexpr auto no_resource = std::numeric_limits<std::size_t>::max();
  std::optional<std::size_t> previous_resource;
  if (!graph.resource_by_node.empty()) {
    std::vector<std::size_t> labels(nodes.size(), no_resource);
    for (std::size_t index = 0U; index < nodes.size(); ++index) {
      if (index == 0U && nodes.size() > 1U) {
        if (const auto direct = graph.direct_resource(Edge{nodes[0], nodes[1]})) {
          labels[index] = *direct;
          continue;
        }
      }
      if (index > 0U) {
        if (const auto direct = graph.direct_resource(Edge{nodes[index - 1U], nodes[index]})) {
          labels[index] = *direct;
          ++path.direct_medial_edges;
          const auto [first_x, first_y, first_layer] = grid.decode(nodes[index - 1U]);
          const auto [second_x, second_y, second_layer] = grid.decode(nodes[index]);
          static_cast<void>(first_layer);
          static_cast<void>(second_layer);
          const auto grid_steps = (first_x > second_x ? first_x - second_x : second_x - first_x) +
                                  (first_y > second_y ? first_y - second_y : second_y - first_y);
          path.direct_medial_grid_steps_bypassed += grid_steps > 1U ? grid_steps - 1U : 0U;
          continue;
        }
      }
      if (nodes[index] < graph.resource_by_node.size()) {
        labels[index] = graph.resource_by_node[nodes[index]];
      }
    }
    const auto first_mapped =
        std::find_if(labels.begin(), labels.end(),
                     [no_resource](const auto label) { return label != no_resource; });
    const auto last_mapped =
        std::find_if(labels.rbegin(), labels.rend(),
                     [no_resource](const auto label) { return label != no_resource; });
    if (first_mapped == labels.end()) {
      path.medial_interior_unmapped_nodes = labels.size();
    }
    const auto append_medial_resource = [&](std::size_t resource, std::size_t portal_node,
                                            std::optional<Edge> selected_grid_edge) {
      if (previous_resource && *previous_resource == resource) {
        return;
      }
      std::optional<Point> transition_portal;
      if (previous_resource &&
          graph.resources[*previous_resource].layer == graph.resources[resource].layer &&
          !graph.topologically_adjacent(*previous_resource, resource)) {
        if (graph.resources_join_at_direct_portal(*previous_resource, resource, portal_node)) {
          const auto [portal_x, portal_y, portal_layer] = grid.decode(portal_node);
          static_cast<void>(portal_layer);
          transition_portal = grid.point(portal_x, portal_y);
          ++path.medial_direct_portal_transitions;
        } else {
          ++path.medial_topology_breaks;
        }
      }
      if (!path.medial_resource_sequence.empty()) {
        path.medial_transition_portals.push_back(transition_portal);
        std::optional<GlobalRouteMedialTransitionEdge> transition_grid_edge;
        if (selected_grid_edge) {
          transition_grid_edge =
              GlobalRouteMedialTransitionEdge{route_point(grid, selected_grid_edge->first),
                                              route_point(grid, selected_grid_edge->second)};
        }
        path.medial_transition_grid_edges.push_back(transition_grid_edge);
      }
      path.medial_resource_sequence.push_back(resource);
      previous_resource = resource;
    };
    for (std::size_t index = 0U; index < labels.size(); ++index) {
      const auto resource = labels[index];
      if (resource == no_resource) {
        ++path.medial_unmapped_nodes;
        previous_resource.reset();
        if (first_mapped != labels.end()) {
          const auto first_index = static_cast<std::size_t>(first_mapped - labels.begin());
          const auto last_index =
              labels.size() - 1U - static_cast<std::size_t>(last_mapped - labels.rbegin());
          if (index > first_index && index < last_index) {
            ++path.medial_interior_unmapped_nodes;
          }
        }
        continue;
      }
      if (previous_resource && *previous_resource == resource) {
        continue;
      }
      const auto portal_node = index > 0U ? nodes[index - 1U] : nodes[index];
      const auto selected_grid_edge =
          index > 0U ? std::optional<Edge>{Edge{nodes[index - 1U], nodes[index]}} : std::nullopt;
      if (previous_resource && selected_grid_edge && !graph.direct_resource(*selected_grid_edge)) {
        const auto endpoint_resource = graph.resource_by_node[nodes[index - 1U]];
        if (endpoint_resource != no_resource && endpoint_resource != *previous_resource &&
            endpoint_resource != resource) {
          append_medial_resource(endpoint_resource, portal_node, selected_grid_edge);
        }
      }
      append_medial_resource(resource, portal_node, selected_grid_edge);
    }
  }
  const auto [last_x, last_y, last_layer] = grid.decode(nodes.back());
  static_cast<void>(last_x);
  static_cast<void>(last_y);
  if (connected_terminals.size() == 1U && !source_access_joins_tree) {
    for (const auto terminal_index : connected_terminals) {
      const auto& terminal = task.terminals[terminal_index];
      if (const auto access = terminal.access_paths.find(nodes.back());
          access != terminal.access_paths.end()) {
        const auto width = access->second.neck_down ? access->second.width : Nanometres{0};
        for (std::size_t i = access->second.points.size(); i > 1U; --i) {
          append_point(GlobalRoutePoint{access->second.points[i - 2U], grid.layers[last_layer]},
                       width,
                       access->second.neck_down ? GlobalRouteSegmentWidthKind::terminal_neck_down
                                                : GlobalRouteSegmentWidthKind::nominal);
        }
        break;
      }
    }
  }
  for (std::size_t index = 0U; index < path.segment_widths.size(); ++index) {
    if (path.segment_widths[index].value() <= 0) {
      continue;
    }
    const auto length =
        geometry::point_distance_nm(path.points[index].position, path.points[index + 1U].position);
    if (path.segment_width_kinds[index] == GlobalRouteSegmentWidthKind::terminal_neck_down) {
      ++path.terminal_neck_down_segments;
      path.terminal_neck_down_length_nm = saturating_add(
          path.terminal_neck_down_length_nm, static_cast<std::uint64_t>(std::ceil(length)));
    } else if (path.segment_width_kinds[index] == GlobalRouteSegmentWidthKind::medial_neck_down) {
      ++path.medial_neck_down_segments;
      path.medial_neck_down_length_nm = saturating_add(
          path.medial_neck_down_length_nm, static_cast<std::uint64_t>(std::ceil(length)));
    } else if (path.segment_width_kinds[index] == GlobalRouteSegmentWidthKind::corridor_neck_down) {
      ++path.corridor_neck_down_segments;
      path.corridor_neck_down_length_nm = saturating_add(
          path.corridor_neck_down_length_nm, static_cast<std::uint64_t>(std::ceil(length)));
    }
  }
  return path;
}

void account_routing_occupancy(GlobalRouteResult& result, const Board& board, const Grid& grid,
                               const CorridorGraph& corridors,
                               const CorridorGraph& corridor_neck_downs,
                               const SharedVoronoiAllocationGraph& allocation,
                               const RoutingOccupancy& occupancy, RoutingObstacleIndex& obstacles,
                               const GlobalRouterConfig& config, RoutingHistory* history,
                               bool charge_shadow_owner_history = false) {
  result.clearance_shadow_conflicts.clear();
  result.clearance_shadow_overflow = 0U;
  result.clearance_shadow_history_increment = 0U;
  result.peak_clearance_shadow_history_increment = 0U;
  result.clearance_shadow_conflict_budget_exhausted = false;
  std::map<NetId, ResolvedNetRoutingRules> resolved_rules;
  resolved_rules.emplace(NetId{}, resolve_net_routing_rules(board, NetId{}));
  for (const auto& net : board.nets) {
    resolved_rules.emplace(net.id, resolve_net_routing_rules(board, net.id));
  }
  const auto rules_for = [&](const NetId& net) -> const ResolvedNetRoutingRules& {
    const auto found = resolved_rules.find(net);
    return found == resolved_rules.end() ? resolved_rules.at(NetId{}) : found->second;
  };
  if (config.reserve_sampled_route_clearance) {
    std::set<std::pair<NetId, std::size_t>> via_nodes;
    for (const auto& path : result.paths) {
      for (std::size_t index = 1U; index < path.points.size(); ++index) {
        if (path.points[index - 1U].layer == path.points[index].layer) {
          continue;
        }
        const auto position = path.points[index].position;
        const auto relative_x = position.x.value() - grid.origin_x;
        const auto relative_y = position.y.value() - grid.origin_y;
        if (relative_x < 0 || relative_y < 0 || relative_x % grid.pitch != 0 ||
            relative_y % grid.pitch != 0) {
          continue;
        }
        const auto x = static_cast<std::size_t>(relative_x / grid.pitch);
        const auto y = static_cast<std::size_t>(relative_y / grid.pitch);
        if (!grid.valid(x, y)) {
          continue;
        }
        for (std::size_t layer = 0U; layer < grid.layers.size(); ++layer) {
          via_nodes.emplace(path.net, grid.node(x, y, layer));
        }
      }
    }

    using ConflictKey =
        std::tuple<RoutingClearanceShadowResourceKind, std::size_t, std::size_t, NetId, NetId>;
    std::set<ConflictKey> unique;
    // A single selected owner edge can cast its shadow over several neighbouring resources. The
    // ordinary phase retains the stable candidate-side update. A bounded fallback phase can also
    // make the owner actionable, but coalesces that history to the strongest update for each
    // owner resource/net pair so rasterization density does not determine its penalty.
    std::map<RoutingHistory::ClearanceEdgeKey, std::size_t> shadow_edge_history_increments;
    std::map<RoutingHistory::ClearanceNodeKey, std::size_t> shadow_node_history_increments;
    const auto append_conflict = [&](RoutingClearanceShadowResourceKind kind,
                                     std::size_t first_node, std::size_t second_node,
                                     const NetId& candidate_net, const NetId& shadow_net,
                                     const ClearanceShadowCertificate& certificate) {
      if (candidate_net == shadow_net) {
        return;
      }
      const auto& candidate_rules = rules_for(candidate_net);
      const auto& shadow_rules = rules_for(shadow_net);
      const auto candidate_radius =
          kind == RoutingClearanceShadowResourceKind::terminal_or_via_node &&
                  via_nodes.contains({candidate_net, first_node})
              ? static_cast<long double>(candidate_rules.via_diameter.value()) / 2.0L
              : static_cast<long double>(candidate_rules.track_width.value()) / 2.0L;
      const auto required =
          candidate_radius +
          static_cast<long double>(
              std::max(candidate_rules.clearance, shadow_rules.clearance).value());
      if (!(certificate.available_nm < required)) {
        return;
      }
      const auto first_net = std::min(candidate_net, shadow_net);
      const auto second_net = std::max(candidate_net, shadow_net);
      const ConflictKey key{kind, std::min(first_node, second_node),
                            std::max(first_node, second_node), first_net, second_net};
      if (!unique.insert(key).second) {
        return;
      }
      if (result.clearance_shadow_conflicts.size() >=
          config.max_clearance_shadow_conflict_resources) {
        result.clearance_shadow_conflict_budget_exhausted = true;
        return;
      }
      const auto [first_x, first_y, first_layer] = grid.decode(first_node);
      const auto [second_x, second_y, second_layer] = grid.decode(second_node);
      if (first_layer >= grid.layers.size() || second_layer >= grid.layers.size()) {
        result.clearance_shadow_conflict_budget_exhausted = true;
        return;
      }
      std::optional<GlobalRoutePoint> shadow_owner_first;
      std::optional<GlobalRoutePoint> shadow_owner_second;
      if (certificate.owner_edge) {
        shadow_owner_first = route_point(grid, certificate.owner_edge->first);
        shadow_owner_second = route_point(grid, certificate.owner_edge->second);
      } else if (certificate.owner_node) {
        shadow_owner_first = route_point(grid, *certificate.owner_node);
        shadow_owner_second = shadow_owner_first;
      }
      const auto deficit = required - certificate.available_nm;
      const auto quantum = config.clearance_shadow_cost_quantum.value();
      const auto history_increment =
          quantum > 0 ? static_cast<std::size_t>(
                            std::max(1.0L, std::ceil(deficit / static_cast<long double>(quantum))))
                      : 1U;
      result.clearance_shadow_conflicts.push_back(RoutingClearanceShadowConflict{
          kind, GlobalRoutePoint{grid.point(first_x, first_y), grid.layers[first_layer]},
          GlobalRoutePoint{grid.point(second_x, second_y), grid.layers[second_layer]}, first_net,
          second_net, Nanometres{static_cast<std::int64_t>(std::ceil(deficit))}, shadow_owner_first,
          shadow_owner_second});
      result.total_overflow = saturating_count_add(result.total_overflow, 1U);
      result.clearance_shadow_overflow = saturating_count_add(result.clearance_shadow_overflow, 1U);
      result.clearance_shadow_history_increment =
          saturating_count_add(result.clearance_shadow_history_increment, history_increment);
      result.peak_clearance_shadow_history_increment =
          std::max(result.peak_clearance_shadow_history_increment, history_increment);
      if (history != nullptr) {
        if (kind == RoutingClearanceShadowResourceKind::planar_edge) {
          const Edge edge{first_node, second_node};
          history->clearance_edges[{candidate_net, edge}] = saturating_count_add(
              history->clearance_edges[{candidate_net, edge}], history_increment);
        } else {
          history->clearance_nodes[{candidate_net, first_node}] = saturating_count_add(
              history->clearance_nodes[{candidate_net, first_node}], history_increment);
        }
        if (charge_shadow_owner_history && certificate.owner_edge) {
          auto& shadow_increment =
              shadow_edge_history_increments[{shadow_net, *certificate.owner_edge}];
          shadow_increment = std::max(shadow_increment, history_increment);
        } else if (charge_shadow_owner_history && certificate.owner_node) {
          auto& shadow_increment =
              shadow_node_history_increments[{shadow_net, *certificate.owner_node}];
          shadow_increment = std::max(shadow_increment, history_increment);
        }
      }
    };
    for (const auto& [edge, occupants] : occupancy.edges) {
      const auto shadows = occupancy.clearance_edges.find(edge);
      if (shadows == occupancy.clearance_edges.end()) {
        continue;
      }
      for (const auto& candidate_net : occupants) {
        for (const auto& [shadow_net, certificate] : shadows->second) {
          append_conflict(RoutingClearanceShadowResourceKind::planar_edge, edge.first, edge.second,
                          candidate_net, shadow_net, certificate);
        }
      }
    }
    for (const auto& [node, occupants] : occupancy.nodes) {
      const auto shadows = occupancy.clearance_nodes.find(node);
      if (shadows == occupancy.clearance_nodes.end()) {
        continue;
      }
      for (const auto& candidate_net : occupants) {
        for (const auto& [shadow_net, certificate] : shadows->second) {
          append_conflict(RoutingClearanceShadowResourceKind::terminal_or_via_node, node, node,
                          candidate_net, shadow_net, certificate);
        }
      }
    }
    if (history != nullptr) {
      for (const auto& [key, increment] : shadow_edge_history_increments) {
        auto& shadow_history = history->clearance_edges[key];
        shadow_history = saturating_count_add(shadow_history, increment);
      }
      for (const auto& [key, increment] : shadow_node_history_increments) {
        auto& shadow_history = history->clearance_nodes[key];
        shadow_history = saturating_count_add(shadow_history, increment);
      }
    }
  }
  for (const auto& [edge, nets] : occupancy.edges) {
    if ((allocation.direct_resource(edge) && !config.single_lane_direct_voronoi_resources) ||
        nets.size() <= config.edge_capacity) {
      continue;
    }
    const auto overflow = nets.size() - config.edge_capacity;
    result.total_overflow = saturating_count_add(result.total_overflow, overflow);
    result.sampled_overflow = saturating_count_add(result.sampled_overflow, overflow);
    if (history != nullptr) {
      history->edges[edge] = saturating_count_add(history->edges[edge], overflow);
    }
  }
  for (const auto& [node, nets] : occupancy.nodes) {
    if (nets.size() <= config.edge_capacity) {
      continue;
    }
    const auto overflow = nets.size() - config.edge_capacity;
    result.total_overflow = saturating_count_add(result.total_overflow, overflow);
    result.sampled_overflow = saturating_count_add(result.sampled_overflow, overflow);
    if (history != nullptr) {
      history->nodes[node] = saturating_count_add(history->nodes[node], overflow);
    }
  }
  const auto account_corridors = [&](const auto& selected, const CorridorGraph& graph,
                                     auto member) {
    for (const auto& [id, nets] : selected) {
      const auto capacity = graph.corridors[id].capacity;
      if (nets.size() <= capacity) {
        continue;
      }
      const auto overflow = nets.size() - capacity;
      result.total_overflow = saturating_count_add(result.total_overflow, overflow);
      result.corridor_overflow = saturating_count_add(result.corridor_overflow, overflow);
      if (history != nullptr) {
        auto& selected_history = history->*member;
        selected_history[id] = saturating_count_add(selected_history[id], overflow);
      }
    }
  };
  account_corridors(occupancy.corridors, corridors, &RoutingHistory::corridors);
  account_corridors(occupancy.corridor_neck_downs, corridor_neck_downs,
                    &RoutingHistory::corridor_neck_downs);
  for (const auto& [resource_id, nets] : occupancy.voronoi_resources) {
    const auto selected = occupancy.voronoi_demand_nm_by_net.find(resource_id);
    const std::map<NetId, std::uint64_t> empty;
    const auto& demands =
        selected == occupancy.voronoi_demand_nm_by_net.end() ? empty : selected->second;
    const auto demand = allocation.occupied_demand(nets, demands);
    const auto capacity = allocation.resources[resource_id].capacity_nm;
    if (demand <= capacity) {
      continue;
    }
    const auto increment = voronoi_history_increment(
        demand - capacity, allocation.minimum_demand_nm, config.scale_voronoi_history_by_overflow);
    result.total_overflow = saturating_count_add(result.total_overflow, 1U);
    result.total_voronoi_history_increment =
        saturating_count_add(result.total_voronoi_history_increment, increment);
    result.peak_voronoi_history_increment =
        std::max(result.peak_voronoi_history_increment, increment);
    if (history != nullptr) {
      history->voronoi_resources[resource_id] =
          saturating_count_add(history->voronoi_resources[resource_id], increment);
    }
  }
  populate_resource_usage(result, grid, allocation, occupancy, config.edge_capacity,
                          config.single_lane_direct_voronoi_resources);
  populate_corridor_usage(result, grid, corridors, occupancy);
  populate_corridor_usage(result, grid, corridor_neck_downs, occupancy, true);
  populate_voronoi_resource_usage(result, grid, allocation, occupancy, obstacles, config);
  populate_medial_path_summary(result, board, config);

  std::map<NetId, NetId> parent;
  const auto register_nets = [&](const auto& usage) {
    if (!usage.congested()) {
      return;
    }
    for (const auto& net : usage.nets) {
      parent.try_emplace(net, net);
    }
  };
  for (const auto& usage : result.resource_usage) {
    register_nets(usage);
  }
  for (const auto& usage : result.corridor_usage) {
    register_nets(usage);
  }
  for (const auto& usage : result.corridor_neck_down_usage) {
    register_nets(usage);
  }
  for (const auto& usage : result.voronoi_resource_usage) {
    register_nets(usage);
  }
  for (const auto& conflict : result.clearance_shadow_conflicts) {
    parent.try_emplace(conflict.first_net, conflict.first_net);
    parent.try_emplace(conflict.second_net, conflict.second_net);
  }
  const auto find_root = [&](NetId net) {
    auto root = net;
    while (parent.at(root) != root) {
      root = parent.at(root);
    }
    return root;
  };
  const auto unite_usage = [&](const auto& usage) {
    if (!usage.congested() || usage.nets.empty()) {
      return;
    }
    auto root = find_root(usage.nets.front());
    for (std::size_t index = 1U; index < usage.nets.size(); ++index) {
      auto other = find_root(usage.nets[index]);
      if (other == root) {
        continue;
      }
      const auto retained = std::min(root, other);
      const auto removed = std::max(root, other);
      parent[removed] = retained;
      root = retained;
    }
  };
  for (const auto& usage : result.resource_usage) {
    unite_usage(usage);
  }
  for (const auto& usage : result.corridor_usage) {
    unite_usage(usage);
  }
  for (const auto& usage : result.corridor_neck_down_usage) {
    unite_usage(usage);
  }
  for (const auto& usage : result.voronoi_resource_usage) {
    unite_usage(usage);
  }
  for (const auto& conflict : result.clearance_shadow_conflicts) {
    auto first = find_root(conflict.first_net);
    auto second = find_root(conflict.second_net);
    if (first != second) {
      const auto retained = std::min(first, second);
      const auto removed = std::max(first, second);
      parent[removed] = retained;
    }
  }
  std::map<NetId, RoutingResidualConflictGroup> groups;
  for (const auto& [net, unused] : parent) {
    static_cast<void>(unused);
    auto& group = groups[find_root(net)];
    group.nets.push_back(net);
  }
  const auto group_for_usage = [&](const auto& usage) -> RoutingResidualConflictGroup* {
    if (!usage.congested() || usage.nets.empty()) {
      return nullptr;
    }
    return &groups.at(find_root(usage.nets.front()));
  };
  for (std::size_t index = 0U; index < result.resource_usage.size(); ++index) {
    const auto& usage = result.resource_usage[index];
    if (auto* group = group_for_usage(usage)) {
      group->sampled_resource_indices.push_back(index);
      group->sampled_overflow =
          saturating_count_add(group->sampled_overflow, usage.demand - usage.capacity);
    }
  }
  const auto add_corridors = [&](const auto& usages) {
    for (const auto& usage : usages) {
      if (auto* group = group_for_usage(usage)) {
        group->corridor_resource_ids.push_back(usage.id);
        group->corridor_overflow =
            saturating_count_add(group->corridor_overflow, usage.demand - usage.capacity);
      }
    }
  };
  add_corridors(result.corridor_usage);
  add_corridors(result.corridor_neck_down_usage);
  for (const auto& usage : result.voronoi_resource_usage) {
    if (auto* group = group_for_usage(usage)) {
      group->physical_resource_ids.push_back(usage.id);
      group->physical_overflow_nm =
          saturating_add(group->physical_overflow_nm, usage.demand_nm - usage.capacity_nm);
    }
  }
  for (std::size_t index = 0U; index < result.clearance_shadow_conflicts.size(); ++index) {
    const auto& conflict = result.clearance_shadow_conflicts[index];
    auto& group = groups.at(find_root(conflict.first_net));
    group.clearance_shadow_resource_indices.push_back(index);
    group.clearance_shadow_overflow = saturating_count_add(group.clearance_shadow_overflow, 1U);
  }
  result.residual_conflict_groups.clear();
  result.residual_conflict_groups.reserve(groups.size());
  for (auto& [root, group] : groups) {
    static_cast<void>(root);
    std::sort(group.nets.begin(), group.nets.end());
    const std::set<NetId> group_nets(group.nets.begin(), group.nets.end());
    for (const auto& path : result.paths) {
      if (!group_nets.contains(path.net)) {
        continue;
      }
      group.path_count = saturating_count_add(group.path_count, 1U);
      if (std::adjacent_find(path.points.begin(), path.points.end(),
                             [](const auto& first, const auto& second) {
                               return first.layer != second.layer;
                             }) != path.points.end()) {
        group.multilayer_path_count = saturating_count_add(group.multilayer_path_count, 1U);
      }
    }
    group.all_nets_have_preserved_geometric_corridors =
        std::all_of(group.nets.begin(), group.nets.end(), [&](const auto& net) {
          bool found = false;
          for (const auto& path : result.paths) {
            if (path.net != net) {
              continue;
            }
            for (const auto& corridor : path.geometric_corridors) {
              if (corridor.selected_path_preserves_topology) {
                found = true;
                group.peak_geometric_lane_capacity =
                    std::max(group.peak_geometric_lane_capacity, corridor.lane_capacity);
              }
            }
          }
          return found;
        });
    group.id = result.residual_conflict_groups.size();
    result.residual_conflict_groups.push_back(std::move(group));
  }
}

struct PathfinderGroupMembership {
  std::string group;
  std::size_t member_order{0};
};

[[nodiscard]] std::map<NetId, PathfinderGroupMembership>
pathfinder_group_membership(const Board& board) {
  std::map<NetId, PathfinderGroupMembership> membership;
  for (const auto& bus : board.routing_buses) {
    for (std::size_t index = 0; index < bus.ordered_nets.size(); ++index) {
      membership.emplace(bus.ordered_nets[index],
                         PathfinderGroupMembership{"bus:" + bus.id.value(), index});
    }
  }
  for (const auto& pair : board.differential_pairs) {
    membership.emplace(pair.positive_net, PathfinderGroupMembership{"pair:" + pair.id.value(), 0U});
    membership.emplace(pair.negative_net, PathfinderGroupMembership{"pair:" + pair.id.value(), 1U});
  }
  return membership;
}

struct StagedIterationZeroOrder {
  std::vector<NetTask> tasks;
  std::vector<RoutingSeedPriorityEvidence> evidence;
};

[[nodiscard]] StagedIterationZeroOrder
staged_iteration_zero_order(const Board& board, const Grid& grid, const std::vector<NetTask>& tasks,
                            const StaticSeedPlan& seed_plan, const CorridorGraph& corridors,
                            const SharedVoronoiAllocationGraph& allocation) {
  const auto membership = pathfinder_group_membership(board);
  struct OrderedTask {
    NetTask task;
    std::size_t member_order{0};
  };
  struct DerivedGroup {
    RoutingSeedPriorityEvidence evidence;
    std::vector<OrderedTask> tasks;
    std::set<std::size_t> primary_nodes;
    std::set<Edge> primary_edges;
    std::set<Point> primary_via_sites;
    std::size_t connection_count{0};
    std::size_t constraint_signals{0};
    std::size_t flexibility_signals{0};
  };
  std::map<std::string, DerivedGroup> groups;
  for (const auto& task : tasks) {
    const auto found = membership.find(task.net);
    const auto key = found == membership.end() ? "net:" + task.net.value() : found->second.group;
    const auto member_order = found == membership.end() ? 0U : found->second.member_order;
    auto [group, inserted] = groups.try_emplace(key);
    if (inserted) {
      group->second.evidence.group = key;
      group->second.evidence.minimum_terminal_portals = std::numeric_limits<std::size_t>::max();
    }
    auto& value = group->second;
    value.evidence.nets.push_back(task.net);
    value.evidence.explicit_priority = std::max(value.evidence.explicit_priority, task.priority);
    value.connection_count = saturating_count_add(
        value.connection_count, task.terminals.empty() ? 0U : task.terminals.size() - 1U);
    for (const auto& terminal : task.terminals) {
      value.evidence.minimum_terminal_portals =
          std::min(value.evidence.minimum_terminal_portals, terminal.grid_nodes.size());
    }
    value.tasks.push_back(OrderedTask{task, member_order});
  }

  // A terminal MST is a deliberately cheap pre-route topology proxy.  It is not a route and it
  // must never establish occupancy, capacity, or DRC legality.  Its only purpose is to expose the
  // same kind of source/target-order crossing pressure used by MPS-style PCB routers before the
  // first PathFinder allocation.  Count one conflict per pair of coupled routing groups so a
  // differential pair or a multi-terminal net cannot inflate the result merely by having several
  // adjacent airwire edges.
  struct TerminalAirwire {
    std::string group;
    Point first;
    Point second;
  };
  std::vector<TerminalAirwire> terminal_airwires;
  for (const auto& [group_key, group] : groups) {
    for (const auto& ordered : group.tasks) {
      const auto& terminals = ordered.task.terminals;
      if (terminals.size() < 2U) {
        continue;
      }
      std::vector<bool> in_tree(terminals.size(), false);
      in_tree.front() = true;
      for (std::size_t selected_count = 1U; selected_count < terminals.size(); ++selected_count) {
        std::optional<std::tuple<long double, std::size_t, std::size_t>> best;
        for (std::size_t first = 0U; first < terminals.size(); ++first) {
          if (!in_tree[first]) {
            continue;
          }
          for (std::size_t second = 0U; second < terminals.size(); ++second) {
            if (in_tree[second]) {
              continue;
            }
            const auto dx = static_cast<long double>(terminals[first].exact_position.x.value()) -
                            static_cast<long double>(terminals[second].exact_position.x.value());
            const auto dy = static_cast<long double>(terminals[first].exact_position.y.value()) -
                            static_cast<long double>(terminals[second].exact_position.y.value());
            const auto candidate = std::tuple{dx * dx + dy * dy, first, second};
            if (!best || candidate < *best) {
              best = candidate;
            }
          }
        }
        if (!best) {
          break;
        }
        const auto first = std::get<1U>(*best);
        const auto second = std::get<2U>(*best);
        in_tree[second] = true;
        if (terminals[first].exact_position != terminals[second].exact_position) {
          terminal_airwires.push_back(TerminalAirwire{group_key, terminals[first].exact_position,
                                                      terminals[second].exact_position});
        }
      }
    }
  }
  std::set<std::pair<std::string, std::string>> terminal_crossing_pairs;
  for (std::size_t first = 0U; first < terminal_airwires.size(); ++first) {
    for (std::size_t second = first + 1U; second < terminal_airwires.size(); ++second) {
      const auto& lhs = terminal_airwires[first];
      const auto& rhs = terminal_airwires[second];
      if (lhs.group == rhs.group || lhs.first == rhs.first || lhs.first == rhs.second ||
          lhs.second == rhs.first || lhs.second == rhs.second ||
          !geometry::segments_intersect(lhs.first, lhs.second, rhs.first, rhs.second)) {
        continue;
      }
      const auto key = std::minmax(lhs.group, rhs.group);
      if (!terminal_crossing_pairs.emplace(key.first, key.second).second) {
        continue;
      }
      ++groups.at(lhs.group).evidence.terminal_crossing_pressure;
      ++groups.at(rhs.group).evidence.terminal_crossing_pressure;
    }
  }

  const auto route_layer_sequence = [&](const GlobalSeedCandidate& candidate) {
    std::vector<std::size_t> sequence;
    for (const auto node : candidate.route.nodes) {
      const auto [x, y, layer] = grid.decode(node);
      static_cast<void>(x);
      static_cast<void>(y);
      if (sequence.empty() || sequence.back() != layer) {
        sequence.push_back(layer);
      }
    }
    return sequence;
  };
  using ViaTransition = std::tuple<Point, std::size_t, std::size_t>;
  const auto route_via_signature = [&](const GlobalSeedCandidate& candidate) {
    std::vector<ViaTransition> signature;
    for (std::size_t index = 1U; index < candidate.route.nodes.size(); ++index) {
      const auto [first_x, first_y, first_layer] = grid.decode(candidate.route.nodes[index - 1U]);
      const auto [second_x, second_y, second_layer] = grid.decode(candidate.route.nodes[index]);
      if (first_x == second_x && first_y == second_y && first_layer != second_layer) {
        signature.emplace_back(grid.point(first_x, first_y), std::min(first_layer, second_layer),
                               std::max(first_layer, second_layer));
      }
    }
    return signature;
  };
  using TopologyElement = std::pair<std::uint8_t, std::size_t>;
  const auto route_topology_signature = [&](const GlobalSeedCandidate& candidate) {
    std::vector<TopologyElement> signature;
    for (std::size_t index = 1U; index < candidate.route.nodes.size(); ++index) {
      const Edge edge{candidate.route.nodes[index - 1U], candidate.route.nodes[index]};
      const auto resources = allocation.resources_for(edge);
      if (!resources.empty()) {
        for (const auto resource : resources) {
          const TopologyElement element{0U, resource};
          if (signature.empty() || signature.back() != element) {
            signature.push_back(element);
          }
        }
      } else if (const auto corridor = corridors.corridor_for(edge)) {
        const TopologyElement element{1U, *corridor};
        if (signature.empty() || signature.back() != element) {
          signature.push_back(element);
        }
      } else {
        const auto [x, y, layer] = grid.decode(edge.first);
        static_cast<void>(x);
        static_cast<void>(y);
        const TopologyElement element{2U, layer};
        if (signature.empty() || signature.back() != element) {
          signature.push_back(element);
        }
      }
    }
    return signature;
  };

  constexpr TopologyElement topology_separator{3U, 0U};
  constexpr std::size_t layer_separator = std::numeric_limits<std::size_t>::max();
  for (auto& [key, group] : groups) {
    static_cast<void>(key);
    std::sort(group.evidence.nets.begin(), group.evidence.nets.end());
    std::sort(group.tasks.begin(), group.tasks.end(), [](const auto& lhs, const auto& rhs) {
      return std::tie(lhs.member_order, lhs.task.net) < std::tie(rhs.member_order, rhs.task.net);
    });
    if (group.evidence.minimum_terminal_portals == std::numeric_limits<std::size_t>::max()) {
      group.evidence.minimum_terminal_portals = 0U;
    }
    std::set<std::size_t> candidate_indices;
    for (const auto& candidate : seed_plan.candidates) {
      if (std::binary_search(group.evidence.nets.begin(), group.evidence.nets.end(),
                             candidate.net)) {
        candidate_indices.insert(candidate.tree_candidate_index);
      }
    }
    std::set<std::vector<TopologyElement>> topologies;
    std::set<std::vector<std::size_t>> layers;
    std::set<std::vector<std::vector<ViaTransition>>> via_assignments;
    std::set<Point> viable_via_sites;
    std::optional<std::size_t> primary_candidate;
    for (const auto candidate_index : candidate_indices) {
      bool complete = true;
      bool corridor_available = true;
      std::vector<TopologyElement> group_topology;
      std::vector<std::size_t> group_layers;
      std::vector<std::vector<ViaTransition>> group_vias;
      std::set<Point> group_via_sites;
      for (const auto& ordered : group.tasks) {
        const auto& task = ordered.task;
        for (std::size_t connection = 0U; connection + 1U < task.terminals.size(); ++connection) {
          const auto candidate = std::find_if(
              seed_plan.candidates.begin(), seed_plan.candidates.end(), [&](const auto& value) {
                return value.net == task.net && value.tree_candidate_index == candidate_index &&
                       value.connection_index == connection;
              });
          if (candidate == seed_plan.candidates.end()) {
            complete = false;
            break;
          }
          const auto topology = route_topology_signature(*candidate);
          group_topology.insert(group_topology.end(), topology.begin(), topology.end());
          group_topology.push_back(topology_separator);
          const auto layer_sequence = route_layer_sequence(*candidate);
          group_layers.insert(group_layers.end(), layer_sequence.begin(), layer_sequence.end());
          group_layers.push_back(layer_separator);
          auto via_signature = route_via_signature(*candidate);
          for (const auto& [site, first_layer, second_layer] : via_signature) {
            static_cast<void>(first_layer);
            static_cast<void>(second_layer);
            group_via_sites.insert(site);
          }
          group_vias.push_back(std::move(via_signature));
          corridor_available =
              corridor_available && !candidate->geometric_corridors.empty() && !topology.empty();
        }
        if (!complete) {
          break;
        }
      }
      if (!complete) {
        continue;
      }
      ++group.evidence.viable_tree_candidates;
      group.evidence.viable_corridor_candidates = saturating_count_add(
          group.evidence.viable_corridor_candidates, corridor_available ? 1U : 0U);
      topologies.insert(std::move(group_topology));
      layers.insert(std::move(group_layers));
      via_assignments.insert(std::move(group_vias));
      viable_via_sites.insert(group_via_sites.begin(), group_via_sites.end());
      if (!primary_candidate) {
        primary_candidate = candidate_index;
      }
    }
    group.evidence.topology_alternatives = topologies.size();
    group.evidence.layer_alternatives = layers.size();
    group.evidence.viable_via_sites = viable_via_sites.size();
    group.evidence.via_assignment_alternatives = via_assignments.size();
    if (primary_candidate) {
      for (const auto& candidate : seed_plan.candidates) {
        if (candidate.tree_candidate_index != *primary_candidate ||
            !std::binary_search(group.evidence.nets.begin(), group.evidence.nets.end(),
                                candidate.net)) {
          continue;
        }
        group.evidence.primary_route_steps = saturating_count_add(
            group.evidence.primary_route_steps,
            candidate.route.nodes.empty() ? 0U : candidate.route.nodes.size() - 1U);
        group.primary_nodes.insert(candidate.route.nodes.begin(), candidate.route.nodes.end());
        for (std::size_t edge = 1U; edge < candidate.route.nodes.size(); ++edge) {
          group.primary_edges.insert(sampled_capacity_edge(
              grid, Edge{candidate.route.nodes[edge - 1U], candidate.route.nodes[edge]}));
        }
        const auto via_signature = route_via_signature(candidate);
        group.evidence.primary_via_transitions =
            saturating_count_add(group.evidence.primary_via_transitions, via_signature.size());
        for (const auto& [site, first_layer, second_layer] : via_signature) {
          static_cast<void>(first_layer);
          static_cast<void>(second_layer);
          group.primary_via_sites.insert(site);
        }
      }
    }
    std::set<NetId> nets(group.evidence.nets.begin(), group.evidence.nets.end());
    group.evidence.physical_envelope_nm = allocation.occupied_demand(nets, {});
    if (group.evidence.physical_envelope_nm == 0U) {
      for (const auto& ordered : group.tasks) {
        const auto& task = ordered.task;
        const auto track_envelope = static_cast<std::uint64_t>(std::max<std::int64_t>(
            0, task.nominal_track_width.value() + 2 * task.physical_clearance.value()));
        const auto via_envelope = static_cast<std::uint64_t>(std::max<std::int64_t>(
            0, task.via_diameter.value() + 2 * task.physical_clearance.value()));
        group.evidence.physical_envelope_nm = saturating_add(
            group.evidence.physical_envelope_nm, std::max(track_envelope, via_envelope));
      }
    }
    if (group.evidence.viable_tree_candidates > 0U) {
      auto freedom = static_cast<std::uint64_t>(
          std::max<std::size_t>(1U, group.evidence.minimum_terminal_portals));
      for (const auto alternatives :
           {group.evidence.viable_corridor_candidates, group.evidence.topology_alternatives,
            group.evidence.layer_alternatives,
            group.evidence.primary_via_transitions == 0U
                ? std::size_t{1U}
                : std::max<std::size_t>(1U, group.evidence.viable_via_sites)}) {
        freedom = saturating_multiply(
            freedom, static_cast<std::uint64_t>(std::max<std::size_t>(1U, alternatives)));
      }
      group.evidence.routing_freedom_score = freedom;
    }
  }

  std::map<std::size_t, std::size_t> node_use;
  std::map<Edge, std::size_t> edge_use;
  std::map<Point, std::size_t> via_site_use;
  for (const auto& [key, group] : groups) {
    static_cast<void>(key);
    for (const auto node : group.primary_nodes) {
      ++node_use[node];
    }
    for (const auto& edge : group.primary_edges) {
      ++edge_use[edge];
    }
    for (const auto& site : group.primary_via_sites) {
      ++via_site_use[site];
    }
  }
  for (auto& [key, group] : groups) {
    static_cast<void>(key);
    for (const auto node : group.primary_nodes) {
      const auto use = node_use.at(node);
      group.evidence.estimated_interference =
          saturating_count_add(group.evidence.estimated_interference, use > 0U ? use - 1U : 0U);
      group.evidence.touches_scarce_resource = group.evidence.touches_scarce_resource || use > 1U;
    }
    for (const auto& edge : group.primary_edges) {
      const auto use = edge_use.at(edge);
      group.evidence.estimated_interference =
          saturating_count_add(group.evidence.estimated_interference, use > 0U ? use - 1U : 0U);
      group.evidence.touches_scarce_resource = group.evidence.touches_scarce_resource || use > 1U;
    }
    for (const auto& site : group.primary_via_sites) {
      const auto use = via_site_use.at(site);
      group.evidence.via_site_contention =
          saturating_count_add(group.evidence.via_site_contention, use > 0U ? use - 1U : 0U);
      group.evidence.touches_scarce_resource = group.evidence.touches_scarce_resource || use > 1U;
    }
  }

  const auto percentile = [](auto values, std::size_t numerator, std::size_t denominator) {
    using Value = typename decltype(values)::value_type;
    if (values.empty()) {
      return Value{};
    }
    std::sort(values.begin(), values.end());
    return values[((values.size() - 1U) * numerator) / denominator];
  };
  std::vector<std::size_t> portals;
  std::vector<std::size_t> interference;
  std::vector<std::size_t> terminal_crossings;
  std::vector<std::size_t> steps;
  std::vector<std::uint64_t> envelopes;
  for (const auto& [key, group] : groups) {
    static_cast<void>(key);
    portals.push_back(group.evidence.minimum_terminal_portals);
    interference.push_back(group.evidence.estimated_interference);
    terminal_crossings.push_back(group.evidence.terminal_crossing_pressure);
    steps.push_back(group.evidence.primary_route_steps);
    envelopes.push_back(group.evidence.physical_envelope_nm);
  }
  const auto portal_low = percentile(portals, 1U, 4U);
  const auto portal_median = percentile(portals, 1U, 2U);
  const auto interference_median = percentile(interference, 1U, 2U);
  const auto interference_high = percentile(interference, 3U, 4U);
  const auto terminal_crossing_median = percentile(terminal_crossings, 1U, 2U);
  const auto terminal_crossing_high = percentile(terminal_crossings, 3U, 4U);
  const auto step_low = percentile(steps, 1U, 4U);
  const auto step_median = percentile(steps, 1U, 2U);
  const auto envelope_median = percentile(envelopes, 1U, 2U);
  const auto envelope_high = percentile(envelopes, 3U, 4U);

  std::vector<DerivedGroup> ordered_groups;
  ordered_groups.reserve(groups.size());
  for (auto& [key, group] : groups) {
    static_cast<void>(key);
    const auto all_two_terminal =
        group.connection_count == group.tasks.size() && !group.tasks.empty();
    const auto via_scarce =
        group.evidence.primary_via_transitions > 0U &&
        (group.evidence.viable_via_sites <= group.evidence.primary_via_transitions ||
         group.evidence.via_assignment_alternatives <= 1U ||
         group.evidence.via_site_contention > 0U);
    const auto trivial = group.evidence.viable_tree_candidates > 0U && all_two_terminal &&
                         group.evidence.primary_route_steps <= step_low &&
                         group.evidence.minimum_terminal_portals >= portal_median &&
                         group.evidence.physical_envelope_nm <= envelope_median &&
                         !group.evidence.touches_scarce_resource && !via_scarce &&
                         group.evidence.terminal_crossing_pressure == 0U;
    // Portal, corridor, topology, and layer counts are correlated views of one access portfolio.
    // Count access scarcity only when the board distribution distinguishes it, and collapse the
    // topology views into one physical-alternative signal so a ubiquitous one-portal first ring
    // cannot label almost every net as constrained.
    const auto access_scarce =
        portal_low < portal_median && group.evidence.minimum_terminal_portals <= portal_low;
    const auto no_complete_tree = group.evidence.viable_tree_candidates == 0U;
    const auto few_physical_alternatives = group.evidence.viable_corridor_candidates <= 1U &&
                                           group.evidence.topology_alternatives <= 1U &&
                                           group.evidence.layer_alternatives <= 1U &&
                                           group.evidence.primary_route_steps > step_median;
    const auto high_interference = group.evidence.estimated_interference > 0U &&
                                   group.evidence.estimated_interference >= interference_high;
    const auto high_terminal_crossing =
        group.evidence.terminal_crossing_pressure > 0U &&
        group.evidence.terminal_crossing_pressure >= terminal_crossing_high;
    const auto large_envelope = group.evidence.physical_envelope_nm > envelope_median &&
                                group.evidence.physical_envelope_nm >= envelope_high;
    group.constraint_signals =
        static_cast<std::size_t>(access_scarce) + static_cast<std::size_t>(no_complete_tree) +
        static_cast<std::size_t>(few_physical_alternatives) +
        static_cast<std::size_t>(high_interference) +
        static_cast<std::size_t>(high_terminal_crossing) + static_cast<std::size_t>(large_envelope);
    group.constraint_signals =
        saturating_count_add(group.constraint_signals, static_cast<std::size_t>(via_scarce));
    group.flexibility_signals =
        static_cast<std::size_t>(group.evidence.viable_tree_candidates > 1U) +
        static_cast<std::size_t>(group.evidence.viable_corridor_candidates > 1U) +
        static_cast<std::size_t>(group.evidence.topology_alternatives > 1U) +
        static_cast<std::size_t>(group.evidence.layer_alternatives > 1U);
    const auto flexible = group.flexibility_signals >= 2U &&
                          group.evidence.minimum_terminal_portals >= portal_median &&
                          group.evidence.estimated_interference <= interference_median &&
                          group.evidence.terminal_crossing_pressure <= terminal_crossing_median;
    group.evidence.wave = trivial                         ? IterationZeroRoutingWave::trivial
                          : group.constraint_signals > 0U ? IterationZeroRoutingWave::constrained
                          : flexible                      ? IterationZeroRoutingWave::flexible
                                                          : IterationZeroRoutingWave::ordinary;
    ordered_groups.push_back(std::move(group));
  }
  const auto wave_rank = [](IterationZeroRoutingWave wave) {
    switch (wave) {
    case IterationZeroRoutingWave::trivial:
      return 0U;
    case IterationZeroRoutingWave::constrained:
      return 1U;
    case IterationZeroRoutingWave::ordinary:
      return 2U;
    case IterationZeroRoutingWave::flexible:
      return 3U;
    }
    return 4U;
  };
  std::stable_sort(
      ordered_groups.begin(), ordered_groups.end(), [&](const auto& lhs, const auto& rhs) {
        if (lhs.evidence.explicit_priority != rhs.evidence.explicit_priority) {
          return lhs.evidence.explicit_priority > rhs.evidence.explicit_priority;
        }
        const auto lhs_wave = wave_rank(lhs.evidence.wave);
        const auto rhs_wave = wave_rank(rhs.evidence.wave);
        if (lhs_wave != rhs_wave) {
          return lhs_wave < rhs_wave;
        }
        if (lhs.evidence.wave == IterationZeroRoutingWave::trivial &&
            lhs.evidence.primary_route_steps != rhs.evidence.primary_route_steps) {
          return lhs.evidence.primary_route_steps < rhs.evidence.primary_route_steps;
        }
        if (lhs.evidence.wave == IterationZeroRoutingWave::constrained) {
          if (lhs.constraint_signals != rhs.constraint_signals) {
            return lhs.constraint_signals > rhs.constraint_signals;
          }
          if (lhs.evidence.estimated_interference != rhs.evidence.estimated_interference) {
            return lhs.evidence.estimated_interference > rhs.evidence.estimated_interference;
          }
          if (lhs.evidence.terminal_crossing_pressure != rhs.evidence.terminal_crossing_pressure) {
            return lhs.evidence.terminal_crossing_pressure >
                   rhs.evidence.terminal_crossing_pressure;
          }
          if (lhs.evidence.routing_freedom_score != rhs.evidence.routing_freedom_score) {
            return lhs.evidence.routing_freedom_score < rhs.evidence.routing_freedom_score;
          }
          if (lhs.evidence.minimum_terminal_portals != rhs.evidence.minimum_terminal_portals) {
            return lhs.evidence.minimum_terminal_portals < rhs.evidence.minimum_terminal_portals;
          }
          if (lhs.evidence.physical_envelope_nm != rhs.evidence.physical_envelope_nm) {
            return lhs.evidence.physical_envelope_nm > rhs.evidence.physical_envelope_nm;
          }
        }
        if (lhs.evidence.wave == IterationZeroRoutingWave::flexible &&
            lhs.flexibility_signals != rhs.flexibility_signals) {
          return lhs.flexibility_signals < rhs.flexibility_signals;
        }
        return lhs.evidence.group < rhs.evidence.group;
      });

  StagedIterationZeroOrder result;
  result.tasks.reserve(tasks.size());
  result.evidence.reserve(ordered_groups.size());
  for (auto& group : ordered_groups) {
    result.evidence.push_back(group.evidence);
    for (auto& ordered : group.tasks) {
      result.tasks.push_back(std::move(ordered.task));
    }
  }
  return result;
}

struct PathfinderPhysicalEnvelopeMetrics {
  std::uint64_t peak_net_nm{0};
  std::uint64_t peak_group_nm{0};
};

[[nodiscard]] PathfinderPhysicalEnvelopeMetrics
pathfinder_physical_envelope_metrics(const Board& board, const std::vector<NetTask>& tasks,
                                     const SharedVoronoiAllocationGraph& allocation) {
  const auto membership = pathfinder_group_membership(board);
  std::map<std::string, std::set<NetId>> group_nets;
  PathfinderPhysicalEnvelopeMetrics result;
  for (const auto& task : tasks) {
    const auto envelope = allocation.net_demand(task.net);
    result.peak_net_nm = std::max(result.peak_net_nm, envelope);
    const auto found = membership.find(task.net);
    const auto key = found == membership.end() ? "net:" + task.net.value() : found->second.group;
    group_nets[key].insert(task.net);
  }
  for (const auto& [key, nets] : group_nets) {
    static_cast<void>(key);
    result.peak_group_nm = std::max(result.peak_group_nm, allocation.occupied_demand(nets, {}));
  }
  return result;
}

[[nodiscard]] std::uint64_t stable_pathfinder_group_rank(std::string_view group,
                                                         std::size_t hard_pass_ordinal) {
  std::uint64_t hash = 14'695'981'039'346'656'037ULL;
  for (const auto character : group) {
    hash ^= static_cast<std::uint64_t>(static_cast<unsigned char>(character));
    hash *= 1'099'511'628'211ULL;
  }
  auto mixed = hash ^ (static_cast<std::uint64_t>(hard_pass_ordinal) + 0x9e3779b97f4a7c15ULL);
  mixed = (mixed ^ (mixed >> 30U)) * 0xbf58476d1ce4e5b9ULL;
  mixed = (mixed ^ (mixed >> 27U)) * 0x94d049bb133111ebULL;
  return mixed ^ (mixed >> 31U);
}

[[nodiscard]] std::uint64_t
global_checkpoint_topology_signature(const GlobalRouteResult& checkpoint) {
  std::uint64_t hash = 14'695'981'039'346'656'037ULL;
  const auto mix = [&hash](std::uint64_t value) {
    hash ^= value;
    hash *= 1'099'511'628'211ULL;
  };
  const auto mix_text = [&mix](std::string_view value) {
    for (const auto character : value) {
      mix(static_cast<std::uint64_t>(static_cast<unsigned char>(character)));
    }
  };
  for (const auto& path : checkpoint.paths) {
    mix_text(path.net.value());
    mix(path.connection_index);
    for (const auto& point : path.points) {
      mix(static_cast<std::uint64_t>(point.position.x.value()));
      mix(static_cast<std::uint64_t>(point.position.y.value()));
      mix_text(point.layer.value());
    }
  }
  return hash;
}

[[nodiscard]] std::size_t
reorder_pathfinder_tasks(const Board& board, const std::vector<RoutingNetPressure>& pressure,
                         const std::vector<NetId>& unrouted_nets, bool order_by_physical_envelope,
                         const SharedVoronoiAllocationGraph& allocation,
                         std::size_t hard_pass_ordinal,
                         const std::map<std::pair<NetId, NetId>, std::size_t>& hard_blockers,
                         const std::map<std::string, RoutingSeedPriorityEvidence>& seed_evidence,
                         std::size_t route_order_variant, std::uint64_t diversification_seed,
                         std::vector<NetTask>& tasks) {
  const auto membership = pathfinder_group_membership(board);
  std::map<NetId, RoutingNetPressure> pressure_by_net;
  for (const auto& item : pressure) {
    pressure_by_net.emplace(item.net, item);
  }

  struct OrderedTask {
    NetTask task;
    std::size_t member_order{0};
  };
  struct OrderedGroup {
    std::string key;
    std::uint32_t priority{0};
    bool unrouted{false};
    std::uint64_t physical_envelope_nm{0};
    std::uint64_t diversified_rank{0};
    std::size_t blocked_rejections{0};
    std::size_t blocking_rejections{0};
    std::size_t minimum_terminal_portals{0};
    std::size_t corridor_alternatives{0};
    std::size_t layer_alternatives{0};
    std::size_t primary_route_steps{0};
    std::size_t fanout{0};
    std::uint64_t routing_freedom_score{0};
    RoutingNetPressure pressure;
    std::set<NetId> nets;
    std::vector<OrderedTask> tasks;
  };
  std::map<std::string, OrderedGroup> groups;
  const std::set<NetId> unrouted(unrouted_nets.begin(), unrouted_nets.end());
  std::vector<NetId> prior_order;
  prior_order.reserve(tasks.size());
  for (const auto& task : tasks) {
    prior_order.push_back(task.net);
    const auto found = membership.find(task.net);
    const auto key = found == membership.end() ? "net:" + task.net.value() : found->second.group;
    const auto member_order = found == membership.end() ? 0U : found->second.member_order;
    auto [group, inserted] = groups.try_emplace(key);
    if (inserted) {
      group->second.key = key;
      group->second.pressure.net = task.net;
    }
    group->second.priority = std::max(group->second.priority, task.priority);
    group->second.unrouted = group->second.unrouted || unrouted.contains(task.net);
    group->second.nets.insert(task.net);
    group->second.fanout = std::max(group->second.fanout, task.terminals.size());
    if (const auto net_pressure = pressure_by_net.find(task.net);
        net_pressure != pressure_by_net.end()) {
      group->second.pressure.legacy_congested_resources =
          saturating_count_add(group->second.pressure.legacy_congested_resources,
                               net_pressure->second.legacy_congested_resources);
    }
    group->second.tasks.push_back(OrderedTask{task, member_order});
  }

  std::vector<OrderedGroup> ordered_groups;
  ordered_groups.reserve(groups.size());
  for (auto& [key, group] : groups) {
    static_cast<void>(key);
    group.physical_envelope_nm = allocation.occupied_demand(group.nets, {});
    group.diversified_rank = stable_pathfinder_group_rank(
        group.key, route_order_variant ^ static_cast<std::size_t>(diversification_seed));
    if (const auto evidence = seed_evidence.find(group.key); evidence != seed_evidence.end()) {
      group.minimum_terminal_portals = evidence->second.minimum_terminal_portals;
      group.corridor_alternatives = evidence->second.viable_corridor_candidates;
      group.layer_alternatives = evidence->second.layer_alternatives;
      group.primary_route_steps = evidence->second.primary_route_steps;
      group.routing_freedom_score = evidence->second.routing_freedom_score;
    }
    for (const auto& [relation, rejected_transitions] : hard_blockers) {
      if (group.nets.contains(relation.first)) {
        group.blocked_rejections =
            saturating_count_add(group.blocked_rejections, rejected_transitions);
      }
      if (group.nets.contains(relation.second)) {
        group.blocking_rejections =
            saturating_count_add(group.blocking_rejections, rejected_transitions);
      }
    }
    std::sort(group.tasks.begin(), group.tasks.end(), [](const auto& lhs, const auto& rhs) {
      return std::tie(lhs.member_order, lhs.task.net) < std::tie(rhs.member_order, rhs.task.net);
    });
    ordered_groups.push_back(std::move(group));
  }
  std::sort(
      ordered_groups.begin(), ordered_groups.end(),
      [order_by_physical_envelope, hard_pass_ordinal, route_order_variant](const auto& lhs,
                                                                           const auto& rhs) {
        if (lhs.priority != rhs.priority) {
          return lhs.priority > rhs.priority;
        }
        if (lhs.unrouted != rhs.unrouted) {
          return lhs.unrouted > rhs.unrouted;
        }
        if (route_order_variant > 0U) {
          switch ((route_order_variant - 1U) % 8U) {
          case 0U:
            if (lhs.routing_freedom_score != rhs.routing_freedom_score) {
              if (lhs.routing_freedom_score == 0U || rhs.routing_freedom_score == 0U) {
                return lhs.routing_freedom_score == 0U;
              }
              return lhs.routing_freedom_score < rhs.routing_freedom_score;
            }
            break;
          case 1U:
            if (lhs.primary_route_steps != rhs.primary_route_steps) {
              return lhs.primary_route_steps > rhs.primary_route_steps;
            }
            break;
          case 2U:
            if (lhs.primary_route_steps != rhs.primary_route_steps) {
              return lhs.primary_route_steps < rhs.primary_route_steps;
            }
            break;
          case 3U:
            if (lhs.fanout != rhs.fanout) {
              return lhs.fanout > rhs.fanout;
            }
            break;
          case 4U:
            if (lhs.corridor_alternatives != rhs.corridor_alternatives) {
              return lhs.corridor_alternatives < rhs.corridor_alternatives;
            }
            break;
          case 5U:
            if (lhs.minimum_terminal_portals != rhs.minimum_terminal_portals) {
              return lhs.minimum_terminal_portals < rhs.minimum_terminal_portals;
            }
            break;
          case 6U:
            if (lhs.layer_alternatives != rhs.layer_alternatives) {
              return lhs.layer_alternatives < rhs.layer_alternatives;
            }
            if (lhs.physical_envelope_nm != rhs.physical_envelope_nm) {
              return lhs.physical_envelope_nm > rhs.physical_envelope_nm;
            }
            break;
          case 7U:
            if (lhs.diversified_rank != rhs.diversified_rank) {
              return lhs.diversified_rank < rhs.diversified_rank;
            }
            break;
          }
        }
        if (order_by_physical_envelope && lhs.physical_envelope_nm != rhs.physical_envelope_nm) {
          return lhs.physical_envelope_nm > rhs.physical_envelope_nm;
        }
        if (lhs.pressure.legacy_congested_resources != rhs.pressure.legacy_congested_resources) {
          return lhs.pressure.legacy_congested_resources > rhs.pressure.legacy_congested_resources;
        }
        if (hard_pass_ordinal > 0U && lhs.blocked_rejections != rhs.blocked_rejections) {
          return lhs.blocked_rejections > rhs.blocked_rejections;
        }
        if (hard_pass_ordinal > 0U && lhs.blocking_rejections != rhs.blocking_rejections) {
          return lhs.blocking_rejections < rhs.blocking_rejections;
        }
        if (hard_pass_ordinal > 0U && lhs.diversified_rank != rhs.diversified_rank) {
          return lhs.diversified_rank < rhs.diversified_rank;
        }
        return lhs.key < rhs.key;
      });

  tasks.clear();
  tasks.reserve(prior_order.size());
  for (auto& group : ordered_groups) {
    for (auto& ordered : group.tasks) {
      tasks.push_back(std::move(ordered.task));
    }
  }
  std::size_t moved = 0U;
  for (std::size_t index = 0; index < tasks.size(); ++index) {
    moved += tasks[index].net == prior_order[index] ? 0U : 1U;
  }
  return moved;
}

struct ReusePlan {
  RoutingOccupancy occupancy;
  std::vector<GlobalRoutePath> paths;
  std::vector<NetId> reused_nets;
  std::vector<NetId> rerouted_nets;
  std::vector<NetId> invalidated_nets;
  std::size_t coupled_group_expansions{0};
  std::uint64_t cost{0};
};

[[nodiscard]] std::size_t expand_affected_routing_groups(const Board& board,
                                                         std::set<NetId>& affected) {
  std::size_t expansions = 0U;
  const auto expand = [&affected, &expansions](const auto& nets) {
    if (!std::any_of(nets.begin(), nets.end(),
                     [&affected](const NetId& net) { return affected.contains(net); })) {
      return;
    }
    for (const auto& net : nets) {
      expansions = saturating_count_add(expansions, affected.insert(net).second ? 1U : 0U);
    }
  };
  for (const auto& bus : board.routing_buses) {
    expand(bus.ordered_nets);
  }
  for (const auto& pair : board.differential_pairs) {
    const std::array nets{pair.positive_net, pair.negative_net};
    expand(nets);
  }
  return expansions;
}

[[nodiscard]] std::optional<std::size_t> route_node(const Grid& grid,
                                                    const GlobalRoutePoint& point) {
  const auto layer = std::find(grid.layers.begin(), grid.layers.end(), point.layer);
  if (layer == grid.layers.end()) {
    return std::nullopt;
  }
  const auto relative_x = point.position.x.value() - grid.origin_x;
  const auto relative_y = point.position.y.value() - grid.origin_y;
  if (relative_x < 0 || relative_y < 0 || relative_x % grid.pitch != 0 ||
      relative_y % grid.pitch != 0) {
    return std::nullopt;
  }
  const auto x = static_cast<std::size_t>(relative_x / grid.pitch);
  const auto y = static_cast<std::size_t>(relative_y / grid.pitch);
  if (!grid.valid(x, y)) {
    return std::nullopt;
  }
  return grid.node(x, y, static_cast<std::size_t>(layer - grid.layers.begin()));
}

void populate_neck_down_obstacle_impacts(GlobalRouteResult& result, const Board& board,
                                         const Grid& grid, RoutingObstacleIndex& obstacles) {
  result.neck_down_obstacle_impacts.clear();
  using Key =
      std::tuple<NetId, ObjectId, GlobalRouteSegmentWidthKind, LayerId, Point, Point, Nanometres>;
  std::set<Key> unique;
  for (const auto& path : result.paths) {
    if (path.segment_widths.size() + 1U != path.points.size() ||
        path.segment_width_kinds.size() != path.segment_widths.size()) {
      continue;
    }
    const auto nominal_width = resolve_net_routing_rules(board, path.net).track_width;
    for (std::size_t index = 0U; index < path.segment_widths.size(); ++index) {
      const auto selected_width = path.segment_widths[index];
      const auto kind = path.segment_width_kinds[index];
      const auto& first = path.points[index];
      const auto& second = path.points[index + 1U];
      if (kind == GlobalRouteSegmentWidthKind::nominal || selected_width.value() <= 0 ||
          selected_width >= nominal_width || first.layer != second.layer) {
        continue;
      }
      auto start = first.position;
      auto end = second.position;
      if (end < start) {
        std::swap(start, end);
      }
      std::vector<ObjectId> blocking_objects;
      if (kind == GlobalRouteSegmentWidthKind::terminal_neck_down) {
        const auto layer = std::find(grid.layers.begin(), grid.layers.end(), first.layer);
        if (layer == grid.layers.end()) {
          continue;
        }
        blocking_objects = obstacles.pin_access_blocking_objects(
            path.net, static_cast<std::size_t>(layer - grid.layers.begin()), start, end,
            nominal_width);
      } else {
        const auto first_node = route_node(grid, first);
        const auto second_node = route_node(grid, second);
        if (!first_node || !second_node) {
          continue;
        }
        blocking_objects = obstacles.edge_blocking_objects(
            path.net, Edge{*first_node, *second_node}, nominal_width);
      }
      for (const auto& obstacle : blocking_objects) {
        const auto key = Key{path.net, obstacle, kind, first.layer, start, end, selected_width};
        if (unique.insert(key).second) {
          result.neck_down_obstacle_impacts.push_back(RoutingNeckDownObstacleImpact{
              path.net, obstacle, kind, first.layer, start, end, selected_width});
        }
      }
    }
  }
}

[[nodiscard]] bool checkpoint_path_clear(const GlobalRoutePath& path, const NetTask& task,
                                         const Grid& grid, RoutingObstacleIndex& obstacles,
                                         const SharedVoronoiAllocationGraph& graph,
                                         const CorridorGraph& corridor_neck_downs,
                                         const GlobalRouterConfig& config) {
  if (path.points.empty() || path.connection_index + 1U >= task.terminals.size() ||
      path.points.front().position != task.terminals[path.connection_index + 1U].exact_position) {
    return false;
  }
  if ((!path.segment_widths.empty() && path.segment_widths.size() + 1U != path.points.size()) ||
      path.segment_width_kinds.size() != path.segment_widths.size()) {
    return false;
  }
  std::size_t neck_down_segments = 0U;
  std::uint64_t neck_down_length_nm = 0U;
  std::size_t medial_neck_down_segments = 0U;
  std::uint64_t medial_neck_down_length_nm = 0U;
  std::size_t corridor_neck_down_segments = 0U;
  std::uint64_t corridor_neck_down_length_nm = 0U;
  long double current_neck_down_run_nm = 0.0L;
  const auto finish_neck_down_run = [&]() {
    const bool valid = current_neck_down_run_nm <=
                       static_cast<long double>(config.max_terminal_neck_down_length.value());
    current_neck_down_run_nm = 0.0L;
    return valid;
  };
  long double current_corridor_neck_down_run_nm = 0.0L;
  const auto finish_corridor_neck_down_run = [&]() {
    const bool valid = current_corridor_neck_down_run_nm <=
                       static_cast<long double>(config.max_corridor_neck_down_length.value());
    current_corridor_neck_down_run_nm = 0.0L;
    return valid;
  };
  if (path.points.size() == 1U) {
    const auto node = route_node(grid, path.points.front());
    if (!node) {
      return false;
    }
    const auto [x, y, layer] = grid.decode(*node);
    const auto position = grid.point(x, y);
    return !obstacles.pin_access_blocked(path.net, layer, position, position);
  }
  for (std::size_t index = 1; index < path.points.size(); ++index) {
    const auto& first = path.points[index - 1U];
    const auto& second = path.points[index];
    const auto stored_width =
        path.segment_widths.empty() ? Nanometres{0} : path.segment_widths[index - 1U];
    const auto width_kind = path.segment_width_kinds.empty() ? GlobalRouteSegmentWidthKind::nominal
                                                             : path.segment_width_kinds[index - 1U];
    const auto previous_width_kind = path.segment_width_kinds.empty() || index <= 1U
                                         ? GlobalRouteSegmentWidthKind::nominal
                                         : path.segment_width_kinds[index - 2U];
    const auto next_width_kind = index >= path.segment_width_kinds.size()
                                     ? GlobalRouteSegmentWidthKind::nominal
                                     : path.segment_width_kinds[index];
    if (stored_width.value() < 0) {
      return false;
    }
    if ((stored_width.value() == 0) != (width_kind == GlobalRouteSegmentWidthKind::nominal)) {
      return false;
    }
    if (width_kind == GlobalRouteSegmentWidthKind::terminal_neck_down) {
      if (!finish_corridor_neck_down_run()) {
        return false;
      }
      if (task.coupled_member || config.terminal_neck_down_width.value() <= 0 ||
          config.max_terminal_neck_down_length.value() <= 0 ||
          stored_width != config.terminal_neck_down_width ||
          stored_width >= task.nominal_track_width || first.layer != second.layer) {
        return false;
      }
      ++neck_down_segments;
      const auto segment_length = geometry::point_distance_nm(first.position, second.position);
      current_neck_down_run_nm += segment_length;
      neck_down_length_nm = saturating_add(neck_down_length_nm,
                                           static_cast<std::uint64_t>(std::ceil(segment_length)));
    } else if (width_kind == GlobalRouteSegmentWidthKind::medial_neck_down) {
      if (!finish_neck_down_run() || !finish_corridor_neck_down_run() || task.coupled_member ||
          stored_width != config.medial_neck_down_width || first.layer != second.layer) {
        return false;
      }
      const auto first_node = route_node(grid, first);
      const auto second_node = route_node(grid, second);
      if (!first_node || !second_node) {
        return false;
      }
      const auto resource = graph.direct_resource(Edge{*first_node, *second_node});
      if (!resource || graph.medial_neck_down_width(*resource, path.net) != stored_width) {
        return false;
      }
      const auto [first_x, first_y, first_layer] = grid.decode(*first_node);
      const auto [second_x, second_y, second_layer] = grid.decode(*second_node);
      if (obstacles.pin_access_blocked(path.net, first_layer, grid.point(first_x, first_y),
                                       grid.point(first_x, first_y)) ||
          obstacles.pin_access_blocked(path.net, second_layer, grid.point(second_x, second_y),
                                       grid.point(second_x, second_y))) {
        return false;
      }
      ++medial_neck_down_segments;
      const auto segment_length = geometry::point_distance_nm(first.position, second.position);
      medial_neck_down_length_nm = saturating_add(
          medial_neck_down_length_nm, static_cast<std::uint64_t>(std::ceil(segment_length)));
    } else if (width_kind == GlobalRouteSegmentWidthKind::corridor_neck_down) {
      if (!finish_neck_down_run() || task.coupled_member ||
          stored_width != config.corridor_neck_down_width ||
          stored_width >= task.nominal_track_width || first.layer != second.layer) {
        return false;
      }
      const auto first_node = route_node(grid, first);
      const auto second_node = route_node(grid, second);
      if (!first_node || !second_node ||
          !corridor_neck_downs.corridor_for(Edge{*first_node, *second_node})) {
        return false;
      }
      const auto [first_x, first_y, layer] = grid.decode(*first_node);
      const auto [second_x, second_y, second_layer] = grid.decode(*second_node);
      if (layer != second_layer ||
          (previous_width_kind != GlobalRouteSegmentWidthKind::corridor_neck_down &&
           obstacles.pin_access_blocked(path.net, layer, grid.point(first_x, first_y),
                                        grid.point(first_x, first_y))) ||
          (next_width_kind != GlobalRouteSegmentWidthKind::corridor_neck_down &&
           obstacles.pin_access_blocked(path.net, layer, grid.point(second_x, second_y),
                                        grid.point(second_x, second_y)))) {
        return false;
      }
      ++corridor_neck_down_segments;
      const auto segment_length = geometry::point_distance_nm(first.position, second.position);
      current_corridor_neck_down_run_nm += segment_length;
      corridor_neck_down_length_nm = saturating_add(
          corridor_neck_down_length_nm, static_cast<std::uint64_t>(std::ceil(segment_length)));
      if (next_width_kind != GlobalRouteSegmentWidthKind::corridor_neck_down &&
          !finish_corridor_neck_down_run()) {
        return false;
      }
    } else if (!finish_neck_down_run() || !finish_corridor_neck_down_run()) {
      return false;
    }
    if (first.layer == second.layer) {
      const auto found = std::find(grid.layers.begin(), grid.layers.end(), first.layer);
      if (found == grid.layers.end() ||
          obstacles.pin_access_blocked(
              path.net, static_cast<std::size_t>(found - grid.layers.begin()), first.position,
              second.position,
              stored_width.value() > 0 ? stored_width : task.nominal_track_width)) {
        return false;
      }
    } else {
      const auto first_node = route_node(grid, first);
      const auto second_node = route_node(grid, second);
      if (!first_node || !second_node || first.position != second.position ||
          obstacles.edge_blocked(path.net, Edge{*first_node, *second_node})) {
        return false;
      }
    }
  }
  return finish_neck_down_run() && finish_corridor_neck_down_run() &&
         neck_down_segments == path.terminal_neck_down_segments &&
         neck_down_length_nm == path.terminal_neck_down_length_nm &&
         medial_neck_down_segments == path.medial_neck_down_segments &&
         medial_neck_down_length_nm == path.medial_neck_down_length_nm &&
         corridor_neck_down_segments == path.corridor_neck_down_segments &&
         corridor_neck_down_length_nm == path.corridor_neck_down_length_nm;
}

[[nodiscard]] ReusePlan
prepare_reuse(const GlobalRouteResult& checkpoint, const std::vector<NetId>& requested_affected,
              const Board& board, const std::vector<NetTask>& all_tasks, const Grid& grid,
              const CorridorGraph& corridors, const VoronoiPathFinderGuidance& voronoi_guidance,
              const CorridorGraph& corridor_neck_downs,
              const SharedVoronoiAllocationGraph& voronoi_allocation,
              RoutingObstacleIndex& obstacles, const GlobalRouterConfig& config,
              bool preserve_unaffected_provisional_congestion = false) {
  ReusePlan result;
  std::map<NetId, Nanometres> nominal_width_by_net;
  for (const auto& task : all_tasks) {
    nominal_width_by_net.emplace(task.net, task.nominal_track_width);
  }
  std::set<NetId> affected(requested_affected.begin(), requested_affected.end());
  if (!preserve_unaffected_provisional_congestion &&
      (!checkpoint.selected_shove_proposals.empty() ||
       checkpoint.configured_selected_nets != config.selected_nets ||
       checkpoint.configured_priority_nets != config.priority_nets ||
       checkpoint.configured_grid_pitch != config.grid_pitch ||
       checkpoint.configured_via_site_pitch != config.via_site_pitch ||
       checkpoint.configured_via_site_phase_x != config.via_site_phase_x ||
       checkpoint.configured_via_site_phase_y != config.via_site_phase_y ||
       checkpoint.configured_diagonal_grid_edges != config.allow_diagonal_grid_edges ||
       checkpoint.configured_crossing_safe_diagonal_grid_edges !=
           config.allow_crossing_safe_diagonal_grid_edges ||
       checkpoint.configured_sampled_route_clearance_reservations !=
           config.reserve_sampled_route_clearance ||
       checkpoint.configured_max_clearance_shadow_conflict_resources !=
           config.max_clearance_shadow_conflict_resources ||
       checkpoint.configured_clearance_shadow_cost_quantum !=
           config.clearance_shadow_cost_quantum ||
       checkpoint.configured_single_lane_direct_voronoi_resources !=
           config.single_lane_direct_voronoi_resources ||
       checkpoint.configured_max_static_seed_candidates_per_connection !=
           config.max_static_seed_candidates_per_connection ||
       checkpoint.configured_max_static_seed_searches != config.max_static_seed_searches ||
       checkpoint.configured_max_static_seed_reprojection_searches !=
           config.max_static_seed_reprojection_searches ||
       checkpoint.configured_static_seed_alternative_edge_penalty !=
           config.static_seed_alternative_edge_penalty ||
       checkpoint.configured_static_seed_corridor_departure_penalty !=
           config.static_seed_corridor_departure_penalty ||
       checkpoint.configured_former_route_memory_cost != config.former_route_memory_cost ||
       checkpoint.configured_terminal_escape_reservation_cost !=
           config.terminal_escape_reservation_cost ||
       checkpoint.configured_max_terminal_escape_reservation_resources !=
           config.max_terminal_escape_reservation_resources ||
       checkpoint.configured_global_spline_tessellation_tolerance !=
           config.global_spline_tessellation_tolerance ||
       checkpoint.configured_global_spline_control_corridor !=
           config.global_spline_control_corridor ||
       checkpoint.configured_global_spline_shove_step != config.global_spline_shove_step ||
       checkpoint.configured_navmesh_funnel_spline_seeds !=
           config.use_navmesh_funnel_spline_seeds ||
       checkpoint.configured_global_navmesh_cell_pitch != config.global_navmesh_cell_pitch ||
       checkpoint.configured_global_spline_minimum_bend_radius !=
           config.global_spline_minimum_bend_radius ||
       checkpoint.configured_max_global_navmesh_cells_per_seed !=
           config.max_global_navmesh_cells_per_seed ||
       checkpoint.configured_max_global_navmesh_search_expansions_per_seed !=
           config.max_global_navmesh_search_expansions_per_seed ||
       checkpoint.configured_global_spline_trajectory_samples !=
           config.global_spline_trajectory_samples ||
       checkpoint.configured_max_global_spline_trajectory_iterations !=
           config.max_global_spline_trajectory_iterations ||
       checkpoint.configured_max_global_spline_trajectory_candidate_checks !=
           config.max_global_spline_trajectory_candidate_checks ||
       checkpoint.configured_global_spline_max_tessellation_depth !=
           config.global_spline_max_tessellation_depth ||
       checkpoint.configured_max_global_spline_samples_per_seed !=
           config.max_global_spline_samples_per_seed ||
       checkpoint.configured_max_global_spline_shove_iterations !=
           config.max_global_spline_shove_iterations ||
       checkpoint.configured_max_static_seed_corridor_cells !=
           config.max_static_seed_corridor_cells ||
       checkpoint.configured_max_global_pns_propagation_depth !=
           config.max_global_pns_propagation_depth ||
       checkpoint.configured_max_global_pns_candidate_checks !=
           config.max_global_pns_candidate_checks ||
       checkpoint.configured_maximum_global_pns_displacement !=
           config.maximum_global_pns_displacement ||
       checkpoint.configured_local_pin_escape_pitch != config.local_pin_escape_pitch ||
       checkpoint.configured_max_local_pin_escape_expansions_per_terminal !=
           config.max_local_pin_escape_expansions_per_terminal ||
       (checkpoint.configured_contended_terminal_fanout &&
        !config.allocate_contended_terminal_fanout) ||
       // Enabling diversification only adds bounded legal terminal portals. A preserved path that
       // used the narrower access set remains valid; disabling it could remove a portal used by the
       // checkpoint and therefore still invalidates reuse.
       (checkpoint.configured_diversify_pin_access_rings && !config.diversify_pin_access_rings) ||
       checkpoint.configured_hard_capacity_start_iteration !=
           config.hard_capacity_start_iteration ||
       checkpoint.configured_max_hard_capacity_ripup_attempts_per_pass !=
           config.max_hard_capacity_ripup_attempts_per_pass ||
       checkpoint.configured_max_hard_capacity_ripup_nets_per_attempt !=
           config.max_hard_capacity_ripup_nets_per_attempt ||
       checkpoint.configured_max_hard_capacity_conflict_set_states_per_attempt !=
           config.max_hard_capacity_conflict_set_states_per_attempt ||
       checkpoint.configured_rollback_failed_hard_capacity_ripup !=
           config.rollback_failed_hard_capacity_ripup ||
       checkpoint.configured_defer_hard_capacity_ripup_victims_to_pass_tail !=
           config.defer_hard_capacity_ripup_victims_to_pass_tail ||
       checkpoint.configured_protect_recovered_hard_capacity_ripup_nets !=
           config.protect_recovered_hard_capacity_ripup_nets ||
       checkpoint.configured_max_hard_capacity_blocker_reuse_invalidations_per_failed_net !=
           config.max_hard_capacity_blocker_reuse_invalidations_per_failed_net ||
       config.allow_crossing_safe_diagonal_grid_edges)) {
    for (const auto& task : all_tasks) {
      affected.insert(task.net);
    }
  }
  affected.insert(checkpoint.unrouted_nets.begin(), checkpoint.unrouted_nets.end());
  std::set<NetId> invalidated;
  if (!preserve_unaffected_provisional_congestion) {
    const auto invalidate_congested = [&](const auto& usages) {
      for (const auto& usage : usages) {
        if (!usage.congested()) {
          continue;
        }
        for (const auto& net : usage.nets) {
          affected.insert(net);
          invalidated.insert(net);
        }
      }
    };
    invalidate_congested(checkpoint.resource_usage);
    invalidate_congested(checkpoint.voronoi_resource_usage);
    invalidate_congested(checkpoint.corridor_usage);
    invalidate_congested(checkpoint.corridor_neck_down_usage);
  }

  std::map<NetId, std::vector<const GlobalRoutePath*>> paths_by_net;
  for (const auto& path : checkpoint.paths) {
    paths_by_net[path.net].push_back(&path);
  }
  std::set<NetId> reusable;
  for (const auto& task : all_tasks) {
    if (affected.contains(task.net)) {
      continue;
    }
    const auto found = paths_by_net.find(task.net);
    const auto expected_paths = task.terminals.size() - 1U;
    bool valid = found != paths_by_net.end() && found->second.size() == expected_paths;
    std::vector<bool> connections(expected_paths, false);
    if (valid) {
      for (const auto* path : found->second) {
        if (path->connection_index >= expected_paths || connections[path->connection_index] ||
            (!preserve_unaffected_provisional_congestion &&
             !checkpoint_path_clear(*path, task, grid, obstacles, voronoi_allocation,
                                    corridor_neck_downs, config))) {
          valid = false;
          break;
        }
        connections[path->connection_index] = true;
      }
    }
    if (valid) {
      reusable.insert(task.net);
    } else {
      affected.insert(task.net);
      invalidated.insert(task.net);
    }
  }

  std::set<NetId> mapping_invalid;
  std::set<NetId> resource_nets_seen;
  for (const auto& usage : checkpoint.resource_usage) {
    std::vector<NetId> relevant;
    for (const auto& net : usage.nets) {
      if (reusable.contains(net)) {
        relevant.push_back(net);
      }
    }
    if (relevant.empty()) {
      continue;
    }
    const auto first = route_node(grid, usage.first);
    const auto second = route_node(grid, usage.second);
    const auto direct =
        first && second ? voronoi_allocation.direct_resource(Edge{*first, *second}) : std::nullopt;
    const bool compatible =
        first && second &&
        (usage.kind == RoutingResourceKind::medial_edge
             ? direct && usage.capacity == (config.single_lane_direct_voronoi_resources
                                                ? config.edge_capacity
                                                : voronoi_allocation.direct_lane_capacity(*direct))
             : usage.capacity == config.edge_capacity);
    if (!compatible && !preserve_unaffected_provisional_congestion) {
      mapping_invalid.insert(relevant.begin(), relevant.end());
    } else {
      resource_nets_seen.insert(relevant.begin(), relevant.end());
    }
  }
  if (!preserve_unaffected_provisional_congestion) {
    for (const auto& net : reusable) {
      if (!resource_nets_seen.contains(net)) {
        mapping_invalid.insert(net);
      }
    }
  }
  for (const auto& net : mapping_invalid) {
    reusable.erase(net);
    affected.insert(net);
    invalidated.insert(net);
  }
  result.coupled_group_expansions = expand_affected_routing_groups(board, affected);
  for (const auto& net : affected) {
    reusable.erase(net);
  }

  std::set<std::pair<Edge, NetId>> checkpoint_corridor_neck_down_edges;
  std::map<std::pair<Edge, NetId>, Nanometres> checkpoint_selected_widths;
  for (const auto& path : checkpoint.paths) {
    if (!reusable.contains(path.net)) {
      continue;
    }
    for (std::size_t index = 0U; index < path.segment_width_kinds.size(); ++index) {
      const auto first = route_node(grid, path.points[index]);
      const auto second = route_node(grid, path.points[index + 1U]);
      if (first && second) {
        const Edge edge{*first, *second};
        auto& width = checkpoint_selected_widths[{edge, path.net}];
        width = std::max(width, path.segment_widths[index]);
        if (path.segment_width_kinds[index] == GlobalRouteSegmentWidthKind::corridor_neck_down) {
          checkpoint_corridor_neck_down_edges.emplace(edge, path.net);
        }
      }
    }
  }

  for (const auto& usage : checkpoint.resource_usage) {
    const auto first = route_node(grid, usage.first);
    const auto second = route_node(grid, usage.second);
    const auto direct =
        first && second ? voronoi_allocation.direct_resource(Edge{*first, *second}) : std::nullopt;
    const bool compatible =
        first && second &&
        (usage.kind == RoutingResourceKind::medial_edge
             ? direct && usage.capacity == (config.single_lane_direct_voronoi_resources
                                                ? config.edge_capacity
                                                : voronoi_allocation.direct_lane_capacity(*direct))
             : usage.capacity == config.edge_capacity);
    if (!compatible) {
      continue;
    }
    for (const auto& net : usage.nets) {
      if (!reusable.contains(net)) {
        continue;
      }
      if (usage.kind == RoutingResourceKind::node) {
        result.occupancy.nodes[*first].insert(net);
      } else {
        const Edge edge{*first, *second};
        const auto selected = checkpoint_selected_widths.find({edge, net});
        const auto selected_width =
            selected == checkpoint_selected_widths.end() ? Nanometres{0} : selected->second;
        result.occupancy.edges[edge].insert(net);
        const bool corridor_neck_down = checkpoint_corridor_neck_down_edges.contains({edge, net});
        if (corridor_neck_down) {
          result.occupancy.corridor_neck_down_edges[edge].insert(net);
          if (const auto corridor = corridor_neck_downs.corridor_for(edge)) {
            result.occupancy.corridor_neck_downs[*corridor].insert(net);
          }
        }
        auto edge_cost = usage.kind == RoutingResourceKind::via_edge
                             ? config.via_cost
                             : direct_edge_base_cost(grid, edge, config);
        if (usage.kind == RoutingResourceKind::planar_edge ||
            usage.kind == RoutingResourceKind::medial_edge) {
          const auto& selected_corridors = corridor_neck_down ? corridor_neck_downs : corridors;
          const auto [centering_numerator, centering_denominator] =
              selected_corridors.centering_ratio(grid, edge);
          if (usage.kind == RoutingResourceKind::planar_edge) {
            edge_cost =
                saturating_add(edge_cost, saturating_multiply(centering_numerator,
                                                              config.corridor_centering_cost) /
                                              centering_denominator);
          }
          edge_cost = saturating_add(
              edge_cost, voronoi_guidance.edge_cost(net, edge, config.voronoi_centering_cost));
          edge_cost = saturating_add(
              edge_cost,
              width_reduction_penalty(usage.first.position, usage.second.position,
                                      nominal_width_by_net.at(net), selected_width,
                                      config.grid_pitch, config.width_reduction_cost_per_grid_step)
                  .cost);
          const RoutingHistory empty_history;
          const auto complement = voronoi_allocation.differential_complement(net);
          const bool complement_allocated =
              complement && std::any_of(result.occupancy.voronoi_resources.begin(),
                                        result.occupancy.voronoi_resources.end(),
                                        [&complement](const auto& entry) {
                                          return entry.second.contains(*complement);
                                        });
          const auto* routing_bus = voronoi_allocation.routing_bus(net);
          const bool routing_bus_member_allocated =
              routing_bus != nullptr &&
              std::any_of(result.occupancy.voronoi_resources.begin(),
                          result.occupancy.voronoi_resources.end(),
                          [&routing_bus, &net](const auto& entry) {
                            return std::any_of(
                                routing_bus->ordered_nets.begin(), routing_bus->ordered_nets.end(),
                                [&entry, &net](const NetId& member) {
                                  return member != net && entry.second.contains(member);
                                });
                          });
          edge_cost = saturating_add(
              edge_cost,
              voronoi_allocation_edge_cost(voronoi_allocation, result.occupancy, empty_history, net,
                                           complement_allocated, routing_bus_member_allocated, edge,
                                           selected_width, config));
          const auto selected_demand = voronoi_allocation.demand_for_width(net, selected_width);
          for (const auto resource : voronoi_allocation.resources_for(edge)) {
            const bool direct_resource = voronoi_allocation.direct_resource(edge) == resource;
            if (!direct_resource &&
                selected_demand > voronoi_allocation.resources[resource].capacity_nm) {
              continue;
            }
            result.occupancy.voronoi_resources[resource].insert(net);
            auto& demand = result.occupancy.voronoi_demand_nm_by_net[resource][net];
            const bool direct_medial_neck_down =
                direct_resource &&
                voronoi_allocation.medial_neck_down_width(resource, net) == selected_width;
            if (direct_medial_neck_down) {
              result.occupancy.direct_medial_neck_downs[resource].insert(net);
              demand = selected_demand;
            } else if (!result.occupancy.direct_medial_neck_downs[resource].contains(net)) {
              demand = std::max(demand, selected_demand);
            }
          }
        }
        result.cost = saturating_add(result.cost, edge_cost);
      }
    }
  }
  for (const auto& path : checkpoint.paths) {
    if (reusable.contains(path.net)) {
      result.paths.push_back(path);
      const auto rules = resolve_net_routing_rules(board, path.net);
      if (config.reserve_sampled_route_clearance) {
        add_path_clearance_occupancy(path, rules.track_width, rules.via_diameter, obstacles, grid,
                                     result.occupancy);
      }
      result.cost = saturating_add(
          result.cost, path_width_reduction_penalty(path, nominal_width_by_net.at(path.net), config,
                                                    GlobalRouteSegmentWidthKind::terminal_neck_down)
                           .cost);
    }
  }
  for (const auto& [edge, nets] : result.occupancy.edges) {
    if (const auto corridor = corridors.corridor_for(edge)) {
      for (const auto& net : nets) {
        const auto narrow = result.occupancy.corridor_neck_down_edges.find(edge);
        if (narrow == result.occupancy.corridor_neck_down_edges.end() ||
            !narrow->second.contains(net)) {
          result.occupancy.corridors[*corridor].insert(net);
        }
      }
    }
  }
  result.reused_nets.assign(reusable.begin(), reusable.end());
  result.invalidated_nets.assign(invalidated.begin(), invalidated.end());
  for (const auto& task : all_tasks) {
    if (!reusable.contains(task.net)) {
      result.rerouted_nets.push_back(task.net);
    }
  }
  return result;
}

GlobalRouteResult route_global_impl(const Board& board, const GlobalRouterConfig& config,
                                    std::stop_token stop_token, const GlobalRouteResult* checkpoint,
                                    const std::vector<NetId>& affected_nets,
                                    RouterTelemetrySink* telemetry,
                                    bool preserve_unaffected_provisional_congestion = false) {
  GlobalRouteResult invalid;
  const auto emit_telemetry = [telemetry](RouterTelemetryEvent event) {
    if (telemetry != nullptr) {
      static_cast<void>(telemetry->try_emit(std::move(event)));
    }
  };
  const auto validation = validate_board(board);
  if (!validation.ok()) {
    invalid.diagnostic = "board failed structural/placement validation";
    return invalid;
  }
  if (stop_token.stop_requested()) {
    invalid.stop_reason = GlobalRouterStopReason::cancelled;
    invalid.diagnostic = "global routing cancelled before grid construction";
    return invalid;
  }

  std::set<NetId> selected;
  for (const auto& net : config.selected_nets) {
    const auto source = std::find_if(board.nets.begin(), board.nets.end(),
                                     [&net](const Net& candidate) { return candidate.id == net; });
    if (!selected.insert(net).second || source == board.nets.end() || source->name.empty()) {
      invalid.diagnostic =
          "selected routing nets must be unique IDs of named nets in the source board";
      return invalid;
    }
  }
  std::set<NetId> prioritized;
  for (const auto& net : config.priority_nets) {
    const auto source = std::find_if(board.nets.begin(), board.nets.end(),
                                     [&net](const Net& candidate) { return candidate.id == net; });
    if (!prioritized.insert(net).second || source == board.nets.end() || source->name.empty() ||
        (!selected.empty() && !selected.contains(net))) {
      invalid.diagnostic = "priority routing nets must be unique named nets inside the selected "
                           "routing scope";
      return invalid;
    }
  }
  if (config.reserve_sampled_route_clearance &&
      config.max_clearance_shadow_conflict_resources == 0U) {
    invalid.diagnostic = "clearance-shadow negotiation requires a positive resource bound";
    return invalid;
  }
  if (!selected.empty()) {
    const auto partially_selected = [&selected](const std::vector<NetId>& members) {
      const auto count =
          std::count_if(members.begin(), members.end(),
                        [&selected](const NetId& net) { return selected.contains(net); });
      return count != 0 && static_cast<std::size_t>(count) != members.size();
    };
    for (const auto& pair : board.differential_pairs) {
      if (partially_selected({pair.positive_net, pair.negative_net})) {
        invalid.diagnostic = "selected routing scope splits a declared differential pair";
        return invalid;
      }
    }
    for (const auto& bus : board.routing_buses) {
      if (partially_selected(bus.ordered_nets)) {
        invalid.diagnostic = "selected routing scope splits a declared routing bus";
        return invalid;
      }
    }
  }

  std::string diagnostic;
  const auto grid = make_grid(board, config, diagnostic);
  if (!grid) {
    invalid.diagnostic = std::move(diagnostic);
    return invalid;
  }
  auto tasks = make_tasks(board, *grid, config);
  RouterTelemetryEvent started;
  started.kind = RouterTelemetryEventKind::routing_started;
  started.total_nets = tasks.size();
  started.detail = board.source_revision;
  emit_telemetry(std::move(started));
  if (tasks.empty()) {
    invalid.stop_reason = GlobalRouterStopReason::complete;
    invalid.configured_selected_nets = config.selected_nets;
    invalid.configured_priority_nets = config.priority_nets;
    invalid.configured_grid_pitch = config.grid_pitch;
    invalid.configured_via_site_pitch = config.via_site_pitch;
    invalid.configured_via_site_phase_x = config.via_site_phase_x;
    invalid.configured_via_site_phase_y = config.via_site_phase_y;
    invalid.configured_diagonal_grid_edges = config.allow_diagonal_grid_edges;
    invalid.configured_crossing_safe_diagonal_grid_edges =
        config.allow_crossing_safe_diagonal_grid_edges;
    invalid.configured_sampled_route_clearance_reservations =
        config.reserve_sampled_route_clearance;
    invalid.configured_max_clearance_shadow_conflict_resources =
        config.max_clearance_shadow_conflict_resources;
    invalid.configured_clearance_shadow_cost_quantum = config.clearance_shadow_cost_quantum;
    invalid.configured_max_static_seed_candidates_per_connection =
        config.max_static_seed_candidates_per_connection;
    invalid.configured_max_static_seed_searches = config.max_static_seed_searches;
    invalid.configured_max_static_seed_reprojection_searches =
        config.max_static_seed_reprojection_searches;
    invalid.configured_max_static_seed_workers = config.max_static_seed_workers;
    invalid.configured_static_seed_alternative_edge_penalty =
        config.static_seed_alternative_edge_penalty;
    invalid.configured_static_seed_corridor_departure_penalty =
        config.static_seed_corridor_departure_penalty;
    invalid.configured_residual_corridor_demand_cost = config.residual_corridor_demand_cost;
    invalid.configured_critical_cut_cost = config.critical_cut_cost;
    invalid.configured_critical_cut_minimum_future_demand =
        config.critical_cut_minimum_future_demand;
    invalid.configured_former_route_memory_cost = config.former_route_memory_cost;
    invalid.configured_terminal_escape_reservation_cost = config.terminal_escape_reservation_cost;
    invalid.configured_max_terminal_escape_reservation_resources =
        config.max_terminal_escape_reservation_resources;
    invalid.configured_global_spline_tessellation_tolerance =
        config.global_spline_tessellation_tolerance;
    invalid.configured_global_spline_control_corridor = config.global_spline_control_corridor;
    invalid.configured_global_spline_shove_step = config.global_spline_shove_step;
    invalid.configured_navmesh_funnel_spline_seeds = config.use_navmesh_funnel_spline_seeds;
    invalid.configured_global_navmesh_cell_pitch = config.global_navmesh_cell_pitch;
    invalid.configured_global_spline_minimum_bend_radius = config.global_spline_minimum_bend_radius;
    invalid.configured_max_global_navmesh_cells_per_seed = config.max_global_navmesh_cells_per_seed;
    invalid.configured_max_global_navmesh_search_expansions_per_seed =
        config.max_global_navmesh_search_expansions_per_seed;
    invalid.configured_global_spline_trajectory_samples = config.global_spline_trajectory_samples;
    invalid.configured_max_global_spline_trajectory_iterations =
        config.max_global_spline_trajectory_iterations;
    invalid.configured_max_global_spline_trajectory_candidate_checks =
        config.max_global_spline_trajectory_candidate_checks;
    invalid.configured_global_spline_max_tessellation_depth =
        config.global_spline_max_tessellation_depth;
    invalid.configured_max_global_spline_samples_per_seed =
        config.max_global_spline_samples_per_seed;
    invalid.configured_max_global_spline_shove_iterations =
        config.max_global_spline_shove_iterations;
    invalid.configured_max_static_seed_corridor_cells = config.max_static_seed_corridor_cells;
    invalid.configured_max_global_pns_propagation_depth = config.max_global_pns_propagation_depth;
    invalid.configured_max_global_pns_candidate_checks = config.max_global_pns_candidate_checks;
    invalid.configured_maximum_global_pns_displacement = config.maximum_global_pns_displacement;
    invalid.multisource_steiner_growth_enabled = config.use_multisource_steiner_growth;
    invalid.bidirectional_route_growth_enabled = config.use_bidirectional_route_growth;
    invalid.configured_bidirectional_route_growth_start_iteration =
        config.bidirectional_route_growth_start_iteration;
    invalid.configured_max_bidirectional_meeting_nodes = config.max_bidirectional_meeting_nodes;
    invalid.configured_max_bidirectional_meeting_expansions =
        config.max_bidirectional_meeting_expansions;
    invalid.atomic_complete_net_routing_enabled = config.atomic_complete_net_routing;
    invalid.configured_selective_congestion_reroute = config.selective_congestion_reroute;
    invalid.configured_selective_full_reroute_interval = config.selective_full_reroute_interval;
    invalid.configured_max_expansions_per_connection = config.max_expansions_per_connection;
    invalid.configured_expansion_retry_heuristic_weight_per_mille =
        config.expansion_retry_heuristic_weight_per_mille;
    invalid.configured_reuse_weighted_search_frontier = config.reuse_weighted_search_frontier;
    invalid.configured_checkpoint_objective = config.checkpoint_objective;
    invalid.configured_scale_voronoi_history_by_overflow = config.scale_voronoi_history_by_overflow;
    invalid.configured_physical_envelope_ordering =
        config.order_routing_groups_by_physical_envelope;
    invalid.configured_present_congestion_multiplier_per_mille =
        config.present_congestion_multiplier_per_mille;
    invalid.configured_max_present_congestion_scale_per_mille =
        config.max_present_congestion_scale_per_mille;
    invalid.configured_present_congestion_escalation_start_iteration =
        config.present_congestion_escalation_start_iteration;
    invalid.configured_diversify_pin_access_rings = config.diversify_pin_access_rings;
    invalid.configured_contended_terminal_fanout = config.allocate_contended_terminal_fanout;
    invalid.configured_medial_neck_down_width = config.medial_neck_down_width;
    invalid.configured_max_medial_neck_down_length = config.max_medial_neck_down_length;
    invalid.configured_max_voronoi_resource_segment_length =
        config.max_voronoi_resource_segment_length;
    invalid.configured_hard_capacity_start_iteration = config.hard_capacity_start_iteration;
    invalid.configured_max_hard_capacity_ripup_attempts_per_pass =
        config.max_hard_capacity_ripup_attempts_per_pass;
    invalid.configured_max_hard_capacity_ripup_nets_per_attempt =
        config.max_hard_capacity_ripup_nets_per_attempt;
    invalid.configured_max_hard_capacity_conflict_set_states_per_attempt =
        config.max_hard_capacity_conflict_set_states_per_attempt;
    invalid.configured_rollback_failed_hard_capacity_ripup =
        config.rollback_failed_hard_capacity_ripup;
    invalid.configured_defer_hard_capacity_ripup_victims_to_pass_tail =
        config.defer_hard_capacity_ripup_victims_to_pass_tail;
    invalid.configured_protect_recovered_hard_capacity_ripup_nets =
        config.protect_recovered_hard_capacity_ripup_nets;
    invalid.diagnostic = "no multi-terminal nets require global routing";
    RouterTelemetryEvent finished;
    finished.kind = RouterTelemetryEventKind::routing_finished;
    finished.total_nets = 0U;
    finished.detail = "no incomplete named nets";
    emit_telemetry(std::move(finished));
    return invalid;
  }
  RoutingObstacleIndex obstacles{board, *grid, config.respect_existing_copper,
                                 !config.treat_saved_zone_fills_as_refillable};
  if (!obstacles.valid()) {
    invalid.diagnostic = obstacles.diagnostic();
    return invalid;
  }
  const auto corridors = build_corridor_graph(*grid, config, obstacles, stop_token, diagnostic);
  if (!corridors) {
    invalid.stop_reason = stop_token.stop_requested() ? GlobalRouterStopReason::cancelled
                                                      : GlobalRouterStopReason::invalid_input;
    invalid.diagnostic = std::move(diagnostic);
    return invalid;
  }
  CorridorGraph corridor_neck_downs;
  std::size_t coupled_nets_ineligible_for_corridor_neck_down = 0U;
  if (config.corridor_neck_down_width.value() > 0) {
    Nanometres maximum_eligible_clearance{0};
    bool has_eligible_net = false;
    for (const auto& task : tasks) {
      if (config.corridor_neck_down_width >= task.nominal_track_width) {
        continue;
      }
      if (task.coupled_member) {
        ++coupled_nets_ineligible_for_corridor_neck_down;
        continue;
      }
      has_eligible_net = true;
      maximum_eligible_clearance = std::max(maximum_eligible_clearance,
                                            resolve_net_routing_rules(board, task.net).clearance);
    }
    if (has_eligible_net) {
      const auto built =
          build_corridor_graph(*grid, config, obstacles, stop_token, diagnostic,
                               config.corridor_neck_down_width, maximum_eligible_clearance);
      if (!built) {
        invalid.stop_reason = stop_token.stop_requested() ? GlobalRouterStopReason::cancelled
                                                          : GlobalRouterStopReason::invalid_input;
        invalid.diagnostic = std::move(diagnostic);
        return invalid;
      }
      corridor_neck_downs = std::move(*built);
    }
  }
  const auto voronoi_evidence =
      build_voronoi_corridor_evidence(board, *grid, tasks, config, stop_token);
  if (voronoi_evidence.cancelled) {
    invalid.stop_reason = GlobalRouterStopReason::cancelled;
    invalid.diagnostic = "global routing cancelled during Voronoi corridor evidence construction";
    return invalid;
  }
  auto voronoi_guidance =
      build_voronoi_pathfinder_guidance(*grid, voronoi_evidence, config, diagnostic);
  if (!voronoi_guidance) {
    invalid.diagnostic = std::move(diagnostic);
    return invalid;
  }
  const auto voronoi_allocation = build_shared_voronoi_allocation_graph(
      board, *grid, tasks, config, obstacles, stop_token, diagnostic);
  if (!voronoi_allocation) {
    invalid.stop_reason = stop_token.stop_requested() ? GlobalRouterStopReason::cancelled
                                                      : GlobalRouterStopReason::invalid_input;
    invalid.diagnostic = std::move(diagnostic);
    return invalid;
  }
  const auto pin_access =
      filter_pin_access(tasks, *grid, obstacles, corridor_neck_downs, *voronoi_allocation, config);
  const auto prepared_routability_history =
      prepare_routability_history(*grid, *voronoi_allocation, config);
  const auto static_seed_plan = build_static_hybrid_seed_plan(
      board, *grid, tasks, *corridors, corridor_neck_downs, *voronoi_allocation, obstacles, config,
      prepared_routability_history.history, stop_token);
  if (stop_token.stop_requested()) {
    invalid.stop_reason = GlobalRouterStopReason::cancelled;
    invalid.diagnostic = "global routing cancelled during static seed corridor planning";
    return invalid;
  }
  const auto physical_envelope_metrics =
      pathfinder_physical_envelope_metrics(board, tasks, *voronoi_allocation);
  const auto all_tasks = tasks;
  // Static seed candidates provide a bounded, deterministic lower bound on future corridor
  // demand.  A corridor is counted only when it occurs in every complete candidate tree for a
  // net; paths that merely could use it are deliberately not reserved.
  const auto seed_corridors = [&](const SearchResult& route) {
    ResidualCorridorDemand demand;
    for (std::size_t index = 1U; index < route.nodes.size(); ++index) {
      const auto kind = index - 1U < route.segment_width_kinds.size()
                            ? route.segment_width_kinds[index - 1U]
                            : GlobalRouteSegmentWidthKind::nominal;
      const Edge edge{route.nodes[index - 1U], route.nodes[index]};
      if (kind == GlobalRouteSegmentWidthKind::corridor_neck_down) {
        if (const auto corridor = corridor_neck_downs.corridor_for(edge)) {
          demand.neck_downs.insert(*corridor);
        }
      } else if (const auto corridor = corridors->corridor_for(edge)) {
        demand.corridors.insert(*corridor);
      }
    }
    return demand;
  };
  const auto residual_corridor_demand_by_net = [&] {
    std::map<NetId, ResidualCorridorDemand> mandatory;
    for (const auto& task : all_tasks) {
      const auto required_connections =
          task.terminals.size() > 1U ? task.terminals.size() - 1U : 0U;
      if (required_connections == 0U) {
        continue;
      }
      std::map<std::size_t, std::map<std::size_t, ResidualCorridorDemand>> tree_connections;
      for (const auto& candidate : static_seed_plan.candidates) {
        if (candidate.net != task.net || candidate.connection_index >= required_connections) {
          continue;
        }
        tree_connections[candidate.tree_candidate_index][candidate.connection_index] =
            seed_corridors(candidate.route);
      }
      std::optional<ResidualCorridorDemand> common;
      for (const auto& [unused_tree, connections] : tree_connections) {
        static_cast<void>(unused_tree);
        if (connections.size() != required_connections) {
          continue;
        }
        bool complete = true;
        ResidualCorridorDemand tree;
        for (std::size_t connection = 0U; connection < required_connections; ++connection) {
          const auto found = connections.find(connection);
          if (found == connections.end()) {
            complete = false;
            break;
          }
          tree.corridors.insert(found->second.corridors.begin(), found->second.corridors.end());
          tree.neck_downs.insert(found->second.neck_downs.begin(), found->second.neck_downs.end());
        }
        if (!complete) {
          continue;
        }
        if (!common) {
          common = std::move(tree);
          continue;
        }
        ResidualCorridorDemand intersection;
        std::set_intersection(common->corridors.begin(), common->corridors.end(),
                              tree.corridors.begin(), tree.corridors.end(),
                              std::inserter(intersection.corridors, intersection.corridors.end()));
        std::set_intersection(
            common->neck_downs.begin(), common->neck_downs.end(), tree.neck_downs.begin(),
            tree.neck_downs.end(),
            std::inserter(intersection.neck_downs, intersection.neck_downs.end()));
        common = std::move(intersection);
      }
      if (common && (!common->corridors.empty() || !common->neck_downs.empty())) {
        mandatory.emplace(task.net, std::move(*common));
      }
    }
    return mandatory;
  }();
  const auto static_critical_cut_summary = [&] {
    std::map<std::size_t, std::size_t> corridor_demand;
    std::map<std::size_t, std::size_t> neck_down_demand;
    for (const auto& [unused_net, demand] : residual_corridor_demand_by_net) {
      static_cast<void>(unused_net);
      for (const auto corridor : demand.corridors) {
        corridor_demand[corridor] = saturating_count_add(corridor_demand[corridor], 1U);
      }
      for (const auto corridor : demand.neck_downs) {
        neck_down_demand[corridor] = saturating_count_add(neck_down_demand[corridor], 1U);
      }
    }
    std::pair<std::size_t, std::size_t> summary;
    const auto count = [&](const auto& demand, const CorridorGraph& graph) {
      for (const auto& [corridor, reserved_nets] : demand) {
        if (critical_cut(graph, corridor, reserved_nets, config)) {
          summary.first = saturating_count_add(summary.first, 1U);
          summary.second = saturating_count_add(summary.second, reserved_nets);
        }
      }
    };
    count(corridor_demand, *corridors);
    count(neck_down_demand, corridor_neck_downs);
    return summary;
  }();
  const auto staged_seed_order = staged_iteration_zero_order(
      board, *grid, all_tasks, static_seed_plan, *corridors, *voronoi_allocation);
  std::map<std::string, RoutingSeedPriorityEvidence> seed_priority_by_group;
  for (const auto& evidence : staged_seed_order.evidence) {
    seed_priority_by_group.emplace(evidence.group, evidence);
  }
  for (std::size_t evidence_index = 0U; evidence_index < staged_seed_order.evidence.size();
       ++evidence_index) {
    const auto& evidence = staged_seed_order.evidence[evidence_index];
    const auto wave = [&]() -> std::string {
      switch (evidence.wave) {
      case IterationZeroRoutingWave::trivial:
        return "trivial";
      case IterationZeroRoutingWave::constrained:
        return "constrained";
      case IterationZeroRoutingWave::ordinary:
        return "ordinary";
      case IterationZeroRoutingWave::flexible:
        return "flexible";
      }
      return "ordinary";
    }();
    for (const auto& net : evidence.nets) {
      RouterTelemetryEvent queued;
      queued.kind = RouterTelemetryEventKind::net_queued;
      queued.net = net;
      queued.stable_id = evidence.group;
      queued.priority_stage = wave;
      queued.explicit_priority = evidence.explicit_priority;
      queued.terminal_portals = evidence.minimum_terminal_portals;
      queued.corridor_alternatives = evidence.viable_corridor_candidates;
      queued.via_transitions = evidence.primary_via_transitions;
      queued.via_alternatives = evidence.via_assignment_alternatives;
      queued.via_contention = evidence.via_site_contention;
      queued.estimated_interference = saturating_count_add(evidence.estimated_interference,
                                                           evidence.terminal_crossing_pressure);
      queued.physical_envelope_nm = evidence.physical_envelope_nm;
      queued.topology_alternatives = evidence.topology_alternatives;
      queued.layer_alternatives = evidence.layer_alternatives;
      queued.priority_ordinal = staged_seed_order.evidence.size() - evidence_index;
      queued.detail =
          "routing_freedom_score=" + std::to_string(evidence.routing_freedom_score) +
          "; terminal_crossing_pressure=" + std::to_string(evidence.terminal_crossing_pressure) +
          "; smaller nonzero freedom values are more constrained";
      emit_telemetry(std::move(queued));
    }
  }
  const auto hard_ripup_group_membership = pathfinder_group_membership(board);
  std::map<NetId, NetTask> task_by_net;
  for (const auto& task : all_tasks) {
    task_by_net.emplace(task.net, task);
  }
  ReusePlan reuse;
  if (checkpoint != nullptr) {
    reuse = prepare_reuse(*checkpoint, affected_nets, board, tasks, *grid, *corridors,
                          *voronoi_guidance, corridor_neck_downs, *voronoi_allocation, obstacles,
                          config, preserve_unaffected_provisional_congestion);
    const std::set<NetId> reused(reuse.reused_nets.begin(), reuse.reused_nets.end());
    tasks.erase(
        std::remove_if(tasks.begin(), tasks.end(),
                       [&reused](const NetTask& task) { return reused.contains(task.net); }),
        tasks.end());
  } else {
    for (const auto& task : tasks) {
      reuse.rerouted_nets.push_back(task.net);
    }
  }
  const auto checkpoint_reused_paths = reuse.paths.size();
  const auto checkpoint_reused_nets = reuse.reused_nets;
  const auto checkpoint_rerouted_nets = reuse.rerouted_nets;
  const auto checkpoint_invalidated_nets = reuse.invalidated_nets;
  std::size_t reordered_iterations = 0U;
  std::size_t reordered_nets = 0U;
  std::size_t peak_net_pressure = 0U;
  std::size_t peak_net_sampled_overflow = 0U;
  std::size_t peak_net_corridor_overflow = 0U;
  std::uint64_t peak_net_total_physical_overflow_nm = 0U;
  std::uint64_t peak_net_physical_overflow_nm = 0U;
  std::size_t peak_net_congested_physical_resources = 0U;
  std::vector<NetId> last_reroute_order;
  last_reroute_order.reserve(tasks.size());
  for (const auto& task : tasks) {
    last_reroute_order.push_back(task.net);
  }
  std::size_t selective_reroute_passes = 0U;
  std::size_t selective_full_reroute_refreshes = 0U;
  std::size_t selective_retained_net_observations = 0U;
  std::size_t selective_retained_path_observations = 0U;
  std::size_t selective_conflict_net_observations = 0U;
  std::size_t selective_coupled_group_expansions = 0U;
  std::size_t hard_capacity_passes = 0U;
  std::size_t hard_sampled_capacity_rejected_transitions = 0U;
  std::size_t hard_corridor_capacity_rejected_transitions = 0U;
  std::size_t hard_voronoi_capacity_rejected_transitions = 0U;
  std::size_t hard_capacity_diversified_passes = 0U;
  std::size_t hard_capacity_diversified_net_positions = 0U;
  std::vector<RoutingHardCapacityPass> hard_capacity_pass_summaries;
  std::map<NetId, std::size_t> hard_capacity_failure_passes_by_net;
  std::map<std::pair<NetId, NetId>, std::size_t> hard_capacity_blockers;
  std::size_t hard_capacity_ripup_attempts = 0U;
  std::size_t hard_capacity_ripped_nets = 0U;
  std::size_t hard_capacity_ripup_recoveries = 0U;
  std::vector<RoutingHardCapacityRipupAttempt> hard_capacity_ripup_records;
  std::size_t hard_capacity_conflict_set_states = 0U;
  std::size_t hard_capacity_joint_ripup_attempts = 0U;
  std::size_t hard_capacity_conflict_sets_withheld = 0U;
  std::size_t hard_capacity_failed_retry_rollbacks = 0U;
  std::size_t hard_capacity_tail_deferred_victims = 0U;
  std::size_t hard_capacity_protected_recoveries = 0U;
  std::size_t hard_capacity_protected_victim_rejections = 0U;
  std::size_t hard_capacity_blocker_reuse_invalidations = 0U;
  std::size_t global_route_beam_peak = 0U;
  std::size_t global_route_beam_candidates_retained = 0U;
  std::size_t routing_order_variants_attempted = 0U;
  std::size_t temporary_illegal_checkpoints = 0U;
  std::size_t large_neighborhood_attempts = 0U;
  std::size_t large_neighborhood_destroyed_nets = 0U;
  std::size_t large_neighborhood_coupled_expansions = 0U;
  std::vector<std::string> large_neighborhood_operators;
  std::size_t voronoi_lane_conflict_passes = 0U;
  std::size_t voronoi_lane_conflict_observations = 0U;
  std::size_t voronoi_lane_conflict_history_increments = 0U;
  std::size_t voronoi_lane_conflict_history_withheld = 0U;
  std::size_t voronoi_lane_conflict_scheduled_reroute_nets = 0U;
  std::size_t voronoi_lane_conflict_coupled_group_expansions = 0U;
  std::size_t iteration_zero_paths = 0U;
  std::size_t iteration_zero_unrouted_nets = 0U;
  std::size_t iteration_zero_total_overflow = 0U;
  std::size_t iteration_zero_sampled_overflow = 0U;
  std::size_t iteration_zero_corridor_overflow = 0U;
  std::size_t iteration_zero_portfolios_evaluated = 0U;
  std::size_t iteration_zero_candidate_index = 0U;
  std::size_t iteration_zero_seed_history_increments = 0U;
  bool iteration_zero_occupancy_aware_portfolio_selected = false;
  bool iteration_zero_staged_priority_portfolio_selected = false;
  bool iteration_zero_staged_priority_qualified = false;
  std::size_t iteration_zero_baseline_unrouted_nets = 0U;
  std::size_t iteration_zero_staged_unrouted_nets = 0U;
  std::size_t iteration_zero_baseline_total_overflow = 0U;
  std::size_t iteration_zero_staged_total_overflow = 0U;
  std::size_t iteration_zero_baseline_clearance_shadow_conflicts = 0U;
  std::size_t iteration_zero_staged_clearance_shadow_conflicts = 0U;
  bool clearance_shadow_owner_history_phase = false;
  std::size_t clearance_shadow_owner_history_passes = 0U;
  std::size_t bidirectional_route_growth_attempts = 0U;
  std::size_t bidirectional_route_growth_meeting_nodes = 0U;
  std::size_t bidirectional_route_growth_accepted = 0U;
  std::size_t bidirectional_route_growth_fallbacks = 0U;
  std::size_t critical_cut_penalty_events = 0U;
  std::uint64_t critical_cut_penalty_total = 0U;
  // These diagnostics are intentionally run-wide rather than checkpoint-local. Former-route
  // memory is ephemeral by design, and a feasibility-first checkpoint can legitimately predate
  // the hard pass that exercised it.
  std::size_t former_route_memory_ripped_paths = 0U;
  std::size_t former_route_memory_edges_peak = 0U;
  std::size_t former_route_memory_penalty_events = 0U;
  std::uint64_t former_route_memory_penalty = 0U;
  RoutingHistory history = prepared_routability_history.history;
  const auto finalize = [&](GlobalRouteResult result) {
    result.configured_selected_nets = config.selected_nets;
    result.configured_priority_nets = config.priority_nets;
    result.configured_grid_pitch = config.grid_pitch;
    result.configured_via_site_pitch = config.via_site_pitch;
    result.configured_via_site_phase_x = config.via_site_phase_x;
    result.configured_via_site_phase_y = config.via_site_phase_y;
    result.configured_diagonal_grid_edges = config.allow_diagonal_grid_edges;
    result.configured_crossing_safe_diagonal_grid_edges =
        config.allow_crossing_safe_diagonal_grid_edges;
    result.configured_sampled_route_clearance_reservations = config.reserve_sampled_route_clearance;
    result.configured_max_clearance_shadow_conflict_resources =
        config.max_clearance_shadow_conflict_resources;
    result.configured_clearance_shadow_cost_quantum = config.clearance_shadow_cost_quantum;
    result.routability_history_entries = prepared_routability_history.entries;
    result.routability_history_edges = prepared_routability_history.edges;
    result.routability_history_pressure = prepared_routability_history.pressure;
    result.routability_history_budget_exhausted = prepared_routability_history.budget_exhausted;
    result.clearance_shadow_owner_history_phase_attempted = clearance_shadow_owner_history_phase;
    result.clearance_shadow_owner_history_passes = clearance_shadow_owner_history_passes;
    result.configured_single_lane_direct_voronoi_resources =
        config.single_lane_direct_voronoi_resources;
    result.configured_max_static_seed_candidates_per_connection =
        config.max_static_seed_candidates_per_connection;
    result.configured_max_static_seed_searches = config.max_static_seed_searches;
    result.configured_max_static_seed_reprojection_searches =
        config.max_static_seed_reprojection_searches;
    result.configured_max_static_seed_workers = config.max_static_seed_workers;
    result.configured_static_seed_alternative_edge_penalty =
        config.static_seed_alternative_edge_penalty;
    result.configured_static_seed_corridor_departure_penalty =
        config.static_seed_corridor_departure_penalty;
    result.configured_residual_corridor_demand_cost = config.residual_corridor_demand_cost;
    result.configured_critical_cut_cost = config.critical_cut_cost;
    result.configured_critical_cut_minimum_future_demand =
        config.critical_cut_minimum_future_demand;
    result.configured_river_following_cost = config.river_following_cost;
    result.configured_former_route_memory_cost = config.former_route_memory_cost;
    result.configured_terminal_escape_reservation_cost = config.terminal_escape_reservation_cost;
    result.configured_max_terminal_escape_reservation_resources =
        config.max_terminal_escape_reservation_resources;
    result.former_route_memory_ripped_paths = former_route_memory_ripped_paths;
    result.former_route_memory_edges = former_route_memory_edges_peak;
    result.former_route_memory_penalty_events = former_route_memory_penalty_events;
    result.former_route_memory_penalty = former_route_memory_penalty;
    result.residual_corridor_demand_nets = residual_corridor_demand_by_net.size();
    for (const auto& [unused_net, demand] : residual_corridor_demand_by_net) {
      static_cast<void>(unused_net);
      result.residual_corridor_demand_resources = saturating_count_add(
          result.residual_corridor_demand_resources,
          saturating_count_add(demand.corridors.size(), demand.neck_downs.size()));
    }
    result.critical_cut_resources = static_critical_cut_summary.first;
    result.critical_cut_reserved_demand = static_critical_cut_summary.second;
    result.critical_cut_penalty_events = critical_cut_penalty_events;
    result.critical_cut_penalty = critical_cut_penalty_total;
    result.configured_global_spline_tessellation_tolerance =
        config.global_spline_tessellation_tolerance;
    result.configured_global_spline_control_corridor = config.global_spline_control_corridor;
    result.configured_global_spline_shove_step = config.global_spline_shove_step;
    result.configured_navmesh_funnel_spline_seeds = config.use_navmesh_funnel_spline_seeds;
    result.configured_global_navmesh_cell_pitch = config.global_navmesh_cell_pitch;
    result.configured_global_spline_minimum_bend_radius = config.global_spline_minimum_bend_radius;
    result.configured_max_global_navmesh_cells_per_seed = config.max_global_navmesh_cells_per_seed;
    result.configured_max_global_navmesh_search_expansions_per_seed =
        config.max_global_navmesh_search_expansions_per_seed;
    result.configured_global_spline_trajectory_samples = config.global_spline_trajectory_samples;
    result.configured_max_global_spline_trajectory_iterations =
        config.max_global_spline_trajectory_iterations;
    result.configured_max_global_spline_trajectory_candidate_checks =
        config.max_global_spline_trajectory_candidate_checks;
    result.configured_global_spline_max_tessellation_depth =
        config.global_spline_max_tessellation_depth;
    result.configured_max_global_spline_samples_per_seed =
        config.max_global_spline_samples_per_seed;
    result.configured_max_global_spline_shove_iterations =
        config.max_global_spline_shove_iterations;
    result.configured_max_static_seed_corridor_cells = config.max_static_seed_corridor_cells;
    result.configured_max_global_pns_propagation_depth = config.max_global_pns_propagation_depth;
    result.configured_max_global_pns_candidate_checks = config.max_global_pns_candidate_checks;
    result.configured_maximum_global_pns_displacement = config.maximum_global_pns_displacement;
    result.static_seed_connections = static_seed_plan.stats.connections;
    result.static_seed_tasks_dispatched = static_seed_plan.stats.tasks_dispatched;
    result.static_seed_workers_used = static_seed_plan.stats.workers_used;
    result.static_seed_searches = static_seed_plan.stats.searches;
    result.static_seed_candidates = static_seed_plan.stats.candidates;
    result.static_seed_search_expansions = static_seed_plan.stats.search_expansions;
    result.static_seed_penalty_reruns = static_seed_plan.stats.penalty_reruns;
    result.static_seed_deduplicated_candidates = static_seed_plan.stats.deduplicated_candidates;
    result.static_seed_via_bearing_candidates = static_seed_plan.stats.via_bearing_candidates;
    result.static_seed_direct_medial_candidates = static_seed_plan.stats.direct_medial_candidates;
    result.static_seed_terminals_without_seed = static_seed_plan.stats.terminals_without_seed;
    result.static_seed_distinct_layer_sequences = static_seed_plan.stats.distinct_layer_sequences;
    result.static_seed_distinct_topology_signatures =
        static_seed_plan.stats.distinct_topology_signatures;
    result.static_seed_corridor_nodes = static_seed_plan.stats.corridor_nodes;
    result.static_seed_spline_samples = static_seed_plan.stats.spline_samples;
    result.static_seed_navmesh_domains = static_seed_plan.stats.navmesh_domains;
    result.static_seed_navmesh_routes = static_seed_plan.stats.navmesh_routes;
    result.static_seed_navmesh_cells = static_seed_plan.stats.navmesh_cells;
    result.static_seed_navmesh_search_expansions = static_seed_plan.stats.navmesh_search_expansions;
    result.static_seed_flood_fill_routes = static_seed_plan.stats.flood_fill_routes;
    result.static_seed_flood_fill_cells_visited = static_seed_plan.stats.flood_fill_cells_visited;
    result.static_seed_navmesh_failures = static_seed_plan.stats.navmesh_failures;
    result.static_seed_navmesh_certificate_domains =
        static_seed_plan.stats.navmesh_certificate_domains;
    result.static_seed_funnel_points = static_seed_plan.stats.funnel_points;
    result.static_seed_trajectory_attempts = static_seed_plan.stats.trajectory_attempts;
    result.static_seed_trajectory_accepted = static_seed_plan.stats.trajectory_accepted;
    result.static_seed_trajectory_control_moves = static_seed_plan.stats.trajectory_control_moves;
    result.static_seed_trajectory_candidate_checks =
        static_seed_plan.stats.trajectory_candidate_checks;
    result.static_seed_trajectory_rejected = static_seed_plan.stats.trajectory_rejected;
    result.static_seed_collision_envelopes = static_seed_plan.stats.collision_envelopes;
    result.static_seed_control_shoves = static_seed_plan.stats.control_shoves;
    result.static_seed_refits = static_seed_plan.stats.refits;
    result.static_seed_refined_candidates = static_seed_plan.stats.refined_candidates;
    result.static_seed_reprojection_searches = static_seed_plan.stats.reprojection_searches;
    result.static_seed_reprojections_accepted = static_seed_plan.stats.reprojections_accepted;
    result.static_seed_reprojections_reverted = static_seed_plan.stats.reprojections_reverted;
    result.static_seed_corridor_departures = static_seed_plan.stats.corridor_departures;
    result.static_seed_complete_trees = static_seed_plan.stats.complete_trees;
    result.static_seed_partial_trees = static_seed_plan.stats.partial_trees;
    result.static_seed_alternative_trees = static_seed_plan.stats.alternative_trees;
    result.static_seed_tree_joins = static_seed_plan.stats.tree_joins;
    result.static_seed_growth_reorderings = static_seed_plan.stats.growth_reorderings;
    result.static_seed_joint_groups = static_seed_plan.stats.joint_groups;
    result.static_seed_joint_group_candidates = static_seed_plan.stats.joint_group_candidates;
    result.static_seed_joint_groups_withheld = static_seed_plan.stats.joint_groups_withheld;
    result.static_seed_budget_exhausted = static_seed_plan.stats.budget_exhausted;
    result.global_pns_proposal_attempts = static_seed_plan.stats.pns_proposal_attempts;
    result.global_pns_eligible_collisions = static_seed_plan.stats.pns_eligible_collisions;
    result.global_pns_ineligible_collisions = static_seed_plan.stats.pns_ineligible_collisions;
    result.global_pns_candidate_checks = static_seed_plan.stats.pns_candidate_checks;
    result.global_pns_local_splits = static_seed_plan.stats.pns_local_splits;
    result.global_pns_proposals_accepted = result.selected_shove_proposals.size();
    result.iteration_zero_paths = iteration_zero_paths;
    result.iteration_zero_unrouted_nets = iteration_zero_unrouted_nets;
    result.iteration_zero_total_overflow = iteration_zero_total_overflow;
    result.iteration_zero_sampled_overflow = iteration_zero_sampled_overflow;
    result.iteration_zero_corridor_overflow = iteration_zero_corridor_overflow;
    result.iteration_zero_portfolios_evaluated = iteration_zero_portfolios_evaluated;
    result.iteration_zero_candidate_index = iteration_zero_candidate_index;
    result.iteration_zero_seed_history_increments = iteration_zero_seed_history_increments;
    result.iteration_zero_selected = result.selected_iteration == 0U && !result.paths.empty();
    result.iteration_zero_occupancy_aware_portfolio_selected =
        iteration_zero_occupancy_aware_portfolio_selected;
    result.iteration_zero_staged_priority_portfolio_selected =
        iteration_zero_staged_priority_portfolio_selected;
    result.iteration_zero_staged_priority_qualified = iteration_zero_staged_priority_qualified;
    result.iteration_zero_baseline_unrouted_nets = iteration_zero_baseline_unrouted_nets;
    result.iteration_zero_staged_unrouted_nets = iteration_zero_staged_unrouted_nets;
    result.iteration_zero_baseline_total_overflow = iteration_zero_baseline_total_overflow;
    result.iteration_zero_staged_total_overflow = iteration_zero_staged_total_overflow;
    result.iteration_zero_baseline_clearance_shadow_conflicts =
        iteration_zero_baseline_clearance_shadow_conflicts;
    result.iteration_zero_staged_clearance_shadow_conflicts =
        iteration_zero_staged_clearance_shadow_conflicts;
    result.iteration_zero_seed_priority_evidence = staged_seed_order.evidence;
    for (const auto& evidence : result.iteration_zero_seed_priority_evidence) {
      switch (evidence.wave) {
      case IterationZeroRoutingWave::trivial:
        ++result.iteration_zero_trivial_groups;
        break;
      case IterationZeroRoutingWave::constrained:
        ++result.iteration_zero_constrained_groups;
        break;
      case IterationZeroRoutingWave::ordinary:
        ++result.iteration_zero_ordinary_groups;
        break;
      case IterationZeroRoutingWave::flexible:
        ++result.iteration_zero_flexible_groups;
        break;
      }
    }
    result.configured_local_pin_escape_pitch = config.local_pin_escape_pitch;
    result.configured_max_local_pin_escape_expansions_per_terminal =
        config.max_local_pin_escape_expansions_per_terminal;
    result.local_pin_escape_paths = pin_access.local_paths;
    result.local_pin_escape_expansions = pin_access.local_expansions;
    result.terminals_with_local_pin_escape = pin_access.terminals_with_local_access;
    result.same_net_terminal_fanout_paths = pin_access.same_net_fanout_paths;
    result.terminals_with_same_net_fanout = pin_access.terminals_with_same_net_fanout;
    result.configured_contended_terminal_fanout = config.allocate_contended_terminal_fanout;
    result.contended_terminals = pin_access.contended_terminals;
    result.contention_triggered_local_paths = pin_access.contention_triggered_local_paths;
    result.voronoi_corridor_evidence = voronoi_evidence.edges;
    result.voronoi_domains_attempted = voronoi_evidence.domains_attempted;
    result.voronoi_domains_accepted = voronoi_evidence.domains_accepted;
    result.voronoi_domains_rejected = voronoi_evidence.domains_rejected;
    result.voronoi_conservative_obstacles = voronoi_evidence.conservative_obstacles;
    result.voronoi_evidence_curved_edge_chords = voronoi_evidence.accepted_curved_edge_chords;
    result.voronoi_withheld_curved_edges = voronoi_evidence.withheld_curved_edges;
    result.voronoi_guided_nets = voronoi_guidance->distance_steps_by_net.size();
    result.voronoi_guidance_seed_nodes = voronoi_guidance->seed_nodes;
    result.direct_voronoi_edges_available = voronoi_allocation->direct_resource_by_edge.size();
    result.direct_voronoi_edges_withheld = voronoi_allocation->withheld_direct_edges;
    result.configured_max_voronoi_resource_segment_length =
        config.max_voronoi_resource_segment_length;
    result.segmented_voronoi_source_resources = voronoi_allocation->segmented_source_resources;
    result.additional_voronoi_segment_resources = voronoi_allocation->additional_segment_resources;
    result.direct_voronoi_resources_retained_atomic =
        voronoi_allocation->direct_resources_retained_atomic;
    result.multisource_steiner_growth_enabled = config.use_multisource_steiner_growth;
    result.bidirectional_route_growth_enabled = config.use_bidirectional_route_growth;
    result.configured_bidirectional_route_growth_start_iteration =
        config.bidirectional_route_growth_start_iteration;
    result.configured_max_bidirectional_meeting_nodes = config.max_bidirectional_meeting_nodes;
    result.configured_max_bidirectional_meeting_expansions =
        config.max_bidirectional_meeting_expansions;
    result.bidirectional_route_growth_attempts = bidirectional_route_growth_attempts;
    result.bidirectional_route_growth_meeting_nodes = bidirectional_route_growth_meeting_nodes;
    result.bidirectional_route_growth_accepted = bidirectional_route_growth_accepted;
    result.bidirectional_route_growth_fallbacks = bidirectional_route_growth_fallbacks;
    result.atomic_complete_net_routing_enabled = config.atomic_complete_net_routing;
    result.configured_selective_congestion_reroute = config.selective_congestion_reroute;
    result.configured_selective_full_reroute_interval = config.selective_full_reroute_interval;
    result.configured_hard_capacity_start_iteration = config.hard_capacity_start_iteration;
    result.hard_capacity_passes = hard_capacity_passes;
    result.hard_sampled_capacity_rejected_transitions = hard_sampled_capacity_rejected_transitions;
    result.hard_corridor_capacity_rejected_transitions =
        hard_corridor_capacity_rejected_transitions;
    result.hard_voronoi_capacity_rejected_transitions = hard_voronoi_capacity_rejected_transitions;
    result.hard_capacity_diversified_passes = hard_capacity_diversified_passes;
    result.hard_capacity_diversified_net_positions = hard_capacity_diversified_net_positions;
    result.hard_capacity_pass_summaries = hard_capacity_pass_summaries;
    result.hard_capacity_net_failures.clear();
    result.hard_capacity_net_failures.reserve(hard_capacity_failure_passes_by_net.size());
    for (const auto& [net, failed_passes] : hard_capacity_failure_passes_by_net) {
      result.hard_capacity_net_failures.push_back(
          RoutingHardCapacityNetFailure{net, failed_passes});
    }
    result.hard_capacity_blockers.clear();
    result.hard_capacity_blockers.reserve(hard_capacity_blockers.size());
    for (const auto& [relation, rejected_transitions] : hard_capacity_blockers) {
      result.hard_capacity_blockers.push_back(
          RoutingHardCapacityBlocker{relation.first, relation.second, rejected_transitions});
    }
    result.configured_max_hard_capacity_ripup_attempts_per_pass =
        config.max_hard_capacity_ripup_attempts_per_pass;
    result.configured_max_hard_capacity_ripup_nets_per_attempt =
        config.max_hard_capacity_ripup_nets_per_attempt;
    result.hard_capacity_ripup_attempts = hard_capacity_ripup_attempts;
    result.hard_capacity_ripped_nets = hard_capacity_ripped_nets;
    result.hard_capacity_ripup_recoveries = hard_capacity_ripup_recoveries;
    result.hard_capacity_ripup_retained_recoveries = static_cast<std::size_t>(
        std::count_if(hard_capacity_ripup_records.begin(), hard_capacity_ripup_records.end(),
                      [](const RoutingHardCapacityRipupAttempt& attempt) {
                        return attempt.retry_completed && attempt.retained_at_pass_end;
                      }));
    result.hard_capacity_ripup_records = hard_capacity_ripup_records;
    result.configured_max_hard_capacity_conflict_set_states_per_attempt =
        config.max_hard_capacity_conflict_set_states_per_attempt;
    result.hard_capacity_conflict_set_states = hard_capacity_conflict_set_states;
    result.hard_capacity_joint_ripup_attempts = hard_capacity_joint_ripup_attempts;
    result.hard_capacity_conflict_sets_withheld = hard_capacity_conflict_sets_withheld;
    result.configured_rollback_failed_hard_capacity_ripup =
        config.rollback_failed_hard_capacity_ripup;
    result.hard_capacity_failed_retry_rollbacks = hard_capacity_failed_retry_rollbacks;
    result.configured_defer_hard_capacity_ripup_victims_to_pass_tail =
        config.defer_hard_capacity_ripup_victims_to_pass_tail;
    result.hard_capacity_tail_deferred_victims = hard_capacity_tail_deferred_victims;
    result.configured_protect_recovered_hard_capacity_ripup_nets =
        config.protect_recovered_hard_capacity_ripup_nets;
    result.hard_capacity_protected_recoveries = hard_capacity_protected_recoveries;
    result.hard_capacity_protected_victim_rejections = hard_capacity_protected_victim_rejections;
    result.configured_max_hard_capacity_blocker_reuse_invalidations_per_failed_net =
        config.max_hard_capacity_blocker_reuse_invalidations_per_failed_net;
    result.hard_capacity_blocker_reuse_invalidations = hard_capacity_blocker_reuse_invalidations;
    result.selective_reroute_passes = selective_reroute_passes;
    result.selective_full_reroute_refreshes = selective_full_reroute_refreshes;
    result.selective_retained_net_observations = selective_retained_net_observations;
    result.selective_retained_path_observations = selective_retained_path_observations;
    result.selective_conflict_net_observations = selective_conflict_net_observations;
    result.selective_coupled_group_expansions = selective_coupled_group_expansions;
    result.configured_max_expansions_per_connection = config.max_expansions_per_connection;
    result.configured_diversify_pin_access_rings = config.diversify_pin_access_rings;
    result.diversified_pin_access_nodes = pin_access.diversified_nodes;
    result.terminals_with_diversified_pin_access = pin_access.terminals_with_diversified_access;
    result.additional_successful_pin_access_rings = pin_access.additional_successful_rings;
    result.configured_expansion_retry_heuristic_weight_per_mille =
        config.expansion_retry_heuristic_weight_per_mille;
    result.configured_reuse_weighted_search_frontier = config.reuse_weighted_search_frontier;
    result.configured_checkpoint_objective = config.checkpoint_objective;
    result.configured_scale_voronoi_history_by_overflow = config.scale_voronoi_history_by_overflow;
    result.voronoi_history_quantum_nm = voronoi_allocation->minimum_demand_nm;
    result.configured_present_congestion_multiplier_per_mille =
        config.present_congestion_multiplier_per_mille;
    result.configured_max_present_congestion_scale_per_mille =
        config.max_present_congestion_scale_per_mille;
    result.configured_present_congestion_escalation_start_iteration =
        config.present_congestion_escalation_start_iteration;
    result.configured_terminal_neck_down_width = config.terminal_neck_down_width;
    result.configured_max_terminal_neck_down_length = config.max_terminal_neck_down_length;
    result.configured_medial_neck_down_width = config.medial_neck_down_width;
    result.configured_max_medial_neck_down_length = config.max_medial_neck_down_length;
    result.configured_corridor_neck_down_width = config.corridor_neck_down_width;
    result.configured_max_corridor_neck_down_length = config.max_corridor_neck_down_length;
    result.configured_width_reduction_cost_per_grid_step =
        config.width_reduction_cost_per_grid_step;
    result.configured_negotiate_voronoi_lane_conflicts = config.negotiate_voronoi_lane_conflicts;
    result.configured_voronoi_lane_conflict_history_cost =
        config.voronoi_lane_conflict_history_cost;
    result.configured_max_voronoi_lane_conflict_history_entries =
        config.max_voronoi_lane_conflict_history_entries;
    result.voronoi_lane_conflict_passes = voronoi_lane_conflict_passes;
    result.voronoi_lane_conflict_observations = voronoi_lane_conflict_observations;
    result.voronoi_lane_conflict_history_entries = saturating_count_add(
        saturating_count_add(
            saturating_count_add(history.direct_portals.size(), history.exact_junctions.size()),
            history.selected_junction_edges.size()),
        history.lane_conflict_resources.size());
    result.voronoi_exact_junction_conflict_history_entries = history.exact_junctions.size();
    result.voronoi_selected_junction_edge_conflict_history_entries =
        history.selected_junction_edges.size();
    result.voronoi_lane_conflict_resource_history_entries = history.lane_conflict_resources.size();
    result.voronoi_lane_conflict_history_increments = voronoi_lane_conflict_history_increments;
    result.voronoi_lane_conflict_history_withheld = voronoi_lane_conflict_history_withheld;
    result.voronoi_lane_conflict_scheduled_reroute_nets =
        voronoi_lane_conflict_scheduled_reroute_nets;
    result.voronoi_lane_conflict_coupled_group_expansions =
        voronoi_lane_conflict_coupled_group_expansions;
    result.coupled_nets_ineligible_for_corridor_neck_down =
        coupled_nets_ineligible_for_corridor_neck_down;
    if (!corridor_neck_downs.corridors.empty()) {
      const auto node_count = grid->layer_size() * grid->layers.size();
      const auto max_steps = static_cast<std::size_t>(config.max_corridor_neck_down_length.value() /
                                                      config.grid_pitch.value());
      result.corridor_neck_down_search_states = node_count * (max_steps + 1U);
    }
    result.medial_neck_down_edges_available = 0U;
    for (const auto& resource : voronoi_allocation->resources) {
      for (const auto& [net, width] : voronoi_allocation->medial_neck_down_width_by_net) {
        const auto selected = voronoi_allocation->medial_neck_down_width(resource.id, net);
        if (!selected || !resource.direct_edge) {
          continue;
        }
        const auto [first_x, first_y, first_layer] = grid->decode(resource.direct_edge->first);
        const auto [second_x, second_y, second_layer] = grid->decode(resource.direct_edge->second);
        const auto first = grid->point(first_x, first_y);
        const auto second = grid->point(second_x, second_y);
        const auto rules = resolve_net_routing_rules(board, net);
        const auto clearance = static_cast<long double>(rules.clearance.value());
        const auto nominal_radius = static_cast<long double>(rules.track_width.value()) / 2.0L;
        const auto narrow_radius = static_cast<long double>(width.value()) / 2.0L;
        if (first_layer == second_layer &&
            obstacles.static_segment_free(net, first_layer, first, first, nominal_radius,
                                          clearance) &&
            obstacles.static_segment_free(net, second_layer, second, second, nominal_radius,
                                          clearance) &&
            obstacles.static_segment_free(net, first_layer, first, second, narrow_radius,
                                          clearance)) {
          ++result.medial_neck_down_edges_available;
        }
      }
    }
    result.pathfinder_reordered_iterations = reordered_iterations;
    result.pathfinder_reordered_nets = reordered_nets;
    result.pathfinder_peak_net_pressure = peak_net_pressure;
    result.pathfinder_peak_net_sampled_overflow = peak_net_sampled_overflow;
    result.pathfinder_peak_net_corridor_overflow = peak_net_corridor_overflow;
    result.pathfinder_peak_net_total_physical_overflow_nm = peak_net_total_physical_overflow_nm;
    result.pathfinder_peak_net_physical_overflow_nm = peak_net_physical_overflow_nm;
    result.pathfinder_peak_net_congested_physical_resources = peak_net_congested_physical_resources;
    result.selected_net_pressure = collect_global_route_net_pressure(result);
    result.configured_physical_envelope_ordering = config.order_routing_groups_by_physical_envelope;
    result.peak_net_routing_envelope_nm = physical_envelope_metrics.peak_net_nm;
    result.peak_routing_group_envelope_nm = physical_envelope_metrics.peak_group_nm;
    result.last_pathfinder_reroute_order = last_reroute_order;
    result.configured_search_diversification = config.enable_search_diversification;
    result.configured_global_route_beam_width = config.global_route_beam_width;
    result.configured_max_large_neighborhood_attempts = config.max_large_neighborhood_attempts;
    result.configured_large_neighborhood_stagnation_iterations =
        config.large_neighborhood_stagnation_iterations;
    result.configured_destroy_route_percent = config.destroy_route_percent;
    result.configured_max_routing_order_variants = config.max_routing_order_variants;
    result.configured_routing_order_diversification_seed =
        config.routing_order_diversification_seed;
    result.global_route_beam_peak = global_route_beam_peak;
    result.global_route_beam_candidates_retained = global_route_beam_candidates_retained;
    result.routing_order_variants_attempted = routing_order_variants_attempted;
    result.temporary_illegal_checkpoints = temporary_illegal_checkpoints;
    result.large_neighborhood_attempts = large_neighborhood_attempts;
    result.large_neighborhood_destroyed_nets = large_neighborhood_destroyed_nets;
    result.large_neighborhood_coupled_expansions = large_neighborhood_coupled_expansions;
    result.large_neighborhood_operators = large_neighborhood_operators;
    obstacles.populate_feedback(
        result, pin_access.failed, pin_access.failures, pin_access.nodes, pin_access.adaptive_nodes,
        pin_access.terminals_with_adaptive_access, pin_access.isolated_nodes_rejected,
        pin_access.terminals_with_isolated_rejections, pin_access.neck_down_nodes,
        pin_access.terminals_with_neck_down_access,
        pin_access.coupled_terminals_ineligible_for_neck_down);
    // These selected-path nominal-width probes are diagnostic evidence, not search attempts.
    // Populate ordinary search feedback first so the probes cannot inflate blocked-edge pressure.
    populate_neck_down_obstacle_impacts(result, board, *grid, obstacles);
    const auto [edge_cache_entries, access_cache_entries] = obstacles.query_cache_entry_counts();
    RouterTelemetryEvent cache_release_started;
    cache_release_started.kind = RouterTelemetryEventKind::search_summary;
    cache_release_started.stable_id = "global-obstacle-query-cache-release";
    cache_release_started.detail =
        "global route selected; releasing obstacle query caches: edge_entries=" +
        std::to_string(edge_cache_entries) +
        ", access_entries=" + std::to_string(access_cache_entries);
    emit_telemetry(std::move(cache_release_started));
    obstacles.release_query_caches();
    RouterTelemetryEvent cache_release_completed;
    cache_release_completed.kind = RouterTelemetryEventKind::search_summary;
    cache_release_completed.stable_id = "global-obstacle-query-cache-release";
    cache_release_completed.detail = "global obstacle query caches released";
    emit_telemetry(std::move(cache_release_completed));
    const auto failure_reason_name = [](RoutingConnectionFailureReason reason) {
      switch (reason) {
      case RoutingConnectionFailureReason::no_access_nodes:
        return std::string_view{"no_access_nodes"};
      case RoutingConnectionFailureReason::search_frontier_exhausted:
        return std::string_view{"search_frontier_exhausted"};
      case RoutingConnectionFailureReason::expansion_limit:
        return std::string_view{"expansion_limit"};
      case RoutingConnectionFailureReason::unmatched_multisource_terminal:
        return std::string_view{"unmatched_multisource_terminal"};
      }
      return std::string_view{"unknown"};
    };
    for (std::size_t failure_index = 0U; failure_index < result.pin_escape_failures.size();
         ++failure_index) {
      const auto& failure = result.pin_escape_failures[failure_index];
      RouterTelemetryEvent failed;
      failed.kind = RouterTelemetryEventKind::net_failed;
      failed.stable_id =
          "pin-escape-failure:" + failure.net.value() + ':' + std::to_string(failure_index);
      failed.net = failure.net;
      failed.route_revision = result.selected_iteration;
      failed.rejection_reason = "no_access_nodes";
      failed.x_nm = failure.position.x.value();
      failed.y_nm = failure.position.y.value();
      failed.detail =
          "selected global terminal has no legal pin escape: footprint=" +
          failure.footprint.value() + ", pad=" + failure.pad.value() +
          ", x_nm=" + std::to_string(failure.position.x.value()) +
          ", y_nm=" + std::to_string(failure.position.y.value()) +
          ", nominal_center_blockers=" + std::to_string(failure.nominal_center_blockers.size()) +
          ", neck_down_center_blockers=" +
          std::to_string(failure.neck_down_center_blockers.size()) +
          ", nominal_center_board_blocked=" +
          (failure.nominal_center_board_blocked ? "yes" : "no") +
          ", neck_down_center_board_blocked=" +
          (failure.neck_down_center_board_blocked ? "yes" : "no");
      emit_telemetry(std::move(failed));
    }
    for (std::size_t failure_index = 0U; failure_index < result.connection_failures.size();
         ++failure_index) {
      const auto& failure = result.connection_failures[failure_index];
      RouterTelemetryEvent failed;
      failed.kind = RouterTelemetryEventKind::net_failed;
      failed.stable_id =
          "global-failure:" + failure.net.value() + ':' + std::to_string(failure_index);
      failed.net = failure.net;
      failed.route_revision = result.selected_iteration;
      failed.search_expansions = failure.expansions;
      failed.rejection_reason = std::string(failure_reason_name(failure.reason));
      if (!failure.attempted_terminals.empty()) {
        failed.x_nm = failure.attempted_terminals.front().position.x.value();
        failed.y_nm = failure.attempted_terminals.front().position.y.value();
      }
      failed.detail =
          "selected global connection failed: reason=" + failed.rejection_reason +
          ", completed_connections=" + std::to_string(failure.completed_connections) +
          ", attempted_terminals=" + std::to_string(failure.attempted_terminals.size()) +
          ", connected_terminals=" + std::to_string(failure.connected_terminals.size()) +
          ", expansions=" + std::to_string(failure.expansions) +
          ", topology_rejected_edges=" + std::to_string(failure.topology_rejected_edges) +
          ", multisource=" + (failure.multisource ? "yes" : "no") +
          ", direct_fallback=" + (failure.direct_edge_fallback_attempted ? "yes" : "no");
      if (!failure.attempted_terminals.empty()) {
        const auto& terminal = failure.attempted_terminals.front();
        failed.detail += ", attempted_footprint=" + terminal.footprint.value() +
                         ", attempted_pad=" + terminal.pad.value() +
                         ", attempted_x_nm=" + std::to_string(terminal.position.x.value()) +
                         ", attempted_y_nm=" + std::to_string(terminal.position.y.value());
      }
      emit_telemetry(std::move(failed));
    }
    if (result.critical_cut_resources > 0U || result.critical_cut_penalty_events > 0U) {
      RouterTelemetryEvent critical_cuts;
      critical_cuts.kind = RouterTelemetryEventKind::search_summary;
      critical_cuts.stable_id = "critical-corridor-cuts";
      critical_cuts.route_revision = result.selected_iteration;
      critical_cuts.detail =
          "critical corridor cuts: resources=" + std::to_string(result.critical_cut_resources) +
          ", reserved_future_lanes=" + std::to_string(result.critical_cut_reserved_demand) +
          ", selected_optional_cut_uses=" + std::to_string(result.critical_cut_penalty_events) +
          ", penalty=" + std::to_string(result.critical_cut_penalty);
      emit_telemetry(std::move(critical_cuts));
    }
    for (const auto& path : result.paths) {
      RouterTelemetryEvent route;
      route.kind = RouterTelemetryEventKind::route_candidate_selected;
      route.stable_id =
          "global-route:" + path.net.value() + ':' + std::to_string(path.connection_index);
      route.net = path.net;
      route.route_revision = result.selected_iteration;
      const auto rules = resolve_net_routing_rules(board, path.net);
      route.route_width_nm =
          static_cast<std::uint64_t>(std::max<std::int64_t>(0, rules.track_width.value()));
      route.route_points.reserve(path.points.size());
      for (const auto& point : path.points) {
        route.route_points.push_back(RouterTelemetryPathPoint{
            point.position.x.value(), point.position.y.value(), point.layer});
      }
      route.detail = "selected global polyline; planning geometry, not committed copper";
      emit_telemetry(std::move(route));
    }
    RouterTelemetryEvent finished;
    finished.kind = RouterTelemetryEventKind::routing_finished;
    finished.iteration = result.selected_iteration;
    finished.total_nets = tasks.size();
    finished.routed_nets = tasks.size() >= result.unrouted_nets.size()
                               ? tasks.size() - result.unrouted_nets.size()
                               : 0U;
    finished.unrouted_nets = result.unrouted_nets.size();
    finished.sampled_overflow = result.sampled_overflow;
    finished.corridor_overflow = result.corridor_overflow;
    finished.physical_overflow_nm = result.total_voronoi_overflow_nm;
    finished.clearance_shadow_conflicts = result.clearance_shadow_conflicts.size();
    finished.ripup_count = result.hard_capacity_ripped_nets;
    finished.route_cost = result.total_cost;
    finished.search_expansions = result.path_search_expansions;
    finished.detail = result.diagnostic;
    emit_telemetry(std::move(finished));
    return result;
  };

  GlobalRouteResult best;
  struct BeamCheckpoint {
    std::uint64_t signature{0};
    GlobalRouteResult result;
  };
  std::vector<BeamCheckpoint> checkpoint_beam;
  const auto retain_checkpoint = [&](const GlobalRouteResult& candidate) {
    if (!config.enable_search_diversification || candidate.paths.empty()) {
      return;
    }
    const auto signature = global_checkpoint_topology_signature(candidate);
    if (std::any_of(checkpoint_beam.begin(), checkpoint_beam.end(),
                    [&](const auto& retained) { return retained.signature == signature; })) {
      return;
    }
    checkpoint_beam.push_back(BeamCheckpoint{signature, candidate});
    ++global_route_beam_candidates_retained;
    std::stable_sort(
        checkpoint_beam.begin(), checkpoint_beam.end(), [&](const auto& lhs, const auto& rhs) {
          if (global_route_checkpoint_better(lhs.result, rhs.result, config.checkpoint_objective)) {
            return true;
          }
          if (global_route_checkpoint_better(rhs.result, lhs.result, config.checkpoint_objective)) {
            return false;
          }
          return lhs.signature < rhs.signature;
        });
    if (checkpoint_beam.size() > config.global_route_beam_width) {
      checkpoint_beam.resize(config.global_route_beam_width);
    }
    global_route_beam_peak = std::max(global_route_beam_peak, checkpoint_beam.size());
  };
  if (checkpoint == nullptr) {
    struct SeedPortfolio {
      GlobalRouteResult result;
      RoutingOccupancy occupancy;
      std::size_t candidate_index{0};
      bool occupancy_aware{false};
      bool staged_priority{false};
    };
    std::optional<SeedPortfolio> selected_portfolio;
    for (std::size_t candidate_index = 0U;
         candidate_index < config.max_static_seed_candidates_per_connection; ++candidate_index) {
      SeedPortfolio portfolio;
      portfolio.candidate_index = candidate_index;
      std::set<std::size_t> selected_shove_ids;
      for (const auto& task : all_tasks) {
        std::vector<const GlobalSeedCandidate*> selected(
            task.terminals.size() > 1U ? task.terminals.size() - 1U : 0U, nullptr);
        for (const auto& candidate : static_seed_plan.candidates) {
          if (candidate.net == task.net && candidate.tree_candidate_index == candidate_index &&
              candidate.connection_index < selected.size()) {
            selected[candidate.connection_index] = &candidate;
          }
        }
        if (selected.empty() ||
            std::any_of(selected.begin(), selected.end(),
                        [](const auto* candidate) { return candidate == nullptr; })) {
          portfolio.result.unrouted_nets.push_back(task.net);
          continue;
        }
        for (const auto* candidate : selected) {
          selected_shove_ids.insert(candidate->shove_proposal_ids.begin(),
                                    candidate->shove_proposal_ids.end());
          add_path_occupancy(task.net, candidate->route.nodes, *grid, *corridors,
                             corridor_neck_downs, candidate->route.segment_widths,
                             candidate->route.segment_width_kinds, *voronoi_allocation,
                             portfolio.occupancy);
          portfolio.result.total_cost =
              saturating_add(portfolio.result.total_cost, candidate->route.cost);
          auto path =
              convert_path(*grid, task, candidate->terminal_index, candidate->connected_terminals,
                           candidate->route, *voronoi_allocation);
          path.geometric_corridors = candidate->geometric_corridors;
          const auto rules = resolve_net_routing_rules(board, task.net);
          if (config.reserve_sampled_route_clearance) {
            add_path_clearance_occupancy(path, rules.track_width, rules.via_diameter, obstacles,
                                         *grid, portfolio.occupancy);
          }
          portfolio.result.total_cost = saturating_add(
              portfolio.result.total_cost,
              path_width_reduction_penalty(path, task.nominal_track_width, config,
                                           GlobalRouteSegmentWidthKind::terminal_neck_down)
                  .cost);
          portfolio.result.paths.push_back(std::move(path));
        }
      }
      for (const auto& proposal : static_seed_plan.shove_proposals) {
        if (selected_shove_ids.contains(proposal.id)) {
          portfolio.result.total_cost =
              saturating_add(portfolio.result.total_cost, proposal.displacement_cost_nm);
          portfolio.result.selected_shove_proposals.push_back(proposal);
        }
      }
      auto portfolio_obstacles = obstacles;
      account_routing_occupancy(portfolio.result, board, *grid, *corridors, corridor_neck_downs,
                                *voronoi_allocation, portfolio.occupancy, portfolio_obstacles,
                                config, nullptr);
      portfolio.result.selected_iteration = 0U;
      portfolio.result.iterations = 0U;
      portfolio.result.diagnostic = "complete static seed portfolio assembled as iteration zero";
      ++iteration_zero_portfolios_evaluated;
      if (!selected_portfolio ||
          global_route_checkpoint_better(portfolio.result, selected_portfolio->result,
                                         config.checkpoint_objective)) {
        selected_portfolio = std::move(portfolio);
      }
    }
    const auto build_occupancy_aware_portfolio = [&](const std::vector<NetTask>& ordered_tasks,
                                                     bool staged_priority) {
      SeedPortfolio occupancy_aware;
      occupancy_aware.candidate_index = config.max_static_seed_candidates_per_connection +
                                        static_cast<std::size_t>(staged_priority);
      occupancy_aware.occupancy_aware = true;
      occupancy_aware.staged_priority = staged_priority;
      std::set<std::size_t> occupancy_aware_shove_ids;
      for (std::size_t task_index = 0U; task_index < ordered_tasks.size(); ++task_index) {
        const auto& task = ordered_tasks[task_index];
        std::map<std::size_t, std::size_t> future_corridor_demand;
        std::map<std::size_t, std::size_t> future_neck_down_demand;
        for (std::size_t future_index = task_index + 1U; future_index < ordered_tasks.size();
             ++future_index) {
          if (const auto found =
                  residual_corridor_demand_by_net.find(ordered_tasks[future_index].net);
              found != residual_corridor_demand_by_net.end()) {
            for (const auto corridor : found->second.corridors) {
              future_corridor_demand[corridor] =
                  saturating_count_add(future_corridor_demand[corridor], 1U);
            }
            for (const auto corridor : found->second.neck_downs) {
              future_neck_down_demand[corridor] =
                  saturating_count_add(future_neck_down_demand[corridor], 1U);
            }
          }
        }
        std::map<std::size_t, std::size_t> critical_corridor_demand;
        std::map<std::size_t, std::size_t> critical_neck_down_demand;
        if (config.critical_cut_cost > 0U) {
          for (const auto& [corridor, demand] : future_corridor_demand) {
            if (critical_cut(*corridors, corridor, demand, config)) {
              critical_corridor_demand.emplace(corridor, demand);
            }
          }
          for (const auto& [corridor, demand] : future_neck_down_demand) {
            if (critical_cut(corridor_neck_downs, corridor, demand, config)) {
              critical_neck_down_demand.emplace(corridor, demand);
            }
          }
        }
        const auto current_reservation = residual_corridor_demand_by_net.find(task.net);
        using TreeChoice = std::tuple<std::size_t, std::uint64_t, std::uint64_t, std::uint64_t,
                                      std::size_t, std::vector<const GlobalSeedCandidate*>>;
        std::optional<TreeChoice> best_tree;
        for (std::size_t candidate_index = 0U;
             candidate_index < config.max_static_seed_candidates_per_connection;
             ++candidate_index) {
          if (task.coupled_member && candidate_index != 0U) {
            continue;
          }
          std::vector<const GlobalSeedCandidate*> selected(
              task.terminals.size() > 1U ? task.terminals.size() - 1U : 0U, nullptr);
          for (const auto& candidate : static_seed_plan.candidates) {
            if (candidate.net == task.net && candidate.tree_candidate_index == candidate_index &&
                candidate.connection_index < selected.size()) {
              selected[candidate.connection_index] = &candidate;
            }
          }
          if (selected.empty() ||
              std::any_of(selected.begin(), selected.end(),
                          [](const auto* candidate) { return candidate == nullptr; })) {
            continue;
          }
          std::size_t pressure = 0U;
          std::uint64_t cost = 0U;
          ResidualCorridorDemand selected_corridors;
          for (const auto* candidate : selected) {
            cost = saturating_add(cost, candidate->route.cost);
            const auto candidate_corridors = seed_corridors(candidate->route);
            selected_corridors.corridors.insert(candidate_corridors.corridors.begin(),
                                                candidate_corridors.corridors.end());
            selected_corridors.neck_downs.insert(candidate_corridors.neck_downs.begin(),
                                                 candidate_corridors.neck_downs.end());
            for (const auto node : candidate->route.nodes) {
              if (const auto found = occupancy_aware.occupancy.nodes.find(node);
                  found != occupancy_aware.occupancy.nodes.end()) {
                pressure = saturating_count_add(pressure, found->second.size());
              }
              if (const auto found = occupancy_aware.occupancy.clearance_nodes.find(node);
                  found != occupancy_aware.occupancy.clearance_nodes.end()) {
                pressure = saturating_count_add(pressure, found->second.size());
              }
            }
            for (std::size_t edge = 1U; edge < candidate->route.nodes.size(); ++edge) {
              const Edge route_edge{candidate->route.nodes[edge - 1U],
                                    candidate->route.nodes[edge]};
              const auto key = sampled_capacity_edge(*grid, route_edge);
              if (const auto found = occupancy_aware.occupancy.edges.find(key);
                  found != occupancy_aware.occupancy.edges.end()) {
                pressure = saturating_count_add(pressure, found->second.size());
              }
              if (const auto found = occupancy_aware.occupancy.clearance_edges.find(route_edge);
                  found != occupancy_aware.occupancy.clearance_edges.end()) {
                pressure = saturating_count_add(pressure, found->second.size());
              }
              if (const auto corridor = corridors->corridor_for(route_edge)) {
                if (const auto found = occupancy_aware.occupancy.corridors.find(*corridor);
                    found != occupancy_aware.occupancy.corridors.end()) {
                  pressure = saturating_count_add(pressure, found->second.size());
                }
              }
              for (const auto resource : voronoi_allocation->resources_for(route_edge)) {
                if (const auto found = occupancy_aware.occupancy.voronoi_resources.find(resource);
                    found != occupancy_aware.occupancy.voronoi_resources.end()) {
                  pressure = saturating_count_add(pressure, found->second.size());
                }
              }
            }
          }
          const auto residual_penalty_for = [&](const auto& selected_ids, const auto& future,
                                                const CorridorGraph& graph,
                                                const auto& occupancy_by_resource) {
            std::uint64_t penalty = 0U;
            if (config.residual_corridor_demand_cost == 0U) {
              return penalty;
            }
            for (const auto corridor_id : selected_ids) {
              const auto future_found = future.find(corridor_id);
              if (future_found == future.end() || corridor_id >= graph.corridors.size()) {
                continue;
              }
              const auto occupied = [&] {
                const auto found = occupancy_by_resource.find(corridor_id);
                return found == occupancy_by_resource.end() ? 0U : found->second.size();
              }();
              const auto capacity = graph.corridors[corridor_id].capacity;
              const auto remaining = capacity > occupied + 1U ? capacity - occupied - 1U : 0U;
              if (future_found->second > remaining) {
                penalty = saturating_add(
                    penalty, saturating_multiply(
                                 static_cast<std::uint64_t>(future_found->second - remaining),
                                 config.residual_corridor_demand_cost));
              }
            }
            return penalty;
          };
          auto residual_penalty =
              residual_penalty_for(selected_corridors.corridors, future_corridor_demand, *corridors,
                                   occupancy_aware.occupancy.corridors);
          residual_penalty =
              saturating_add(residual_penalty,
                             residual_penalty_for(selected_corridors.neck_downs,
                                                  future_neck_down_demand, corridor_neck_downs,
                                                  occupancy_aware.occupancy.corridor_neck_downs));
          const auto critical_penalty_for = [&](const auto& selected_ids, const auto& critical,
                                                const CorridorGraph& graph,
                                                const std::set<std::size_t>* reservation) {
            std::uint64_t penalty = 0U;
            for (const auto corridor : selected_ids) {
              if ((reservation != nullptr && reservation->contains(corridor)) ||
                  corridor >= graph.corridors.size()) {
                continue;
              }
              if (const auto cut = critical.find(corridor); cut != critical.end()) {
                penalty = saturating_add(
                    penalty, critical_cut_penalty(cut->second, graph.corridors[corridor].capacity,
                                                  config.critical_cut_cost));
              }
            }
            return penalty;
          };
          const auto* current_corridor_reservation =
              current_reservation == residual_corridor_demand_by_net.end()
                  ? nullptr
                  : &current_reservation->second.corridors;
          const auto* current_neck_down_reservation =
              current_reservation == residual_corridor_demand_by_net.end()
                  ? nullptr
                  : &current_reservation->second.neck_downs;
          auto critical_penalty =
              critical_penalty_for(selected_corridors.corridors, critical_corridor_demand,
                                   *corridors, current_corridor_reservation);
          critical_penalty = saturating_add(
              critical_penalty,
              critical_penalty_for(selected_corridors.neck_downs, critical_neck_down_demand,
                                   corridor_neck_downs, current_neck_down_reservation));
          TreeChoice choice{pressure, critical_penalty, residual_penalty,
                            cost,     candidate_index,  std::move(selected)};
          if (!best_tree || choice < *best_tree) {
            best_tree = std::move(choice);
          }
        }
        if (!best_tree) {
          occupancy_aware.result.unrouted_nets.push_back(task.net);
          continue;
        }
        for (const auto* candidate : std::get<5>(*best_tree)) {
          occupancy_aware_shove_ids.insert(candidate->shove_proposal_ids.begin(),
                                           candidate->shove_proposal_ids.end());
          add_path_occupancy(task.net, candidate->route.nodes, *grid, *corridors,
                             corridor_neck_downs, candidate->route.segment_widths,
                             candidate->route.segment_width_kinds, *voronoi_allocation,
                             occupancy_aware.occupancy);
          occupancy_aware.result.total_cost =
              saturating_add(occupancy_aware.result.total_cost, candidate->route.cost);
          auto path =
              convert_path(*grid, task, candidate->terminal_index, candidate->connected_terminals,
                           candidate->route, *voronoi_allocation);
          path.geometric_corridors = candidate->geometric_corridors;
          const auto rules = resolve_net_routing_rules(board, task.net);
          if (config.reserve_sampled_route_clearance) {
            add_path_clearance_occupancy(path, rules.track_width, rules.via_diameter, obstacles,
                                         *grid, occupancy_aware.occupancy);
          }
          occupancy_aware.result.total_cost = saturating_add(
              occupancy_aware.result.total_cost,
              path_width_reduction_penalty(path, task.nominal_track_width, config,
                                           GlobalRouteSegmentWidthKind::terminal_neck_down)
                  .cost);
          occupancy_aware.result.paths.push_back(std::move(path));
        }
      }
      for (const auto& proposal : static_seed_plan.shove_proposals) {
        if (occupancy_aware_shove_ids.contains(proposal.id)) {
          occupancy_aware.result.total_cost =
              saturating_add(occupancy_aware.result.total_cost, proposal.displacement_cost_nm);
          occupancy_aware.result.selected_shove_proposals.push_back(proposal);
        }
      }
      auto occupancy_aware_obstacles = obstacles;
      account_routing_occupancy(occupancy_aware.result, board, *grid, *corridors,
                                corridor_neck_downs, *voronoi_allocation, occupancy_aware.occupancy,
                                occupancy_aware_obstacles, config, nullptr);
      occupancy_aware.result.selected_iteration = 0U;
      occupancy_aware.result.iterations = 0U;
      occupancy_aware.result.diagnostic =
          staged_priority
              ? "staged weak-occupancy static seed portfolio assembled as iteration zero"
              : "baseline weak-occupancy static seed portfolio assembled as iteration zero";
      return occupancy_aware;
    };
    auto baseline_occupancy_aware = build_occupancy_aware_portfolio(all_tasks, false);
    auto staged_occupancy_aware = build_occupancy_aware_portfolio(staged_seed_order.tasks, true);
    iteration_zero_portfolios_evaluated =
        saturating_count_add(iteration_zero_portfolios_evaluated, 2U);
    iteration_zero_baseline_unrouted_nets = baseline_occupancy_aware.result.unrouted_nets.size();
    iteration_zero_staged_unrouted_nets = staged_occupancy_aware.result.unrouted_nets.size();
    iteration_zero_baseline_total_overflow = baseline_occupancy_aware.result.total_overflow;
    iteration_zero_staged_total_overflow = staged_occupancy_aware.result.total_overflow;
    iteration_zero_baseline_clearance_shadow_conflicts =
        baseline_occupancy_aware.result.clearance_shadow_conflicts.size();
    iteration_zero_staged_clearance_shadow_conflicts =
        staged_occupancy_aware.result.clearance_shadow_conflicts.size();
    const auto staged_not_less_feasible =
        !global_route_checkpoint_better(baseline_occupancy_aware.result,
                                        staged_occupancy_aware.result,
                                        config.checkpoint_objective) &&
        staged_occupancy_aware.result.clearance_shadow_conflicts.size() <=
            baseline_occupancy_aware.result.clearance_shadow_conflicts.size();
    const auto staged_changes_small_checkpoint =
        staged_occupancy_aware.result.unrouted_nets.size() !=
            baseline_occupancy_aware.result.unrouted_nets.size() ||
        staged_occupancy_aware.result.total_overflow !=
            baseline_occupancy_aware.result.total_overflow ||
        staged_occupancy_aware.result.clearance_shadow_conflicts.size() !=
            baseline_occupancy_aware.result.clearance_shadow_conflicts.size() ||
        staged_occupancy_aware.result.total_cost != baseline_occupancy_aware.result.total_cost;
    // Until the owned 4-20 component gate proves end-to-end negotiation parity, a changed small
    // seed is only qualified when it already completes. This is the measured baseline fallback,
    // not a legality exception. Large conflict diagnostics can still exercise staged allocation
    // without doubling the complete PathFinder run.
    constexpr std::size_t staged_seed_large_board_group_threshold = 20U;
    const auto qualification_scale_safe =
        staged_seed_order.evidence.size() > staged_seed_large_board_group_threshold ||
        !staged_changes_small_checkpoint || staged_occupancy_aware.result.complete();
    iteration_zero_staged_priority_qualified = staged_not_less_feasible && qualification_scale_safe;
    if (!selected_portfolio ||
        global_route_checkpoint_better(baseline_occupancy_aware.result, selected_portfolio->result,
                                       config.checkpoint_objective)) {
      selected_portfolio = std::move(baseline_occupancy_aware);
    }
    if (iteration_zero_staged_priority_qualified &&
        (!selected_portfolio ||
         global_route_checkpoint_better(staged_occupancy_aware.result, selected_portfolio->result,
                                        config.checkpoint_objective))) {
      selected_portfolio = std::move(staged_occupancy_aware);
    }
    if (selected_portfolio) {
      best = std::move(selected_portfolio->result);
      iteration_zero_candidate_index = selected_portfolio->candidate_index;
      iteration_zero_occupancy_aware_portfolio_selected = selected_portfolio->occupancy_aware;
      iteration_zero_staged_priority_portfolio_selected = selected_portfolio->staged_priority;
      GlobalRouteResult history_evidence;
      account_routing_occupancy(history_evidence, board, *grid, *corridors, corridor_neck_downs,
                                *voronoi_allocation, selected_portfolio->occupancy, obstacles,
                                config, &history);
      const auto sum_history = [](const auto& entries) {
        std::size_t total = 0U;
        for (const auto& [key, value] : entries) {
          static_cast<void>(key);
          total = saturating_count_add(total, value);
        }
        return total;
      };
      iteration_zero_seed_history_increments =
          saturating_count_add(sum_history(history.edges), sum_history(history.nodes));
      iteration_zero_seed_history_increments = saturating_count_add(
          iteration_zero_seed_history_increments, sum_history(history.corridors));
      iteration_zero_seed_history_increments = saturating_count_add(
          iteration_zero_seed_history_increments, sum_history(history.corridor_neck_downs));
      iteration_zero_seed_history_increments = saturating_count_add(
          iteration_zero_seed_history_increments, sum_history(history.voronoi_resources));
    }
    iteration_zero_paths = best.paths.size();
    iteration_zero_unrouted_nets = best.unrouted_nets.size();
    iteration_zero_total_overflow = best.total_overflow;
    iteration_zero_sampled_overflow = best.sampled_overflow;
    iteration_zero_corridor_overflow = best.corridor_overflow;
    if (best.paths.empty()) {
      best = GlobalRouteResult{};
      best.unrouted_nets.resize(tasks.size() + 1U);
      best.total_overflow = std::numeric_limits<std::size_t>::max();
      best.total_cost = std::numeric_limits<std::uint64_t>::max();
    }
  } else {
    best.unrouted_nets.resize(tasks.size() + 1U);
    best.total_overflow = std::numeric_limits<std::size_t>::max();
    best.total_cost = std::numeric_limits<std::uint64_t>::max();
  }
  retain_checkpoint(best);
  // Keep every exact-reprojected spline corridor active during negotiated routing. The static
  // portfolio is the first checkpoint, not a disposable warm-up: later PathFinder passes may
  // leave a corridor to resolve congestion, but pay the same explicit departure cost.
  std::map<std::pair<NetId, std::size_t>, std::set<std::size_t>> spline_corridors_by_connection;
  std::map<NetId, std::set<std::size_t>> spline_corridors_by_net;
  std::map<std::pair<NetId, std::size_t>, std::vector<RoutingGeometricCorridorRun>>
      geometric_corridors_by_connection;
  std::map<NetId, std::vector<RoutingGeometricCorridorRun>> geometric_corridors_by_net;
  for (const auto& candidate : static_seed_plan.candidates) {
    if (!candidate.spline_refined || candidate.spline_corridor_nodes.empty()) {
      continue;
    }
    auto& connection = spline_corridors_by_connection[{candidate.net, candidate.connection_index}];
    connection.insert(candidate.spline_corridor_nodes.begin(),
                      candidate.spline_corridor_nodes.end());
    auto& net = spline_corridors_by_net[candidate.net];
    net.insert(candidate.spline_corridor_nodes.begin(), candidate.spline_corridor_nodes.end());
    if (!candidate.geometric_corridors.empty()) {
      geometric_corridors_by_connection.try_emplace(
          std::pair{candidate.net, candidate.connection_index}, candidate.geometric_corridors);
      auto& net_corridors = geometric_corridors_by_net[candidate.net];
      net_corridors.insert(net_corridors.end(), candidate.geometric_corridors.begin(),
                           candidate.geometric_corridors.end());
    }
  }
  std::size_t unchanged_iterations = 0;
  std::uint64_t present_congestion_scale_per_mille = 1'000U;

  for (std::size_t iteration = 1; iteration <= config.max_iterations; ++iteration) {
    RouterTelemetryEvent iteration_started;
    iteration_started.kind = RouterTelemetryEventKind::pathfinder_iteration_started;
    iteration_started.iteration = iteration;
    iteration_started.total_nets = tasks.size();
    emit_telemetry(std::move(iteration_started));
    const bool enforce_hard_capacity = config.hard_capacity_start_iteration > 0U &&
                                       iteration >= config.hard_capacity_start_iteration;
    if (enforce_hard_capacity) {
      ++hard_capacity_passes;
    }
    std::vector<NetId> attempted_hard_capacity_order;
    if (enforce_hard_capacity) {
      attempted_hard_capacity_order.reserve(tasks.size());
      for (const auto& task : tasks) {
        attempted_hard_capacity_order.push_back(task.net);
      }
    }
    if (iteration >= config.present_congestion_escalation_start_iteration) {
      present_congestion_scale_per_mille =
          std::min(config.max_present_congestion_scale_per_mille,
                   scale_per_mille(present_congestion_scale_per_mille,
                                   config.present_congestion_multiplier_per_mille));
    }
    auto iteration_config = config;
    iteration_config.present_congestion_cost =
        scale_per_mille(config.present_congestion_cost, present_congestion_scale_per_mille);
    iteration_config.corridor_present_cost =
        scale_per_mille(config.corridor_present_cost, present_congestion_scale_per_mille);
    iteration_config.voronoi_resource_present_cost =
        scale_per_mille(config.voronoi_resource_present_cost, present_congestion_scale_per_mille);
    auto occupancy = reuse.occupancy;
    std::map<std::pair<NetId, NetId>, std::size_t> iteration_hard_capacity_blockers;
    GlobalRouteResult current;
    // Selective reroute treats an in-progress pass as a checkpoint before finalization. Carry the
    // lattice/access identity now so compatible same-run reuse is not conservatively invalidated.
    current.configured_selected_nets = config.selected_nets;
    current.configured_priority_nets = config.priority_nets;
    current.configured_grid_pitch = config.grid_pitch;
    current.configured_via_site_pitch = config.via_site_pitch;
    current.configured_via_site_phase_x = config.via_site_phase_x;
    current.configured_via_site_phase_y = config.via_site_phase_y;
    current.configured_diagonal_grid_edges = config.allow_diagonal_grid_edges;
    current.configured_crossing_safe_diagonal_grid_edges =
        config.allow_crossing_safe_diagonal_grid_edges;
    current.configured_sampled_route_clearance_reservations =
        config.reserve_sampled_route_clearance;
    current.configured_max_clearance_shadow_conflict_resources =
        config.max_clearance_shadow_conflict_resources;
    current.configured_clearance_shadow_cost_quantum = config.clearance_shadow_cost_quantum;
    current.configured_single_lane_direct_voronoi_resources =
        config.single_lane_direct_voronoi_resources;
    current.configured_max_static_seed_candidates_per_connection =
        config.max_static_seed_candidates_per_connection;
    current.configured_max_static_seed_searches = config.max_static_seed_searches;
    current.configured_max_static_seed_reprojection_searches =
        config.max_static_seed_reprojection_searches;
    current.configured_max_static_seed_workers = config.max_static_seed_workers;
    current.configured_static_seed_alternative_edge_penalty =
        config.static_seed_alternative_edge_penalty;
    current.configured_static_seed_corridor_departure_penalty =
        config.static_seed_corridor_departure_penalty;
    current.configured_former_route_memory_cost = config.former_route_memory_cost;
    current.configured_terminal_escape_reservation_cost = config.terminal_escape_reservation_cost;
    current.configured_max_terminal_escape_reservation_resources =
        config.max_terminal_escape_reservation_resources;
    current.configured_global_spline_tessellation_tolerance =
        config.global_spline_tessellation_tolerance;
    current.configured_global_spline_control_corridor = config.global_spline_control_corridor;
    current.configured_global_spline_shove_step = config.global_spline_shove_step;
    current.configured_navmesh_funnel_spline_seeds = config.use_navmesh_funnel_spline_seeds;
    current.configured_global_navmesh_cell_pitch = config.global_navmesh_cell_pitch;
    current.configured_global_spline_minimum_bend_radius = config.global_spline_minimum_bend_radius;
    current.configured_max_global_navmesh_cells_per_seed = config.max_global_navmesh_cells_per_seed;
    current.configured_max_global_navmesh_search_expansions_per_seed =
        config.max_global_navmesh_search_expansions_per_seed;
    current.configured_global_spline_trajectory_samples = config.global_spline_trajectory_samples;
    current.configured_max_global_spline_trajectory_iterations =
        config.max_global_spline_trajectory_iterations;
    current.configured_max_global_spline_trajectory_candidate_checks =
        config.max_global_spline_trajectory_candidate_checks;
    current.configured_global_spline_max_tessellation_depth =
        config.global_spline_max_tessellation_depth;
    current.configured_max_global_spline_samples_per_seed =
        config.max_global_spline_samples_per_seed;
    current.configured_max_global_spline_shove_iterations =
        config.max_global_spline_shove_iterations;
    current.configured_max_static_seed_corridor_cells = config.max_static_seed_corridor_cells;
    current.configured_max_global_pns_propagation_depth = config.max_global_pns_propagation_depth;
    current.configured_max_global_pns_candidate_checks = config.max_global_pns_candidate_checks;
    current.configured_maximum_global_pns_displacement = config.maximum_global_pns_displacement;
    current.configured_local_pin_escape_pitch = config.local_pin_escape_pitch;
    current.configured_max_local_pin_escape_expansions_per_terminal =
        config.max_local_pin_escape_expansions_per_terminal;
    current.configured_contended_terminal_fanout = config.allocate_contended_terminal_fanout;
    current.configured_diversify_pin_access_rings = config.diversify_pin_access_rings;
    current.configured_hard_capacity_start_iteration = config.hard_capacity_start_iteration;
    current.configured_max_hard_capacity_ripup_attempts_per_pass =
        config.max_hard_capacity_ripup_attempts_per_pass;
    current.configured_max_hard_capacity_ripup_nets_per_attempt =
        config.max_hard_capacity_ripup_nets_per_attempt;
    current.configured_max_hard_capacity_conflict_set_states_per_attempt =
        config.max_hard_capacity_conflict_set_states_per_attempt;
    current.configured_rollback_failed_hard_capacity_ripup =
        config.rollback_failed_hard_capacity_ripup;
    current.configured_defer_hard_capacity_ripup_victims_to_pass_tail =
        config.defer_hard_capacity_ripup_victims_to_pass_tail;
    current.configured_protect_recovered_hard_capacity_ripup_nets =
        config.protect_recovered_hard_capacity_ripup_nets;
    current.configured_max_hard_capacity_blocker_reuse_invalidations_per_failed_net =
        config.max_hard_capacity_blocker_reuse_invalidations_per_failed_net;
    current.iterations = iteration;
    current.selected_iteration = iteration;
    current.selected_present_congestion_scale_per_mille = present_congestion_scale_per_mille;
    current.selected_effective_present_congestion_cost = iteration_config.present_congestion_cost;
    current.selected_effective_corridor_present_cost = iteration_config.corridor_present_cost;
    current.selected_effective_voronoi_resource_present_cost =
        iteration_config.voronoi_resource_present_cost;
    current.total_cost = reuse.cost;
    current.paths = reuse.paths;
    current.reused_paths = checkpoint_reused_paths;
    current.reused_nets = checkpoint_reused_nets;
    current.rerouted_nets = checkpoint_rerouted_nets;
    current.invalidated_reuse_nets = checkpoint_invalidated_nets;

    std::deque<NetTask> pending_tasks(tasks.begin(), tasks.end());
    std::set<NetId> routed_this_pass;
    std::map<NetId, std::uint64_t> routed_cost_this_pass;
    std::set<NetId> hard_ripup_attempted_nets;
    std::set<NetId> hard_ripup_retry_nets;
    std::set<NetId> hard_ripup_protected_nets;
    std::map<NetId, std::size_t> hard_ripup_record_by_net;
    std::vector<std::size_t> pass_hard_ripup_records;
    FormerRouteMemory former_route_memory;
    struct PendingHardRipupRollback {
      NetId failed_net;
      RoutingOccupancy occupancy_before;
      FormerRouteMemory former_route_memory_before;
      std::vector<GlobalRoutePath> paths_before;
      std::uint64_t total_cost_before{0U};
      std::set<NetId> routed_nets_before;
      std::map<NetId, std::uint64_t> routed_costs_before;
      std::vector<NetId> queued_victims;
      std::size_t record_index{0U};
    };
    std::optional<PendingHardRipupRollback> pending_hard_ripup_rollback;
    std::size_t pass_hard_ripup_attempts = 0U;
    while (!pending_tasks.empty()) {
      auto task = std::move(pending_tasks.front());
      pending_tasks.pop_front();
      if (stop_token.stop_requested()) {
        current.stop_reason = GlobalRouterStopReason::cancelled;
        current.diagnostic = "global routing cancelled";
        return finalize(std::move(current));
      }
      // Estimate only demand that remains after this task.  The estimate is rebuilt when a
      // rip-up appends work, so local destruction/rebuild passes do not keep reserving capacity
      // for a net that has already been successfully restored.
      std::map<std::size_t, std::size_t> future_corridor_demand;
      std::map<std::size_t, std::size_t> future_neck_down_demand;
      if (config.residual_corridor_demand_cost > 0U) {
        for (const auto& future_task : pending_tasks) {
          const auto future = residual_corridor_demand_by_net.find(future_task.net);
          if (future == residual_corridor_demand_by_net.end()) {
            continue;
          }
          for (const auto corridor : future->second.corridors) {
            future_corridor_demand[corridor] =
                saturating_count_add(future_corridor_demand[corridor], 1U);
          }
          for (const auto corridor : future->second.neck_downs) {
            future_neck_down_demand[corridor] =
                saturating_count_add(future_neck_down_demand[corridor], 1U);
          }
        }
      }
      std::map<std::size_t, std::size_t> critical_corridor_cut_demand;
      std::map<std::size_t, std::size_t> critical_neck_down_cut_demand;
      if (config.critical_cut_cost > 0U) {
        for (const auto& [corridor, demand] : future_corridor_demand) {
          if (critical_cut(*corridors, corridor, demand, config)) {
            critical_corridor_cut_demand.emplace(corridor, demand);
          }
        }
        for (const auto& [corridor, demand] : future_neck_down_demand) {
          if (critical_cut(corridor_neck_downs, corridor, demand, config)) {
            critical_neck_down_cut_demand.emplace(corridor, demand);
          }
        }
      }
      const auto current_reservation = residual_corridor_demand_by_net.find(task.net);
      std::optional<TerminalEscapeReservation> terminal_escape_reservation;
      if (iteration_config.terminal_escape_reservation_cost > 0U &&
          iteration_config.max_terminal_escape_reservation_resources > 0U &&
          !pending_tasks.empty()) {
        terminal_escape_reservation = build_terminal_escape_reservation(
            *grid, task, pending_tasks, iteration_config.max_terminal_escape_reservation_resources);
        current.terminal_escape_reservation_terminals =
            saturating_count_add(current.terminal_escape_reservation_terminals,
                                 terminal_escape_reservation->candidates_by_terminal.size());
        current.terminal_escape_reservation_candidates =
            saturating_count_add(current.terminal_escape_reservation_candidates,
                                 terminal_escape_reservation->candidates.size());
        current.terminal_escape_reservation_resources = saturating_count_add(
            current.terminal_escape_reservation_resources, terminal_escape_reservation->resources);
        current.terminal_escape_reservation_withheld_candidates =
            saturating_count_add(current.terminal_escape_reservation_withheld_candidates,
                                 terminal_escape_reservation->withheld_candidates);
      }
      std::optional<RoutingOccupancy> atomic_occupancy_before;
      std::optional<GlobalRouteResult> atomic_current_before;
      if (config.atomic_complete_net_routing) {
        atomic_occupancy_before = occupancy;
        atomic_current_before = current;
      }
      const auto task_cost_before = current.total_cost;
      const auto task_failure_count_before = current.connection_failures.size();
      SearchEffort task_search_effort;
      std::map<NetId, std::size_t> task_hard_capacity_blockers;
      std::map<std::vector<NetId>, std::size_t> task_hard_capacity_blocker_sets;
      enum class TaskRouteStatus { complete, incomplete, cancelled };
      const auto route_task = [&](bool multisource) {
        std::set<std::size_t> tree(task.terminals.front().grid_nodes.begin(),
                                   task.terminals.front().grid_nodes.end());
        std::vector<bool> connected(task.terminals.size(), false);
        connected.front() = true;
        std::vector<bool> deferred_inaccessible_counted(task.terminals.size(), false);
        const auto terminal_evidence = [](const Terminal& terminal) {
          return RoutingTerminalEvidence{terminal.footprint, terminal.pad, terminal.exact_position};
        };
        const auto record_connection_failure =
            [&](RoutingConnectionFailureReason reason, bool used_multisource,
                bool direct_edge_fallback_attempted, std::size_t fixed_terminal,
                const std::vector<bool>& connected_terminals, std::size_t expansions,
                std::size_t topology_rejected_edges) {
              RoutingConnectionFailure failure;
              failure.net = task.net;
              failure.completed_connections =
                  static_cast<std::size_t>(
                      std::count(connected_terminals.begin(), connected_terminals.end(), true)) -
                  1U;
              failure.multisource = used_multisource;
              failure.direct_edge_fallback_attempted = direct_edge_fallback_attempted;
              failure.reason = reason;
              failure.expansions = expansions;
              failure.topology_rejected_edges = topology_rejected_edges;
              for (std::size_t terminal_index = 0U; terminal_index < task.terminals.size();
                   ++terminal_index) {
                if (connected_terminals[terminal_index]) {
                  failure.connected_terminals.push_back(
                      terminal_evidence(task.terminals[terminal_index]));
                } else if (used_multisource || terminal_index == fixed_terminal) {
                  failure.attempted_terminals.push_back(
                      terminal_evidence(task.terminals[terminal_index]));
                }
              }
              current.connection_failures.push_back(std::move(failure));
            };
        for (std::size_t connection = 1; connection < task.terminals.size(); ++connection) {
          if (std::all_of(connected.begin(), connected.end(), [](bool state) { return state; })) {
            break;
          }
          const auto fixed_terminal = static_cast<std::size_t>(
              std::find(connected.begin() + 1, connected.end(), false) - connected.begin());
          std::vector<std::size_t> starts;
          if (multisource) {
            for (std::size_t terminal_index = 1U; terminal_index < task.terminals.size();
                 ++terminal_index) {
              if (!connected[terminal_index]) {
                starts.insert(starts.end(), task.terminals[terminal_index].grid_nodes.begin(),
                              task.terminals[terminal_index].grid_nodes.end());
              }
            }
            std::sort(starts.begin(), starts.end());
            starts.erase(std::unique(starts.begin(), starts.end()), starts.end());
          } else {
            starts = task.terminals[fixed_terminal].grid_nodes;
          }
          const bool attempt_direct = config.use_direct_voronoi_resource_edges &&
                                      !voronoi_allocation->direct_resource_by_edge.empty() &&
                                      !starts.empty() && !tree.empty();
          std::optional<TerminalEscapeReservationAvailability> terminal_escape_availability;
          if (terminal_escape_reservation) {
            terminal_escape_availability =
                terminal_escape_reservation_availability(*terminal_escape_reservation, occupancy);
          }
          SearchCostPolicy negotiated_spline_policy;
          negotiated_spline_policy.residual_corridor_demand = &future_corridor_demand;
          negotiated_spline_policy.residual_corridor_neck_down_demand = &future_neck_down_demand;
          negotiated_spline_policy.residual_corridor_demand_cost =
              iteration_config.residual_corridor_demand_cost;
          negotiated_spline_policy.critical_corridor_cut_demand = &critical_corridor_cut_demand;
          negotiated_spline_policy.critical_corridor_neck_down_cut_demand =
              &critical_neck_down_cut_demand;
          if (current_reservation != residual_corridor_demand_by_net.end()) {
            negotiated_spline_policy.current_reserved_corridors =
                &current_reservation->second.corridors;
            negotiated_spline_policy.current_reserved_corridor_neck_downs =
                &current_reservation->second.neck_downs;
          }
          negotiated_spline_policy.critical_cut_cost = iteration_config.critical_cut_cost;
          negotiated_spline_policy.former_route_memory = &former_route_memory;
          negotiated_spline_policy.former_route_memory_cost =
              iteration_config.former_route_memory_cost;
          if (terminal_escape_reservation && terminal_escape_availability) {
            negotiated_spline_policy.terminal_escape_reservation = &*terminal_escape_reservation;
            negotiated_spline_policy.terminal_escape_availability = &*terminal_escape_availability;
            negotiated_spline_policy.terminal_escape_reservation_cost =
                iteration_config.terminal_escape_reservation_cost;
          }
          const std::set<std::size_t>* spline_corridor = nullptr;
          if (multisource) {
            if (const auto found = spline_corridors_by_net.find(task.net);
                found != spline_corridors_by_net.end()) {
              spline_corridor = &found->second;
            }
          } else if (const auto found =
                         spline_corridors_by_connection.find({task.net, fixed_terminal - 1U});
                     found != spline_corridors_by_connection.end()) {
            spline_corridor = &found->second;
          }
          if (spline_corridor != nullptr) {
            negotiated_spline_policy.kind = SearchCostPolicyKind::corridor_reprojection;
            negotiated_spline_policy.corridor_nodes = *spline_corridor;
            negotiated_spline_policy.corridor_departure_penalty =
                config.static_seed_corridor_departure_penalty;
          }
          const auto connection_expansions_before = task_search_effort.expansions;
          const auto connection_topology_rejections_before = current.topology_rejected_search_edges;
          const auto bounded_search = [&](bool use_direct_edges) {
            const auto invoke = [&](const std::vector<std::size_t>& search_starts,
                                    const std::set<std::size_t>& search_goals,
                                    std::uint32_t heuristic_weight_per_mille) {
              if (negotiated_spline_policy.kind == SearchCostPolicyKind::corridor_reprojection) {
                ++current.negotiated_spline_guided_searches;
              }
              auto result = search_path(
                  *grid, search_starts, search_goals, occupancy, history, *corridors,
                  corridor_neck_downs, *voronoi_guidance, *voronoi_allocation, obstacles, task,
                  iteration_config, use_direct_edges, enforce_hard_capacity,
                  heuristic_weight_per_mille, negotiated_spline_policy, stop_token);
              current.residual_corridor_demand_penalty_events =
                  saturating_count_add(current.residual_corridor_demand_penalty_events,
                                       result.residual_corridor_demand_penalty_events);
              current.residual_corridor_demand_penalty =
                  saturating_add(current.residual_corridor_demand_penalty,
                                 result.residual_corridor_demand_penalty);
              current.former_route_memory_penalty_events =
                  saturating_count_add(current.former_route_memory_penalty_events,
                                       result.former_route_memory_penalty_events);
              current.former_route_memory_penalty = saturating_add(
                  current.former_route_memory_penalty, result.former_route_memory_penalty);
              former_route_memory_penalty_events = saturating_count_add(
                  former_route_memory_penalty_events, result.former_route_memory_penalty_events);
              former_route_memory_penalty =
                  saturating_add(former_route_memory_penalty, result.former_route_memory_penalty);
              current.terminal_escape_reservation_penalty_events =
                  saturating_count_add(current.terminal_escape_reservation_penalty_events,
                                       result.terminal_escape_reservation_penalty_events);
              current.terminal_escape_reservation_penalty =
                  saturating_add(current.terminal_escape_reservation_penalty,
                                 result.terminal_escape_reservation_penalty);
              critical_cut_penalty_events = saturating_count_add(
                  critical_cut_penalty_events, result.critical_cut_penalty_events);
              critical_cut_penalty_total =
                  saturating_add(critical_cut_penalty_total, result.critical_cut_penalty);
              if (negotiated_spline_policy.kind == SearchCostPolicyKind::corridor_reprojection) {
                current.negotiated_spline_corridor_departures = saturating_count_add(
                    current.negotiated_spline_corridor_departures,
                    static_cast<std::size_t>(std::count_if(
                        result.nodes.begin(), result.nodes.end(), [&](const auto node) {
                          return !negotiated_spline_policy.corridor_nodes.contains(node);
                        })));
              }
              hard_sampled_capacity_rejected_transitions =
                  saturating_count_add(hard_sampled_capacity_rejected_transitions,
                                       result.hard_sampled_capacity_rejected_transitions);
              hard_corridor_capacity_rejected_transitions =
                  saturating_count_add(hard_corridor_capacity_rejected_transitions,
                                       result.hard_corridor_capacity_rejected_transitions);
              hard_voronoi_capacity_rejected_transitions =
                  saturating_count_add(hard_voronoi_capacity_rejected_transitions,
                                       result.hard_voronoi_capacity_rejected_transitions);
              for (const auto& [blocker, rejected_transitions] :
                   result.hard_capacity_blocking_nets) {
                task_hard_capacity_blockers[blocker] = saturating_count_add(
                    task_hard_capacity_blockers[blocker], rejected_transitions);
              }
              hard_capacity_conflict_sets_withheld =
                  saturating_count_add(hard_capacity_conflict_sets_withheld,
                                       result.hard_capacity_conflict_sets_withheld);
              for (const auto& [blocker_set, observations] : result.hard_capacity_blocker_sets) {
                if (const auto found = task_hard_capacity_blocker_sets.find(blocker_set);
                    found != task_hard_capacity_blocker_sets.end()) {
                  found->second = saturating_count_add(found->second, observations);
                } else if (task_hard_capacity_blocker_sets.size() <
                           config.max_hard_capacity_conflict_set_states_per_attempt) {
                  task_hard_capacity_blocker_sets.emplace(blocker_set, observations);
                } else {
                  hard_capacity_conflict_sets_withheld =
                      saturating_count_add(hard_capacity_conflict_sets_withheld, 1U);
                }
              }
              task_search_effort.invocations =
                  saturating_count_add(task_search_effort.invocations, 1U);
              task_search_effort.expansions =
                  saturating_count_add(task_search_effort.expansions, result.expansions);
              if (result.encountered_expansion_limit) {
                task_search_effort.expansion_limited =
                    saturating_count_add(task_search_effort.expansion_limited, 1U);
              }
              task_search_effort.weighted_frontier_continuations =
                  saturating_count_add(task_search_effort.weighted_frontier_continuations,
                                       result.weighted_frontier_continuation_attempted ? 1U : 0U);
              task_search_effort.weighted_frontier_recoveries =
                  saturating_count_add(task_search_effort.weighted_frontier_recoveries,
                                       result.weighted_frontier_continuation_recovered ? 1U : 0U);
              task_search_effort.weighted_frontier_states_reused =
                  saturating_count_add(task_search_effort.weighted_frontier_states_reused,
                                       result.weighted_frontier_states_reused);
              task_search_effort.weighted_frontier_continuation_limits =
                  saturating_count_add(task_search_effort.weighted_frontier_continuation_limits,
                                       result.weighted_frontier_continuation_limits);
              return result;
            };
            const bool bidirectional_eligible =
                config.use_bidirectional_route_growth && !multisource &&
                iteration >= config.bidirectional_route_growth_start_iteration &&
                config.max_bidirectional_meeting_nodes > 0U &&
                config.max_bidirectional_meeting_expansions > 0U;
            if (bidirectional_eligible) {
              ++bidirectional_route_growth_attempts;
              const auto meetings = find_bidirectional_meeting_nodes(
                  *grid, starts, tree, *voronoi_allocation, obstacles, task, iteration_config,
                  use_direct_edges, stop_token);
              bidirectional_route_growth_meeting_nodes = saturating_count_add(
                  bidirectional_route_growth_meeting_nodes, meetings.nodes.size());
              std::optional<SearchResult> best_meeting_route;
              for (const auto meeting : meetings.nodes) {
                const std::set<std::size_t> meeting_goal{meeting};
                auto from_terminal = invoke(starts, meeting_goal, 1'000U);
                if (from_terminal.cancelled) {
                  return from_terminal;
                }
                if (from_terminal.nodes.empty()) {
                  continue;
                }
                const std::vector<std::size_t> tree_starts(tree.begin(), tree.end());
                auto from_tree = invoke(tree_starts, meeting_goal, 1'000U);
                if (from_tree.cancelled) {
                  return from_tree;
                }
                if (from_tree.nodes.empty()) {
                  continue;
                }
                SearchResult combined;
                combined.cost = saturating_add(from_terminal.cost, from_tree.cost);
                combined.expansions =
                    saturating_count_add(from_terminal.expansions, from_tree.expansions);
                combined.topology_rejected_edges = saturating_count_add(
                    from_terminal.topology_rejected_edges, from_tree.topology_rejected_edges);
                combined.hard_sampled_capacity_rejected_transitions =
                    saturating_count_add(from_terminal.hard_sampled_capacity_rejected_transitions,
                                         from_tree.hard_sampled_capacity_rejected_transitions);
                combined.hard_corridor_capacity_rejected_transitions =
                    saturating_count_add(from_terminal.hard_corridor_capacity_rejected_transitions,
                                         from_tree.hard_corridor_capacity_rejected_transitions);
                combined.hard_voronoi_capacity_rejected_transitions =
                    saturating_count_add(from_terminal.hard_voronoi_capacity_rejected_transitions,
                                         from_tree.hard_voronoi_capacity_rejected_transitions);
                combined.nodes = std::move(from_terminal.nodes);
                combined.segment_widths = std::move(from_terminal.segment_widths);
                combined.segment_width_kinds = std::move(from_terminal.segment_width_kinds);
                // If the exact halves rediscover a channel, retain the simple connection rather
                // than materializing a same-net loop around their meeting point.
                for (std::size_t reverse = 0U; reverse < from_tree.segment_widths.size();
                     ++reverse) {
                  const auto node_index = from_tree.nodes.size() - 2U - reverse;
                  const auto segment_index = from_tree.segment_widths.size() - 1U - reverse;
                  const auto existing = std::find(combined.nodes.begin(), combined.nodes.end(),
                                                  from_tree.nodes[node_index]);
                  if (existing != combined.nodes.end()) {
                    const auto retained_nodes = static_cast<std::size_t>(
                        std::distance(combined.nodes.begin(), existing) + 1U);
                    combined.nodes.resize(retained_nodes);
                    combined.segment_widths.resize(retained_nodes - 1U);
                    combined.segment_width_kinds.resize(retained_nodes - 1U);
                    continue;
                  }
                  combined.nodes.push_back(from_tree.nodes[node_index]);
                  combined.segment_widths.push_back(from_tree.segment_widths[segment_index]);
                  combined.segment_width_kinds.push_back(
                      from_tree.segment_width_kinds[segment_index]);
                }
                if (!best_meeting_route ||
                    std::tie(combined.cost, combined.nodes) <
                        std::tie(best_meeting_route->cost, best_meeting_route->nodes)) {
                  best_meeting_route = std::move(combined);
                }
              }
              if (best_meeting_route) {
                ++bidirectional_route_growth_accepted;
                return std::move(*best_meeting_route);
              }
              ++bidirectional_route_growth_fallbacks;
            }
            auto result = invoke(starts, tree, 1'000U);
            if (result.weighted_frontier_continuation_attempted) {
              task_search_effort.weighted_retries =
                  saturating_count_add(task_search_effort.weighted_retries, 1U);
              if (!result.cancelled) {
                auto continued = std::move(result);
                auto retry =
                    invoke(starts, tree, config.expansion_retry_heuristic_weight_per_mille);
                if (retry.cancelled || !retry.nodes.empty()) {
                  result = std::move(retry);
                } else if (!continued.nodes.empty()) {
                  result = std::move(continued);
                } else {
                  result = std::move(retry);
                }
              }
              if (!result.nodes.empty()) {
                task_search_effort.weighted_recoveries =
                    saturating_count_add(task_search_effort.weighted_recoveries, 1U);
              }
            } else if (!result.cancelled && result.nodes.empty() &&
                       result.expansion_limit_reached &&
                       config.expansion_retry_heuristic_weight_per_mille > 1'000U) {
              task_search_effort.weighted_retries =
                  saturating_count_add(task_search_effort.weighted_retries, 1U);
              auto retry = invoke(starts, tree, config.expansion_retry_heuristic_weight_per_mille);
              if (!retry.nodes.empty()) {
                task_search_effort.weighted_recoveries =
                    saturating_count_add(task_search_effort.weighted_recoveries, 1U);
              }
              result = std::move(retry);
            }
            return result;
          };
          auto search = bounded_search(attempt_direct);
          current.topology_rejected_search_edges += search.topology_rejected_edges;
          if (search.cancelled) {
            current.stop_reason = GlobalRouterStopReason::cancelled;
            current.diagnostic = "global routing cancelled during path search";
            return TaskRouteStatus::cancelled;
          }
          if (search.nodes.empty() && attempt_direct) {
            ++current.direct_voronoi_search_fallbacks;
            auto fallback = bounded_search(false);
            current.topology_rejected_search_edges += fallback.topology_rejected_edges;
            if (fallback.cancelled) {
              current.stop_reason = GlobalRouterStopReason::cancelled;
              current.diagnostic = "global routing cancelled during sampled fallback search";
              return TaskRouteStatus::cancelled;
            }
            search = std::move(fallback);
          }
          if (search.nodes.empty()) {
            const auto reason = starts.empty() || tree.empty()
                                    ? RoutingConnectionFailureReason::no_access_nodes
                                : search.expansion_limit_reached
                                    ? RoutingConnectionFailureReason::expansion_limit
                                    : RoutingConnectionFailureReason::search_frontier_exhausted;
            record_connection_failure(
                reason, multisource, attempt_direct, fixed_terminal, connected,
                task_search_effort.expansions - connection_expansions_before,
                current.topology_rejected_search_edges - connection_topology_rejections_before);
            return TaskRouteStatus::incomplete;
          }
          auto selected_terminal = fixed_terminal;
          if (multisource) {
            const auto selected_node = search.nodes.front();
            const auto selected =
                std::find_if(task.terminals.begin() + 1, task.terminals.end(),
                             [&connected, selected_node, &task](const Terminal& terminal) {
                               const auto terminal_index =
                                   static_cast<std::size_t>(&terminal - task.terminals.data());
                               return !connected[terminal_index] &&
                                      std::binary_search(terminal.grid_nodes.begin(),
                                                         terminal.grid_nodes.end(), selected_node);
                             });
            if (selected == task.terminals.end()) {
              record_connection_failure(
                  RoutingConnectionFailureReason::unmatched_multisource_terminal, true,
                  attempt_direct, fixed_terminal, connected,
                  task_search_effort.expansions - connection_expansions_before,
                  current.topology_rejected_search_edges - connection_topology_rejections_before);
              return TaskRouteStatus::incomplete;
            }
            selected_terminal = static_cast<std::size_t>(selected - task.terminals.begin());
            ++current.multisource_steiner_connections;
            if (selected_terminal != fixed_terminal) {
              ++current.multisource_steiner_reordered_connections;
            }
            for (std::size_t terminal_index = 1U; terminal_index < task.terminals.size();
                 ++terminal_index) {
              if (!connected[terminal_index] && terminal_index != selected_terminal &&
                  task.terminals[terminal_index].grid_nodes.empty() &&
                  !deferred_inaccessible_counted[terminal_index]) {
                deferred_inaccessible_counted[terminal_index] = true;
                ++current.multisource_steiner_deferred_inaccessible_terminals;
              }
            }
          }
          std::vector<std::size_t> connected_terminals;
          for (std::size_t terminal_index = 0U; terminal_index < connected.size();
               ++terminal_index) {
            if (connected[terminal_index]) {
              connected_terminals.push_back(terminal_index);
            }
          }
          add_path_occupancy(task.net, search.nodes, *grid, *corridors, corridor_neck_downs,
                             search.segment_widths, search.segment_width_kinds, *voronoi_allocation,
                             occupancy);
          current.total_cost = saturating_add(current.total_cost, search.cost);
          current.river_following_edges =
              saturating_count_add(current.river_following_edges, search.river_following_edges);
          current.river_following_cost_discount = saturating_add(
              current.river_following_cost_discount, search.river_following_cost_discount);
          auto path = convert_path(*grid, task, selected_terminal, connected_terminals, search,
                                   *voronoi_allocation);
          if (multisource) {
            if (const auto geometric = geometric_corridors_by_net.find(task.net);
                geometric != geometric_corridors_by_net.end()) {
              path.geometric_corridors = geometric->second;
            }
          } else if (const auto geometric =
                         geometric_corridors_by_connection.find({task.net, path.connection_index});
                     geometric != geometric_corridors_by_connection.end()) {
            path.geometric_corridors = geometric->second;
          }
          const auto topology_preserved =
              spline_corridor == nullptr ||
              std::all_of(search.nodes.begin(), search.nodes.end(),
                          [&](const auto node) { return spline_corridor->contains(node); });
          for (auto& corridor_run : path.geometric_corridors) {
            corridor_run.selected_path_preserves_topology = topology_preserved;
          }
          const auto rules = resolve_net_routing_rules(board, task.net);
          if (config.reserve_sampled_route_clearance) {
            add_path_clearance_occupancy(path, rules.track_width, rules.via_diameter, obstacles,
                                         *grid, occupancy);
          }
          current.total_cost = saturating_add(
              current.total_cost,
              path_width_reduction_penalty(path, task.nominal_track_width, config,
                                           GlobalRouteSegmentWidthKind::terminal_neck_down)
                  .cost);
          current.paths.push_back(std::move(path));
          // The initial terminal contributes several alternative access portals as search goals,
          // but only the goal selected by the first path has physical access copper. Retaining the
          // unselected portals in the growing tree lets later branches terminate on phantom
          // copper and yields a globally "complete" yet disconnected detailed route.
          if (connection == 1U) {
            tree.clear();
          }
          tree.insert(search.nodes.begin(), search.nodes.end());
          connected[selected_terminal] = true;
          if (const auto access =
                  task.terminals[selected_terminal].access_paths.find(search.nodes.front());
              access != task.terminals[selected_terminal].access_paths.end()) {
            for (const auto joined : access->second.joined_terminals) {
              if (joined < connected.size()) {
                connected[joined] = true;
              }
            }
          }
        }
        return TaskRouteStatus::complete;
      };

      auto status = TaskRouteStatus::incomplete;
      if (config.use_multisource_steiner_growth && task.terminals.size() > 2U) {
        const auto occupancy_before = occupancy;
        const auto current_before = current;
        status = route_task(true);
        if (status == TaskRouteStatus::incomplete) {
          auto multisource_occupancy = std::move(occupancy);
          auto multisource_current = std::move(current);
          occupancy = occupancy_before;
          current = current_before;
          status = route_task(false);
          if (status != TaskRouteStatus::cancelled) {
            ++current.multisource_steiner_fixed_order_fallbacks;
            if (status == TaskRouteStatus::complete) {
              ++current.multisource_steiner_fixed_order_recoveries;
            } else {
              const auto fixed_paths = current.paths.size() - current_before.paths.size();
              const auto multisource_paths =
                  multisource_current.paths.size() - current_before.paths.size();
              if (multisource_paths > fixed_paths) {
                occupancy = std::move(multisource_occupancy);
                current = std::move(multisource_current);
                current.multisource_steiner_fixed_order_fallbacks =
                    current_before.multisource_steiner_fixed_order_fallbacks + 1U;
              }
            }
          }
        }
      } else {
        status = route_task(config.use_multisource_steiner_growth);
      }
      if (status == TaskRouteStatus::cancelled) {
        current.path_search_invocations =
            saturating_count_add(current.path_search_invocations, task_search_effort.invocations);
        current.path_search_expansions =
            saturating_count_add(current.path_search_expansions, task_search_effort.expansions);
        current.expansion_limited_searches = saturating_count_add(
            current.expansion_limited_searches, task_search_effort.expansion_limited);
        current.weighted_search_retries = saturating_count_add(current.weighted_search_retries,
                                                               task_search_effort.weighted_retries);
        current.weighted_search_recoveries = saturating_count_add(
            current.weighted_search_recoveries, task_search_effort.weighted_recoveries);
        current.weighted_frontier_continuations =
            saturating_count_add(current.weighted_frontier_continuations,
                                 task_search_effort.weighted_frontier_continuations);
        current.weighted_frontier_recoveries = saturating_count_add(
            current.weighted_frontier_recoveries, task_search_effort.weighted_frontier_recoveries);
        current.weighted_frontier_states_reused =
            saturating_count_add(current.weighted_frontier_states_reused,
                                 task_search_effort.weighted_frontier_states_reused);
        current.weighted_frontier_continuation_limits =
            saturating_count_add(current.weighted_frontier_continuation_limits,
                                 task_search_effort.weighted_frontier_continuation_limits);
        return finalize(std::move(current));
      }
      if (status == TaskRouteStatus::incomplete && enforce_hard_capacity) {
        for (const auto& [blocker, rejected_transitions] : task_hard_capacity_blockers) {
          if (blocker == task.net) {
            continue;
          }
          const auto relation = std::pair{task.net, blocker};
          iteration_hard_capacity_blockers[relation] = saturating_count_add(
              iteration_hard_capacity_blockers[relation], rejected_transitions);
          hard_capacity_blockers[relation] =
              saturating_count_add(hard_capacity_blockers[relation], rejected_transitions);
        }
      }
      if (status == TaskRouteStatus::incomplete && config.atomic_complete_net_routing) {
        const auto& before = *atomic_current_before;
        const auto attempted_paths = current.paths.size() - before.paths.size();
        const auto attempted_cost = current.total_cost - before.total_cost;
        const auto topology_rejections =
            current.topology_rejected_search_edges - before.topology_rejected_search_edges;
        const auto direct_fallbacks =
            current.direct_voronoi_search_fallbacks - before.direct_voronoi_search_fallbacks;
        const auto fixed_order_fallbacks = current.multisource_steiner_fixed_order_fallbacks -
                                           before.multisource_steiner_fixed_order_fallbacks;
        std::vector<RoutingConnectionFailure> connection_failures(
            current.connection_failures.begin() +
                static_cast<std::ptrdiff_t>(before.connection_failures.size()),
            current.connection_failures.end());
        occupancy = std::move(*atomic_occupancy_before);
        current = std::move(*atomic_current_before);
        current.topology_rejected_search_edges =
            saturating_count_add(current.topology_rejected_search_edges, topology_rejections);
        current.direct_voronoi_search_fallbacks =
            saturating_count_add(current.direct_voronoi_search_fallbacks, direct_fallbacks);
        current.multisource_steiner_fixed_order_fallbacks = saturating_count_add(
            current.multisource_steiner_fixed_order_fallbacks, fixed_order_fallbacks);
        current.connection_failures.insert(current.connection_failures.end(),
                                           std::make_move_iterator(connection_failures.begin()),
                                           std::make_move_iterator(connection_failures.end()));
        current.rolled_back_incomplete_net_trees =
            saturating_count_add(current.rolled_back_incomplete_net_trees, 1U);
        current.rolled_back_incomplete_net_paths =
            saturating_count_add(current.rolled_back_incomplete_net_paths, attempted_paths);
        current.rolled_back_incomplete_net_cost =
            saturating_add(current.rolled_back_incomplete_net_cost, attempted_cost);
      }
      current.path_search_invocations =
          saturating_count_add(current.path_search_invocations, task_search_effort.invocations);
      current.path_search_expansions =
          saturating_count_add(current.path_search_expansions, task_search_effort.expansions);
      current.expansion_limited_searches = saturating_count_add(
          current.expansion_limited_searches, task_search_effort.expansion_limited);
      current.weighted_search_retries = saturating_count_add(current.weighted_search_retries,
                                                             task_search_effort.weighted_retries);
      current.weighted_search_recoveries = saturating_count_add(
          current.weighted_search_recoveries, task_search_effort.weighted_recoveries);
      current.weighted_frontier_continuations =
          saturating_count_add(current.weighted_frontier_continuations,
                               task_search_effort.weighted_frontier_continuations);
      current.weighted_frontier_recoveries = saturating_count_add(
          current.weighted_frontier_recoveries, task_search_effort.weighted_frontier_recoveries);
      current.weighted_frontier_states_reused =
          saturating_count_add(current.weighted_frontier_states_reused,
                               task_search_effort.weighted_frontier_states_reused);
      current.weighted_frontier_continuation_limits =
          saturating_count_add(current.weighted_frontier_continuation_limits,
                               task_search_effort.weighted_frontier_continuation_limits);
      if (status == TaskRouteStatus::incomplete && pending_hard_ripup_rollback &&
          pending_hard_ripup_rollback->failed_net == task.net) {
        occupancy = std::move(pending_hard_ripup_rollback->occupancy_before);
        former_route_memory = std::move(pending_hard_ripup_rollback->former_route_memory_before);
        current.paths = std::move(pending_hard_ripup_rollback->paths_before);
        current.total_cost = pending_hard_ripup_rollback->total_cost_before;
        current.former_route_memory_ripped_paths = former_route_memory.ripped_paths;
        current.former_route_memory_edges = former_route_memory.edges.size();
        routed_this_pass = std::move(pending_hard_ripup_rollback->routed_nets_before);
        routed_cost_this_pass = std::move(pending_hard_ripup_rollback->routed_costs_before);
        for (const auto& victim : pending_hard_ripup_rollback->queued_victims) {
          const auto queued = std::find_if(
              pending_tasks.begin(), pending_tasks.end(),
              [&victim](const NetTask& queued_task) { return queued_task.net == victim; });
          if (queued != pending_tasks.end()) {
            pending_tasks.erase(queued);
          }
        }
        auto& record = hard_capacity_ripup_records[pending_hard_ripup_rollback->record_index];
        record.rolled_back_after_failed_retry = true;
        hard_ripup_retry_nets.erase(task.net);
        hard_capacity_failed_retry_rollbacks =
            saturating_count_add(hard_capacity_failed_retry_rollbacks, 1U);
        pending_hard_ripup_rollback.reset();
      }
      bool deferred_after_ripup = false;
      if (status == TaskRouteStatus::incomplete && enforce_hard_capacity &&
          config.max_hard_capacity_ripup_attempts_per_pass > 0U &&
          pass_hard_ripup_attempts < config.max_hard_capacity_ripup_attempts_per_pass &&
          !hard_ripup_group_membership.contains(task.net) &&
          !hard_ripup_attempted_nets.contains(task.net)) {
        std::vector<std::pair<NetId, std::size_t>> eligible_blockers;
        for (const auto& [blocker, rejected_transitions] : task_hard_capacity_blockers) {
          const bool ordinarily_eligible =
              routed_this_pass.contains(blocker) && !hard_ripup_group_membership.contains(blocker);
          if (ordinarily_eligible && hard_ripup_protected_nets.contains(blocker)) {
            hard_capacity_protected_victim_rejections =
                saturating_count_add(hard_capacity_protected_victim_rejections, 1U);
          } else if (ordinarily_eligible) {
            eligible_blockers.emplace_back(blocker, rejected_transitions);
          }
        }
        std::sort(eligible_blockers.begin(), eligible_blockers.end(),
                  [](const auto& lhs, const auto& rhs) {
                    return lhs.second != rhs.second ? lhs.second > rhs.second
                                                    : lhs.first < rhs.first;
                  });
        std::set<NetId> eligible_nets;
        for (const auto& [blocker, rejected_transitions] : eligible_blockers) {
          static_cast<void>(rejected_transitions);
          eligible_nets.insert(blocker);
        }
        std::vector<NetId> victims;
        bool joint_victims_selected = false;
        if (config.max_hard_capacity_conflict_set_states_per_attempt > 0U) {
          std::size_t best_cleared_observations = 0U;
          std::size_t best_individual_observations = 0U;
          std::size_t evaluated_states = 0U;
          for (const auto& [candidate, candidate_observations] : task_hard_capacity_blocker_sets) {
            static_cast<void>(candidate_observations);
            if (candidate.empty() ||
                candidate.size() > config.max_hard_capacity_ripup_nets_per_attempt ||
                !std::all_of(
                    candidate.begin(), candidate.end(),
                    [&eligible_nets](const NetId& net) { return eligible_nets.contains(net); })) {
              continue;
            }
            if (evaluated_states >= config.max_hard_capacity_conflict_set_states_per_attempt) {
              break;
            }
            ++evaluated_states;
            hard_capacity_conflict_set_states =
                saturating_count_add(hard_capacity_conflict_set_states, 1U);
            std::size_t cleared_observations = 0U;
            for (const auto& [blocker_set, observations] : task_hard_capacity_blocker_sets) {
              if (std::includes(candidate.begin(), candidate.end(), blocker_set.begin(),
                                blocker_set.end())) {
                cleared_observations = saturating_count_add(cleared_observations, observations);
              }
            }
            std::size_t individual_observations = 0U;
            for (const auto& blocker : candidate) {
              individual_observations = saturating_count_add(
                  individual_observations, task_hard_capacity_blockers.at(blocker));
            }
            const bool better = cleared_observations > best_cleared_observations ||
                                (cleared_observations == best_cleared_observations &&
                                 (individual_observations > best_individual_observations ||
                                  (individual_observations == best_individual_observations &&
                                   (victims.empty() || candidate.size() < victims.size() ||
                                    (candidate.size() == victims.size() && candidate < victims)))));
            if (better) {
              victims = candidate;
              best_cleared_observations = cleared_observations;
              best_individual_observations = individual_observations;
            }
          }
          joint_victims_selected = victims.size() > 1U;
        }
        if (victims.empty()) {
          const auto victim_count =
              std::min(eligible_blockers.size(), config.max_hard_capacity_ripup_nets_per_attempt);
          victims.reserve(victim_count);
          for (std::size_t index = 0U; index < victim_count; ++index) {
            victims.push_back(eligible_blockers[index].first);
          }
          std::sort(victims.begin(), victims.end());
        }
        if (!victims.empty()) {
          if (joint_victims_selected) {
            hard_capacity_joint_ripup_attempts =
                saturating_count_add(hard_capacity_joint_ripup_attempts, 1U);
          }
          std::optional<PendingHardRipupRollback> rollback;
          if (config.rollback_failed_hard_capacity_ripup) {
            rollback.emplace(PendingHardRipupRollback{
                task.net, occupancy, former_route_memory, current.paths, current.total_cost,
                routed_this_pass, routed_cost_this_pass, victims, 0U});
          }
          for (const auto& victim : victims) {
            for (const auto& path : current.paths) {
              if (path.net == victim) {
                const auto ripped_paths_before = former_route_memory.ripped_paths;
                remember_ripped_path(path, *grid, former_route_memory);
                former_route_memory_ripped_paths =
                    saturating_count_add(former_route_memory_ripped_paths,
                                         former_route_memory.ripped_paths - ripped_paths_before);
              }
            }
            RouterTelemetryEvent ripped;
            ripped.kind = RouterTelemetryEventKind::route_ripped_up;
            ripped.iteration = iteration;
            ripped.net = victim;
            ripped.other_net = task.net;
            ripped.ripup_count = saturating_count_add(hard_capacity_ripped_nets, 1U);
            ripped.detail = "hard-capacity rip-up; former-route-memory edges=" +
                            std::to_string(former_route_memory.edges.size()) +
                            ", cost=" + std::to_string(iteration_config.former_route_memory_cost);
            emit_telemetry(std::move(ripped));
            remove_net_occupancy(victim, occupancy);
            std::erase_if(current.paths,
                          [&victim](const GlobalRoutePath& path) { return path.net == victim; });
            if (const auto cost = routed_cost_this_pass.find(victim);
                cost != routed_cost_this_pass.end()) {
              current.total_cost =
                  cost->second > current.total_cost ? 0U : current.total_cost - cost->second;
              routed_cost_this_pass.erase(cost);
            }
            routed_this_pass.erase(victim);
          }
          current.former_route_memory_ripped_paths = former_route_memory.ripped_paths;
          current.former_route_memory_edges = former_route_memory.edges.size();
          former_route_memory_edges_peak =
              std::max(former_route_memory_edges_peak, former_route_memory.edges.size());
          current.connection_failures.resize(task_failure_count_before);
          if (config.defer_hard_capacity_ripup_victims_to_pass_tail) {
            for (const auto& victim : victims) {
              pending_tasks.push_back(task_by_net.at(victim));
            }
            hard_capacity_tail_deferred_victims =
                saturating_count_add(hard_capacity_tail_deferred_victims, victims.size());
          } else {
            for (auto victim = victims.rbegin(); victim != victims.rend(); ++victim) {
              pending_tasks.push_front(task_by_net.at(*victim));
            }
          }
          pending_tasks.push_front(task);
          hard_ripup_attempted_nets.insert(task.net);
          hard_ripup_retry_nets.insert(task.net);
          ++pass_hard_ripup_attempts;
          hard_capacity_ripup_attempts = saturating_count_add(hard_capacity_ripup_attempts, 1U);
          hard_capacity_ripped_nets =
              saturating_count_add(hard_capacity_ripped_nets, victims.size());
          const auto record_index = hard_capacity_ripup_records.size();
          hard_capacity_ripup_records.push_back(
              RoutingHardCapacityRipupAttempt{iteration, task.net, victims, false, false, false});
          hard_ripup_record_by_net[task.net] = record_index;
          pass_hard_ripup_records.push_back(record_index);
          if (rollback) {
            rollback->record_index = record_index;
            pending_hard_ripup_rollback = std::move(*rollback);
          }
          deferred_after_ripup = true;
        }
      }
      if (status == TaskRouteStatus::complete) {
        routed_this_pass.insert(task.net);
        routed_cost_this_pass[task.net] =
            current.total_cost >= task_cost_before ? current.total_cost - task_cost_before : 0U;
        if (hard_ripup_retry_nets.erase(task.net) > 0U) {
          hard_capacity_ripup_recoveries = saturating_count_add(hard_capacity_ripup_recoveries, 1U);
          hard_capacity_ripup_records[hard_ripup_record_by_net.at(task.net)].retry_completed = true;
          if (config.protect_recovered_hard_capacity_ripup_nets &&
              hard_ripup_protected_nets.insert(task.net).second) {
            hard_capacity_protected_recoveries =
                saturating_count_add(hard_capacity_protected_recoveries, 1U);
          }
          if (pending_hard_ripup_rollback && pending_hard_ripup_rollback->failed_net == task.net) {
            pending_hard_ripup_rollback.reset();
          }
        }
      } else if (!deferred_after_ripup) {
        current.unrouted_nets.push_back(task.net);
      }
    }

    for (const auto record_index : pass_hard_ripup_records) {
      auto& record = hard_capacity_ripup_records[record_index];
      record.retained_at_pass_end = routed_this_pass.contains(record.failed_net);
    }

    if (clearance_shadow_owner_history_phase) {
      clearance_shadow_owner_history_passes =
          saturating_count_add(clearance_shadow_owner_history_passes, 1U);
    }
    account_routing_occupancy(current, board, *grid, *corridors, corridor_neck_downs,
                              *voronoi_allocation, occupancy, obstacles, config, &history,
                              clearance_shadow_owner_history_phase);
    std::vector<NetId> lane_conflict_affected;
    using SelectedJunctionKey = std::tuple<NetId, std::size_t, std::size_t>;
    std::map<SelectedJunctionKey, std::set<Edge>> sampled_edges_by_exact_junction;
    for (const auto& path : current.paths) {
      if (path.medial_resource_sequence.size() < 2U ||
          path.medial_transition_portals.size() + 1U != path.medial_resource_sequence.size() ||
          path.medial_transition_grid_edges.size() != path.medial_transition_portals.size()) {
        continue;
      }
      for (std::size_t transition_index = 0U;
           transition_index < path.medial_transition_portals.size(); ++transition_index) {
        if (path.medial_transition_portals[transition_index] ||
            !path.medial_transition_grid_edges[transition_index]) {
          continue;
        }
        const auto& selected_edge = *path.medial_transition_grid_edges[transition_index];
        const auto first_node = route_node(*grid, selected_edge.first);
        const auto second_node = route_node(*grid, selected_edge.second);
        if (!first_node || !second_node || *first_node == *second_node) {
          continue;
        }
        const Edge resources{path.medial_resource_sequence[transition_index],
                             path.medial_resource_sequence[transition_index + 1U]};
        sampled_edges_by_exact_junction[{path.net, resources.first, resources.second}].insert(
            Edge{*first_node, *second_node});
      }
    }
    const auto lane_conflict_history_entry_count = [&history]() {
      return saturating_count_add(
          saturating_count_add(
              saturating_count_add(history.direct_portals.size(), history.exact_junctions.size()),
              history.selected_junction_edges.size()),
          history.lane_conflict_resources.size());
    };
    if (config.negotiate_voronoi_lane_conflicts &&
        current.unaccepted_selected_voronoi_lane_transitions > 0U) {
      ++voronoi_lane_conflict_passes;
      std::set<NetId> affected;
      for (const auto& transition : current.voronoi_lane_transitions) {
        if (transition.static_clearance_checked && transition.mutual_clearance_checked) {
          continue;
        }
        voronoi_lane_conflict_observations =
            saturating_count_add(voronoi_lane_conflict_observations, 1U);
        affected.insert(transition.net);
        if (transition.direct_portal_bridge) {
          const auto portal =
              route_node(*grid, GlobalRoutePoint{transition.medial_junction, transition.layer});
          if (!portal) {
            voronoi_lane_conflict_history_withheld =
                saturating_count_add(voronoi_lane_conflict_history_withheld, 1U);
            continue;
          }
          for (const auto resource_id :
               {transition.first_resource_id, transition.second_resource_id}) {
            if (resource_id >= voronoi_allocation->resources.size()) {
              continue;
            }
            const auto& direct_edge = voronoi_allocation->resources[resource_id].direct_edge;
            if (!direct_edge || (direct_edge->first != *portal && direct_edge->second != *portal)) {
              continue;
            }
            const RoutingHistory::DirectPortalKey key{transition.net, resource_id, *portal};
            const auto found = history.direct_portals.find(key);
            if (found == history.direct_portals.end()) {
              const auto history_entries = lane_conflict_history_entry_count();
              if (history_entries >= config.max_voronoi_lane_conflict_history_entries) {
                voronoi_lane_conflict_history_withheld =
                    saturating_count_add(voronoi_lane_conflict_history_withheld, 1U);
                continue;
              }
              history.direct_portals.emplace(key, 1U);
            } else {
              found->second = saturating_count_add(found->second, 1U);
            }
            voronoi_lane_conflict_history_increments =
                saturating_count_add(voronoi_lane_conflict_history_increments, 1U);
          }
        } else {
          const Edge resources{transition.first_resource_id, transition.second_resource_id};
          for (const auto resource_id : {resources.first, resources.second}) {
            const RoutingHistory::LaneConflictResourceKey resource_history_key{transition.net,
                                                                               resource_id};
            const auto resource_history_found =
                history.lane_conflict_resources.find(resource_history_key);
            if (resource_history_found == history.lane_conflict_resources.end()) {
              if (lane_conflict_history_entry_count() >=
                  config.max_voronoi_lane_conflict_history_entries) {
                voronoi_lane_conflict_history_withheld =
                    saturating_count_add(voronoi_lane_conflict_history_withheld, 1U);
                continue;
              }
              history.lane_conflict_resources.emplace(resource_history_key, 1U);
            } else {
              resource_history_found->second =
                  saturating_count_add(resource_history_found->second, 1U);
            }
            voronoi_lane_conflict_history_increments =
                saturating_count_add(voronoi_lane_conflict_history_increments, 1U);
          }
          const RoutingHistory::ExactJunctionKey resource_key{transition.net, resources};
          const auto resource_found = history.exact_junctions.find(resource_key);
          if (resource_found == history.exact_junctions.end()) {
            if (lane_conflict_history_entry_count() >=
                config.max_voronoi_lane_conflict_history_entries) {
              voronoi_lane_conflict_history_withheld =
                  saturating_count_add(voronoi_lane_conflict_history_withheld, 1U);
            } else {
              history.exact_junctions.emplace(resource_key, 1U);
              voronoi_lane_conflict_history_increments =
                  saturating_count_add(voronoi_lane_conflict_history_increments, 1U);
            }
          } else {
            resource_found->second = saturating_count_add(resource_found->second, 1U);
            voronoi_lane_conflict_history_increments =
                saturating_count_add(voronoi_lane_conflict_history_increments, 1U);
          }
          const auto selected_edges = sampled_edges_by_exact_junction.find(
              {transition.net, resources.first, resources.second});
          if (selected_edges == sampled_edges_by_exact_junction.end()) {
            voronoi_lane_conflict_history_withheld =
                saturating_count_add(voronoi_lane_conflict_history_withheld, 1U);
            continue;
          }
          for (const auto selected_edge : selected_edges->second) {
            const RoutingHistory::SelectedJunctionEdgeKey key{transition.net, selected_edge};
            const auto found = history.selected_junction_edges.find(key);
            if (found == history.selected_junction_edges.end()) {
              const auto history_entries = lane_conflict_history_entry_count();
              if (history_entries >= config.max_voronoi_lane_conflict_history_entries) {
                voronoi_lane_conflict_history_withheld =
                    saturating_count_add(voronoi_lane_conflict_history_withheld, 1U);
                continue;
              }
              history.selected_junction_edges.emplace(key, 1U);
            } else {
              found->second = saturating_count_add(found->second, 1U);
            }
            voronoi_lane_conflict_history_increments =
                saturating_count_add(voronoi_lane_conflict_history_increments, 1U);
          }
        }
      }
      voronoi_lane_conflict_coupled_group_expansions =
          saturating_count_add(voronoi_lane_conflict_coupled_group_expansions,
                               expand_affected_routing_groups(board, affected));
      lane_conflict_affected.assign(affected.begin(), affected.end());
      if (iteration < config.max_iterations) {
        voronoi_lane_conflict_scheduled_reroute_nets = saturating_count_add(
            voronoi_lane_conflict_scheduled_reroute_nets, lane_conflict_affected.size());
      }
    }
    if (current.total_overflow > 0U || !current.clearance_shadow_conflicts.empty() ||
        current.unaccepted_selected_voronoi_lane_transitions > 0U) {
      temporary_illegal_checkpoints = saturating_count_add(temporary_illegal_checkpoints, 1U);
    }
    retain_checkpoint(current);
    const auto net_pressure = collect_global_route_net_pressure(current);
    if (enforce_hard_capacity) {
      hard_capacity_pass_summaries.push_back(
          RoutingHardCapacityPass{iteration, std::move(attempted_hard_capacity_order),
                                  current.unrouted_nets, current.connection_failures});
      for (const auto& net : current.unrouted_nets) {
        hard_capacity_failure_passes_by_net[net] =
            saturating_count_add(hard_capacity_failure_passes_by_net[net], 1U);
      }
    }
    for (const auto& pressure : net_pressure) {
      peak_net_pressure = std::max(peak_net_pressure, pressure.legacy_congested_resources);
      peak_net_sampled_overflow = std::max(peak_net_sampled_overflow, pressure.sampled_overflow);
      peak_net_corridor_overflow = std::max(peak_net_corridor_overflow, pressure.corridor_overflow);
      peak_net_total_physical_overflow_nm =
          std::max(peak_net_total_physical_overflow_nm, pressure.total_physical_overflow_nm);
      peak_net_physical_overflow_nm =
          std::max(peak_net_physical_overflow_nm, pressure.peak_physical_overflow_nm);
      peak_net_congested_physical_resources =
          std::max(peak_net_congested_physical_resources, pressure.congested_physical_resources);
    }

    RouterTelemetryEvent iteration_completed;
    iteration_completed.kind = RouterTelemetryEventKind::pathfinder_iteration_completed;
    iteration_completed.iteration = iteration;
    iteration_completed.total_nets = tasks.size();
    iteration_completed.routed_nets = tasks.size() >= current.unrouted_nets.size()
                                          ? tasks.size() - current.unrouted_nets.size()
                                          : 0U;
    iteration_completed.unrouted_nets = current.unrouted_nets.size();
    iteration_completed.sampled_overflow = current.sampled_overflow;
    iteration_completed.corridor_overflow = current.corridor_overflow;
    iteration_completed.physical_overflow_nm = current.total_voronoi_overflow_nm;
    iteration_completed.clearance_shadow_conflicts = current.clearance_shadow_conflicts.size();
    iteration_completed.ripup_count = hard_capacity_ripped_nets;
    iteration_completed.route_cost = current.total_cost;
    iteration_completed.search_expansions = current.path_search_expansions;
    emit_telemetry(std::move(iteration_completed));

    if (global_route_checkpoint_better(current, best, config.checkpoint_objective)) {
      best = current;
      unchanged_iterations = 0;
      RouterTelemetryEvent checkpoint_updated;
      checkpoint_updated.kind = RouterTelemetryEventKind::best_checkpoint_updated;
      checkpoint_updated.iteration = iteration;
      checkpoint_updated.total_nets = tasks.size();
      checkpoint_updated.routed_nets =
          tasks.size() >= best.unrouted_nets.size() ? tasks.size() - best.unrouted_nets.size() : 0U;
      checkpoint_updated.unrouted_nets = best.unrouted_nets.size();
      checkpoint_updated.sampled_overflow = best.sampled_overflow;
      checkpoint_updated.corridor_overflow = best.corridor_overflow;
      checkpoint_updated.physical_overflow_nm = best.total_voronoi_overflow_nm;
      checkpoint_updated.clearance_shadow_conflicts = best.clearance_shadow_conflicts.size();
      checkpoint_updated.ripup_count = hard_capacity_ripped_nets;
      checkpoint_updated.route_cost = best.total_cost;
      checkpoint_updated.search_expansions = best.path_search_expansions;
      emit_telemetry(std::move(checkpoint_updated));
    } else {
      ++unchanged_iterations;
    }
    const bool pending_lane_conflict_negotiation =
        config.negotiate_voronoi_lane_conflicts &&
        current.unaccepted_selected_voronoi_lane_transitions > 0U;
    if (current.unrouted_nets.empty() && current.total_overflow == 0U &&
        !pending_lane_conflict_negotiation) {
      current.stop_reason = GlobalRouterStopReason::complete;
      if (!config.use_free_space_corridors) {
        current.diagnostic = checkpoint == nullptr
                                 ? "all global connections routed without overflow"
                                 : "affected routing cone completed with compatible paths reused";
      } else {
        current.diagnostic =
            checkpoint == nullptr
                ? "all global connections allocated through free-space corridors without overflow"
                : "affected routing cone completed with compatible paths reused through "
                  "free-space corridors";
      }
      if (global_route_checkpoint_better(best, current, config.checkpoint_objective)) {
        best.stop_reason = GlobalRouterStopReason::complete;
        best.iterations = iteration;
        best.diagnostic = "iteration-zero static seed checkpoint remained the best feasible plan";
        return finalize(std::move(best));
      }
      return finalize(std::move(current));
    }
    bool large_neighborhood_selected = false;
    if (config.enable_search_diversification && iteration < config.max_iterations &&
        large_neighborhood_attempts < config.max_large_neighborhood_attempts &&
        unchanged_iterations >= config.large_neighborhood_stagnation_iterations) {
      const auto stage = large_neighborhood_attempts % 5U;
      std::set<NetId> affected(current.unrouted_nets.begin(), current.unrouted_nets.end());
      if (affected.empty() && !net_pressure.empty()) {
        const auto most_constrained = std::max_element(
            net_pressure.begin(), net_pressure.end(), [&](const auto& lhs, const auto& rhs) {
              return routing_net_pressure_more_severe(rhs, lhs, config.checkpoint_objective);
            });
        if (most_constrained != net_pressure.end()) {
          affected.insert(most_constrained->net);
        }
      }
      const auto expand_conflict_neighbors = [&]() {
        const auto before = affected;
        const auto expand_usages = [&](const auto& usages) {
          for (const auto& usage : usages) {
            if (!usage.congested() ||
                !std::any_of(usage.nets.begin(), usage.nets.end(),
                             [&](const auto& net) { return before.contains(net); })) {
              continue;
            }
            affected.insert(usage.nets.begin(), usage.nets.end());
          }
        };
        expand_usages(current.resource_usage);
        expand_usages(current.corridor_usage);
        expand_usages(current.corridor_neck_down_usage);
        expand_usages(current.voronoi_resource_usage);
        for (const auto& conflict : current.clearance_shadow_conflicts) {
          if (before.contains(conflict.first_net) || before.contains(conflict.second_net)) {
            affected.insert(conflict.first_net);
            affected.insert(conflict.second_net);
          }
        }
      };
      expand_conflict_neighbors();
      if (stage >= 1U) {
        expand_conflict_neighbors();
      }
      if (stage >= 2U) {
        expand_conflict_neighbors();
      }
      if (stage >= 3U) {
        const auto before = affected;
        for (const auto& group : current.residual_conflict_groups) {
          if (std::any_of(group.nets.begin(), group.nets.end(),
                          [&](const auto& net) { return before.contains(net); })) {
            affected.insert(group.nets.begin(), group.nets.end());
          }
        }
      }
      if (stage >= 4U) {
        std::vector<Point> terminal_and_via_anchors;
        for (const auto& candidate : all_tasks) {
          if (!affected.contains(candidate.net)) {
            continue;
          }
          for (const auto& terminal : candidate.terminals) {
            terminal_and_via_anchors.push_back(terminal.exact_position);
          }
        }
        for (const auto& path : current.paths) {
          if (!affected.contains(path.net)) {
            continue;
          }
          for (std::size_t index = 1U; index < path.points.size(); ++index) {
            if (path.points[index - 1U].layer != path.points[index].layer) {
              terminal_and_via_anchors.push_back(path.points[index].position);
            }
          }
        }
        const auto cluster_radius_nm = static_cast<long double>(config.grid_pitch.value()) * 2.0L;
        const auto near_cluster = [&](Point point) {
          return std::any_of(
              terminal_and_via_anchors.begin(), terminal_and_via_anchors.end(), [&](Point anchor) {
                return std::abs(static_cast<long double>(point.x.value()) -
                                static_cast<long double>(anchor.x.value())) <= cluster_radius_nm &&
                       std::abs(static_cast<long double>(point.y.value()) -
                                static_cast<long double>(anchor.y.value())) <= cluster_radius_nm;
              });
        };
        for (const auto& candidate : all_tasks) {
          if (std::any_of(
                  candidate.terminals.begin(), candidate.terminals.end(),
                  [&](const auto& terminal) { return near_cluster(terminal.exact_position); })) {
            affected.insert(candidate.net);
          }
        }
        for (const auto& path : current.paths) {
          bool clustered_via = false;
          for (std::size_t index = 1U; index < path.points.size(); ++index) {
            clustered_via = path.points[index - 1U].layer != path.points[index].layer &&
                            near_cluster(path.points[index].position);
            if (clustered_via) {
              break;
            }
          }
          if (clustered_via) {
            affected.insert(path.net);
          }
        }
        std::vector<std::pair<std::uint64_t, NetId>> randomized;
        randomized.reserve(all_tasks.size());
        for (const auto& candidate : all_tasks) {
          randomized.emplace_back(
              stable_pathfinder_group_rank(candidate.net.value(),
                                           large_neighborhood_attempts ^
                                               config.routing_order_diversification_seed),
              candidate.net);
        }
        std::sort(randomized.begin(), randomized.end());
        const auto random_count = std::max<std::size_t>(
            1U, (all_tasks.size() * config.destroy_route_percent + 99U) / 100U);
        for (std::size_t index = 0U; index < std::min(random_count, randomized.size()); ++index) {
          affected.insert(randomized[index].second);
        }
        auto pressure_order = net_pressure;
        std::stable_sort(
            pressure_order.begin(), pressure_order.end(), [&](const auto& lhs, const auto& rhs) {
              return routing_net_pressure_more_severe(lhs, rhs, config.checkpoint_objective);
            });
        for (std::size_t index = 0U; index < std::min(random_count, pressure_order.size());
             ++index) {
          affected.insert(pressure_order[index].net);
        }
      }
      large_neighborhood_coupled_expansions = saturating_count_add(
          large_neighborhood_coupled_expansions, expand_affected_routing_groups(board, affected));
      const std::array<std::string_view, 5U> operators{
          "destroy_hotspot", "destroy_hotspot_and_immediate_neighbours", "destroy_affected_portals",
          "destroy_local_routing_region", "destroy_terminal_via_cluster_and_random_routes"};
      const auto& source =
          checkpoint_beam.empty()
              ? current
              : checkpoint_beam[large_neighborhood_attempts % checkpoint_beam.size()].result;
      std::vector<NetId> affected_nets(affected.begin(), affected.end());
      auto next_reuse = prepare_reuse(source, affected_nets, board, all_tasks, *grid, *corridors,
                                      *voronoi_guidance, corridor_neck_downs, *voronoi_allocation,
                                      obstacles, config, true);
      const std::set<NetId> rerouted(next_reuse.rerouted_nets.begin(),
                                     next_reuse.rerouted_nets.end());
      tasks = all_tasks;
      tasks.erase(std::remove_if(
                      tasks.begin(), tasks.end(),
                      [&](const NetTask& candidate) { return !rerouted.contains(candidate.net); }),
                  tasks.end());
      reuse = std::move(next_reuse);
      large_neighborhood_destroyed_nets =
          saturating_count_add(large_neighborhood_destroyed_nets, rerouted.size());
      large_neighborhood_operators.emplace_back(operators[stage]);
      ++large_neighborhood_attempts;
      unchanged_iterations = 0U;
      large_neighborhood_selected = true;
      RouterTelemetryEvent destruction;
      destruction.kind = RouterTelemetryEventKind::search_summary;
      destruction.stable_id =
          "global-large-neighborhood:" + std::to_string(large_neighborhood_attempts);
      destruction.total_nets = all_tasks.size();
      destruction.unrouted_nets = current.unrouted_nets.size();
      destruction.ripup_count = rerouted.size();
      destruction.detail = "large-neighborhood escape: operator=" + std::string(operators[stage]) +
                           ", unfrozen_nets=" + std::to_string(rerouted.size()) +
                           ", beam_candidates=" + std::to_string(checkpoint_beam.size());
      emit_telemetry(std::move(destruction));
    }
    const auto stagnation_limit =
        config.selective_full_reroute_interval == 0U
            ? std::size_t{5U}
            : std::max(std::size_t{5U},
                       saturating_count_add(config.selective_full_reroute_interval, 1U));
    // A constructive hard-capacity pass can remain incomplete while a later net order succeeds.
    // Honor the caller's explicit strict-pass budget instead of interpreting those expected
    // incomplete checkpoints as soft-objective stagnation.
    const bool hard_capacity_phase_pending = config.hard_capacity_start_iteration > iteration;
    if (!enforce_hard_capacity && !hard_capacity_phase_pending &&
        !pending_lane_conflict_negotiation && unchanged_iterations >= stagnation_limit) {
      const bool owner_history_fallback_available =
          config.reserve_sampled_route_clearance && !current.clearance_shadow_conflicts.empty() &&
          !clearance_shadow_owner_history_phase && iteration < config.max_iterations;
      if (owner_history_fallback_available) {
        // Preserve the ordinary best checkpoint, then give attributed shadow owners one bounded
        // negotiation window on their own resources. If it cannot improve feasibility, the
        // existing checkpoint is returned unchanged at the next stagnation boundary.
        clearance_shadow_owner_history_phase = true;
        unchanged_iterations = 0U;
      } else {
        best.stop_reason = GlobalRouterStopReason::stagnated;
        best.iterations = iteration;
        best.diagnostic = "no objective improvement within the configured stagnation window";
        return finalize(std::move(best));
      }
    }
    if (!large_neighborhood_selected && config.selective_congestion_reroute &&
        iteration < config.max_iterations) {
      selective_reroute_passes = saturating_count_add(selective_reroute_passes, 1U);
      const bool full_refresh =
          config.selective_full_reroute_interval > 0U &&
          selective_reroute_passes % config.selective_full_reroute_interval == 0U;
      if (full_refresh) {
        selective_full_reroute_refreshes =
            saturating_count_add(selective_full_reroute_refreshes, 1U);
        reuse = {};
        tasks = all_tasks;
        reuse.rerouted_nets.reserve(tasks.size());
        for (const auto& task : tasks) {
          reuse.rerouted_nets.push_back(task.net);
        }
      } else {
        std::vector<NetId> hard_blocker_affected;
        if (enforce_hard_capacity &&
            config.max_hard_capacity_blocker_reuse_invalidations_per_failed_net > 0U) {
          const std::set<NetId> failed(current.unrouted_nets.begin(), current.unrouted_nets.end());
          std::map<NetId, std::vector<std::pair<NetId, std::size_t>>> blockers_by_failed_net;
          for (const auto& [relation, rejected_transitions] : iteration_hard_capacity_blockers) {
            if (failed.contains(relation.first)) {
              blockers_by_failed_net[relation.first].emplace_back(relation.second,
                                                                  rejected_transitions);
            }
          }
          std::set<NetId> selected_blockers;
          for (auto& [failed_net, blockers] : blockers_by_failed_net) {
            static_cast<void>(failed_net);
            std::sort(blockers.begin(), blockers.end(), [](const auto& lhs, const auto& rhs) {
              return lhs.second != rhs.second ? lhs.second > rhs.second : lhs.first < rhs.first;
            });
            const auto count =
                std::min(blockers.size(),
                         config.max_hard_capacity_blocker_reuse_invalidations_per_failed_net);
            for (std::size_t index = 0U; index < count; ++index) {
              selected_blockers.insert(blockers[index].first);
            }
          }
          hard_blocker_affected.assign(selected_blockers.begin(), selected_blockers.end());
          hard_capacity_blocker_reuse_invalidations = saturating_count_add(
              hard_capacity_blocker_reuse_invalidations, hard_blocker_affected.size());
        }
        hard_blocker_affected.insert(hard_blocker_affected.end(), lane_conflict_affected.begin(),
                                     lane_conflict_affected.end());
        std::sort(hard_blocker_affected.begin(), hard_blocker_affected.end());
        hard_blocker_affected.erase(
            std::unique(hard_blocker_affected.begin(), hard_blocker_affected.end()),
            hard_blocker_affected.end());
        auto next_reuse = prepare_reuse(current, hard_blocker_affected, board, all_tasks, *grid,
                                        *corridors, *voronoi_guidance, corridor_neck_downs,
                                        *voronoi_allocation, obstacles, config);
        selective_retained_net_observations = saturating_count_add(
            selective_retained_net_observations, next_reuse.reused_nets.size());
        selective_retained_path_observations =
            saturating_count_add(selective_retained_path_observations, next_reuse.paths.size());
        selective_conflict_net_observations = saturating_count_add(
            selective_conflict_net_observations, next_reuse.rerouted_nets.size());
        selective_coupled_group_expansions = saturating_count_add(
            selective_coupled_group_expansions, next_reuse.coupled_group_expansions);
        const std::set<NetId> rerouted(next_reuse.rerouted_nets.begin(),
                                       next_reuse.rerouted_nets.end());
        tasks = all_tasks;
        tasks.erase(std::remove_if(
                        tasks.begin(), tasks.end(),
                        [&rerouted](const NetTask& task) { return !rerouted.contains(task.net); }),
                    tasks.end());
        reuse = std::move(next_reuse);
      }
    }
    if (config.reorder_nets_by_pathfinder_pressure && iteration < config.max_iterations) {
      const auto next_iteration = iteration + 1U;
      const auto next_hard_pass_ordinal =
          config.hard_capacity_start_iteration > 0U &&
                  next_iteration >= config.hard_capacity_start_iteration
              ? next_iteration - config.hard_capacity_start_iteration + 1U
              : 0U;
      const auto route_order_variant =
          config.enable_search_diversification
              ? ((next_iteration - 1U) % config.max_routing_order_variants) + 1U
              : 0U;
      routing_order_variants_attempted = saturating_count_add(routing_order_variants_attempted,
                                                              route_order_variant > 0U ? 1U : 0U);
      const auto moved = reorder_pathfinder_tasks(
          board, net_pressure, current.unrouted_nets,
          config.order_routing_groups_by_physical_envelope, *voronoi_allocation,
          next_hard_pass_ordinal, iteration_hard_capacity_blockers, seed_priority_by_group,
          route_order_variant, config.routing_order_diversification_seed, tasks);
      if (next_hard_pass_ordinal > 0U) {
        hard_capacity_diversified_passes =
            saturating_count_add(hard_capacity_diversified_passes, 1U);
        hard_capacity_diversified_net_positions =
            saturating_count_add(hard_capacity_diversified_net_positions, moved);
      }
      if (moved > 0U) {
        ++reordered_iterations;
        reordered_nets = std::numeric_limits<std::size_t>::max() - reordered_nets < moved
                             ? std::numeric_limits<std::size_t>::max()
                             : reordered_nets + moved;
      }
      last_reroute_order.clear();
      for (const auto& task : tasks) {
        last_reroute_order.push_back(task.net);
      }
    }
  }

  best.stop_reason = best.unrouted_nets.empty() && best.total_overflow == 0U
                         ? GlobalRouterStopReason::complete
                         : GlobalRouterStopReason::iteration_limit;
  best.iterations = config.max_iterations;
  best.diagnostic = best.stop_reason == GlobalRouterStopReason::complete
                        ? "global routing completed; bounded direct-portal lane-conflict "
                          "negotiation exhausted before all advisory bridges were accepted"
                        : "global-router iteration limit reached";
  return finalize(std::move(best));
}

} // namespace

std::vector<RoutingNetPressure> collect_global_route_net_pressure(const GlobalRouteResult& result) {
  std::map<NetId, RoutingNetPressure> pressure;
  const auto item_for = [&pressure](const NetId& net) -> RoutingNetPressure& {
    auto [found, inserted] = pressure.try_emplace(net);
    if (inserted) {
      found->second.net = net;
    }
    return found->second;
  };
  const auto account_discrete = [&](const auto& usages, bool corridor) {
    for (const auto& usage : usages) {
      if (!usage.congested()) {
        continue;
      }
      const auto overflow = usage.demand - usage.capacity;
      for (const auto& net : usage.nets) {
        auto& item = item_for(net);
        item.legacy_congested_resources = saturating_count_add(item.legacy_congested_resources, 1U);
        if (corridor) {
          item.corridor_overflow = saturating_count_add(item.corridor_overflow, overflow);
        } else {
          item.sampled_overflow = saturating_count_add(item.sampled_overflow, overflow);
        }
      }
    }
  };
  account_discrete(result.resource_usage, false);
  account_discrete(result.corridor_usage, true);
  account_discrete(result.corridor_neck_down_usage, true);
  for (const auto& usage : result.voronoi_resource_usage) {
    if (!usage.congested()) {
      continue;
    }
    const auto overflow = usage.demand_nm - usage.capacity_nm;
    for (const auto& net : usage.nets) {
      auto& item = item_for(net);
      item.legacy_congested_resources = saturating_count_add(item.legacy_congested_resources, 1U);
      item.total_physical_overflow_nm = saturating_add(item.total_physical_overflow_nm, overflow);
      item.peak_physical_overflow_nm = std::max(item.peak_physical_overflow_nm, overflow);
      item.congested_physical_resources =
          saturating_count_add(item.congested_physical_resources, 1U);
    }
  }
  for (const auto& conflict : result.clearance_shadow_conflicts) {
    for (const auto& net : {conflict.first_net, conflict.second_net}) {
      auto& item = item_for(net);
      item.legacy_congested_resources = saturating_count_add(item.legacy_congested_resources, 1U);
      item.clearance_shadow_overflow = saturating_count_add(item.clearance_shadow_overflow, 1U);
    }
  }
  std::vector<RoutingNetPressure> output;
  output.reserve(pressure.size());
  for (auto& [net, item] : pressure) {
    static_cast<void>(net);
    output.push_back(std::move(item));
  }
  return output;
}

RoutingNetPressure collect_global_route_group_pressure(const GlobalRouteResult& result,
                                                       const std::vector<NetId>& group_nets) {
  RoutingNetPressure pressure;
  const std::set<NetId> selected(group_nets.begin(), group_nets.end());
  if (selected.empty()) {
    return pressure;
  }
  pressure.net = *selected.begin();
  const auto participates = [&selected](const auto& usage) {
    return std::any_of(usage.nets.begin(), usage.nets.end(),
                       [&selected](const auto& net) { return selected.contains(net); });
  };
  const auto account_discrete = [&](const auto& usages, bool corridor) {
    for (const auto& usage : usages) {
      if (!usage.congested() || !participates(usage)) {
        continue;
      }
      const auto overflow = usage.demand - usage.capacity;
      pressure.legacy_congested_resources =
          saturating_count_add(pressure.legacy_congested_resources, 1U);
      if (corridor) {
        pressure.corridor_overflow = saturating_count_add(pressure.corridor_overflow, overflow);
      } else {
        pressure.sampled_overflow = saturating_count_add(pressure.sampled_overflow, overflow);
      }
    }
  };
  account_discrete(result.resource_usage, false);
  account_discrete(result.corridor_usage, true);
  account_discrete(result.corridor_neck_down_usage, true);
  for (const auto& usage : result.voronoi_resource_usage) {
    if (!usage.congested() || !participates(usage)) {
      continue;
    }
    const auto overflow = usage.demand_nm - usage.capacity_nm;
    pressure.legacy_congested_resources =
        saturating_count_add(pressure.legacy_congested_resources, 1U);
    pressure.total_physical_overflow_nm =
        saturating_add(pressure.total_physical_overflow_nm, overflow);
    pressure.peak_physical_overflow_nm = std::max(pressure.peak_physical_overflow_nm, overflow);
    pressure.congested_physical_resources =
        saturating_count_add(pressure.congested_physical_resources, 1U);
  }
  for (const auto& conflict : result.clearance_shadow_conflicts) {
    if (!selected.contains(conflict.first_net) && !selected.contains(conflict.second_net)) {
      continue;
    }
    pressure.legacy_congested_resources =
        saturating_count_add(pressure.legacy_congested_resources, 1U);
    pressure.clearance_shadow_overflow =
        saturating_count_add(pressure.clearance_shadow_overflow, 1U);
  }
  return pressure;
}

bool routing_net_pressure_more_severe(const RoutingNetPressure& lhs, const RoutingNetPressure& rhs,
                                      GlobalRouteCheckpointObjective objective) {
  if (objective == GlobalRouteCheckpointObjective::dimensioned_physical_overflow) {
    return std::tie(lhs.legacy_congested_resources, lhs.total_physical_overflow_nm,
                    lhs.peak_physical_overflow_nm, lhs.sampled_overflow, lhs.corridor_overflow,
                    lhs.congested_physical_resources) >
           std::tie(rhs.legacy_congested_resources, rhs.total_physical_overflow_nm,
                    rhs.peak_physical_overflow_nm, rhs.sampled_overflow, rhs.corridor_overflow,
                    rhs.congested_physical_resources);
  }
  return lhs.legacy_congested_resources > rhs.legacy_congested_resources;
}

bool global_route_checkpoint_better(const GlobalRouteResult& lhs, const GlobalRouteResult& rhs,
                                    GlobalRouteCheckpointObjective objective) {
  if (objective == GlobalRouteCheckpointObjective::dimensioned_physical_overflow) {
    const auto lhs_key = std::tuple{lhs.unrouted_nets.size(),
                                    lhs.total_voronoi_overflow_nm,
                                    lhs.peak_voronoi_overflow_nm,
                                    lhs.sampled_overflow,
                                    lhs.corridor_overflow,
                                    lhs.congested_voronoi_resources,
                                    lhs.unaccepted_selected_voronoi_lane_transitions,
                                    lhs.total_cost};
    const auto rhs_key = std::tuple{rhs.unrouted_nets.size(),
                                    rhs.total_voronoi_overflow_nm,
                                    rhs.peak_voronoi_overflow_nm,
                                    rhs.sampled_overflow,
                                    rhs.corridor_overflow,
                                    rhs.congested_voronoi_resources,
                                    rhs.unaccepted_selected_voronoi_lane_transitions,
                                    rhs.total_cost};
    return lhs_key != rhs_key ? lhs_key < rhs_key : lhs.paths.size() > rhs.paths.size();
  }
  const auto lhs_key = std::tuple{lhs.unrouted_nets.size(), lhs.total_overflow,
                                  lhs.unaccepted_selected_voronoi_lane_transitions, lhs.total_cost};
  const auto rhs_key = std::tuple{rhs.unrouted_nets.size(), rhs.total_overflow,
                                  rhs.unaccepted_selected_voronoi_lane_transitions, rhs.total_cost};
  return lhs_key != rhs_key ? lhs_key < rhs_key : lhs.paths.size() > rhs.paths.size();
}

GlobalRouteResult route_global(const Board& board, const GlobalRouterConfig& config,
                               std::stop_token stop_token, RouterTelemetrySink* telemetry) {
  return route_global_impl(board, config, stop_token, nullptr, {}, telemetry);
}

GlobalRouteResult route_global_incremental(const Board& board, const GlobalRouteResult& checkpoint,
                                           const std::vector<NetId>& affected_nets,
                                           const GlobalRouterConfig& config,
                                           std::stop_token stop_token,
                                           RouterTelemetrySink* telemetry,
                                           bool preserve_unaffected_provisional_congestion) {
  return route_global_impl(board, config, stop_token, &checkpoint, affected_nets, telemetry,
                           preserve_unaffected_provisional_congestion);
}

} // namespace flux
