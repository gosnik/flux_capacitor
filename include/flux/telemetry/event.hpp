#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace flux::telemetry {

inline constexpr std::string_view protocol_name = "flux.telemetry.v1";
inline constexpr std::uint32_t telemetry_schema_version = 1;

enum class Level { normal, diagnostic, trace };

enum class EventType {
  snapshot,
  routing_started,
  routing_finished,
  net_queued,
  net_started,
  net_completed,
  net_failed,
  route_candidate_selected,
  route_committed,
  route_removed,
  route_ripped_up,
  via_candidate_generated,
  via_candidate_rejected,
  via_assigned,
  via_removed,
  pathfinder_iteration_started,
  pathfinder_iteration_completed,
  conflict_added,
  conflict_removed,
  resource_occupancy_updated,
  hydraulic_field_updated,
  residual_group_updated,
  checkpoint_selected,
  best_checkpoint_updated,
  placement_proposal_started,
  placement_proposal_accepted,
  placement_proposal_rejected,
  search_summary,
  unknown
};

enum class RouteState { current, provisional, committed, ripped_up, removed };
enum class RouteGeometry { polyline, quintic_ph };
enum class ViaState { candidate, selected, contested, rejected, removed };
enum class ConflictKind { congestion, clearance_shadow, regional, physical, unknown };
enum class ResourceKind { sampled, corridor, physical, via_site, unknown };

struct Point {
  std::int64_t x_nm{0};
  std::int64_t y_nm{0};
  std::string layer;

  bool operator==(const Point&) const = default;
};

struct Route {
  std::string id;
  std::string net;
  std::uint64_t revision{0};
  RouteState state{RouteState::current};
  std::uint64_t width_nm{0};
  std::uint64_t cost{0};
  std::vector<Point> points;
  RouteGeometry geometry{RouteGeometry::polyline};
  std::uint32_t spline_degree{0};
  std::vector<Point> control_points;
  std::uint64_t topology_signature{0};
  std::size_t topology_gate_count{0U};
  bool topology_gate_sequence_legal{false};
  std::string family;
  std::string lifecycle;

  bool operator==(const Route&) const = default;
};

struct Via {
  std::string id;
  std::string net;
  std::uint64_t revision{0};
  Point position;
  std::string to_layer;
  std::uint64_t diameter_nm{0};
  double score{0.0};
  std::int64_t clearance_margin_nm{0};
  std::size_t competing_nets{0};
  double local_congestion{0.0};
  std::size_t remaining_alternatives{0};
  ViaState state{ViaState::candidate};
  std::string rejection_reason;

  bool operator==(const Via&) const = default;
};

struct Conflict {
  std::string id;
  ConflictKind kind{ConflictKind::unknown};
  std::string resource;
  std::string group;
  Point location;
  std::vector<std::string> nets;
  std::int64_t clearance_deficit_nm{0};
  double pressure{0.0};
  double history{0.0};

  bool operator==(const Conflict&) const = default;
};

struct ResourceOccupancy {
  std::string id;
  ResourceKind kind{ResourceKind::unknown};
  std::string layer;
  Point start;
  Point end;
  double demand{0.0};
  double capacity{0.0};
  std::uint64_t channel_width_nm{0};
  std::vector<std::string> nets;

  [[nodiscard]] double utilization() const;
  bool operator==(const ResourceOccupancy&) const = default;
};

// Bounded diagnostic heatmap. Values are row-major normalized 16-bit maxima over `stride`
// prepared NavMesh cells. Geometry remains advisory and layer-specific.
struct HydraulicField {
  std::string id;
  std::string net;
  std::string layer;
  std::uint64_t revision{0};
  std::int64_t origin_x_nm{0};
  std::int64_t origin_y_nm{0};
  std::uint64_t cell_pitch_nm{0};
  std::size_t width{0U};
  std::size_t height{0U};
  std::size_t stride{0U};
  std::uint32_t maximum_flow{0U};
  std::uint32_t maximum_erosion{0U};
  std::uint32_t maximum_terrain_height{0U};
  std::uint32_t maximum_downstream_pressure{0U};
  std::uint32_t maximum_success_prior{0U};
  std::vector<std::uint16_t> flow;
  std::vector<std::uint16_t> erosion;
  std::vector<std::uint16_t> heightmap;
  std::vector<std::uint16_t> free_space;
  std::vector<std::uint16_t> downstream_pressure;
  std::vector<std::uint16_t> success_prior;

  bool operator==(const HydraulicField&) const = default;
};

struct Priority {
  std::string net;
  std::string group;
  std::string staged_class;
  double explicit_priority{0.0};
  double terminal_scarcity{0.0};
  double corridor_scarcity{0.0};
  double via_scarcity{0.0};
  double routing_interference{0.0};
  double physical_width_demand{0.0};
  std::size_t topology_alternatives{0};
  std::size_t layer_alternatives{0};
  double final_priority{0.0};
  std::uint64_t stable_tie_break{0};

  bool operator==(const Priority&) const = default;
};

// Immutable, KiCad-derived scene evidence. Generic primitives keep the live protocol independent
// of the complete canonical router IR while retaining enough exact integer geometry for a viewer
// to draw the source board and distinguish object roles.
struct SourceLayer {
  std::string id;
  std::string name;
  std::string kind;
  std::uint32_t stack_order{0};

  bool operator==(const SourceLayer&) const = default;
};

struct SourceNet {
  std::string id;
  std::string name;
  std::uint32_t priority{0};

  bool operator==(const SourceNet&) const = default;
};

struct SourcePrimitive {
  std::string id;
  std::string kind;
  std::string parent;
  std::string reference;
  std::string net;
  std::vector<std::string> layers;
  std::vector<Point> points;
  std::vector<std::vector<Point>> holes;
  std::uint64_t width_nm{0};
  std::uint64_t height_nm{0};
  std::uint64_t diameter_nm{0};
  std::uint64_t drill_nm{0};
  std::int64_t rotation_microdegrees{0};
  std::string shape;
  std::string side;
  bool locked{false};
  bool filled{false};

  bool operator==(const SourcePrimitive&) const = default;
};

struct SourceBoard {
  std::string id;
  std::string path;
  std::string revision;
  std::uint32_t board_schema{0};
  std::vector<SourceLayer> layers;
  std::vector<SourceNet> nets;
  std::vector<SourcePrimitive> primitives;

  bool operator==(const SourceBoard&) const = default;
};

struct IterationMetrics {
  std::size_t iteration{0};
  std::size_t total_nets{0};
  std::size_t routed_nets{0};
  std::size_t unrouted_nets{0};
  std::size_t total_groups{0};
  std::size_t routed_groups{0};
  std::size_t unrouted_groups{0};
  std::size_t sampled_overflow{0};
  std::size_t corridor_overflow{0};
  std::uint64_t physical_overflow_nm{0};
  std::size_t clearance_shadow_conflicts{0};
  std::size_t regional_conflicts{0};
  std::uint64_t route_cost{0};
  std::size_t vias{0};
  std::size_t rip_ups{0};
  std::size_t search_expansions{0};
  std::uint64_t selected_checkpoint{0};
  std::uint64_t best_checkpoint{0};
  bool selected_is_best{false};

  bool operator==(const IterationMetrics&) const = default;
};

struct Counters {
  std::uint64_t produced{0};
  std::uint64_t dropped{0};
  std::uint64_t coalesced{0};
  std::uint64_t serialized{0};
  std::uint64_t snapshots{0};

  bool operator==(const Counters&) const = default;
};

// One representation is used inside a live WebSocket batch and as one JSONL trace record.
// Snapshot events carry complete bounded collections; incremental events normally carry one item.
struct Event {
  std::uint32_t schema{telemetry_schema_version};
  std::uint64_t sequence{0};
  std::uint64_t monotonic_time_us{0};
  Level level{Level::normal};
  EventType type{EventType::unknown};
  std::string source_identity;
  std::uint64_t configuration_revision{0};
  std::optional<std::string> active_net;
  std::optional<std::string> active_group;
  // Legacy summary snapshots omit these arrays and must not erase incrementally received geometry.
  bool source_state_included{false};
  bool dynamic_state_included{false};
  std::vector<std::shared_ptr<const SourceBoard>> source_boards;
  std::vector<Route> routes;
  std::vector<Via> vias;
  std::vector<Conflict> conflicts;
  std::vector<ResourceOccupancy> resources;
  std::vector<HydraulicField> hydraulic_fields;
  std::vector<Priority> priorities;
  std::optional<IterationMetrics> metrics;
  std::optional<Counters> counters;
  std::string message;

  bool operator==(const Event&) const = default;
};

struct DecodeResult {
  std::vector<Event> events;
  std::string error;

  [[nodiscard]] explicit operator bool() const { return error.empty(); }
};

[[nodiscard]] std::string_view to_string(Level value);
[[nodiscard]] std::string_view to_string(EventType value);
[[nodiscard]] std::string_view to_string(RouteState value);
[[nodiscard]] std::string_view to_string(RouteGeometry value);
[[nodiscard]] std::string_view to_string(ViaState value);
[[nodiscard]] std::string_view to_string(ConflictKind value);
[[nodiscard]] std::string_view to_string(ResourceKind value);

[[nodiscard]] Level level_from_string(std::string_view value);
[[nodiscard]] EventType event_type_from_string(std::string_view value);
[[nodiscard]] RouteState route_state_from_string(std::string_view value);
[[nodiscard]] RouteGeometry route_geometry_from_string(std::string_view value);
[[nodiscard]] ViaState via_state_from_string(std::string_view value);
[[nodiscard]] ConflictKind conflict_kind_from_string(std::string_view value);
[[nodiscard]] ResourceKind resource_kind_from_string(std::string_view value);

[[nodiscard]] DecodeResult decode_events(std::string_view json_or_jsonl);
[[nodiscard]] std::string encode_event(const Event& event);

} // namespace flux::telemetry
