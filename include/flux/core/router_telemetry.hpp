#pragma once

#include "flux/core/ids.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace flux {

struct Board;
namespace telemetry {
struct SourceBoard;
}

enum class RouterTelemetryLevel { normal, diagnostic, trace };

enum class RouterTelemetryEventKind {
  routing_started,
  routing_finished,
  pathfinder_iteration_started,
  pathfinder_iteration_completed,
  best_checkpoint_updated,
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
  conflict_added,
  conflict_removed,
  residual_group_updated,
  checkpoint_selected,
  search_summary,
  placement_proposal_started,
  placement_proposal_accepted,
  placement_proposal_rejected
};

struct RouterTelemetryPathPoint {
  std::int64_t x_nm{0};
  std::int64_t y_nm{0};
  LayerId layer;

  bool operator==(const RouterTelemetryPathPoint&) const = default;
};

enum class RouterTelemetryRouteGeometry { polyline, cubic_bspline };

// Compact semantic evidence copied from an authoritative algorithm boundary. Unused fields stay
// zero/empty for that event kind. This object is diagnostic only and is never read by the router.
struct RouterTelemetryEvent {
  RouterTelemetryEventKind kind{RouterTelemetryEventKind::routing_started};
  RouterTelemetryLevel minimum_level{RouterTelemetryLevel::normal};
  std::uint64_t sequence{0};
  std::uint64_t route_revision{0};
  std::uint64_t route_width_nm{0};
  std::uint64_t candidate_cost{0};
  std::vector<RouterTelemetryPathPoint> route_points;
  RouterTelemetryRouteGeometry route_geometry{RouterTelemetryRouteGeometry::polyline};
  std::uint32_t spline_degree{0};
  std::vector<RouterTelemetryPathPoint> spline_control_points;
  std::uint64_t topology_signature{0};
  std::size_t topology_gate_count{0U};
  bool topology_gate_sequence_legal{false};
  std::size_t iteration{0};
  std::size_t total_nets{0};
  std::size_t routed_nets{0};
  std::size_t unrouted_nets{0};
  std::size_t sampled_overflow{0};
  std::size_t corridor_overflow{0};
  std::uint64_t physical_overflow_nm{0};
  std::size_t clearance_shadow_conflicts{0};
  std::size_t via_count{0};
  std::size_t ripup_count{0};
  std::uint64_t route_cost{0};
  std::size_t search_expansions{0};
  std::size_t total_groups{0};
  std::size_t regional_conflicts{0};
  std::string priority_stage;
  std::uint32_t explicit_priority{0};
  std::size_t terminal_portals{0};
  std::size_t corridor_alternatives{0};
  std::size_t via_transitions{0};
  std::size_t via_alternatives{0};
  std::size_t via_contention{0};
  std::size_t estimated_interference{0};
  std::uint64_t physical_envelope_nm{0};
  std::size_t topology_alternatives{0};
  std::size_t layer_alternatives{0};
  std::size_t priority_ordinal{0};
  NetId net;
  NetId other_net;
  LayerId layer;
  LayerId last_layer;
  std::int64_t x_nm{0};
  std::int64_t y_nm{0};
  std::uint64_t diameter_nm{0};
  double score{0.0};
  std::int64_t clearance_margin_nm{0};
  std::size_t competing_nets{0};
  double local_congestion{0.0};
  std::size_t remaining_alternatives{0};
  std::string rejection_reason;
  std::string resource_id;
  std::string group_id;
  std::string conflict_kind;
  std::int64_t clearance_deficit_nm{0};
  double pressure{0.0};
  double history{0.0};
  std::string stable_id;
  std::string detail;

  bool operator==(const RouterTelemetryEvent&) const = default;
};

class RouterTelemetrySink {
public:
  virtual ~RouterTelemetrySink() = default;

  // Lets an instrumentation boundary avoid preparing diagnostic-only evidence. This is observer
  // configuration, never routing state, and may only suppress work that has no algorithmic side
  // effects.
  [[nodiscard]] virtual bool accepts(RouterTelemetryLevel level) const noexcept = 0;

  // Producers never wait. False means the observation was dropped; routing must ignore it.
  [[nodiscard]] virtual bool try_emit(RouterTelemetryEvent event) noexcept = 0;
};

struct RouterTelemetryConfig {
  std::size_t queue_capacity{4096};
  std::size_t websocket_session_queue_capacity{64};
  std::uint32_t publish_rate_hz{10};
  RouterTelemetryLevel level{RouterTelemetryLevel::normal};
  std::optional<std::string> listen_address{"127.0.0.1"};
  std::optional<std::uint16_t> listen_port{8765U};
  std::optional<std::filesystem::path> record_path;

  [[nodiscard]] bool websocket_enabled() const {
    return listen_address.has_value() && listen_port.has_value();
  }
  bool operator==(const RouterTelemetryConfig&) const = default;
};

struct RouterTelemetryState {
  std::uint64_t sequence{0};
  std::size_t pathfinder_iteration{0};
  std::size_t total_nets{0};
  std::size_t routed_nets{0};
  std::size_t unrouted_nets{0};
  std::size_t active_conflicts{0};
  std::size_t total_groups{0};
  std::size_t regional_conflicts{0};
  std::size_t sampled_conflicts{0};
  std::size_t corridor_conflicts{0};
  std::size_t clearance_shadow_conflicts{0};
  std::uint64_t physical_overflow_nm{0};
  std::size_t via_count{0};
  std::size_t ripup_count{0};
  std::uint64_t route_cost{0};
  std::size_t search_expansions{0};
  NetId active_net;
  std::string source_identity;
  bool routing_finished{false};
  bool dynamic_state_included{false};
  bool source_state_included{false};
  std::vector<std::shared_ptr<const telemetry::SourceBoard>> source_boards;
  std::vector<RouterTelemetryEvent> observed_priorities;
  std::vector<RouterTelemetryEvent> observed_routes;
  std::vector<RouterTelemetryEvent> observed_vias;
  std::vector<RouterTelemetryEvent> observed_conflicts;

  bool operator==(const RouterTelemetryState&) const = default;
};

struct RouterTelemetryStatistics {
  std::uint64_t produced{0};
  std::uint64_t consumed{0};
  std::uint64_t dropped{0};
  std::uint64_t coalesced{0};
  std::uint64_t snapshots{0};
  std::uint64_t websocket_messages{0};
  std::uint64_t websocket_dropped{0};
  std::uint64_t transport_failures{0};
  std::uint64_t recorder_failures{0};

  bool operator==(const RouterTelemetryStatistics&) const = default;
};

// Owns only diagnostic state. Construction starts the consumer and optional Beast I/O threads;
// destruction drains what is already queued and stops them. Failure is observable but never thrown
// into routing after construction.
class RouterTelemetryService final : public RouterTelemetrySink {
public:
  explicit RouterTelemetryService(RouterTelemetryConfig config);
  ~RouterTelemetryService() override;

  RouterTelemetryService(const RouterTelemetryService&) = delete;
  RouterTelemetryService& operator=(const RouterTelemetryService&) = delete;

  [[nodiscard]] bool accepts(RouterTelemetryLevel level) const noexcept override;
  [[nodiscard]] bool try_emit(RouterTelemetryEvent event) noexcept override;
  // Publishes immutable source geometry before routing starts. Scene preparation occurs on the
  // caller/main thread; JSON serialization and socket I/O remain on the telemetry thread.
  void publish_source_board(const Board& board, std::string source_path);
  void stop() noexcept;
  [[nodiscard]] RouterTelemetryState state() const;
  [[nodiscard]] RouterTelemetryStatistics statistics() const;
  [[nodiscard]] const RouterTelemetryConfig& config() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

[[nodiscard]] std::string_view router_telemetry_event_kind_name(RouterTelemetryEventKind kind);
[[nodiscard]] std::string serialize_router_telemetry_event(const RouterTelemetryEvent& event);
[[nodiscard]] std::string
serialize_router_telemetry_snapshot(const RouterTelemetryState& state,
                                    const RouterTelemetryStatistics& statistics);

} // namespace flux
