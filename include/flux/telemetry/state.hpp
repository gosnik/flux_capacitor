#pragma once

#include "flux/telemetry/event.hpp"

#include <deque>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace flux::telemetry {

struct Activity {
  std::uint64_t session_generation{0};
  std::uint64_t sequence{0};
  std::uint64_t monotonic_time_us{0};
  EventType type{EventType::unknown};
  Level level{Level::normal};
  std::optional<std::string> net;
  std::string message;
};

struct LoggedDiagnostic {
  double value{0.0};
  std::uint64_t updated_time_us{0};
  std::string scope;
  bool operator==(const LoggedDiagnostic&) const = default;
};

// Rebases independent router-process clocks into one append-only viewer timeline. The live
// WebSocket server is intentionally process-local, so route/detail phases can reconnect with
// sequence and monotonic clocks starting at zero. This observer-only adapter preserves their
// ordering and attaches an explicit session boundary without altering event payload semantics.
class LiveSessionTimeline {
public:
  void clear();
  [[nodiscard]] std::optional<Event> begin_session(std::uint64_t generation);
  [[nodiscard]] Event rebase(Event event);

  [[nodiscard]] std::uint64_t generation() const { return generation_; }
  [[nodiscard]] std::uint64_t last_sequence() const { return last_sequence_; }
  [[nodiscard]] std::uint64_t last_monotonic_time_us() const { return last_time_us_; }

private:
  std::uint64_t generation_{0};
  std::uint64_t last_sequence_{0};
  std::uint64_t last_time_us_{0};
  std::uint64_t raw_sequence_origin_{0};
  std::uint64_t raw_time_origin_{0};
  std::uint64_t rebased_sequence_origin_{0};
  std::uint64_t rebased_time_origin_{0};
  bool awaiting_first_event_{false};
};

class LiveState {
public:
  void clear();
  void apply(const Event& event);

  [[nodiscard]] std::uint64_t sequence() const { return sequence_; }
  [[nodiscard]] std::uint64_t observed_sequence_gaps() const { return observed_sequence_gaps_; }
  [[nodiscard]] std::size_t iteration() const { return iteration_; }
  [[nodiscard]] bool routing_active() const { return routing_active_; }
  [[nodiscard]] const std::string& status() const { return status_; }
  [[nodiscard]] const std::string& source_identity() const { return source_identity_; }
  [[nodiscard]] std::uint64_t configuration_revision() const { return configuration_revision_; }
  [[nodiscard]] const std::optional<std::string>& active_net() const { return active_net_; }
  [[nodiscard]] const std::optional<std::string>& active_group() const { return active_group_; }
  [[nodiscard]] const std::map<std::string, Route>& routes() const { return routes_; }
  [[nodiscard]] const std::map<std::string, Via>& vias() const { return vias_; }
  [[nodiscard]] const std::map<std::string, Conflict>& conflicts() const { return conflicts_; }
  [[nodiscard]] const std::map<std::string, ResourceOccupancy>& resources() const {
    return resources_;
  }
  [[nodiscard]] const std::map<std::string, HydraulicField>& hydraulic_fields() const {
    return hydraulic_fields_;
  }
  [[nodiscard]] const std::map<std::string, Priority>& priorities() const { return priorities_; }
  [[nodiscard]] const std::map<std::string, std::shared_ptr<const SourceBoard>>&
  source_boards() const {
    return source_boards_;
  }
  [[nodiscard]] const std::vector<IterationMetrics>& iteration_history() const {
    return iteration_history_;
  }
  [[nodiscard]] const std::optional<IterationMetrics>& metrics() const { return metrics_; }
  [[nodiscard]] const std::optional<PhaseProgress>& progress() const { return progress_; }
  [[nodiscard]] const std::optional<ActiveRegion>& active_region() const {
    return active_region_;
  }
  [[nodiscard]] const std::optional<std::uint64_t>& last_metrics_time_us() const {
    return last_metrics_time_us_;
  }
  [[nodiscard]] const std::optional<std::uint64_t>& last_geometry_time_us() const {
    return last_geometry_time_us_;
  }
  [[nodiscard]] const Counters& counters() const { return counters_; }
  [[nodiscard]] const std::map<std::string, LoggedDiagnostic>& logged_diagnostics() const {
    return logged_diagnostics_;
  }
  [[nodiscard]] const std::deque<Activity>& activity() const { return activity_; }
  [[nodiscard]] std::uint64_t dropped_oversized_geometry_updates() const {
    return dropped_oversized_geometry_updates_;
  }

private:
  void apply_snapshot(const Event& event);
  void append_activity(const Event& event);
  [[nodiscard]] bool mandatory_geometry_fits(const Event& event, bool reset_boundary) const;
  void trim_geometry_to_budget();

  std::uint64_t sequence_{0};
  std::uint64_t observed_sequence_gaps_{0};
  std::size_t iteration_{0};
  bool routing_active_{false};
  std::string status_{"Waiting for telemetry"};
  std::string source_identity_;
  std::uint64_t configuration_revision_{0};
  std::optional<std::string> active_net_;
  std::optional<std::string> active_group_;
  std::map<std::string, Route> routes_;
  std::map<std::string, Via> vias_;
  std::map<std::string, Conflict> conflicts_;
  std::map<std::string, ResourceOccupancy> resources_;
  std::map<std::string, HydraulicField> hydraulic_fields_;
  std::map<std::string, Priority> priorities_;
  std::map<std::string, std::shared_ptr<const SourceBoard>> source_boards_;
  std::vector<IterationMetrics> iteration_history_;
  std::optional<IterationMetrics> metrics_;
  std::optional<PhaseProgress> progress_;
  std::optional<ActiveRegion> active_region_;
  std::optional<std::uint64_t> last_metrics_time_us_;
  std::optional<std::uint64_t> last_geometry_time_us_;
  Counters counters_;
  std::map<std::string, LoggedDiagnostic> logged_diagnostics_;
  std::deque<Activity> activity_;
  std::uint64_t dropped_oversized_geometry_updates_{0U};
};

// Chronological, viewer-side history of non-authoritative telemetry. Events are retained as a
// bounded delta window behind one reduced base state, so recovery snapshots never multiply full
// geometry copies. Immutable source-board scenes remain shared through LiveState.
class LiveStateHistory {
public:
  void clear();
  void append(const Event& event);
  void seek(std::size_t event_count);

  [[nodiscard]] const LiveState& state() const { return state_; }
  [[nodiscard]] std::size_t position() const { return position_; }
  // Positions remain monotonic across trimming so callers can keep a paused/replay cursor.
  [[nodiscard]] std::size_t size() const { return base_position_ + events_.size(); }
  [[nodiscard]] std::size_t retained_begin() const { return base_position_; }
  [[nodiscard]] std::uint64_t dropped_events() const { return dropped_events_; }
  [[nodiscard]] std::size_t retained_bytes() const { return retained_bytes_; }
  [[nodiscard]] const Event* event(std::size_t index) const {
    return index >= base_position_ && index - base_position_ < events_.size()
               ? &events_[index - base_position_]
               : nullptr;
  }

private:
  static constexpr std::size_t maximum_retained_events = 4'096U;
  static constexpr std::size_t maximum_retained_bytes = 64U * 1024U * 1024U;
  std::vector<Event> events_;
  LiveState tail_state_;
  LiveState state_;
  LiveState base_state_;
  std::size_t base_position_{0U};
  std::size_t position_{0U};
  std::uint64_t dropped_events_{0U};
  std::size_t retained_bytes_{0U};
};

} // namespace flux::telemetry
