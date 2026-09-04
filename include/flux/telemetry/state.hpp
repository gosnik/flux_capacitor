#pragma once

#include "flux/telemetry/event.hpp"

#include <deque>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace flux::telemetry {

struct Activity {
  std::uint64_t sequence{0};
  std::uint64_t monotonic_time_us{0};
  EventType type{EventType::unknown};
  Level level{Level::normal};
  std::optional<std::string> net;
  std::string message;
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
  [[nodiscard]] const Counters& counters() const { return counters_; }
  [[nodiscard]] const std::deque<Activity>& activity() const { return activity_; }

private:
  void apply_snapshot(const Event& event);
  void append_activity(const Event& event);

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
  Counters counters_;
  std::deque<Activity> activity_;
};

// Chronological, viewer-side history of non-authoritative telemetry. Events are retained as
// deltas and periodically reduced into checkpoints so timeline seeks do not require a complete
// geometry copy for every event. Immutable source-board scenes remain shared through LiveState.
class LiveStateHistory {
public:
  void clear();
  void append(const Event& event);
  void seek(std::size_t event_count);

  [[nodiscard]] const LiveState& state() const { return state_; }
  [[nodiscard]] std::size_t position() const { return position_; }
  [[nodiscard]] std::size_t size() const { return events_.size(); }
  [[nodiscard]] const Event* event(std::size_t index) const {
    return index < events_.size() ? &events_[index] : nullptr;
  }

private:
  struct Checkpoint {
    std::size_t position{0U};
    LiveState state;
  };

  static constexpr std::size_t checkpoint_interval = 128U;
  std::vector<Event> events_;
  std::vector<Checkpoint> checkpoints_;
  LiveState tail_state_;
  LiveState state_;
  std::size_t position_{0U};
};

} // namespace flux::telemetry
