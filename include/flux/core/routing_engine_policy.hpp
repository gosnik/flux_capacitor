#pragma once

#include "flux/core/detailed_router.hpp"
#include "flux/core/regional_router.hpp"
#include "flux/core/routability_knowledge.hpp"

#include <array>
#include <compare>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace flux {

enum class RoutingEngineAction;
// Lower lexicographically is better. Known failures always rank ahead of missing evidence.
struct RoutingQuality {
  static constexpr std::size_t unknown_rank = std::numeric_limits<std::size_t>::max() / 4U;
  std::array<std::size_t, 7U> ranks{};
  [[nodiscard]] auto operator<=>(const RoutingQuality&) const = default;
};

[[nodiscard]] RoutingQuality routing_quality(const RoutingPlanResult* plan,
                                             const DetailedRouteResult* detailed);
[[nodiscard]] RoutingEngineAction diagnosis_action(const RoutabilityDiagnosis& diagnosis,
                                                   bool xpbd_enabled, bool erosion_enabled);
[[nodiscard]] bool ready_for_materialization(const RoutingPlanResult& plan,
                                             const std::vector<RoutabilityDiagnosis>& diagnoses);

// Standalone scheduling data for bounded engine experiments.  `strategy` deliberately remains
// an adapter-owned stable integer: the policy can be tested without coupling to engine actions.
using RoutingExperimentStrategyId = std::size_t;

struct RoutingPhysicalEvidence {
  bool complete{false};
  std::size_t open_connections{std::numeric_limits<std::size_t>::max()};
  std::size_t drc_violations{std::numeric_limits<std::size_t>::max()};
  std::size_t malformed_paths{std::numeric_limits<std::size_t>::max()};
  bool operator==(const RoutingPhysicalEvidence&) const = default;
};

// Coarse global overflow is intentionally absent: only complete physical evidence affects
// experiment priority. Opens, DRC, and malformed geometry are quality gates.
[[nodiscard]] long double routing_physical_gain(const RoutingPhysicalEvidence& before,
                                                const RoutingPhysicalEvidence& after);

struct RoutingExperimentCandidate {
  RoutingExperimentStrategyId strategy{0U};
  std::string state_key;
  bool eligible{true};
  // Lower values win only the initial-exploration tie; measured throughput governs later turns.
  std::size_t diagnostic_priority{0U};
};

struct RoutingExperimentTrial {
  RoutingExperimentStrategyId strategy{0U};
  std::string state_key;
  std::uint64_t active_elapsed_us{0U};
  long double physical_gain{0.0L};
  std::uint64_t dispatched_sequence{0U};
  bool exhausted{false};
  bool stagnant{false};
  bool operator==(const RoutingExperimentTrial&) const = default;
};

struct RoutingExperimentScheduleState {
  std::vector<RoutingExperimentTrial> trials;
  std::uint64_t last_dispatched_sequence{0U};
  bool operator==(const RoutingExperimentScheduleState&) const = default;
};

struct RoutingExperimentSelection {
  RoutingExperimentStrategyId strategy{0U};
  std::string state_key;
  bool initial_exploration{false};
  std::size_t diagnostic_priority{0U};
  long double rolling_gain_per_us{0.0L};
};

// Pure deterministic selection.  A trial marked exhausted or stagnant suppresses only that
// strategy/state pair; a changed state key gets fresh initial exploration.  Callers append the
// resulting trial after execution and assign sequence monotonically.
[[nodiscard]] std::optional<RoutingExperimentSelection>
select_routing_experiment(const std::vector<RoutingExperimentCandidate>& candidates,
                          const RoutingExperimentScheduleState& state);
} // namespace flux
