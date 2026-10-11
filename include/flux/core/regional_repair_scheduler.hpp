#pragma once

#include <array>
#include <cstdint>
#include <optional>

namespace flux {

// Repair scopes increase in coordination cost. The ordinal is intentionally stable: it defines
// deterministic tie order and makes scheduler observations serializable without wall-clock data.
enum class RegionalRepairFamily : std::uint8_t {
  geometry = 0U,
  single = 1U,
  pair = 2U,
  multi = 3U,
};

enum class RegionalRepairScheduleReason : std::uint8_t {
  unavailable,
  warmup,
  exploration,
  productivity,
  no_gain_round_robin,
};

struct RegionalRepairFamilyObservation {
  bool available{false};
  std::uint32_t attempts{0U};
  // Accepted reduction in physical-error count, accumulated for this family in the current
  // immutable regional scheduling scope. This is a deterministic quality signal, not timing.
  std::uint32_t accepted_physical_error_reduction{0U};
  // Deterministic proxy for native primitive/check work consumed by those attempts.
  std::uint32_t work_units{0U};
};

struct RegionalRepairScheduleDecision {
  std::optional<RegionalRepairFamily> family;
  RegionalRepairScheduleReason reason{RegionalRepairScheduleReason::unavailable};
};

// State contains scheduling order only. It deliberately excludes elapsed time: callers may
// publish elapsed time as diagnostics, but it must not alter a repeated routing result.
struct RegionalRepairSchedulerState {
  std::uint32_t decisions{0U};
  std::uint8_t exploration_tie_cursor{0U};
  std::uint8_t no_gain_round_robin_cursor{0U};
};

[[nodiscard]] constexpr std::uint8_t regional_repair_family_index(
    const RegionalRepairFamily family) {
  return static_cast<std::uint8_t>(family);
}

[[nodiscard]] constexpr RegionalRepairFamily regional_repair_family_from_index(
    const std::uint8_t index) {
  return static_cast<RegionalRepairFamily>(index % 4U);
}

// Selects one repair family without mutating the supplied observations. The caller records the
// attempted family and feeds the updated deterministic counters into the next call.
[[nodiscard]] inline RegionalRepairScheduleDecision schedule_regional_repair_family(
    const std::array<RegionalRepairFamilyObservation, 4U>& observations,
    RegionalRepairSchedulerState& state) {
  const auto available = [&](const std::uint8_t index) { return observations[index].available; };
  const auto make_decision = [&](const std::uint8_t index,
                                 const RegionalRepairScheduleReason reason) {
    ++state.decisions;
    return RegionalRepairScheduleDecision{regional_repair_family_from_index(index), reason};
  };

  // Every newly available family receives one attempt before productivity can dominate.
  for (std::uint8_t index = 0U; index < 4U; ++index) {
    if (available(index) && observations[index].attempts == 0U) {
      return make_decision(index, RegionalRepairScheduleReason::warmup);
    }
  }

  bool any_available = false;
  for (std::uint8_t index = 0U; index < 4U; ++index) any_available = any_available || available(index);
  if (!any_available) return {};

  // The fourth, eighth, ... scheduled decision explores the least-tried eligible family. A
  // rotating tie cursor prevents a persistent low-ordinal bias among equally tried families.
  if (state.decisions % 4U == 3U) {
    std::uint32_t least_attempts = UINT32_MAX;
    for (std::uint8_t index = 0U; index < 4U; ++index) {
      if (available(index)) least_attempts =
          observations[index].attempts < least_attempts ? observations[index].attempts : least_attempts;
    }
    for (std::uint8_t offset = 0U; offset < 4U; ++offset) {
      const auto index = static_cast<std::uint8_t>((state.exploration_tie_cursor + offset) % 4U);
      if (available(index) && observations[index].attempts == least_attempts) {
        state.exploration_tie_cursor = static_cast<std::uint8_t>((index + 1U) % 4U);
        return make_decision(index, RegionalRepairScheduleReason::exploration);
      }
    }
  }

  bool any_gain = false;
  std::uint8_t best = 0U;
  for (std::uint8_t index = 0U; index < 4U; ++index) {
    if (!available(index) || observations[index].accepted_physical_error_reduction == 0U) continue;
    if (!any_gain) {
      any_gain = true;
      best = index;
      continue;
    }
    // Compare reduction / max(1, work) exactly without floating point. The products fit in
    // uint64 because both inputs are uint32.
    const auto candidate_work = observations[index].work_units == 0U ? 1U : observations[index].work_units;
    const auto best_work = observations[best].work_units == 0U ? 1U : observations[best].work_units;
    if (static_cast<std::uint64_t>(observations[index].accepted_physical_error_reduction) *
            static_cast<std::uint64_t>(best_work) >
        static_cast<std::uint64_t>(observations[best].accepted_physical_error_reduction) *
            static_cast<std::uint64_t>(candidate_work)) {
      best = index;
    }
  }
  if (any_gain) return make_decision(best, RegionalRepairScheduleReason::productivity);

  // With no accepted reduction, all available families receive deterministic round-robin turns.
  for (std::uint8_t offset = 0U; offset < 4U; ++offset) {
    const auto index = static_cast<std::uint8_t>((state.no_gain_round_robin_cursor + offset) % 4U);
    if (available(index)) {
      state.no_gain_round_robin_cursor = static_cast<std::uint8_t>((index + 1U) % 4U);
      return make_decision(index, RegionalRepairScheduleReason::no_gain_round_robin);
    }
  }
  return {};
}

} // namespace flux
