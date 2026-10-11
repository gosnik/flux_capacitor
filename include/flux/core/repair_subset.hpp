#pragma once

#include <algorithm>
#include <cstddef>
#include <functional>
#include <vector>

namespace flux {
// Each group is an indivisible set of dependent moves. The oracle must assess the entire hybrid
// scene against the immutable baseline, including all restored groups, on EVERY invocation.
struct RepairSubsetAssessment {
  std::vector<std::size_t> blocked_groups;
  bool strictly_improved{false};
};
struct RepairSubsetResult {
  std::vector<bool> retained;
  std::size_t assessments{0U};
  bool accepted{false};
};
inline RepairSubsetResult retain_safe_repair_subset(
    std::size_t group_count, std::size_t max_assessments,
    const std::function<RepairSubsetAssessment(const std::vector<bool>&)>& assess) {
  RepairSubsetResult result{std::vector<bool>(group_count, true)};
  while (result.assessments < max_assessments &&
         std::any_of(result.retained.begin(), result.retained.end(), [](bool v) { return v; })) {
    const auto assessment = assess(result.retained);
    ++result.assessments;
    bool removed = false;
    for (const auto group : assessment.blocked_groups) {
      if (group >= group_count) { // An invalid certificate must never authorize a repair.
        result.retained.assign(group_count, false);
        return result;
      }
      if (result.retained[group]) {
        result.retained[group] = false;
        removed = true;
      }
    }
    if (removed) continue; // Restoration changes the scene: prior checks cannot be reused.
    if (assessment.blocked_groups.empty() && assessment.strictly_improved) {
      result.accepted = true;
      return result;
    }
    break;
  }
  // Exhausting the bound after restoration is not evidence that the remaining moves are safe.
  result.retained.assign(group_count, false);
  return result;
}
} // namespace flux
