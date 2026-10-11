#pragma once

#include "flux/core/connectivity.hpp"

#include <map>
#include <string>
#include <utility>

namespace flux {
// Component numbers are report-local. Preserve the membership of every old pad group,
// while allowing the incoming same-net connection to merge formerly separate groups.
[[nodiscard]] inline bool preserves_existing_pad_components(
    const ConnectivityReport& before, const ConnectivityReport& after) {
  using PadKey = std::pair<std::string, std::string>;
  using ComponentKey = std::pair<std::string, std::size_t>;
  std::map<PadKey, std::size_t> after_component;
  for (const auto& pad : after.pad_components)
    if (!after_component.emplace(PadKey{pad.net.value(), pad.pad.value()}, pad.component).second)
      return false;
  std::map<ComponentKey, std::size_t> retained_membership;
  for (const auto& pad : before.pad_components) {
    const auto found = after_component.find({pad.net.value(), pad.pad.value()});
    if (found == after_component.end()) return false;
    const auto [group, inserted] = retained_membership.emplace(
        ComponentKey{pad.net.value(), pad.component}, found->second);
    if (!inserted && group->second != found->second) return false;
  }
  return true;
}
} // namespace flux
