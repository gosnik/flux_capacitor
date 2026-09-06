#include "flux/core/regional_router.hpp"

#include <algorithm>
#include <iostream>
#include <stdexcept>

namespace {
using namespace flux;
Point point(std::int64_t x, std::int64_t y) { return {Nanometres{x}, Nanometres{y}}; }
void require(bool ok, const char* message) {
  if (!ok)
    throw std::runtime_error(message);
}
void check(bool reverse, bool rotate) {
  const auto transform = [&](Point p) { return rotate ? Point{p.y, p.x} : p; };
  RegionalChannel channel;
  channel.id = 19;
  channel.layer = LayerId{"F.Cu"};
  std::vector<RegionalRouteFragment> fragments;
  std::vector<RegionalRouteTemplate> templates;
  for (std::size_t i = 0; i < 3; ++i) {
    const std::int64_t x = i == 1 ? 5'000'000 : 0;
    const std::int64_t y = i == 2 ? 5'000'000 : 0;
    RegionalRouteFragment f;
    f.id = {NetId{"support-" + std::to_string(i)}, 0, 0, 0};
    f.layer = channel.layer;
    f.start = transform(point(x, y));
    f.end = transform(point(x + 2'000'000, y));
    f.width = Nanometres{250'000};
    f.clearance = Nanometres{200'000};
    fragments.push_back(f);
    RegionalRouteTemplate t;
    t.id = 97 - i; // Metadata IDs deliberately differ from vector positions.
    t.fragment = f.id;
    t.routing_layer = f.layer;
    t.channel_sequence = {channel.id, channel.id};
    templates.push_back(t);
    RegionalCrossSection section;
    section.first_wall = transform(point(x + 1'000'000, y - 400'000));
    section.second_wall = transform(point(x + 1'000'000, y + 400'000));
    if (reverse)
      std::swap(section.first_wall, section.second_wall);
    section.usable_width = Nanometres{800'000};
    channel.cross_sections.push_back(section);
  }
  const auto scopes = regional_channel_demand_scopes(fragments, {channel}, templates);
  require(scopes.size() == 3, "remote longitudinal and lateral resources must remain separate");
  for (std::size_t i = 0; i < scopes.size(); ++i) {
    require(scopes[i].channel_id == channel.id && scopes[i].localized &&
                scopes[i].template_indices == std::vector<std::size_t>{i} &&
                scopes[i].cross_section_indices == std::vector<std::size_t>{i} &&
                scopes[i].maximum_width == Nanometres{800'000},
            "scopes must preserve exact section/portfolio identity without duplicate demand");
  }
  const auto legacy = regional_channel_demand_scopes(fragments, {channel}, templates, false);
  require(legacy.size() == 1 && !legacy.front().localized &&
              legacy.front().template_indices.size() == 3,
          "disabled local occupancy must preserve whole-channel evidence");

  auto unknown = templates;
  unknown.front().selected_global_topology = false; // No alternative polyline recorded.
  for (const auto& s : regional_channel_demand_scopes(fragments, {channel}, unknown))
    require(std::binary_search(s.template_indices.begin(), s.template_indices.end(), 0),
            "missing alternate geometry must remain conservatively charged");
  unknown.front().topology_centerline = {fragments.front().start, fragments.front().end};
  require(regional_channel_demand_scopes(fragments, {channel}, unknown).size() == 3,
          "an explicit alternate polyline must restore local evidence");
  unknown.front().routing_layer = LayerId{"B.Cu"};
  for (const auto& s : regional_channel_demand_scopes(fragments, {channel}, unknown))
    require(std::binary_search(s.template_indices.begin(), s.template_indices.end(), 0),
            "wrong-layer evidence must not establish local support");

  auto invalid = channel;
  invalid.cross_sections.front().second_wall = invalid.cross_sections.front().first_wall;
  const auto conservative = regional_channel_demand_scopes(fragments, {invalid}, templates);
  require(conservative.size() == 1 && !conservative.front().localized,
          "degenerate walls must retain legacy evidence");
  const auto missing = regional_channel_demand_scopes(fragments, {}, templates);
  require(missing.size() == 1 && missing.front().maximum_width == Nanometres{0} &&
              missing.front().template_indices.size() == 3,
          "unknown channels must remain zero-capacity resources rather than disappear");
}
} // namespace

int main() {
  try {
    for (const bool reverse : {false, true})
      for (const bool rotate : {false, true})
        check(reverse, rotate);
    std::cout << "PASS finite channel occupancy and conservative fallback\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
