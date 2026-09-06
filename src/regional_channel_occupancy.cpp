#include "flux/core/geometry.hpp"
#include "flux/core/regional_router.hpp"

#include <algorithm>
#include <map>
#include <numeric>
#include <tuple>

namespace flux {

std::vector<std::size_t>
regional_template_copper_aliases(const std::vector<RegionalRouteFragment>& fragments,
                                 const std::vector<RegionalRouteTemplate>& templates,
                                 bool share_identical) {
  std::vector<std::size_t> aliases(templates.size());
  std::iota(aliases.begin(), aliases.end(), 0U);
  if (!share_identical)
    return aliases;
  std::map<RegionalRouteFragmentId, const RegionalRouteFragment*> by_id;
  for (const auto& f : fragments)
    by_id.emplace(f.id, &f);
  using Key = std::tuple<NetId, LayerId, Nanometres, Nanometres, bool, bool, std::vector<Point>>;
  std::map<Key, std::size_t> identities;
  for (std::size_t i = 0; i < templates.size(); ++i) {
    const auto& t = templates[i];
    const auto found = by_id.find(t.fragment);
    if (found == by_id.end())
      continue;
    const auto& f = *found->second;
    if (f.width.value() <= 0 || f.clearance.value() < 0 || f.id.net.value().empty() ||
        t.routing_layer.value().empty())
      continue;
    auto points = t.topology_centerline;
    if (points.size() < 2U && t.selected_global_topology && f.layer == t.routing_layer)
      points = {f.start, f.end};
    points.erase(std::unique(points.begin(), points.end()), points.end());
    if (points.size() < 2U)
      continue;
    auto reverse = points;
    std::reverse(reverse.begin(), reverse.end());
    if (reverse < points)
      points = std::move(reverse);
    Key key{f.id.net,         t.routing_layer,  f.width,          f.clearance,
            f.explicit_width, f.coupled_member, std::move(points)};
    aliases[i] = identities.emplace(std::move(key), i).first->second;
  }
  return aliases;
}

std::vector<RegionalChannelDemandScope>
regional_channel_demand_scopes(const std::vector<RegionalRouteFragment>& fragments,
                               const std::vector<RegionalChannel>& channels,
                               const std::vector<RegionalRouteTemplate>& templates,
                               bool local_occupancy, bool centerline_cuts) {
  std::map<RegionalRouteFragmentId, const RegionalRouteFragment*> fragment_index;
  for (const auto& fragment : fragments)
    fragment_index.emplace(fragment.id, &fragment);
  std::map<std::size_t, const RegionalChannel*> channel_index;
  for (const auto& channel : channels)
    channel_index.emplace(channel.id, &channel);
  std::map<std::size_t, std::vector<std::size_t>> users;
  for (std::size_t index = 0; index < templates.size(); ++index) {
    auto ids = templates[index].channel_sequence;
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
    for (const auto id : ids)
      users[id].push_back(index);
  }
  std::vector<RegionalChannelDemandScope> result;
  for (const auto& [id, occupants] : users) {
    const auto found = channel_index.find(id);
    if (found == channel_index.end()) {
      // An unknown channel must still fail the capacity check, not disappear.
      result.push_back({id, {}, occupants, Nanometres{0}, false});
      continue;
    }
    const auto& channel = *found->second;
    RegionalChannelDemandScope whole{id, {}, occupants, Nanometres{0}, false};
    for (std::size_t s = 0; s < channel.cross_sections.size(); ++s) {
      whole.cross_section_indices.push_back(s);
      whole.maximum_width = std::max(whole.maximum_width, channel.cross_sections[s].usable_width);
    }
    // Legacy recordings and degenerate walls cannot establish longitudinal support.
    if (!local_occupancy || channel.cross_sections.empty() ||
        std::any_of(channel.cross_sections.begin(), channel.cross_sections.end(),
                    [](const auto& s) { return s.first_wall == s.second_wall; })) {
      result.push_back(std::move(whole));
      continue;
    }
    std::vector<std::vector<std::size_t>> support(channel.cross_sections.size());
    for (const auto index : occupants) {
      const auto& route = templates[index];
      const auto f = fragment_index.find(route.fragment);
      std::vector<Point> points = route.topology_centerline;
      bool known = f != fragment_index.end() && f->second->width.value() > 0 &&
                   f->second->clearance.value() >= 0 && route.routing_layer == channel.layer;
      if (known && points.size() < 2U && route.selected_global_topology &&
          f->second->layer == channel.layer && f->second->start != f->second->end)
        points = {f->second->start, f->second->end};
      known = known && points.size() >= 2U;
      std::vector<std::size_t> touched;
      if (known) {
        const auto radius = centerline_cuts
                                ? 0.0L
                                : static_cast<long double>(f->second->width.value()) / 2.0L +
                                      static_cast<long double>(f->second->clearance.value());
        for (std::size_t s = 0; s < channel.cross_sections.size(); ++s) {
          const auto& wall = channel.cross_sections[s];
          // Walls are finite physical resources, not infinite transverse strips.
          // The default includes envelope contact. The experimental cut bound
          // includes only centerline contact; it cannot certify nearby clearance.
          for (std::size_t p = 1; p < points.size(); ++p) {
            if (geometry::segment_distance_nm(points[p - 1U], points[p], wall.first_wall,
                                              wall.second_wall) <= radius) {
              touched.push_back(s);
              break;
            }
          }
        }
      }
      if (touched.empty()) {
        // No evidence of a local section is not evidence of zero resource use.
        for (auto& section : support)
          section.push_back(index);
      } else {
        for (const auto s : touched)
          support[s].push_back(index);
      }
    }
    std::map<std::vector<std::size_t>, std::vector<std::size_t>> groups;
    for (std::size_t s = 0; s < support.size(); ++s)
      if (!support[s].empty())
        groups[support[s]].push_back(s);
    if (groups.size() == 1U && groups.begin()->first == occupants) {
      result.push_back(std::move(whole));
      continue;
    }
    for (auto& [members, sections] : groups) {
      Nanometres width;
      for (const auto s : sections)
        width = std::max(width, channel.cross_sections[s].usable_width);
      result.push_back({id, std::move(sections), members, width, true});
    }
  }
  return result;
}

} // namespace flux
