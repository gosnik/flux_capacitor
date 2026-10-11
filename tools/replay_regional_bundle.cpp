// Diagnostic replay of a recorded regional template allocator. No copper is produced.
#include "flux/core/geometry.hpp"
#include "flux/core/regional_router.hpp"
#include "flux/core/routing_rules.hpp"
#include "flux/io/kicad_import.hpp"

#include <boost/json.hpp>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>

namespace {
namespace json = boost::json;
std::string string(const json::value& v) { return std::string(v.as_string()); }
std::int64_t number(const json::value& v) { return v.to_number<std::int64_t>(); }
std::size_t index(const json::value& v) { return v.to_number<std::size_t>(); }
flux::Point point(const json::value& v) {
  return {flux::Nanometres{number(v.at(0))}, flux::Nanometres{number(v.at(1))}};
}
flux::RegionalRouteFragmentId fragment_id(const json::value& v) {
  return {flux::NetId{string(v.at(0))}, index(v.at(1)), index(v.at(2)), index(v.at(3))};
}
std::vector<std::size_t> indices(const json::value& v) {
  std::vector<std::size_t> out;
  for (const auto& x : v.as_array())
    out.push_back(index(x));
  return out;
}
std::vector<flux::LayerId> layers(const json::value& v) {
  std::vector<flux::LayerId> out;
  for (const auto& x : v.as_array())
    out.emplace_back(string(x));
  return out;
}
std::optional<std::size_t> optional_index(const json::value& v) {
  return v.is_null() ? std::nullopt : std::optional<std::size_t>{index(v)};
}
json::array forced_capacity_conflicts(const flux::RegionalRouteResult& regional,
                                      const std::vector<flux::RegionalChannelDemandScope>& scopes,
                                      const std::vector<flux::RegionalRouteTemplate>& templates,
                                      const std::vector<std::vector<std::size_t>>& domains) {
  const auto aliases = flux::regional_template_copper_aliases(
      regional.fragments, templates, regional.configured.shared_channel_copper);
  std::map<flux::RegionalRouteFragmentId, const flux::RegionalRouteFragment*> fragments;
  for (const auto& f : regional.fragments)
    fragments.emplace(f.id, &f);
  json::array conflicts;
  for (std::size_t i = 0; i < scopes.size(); ++i) {
    const auto& scope = scopes[i];
    std::int64_t demand = 0, clearance = 0;
    json::array forced;
    json::array contacts;
    std::set<std::size_t> charged;
    for (const auto& domain : domains) {
      if (!std::all_of(domain.begin(), domain.end(), [&](const auto id) {
            return std::binary_search(scope.template_indices.begin(), scope.template_indices.end(),
                                      id);
          }))
        continue;
      const auto found = fragments.find(templates[domain.front()].fragment);
      if (found == fragments.end())
        continue;
      const auto& f = *found->second;
      if (regional.configured.shared_channel_copper) {
        // A varying domain may alias another branch in some selections. Omit it
        // from this lower bound; summing it could report a false mandatory core.
        if (!std::all_of(domain.begin(), domain.end(),
                         [&](const auto id) { return aliases[id] == aliases[domain.front()]; }) ||
            !charged.insert(aliases[domain.front()]).second)
          continue;
      }
      demand += f.width.value() + f.clearance.value();
      clearance = std::max(clearance, f.clearance.value());
      forced.emplace_back(json::array{f.id.net.value(), f.id.connection_index, f.id.run_index,
                                      f.id.fragment_index});
      const auto channel = std::find_if(regional.channels.begin(), regional.channels.end(),
                                        [&](const auto& c) { return c.id == scope.channel_id; });
      auto points = templates[domain.front()].topology_centerline;
      if (points.size() < 2 && templates[domain.front()].selected_global_topology &&
          templates[domain.front()].routing_layer == f.layer)
        points = {f.start, f.end};
      if (channel != regional.channels.end() && points.size() >= 2) {
        for (const auto section : scope.cross_section_indices) {
          if (section >= channel->cross_sections.size())
            continue;
          const auto& wall = channel->cross_sections[section];
          if (wall.first_wall == wall.second_wall)
            continue;
          auto distance = std::numeric_limits<long double>::infinity();
          for (std::size_t p = 1; p < points.size(); ++p)
            distance = std::min(
                distance, flux::geometry::segment_distance_nm(points[p - 1], points[p],
                                                              wall.first_wall, wall.second_wall));
          contacts.emplace_back(json::object{
              {"template_index", domain.front()},
              {"domain_size", domain.size()},
              {"section", section},
              {"distance_nm", static_cast<double>(distance)},
              {"centerline_contact", distance <= 0.01L},
              {"copper_contact", distance <= f.width.value() / 2.0L},
              {"clearance_contact", distance <= f.width.value() / 2.0L + f.clearance.value()}});
        }
      }
    }
    demand += clearance;
    if (demand <= scope.maximum_width.value())
      continue;
    json::array sections;
    for (const auto s : scope.cross_section_indices)
      sections.push_back(s);
    conflicts.emplace_back(json::object{{"scope", i},
                                        {"channel", scope.channel_id},
                                        {"localized", scope.localized},
                                        {"sections", std::move(sections)},
                                        {"forced_fragments", std::move(forced)},
                                        {"first_domain_template_contacts", std::move(contacts)},
                                        {"demand_nm", demand},
                                        {"maximum_width_nm", scope.maximum_width.value()}});
  }
  return conflicts;
}
} // namespace

int main(int argc, char** argv) {
  try {
    std::optional<std::size_t> maximum_states;
    if (argc >= 3 && std::string_view(argv[argc - 2]) == "--max-states") {
      const std::string value(argv[argc - 1]);
      if (value.empty() || value.find_first_not_of("0123456789") != std::string::npos)
        throw std::runtime_error("--max-states must be a positive integer");
      maximum_states = std::stoull(value);
      if (*maximum_states == 0)
        throw std::runtime_error("--max-states must be positive");
      argc -= 2;
    }
    if (argc != 3 && argc != 5)
      throw std::runtime_error("usage: flux-replay-regional-bundle BOARD.kicad_pcb "
                               "CONTEXT.json | TELEMETRY.jsonl GROUP OCCURRENCE [--max-states N]");
    auto imported = flux::io::import_kicad_pcb_file(argv[1]);
    if (!imported.ok())
      throw std::runtime_error("cannot import source board");
    std::ifstream input(argv[2]);
    if (!input)
      throw std::runtime_error("cannot read bundle context");
    json::value context;
    if (argc == 3) {
      context = json::parse(std::string(std::istreambuf_iterator<char>(input), {}));
    } else {
      auto occurrence = std::stoul(argv[4]);
      if (occurrence == 0)
        throw std::runtime_error("occurrence must be one-based");
      std::string line;
      while (std::getline(input, line)) {
        const auto event = json::parse(line);
        const auto* group = event.as_object().if_contains("active_group");
        if (group && string(*group) == argv[3] && --occurrence == 0) {
          context = json::parse(string(event.at("message")));
          break;
        }
      }
      if (context.is_null())
        throw std::runtime_error("requested context occurrence not found");
    }
    const auto schema = index(context.at("schema"));
    if (schema != 4U)
      throw std::runtime_error("bundle context requires schema 4 realization-mode provenance; "
                               "use the matching historical replay executable for older captures");
    const auto& board = *imported.board;
    if (string(context.at("source_revision")) != board.source_revision)
      throw std::runtime_error("bundle context belongs to a different source revision");
    const json::array minimum_rules{
        board.rules.minimum_clearance.value(),     board.rules.minimum_track_width.value(),
        board.rules.minimum_via_diameter.value(),  board.rules.minimum_via_annular_width.value(),
        board.rules.minimum_through_drill.value(), board.rules.copper_to_edge_clearance.value(),
        board.rules.hole_clearance.value(),        board.rules.hole_to_hole_clearance.value()};
    if (context.at("board_minimum_rules_nm") != minimum_rules)
      throw std::runtime_error("recorded project minimum rules differ from replay rules");
    json::object net_rules;
    for (const auto& net : board.nets) {
      const auto rules = flux::resolve_net_routing_rules(board, net.id);
      net_rules[net.id.value()] = json::array{rules.clearance.value(),
                                              rules.track_width.value(),
                                              rules.via_diameter.value(),
                                              rules.via_drill.value(),
                                              rules.differential_pair_track_width.value(),
                                              rules.differential_pair_gap.value(),
                                              rules.differential_pair_via_gap.value()};
    }
    if (context.at("net_routing_rules_nm") != net_rules)
      throw std::runtime_error("recorded net routing rules differ from replay rules");
    flux::RegionalRouteResult regional;
    regional.configured.max_access_assignment_states = index(context.at("max_access_states"));
    regional.configured.max_access_blockers_per_window = index(context.at("max_access_blockers"));
    const auto required_mode = [&](const char* name) {
      const auto* value = context.as_object().if_contains(name);
      if (value == nullptr || !value->is_bool())
        throw std::runtime_error("schema 4 bundle context requires boolean " + std::string(name));
      return value->as_bool();
    };
    regional.configured.frozen_copper_context = required_mode("frozen_copper_context");
    regional.configured.enable_spline_realization = required_mode("enable_spline_realization");
    regional.configured.enable_straight_realization = required_mode("enable_straight_realization");
    flux::RegionalRoutingWindow window;
    window.id = index(context.at("window"));
    for (const auto& x : context.at("nets").as_array())
      window.nets.emplace_back(string(x));
    for (const auto& x : context.at("owned").as_array())
      window.owned_fragments.push_back(fragment_id(x));
    for (const auto& x : context.at("fragments").as_array()) {
      flux::RegionalRouteFragment f;
      f.id = fragment_id(x.at("id"));
      f.layer = flux::LayerId{string(x.at("layer"))};
      f.start = point(x.at("start"));
      f.end = point(x.at("end"));
      f.width = flux::Nanometres{number(x.at("width"))};
      f.clearance = flux::Nanometres{number(x.at("clearance"))};
      f.explicit_width = x.at("explicit_width").as_bool();
      if (const auto* coupled = x.as_object().if_contains("coupled_member"))
        f.coupled_member = coupled->as_bool();
      f.begins_at_terminal = x.at("begins_at_terminal").as_bool();
      f.ends_at_terminal = x.at("ends_at_terminal").as_bool();
      regional.fragments.push_back(f);
    }
    std::vector<flux::RegionalRouteTemplate> templates;
    for (const auto& x : context.at("templates").as_array()) {
      flux::RegionalRouteTemplate t;
      t.id = index(x.at("id"));
      t.fragment = fragment_id(x.at("fragment"));
      t.routing_layer = flux::LayerId{string(x.at("layer"))};
      t.channel_sequence = indices(x.at("channels"));
      t.layer_sequence = layers(x.at("layers"));
      t.via_choices = indices(x.at("vias"));
      t.selected_global_topology = x.at("global").as_bool();
      t.entry_via_choice = optional_index(x.at("entry_via"));
      t.exit_via_choice = optional_index(x.at("exit_via"));
      t.terminal_entry_choice = optional_index(x.at("entry_terminal"));
      t.terminal_exit_choice = optional_index(x.at("exit_terminal"));
      if (schema >= 3U)
        for (const auto& p : x.at("centerline").as_array())
          t.topology_centerline.push_back(point(p));
      templates.push_back(t);
    }
    for (const auto& x : context.at("channels").as_array()) {
      flux::RegionalChannel c;
      c.id = index(x.at("id"));
      if (schema >= 3U)
        c.layer = flux::LayerId{string(x.at("layer"))};
      for (const auto& width : x.at("widths").as_array()) {
        flux::RegionalCrossSection s;
        s.usable_width = flux::Nanometres{number(width)};
        c.cross_sections.push_back(s);
      }
      if (schema >= 3U) {
        if (x.at("walls").as_array().size() != c.cross_sections.size())
          throw std::runtime_error("cross-section geometry/width count mismatch");
        for (std::size_t i = 0; i < c.cross_sections.size(); ++i) {
          c.cross_sections[i].first_wall = point(x.at("walls").at(i).at(0));
          c.cross_sections[i].second_wall = point(x.at("walls").at(i).at(1));
        }
      }
      regional.channels.push_back(c);
    }
    for (const auto& x : context.at("vias").as_array()) {
      flux::RegionalViaChoice v;
      v.id = index(x.at("id"));
      v.region_id = index(x.at("region"));
      v.net = flux::NetId{string(x.at("net"))};
      v.site = point(x.at("site"));
      v.span = layers(x.at("span"));
      v.diameter = flux::Nanometres{number(x.at("diameter"))};
      v.drill = flux::Nanometres{number(x.at("drill"))};
      v.selected_global_site = x.at("global").as_bool();
      for (const auto& r : x.at("runups").as_array()) {
        flux::RegionalViaRunup runup;
        runup.layer = flux::LayerId{string(r.at("layer"))};
        runup.anchor = point(r.at("anchor"));
        for (const auto& p : r.at("points").as_array())
          runup.centerline.push_back(point(p));
        v.attached_runups.push_back(runup);
      }
      regional.via_choices.push_back(v);
    }
    for (const auto& x : context.at("terminals").as_array()) {
      flux::RegionalTerminalChoice t;
      t.id = index(x.at("id"));
      t.net = flux::NetId{string(x.at("net"))};
      t.layer = flux::LayerId{string(x.at("layer"))};
      t.width = flux::Nanometres{number(x.at("width"))};
      for (const auto& p : x.at("points").as_array())
        t.access_path.push_back(point(p));
      regional.terminal_choices.push_back(t);
    }
    for (const auto& x : context.at("evidence").as_array()) {
      flux::RegionalViaRegionEvidence e;
      e.region_id = index(x.at("region"));
      e.net = flux::NetId{string(x.at("net"))};
      e.span = layers(x.at("span"));
      e.competing_regions = index(x.at("competing"));
      e.conflicting_candidate_pairs = index(x.at("conflicting_pairs"));
      e.terminal_escape_conflicts = index(x.at("terminal_conflicts"));
      regional.via_region_evidence.push_back(e);
    }
    std::vector<std::vector<std::size_t>> domains;
    for (const auto& x : context.at("domains").as_array())
      domains.push_back(indices(x));
    for (const auto& domain : domains) {
      if (domain.empty())
        throw std::runtime_error("empty recorded template domain");
      for (const auto t : domain)
        if (t >= templates.size())
          throw std::runtime_error("invalid recorded template reference");
      window.fragments.push_back(templates[domain.front()].fragment);
    }
    for (const bool cuts : {false, true}) {
      if (cuts && schema < 3U)
        continue;
      regional.configured.centerline_channel_cuts = cuts;
      for (const bool shared : {false, true}) {
        if (shared && schema < 3U)
          continue;
        regional.configured.shared_channel_copper = shared;
        const auto aliases =
            flux::regional_template_copper_aliases(regional.fragments, templates, shared);
        const std::set<std::size_t> identities(aliases.begin(), aliases.end());
        for (const bool local : {false, true}) {
          if (cuts && !local)
            continue;
          if (local && schema < 3U)
            continue;
          regional.configured.local_channel_occupancy = local;
          const auto scopes = flux::regional_channel_demand_scopes(
              regional.fragments, regional.channels, templates, local, cuts);
          const auto capacity_conflicts =
              forced_capacity_conflicts(regional, scopes, templates, domains);
          for (const auto [forward, resources] :
               {std::pair{false, false}, std::pair{true, false}, std::pair{true, true}}) {
            regional.configured.enable_bundle_resource_forward_checking = resources;
            const auto start = std::chrono::steady_clock::now();
            const auto result = flux::enumerate_regional_bundle_choices(
                board, regional, window, templates, domains,
                maximum_states.value_or(index(context.at("max_states"))), forward);
            const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                                     std::chrono::steady_clock::now() - start)
                                     .count();
            json::object output{
                {"authoritative", false},
                {"recorded_max_states", index(context.at("max_states"))},
                {"replay_max_states", maximum_states.value_or(index(context.at("max_states")))},
                {"window", window.id},
                {"frozen_copper_context", regional.configured.frozen_copper_context},
                {"enable_spline_realization", regional.configured.enable_spline_realization},
                {"enable_straight_realization", regional.configured.enable_straight_realization},
                {"forward_checking", forward},
                {"resource_forward_checking", resources},
                {"local_channel_occupancy", local},
                {"shared_channel_copper", shared},
                {"centerline_channel_cuts", cuts},
                {"unique_copper_identities", identities.size()},
                {"aliased_templates", templates.size() - identities.size()},
                {"coupling_metadata_recorded",
                 std::all_of(
                     context.at("fragments").as_array().begin(),
                     context.at("fragments").as_array().end(),
                     [](const auto& f) { return f.as_object().contains("coupled_member"); })},
                {"forced_capacity_bound",
                 shared ? "fixed_copper_identity_domains_only" : "all_forced_domains"},
                {"demand_scopes", scopes.size()},
                {"localized_scopes", std::count_if(scopes.begin(), scopes.end(),
                                                   [](const auto& s) { return s.localized; })},
                {"forced_capacity_conflicts", capacity_conflicts},
                {"choice_semantics", forward ? "access_and_coupling_screened_unknowns_retained"
                                             : "raw_via_identity_and_capacity_leaves"},
                {"states", result.states},
                {"choices", result.choices.size()},
                {"exhausted", result.exhausted},
                {"elapsed_us", elapsed},
                {"access_pruned", result.access_pruned_states},
                {"terminal_pair_pruned", result.terminal_pair_pruned_states},
                {"via_pair_pruned", result.via_pair_pruned_states},
                {"terminal_via_pruned", result.terminal_via_pruned_states},
                {"frozen_copper_via_pruned", result.frozen_copper_via_pruned_states},
                {"lane_capacity_pruned", result.lane_capacity_pruned_states},
                {"run_layer_pruned", result.run_layer_pruned_states},
                {"coupled_pruned", result.coupled_pruned_states},
                {"forward_wipeouts", result.forward_domain_wipeouts},
                {"pairwise_access_checks", result.pairwise_access_checks},
                {"forward_capacity_rejections", result.forward_capacity_rejections},
                {"forward_via_identity_rejections", result.forward_via_identity_rejections},
                {"access_states", result.access_assignment_states},
                {"access_exhaustions", result.access_assignment_exhaustions}};
            std::cout << json::serialize(output) << std::endl;
          }
        }
      }
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
