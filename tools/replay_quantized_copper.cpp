// Compare probe-grid resolutions on one recorded selected allocation. Diagnostic only.
#include "flux/core/quantized_copper_oracle.hpp"
#include "flux/core/detailed_router.hpp"
#include "flux/core/regional_router.hpp"
#include "flux/io/kicad_export.hpp"
#include "flux/core/routing_rules.hpp"
#include "flux/io/kicad_import.hpp"
#include <algorithm>
#include <boost/json.hpp>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <stdexcept>

int main(int argc, char** argv) {
  try {
    if (argc != 4 && argc != 6)
      throw std::runtime_error(
          "usage: flux-replay-quantized-copper BOARD TELEMETRY SNAPSHOT_SEQUENCE [NET_A NET_B | --detail-preview OUTPUT | --regional-straight MAX_STATES]");
    const auto imported = flux::io::import_kicad_pcb_file(argv[1]);
    if (!imported.strict())
      throw std::runtime_error("source must import strictly under current sibling project rules");
    const auto& board = *imported.board;
    std::size_t consumed = 0U;
    const auto sequence = std::stoull(argv[3], &consumed);
    if (consumed != std::string{argv[3]}.size())
      throw std::runtime_error("invalid snapshot sequence");
    std::ifstream input(argv[2]);
    if (!input)
      throw std::runtime_error("cannot read telemetry");
    flux::GlobalRouteResult global;
    boost::json::array snapshot_routes;
    std::string line;
    bool found = false;
    while (std::getline(input, line)) {
      const auto parsed = boost::json::parse(line);
      const auto& event = parsed.as_object();
      if (event.at("type") != "snapshot" ||
          boost::json::value_to<std::uint64_t>(event.at("sequence")) != sequence)
        continue;
      found = true;
      if (boost::json::value_to<std::string>(event.at("source_identity")) != board.source_revision)
        throw std::runtime_error("snapshot does not match source PCB revision");
      snapshot_routes = event.at("routes").as_array();
      std::set<std::string> ids;
      for (const auto& value : event.at("routes").as_array()) {
        const auto& route = value.as_object();
        const auto id = boost::json::value_to<std::string>(route.at("id"));
        if (!id.starts_with("global-route:") || route.at("family") != "selected" ||
            route.at("lifecycle") != "active")
          continue;
        if (!ids.insert(id).second || route.at("geometry") != "polyline")
          throw std::runtime_error("duplicate or non-polyline selected route");
        flux::GlobalRoutePath path;
        path.net = flux::NetId{boost::json::value_to<std::string>(route.at("net"))};
        path.connection_index = std::stoull(id.substr(id.rfind(':') + 1U));
        if (boost::json::value_to<std::int64_t>(route.at("width_nm")) !=
            flux::resolve_net_routing_rules(board, path.net).track_width.value())
          throw std::runtime_error("recorded width differs from current source rules");
        for (const auto& p : route.at("points").as_array()) {
          const auto& point = p.as_object();
          path.points.push_back(
              {{flux::Nanometres{boost::json::value_to<std::int64_t>(point.at("x_nm"))},
                flux::Nanometres{boost::json::value_to<std::int64_t>(point.at("y_nm"))}},
               flux::LayerId{boost::json::value_to<std::string>(point.at("layer"))}});
        }
        if (path.points.size() < 2U)
          throw std::runtime_error("selected route has insufficient geometry");
        global.paths.push_back(std::move(path));
      }
      break;
    }
    if (!found || global.paths.empty())
      throw std::runtime_error("snapshot contains no active selected allocation");
    if (argc == 6 && std::string_view{argv[4]} == "--regional-straight") {
      std::size_t state_chars = 0U;
      const auto states = std::stoull(argv[5], &state_chars);
      if (state_chars != std::string{argv[5]}.size() || states == 0U || states > 4096U)
        throw std::runtime_error("regional replay state budget must be 1..4096");
      // The snapshot has selected geometry but not global capacity/corridor certificates.
      // Rebuild scope solely from exact copper witnesses, with no implicit scope retries.
      // This is a physical diagnostic experiment, never a replay of routing qualification.
      flux::RegionalRouterConfig config;
      config.enable_spline_realization = false;
      config.enable_straight_realization = true;
      config.enable_terminal_portal_templates = true;
      config.frozen_copper_context = true;
      config.max_physical_scope_retries = 0U;
      config.max_bundle_search_states = states;
      const auto before = flux::probe_quantized_copper_feasibility(board, global);
      config.physical_conflict_witnesses = before.diagnoses;
      const auto regional = flux::analyze_regional_conflicts(board, global, config);
      boost::json::object categories;
      for (const auto& [category, count] : regional.straight_scene_drc_categories)
        categories[category] = count;
      boost::json::array diagnoses;
      for (const auto& d : regional.local_routability_diagnoses) {
        if (!d.resource.stable_id.starts_with("native-scene:") &&
            !d.resource.stable_id.starts_with("straight-scene:")) continue;
        boost::json::array nets;
        for (const auto& net : d.resource.nets) nets.emplace_back(net.value());
        diagnoses.emplace_back(boost::json::object{
            {"id", d.resource.stable_id}, {"layer", d.resource.layer.value()},
            {"nets", std::move(nets)}, {"detail", d.detail},
            {"bounds_nm", boost::json::array{d.resource.bounds.minimum.x.value(),
                                             d.resource.bounds.minimum.y.value(),
                                             d.resource.bounds.maximum.x.value(),
                                             d.resource.bounds.maximum.y.value()}}});
      }
      std::cout << boost::json::serialize(boost::json::object{
          {"authoritative", false}, {"source_revision", board.source_revision},
          {"snapshot_sequence", sequence}, {"scope", "exact_copper_witnesses_only"},
          {"max_states", states}, {"scope_retries", 0},
          {"paths", global.paths.size()}, {"before_drc_violations", before.drc_violations},
          {"before_opens", before.open_connections}, {"windows", regional.windows.size()},
          {"collapsed_via_adapters", regional.straight_collapsed_via_adapters},
          {"candidate_available", regional.spline_candidate_available},
          {"native_manifest_available", regional.native_realization_manifest_available},
          {"straight_scene_checks", regional.linear_scaffold_checks},
          {"straight_scene_drc_violations", regional.straight_scene_drc_violations},
          {"straight_scene_open_connections", regional.straight_scene_open_connections},
          {"categories", std::move(categories)}, {"planned_vias", regional.planned_via_transitions},
          {"diagnostic", regional.spline_candidate_diagnostic}, {"diagnoses", std::move(diagnoses)}})
                << '\n';
      return 0;
    }
    if (argc == 6 && std::string_view{argv[4]} == "--detail-preview") {
      // Snapshot geometry omits allocation certificates. A private diagnostic copy may enter
      // materialization, but this output cannot assert that the original routing plan passed.
      global.stop_reason = flux::GlobalRouterStopReason::complete;
      flux::DetailedRouterConfig config;
      config.continuous_refinement = false;
      config.circular_arc_refinement = false;
      config.restrict_straight_track_angles = true;
      config.bounded_residual_conflict_refinement = false;
      const auto detailed = flux::materialize_detailed_route(board, global, config);
      bool written = false;
      if (detailed.candidate) {
        const auto exported = flux::io::export_kicad_candidate_file(
            argv[1], argv[5], board, *detailed.candidate);
        written = exported.ok();
      }
      std::cout << boost::json::serialize(boost::json::object{
          {"authoritative", false}, {"snapshot_sequence", sequence},
          {"paths", global.paths.size()}, {"detail_complete", detailed.complete()},
          {"tracks", detailed.added_tracks}, {"arcs", detailed.added_track_arcs},
          {"vias", detailed.added_vias}, {"validation_errors", detailed.validation.error_count()},
          {"opens", detailed.connectivity.open_connections},
          {"drc_violations", detailed.copper_drc.violation_count()},
          {"diagnostic", detailed.diagnostic}, {"preview_written", written}}) << '\n';
      return written ? 0 : 1;
    }
    if (argc == 6) {
      const std::string first_net{argv[4]}, second_net{argv[5]};
      if (first_net == second_net)
        throw std::runtime_error("portfolio comparison requires two different selected nets");
      using Portfolio = std::map<std::size_t, std::vector<flux::GlobalRoutePath>>;
      const auto portfolios = [&](const std::string& net) {
        std::set<std::size_t> expected;
        for (const auto& path : global.paths)
          if (path.net.value() == net)
            expected.insert(path.connection_index);
        if (expected.empty())
          throw std::runtime_error("portfolio net is not in selected allocation");
        Portfolio candidates;
        for (const auto& entry : snapshot_routes) {
          const auto& route = entry.as_object();
          const auto id = boost::json::value_to<std::string>(route.at("id"));
          if (!id.starts_with("coarse:") || route.at("lifecycle") != "retained_fallback" ||
              boost::json::value_to<std::string>(route.at("net")) != net)
            continue;
          const auto slot_separator = id.rfind(':');
          const auto connection_separator = id.rfind(':', slot_separator - 1U);
          const auto slot = std::stoull(id.substr(slot_separator + 1U));
          flux::GlobalRoutePath path;
          path.net = flux::NetId{net};
          path.connection_index = std::stoull(
              id.substr(connection_separator + 1U, slot_separator - connection_separator - 1U));
          if (route.at("geometry") != "polyline" ||
              boost::json::value_to<std::int64_t>(route.at("width_nm")) !=
                  flux::resolve_net_routing_rules(board, path.net).track_width.value())
            throw std::runtime_error("portfolio geometry or source-rule width mismatch");
          for (const auto& p : route.at("points").as_array()) {
            const auto& point = p.as_object();
            path.points.push_back(
                {{flux::Nanometres{boost::json::value_to<std::int64_t>(point.at("x_nm"))},
                  flux::Nanometres{boost::json::value_to<std::int64_t>(point.at("y_nm"))}},
                 flux::LayerId{boost::json::value_to<std::string>(point.at("layer"))}});
          }
          candidates[slot].push_back(std::move(path));
        }
        for (auto it = candidates.begin(); it != candidates.end();) {
          std::set<std::size_t> actual;
          for (const auto& path : it->second)
            actual.insert(path.connection_index);
          if (actual != expected || actual.size() != it->second.size())
            it = candidates.erase(it); // Do not compare a partial or duplicate net tree.
          else
            ++it;
        }
        return candidates;
      };
      const auto first_candidates = portfolios(first_net),
                 second_candidates = portfolios(second_net);
      if (first_candidates.empty() || second_candidates.empty() || first_candidates.size() > 32U ||
          second_candidates.size() > 32U)
        throw std::runtime_error("missing complete net portfolios or excessive comparison size");
      for (const auto& [first_slot, first_paths] : first_candidates) {
        for (const auto& [second_slot, second_paths] : second_candidates) {
          auto alternative = global;
          std::erase_if(alternative.paths, [&](const auto& p) {
            return p.net.value() == first_net || p.net.value() == second_net;
          });
          alternative.paths.insert(alternative.paths.end(), first_paths.begin(), first_paths.end());
          alternative.paths.insert(alternative.paths.end(), second_paths.begin(),
                                   second_paths.end());
          const auto result = flux::probe_quantized_copper_feasibility(board, alternative);
          const auto pair_conflicts =
              std::count_if(result.diagnoses.begin(), result.diagnoses.end(), [&](const auto& d) {
                return d.resource.nets.size() == 2U &&
                       std::find(d.resource.nets.begin(), d.resource.nets.end(),
                                 flux::NetId{first_net}) != d.resource.nets.end() &&
                       std::find(d.resource.nets.begin(), d.resource.nets.end(),
                                 flux::NetId{second_net}) != d.resource.nets.end();
              });
          std::cout << boost::json::serialize(
                           boost::json::object{{"authoritative", false},
                                               {"snapshot_sequence", sequence},
                                               {"pitch_nm", 1},
                                               {"net_a", first_net},
                                               {"net_b", second_net},
                                               {"slot_a", first_slot},
                                               {"slot_b", second_slot},
                                               {"paths", alternative.paths.size()},
                                               {"materialized", result.paths_materialized},
                                               {"malformed", result.malformed_paths},
                                               {"opens", result.open_connections},
                                               {"raw_drc", result.drc_violations},
                                               {"pair_diagnoses", pair_conflicts},
                                               {"via_count", result.generated_vias},
                                               {"elapsed_us", result.elapsed_us}})
                    << '\n';
        }
      }
      return 0;
    }
    for (const auto pitch : {100000LL, 10000LL, 1LL}) {
      flux::QuantizedCopperOracleConfig config;
      config.grid_pitch = flux::Nanometres{pitch};
      const auto result = flux::probe_quantized_copper_feasibility(board, global, config);
      boost::json::object categories;
      for (const auto& [category, count] : result.drc_violation_counts)
        categories[category] = count;
      boost::json::object output{{"authoritative", false},
                                 {"source_revision", board.source_revision},
                                 {"snapshot_sequence", sequence},
                                 {"pitch_nm", pitch},
                                 {"paths", global.paths.size()},
                                 {"materialized", result.paths_materialized},
                                 {"malformed", result.malformed_paths},
                                 {"tracks", result.generated_tracks},
                                 {"vias", result.generated_vias},
                                 {"drc_violations", result.drc_violations},
                                 {"categories", std::move(categories)},
                                 {"opens", result.open_connections},
                                 {"diagnoses", result.diagnoses.size()},
                                 {"elapsed_us", result.elapsed_us}};
      boost::json::array open_nets;
      for (const auto& diagnosis : result.diagnoses)
        if (diagnosis.resource.stable_id.find("quantized-copper:open:") != std::string::npos)
          for (const auto& net : diagnosis.resource.nets) open_nets.emplace_back(net.value());
      output["open_net_ids"] = std::move(open_nets);
      std::size_t immutable_planar_diagnoses = 0U;
      for (const auto& diagnosis : result.diagnoses)
        if (diagnosis.resource.stable_id.find(":immutable:") != std::string::npos &&
            diagnosis.kind == flux::RoutabilityFailureKind::quantization_clearance &&
            diagnosis.suggested_relaxation == flux::RoutabilityRelaxation::change_corridor)
          ++immutable_planar_diagnoses;
      output["immutable_planar_obstacle_diagnoses"] = immutable_planar_diagnoses;
      std::map<std::string, std::vector<const flux::RoutabilityDiagnosis*>> groups;
      for (const auto& diagnosis : result.diagnoses) {
        if (diagnosis.resource.nets.size() != 2U)
          continue;
        const auto key = diagnosis.resource.nets[0].value() + ":" +
                         diagnosis.resource.nets[1].value() + ":" +
                         diagnosis.resource.layer.value();
        groups[key].push_back(&diagnosis);
      }
      std::vector<decltype(groups)::const_iterator> ranked;
      for (auto it = groups.begin(); it != groups.end(); ++it)
        ranked.push_back(it);
      std::sort(ranked.begin(), ranked.end(), [](auto lhs, auto rhs) {
        if (lhs->second.size() != rhs->second.size())
          return lhs->second.size() > rhs->second.size();
        return lhs->first < rhs->first;
      });
      boost::json::array largest;
      for (std::size_t i = 0; i < std::min<std::size_t>(ranked.size(), 5U); ++i) {
        const auto& group = *ranked[i];
        const auto& worst =
            **std::max_element(group.second.begin(), group.second.end(), [](auto lhs, auto rhs) {
              return lhs->clearance_deficit < rhs->clearance_deficit;
            });
        boost::json::object witness{
            {"net_pair_and_layer", group.first},
            {"coalesced_diagnoses", group.second.size()},
            {"max_deficit_nm", worst.clearance_deficit.value()},
            {"witness_id", worst.resource.stable_id},
            {"witness_bounds_nm", boost::json::array{worst.resource.bounds.minimum.x.value(),
                                                     worst.resource.bounds.minimum.y.value(),
                                                     worst.resource.bounds.maximum.x.value(),
                                                     worst.resource.bounds.maximum.y.value()}}};
        if (!worst.source_segment_pairs.empty()) {
          const auto encode_segment = [](const flux::RoutabilitySourceSegment& segment) {
            return boost::json::object{
                {"net", segment.net.value()},
                {"connection_index", segment.connection_index},
                {"segment_index", segment.segment_index},
                {"start_nm", boost::json::array{segment.start.x.value(), segment.start.y.value()}},
                {"end_nm", boost::json::array{segment.end.x.value(), segment.end.y.value()}}};
          };
          const auto& pair = worst.source_segment_pairs.front();
          witness["first_source_pair"] =
              boost::json::array{encode_segment(pair.first), encode_segment(pair.second)};
        }
        largest.push_back(std::move(witness));
      }
      output["largest_pair_groups"] = std::move(largest);
      std::cout << boost::json::serialize(output) << '\n';
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
