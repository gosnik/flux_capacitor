#include "flux/core/routing_engine.hpp"
#include "flux/core/cpu_worker_budget.hpp"

#include "flux/core/routability_knowledge.hpp"
#include "flux/core/router_telemetry.hpp"
#include "flux/core/routing_feedback.hpp"
#include "flux/core/routing_kernel.hpp"
#include "flux/core/routing_engine_policy.hpp"
#include <mutex>
#include "flux/core/routing_rules.hpp"
#include "flux/core/copper_geometry.hpp"
#include "scaffold_terminal_rebind.hpp"
#include "scaffold_physical_demands.hpp"
#include "scaffold_access.hpp"


#include <algorithm>
#include <array>
#include <chrono>
#include <functional>
#include <fstream>
#ifdef __linux__
#include <unistd.h>
#endif
#include <limits>
#include <set>
#include <sstream>
#include <thread>
#include <tuple>

namespace flux {
namespace {
std::size_t plus(std::size_t a, std::size_t b) {
  return b > std::numeric_limits<std::size_t>::max() - a ? std::numeric_limits<std::size_t>::max()
                                                         : a + b;
}
[[nodiscard]] std::vector<GlobalRoutePath> scaffold_requests(const Board& board,
                                                              const GlobalRouteResult& global);

std::size_t difference(std::size_t newer, std::size_t older) {
  return newer > older ? newer - older : 0U;
}
std::optional<Board> scaffold_physical_scene(Board source, const DetailedRouterConfig& detailed,
                                             const GlobalRouteResult& global) {
  std::set<NetId> routed;
  for (const auto& path : global.paths) routed.insert(path.net);
  // The scaffold can repair a coarse-unrouted connection through fallback requests, so its
  // replacement authority must cover those nets too rather than only materialized coarse paths.
  for (const auto& request : scaffold_requests(source, global)) routed.insert(request.net);
  if (detailed.existing_routing == ExistingRoutingPolicy::preserve) {
    // Match detailed materialization validation: an exact removal scope is invalid when source
    // routing is preserved, and must not be allowed to yield a scaffold completion.
    if (detailed.existing_routing_replacement_scope) return std::nullopt;
    return source;
  }
  std::set<NetId> replacement = routed;
  if (detailed.existing_routing_replacement_scope) {
    replacement.clear();
    for (const auto& net : *detailed.existing_routing_replacement_scope) {
      const auto found = std::find_if(source.nets.begin(), source.nets.end(), [&net](const Net& candidate) {
        return candidate.id == net && !candidate.name.empty();
      });
      if (found == source.nets.end() || !routed.contains(net) || !replacement.insert(net).second)
        return std::nullopt;
    }
  }
  // Keep every physical obstacle from the materialization source except copper the caller
  // authorized detailed materialization to replace.  This mirrors the work-plan planning scene
  // while retaining source-only geometry omitted by coarse planning.
  std::erase_if(source.tracks, [&replacement](const auto& track) {
    return !track.locked && replacement.contains(track.net);
  });
  std::erase_if(source.track_beziers, [&replacement](const auto& curve) {
    return !curve.locked && replacement.contains(curve.net);
  });
  std::erase_if(source.track_arcs, [&replacement](const auto& arc) {
    return !arc.locked && replacement.contains(arc.net);
  });
  std::erase_if(source.vias, [&replacement](const auto& via) {
    return !via.locked && replacement.contains(via.net);
  });
  return source;
}
bool physical_scene_improved(const RoutingQuality& previous, const RoutingQuality& candidate) {
  // Physical priority intentionally excludes coarse global overflow (rank 5) and only compares
  // open, DRC, malformed, and bend evidence after both sides have physical observations.
  if (previous.ranks[0] > 2U || candidate.ranks[0] > 2U) return false;
  bool improved = false;
  for (std::size_t rank = 1U; rank <= 4U; ++rank) {
    if (candidate.ranks[rank] > previous.ranks[rank]) return false;
    improved = improved || (previous.ranks[rank] < RoutingQuality::unknown_rank &&
                            candidate.ranks[rank] < previous.ranks[rank]);
  }
  return improved;
}
void append_path_constraints(std::ostringstream& out, const GlobalRoutePath& path) {
  for (const auto width : path.segment_widths) out << "width=" << width.value() << ';';
  for (const auto kind : path.segment_width_kinds) out << "width-kind=" << static_cast<int>(kind) << ';';
  const auto point = [&](Point p) { out << p.x.value() << ',' << p.y.value() << ';'; };
  for (const auto& corridor : path.geometric_corridors) {
    out << "corridor=" << corridor.layer.value().size() << ':' << corridor.layer.value()
        << ':' << corridor.required_envelope_width.value() << ':'
        << corridor.minimum_physical_cross_section_width.value() << ':' << corridor.lane_capacity
        << ':' << corridor.selected_path_preserves_topology << ';';
    for (const auto& cell : corridor.topology_cells) {
      out << "cell=" << cell.vertices.size() << ';';
      for (const auto p : cell.vertices) point(p);
    }
    for (const auto level : corridor.topology_cell_refinement_levels) out << "level=" << level << ';';
    for (const auto& portal : corridor.portals) {
      out << "portal="; point(portal.left); point(portal.right);
    }
    out << "signature=" << corridor.topology_signature.valid << ':'
        << corridor.topology_signature.fingerprint << ';';
    for (const auto& gate : corridor.topology_signature.gates) {
      out << "gate=" << gate.ordinal << ':' << gate.from_cell << ':' << gate.to_cell << ':'
          << gate.left_boundary_id << ':' << gate.right_boundary_id << ';';
      for (const auto p : {gate.left_boundary, gate.right_boundary, gate.from_center, gate.to_center}) point(p);
    }
    out << "lane=" << corridor.lane_centerline.size() << ';';
    for (const auto p : corridor.lane_centerline) point(p);
  }
}
void retain_current_routability_knowledge_impl(RoutabilityKnowledge& checkpoint,
                                               const RoutabilityKnowledge& current,
                                               std::size_t maximum_entries) {
  // A resumed regional kernel intentionally runs with an immutable XPBD/history snapshot.  When
  // it publishes, retain feedback learned while it was parked so its older input cannot erase a
  // rejected global retry's negotiated costs.  Existing checkpoint entries remain, allowing the
  // bounded solve to contribute its own newly observed evidence.
  const auto preserve_resource_extent = [](RoutabilityResourceReference& retained,
                                           const RoutabilityResourceReference& newer) {
    retained.bounds.minimum.x = std::min(retained.bounds.minimum.x, newer.bounds.minimum.x);
    retained.bounds.minimum.y = std::min(retained.bounds.minimum.y, newer.bounds.minimum.y);
    retained.bounds.maximum.x = std::max(retained.bounds.maximum.x, newer.bounds.maximum.x);
    retained.bounds.maximum.y = std::max(retained.bounds.maximum.y, newer.bounds.maximum.y);
    retained.nets.insert(retained.nets.end(), newer.nets.begin(), newer.nets.end());
    std::sort(retained.nets.begin(), retained.nets.end());
    retained.nets.erase(std::unique(retained.nets.begin(), retained.nets.end()), retained.nets.end());
  };
  for (const auto& entry : current.history) {
    const auto found = std::find_if(checkpoint.history.begin(), checkpoint.history.end(),
        [&](const auto& candidate) {
          return std::tie(candidate.kind, candidate.stage, candidate.suggested_relaxation,
                          candidate.resource.stable_id) ==
                 std::tie(entry.kind, entry.stage, entry.suggested_relaxation,
                          entry.resource.stable_id);
        });
    if (found == checkpoint.history.end()) {
      if (checkpoint.history.size() >= maximum_entries) {
        const auto obsolete = std::find_if(checkpoint.history.begin(), checkpoint.history.end(),
            [&](const auto& candidate) {
              return std::none_of(current.history.begin(), current.history.end(),
                  [&](const auto& newer) {
                    return std::tie(candidate.kind, candidate.stage,
                                    candidate.suggested_relaxation, candidate.resource.stable_id) ==
                           std::tie(newer.kind, newer.stage,
                                    newer.suggested_relaxation, newer.resource.stable_id);
                  });
            });
        if (obsolete != checkpoint.history.end()) checkpoint.history.erase(obsolete);
      }
      if (checkpoint.history.size() >= maximum_entries) {
        checkpoint.dropped_history_entries = plus(checkpoint.dropped_history_entries, 1U);
        continue;
      }
      checkpoint.history.push_back(entry);
    } else {
      // The current state is newer than the parked snapshot, including its coalesced bounds and
      // affected-net attribution.  Preserve the strongest monotonically accumulated pressure.
      preserve_resource_extent(found->resource, entry.resource);
      found->observations = std::max(found->observations, entry.observations);
      found->pressure = std::max(found->pressure, entry.pressure);
    }
  }
  for (const auto& entry : current.success_history) {
    const auto found = std::find_if(checkpoint.success_history.begin(), checkpoint.success_history.end(),
        [&](const auto& candidate) { return candidate.resource.stable_id == entry.resource.stable_id; });
    if (found == checkpoint.success_history.end()) {
      if (checkpoint.success_history.size() >= maximum_entries) {
        const auto obsolete = std::find_if(checkpoint.success_history.begin(), checkpoint.success_history.end(),
            [&](const auto& candidate) {
              return std::none_of(current.success_history.begin(), current.success_history.end(),
                  [&](const auto& newer) { return candidate.resource.stable_id == newer.resource.stable_id; });
            });
        if (obsolete != checkpoint.success_history.end()) checkpoint.success_history.erase(obsolete);
      }
      if (checkpoint.success_history.size() >= maximum_entries) {
        checkpoint.dropped_history_entries = plus(checkpoint.dropped_history_entries, 1U);
        continue;
      }
      checkpoint.success_history.push_back(entry);
    } else {
      preserve_resource_extent(found->resource, entry.resource);
      found->observations = std::max(found->observations, entry.observations);
      found->strength = std::max(found->strength, entry.strength);
    }
  }
  // Pending diagnoses choose the next action. Current diagnoses are authoritative; do not
  // resurrect a resolved diagnosis merely because it existed in an older parked snapshot.
  checkpoint.diagnoses = current.diagnoses;
  checkpoint.coalesced_diagnoses =
      std::max(checkpoint.coalesced_diagnoses, current.coalesced_diagnoses);
  checkpoint.dropped_diagnoses = std::max(checkpoint.dropped_diagnoses, current.dropped_diagnoses);
  checkpoint.dropped_history_entries =
      std::max(checkpoint.dropped_history_entries, current.dropped_history_entries);
  std::sort(checkpoint.history.begin(), checkpoint.history.end(), [](const auto& lhs, const auto& rhs) {
    return std::tie(lhs.stage, lhs.kind, lhs.suggested_relaxation, lhs.resource.stable_id) <
           std::tie(rhs.stage, rhs.kind, rhs.suggested_relaxation, rhs.resource.stable_id);
  });
  std::sort(checkpoint.success_history.begin(), checkpoint.success_history.end(),
            [](const auto& lhs, const auto& rhs) {
              return lhs.resource.stable_id < rhs.resource.stable_id;
            });
}
std::size_t score(const RoutingPlanResult& p) {
  const auto physical = p.regional.straight_scene_evaluated
                            ? p.regional.straight_scene_drc_violations
                            : p.regional.native_scene_total_conflicts;
  return plus(plus(p.coarse_global.unrouted_nets.size(), p.coarse_global.total_overflow), physical);
}
std::size_t score(const DetailedRouteResult& d) {
  return plus(plus(d.validation.error_count(), d.connectivity.open_connections),
              d.copper_drc.violation_count());
}
bool topology_connected(const RoutingPlanResult& p) {
  return p.coarse_global.unrouted_nets.empty() &&
         p.coarse_global.unaccepted_selected_voronoi_lane_transitions == 0U &&
         p.coarse_global.stop_reason != GlobalRouterStopReason::invalid_input &&
         p.coarse_global.stop_reason != GlobalRouterStopReason::cancelled;
}
bool topology_ready(const RoutingPlanResult& p) {
  return topology_connected(p) && p.coarse_global.total_overflow == 0U;
}
auto plan_order(const RoutingPlanResult& p) { return routing_quality(&p, nullptr); }
auto detailed_order(const DetailedRouteResult& d) { return routing_quality(nullptr, &d); }
RoutingPhysicalEvidence physical_evidence(const RoutingPlanResult* plan,
                                           const DetailedRouteResult* detailed) {
  if (detailed && detailed->candidate)
    return {true, detailed->connectivity.open_connections, detailed->copper_drc.violation_count(),
            detailed->validation.error_count()};
  if (!plan) return {};
  if (plan->regional_evaluated && plan->regional.native_scene_evaluated) {
    const auto& region = plan->regional;
    return {region.native_scene_complete, region.native_scene_open_connections,
            region.native_scene_total_conflicts, 0U};
  }
  // A local straight checkpoint does not certify complete scene coverage.
  if (plan->regional_evaluated && plan->regional.straight_scene_evaluated) return {};
  const auto& oracle = plan->quantized_copper_oracle;
  return {oracle.evaluated, oracle.open_connections, oracle.drc_violations, oracle.malformed_paths};
}
bool has_evaluated_physical_scene(const RoutingPlanResult& plan) {
  return plan.quantized_copper_oracle.evaluated ||
      (plan.regional_evaluated &&
       (plan.regional.native_scene_evaluated || plan.regional.straight_scene_evaluated));
}
std::size_t action_quality(RoutingEngineAction action, const RoutingPlanResult* plan,
                           const DetailedRouteResult* detailed) {
  const auto unknown = std::numeric_limits<std::size_t>::max();
  if (action == RoutingEngineAction::materialize_copper)
    return detailed && detailed->candidate ? score(*detailed) : unknown;
  if (!plan)
    return unknown;
  if (action == RoutingEngineAction::global_allocation ||
      action == RoutingEngineAction::feedback_global_replan ||
      action == RoutingEngineAction::usped_global_replan)
    return plus(plan->coarse_global.unrouted_nets.size(), plan->coarse_global.total_overflow);
  if (action == RoutingEngineAction::probe_quantized_copper)
    return plan->quantized_copper_oracle.evaluated
               ? plus(plan->quantized_copper_oracle.open_connections,
                      plan->quantized_copper_oracle.drc_violations)
               : unknown;
  if (!plan->regional_evaluated)
    return unknown;
  return plan->regional.straight_scene_evaluated
             ? plus(plan->regional.straight_scene_open_connections,
                    plan->regional.straight_scene_drc_violations)
         : plan->regional.native_scene_evaluated && plan->regional.native_scene_complete
             ? plus(plan->regional.native_scene_total_conflicts, plan->regional.native_scene_open_connections)
             : unknown;
}
void activate_full_strength(GlobalRouterConfig& global_config) {
  global_config.enable_search_diversification = true;
  global_config.atomic_complete_net_routing = true;
  global_config.diversify_pin_access_rings = true;
  global_config.use_multisource_steiner_growth = true;
  global_config.use_bidirectional_route_growth = true;
  global_config.expansion_retry_heuristic_weight_per_mille =
      std::max(global_config.expansion_retry_heuristic_weight_per_mille, std::uint32_t{1250U});
  global_config.reuse_weighted_search_frontier = true;
  global_config.bidirectional_route_growth_start_iteration = 1U;
  global_config.large_neighborhood_stagnation_iterations = 1U;
  global_config.hard_capacity_start_iteration = 1U;
  global_config.max_hard_capacity_ripup_attempts_per_pass =
      std::max(global_config.max_hard_capacity_ripup_attempts_per_pass, std::size_t{8U});
  global_config.max_hard_capacity_ripup_nets_per_attempt =
      std::max(global_config.max_hard_capacity_ripup_nets_per_attempt, std::size_t{4U});
  global_config.max_hard_capacity_conflict_set_states_per_attempt =
      std::max(global_config.max_hard_capacity_conflict_set_states_per_attempt, std::size_t{64U});
  global_config.rollback_failed_hard_capacity_ripup = true;
  global_config.defer_hard_capacity_ripup_victims_to_pass_tail = true;
  global_config.protect_recovered_hard_capacity_ripup_nets = true;
  global_config.selective_congestion_reroute = true;
  global_config.selective_full_reroute_interval = 2U;
  global_config.max_hard_capacity_blocker_reuse_invalidations_per_failed_net = std::max(
      global_config.max_hard_capacity_blocker_reuse_invalidations_per_failed_net, std::size_t{4U});
  global_config.global_navmesh_usped.enabled = true;
  // Preserve user-selected topology backends. Enable shared-medial lane feedback only
  // when that backend is already in use, rather than silently replacing the graph.
  if (global_config.use_voronoi_allocation_resources) {
    global_config.negotiate_voronoi_lane_conflicts = true;
  }
}
[[nodiscard]] std::vector<GlobalRoutePath> scaffold_requests(const Board& board,
                                                            const GlobalRouteResult& global) {
  auto requests = global.paths;
  // A failed global search must not silently omit a connection from the constructive action.
  // Build a deterministic spanning set for failed nets; their exact pad layers remain available
  // to the scaffold search. Existing same-net copper may satisfy these requests without additions.
  const auto pads = copper_pad_shapes(board);
  std::set<NetId> missing(global.unrouted_nets.begin(), global.unrouted_nets.end());
  for (const auto& failure : global.connection_failures) missing.insert(failure.net);
  for (const auto& net : missing) {
    std::vector<const CopperPadShape*> terminals;
    for (const auto& pad : pads)
      if (pad.net == net && !pad.layers.empty()) terminals.push_back(&pad);
    if (terminals.size() < 2U) continue;
    std::size_t next = 0U;
    for (const auto& path : requests)
      if (path.net == net) next = std::max(next, path.connection_index + 1U);
    std::vector<bool> connected(terminals.size(), false);
    connected[0] = true;
    for (std::size_t count = 1U; count < terminals.size(); ++count) {
      long double best = std::numeric_limits<long double>::infinity();
      std::size_t from = 0U, to = 0U;
      for (std::size_t i = 0; i < terminals.size(); ++i) if (connected[i])
        for (std::size_t j = 0; j < terminals.size(); ++j) if (!connected[j]) {
          const auto dx = static_cast<long double>(terminals[i]->position.x.value()) - terminals[j]->position.x.value();
          const auto dy = static_cast<long double>(terminals[i]->position.y.value()) - terminals[j]->position.y.value();
          const auto distance = dx * dx + dy * dy;
          if (distance < best) { best = distance; from = i; to = j; }
        }
      connected[to] = true;
      GlobalRoutePath request;
      request.net = net;
      request.connection_index = next++;
      request.points = {{terminals[from]->position, terminals[from]->layers.front()},
                        {terminals[to]->position, terminals[to]->layers.front()}};
      requests.push_back(std::move(request));
    }
  }
  return requests;
}
void append_scaffold_proposals(CopperScaffoldResult& seed,
    const std::vector<GlobalRoutePath>& candidates, const std::vector<GlobalRoutePath>& requests) {
  for (const auto& candidate : candidates) {
    if (candidate.points.size() < 2U) continue;
    const auto matches = [&](const GlobalRoutePath& request) {
      return request.net == candidate.net && request.connection_index == candidate.connection_index &&
          request.points.size() >= 2U && request.points.front() == candidate.points.front() &&
          request.points.back() == candidate.points.back();
    };
    // Previously checked branch attachments carry their original request association in seed.
    // New proposals must identify the same physical terminals, not merely reuse a plan index.
    if (std::count_if(requests.begin(), requests.end(), matches) == 1)
      seed.paths.push_back(candidate);
  }
}

// A failed bounded search is not a capacity proof. Preserve attributed owners and spatial
// scope for targeted repair/global diversification without inventing a clearance deficit.
[[nodiscard]] std::vector<RoutabilityDiagnosis> scaffold_diagnoses(const CopperScaffoldResult& scaffold) {
  std::vector<RoutabilityDiagnosis> diagnoses;
  for (const auto index : scaffold.missing_indices) {
    if (index >= scaffold.requests.size() || index >= scaffold.request_evidence.size()) continue;
    const auto& request = scaffold.requests[index];
    const auto& evidence = scaffold.request_evidence[index];
    if (request.points.empty() || evidence.blocker_objects.empty()) continue;
    auto minimum = request.points.front().position, maximum = minimum;
    const auto include = [&](Point point) {
      minimum.x = std::min(minimum.x, point.x); minimum.y = std::min(minimum.y, point.y);
      maximum.x = std::max(maximum.x, point.x); maximum.y = std::max(maximum.y, point.y);
    };
    for (const auto& point : request.points) include(point.position);
    for (const auto& id : evidence.blocker_objects) {
      for (const auto& track : scaffold.copper.tracks) if (track.id == id) {
        include(track.start); include(track.end);
      }
      for (const auto& arc : scaffold.copper.track_arcs) if (arc.id == id) {
        include(arc.start); include(arc.middle); include(arc.end);
      }
      for (const auto& curve : scaffold.copper.track_beziers) if (curve.id == id) {
        include(curve.start); include(curve.control1); include(curve.control2); include(curve.end);
      }
      for (const auto& via : scaffold.copper.vias) if (via.id == id) include(via.position);
      for (const auto& footprint : scaffold.copper.footprints)
        for (const auto& pad : footprint.pads)
          if (pad.id == id)
            include(geometry::transform_point(pad.local_position, footprint.position,
                                              footprint.rotation, footprint.side));
    }
    RoutabilityDiagnosis diagnosis;
    diagnosis.kind = RoutabilityFailureKind::topological_trap;
    diagnosis.stage = RoutabilityFailureStage::local_pns;
    diagnosis.resource.stable_id = "scaffold-request:" + evidence.request_identity;
    diagnosis.resource.layer = request.points.front().layer;
    diagnosis.resource.bounds = {minimum, maximum};
    diagnosis.resource.nets = evidence.blocker_nets;
    diagnosis.resource.nets.push_back(request.net);
    std::sort(diagnosis.resource.nets.begin(), diagnosis.resource.nets.end());
    diagnosis.resource.nets.erase(std::unique(diagnosis.resource.nets.begin(), diagnosis.resource.nets.end()),
                                  diagnosis.resource.nets.end());
    diagnosis.suggested_relaxation = RoutabilityRelaxation::change_corridor;
    diagnosis.contributes_to_history = false;
    diagnosis.detail = "bounded scaffold search blocked; owners=" +
        std::to_string(evidence.blocker_objects.size()) + "; termination=" +
        std::string(to_string(evidence.termination));
    diagnoses.push_back(std::move(diagnosis));
  }
  return diagnoses;
}

// A partial physical repair cannot certify the original whole-board request set.
[[nodiscard]] std::optional<RoutingPlanResult> certified_scaffold_plan(
    const Board& source, const RoutingPlanResult& prior, const CopperScaffoldResult& scaffold,
    QuantizedCopperOracleConfig certificate, std::stop_token token,
    std::optional<QuantizedCopperOracleResult>& evidence) {
  if (!scaffold.complete || token.stop_requested()) return std::nullopt;
  auto plan = prior;
  plan.paths = scaffold.paths;
  for (auto& path : plan.paths) path.legal_scaffold = true;
  plan.coarse_global = GlobalRouteResult{};
  plan.coarse_global.paths = plan.paths;
  plan.coarse_global.stop_reason = GlobalRouterStopReason::complete;
  certificate.enabled = true;
  certificate.grid_pitch = Nanometres{1};
  certificate.preserve_path_endpoints = true;
  plan.quantized_copper_oracle = probe_quantized_copper_feasibility(
      source, plan.coarse_global, certificate, token);
  evidence = plan.quantized_copper_oracle;
  plan.legal_scaffold_complete = plan.quantized_copper_oracle.feasible();
  if (!plan.legal_scaffold_complete) return std::nullopt;
  plan.regional = RegionalRouteResult{};
  plan.regional_evaluated = false;
  plan.stop_reason = RoutingPlanStopReason::complete;
  plan.diagnostic = scaffold.diagnostic;
  return plan;
}

[[nodiscard]] Board provisional_copper_scaffold(const Board& source,
                                                const std::vector<GlobalRoutePath>& paths) {
  Board scaffold;
  scaffold.layers = source.layers;
  scaffold.rules = source.rules;
  scaffold.nets = source.nets;
  scaffold.net_classes = source.net_classes;
  std::size_t ordinal = 0U;
  for (const auto& path : paths) {
    const auto rules = resolve_net_routing_rules(source, path.net);
    for (std::size_t index = 0U; index + 1U < path.points.size(); ++index) {
      const auto& first = path.points[index];
      const auto& second = path.points[index + 1U];
      if (first.layer == second.layer) {
        const auto width = index < path.segment_widths.size() && path.segment_widths[index].value() > 0
                               ? path.segment_widths[index] : rules.track_width;
        scaffold.tracks.push_back({ObjectId{"terrain-scaffold-track-" + std::to_string(ordinal++)},
                                   path.net, first.layer, first.position, second.position, width, false});
      } else if (first.position == second.position) {
        scaffold.vias.push_back({ObjectId{"terrain-scaffold-via-" + std::to_string(ordinal++)},
                                 path.net, first.position, rules.via_diameter,
                                 rules.via_drill, {first.layer, second.layer}, false});
      }
    }
  }
  return scaffold;
}
[[nodiscard]] bool same_scaffold_request_domain(const CopperScaffoldResult& lhs,
                                                const CopperScaffoldResult& rhs) {
  if (lhs.request_count != rhs.request_count || lhs.request_identities.size() != lhs.request_count ||
      rhs.request_identities.size() != rhs.request_count)
    return false;
  if (lhs.request_identities == rhs.request_identities) return true;
  auto left = lhs.request_identities;
  auto right = rhs.request_identities;
  std::sort(left.begin(), left.end());
  std::sort(right.begin(), right.end());
  return left == right;
}
[[nodiscard]] std::set<std::string>
scaffold_satisfied_request_identities(const CopperScaffoldResult& scaffold) {
  std::set<std::string> result;
  for (const auto index : scaffold.satisfied_indices)
    if (index < scaffold.request_identities.size()) result.insert(scaffold.request_identities[index]);
  return result;
}
[[nodiscard]] bool strict_scaffold_coverage_improvement(const CopperScaffoldResult& candidate,
                                                         const CopperScaffoldResult& incumbent) {
  if (!same_scaffold_request_domain(candidate, incumbent)) return false;
  const auto candidate_satisfied = scaffold_satisfied_request_identities(candidate);
  const auto incumbent_satisfied = scaffold_satisfied_request_identities(incumbent);
  return candidate_satisfied.size() > incumbent_satisfied.size() &&
      std::includes(candidate_satisfied.begin(), candidate_satisfied.end(),
                    incumbent_satisfied.begin(), incumbent_satisfied.end());
}
// A scaffold can satisfy a request by attaching it to copper that is already part of the same
// connected component. Such a request legitimately contributes no new path, so `paths.size()`
// is not a measure of checkpoint progress. Request identities scope the comparison: a changed
// domain is a fresh, reconstructed checkpoint and cannot be ranked by unrelated index counts.
[[nodiscard]] bool better_scaffold_checkpoint(const CopperScaffoldResult& candidate,
                                               const CopperScaffoldResult& incumbent) {
  if (!same_scaffold_request_domain(candidate, incumbent))
    return scaffold_preserves_satisfied_requests(candidate, incumbent);
  if (incumbent.complete && !candidate.complete) return false;
  if (candidate.complete != incumbent.complete) return candidate.complete;
  if (strict_scaffold_coverage_improvement(candidate, incumbent)) return true;
  // Fresh negative evidence matters even when no route was added. Only refresh an identical
  // checked scene/domain, so blocker ownership never migrates onto a different checkpoint.
  return candidate.satisfied_indices == incumbent.satisfied_indices && candidate.paths == incumbent.paths &&
      (candidate.request_evidence != incumbent.request_evidence ||
       candidate.exchange_rollbacks != incumbent.exchange_rollbacks);
}
void emit_engine_event(RouterTelemetrySink* sink, const char* phase, RoutingEngineAction action,
                       std::size_t generation, std::size_t work, std::string detail,
                       std::uint64_t elapsed_us = 0U) {
  if (!sink)
    return;
  RouterTelemetryEvent event;
  event.kind = RouterTelemetryEventKind::search_summary;
  event.stable_id = "engine-action:" + std::to_string(generation);
  event.progress_phase = std::string(to_string(action)) + " / " + phase;
  event.progress_work_units = work;
  event.progress_elapsed_us = elapsed_us;
  if (std::string_view{phase} == "begin" || std::string_view{phase} == "end" ||
      std::string_view{phase} == "suspend" || std::string_view{phase} == "resume") {
    event.region_update = true;
    event.region_active = false;
  }
  event.detail = "action=" + std::string(to_string(action)) +
                 ", generation=" + std::to_string(generation) + ", work=" + std::to_string(work) +
                 "; " + detail;
  static_cast<void>(sink->try_emit(std::move(event)));
}

// Event detail uses semicolon-delimited fields. Keep evidence values parseable even when an
// externally supplied identifier contains punctuation, while retaining a deterministic compact
// representation for diagnostic tools.
[[nodiscard]] std::string telemetry_evidence_list(std::vector<std::string> values, bool preserve_order = false) {
  if (!preserve_order) {
    std::sort(values.begin(), values.end());
    values.erase(std::unique(values.begin(), values.end()), values.end());
  }
  constexpr char hex[] = "0123456789ABCDEF";
  std::ostringstream out;
  out << values.size() << ':';
  for (const auto& value : values) {
    std::string escaped;
    escaped.reserve(value.size());
    for (const auto character : value) {
      const auto byte = static_cast<unsigned char>(character);
      if ((byte >= 'a' && byte <= 'z') || (byte >= 'A' && byte <= 'Z') ||
          (byte >= '0' && byte <= '9') || byte == '-' || byte == '_' || byte == '.')
        escaped.push_back(character);
      else {
        escaped.push_back('%');
        escaped.push_back(hex[byte >> 4U]);
        escaped.push_back(hex[byte & 0x0fU]);
      }
    }
    out << escaped.size() << ':' << escaped;
  }
  return std::move(out).str();
}
} // namespace

std::vector<std::size_t> select_physical_repair_requests(
    const CopperScaffoldResult& scaffold, const std::vector<std::size_t>& attempts,
    std::size_t limit) {
  std::vector<std::size_t> remaining;
  for (const auto index : scaffold.missing_indices)
    if (index < scaffold.requests.size()) remaining.push_back(index);
  const auto count = [&](std::size_t i) { return i < attempts.size() ? attempts[i] : 0U; };
  std::sort(remaining.begin(), remaining.end(), [&](auto a, auto b) {
    return std::pair{count(a), a} < std::pair{count(b), b};
  });
  remaining.erase(std::unique(remaining.begin(), remaining.end()), remaining.end());
  if (remaining.empty() || limit == 0U) return {};
  std::vector<std::size_t> selected{remaining.front()};
  remaining.erase(remaining.begin());
  const auto linked = [&](std::size_t a, std::size_t b) {
    if (scaffold.requests[a].net == scaffold.requests[b].net) return true;
    if (a >= scaffold.request_evidence.size() || b >= scaffold.request_evidence.size()) return false;
    const auto& left = scaffold.request_evidence[a];
    const auto& right = scaffold.request_evidence[b];
    for (const auto& net : left.blocker_nets)
      if (net == scaffold.requests[b].net ||
          std::find(right.blocker_nets.begin(), right.blocker_nets.end(), net) != right.blocker_nets.end())
        return true;
    return std::find(right.blocker_nets.begin(), right.blocker_nets.end(), scaffold.requests[a].net) !=
        right.blocker_nets.end();
  };
  while (selected.size() < limit) {
    const auto next = std::find_if(remaining.begin(), remaining.end(), [&](auto i) {
      return count(i) == count(selected.front()) &&
          std::any_of(selected.begin(), selected.end(), [&](auto j) { return linked(i, j); });
    });
    if (next == remaining.end()) break;
    selected.push_back(*next);
    remaining.erase(next);
  }
  return selected;
}

std::vector<std::string> physical_repair_evidence_tokens(const CopperScaffoldResult& scaffold) {
  std::vector<std::string> tokens(scaffold.requests.size());
  for (const auto index : scaffold.missing_indices) {
    if (index >= scaffold.requests.size()) continue;
    const auto& request = scaffold.requests[index];
    std::vector<ObjectId> blockers;
    if (index < scaffold.request_evidence.size()) {
      blockers = scaffold.request_evidence[index].blocker_objects;
      std::sort(blockers.begin(), blockers.end());
      blockers.erase(std::unique(blockers.begin(), blockers.end()), blockers.end());
    }
    std::ostringstream record;
    record << index << ':' << request.net.value().size() << ':' << request.net.value() << ':'
           << request.connection_index << ':';
    for (const auto& blocker : blockers)
      record << blocker.value().size() << ':' << blocker.value();
    tokens[index] = record.str();
  }
  return tokens;
}

void reconcile_physical_repair_attempts(bool geometry_changed,
                                        const std::vector<std::string>& previous_tokens,
                                        const std::vector<std::string>& current_tokens,
                                        std::vector<std::size_t>& attempts) {
  if (geometry_changed || previous_tokens.size() != current_tokens.size() ||
      attempts.size() != current_tokens.size()) {
    attempts.assign(current_tokens.size(), 0U);
    return;
  }
  for (std::size_t index = 0U; index < current_tokens.size(); ++index)
    if (previous_tokens[index] != current_tokens[index]) attempts[index] = 0U;
}

std::string physical_repair_evidence_token(const CopperScaffoldResult& scaffold) {
  auto records = physical_repair_evidence_tokens(scaffold);
  std::erase(records, std::string{});
  std::sort(records.begin(), records.end());
  records.erase(std::unique(records.begin(), records.end()), records.end());
  std::ostringstream token;
  for (const auto& record : records) token << record.size() << ':' << record;
  return token.str();
}

void retain_current_routability_knowledge(RoutabilityKnowledge& checkpoint,
                                          const RoutabilityKnowledge& current,
                                          std::size_t maximum_entries) {
  retain_current_routability_knowledge_impl(checkpoint, current, maximum_entries);
}
void RoutingEngineProgressBudget::consume(std::size_t work_units) noexcept {
  consumed = plus(consumed, work_units);
}
std::size_t RoutingEngineProgressBudget::earn(std::size_t credit, std::size_t ceiling) noexcept {
  if (ceiling <= allowance || credit == 0U)
    return 0U;
  const auto prior = allowance;
  allowance = std::min(ceiling, plus(allowance, credit));
  const auto added = allowance - prior;
  earned = plus(earned, added);
  return added;
}
bool RoutingEngineProgressBudget::strict_physical_improvement(
    const RoutingQuality& previous, const RoutingQuality& candidate,
    bool connectivity_and_hard_gates_preserved) noexcept {
  if (!connectivity_and_hard_gates_preserved)
    return false;
  bool improved = false;
  for (std::size_t index = 1U; index <= 4U; ++index) {
    if (candidate.ranks[index] > previous.ranks[index])
      return false;
    // Learning a previously unknown metric is evidence, not measured physical improvement.
    improved = improved || (previous.ranks[index] < RoutingQuality::unknown_rank &&
                            candidate.ranks[index] < previous.ranks[index]);
  }
  return improved;
}
struct RoutingEngine::Deadline {
  std::stop_source source;
  std::jthread thread;
  explicit Deadline(std::chrono::milliseconds duration)
      : thread([this, duration](std::stop_token stop) {
          const auto end = std::chrono::steady_clock::now() + duration;
          while (!stop.stop_requested()) {
            const auto now = std::chrono::steady_clock::now();
            if (now >= end) {
              source.request_stop();
              return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
          }
        }) {}
};
struct RoutingEngine::Pending {
  RoutingEngineAction action{};
  RoutingEngineStepBudget budget{};
  bool tightening_improved{false};
  std::optional<RoutingPlanResult> plan;
  std::optional<DetailedRouteResult> detailed;
  std::shared_ptr<CopperScaffoldResult> scaffold;
  std::unique_ptr<RoutingKernel> kernel;
  std::unique_ptr<std::stop_callback<std::function<void()>>> deadline_link;
  std::chrono::steady_clock::time_point started{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_progress{started};
  std::uint64_t active_us{0U};
  std::uint64_t sampled_active_us{0U};
  RoutingQuality sampled_quality{};
  RoutingPhysicalEvidence sampled_evidence{};
  RoutingPhysicalEvidence best_evidence{};
  std::shared_ptr<RegionalRoutingPreparation> preparation;
  std::string dependency_key;
  std::size_t yields{0U};
  std::size_t scaffold_satisfied_before{0U};
  std::size_t scaffold_request_count_before{0U};
  std::vector<std::string> scaffold_request_identities_before;
  std::set<std::string> scaffold_satisfied_identities_before;
  std::size_t live_copper_terrain_revision{0U};
  std::size_t live_copper_terrain_features{0U};
  bool live_copper_terrain_provisional{false};
  std::size_t ordinal{0U};
  std::string input_key;
  std::string physical_context_key;
  std::string experiment_state_key;
  std::size_t quality_before{0U};
  RoutingQuality before;
  RoutingQuality best_scene;
  std::optional<RoutingQuality> evaluated_scene;
  bool hard_gates_before{false};
  std::size_t budget_work_units{0U};
  std::optional<RegionalDecisionPoint> decision;
  std::size_t stagnant_decisions{0U};
  bool evaluated_checkpoint{false};
  bool preparing{true};
  bool scaffold_regional{false};
  std::vector<std::size_t> physical_requests;
  std::optional<RoutingPhysicalRepairEvidence> physical_evidence;
  std::optional<QuantizedCopperOracleResult> scaffold_oracle;
  std::string scaffold_physical_key;
  std::vector<ScaffoldTerminalRebind> scaffold_terminal_rebinds;
  bool physical_improvement{false};
  bool evidence_recorded{false};
  ~Pending() {
    deadline_link.reset();
    kernel.reset(); // Join before any callback target or checkpoint member is destroyed.
  }
};
struct RoutingEngine::Speculation {
  std::mutex mutex;
  std::shared_ptr<RegionalRoutingPreparation> prepared;
  bool ready{false};
  bool physical{false};
  std::string physical_key;
  std::jthread worker;
};
[[nodiscard]] bool has_pending_physical_repair(const CopperScaffoldResult* scaffold) {
  return scaffold && !scaffold->complete &&
      std::any_of(scaffold->missing_indices.begin(), scaffold->missing_indices.end(),
                  [&](const auto index) { return index < scaffold->requests.size(); });
}

void RoutingEngine::start_speculative_preparation() {
  const auto retained = state_.legal_scaffold;
  const bool physical = has_pending_physical_repair(retained.get());
  if (!config_.speculative_preparation || speculation_ ||
      (physical ? static_cast<bool>(physical_preparation_) : static_cast<bool>(regional_preparation_))) return;
  auto worker_lease = acquire_cpu_workers(1U);
  if (worker_lease.workers() == 0U) return;
  speculation_ = std::make_unique<Speculation>();
  auto* job = speculation_.get();
  job->physical = physical;
  if (physical) job->physical_key = physical_dependency_key(true);
  ++state_.speculative_preparations_started;
  job->worker = std::jthread([this, job, retained, lease = std::move(worker_lease)](std::stop_token stop) mutable {
    auto active_lease = std::move(lease); // Release on worker exit, not when the job is collected.
    std::shared_ptr<RegionalRoutingPreparation> prepared;
    try {
      auto config = config_.planner.regional;
      if (job->physical) {
        config.frozen_copper_context = true;
        config.expand_frozen_copper_scope = false;
        config.prioritize_frozen_scope_expansion = false;
        config.require_legal_linear_scaffold = false;
      }
      prepared = prepare_regional_routing(job->physical ? retained->copper : planning_board_,
                                           GlobalRouteResult{}, config, stop);
    } catch (...) {
      // Speculation is optional; foreground preparation remains authoritative.
    }
    std::lock_guard lock(job->mutex);
    if (!stop.stop_requested()) job->prepared = std::move(prepared);
    job->ready = true;
  });
}
void RoutingEngine::stop_speculative_preparation() {
  if (!speculation_) return;
  speculation_->worker.request_stop();
  speculation_.reset();
}
void RoutingEngine::acquire_regional_preparation() {
  if (regional_preparation_) {
    ++state_.preparation_reuses;
    return;
  }
  if (speculation_) {
    std::lock_guard lock(speculation_->mutex);
    if (speculation_->ready && speculation_->prepared && !speculation_->physical) {
      regional_preparation_ = std::move(speculation_->prepared);
      ++state_.speculative_preparations_used;
    } else {
      speculation_->worker.request_stop();
      ++state_.speculative_preparations_discarded;
    }
  }
  // Never wait for the speculative worker on the foreground routing path.
  if (!regional_preparation_)
    regional_preparation_ = std::make_shared<RegionalRoutingPreparation>();
}
const char* to_string(RoutingEngineAction x) {
  switch (x) {
  case RoutingEngineAction::none:
    return "none";
  case RoutingEngineAction::global_allocation:
    return "global";
  case RoutingEngineAction::regional_realization:
    return "regional";
  case RoutingEngineAction::probe_quantized_copper:
    return "quantized-probe";
  case RoutingEngineAction::construct_copper_scaffold:
    return "copper-scaffold";
  case RoutingEngineAction::tighten_routes:
    return "tighten";
  case RoutingEngineAction::materialize_copper:
    return "materialize";
  case RoutingEngineAction::confined_erosion_repair:
    return "erosion";
  case RoutingEngineAction::xpbd_repair:
    return "xpbd";
  case RoutingEngineAction::usped_global_replan:
    return "usped";
  case RoutingEngineAction::feedback_global_replan:
    return "feedback-global";
  }
  return "unknown";
}
const char* to_string(RoutingEngineOutcome x) {
  switch (x) {
  case RoutingEngineOutcome::progressed:
    return "progressed";
  case RoutingEngineOutcome::yielded:
    return "yielded";
  case RoutingEngineOutcome::awaiting_materialization:
    return "awaiting-materialization";
  case RoutingEngineOutcome::awaiting_external_validation:
    return "awaiting-external-validation";
  case RoutingEngineOutcome::complete:
    return "complete";
  case RoutingEngineOutcome::exhausted:
    return "exhausted";
  case RoutingEngineOutcome::timed_out:
    return "timed-out";
  case RoutingEngineOutcome::cancelled:
    return "cancelled";
  }
  return "unknown";
}
bool routing_engine_action_uses_physical_continuation(RoutingEngineAction action) noexcept {
  return action == RoutingEngineAction::regional_realization ||
         action == RoutingEngineAction::xpbd_repair ||
         action == RoutingEngineAction::confined_erosion_repair;
}
std::string routing_engine_negotiation_dependency_key(const RoutabilityKnowledge& knowledge) {
  const auto resource_key = [](const RoutabilityResourceReference& resource) {
    std::ostringstream out;
    const auto string = [&](const std::string& value) { out << value.size() << ':' << value; };
    string(resource.stable_id);
    string(resource.layer.value());
    out << ':' << resource.bounds.minimum.x.value() << ',' << resource.bounds.minimum.y.value()
        << ':' << resource.bounds.maximum.x.value() << ',' << resource.bounds.maximum.y.value();
    auto nets = resource.nets;
    std::sort(nets.begin(), nets.end());
    for (const auto& net : nets) string(net.value());
    return out.str();
  };
  std::vector<std::string> records;
  for (const auto& entry : knowledge.history) {
    std::ostringstream out;
    out << "history:" << static_cast<int>(entry.kind) << ':' << static_cast<int>(entry.stage)
        << ':' << static_cast<int>(entry.suggested_relaxation) << ':' << entry.observations
        << ':' << entry.pressure << ':' << resource_key(entry.resource);
    records.push_back(out.str());
  }
  for (const auto& entry : knowledge.success_history) {
    std::ostringstream out;
    out << "success:" << entry.observations << ':' << entry.strength << ':'
        << entry.start.x.value() << ',' << entry.start.y.value() << ':'
        << entry.end.x.value() << ',' << entry.end.y.value() << ':' << resource_key(entry.resource);
    records.push_back(out.str());
  }
  std::sort(records.begin(), records.end());
  std::ostringstream out;
  for (const auto& record : records) out << record.size() << ':' << record;
  return out.str();
}
std::string routing_engine_continuation_dependency_key(
    RoutingEngineAction action, const RoutingPlanResult* plan,
    const LiveCopperTerrainSnapshot& terrain, std::string_view experiment_state_key) {
  if (!routing_engine_action_uses_physical_continuation(action))
    return std::string(experiment_state_key);
  std::vector<std::string> records;
  if (plan) {
    for (const auto& path : plan->paths) {
      std::ostringstream record;
      record << "path=" << path.net.value() << ':' << path.connection_index << ';';
      for (const auto& point : path.points)
        record << point.layer.value() << ':' << point.position.x.value() << ','
               << point.position.y.value() << ';';
      append_path_constraints(record, path);
      records.push_back(record.str());
    }
    for (const auto& assignment : plan->regional.bundle_assignments)
      for (const auto& curve : assignment.spline_realizations) {
        std::ostringstream record;
        record << "native=" << curve.fragment.net.value() << ':' << curve.layer.value()
               << ':' << curve.width.value() << ';';
        for (const auto& piece : curve.native_pieces) {
          record << static_cast<int>(piece.kind) << ':';
          for (const auto point : {piece.start, piece.middle, piece.end, piece.control1, piece.control2})
            record << point.x.value() << ',' << point.y.value() << ';';
        }
        records.push_back(record.str());
      }
    for (const auto& path : plan->coarse_global.paths) {
      std::ostringstream record;
      record << "coarse=" << path.net.value() << ':' << path.connection_index << ';';
      for (const auto& point : path.points)
        record << point.layer.value() << ':' << point.position.x.value() << ','
               << point.position.y.value() << ';';
      append_path_constraints(record, path);
      records.push_back(record.str());
    }
  }
  for (const auto& feature : terrain.features) {
    std::ostringstream record;
    record << "terrain-shape=" << static_cast<int>(feature.kind) << ':' << feature.net.value()
           << ':' << feature.layer.value() << ':' << feature.bounds.minimum.x.value() << ','
           << feature.bounds.minimum.y.value() << ':' << feature.bounds.maximum.x.value() << ','
           << feature.bounds.maximum.y.value() << ':' << feature.axis_start.x.value() << ','
           << feature.axis_start.y.value() << ':' << feature.axis_end.x.value() << ','
           << feature.axis_end.y.value() << ':' << feature.radius.value() << ':' << feature.directional;
    records.push_back(record.str());
  }
  std::sort(records.begin(), records.end());
  records.erase(std::unique(records.begin(), records.end()), records.end());
  std::ostringstream data;
  for (const auto& record : records) data << record.size() << ':' << record;
  return data.str();
}
RoutingEngine::RoutingEngine(Board p, Board m, RoutingEngineConfig c, RouterTelemetrySink* t)
    : planning_board_(std::move(p)), materialization_source_(std::move(m)), config_(std::move(c)), telemetry_(t) {
  // Saved fills are provisional only in this explicitly authorized workflow.  A scaffold
  // certifies fixed copper, so its blocked-fill evidence is inapplicable until refill; retain
  // the existing provisional materialization path and its mandatory external refill gate.
  if (config_.detailed.defer_saved_zone_fill_clearance_to_refill)
    config_.copper_scaffold.enabled = false;
  // Keep a finite engine-level ceiling even if a caller uses zero for the initial grant.  The
  // latter means "start at the configured ceiling", which is useful for compatibility probes
  // without creating an unbounded regional continuation.
  const auto ceiling = std::max<std::size_t>(1U, config_.max_regional_work_units);
  const auto initial = config_.initial_regional_work_units == 0U
      ? ceiling : std::min(config_.initial_regional_work_units, ceiling);
  state_.regional_work_budget.allowance = initial;
  state_.preparation_work_budget.allowance = config_.preparation_work_units;
  state_.global_retry_budget.allowance = std::min(config_.initial_global_retry_attempts,
                                                config_.max_global_retry_attempts);
  state_.changed_context_retry_budget.allowance = config_.max_changed_context_retries;
  state_.live_copper_terrain = config_.planner.global.live_copper_terrain;
  state_.live_copper_terrain_revision = config_.planner.global.live_copper_terrain_revision;
  state_.live_copper_terrain_features = state_.live_copper_terrain.features.size();
  // These planner modes depend on the legacy interleaved probe/portfolio state; preserve their
  // established semantics until their inner checkpoints gain a split-stage equivalent.
  if (config_.planner.oracle_first_feedback || config_.planner.enable_assembled_straight_feedback)
    config_.planning_mode = RoutingEnginePlanningMode::legacy_plan_routes;
  if (!config_.planner.curved_tracks) {
    config_.detailed.circular_arc_refinement = false;
    config_.detailed.restrict_straight_track_angles = true;
    config_.planner.global.use_navmesh_funnel_spline_seeds = false;
    config_.planner.global.single_lane_direct_voronoi_resources = true;
  }
  config_.planner.global.clearance_shadows_advisory_only =
      config_.planner.clearance_shadows_advisory.value_or(
          config_.planner.quantized_copper_oracle.enabled &&
          config_.planner.max_routability_feedback_retries > 0U);
  config_.planner.regional.enable_spline_realization = config_.planner.curved_tracks;
  config_.planner.regional.enable_straight_realization = !config_.planner.curved_tracks;
  config_.planner.regional.treat_saved_zone_fills_as_refillable =
      config_.planner.global.treat_saved_zone_fills_as_refillable;
  if (config_.shared_deadline.count() > 0)
    deadline_ = std::make_unique<Deadline>(config_.shared_deadline);
}
RoutingEngine::~RoutingEngine() {
  pending_.reset();
  parked_.clear();
  stop_speculative_preparation();
}
bool RoutingEngine::terminal() const noexcept {
  return state_.outcome == RoutingEngineOutcome::complete ||
         state_.outcome == RoutingEngineOutcome::exhausted ||
         state_.outcome == RoutingEngineOutcome::timed_out ||
         state_.outcome == RoutingEngineOutcome::cancelled;
}
const RoutingPlanResult* RoutingEngine::selected_plan() const noexcept {
  return state_.accepted_plan                                        ? &*state_.accepted_plan
         : state_.best_partial_detailed && state_.best_detailed_plan ? &*state_.best_detailed_plan
         : state_.best_partial_plan                                  ? &*state_.best_partial_plan
         : state_.working_plan                                       ? &*state_.working_plan
                                                                     : nullptr;
}
const DetailedRouteResult* RoutingEngine::selected_detailed() const noexcept {
  return state_.accepted_detailed       ? &*state_.accepted_detailed
         : state_.best_partial_detailed ? &*state_.best_partial_detailed
         : state_.working_detailed      ? &*state_.working_detailed
                                        : nullptr;
}
std::size_t RoutingEngine::working_quality() const {
  return state_.working_detailed ? score(*state_.working_detailed)
         : state_.working_plan   ? score(*state_.working_plan)
                                 : std::numeric_limits<std::size_t>::max();
}
std::string RoutingEngine::evidence_key(RoutingEngineAction action) const {
  if (action == RoutingEngineAction::construct_copper_scaffold) {
    // Regional counters, resource histories and speculative terrain do not change the physical
    // input of this search. Only changed requests or a changed retained scene justify a retry.
    std::ostringstream key;
    key << "scaffold:";
    const auto append = [&](const GlobalRoutePath& path) {
      key << path.net.value().size() << ':' << path.net.value() << ':' << path.connection_index;
      for (const auto& p : path.points)
        key << ':' << p.position.x.value() << ',' << p.position.y.value() << ','
            << p.layer.value().size() << ':' << p.layer.value();
      for (const auto w : path.segment_widths) key << ":w" << w.value();
      for (const auto kind : path.segment_width_kinds) key << ":k" << static_cast<int>(kind);
      key << ":neck" << path.terminal_neck_down_segments << ',' << path.medial_neck_down_segments
          << ',' << path.corridor_neck_down_segments;
      key << ';';
    };
    // Requests depend on pads and the coarse plan, neither of which changes when authorized
    // tracks/vias are removed. Avoid copying the physical board on every scheduler query;
    // the immutable replacement policy is validated when the action constructs its scene.
    if (state_.legal_scaffold && !state_.legal_scaffold->requests.empty())
      for (const auto& path : state_.legal_scaffold->requests) append(path);
    else if (state_.working_plan)
      for (const auto& path : scaffold_requests(materialization_source_, state_.working_plan->coarse_global))
        append(path);
    // New global proposals may improve an unresolved request without replacing its identity.
    if (state_.working_plan) for (const auto& path : state_.working_plan->coarse_global.paths) append(path);
    key << "retained:";
    if (state_.legal_scaffold) for (const auto& path : state_.legal_scaffold->paths) append(path);
    return key.str();
  }
  // Canonical content, never a telemetry revision or container traversal order. A/B/A
  // state changes must still hit the same cycle guard; actual cost/evidence changes may retry.
  std::vector<std::string> records;
  const bool geometry_only = action == RoutingEngineAction::materialize_copper ||
      action == RoutingEngineAction::tighten_routes;
  if (!geometry_only) for (const auto& feature : state_.live_copper_terrain.features) {
    std::ostringstream record;
    record << "terrain=" << state_.live_copper_terrain.provisional << ':'
           << static_cast<int>(feature.kind) << ':' << feature.net.value() << ':'
           << feature.layer.value() << ':' << feature.bounds.minimum.x.value() << ','
           << feature.bounds.minimum.y.value() << ':' << feature.bounds.maximum.x.value() << ','
           << feature.bounds.maximum.y.value() << ':' << feature.pressure << ':'
           << feature.axis_start.x.value() << ',' << feature.axis_start.y.value() << ':'
           << feature.axis_end.x.value() << ',' << feature.axis_end.y.value() << ':'
           << feature.radius.value() << ':' << feature.directional;
    records.push_back(record.str());
  }
  if (state_.working_plan) {
    records.push_back("physical=" + physical_dependency_key());
    // Negotiation pressure changes global allocation inputs, not a regional geometry solve.
    // Repeating an identical diagnosis must not unlock the same local repair indefinitely.
    if (action == RoutingEngineAction::none || action == RoutingEngineAction::global_allocation ||
        action == RoutingEngineAction::feedback_global_replan || action == RoutingEngineAction::usped_global_replan)
      records.push_back("knowledge=" +
          routing_engine_negotiation_dependency_key(state_.working_plan->routability_knowledge));
  }
  if (!geometry_only) for (const auto& d : state_.pending_diagnoses) {
    std::ostringstream record;
    record << "diagnosis=" << static_cast<int>(d.kind) << ':' << static_cast<int>(d.stage) << ':'
           << static_cast<int>(d.suggested_relaxation) << ':' << d.resource.stable_id << ':'
           << d.resource.layer.value() << ':' << d.resource.bounds.minimum.x.value() << ','
           << d.resource.bounds.minimum.y.value() << ':' << d.resource.bounds.maximum.x.value() << ','
           << d.resource.bounds.maximum.y.value() << ':' << d.clearance_deficit.value()
           << ':' << d.severity << ':' << d.contributes_to_history << ':' << d.locally_resolved;
    if (d.planned_capacity) record << ":planned=" << *d.planned_capacity;
    if (d.observed_capacity) record << ":observed=" << *d.observed_capacity;
    for (const auto& pair : d.source_segment_pairs)
      for (const auto* source : {&pair.first, &pair.second})
        record << ":source=" << source->net.value().size() << ':' << source->net.value()
               << ':' << source->connection_index << ':' << source->segment_index
               << ':' << source->layer.value().size() << ':' << source->layer.value()
               << ':' << source->start.x.value() << ',' << source->start.y.value()
               << ':' << source->end.x.value() << ',' << source->end.y.value();
    auto nets = d.resource.nets;
    std::sort(nets.begin(), nets.end());
    for (const auto& net : nets) record << ':' << net.value();
    records.push_back(record.str());
  }
  std::sort(records.begin(), records.end());
  records.erase(std::unique(records.begin(), records.end()), records.end());
  std::ostringstream data;
  data << static_cast<int>(action) << ':';
  for (const auto& record : records) data << record.size() << ':' << record;
  return data.str();
}
std::string RoutingEngine::experiment_state_key() const {
  // `none` retains the canonical geometry, terrain, diagnosis, and negotiation-input records
  // while intentionally excluding the strategy identifier appended by evidence_key(action).
  auto key = evidence_key(RoutingEngineAction::none);
  if (const auto found = experiment_keys_.find(key); found != experiment_keys_.end())
    return std::to_string(found->second);
  constexpr std::size_t maximum_contexts = 4096U;
  constexpr std::size_t maximum_key_bytes = 64U * 1024U * 1024U;
  if (experiment_keys_.size() >= maximum_contexts || key.size() > maximum_key_bytes - experiment_key_bytes_) {
    experiment_metadata_exhausted_ = true;
    return {};
  }
  experiment_key_bytes_ += key.size();
  const auto [entry, inserted] = experiment_keys_.try_emplace(std::move(key), experiment_keys_.size());
  static_cast<void>(inserted);
  return std::to_string(entry->second);
}
std::string RoutingEngine::physical_dependency_key(bool retained_scene, bool include_evidence) const {
  // Intern full geometry once, independently of changing diagnoses/history. Exact string
  // equality avoids hash collisions without storing large corridors in every history context.
  RoutingPlanResult physical;
  const auto* dependency_plan = state_.working_plan ? &*state_.working_plan : nullptr;
  if (retained_scene && state_.legal_scaffold && !state_.legal_scaffold->requests.empty() &&
      !state_.legal_scaffold->complete) {
    physical.paths = state_.legal_scaffold->paths;
    physical.coarse_global.paths = state_.legal_scaffold->requests;
    dependency_plan = &physical;
  }
  auto key = routing_engine_continuation_dependency_key(
      RoutingEngineAction::regional_realization, dependency_plan, state_.live_copper_terrain, {});
  if (state_.legal_scaffold) {
    std::string coverage = "checked-requests:";
    for (const auto index : state_.legal_scaffold->satisfied_indices)
      coverage += std::to_string(index) + ",";
    key += std::to_string(coverage.size()) + ":" + coverage;
    if (include_evidence) {
      const auto evidence = physical_repair_evidence_token(*state_.legal_scaffold);
      key += std::to_string(evidence.size()) + ":" + evidence;
    }
  }
  // Individual path/copper records are also interned: a one-curve edit must not duplicate
  // all unchanged corridor polygons retained by every other path in the scene.
  std::string compact;
  std::size_t offset = 0U;
  constexpr std::size_t maximum_key_bytes = 64U * 1024U * 1024U;
  while (offset < key.size()) {
    const auto colon = key.find(':', offset);
    const auto length = static_cast<std::size_t>(std::stoull(key.substr(offset, colon - offset)));
    auto record = key.substr(colon + 1U, length);
    offset = colon + 1U + length;
    auto found = physical_records_.find(record);
    if (found == physical_records_.end()) {
      if (record.size() > maximum_key_bytes - experiment_key_bytes_) {
        experiment_metadata_exhausted_ = true;
        return {};
      }
      experiment_key_bytes_ += record.size();
      found = physical_records_.try_emplace(std::move(record), physical_records_.size()).first;
    }
    compact += std::to_string(found->second) + ":";
  }
  key = std::move(compact);
  if (const auto found = physical_keys_.find(key); found != physical_keys_.end())
    return std::to_string(found->second);
  if (physical_keys_.size() >= 4096U || key.size() > maximum_key_bytes - experiment_key_bytes_) {
    experiment_metadata_exhausted_ = true;
    return {};
  }
  experiment_key_bytes_ += key.size();
  const auto [entry, inserted] = physical_keys_.try_emplace(std::move(key), physical_keys_.size());
  static_cast<void>(inserted);
  return std::to_string(entry->second);
}
std::string RoutingEngine::continuation_key(RoutingEngineAction action) const {
  return routing_engine_action_uses_physical_continuation(action)
      ? physical_dependency_key(action == RoutingEngineAction::regional_realization &&
                                config_.max_physical_region_requests != 0U &&
                                has_pending_physical_repair(state_.legal_scaffold.get()) &&
                                !config_.detailed.defer_saved_zone_fill_clearance_to_refill)
      : experiment_state_key();
}
void RoutingEngine::release_pending() {
  if (!pending_) return;
  auto preparation = pending_->preparation;
  const bool physical = pending_->scaffold_regional;
  const auto key = pending_->scaffold_physical_key;
  pending_.reset(); // stop and join before the lease can be reused
  if (preparation && physical) {
    const auto bytes = preparation->retained_bytes_estimate();
    const bool oversized = bytes && config_.max_parked_experiment_bytes != 0U &&
        *bytes > config_.max_parked_experiment_bytes / 2U;
    if (!oversized && state_.legal_scaffold && !state_.legal_scaffold->complete &&
        key == physical_dependency_key(true)) {
      physical_preparation_ = std::move(preparation);
      physical_preparation_key_ = key;
    } else {
      physical_preparation_.reset();
      physical_preparation_key_.clear();
    }
  } else if (preparation) regional_preparation_ = std::move(preparation);
}
void RoutingEngine::prune_parked_experiments() {
  std::erase_if(parked_, [&](const auto& item) {
    if (item->dependency_key == continuation_key(item->action)) return false;
    ++state_.experiment_invalidated;
    state_.experiment_schedule.trials.push_back({static_cast<std::size_t>(item->action),
        item->experiment_state_key, 0U, 0.0L, ++state_.experiment_dispatch_sequence, true, false});
    state_.experiment_schedule.last_dispatched_sequence = state_.experiment_dispatch_sequence;
    return true; // unique_ptr destruction stops and joins the obsolete private transaction
  });
  bool pressure = false;
#ifdef __linux__
  if (config_.max_parked_experiment_bytes != 0U) {
    std::ifstream memory("/proc/self/statm");
    std::size_t virtual_pages = 0U, resident_pages = 0U;
    if (memory >> virtual_pages >> resident_pages) {
      const auto page_bytes = ::sysconf(_SC_PAGESIZE);
      pressure = page_bytes > 0 && resident_pages >
          config_.max_parked_experiment_bytes / static_cast<std::size_t>(page_bytes);
    }
  }
#endif
  while (!parked_.empty() && (pressure || parked_.size() > config_.max_parked_experiments)) {
    // FIFO eviction is deterministic. Retire this strategy/input rather than restarting the
    // same expensive preparation immediately after evicting its frontier.
    const auto& victim = *parked_.front();
    state_.experiment_schedule.trials.push_back({static_cast<std::size_t>(victim.action),
        victim.experiment_state_key, 0U, 0.0L, ++state_.experiment_dispatch_sequence, true, false});
    state_.experiment_schedule.last_dispatched_sequence = state_.experiment_dispatch_sequence;
    parked_.erase(parked_.begin());
    ++state_.experiment_memory_evictions;
  }
}
bool RoutingEngine::yield_experiment(bool force) {
  const auto action = pending_->action;
  const bool schedulable = action == RoutingEngineAction::regional_realization ||
      action == RoutingEngineAction::xpbd_repair || action == RoutingEngineAction::confined_erosion_repair ||
      action == RoutingEngineAction::feedback_global_replan || action == RoutingEngineAction::usped_global_replan;
  const auto elapsed = pending_->active_us - pending_->sampled_active_us;
  if (!schedulable || (!force && elapsed < static_cast<std::uint64_t>(
          std::max<std::int64_t>(1, config_.experiment_quantum.count())) * 1000U)) return false;
  RoutingEngineAttempt sample;
  sample.action = action;
  // Keep scheduler accounting bound to the snapshot which created this continuation.  Relabeling
  // it after feedback would make an old solver input suppress a fresh scheduling state.
  sample.experiment_state_key = pending_->experiment_state_key;
  sample.elapsed_microseconds = elapsed;
  sample.physical_before = pending_->sampled_quality;
  sample.physical_after = pending_->best_scene;
  sample.measured_before = pending_->sampled_evidence;
  sample.measured_after = pending_->best_evidence;
  sample.accepted = true; // a yielded continuation is unfinished, not stagnant/exhausted
  record_experiment(sample, false);
  pending_->sampled_active_us = pending_->active_us;
  pending_->sampled_quality = pending_->best_scene;
  pending_->sampled_evidence = pending_->best_evidence;
  // Release stale continuations before admission. A full live pool must time-slice its
  // existing work instead of admitting fresh strategies and evicting unfinished setup.
  prune_parked_experiments();
  const auto next = select_action();
  if (next == RoutingEngineAction::none || next == action) return false;
  pending_->dependency_key = continuation_key(action);
  emit_engine_event(telemetry_, "suspend", action, state_.generation,
      pending_->kernel->work_units(), "retaining experiment frontier; active_us=" +
          std::to_string(pending_->active_us), pending_->active_us);
  // Exchange slots atomically: parking first would temporarily exceed the limit and
  // FIFO-evict the very continuation selected for resumption.
  std::unique_ptr<Pending> resumed;
  const auto target = std::find_if(parked_.begin(), parked_.end(), [&](const auto& item) {
    return item->action == next;
  });
  if (target != parked_.end()) {
    resumed = std::move(*target);
    parked_.erase(target);
  }
  parked_.push_back(std::move(pending_));
  pending_ = std::move(resumed);
  ++state_.experiment_suspended;
  prune_parked_experiments();
  if (pending_) {
    ++state_.experiment_resumed;
    emit_engine_event(telemetry_, "resume", pending_->action, state_.generation,
        pending_->kernel->work_units(), "resuming unchanged experiment inputs; active_us=" +
            std::to_string(pending_->active_us), pending_->active_us);
  }
  state_.next_action = next;
  state_.outcome = RoutingEngineOutcome::progressed;
  return true;
}
void RoutingEngine::record_experiment(const RoutingEngineAttempt& attempt, bool finished) {
  if (attempt.action == RoutingEngineAction::none)
    return;
  RoutingExperimentTrial trial;
  trial.strategy = static_cast<RoutingExperimentStrategyId>(attempt.action);
  trial.state_key = attempt.experiment_state_key.empty() ? experiment_state_key()
                                                         : attempt.experiment_state_key;
  trial.active_elapsed_us = finished && pending_
      ? difference(attempt.elapsed_microseconds, pending_->sampled_active_us)
      : attempt.elapsed_microseconds;
  trial.physical_gain = routing_physical_gain(attempt.measured_before, attempt.measured_after);
  trial.dispatched_sequence = ++state_.experiment_dispatch_sequence;
  trial.stagnant = !attempt.accepted && !attempt.physical_improvement;
  trial.exhausted = finished;
  if (trial.exhausted) ++state_.experiment_retired;
  state_.experiment_schedule.last_dispatched_sequence = trial.dispatched_sequence;
  state_.experiment_schedule.trials.push_back(std::move(trial));

}
void RoutingEngine::replace_live_copper_terrain(LiveCopperTerrainSnapshot snapshot) {
  if (snapshot.provisional && state_.legal_scaffold && !state_.legal_scaffold->paths.empty())
    snapshot = make_live_copper_terrain_snapshot(state_.legal_scaffold->copper, false, 4'096U);
  if (snapshot == state_.live_copper_terrain)
    return;
  state_.live_copper_terrain = std::move(snapshot);
  ++state_.live_copper_terrain_revision;
  ++state_.live_copper_terrain_snapshot_changes;
  state_.live_copper_terrain_features = state_.live_copper_terrain.features.size();
  if (state_.live_copper_terrain.provisional)
    ++state_.live_copper_terrain_provisional_replacements;
  else
    ++state_.live_copper_terrain_exact_replacements;
}
RoutingEngineAction RoutingEngine::select_action(RoutingEngineAction excluded) {
  if (!state_.working_plan)
    return RoutingEngineAction::global_allocation;
  const bool terminal_rebind_ready = [&] {
    if (!state_.legal_scaffold || state_.legal_scaffold->requests.empty()) return false;
    return !derive_scaffold_terminal_rebinds(materialization_source_, *state_.legal_scaffold,
        state_.scaffold_terminal_rebind_attempted_proofs, physical_dependency_key(true, false)).second.empty();
  }();
  if (config_.copper_scaffold.enabled && !state_.working_plan->legal_scaffold_complete &&
      (!state_.working_plan->coarse_global.paths.empty() ||
       !state_.working_plan->coarse_global.unrouted_nets.empty()) &&
      excluded != RoutingEngineAction::construct_copper_scaffold &&
      (terminal_rebind_ready || std::none_of(state_.attempts.begin(), state_.attempts.end(), [&](const auto& attempt) {
        return attempt.action == RoutingEngineAction::construct_copper_scaffold &&
            attempt.evidence_key == evidence_key(RoutingEngineAction::construct_copper_scaffold);
      }))) return RoutingEngineAction::construct_copper_scaffold;
  const auto materialization_key = evidence_key(RoutingEngineAction::materialize_copper);
  const bool materialized = std::any_of(state_.attempts.begin(), state_.attempts.end(), [&](const auto& attempt) {
    return attempt.action == RoutingEngineAction::materialize_copper && attempt.evidence_key == materialization_key;
  });
  if (config_.materialize && !state_.working_detailed && !materialized &&
      ready_for_materialization(*state_.working_plan, state_.pending_diagnoses))
    return needs_tightening() ? RoutingEngineAction::tighten_routes
                              : RoutingEngineAction::materialize_copper;
  if (state_.working_detailed && state_.working_detailed->complete())
    return RoutingEngineAction::none;
  if (excluded != RoutingEngineAction::regional_realization &&
      config_.max_physical_region_requests != 0U &&
      has_pending_physical_repair(state_.legal_scaffold.get()) &&
      !config_.detailed.defer_saved_zone_fill_clearance_to_refill) {
    const bool new_scene = physical_request_attempt_key_ != physical_dependency_key(true);
    const bool untouched = std::any_of(state_.legal_scaffold->missing_indices.begin(),
        state_.legal_scaffold->missing_indices.end(), [&](const auto index) {
          return index >= physical_request_attempts_.size() || physical_request_attempts_[index] == 0U;
        });
    // Retiring one bounded region must not retire the remaining demand components.
    if (new_scene || untouched) return RoutingEngineAction::regional_realization;
  }
  // Detailed materialization already performs bounded residual-joint/neighborhood retries.
  // A failed curve must not trigger a whole-board topology replacement of proven copper.
  if (state_.working_plan->legal_scaffold_complete && state_.working_detailed)
    return RoutingEngineAction::none;
  if (has_pending_physical_repair(state_.legal_scaffold.get()) &&
      !config_.detailed.defer_saved_zone_fill_clearance_to_refill &&
      excluded != RoutingEngineAction::regional_realization &&
      std::none_of(state_.attempts.begin(), state_.attempts.end(), [&](const auto& attempt) {
        return attempt.action == RoutingEngineAction::regional_realization &&
            attempt.evidence_key == evidence_key(RoutingEngineAction::regional_realization);
      })) return RoutingEngineAction::regional_realization;
  const bool connected = topology_connected(*state_.working_plan);
  // A connected topology first needs its bounded physical prefix. Switching to a global
  // replan merely because preparation yielded would invalidate that prefix before it can
  // produce any evidence. A proven access failure or completed local attempt opens the other
  // strategies; XPBD and AVBD are already enabled within this first regional realization.
  const bool access_failure = std::any_of(state_.pending_diagnoses.begin(), state_.pending_diagnoses.end(),
      [](const auto& diagnosis) { return diagnosis.resource.stable_id.starts_with("regional-access:"); });
  const auto regional_key = evidence_key(RoutingEngineAction::regional_realization);
  const bool regional_tried = std::any_of(state_.attempts.begin(), state_.attempts.end(),
      [&](const auto& attempt) { return attempt.action == RoutingEngineAction::regional_realization &&
          attempt.evidence_key == regional_key; });
  if (connected && !state_.working_plan->regional_evaluated && !access_failure && !regional_tried &&
      excluded != RoutingEngineAction::regional_realization)
    return RoutingEngineAction::regional_realization;
  auto wanted = !connected || state_.pending_diagnoses.empty()
                          ? RoutingEngineAction::feedback_global_replan
                          : diagnosis_action(state_.pending_diagnoses.front(),
                              config_.enable_xpbd && config_.planner.curved_tracks &&
                                  config_.planner.regional.enable_xpbd_relaxation,
                              config_.enable_confined_erosion && config_.planner.curved_tracks &&
                                  config_.planner.regional.max_confined_anisotropic_erosion_checks > 0U);
  if (connected && state_.pending_diagnoses.empty() &&
      !state_.working_plan->regional.packing_proven() &&
      (!state_.working_plan->coarse_global.unresolved_corridor_packing.empty() ||
       state_.working_plan->coarse_global.corridor_packing_obligation_budget_exhausted))
    wanted = RoutingEngineAction::regional_realization;
  if (wanted == RoutingEngineAction::usped_global_replan && !config_.enable_usped_replan)
    wanted = RoutingEngineAction::feedback_global_replan;
  std::vector<RoutingEngineAction> actions{wanted};
  const auto add = [&](RoutingEngineAction a) {
    if (std::find(actions.begin(), actions.end(), a) == actions.end())
      actions.push_back(a);
  };
  if (connected) {
    if (config_.enable_xpbd && config_.planner.curved_tracks &&
        config_.planner.regional.enable_xpbd_relaxation)
      add(RoutingEngineAction::xpbd_repair);
    if (config_.enable_confined_erosion && config_.planner.curved_tracks &&
        config_.planner.regional.max_confined_anisotropic_erosion_checks > 0U)
      add(RoutingEngineAction::confined_erosion_repair);
    add(RoutingEngineAction::regional_realization);
    add(RoutingEngineAction::feedback_global_replan);
  }
  if (config_.enable_usped_replan)
    add(RoutingEngineAction::usped_global_replan);
  std::vector<RoutingExperimentCandidate> experiments;
  const auto state_key = experiment_state_key();
  const bool live_pool_full = pending_ && parked_.size() >= config_.max_parked_experiments;
  for (std::size_t index = 0; index < actions.size(); ++index)
    if (actions[index] != excluded && (!live_pool_full || actions[index] == pending_->action ||
        std::any_of(parked_.begin(), parked_.end(), [&](const auto& item) {
          return item->action == actions[index] &&
              item->dependency_key == continuation_key(actions[index]);
        })))
      experiments.push_back({static_cast<RoutingExperimentStrategyId>(actions[index]), state_key, true, index});
  if (const auto selected = select_routing_experiment(experiments, state_.experiment_schedule)) {
    return static_cast<RoutingEngineAction>(selected->strategy);
  }
  state_.experiment_state_suppressions += experiments.size();
  return RoutingEngineAction::none;
}
void RoutingEngine::publish_retained_copper(bool force) {
  if (!telemetry_ || !state_.legal_scaffold ||
      published_scaffold_.lock() == state_.legal_scaffold) return;
  const auto now = std::chrono::steady_clock::now();
  if (!force && now - last_copper_publication_ < std::chrono::milliseconds{250}) return;
  try {
    const auto& scaffold = *state_.legal_scaffold;
    telemetry_->publish_generated_copper(scaffold.copper,
        "Retained checked copper: " + std::to_string(scaffold.satisfied_indices.size()) + "/" +
        std::to_string(scaffold.request_count) + " requests; provisional until whole-board validation");
    published_scaffold_ = state_.legal_scaffold;
    last_copper_publication_ = now;
  } catch (...) {
    // Viewer publication cannot change routing, acceptance, or cancellation semantics.
  }
}
void RoutingEngine::collect_diagnoses() {
  state_.pending_diagnoses.clear();
  if (state_.working_detailed)
    state_.pending_diagnoses = state_.working_detailed->routability_diagnoses;
  if (state_.pending_diagnoses.empty() && state_.working_detailed)
    state_.pending_diagnoses =
        diagnose_detailed_routability(*state_.working_detailed, config_.planner.routability_probe);
  if (state_.pending_diagnoses.empty() && state_.working_plan)
    state_.pending_diagnoses = diagnose_regional_routability(state_.working_plan->regional,
                                                             config_.planner.routability_probe);
  if (state_.pending_diagnoses.empty() && state_.working_plan)
    state_.pending_diagnoses = probe_global_routability(state_.working_plan->coarse_global,
                                                        config_.planner.routability_probe);
  // Frozen-region witnesses guide incomplete-scaffold repair only. The complete scaffold's
  // oracle is retained as diagnostics, but describes different copper from a global/regional
  // candidate. Importing it here also contaminates global history before candidate handoff.
  // Such a witness needs an explicit geometry mapping before it can become a repair obligation.
  const bool candidate_ready = state_.working_plan &&
      ready_for_materialization(*state_.working_plan, state_.pending_diagnoses);
  const auto physical_key = state_.legal_scaffold ? physical_dependency_key(true) : std::string{};
  if (!candidate_ready && state_.legal_scaffold && !state_.legal_scaffold->complete &&
      !physical_key.empty()) {
    for (const auto& evidence : state_.physical_repair_evidence)
      if (evidence.physical_key == physical_key)
        for (const auto& diagnosis : evidence.diagnoses)
          if (std::find(state_.pending_diagnoses.begin(), state_.pending_diagnoses.end(), diagnosis) ==
              state_.pending_diagnoses.end()) state_.pending_diagnoses.push_back(diagnosis);
  }
  if (state_.legal_scaffold && !state_.legal_scaffold->complete) {
    auto physical = scaffold_diagnoses(*state_.legal_scaffold);
    state_.pending_diagnoses.insert(state_.pending_diagnoses.end(), physical.begin(), physical.end());
  }
  if (state_.working_plan && !state_.working_plan->regional.packing_proven() &&
      (!state_.working_detailed || !state_.working_detailed->complete())) {
    // Missing packing evidence survives other diagnoses, but never masquerades as a measured
    // capacity failure or earns congestion history. A whole-board legal scene discharges it.
    const auto obligations = probe_global_routability(state_.working_plan->coarse_global,
                                                      config_.planner.routability_probe);
    for (const auto& obligation : obligations) {
      if (obligation.kind != RoutabilityFailureKind::packing_unresolved) continue;
      const auto exists = std::any_of(state_.pending_diagnoses.begin(), state_.pending_diagnoses.end(),
          [&](const auto& diagnosis) { return diagnosis.kind == obligation.kind &&
              diagnosis.resource == obligation.resource; });
      if (!exists) state_.pending_diagnoses.push_back(obligation);
    }
  } else {
    std::erase_if(state_.pending_diagnoses, [](const auto& diagnosis) {
      return diagnosis.kind == RoutabilityFailureKind::packing_unresolved;
    });
  }
  std::erase_if(state_.pending_diagnoses, [](const auto& d) { return d.locally_resolved; });
  std::stable_sort(state_.pending_diagnoses.begin(), state_.pending_diagnoses.end(),
                   [](const auto& a, const auto& b) {
                     return std::tie(b.severity, a.resource.stable_id, a.resource.nets) <
                            std::tie(a.severity, b.resource.stable_id, b.resource.nets);
                   });
  if (telemetry_ && state_.working_plan) {
    RouterTelemetryEvent packing;
    packing.kind = RouterTelemetryEventKind::search_summary;
    packing.stable_id = "engine-packing-proof";
    packing.packing_obligations_update = true;
    const bool proven = state_.working_plan->regional.packing_proven() ||
        (state_.working_detailed && state_.working_detailed->complete());
    packing.unresolved_packing_obligations = proven ? 0U :
        state_.working_plan->coarse_global.unresolved_corridor_packing.size();
    packing.packing_obligation_budget_exhausted = !proven &&
        state_.working_plan->coarse_global.corridor_packing_obligation_budget_exhausted;
    packing.detail = "packing proof: unresolved=" + std::to_string(packing.unresolved_packing_obligations) +
        ", physical_scene_proven=" + (proven ? "yes" : "no");
    static_cast<void>(telemetry_->try_emit(std::move(packing)));
  }
}
bool RoutingEngine::needs_tightening() const {
  // Quality polishing must never consume the last action needed to emit legal copper.
  // Completed kernel results are scheduled before their attempt is appended below.
  const auto used = state_.attempts.size() + (pending_ && pending_->plan ? 1U : 0U);
  const auto required = config_.materialize ? 2U : 1U;
  if (used > config_.max_actions || config_.max_actions - used < required) return false;
  return config_.enable_route_tightening && config_.planner.curved_tracks &&
         !working_plan_tightened_ && state_.working_plan &&
         state_.working_plan->regional.spline_candidate_available;
}
void RoutingEngine::schedule_after_plan() {
  if (state_.working_plan &&
      ready_for_materialization(*state_.working_plan, state_.pending_diagnoses) && needs_tightening()) {
    state_.next_action = RoutingEngineAction::tighten_routes;
    state_.outcome = RoutingEngineOutcome::progressed;
    return;
  }
  if (!config_.materialize) {
    // Plan-only callers still require a complete global and regional transaction.  Continuing
    // here preserves normal feedback/repair behavior without paying the detailed-copper stage.
    if (state_.working_plan &&
        ready_for_materialization(*state_.working_plan, state_.pending_diagnoses)) {
      state_.accepted_plan = state_.working_plan;
      state_.outcome = RoutingEngineOutcome::awaiting_materialization;
      state_.next_action = RoutingEngineAction::none;
    } else if (config_.planning_mode == RoutingEnginePlanningMode::legacy_plan_routes) {
      state_.outcome = RoutingEngineOutcome::exhausted;
      state_.next_action = RoutingEngineAction::none;
    } else {
      collect_diagnoses();
      state_.next_action = select_action();
      state_.outcome = state_.next_action == RoutingEngineAction::none
                           ? RoutingEngineOutcome::exhausted
                           : RoutingEngineOutcome::progressed;
    }
    return;
  }
  collect_diagnoses();
  state_.next_action = select_action();
  state_.outcome = state_.next_action == RoutingEngineAction::none
      ? RoutingEngineOutcome::exhausted : RoutingEngineOutcome::progressed;
}
void RoutingEngine::schedule_after_detailed() {
  if (state_.working_detailed && state_.working_detailed->complete()) {
    state_.accepted_plan = state_.working_plan;
    state_.accepted_detailed = state_.working_detailed;
    state_.external_validation_required = true;
    state_.outcome = RoutingEngineOutcome::awaiting_external_validation;
    state_.next_action = RoutingEngineAction::none;
    return;
  }
  collect_diagnoses();
  state_.next_action = select_action();
  state_.outcome = state_.next_action == RoutingEngineAction::none
                       ? RoutingEngineOutcome::exhausted
                       : RoutingEngineOutcome::progressed;
}

RoutingEngineStepResult RoutingEngine::advance(RoutingEngineStepBudget budget,
                                               std::stop_token stop) {
  RoutingEngineStepResult out;
  out.outcome = state_.outcome;
  out.generation = state_.generation;
  if (experiment_metadata_exhausted_ || state_.experiment_schedule.trials.size() >= 4096U) {
    pending_.reset();
    parked_.clear();
    stop_speculative_preparation();
    state_.outcome = RoutingEngineOutcome::exhausted;
    out.outcome = state_.outcome;
    out.diagnostic = "bounded experiment metadata capacity reached; checkpoint retained";
    emit_engine_event(telemetry_, "end", RoutingEngineAction::none, state_.generation, 0U,
        out.diagnostic + "; key_bytes=" + std::to_string(experiment_key_bytes_) +
        "; physical_states=" + std::to_string(physical_keys_.size()) +
        "; physical_records=" + std::to_string(physical_records_.size()) +
        "; history_states=" + std::to_string(experiment_keys_.size()));
    return out;
  }
  if (terminal()) {
    pending_.reset();
    parked_.clear();
    out.diagnostic = "engine awaits caller or is terminal";
    return out;
  }
  if (stop.stop_requested() || (deadline_ && deadline_->source.stop_requested())) {
    if (pending_ && pending_->action == RoutingEngineAction::construct_copper_scaffold) {
      // Join before reading the worker's immutable connection checkpoint. Cancellation at a
      // search checkpoint may unwind its stack before construct_copper_scaffold returns.
      pending_->deadline_link.reset();
      pending_->kernel.reset();
      if (pending_->scaffold && (!state_.legal_scaffold ||
          better_scaffold_checkpoint(*pending_->scaffold, *state_.legal_scaffold)))
        state_.legal_scaffold = pending_->scaffold;
      publish_retained_copper(true);
    }
    pending_.reset();
    parked_.clear();
    stop_speculative_preparation();
    state_.outcome = deadline_ && deadline_->source.stop_requested()
                         ? RoutingEngineOutcome::timed_out
                         : RoutingEngineOutcome::cancelled;
    out.outcome = state_.outcome;
    out.diagnostic = "cancelled before action";
    return out;
  }
  if (state_.outcome == RoutingEngineOutcome::awaiting_materialization ||
      state_.outcome == RoutingEngineOutcome::awaiting_external_validation)
    return out;
  if (!pending_ && (state_.attempts.size() >= config_.max_actions ||
      (config_.max_actions_without_physical_progress > 0U &&
       actions_without_physical_progress_ >= config_.max_actions_without_physical_progress))) {
    parked_.clear();
    state_.outcome = RoutingEngineOutcome::exhausted;
    out.outcome = state_.outcome;
    out.diagnostic = state_.attempts.size() >= config_.max_actions
        ? "absolute action safety limit exhausted" : "physical progress stagnation limit reached";
    return out;
  }
  if (!pending_) {
    prune_parked_experiments();
    auto next = state_.next_action == RoutingEngineAction::none ? select_action() : state_.next_action;
    if (next == RoutingEngineAction::materialize_copper) {
      const auto key = evidence_key(next);
      if (std::any_of(state_.attempts.begin(), state_.attempts.end(), [&](const auto& attempt) {
            return attempt.action == next && attempt.evidence_key == key;
          })) next = select_action(RoutingEngineAction::materialize_copper);
    }
    state_.next_action = next;
    const auto found = std::find_if(parked_.begin(), parked_.end(), [&](const auto& item) {
      return item->action == next;
    });
    if (found != parked_.end()) {
      pending_ = std::move(*found);
      parked_.erase(found);
      ++state_.experiment_resumed;
      emit_engine_event(telemetry_, "resume", pending_->action, state_.generation,
          pending_->kernel->work_units(), "resuming unchanged experiment inputs; active_us=" +
              std::to_string(pending_->active_us), pending_->active_us);
    }
  }
  if (!pending_) {
    pending_ = std::make_unique<Pending>();
    pending_->action =
        state_.next_action == RoutingEngineAction::none ? select_action() : state_.next_action;
    pending_->budget = budget;
    pending_->ordinal = state_.attempts.size();
    pending_->input_key = evidence_key(pending_->action);
    pending_->physical_context_key = state_.physical_context_key;
    pending_->experiment_state_key = experiment_state_key();
    pending_->live_copper_terrain_revision = state_.live_copper_terrain_revision;
    pending_->live_copper_terrain_features = state_.live_copper_terrain_features;
    pending_->live_copper_terrain_provisional = state_.live_copper_terrain.provisional;
    if (state_.legal_scaffold) {
      pending_->scaffold_satisfied_before = state_.legal_scaffold->satisfied_indices.size();
      pending_->scaffold_request_count_before = state_.legal_scaffold->request_count;
      pending_->scaffold_request_identities_before = state_.legal_scaffold->request_identities;
      pending_->scaffold_satisfied_identities_before =
          scaffold_satisfied_request_identities(*state_.legal_scaffold);
    }
    pending_->quality_before =
        action_quality(pending_->action, state_.working_plan ? &*state_.working_plan : nullptr,
                       state_.best_partial_detailed ? &*state_.best_partial_detailed : nullptr);
    if (pending_->action == RoutingEngineAction::none) {
      pending_.reset();
      parked_.clear();
      state_.outcome = RoutingEngineOutcome::exhausted;
      out.outcome = state_.outcome;
      out.diagnostic = "no eligible action remains for this routing state";
      return out;
    }
    const auto action = pending_->action;
    if ((action == RoutingEngineAction::global_allocation ||
         action == RoutingEngineAction::feedback_global_replan ||
         action == RoutingEngineAction::usped_global_replan) &&
        (!config_.copper_scaffold.enabled || state_.legal_scaffold))
      start_speculative_preparation();
    const bool regional_action = action == RoutingEngineAction::regional_realization ||
        action == RoutingEngineAction::xpbd_repair ||
        action == RoutingEngineAction::confined_erosion_repair;
    // Keep XPBD/erosion's whole-plan alternatives available if the retained topology is trapped.
    pending_->scaffold_regional = config_.max_physical_region_requests != 0U &&
        action == RoutingEngineAction::regional_realization &&
        has_pending_physical_repair(state_.legal_scaffold.get()) &&
        !config_.detailed.defer_saved_zone_fill_clearance_to_refill;
    if (pending_->scaffold_regional) {
      ++state_.scaffold_regional_actions;
      pending_->scaffold_physical_key = physical_dependency_key(true);
      const auto geometry_key = physical_dependency_key(true, false);
      const auto evidence_tokens = physical_repair_evidence_tokens(*state_.legal_scaffold);
      reconcile_physical_repair_attempts(physical_request_geometry_key_ != geometry_key,
                                         physical_request_evidence_tokens_, evidence_tokens,
                                         physical_request_attempts_);
      physical_request_attempt_key_ = pending_->scaffold_physical_key;
      physical_request_geometry_key_ = geometry_key;
      physical_request_evidence_tokens_ = evidence_tokens;
      pending_->physical_requests = select_physical_repair_requests(*state_.legal_scaffold,
          physical_request_attempts_, config_.max_physical_region_requests);
      for (const auto index : pending_->physical_requests)
        ++physical_request_attempts_[index];
      state_.scaffold_regional_missing_requests = pending_->physical_requests.size();
      if (speculation_) {
        bool finished = false;
        {
          std::lock_guard lock(speculation_->mutex);
          finished = speculation_->ready;
          if (finished && speculation_->physical && speculation_->prepared &&
              speculation_->physical_key == pending_->scaffold_physical_key) {
            physical_preparation_ = std::move(speculation_->prepared);
            physical_preparation_key_ = pending_->scaffold_physical_key;
            ++state_.speculative_preparations_used;
          } else {
            speculation_->worker.request_stop();
            ++state_.speculative_preparations_discarded;
          }
        }
        if (finished) speculation_.reset();
      }
      if (physical_preparation_ && physical_preparation_key_ == pending_->scaffold_physical_key) {
        pending_->preparation = std::move(physical_preparation_);
        ++state_.physical_preparation_reuses;
      } else {
        physical_preparation_.reset();
        pending_->preparation = std::make_shared<RegionalRoutingPreparation>();
      }
    } else if (action == RoutingEngineAction::construct_copper_scaffold) {
      pending_->scaffold_physical_key = physical_dependency_key(true);
    }
    if (regional_action && !pending_->scaffold_regional) acquire_regional_preparation();
    pending_->before = routing_quality(state_.working_plan ? &*state_.working_plan : nullptr,
                                      state_.working_detailed ? &*state_.working_detailed : nullptr);
    pending_->best_scene = pending_->before;
    pending_->sampled_quality = pending_->before;
    pending_->sampled_evidence = physical_evidence(
        state_.working_plan ? &*state_.working_plan : nullptr,
        state_.working_detailed ? &*state_.working_detailed : nullptr);
    pending_->best_evidence = pending_->sampled_evidence;
    pending_->dependency_key = continuation_key(pending_->action);
    ++state_.experiment_started;
    if (std::none_of(state_.experiment_schedule.trials.begin(), state_.experiment_schedule.trials.end(),
        [&](const auto& trial) { return trial.strategy == static_cast<std::size_t>(action) &&
            trial.state_key == pending_->experiment_state_key; }))
      ++state_.experiment_initial_explorations;
    // Physical progress preserves connectivity; abstract overflow does not gate its priority.
    pending_->hard_gates_before = state_.working_plan && topology_connected(*state_.working_plan);
    if (state_.working_plan && has_evaluated_physical_scene(*state_.working_plan)) {
      const auto quality = routing_quality(&*state_.working_plan, nullptr);
      if (!state_.best_evaluated_physical_scene ||
          RoutingEngineProgressBudget::strict_physical_improvement(
              *state_.best_evaluated_physical_scene, quality, true))
        state_.best_evaluated_physical_scene = quality;
    }
    pending_->evaluated_scene = state_.best_evaluated_physical_scene;
    if (regional_action && !pending_->scaffold_regional)
      pending_->preparation = std::move(regional_preparation_);
    const auto preparation = pending_->preparation;
    const bool decisions = config_.bounded_regional_decisions;
    const auto prior = state_.working_plan;
    const auto accepted = state_.accepted_detailed;
    const auto diagnoses = state_.pending_diagnoses;
    auto planner = config_.planner;
    planner.regional.enable_xpbd_relaxation =
        planner.regional.enable_xpbd_relaxation && config_.enable_xpbd;
    planner.global.live_copper_terrain = state_.live_copper_terrain;
    planner.global.live_copper_terrain_revision = state_.live_copper_terrain_revision;
    if (config_.planning_mode == RoutingEnginePlanningMode::split_stages) {
      const auto total = std::max<std::size_t>(1U, planner.global.max_iterations);
      const auto epochs = std::min(planner.max_routability_feedback_retries, total) + 1U;
      const auto epoch = total / epochs + (total % epochs != 0U ? 1U : 0U);
      planner.global.max_iterations =
          std::min(total, pending_->action == RoutingEngineAction::global_allocation &&
                                      planner.interleave_routability_feedback &&
                                      planner.max_routability_feedback_retries > 0U
                                  ? std::size_t{1U}
                                  : std::max<std::size_t>(1U, epoch));
    }
    const auto detail = config_.detailed;
    const auto scaffold_config = config_.copper_scaffold;
    const auto retained_scaffold = state_.legal_scaffold;
    std::optional<std::vector<GlobalRoutePath>> rebound_requests;
    if (action == RoutingEngineAction::construct_copper_scaffold && retained_scaffold) {
      auto rebound = derive_scaffold_terminal_rebinds(materialization_source_, *retained_scaffold,
          state_.scaffold_terminal_rebind_attempted_proofs, physical_dependency_key(true, false));
      if (!rebound.second.empty()) {
        rebound_requests = std::move(rebound.first);
        pending_->scaffold_terminal_rebinds = std::move(rebound.second);
        // Consume before starting the kernel, including a cancelled or rejected transaction.
        // Each exact producer proof gets one attempt per immutable physical scene.
        for (const auto& binding : pending_->scaffold_terminal_rebinds)
          state_.scaffold_terminal_rebind_attempted_proofs.push_back(binding.proof);
      }
    }
    const auto mode = config_.planning_mode;
    const auto sampled_repair = config_.post_negotiation_sampled_conflict_repair;
    const auto repair_cones = config_.max_post_negotiation_sampled_conflict_repair_cones;
    auto* tx = pending_.get();
    auto* board = &planning_board_;
    auto* source = &materialization_source_;
    auto* telemetry = telemetry_;
    emit_engine_event(telemetry_, "begin", action, state_.generation, 0U, "kernel created");
    tx->kernel = std::make_unique<RoutingKernel>([tx, action, prior, accepted, diagnoses, planner,
                                                  detail, scaffold_config, retained_scaffold, rebound_requests,
                                                  mode, sampled_repair, repair_cones, board,
                                                  source, preparation, decisions,
                                                  telemetry](std::stop_token token) mutable {
      if (action == RoutingEngineAction::global_allocation ||
          action == RoutingEngineAction::feedback_global_replan ||
          action == RoutingEngineAction::usped_global_replan) {
        auto p = planner;
        if (p.full_strength_first_pass) {
          activate_full_strength(p.global);
          if (action == RoutingEngineAction::global_allocation &&
              !p.clearance_shadows_advisory.has_value())
            p.global.clearance_shadows_advisory_only = false;
        }
        if (tx->budget.global_iterations)
          p.global.max_iterations = std::min(p.global.max_iterations, tx->budget.global_iterations);
        auto knowledge = prior ? prior->routability_knowledge : RoutabilityKnowledge{};
        if (!prior) {
          knowledge.history = p.global.routability_history;
          knowledge.success_history = p.global.hydraulic_success_history;
        }
        accumulate_routability_knowledge(knowledge, diagnoses);
        p.global.routability_history = knowledge.history;
        p.global.hydraulic_success_history = knowledge.success_history;
        p.global.feedback_epoch_index +=
            tx->action == RoutingEngineAction::global_allocation ? 0U : tx->ordinal + 1U;
        if (action == RoutingEngineAction::usped_global_replan) {
          p.global.global_navmesh_usped.enabled = true;
          p.global.use_navmesh_funnel_spline_seeds = true;
        }
        if (mode == RoutingEnginePlanningMode::legacy_plan_routes)
          tx->plan = plan_routes_legacy(*board, p, token, telemetry);
        else {
          RoutingPlanResult r;
          std::set<NetId> affected;
          for (const auto& d : diagnoses)
            affected.insert(d.resource.nets.begin(), d.resource.nets.end());
          if (prior)
            affected.insert(prior->coarse_global.unrouted_nets.begin(),
                            prior->coarse_global.unrouted_nets.end());
          if (sampled_repair)
            r.coarse_global = route_global_with_sampled_conflict_repair(
                *board, p.global, repair_cones, token, telemetry);
          else if (prior && !prior->coarse_global.paths.empty() && !affected.empty())
            r.coarse_global = route_global_incremental(
                *board, prior->coarse_global, std::vector<NetId>(affected.begin(), affected.end()),
                p.global, token, telemetry, true);
          else
            r.coarse_global = route_global(*board, p.global, token, telemetry);
          r.routability_knowledge = std::move(knowledge);
          if (prior)
            r.cumulative_candidate_pipeline_stats = prior->cumulative_candidate_pipeline_stats;
          r.global_feedback_epochs = prior ? prior->global_feedback_epochs + 1U : 1U;
          r.cumulative_global_iterations =
              (prior ? prior->cumulative_global_iterations : 0U) + r.coarse_global.iterations;
          r.paths = r.coarse_global.paths;
          r.stop_reason = r.coarse_global.complete()
                              ? RoutingPlanStopReason::regional_conflicts
                              : RoutingPlanStopReason::coarse_global_incomplete;
          r.diagnostic = "split global checkpoint";
          tx->plan = std::move(r);
        }
        return;
      }
      if (action == RoutingEngineAction::construct_copper_scaffold && prior) {
        auto domain = prior->coarse_global;
        auto requests = retained_scaffold && !retained_scaffold->requests.empty()
            ? retained_scaffold->requests : scaffold_requests(*source, domain);
        if (rebound_requests) requests = *rebound_requests;
        if (retained_scaffold && retained_scaffold->complete) {
          requests = append_residual_physical_demands(*retained_scaffold);
          if (requests.size() > retained_scaffold->requests.size())
            emit_engine_event(telemetry, "physical-demands", action, 0U, 0U,
                "appended=" + std::to_string(requests.size() - retained_scaffold->requests.size()) +
                "; preserved=" + std::to_string(retained_scaffold->requests.size()));
        }
        domain.paths = requests;
        const auto scaffold = scaffold_physical_scene(*source, detail, domain);
        if (!scaffold) return;
        CopperScaffoldResult seed;
        if (retained_scaffold) seed = *retained_scaffold;
        // Coarse replans provide proposals, never replacement identities for checked demands.
        append_scaffold_proposals(seed, prior->coarse_global.paths, requests);
        tx->scaffold = std::make_shared<CopperScaffoldResult>(construct_copper_scaffold(
            *scaffold, requests, scaffold_config, token, &seed,
            [tx, telemetry, action](const CopperScaffoldResult& checkpoint) {
              tx->scaffold = std::make_shared<CopperScaffoldResult>(checkpoint);
              emit_engine_event(telemetry, "checkpoint", action, 0U, checkpoint.counters.expansions,
                  "requests=" + std::to_string(checkpoint.request_count) +
                  "; satisfied=" + std::to_string(checkpoint.satisfied_indices.size()) +
                  "; missing=" + std::to_string(checkpoint.missing_indices.size()));
            }));
        // Regional proposal admission can subsequently rebuild this checkpoint with no
        // search. Persist this pass's evidence now, rather than reporting those zeroed
        // counters as if they described the search that produced the retained copper.
        if (telemetry) {
          const auto& result = *tx->scaffold;
          const auto& c = result.counters;
          const auto object_list = [](const std::vector<ObjectId>& objects) {
            std::vector<std::string> values;
            values.reserve(objects.size());
            for (const auto& object : objects) values.push_back(object.value());
            return telemetry_evidence_list(std::move(values));
          };
          const auto net_list = [](const std::vector<NetId>& nets) {
            std::vector<std::string> values;
            values.reserve(nets.size());
            for (const auto& net : nets) values.push_back(net.value());
            return telemetry_evidence_list(std::move(values));
          };
          emit_engine_event(telemetry, "scaffold-search-result", action, tx->ordinal, c.expansions,
              "satisfied=" + std::to_string(result.satisfied_indices.size()) +
              "; missing=" + std::to_string(result.missing_indices.size()) +
              "; search_us=" + std::to_string(c.search_microseconds) +
              "; edge_validation_us=" + std::to_string(c.edge_validation_microseconds) +
              "; attachment_us=" + std::to_string(c.attachment_generation_microseconds) +
              "; outward_probes=" + std::to_string(c.outward_attachment_probes) +
              "; outward_attachments=" + std::to_string(c.outward_attachments) +
              "; goal_rejections=" + std::to_string(c.goal_rejections) +
              "; victim_attempts=" + std::to_string(c.victim_exchanges_attempted) +
              "; victim_accepted=" + std::to_string(c.victim_exchanges_accepted) +
              "; coupled_attempts=" + std::to_string(c.coupled_victim_exchanges_attempted) +
              "; coupled_accepted=" + std::to_string(c.coupled_victim_exchanges_accepted) +
              "; coupled_budget_exhaustions=" + std::to_string(c.coupled_expansion_budget_exhaustions) +
              "; transactional_terminal_rebind_attempts=" + std::to_string(c.transactional_terminal_rebind_attempts) +
              "; transactional_terminal_rebinds_accepted=" + std::to_string(c.transactional_terminal_rebinds_accepted) +
              "; preservation_repair_attempts=" + std::to_string(c.preservation_repair_attempts) +
              "; preservation_demands_accepted=" + std::to_string(c.preservation_demands_accepted) +
              "; preservation_repair_rollbacks=" + std::to_string(c.preservation_repair_rollbacks) +
              "; rollbacks=" + std::to_string(c.exchange_rollbacks) +
              "; rollback_incoming=" + std::to_string(c.exchange_rollbacks_incoming_unroutable) +
              "; rollback_victim=" + std::to_string(c.exchange_rollbacks_victim_unroutable) +
              "; rollback_connectivity=" + std::to_string(c.exchange_rollbacks_connectivity_not_preserved) +
              "; rollback_cancelled=" + std::to_string(c.exchange_rollbacks_cancelled));
          for (const auto index : result.missing_indices) {
            if (index >= result.request_evidence.size()) continue;
            const auto& evidence = result.request_evidence[index];
            emit_engine_event(telemetry, "scaffold-request-result", action, tx->ordinal, 0U,
                "request=" + std::to_string(index) + "; net=" + evidence.net.value() +
                "; connection=" + std::to_string(evidence.connection_index) +
                "; termination=" + std::string(to_string(evidence.termination)) +
                "; source_anchors=" + std::to_string(evidence.source_component_anchors) +
                "; target_anchors=" + std::to_string(evidence.target_component_anchors) +
                "; source_attachments=" + std::to_string(evidence.source_legal_attachments) +
                "; target_attachments=" + std::to_string(evidence.target_legal_attachments) +
                "; blocker_objects=" + std::to_string(evidence.blocker_objects.size()) +
                "; blocker_ids=" + object_list(evidence.blocker_objects) +
                "; blocker_nets=" + net_list(evidence.blocker_nets));
          }
          for (const auto& rollback : result.exchange_rollbacks) {
            emit_engine_event(telemetry, "scaffold-exchange-rollback", action, tx->ordinal, 0U,
                "incoming=" + telemetry_evidence_list({rollback.request_identity}) +
                "; reason=" + std::string(to_string(rollback.reason)) +
                "; victims=" + telemetry_evidence_list(rollback.victim_request_identities, true) +
                "; disconnected_requests=" + telemetry_evidence_list(rollback.disconnected_request_identities) +
                "; pad_components_split=" + (rollback.pad_components_split ? "true" : "false") +
                "; blocker_ids=" + object_list(rollback.blocker_objects) +
                "; blocker_nets=" + net_list(rollback.blocker_nets));
          }
        }
        tx->plan = certified_scaffold_plan(*scaffold, *prior, *tx->scaffold,
                                           planner.quantized_copper_oracle, token, tx->scaffold_oracle);
        return;
      }
      if (action == RoutingEngineAction::probe_quantized_copper && prior) {
        auto p = *prior;
        p.quantized_copper_oracle = probe_quantized_copper_feasibility(
            *board, p.coarse_global, planner.quantized_copper_oracle, token);
        accumulate_routability_knowledge(p.routability_knowledge,
                                         p.quantized_copper_oracle.diagnoses);
        ++p.cheap_physical_probes;
        tx->plan = std::move(p);
        return;
      }
      if ((action == RoutingEngineAction::regional_realization ||
           action == RoutingEngineAction::confined_erosion_repair ||
           action == RoutingEngineAction::xpbd_repair) &&
          prior) {
        auto p = *prior;
        auto r = planner.regional;
        accumulate_routability_knowledge(p.routability_knowledge, diagnoses);
        r.enable_spline_realization = planner.curved_tracks;
        r.enable_straight_realization = !planner.curved_tracks;
        // Every bounded physical prefix may use the coupled XPBD/AVBD proposal stage.
        // The dedicated XPBD action remains an additional diagnosis-driven repair attempt.
        r.enable_xpbd_relaxation = planner.curved_tracks && planner.regional.enable_xpbd_relaxation;
        r.enable_xpbd_feedback = r.enable_xpbd_relaxation;
        if (action != RoutingEngineAction::confined_erosion_repair)
          r.max_confined_anisotropic_erosion_checks = 0U;
        if (planner.full_strength_first_pass) {
          r.frozen_copper_context = true;
          r.expand_frozen_copper_scope = true;
          r.require_legal_linear_scaffold = false;
        }
        if (tx->budget.regional_bundle_states)
          r.max_bundle_search_states =
              std::min(r.max_bundle_search_states, tx->budget.regional_bundle_states);
        if (action == RoutingEngineAction::confined_erosion_repair &&
            tx->budget.confined_erosion_checks)
          r.max_confined_anisotropic_erosion_checks = std::min(
              r.max_confined_anisotropic_erosion_checks, tx->budget.confined_erosion_checks);
        if (r.enable_xpbd_relaxation) {
          if (tx->budget.xpbd_iterations)
            r.xpbd.iterations = std::min(r.xpbd.iterations, tx->budget.xpbd_iterations);
        }
        r.physical_conflict_witnesses = diagnoses;
        r.xpbd_history = p.routability_knowledge.history;
        RegionalDecisionObserver observer;
        if (decisions) {
          r.max_spline_window_workers = 1U;
          observer = [tx, probe = planner.routability_probe](const RegionalDecisionPoint& point) {
            tx->decision = point;
            if (tx->scaffold_regional && point.checkpoint) {
              auto diagnoses = diagnose_regional_routability(*point.checkpoint, probe);
              if (diagnoses.size() > 64U) diagnoses.resize(64U);
              tx->physical_evidence = RoutingPhysicalRepairEvidence{
                  .physical_key = tx->scaffold_physical_key,
                  .request_indices = tx->physical_requests,
                  .diagnoses = std::move(diagnoses),
                  .packing_rejections = point.checkpoint->candidate_pipeline_stats.candidate_preflight_packing_rejections,
                  .no_good_rejections = point.checkpoint->conflict_directed_no_good_rejections,
                  .no_good_unknown_withheld = point.checkpoint->conflict_directed_no_good_unknown_withheld,
                  .regional_fragments = point.checkpoint->fragments.size(),
                  .regional_windows = point.checkpoint->windows.size(),
                  .regional_bundle_searches =
                      point.checkpoint->candidate_pipeline_stats.bundle_search_invocations,
                  .regional_physical_requests = point.checkpoint->source_physical_repair_requests,
                  .regional_admitted_physical_requests =
                      point.checkpoint->admitted_physical_repair_requests,
                  .regional_configured_blocker_objects =
                      point.checkpoint->configured_physical_repair_blockers,
                  .regional_blocker_objects = point.checkpoint->resolved_physical_repair_blockers,
                  .scene_evaluated = point.scene_evaluated,
                  .candidate_available = point.checkpoint->spline_candidate_available,
                  .diagnostic = point.diagnostic};
            }
          };
        }
        if (tx->scaffold_regional && retained_scaffold) {
          auto domain = prior->coarse_global;
          domain.paths = retained_scaffold->requests;
          const auto physical_source = scaffold_physical_scene(*source, detail, domain);
          if (!physical_source) return;
          GlobalRouteResult unresolved;
          unresolved.stop_reason = GlobalRouterStopReason::complete;
          r.physical_scaffold_requests.clear();
          for (const auto index : tx->physical_requests)
            if (index < retained_scaffold->requests.size()) {
              const auto& demand = retained_scaffold->requests[index];
              GlobalRoutePath path;
              path.net = demand.net; path.connection_index = demand.connection_index;
              path.points = demand.points;
              path.segment_widths = demand.segment_widths;
              path.segment_width_kinds = demand.segment_width_kinds;
              // Resource/corridor IDs belong to the old allocation, not this physical scene.
              unresolved.paths.push_back(std::move(path));

              RegionalPhysicalRepairRequest scope;
              scope.net = demand.net;
              scope.connection_index = demand.connection_index;
              if (index < retained_scaffold->request_evidence.size()) {
                scope.blocker_objects = retained_scaffold->request_evidence[index].blocker_objects;
                scope.configured_blocker_objects = scope.blocker_objects.size();
                const auto exists_in_scene = [&](const ObjectId& id) {
                  const auto has = [&](const auto& objects) {
                    return std::any_of(objects.begin(), objects.end(),
                                       [&](const auto& object) { return object.id == id; });
                  };
                  if (has(retained_scaffold->copper.tracks) ||
                      has(retained_scaffold->copper.track_arcs) ||
                      has(retained_scaffold->copper.track_beziers) ||
                      has(retained_scaffold->copper.vias) || has(retained_scaffold->copper.zones))
                    return true;
                  for (const auto& footprint : retained_scaffold->copper.footprints)
                    if (std::any_of(footprint.pads.begin(), footprint.pads.end(),
                                    [&](const auto& pad) { return pad.id == id; }))
                      return true;
                  return false;
                };
                std::erase_if(scope.blocker_objects,
                              [&](const ObjectId& id) { return !exists_in_scene(id); });
                std::sort(scope.blocker_objects.begin(), scope.blocker_objects.end());
                scope.blocker_objects.erase(
                    std::unique(scope.blocker_objects.begin(), scope.blocker_objects.end()),
                    scope.blocker_objects.end());
              }
              r.physical_scaffold_requests.push_back(std::move(scope));
            }
          std::sort(r.physical_scaffold_requests.begin(), r.physical_scaffold_requests.end(),
                    [](const auto& lhs, const auto& rhs) {
                      return std::tie(lhs.net, lhs.connection_index, lhs.blocker_objects) <
                             std::tie(rhs.net, rhs.connection_index, rhs.blocker_objects);
                    });
          r.physical_scaffold_requests.erase(
              std::unique(r.physical_scaffold_requests.begin(), r.physical_scaffold_requests.end()),
              r.physical_scaffold_requests.end());
          // Established copper remains immutable board context. Failed-demand scopes contain
          // only original request fragments; blocker objects merely bound each repair window.
          r.frozen_copper_context = true;
          r.expand_frozen_copper_scope = false;
          r.prioritize_frozen_scope_expansion = false;
          r.require_legal_linear_scaffold = false;
          emit_engine_event(telemetry, "physical-region", action, 0U, 0U,
              "retained physical copper; missing_requests=" + std::to_string(unresolved.paths.size()));
          auto regional = analyze_regional_conflicts(retained_scaffold->copper, unresolved, r,
              token, telemetry, std::move(observer), preparation);
          tx->physical_evidence = RoutingPhysicalRepairEvidence{
              .physical_key = tx->scaffold_physical_key,
              .request_indices = tx->physical_requests,
              .diagnoses = diagnose_regional_routability(regional, planner.routability_probe),
              .packing_rejections = regional.candidate_pipeline_stats.candidate_preflight_packing_rejections,
              .no_good_rejections = regional.conflict_directed_no_good_rejections,
              .no_good_unknown_withheld = regional.conflict_directed_no_good_unknown_withheld,
              .regional_fragments = regional.fragments.size(),
              .regional_windows = regional.windows.size(),
              .regional_bundle_searches =
                  regional.candidate_pipeline_stats.bundle_search_invocations,
              .regional_physical_requests = regional.source_physical_repair_requests,
              .regional_admitted_physical_requests = regional.admitted_physical_repair_requests,
              .regional_configured_blocker_objects =
                  regional.configured_physical_repair_blockers,
              .regional_blocker_objects = regional.resolved_physical_repair_blockers,
              .scene_evaluated = regional.native_scene_evaluated || regional.straight_scene_evaluated,
              .candidate_available = regional.spline_candidate_available,
              .diagnostic = regional.diagnostic};
          if (tx->physical_evidence->diagnoses.size() > 64U)
            tx->physical_evidence->diagnoses.resize(64U);
          if (token.stop_requested()) return;
          auto proposals = *retained_scaffold;
          // The regional whole-board gate can fail because other demand components remain
          // open. Treat its paths only as proposals: per-request exact admission below retains
          // independently legal connections without weakening whole-board certification.
          append_scaffold_proposals(proposals, regional.spline_candidate_paths, retained_scaffold->requests);
          auto checks = scaffold_config;
          checks.maximum_expansions_per_connection = 0U;
          checks.maximum_victim_exchanges = 0U;
          checks.maximum_coupled_exchange_attempts = 0U;
          checks.maximum_displacement_attempts = 0U;
          checks.maximum_joint_passage_candidates = 0U;
          tx->scaffold = std::make_shared<CopperScaffoldResult>(construct_copper_scaffold(
              *physical_source, retained_scaffold->requests, checks, token, &proposals));
          tx->plan = certified_scaffold_plan(*physical_source, *prior, *tx->scaffold,
                                             planner.quantized_copper_oracle, token, tx->scaffold_oracle);
          return;
        }
        p.regional = analyze_regional_conflicts(*board, p.coarse_global, r, token, telemetry,
                                                std::move(observer), preparation);
        p.regional_evaluated = true;
        accumulate_regional_candidate_pipeline_stats(p.cumulative_candidate_pipeline_stats,
                                                     p.regional.candidate_pipeline_stats);
        ++p.regional_feedback_passes;
        const auto regional_diagnoses =
            diagnose_regional_routability(p.regional, planner.routability_probe);
        accumulate_routability_knowledge(p.routability_knowledge, regional_diagnoses);
        p.paths = p.regional.spline_candidate_available ? p.regional.spline_candidate_paths
                                                        : p.coarse_global.paths;
        p.stop_reason =
            topology_connected(p) && p.regional.complete() &&
                    (p.coarse_global.complete() || p.regional.spline_candidate_available)
                ? RoutingPlanStopReason::complete
                : RoutingPlanStopReason::regional_conflicts;
        p.diagnostic = p.regional.diagnostic;
        tx->plan = std::move(p);
        return;
      }
      if (action == RoutingEngineAction::tighten_routes && prior) {
        auto p = *prior;
        const bool shortened = tighten_regional_spline_candidate(
            *board, p.coarse_global, p.regional, planner.regional, token, telemetry);
        tx->tightening_improved = shortened;
        if (shortened) p.paths = p.regional.spline_candidate_paths;
        p.diagnostic = shortened ? "certified route tightening accepted"
                                 : "route tightening retained certified input";
        tx->plan = std::move(p);
        return;
      }
      if (action == RoutingEngineAction::materialize_copper && prior) {
        auto candidate = accepted ? materialize_detailed_route_incremental(*source, *prior,
                                                                           *accepted, detail, token)
                                  : materialize_detailed_route(*source, *prior, detail, token);
        // Retain the pre-engine conventional materialization fallback. This is a separate exact
        // copper transaction, never a promotion based merely on combinatorial connectivity.
        if (!token.stop_requested() && topology_ready(*prior) &&
            candidate.stop_reason == DetailedRouterStopReason::incomplete_global_route) {
          auto coarse = prior->coarse_global;
          coarse.stop_reason = GlobalRouterStopReason::complete;
          auto fallback = materialize_detailed_route(*source, coarse, detail, token);
          if (detailed_order(fallback) < detailed_order(candidate))
            candidate = std::move(fallback);
        }
        tx->detailed = std::move(candidate);
      }
    });
    if (deadline_)
      pending_->deadline_link = std::make_unique<std::stop_callback<std::function<void()>>>(
          deadline_->source.get_token(), [k = pending_->kernel.get()] { k->request_stop(); });
  }
  const auto before = pending_->quality_before;
  const auto action = pending_->action;
  const bool pending_regional_action = action == RoutingEngineAction::regional_realization ||
      action == RoutingEngineAction::xpbd_repair ||
      action == RoutingEngineAction::confined_erosion_repair;
  const auto work = pending_->kernel->work_units();
  const auto publish_progress = [&](std::size_t units) {
    const auto now = std::chrono::steady_clock::now();
    if (now - pending_->last_progress < std::chrono::milliseconds{500}) return;
    pending_->last_progress = now;
    emit_engine_event(telemetry_, "running", action, state_.generation, units,
                      "awaiting next routing checkpoint; geometry remains provisional",
                      static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                          now - pending_->started).count()));
  };
  auto& active_regional_budget = pending_->preparing ? state_.preparation_work_budget
                                                    : state_.regional_work_budget;
  auto slice_work = budget.work_units == 0U ? 256U : budget.work_units;
  const auto active_start = std::chrono::steady_clock::now();
  const auto status = pending_->kernel->advance(
      {.work_units = slice_work,
       .time_slice =
           budget.time_slice.count() <= 0 ? std::chrono::milliseconds{50} : budget.time_slice},
      stop, publish_progress);
  const auto active_delta = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - active_start).count());
  pending_->active_us = plus(pending_->active_us, active_delta);
  state_.experiment_active_us = plus(state_.experiment_active_us, active_delta);
  publish_progress(pending_->kernel->work_units());
  out.action = action;
  out.work_units = pending_->kernel->work_units() - work;
  const auto regional_budget_diagnostic = [&] {
    return "scheduler=deadline_experiments; preparation_work=" +
        std::to_string(state_.preparation_work_budget.consumed) + "; repair_work=" +
        std::to_string(state_.regional_work_budget.consumed) + "; active_us=" +
        std::to_string(state_.experiment_active_us) + "; suspended=" +
        std::to_string(state_.experiment_suspended) + "; resumed=" +
        std::to_string(state_.experiment_resumed) + "; invalidated=" +
        std::to_string(state_.experiment_invalidated);
  };
  if (pending_regional_action) {
    const auto total_work = pending_->kernel->work_units();
    active_regional_budget.consume(difference(total_work, pending_->budget_work_units));
    pending_->budget_work_units = total_work;
  }
  if (pending_->physical_evidence) {
    const auto& evidence = *pending_->physical_evidence;
    auto existing = std::find_if(state_.physical_repair_evidence.begin(), state_.physical_repair_evidence.end(),
        [&](const auto& item) { return item.physical_key == evidence.physical_key &&
            item.request_indices == evidence.request_indices; });
    if (existing != state_.physical_repair_evidence.end()) *existing = evidence;
    else {
      state_.physical_repair_evidence.push_back(evidence);
      const auto limit = std::max<std::size_t>(1U, config_.max_physical_failure_records);
      if (state_.physical_repair_evidence.size() > limit)
        state_.physical_repair_evidence.erase(state_.physical_repair_evidence.begin());
    }
    pending_->physical_evidence.reset();
    collect_diagnoses();
  }
  if (status == RoutingKernelStatus::yielded) {
    ++pending_->yields;
    if (action == RoutingEngineAction::construct_copper_scaffold && pending_->scaffold &&
        (!state_.legal_scaffold ||
         better_scaffold_checkpoint(*pending_->scaffold, *state_.legal_scaffold)))
      state_.legal_scaffold = pending_->scaffold;
    if (action == RoutingEngineAction::construct_copper_scaffold) publish_retained_copper();
    // A kernel yield parks all mutable child work, including parallel candidate workers.
    // Paused transactions can be retained or retired without exposing their private scene.
    const bool active_limit = pending_regional_action && config_.max_experiment_active_time.count() > 0 &&
        pending_->active_us >= static_cast<std::uint64_t>(config_.max_experiment_active_time.count()) * 1000U;
    if (pending_->decision || active_limit) {
      auto decision = pending_->decision ? std::move(*pending_->decision) : RegionalDecisionPoint{};
      if (!pending_->decision)
        decision.diagnostic = "experiment active-time limit reached at parked checkpoint";
      pending_->decision.reset();
      if (decision.phase == RegionalDecisionPhase::physical_feedback_started || decision.scene_evaluated)
        pending_->preparing = false;
      const bool preparation_exhausted = false;
      out.decision_point = true;
      ++state_.decision_points;
      // A candidate/bounded lane batch is actual search even when its geometry
      // solver has no numeric kernel counters. Phase announcements and skipped
      // portfolio entries remain observable but cannot consume this budget.
      if (decision.phase == RegionalDecisionPhase::lane_search_progress ||
          decision.phase == RegionalDecisionPhase::spline_candidate_completed)
        ++pending_->stagnant_decisions;
      state_.regional_phase = decision.diagnostic;
      if (decision.checkpoint && state_.working_plan && !pending_->scaffold_regional) {
        auto diagnoses = diagnose_regional_routability(*decision.checkpoint,
                                                       config_.planner.routability_probe);
        std::erase_if(diagnoses, [](const auto& d) { return d.locally_resolved; });
        if (decision.scene_evaluated || !diagnoses.empty())
          state_.pending_diagnoses = std::move(diagnoses);
        if (decision.scene_evaluated) {
          pending_->evaluated_checkpoint = true;
          pending_->evidence_recorded = true;
          auto checkpoint = *state_.working_plan;
          checkpoint.regional = *decision.checkpoint;
          checkpoint.regional_evaluated = true;
          checkpoint.paths = checkpoint.regional.spline_candidate_available
              ? checkpoint.regional.spline_candidate_paths : checkpoint.coarse_global.paths;
          checkpoint.stop_reason = topology_connected(checkpoint) && checkpoint.regional.complete() &&
                  (checkpoint.coarse_global.complete() || checkpoint.regional.spline_candidate_available)
              ? RoutingPlanStopReason::complete : RoutingPlanStopReason::regional_conflicts;
          const auto quality = routing_quality(&checkpoint, nullptr);
          if (quality < pending_->best_scene) {
            pending_->best_scene = quality;
            pending_->best_evidence = physical_evidence(&checkpoint, nullptr);
            pending_->stagnant_decisions = 0U;
          }
          // Only comparable whole-scene physical improvement changes measured priority.
          const bool hard_gates_preserved = pending_->hard_gates_before && topology_connected(checkpoint);
          const bool physical_improvement = pending_->evaluated_scene &&
              hard_gates_preserved && physical_scene_improved(*pending_->evaluated_scene, quality);
          if (physical_improvement) {
            pending_->physical_improvement = true;
          }
          if (!state_.best_evaluated_physical_scene || physical_improvement)
            state_.best_evaluated_physical_scene = quality;
          pending_->evaluated_scene = state_.best_evaluated_physical_scene;
          state_.working_plan = std::move(checkpoint);
          state_.working_detailed.reset();
          if (!state_.best_partial_plan || routing_quality(&*state_.working_plan, nullptr) <
                  routing_quality(&*state_.best_partial_plan, nullptr))
            state_.best_partial_plan = state_.working_plan;
        }
      }
      const bool access_failed = decision.experiment && decision.experiment->access_infeasible;
      if (access_failed) {
        RoutabilityDiagnosis diagnosis;
        diagnosis.kind = RoutabilityFailureKind::topological_trap;
        diagnosis.suggested_relaxation = RoutabilityRelaxation::change_corridor;
        diagnosis.resource.stable_id = "regional-access:" + std::to_string(
            decision.experiment->window_id.value_or(0U));
        for (const auto& fragment : decision.experiment->dependencies.owned_fragments)
          diagnosis.resource.nets.push_back(fragment.net);
        state_.pending_diagnoses.insert(state_.pending_diagnoses.begin(), std::move(diagnosis));
      }
      const bool ready = decision.scene_evaluated && state_.working_plan &&
          ready_for_materialization(*state_.working_plan, state_.pending_diagnoses);
      const bool stalled = pending_->stagnant_decisions >=
          std::max<std::size_t>(1U, config_.max_regional_decisions_without_improvement);
      const bool work_budget_exhausted = active_limit;
      const bool exhausted = state_.decision_points >= config_.max_decision_points;
      if (ready || stalled || work_budget_exhausted || exhausted) {
        RoutingEngineAttempt attempt;
        attempt.action = action;
        attempt.evidence_key = pending_->input_key;
        attempt.physical_context_key = pending_->physical_context_key;
        attempt.experiment_state_key = pending_->experiment_state_key;
        attempt.ordinal = state_.attempts.size();
        attempt.work_units = pending_->kernel->work_units();
        attempt.quality_before = before;
        attempt.physical_before = pending_->sampled_quality;
        attempt.physical_after = pending_->best_scene;
        attempt.measured_before = pending_->sampled_evidence;
        attempt.measured_after = pending_->best_evidence;
        attempt.quality_after = action_quality(action,
            state_.working_plan ? &*state_.working_plan : nullptr, nullptr);
        attempt.accepted = pending_->best_scene < pending_->before;
        attempt.physical_improvement = pending_->physical_improvement;
        attempt.evidence_recorded = pending_->evidence_recorded;
        attempt.interrupted_at_decision = true;
        attempt.live_copper_terrain_revision = pending_->live_copper_terrain_revision;
        attempt.live_copper_terrain_features = pending_->live_copper_terrain_features;
        attempt.live_copper_terrain_provisional = pending_->live_copper_terrain_provisional;
        attempt.elapsed_microseconds = pending_->active_us;
        attempt.diagnostic = ready ? "evaluated scene ready"
            : preparation_exhausted ? "regional preparation reserve exhausted"
            : work_budget_exhausted ? "experiment active-time limit reached"
            : "regional decision budget reached";
        state_.attempts.push_back(std::move(attempt));
        record_experiment(state_.attempts.back());
        // Cancel before another action can acquire the reusable cache. Retain failed
        // chunks in the cycle guard so a different family gets the next opportunity.
        emit_engine_event(telemetry_, "end", action, state_.generation,
            pending_->kernel->work_units(), state_.attempts.back().diagnostic + "; " +
                regional_budget_diagnostic(),
            state_.attempts.back().elapsed_microseconds);
        if (work_budget_exhausted && !ready) {
          // A bounded continuation may have inspected a worse checkpoint.  Its best retained
          // evidence remains selectable; restore it before the destructor requests the worker
          // stop and before another repair family receives the board state.
          if (state_.best_partial_plan) {
            state_.working_plan = state_.best_partial_plan;
            state_.working_detailed.reset();
            collect_diagnoses();
          }
        }
        if (pending_->evaluated_checkpoint && state_.working_plan) {
          ++state_.working_plan->regional_feedback_passes;
          accumulate_regional_candidate_pipeline_stats(
              state_.working_plan->cumulative_candidate_pipeline_stats,
              state_.working_plan->regional.candidate_pipeline_stats);
          replace_live_copper_terrain(make_live_copper_terrain_snapshot(
              provisional_copper_scaffold(planning_board_, state_.working_plan->paths), true, 4'096U));
          if (!state_.best_partial_plan ||
              plan_order(*state_.working_plan) <= plan_order(*state_.best_partial_plan))
            state_.best_partial_plan = state_.working_plan;
        }
        if (!ready && state_.best_partial_plan && state_.working_plan &&
            plan_order(*state_.best_partial_plan) < plan_order(*state_.working_plan)) {
          state_.working_plan = state_.best_partial_plan;
          state_.working_detailed.reset();
          collect_diagnoses();
          replace_live_copper_terrain(make_live_copper_terrain_snapshot(
              provisional_copper_scaffold(planning_board_, state_.working_plan->paths), true, 4'096U));
        }
        if (pending_->evaluated_checkpoint && state_.working_plan) {
          const auto context = evidence_key(RoutingEngineAction::none);
          if (context != state_.physical_context_key) {
            state_.physical_context_key = context;
            ++state_.physical_context_changes;
          }
        }
        working_plan_tightened_ = false;
        const auto next = ready ? (needs_tightening() ? RoutingEngineAction::tighten_routes
                                                     : RoutingEngineAction::materialize_copper)
                                : select_action(action);
        release_pending();
        ++state_.regional_action_switches;
        ++state_.generation;
        state_.next_action = next;
        state_.outcome = exhausted || next == RoutingEngineAction::none
            ? RoutingEngineOutcome::exhausted : RoutingEngineOutcome::progressed;
        if (ready && !config_.materialize && !needs_tightening()) {
          state_.accepted_plan = state_.working_plan;
          state_.next_action = RoutingEngineAction::none;
          state_.outcome = RoutingEngineOutcome::awaiting_materialization;
        }
        if (terminal() || state_.outcome == RoutingEngineOutcome::awaiting_materialization)
          parked_.clear();
        out.outcome = state_.outcome;
        out.generation = state_.generation;
        out.diagnostic = state_.attempts.back().diagnostic;
        return out;
      }
      pending_->dependency_key = continuation_key(action);
      if (yield_experiment(access_failed)) {
        out.outcome = RoutingEngineOutcome::progressed;
        out.diagnostic = "experiment parked; another strategy selected";
        return out;
      }
      ++state_.regional_continuations_resumed;
      emit_engine_event(telemetry_, "decision", action, state_.generation,
          pending_->kernel->work_units(), decision.diagnostic + "; resume retained frontier");
      out.diagnostic = decision.diagnostic;
      out.outcome = RoutingEngineOutcome::yielded;
      return out;
    }
    // Scalar progress is time-throttled across yields as well as non-yielding operations.
    out.outcome = RoutingEngineOutcome::yielded;
    out.diagnostic = "kernel yielded";
    if (yield_experiment()) {
      out.outcome = RoutingEngineOutcome::progressed;
      out.diagnostic = "experiment parked; another strategy selected";
    }
    return out;
  }
  const bool new_scaffold_oracle = pending_->scaffold_oracle.has_value();
  if (pending_->scaffold_oracle) {
    state_.scaffold_oracle = std::move(pending_->scaffold_oracle);
  }
  if (pending_->scaffold) {
    // Only the builder's published, exact-checked subset is visible, including on cancellation.
    if (!state_.legal_scaffold ||
        better_scaffold_checkpoint(*pending_->scaffold, *state_.legal_scaffold))
      state_.legal_scaffold = pending_->scaffold;
    publish_retained_copper(true);
    if (state_.legal_scaffold) {
      replace_live_copper_terrain(make_live_copper_terrain_snapshot(state_.legal_scaffold->copper, false, 4'096U));
      if (new_scaffold_oracle && pending_->scaffold && pending_->scaffold->complete)
        state_.scaffold_oracle_key = physical_dependency_key(true);
      collect_diagnoses();
    }
  }
  if (action == RoutingEngineAction::construct_copper_scaffold && !pending_->scaffold_terminal_rebinds.empty()) {
    const bool adopted = state_.legal_scaffold == pending_->scaffold;
    if (adopted) state_.scaffold_terminal_rebinds += pending_->scaffold_terminal_rebinds.size();
    if (adopted) for (const auto& binding : pending_->scaffold_terminal_rebinds)
      if (std::find(state_.scaffold_terminal_rebind_proofs.begin(),
                    state_.scaffold_terminal_rebind_proofs.end(), binding.proof) ==
          state_.scaffold_terminal_rebind_proofs.end())
        state_.scaffold_terminal_rebind_proofs.push_back(binding.proof);
  }
  if (pending_->kernel->stop_token().stop_requested()) {
    pending_.reset();
    parked_.clear();
    stop_speculative_preparation();
    state_.outcome = deadline_ && deadline_->source.stop_requested()
                         ? RoutingEngineOutcome::timed_out
                         : RoutingEngineOutcome::cancelled;
    out.outcome = state_.outcome;
    out.diagnostic = "kernel stopped; checkpoint retained";
    return out;
  }
  if (pending_->plan) {
    if (action == RoutingEngineAction::global_allocation ||
        action == RoutingEngineAction::feedback_global_replan ||
        action == RoutingEngineAction::usped_global_replan)
      state_.global_iterations_used =
          plus(state_.global_iterations_used, pending_->plan->coarse_global.iterations);
    working_plan_tightened_ = action == RoutingEngineAction::tighten_routes;
    if (working_plan_tightened_) {
      ++state_.tightening_actions;
      if (pending_->tightening_improved)
        ++state_.tightening_improvements;
    }
    if (state_.working_plan)
      retain_current_routability_knowledge(pending_->plan->routability_knowledge,
                                           state_.working_plan->routability_knowledge,
                                           config_.planner.routability_probe.max_history_entries);
    state_.working_plan = std::move(pending_->plan);
    if (action != RoutingEngineAction::probe_quantized_copper) {
      replace_live_copper_terrain(make_live_copper_terrain_snapshot(
          provisional_copper_scaffold(planning_board_, state_.working_plan->paths), true,
          4'096U));
    }
    if (!state_.best_partial_plan ||
        plan_order(*state_.working_plan) < plan_order(*state_.best_partial_plan) ||
        (working_plan_tightened_ &&
         plan_order(*state_.working_plan) == plan_order(*state_.best_partial_plan)))
      state_.best_partial_plan = state_.working_plan;
    state_.working_detailed.reset();
    // A new topology can earn regional work after its physical probe demonstrates improvement.
    // Do not permanently starve regional repair merely because the old topology spent its allowance.
    if (has_evaluated_physical_scene(*state_.working_plan)) {
      pending_->evidence_recorded = true;
      const auto quality = routing_quality(&*state_.working_plan, nullptr);
      const bool hard_gates_preserved = pending_->hard_gates_before && topology_connected(*state_.working_plan);
      if (state_.best_evaluated_physical_scene &&
          hard_gates_preserved && physical_scene_improved(*state_.best_evaluated_physical_scene, quality)) {
        pending_->physical_improvement = true;
        state_.best_evaluated_physical_scene = quality;
      } else if (!state_.best_evaluated_physical_scene) {
        state_.best_evaluated_physical_scene = quality;
      }
    }
    if (action == RoutingEngineAction::construct_copper_scaffold) {
      state_.pending_diagnoses.clear();
      schedule_after_plan();
    } else if (action == RoutingEngineAction::probe_quantized_copper) {
      state_.pending_diagnoses = state_.working_plan->quantized_copper_oracle.diagnoses;
      state_.next_action = RoutingEngineAction::regional_realization;
      state_.outcome = RoutingEngineOutcome::progressed;
    } else if (config_.planning_mode == RoutingEnginePlanningMode::split_stages &&
               (action == RoutingEngineAction::global_allocation ||
                action == RoutingEngineAction::feedback_global_replan ||
                action == RoutingEngineAction::usped_global_replan)) {
      const bool certify_connected_seed =
          topology_connected(*state_.working_plan) &&
          ((action == RoutingEngineAction::global_allocation &&
            config_.planner.full_strength_first_pass) ||
           state_.global_iterations_used >= config_.planner.global.max_iterations);
      if (config_.copper_scaffold.enabled && (!state_.working_plan->coarse_global.paths.empty() ||
                                            !state_.working_plan->coarse_global.unrouted_nets.empty())) {
        state_.next_action = RoutingEngineAction::construct_copper_scaffold;
      } else if (!topology_ready(*state_.working_plan) && !certify_connected_seed) {
        state_.pending_diagnoses = probe_global_routability(state_.working_plan->coarse_global,
                                                            config_.planner.routability_probe);
        state_.next_action = select_action();
      } else {
        state_.next_action = config_.planner.quantized_copper_oracle.enabled
                                 ? RoutingEngineAction::probe_quantized_copper
                                 : RoutingEngineAction::regional_realization;
      }
      state_.outcome = RoutingEngineOutcome::progressed;
    } else
      schedule_after_plan();
  }
  if (pending_->detailed) {
    state_.working_detailed = std::move(pending_->detailed);
    if (state_.working_detailed->candidate) {
      replace_live_copper_terrain(make_live_copper_terrain_snapshot(
          *state_.working_detailed->candidate, false, 4'096U));
    }
    if (!state_.best_partial_detailed ||
        detailed_order(*state_.working_detailed) < detailed_order(*state_.best_partial_detailed)) {
      state_.best_partial_detailed = state_.working_detailed;
      state_.best_detailed_plan = state_.working_plan;
    }
    schedule_after_detailed();
  }
  if (action == RoutingEngineAction::construct_copper_scaffold && !pending_->plan &&
      (!state_.working_plan || !state_.working_plan->legal_scaffold_complete)) {
    // Completed declared demands can expose a new physical domain. Re-enter immediately
    // only after the retained input changed; an unchanged failed construction stays bounded.
    const bool residual_domain = state_.legal_scaffold && state_.legal_scaffold->complete &&
        pending_->input_key != evidence_key(RoutingEngineAction::construct_copper_scaffold) &&
        append_residual_physical_demands(*state_.legal_scaffold).size() > state_.legal_scaffold->requests.size();
    state_.next_action = select_action(residual_domain ? RoutingEngineAction::none
        : RoutingEngineAction::construct_copper_scaffold);
    state_.outcome = state_.next_action == RoutingEngineAction::none
        ? RoutingEngineOutcome::exhausted : RoutingEngineOutcome::progressed;
  }
  RoutingEngineAttempt attempt{.action = action,
                               .evidence_key = pending_->input_key,
                               .physical_context_key = pending_->physical_context_key,
                               .experiment_state_key = pending_->experiment_state_key,
                               .ordinal = state_.attempts.size(),
                               .work_units = pending_->kernel->work_units(),
                               .quality_before = before,
                               .physical_before = pending_->sampled_quality,
                               .physical_after = routing_quality(
                                   state_.working_plan ? &*state_.working_plan : nullptr,
                                   state_.working_detailed ? &*state_.working_detailed : nullptr),
                               .quality_after = action_quality(
                                   action, state_.working_plan ? &*state_.working_plan : nullptr,
                                   state_.working_detailed ? &*state_.working_detailed : nullptr),
                               .measured_before = {},
                               .measured_after = {},
                               .elapsed_microseconds = pending_->active_us,
                               .live_copper_terrain_revision =
                                   pending_->live_copper_terrain_revision,
                               .live_copper_terrain_features =
                                   pending_->live_copper_terrain_features,
                               .live_copper_terrain_provisional =
                                   pending_->live_copper_terrain_provisional,
                               .diagnostic = {}};
  attempt.measured_before = pending_->sampled_evidence;
  attempt.measured_after = physical_evidence(state_.working_plan ? &*state_.working_plan : nullptr,
      state_.working_detailed ? &*state_.working_detailed : nullptr);
  attempt.accepted = routing_quality(state_.working_plan ? &*state_.working_plan : nullptr,
                                      state_.working_detailed ? &*state_.working_detailed : nullptr)
                     < pending_->before;
  attempt.physical_improvement = pending_->physical_improvement;
  attempt.evidence_recorded = pending_->evidence_recorded;
  if (action == RoutingEngineAction::probe_quantized_copper)
    attempt.accepted = !pending_->evaluated_scene || pending_->physical_improvement;
  attempt.accepted = attempt.accepted || pending_->tightening_improved;
  attempt.diagnostic = state_.working_plan ? state_.working_plan->diagnostic : "no checkpoint";
  if (pending_->scaffold && state_.legal_scaffold) {
    const auto& scaffold = *state_.legal_scaffold;
    const auto satisfied = scaffold.satisfied_indices.size();
    CopperScaffoldResult baseline;
    baseline.request_count = pending_->scaffold_request_count_before;
    baseline.request_identities = pending_->scaffold_request_identities_before;
    for (std::size_t request = 0U; request < baseline.request_identities.size(); ++request)
      if (pending_->scaffold_satisfied_identities_before.contains(baseline.request_identities[request]))
        baseline.satisfied_indices.push_back(request);
    const bool same_domain = same_scaffold_request_domain(scaffold, baseline);
    const bool scaffold_progress = same_domain
        ? strict_scaffold_coverage_improvement(scaffold, baseline)
        : !scaffold.satisfied_indices.empty();
    // A checked subset is a real, recoverable transaction even though it remains ineligible for
    // plan completion until the whole-scene quantized oracle succeeds. Marking it accepted keeps
    // experiment ranking from retiring this constructive action as stagnant.
    attempt.accepted = attempt.accepted || scaffold_progress;
    attempt.physical_improvement = attempt.physical_improvement || scaffold_progress;
    attempt.diagnostic = state_.legal_scaffold->diagnostic;
    attempt.diagnostic += "; scaffold requests=" + std::to_string(scaffold.request_count) +
        ", prior_requests=" + std::to_string(pending_->scaffold_request_count_before) +
        ", satisfied=" + std::to_string(satisfied) +
        ", gained=" + std::to_string(same_domain
            ? difference(satisfied, pending_->scaffold_satisfied_before) : satisfied) +
        ", domain=" + (same_domain ? "compatible" : "reconstructed");
  }
  actions_without_physical_progress_ = attempt.physical_improvement ? 0U
      : plus(actions_without_physical_progress_, 1U);
  state_.attempts.push_back(std::move(attempt));
  record_experiment(state_.attempts.back());
  if (pending_->evidence_recorded && state_.best_partial_plan && state_.working_plan &&
      plan_order(*state_.best_partial_plan) < plan_order(*state_.working_plan)) {
    auto incumbent = *state_.best_partial_plan;
    const auto& failed_diagnoses = state_.working_plan->regional_evaluated
        ? state_.working_plan->regional.local_routability_diagnoses
        : state_.working_plan->quantized_copper_oracle.diagnoses;
    accumulate_routability_knowledge(incumbent.routability_knowledge, failed_diagnoses);
    state_.working_plan = std::move(incumbent);
    state_.best_partial_plan = state_.working_plan;
    collect_diagnoses(); // Rejected geometry contributes history, not stale repair ownership.
    state_.next_action = select_action();
    state_.outcome = state_.next_action == RoutingEngineAction::none
        ? RoutingEngineOutcome::exhausted : RoutingEngineOutcome::progressed;
    replace_live_copper_terrain(make_live_copper_terrain_snapshot(
        provisional_copper_scaffold(planning_board_, state_.working_plan->paths), true, 4'096U));
  }
  if (pending_->evidence_recorded) {
    const auto context = evidence_key(RoutingEngineAction::none);
    if (context != state_.physical_context_key) {
      state_.physical_context_key = context;
      ++state_.physical_context_changes;
    }
    // Context re-entry is considered after rollback and after publishing evaluated evidence.
    if (action == RoutingEngineAction::probe_quantized_copper &&
        state_.outcome == RoutingEngineOutcome::exhausted) {
      state_.next_action = select_action();
      if (state_.next_action != RoutingEngineAction::none)
        state_.outcome = RoutingEngineOutcome::progressed;
    }
  }
  if (state_.outcome == RoutingEngineOutcome::progressed &&
      state_.next_action != RoutingEngineAction::materialize_copper &&
      state_.next_action != RoutingEngineAction::tighten_routes &&
      state_.next_action != RoutingEngineAction::probe_quantized_copper &&
      action != RoutingEngineAction::probe_quantized_copper &&
      !(state_.next_action == RoutingEngineAction::regional_realization &&
        (action == RoutingEngineAction::global_allocation ||
         action == RoutingEngineAction::feedback_global_replan ||
         action == RoutingEngineAction::usped_global_replan) &&
        topology_connected(*state_.working_plan))) {
    state_.next_action = select_action();
    if (state_.next_action == RoutingEngineAction::none)
      state_.outcome = RoutingEngineOutcome::exhausted;
  }
  emit_engine_event(
      telemetry_, "end", action, state_.generation, state_.attempts.back().work_units,
      "accepted=" + std::string(state_.attempts.back().accepted ? "yes" : "no") +
          ", elapsed_us=" + std::to_string(state_.attempts.back().elapsed_microseconds) + ", " +
          regional_budget_diagnostic(),
      state_.attempts.back().elapsed_microseconds);
  release_pending();
  if (terminal() || state_.outcome == RoutingEngineOutcome::awaiting_materialization ||
      state_.outcome == RoutingEngineOutcome::awaiting_external_validation) parked_.clear();
  ++state_.generation;
  out.outcome = state_.outcome;
  out.generation = state_.generation;
  out.accepted_checkpoint_changed =
      state_.outcome == RoutingEngineOutcome::awaiting_materialization ||
      state_.outcome == RoutingEngineOutcome::awaiting_external_validation;
  out.diagnostic = state_.attempts.back().diagnostic;
  return out;
}
void RoutingEngine::acknowledge_external_validation(std::size_t generation, bool passed,
                                                    std::vector<RoutabilityDiagnosis> diagnoses,
                                                    std::string diagnostic) {
  if (!state_.external_validation_required || generation != state_.generation)
    return;
  if (deadline_ && deadline_->source.stop_requested()) {
    state_.outcome = RoutingEngineOutcome::timed_out;
    state_.external_validation_required = false;
    return;
  }
  state_.external_validation_required = false;
  if (passed) {
    state_.outcome = RoutingEngineOutcome::complete;
    state_.next_action = RoutingEngineAction::none;
  } else {
    // Internal DRC is necessary but external failure is authoritative for publication.  Demote
    // the transaction before selecting a repair so it cannot be returned as a completed board or
    // silently reused by an incremental materialization pass.
    state_.accepted_plan.reset();
    state_.accepted_detailed.reset();
    state_.working_detailed.reset();
    state_.best_partial_detailed.reset();
    state_.best_detailed_plan.reset();
    state_.best_partial_plan.reset();
    if (state_.working_plan) {
      state_.working_plan->stop_reason = RoutingPlanStopReason::regional_conflicts;
      accumulate_routability_knowledge(state_.working_plan->routability_knowledge, diagnoses);
    }
    state_.pending_diagnoses = std::move(diagnoses);
    const auto context = evidence_key(RoutingEngineAction::none);
    if (context != state_.physical_context_key) {
      state_.physical_context_key = context;
      ++state_.physical_context_changes;
    }
    state_.next_action = select_action();
    state_.outcome = state_.next_action == RoutingEngineAction::none
                         ? RoutingEngineOutcome::exhausted
                         : RoutingEngineOutcome::progressed;
  }
  if (!state_.attempts.empty() && !diagnostic.empty())
    state_.attempts.back().diagnostic += "; external validation: " + std::move(diagnostic);
}
} // namespace flux
