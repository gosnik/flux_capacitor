#include "flux/core/board_free_space.hpp"
#include "flux/core/detailed_router.hpp"
#include "flux/core/evaluate.hpp"
#include "flux/core/free_space_offset.hpp"
#include "flux/core/free_space_skeleton.hpp"
#include "flux/core/global_router.hpp"
#include "flux/core/integrated_optimizer.hpp"
#include "flux/core/regional_router.hpp"
#include "flux/core/router_telemetry.hpp"
#include "flux/core/routing_compute.hpp"
#include "flux/core/routing_rules.hpp"
#include "flux/io/differential_pair_constraints.hpp"
#include "flux/io/kicad_export.hpp"
#include "flux/io/kicad_import.hpp"
#include "flux/io/kicad_netclass_snapshot.hpp"
#include "flux/io/orientation_constraints.hpp"
#include "flux/io/placement_envelope_constraints.hpp"
#include "flux/io/placement_group_constraints.hpp"
#include "flux/io/placement_move_priorities.hpp"
#include "flux/io/placement_region_constraints.hpp"
#include "flux/io/relative_placement_constraints.hpp"
#include "flux/io/routing_bus_constraints.hpp"
#include "flux/io/specctra.hpp"
#include "flux/generated/parity_manifest.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <system_error>
#include <tuple>
#include <utility>
#include <vector>

namespace {

using flux::Board;
using flux::BoardSide;
using flux::Footprint;
using flux::Layer;
using flux::LayerId;
using flux::LayerKind;
using flux::Microdegrees;
using flux::Nanometres;
using flux::Net;
using flux::NetId;
using flux::ObjectId;
using flux::Pad;
using flux::Point;
using flux::Polygon;
using flux::TrackSegment;

[[nodiscard]] Point mm(double x, double y) {
  return Point{Nanometres::from_mm(x), Nanometres::from_mm(y)};
}

[[nodiscard]] Polygon rectangle(double half_width_mm, double half_height_mm) {
  return Polygon{{mm(-half_width_mm, -half_height_mm), mm(half_width_mm, -half_height_mm),
                  mm(half_width_mm, half_height_mm), mm(-half_width_mm, half_height_mm)}};
}

[[nodiscard]] std::string json_quote(std::string_view value) {
  constexpr char digits[] = "0123456789abcdef";
  std::string result{"\""};
  for (const auto character : value) {
    const auto byte = static_cast<unsigned char>(character);
    switch (character) {
    case '"':
      result += "\\\"";
      break;
    case '\\':
      result += "\\\\";
      break;
    case '\b':
      result += "\\b";
      break;
    case '\f':
      result += "\\f";
      break;
    case '\n':
      result += "\\n";
      break;
    case '\r':
      result += "\\r";
      break;
    case '\t':
      result += "\\t";
      break;
    default:
      if (byte < 0x20U) {
        result += "\\u00";
        result.push_back(digits[byte >> 4U]);
        result.push_back(digits[byte & 0x0FU]);
      } else {
        result.push_back(character);
      }
      break;
    }
  }
  result.push_back('"');
  return result;
}

void print_net_ids(const std::vector<NetId>& nets) {
  std::cout << '[';
  for (std::size_t index = 0; index < nets.size(); ++index) {
    std::cout << (index == 0U ? "" : ", ") << json_quote(nets[index].value());
  }
  std::cout << ']';
}

[[nodiscard]] std::string json_string_array(const std::vector<std::string>& values) {
  std::string result{"["};
  for (std::size_t index = 0; index < values.size(); ++index) {
    result += index == 0U ? "" : ", ";
    result += json_quote(values[index]);
  }
  result += ']';
  return result;
}

void print_pin_escape_failures(const std::vector<flux::PinEscapeFailure>& failures) {
  std::cout << '[';
  for (std::size_t index = 0U; index < failures.size(); ++index) {
    const auto& failure = failures[index];
    std::cout << (index == 0U ? "" : ", ")
              << "{\"footprint\": " << json_quote(failure.footprint.value())
              << ", \"pad\": " << json_quote(failure.pad.value())
              << ", \"net\": " << json_quote(failure.net.value()) << ", \"position_nm\": ["
              << failure.position.x.value() << ", " << failure.position.y.value()
              << "], \"nominal_center_blockers\": ";
    std::cout << '[';
    for (std::size_t blocker_index = 0U; blocker_index < failure.nominal_center_blockers.size();
         ++blocker_index) {
      std::cout << (blocker_index == 0U ? "" : ", ")
                << json_quote(failure.nominal_center_blockers[blocker_index].value());
    }
    std::cout << "], \"neck_down_center_blockers\": [";
    for (std::size_t blocker_index = 0U; blocker_index < failure.neck_down_center_blockers.size();
         ++blocker_index) {
      std::cout << (blocker_index == 0U ? "" : ", ")
                << json_quote(failure.neck_down_center_blockers[blocker_index].value());
    }
    std::cout << "], \"nominal_center_board_blocked\": "
              << (failure.nominal_center_board_blocked ? "true" : "false")
              << ", \"neck_down_center_board_blocked\": "
              << (failure.neck_down_center_board_blocked ? "true" : "false") << '}';
  }
  std::cout << ']';
}

[[nodiscard]] std::string_view
routing_connection_failure_reason_name(flux::RoutingConnectionFailureReason reason) {
  switch (reason) {
  case flux::RoutingConnectionFailureReason::no_access_nodes:
    return "no_access_nodes";
  case flux::RoutingConnectionFailureReason::search_frontier_exhausted:
    return "search_frontier_exhausted";
  case flux::RoutingConnectionFailureReason::expansion_limit:
    return "expansion_limit";
  case flux::RoutingConnectionFailureReason::unmatched_multisource_terminal:
    return "unmatched_multisource_terminal";
  }
  return "unknown";
}

void print_routing_terminal_evidence(const std::vector<flux::RoutingTerminalEvidence>& terminals) {
  std::cout << '[';
  for (std::size_t index = 0U; index < terminals.size(); ++index) {
    const auto& terminal = terminals[index];
    std::cout << (index == 0U ? "" : ", ")
              << "{\"footprint\": " << json_quote(terminal.footprint.value())
              << ", \"pad\": " << json_quote(terminal.pad.value()) << ", \"position_nm\": ["
              << terminal.position.x.value() << ", " << terminal.position.y.value() << "]}";
  }
  std::cout << ']';
}

void print_routing_connection_failures(
    const std::vector<flux::RoutingConnectionFailure>& failures) {
  std::cout << '[';
  for (std::size_t index = 0U; index < failures.size(); ++index) {
    const auto& failure = failures[index];
    std::cout << (index == 0U ? "" : ", ") << "{\"net\": " << json_quote(failure.net.value())
              << ", \"completed_connections\": " << failure.completed_connections
              << ", \"multisource\": " << (failure.multisource ? "true" : "false")
              << ", \"direct_edge_fallback_attempted\": "
              << (failure.direct_edge_fallback_attempted ? "true" : "false") << ", \"reason\": "
              << json_quote(routing_connection_failure_reason_name(failure.reason))
              << ", \"expansions\": " << failure.expansions
              << ", \"topology_rejected_edges\": " << failure.topology_rejected_edges
              << ", \"attempted_terminals\": ";
    print_routing_terminal_evidence(failure.attempted_terminals);
    std::cout << ", \"connected_terminals\": ";
    print_routing_terminal_evidence(failure.connected_terminals);
    std::cout << '}';
  }
  std::cout << ']';
}

void print_hard_capacity_pass_summaries(
    const std::vector<flux::RoutingHardCapacityPass>& summaries) {
  std::cout << '[';
  for (std::size_t index = 0U; index < summaries.size(); ++index) {
    const auto& summary = summaries[index];
    std::cout << (index == 0U ? "" : ", ") << "{\"iteration\": " << summary.iteration
              << ", \"attempted_order\": ";
    print_net_ids(summary.attempted_order);
    std::cout << ", \"unrouted_nets\": ";
    print_net_ids(summary.unrouted_nets);
    std::cout << ", \"connection_failures\": ";
    print_routing_connection_failures(summary.connection_failures);
    std::cout << '}';
  }
  std::cout << ']';
}

void print_hard_capacity_net_failures(
    const std::vector<flux::RoutingHardCapacityNetFailure>& failures) {
  std::cout << '[';
  for (std::size_t index = 0U; index < failures.size(); ++index) {
    const auto& failure = failures[index];
    std::cout << (index == 0U ? "" : ", ") << "{\"net\": " << json_quote(failure.net.value())
              << ", \"failed_passes\": " << failure.failed_passes << '}';
  }
  std::cout << ']';
}

void print_hard_capacity_blockers(const std::vector<flux::RoutingHardCapacityBlocker>& blockers) {
  std::cout << '[';
  for (std::size_t index = 0U; index < blockers.size(); ++index) {
    const auto& blocker = blockers[index];
    std::cout << (index == 0U ? "" : ", ")
              << "{\"failed_net\": " << json_quote(blocker.failed_net.value())
              << ", \"blocking_net\": " << json_quote(blocker.blocking_net.value())
              << ", \"rejected_transitions\": " << blocker.rejected_transitions << '}';
  }
  std::cout << ']';
}

void print_hard_capacity_ripup_records(
    const std::vector<flux::RoutingHardCapacityRipupAttempt>& attempts) {
  std::cout << '[';
  for (std::size_t index = 0U; index < attempts.size(); ++index) {
    const auto& attempt = attempts[index];
    std::cout << (index == 0U ? "" : ", ") << "{\"iteration\": " << attempt.iteration
              << ", \"failed_net\": " << json_quote(attempt.failed_net.value())
              << ", \"victims\": ";
    print_net_ids(attempt.victims);
    std::cout << ", \"retry_completed\": " << (attempt.retry_completed ? "true" : "false")
              << ", \"retained_at_pass_end\": " << (attempt.retained_at_pass_end ? "true" : "false")
              << ", \"rolled_back_after_failed_retry\": "
              << (attempt.rolled_back_after_failed_retry ? "true" : "false") << '}';
  }
  std::cout << ']';
}

void print_routing_net_pressure(const std::vector<flux::RoutingNetPressure>& pressure) {
  std::cout << '[';
  for (std::size_t index = 0U; index < pressure.size(); ++index) {
    const auto& item = pressure[index];
    std::cout << (index == 0U ? "" : ", ") << "{\"net\": " << json_quote(item.net.value())
              << ", \"legacy_congested_resources\": " << item.legacy_congested_resources
              << ", \"sampled_overflow\": " << item.sampled_overflow
              << ", \"clearance_shadow_overflow\": " << item.clearance_shadow_overflow
              << ", \"corridor_overflow\": " << item.corridor_overflow
              << ", \"total_physical_overflow_nm\": " << item.total_physical_overflow_nm
              << ", \"peak_physical_overflow_nm\": " << item.peak_physical_overflow_nm
              << ", \"congested_physical_resources\": " << item.congested_physical_resources << '}';
  }
  std::cout << ']';
}

void print_congested_voronoi_resource_usage(
    const std::vector<flux::RoutingVoronoiResourceUsage>& resources) {
  std::cout << '[';
  std::size_t emitted = 0U;
  for (const auto& resource : resources) {
    if (!resource.congested()) {
      continue;
    }
    std::cout << (emitted++ == 0U ? "" : ", ") << "{\"id\": " << resource.id
              << ", \"layer\": " << json_quote(resource.layer.value()) << ", \"start_nm\": ["
              << resource.start.x.value() << ", " << resource.start.y.value() << "], \"end_nm\": ["
              << resource.end.x.value() << ", " << resource.end.y.value()
              << "], \"channel_width_nm\": " << resource.channel_width.value()
              << ", \"demand_nm\": " << resource.demand_nm
              << ", \"capacity_nm\": " << resource.capacity_nm
              << ", \"overflow_nm\": " << resource.demand_nm - resource.capacity_nm
              << ", \"conservative_domain\": " << (resource.conservative_domain ? "true" : "false")
              << ", \"curved_bisector_chord\": "
              << (resource.curved_bisector_chord ? "true" : "false") << ", \"nets\": ";
    print_net_ids(resource.nets);
    std::cout << '}';
  }
  std::cout << ']';
}

[[nodiscard]] std::string_view routing_resource_kind_name(flux::RoutingResourceKind kind) {
  switch (kind) {
  case flux::RoutingResourceKind::node:
    return "node";
  case flux::RoutingResourceKind::planar_edge:
    return "planar_edge";
  case flux::RoutingResourceKind::via_edge:
    return "via_edge";
  case flux::RoutingResourceKind::medial_edge:
    return "medial_edge";
  }
  return "unknown";
}

void print_congested_sampled_resource_usage(
    const std::vector<flux::RoutingResourceUsage>& resources) {
  std::cout << '[';
  std::size_t emitted = 0U;
  for (const auto& resource : resources) {
    if (!resource.congested()) {
      continue;
    }
    std::cout << (emitted++ == 0U ? "" : ", ")
              << "{\"kind\": " << json_quote(routing_resource_kind_name(resource.kind))
              << ", \"first\": {\"layer\": " << json_quote(resource.first.layer.value())
              << ", \"point_nm\": [" << resource.first.position.x.value() << ", "
              << resource.first.position.y.value()
              << "]}, \"second\": {\"layer\": " << json_quote(resource.second.layer.value())
              << ", \"point_nm\": [" << resource.second.position.x.value() << ", "
              << resource.second.position.y.value() << "]}, \"demand\": " << resource.demand
              << ", \"capacity\": " << resource.capacity << ", \"nets\": ";
    print_net_ids(resource.nets);
    std::cout << '}';
  }
  std::cout << ']';
}

void print_physical_obstacle_impacts(
    const std::vector<flux::RoutingPhysicalObstacleImpact>& impacts) {
  std::cout << '[';
  for (std::size_t index = 0U; index < impacts.size(); ++index) {
    const auto& impact = impacts[index];
    std::cout << (index == 0U ? "" : ", ") << "{\"resource_id\": " << impact.resource_id
              << ", \"obstacle\": " << json_quote(impact.obstacle.value())
              << ", \"layer\": " << json_quote(impact.layer.value()) << ", \"start_nm\": ["
              << impact.start.x.value() << ", " << impact.start.y.value() << "], \"end_nm\": ["
              << impact.end.x.value() << ", " << impact.end.y.value()
              << "], \"overflow_nm\": " << impact.overflow_nm << ", \"routed_nets\": ";
    print_net_ids(impact.routed_nets);
    std::cout << '}';
  }
  std::cout << ']';
}

void print_footprint_pressure(const std::vector<flux::IntegratedFootprintPressure>& pressure) {
  std::cout << '[';
  for (std::size_t index = 0U; index < pressure.size(); ++index) {
    const auto& item = pressure[index];
    std::cout << (index == 0U ? "" : ", ")
              << "{\"footprint\": " << json_quote(item.footprint.value())
              << ", \"bounds_congested_channel\": "
              << (item.bounds_congested_channel ? "true" : "false")
              << ", \"blocks_route\": " << (item.blocks_route ? "true" : "false")
              << ", \"terminates_affected_net\": "
              << (item.terminates_affected_net ? "true" : "false")
              << ", \"legacy_pressure\": " << item.legacy_pressure
              << ", \"physical_pressure_nm\": " << item.physical_pressure_nm
              << ", \"sampled_pressure\": " << item.sampled_pressure
              << ", \"corridor_pressure\": " << item.corridor_pressure
              << ", \"distinct_connected_nets\": " << item.distinct_connected_nets << '}';
  }
  std::cout << ']';
}

[[nodiscard]] Board make_demo_board(bool invalid, bool routed = true) {
  const LayerId front{"F.Cu"};
  const LayerId back{"B.Cu"};
  const NetId signal{"N1"};

  Board board;
  board.source_revision = "built-in-demo-v1";
  board.outline = Polygon{{mm(0.0, 0.0), mm(100.0, 0.0), mm(100.0, 80.0), mm(0.0, 80.0)}};
  board.layers = {{front, "F.Cu", LayerKind::copper, 0}, {back, "B.Cu", LayerKind::copper, 1}};
  board.nets = {{signal, "SIGNAL", 0}};

  Footprint left;
  left.id = ObjectId{"fp-left"};
  left.reference = "J1";
  left.position = mm(20.0, 40.0);
  left.rotation = Microdegrees::from_degrees(0.0);
  left.courtyard = rectangle(5.0, 4.0);
  left.pads = {{ObjectId{"pad-left"}, signal, mm(0.0, 0.0), {front}}};

  Footprint right;
  right.id = ObjectId{"fp-right"};
  right.reference = "J2";
  right.position = invalid ? mm(24.0, 40.0) : mm(80.0, 40.0);
  right.rotation = Microdegrees::from_degrees(180.0);
  right.courtyard = rectangle(5.0, 4.0);
  right.pads = {{ObjectId{"pad-right"}, signal, mm(0.0, 0.0), {front}}};

  board.footprints = {left, right};
  if (routed) {
    board.tracks = {{ObjectId{"track-1"}, signal, front, left.position, right.position,
                     Nanometres::from_mm(0.25), false}};
  }
  return board;
}

void print_result(const flux::EvaluationResult& result) {
  const auto& objective = result.objective;
  std::cout << "{\n"
            << "  \"valid\": " << (result.validation.ok() ? "true" : "false") << ",\n"
            << "  \"validation_errors\": " << result.validation.error_count() << ",\n"
            << "  \"objective\": {\n"
            << "    \"hard_placement_violations\": " << objective.hard_placement_violations << ",\n"
            << "    \"routing_rule_violations\": " << objective.routing_rule_violations << ",\n"
            << "    \"open_connections\": " << objective.open_connections << ",\n"
            << "    \"unrouted_nets\": " << objective.unrouted_nets << ",\n"
            << "    \"via_count\": " << objective.via_count << ",\n"
            << "    \"routed_length_nm\": " << objective.routed_length_nm << "\n"
            << "  },\n"
            << "  \"issues\": [";
  for (std::size_t i = 0; i < result.validation.issues.size(); ++i) {
    if (i != 0U) {
      std::cout << ',';
    }
    std::cout << "\n    {\"code\": \"" << result.validation.issues[i].code << "\"}";
  }
  if (!result.validation.issues.empty()) {
    std::cout << '\n';
  }
  std::cout << "  ]\n}\n";
}

[[nodiscard]] std::string_view stop_reason_name(flux::GlobalRouterStopReason reason) {
  switch (reason) {
  case flux::GlobalRouterStopReason::complete:
    return "complete";
  case flux::GlobalRouterStopReason::stagnated:
    return "stagnated";
  case flux::GlobalRouterStopReason::iteration_limit:
    return "iteration_limit";
  case flux::GlobalRouterStopReason::invalid_input:
    return "invalid_input";
  case flux::GlobalRouterStopReason::cancelled:
    return "cancelled";
  }
  return "unknown";
}

[[nodiscard]] std::string_view
corridor_orientation_name(flux::RoutingCorridorOrientation orientation) {
  return orientation == flux::RoutingCorridorOrientation::horizontal ? "horizontal" : "vertical";
}

[[nodiscard]] std::string_view segment_width_kind_name(flux::GlobalRouteSegmentWidthKind kind) {
  switch (kind) {
  case flux::GlobalRouteSegmentWidthKind::nominal:
    return "nominal";
  case flux::GlobalRouteSegmentWidthKind::terminal_neck_down:
    return "terminal_neck_down";
  case flux::GlobalRouteSegmentWidthKind::medial_neck_down:
    return "medial_neck_down";
  case flux::GlobalRouteSegmentWidthKind::corridor_neck_down:
    return "corridor_neck_down";
  }
  return "unknown";
}

[[nodiscard]] std::string_view iteration_zero_wave_name(flux::IterationZeroRoutingWave wave) {
  switch (wave) {
  case flux::IterationZeroRoutingWave::trivial:
    return "trivial";
  case flux::IterationZeroRoutingWave::constrained:
    return "constrained";
  case flux::IterationZeroRoutingWave::ordinary:
    return "ordinary";
  case flux::IterationZeroRoutingWave::flexible:
    return "flexible";
  }
  return "unknown";
}

[[nodiscard]] std::string_view stop_reason_name(flux::DetailedRouterStopReason reason) {
  switch (reason) {
  case flux::DetailedRouterStopReason::complete:
    return "complete";
  case flux::DetailedRouterStopReason::invalid_input:
    return "invalid_input";
  case flux::DetailedRouterStopReason::incomplete_global_route:
    return "incomplete_global_route";
  case flux::DetailedRouterStopReason::invalid_path:
    return "invalid_path";
  case flux::DetailedRouterStopReason::structural_violations:
    return "structural_violations";
  case flux::DetailedRouterStopReason::connectivity_incomplete:
    return "connectivity_incomplete";
  case flux::DetailedRouterStopReason::copper_violations:
    return "copper_violations";
  case flux::DetailedRouterStopReason::zone_refill_required:
    return "zone_refill_required";
  case flux::DetailedRouterStopReason::cancelled:
    return "cancelled";
  }
  return "unknown";
}

[[nodiscard]] std::string_view
checkpoint_objective_name(flux::GlobalRouteCheckpointObjective objective) {
  switch (objective) {
  case flux::GlobalRouteCheckpointObjective::aggregate_overflow:
    return "aggregate_overflow";
  case flux::GlobalRouteCheckpointObjective::dimensioned_physical_overflow:
    return "dimensioned_physical_overflow";
  }
  return "unknown";
}

struct ResidualConflictSummary {
  std::size_t peak_nets{0};
  std::size_t peak_sampled_overflow{0};
  std::size_t peak_clearance_shadow_overflow{0};
  std::size_t topology_preserved_groups{0};
  std::size_t single_layer_groups{0};
};

[[nodiscard]] ResidualConflictSummary
summarize_residual_conflicts(const flux::GlobalRouteResult& result) {
  ResidualConflictSummary summary;
  for (const auto& group : result.residual_conflict_groups) {
    summary.peak_nets = std::max(summary.peak_nets, group.nets.size());
    summary.peak_sampled_overflow = std::max(summary.peak_sampled_overflow, group.sampled_overflow);
    summary.peak_clearance_shadow_overflow =
        std::max(summary.peak_clearance_shadow_overflow, group.clearance_shadow_overflow);
    summary.topology_preserved_groups +=
        group.all_nets_have_preserved_geometric_corridors ? 1U : 0U;
    summary.single_layer_groups += group.multilayer_path_count == 0U ? 1U : 0U;
  }
  return summary;
}

void print_route_result(const flux::GlobalRouteResult& result,
                        const flux::RegionalRouteResult* regional = nullptr,
                        const flux::RouterTelemetryStatistics* telemetry = nullptr,
                        const flux::RoutingPlanResult* routing_plan = nullptr) {
  std::size_t point_count = 0;
  std::size_t geometric_corridor_runs = 0U;
  std::size_t geometric_corridor_cells = 0U;
  std::size_t preserved_geometric_corridor_runs = 0U;
  std::size_t geometric_corridor_topology_signatures = 0U;
  std::size_t geometric_corridor_topology_gates = 0U;
  std::size_t peak_geometric_lane_capacity = 0U;
  const auto attributed_clearance_shadow_conflicts = static_cast<std::size_t>(std::count_if(
      result.clearance_shadow_conflicts.begin(), result.clearance_shadow_conflicts.end(),
      [](const auto& conflict) {
        return conflict.shadow_owner_first.has_value() && conflict.shadow_owner_second.has_value();
      }));
  const auto residual_summary = summarize_residual_conflicts(result);
  for (const auto& path : result.paths) {
    point_count += path.points.size();
    geometric_corridor_runs += path.geometric_corridors.size();
    for (const auto& corridor : path.geometric_corridors) {
      geometric_corridor_cells += corridor.topology_cells.size();
      preserved_geometric_corridor_runs += corridor.selected_path_preserves_topology ? 1U : 0U;
      geometric_corridor_topology_signatures += corridor.topology_signature.valid ? 1U : 0U;
      geometric_corridor_topology_gates += corridor.topology_signature.gates.size();
      peak_geometric_lane_capacity = std::max(peak_geometric_lane_capacity, corridor.lane_capacity);
    }
  }
  std::cout
      << "{\n"
      << "  \"complete\": " << (result.complete() ? "true" : "false") << ",\n"
      << "  \"stop_reason\": \"" << stop_reason_name(result.stop_reason) << "\",\n"
      << "  \"iterations\": " << result.iterations << ",\n"
      << "  \"selected_iteration\": " << result.selected_iteration << ",\n"
      << "  \"routability_history_entries\": " << result.routability_history_entries << ",\n"
      << "  \"routability_history_edges\": " << result.routability_history_edges << ",\n"
      << "  \"routability_history_pressure\": " << result.routability_history_pressure << ",\n"
      << "  \"routability_history_budget_exhausted\": "
      << (result.routability_history_budget_exhausted ? "true" : "false") << ",\n"
      << "  \"paths\": " << result.paths.size() << ",\n"
      << "  \"path_points\": " << point_count << ",\n"
      << "  \"geometric_corridor_runs\": " << geometric_corridor_runs << ",\n"
      << "  \"geometric_corridor_cells\": " << geometric_corridor_cells << ",\n"
      << "  \"preserved_geometric_corridor_runs\": " << preserved_geometric_corridor_runs << ",\n"
      << "  \"geometric_corridor_topology_signatures\": " << geometric_corridor_topology_signatures
      << ",\n"
      << "  \"geometric_corridor_topology_gates\": " << geometric_corridor_topology_gates << ",\n"
      << "  \"peak_geometric_lane_capacity\": " << peak_geometric_lane_capacity << ",\n"
      << "  \"residual_conflict_groups\": " << result.residual_conflict_groups.size() << ",\n"
      << "  \"peak_residual_conflict_group_nets\": " << residual_summary.peak_nets << ",\n"
      << "  \"peak_residual_group_sampled_overflow\": " << residual_summary.peak_sampled_overflow
      << ",\n"
      << "  \"peak_residual_group_clearance_shadow_overflow\": "
      << residual_summary.peak_clearance_shadow_overflow << ",\n"
      << "  \"topology_preserved_residual_conflict_groups\": "
      << residual_summary.topology_preserved_groups << ",\n"
      << "  \"single_layer_residual_conflict_groups\": " << residual_summary.single_layer_groups
      << ",\n"
      << "  \"clearance_shadow_conflicts\": " << result.clearance_shadow_conflicts.size() << ",\n"
      << "  \"clearance_shadow_owner_attributed_conflicts\": "
      << attributed_clearance_shadow_conflicts << ",\n"
      << "  \"clearance_shadow_overflow\": " << result.clearance_shadow_overflow << ",\n"
      << "  \"clearance_shadow_history_increment\": " << result.clearance_shadow_history_increment
      << ",\n"
      << "  \"peak_clearance_shadow_history_increment\": "
      << result.peak_clearance_shadow_history_increment << ",\n"
      << "  \"clearance_shadow_owner_history_phase_attempted\": "
      << (result.clearance_shadow_owner_history_phase_attempted ? "true" : "false") << ",\n"
      << "  \"clearance_shadow_owner_history_passes\": "
      << result.clearance_shadow_owner_history_passes << ",\n"
      << "  \"clearance_shadow_conflict_budget_exhausted\": "
      << (result.clearance_shadow_conflict_budget_exhausted ? "true" : "false") << ",\n"
      << "  \"unrouted_nets\": " << result.unrouted_nets.size() << ",\n"
      << "  \"configured_selected_net_ids\": ";
  print_net_ids(result.configured_selected_nets);
  std::cout << ",\n  \"configured_priority_net_ids\": ";
  print_net_ids(result.configured_priority_nets);
  std::cout
      << ",\n"
      << "  \"configured_grid_pitch_nm\": " << result.configured_grid_pitch.value() << ",\n"
      << "  \"configured_via_site_pitch_nm\": " << result.configured_via_site_pitch.value() << ",\n"
      << "  \"configured_via_site_phase_x\": " << result.configured_via_site_phase_x << ",\n"
      << "  \"configured_via_site_phase_y\": " << result.configured_via_site_phase_y << ",\n"
      << "  \"configured_diagonal_grid_edges\": "
      << (result.configured_diagonal_grid_edges ? "true" : "false") << ",\n"
      << "  \"configured_crossing_safe_diagonal_grid_edges\": "
      << (result.configured_crossing_safe_diagonal_grid_edges ? "true" : "false") << ",\n"
      << "  \"configured_sampled_route_clearance_reservations\": "
      << (result.configured_sampled_route_clearance_reservations ? "true" : "false") << ",\n"
      << "  \"configured_clearance_shadow_cost_quantum_nm\": "
      << result.configured_clearance_shadow_cost_quantum.value() << ",\n"
      << "  \"configured_capacity_reserve_per_mille\": "
      << result.configured_capacity_reserve_per_mille << ",\n"
      << "  \"configured_capacity_reserve_cost\": " << result.configured_capacity_reserve_cost
      << ",\n"
      << "  \"configured_single_lane_direct_voronoi_resources\": "
      << (result.configured_single_lane_direct_voronoi_resources ? "true" : "false") << ",\n"
      << "  \"configured_max_static_seed_candidates_per_connection\": "
      << result.configured_max_static_seed_candidates_per_connection << ",\n"
      << "  \"configured_max_static_seed_searches\": " << result.configured_max_static_seed_searches
      << ",\n"
      << "  \"configured_static_seed_rrt_connect_alternatives\": "
      << (result.configured_static_seed_rrt_connect_alternatives ? "true" : "false") << ",\n"
      << "  \"configured_max_static_seed_rrt_connect_candidates\": "
      << result.configured_max_static_seed_rrt_connect_candidates << ",\n"
      << "  \"configured_max_static_seed_rrt_connect_iterations\": "
      << result.configured_max_static_seed_rrt_connect_iterations << ",\n"
      << "  \"configured_static_seed_hydraulic_mandatory\": true,\n"
      << "  \"configured_max_static_seed_hydraulic_candidates\": "
      << result.configured_max_static_seed_hydraulic_candidates << ",\n"
      << "  \"configured_static_seed_hydraulic_gpu_preferred\": "
      << (result.configured_static_seed_hydraulic_flow.prefer_gpu ? "true" : "false") << ",\n"
      << "  \"configured_max_static_seed_workers\": " << result.configured_max_static_seed_workers
      << ",\n"
      << "  \"configured_static_seed_alternative_edge_penalty\": "
      << result.configured_static_seed_alternative_edge_penalty << ",\n"
      << "  \"configured_static_seed_corridor_departure_penalty\": "
      << result.configured_static_seed_corridor_departure_penalty << ",\n"
      << "  \"configured_residual_corridor_demand_cost\": "
      << result.configured_residual_corridor_demand_cost << ",\n"
      << "  \"configured_critical_cut_cost\": " << result.configured_critical_cut_cost << ",\n"
      << "  \"configured_critical_cut_minimum_future_demand\": "
      << result.configured_critical_cut_minimum_future_demand << ",\n"
      << "  \"configured_river_following_cost\": " << result.configured_river_following_cost
      << ",\n"
      << "  \"river_following_edges\": " << result.river_following_edges << ",\n"
      << "  \"river_following_cost_discount\": " << result.river_following_cost_discount << ",\n"
      << "  \"configured_targeted_track_spreading_cost\": "
      << result.configured_targeted_track_spreading_cost << ",\n"
      << "  \"targeted_track_spreading_edges\": " << result.targeted_track_spreading_edges << ",\n"
      << "  \"targeted_track_spreading_penalty\": " << result.targeted_track_spreading_penalty
      << ",\n"
      << "  \"configured_former_route_memory_cost\": " << result.configured_former_route_memory_cost
      << ",\n"
      << "  \"former_route_memory_ripped_paths\": " << result.former_route_memory_ripped_paths
      << ",\n"
      << "  \"former_route_memory_edges\": " << result.former_route_memory_edges << ",\n"
      << "  \"former_route_memory_nodes\": " << result.former_route_memory_nodes << ",\n"
      << "  \"former_route_memory_penalty_events\": " << result.former_route_memory_penalty_events
      << ",\n"
      << "  \"former_route_memory_penalty\": " << result.former_route_memory_penalty << ",\n"
      << "  \"configured_terminal_escape_reservation_cost\": "
      << result.configured_terminal_escape_reservation_cost << ",\n"
      << "  \"configured_terminal_escape_proximity_cost\": "
      << result.configured_terminal_escape_proximity_cost << ",\n"
      << "  \"configured_max_terminal_escape_reservation_resources\": "
      << result.configured_max_terminal_escape_reservation_resources << ",\n"
      << "  \"terminal_escape_reservation_terminals\": "
      << result.terminal_escape_reservation_terminals << ",\n"
      << "  \"terminal_escape_reservation_candidates\": "
      << result.terminal_escape_reservation_candidates << ",\n"
      << "  \"terminal_escape_reservation_resources\": "
      << result.terminal_escape_reservation_resources << ",\n"
      << "  \"terminal_escape_reservation_withheld_candidates\": "
      << result.terminal_escape_reservation_withheld_candidates << ",\n"
      << "  \"terminal_escape_reservation_penalty_events\": "
      << result.terminal_escape_reservation_penalty_events << ",\n"
      << "  \"terminal_escape_reservation_penalty\": " << result.terminal_escape_reservation_penalty
      << ",\n"
      << "  \"terminal_escape_proximity_penalty_events\": "
      << result.terminal_escape_proximity_penalty_events << ",\n"
      << "  \"terminal_escape_proximity_penalty\": " << result.terminal_escape_proximity_penalty
      << ",\n"
      << "  \"residual_corridor_demand_nets\": " << result.residual_corridor_demand_nets << ",\n"
      << "  \"residual_corridor_demand_resources\": " << result.residual_corridor_demand_resources
      << ",\n"
      << "  \"residual_corridor_demand_penalty_events\": "
      << result.residual_corridor_demand_penalty_events << ",\n"
      << "  \"residual_corridor_demand_penalty\": " << result.residual_corridor_demand_penalty
      << ",\n"
      << "  \"capacity_reserve_penalty_events\": " << result.capacity_reserve_penalty_events
      << ",\n"
      << "  \"corridor_capacity_reserve_penalty_events\": "
      << result.corridor_capacity_reserve_penalty_events << ",\n"
      << "  \"physical_capacity_reserve_penalty_events\": "
      << result.physical_capacity_reserve_penalty_events << ",\n"
      << "  \"clearance_shadow_reserve_penalty_events\": "
      << result.clearance_shadow_reserve_penalty_events << ",\n"
      << "  \"capacity_reserve_penalty\": " << result.capacity_reserve_penalty << ",\n"
      << "  \"critical_cut_resources\": " << result.critical_cut_resources << ",\n"
      << "  \"critical_cut_reserved_demand\": " << result.critical_cut_reserved_demand << ",\n"
      << "  \"critical_cut_penalty_events\": " << result.critical_cut_penalty_events << ",\n"
      << "  \"critical_cut_penalty\": " << result.critical_cut_penalty << ",\n"
      << "  \"configured_navmesh_funnel_spline_seeds\": "
      << (result.configured_navmesh_funnel_spline_seeds ? "true" : "false") << ",\n"
      << "  \"configured_global_navmesh_cell_pitch_nm\": "
      << result.configured_global_navmesh_cell_pitch.value() << ",\n"
      << "  \"configured_global_spline_minimum_bend_radius_nm\": "
      << result.configured_global_spline_minimum_bend_radius.value() << ",\n"
      << "  \"static_seed_connections\": " << result.static_seed_connections << ",\n"
      << "  \"static_seed_tasks_dispatched\": " << result.static_seed_tasks_dispatched << ",\n"
      << "  \"static_seed_workers_used\": " << result.static_seed_workers_used << ",\n"
      << "  \"static_seed_searches\": " << result.static_seed_searches << ",\n"
      << "  \"static_seed_candidates\": " << result.static_seed_candidates << ",\n"
      << "  \"static_seed_search_expansions\": " << result.static_seed_search_expansions << ",\n"
      << "  \"static_seed_penalty_reruns\": " << result.static_seed_penalty_reruns << ",\n"
      << "  \"static_seed_deduplicated_candidates\": " << result.static_seed_deduplicated_candidates
      << ",\n"
      << "  \"static_seed_via_bearing_candidates\": " << result.static_seed_via_bearing_candidates
      << ",\n"
      << "  \"static_seed_direct_medial_candidates\": "
      << result.static_seed_direct_medial_candidates << ",\n"
      << "  \"static_seed_terminals_without_seed\": " << result.static_seed_terminals_without_seed
      << ",\n"
      << "  \"static_seed_distinct_layer_sequences\": "
      << result.static_seed_distinct_layer_sequences << ",\n"
      << "  \"static_seed_distinct_topology_signatures\": "
      << result.static_seed_distinct_topology_signatures << ",\n"
      << "  \"static_seed_corridor_nodes\": " << result.static_seed_corridor_nodes << ",\n"
      << "  \"static_seed_spline_samples\": " << result.static_seed_spline_samples << ",\n"
      << "  \"static_seed_navmesh_domains\": " << result.static_seed_navmesh_domains << ",\n"
      << "  \"static_seed_navmesh_routes\": " << result.static_seed_navmesh_routes << ",\n"
      << "  \"static_seed_navmesh_cells\": " << result.static_seed_navmesh_cells << ",\n"
      << "  \"static_seed_navmesh_refined_parents\": " << result.static_seed_navmesh_refined_parents
      << ",\n"
      << "  \"static_seed_navmesh_refined_leaves\": " << result.static_seed_navmesh_refined_leaves
      << ",\n"
      << "  \"static_seed_navmesh_rejected_leaves\": " << result.static_seed_navmesh_rejected_leaves
      << ",\n"
      << "  \"static_seed_navmesh_gpu_wavefront_routes\": "
      << result.static_seed_navmesh_gpu_wavefront_routes << ",\n"
      << "  \"static_seed_navmesh_gpu_wavefront_cpu_fallbacks\": "
      << result.static_seed_navmesh_gpu_wavefront_cpu_fallbacks << ",\n"
      << "  \"static_seed_navmesh_search_expansions\": "
      << result.static_seed_navmesh_search_expansions << ",\n"
      << "  \"static_seed_flood_fill_routes\": " << result.static_seed_flood_fill_routes << ",\n"
      << "  \"static_seed_flood_fill_cells_visited\": "
      << result.static_seed_flood_fill_cells_visited << ",\n"
      << "  \"static_seed_navmesh_failures\": " << result.static_seed_navmesh_failures << ",\n"
      << "  \"static_seed_rrt_connect_attempts\": " << result.static_seed_rrt_connect_attempts
      << ",\n"
      << "  \"static_seed_rrt_connect_accepted\": " << result.static_seed_rrt_connect_accepted
      << ",\n"
      << "  \"static_seed_rrt_connect_failures\": " << result.static_seed_rrt_connect_failures
      << ",\n"
      << "  \"static_seed_rrt_connect_iterations\": " << result.static_seed_rrt_connect_iterations
      << ",\n"
      << "  \"static_seed_hydraulic_attempts\": " << result.static_seed_hydraulic_attempts << ",\n"
      << "  \"static_seed_hydraulic_accepted\": " << result.static_seed_hydraulic_accepted << ",\n"
      << "  \"static_seed_hydraulic_failures\": " << result.static_seed_hydraulic_failures << ",\n"
      << "  \"static_seed_hydraulic_gpu_routes\": " << result.static_seed_hydraulic_gpu_routes
      << ",\n"
      << "  \"static_seed_hydraulic_gpu_clearance_cache_hits\": "
      << result.static_seed_hydraulic_gpu_clearance_cache_hits << ",\n"
      << "  \"static_seed_hydraulic_gpu_peak_batch_size\": "
      << result.static_seed_hydraulic_gpu_peak_batch_size << ",\n"
      << "  \"static_seed_hydraulic_gpu_buffer_reuses\": "
      << result.static_seed_hydraulic_gpu_buffer_reuses << ",\n"
      << "  \"static_seed_hydraulic_gpu_buffer_growths\": "
      << result.static_seed_hydraulic_gpu_buffer_growths << ",\n"
      << "  \"static_seed_hydraulic_cpu_fallbacks\": " << result.static_seed_hydraulic_cpu_fallbacks
      << ",\n"
      << "  \"static_seed_hydraulic_relaxation_passes\": "
      << result.static_seed_hydraulic_relaxation_passes << ",\n"
      << "  \"static_seed_hydraulic_droplet_steps\": " << result.static_seed_hydraulic_droplet_steps
      << ",\n"
      << "  \"static_seed_hydraulic_discovery_cells\": "
      << result.static_seed_hydraulic_discovery_cells << ",\n"
      << "  \"static_seed_hydraulic_exact_certified_chords\": "
      << result.static_seed_hydraulic_exact_certified_chords << ",\n"
      << "  \"static_seed_hydraulic_projection_failures\": "
      << result.static_seed_hydraulic_projection_failures << ",\n"
      << "  \"static_seed_navmesh_certificate_domains\": "
      << result.static_seed_navmesh_certificate_domains << ",\n"
      << "  \"static_seed_funnel_points\": " << result.static_seed_funnel_points << ",\n"
      << "  \"static_seed_trajectory_attempts\": " << result.static_seed_trajectory_attempts
      << ",\n"
      << "  \"static_seed_trajectory_accepted\": " << result.static_seed_trajectory_accepted
      << ",\n"
      << "  \"static_seed_trajectory_control_moves\": "
      << result.static_seed_trajectory_control_moves << ",\n"
      << "  \"static_seed_trajectory_candidate_checks\": "
      << result.static_seed_trajectory_candidate_checks << ",\n"
      << "  \"static_seed_trajectory_rejected\": " << result.static_seed_trajectory_rejected
      << ",\n"
      << "  \"negotiated_spline_guided_searches\": " << result.negotiated_spline_guided_searches
      << ",\n"
      << "  \"negotiated_spline_corridor_departures\": "
      << result.negotiated_spline_corridor_departures << ",\n"
      << "  \"static_seed_collision_envelopes\": " << result.static_seed_collision_envelopes
      << ",\n"
      << "  \"static_seed_control_shoves\": " << result.static_seed_control_shoves << ",\n"
      << "  \"static_seed_refits\": " << result.static_seed_refits << ",\n"
      << "  \"static_seed_refined_candidates\": " << result.static_seed_refined_candidates << ",\n"
      << "  \"static_seed_reprojection_searches\": " << result.static_seed_reprojection_searches
      << ",\n"
      << "  \"static_seed_reprojections_accepted\": " << result.static_seed_reprojections_accepted
      << ",\n"
      << "  \"static_seed_reprojections_reverted\": " << result.static_seed_reprojections_reverted
      << ",\n"
      << "  \"static_seed_corridor_departures\": " << result.static_seed_corridor_departures
      << ",\n"
      << "  \"static_seed_complete_trees\": " << result.static_seed_complete_trees << ",\n"
      << "  \"static_seed_partial_trees\": " << result.static_seed_partial_trees << ",\n"
      << "  \"static_seed_alternative_trees\": " << result.static_seed_alternative_trees << ",\n"
      << "  \"static_seed_tree_joins\": " << result.static_seed_tree_joins << ",\n"
      << "  \"static_seed_growth_reorderings\": " << result.static_seed_growth_reorderings << ",\n"
      << "  \"static_seed_joint_groups\": " << result.static_seed_joint_groups << ",\n"
      << "  \"static_seed_joint_group_candidates\": " << result.static_seed_joint_group_candidates
      << ",\n"
      << "  \"static_seed_joint_groups_withheld\": " << result.static_seed_joint_groups_withheld
      << ",\n"
      << "  \"static_seed_budget_exhausted\": "
      << (result.static_seed_budget_exhausted ? "true" : "false") << ",\n"
      << "  \"global_pns_proposal_attempts\": " << result.global_pns_proposal_attempts << ",\n"
      << "  \"global_pns_eligible_collisions\": " << result.global_pns_eligible_collisions << ",\n"
      << "  \"global_pns_ineligible_collisions\": " << result.global_pns_ineligible_collisions
      << ",\n"
      << "  \"global_pns_candidate_checks\": " << result.global_pns_candidate_checks << ",\n"
      << "  \"global_pns_local_splits\": " << result.global_pns_local_splits << ",\n"
      << "  \"global_pns_proposals_accepted\": " << result.global_pns_proposals_accepted << ",\n"
      << "  \"selected_shove_proposals\": " << result.selected_shove_proposals.size() << ",\n"
      << "  \"iteration_zero_paths\": " << result.iteration_zero_paths << ",\n"
      << "  \"iteration_zero_unrouted_nets\": " << result.iteration_zero_unrouted_nets << ",\n"
      << "  \"iteration_zero_total_overflow\": " << result.iteration_zero_total_overflow << ",\n"
      << "  \"iteration_zero_sampled_overflow\": " << result.iteration_zero_sampled_overflow
      << ",\n"
      << "  \"iteration_zero_corridor_overflow\": " << result.iteration_zero_corridor_overflow
      << ",\n"
      << "  \"iteration_zero_portfolios_evaluated\": " << result.iteration_zero_portfolios_evaluated
      << ",\n"
      << "  \"iteration_zero_candidate_index\": " << result.iteration_zero_candidate_index << ",\n"
      << "  \"iteration_zero_seed_history_increments\": "
      << result.iteration_zero_seed_history_increments << ",\n"
      << "  \"iteration_zero_selected\": " << (result.iteration_zero_selected ? "true" : "false")
      << ",\n"
      << "  \"iteration_zero_occupancy_aware_portfolio_selected\": "
      << (result.iteration_zero_occupancy_aware_portfolio_selected ? "true" : "false") << ",\n"
      << "  \"iteration_zero_staged_priority_portfolio_selected\": "
      << (result.iteration_zero_staged_priority_portfolio_selected ? "true" : "false") << ",\n"
      << "  \"iteration_zero_staged_priority_qualified\": "
      << (result.iteration_zero_staged_priority_qualified ? "true" : "false") << ",\n"
      << "  \"iteration_zero_baseline_unrouted_nets\": "
      << result.iteration_zero_baseline_unrouted_nets << ",\n"
      << "  \"iteration_zero_staged_unrouted_nets\": " << result.iteration_zero_staged_unrouted_nets
      << ",\n"
      << "  \"iteration_zero_baseline_total_overflow\": "
      << result.iteration_zero_baseline_total_overflow << ",\n"
      << "  \"iteration_zero_staged_total_overflow\": "
      << result.iteration_zero_staged_total_overflow << ",\n"
      << "  \"iteration_zero_baseline_clearance_shadow_conflicts\": "
      << result.iteration_zero_baseline_clearance_shadow_conflicts << ",\n"
      << "  \"iteration_zero_staged_clearance_shadow_conflicts\": "
      << result.iteration_zero_staged_clearance_shadow_conflicts << ",\n"
      << "  \"iteration_zero_trivial_groups\": " << result.iteration_zero_trivial_groups << ",\n"
      << "  \"iteration_zero_constrained_groups\": " << result.iteration_zero_constrained_groups
      << ",\n"
      << "  \"iteration_zero_ordinary_groups\": " << result.iteration_zero_ordinary_groups << ",\n"
      << "  \"iteration_zero_flexible_groups\": " << result.iteration_zero_flexible_groups << ",\n"
      << "  \"iteration_zero_seed_priority_evidence\": [";
  for (std::size_t index = 0U; index < result.iteration_zero_seed_priority_evidence.size();
       ++index) {
    const auto& evidence = result.iteration_zero_seed_priority_evidence[index];
    std::cout << (index == 0U ? "" : ",") << "\n    {\"group\": " << json_quote(evidence.group)
              << ", \"wave\": " << json_quote(iteration_zero_wave_name(evidence.wave))
              << ", \"nets\": ";
    print_net_ids(evidence.nets);
    std::cout << ", \"explicit_priority\": " << evidence.explicit_priority
              << ", \"minimum_terminal_portals\": " << evidence.minimum_terminal_portals
              << ", \"viable_tree_candidates\": " << evidence.viable_tree_candidates
              << ", \"viable_corridor_candidates\": " << evidence.viable_corridor_candidates
              << ", \"topology_alternatives\": " << evidence.topology_alternatives
              << ", \"layer_alternatives\": " << evidence.layer_alternatives
              << ", \"primary_via_transitions\": " << evidence.primary_via_transitions
              << ", \"viable_via_sites\": " << evidence.viable_via_sites
              << ", \"via_assignment_alternatives\": " << evidence.via_assignment_alternatives
              << ", \"via_site_contention\": " << evidence.via_site_contention
              << ", \"terminal_crossing_pressure\": " << evidence.terminal_crossing_pressure
              << ", \"estimated_interference\": " << evidence.estimated_interference
              << ", \"physical_envelope_nm\": " << evidence.physical_envelope_nm
              << ", \"primary_route_steps\": " << evidence.primary_route_steps
              << ", \"routing_freedom_score\": " << evidence.routing_freedom_score
              << ", \"touches_scarce_resource\": "
              << (evidence.touches_scarce_resource ? "true" : "false") << '}';
  }
  if (!result.iteration_zero_seed_priority_evidence.empty()) {
    std::cout << '\n';
  }
  std::cout << "  ],\n"
            << "  \"unrouted_net_ids\": ";
  print_net_ids(result.unrouted_nets);
  std::cout << ",\n"
            << "  \"connection_failures\": ";
  print_routing_connection_failures(result.connection_failures);
  std::cout << ",\n"
            << "  \"total_overflow\": " << result.total_overflow << ",\n"
            << "  \"sampled_overflow\": " << result.sampled_overflow << ",\n"
            << "  \"corridor_overflow\": " << result.corridor_overflow << ",\n"
            << "  \"total_cost\": " << result.total_cost << ",\n"
            << "  \"planned_via_transitions\": " << result.planned_via_transitions << ",\n"
            << "  \"reused_existing_via_transitions\": " << result.reused_existing_via_transitions
            << ",\n"
            << "  \"reused_component_pad_transitions\": " << result.reused_component_pad_transitions
            << ",\n"
            << "  \"via_congestion_cost_discount\": " << result.via_congestion_cost_discount
            << ",\n"
            << "  \"vertical_feature_reuse_cost_discount\": "
            << result.vertical_feature_reuse_cost_discount << ",\n"
            << "  \"configured_checkpoint_objective\": \""
            << checkpoint_objective_name(result.configured_checkpoint_objective) << "\",\n"
            << "  \"configured_physical_overflow_history\": "
            << (result.configured_scale_voronoi_history_by_overflow ? "true" : "false") << ",\n"
            << "  \"voronoi_history_quantum_nm\": " << result.voronoi_history_quantum_nm << ",\n"
            << "  \"total_voronoi_history_increment\": " << result.total_voronoi_history_increment
            << ",\n"
            << "  \"peak_voronoi_history_increment\": " << result.peak_voronoi_history_increment
            << ",\n"
            << "  \"configured_physical_envelope_ordering\": "
            << (result.configured_physical_envelope_ordering ? "true" : "false") << ",\n"
            << "  \"peak_net_routing_envelope_nm\": " << result.peak_net_routing_envelope_nm
            << ",\n"
            << "  \"peak_routing_group_envelope_nm\": " << result.peak_routing_group_envelope_nm
            << ",\n"
            << "  \"configured_present_congestion_multiplier_per_mille\": "
            << result.configured_present_congestion_multiplier_per_mille << ",\n"
            << "  \"configured_max_present_congestion_scale_per_mille\": "
            << result.configured_max_present_congestion_scale_per_mille << ",\n"
            << "  \"configured_present_congestion_escalation_start_iteration\": "
            << result.configured_present_congestion_escalation_start_iteration << ",\n"
            << "  \"configured_hard_capacity_start_iteration\": "
            << result.configured_hard_capacity_start_iteration << ",\n"
            << "  \"configured_max_hard_capacity_ripup_attempts_per_pass\": "
            << result.configured_max_hard_capacity_ripup_attempts_per_pass << ",\n"
            << "  \"configured_max_hard_capacity_ripup_nets_per_attempt\": "
            << result.configured_max_hard_capacity_ripup_nets_per_attempt << ",\n"
            << "  \"configured_max_hard_capacity_conflict_set_states_per_attempt\": "
            << result.configured_max_hard_capacity_conflict_set_states_per_attempt << ",\n"
            << "  \"hard_capacity_passes\": " << result.hard_capacity_passes << ",\n"
            << "  \"post_negotiation_cone_repair_attempts\": "
            << result.post_negotiation_cone_repair_attempts << ",\n"
            << "  \"post_negotiation_cone_repairs_accepted\": "
            << result.post_negotiation_cone_repairs_accepted << ",\n"
            << "  \"post_negotiation_cone_repair_nets_attempted\": "
            << result.post_negotiation_cone_repair_nets_attempted << ",\n"
            << "  \"hard_sampled_capacity_rejected_transitions\": "
            << result.hard_sampled_capacity_rejected_transitions << ",\n"
            << "  \"hard_corridor_capacity_rejected_transitions\": "
            << result.hard_corridor_capacity_rejected_transitions << ",\n"
            << "  \"hard_voronoi_capacity_rejected_transitions\": "
            << result.hard_voronoi_capacity_rejected_transitions << ",\n"
            << "  \"hard_capacity_diversified_passes\": " << result.hard_capacity_diversified_passes
            << ",\n"
            << "  \"hard_capacity_diversified_net_positions\": "
            << result.hard_capacity_diversified_net_positions
            << ",\n  \"hard_capacity_pass_summaries\": ";
  print_hard_capacity_pass_summaries(result.hard_capacity_pass_summaries);
  std::cout << ",\n  \"hard_capacity_net_failures\": ";
  print_hard_capacity_net_failures(result.hard_capacity_net_failures);
  std::cout << ",\n  \"hard_capacity_blockers\": ";
  print_hard_capacity_blockers(result.hard_capacity_blockers);
  std::cout
      << ",\n"
      << "  \"hard_capacity_ripup_attempts\": " << result.hard_capacity_ripup_attempts << ",\n"
      << "  \"hard_capacity_ripped_nets\": " << result.hard_capacity_ripped_nets << ",\n"
      << "  \"hard_capacity_ripup_recoveries\": " << result.hard_capacity_ripup_recoveries << ",\n"
      << "  \"hard_capacity_ripup_retained_recoveries\": "
      << result.hard_capacity_ripup_retained_recoveries << ",\n"
      << "  \"hard_capacity_conflict_set_states\": " << result.hard_capacity_conflict_set_states
      << ",\n"
      << "  \"hard_capacity_joint_ripup_attempts\": " << result.hard_capacity_joint_ripup_attempts
      << ",\n"
      << "  \"hard_capacity_conflict_sets_withheld\": "
      << result.hard_capacity_conflict_sets_withheld << ",\n"
      << "  \"configured_max_soft_capacity_cut_probe_expansions\": "
      << result.configured_max_soft_capacity_cut_probe_expansions << ",\n"
      << "  \"hard_capacity_soft_cut_probes\": " << result.hard_capacity_soft_cut_probes << ",\n"
      << "  \"hard_capacity_soft_cut_routes\": " << result.hard_capacity_soft_cut_routes << ",\n"
      << "  \"hard_capacity_soft_cut_blockers\": " << result.hard_capacity_soft_cut_blockers
      << ",\n"
      << "  \"configured_rollback_failed_hard_capacity_ripup\": "
      << (result.configured_rollback_failed_hard_capacity_ripup ? "true" : "false") << ",\n"
      << "  \"hard_capacity_failed_retry_rollbacks\": "
      << result.hard_capacity_failed_retry_rollbacks << ",\n"
      << "  \"configured_defer_hard_capacity_ripup_victims_to_pass_tail\": "
      << (result.configured_defer_hard_capacity_ripup_victims_to_pass_tail ? "true" : "false")
      << ",\n"
      << "  \"hard_capacity_tail_deferred_victims\": " << result.hard_capacity_tail_deferred_victims
      << ",\n"
      << "  \"configured_protect_recovered_hard_capacity_ripup_nets\": "
      << (result.configured_protect_recovered_hard_capacity_ripup_nets ? "true" : "false") << ",\n"
      << "  \"hard_capacity_protected_recoveries\": " << result.hard_capacity_protected_recoveries
      << ",\n"
      << "  \"hard_capacity_protected_victim_rejections\": "
      << result.hard_capacity_protected_victim_rejections << ",\n"
      << "  \"configured_max_hard_capacity_blocker_reuse_invalidations_per_failed_net\": "
      << result.configured_max_hard_capacity_blocker_reuse_invalidations_per_failed_net << ",\n"
      << "  \"hard_capacity_blocker_reuse_invalidations\": "
      << result.hard_capacity_blocker_reuse_invalidations
      << ",\n  \"hard_capacity_ripup_records\": ";
  print_hard_capacity_ripup_records(result.hard_capacity_ripup_records);
  std::cout << ",\n"
            << "  \"selected_present_congestion_scale_per_mille\": "
            << result.selected_present_congestion_scale_per_mille << ",\n"
            << "  \"selected_effective_present_congestion_cost\": "
            << result.selected_effective_present_congestion_cost << ",\n"
            << "  \"selected_effective_corridor_present_cost\": "
            << result.selected_effective_corridor_present_cost << ",\n"
            << "  \"selected_effective_voronoi_resource_present_cost\": "
            << result.selected_effective_voronoi_resource_present_cost << ",\n"
            << "  \"selected_effective_capacity_reserve_cost\": "
            << result.selected_effective_capacity_reserve_cost << ",\n"
            << "  \"configured_max_expansions_per_connection\": "
            << result.configured_max_expansions_per_connection << ",\n"
            << "  \"configured_expansion_retry_heuristic_weight_per_mille\": "
            << result.configured_expansion_retry_heuristic_weight_per_mille << ",\n"
            << "  \"configured_reuse_weighted_search_frontier\": "
            << (result.configured_reuse_weighted_search_frontier ? "true" : "false") << ",\n"
            << "  \"path_search_invocations\": " << result.path_search_invocations << ",\n"
            << "  \"path_search_expansions\": " << result.path_search_expansions << ",\n"
            << "  \"expansion_limited_searches\": " << result.expansion_limited_searches << ",\n"
            << "  \"weighted_search_retries\": " << result.weighted_search_retries << ",\n"
            << "  \"weighted_search_recoveries\": " << result.weighted_search_recoveries << ",\n"
            << "  \"weighted_frontier_continuations\": " << result.weighted_frontier_continuations
            << ",\n"
            << "  \"weighted_frontier_recoveries\": " << result.weighted_frontier_recoveries
            << ",\n"
            << "  \"weighted_frontier_states_reused\": " << result.weighted_frontier_states_reused
            << ",\n"
            << "  \"weighted_frontier_continuation_limits\": "
            << result.weighted_frontier_continuation_limits << ",\n"
            << "  \"blocked_search_edges\": " << result.blocked_search_edges << ",\n"
            << "  \"board_edge_rejections\": " << result.board_edge_rejections << ",\n"
            << "  \"zone_hole_broad_phase_rejections\": " << result.zone_hole_broad_phase_rejections
            << ",\n"
            << "  \"zone_hole_exact_checks\": " << result.zone_hole_exact_checks << ",\n"
            << "  \"failed_pin_escapes\": " << result.failed_pin_escapes << ",\n"
            << "  \"pin_escape_failures\": ";
  print_pin_escape_failures(result.pin_escape_failures);
  std::cout
      << ",\n"
      << "  \"pin_access_nodes\": " << result.pin_access_nodes << ",\n"
      << "  \"configured_local_pin_escape_pitch_nm\": "
      << result.configured_local_pin_escape_pitch.value() << ",\n"
      << "  \"configured_max_local_pin_escape_expansions_per_terminal\": "
      << result.configured_max_local_pin_escape_expansions_per_terminal << ",\n"
      << "  \"local_pin_escape_paths\": " << result.local_pin_escape_paths << ",\n"
      << "  \"local_pin_escape_expansions\": " << result.local_pin_escape_expansions << ",\n"
      << "  \"terminals_with_local_pin_escape\": " << result.terminals_with_local_pin_escape
      << ",\n"
      << "  \"same_net_terminal_fanout_paths\": " << result.same_net_terminal_fanout_paths << ",\n"
      << "  \"terminals_with_same_net_fanout\": " << result.terminals_with_same_net_fanout << ",\n"
      << "  \"configured_contended_terminal_fanout\": "
      << (result.configured_contended_terminal_fanout ? "true" : "false") << ",\n"
      << "  \"contended_terminals\": " << result.contended_terminals << ",\n"
      << "  \"contention_triggered_local_paths\": " << result.contention_triggered_local_paths
      << ",\n"
      << "  \"adaptive_pin_access_nodes\": " << result.adaptive_pin_access_nodes << ",\n"
      << "  \"terminals_with_adaptive_pin_access\": " << result.terminals_with_adaptive_pin_access
      << ",\n"
      << "  \"configured_diversify_pin_access_rings\": "
      << (result.configured_diversify_pin_access_rings ? "true" : "false") << ",\n"
      << "  \"diversified_pin_access_nodes\": " << result.diversified_pin_access_nodes << ",\n"
      << "  \"terminals_with_diversified_pin_access\": "
      << result.terminals_with_diversified_pin_access << ",\n"
      << "  \"additional_successful_pin_access_rings\": "
      << result.additional_successful_pin_access_rings << ",\n"
      << "  \"isolated_pin_access_nodes_rejected\": " << result.isolated_pin_access_nodes_rejected
      << ",\n"
      << "  \"terminals_with_isolated_pin_access_rejections\": "
      << result.terminals_with_isolated_pin_access_rejections << ",\n"
      << "  \"terminal_neck_down_access_nodes\": " << result.terminal_neck_down_access_nodes
      << ",\n"
      << "  \"terminals_with_neck_down_access\": " << result.terminals_with_neck_down_access
      << ",\n"
      << "  \"coupled_terminals_ineligible_for_neck_down\": "
      << result.coupled_terminals_ineligible_for_neck_down << ",\n"
      << "  \"configured_terminal_neck_down_width_nm\": "
      << result.configured_terminal_neck_down_width.value() << ",\n"
      << "  \"configured_max_terminal_neck_down_length_nm\": "
      << result.configured_max_terminal_neck_down_length.value() << ",\n"
      << "  \"used_terminal_neck_down_segments\": " << result.used_terminal_neck_down_segments
      << ",\n"
      << "  \"total_terminal_neck_down_length_nm\": " << result.total_terminal_neck_down_length_nm
      << ",\n"
      << "  \"configured_medial_neck_down_width_nm\": "
      << result.configured_medial_neck_down_width.value() << ",\n"
      << "  \"configured_max_medial_neck_down_length_nm\": "
      << result.configured_max_medial_neck_down_length.value() << ",\n"
      << "  \"medial_neck_down_edges_available\": " << result.medial_neck_down_edges_available
      << ",\n"
      << "  \"used_medial_neck_down_segments\": " << result.used_medial_neck_down_segments << ",\n"
      << "  \"total_medial_neck_down_length_nm\": " << result.total_medial_neck_down_length_nm
      << ",\n"
      << "  \"configured_corridor_neck_down_width_nm\": "
      << result.configured_corridor_neck_down_width.value() << ",\n"
      << "  \"configured_max_corridor_neck_down_length_nm\": "
      << result.configured_max_corridor_neck_down_length.value() << ",\n"
      << "  \"corridor_neck_down_search_states\": " << result.corridor_neck_down_search_states
      << ",\n"
      << "  \"coupled_nets_ineligible_for_corridor_neck_down\": "
      << result.coupled_nets_ineligible_for_corridor_neck_down << ",\n"
      << "  \"used_corridor_neck_down_segments\": " << result.used_corridor_neck_down_segments
      << ",\n"
      << "  \"total_corridor_neck_down_length_nm\": " << result.total_corridor_neck_down_length_nm
      << ",\n"
      << "  \"configured_width_reduction_cost_per_grid_step\": "
      << result.configured_width_reduction_cost_per_grid_step << ",\n"
      << "  \"total_equivalent_width_reduction_length_nm\": "
      << result.total_equivalent_width_reduction_length_nm << ",\n"
      << "  \"total_width_reduction_cost\": " << result.total_width_reduction_cost << ",\n"
      << "  \"topology_rejected_search_edges\": " << result.topology_rejected_search_edges << ",\n"
      << "  \"medial_topology_breaks\": " << result.medial_topology_breaks << ",\n"
      << "  \"medial_direct_portal_transitions\": " << result.medial_direct_portal_transitions
      << ",\n"
      << "  \"medial_unmapped_nodes\": " << result.medial_unmapped_nodes << ",\n"
      << "  \"medial_interior_unmapped_nodes\": " << result.medial_interior_unmapped_nodes << ",\n"
      << "  \"topologically_complete_medial_paths\": " << result.topologically_complete_medial_paths
      << ",\n"
      << "  \"direct_voronoi_edges_available\": " << result.direct_voronoi_edges_available << ",\n"
      << "  \"direct_voronoi_edges_withheld\": " << result.direct_voronoi_edges_withheld << ",\n"
      << "  \"used_direct_voronoi_edges\": " << result.used_direct_voronoi_edges << ",\n"
      << "  \"direct_voronoi_grid_steps_bypassed\": " << result.direct_voronoi_grid_steps_bypassed
      << ",\n"
      << "  \"direct_voronoi_search_fallbacks\": " << result.direct_voronoi_search_fallbacks
      << ",\n"
      << "  \"multisource_steiner_growth_enabled\": "
      << (result.multisource_steiner_growth_enabled ? "true" : "false") << ",\n"
      << "  \"multisource_steiner_connections\": " << result.multisource_steiner_connections
      << ",\n"
      << "  \"multisource_steiner_reordered_connections\": "
      << result.multisource_steiner_reordered_connections << ",\n"
      << "  \"multisource_steiner_deferred_inaccessible_terminals\": "
      << result.multisource_steiner_deferred_inaccessible_terminals << ",\n"
      << "  \"multisource_steiner_fixed_order_fallbacks\": "
      << result.multisource_steiner_fixed_order_fallbacks << ",\n"
      << "  \"multisource_steiner_fixed_order_recoveries\": "
      << result.multisource_steiner_fixed_order_recoveries << ",\n"
      << "  \"bidirectional_route_growth_enabled\": "
      << (result.bidirectional_route_growth_enabled ? "true" : "false") << ",\n"
      << "  \"configured_bidirectional_route_growth_start_iteration\": "
      << result.configured_bidirectional_route_growth_start_iteration << ",\n"
      << "  \"configured_max_bidirectional_meeting_nodes\": "
      << result.configured_max_bidirectional_meeting_nodes << ",\n"
      << "  \"configured_max_bidirectional_meeting_expansions\": "
      << result.configured_max_bidirectional_meeting_expansions << ",\n"
      << "  \"bidirectional_route_growth_attempts\": " << result.bidirectional_route_growth_attempts
      << ",\n"
      << "  \"bidirectional_route_growth_meeting_nodes\": "
      << result.bidirectional_route_growth_meeting_nodes << ",\n"
      << "  \"bidirectional_route_growth_accepted\": " << result.bidirectional_route_growth_accepted
      << ",\n"
      << "  \"bidirectional_route_growth_fallbacks\": "
      << result.bidirectional_route_growth_fallbacks << ",\n"
      << "  \"atomic_complete_net_routing_enabled\": "
      << (result.atomic_complete_net_routing_enabled ? "true" : "false") << ",\n"
      << "  \"configured_selective_congestion_reroute\": "
      << (result.configured_selective_congestion_reroute ? "true" : "false") << ",\n"
      << "  \"configured_selective_full_reroute_interval\": "
      << result.configured_selective_full_reroute_interval << ",\n"
      << "  \"selective_reroute_passes\": " << result.selective_reroute_passes << ",\n"
      << "  \"selective_full_reroute_refreshes\": " << result.selective_full_reroute_refreshes
      << ",\n"
      << "  \"selective_retained_net_observations\": " << result.selective_retained_net_observations
      << ",\n"
      << "  \"selective_retained_path_observations\": "
      << result.selective_retained_path_observations << ",\n"
      << "  \"selective_conflict_net_observations\": " << result.selective_conflict_net_observations
      << ",\n"
      << "  \"selective_coupled_group_expansions\": " << result.selective_coupled_group_expansions
      << ",\n"
      << "  \"rolled_back_incomplete_net_trees\": " << result.rolled_back_incomplete_net_trees
      << ",\n"
      << "  \"rolled_back_incomplete_net_paths\": " << result.rolled_back_incomplete_net_paths
      << ",\n"
      << "  \"rolled_back_incomplete_net_cost\": " << result.rolled_back_incomplete_net_cost
      << ",\n"
      << "  \"routing_resources\": " << result.resource_usage.size() << ",\n"
      << "  \"congested_resources\": " << result.congested_resources << ",\n"
      << "  \"peak_resource_demand\": " << result.peak_resource_demand << ",\n"
      << "  \"congested_sampled_resource_usage\": ";
  print_congested_sampled_resource_usage(result.resource_usage);
  std::cout << ",\n"
            << "  \"pathfinder_reordered_iterations\": " << result.pathfinder_reordered_iterations
            << ",\n"
            << "  \"pathfinder_reordered_nets\": " << result.pathfinder_reordered_nets << ",\n"
            << "  \"pathfinder_peak_net_pressure\": " << result.pathfinder_peak_net_pressure
            << ",\n"
            << "  \"pathfinder_peak_net_sampled_overflow\": "
            << result.pathfinder_peak_net_sampled_overflow << ",\n"
            << "  \"pathfinder_peak_net_corridor_overflow\": "
            << result.pathfinder_peak_net_corridor_overflow << ",\n"
            << "  \"pathfinder_peak_net_total_physical_overflow_nm\": "
            << result.pathfinder_peak_net_total_physical_overflow_nm << ",\n"
            << "  \"pathfinder_peak_net_physical_overflow_nm\": "
            << result.pathfinder_peak_net_physical_overflow_nm << ",\n"
            << "  \"pathfinder_peak_net_congested_physical_resources\": "
            << result.pathfinder_peak_net_congested_physical_resources << ",\n"
            << "  \"selected_net_pressure\": ";
  print_routing_net_pressure(result.selected_net_pressure);
  std::cout << ",\n  \"congested_voronoi_resource_usage\": ";
  print_congested_voronoi_resource_usage(result.voronoi_resource_usage);
  std::cout
      << ",\n"
      << "  \"routing_corridors\": " << result.corridor_usage.size() << ",\n"
      << "  \"congested_corridors\": " << result.congested_corridors << ",\n"
      << "  \"neck_down_corridors\": " << result.neck_down_corridors << ",\n"
      << "  \"peak_corridor_demand\": " << result.peak_corridor_demand << ",\n"
      << "  \"corridor_neck_downs\": " << result.corridor_neck_down_usage.size() << ",\n"
      << "  \"congested_corridor_neck_downs\": " << result.congested_corridor_neck_downs << ",\n"
      << "  \"peak_corridor_neck_down_demand\": " << result.peak_corridor_neck_down_demand << ",\n"
      << "  \"voronoi_corridor_edges\": " << result.voronoi_corridor_evidence.size() << ",\n"
      << "  \"voronoi_domains_attempted\": " << result.voronoi_domains_attempted << ",\n"
      << "  \"voronoi_domains_accepted\": " << result.voronoi_domains_accepted << ",\n"
      << "  \"voronoi_domains_rejected\": " << result.voronoi_domains_rejected << ",\n"
      << "  \"voronoi_conservative_obstacles\": " << result.voronoi_conservative_obstacles << ",\n"
      << "  \"voronoi_evidence_curved_edge_chords\": " << result.voronoi_evidence_curved_edge_chords
      << ",\n"
      << "  \"voronoi_withheld_curved_edges\": " << result.voronoi_withheld_curved_edges << ",\n"
      << "  \"voronoi_guided_nets\": " << result.voronoi_guided_nets << ",\n"
      << "  \"voronoi_guidance_seed_nodes\": " << result.voronoi_guidance_seed_nodes << ",\n"
      << "  \"voronoi_allocation_resources\": " << result.voronoi_resource_usage.size() << ",\n"
      << "  \"voronoi_allocation_curved_edge_chords\": "
      << result.voronoi_allocation_curved_edge_chords << ",\n"
      << "  \"voronoi_allocation_withheld_curved_edges\": "
      << result.voronoi_allocation_withheld_curved_edges << ",\n"
      << "  \"configured_max_voronoi_resource_segment_length_nm\": "
      << result.configured_max_voronoi_resource_segment_length.value() << ",\n"
      << "  \"segmented_voronoi_source_resources\": " << result.segmented_voronoi_source_resources
      << ",\n"
      << "  \"additional_voronoi_segment_resources\": "
      << result.additional_voronoi_segment_resources << ",\n"
      << "  \"direct_voronoi_resources_retained_atomic\": "
      << result.direct_voronoi_resources_retained_atomic << ",\n"
      << "  \"congested_voronoi_resources\": " << result.congested_voronoi_resources << ",\n"
      << "  \"width_reduced_voronoi_allocations\": " << result.width_reduced_voronoi_allocations
      << ",\n"
      << "  \"total_voronoi_demand_reduction_nm\": " << result.total_voronoi_demand_reduction_nm
      << ",\n"
      << "  \"total_voronoi_overflow_nm\": " << result.total_voronoi_overflow_nm << ",\n"
      << "  \"peak_voronoi_overflow_nm\": " << result.peak_voronoi_overflow_nm << ",\n"
      << "  \"peak_voronoi_demand_nm\": " << result.peak_voronoi_demand_nm << ",\n"
      << "  \"coupled_differential_pair_resources\": " << result.coupled_differential_pair_resources
      << ",\n"
      << "  \"coupled_routing_bus_resources\": " << result.coupled_routing_bus_resources << ",\n"
      << "  \"packed_voronoi_resources\": " << result.packed_voronoi_resources << ",\n"
      << "  \"voronoi_lane_assignments\": " << result.voronoi_lane_assignments.size() << ",\n"
      << "  \"static_clearance_checked_voronoi_lanes\": "
      << result.static_clearance_checked_voronoi_lanes << ",\n"
      << "  \"voronoi_lane_transitions\": " << result.voronoi_lane_transitions.size() << ",\n"
      << "  \"accepted_voronoi_lane_transitions\": " << result.accepted_voronoi_lane_transitions
      << ",\n"
      << "  \"unaccepted_selected_voronoi_lane_transitions\": "
      << result.unaccepted_selected_voronoi_lane_transitions << ",\n"
      << "  \"direct_portal_lane_transitions\": " << result.direct_portal_lane_transitions << ",\n"
      << "  \"accepted_direct_portal_lane_transitions\": "
      << result.accepted_direct_portal_lane_transitions << ",\n"
      << "  \"unaccepted_direct_portal_lane_transitions\": "
      << result.unaccepted_direct_portal_lane_transitions << ",\n"
      << "  \"configured_negotiate_voronoi_lane_conflicts\": "
      << (result.configured_negotiate_voronoi_lane_conflicts ? "true" : "false") << ",\n"
      << "  \"configured_voronoi_lane_conflict_history_cost\": "
      << result.configured_voronoi_lane_conflict_history_cost << ",\n"
      << "  \"configured_max_voronoi_lane_conflict_history_entries\": "
      << result.configured_max_voronoi_lane_conflict_history_entries << ",\n"
      << "  \"voronoi_lane_conflict_passes\": " << result.voronoi_lane_conflict_passes << ",\n"
      << "  \"voronoi_lane_conflict_observations\": " << result.voronoi_lane_conflict_observations
      << ",\n"
      << "  \"voronoi_lane_conflict_history_entries\": "
      << result.voronoi_lane_conflict_history_entries << ",\n"
      << "  \"voronoi_exact_junction_conflict_history_entries\": "
      << result.voronoi_exact_junction_conflict_history_entries << ",\n"
      << "  \"voronoi_selected_junction_edge_conflict_history_entries\": "
      << result.voronoi_selected_junction_edge_conflict_history_entries << ",\n"
      << "  \"voronoi_lane_conflict_resource_history_entries\": "
      << result.voronoi_lane_conflict_resource_history_entries << ",\n"
      << "  \"voronoi_lane_conflict_history_increments\": "
      << result.voronoi_lane_conflict_history_increments << ",\n"
      << "  \"voronoi_lane_conflict_history_withheld\": "
      << result.voronoi_lane_conflict_history_withheld << ",\n"
      << "  \"voronoi_lane_conflict_scheduled_reroute_nets\": "
      << result.voronoi_lane_conflict_scheduled_reroute_nets << ",\n"
      << "  \"voronoi_lane_conflict_coupled_group_expansions\": "
      << result.voronoi_lane_conflict_coupled_group_expansions << ",\n"
      << "  \"unselected_voronoi_lane_transitions\": " << result.unselected_voronoi_lane_transitions
      << ",\n"
      << "  \"reoriented_voronoi_resources\": " << result.reoriented_voronoi_resources << ",\n"
      << "  \"voronoi_lane_orientation_states\": " << result.voronoi_lane_orientation_states
      << ",\n"
      << "  \"solved_voronoi_lane_orientation_components\": "
      << result.solved_voronoi_lane_orientation_components << ",\n"
      << "  \"withheld_voronoi_lane_orientation_components\": "
      << result.withheld_voronoi_lane_orientation_components << ",\n"
      << "  \"largest_voronoi_lane_orientation_component_resources\": "
      << result.largest_voronoi_lane_orientation_component_resources << ",\n"
      << "  \"solved_voronoi_lane_orientation_junctions\": "
      << result.solved_voronoi_lane_orientation_junctions << ",\n"
      << "  \"withheld_voronoi_lane_orientation_junctions\": "
      << result.withheld_voronoi_lane_orientation_junctions << ",\n"
      << "  \"mitered_voronoi_lane_transitions\": " << result.mitered_voronoi_lane_transitions
      << ",\n"
      << "  \"withheld_voronoi_lane_transitions\": " << result.withheld_voronoi_lane_transitions
      << ",\n"
      << "  \"withheld_voronoi_transition_clearance_checks\": "
      << result.withheld_voronoi_transition_clearance_checks << ",\n"
      << "  \"reused_paths\": " << result.reused_paths << ",\n"
      << "  \"reused_nets\": " << result.reused_nets.size() << ",\n"
      << "  \"rerouted_nets\": " << result.rerouted_nets.size() << ",\n"
      << "  \"invalidated_reuse_nets\": " << result.invalidated_reuse_nets.size()
      << ",\n  \"configured_search_diversification\": "
      << (result.configured_search_diversification ? "true" : "false")
      << ",\n  \"configured_global_route_beam_width\": "
      << result.configured_global_route_beam_width
      << ",\n  \"configured_max_large_neighborhood_attempts\": "
      << result.configured_max_large_neighborhood_attempts
      << ",\n  \"configured_large_neighborhood_stagnation_iterations\": "
      << result.configured_large_neighborhood_stagnation_iterations
      << ",\n  \"configured_destroy_route_percent\": " << result.configured_destroy_route_percent
      << ",\n  \"configured_max_routing_order_variants\": "
      << result.configured_max_routing_order_variants
      << ",\n  \"configured_routing_order_diversification_seed\": "
      << json_quote(std::to_string(result.configured_routing_order_diversification_seed))
      << ",\n  \"global_route_beam_peak\": " << result.global_route_beam_peak
      << ",\n  \"global_route_beam_candidates_retained\": "
      << result.global_route_beam_candidates_retained
      << ",\n  \"routing_order_variants_attempted\": " << result.routing_order_variants_attempted
      << ",\n  \"temporary_illegal_checkpoints\": " << result.temporary_illegal_checkpoints
      << ",\n  \"large_neighborhood_attempts\": " << result.large_neighborhood_attempts
      << ",\n  \"large_neighborhood_destroyed_nets\": " << result.large_neighborhood_destroyed_nets
      << ",\n  \"large_neighborhood_coupled_expansions\": "
      << result.large_neighborhood_coupled_expansions << ",\n  \"large_neighborhood_operators\": [";
  for (std::size_t index = 0U; index < result.large_neighborhood_operators.size(); ++index) {
    std::cout << (index == 0U ? "" : ", ")
              << json_quote(result.large_neighborhood_operators[index]);
  }
  std::cout << ']';
  std::cout << ",\n  \"initial_pathfinder_order\": [";
  for (std::size_t index = 0; index < result.initial_pathfinder_order.size(); ++index) {
    std::cout << (index == 0U ? "" : ", ")
              << json_quote(result.initial_pathfinder_order[index].value());
  }
  std::cout << ']';
  std::cout << ",\n  \"last_pathfinder_reroute_order\": [";
  for (std::size_t index = 0; index < result.last_pathfinder_reroute_order.size(); ++index) {
    std::cout << (index == 0U ? "" : ", ")
              << json_quote(result.last_pathfinder_reroute_order[index].value());
  }
  std::cout << "],\n"
            << "  \"obstacle_impacts\": [";
  for (std::size_t i = 0; i < result.obstacle_impacts.size(); ++i) {
    const auto& impact = result.obstacle_impacts[i];
    std::cout << (i == 0U ? "" : ",")
              << "\n    {\"routed_net\": " << json_quote(impact.routed_net.value())
              << ", \"obstacle\": " << json_quote(impact.obstacle.value())
              << ", \"rejected_edges\": " << impact.rejected_edges << '}';
  }
  if (!result.obstacle_impacts.empty()) {
    std::cout << '\n';
  }
  std::cout << "  ],\n  \"neck_down_obstacle_impacts\": [";
  for (std::size_t i = 0; i < result.neck_down_obstacle_impacts.size(); ++i) {
    const auto& impact = result.neck_down_obstacle_impacts[i];
    std::cout << (i == 0U ? "" : ",")
              << "\n    {\"routed_net\": " << json_quote(impact.routed_net.value())
              << ", \"obstacle\": " << json_quote(impact.obstacle.value())
              << ", \"kind\": " << json_quote(segment_width_kind_name(impact.width_kind))
              << ", \"layer\": " << json_quote(impact.layer.value()) << ", \"start_nm\": ["
              << impact.start.x.value() << ", " << impact.start.y.value() << "], \"end_nm\": ["
              << impact.end.x.value() << ", " << impact.end.y.value()
              << "], \"selected_width_nm\": " << impact.selected_width.value() << '}';
  }
  if (!result.neck_down_obstacle_impacts.empty()) {
    std::cout << '\n';
  }
  std::cout << "  ],\n  \"physical_channel_obstacle_impacts\": ";
  print_physical_obstacle_impacts(result.physical_obstacle_impacts);
  std::cout << ",\n  \"corridors\": [";
  for (std::size_t i = 0; i < result.corridor_usage.size(); ++i) {
    const auto& corridor = result.corridor_usage[i];
    std::cout << (i == 0U ? "" : ",") << "\n    {\"id\": " << corridor.id
              << ", \"layer\": " << json_quote(corridor.layer.value()) << ", \"orientation\": "
              << json_quote(corridor_orientation_name(corridor.orientation))
              << ", \"bounds_nm\": [[" << corridor.first_lane_start.x.value() << ", "
              << corridor.first_lane_start.y.value() << "], [" << corridor.last_lane_end.x.value()
              << ", " << corridor.last_lane_end.y.value() << "]], \"sampled_medial_nm\": [["
              << corridor.sampled_medial_start.x.value() << ", "
              << corridor.sampled_medial_start.y.value() << "], ["
              << corridor.sampled_medial_end.x.value() << ", "
              << corridor.sampled_medial_end.y.value()
              << "]], \"cross_section_span_nm\": " << corridor.cross_section_span.value()
              << ", \"capacity\": " << corridor.capacity << ", \"demand\": " << corridor.demand
              << ", \"neck_down\": " << (corridor.neck_down ? "true" : "false") << ", \"nets\": [";
    for (std::size_t net_index = 0; net_index < corridor.nets.size(); ++net_index) {
      std::cout << (net_index == 0U ? "" : ", ") << json_quote(corridor.nets[net_index].value());
    }
    std::cout << "]}";
  }
  if (!result.corridor_usage.empty()) {
    std::cout << '\n';
  }
  std::cout << "  ],\n  \"voronoi_resources\": [";
  for (std::size_t index = 0; index < result.voronoi_resource_usage.size(); ++index) {
    const auto& resource = result.voronoi_resource_usage[index];
    std::cout << (index == 0U ? "" : ",") << "\n    {\"id\": " << resource.id
              << ", \"layer\": " << json_quote(resource.layer.value()) << ", \"segment_nm\": [["
              << resource.start.x.value() << ", " << resource.start.y.value() << "], ["
              << resource.end.x.value() << ", " << resource.end.y.value()
              << "]], \"channel_width_nm\": " << resource.channel_width.value()
              << ", \"capacity_nm\": " << resource.capacity_nm
              << ", \"demand_nm\": " << resource.demand_nm
              << ", \"conservative_domain\": " << (resource.conservative_domain ? "true" : "false")
              << ", \"curved_bisector_chord\": "
              << (resource.curved_bisector_chord ? "true" : "false") << ", \"nets\": [";
    for (std::size_t net_index = 0; net_index < resource.nets.size(); ++net_index) {
      std::cout << (net_index == 0U ? "" : ", ") << json_quote(resource.nets[net_index].value());
    }
    std::cout << "]}";
  }
  if (!result.voronoi_resource_usage.empty()) {
    std::cout << '\n';
  }
  std::cout << "  ],\n  \"voronoi_lanes\": [";
  for (std::size_t index = 0; index < result.voronoi_lane_assignments.size(); ++index) {
    const auto& lane = result.voronoi_lane_assignments[index];
    std::cout << (index == 0U ? "" : ",") << "\n    {\"resource_id\": " << lane.resource_id
              << ", \"net\": " << json_quote(lane.net.value())
              << ", \"signed_offset_nm\": " << lane.signed_offset_nm
              << ", \"track_width_nm\": " << lane.track_width.value()
              << ", \"shifted_segment_nm\": [[" << lane.shifted_start.x.value() << ", "
              << lane.shifted_start.y.value() << "], [" << lane.shifted_end.x.value() << ", "
              << lane.shifted_end.y.value() << "]], \"differential_pair_member\": "
              << (lane.differential_pair_member ? "true" : "false") << ", \"routing_bus\": "
              << (lane.routing_bus ? json_quote(lane.routing_bus->value()) : "null")
              << ", \"static_clearance_checked\": "
              << (lane.static_clearance_checked ? "true" : "false") << '}';
  }
  if (!result.voronoi_lane_assignments.empty()) {
    std::cout << '\n';
  }
  std::cout << "  ],\n  \"voronoi_transitions\": [";
  for (std::size_t index = 0; index < result.voronoi_lane_transitions.size(); ++index) {
    const auto& transition = result.voronoi_lane_transitions[index];
    std::cout << (index == 0U ? "" : ",")
              << "\n    {\"first_resource_id\": " << transition.first_resource_id
              << ", \"second_resource_id\": " << transition.second_resource_id
              << ", \"net\": " << json_quote(transition.net.value())
              << ", \"layer\": " << json_quote(transition.layer.value())
              << ", \"medial_junction_nm\": [" << transition.medial_junction.x.value() << ", "
              << transition.medial_junction.y.value() << "], \"bridge_nm\": [["
              << transition.first_lane_endpoint.x.value() << ", "
              << transition.first_lane_endpoint.y.value() << "], ["
              << transition.second_lane_endpoint.x.value() << ", "
              << transition.second_lane_endpoint.y.value()
              << "]], \"track_width_nm\": " << transition.track_width.value()
              << ", \"differential_pair_member\": "
              << (transition.differential_pair_member ? "true" : "false") << ", \"routing_bus\": "
              << (transition.routing_bus ? json_quote(transition.routing_bus->value()) : "null")
              << ", \"static_clearance_checked\": "
              << (transition.static_clearance_checked ? "true" : "false")
              << ", \"mutual_clearance_checked\": "
              << (transition.mutual_clearance_checked ? "true" : "false")
              << ", \"direct_portal_bridge\": "
              << (transition.direct_portal_bridge ? "true" : "false") << ", \"miter_nm\": ";
    if (transition.miter) {
      std::cout << '[' << transition.miter->x.value() << ", " << transition.miter->y.value() << ']';
    } else {
      std::cout << "null";
    }
    std::cout << '}';
  }
  if (!result.voronoi_lane_transitions.empty()) {
    std::cout << '\n';
  }
  std::cout << "  ],\n  \"medial_paths\": [";
  for (std::size_t index = 0; index < result.paths.size(); ++index) {
    const auto& path = result.paths[index];
    std::cout << (index == 0U ? "" : ",") << "\n    {\"net\": " << json_quote(path.net.value())
              << ", \"connection_index\": " << path.connection_index
              << ", \"topology_breaks\": " << path.medial_topology_breaks
              << ", \"direct_portal_transitions\": " << path.medial_direct_portal_transitions
              << ", \"unmapped_nodes\": " << path.medial_unmapped_nodes
              << ", \"interior_unmapped_nodes\": " << path.medial_interior_unmapped_nodes
              << ", \"topology_complete\": " << (path.medial_topology_complete() ? "true" : "false")
              << ", \"resource_sequence\": [";
    for (std::size_t resource = 0; resource < path.medial_resource_sequence.size(); ++resource) {
      std::cout << (resource == 0U ? "" : ", ") << path.medial_resource_sequence[resource];
    }
    std::cout << "], \"layer_transitions\": [";
    bool first_layer_transition = true;
    for (std::size_t point = 1U; point < path.points.size(); ++point) {
      if (path.points[point - 1U].layer == path.points[point].layer) {
        continue;
      }
      std::cout << (first_layer_transition ? "" : ", ") << "{\"position_nm\": ["
                << path.points[point].position.x.value() << ", "
                << path.points[point].position.y.value()
                << "], \"from\": " << json_quote(path.points[point - 1U].layer.value())
                << ", \"to\": " << json_quote(path.points[point].layer.value()) << '}';
      first_layer_transition = false;
    }
    std::cout << "]}";
  }
  if (!result.paths.empty()) {
    std::cout << '\n';
  }
  std::cout << "  ],\n  \"regional_conflict_fragments\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->fragments.size();
  }
  std::cout << ",\n  \"regional_conflict_edges\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->conflict_edges.size();
  }
  std::cout << ",\n  \"regional_biconnected_blocks\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->biconnected_blocks.size();
  }
  std::cout << ",\n  \"regional_articulation_fragments\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->articulation_fragments.size();
  }
  std::cout << ",\n  \"regional_windows\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->windows.size();
  }
  std::cout << ",\n  \"regional_geometrically_split_windows\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->geometrically_split_windows;
  }
  std::cout << ",\n  \"regional_coupled_window_merges\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->coupled_window_merges;
  }
  std::cout << ",\n  \"regional_uncuttable_oversized_blocks\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->uncuttable_oversized_blocks;
  }
  std::cout << ",\n  \"regional_window_fragment_anchor_expansions\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->window_fragment_anchor_expansions;
  }
  std::cout << ",\n  \"regional_peak_window_fragment_anchor_expansion_nm\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->peak_window_fragment_anchor_expansion.value();
  }
  std::cout << ",\n  \"regional_eligible_windows\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << std::count_if(regional->windows.begin(), regional->windows.end(),
                               [](const auto& window) { return window.eligible(); });
  }
  std::cout << ",\n  \"regional_refittable_terminal_width_fragments\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->refittable_terminal_width_fragments;
  }
  std::cout << ",\n  \"regional_fixed_resource_width_fragments\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->fixed_resource_width_fragments;
  }
  std::cout << ",\n  \"regional_channels\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->channels.size();
  }
  std::cout << ",\n  \"regional_fragments_without_channel_topology\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->fragments_without_channel_topology;
  }
  std::cout << ",\n  \"regional_selected_global_narrow_corridor_fallbacks\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->selected_global_narrow_corridor_fallbacks;
  }
  std::cout << ",\n  \"regional_selected_lane_cross_sections\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->selected_lane_cross_sections;
  }
  std::cout << ",\n  \"regional_selected_geometric_lane_reorders\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->selected_geometric_lane_reorders;
  }
  std::cout << ",\n  \"regional_avoided_lane_order_inversions\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->avoided_lane_order_inversions;
  }
  const auto print_lane_solver_metric = [&](std::string_view name, std::size_t value) {
    std::cout << ",\n  \"" << name << "\": ";
    if (regional == nullptr) {
      std::cout << "null";
    } else {
      std::cout << value;
    }
  };
  print_lane_solver_metric("regional_lane_solver_queries",
                           regional == nullptr ? 0U : regional->lane_solver_queries);
  print_lane_solver_metric("regional_lane_solver_cache_hits",
                           regional == nullptr ? 0U : regional->lane_solver_cache_hits);
  print_lane_solver_metric("regional_lane_solver_states",
                           regional == nullptr ? 0U : regional->lane_solver_states);
  print_lane_solver_metric("regional_lane_solver_pruned_states",
                           regional == nullptr ? 0U : regional->lane_solver_pruned_states);
  print_lane_solver_metric("regional_lane_solver_analytical_rejections",
                           regional == nullptr ? 0U : regional->lane_solver_analytical_rejections);
  print_lane_solver_metric("regional_lane_solver_budget_exhaustions",
                           regional == nullptr ? 0U : regional->lane_solver_budget_exhaustions);
  print_lane_solver_metric("regional_partial_lane_capacity_pruned_states",
                           regional == nullptr ? 0U
                                               : regional->partial_lane_capacity_pruned_states);
  std::cout << ",\n  \"regional_complete_bundle_assignments\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << std::count_if(regional->bundle_assignments.begin(),
                               regional->bundle_assignments.end(),
                               [](const auto& assignment) { return assignment.complete(); });
  }
  const auto print_bundle_stop_count = [&](std::string_view name,
                                           flux::RegionalBundleStopReason reason) {
    std::cout << ",\n  \"" << name << "\": ";
    if (regional == nullptr) {
      std::cout << "null";
    } else {
      std::cout << std::count_if(
          regional->bundle_assignments.begin(), regional->bundle_assignments.end(),
          [&](const auto& assignment) { return assignment.stop_reason == reason; });
    }
  };
  print_bundle_stop_count("regional_missing_corridor_bundle_assignments",
                          flux::RegionalBundleStopReason::missing_corridor_topology);
  print_bundle_stop_count("regional_no_fitting_lane_bundle_assignments",
                          flux::RegionalBundleStopReason::no_fitting_lane_order);
  print_bundle_stop_count("regional_ineligible_bundle_assignments",
                          flux::RegionalBundleStopReason::window_ineligible);
  print_bundle_stop_count("regional_exhausted_bundle_assignments",
                          flux::RegionalBundleStopReason::state_budget_exhausted);
  print_bundle_stop_count("regional_failed_spline_bundle_assignments",
                          flux::RegionalBundleStopReason::spline_realization_failed);
  std::cout << ",\n  \"regional_spline_short_realization_rejections\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->spline_short_realization_rejections;
  }
  std::cout << ",\n  \"regional_spline_topology_rejections\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->spline_topology_rejections;
  }
  std::cout << ",\n  \"regional_spline_topology_endpoint_rejections\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->spline_topology_endpoint_rejections;
  }
  std::cout << ",\n  \"regional_spline_topology_interior_rejections\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->spline_topology_interior_rejections;
  }
  std::cout << ",\n  \"regional_spline_topology_gate_signatures\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->spline_topology_gate_signatures;
  }
  std::cout << ",\n  \"regional_spline_topology_gates\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->spline_topology_gates;
  }
  std::cout << ",\n  \"regional_spline_topology_gate_rejections\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->spline_topology_gate_rejections;
  }
  std::cout << ",\n  \"regional_spline_mutual_clearance_rejections\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->spline_mutual_clearance_rejections;
  }
  std::cout << ",\n  \"regional_spline_continuous_hulls_checked\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->spline_continuous_hulls_checked;
  }
  std::cout << ",\n  \"regional_spline_continuous_envelope_rejections\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->spline_continuous_envelope_rejections;
  }
  std::cout << ",\n  \"regional_spline_continuous_budget_exhaustions\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->spline_continuous_budget_exhaustions;
  }
  std::cout << ",\n  \"regional_spline_lane_targets\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->spline_lane_targets;
  }
  std::cout << ",\n  \"regional_spline_multi_target_fragments\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->spline_multi_target_fragments;
  }
  std::cout << ",\n  \"regional_spline_merged_runs\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->spline_merged_runs;
  }
  std::cout << ",\n  \"regional_spline_merged_fragments\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->spline_merged_fragments;
  }
  std::cout << ",\n  \"regional_spline_interpolation_portfolio_candidates\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->spline_interpolation_portfolio_candidates;
  }
  std::cout << ",\n  \"regional_spline_interpolation_portfolio_offset_rejections\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->spline_interpolation_portfolio_offset_rejections;
  }
  std::cout << ",\n  \"regional_spline_interpolation_portfolio_accepted\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->spline_interpolation_portfolio_accepted;
  }
  std::cout << ",\n  \"regional_spline_interpolation_portfolio_windows_improved\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->spline_interpolation_portfolio_windows_improved;
  }
  const auto print_spline_performance_metric = [&](std::string_view name, std::uint64_t value) {
    std::cout << ",\n  \"" << name << "\": ";
    if (regional == nullptr)
      std::cout << "null";
    else
      std::cout << value;
  };
  print_spline_performance_metric(
      "regional_spline_interpolation_gpu_batches",
      regional == nullptr ? 0U : regional->spline_interpolation_gpu_batches);
  print_spline_performance_metric(
      "regional_spline_interpolation_gpu_fallbacks",
      regional == nullptr ? 0U : regional->spline_interpolation_gpu_fallbacks);
  print_spline_performance_metric(
      "regional_spline_interpolation_gpu_candidates",
      regional == nullptr ? 0U : regional->spline_interpolation_gpu_candidates);
  print_spline_performance_metric(
      "regional_spline_interpolation_exact_candidates",
      regional == nullptr ? 0U : regional->spline_interpolation_exact_candidates);
  print_spline_performance_metric("regional_spline_geometry_cache_hits",
                                  regional == nullptr ? 0U
                                                      : regional->spline_geometry_cache_hits);
  print_spline_performance_metric("regional_spline_geometry_cache_misses",
                                  regional == nullptr ? 0U
                                                      : regional->spline_geometry_cache_misses);
  print_spline_performance_metric(
      "regional_spline_offset_broadphase_batches",
      regional == nullptr ? 0U : regional->spline_offset_broadphase_batches);
  print_spline_performance_metric(
      "regional_spline_offset_broadphase_gpu_batches",
      regional == nullptr ? 0U : regional->spline_offset_broadphase_gpu_batches);
  print_spline_performance_metric(
      "regional_spline_offset_broadphase_rejections",
      regional == nullptr ? 0U : regional->spline_offset_broadphase_rejections);
  print_spline_performance_metric("regional_spline_offset_exact_pairs",
                                  regional == nullptr ? 0U
                                                      : regional->spline_offset_exact_pairs);
  print_spline_performance_metric(
      "regional_spline_offset_subdivision_nodes",
      regional == nullptr ? 0U : regional->spline_offset_subdivision_nodes);
  print_spline_performance_metric(
      "regional_spline_offset_isolated_leaf_pairs",
      regional == nullptr ? 0U : regional->spline_offset_isolated_leaf_pairs);
  print_spline_performance_metric(
      "regional_spline_offset_node_budget_exhaustions",
      regional == nullptr ? 0U : regional->spline_offset_node_budget_exhaustions);
  print_spline_performance_metric(
      "regional_spline_interpolation_rank_us",
      regional == nullptr ? 0U
                          : static_cast<std::uint64_t>(regional->spline_interpolation_rank_us));
  print_spline_performance_metric(
      "regional_spline_interpolation_exact_us",
      regional == nullptr ? 0U
                          : static_cast<std::uint64_t>(regional->spline_interpolation_exact_us));
  print_spline_performance_metric(
      "regional_spline_adaptive_assessment_us",
      regional == nullptr ? 0U
                          : static_cast<std::uint64_t>(regional->spline_adaptive_assessment_us));
  print_spline_performance_metric(
      "regional_spline_offset_broadphase_us",
      regional == nullptr ? 0U
                          : static_cast<std::uint64_t>(regional->spline_offset_broadphase_us));
  print_spline_performance_metric(
      "regional_spline_offset_exact_us",
      regional == nullptr ? 0U
                          : static_cast<std::uint64_t>(regional->spline_offset_exact_us));
  print_spline_performance_metric(
      "regional_spline_realization_us",
      regional == nullptr ? 0U
                          : static_cast<std::uint64_t>(regional->spline_realization_us));
  std::cout << ",\n  \"regional_spline_joint_shove_attempts\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->spline_joint_shove_attempts;
  }
  std::cout << ",\n  \"regional_spline_joint_shove_accepted\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->spline_joint_shove_accepted;
  }
  std::cout << ",\n  \"regional_spline_adaptive_pair_shove_attempts\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->spline_adaptive_pair_shove_attempts;
  }
  std::cout << ",\n  \"regional_spline_adaptive_pair_shove_accepted\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->spline_adaptive_pair_shove_accepted;
  }
  std::cout << ",\n  \"regional_spline_adaptive_pair_windows_improved\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->spline_adaptive_pair_windows_improved;
  }
  std::cout << ",\n  \"regional_spline_adaptive_pair_windows_legalized\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->spline_adaptive_pair_windows_legalized;
  }
  std::cout << ",\n  \"regional_residual_conflict_cone_retries\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->residual_conflict_cone_retries;
  }
  std::cout << ",\n  \"regional_residual_conflict_cones_built\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->residual_conflict_cones_built;
  }
  std::cout << ",\n  \"regional_residual_conflict_cone_windows_merged\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->residual_conflict_cone_windows_merged;
  }
  std::cout << ",\n  \"regional_residual_conflict_cone_stub_fragments\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->residual_conflict_cone_stub_fragments;
  }
  std::cout << ",\n  \"regional_residual_conflict_cone_reused_assignments\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->residual_conflict_cone_reused_assignments;
  }
  std::cout << ",\n  \"regional_residual_conflict_cone_recomputed_assignments\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->residual_conflict_cone_recomputed_assignments;
  }
  std::cout << ",\n  \"regional_residual_conflict_cone_retry_withheld_assignments\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->residual_conflict_cone_retry_withheld_assignments;
  }
  std::cout << ",\n  \"regional_residual_conflict_cone_selected\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << (regional->residual_conflict_cone_selected ? "true" : "false");
  }
  std::cout << ",\n  \"regional_residual_cone_template_coverage_states\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->residual_cone_template_coverage_states;
  }
  std::cout << ",\n  \"regional_residual_cone_template_coverage_retained\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->residual_cone_template_coverage_retained;
  }
  std::cout << ",\n  \"regional_residual_conflict_cone_attempts\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    const auto print_fragment_ids = [](const auto& fragments) {
      std::cout << '[';
      for (std::size_t fragment = 0U; fragment < fragments.size(); ++fragment) {
        const auto& failure = fragments[fragment];
        const auto& id = failure.fragment;
        std::cout << (fragment == 0U ? "" : ",") << "{\"net\":" << json_quote(id.net.value())
                  << ",\"connection\":" << id.connection_index << ",\"run\":" << id.run_index
                  << ",\"fragment\":" << id.fragment_index
                  << ",\"merged_fragments\":" << failure.merged_fragments << '}';
      }
      std::cout << ']';
    };
    std::cout << '[';
    for (std::size_t index = 0U; index < regional->residual_conflict_cone_attempts.size();
         ++index) {
      const auto& attempt = regional->residual_conflict_cone_attempts[index];
      std::cout << (index == 0U ? "" : ",") << "{\"root_window\":" << attempt.root_window_id
                << ",\"source_windows\":[";
      for (std::size_t member = 0U; member < attempt.source_window_ids.size(); ++member) {
        std::cout << (member == 0U ? "" : ",") << attempt.source_window_ids[member];
      }
      std::cout << "]"
                << ",\"fragments\":" << attempt.fragment_count << ",\"nets\":" << attempt.net_count
                << ",\"assignments\":" << attempt.assignment_count
                << ",\"incomplete_assignments\":" << attempt.incomplete_assignments
                << ",\"topology_rejections\":" << attempt.topology_rejections
                << ",\"mutual_clearance_rejections\":" << attempt.mutual_clearance_rejections
                << ",\"quantized_clearance_rejections\":" << attempt.quantized_clearance_rejections
                << ",\"spline_candidates_evaluated\":" << attempt.spline_candidates_evaluated
                << ",\"spline_candidates_legal\":" << attempt.spline_candidates_legal
                << ",\"spline_merged_runs\":" << attempt.spline_merged_runs
                << ",\"spline_merged_fragments\":" << attempt.spline_merged_fragments
                << ",\"topology_rejected_fragments\":";
      print_fragment_ids(attempt.topology_rejected_fragments);
      std::cout << ",\"mutual_clearance_rejected_fragments\":";
      print_fragment_ids(attempt.mutual_clearance_rejected_fragments);
      std::cout << ",\"quantized_clearance_rejected_fragments\":";
      print_fragment_ids(attempt.quantized_clearance_rejected_fragments);
      std::cout << ",\"template_coverage_states\":" << attempt.template_coverage_states
                << ",\"template_coverage_retained\":" << attempt.template_coverage_retained
                << ",\"template_coverage_coupled_rejections\":"
                << attempt.template_coverage_coupled_rejections
                << ",\"template_coverage_access_rejections\":"
                << attempt.template_coverage_access_rejections
                << ",\"template_coverage_packing_rejections\":"
                << attempt.template_coverage_packing_rejections
                << ",\"stub_fragments\":" << attempt.stub_fragments
                << ",\"complete\":" << (attempt.complete ? "true" : "false")
                << ",\"selected\":" << (attempt.selected ? "true" : "false") << '}';
    }
    std::cout << ']';
  }
  const auto print_bundle_stop_windows = [&](std::string_view name,
                                             flux::RegionalBundleStopReason reason) {
    std::cout << ",\n  \"" << name << "\": ";
    if (regional == nullptr) {
      std::cout << "null";
      return;
    }
    std::cout << '[';
    bool first = true;
    for (const auto& assignment : regional->bundle_assignments) {
      if (assignment.stop_reason != reason) {
        continue;
      }
      if (!first) {
        std::cout << ',';
      }
      std::cout << assignment.window_id;
      first = false;
    }
    std::cout << ']';
  };
  print_bundle_stop_windows("regional_no_fitting_lane_window_ids",
                            flux::RegionalBundleStopReason::no_fitting_lane_order);
  print_bundle_stop_windows("regional_ineligible_window_ids",
                            flux::RegionalBundleStopReason::window_ineligible);
  std::cout << ",\n  \"regional_unresolved_windows\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    const auto failure_name = [](flux::RegionalWindowFailureReason reason) {
      switch (reason) {
      case flux::RegionalWindowFailureReason::none:
        return "none";
      case flux::RegionalWindowFailureReason::oversized_fragment_set:
        return "oversized_fragment_set";
      case flux::RegionalWindowFailureReason::oversized_net_set:
        return "oversized_net_set";
      case flux::RegionalWindowFailureReason::uncuttable_fragment:
        return "uncuttable_fragment";
      case flux::RegionalWindowFailureReason::coupled_atom_split:
        return "coupled_atom_split";
      }
      return "unknown";
    };
    std::cout << '[';
    bool first = true;
    for (const auto& window : regional->windows) {
      const auto assignment =
          std::find_if(regional->bundle_assignments.begin(), regional->bundle_assignments.end(),
                       [&](const auto& candidate) { return candidate.window_id == window.id; });
      if (assignment == regional->bundle_assignments.end() || assignment->complete()) {
        continue;
      }
      if (!first) {
        std::cout << ',';
      }
      std::cout << "{\"id\":" << window.id
                << ",\"failure\":" << json_quote(failure_name(window.failure_reason))
                << ",\"fragments\":" << window.fragments.size() << ",\"nets\":";
      print_net_ids(window.nets);
      std::cout << ",\"fragment_details\":[";
      for (std::size_t fragment_index = 0U; fragment_index < window.fragments.size();
           ++fragment_index) {
        const auto& fragment_id = window.fragments[fragment_index];
        const auto fragment =
            std::find_if(regional->fragments.begin(), regional->fragments.end(),
                         [&](const auto& candidate) { return candidate.id == fragment_id; });
        std::cout << (fragment_index == 0U ? "" : ",");
        if (fragment == regional->fragments.end()) {
          std::cout << "null";
          continue;
        }
        const auto owned = std::binary_search(window.owned_fragments.begin(),
                                              window.owned_fragments.end(), fragment_id);
        const auto template_count = static_cast<std::size_t>(std::count_if(
            assignment->templates.begin(), assignment->templates.end(), [&](const auto& value) {
              return value.fragment == fragment_id && !value.channel_sequence.empty();
            }));
        std::cout << "{\"net\":" << json_quote(fragment_id.net.value())
                  << ",\"connection\":" << fragment_id.connection_index
                  << ",\"run\":" << fragment_id.run_index
                  << ",\"fragment\":" << fragment_id.fragment_index
                  << ",\"layer\":" << json_quote(fragment->layer.value())
                  << ",\"start_x_nm\":" << fragment->start.x.value()
                  << ",\"start_y_nm\":" << fragment->start.y.value()
                  << ",\"end_x_nm\":" << fragment->end.x.value()
                  << ",\"end_y_nm\":" << fragment->end.y.value()
                  << ",\"width_nm\":" << fragment->width.value()
                  << ",\"clearance_nm\":" << fragment->clearance.value()
                  << ",\"owned\":" << (owned ? "true" : "false")
                  << ",\"coupled\":" << (fragment->coupled_member ? "true" : "false")
                  << ",\"explicit_width\":" << (fragment->explicit_width ? "true" : "false")
                  << ",\"template_count\":" << template_count << '}';
      }
      std::cout << "]"
                << ",\"conflict_details\":[";
      for (std::size_t conflict_index = 0U; conflict_index < window.conflict_edges.size();
           ++conflict_index) {
        const auto edge_id = window.conflict_edges[conflict_index];
        std::cout << (conflict_index == 0U ? "" : ",");
        if (edge_id >= regional->conflict_edges.size()) {
          std::cout << "null";
          continue;
        }
        const auto& edge = regional->conflict_edges[edge_id];
        std::cout << "{\"id\":" << edge.id
                  << ",\"first_net\":" << json_quote(edge.first.net.value())
                  << ",\"second_net\":" << json_quote(edge.second.net.value())
                  << ",\"layer\":" << json_quote(edge.layer.value())
                  << ",\"minimum_x_nm\":" << edge.conflict_bounds.minimum.x.value()
                  << ",\"minimum_y_nm\":" << edge.conflict_bounds.minimum.y.value()
                  << ",\"maximum_x_nm\":" << edge.conflict_bounds.maximum.x.value()
                  << ",\"maximum_y_nm\":" << edge.conflict_bounds.maximum.y.value()
                  << ",\"deficit_nm\":" << edge.peak_clearance_deficit.value()
                  << ",\"overlap_length_nm\":" << edge.observed_overlap_length_nm << '}';
      }
      std::cout << "]"
                << ",\"conflicts\":" << window.conflict_edges.size()
                << ",\"bundle_states\":" << assignment->bundle_search_states
                << ",\"partial_access_pruned\":" << assignment->partial_access_pruned_states
                << ",\"partial_access_assignment_states\":"
                << assignment->partial_access_assignment_states
                << ",\"partial_via_pair_pruned\":" << assignment->partial_via_pair_pruned_states
                << ",\"partial_terminal_via_pruned\":"
                << assignment->partial_terminal_via_pruned_states
                << ",\"partial_access_blockers\":[";
      for (std::size_t blocker_index = 0U;
           blocker_index < assignment->partial_access_blockers.size(); ++blocker_index) {
        const auto& blocker = assignment->partial_access_blockers[blocker_index];
        const auto print_via = [&](const std::optional<std::size_t>& choice_id) {
          if (!choice_id) {
            std::cout << "null";
            return;
          }
          const auto choice =
              std::find_if(regional->via_choices.begin(), regional->via_choices.end(),
                           [&](const auto& candidate) { return candidate.id == *choice_id; });
          if (choice == regional->via_choices.end()) {
            std::cout << "null";
            return;
          }
          std::cout << "{\"choice\":" << choice->id << ",\"region\":" << choice->region_id
                    << ",\"net\":" << json_quote(choice->net.value())
                    << ",\"x_nm\":" << choice->site.x.value()
                    << ",\"y_nm\":" << choice->site.y.value()
                    << ",\"selected_global\":" << (choice->selected_global_site ? "true" : "false")
                    << '}';
        };
        std::cout << (blocker_index == 0U ? "" : ",") << "{\"kind\":"
                  << json_quote(blocker.kind == flux::RegionalAccessBlockerKind::via_pair
                                    ? "via_pair"
                                    : "terminal_via")
                  << ",\"observations\":" << blocker.observations << ",\"first_via\":";
        print_via(blocker.first_via_choice);
        std::cout << ",\"second_via\":";
        print_via(blocker.second_via_choice);
        std::cout << ",\"terminal_choice\":";
        if (blocker.terminal_choice) {
          const auto terminal = std::find_if(
              regional->terminal_choices.begin(), regional->terminal_choices.end(),
              [&](const auto& candidate) { return candidate.id == *blocker.terminal_choice; });
          if (terminal != regional->terminal_choices.end()) {
            std::cout << "{\"choice\":" << terminal->id
                      << ",\"net\":" << json_quote(terminal->net.value())
                      << ",\"layer\":" << json_quote(terminal->layer.value())
                      << ",\"x_nm\":" << terminal->portal.x.value()
                      << ",\"y_nm\":" << terminal->portal.y.value() << '}';
          } else {
            std::cout << "null";
          }
        } else {
          std::cout << "null";
        }
        std::cout << '}';
      }
      std::cout
          << "]"
          << ",\"access_assignment_states\":" << assignment->access_assignment_states
          << ",\"coupled_rejected\":" << assignment->coupled_rejected_states
          << ",\"access_rejected\":" << assignment->access_rejected_states
          << ",\"terminal_pair_rejected\":" << assignment->terminal_pair_rejected_states
          << ",\"via_pair_rejected\":" << assignment->via_pair_rejected_states
          << ",\"terminal_via_rejected\":" << assignment->terminal_via_rejected_states
          << ",\"packing_rejected\":" << assignment->packing_rejected_states
          << ",\"selected_template_conflicts\":" << assignment->selected_template_conflicts
          << ",\"selected_residual_conflicts\":" << assignment->selected_residual_conflicts
          << ",\"joint_spline_candidate\":"
          << (assignment->joint_spline_candidate ? "true" : "false")
          << ",\"spline_candidate_rank\":" << assignment->spline_candidate_rank
          << ",\"selected_alternative_templates\":"
          << std::count_if(assignment->selected_templates.begin(),
                           assignment->selected_templates.end(),
                           [&](const auto template_id) {
                             return template_id < assignment->templates.size() &&
                                    !assignment->templates[template_id].selected_global_topology;
                           })
          << ",\"spline_realizations\":" << assignment->spline_realizations.size()
          << ",\"spline_topology_rejected\":"
          << std::count_if(assignment->spline_realizations.begin(),
                           assignment->spline_realizations.end(),
                           [](const auto& realization) { return !realization.topology_legal; })
          << ",\"spline_shape_rejected\":"
          << std::count_if(assignment->spline_realizations.begin(),
                           assignment->spline_realizations.end(),
                           [](const auto& realization) { return !realization.shape_legal; })
          << ",\"spline_mutual_rejected\":"
          << std::count_if(assignment->spline_realizations.begin(),
                           assignment->spline_realizations.end(),
                           [](const auto& realization) { return !realization.mutually_legal; })
          << ",\"spline_quantized_rejected\":"
          << std::count_if(
                 assignment->spline_realizations.begin(), assignment->spline_realizations.end(),
                 [](const auto& realization) { return !realization.quantized_mutually_legal; })
          << ",\"spline_candidates_evaluated\":" << assignment->spline_candidates_evaluated
          << ",\"spline_candidates_legal\":" << assignment->spline_candidates_legal
          << ",\"spline_candidate_min_topology_rejections\":"
          << assignment->spline_candidate_min_topology_rejections
          << ",\"spline_candidate_min_shape_rejections\":"
          << assignment->spline_candidate_min_shape_rejections
          << ",\"spline_candidate_min_mutual_clearance_rejections\":"
          << assignment->spline_candidate_min_mutual_clearance_rejections
          << ",\"spline_candidate_min_quantized_clearance_rejections\":"
          << assignment->spline_candidate_min_quantized_clearance_rejections
          << ",\"spline_candidate_summaries\":[";
      for (std::size_t candidate_index = 0U;
           candidate_index < assignment->spline_candidate_summaries.size(); ++candidate_index) {
        const auto& candidate = assignment->spline_candidate_summaries[candidate_index];
        std::cout << (candidate_index == 0U ? "" : ",") << "{\"rank\":" << candidate.rank
                  << ",\"templates\":[";
        for (std::size_t template_index = 0U; template_index < candidate.templates.size();
             ++template_index) {
          const auto template_id = candidate.templates[template_index];
          std::cout << (template_index == 0U ? "" : ",") << template_id;
        }
        std::cout << "]"
                  << ",\"short\":" << candidate.short_realizations
                  << ",\"topology\":" << candidate.topology_rejections
                  << ",\"shape\":" << candidate.shape_rejections
                  << ",\"mutual\":" << candidate.mutual_clearance_rejections
                  << ",\"quantized\":" << candidate.quantized_clearance_rejections
                  << ",\"complete\":" << (candidate.complete ? "true" : "false") << '}';
      }
      std::cout << "]"
                << ",\"template_details\":[";
      bool first_template = true;
      for (const auto& route_template : assignment->templates) {
        if (std::find(window.fragments.begin(), window.fragments.end(), route_template.fragment) ==
            window.fragments.end()) {
          continue;
        }
        std::cout << (first_template ? "" : ",") << "{\"id\":" << route_template.id
                  << ",\"net\":" << json_quote(route_template.fragment.net.value())
                  << ",\"connection\":" << route_template.fragment.connection_index
                  << ",\"run\":" << route_template.fragment.run_index
                  << ",\"fragment\":" << route_template.fragment.fragment_index
                  << ",\"routing_layer\":" << json_quote(route_template.routing_layer.value())
                  << ",\"selected\":"
                  << (std::find(assignment->selected_templates.begin(),
                                assignment->selected_templates.end(),
                                route_template.id) != assignment->selected_templates.end()
                          ? "true"
                          : "false")
                  << ",\"selected_global_topology\":"
                  << (route_template.selected_global_topology ? "true" : "false")
                  << ",\"exact_static_checked\":"
                  << (route_template.exact_static_checked ? "true" : "false")
                  << ",\"topology_points\":" << route_template.topology_centerline.size()
                  << ",\"topology_gates\":" << route_template.topology_signature.gates.size()
                  << ",\"channels\":" << route_template.channel_sequence.size()
                  << ",\"vias\":" << route_template.via_choices.size()
                  << ",\"lower_bound_length_nm\":" << route_template.lower_bound_length_nm
                  << ",\"avoided_conflicts\":" << route_template.avoided_conflict_edges.size()
                  << '}';
        first_template = false;
      }
      std::cout << "]"
                << ",\"spline_details\":[";
      for (std::size_t spline_index = 0U; spline_index < assignment->spline_realizations.size();
           ++spline_index) {
        const auto& spline = assignment->spline_realizations[spline_index];
        std::cout << (spline_index == 0U ? "" : ",")
                  << "{\"net\":" << json_quote(spline.fragment.net.value())
                  << ",\"connection\":" << spline.fragment.connection_index
                  << ",\"run\":" << spline.fragment.run_index
                  << ",\"fragment\":" << spline.fragment.fragment_index
                  << ",\"merged_fragments\":" << spline.merged_fragments
                  << ",\"topology_legal\":" << (spline.topology_legal ? "true" : "false")
                  << ",\"endpoint_topology_legal\":"
                  << (spline.endpoint_topology_legal ? "true" : "false")
                  << ",\"interior_topology_legal\":"
                  << (spline.interior_topology_legal ? "true" : "false")
                  << ",\"gate_sequence_legal\":"
                  << (spline.topology_gate_sequence_legal ? "true" : "false")
                  << ",\"topology_diagnostic\":" << json_quote(spline.topology_diagnostic)
                  << ",\"mutually_legal\":" << (spline.mutually_legal ? "true" : "false")
                  << ",\"quantized_mutually_legal\":"
                  << (spline.quantized_mutually_legal ? "true" : "false") << '}';
      }
      std::cout << "]}";
      first = false;
    }
    std::cout << ']';
  }
  std::cout << ",\n  \"regional_bundle_search_states\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::size_t states = 0U;
    for (const auto& assignment : regional->bundle_assignments) {
      states += assignment.bundle_search_states;
    }
    std::cout << states;
  }
  std::cout << ",\n  \"regional_selected_residual_conflicts\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::size_t conflicts = 0U;
    for (const auto& assignment : regional->bundle_assignments) {
      conflicts += assignment.selected_residual_conflicts;
    }
    std::cout << conflicts;
  }
  std::cout << ",\n  \"regional_selected_template_conflicts\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::size_t conflicts = 0U;
    for (const auto& assignment : regional->bundle_assignments) {
      conflicts += assignment.selected_template_conflicts;
    }
    std::cout << conflicts;
  }
  const auto print_rejected_bundle_states = [&](std::string_view name, const auto& selector) {
    std::cout << ",\n  \"" << name << "\": ";
    if (regional == nullptr) {
      std::cout << "null";
    } else {
      std::size_t states = 0U;
      for (const auto& assignment : regional->bundle_assignments) {
        states += selector(assignment);
      }
      std::cout << states;
    }
  };
  print_rejected_bundle_states(
      "regional_partial_access_pruned_bundle_states",
      [](const auto& assignment) { return assignment.partial_access_pruned_states; });
  print_rejected_bundle_states(
      "regional_partial_access_assignment_states",
      [](const auto& assignment) { return assignment.partial_access_assignment_states; });
  print_rejected_bundle_states(
      "regional_partial_access_assignment_exhaustions",
      [](const auto& assignment) { return assignment.partial_access_assignment_exhaustions; });
  print_rejected_bundle_states(
      "regional_partial_terminal_pair_pruned_bundle_states",
      [](const auto& assignment) { return assignment.partial_terminal_pair_pruned_states; });
  print_rejected_bundle_states(
      "regional_partial_via_pair_pruned_bundle_states",
      [](const auto& assignment) { return assignment.partial_via_pair_pruned_states; });
  print_rejected_bundle_states(
      "regional_partial_terminal_via_pruned_bundle_states",
      [](const auto& assignment) { return assignment.partial_terminal_via_pruned_states; });
  print_rejected_bundle_states(
      "regional_partial_lane_capacity_pruned_bundle_states",
      [](const auto& assignment) { return assignment.partial_lane_capacity_pruned_states; });
  print_rejected_bundle_states(
      "regional_coupled_rejected_bundle_states",
      [](const auto& assignment) { return assignment.coupled_rejected_states; });
  print_rejected_bundle_states(
      "regional_access_rejected_bundle_states",
      [](const auto& assignment) { return assignment.access_rejected_states; });
  print_rejected_bundle_states(
      "regional_invalid_access_rejected_bundle_states",
      [](const auto& assignment) { return assignment.invalid_access_rejected_states; });
  print_rejected_bundle_states(
      "regional_terminal_pair_rejected_bundle_states",
      [](const auto& assignment) { return assignment.terminal_pair_rejected_states; });
  print_rejected_bundle_states(
      "regional_via_pair_rejected_bundle_states",
      [](const auto& assignment) { return assignment.via_pair_rejected_states; });
  print_rejected_bundle_states(
      "regional_terminal_via_rejected_bundle_states",
      [](const auto& assignment) { return assignment.terminal_via_rejected_states; });
  print_rejected_bundle_states(
      "regional_packing_rejected_bundle_states",
      [](const auto& assignment) { return assignment.packing_rejected_states; });
  print_rejected_bundle_states("regional_access_assignment_states", [](const auto& assignment) {
    return assignment.access_assignment_states;
  });
  print_rejected_bundle_states(
      "regional_access_assignment_exhaustions",
      [](const auto& assignment) { return assignment.access_assignment_exhaustions; });
  print_rejected_bundle_states(
      "regional_selected_via_alternative_switches",
      [](const auto& assignment) { return assignment.selected_via_alternative_switches; });
  print_rejected_bundle_states(
      "regional_selected_alternative_templates", [](const auto& assignment) {
        return static_cast<std::size_t>(
            std::count_if(assignment.selected_templates.begin(),
                          assignment.selected_templates.end(), [&](const auto template_id) {
                            return template_id < assignment.templates.size() &&
                                   !assignment.templates[template_id].selected_global_topology;
                          }));
      });
  print_rejected_bundle_states("regional_selected_relocated_vias", [&](const auto& assignment) {
    return static_cast<std::size_t>(std::count_if(
        assignment.selected_via_choices.begin(), assignment.selected_via_choices.end(),
        [&](const auto choice_id) {
          const auto found =
              std::find_if(regional->via_choices.begin(), regional->via_choices.end(),
                           [&](const auto& candidate) { return candidate.id == choice_id; });
          return found != regional->via_choices.end() && !found->selected_global_site;
        }));
  });
  std::cout << ",\n  \"regional_spline_candidate_available\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << (regional->spline_candidate_available ? "true" : "false");
  }
  std::cout << ",\n  \"regional_complete_via_dependent_routes\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->complete_via_dependent_routes;
  }
  std::cout << ",\n  \"regional_planned_layer_runs\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->planned_layer_runs;
  }
  std::cout << ",\n  \"regional_planned_via_transitions\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->planned_via_transitions;
  }
  std::cout << ",\n  \"regional_stitched_multilayer_runs\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->stitched_multilayer_runs;
  }
  std::cout << ",\n  \"regional_spline_realizations\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::size_t realizations = 0U;
    for (const auto& assignment : regional->bundle_assignments) {
      realizations += assignment.spline_realizations.size();
    }
    std::cout << realizations;
  }
  std::cout << ",\n  \"regional_terminal_choices\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->terminal_choices.size();
  }
  std::cout << ",\n  \"regional_selected_terminal_portal_choices\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->selected_terminal_portal_choices;
  }
  std::cout << ",\n  \"regional_terminal_navmesh_template_checks\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->terminal_navmesh_template_checks;
  }
  std::cout << ",\n  \"regional_accepted_terminal_corridor_rebinds\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->accepted_terminal_corridor_rebinds;
  }
  std::cout << ",\n  \"regional_accepted_terminal_navmesh_templates\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->accepted_terminal_navmesh_templates;
  }
  std::cout << ",\n  \"regional_rejected_terminal_navmesh_templates\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->rejected_terminal_navmesh_templates;
  }
  std::cout << ",\n  \"regional_terminal_navmesh_template_budget_exhausted\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << (regional->terminal_navmesh_template_budget_exhausted ? "true" : "false");
  }
  std::cout << ",\n  \"regional_via_choices\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->via_choices.size();
  }
  std::cout << ",\n  \"regional_via_candidate_regions\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->via_region_evidence.size();
  }
  std::cout << ",\n  \"regional_contested_via_regions\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->contested_via_regions;
  }
  std::cout << ",\n  \"regional_peak_via_region_contention\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->peak_via_region_contention;
  }
  std::cout << ",\n  \"regional_via_region_evidence\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << '[';
    for (std::size_t index = 0U; index < regional->via_region_evidence.size(); ++index) {
      const auto& evidence = regional->via_region_evidence[index];
      std::cout << (index == 0U ? "" : ",") << "{\"region\":" << evidence.region_id
                << ",\"net\":" << json_quote(evidence.net.value()) << ",\"span\":[";
      for (std::size_t layer = 0U; layer < evidence.span.size(); ++layer) {
        std::cout << (layer == 0U ? "" : ",") << json_quote(evidence.span[layer].value());
      }
      std::cout << "],\"candidate_choices\":[";
      for (std::size_t choice = 0U; choice < evidence.candidate_choices.size(); ++choice) {
        std::cout << (choice == 0U ? "" : ",") << evidence.candidate_choices[choice];
      }
      std::cout << "],\"selected_global_candidates\":" << evidence.selected_global_candidates
                << ",\"competing_regions\":" << evidence.competing_regions
                << ",\"conflicting_candidate_pairs\":" << evidence.conflicting_candidate_pairs
                << ",\"terminal_escape_conflicts\":" << evidence.terminal_escape_conflicts
                << ",\"overlapping_span_regions\":" << evidence.overlapping_span_regions
                << ",\"contested\":" << (evidence.contested ? "true" : "false")
                << ",\"alternative_search_deferred\":"
                << (evidence.alternative_search_deferred ? "true" : "false") << '}';
    }
    std::cout << ']';
  }
  std::cout << ",\n  \"regional_via_site_candidate_checks\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->via_site_candidate_checks;
  }
  std::cout << ",\n  \"regional_via_site_diversity_rejections\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->via_site_diversity_rejections;
  }
  std::cout << ",\n  \"regional_via_site_candidate_budget_exhausted\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << (regional->via_site_candidate_budget_exhausted ? "true" : "false");
  }
  std::cout << ",\n  \"regional_selected_via_site_candidate_budget_exhausted\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << (regional->selected_via_site_candidate_budget_exhausted ? "true" : "false");
  }
  std::cout << ",\n  \"regional_alternative_template_checks\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->alternative_template_checks;
  }
  std::cout << ",\n  \"regional_alternative_template_budget_exhausted\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << (regional->alternative_template_budget_exhausted ? "true" : "false");
  }
  std::cout << ",\n  \"regional_accepted_alternative_templates\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->accepted_alternative_templates;
  }
  std::cout << ",\n  \"regional_joint_repair_windows_attempted\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->joint_repair_windows_attempted;
  }
  std::cout << ",\n  \"regional_joint_repair_windows_with_expanded_portfolio\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->joint_repair_windows_with_expanded_portfolio;
  }
  std::cout << ",\n  \"regional_joint_repair_windows_completed\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->joint_repair_windows_completed;
  }
  std::cout << ",\n  \"regional_joint_repair_window_ids\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << '[';
    for (std::size_t index = 0U; index < regional->joint_repair_window_ids.size(); ++index) {
      std::cout << (index == 0U ? "" : ",") << regional->joint_repair_window_ids[index];
    }
    std::cout << ']';
  }
  std::cout << ",\n  \"regional_joint_repair_completed_window_ids\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << '[';
    for (std::size_t index = 0U; index < regional->joint_repair_completed_window_ids.size();
         ++index) {
      std::cout << (index == 0U ? "" : ",") << regional->joint_repair_completed_window_ids[index];
    }
    std::cout << ']';
  }
  std::cout << ",\n  \"regional_joint_repair_template_checks\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->joint_repair_template_checks;
  }
  std::cout << ",\n  \"regional_joint_repair_accepted_templates\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->joint_repair_accepted_templates;
  }
  std::cout << ",\n  \"regional_joint_repair_template_budget_exhausted\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << (regional->joint_repair_template_budget_exhausted ? "true" : "false");
  }
  std::cout << ",\n  \"regional_local_layer_bypass_checks\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->local_layer_bypass_checks;
  }
  std::cout << ",\n  \"regional_accepted_local_layer_bypasses\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->accepted_local_layer_bypasses;
  }
  std::cout << ",\n  \"regional_local_layer_bypass_budget_exhausted\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << (regional->local_layer_bypass_budget_exhausted ? "true" : "false");
  }
  std::cout << ",\n  \"regional_joint_spline_candidates_evaluated\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->joint_spline_candidates_evaluated;
  }
  std::cout << ",\n  \"regional_spline_portfolio_windows\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->spline_portfolio_windows;
  }
  std::cout << ",\n  \"regional_spline_portfolio_window_ids\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << '[';
    for (std::size_t index = 0U; index < regional->spline_portfolio_window_ids.size(); ++index) {
      std::cout << (index == 0U ? "" : ",") << regional->spline_portfolio_window_ids[index];
    }
    std::cout << ']';
  }
  std::cout << ",\n  \"regional_joint_spline_candidates_legal\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->joint_spline_candidates_legal;
  }
  std::cout << ",\n  \"regional_joint_spline_windows_recovered\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->joint_spline_windows_recovered;
  }
  std::cout << ",\n  \"regional_joint_spline_recovered_window_ids\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << '[';
    for (std::size_t index = 0U; index < regional->joint_spline_recovered_window_ids.size();
         ++index) {
      std::cout << (index == 0U ? "" : ",") << regional->joint_spline_recovered_window_ids[index];
    }
    std::cout << ']';
  }
  std::cout << ",\n  \"regional_joint_spline_failed_fallback_improvements\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->joint_spline_failed_fallback_improvements;
  }
  const auto print_regional_diagnosis_metric = [&](std::string_view name, std::size_t value) {
    std::cout << ",\n  \"" << name << "\": ";
    if (regional == nullptr) {
      std::cout << "null";
    } else {
      std::cout << value;
    }
  };
  print_regional_diagnosis_metric(
      "regional_joint_spline_candidates_generated",
      regional == nullptr ? 0U : regional->joint_spline_candidates_generated);
  print_regional_diagnosis_metric(
      "regional_joint_spline_active_candidates_retained",
      regional == nullptr ? 0U : regional->joint_spline_active_candidates_retained);
  print_regional_diagnosis_metric(
      "regional_joint_spline_fallback_candidates_retained",
      regional == nullptr ? 0U : regional->joint_spline_fallback_candidates_retained);
  print_regional_diagnosis_metric(
      "regional_joint_spline_fallback_candidates_dropped",
      regional == nullptr ? 0U : regional->joint_spline_fallback_candidates_dropped);
  print_regional_diagnosis_metric(
      "regional_joint_spline_fallback_candidates_promoted",
      regional == nullptr ? 0U : regional->joint_spline_fallback_candidates_promoted);
  print_regional_diagnosis_metric(
      "regional_joint_spline_fallback_promotion_waves",
      regional == nullptr ? 0U : regional->joint_spline_fallback_promotion_waves);
  print_regional_diagnosis_metric(
      "regional_joint_spline_fallback_candidates_remaining",
      regional == nullptr ? 0U : regional->joint_spline_fallback_candidates_remaining);
  print_regional_diagnosis_metric(
      "regional_joint_spline_fallback_windows_recovered",
      regional == nullptr ? 0U : regional->joint_spline_fallback_windows_recovered);
  print_regional_diagnosis_metric(
      "regional_local_routability_diagnoses",
      regional == nullptr ? 0U : regional->local_routability_diagnoses.size());
  print_regional_diagnosis_metric(
      "regional_locally_resolved_routability_diagnoses",
      regional == nullptr ? 0U : regional->locally_resolved_routability_diagnoses);
  print_regional_diagnosis_metric(
      "regional_dropped_local_routability_diagnoses",
      regional == nullptr ? 0U : regional->dropped_local_routability_diagnoses);
  print_regional_diagnosis_metric(
      "regional_diagnosis_directed_spline_selections",
      regional == nullptr ? 0U : regional->diagnosis_directed_spline_selections);
  print_regional_diagnosis_metric(
      "regional_diagnosis_directed_template_changes",
      regional == nullptr ? 0U : regional->diagnosis_directed_template_changes);
  print_regional_diagnosis_metric("regional_diagnosis_directed_via_changes",
                                  regional == nullptr ? 0U
                                                      : regional->diagnosis_directed_via_changes);
  print_regional_diagnosis_metric(
      "regional_diagnosis_directed_lane_order_changes",
      regional == nullptr ? 0U : regional->diagnosis_directed_lane_order_changes);
  print_regional_diagnosis_metric("regional_quantized_segment_pair_checks",
                                  regional == nullptr ? 0U
                                                      : regional->quantized_segment_pair_checks);
  print_regional_diagnosis_metric("regional_quantized_clearance_conflicts",
                                  regional == nullptr ? 0U
                                                      : regional->quantized_clearance_conflicts);
  print_regional_diagnosis_metric("regional_quantization_feedback_attempts",
                                  regional == nullptr ? 0U
                                                      : regional->quantization_feedback_attempts);
  print_regional_diagnosis_metric("regional_quantization_feedback_accepted",
                                  regional == nullptr ? 0U
                                                      : regional->quantization_feedback_accepted);
  print_regional_diagnosis_metric(
      "regional_quantization_feedback_windows_legalized",
      regional == nullptr ? 0U : regional->quantization_feedback_windows_legalized);
  print_regional_diagnosis_metric(
      "regional_quantization_check_budget_exhausted_windows",
      regional == nullptr ? 0U : regional->quantization_check_budget_exhausted_windows);
  print_regional_diagnosis_metric("regional_xpbd_windows_attempted",
                                  regional == nullptr ? 0U : regional->xpbd_windows_attempted);
  print_regional_diagnosis_metric(
      "regional_xpbd_unconflicted_candidates_skipped",
      regional == nullptr ? 0U : regional->xpbd_unconflicted_candidates_skipped);
  print_regional_diagnosis_metric("regional_xpbd_windows_improved",
                                  regional == nullptr ? 0U : regional->xpbd_windows_improved);
  print_regional_diagnosis_metric("regional_xpbd_windows_legalized",
                                  regional == nullptr ? 0U : regional->xpbd_windows_legalized);
  print_regional_diagnosis_metric("regional_xpbd_gpu_invocations",
                                  regional == nullptr ? 0U : regional->xpbd_gpu_invocations);
  print_regional_diagnosis_metric("regional_xpbd_cpu_fallbacks",
                                  regional == nullptr ? 0U : regional->xpbd_cpu_fallbacks);
  print_regional_diagnosis_metric("regional_xpbd_topology_rejections",
                                  regional == nullptr ? 0U : regional->xpbd_topology_rejections);
  print_regional_diagnosis_metric("regional_xpbd_pressure_cells",
                                  regional == nullptr ? 0U : regional->xpbd_pressure_cells);
  print_regional_diagnosis_metric("regional_xpbd_terrain_cells_sampled",
                                  regional == nullptr ? 0U : regional->xpbd_terrain_cells_sampled);
  print_regional_diagnosis_metric("regional_xpbd_obstacle_segments",
                                  regional == nullptr ? 0U : regional->xpbd_obstacle_segments);
  print_regional_diagnosis_metric(
      "regional_xpbd_obstacle_segment_budget_exhaustions",
      regional == nullptr ? 0U : regional->xpbd_obstacle_segment_budget_exhaustions);
  print_regional_diagnosis_metric("regional_xpbd_constraints",
                                  regional == nullptr ? 0U : regional->xpbd_constraints);
  print_regional_diagnosis_metric("regional_xpbd_broadphase_cells",
                                  regional == nullptr ? 0U : regional->xpbd_broadphase_cells);
  print_regional_diagnosis_metric("regional_xpbd_broadphase_pair_emissions",
                                  regional == nullptr ? 0U
                                                      : regional->xpbd_broadphase_pair_emissions);
  print_regional_diagnosis_metric(
      "regional_xpbd_resident_input_buffer_reuses",
      regional == nullptr ? 0U : regional->xpbd_resident_input_buffer_reuses);
  print_regional_diagnosis_metric("regional_xpbd_gpu_broadphase_invocations",
                                  regional == nullptr ? 0U
                                                      : regional->xpbd_gpu_broadphase_invocations);
  print_regional_diagnosis_metric("regional_xpbd_broadphase_fallbacks",
                                  regional == nullptr ? 0U : regional->xpbd_broadphase_fallbacks);
  print_regional_diagnosis_metric(
      "regional_xpbd_gpu_pressure_field_invocations",
      regional == nullptr ? 0U : regional->xpbd_gpu_pressure_field_invocations);
  print_regional_diagnosis_metric("regional_xpbd_unresolved_trace_pairs",
                                  regional == nullptr ? 0U : regional->xpbd_unresolved_trace_pairs);
  print_regional_diagnosis_metric("regional_xpbd_initial_violations",
                                  regional == nullptr ? 0U : regional->xpbd_initial_violations);
  print_regional_diagnosis_metric("regional_xpbd_final_violations",
                                  regional == nullptr ? 0U : regional->xpbd_final_violations);
  print_regional_diagnosis_metric("regional_xpbd_peak_initial_residual_nm",
                                  regional == nullptr
                                      ? 0U
                                      : static_cast<std::size_t>(std::max<std::int64_t>(
                                            0, regional->xpbd_peak_initial_residual.value())));
  print_regional_diagnosis_metric("regional_xpbd_peak_final_residual_nm",
                                  regional == nullptr
                                      ? 0U
                                      : static_cast<std::size_t>(std::max<std::int64_t>(
                                            0, regional->xpbd_peak_final_residual.value())));
  print_regional_diagnosis_metric("regional_xpbd_broadphase_us",
                                  regional == nullptr ? 0U : regional->xpbd_broadphase_us);
  print_regional_diagnosis_metric("regional_xpbd_constraint_generation_us",
                                  regional == nullptr ? 0U
                                                      : regional->xpbd_constraint_generation_us);
  print_regional_diagnosis_metric("regional_xpbd_solve_us",
                                  regional == nullptr ? 0U : regional->xpbd_solve_us);
  print_regional_diagnosis_metric("regional_xpbd_stress_reduction_us",
                                  regional == nullptr ? 0U : regional->xpbd_stress_reduction_us);
  print_regional_diagnosis_metric("regional_xpbd_readback_us",
                                  regional == nullptr ? 0U : regional->xpbd_readback_us);
  std::cout << ",\n  \"regional_xpbd_peak_pressure\": ";
  if (regional == nullptr)
    std::cout << "null";
  else
    std::cout << static_cast<double>(regional->xpbd_peak_pressure);
  std::cout << ",\n  \"regional_xpbd_obstacle_stress_nm\": ";
  if (regional == nullptr)
    std::cout << "null";
  else
    std::cout << static_cast<double>(regional->xpbd_stress.obstacle_nm);
  std::cout << ",\n  \"regional_xpbd_trace_clearance_stress_nm\": ";
  if (regional == nullptr)
    std::cout << "null";
  else
    std::cout << static_cast<double>(regional->xpbd_stress.trace_clearance_nm);
  std::cout << ",\n  \"regional_xpbd_stretch_bend_stress_nm\": ";
  if (regional == nullptr)
    std::cout << "null";
  else
    std::cout << static_cast<double>(regional->xpbd_stress.stretch_bend_nm);
  std::cout << ",\n  \"regional_xpbd_corridor_stress_nm\": ";
  if (regional == nullptr)
    std::cout << "null";
  else
    std::cout << static_cast<double>(regional->xpbd_stress.corridor_nm);
  std::cout << ",\n  \"regional_xpbd_via_stress_nm\": ";
  if (regional == nullptr)
    std::cout << "null";
  else
    std::cout << static_cast<double>(regional->xpbd_stress.via_nm);
  std::cout << ",\n  \"regional_peak_quantized_clearance_deficit_nm\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->peak_quantized_clearance_deficit.value();
  }
  std::cout << ",\n  \"regional_joint_spline_failed_improved_window_ids\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << '[';
    for (std::size_t index = 0U; index < regional->joint_spline_failed_improved_window_ids.size();
         ++index) {
      std::cout << (index == 0U ? "" : ",")
                << regional->joint_spline_failed_improved_window_ids[index];
    }
    std::cout << ']';
  }
  std::cout << ",\n  \"regional_alternative_domain_rejections\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->alternative_domain_rejections;
  }
  std::cout << ",\n  \"regional_alternative_navmesh_rejections\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->alternative_navmesh_rejections;
  }
  std::cout << ",\n  \"regional_alternative_runup_rejections\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->alternative_runup_rejections;
  }
  std::cout << ",\n  \"regional_alternative_via_span_rejections\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->alternative_via_span_rejections;
  }
  std::cout << ",\n  \"regional_alternative_topology_cell_rejections\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->alternative_topology_cell_rejections;
  }
  std::cout << ",\n  \"regional_alternative_priority_windows\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->alternative_priority_windows;
  }
  std::cout << ",\n  \"regional_alternative_residual_priority_windows\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->alternative_residual_priority_windows;
  }
  std::cout << ",\n  \"regional_alternative_residual_followup_checks\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->alternative_residual_followup_checks;
  }
  std::cout << ",\n  \"regional_same_layer_detour_checks\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->same_layer_detour_checks;
  }
  std::cout << ",\n  \"regional_accepted_same_layer_detours\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->accepted_same_layer_detours;
  }
  std::cout << ",\n  \"regional_unmapped_shadow_conflicts\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->unmapped_clearance_shadow_conflicts;
  }
  std::cout << ",\n  \"regional_unmapped_capacity_conflicts\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->unmapped_capacity_conflicts;
  }
  std::cout << ",\n  \"regional_total_overlap_length_nm\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->total_observed_overlap_length_nm;
  }
  std::cout << ",\n  \"regional_peak_clearance_deficit_nm\": ";
  if (regional == nullptr) {
    std::cout << "null";
  } else {
    std::cout << regional->peak_clearance_deficit.value();
  }
  std::cout << ",\n  \"telemetry_produced\": ";
  if (telemetry == nullptr) {
    std::cout << "null";
  } else {
    std::cout << telemetry->produced;
  }
  std::cout << ",\n  \"telemetry_consumed\": ";
  if (telemetry == nullptr) {
    std::cout << "null";
  } else {
    std::cout << telemetry->consumed;
  }
  std::cout << ",\n  \"telemetry_dropped\": ";
  if (telemetry == nullptr) {
    std::cout << "null";
  } else {
    std::cout << telemetry->dropped;
  }
  std::cout << ",\n  \"telemetry_snapshots\": ";
  if (telemetry == nullptr) {
    std::cout << "null";
  } else {
    std::cout << telemetry->snapshots;
  }
  std::cout << ",\n  \"telemetry_websocket_messages\": ";
  if (telemetry == nullptr) {
    std::cout << "null";
  } else {
    std::cout << telemetry->websocket_messages;
  }
  std::cout << ",\n  \"telemetry_websocket_dropped\": ";
  if (telemetry == nullptr) {
    std::cout << "null";
  } else {
    std::cout << telemetry->websocket_dropped;
  }
  std::cout << ",\n  \"telemetry_transport_failures\": ";
  if (telemetry == nullptr) {
    std::cout << "null";
  } else {
    std::cout << telemetry->transport_failures;
  }
  std::cout << ",\n  \"telemetry_recorder_failures\": ";
  if (telemetry == nullptr) {
    std::cout << "null";
  } else {
    std::cout << telemetry->recorder_failures;
  }
  std::cout << ",\n  \"global_feedback_passes\": ";
  if (routing_plan == nullptr) {
    std::cout << "null";
  } else {
    std::cout << routing_plan->global_feedback_passes;
  }
  std::cout << ",\n  \"global_feedback_epochs\": ";
  if (routing_plan == nullptr) {
    std::cout << "null";
  } else {
    std::cout << routing_plan->global_feedback_epochs;
  }
  std::cout << ",\n  \"cumulative_global_iterations\": ";
  if (routing_plan == nullptr) {
    std::cout << "null";
  } else {
    std::cout << routing_plan->cumulative_global_iterations;
  }
  std::cout << ",\n  \"retained_global_portfolio_peak\": ";
  if (routing_plan == nullptr) {
    std::cout << "null";
  } else {
    std::cout << routing_plan->retained_global_portfolio_peak;
  }
  std::cout << ",\n  \"retained_global_portfolios_evaluated\": ";
  if (routing_plan == nullptr) {
    std::cout << "null";
  } else {
    std::cout << routing_plan->retained_global_portfolios_evaluated;
  }
  std::cout << ",\n  \"intermediate_regional_probes\": ";
  if (routing_plan == nullptr) {
    std::cout << "null";
  } else {
    std::cout << routing_plan->intermediate_regional_probes;
  }
  std::cout << ",\n  \"routing_plan_complete\": ";
  if (routing_plan == nullptr) {
    std::cout << "null";
  } else {
    std::cout << (routing_plan->complete() ? "true" : "false");
  }
  std::cout << ",\n  \"regional_feedback_passes\": ";
  if (routing_plan == nullptr) {
    std::cout << "null";
  } else {
    std::cout << routing_plan->regional_feedback_passes;
  }
  std::cout << ",\n  \"cheap_physical_probes\": ";
  if (routing_plan == nullptr) {
    std::cout << "null";
  } else {
    std::cout << routing_plan->cheap_physical_probes;
  }
  std::cout << ",\n  \"routability_feedback_budget_exhausted\": ";
  if (routing_plan == nullptr) {
    std::cout << "null";
  } else {
    std::cout << (routing_plan->routability_feedback_budget_exhausted ? "true" : "false");
  }
  std::cout << ",\n  \"routability_feedback_probes\": ";
  if (routing_plan == nullptr) {
    std::cout << "null";
  } else {
    std::cout << '[';
    for (std::size_t index = 0U; index < routing_plan->feedback_probes.size(); ++index) {
      const auto& probe = routing_plan->feedback_probes[index];
      std::cout << (index == 0U ? "" : ",") << "{\"pass\":" << probe.pass
                << ",\"global_epoch_iterations\":" << probe.global_epoch_iterations
                << ",\"cumulative_global_iterations\":" << probe.cumulative_global_iterations
                << ",\"retained_global_portfolios\":" << probe.retained_global_portfolios
                << ",\"intermediate_regional_probe\":"
                << (probe.intermediate_regional_probe ? "true" : "false")
                << ",\"history_entries_before\":" << probe.history_entries_before
                << ",\"history_pressure_before\":" << probe.history_pressure_before
                << ",\"global_diagnoses\":" << probe.global_diagnoses
                << ",\"history_entries_after_global\":" << probe.history_entries_after_global
                << ",\"history_pressure_after_global\":" << probe.history_pressure_after_global
                << ",\"candidates\":[";
      for (std::size_t candidate_index = 0U; candidate_index < probe.candidates.size();
           ++candidate_index) {
        const auto& candidate = probe.candidates[candidate_index];
        std::cout << (candidate_index == 0U ? "" : ",") << "{\"history_informed_global\":"
                  << (candidate.history_informed_global ? "true" : "false")
                  << ",\"incremental_global\":" << (candidate.incremental_global ? "true" : "false")
                  << ",\"feedback_fell_back_to_full_global\":"
                  << (candidate.feedback_fell_back_to_full_global ? "true" : "false")
                  << ",\"feedback_affected_nets\":" << candidate.feedback_affected_nets
                  << ",\"global_reused_nets\":" << candidate.global_reused_nets
                  << ",\"global_rerouted_nets\":" << candidate.global_rerouted_nets
                  << ",\"global_unrouted_nets\":" << candidate.global_unrouted_nets
                  << ",\"global_total_overflow\":" << candidate.global_total_overflow
                  << ",\"global_clearance_shadow_conflicts\":"
                  << candidate.global_clearance_shadow_conflicts
                  << ",\"global_unaccepted_lane_transitions\":"
                  << candidate.global_unaccepted_lane_transitions
                  << ",\"coarse_topology_complete\":"
                  << (candidate.coarse_topology_complete ? "true" : "false")
                  << ",\"regional_consulted\":" << (candidate.regional_consulted ? "true" : "false")
                  << ",\"regional_incomplete_bundle_assignments\":"
                  << candidate.regional_incomplete_bundle_assignments
                  << ",\"regional_diagnoses\":" << candidate.regional_diagnoses
                  << ",\"plan_complete\":" << (candidate.plan_complete ? "true" : "false") << '}';
      }
      std::cout << ']'
                << ",\"history_entries_after_regional\":" << probe.history_entries_after_regional
                << ",\"history_pressure_after_regional\":" << probe.history_pressure_after_regional
                << ",\"next_feedback_uses_repair_branch\":"
                << (probe.next_feedback_uses_repair_branch ? "true" : "false")
                << ",\"next_feedback_affected_nets\":" << probe.next_feedback_affected_nets << '}';
    }
    std::cout << ']';
  }
  std::cout << ",\n  \"routability_diagnoses\": ";
  if (routing_plan == nullptr) {
    std::cout << "null";
  } else {
    std::cout << '[';
    for (std::size_t index = 0U; index < routing_plan->routability_knowledge.diagnoses.size();
         ++index) {
      const auto& diagnosis = routing_plan->routability_knowledge.diagnoses[index];
      std::cout << (index == 0U ? "" : ",")
                << "{\"kind\":" << json_quote(flux::to_string(diagnosis.kind))
                << ",\"stage\":" << json_quote(flux::to_string(diagnosis.stage))
                << ",\"resource\":" << json_quote(diagnosis.resource.stable_id)
                << ",\"layer\":" << json_quote(diagnosis.resource.layer.value())
                << ",\"severity\":" << diagnosis.severity
                << ",\"clearance_deficit_nm\":" << diagnosis.clearance_deficit.value()
                << ",\"relaxation\":" << json_quote(flux::to_string(diagnosis.suggested_relaxation))
                << ",\"contributes_to_history\":"
                << (diagnosis.contributes_to_history ? "true" : "false") << ",\"nets\":";
      print_net_ids(diagnosis.resource.nets);
      std::cout << '}';
    }
    std::cout << ']';
  }
  std::cout << ",\n  \"diagnostic\": " << json_quote(result.diagnostic) << "\n}\n";
}

struct ExportSummary {
  bool written{false};
  std::size_t errors{0};
  std::size_t generated_ids{0};
  std::vector<std::string> issue_codes;
  bool preview_only{false};
};

void print_detailed_result(const flux::DetailedRouteResult& result,
                           std::optional<ExportSummary> exported = std::nullopt,
                           std::optional<std::string_view> workflow = std::nullopt,
                           const flux::GlobalRouteResult* global = nullptr) {
  const auto pair_gap_violations = static_cast<std::size_t>(std::count_if(
      result.copper_drc.violations.begin(), result.copper_drc.violations.end(),
      [](const auto& violation) { return violation.code == "differential_pair_gap"; }));
  const auto pair_skew_violations = static_cast<std::size_t>(std::count_if(
      result.copper_drc.violations.begin(), result.copper_drc.violations.end(),
      [](const auto& violation) { return violation.code == "differential_pair_skew"; }));
  std::map<std::string, std::size_t> copper_violation_counts;
  for (const auto& violation : result.copper_drc.violations) {
    ++copper_violation_counts[violation.code];
  }
  const auto attributed_clearance_shadow_conflicts =
      global == nullptr ? 0U
                        : static_cast<std::size_t>(std::count_if(
                              global->clearance_shadow_conflicts.begin(),
                              global->clearance_shadow_conflicts.end(), [](const auto& conflict) {
                                return conflict.shadow_owner_first.has_value() &&
                                       conflict.shadow_owner_second.has_value();
                              }));
  const auto residual_summary =
      global == nullptr ? ResidualConflictSummary{} : summarize_residual_conflicts(*global);
  std::cout << "{\n"
            << "  \"workflow\": " << (workflow ? json_quote(*workflow) : "null") << ",\n"
            << "  \"complete\": " << (result.complete() ? "true" : "false") << ",\n"
            << "  \"provisional_complete\": "
            << (result.provisional_zone_refill_candidate() ? "true" : "false") << ",\n"
            << "  \"zone_refill_required\": "
            << (result.stop_reason == flux::DetailedRouterStopReason::zone_refill_required
                    ? "true"
                    : "false")
            << ",\n"
            << "  \"stop_reason\": \"" << stop_reason_name(result.stop_reason) << "\",\n"
            << "  \"candidate_available\": " << (result.candidate ? "true" : "false") << ",\n"
            << "  \"global_complete\": "
            << (global == nullptr    ? "null"
                : global->complete() ? "true"
                                     : "false")
            << ",\n"
            << "  \"global_stop_reason\": "
            << (global == nullptr ? "null" : json_quote(stop_reason_name(global->stop_reason)))
            << ",\n"
            << "  \"global_diagnostic\": "
            << (global == nullptr ? "null" : json_quote(global->diagnostic)) << ",\n"
            << "  \"global_iterations\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->iterations;
  }
  std::cout << ",\n  \"configured_priority_net_ids\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    print_net_ids(global->configured_priority_nets);
  }
  std::cout << ",\n  \"configured_crossing_safe_diagonal_grid_edges\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << (global->configured_crossing_safe_diagonal_grid_edges ? "true" : "false");
  }
  std::cout << ",\n  \"configured_sampled_route_clearance_reservations\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << (global->configured_sampled_route_clearance_reservations ? "true" : "false");
  }
  std::cout << ",\n  \"configured_single_lane_direct_voronoi_resources\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << (global->configured_single_lane_direct_voronoi_resources ? "true" : "false");
  }
  std::cout << ",\n  \"static_seed_searches\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->static_seed_searches;
  }
  std::cout << ",\n  \"static_seed_connections\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->static_seed_connections;
  }
  std::cout << ",\n  \"static_seed_candidates\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->static_seed_candidates;
  }
  std::cout << ",\n  \"static_seed_terminals_without_seed\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->static_seed_terminals_without_seed;
  }
  std::cout << ",\n  \"static_seed_reprojections_accepted\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->static_seed_reprojections_accepted;
  }
  std::cout << ",\n  \"static_seed_corridor_nodes\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->static_seed_corridor_nodes;
  }
  std::cout << ",\n  \"static_seed_spline_samples\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->static_seed_spline_samples;
  }
  std::cout << ",\n  \"static_seed_navmesh_routes\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->static_seed_navmesh_routes;
  }
  std::cout << ",\n  \"static_seed_navmesh_refined_parents\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->static_seed_navmesh_refined_parents;
  }
  std::cout << ",\n  \"static_seed_navmesh_refined_leaves\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->static_seed_navmesh_refined_leaves;
  }
  std::cout << ",\n  \"static_seed_navmesh_rejected_leaves\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->static_seed_navmesh_rejected_leaves;
  }
  std::cout << ",\n  \"static_seed_navmesh_gpu_wavefront_routes\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->static_seed_navmesh_gpu_wavefront_routes;
  }
  std::cout << ",\n  \"static_seed_navmesh_gpu_wavefront_cpu_fallbacks\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->static_seed_navmesh_gpu_wavefront_cpu_fallbacks;
  }
  std::cout << ",\n  \"static_seed_flood_fill_routes\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->static_seed_flood_fill_routes;
  }
  std::cout << ",\n  \"static_seed_flood_fill_cells_visited\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->static_seed_flood_fill_cells_visited;
  }
  std::cout << ",\n  \"static_seed_navmesh_failures\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->static_seed_navmesh_failures;
  }
  std::cout << ",\n  \"static_seed_navmesh_certificate_domains\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->static_seed_navmesh_certificate_domains;
  }
  std::cout << ",\n  \"static_seed_trajectory_accepted\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->static_seed_trajectory_accepted;
  }
  std::cout << ",\n  \"static_seed_trajectory_rejected\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->static_seed_trajectory_rejected;
  }
  std::cout << ",\n  \"negotiated_spline_guided_searches\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->negotiated_spline_guided_searches;
  }
  std::cout << ",\n  \"negotiated_spline_corridor_departures\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->negotiated_spline_corridor_departures;
  }
  std::cout << ",\n  \"static_seed_budget_exhausted\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << (global->static_seed_budget_exhausted ? "true" : "false");
  }
  std::cout << ",\n  \"iteration_zero_paths\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->iteration_zero_paths;
  }
  std::cout << ",\n  \"iteration_zero_unrouted_nets\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->iteration_zero_unrouted_nets;
  }
  std::cout << ",\n  \"iteration_zero_total_overflow\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->iteration_zero_total_overflow;
  }
  std::cout << ",\n  \"iteration_zero_seed_history_increments\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->iteration_zero_seed_history_increments;
  }
  std::cout << ",\n  \"global_unrouted_nets\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->unrouted_nets.size();
  }
  std::cout << ",\n  \"global_unrouted_net_ids\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    print_net_ids(global->unrouted_nets);
  }
  std::cout << ",\n  \"global_connection_failures\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    print_routing_connection_failures(global->connection_failures);
  }
  std::cout << ",\n  \"global_selected_net_pressure\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    print_routing_net_pressure(global->selected_net_pressure);
  }
  std::cout << ",\n  \"global_congested_sampled_resource_usage\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    print_congested_sampled_resource_usage(global->resource_usage);
  }
  std::cout << ",\n  \"global_total_overflow\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->total_overflow;
  }
  std::cout << ",\n  \"global_sampled_overflow\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->sampled_overflow;
  }
  std::cout << ",\n  \"global_corridor_overflow\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->corridor_overflow;
  }
  std::cout << ",\n  \"global_clearance_shadow_conflicts\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->clearance_shadow_conflicts.size();
  }
  std::cout << ",\n  \"global_clearance_shadow_owner_attributed_conflicts\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << attributed_clearance_shadow_conflicts;
  }
  std::cout << ",\n  \"global_clearance_shadow_overflow\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->clearance_shadow_overflow;
  }
  std::cout << ",\n  \"global_clearance_shadow_history_increment\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->clearance_shadow_history_increment;
  }
  std::cout << ",\n  \"global_peak_clearance_shadow_history_increment\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->peak_clearance_shadow_history_increment;
  }
  std::cout << ",\n  \"global_clearance_shadow_owner_history_phase_attempted\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << (global->clearance_shadow_owner_history_phase_attempted ? "true" : "false");
  }
  std::cout << ",\n  \"global_clearance_shadow_owner_history_passes\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->clearance_shadow_owner_history_passes;
  }
  std::cout << ",\n  \"global_residual_conflict_groups\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->residual_conflict_groups.size();
  }
  std::cout << ",\n  \"global_peak_residual_conflict_group_nets\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << residual_summary.peak_nets;
  }
  std::cout << ",\n  \"global_peak_residual_group_sampled_overflow\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << residual_summary.peak_sampled_overflow;
  }
  std::cout << ",\n  \"global_peak_residual_group_clearance_shadow_overflow\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << residual_summary.peak_clearance_shadow_overflow;
  }
  std::cout << ",\n  \"global_topology_preserved_residual_conflict_groups\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << residual_summary.topology_preserved_groups;
  }
  std::cout << ",\n  \"global_single_layer_residual_conflict_groups\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << residual_summary.single_layer_groups;
  }
  std::cout << ",\n  \"global_post_negotiation_cone_repair_attempts\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->post_negotiation_cone_repair_attempts;
  }
  std::cout << ",\n  \"global_post_negotiation_cone_repairs_accepted\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->post_negotiation_cone_repairs_accepted;
  }
  std::cout << ",\n  \"global_post_negotiation_cone_repair_nets_attempted\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->post_negotiation_cone_repair_nets_attempted;
  }
  std::cout << ",\n  \"global_physical_overflow_nm\": ";
  if (global == nullptr) {
    std::cout << "null";
  } else {
    std::cout << global->total_voronoi_overflow_nm;
  }
  std::cout
      << ",\n"
      << "  \"added_tracks\": " << result.added_tracks << ",\n"
      << "  \"added_track_arcs\": " << result.added_track_arcs << ",\n"
      << "  \"unrounded_non_junction_bends\": " << result.unrounded_non_junction_bends << ",\n"
      << "  \"added_vias\": " << result.added_vias << ",\n"
      << "  \"removed_tracks\": " << result.removed_tracks << ",\n"
      << "  \"removed_track_arcs\": " << result.removed_track_arcs << ",\n"
      << "  \"removed_vias\": " << result.removed_vias << ",\n"
      << "  \"reused_tracks\": " << result.reused_tracks << ",\n"
      << "  \"reused_track_arcs\": " << result.reused_track_arcs << ",\n"
      << "  \"reused_vias\": " << result.reused_vias << ",\n"
      << "  \"pulled_tight_corners\": " << result.pulled_tight_corners << ",\n"
      << "  \"continuously_refined_points\": " << result.continuously_refined_points << ",\n"
      << "  \"continuous_refinement_iterations\": " << result.continuous_refinement_iterations
      << ",\n"
      << "  \"attempted_continuous_refinement_candidates\": "
      << result.attempted_continuous_refinement_candidates << ",\n"
      << "  \"accepted_continuous_refinement_candidates\": "
      << result.accepted_continuous_refinement_candidates << ",\n"
      << "  \"rejected_continuous_refinement_candidates\": "
      << result.rejected_continuous_refinement_candidates << ",\n"
      << "  \"withheld_continuous_refinement_candidates\": "
      << result.withheld_continuous_refinement_candidates << ",\n"
      << "  \"continuous_refinement_budget_exhausted\": "
      << (result.continuous_refinement_budget_exhausted ? "true" : "false") << ",\n"
      << "  \"materialized_terminal_neck_down_paths\": "
      << result.materialized_terminal_neck_down_paths << ",\n"
      << "  \"materialized_terminal_neck_down_segments\": "
      << result.materialized_terminal_neck_down_segments << ",\n"
      << "  \"materialized_medial_neck_down_paths\": " << result.materialized_medial_neck_down_paths
      << ",\n"
      << "  \"materialized_medial_neck_down_segments\": "
      << result.materialized_medial_neck_down_segments << ",\n"
      << "  \"materialized_corridor_neck_down_paths\": "
      << result.materialized_corridor_neck_down_paths << ",\n"
      << "  \"materialized_corridor_neck_down_segments\": "
      << result.materialized_corridor_neck_down_segments << ",\n"
      << "  \"width_aware_simplified_points\": " << result.width_aware_simplified_points << ",\n"
      << "  \"width_aware_pulled_tight_paths\": " << result.width_aware_pulled_tight_paths << ",\n"
      << "  \"width_aware_pulled_tight_corners\": " << result.width_aware_pulled_tight_corners
      << ",\n"
      << "  \"width_aware_continuously_refined_paths\": "
      << result.width_aware_continuously_refined_paths << ",\n"
      << "  \"width_aware_continuously_refined_points\": "
      << result.width_aware_continuously_refined_points << ",\n"
      << "  \"attempted_topology_locked_paths\": " << result.attempted_topology_locked_paths
      << ",\n"
      << "  \"materialized_topology_locked_paths\": " << result.materialized_topology_locked_paths
      << ",\n"
      << "  \"topology_locked_rejection_copper_violations\": "
      << result.topology_locked_rejection_copper_violations << ",\n"
      << "  \"topology_locked_rejection_first_violation\": "
      << json_quote(result.topology_locked_rejection_first_violation) << ",\n"
      << "  \"materialized_multilayer_topology_locked_paths\": "
      << result.materialized_multilayer_topology_locked_paths << ",\n"
      << "  \"topology_locked_path_fallbacks\": " << result.topology_locked_path_fallbacks << ",\n"
      << "  \"preserved_topology_locked_terminal_access_paths\": "
      << result.preserved_topology_locked_terminal_access_paths << ",\n"
      << "  \"attempted_coupled_terminal_fanout_groups\": "
      << result.attempted_coupled_terminal_fanout_groups << ",\n"
      << "  \"attempted_coupled_terminal_fanout_candidates\": "
      << result.attempted_coupled_terminal_fanout_candidates << ",\n"
      << "  \"materialized_coupled_terminal_fanout_groups\": "
      << result.materialized_coupled_terminal_fanout_groups << ",\n"
      << "  \"synchronized_coupled_terminal_fanouts\": "
      << result.synchronized_coupled_terminal_fanouts << ",\n"
      << "  \"coupled_terminal_fanout_fallbacks\": " << result.coupled_terminal_fanout_fallbacks
      << ",\n"
      << "  \"attempted_shared_resource_fanout_groups\": "
      << result.attempted_shared_resource_fanout_groups << ",\n"
      << "  \"attempted_shared_resource_fanout_candidates\": "
      << result.attempted_shared_resource_fanout_candidates << ",\n"
      << "  \"materialized_shared_resource_fanout_groups\": "
      << result.materialized_shared_resource_fanout_groups << ",\n"
      << "  \"synchronized_shared_resource_fanouts\": "
      << result.synchronized_shared_resource_fanouts << ",\n"
      << "  \"shared_resource_fanout_fallbacks\": " << result.shared_resource_fanout_fallbacks
      << ",\n"
      << "  \"shared_resource_fanout_failure_count\": "
      << result.shared_resource_fanout_failures.size() << ",\n"
      << "  \"attempted_coupled_multilayer_groups\": " << result.attempted_coupled_multilayer_groups
      << ",\n"
      << "  \"materialized_coupled_multilayer_groups\": "
      << result.materialized_coupled_multilayer_groups << ",\n"
      << "  \"synchronized_coupled_via_arrays\": " << result.synchronized_coupled_via_arrays
      << ",\n"
      << "  \"coupled_multilayer_group_fallbacks\": " << result.coupled_multilayer_group_fallbacks
      << ",\n"
      << "  \"configured_maximum_circular_arc_radius_nm\": "
      << result.materialized_config.maximum_circular_arc_radius.value() << ",\n"
      << "  \"configured_circular_arc_refinement\": "
      << (result.materialized_config.circular_arc_refinement ? "true" : "false") << ",\n"
      << "  \"configured_spline_tessellation_tolerance_nm\": "
      << result.materialized_config.spline_tessellation_tolerance.value() << ",\n"
      << "  \"configured_spline_arc_fit_tolerance_nm\": "
      << result.materialized_config.spline_arc_fit_tolerance.value() << ",\n"
      << "  \"configured_spline_control_corridor_nm\": "
      << result.materialized_config.spline_control_corridor.value() << ",\n"
      << "  \"configured_max_pns_propagation_depth\": "
      << result.materialized_config.max_pns_propagation_depth << ",\n"
      << "  \"configured_max_pns_candidate_checks\": "
      << result.materialized_config.max_pns_candidate_checks << ",\n"
      << "  \"configured_circular_arc_backtracking_steps\": "
      << result.materialized_config.circular_arc_backtracking_steps << ",\n"
      << "  \"configured_max_circular_arc_candidate_checks\": "
      << result.materialized_config.max_circular_arc_candidate_checks << ",\n"
      << "  \"attempted_circular_arc_corners\": " << result.attempted_circular_arc_corners << ",\n"
      << "  \"attempted_circular_arc_candidates\": " << result.attempted_circular_arc_candidates
      << ",\n"
      << "  \"rejected_circular_arc_candidates\": " << result.rejected_circular_arc_candidates
      << ",\n"
      << "  \"fitted_circular_arcs\": " << result.fitted_circular_arcs << ",\n"
      << "  \"radius_reduced_circular_arcs\": " << result.radius_reduced_circular_arcs << ",\n"
      << "  \"withheld_circular_arc_corners\": " << result.withheld_circular_arc_corners << ",\n"
      << "  \"circular_arc_candidate_budget_exhausted\": "
      << (result.circular_arc_candidate_budget_exhausted ? "true" : "false") << ",\n"
      << "  \"width_aware_circular_arc_paths\": " << result.width_aware_circular_arc_paths << ",\n"
      << "  \"width_aware_fitted_circular_arcs\": " << result.width_aware_fitted_circular_arcs
      << ",\n"
      << "  \"attempted_quintic_ph_runs\": " << result.attempted_quintic_ph_runs << ",\n"
      << "  \"fitted_quintic_ph_runs\": " << result.fitted_quintic_ph_runs << ",\n"
      << "  \"adaptive_spline_tessellation_samples\": "
      << result.adaptive_spline_tessellation_samples << ",\n"
      << "  \"spline_fitted_track_segments\": " << result.spline_fitted_track_segments << ",\n"
      << "  \"spline_collision_envelopes\": " << result.spline_collision_envelopes << ",\n"
      << "  \"spline_navmesh_routes\": " << result.spline_navmesh_routes << ",\n"
      << "  \"spline_navmesh_failures\": " << result.spline_navmesh_failures << ",\n"
      << "  \"spline_navmesh_refined_parents\": " << result.spline_navmesh_refined_parents << ",\n"
      << "  \"spline_navmesh_refined_leaves\": " << result.spline_navmesh_refined_leaves << ",\n"
      << "  \"spline_navmesh_rejected_leaves\": " << result.spline_navmesh_rejected_leaves << ",\n"
      << "  \"spline_navmesh_gpu_wavefront_routes\": " << result.spline_navmesh_gpu_wavefront_routes
      << ",\n"
      << "  \"spline_navmesh_gpu_wavefront_cpu_fallbacks\": "
      << result.spline_navmesh_gpu_wavefront_cpu_fallbacks << ",\n"
      << "  \"spline_navmesh_certificate_fallbacks\": "
      << result.spline_navmesh_certificate_fallbacks << ",\n"
      << "  \"spline_navmesh_topology_signatures\": " << result.spline_navmesh_topology_signatures
      << ",\n"
      << "  \"spline_navmesh_topology_gates\": " << result.spline_navmesh_topology_gates << ",\n"
      << "  \"spline_navmesh_topology_rejections\": " << result.spline_navmesh_topology_rejections
      << ",\n"
      << "  \"spline_navmesh_last_diagnostic\": "
      << json_quote(result.spline_navmesh_last_diagnostic) << ",\n"
      << "  \"spline_navmesh_last_topology_diagnostic\": "
      << json_quote(result.spline_navmesh_last_topology_diagnostic) << ",\n"
      << "  \"spline_funnel_points\": " << result.spline_funnel_points << ",\n"
      << "  \"spline_seed_candidates\": " << result.spline_seed_candidates << ",\n"
      << "  \"spline_seed_exact_legal\": " << result.spline_seed_exact_legal << ",\n"
      << "  \"spline_seed_portfolio_improvements\": " << result.spline_seed_portfolio_improvements
      << ",\n"
      << "  \"spline_trajectory_attempts\": " << result.spline_trajectory_attempts << ",\n"
      << "  \"spline_trajectory_accepted\": " << result.spline_trajectory_accepted << ",\n"
      << "  \"spline_trajectory_control_moves\": " << result.spline_trajectory_control_moves
      << ",\n"
      << "  \"spline_trajectory_candidate_checks\": " << result.spline_trajectory_candidate_checks
      << ",\n"
      << "  \"spline_multistart_attempts\": " << result.spline_multistart_attempts << ",\n"
      << "  \"spline_multistart_recoveries\": " << result.spline_multistart_recoveries << ",\n"
      << "  \"spline_gpu_precondition_attempts\": " << result.spline_gpu_precondition_attempts
      << ",\n"
      << "  \"spline_gpu_precondition_dispatches\": " << result.spline_gpu_precondition_dispatches
      << ",\n"
      << "  \"spline_gpu_precondition_cpu_fallbacks\": "
      << result.spline_gpu_precondition_cpu_fallbacks << ",\n"
      << "  \"spline_gpu_precondition_accepted\": " << result.spline_gpu_precondition_accepted
      << ",\n"
      << "  \"spline_gpu_precondition_legal\": " << result.spline_gpu_precondition_legal << ",\n"
      << "  \"spline_gpu_precondition_solve_us\": " << result.spline_gpu_precondition_solve_us
      << ",\n"
      << "  \"spline_gpu_polish_batches\": " << result.spline_gpu_polish_batches << ",\n"
      << "  \"spline_gpu_polish_dispatches\": " << result.spline_gpu_polish_dispatches << ",\n"
      << "  \"spline_gpu_polish_cpu_fallbacks\": " << result.spline_gpu_polish_cpu_fallbacks
      << ",\n"
      << "  \"spline_gpu_polish_candidates\": " << result.spline_gpu_polish_candidates << ",\n"
      << "  \"spline_gpu_polish_exact_shortlist_checks\": "
      << result.spline_gpu_polish_exact_shortlist_checks << ",\n"
      << "  \"spline_parallel_exact_waves\": " << result.spline_parallel_exact_waves << ",\n"
      << "  \"spline_parallel_exact_checks\": " << result.spline_parallel_exact_checks << ",\n"
      << "  \"spline_parallel_route_tasks\": " << result.spline_parallel_route_tasks << ",\n"
      << "  \"spline_parallel_route_workers\": " << result.spline_parallel_route_workers << ",\n"
      << "  \"spline_parallel_route_candidates\": " << result.spline_parallel_route_candidates
      << ",\n"
      << "  \"spline_parallel_route_commits\": " << result.spline_parallel_route_commits << ",\n"
      << "  \"spline_parallel_route_merge_rejections\": "
      << result.spline_parallel_route_merge_rejections << ",\n"
      << "  \"spline_trajectory_initial_violations\": "
      << result.spline_trajectory_initial_violations << ",\n"
      << "  \"spline_trajectory_final_violations\": " << result.spline_trajectory_final_violations
      << ",\n"
      << "  \"spline_trajectory_initial_shape_violations\": "
      << result.spline_trajectory_initial_shape_violations << ",\n"
      << "  \"spline_trajectory_final_shape_violations\": "
      << result.spline_trajectory_final_shape_violations << ",\n"
      << "  \"spline_trajectory_final_self_intersections\": "
      << result.spline_trajectory_final_self_intersections << ",\n"
      << "  \"spline_trajectory_final_cusps\": " << result.spline_trajectory_final_cusps << ",\n"
      << "  \"spline_trajectory_final_control_station_reversals\": "
      << result.spline_trajectory_final_control_station_reversals << ",\n"
      << "  \"spline_trajectory_final_station_reversals\": "
      << result.spline_trajectory_final_station_reversals << ",\n"
      << "  \"spline_trajectory_final_excess_turn_violations\": "
      << result.spline_trajectory_final_excess_turn_violations << ",\n"
      << "  \"spline_trajectory_final_unintended_inflections\": "
      << result.spline_trajectory_final_unintended_inflections << ",\n"
      << "  \"spline_trajectory_final_bend_radius_violations\": "
      << result.spline_trajectory_final_bend_radius_violations << ",\n"
      << "  \"spline_trajectory_initial_continuous_violations\": "
      << result.spline_trajectory_initial_continuous_violations << ",\n"
      << "  \"spline_trajectory_final_continuous_violations\": "
      << result.spline_trajectory_final_continuous_violations << ",\n"
      << "  \"spline_continuous_hulls_checked\": " << result.spline_continuous_hulls_checked
      << ",\n"
      << "  \"spline_continuous_envelope_rejections\": "
      << result.spline_continuous_envelope_rejections << ",\n"
      << "  \"spline_continuous_budget_exhaustions\": "
      << result.spline_continuous_budget_exhaustions << ",\n"
      << "  \"spline_final_validation_rejections\": " << result.spline_final_validation_rejections
      << ",\n"
      << "  \"spline_final_copper_rejections\": " << result.spline_final_copper_rejections << ",\n"
      << "  \"spline_final_connectivity_rejections\": "
      << result.spline_final_connectivity_rejections << ",\n"
      << "  \"spline_last_materialization_diagnostic\": "
      << json_quote(result.spline_last_materialization_diagnostic) << ",\n"
      << "  \"spline_route_failure_details\": "
      << json_string_array(result.spline_route_failure_details) << ",\n"
      << "  \"spline_control_point_shoves\": " << result.spline_control_point_shoves << ",\n"
      << "  \"spline_refits\": " << result.spline_refits << ",\n"
      << "  \"pns_shove_attempts\": " << result.pns_shove_attempts << ",\n"
      << "  \"pns_shove_candidates\": " << result.pns_shove_candidates << ",\n"
      << "  \"accepted_pns_shoves\": " << result.accepted_pns_shoves << ",\n"
      << "  \"pns_moved_tracks\": " << result.pns_moved_tracks << ",\n"
      << "  \"pns_moved_vias\": " << result.pns_moved_vias << ",\n"
      << "  \"pns_added_dogleg_tracks\": " << result.pns_added_dogleg_tracks << ",\n"
      << "  \"residual_conflict_handoff_attempted\": "
      << (result.residual_conflict_handoff_attempted ? "true" : "false") << ",\n"
      << "  \"residual_conflict_groups_attempted\": " << result.residual_conflict_groups_attempted
      << ",\n"
      << "  \"residual_conflict_nets_attempted\": " << result.residual_conflict_nets_attempted
      << ",\n"
      << "  \"residual_sampled_overflow_attempted\": " << result.residual_sampled_overflow_attempted
      << ",\n"
      << "  \"residual_clearance_shadow_overflow_attempted\": "
      << result.residual_clearance_shadow_overflow_attempted << ",\n"
      << "  \"residual_joint_candidate_states\": " << result.residual_joint_candidate_states
      << ",\n"
      << "  \"residual_joint_groups_optimized\": " << result.residual_joint_groups_optimized
      << ",\n"
      << "  \"residual_joint_spline_routes\": " << result.residual_joint_spline_routes << ",\n"
      << "  \"residual_joint_optimization_sweeps\": " << result.residual_joint_optimization_sweeps
      << ",\n"
      << "  \"residual_joint_control_moves\": " << result.residual_joint_control_moves << ",\n"
      << "  \"residual_joint_exact_transactions_rejected\": "
      << result.residual_joint_exact_transactions_rejected << ",\n"
      << "  \"residual_joint_transaction_accepted\": "
      << (result.residual_joint_transaction_accepted ? "true" : "false") << ",\n"
      << "  \"spline_candidate_budget_exhausted\": "
      << (result.spline_candidate_budget_exhausted ? "true" : "false") << ",\n"
      << "  \"pns_candidate_budget_exhausted\": "
      << (result.pns_candidate_budget_exhausted ? "true" : "false") << ",\n"
      << "  \"configured_differential_pair_skew_tuning_step_nm\": "
      << result.materialized_config.differential_pair_skew_tuning_step.value() << ",\n"
      << "  \"configured_maximum_differential_pair_skew_tuning_amplitude_nm\": "
      << result.materialized_config.maximum_differential_pair_skew_tuning_amplitude.value() << ",\n"
      << "  \"configured_minimum_differential_pair_skew_tuning_lead_nm\": "
      << result.materialized_config.minimum_differential_pair_skew_tuning_lead.value() << ",\n"
      << "  \"configured_minimum_differential_pair_skew_tuning_spacing_nm\": "
      << result.materialized_config.minimum_differential_pair_skew_tuning_spacing.value() << ",\n"
      << "  \"configured_max_differential_pair_skew_tuning_candidates\": "
      << result.materialized_config.max_differential_pair_skew_tuning_candidates << ",\n"
      << "  \"configured_max_differential_pair_skew_tuning_periods\": "
      << result.materialized_config.max_differential_pair_skew_tuning_periods << ",\n"
      << "  \"configured_differential_pair_skew_tuning_batch_assignment\": "
      << (result.materialized_config.differential_pair_skew_tuning_batch_assignment ? "true"
                                                                                    : "false")
      << ",\n"
      << "  \"configured_max_differential_pair_skew_tuning_assignment_candidates_per_pair\": "
      << result.materialized_config.max_differential_pair_skew_tuning_assignment_candidates_per_pair
      << ",\n"
      << "  \"configured_max_differential_pair_skew_tuning_assignment_states\": "
      << result.materialized_config.max_differential_pair_skew_tuning_assignment_states << ",\n"
      << "  \"attempted_differential_pair_skew_tuning_pairs\": "
      << result.attempted_differential_pair_skew_tuning_pairs << ",\n"
      << "  \"attempted_differential_pair_skew_tuning_candidates\": "
      << result.attempted_differential_pair_skew_tuning_candidates << ",\n"
      << "  \"tuned_differential_pairs\": " << result.tuned_differential_pairs << ",\n"
      << "  \"withheld_differential_pair_skew_tuning_pairs\": "
      << result.withheld_differential_pair_skew_tuning_pairs << ",\n"
      << "  \"assessed_differential_pair_skew_tuning_regions\": "
      << result.assessed_differential_pair_skew_tuning_regions << ",\n"
      << "  \"assessed_any_direction_differential_pair_skew_tuning_regions\": "
      << result.assessed_any_direction_differential_pair_skew_tuning_regions << ",\n"
      << "  \"withheld_differential_pair_skew_tuning_regions\": "
      << result.withheld_differential_pair_skew_tuning_regions << ",\n"
      << "  \"withheld_differential_pair_skew_tuning_free_space_candidates\": "
      << result.withheld_differential_pair_skew_tuning_free_space_candidates << ",\n"
      << "  \"withheld_differential_pair_skew_tuning_quantization_candidates\": "
      << result.withheld_differential_pair_skew_tuning_quantization_candidates << ",\n"
      << "  \"maximum_differential_pair_skew_tuning_period_capacity\": "
      << result.maximum_differential_pair_skew_tuning_period_capacity << ",\n"
      << "  \"materialized_differential_pair_skew_tuning_periods\": "
      << result.materialized_differential_pair_skew_tuning_periods << ",\n"
      << "  \"materialized_any_direction_differential_pair_skew_tuning_patterns\": "
      << result.materialized_any_direction_differential_pair_skew_tuning_patterns << ",\n"
      << "  \"legal_differential_pair_skew_tuning_candidates\": "
      << result.legal_differential_pair_skew_tuning_candidates << ",\n"
      << "  \"differential_pair_skew_tuning_assignment_attempted\": "
      << (result.differential_pair_skew_tuning_assignment_attempted ? "true" : "false") << ",\n"
      << "  \"differential_pair_skew_tuning_assignment_states\": "
      << result.differential_pair_skew_tuning_assignment_states << ",\n"
      << "  \"rejected_differential_pair_skew_tuning_assignment_conflicts\": "
      << result.rejected_differential_pair_skew_tuning_assignment_conflicts << ",\n"
      << "  \"differential_pair_skew_tuning_assignment_budget_exhausted\": "
      << (result.differential_pair_skew_tuning_assignment_budget_exhausted ? "true" : "false")
      << ",\n"
      << "  \"added_differential_pair_skew_tuning_length_nm\": "
      << result.added_differential_pair_skew_tuning_length_nm << ",\n"
      << "  \"differential_pair_skew_tuning_budget_exhausted\": "
      << (result.differential_pair_skew_tuning_budget_exhausted ? "true" : "false") << ",\n"
      << "  \"reused_nets\": " << result.reused_nets.size() << ",\n"
      << "  \"rematerialized_nets\": " << result.rematerialized_nets.size() << ",\n"
      << "  \"invalidated_reuse_nets\": " << result.invalidated_reuse_nets.size() << ",\n"
      << "  \"fallback_full_materialization\": "
      << (result.fallback_full_materialization ? "true" : "false") << ",\n"
      << "  \"fallback_orthogonal_materialization\": "
      << (result.fallback_orthogonal_materialization ? "true" : "false") << ",\n"
      << "  \"fallback_discrete_materialization\": "
      << (result.fallback_discrete_materialization ? "true" : "false") << ",\n"
      << "  \"fallback_topology_locked_materialization\": "
      << (result.fallback_topology_locked_materialization ? "true" : "false") << ",\n"
      << "  \"fallback_topology_locked_subset_materialization\": "
      << (result.fallback_topology_locked_subset_materialization ? "true" : "false") << ",\n"
      << "  \"attempted_topology_locked_subset_repair_nets\": "
      << result.attempted_topology_locked_subset_repair_nets << ",\n"
      << "  \"attempted_topology_locked_subset_repair_candidates\": "
      << result.attempted_topology_locked_subset_repair_candidates << ",\n"
      << "  \"retained_topology_locked_subset_paths\": "
      << result.retained_topology_locked_subset_paths << ",\n"
      << "  \"fallback_polyline_materialization\": "
      << (result.fallback_polyline_materialization ? "true" : "false") << ",\n"
      << "  \"direct_medial_sampled_fallback_attempted\": "
      << (result.direct_medial_sampled_fallback_attempted ? "true" : "false") << ",\n"
      << "  \"direct_medial_sampled_fallback_accepted\": "
      << (result.direct_medial_sampled_fallback_accepted ? "true" : "false") << ",\n"
      << "  \"discarded_direct_medial_edges\": " << result.discarded_direct_medial_edges << ",\n"
      << "  \"discarded_direct_medial_copper_violations\": "
      << result.discarded_direct_medial_copper_violations << ",\n"
      << "  \"discarded_direct_medial_topology_complete_paths\": "
      << result.discarded_direct_medial_topology_complete_paths << ",\n"
      << "  \"discarded_direct_medial_portal_transitions\": "
      << result.discarded_direct_medial_portal_transitions << ",\n"
      << "  \"discarded_accepted_direct_portal_lane_transitions\": "
      << result.discarded_accepted_direct_portal_lane_transitions << ",\n"
      << "  \"discarded_voronoi_lane_conflict_passes\": "
      << result.discarded_voronoi_lane_conflict_passes << ",\n"
      << "  \"discarded_voronoi_lane_conflict_observations\": "
      << result.discarded_voronoi_lane_conflict_observations << ",\n"
      << "  \"discarded_voronoi_lane_conflict_history_entries\": "
      << result.discarded_voronoi_lane_conflict_history_entries << ",\n"
      << "  \"discarded_voronoi_exact_junction_conflict_history_entries\": "
      << result.discarded_voronoi_exact_junction_conflict_history_entries << ",\n"
      << "  \"discarded_unaccepted_selected_voronoi_lane_transitions\": "
      << result.discarded_unaccepted_selected_voronoi_lane_transitions << ",\n"
      << "  \"discarded_materialized_topology_locked_paths\": "
      << result.discarded_materialized_topology_locked_paths << ",\n"
      << "  \"discarded_topology_locked_rejection_copper_violations\": "
      << result.discarded_topology_locked_rejection_copper_violations << ",\n"
      << "  \"discarded_attempted_topology_locked_subset_repair_candidates\": "
      << result.discarded_attempted_topology_locked_subset_repair_candidates << ",\n"
      << "  \"discarded_attempted_shared_resource_fanout_groups\": "
      << result.discarded_attempted_shared_resource_fanout_groups << ",\n"
      << "  \"discarded_attempted_shared_resource_fanout_candidates\": "
      << result.discarded_attempted_shared_resource_fanout_candidates << ",\n"
      << "  \"discarded_materialized_shared_resource_fanout_groups\": "
      << result.discarded_materialized_shared_resource_fanout_groups << ",\n"
      << "  \"discarded_synchronized_shared_resource_fanouts\": "
      << result.discarded_synchronized_shared_resource_fanouts << ",\n"
      << "  \"discarded_shared_resource_fanout_fallbacks\": "
      << result.discarded_shared_resource_fanout_fallbacks << ",\n"
      << "  \"discarded_shared_resource_fanout_failure_count\": "
      << result.discarded_shared_resource_fanout_failures.size() << ",\n"
      << "  \"shared_resource_fanout_failure_details\": [";
  for (std::size_t index = 0U; index < result.shared_resource_fanout_failures.size(); ++index) {
    const auto& failure = result.shared_resource_fanout_failures[index];
    std::cout << (index == 0U ? "" : ", ") << "{\"resource_id\": " << failure.resource_id
              << ", \"layer\": " << json_quote(failure.layer.value()) << ", \"start_nm\": ["
              << failure.start.x.value() << ", " << failure.start.y.value() << "], \"end_nm\": ["
              << failure.end.x.value() << ", " << failure.end.y.value() << "], \"nets\": [";
    for (std::size_t net = 0U; net < failure.nets.size(); ++net) {
      std::cout << (net == 0U ? "" : ", ") << json_quote(failure.nets[net].value());
    }
    std::cout << "], \"conflicting_nets\": [";
    for (std::size_t net = 0U; net < failure.conflicting_nets.size(); ++net) {
      std::cout << (net == 0U ? "" : ", ") << json_quote(failure.conflicting_nets[net].value());
    }
    std::cout << "], \"fallback_nets\": [";
    for (std::size_t net = 0U; net < failure.fallback_nets.size(); ++net) {
      std::cout << (net == 0U ? "" : ", ") << json_quote(failure.fallback_nets[net].value());
    }
    std::cout << "], \"members\": [";
    for (std::size_t member = 0U; member < failure.members.size(); ++member) {
      const auto& evidence = failure.members[member];
      const auto print_point = [](flux::Point point) {
        std::cout << "[" << point.x.value() << ", " << point.y.value() << "]";
      };
      std::cout << (member == 0U ? "" : ", ") << "{\"net\": " << json_quote(evidence.net.value())
                << ", \"start_anchor_nm\": ";
      print_point(evidence.start_anchor);
      std::cout << ", \"start_lane_nm\": ";
      print_point(evidence.start_lane);
      std::cout << ", \"end_lane_nm\": ";
      print_point(evidence.end_lane);
      std::cout << ", \"end_anchor_nm\": ";
      print_point(evidence.end_anchor);
      std::cout << "}";
    }
    std::cout << "]}";
  }
  std::cout << "],\n"
            << "  \"validation_errors\": " << result.validation.error_count() << ",\n"
            << "  \"open_connections\": " << result.connectivity.open_connections << ",\n"
            << "  \"copper_violations\": " << result.copper_drc.violation_count() << ",\n"
            << "  \"open_net_details\": [";
  bool first_open_net = true;
  for (const auto& net : result.connectivity.nets) {
    if (net.open_connections == 0U) {
      continue;
    }
    std::cout << (first_open_net ? "" : ", ") << "{\"net\": " << json_quote(net.net.value())
              << ", \"pad_count\": " << net.pad_count
              << ", \"connected_pad_groups\": " << net.connected_pad_groups
              << ", \"open_connections\": " << net.open_connections << "}";
    first_open_net = false;
  }
  std::cout << "],\n  \"copper_violation_counts\": {";
  bool first_violation_code = true;
  for (const auto& [code, count] : copper_violation_counts) {
    std::cout << (first_violation_code ? "" : ", ") << json_quote(code) << ": " << count;
    first_violation_code = false;
  }
  std::cout << "},\n  \"first_copper_violation\": ";
  if (result.copper_drc.violations.empty()) {
    std::cout << "null";
  } else {
    const auto& violation = result.copper_drc.violations.front();
    std::cout << "{\"code\": " << json_quote(violation.code)
              << ", \"required_clearance_nm\": " << violation.required_clearance.value()
              << ", \"actual_distance_nm\": " << static_cast<double>(violation.actual_distance_nm)
              << ", \"object_ids\": [";
    for (std::size_t index = 0; index < violation.object_ids.size(); ++index) {
      std::cout << (index == 0U ? "" : ", ") << json_quote(violation.object_ids[index]);
    }
    std::cout << "]}";
  }
  std::cout << ",\n"
            << "  \"differential_pair_gap_violations\": " << pair_gap_violations << ",\n"
            << "  \"differential_pair_skew_violations\": " << pair_skew_violations << ",\n"
            << "  \"routability_diagnoses\": [";
  for (std::size_t index = 0U; index < result.routability_diagnoses.size(); ++index) {
    const auto& diagnosis = result.routability_diagnoses[index];
    std::cout << (index == 0U ? "" : ",")
              << "{\"kind\":" << json_quote(flux::to_string(diagnosis.kind))
              << ",\"stage\":" << json_quote(flux::to_string(diagnosis.stage))
              << ",\"resource\":" << json_quote(diagnosis.resource.stable_id)
              << ",\"severity\":" << diagnosis.severity << ",\"nets\":";
    print_net_ids(diagnosis.resource.nets);
    std::cout << '}';
  }
  std::cout << "],\n"
            << "  \"final_routability_feedback_attempted\": "
            << (result.final_routability_feedback_attempted ? "true" : "false") << ",\n"
            << "  \"final_routability_feedback_selected\": "
            << (result.final_routability_feedback_selected ? "true" : "false") << ",\n"
            << "  \"final_routability_feedback_passes\": "
            << result.final_routability_feedback_passes << ",\n"
            << "  \"final_routability_feedback_improvements\": "
            << result.final_routability_feedback_improvements << ",\n"
            << "  \"final_routability_diagnoses_consumed\": "
            << result.final_routability_diagnoses_consumed << ",\n"
            << "  \"diagnostic\": " << json_quote(result.diagnostic);
  if (exported) {
    std::cout << ",\n  \"artifact_written\": " << (exported->written ? "true" : "false")
              << ",\n  \"artifact_preview_only\": " << (exported->preview_only ? "true" : "false")
              << ",\n  \"export_errors\": " << exported->errors
              << ",\n  \"generated_kicad_ids\": " << exported->generated_ids
              << ",\n  \"export_issue_codes\": [";
    for (std::size_t index = 0; index < exported->issue_codes.size(); ++index) {
      std::cout << (index == 0U ? "" : ", ") << json_quote(exported->issue_codes[index]);
    }
    std::cout << ']';
  }
  std::cout << "\n}\n";
}

[[nodiscard]] std::string_view stop_reason_name(flux::IntegratedOptimizerStopReason reason) {
  switch (reason) {
  case flux::IntegratedOptimizerStopReason::complete:
    return "complete";
  case flux::IntegratedOptimizerStopReason::no_improving_move:
    return "no_improving_move";
  case flux::IntegratedOptimizerStopReason::pass_limit:
    return "pass_limit";
  case flux::IntegratedOptimizerStopReason::evaluation_limit:
    return "evaluation_limit";
  case flux::IntegratedOptimizerStopReason::invalid_config:
    return "invalid_config";
  case flux::IntegratedOptimizerStopReason::cancelled:
    return "cancelled";
  }
  return "unknown";
}

[[nodiscard]] std::string_view stop_reason_name(flux::GlobalPlacerStopReason reason) {
  switch (reason) {
  case flux::GlobalPlacerStopReason::complete:
    return "complete";
  case flux::GlobalPlacerStopReason::invalid_input:
    return "invalid_input";
  case flux::GlobalPlacerStopReason::infeasible:
    return "infeasible";
  case flux::GlobalPlacerStopReason::cancelled:
    return "cancelled";
  }
  return "unknown";
}

[[nodiscard]] std::string_view placement_move_kind_name(flux::PlacementMoveKind kind) {
  switch (kind) {
  case flux::PlacementMoveKind::global_seed:
    return "global_seed";
  case flux::PlacementMoveKind::rotation_repair:
    return "rotation_repair";
  case flux::PlacementMoveKind::side_flip_repair:
    return "side_flip_repair";
  case flux::PlacementMoveKind::orientation_repair:
    return "orientation_repair";
  case flux::PlacementMoveKind::routing_guided_repair:
    return "routing_guided_repair";
  case flux::PlacementMoveKind::neck_down_obstacle_repair:
    return "neck_down_obstacle_repair";
  case flux::PlacementMoveKind::physical_channel_obstacle_repair:
    return "physical_channel_obstacle_repair";
  case flux::PlacementMoveKind::swap_repair:
    return "swap_repair";
  case flux::PlacementMoveKind::group_repair:
    return "group_repair";
  case flux::PlacementMoveKind::relative_component_repair:
    return "relative_component_repair";
  case flux::PlacementMoveKind::local_repair:
    return "local_repair";
  }
  return "unknown";
}

[[nodiscard]] std::string_view board_side_name(BoardSide side) {
  return side == BoardSide::front ? "front" : "back";
}

void print_integrated_score(const flux::IntegratedScore& score) {
  std::cout << "{\"placement_violations\": " << score.placement_violations
            << ", \"global_route_failures\": " << score.global_route_failures
            << ", \"unrouted_nets\": " << score.unrouted_nets
            << ", \"failed_pin_escapes\": " << score.failed_pin_escapes
            << ", \"best_hard_capacity_unrouted_nets\": " << score.best_hard_capacity_unrouted_nets
            << ", \"configured_dimensioned_physical_overflow\": "
            << (score.configured_dimensioned_physical_overflow ? "true" : "false")
            << ", \"total_physical_overflow_nm\": " << score.total_physical_overflow_nm
            << ", \"peak_physical_overflow_nm\": " << score.peak_physical_overflow_nm
            << ", \"sampled_overflow\": " << score.sampled_overflow
            << ", \"corridor_overflow\": " << score.corridor_overflow
            << ", \"congested_physical_resources\": " << score.congested_physical_resources
            << ", \"global_overflow\": " << score.global_overflow
            << ", \"copper_violations\": " << score.copper_violations
            << ", \"open_connections\": " << score.open_connections
            << ", \"width_reduction_cost\": " << score.width_reduction_cost
            << ", \"routed_length_nm\": " << score.routed_length_nm
            << ", \"route_cost\": " << score.route_cost
            << ", \"side_changes\": " << score.side_changes
            << ", \"rotation_change_microdegrees\": " << score.rotation_change_microdegrees
            << ", \"displacement_nm\": " << score.displacement_nm << '}';
}

void print_integrated_result(const flux::IntegratedOptimizationResult& result,
                             std::optional<ExportSummary> exported = std::nullopt) {
  const auto pair_gap_violations = static_cast<std::size_t>(std::count_if(
      result.detailed_route.copper_drc.violations.begin(),
      result.detailed_route.copper_drc.violations.end(),
      [](const auto& violation) { return violation.code == "differential_pair_gap"; }));
  const auto pair_skew_violations = static_cast<std::size_t>(std::count_if(
      result.detailed_route.copper_drc.violations.begin(),
      result.detailed_route.copper_drc.violations.end(),
      [](const auto& violation) { return violation.code == "differential_pair_skew"; }));
  std::cout
      << "{\n"
      << "  \"complete\": " << (result.complete() ? "true" : "false") << ",\n"
      << "  \"feasible\": " << (result.feasible() ? "true" : "false") << ",\n"
      << "  \"stop_reason\": \"" << stop_reason_name(result.stop_reason) << "\",\n"
      << "  \"configured_max_placement_passes\": " << result.configured_max_passes << ",\n"
      << "  \"configured_max_candidate_evaluations\": "
      << result.configured_max_candidate_evaluations << ",\n"
      << "  \"configured_max_candidate_evaluation_workers\": "
      << result.configured_max_candidate_evaluation_workers << ",\n"
      << "  \"configured_max_full_routing_candidates_per_wave\": "
      << result.configured_max_full_routing_candidates_per_wave << ",\n"
      << "  \"configured_candidate_evaluation_offset\": "
      << result.configured_candidate_evaluation_offset << ",\n"
      << "  \"candidate_evaluations_skipped\": " << result.candidate_evaluations_skipped << ",\n"
      << "  \"configured_move_step_nm\": " << result.configured_move_step.value() << ",\n"
      << "  \"candidate_evaluations\": " << result.candidate_evaluations << ",\n"
      << "  \"candidate_evaluation_waves\": " << result.candidate_evaluation_waves << ",\n"
      << "  \"candidate_evaluation_tasks_dispatched\": "
      << result.candidate_evaluation_tasks_dispatched << ",\n"
      << "  \"peak_candidate_evaluation_workers\": " << result.peak_candidate_evaluation_workers
      << ",\n"
      << "  \"shortlisted_full_route_evaluations\": " << result.shortlisted_full_route_evaluations
      << ",\n"
      << "  \"neck_down_obstacle_move_proposals\": " << result.neck_down_obstacle_move_proposals
      << ",\n"
      << "  \"neck_down_obstacle_candidate_evaluations\": "
      << result.neck_down_obstacle_candidate_evaluations << ",\n"
      << "  \"neck_down_obstacle_moves_accepted\": " << result.neck_down_obstacle_moves_accepted
      << ",\n"
      << "  \"physical_channel_obstacle_move_proposals\": "
      << result.physical_channel_obstacle_move_proposals << ",\n"
      << "  \"physical_channel_obstacle_candidate_evaluations\": "
      << result.physical_channel_obstacle_candidate_evaluations << ",\n"
      << "  \"physical_channel_obstacle_moves_accepted\": "
      << result.physical_channel_obstacle_moves_accepted << ",\n"
      << "  \"board_candidate_validations\": " << result.board_candidate_validations << ",\n"
      << "  \"reused_static_zone_validations\": " << result.reused_static_zone_validations << ",\n"
      << "  \"incremental_route_evaluations\": " << result.incremental_route_evaluations << ",\n"
      << "  \"full_route_evaluations\": " << result.full_route_evaluations << ",\n"
      << "  \"checkpoint_paths_reused\": " << result.checkpoint_paths_reused << ",\n"
      << "  \"incremental_detailed_evaluations\": " << result.incremental_detailed_evaluations
      << ",\n"
      << "  \"checkpoint_tracks_reused\": " << result.checkpoint_tracks_reused << ",\n"
      << "  \"checkpoint_vias_reused\": " << result.checkpoint_vias_reused << ",\n"
      << "  \"detailed_reuse_fallbacks\": " << result.detailed_reuse_fallbacks << ",\n"
      << "  \"tentative_placement_envelopes\": " << result.tentative_placement_envelopes << ",\n"
      << "  \"human_placement_review_required\": "
      << (result.human_placement_review_required ? "true" : "false") << ",\n"
      << "  \"configured_placement_move_priorities\": "
      << (result.configured_placement_move_priorities ? "true" : "false") << ",\n"
      << "  \"suggested_placement_move_priorities\": "
      << (result.suggested_placement_move_priorities ? "true" : "false") << ",\n"
      << "  \"placement_move_priority_review_required\": "
      << (result.placement_move_priority_review_required ? "true" : "false") << ",\n"
      << "  \"placement_priority_tiers_unlocked\": " << result.placement_priority_tiers_unlocked
      << ",\n"
      << "  \"placement_priority_escalations\": " << result.placement_priority_escalations << ",\n"
      << "  \"accepted_moves\": " << result.accepted_moves.size() << ",\n"
      << "  \"pareto_archive_size\": " << result.pareto_archive.size() << ",\n"
      << "  \"pareto_candidates_discarded\": " << result.pareto_candidates_discarded << ",\n"
      << "  \"global_placement_attempted\": " << (result.global_placement ? "true" : "false")
      << ",\n"
      << "  \"global_seed_accepted\": " << (result.global_seed_accepted ? "true" : "false") << ",\n"
      << "  \"global_repairable_tentative_source_accepted\": "
      << (result.global_placement && result.global_placement->repairable_tentative_source_accepted
              ? "true"
              : "false")
      << ",\n"
      << "  \"global_repairable_tentative_source_errors\": "
      << (result.global_placement ? result.global_placement->repairable_tentative_source_errors
                                  : 0U)
      << ",\n"
      << "  \"global_constraint_components\": "
      << (result.global_placement ? result.global_placement->constraint_components : 0U) << ",\n"
      << "  \"global_immobilized_constraint_components\": "
      << (result.global_placement ? result.global_placement->immobilized_constraint_components : 0U)
      << ",\n"
      << "  \"global_atomically_legalized_components\": "
      << (result.global_placement ? result.global_placement->atomically_legalized_components : 0U)
      << ",\n"
      << "  \"global_moved_components\": "
      << (result.global_placement ? result.global_placement->moved_components : 0U) << ",\n"
      << "  \"global_routing_guides\": "
      << (result.global_placement ? result.global_placement->routing_guides : 0U) << ",\n"
      << "  \"global_routing_guided_footprints\": "
      << (result.global_placement ? result.global_placement->routing_guided_footprints : 0U)
      << ",\n"
      << "  \"global_assignment_legalization_attempted\": "
      << (result.global_placement && result.global_placement->assignment_legalization_attempted
              ? "true"
              : "false")
      << ",\n"
      << "  \"global_assignment_legalization_withheld\": "
      << (result.global_placement && result.global_placement->assignment_legalization_withheld
              ? "true"
              : "false")
      << ",\n"
      << "  \"global_assignment_legalization_accepted\": "
      << (result.global_placement && result.global_placement->assignment_legalization_accepted
              ? "true"
              : "false")
      << ",\n"
      << "  \"global_assignment_search_budget_exhausted\": "
      << (result.global_placement && result.global_placement->assignment_search_budget_exhausted
              ? "true"
              : "false")
      << ",\n"
      << "  \"global_assignment_components\": "
      << (result.global_placement ? result.global_placement->assignment_components : 0U) << ",\n"
      << "  \"global_assignment_candidate_edges\": "
      << (result.global_placement ? result.global_placement->assignment_candidate_edges : 0U)
      << ",\n"
      << "  \"global_assignment_search_states\": "
      << (result.global_placement ? result.global_placement->assignment_search_states : 0U) << ",\n"
      << "  \"placement_violations\": " << result.score.placement_violations << ",\n"
      << "  \"global_route_failures\": " << result.score.global_route_failures << ",\n"
      << "  \"unrouted_nets\": " << result.score.unrouted_nets << ",\n"
      << "  \"best_hard_capacity_unrouted_nets\": " << result.score.best_hard_capacity_unrouted_nets
      << ",\n"
      << "  \"configured_dimensioned_physical_overflow\": "
      << (result.score.configured_dimensioned_physical_overflow ? "true" : "false") << ",\n"
      << "  \"integrated_total_physical_overflow_nm\": " << result.score.total_physical_overflow_nm
      << ",\n"
      << "  \"integrated_peak_physical_overflow_nm\": " << result.score.peak_physical_overflow_nm
      << ",\n"
      << "  \"integrated_sampled_overflow\": " << result.score.sampled_overflow << ",\n"
      << "  \"integrated_corridor_overflow\": " << result.score.corridor_overflow << ",\n"
      << "  \"integrated_congested_physical_resources\": "
      << result.score.congested_physical_resources << ",\n"
      << "  \"unrouted_net_ids\": ";
  print_net_ids(result.global_route.unrouted_nets);
  std::cout << ",\n"
            << "  \"connection_failures\": ";
  print_routing_connection_failures(result.global_route.connection_failures);
  std::cout
      << ",\n"
      << "  \"global_route_iterations\": " << result.global_route.iterations << ",\n"
      << "  \"configured_grid_pitch_nm\": " << result.global_route.configured_grid_pitch.value()
      << ",\n"
      << "  \"configured_via_site_pitch_nm\": "
      << result.global_route.configured_via_site_pitch.value() << ",\n"
      << "  \"configured_via_site_phase_x\": " << result.global_route.configured_via_site_phase_x
      << ",\n"
      << "  \"configured_via_site_phase_y\": " << result.global_route.configured_via_site_phase_y
      << ",\n"
      << "  \"configured_diagonal_grid_edges\": "
      << (result.global_route.configured_diagonal_grid_edges ? "true" : "false") << ",\n"
      << "  \"configured_crossing_safe_diagonal_grid_edges\": "
      << (result.global_route.configured_crossing_safe_diagonal_grid_edges ? "true" : "false")
      << ",\n"
      << "  \"static_seed_searches\": " << result.global_route.static_seed_searches << ",\n"
      << "  \"static_seed_candidates\": " << result.global_route.static_seed_candidates << ",\n"
      << "  \"static_seed_corridor_nodes\": " << result.global_route.static_seed_corridor_nodes
      << ",\n"
      << "  \"configured_residual_corridor_demand_cost\": "
      << result.global_route.configured_residual_corridor_demand_cost << ",\n"
      << "  \"configured_critical_cut_cost\": " << result.global_route.configured_critical_cut_cost
      << ",\n"
      << "  \"configured_critical_cut_minimum_future_demand\": "
      << result.global_route.configured_critical_cut_minimum_future_demand << ",\n"
      << "  \"configured_river_following_cost\": "
      << result.global_route.configured_river_following_cost << ",\n"
      << "  \"river_following_edges\": " << result.global_route.river_following_edges << ",\n"
      << "  \"river_following_cost_discount\": "
      << result.global_route.river_following_cost_discount << ",\n"
      << "  \"configured_targeted_track_spreading_cost\": "
      << result.global_route.configured_targeted_track_spreading_cost << ",\n"
      << "  \"targeted_track_spreading_edges\": "
      << result.global_route.targeted_track_spreading_edges << ",\n"
      << "  \"targeted_track_spreading_penalty\": "
      << result.global_route.targeted_track_spreading_penalty << ",\n"
      << "  \"configured_former_route_memory_cost\": "
      << result.global_route.configured_former_route_memory_cost << ",\n"
      << "  \"former_route_memory_ripped_paths\": "
      << result.global_route.former_route_memory_ripped_paths << ",\n"
      << "  \"former_route_memory_edges\": " << result.global_route.former_route_memory_edges
      << ",\n"
      << "  \"former_route_memory_nodes\": " << result.global_route.former_route_memory_nodes
      << ",\n"
      << "  \"former_route_memory_penalty_events\": "
      << result.global_route.former_route_memory_penalty_events << ",\n"
      << "  \"former_route_memory_penalty\": " << result.global_route.former_route_memory_penalty
      << ",\n"
      << "  \"configured_terminal_escape_reservation_cost\": "
      << result.global_route.configured_terminal_escape_reservation_cost << ",\n"
      << "  \"configured_terminal_escape_proximity_cost\": "
      << result.global_route.configured_terminal_escape_proximity_cost << ",\n"
      << "  \"configured_max_terminal_escape_reservation_resources\": "
      << result.global_route.configured_max_terminal_escape_reservation_resources << ",\n"
      << "  \"terminal_escape_reservation_terminals\": "
      << result.global_route.terminal_escape_reservation_terminals << ",\n"
      << "  \"terminal_escape_reservation_candidates\": "
      << result.global_route.terminal_escape_reservation_candidates << ",\n"
      << "  \"terminal_escape_reservation_resources\": "
      << result.global_route.terminal_escape_reservation_resources << ",\n"
      << "  \"terminal_escape_reservation_withheld_candidates\": "
      << result.global_route.terminal_escape_reservation_withheld_candidates << ",\n"
      << "  \"terminal_escape_reservation_penalty_events\": "
      << result.global_route.terminal_escape_reservation_penalty_events << ",\n"
      << "  \"terminal_escape_reservation_penalty\": "
      << result.global_route.terminal_escape_reservation_penalty << ",\n"
      << "  \"terminal_escape_proximity_penalty_events\": "
      << result.global_route.terminal_escape_proximity_penalty_events << ",\n"
      << "  \"terminal_escape_proximity_penalty\": "
      << result.global_route.terminal_escape_proximity_penalty << ",\n"
      << "  \"residual_corridor_demand_nets\": "
      << result.global_route.residual_corridor_demand_nets << ",\n"
      << "  \"residual_corridor_demand_resources\": "
      << result.global_route.residual_corridor_demand_resources << ",\n"
      << "  \"residual_corridor_demand_penalty_events\": "
      << result.global_route.residual_corridor_demand_penalty_events << ",\n"
      << "  \"residual_corridor_demand_penalty\": "
      << result.global_route.residual_corridor_demand_penalty << ",\n"
      << "  \"critical_cut_resources\": " << result.global_route.critical_cut_resources << ",\n"
      << "  \"critical_cut_reserved_demand\": " << result.global_route.critical_cut_reserved_demand
      << ",\n"
      << "  \"critical_cut_penalty_events\": " << result.global_route.critical_cut_penalty_events
      << ",\n"
      << "  \"critical_cut_penalty\": " << result.global_route.critical_cut_penalty << ",\n"
      << "  \"static_seed_budget_exhausted\": "
      << (result.global_route.static_seed_budget_exhausted ? "true" : "false") << ",\n"
      << "  \"global_route_selected_iteration\": " << result.global_route.selected_iteration
      << ",\n"
      << "  \"global_route_total_overflow\": " << result.global_route.total_overflow << ",\n"
      << "  \"sampled_overflow\": " << result.global_route.sampled_overflow << ",\n"
      << "  \"corridor_overflow\": " << result.global_route.corridor_overflow << ",\n"
      << "  \"configured_checkpoint_objective\": \""
      << checkpoint_objective_name(result.global_route.configured_checkpoint_objective) << "\",\n"
      << "  \"configured_physical_overflow_history\": "
      << (result.global_route.configured_scale_voronoi_history_by_overflow ? "true" : "false")
      << ",\n"
      << "  \"voronoi_history_quantum_nm\": " << result.global_route.voronoi_history_quantum_nm
      << ",\n"
      << "  \"total_voronoi_history_increment\": "
      << result.global_route.total_voronoi_history_increment << ",\n"
      << "  \"peak_voronoi_history_increment\": "
      << result.global_route.peak_voronoi_history_increment << ",\n"
      << "  \"configured_physical_envelope_ordering\": "
      << (result.global_route.configured_physical_envelope_ordering ? "true" : "false") << ",\n"
      << "  \"peak_net_routing_envelope_nm\": " << result.global_route.peak_net_routing_envelope_nm
      << ",\n"
      << "  \"peak_routing_group_envelope_nm\": "
      << result.global_route.peak_routing_group_envelope_nm << ",\n"
      << "  \"configured_present_congestion_multiplier_per_mille\": "
      << result.global_route.configured_present_congestion_multiplier_per_mille << ",\n"
      << "  \"configured_max_present_congestion_scale_per_mille\": "
      << result.global_route.configured_max_present_congestion_scale_per_mille << ",\n"
      << "  \"configured_present_congestion_escalation_start_iteration\": "
      << result.global_route.configured_present_congestion_escalation_start_iteration << ",\n"
      << "  \"configured_hard_capacity_start_iteration\": "
      << result.global_route.configured_hard_capacity_start_iteration << ",\n"
      << "  \"configured_max_hard_capacity_ripup_attempts_per_pass\": "
      << result.global_route.configured_max_hard_capacity_ripup_attempts_per_pass << ",\n"
      << "  \"configured_max_hard_capacity_ripup_nets_per_attempt\": "
      << result.global_route.configured_max_hard_capacity_ripup_nets_per_attempt << ",\n"
      << "  \"configured_max_hard_capacity_conflict_set_states_per_attempt\": "
      << result.global_route.configured_max_hard_capacity_conflict_set_states_per_attempt << ",\n"
      << "  \"hard_capacity_passes\": " << result.global_route.hard_capacity_passes << ",\n"
      << "  \"post_negotiation_cone_repair_attempts\": "
      << result.global_route.post_negotiation_cone_repair_attempts << ",\n"
      << "  \"post_negotiation_cone_repairs_accepted\": "
      << result.global_route.post_negotiation_cone_repairs_accepted << ",\n"
      << "  \"post_negotiation_cone_repair_nets_attempted\": "
      << result.global_route.post_negotiation_cone_repair_nets_attempted << ",\n"
      << "  \"hard_sampled_capacity_rejected_transitions\": "
      << result.global_route.hard_sampled_capacity_rejected_transitions << ",\n"
      << "  \"hard_corridor_capacity_rejected_transitions\": "
      << result.global_route.hard_corridor_capacity_rejected_transitions << ",\n"
      << "  \"hard_voronoi_capacity_rejected_transitions\": "
      << result.global_route.hard_voronoi_capacity_rejected_transitions << ",\n"
      << "  \"hard_capacity_diversified_passes\": "
      << result.global_route.hard_capacity_diversified_passes << ",\n"
      << "  \"hard_capacity_diversified_net_positions\": "
      << result.global_route.hard_capacity_diversified_net_positions
      << ",\n  \"hard_capacity_pass_summaries\": ";
  print_hard_capacity_pass_summaries(result.global_route.hard_capacity_pass_summaries);
  std::cout << ",\n  \"hard_capacity_net_failures\": ";
  print_hard_capacity_net_failures(result.global_route.hard_capacity_net_failures);
  std::cout << ",\n  \"hard_capacity_blockers\": ";
  print_hard_capacity_blockers(result.global_route.hard_capacity_blockers);
  std::cout
      << ",\n"
      << "  \"hard_capacity_ripup_attempts\": " << result.global_route.hard_capacity_ripup_attempts
      << ",\n"
      << "  \"hard_capacity_ripped_nets\": " << result.global_route.hard_capacity_ripped_nets
      << ",\n"
      << "  \"hard_capacity_ripup_recoveries\": "
      << result.global_route.hard_capacity_ripup_recoveries << ",\n"
      << "  \"hard_capacity_ripup_retained_recoveries\": "
      << result.global_route.hard_capacity_ripup_retained_recoveries << ",\n"
      << "  \"hard_capacity_conflict_set_states\": "
      << result.global_route.hard_capacity_conflict_set_states << ",\n"
      << "  \"hard_capacity_joint_ripup_attempts\": "
      << result.global_route.hard_capacity_joint_ripup_attempts << ",\n"
      << "  \"hard_capacity_conflict_sets_withheld\": "
      << result.global_route.hard_capacity_conflict_sets_withheld << ",\n"
      << "  \"configured_max_soft_capacity_cut_probe_expansions\": "
      << result.global_route.configured_max_soft_capacity_cut_probe_expansions << ",\n"
      << "  \"hard_capacity_soft_cut_probes\": "
      << result.global_route.hard_capacity_soft_cut_probes << ",\n"
      << "  \"hard_capacity_soft_cut_routes\": "
      << result.global_route.hard_capacity_soft_cut_routes << ",\n"
      << "  \"hard_capacity_soft_cut_blockers\": "
      << result.global_route.hard_capacity_soft_cut_blockers << ",\n"
      << "  \"configured_rollback_failed_hard_capacity_ripup\": "
      << (result.global_route.configured_rollback_failed_hard_capacity_ripup ? "true" : "false")
      << ",\n"
      << "  \"hard_capacity_failed_retry_rollbacks\": "
      << result.global_route.hard_capacity_failed_retry_rollbacks << ",\n"
      << "  \"configured_defer_hard_capacity_ripup_victims_to_pass_tail\": "
      << (result.global_route.configured_defer_hard_capacity_ripup_victims_to_pass_tail ? "true"
                                                                                        : "false")
      << ",\n"
      << "  \"hard_capacity_tail_deferred_victims\": "
      << result.global_route.hard_capacity_tail_deferred_victims << ",\n"
      << "  \"configured_protect_recovered_hard_capacity_ripup_nets\": "
      << (result.global_route.configured_protect_recovered_hard_capacity_ripup_nets ? "true"
                                                                                    : "false")
      << ",\n"
      << "  \"hard_capacity_protected_recoveries\": "
      << result.global_route.hard_capacity_protected_recoveries << ",\n"
      << "  \"hard_capacity_protected_victim_rejections\": "
      << result.global_route.hard_capacity_protected_victim_rejections << ",\n"
      << "  \"configured_max_hard_capacity_blocker_reuse_invalidations_per_failed_net\": "
      << result.global_route.configured_max_hard_capacity_blocker_reuse_invalidations_per_failed_net
      << ",\n"
      << "  \"hard_capacity_blocker_reuse_invalidations\": "
      << result.global_route.hard_capacity_blocker_reuse_invalidations
      << ",\n  \"hard_capacity_ripup_records\": ";
  print_hard_capacity_ripup_records(result.global_route.hard_capacity_ripup_records);
  std::cout
      << ",\n"
      << "  \"selected_present_congestion_scale_per_mille\": "
      << result.global_route.selected_present_congestion_scale_per_mille << ",\n"
      << "  \"selected_effective_present_congestion_cost\": "
      << result.global_route.selected_effective_present_congestion_cost << ",\n"
      << "  \"selected_effective_corridor_present_cost\": "
      << result.global_route.selected_effective_corridor_present_cost << ",\n"
      << "  \"selected_effective_voronoi_resource_present_cost\": "
      << result.global_route.selected_effective_voronoi_resource_present_cost << ",\n"
      << "  \"configured_max_expansions_per_connection\": "
      << result.global_route.configured_max_expansions_per_connection << ",\n"
      << "  \"configured_expansion_retry_heuristic_weight_per_mille\": "
      << result.global_route.configured_expansion_retry_heuristic_weight_per_mille << ",\n"
      << "  \"configured_reuse_weighted_search_frontier\": "
      << (result.global_route.configured_reuse_weighted_search_frontier ? "true" : "false") << ",\n"
      << "  \"path_search_invocations\": " << result.global_route.path_search_invocations << ",\n"
      << "  \"path_search_expansions\": " << result.global_route.path_search_expansions << ",\n"
      << "  \"expansion_limited_searches\": " << result.global_route.expansion_limited_searches
      << ",\n"
      << "  \"zone_hole_broad_phase_rejections\": "
      << result.global_route.zone_hole_broad_phase_rejections << ",\n"
      << "  \"zone_hole_exact_checks\": " << result.global_route.zone_hole_exact_checks << ",\n"
      << "  \"weighted_search_retries\": " << result.global_route.weighted_search_retries << ",\n"
      << "  \"weighted_search_recoveries\": " << result.global_route.weighted_search_recoveries
      << ",\n"
      << "  \"weighted_frontier_continuations\": "
      << result.global_route.weighted_frontier_continuations << ",\n"
      << "  \"weighted_frontier_recoveries\": " << result.global_route.weighted_frontier_recoveries
      << ",\n"
      << "  \"weighted_frontier_states_reused\": "
      << result.global_route.weighted_frontier_states_reused << ",\n"
      << "  \"weighted_frontier_continuation_limits\": "
      << result.global_route.weighted_frontier_continuation_limits << ",\n"
      << "  \"failed_pin_escapes\": " << result.score.failed_pin_escapes << ",\n"
      << "  \"pin_escape_failures\": ";
  print_pin_escape_failures(result.global_route.pin_escape_failures);
  std::cout
      << ",\n"
      << "  \"pin_access_nodes\": " << result.global_route.pin_access_nodes << ",\n"
      << "  \"adaptive_pin_access_nodes\": " << result.global_route.adaptive_pin_access_nodes
      << ",\n"
      << "  \"terminals_with_adaptive_pin_access\": "
      << result.global_route.terminals_with_adaptive_pin_access << ",\n"
      << "  \"configured_contended_terminal_fanout\": "
      << (result.global_route.configured_contended_terminal_fanout ? "true" : "false") << ",\n"
      << "  \"contended_terminals\": " << result.global_route.contended_terminals << ",\n"
      << "  \"contention_triggered_local_paths\": "
      << result.global_route.contention_triggered_local_paths << ",\n"
      << "  \"configured_diversify_pin_access_rings\": "
      << (result.global_route.configured_diversify_pin_access_rings ? "true" : "false") << ",\n"
      << "  \"diversified_pin_access_nodes\": " << result.global_route.diversified_pin_access_nodes
      << ",\n"
      << "  \"terminals_with_diversified_pin_access\": "
      << result.global_route.terminals_with_diversified_pin_access << ",\n"
      << "  \"additional_successful_pin_access_rings\": "
      << result.global_route.additional_successful_pin_access_rings << ",\n"
      << "  \"isolated_pin_access_nodes_rejected\": "
      << result.global_route.isolated_pin_access_nodes_rejected << ",\n"
      << "  \"terminals_with_isolated_pin_access_rejections\": "
      << result.global_route.terminals_with_isolated_pin_access_rejections << ",\n"
      << "  \"terminal_neck_down_access_nodes\": "
      << result.global_route.terminal_neck_down_access_nodes << ",\n"
      << "  \"terminals_with_neck_down_access\": "
      << result.global_route.terminals_with_neck_down_access << ",\n"
      << "  \"coupled_terminals_ineligible_for_neck_down\": "
      << result.global_route.coupled_terminals_ineligible_for_neck_down << ",\n"
      << "  \"configured_terminal_neck_down_width_nm\": "
      << result.global_route.configured_terminal_neck_down_width.value() << ",\n"
      << "  \"configured_max_terminal_neck_down_length_nm\": "
      << result.global_route.configured_max_terminal_neck_down_length.value() << ",\n"
      << "  \"used_terminal_neck_down_segments\": "
      << result.global_route.used_terminal_neck_down_segments << ",\n"
      << "  \"total_terminal_neck_down_length_nm\": "
      << result.global_route.total_terminal_neck_down_length_nm << ",\n"
      << "  \"configured_medial_neck_down_width_nm\": "
      << result.global_route.configured_medial_neck_down_width.value() << ",\n"
      << "  \"configured_max_medial_neck_down_length_nm\": "
      << result.global_route.configured_max_medial_neck_down_length.value() << ",\n"
      << "  \"medial_neck_down_edges_available\": "
      << result.global_route.medial_neck_down_edges_available << ",\n"
      << "  \"used_medial_neck_down_segments\": "
      << result.global_route.used_medial_neck_down_segments << ",\n"
      << "  \"total_medial_neck_down_length_nm\": "
      << result.global_route.total_medial_neck_down_length_nm << ",\n"
      << "  \"configured_corridor_neck_down_width_nm\": "
      << result.global_route.configured_corridor_neck_down_width.value() << ",\n"
      << "  \"configured_max_corridor_neck_down_length_nm\": "
      << result.global_route.configured_max_corridor_neck_down_length.value() << ",\n"
      << "  \"corridor_neck_down_search_states\": "
      << result.global_route.corridor_neck_down_search_states << ",\n"
      << "  \"coupled_nets_ineligible_for_corridor_neck_down\": "
      << result.global_route.coupled_nets_ineligible_for_corridor_neck_down << ",\n"
      << "  \"used_corridor_neck_down_segments\": "
      << result.global_route.used_corridor_neck_down_segments << ",\n"
      << "  \"total_corridor_neck_down_length_nm\": "
      << result.global_route.total_corridor_neck_down_length_nm << ",\n"
      << "  \"neck_down_obstacle_impacts\": "
      << result.global_route.neck_down_obstacle_impacts.size() << ",\n"
      << "  \"physical_channel_obstacle_impact_count\": "
      << result.global_route.physical_obstacle_impacts.size() << ",\n"
      << "  \"configured_width_reduction_cost_per_grid_step\": "
      << result.global_route.configured_width_reduction_cost_per_grid_step << ",\n"
      << "  \"total_equivalent_width_reduction_length_nm\": "
      << result.global_route.total_equivalent_width_reduction_length_nm << ",\n"
      << "  \"total_width_reduction_cost\": " << result.global_route.total_width_reduction_cost
      << ",\n"
      << "  \"topology_rejected_search_edges\": "
      << result.global_route.topology_rejected_search_edges << ",\n"
      << "  \"medial_topology_breaks\": " << result.global_route.medial_topology_breaks << ",\n"
      << "  \"medial_direct_portal_transitions\": "
      << result.global_route.medial_direct_portal_transitions << ",\n"
      << "  \"medial_unmapped_nodes\": " << result.global_route.medial_unmapped_nodes << ",\n"
      << "  \"medial_interior_unmapped_nodes\": "
      << result.global_route.medial_interior_unmapped_nodes << ",\n"
      << "  \"topologically_complete_medial_paths\": "
      << result.global_route.topologically_complete_medial_paths << ",\n"
      << "  \"direct_voronoi_edges_available\": "
      << result.global_route.direct_voronoi_edges_available << ",\n"
      << "  \"direct_voronoi_edges_withheld\": "
      << result.global_route.direct_voronoi_edges_withheld << ",\n"
      << "  \"used_direct_voronoi_edges\": " << result.global_route.used_direct_voronoi_edges
      << ",\n"
      << "  \"direct_voronoi_grid_steps_bypassed\": "
      << result.global_route.direct_voronoi_grid_steps_bypassed << ",\n"
      << "  \"direct_voronoi_search_fallbacks\": "
      << result.global_route.direct_voronoi_search_fallbacks << ",\n"
      << "  \"multisource_steiner_growth_enabled\": "
      << (result.global_route.multisource_steiner_growth_enabled ? "true" : "false") << ",\n"
      << "  \"multisource_steiner_connections\": "
      << result.global_route.multisource_steiner_connections << ",\n"
      << "  \"multisource_steiner_reordered_connections\": "
      << result.global_route.multisource_steiner_reordered_connections << ",\n"
      << "  \"multisource_steiner_deferred_inaccessible_terminals\": "
      << result.global_route.multisource_steiner_deferred_inaccessible_terminals << ",\n"
      << "  \"multisource_steiner_fixed_order_fallbacks\": "
      << result.global_route.multisource_steiner_fixed_order_fallbacks << ",\n"
      << "  \"multisource_steiner_fixed_order_recoveries\": "
      << result.global_route.multisource_steiner_fixed_order_recoveries << ",\n"
      << "  \"bidirectional_route_growth_enabled\": "
      << (result.global_route.bidirectional_route_growth_enabled ? "true" : "false") << ",\n"
      << "  \"configured_bidirectional_route_growth_start_iteration\": "
      << result.global_route.configured_bidirectional_route_growth_start_iteration << ",\n"
      << "  \"configured_max_bidirectional_meeting_nodes\": "
      << result.global_route.configured_max_bidirectional_meeting_nodes << ",\n"
      << "  \"configured_max_bidirectional_meeting_expansions\": "
      << result.global_route.configured_max_bidirectional_meeting_expansions << ",\n"
      << "  \"bidirectional_route_growth_attempts\": "
      << result.global_route.bidirectional_route_growth_attempts << ",\n"
      << "  \"bidirectional_route_growth_meeting_nodes\": "
      << result.global_route.bidirectional_route_growth_meeting_nodes << ",\n"
      << "  \"bidirectional_route_growth_accepted\": "
      << result.global_route.bidirectional_route_growth_accepted << ",\n"
      << "  \"bidirectional_route_growth_fallbacks\": "
      << result.global_route.bidirectional_route_growth_fallbacks << ",\n"
      << "  \"atomic_complete_net_routing_enabled\": "
      << (result.global_route.atomic_complete_net_routing_enabled ? "true" : "false") << ",\n"
      << "  \"configured_selective_congestion_reroute\": "
      << (result.global_route.configured_selective_congestion_reroute ? "true" : "false") << ",\n"
      << "  \"configured_selective_full_reroute_interval\": "
      << result.global_route.configured_selective_full_reroute_interval << ",\n"
      << "  \"selective_reroute_passes\": " << result.global_route.selective_reroute_passes << ",\n"
      << "  \"selective_full_reroute_refreshes\": "
      << result.global_route.selective_full_reroute_refreshes << ",\n"
      << "  \"selective_retained_net_observations\": "
      << result.global_route.selective_retained_net_observations << ",\n"
      << "  \"selective_retained_path_observations\": "
      << result.global_route.selective_retained_path_observations << ",\n"
      << "  \"selective_conflict_net_observations\": "
      << result.global_route.selective_conflict_net_observations << ",\n"
      << "  \"selective_coupled_group_expansions\": "
      << result.global_route.selective_coupled_group_expansions << ",\n"
      << "  \"rolled_back_incomplete_net_trees\": "
      << result.global_route.rolled_back_incomplete_net_trees << ",\n"
      << "  \"rolled_back_incomplete_net_paths\": "
      << result.global_route.rolled_back_incomplete_net_paths << ",\n"
      << "  \"rolled_back_incomplete_net_cost\": "
      << result.global_route.rolled_back_incomplete_net_cost << ",\n"
      << "  \"global_overflow\": " << result.score.global_overflow << ",\n"
      << "  \"routing_resources\": " << result.global_route.resource_usage.size() << ",\n"
      << "  \"congested_resources\": " << result.global_route.congested_resources << ",\n"
      << "  \"peak_resource_demand\": " << result.global_route.peak_resource_demand << ",\n"
      << "  \"congested_sampled_resource_usage\": ";
  print_congested_sampled_resource_usage(result.global_route.resource_usage);
  std::cout << ",\n"
            << "  \"pathfinder_reordered_iterations\": "
            << result.global_route.pathfinder_reordered_iterations << ",\n"
            << "  \"pathfinder_reordered_nets\": " << result.global_route.pathfinder_reordered_nets
            << ",\n"
            << "  \"pathfinder_peak_net_pressure\": "
            << result.global_route.pathfinder_peak_net_pressure << ",\n"
            << "  \"pathfinder_peak_net_sampled_overflow\": "
            << result.global_route.pathfinder_peak_net_sampled_overflow << ",\n"
            << "  \"pathfinder_peak_net_corridor_overflow\": "
            << result.global_route.pathfinder_peak_net_corridor_overflow << ",\n"
            << "  \"pathfinder_peak_net_total_physical_overflow_nm\": "
            << result.global_route.pathfinder_peak_net_total_physical_overflow_nm << ",\n"
            << "  \"pathfinder_peak_net_physical_overflow_nm\": "
            << result.global_route.pathfinder_peak_net_physical_overflow_nm << ",\n"
            << "  \"pathfinder_peak_net_congested_physical_resources\": "
            << result.global_route.pathfinder_peak_net_congested_physical_resources << ",\n"
            << "  \"selected_net_pressure\": ";
  print_routing_net_pressure(result.global_route.selected_net_pressure);
  std::cout << ",\n  \"congested_voronoi_resource_usage\": ";
  print_congested_voronoi_resource_usage(result.global_route.voronoi_resource_usage);
  std::cout << ",\n  \"physical_channel_obstacle_impacts\": ";
  print_physical_obstacle_impacts(result.global_route.physical_obstacle_impacts);
  std::cout << ",\n  \"footprint_pressure\": ";
  print_footprint_pressure(result.footprint_pressure);
  std::cout
      << ",\n"
      << "  \"routing_corridors\": " << result.global_route.corridor_usage.size() << ",\n"
      << "  \"congested_corridors\": " << result.global_route.congested_corridors << ",\n"
      << "  \"neck_down_corridors\": " << result.global_route.neck_down_corridors << ",\n"
      << "  \"peak_corridor_demand\": " << result.global_route.peak_corridor_demand << ",\n"
      << "  \"corridor_neck_downs\": " << result.global_route.corridor_neck_down_usage.size()
      << ",\n"
      << "  \"congested_corridor_neck_downs\": "
      << result.global_route.congested_corridor_neck_downs << ",\n"
      << "  \"peak_corridor_neck_down_demand\": "
      << result.global_route.peak_corridor_neck_down_demand << ",\n"
      << "  \"voronoi_allocation_resources\": " << result.global_route.voronoi_resource_usage.size()
      << ",\n"
      << "  \"voronoi_evidence_curved_edge_chords\": "
      << result.global_route.voronoi_evidence_curved_edge_chords << ",\n"
      << "  \"voronoi_allocation_curved_edge_chords\": "
      << result.global_route.voronoi_allocation_curved_edge_chords << ",\n"
      << "  \"voronoi_allocation_withheld_curved_edges\": "
      << result.global_route.voronoi_allocation_withheld_curved_edges << ",\n"
      << "  \"configured_max_voronoi_resource_segment_length_nm\": "
      << result.global_route.configured_max_voronoi_resource_segment_length.value() << ",\n"
      << "  \"segmented_voronoi_source_resources\": "
      << result.global_route.segmented_voronoi_source_resources << ",\n"
      << "  \"additional_voronoi_segment_resources\": "
      << result.global_route.additional_voronoi_segment_resources << ",\n"
      << "  \"direct_voronoi_resources_retained_atomic\": "
      << result.global_route.direct_voronoi_resources_retained_atomic << ",\n"
      << "  \"congested_voronoi_resources\": " << result.global_route.congested_voronoi_resources
      << ",\n"
      << "  \"width_reduced_voronoi_allocations\": "
      << result.global_route.width_reduced_voronoi_allocations << ",\n"
      << "  \"total_voronoi_demand_reduction_nm\": "
      << result.global_route.total_voronoi_demand_reduction_nm << ",\n"
      << "  \"total_voronoi_overflow_nm\": " << result.global_route.total_voronoi_overflow_nm
      << ",\n"
      << "  \"peak_voronoi_overflow_nm\": " << result.global_route.peak_voronoi_overflow_nm << ",\n"
      << "  \"coupled_differential_pair_resources\": "
      << result.global_route.coupled_differential_pair_resources << ",\n"
      << "  \"coupled_routing_bus_resources\": "
      << result.global_route.coupled_routing_bus_resources << ",\n"
      << "  \"packed_voronoi_resources\": " << result.global_route.packed_voronoi_resources << ",\n"
      << "  \"voronoi_lane_assignments\": " << result.global_route.voronoi_lane_assignments.size()
      << ",\n"
      << "  \"static_clearance_checked_voronoi_lanes\": "
      << result.global_route.static_clearance_checked_voronoi_lanes << ",\n"
      << "  \"voronoi_lane_transitions\": " << result.global_route.voronoi_lane_transitions.size()
      << ",\n"
      << "  \"accepted_voronoi_lane_transitions\": "
      << result.global_route.accepted_voronoi_lane_transitions << ",\n"
      << "  \"unaccepted_selected_voronoi_lane_transitions\": "
      << result.global_route.unaccepted_selected_voronoi_lane_transitions << ",\n"
      << "  \"direct_portal_lane_transitions\": "
      << result.global_route.direct_portal_lane_transitions << ",\n"
      << "  \"accepted_direct_portal_lane_transitions\": "
      << result.global_route.accepted_direct_portal_lane_transitions << ",\n"
      << "  \"unaccepted_direct_portal_lane_transitions\": "
      << result.global_route.unaccepted_direct_portal_lane_transitions << ",\n"
      << "  \"configured_negotiate_voronoi_lane_conflicts\": "
      << (result.global_route.configured_negotiate_voronoi_lane_conflicts ? "true" : "false")
      << ",\n"
      << "  \"voronoi_lane_conflict_passes\": " << result.global_route.voronoi_lane_conflict_passes
      << ",\n"
      << "  \"voronoi_lane_conflict_observations\": "
      << result.global_route.voronoi_lane_conflict_observations << ",\n"
      << "  \"voronoi_lane_conflict_history_entries\": "
      << result.global_route.voronoi_lane_conflict_history_entries << ",\n"
      << "  \"voronoi_exact_junction_conflict_history_entries\": "
      << result.global_route.voronoi_exact_junction_conflict_history_entries << ",\n"
      << "  \"voronoi_selected_junction_edge_conflict_history_entries\": "
      << result.global_route.voronoi_selected_junction_edge_conflict_history_entries << ",\n"
      << "  \"voronoi_lane_conflict_resource_history_entries\": "
      << result.global_route.voronoi_lane_conflict_resource_history_entries << ",\n"
      << "  \"voronoi_lane_conflict_history_increments\": "
      << result.global_route.voronoi_lane_conflict_history_increments << ",\n"
      << "  \"voronoi_lane_conflict_history_withheld\": "
      << result.global_route.voronoi_lane_conflict_history_withheld << ",\n"
      << "  \"voronoi_lane_conflict_scheduled_reroute_nets\": "
      << result.global_route.voronoi_lane_conflict_scheduled_reroute_nets << ",\n"
      << "  \"unselected_voronoi_lane_transitions\": "
      << result.global_route.unselected_voronoi_lane_transitions << ",\n"
      << "  \"reoriented_voronoi_resources\": " << result.global_route.reoriented_voronoi_resources
      << ",\n"
      << "  \"voronoi_lane_orientation_states\": "
      << result.global_route.voronoi_lane_orientation_states << ",\n"
      << "  \"solved_voronoi_lane_orientation_components\": "
      << result.global_route.solved_voronoi_lane_orientation_components << ",\n"
      << "  \"withheld_voronoi_lane_orientation_components\": "
      << result.global_route.withheld_voronoi_lane_orientation_components << ",\n"
      << "  \"largest_voronoi_lane_orientation_component_resources\": "
      << result.global_route.largest_voronoi_lane_orientation_component_resources << ",\n"
      << "  \"solved_voronoi_lane_orientation_junctions\": "
      << result.global_route.solved_voronoi_lane_orientation_junctions << ",\n"
      << "  \"withheld_voronoi_lane_orientation_junctions\": "
      << result.global_route.withheld_voronoi_lane_orientation_junctions << ",\n"
      << "  \"mitered_voronoi_lane_transitions\": "
      << result.global_route.mitered_voronoi_lane_transitions << ",\n"
      << "  \"attempted_continuous_refinement_candidates\": "
      << result.detailed_route.attempted_continuous_refinement_candidates << ",\n"
      << "  \"accepted_continuous_refinement_candidates\": "
      << result.detailed_route.accepted_continuous_refinement_candidates << ",\n"
      << "  \"rejected_continuous_refinement_candidates\": "
      << result.detailed_route.rejected_continuous_refinement_candidates << ",\n"
      << "  \"withheld_continuous_refinement_candidates\": "
      << result.detailed_route.withheld_continuous_refinement_candidates << ",\n"
      << "  \"continuous_refinement_budget_exhausted\": "
      << (result.detailed_route.continuous_refinement_budget_exhausted ? "true" : "false") << ",\n"
      << "  \"materialized_terminal_neck_down_paths\": "
      << result.detailed_route.materialized_terminal_neck_down_paths << ",\n"
      << "  \"materialized_terminal_neck_down_segments\": "
      << result.detailed_route.materialized_terminal_neck_down_segments << ",\n"
      << "  \"materialized_medial_neck_down_paths\": "
      << result.detailed_route.materialized_medial_neck_down_paths << ",\n"
      << "  \"materialized_medial_neck_down_segments\": "
      << result.detailed_route.materialized_medial_neck_down_segments << ",\n"
      << "  \"materialized_corridor_neck_down_paths\": "
      << result.detailed_route.materialized_corridor_neck_down_paths << ",\n"
      << "  \"materialized_corridor_neck_down_segments\": "
      << result.detailed_route.materialized_corridor_neck_down_segments << ",\n"
      << "  \"width_aware_simplified_points\": "
      << result.detailed_route.width_aware_simplified_points << ",\n"
      << "  \"width_aware_pulled_tight_paths\": "
      << result.detailed_route.width_aware_pulled_tight_paths << ",\n"
      << "  \"width_aware_pulled_tight_corners\": "
      << result.detailed_route.width_aware_pulled_tight_corners << ",\n"
      << "  \"width_aware_continuously_refined_paths\": "
      << result.detailed_route.width_aware_continuously_refined_paths << ",\n"
      << "  \"width_aware_continuously_refined_points\": "
      << result.detailed_route.width_aware_continuously_refined_points << ",\n"
      << "  \"materialized_topology_locked_paths\": "
      << result.detailed_route.materialized_topology_locked_paths << ",\n"
      << "  \"materialized_multilayer_topology_locked_paths\": "
      << result.detailed_route.materialized_multilayer_topology_locked_paths << ",\n"
      << "  \"preserved_topology_locked_terminal_access_paths\": "
      << result.detailed_route.preserved_topology_locked_terminal_access_paths << ",\n"
      << "  \"attempted_coupled_terminal_fanout_groups\": "
      << result.detailed_route.attempted_coupled_terminal_fanout_groups << ",\n"
      << "  \"attempted_coupled_terminal_fanout_candidates\": "
      << result.detailed_route.attempted_coupled_terminal_fanout_candidates << ",\n"
      << "  \"materialized_coupled_terminal_fanout_groups\": "
      << result.detailed_route.materialized_coupled_terminal_fanout_groups << ",\n"
      << "  \"synchronized_coupled_terminal_fanouts\": "
      << result.detailed_route.synchronized_coupled_terminal_fanouts << ",\n"
      << "  \"coupled_terminal_fanout_fallbacks\": "
      << result.detailed_route.coupled_terminal_fanout_fallbacks << ",\n"
      << "  \"attempted_shared_resource_fanout_groups\": "
      << result.detailed_route.attempted_shared_resource_fanout_groups << ",\n"
      << "  \"attempted_shared_resource_fanout_candidates\": "
      << result.detailed_route.attempted_shared_resource_fanout_candidates << ",\n"
      << "  \"materialized_shared_resource_fanout_groups\": "
      << result.detailed_route.materialized_shared_resource_fanout_groups << ",\n"
      << "  \"synchronized_shared_resource_fanouts\": "
      << result.detailed_route.synchronized_shared_resource_fanouts << ",\n"
      << "  \"shared_resource_fanout_fallbacks\": "
      << result.detailed_route.shared_resource_fanout_fallbacks << ",\n"
      << "  \"shared_resource_fanout_failure_count\": "
      << result.detailed_route.shared_resource_fanout_failures.size() << ",\n"
      << "  \"attempted_coupled_multilayer_groups\": "
      << result.detailed_route.attempted_coupled_multilayer_groups << ",\n"
      << "  \"materialized_coupled_multilayer_groups\": "
      << result.detailed_route.materialized_coupled_multilayer_groups << ",\n"
      << "  \"synchronized_coupled_via_arrays\": "
      << result.detailed_route.synchronized_coupled_via_arrays << ",\n"
      << "  \"coupled_multilayer_group_fallbacks\": "
      << result.detailed_route.coupled_multilayer_group_fallbacks << ",\n"
      << "  \"configured_maximum_circular_arc_radius_nm\": "
      << result.detailed_route.materialized_config.maximum_circular_arc_radius.value() << ",\n"
      << "  \"configured_circular_arc_backtracking_steps\": "
      << result.detailed_route.materialized_config.circular_arc_backtracking_steps << ",\n"
      << "  \"configured_max_circular_arc_candidate_checks\": "
      << result.detailed_route.materialized_config.max_circular_arc_candidate_checks << ",\n"
      << "  \"attempted_circular_arc_corners\": "
      << result.detailed_route.attempted_circular_arc_corners << ",\n"
      << "  \"attempted_circular_arc_candidates\": "
      << result.detailed_route.attempted_circular_arc_candidates << ",\n"
      << "  \"rejected_circular_arc_candidates\": "
      << result.detailed_route.rejected_circular_arc_candidates << ",\n"
      << "  \"fitted_circular_arcs\": " << result.detailed_route.fitted_circular_arcs << ",\n"
      << "  \"unrounded_non_junction_bends\": "
      << result.detailed_route.unrounded_non_junction_bends << ",\n"
      << "  \"radius_reduced_circular_arcs\": "
      << result.detailed_route.radius_reduced_circular_arcs << ",\n"
      << "  \"withheld_circular_arc_corners\": "
      << result.detailed_route.withheld_circular_arc_corners << ",\n"
      << "  \"circular_arc_candidate_budget_exhausted\": "
      << (result.detailed_route.circular_arc_candidate_budget_exhausted ? "true" : "false") << ",\n"
      << "  \"width_aware_circular_arc_paths\": "
      << result.detailed_route.width_aware_circular_arc_paths << ",\n"
      << "  \"width_aware_fitted_circular_arcs\": "
      << result.detailed_route.width_aware_fitted_circular_arcs << ",\n"
      << "  \"attempted_quintic_ph_runs\": " << result.detailed_route.attempted_quintic_ph_runs
      << ",\n"
      << "  \"fitted_quintic_ph_runs\": " << result.detailed_route.fitted_quintic_ph_runs << ",\n"
      << "  \"adaptive_spline_tessellation_samples\": "
      << result.detailed_route.adaptive_spline_tessellation_samples << ",\n"
      << "  \"spline_collision_envelopes\": " << result.detailed_route.spline_collision_envelopes
      << ",\n"
      << "  \"spline_navmesh_routes\": " << result.detailed_route.spline_navmesh_routes << ",\n"
      << "  \"spline_navmesh_failures\": " << result.detailed_route.spline_navmesh_failures << ",\n"
      << "  \"spline_navmesh_refined_parents\": "
      << result.detailed_route.spline_navmesh_refined_parents << ",\n"
      << "  \"spline_navmesh_refined_leaves\": "
      << result.detailed_route.spline_navmesh_refined_leaves << ",\n"
      << "  \"spline_navmesh_rejected_leaves\": "
      << result.detailed_route.spline_navmesh_rejected_leaves << ",\n"
      << "  \"spline_navmesh_gpu_wavefront_routes\": "
      << result.detailed_route.spline_navmesh_gpu_wavefront_routes << ",\n"
      << "  \"spline_navmesh_gpu_wavefront_cpu_fallbacks\": "
      << result.detailed_route.spline_navmesh_gpu_wavefront_cpu_fallbacks << ",\n"
      << "  \"spline_navmesh_certificate_fallbacks\": "
      << result.detailed_route.spline_navmesh_certificate_fallbacks << ",\n"
      << "  \"spline_navmesh_topology_signatures\": "
      << result.detailed_route.spline_navmesh_topology_signatures << ",\n"
      << "  \"spline_navmesh_topology_gates\": "
      << result.detailed_route.spline_navmesh_topology_gates << ",\n"
      << "  \"spline_navmesh_topology_rejections\": "
      << result.detailed_route.spline_navmesh_topology_rejections << ",\n"
      << "  \"spline_seed_candidates\": " << result.detailed_route.spline_seed_candidates << ",\n"
      << "  \"spline_seed_exact_legal\": " << result.detailed_route.spline_seed_exact_legal << ",\n"
      << "  \"spline_seed_portfolio_improvements\": "
      << result.detailed_route.spline_seed_portfolio_improvements << ",\n"
      << "  \"spline_trajectory_accepted\": " << result.detailed_route.spline_trajectory_accepted
      << ",\n"
      << "  \"spline_multistart_attempts\": " << result.detailed_route.spline_multistart_attempts
      << ",\n"
      << "  \"spline_multistart_recoveries\": "
      << result.detailed_route.spline_multistart_recoveries << ",\n"
      << "  \"spline_gpu_precondition_attempts\": "
      << result.detailed_route.spline_gpu_precondition_attempts << ",\n"
      << "  \"spline_gpu_precondition_dispatches\": "
      << result.detailed_route.spline_gpu_precondition_dispatches << ",\n"
      << "  \"spline_gpu_precondition_cpu_fallbacks\": "
      << result.detailed_route.spline_gpu_precondition_cpu_fallbacks << ",\n"
      << "  \"spline_gpu_precondition_accepted\": "
      << result.detailed_route.spline_gpu_precondition_accepted << ",\n"
      << "  \"spline_gpu_precondition_legal\": "
      << result.detailed_route.spline_gpu_precondition_legal << ",\n"
      << "  \"spline_gpu_precondition_solve_us\": "
      << result.detailed_route.spline_gpu_precondition_solve_us << ",\n"
      << "  \"spline_gpu_polish_batches\": " << result.detailed_route.spline_gpu_polish_batches
      << ",\n"
      << "  \"spline_gpu_polish_dispatches\": "
      << result.detailed_route.spline_gpu_polish_dispatches << ",\n"
      << "  \"spline_gpu_polish_cpu_fallbacks\": "
      << result.detailed_route.spline_gpu_polish_cpu_fallbacks << ",\n"
      << "  \"spline_gpu_polish_candidates\": "
      << result.detailed_route.spline_gpu_polish_candidates << ",\n"
      << "  \"spline_gpu_polish_exact_shortlist_checks\": "
      << result.detailed_route.spline_gpu_polish_exact_shortlist_checks << ",\n"
      << "  \"spline_parallel_exact_waves\": " << result.detailed_route.spline_parallel_exact_waves
      << ",\n"
      << "  \"spline_parallel_exact_checks\": "
      << result.detailed_route.spline_parallel_exact_checks << ",\n"
      << "  \"spline_parallel_route_tasks\": " << result.detailed_route.spline_parallel_route_tasks
      << ",\n"
      << "  \"spline_parallel_route_workers\": "
      << result.detailed_route.spline_parallel_route_workers << ",\n"
      << "  \"spline_parallel_route_candidates\": "
      << result.detailed_route.spline_parallel_route_candidates << ",\n"
      << "  \"spline_parallel_route_commits\": "
      << result.detailed_route.spline_parallel_route_commits << ",\n"
      << "  \"spline_parallel_route_merge_rejections\": "
      << result.detailed_route.spline_parallel_route_merge_rejections << ",\n"
      << "  \"spline_control_point_shoves\": " << result.detailed_route.spline_control_point_shoves
      << ",\n"
      << "  \"spline_refits\": " << result.detailed_route.spline_refits << ",\n"
      << "  \"pns_shove_attempts\": " << result.detailed_route.pns_shove_attempts << ",\n"
      << "  \"accepted_pns_shoves\": " << result.detailed_route.accepted_pns_shoves << ",\n"
      << "  \"pns_moved_tracks\": " << result.detailed_route.pns_moved_tracks << ",\n"
      << "  \"pns_moved_vias\": " << result.detailed_route.pns_moved_vias << ",\n"
      << "  \"residual_conflict_handoff_attempted\": "
      << (result.detailed_route.residual_conflict_handoff_attempted ? "true" : "false") << ",\n"
      << "  \"residual_conflict_groups_attempted\": "
      << result.detailed_route.residual_conflict_groups_attempted << ",\n"
      << "  \"residual_joint_candidate_states\": "
      << result.detailed_route.residual_joint_candidate_states << ",\n"
      << "  \"residual_joint_transaction_accepted\": "
      << (result.detailed_route.residual_joint_transaction_accepted ? "true" : "false") << ",\n"
      << "  \"configured_differential_pair_skew_tuning_step_nm\": "
      << result.detailed_route.materialized_config.differential_pair_skew_tuning_step.value()
      << ",\n"
      << "  \"configured_maximum_differential_pair_skew_tuning_amplitude_nm\": "
      << result.detailed_route.materialized_config.maximum_differential_pair_skew_tuning_amplitude
             .value()
      << ",\n"
      << "  \"configured_minimum_differential_pair_skew_tuning_lead_nm\": "
      << result.detailed_route.materialized_config.minimum_differential_pair_skew_tuning_lead
             .value()
      << ",\n"
      << "  \"configured_minimum_differential_pair_skew_tuning_spacing_nm\": "
      << result.detailed_route.materialized_config.minimum_differential_pair_skew_tuning_spacing
             .value()
      << ",\n"
      << "  \"configured_max_differential_pair_skew_tuning_candidates\": "
      << result.detailed_route.materialized_config.max_differential_pair_skew_tuning_candidates
      << ",\n"
      << "  \"configured_max_differential_pair_skew_tuning_periods\": "
      << result.detailed_route.materialized_config.max_differential_pair_skew_tuning_periods
      << ",\n"
      << "  \"configured_differential_pair_skew_tuning_batch_assignment\": "
      << (result.detailed_route.materialized_config.differential_pair_skew_tuning_batch_assignment
              ? "true"
              : "false")
      << ",\n"
      << "  \"configured_max_differential_pair_skew_tuning_assignment_candidates_per_pair\": "
      << result.detailed_route.materialized_config
             .max_differential_pair_skew_tuning_assignment_candidates_per_pair
      << ",\n"
      << "  \"configured_max_differential_pair_skew_tuning_assignment_states\": "
      << result.detailed_route.materialized_config
             .max_differential_pair_skew_tuning_assignment_states
      << ",\n"
      << "  \"attempted_differential_pair_skew_tuning_pairs\": "
      << result.detailed_route.attempted_differential_pair_skew_tuning_pairs << ",\n"
      << "  \"attempted_differential_pair_skew_tuning_candidates\": "
      << result.detailed_route.attempted_differential_pair_skew_tuning_candidates << ",\n"
      << "  \"tuned_differential_pairs\": " << result.detailed_route.tuned_differential_pairs
      << ",\n"
      << "  \"withheld_differential_pair_skew_tuning_pairs\": "
      << result.detailed_route.withheld_differential_pair_skew_tuning_pairs << ",\n"
      << "  \"assessed_differential_pair_skew_tuning_regions\": "
      << result.detailed_route.assessed_differential_pair_skew_tuning_regions << ",\n"
      << "  \"assessed_any_direction_differential_pair_skew_tuning_regions\": "
      << result.detailed_route.assessed_any_direction_differential_pair_skew_tuning_regions << ",\n"
      << "  \"withheld_differential_pair_skew_tuning_regions\": "
      << result.detailed_route.withheld_differential_pair_skew_tuning_regions << ",\n"
      << "  \"withheld_differential_pair_skew_tuning_free_space_candidates\": "
      << result.detailed_route.withheld_differential_pair_skew_tuning_free_space_candidates << ",\n"
      << "  \"withheld_differential_pair_skew_tuning_quantization_candidates\": "
      << result.detailed_route.withheld_differential_pair_skew_tuning_quantization_candidates
      << ",\n"
      << "  \"maximum_differential_pair_skew_tuning_period_capacity\": "
      << result.detailed_route.maximum_differential_pair_skew_tuning_period_capacity << ",\n"
      << "  \"materialized_differential_pair_skew_tuning_periods\": "
      << result.detailed_route.materialized_differential_pair_skew_tuning_periods << ",\n"
      << "  \"materialized_any_direction_differential_pair_skew_tuning_patterns\": "
      << result.detailed_route.materialized_any_direction_differential_pair_skew_tuning_patterns
      << ",\n"
      << "  \"legal_differential_pair_skew_tuning_candidates\": "
      << result.detailed_route.legal_differential_pair_skew_tuning_candidates << ",\n"
      << "  \"differential_pair_skew_tuning_assignment_attempted\": "
      << (result.detailed_route.differential_pair_skew_tuning_assignment_attempted ? "true"
                                                                                   : "false")
      << ",\n"
      << "  \"differential_pair_skew_tuning_assignment_states\": "
      << result.detailed_route.differential_pair_skew_tuning_assignment_states << ",\n"
      << "  \"rejected_differential_pair_skew_tuning_assignment_conflicts\": "
      << result.detailed_route.rejected_differential_pair_skew_tuning_assignment_conflicts << ",\n"
      << "  \"differential_pair_skew_tuning_assignment_budget_exhausted\": "
      << (result.detailed_route.differential_pair_skew_tuning_assignment_budget_exhausted ? "true"
                                                                                          : "false")
      << ",\n"
      << "  \"added_differential_pair_skew_tuning_length_nm\": "
      << result.detailed_route.added_differential_pair_skew_tuning_length_nm << ",\n"
      << "  \"differential_pair_skew_tuning_budget_exhausted\": "
      << (result.detailed_route.differential_pair_skew_tuning_budget_exhausted ? "true" : "false")
      << ",\n"
      << "  \"copper_violations\": " << result.score.copper_violations << ",\n"
      << "  \"differential_pair_gap_violations\": " << pair_gap_violations << ",\n"
      << "  \"differential_pair_skew_violations\": " << pair_skew_violations << ",\n"
      << "  \"open_connections\": " << result.score.open_connections << ",\n"
      << "  \"width_reduction_cost\": " << result.score.width_reduction_cost << ",\n"
      << "  \"routed_length_nm\": " << result.score.routed_length_nm << ",\n"
      << "  \"route_cost\": " << result.score.route_cost << ",\n"
      << "  \"side_changes\": " << result.score.side_changes << ",\n"
      << "  \"rotation_change_microdegrees\": " << result.score.rotation_change_microdegrees
      << ",\n"
      << "  \"displacement_nm\": " << result.score.displacement_nm;
  if (result.global_placement) {
    std::cout << ",\n  \"global_placement_stop_reason\": "
              << json_quote(stop_reason_name(result.global_placement->stop_reason))
              << ",\n  \"global_placement_diagnostic\": "
              << json_quote(result.global_placement->diagnostic)
              << ",\n  \"global_hpwl_before_nm\": " << result.global_placement->before.hpwl_nm
              << ",\n  \"global_hpwl_after_nm\": " << result.global_placement->after.hpwl_nm
              << ",\n  \"global_target_distance_before_nm\": "
              << result.global_placement->before.analytical_target_distance_nm
              << ",\n  \"global_target_distance_after_nm\": "
              << result.global_placement->after.analytical_target_distance_nm
              << ",\n  \"global_moved_footprints\": " << result.global_placement->moved_footprints
              << ",\n  \"global_legalization_attempts\": "
              << result.global_placement->legalization_attempts;
  }
  std::cout << ",\n  \"placement_priority_metrics\": [";
  for (std::size_t index = 0U; index < result.placement_priority_metrics.size(); ++index) {
    const auto& metric = result.placement_priority_metrics[index];
    std::cout << (index == 0U ? "" : ",") << "\n    {\"class\": "
              << json_quote(flux::placement_move_priority_class_name(metric.priority_class))
              << ", \"footprints\": " << metric.footprints
              << ", \"proposals\": " << metric.proposals
              << ", \"candidate_evaluations\": " << metric.candidate_evaluations
              << ", \"accepted_moves\": " << metric.accepted_moves << '}';
  }
  if (!result.placement_priority_metrics.empty()) {
    std::cout << '\n';
  }
  std::cout << "  ],\n  \"pareto_archive\": [";
  for (std::size_t index = 0U; index < result.pareto_archive.size(); ++index) {
    const auto& checkpoint = result.pareto_archive[index];
    std::cout << (index == 0U ? "" : ",")
              << "\n    {\"full_route_evaluation\": " << checkpoint.full_route_evaluation
              << ", \"score\": ";
    print_integrated_score(checkpoint.score);
    std::cout << '}';
  }
  if (!result.pareto_archive.empty()) {
    std::cout << '\n';
  }
  std::cout << "  ],\n  \"moves\": [";
  for (std::size_t i = 0; i < result.accepted_moves.size(); ++i) {
    const auto& move = result.accepted_moves[i];
    std::cout << (i == 0U ? "" : ",")
              << "\n    {\"footprint\": " << json_quote(move.footprint.value())
              << ", \"kind\": " << json_quote(placement_move_kind_name(move.kind))
              << ", \"from_nm\": [" << move.from.x.value() << ", " << move.from.y.value()
              << "], \"to_nm\": [" << move.to.x.value() << ", " << move.to.y.value()
              << "], \"from_rotation_microdegrees\": " << move.from_rotation.value()
              << ", \"to_rotation_microdegrees\": " << move.to_rotation.value()
              << ", \"from_side\": " << json_quote(board_side_name(move.from_side))
              << ", \"to_side\": " << json_quote(board_side_name(move.to_side));
    if (move.partner_footprint && move.partner_from && move.partner_to) {
      std::cout << ", \"partner_footprint\": " << json_quote(move.partner_footprint->value())
                << ", \"partner_from_nm\": [" << move.partner_from->x.value() << ", "
                << move.partner_from->y.value() << "]"
                << ", \"partner_to_nm\": [" << move.partner_to->x.value() << ", "
                << move.partner_to->y.value() << "]";
    }
    if (move.placement_group) {
      std::cout << ", \"placement_group\": " << json_quote(move.placement_group->value());
    }
    if (move.priority_class) {
      std::cout << ", \"priority_class\": "
                << json_quote(flux::placement_move_priority_class_name(*move.priority_class));
    }
    std::cout << ", \"relative_constraints\": [";
    for (std::size_t constraint_index = 0U; constraint_index < move.relative_constraints.size();
         ++constraint_index) {
      std::cout << (constraint_index == 0U ? "" : ", ")
                << json_quote(move.relative_constraints[constraint_index].value());
    }
    std::cout << "], \"constraint_groups\": [";
    for (std::size_t group_index = 0U; group_index < move.constraint_groups.size(); ++group_index) {
      std::cout << (group_index == 0U ? "" : ", ")
                << json_quote(move.constraint_groups[group_index].value());
    }
    std::cout << ']';
    std::cout << ", \"companion_moves\": [";
    for (std::size_t companion_index = 0U; companion_index < move.companion_moves.size();
         ++companion_index) {
      const auto& companion = move.companion_moves[companion_index];
      std::cout << (companion_index == 0U ? "" : ", ")
                << "{\"footprint\": " << json_quote(companion.footprint.value())
                << ", \"from_nm\": [" << companion.from.x.value() << ", "
                << companion.from.y.value() << "], \"to_nm\": [" << companion.to.x.value() << ", "
                << companion.to.y.value() << "]}";
    }
    std::cout << ']';
    std::cout << ", \"before\": ";
    print_integrated_score(move.before);
    std::cout << ", \"after\": ";
    print_integrated_score(move.after);
    std::cout << ", \"routing_obstacles\": [";
    for (std::size_t obstacle_index = 0; obstacle_index < move.routing_obstacles.size();
         ++obstacle_index) {
      std::cout << (obstacle_index == 0U ? "" : ", ")
                << json_quote(move.routing_obstacles[obstacle_index].value());
    }
    std::cout << "], \"routing_pressure_resources\": " << move.routing_pressure.size()
              << ", \"routing_corridor_pressure_resources\": "
              << move.routing_corridor_pressure.size()
              << ", \"routing_physical_pressure_resources\": "
              << move.routing_physical_pressure.size()
              << ", \"neck_down_obstacle_impacts\": " << move.neck_down_obstacles.size()
              << ", \"physical_channel_obstacle_impacts\": "
              << move.physical_channel_obstacles.size() << ", \"affected_nets\": [";
    for (std::size_t net_index = 0; net_index < move.affected_nets.size(); ++net_index) {
      std::cout << (net_index == 0U ? "" : ", ")
                << json_quote(move.affected_nets[net_index].value());
    }
    std::cout << "]}";
  }
  if (!result.accepted_moves.empty()) {
    std::cout << '\n';
  }
  std::cout << "  ],\n  \"diagnostic\": " << json_quote(result.diagnostic);
  if (exported) {
    std::cout << ",\n  \"artifact_written\": " << (exported->written ? "true" : "false")
              << ",\n  \"export_errors\": " << exported->errors
              << ",\n  \"generated_kicad_ids\": " << exported->generated_ids
              << ",\n  \"export_issue_codes\": [";
    for (std::size_t i = 0; i < exported->issue_codes.size(); ++i) {
      std::cout << (i == 0U ? "" : ", ") << json_quote(exported->issue_codes[i]);
    }
    std::cout << ']';
  }
  std::cout << "\n}\n";
}

[[nodiscard]] Board make_route_conflict_demo(bool lock_left = false) {
  auto board = make_demo_board(false, false);
  board.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0}};
  board.footprints.front().locked = lock_left;
  const NetId blocker{"N2"};
  board.nets.push_back(Net{blocker, "LOCKED_BLOCKER", 0});
  board.tracks.push_back({ObjectId{"locked-blocker"}, blocker, LayerId{"F.Cu"}, mm(50.0, 0.625),
                          mm(50.0, 79.375), Nanometres::from_mm(0.25), true});
  return board;
}

[[nodiscard]] Board make_group_conflict_demo() {
  auto board = make_route_conflict_demo();
  board.source_revision = "built-in-group-demo-v1";
  board.footprints[0].position = mm(20.0, 40.0);
  board.footprints[1].position = mm(80.0, 40.0);
  board.footprints[1].locked = true;
  const NetId second{"N3"};
  board.nets.push_back(Net{second, "SECOND", 0});
  Footprint companion;
  companion.id = ObjectId{"fp-companion"};
  companion.reference = "U2";
  companion.position = mm(20.0, 20.0);
  companion.courtyard = rectangle(2.0, 2.0);
  companion.pads = {{ObjectId{"pad-companion"}, second, Point{}, {LayerId{"F.Cu"}}}};
  Footprint fixed = companion;
  fixed.id = ObjectId{"fp-second-fixed"};
  fixed.reference = "J3";
  fixed.position = mm(80.0, 20.0);
  fixed.locked = true;
  fixed.pads.front().id = ObjectId{"pad-second-fixed"};
  board.footprints.push_back(std::move(companion));
  board.footprints.push_back(std::move(fixed));
  board.placement_groups = {flux::PlacementGroup{flux::PlacementGroupId{"motion-stage"},
                                                 "Motion stage",
                                                 {ObjectId{"fp-left"}, ObjectId{"fp-companion"}}}};
  return board;
}

[[nodiscard]] Board make_constraint_component_conflict_demo() {
  auto board = make_group_conflict_demo();
  board.source_revision = "built-in-constraint-component-demo-v1";
  Footprint anchor;
  anchor.id = ObjectId{"fp-external-anchor"};
  anchor.reference = "U3";
  anchor.position = mm(20.0, 30.0);
  anchor.courtyard = rectangle(2.0, 2.0);
  board.footprints.push_back(std::move(anchor));
  board.relative_placement_constraints = {flux::RelativePlacementConstraint{
      flux::RelativePlacementConstraintId{"stage-anchor"}, "Stage anchor", ObjectId{"fp-left"},
      ObjectId{"fp-external-anchor"}, mm(0.0, -10.0), Nanometres::from_mm(10.0), Microdegrees{},
      Nanometres{}, Nanometres{}, std::nullopt, std::nullopt}};
  return board;
}

[[nodiscard]] Board make_global_routing_guidance_demo() {
  const LayerId front{"F.Cu"};
  const NetId fixed_net{"fixed-net"};
  const NetId movable_net{"movable-net"};
  Board board;
  board.source_revision = "built-in-global-routing-guidance-v1";
  board.outline = Polygon{{mm(0.0, 0.0), mm(100.0, 0.0), mm(100.0, 20.0), mm(0.0, 20.0)}};
  board.layers = {{front, "F.Cu", LayerKind::copper, 0}};
  board.nets = {{fixed_net, "FIXED", 0}, {movable_net, "MOVABLE", 0}};

  Footprint left;
  left.id = ObjectId{"fixed-left"};
  left.reference = "J1";
  left.position = mm(20.0, 9.0);
  left.locked = true;
  left.courtyard = rectangle(0.1, 0.1);
  left.pads = {{ObjectId{"fixed-left-pad"}, fixed_net, Point{}, {front}}};
  Footprint right = left;
  right.id = ObjectId{"fixed-right"};
  right.reference = "J2";
  right.position = mm(80.0, 9.0);
  right.pads.front().id = ObjectId{"fixed-right-pad"};

  Footprint movable;
  movable.id = ObjectId{"movable-route"};
  movable.reference = "U1";
  movable.position = mm(50.0, 11.0);
  movable.courtyard = rectangle(0.1, 0.1);
  // A single terminal keeps this fixture focused on moving the physical blocker; the guided
  // placement pass is evaluated by the shorter fixed-net route it releases, not by a second
  // unrelated connection through the keepout.
  movable.pads = {{ObjectId{"movable-anchor-pad"}, movable_net, Point{}, {front}}};
  // This is intentionally a routing keepout rather than a placement courtyard: it is a moving
  // physical blocker of the fixed net and exercises owner-attributed placement feedback.
  movable.keepouts = {{ObjectId{"movable-route-keepout"},
                       rectangle(0.5, 3.0),
                       {front},
                       flux::KeepoutRules{true, false, false, false, false},
                       "moving route blocker"}};
  board.footprints = {std::move(left), std::move(right), std::move(movable)};
  return board;
}

[[nodiscard]] Board make_rotation_conflict_demo() {
  auto board = make_demo_board(false, false);
  board.source_revision = "built-in-rotation-demo-v1";
  board.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0}};
  board.footprints.front().locked = true;
  board.footprints[1].position = mm(55.0, 40.0);
  board.footprints[1].rotation = Microdegrees{};
  board.footprints[1].orientation_constraints.allowed_rotations = {
      Microdegrees{}, Microdegrees::from_degrees(180.0)};
  board.footprints[1].orientation_constraints.allow_back = false;
  board.footprints[1].pads.front().local_position = mm(10.0, 0.0);
  const NetId blocker{"N2"};
  board.nets.push_back(Net{blocker, "LOCKED_BLOCKER", 0});
  board.tracks.push_back({ObjectId{"locked-rotation-barrier"}, blocker, LayerId{"F.Cu"},
                          mm(50.0, 0.625), mm(50.0, 79.375), Nanometres::from_mm(0.25), true});
  return board;
}

[[nodiscard]] Board make_quality_refinement_demo() {
  auto board = make_demo_board(false, false);
  board.source_revision = "built-in-quality-refinement-demo-v1";
  board.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0}};
  board.footprints.front().locked = true;
  board.footprints.front().courtyard = rectangle(10.0, 10.0);
  board.footprints[1].courtyard = rectangle(10.0, 10.0);
  return board;
}

[[nodiscard]] Board make_incremental_route_demo() {
  auto board = make_demo_board(false, false);
  board.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0}};
  const NetId second{"N2"};
  board.nets.push_back(Net{second, "SECOND", 0});
  Footprint left;
  left.id = ObjectId{"fp-second-left"};
  left.reference = "J3";
  left.position = mm(20.0, 20.0);
  left.courtyard = rectangle(2.0, 2.0);
  left.pads = {{ObjectId{"pad-second-left"}, second, Point{}, {LayerId{"F.Cu"}}}};
  Footprint right = left;
  right.id = ObjectId{"fp-second-right"};
  right.reference = "J4";
  right.position = mm(80.0, 20.0);
  right.pads.front().id = ObjectId{"pad-second-right"};
  board.footprints.push_back(std::move(left));
  board.footprints.push_back(std::move(right));
  return board;
}

void print_import_result(const flux::io::KicadImportResult& imported) {
  std::size_t warnings = 0;
  std::map<std::string, std::size_t> diagnostic_counts;
  for (const auto& diagnostic : imported.diagnostics) {
    ++diagnostic_counts[diagnostic.code];
    if (diagnostic.severity == flux::io::ImportSeverity::warning) {
      ++warnings;
    }
  }
  const auto errors = imported.diagnostics.size() - warnings;
  std::cout << "{\n"
            << "  \"ok\": " << (imported.ok() ? "true" : "false") << ",\n"
            << "  \"strict\": " << (imported.strict() ? "true" : "false") << ",\n"
            << "  \"lossy\": " << (imported.lossy ? "true" : "false") << ",\n"
            << "  \"import_errors\": " << errors << ",\n"
            << "  \"import_warnings\": " << warnings;
  if (imported.board) {
    const auto evaluation = flux::evaluate_board(*imported.board);
    auto keepout_count = imported.board->keepouts.size();
    std::size_t zone_fill_regions = 0;
    std::size_t zone_fill_holes = 0;
    for (const auto& zone : imported.board->zones) {
      zone_fill_regions += zone.fills.size();
      for (const auto& fill : zone.fills) {
        zone_fill_holes += fill.holes.size();
      }
    }
    for (const auto& footprint : imported.board->footprints) {
      keepout_count += footprint.keepouts.size();
    }
    std::map<std::string, std::size_t> validation_counts;
    for (const auto& issue : evaluation.validation.issues) {
      ++validation_counts[issue.code];
    }
    std::cout << ",\n"
              << "  \"source_revision\": " << json_quote(imported.board->source_revision) << ",\n"
              << "  \"board_cutouts\": " << imported.board->cutouts.size() << ",\n"
              << "  \"layers\": " << imported.board->layers.size() << ",\n"
              << "  \"nets\": " << imported.board->nets.size() << ",\n"
              << "  \"net_classes\": " << imported.board->net_classes.size() << ",\n"
              << "  \"differential_pairs\": " << imported.board->differential_pairs.size() << ",\n"
              << "  \"routing_buses\": " << imported.board->routing_buses.size() << ",\n"
              << "  \"placement_groups\": " << imported.board->placement_groups.size() << ",\n"
              << "  \"placement_regions\": " << imported.board->placement_regions.size() << ",\n"
              << "  \"relative_placement_constraints\": "
              << imported.board->relative_placement_constraints.size() << ",\n"
              << "  \"footprints\": " << imported.board->footprints.size() << ",\n"
              << "  \"tracks\": " << imported.board->tracks.size() << ",\n"
              << "  \"track_arcs\": " << imported.board->track_arcs.size() << ",\n"
              << "  \"vias\": " << imported.board->vias.size() << ",\n"
              << "  \"zones\": " << imported.board->zones.size() << ",\n"
              << "  \"zone_fill_regions\": " << zone_fill_regions << ",\n"
              << "  \"zone_fill_holes\": " << zone_fill_holes << ",\n"
              << "  \"keepouts\": " << keepout_count << ",\n"
              << "  \"core_validation_errors\": " << evaluation.validation.error_count() << ",\n"
              << "  \"open_connections\": " << evaluation.connectivity.open_connections
              << ",\n  \"connectivity_open_nets\": [";
    std::size_t open_net_index = 0U;
    for (const auto& connectivity : evaluation.connectivity.nets) {
      if (connectivity.open_connections == 0U) {
        continue;
      }
      const auto net = std::find_if(
          imported.board->nets.begin(), imported.board->nets.end(),
          [&connectivity](const auto& candidate) { return candidate.id == connectivity.net; });
      std::cout << (open_net_index++ == 0U ? "" : ",")
                << "\n    {\"id\": " << json_quote(connectivity.net.value()) << ", \"name\": "
                << json_quote(net == imported.board->nets.end() ? std::string{} : net->name)
                << ", \"pad_count\": " << connectivity.pad_count
                << ", \"connected_pad_groups\": " << connectivity.connected_pad_groups
                << ", \"open_connections\": " << connectivity.open_connections << '}';
    }
    if (open_net_index > 0U) {
      std::cout << '\n';
    }
    std::cout << "  ],\n  \"core_issue_counts\": {";
    std::size_t issue_index = 0;
    for (const auto& [code, count] : validation_counts) {
      std::cout << (issue_index++ == 0U ? "" : ", ") << '"' << code << "\": " << count;
    }
    std::cout << '}';
  }
  std::cout << ",\n  \"import_issue_counts\": {";
  std::size_t diagnostic_index = 0;
  for (const auto& [code, count] : diagnostic_counts) {
    std::cout << (diagnostic_index++ == 0U ? "" : ", ") << '"' << code << "\": " << count;
  }
  std::cout << "}\n}\n";
}

[[nodiscard]] std::string_view
kicad_change_operation_name(flux::io::KicadChangeOperation operation) {
  switch (operation) {
  case flux::io::KicadChangeOperation::create:
    return "create";
  case flux::io::KicadChangeOperation::update:
    return "update";
  case flux::io::KicadChangeOperation::remove:
    return "remove";
  }
  return "unknown";
}

void print_point_json(Point point) {
  std::cout << '[' << point.x.value() << ", " << point.y.value() << ']';
}

void print_kicad_footprint_state(const flux::io::KicadFootprintTransformState& state) {
  std::cout << "{\"position_nm\": ";
  print_point_json(state.position);
  std::cout << ", \"rotation_microdegrees\": " << state.rotation.value()
            << ", \"side\": " << json_quote(board_side_name(state.side)) << '}';
}

void print_kicad_change_state(const flux::io::KicadTrackChangeState& state) {
  std::cout << "{\"id\": " << json_quote(state.id.value())
            << ", \"net\": " << json_quote(state.net_name)
            << ", \"layer\": " << json_quote(state.layer.value()) << ", \"start_nm\": ";
  print_point_json(state.start);
  std::cout << ", \"end_nm\": ";
  print_point_json(state.end);
  std::cout << ", \"width_nm\": " << state.width.value()
            << ", \"locked\": " << (state.locked ? "true" : "false") << '}';
}

void print_kicad_change_state(const flux::io::KicadTrackArcChangeState& state) {
  std::cout << "{\"id\": " << json_quote(state.id.value())
            << ", \"net\": " << json_quote(state.net_name)
            << ", \"layer\": " << json_quote(state.layer.value()) << ", \"start_nm\": ";
  print_point_json(state.start);
  std::cout << ", \"middle_nm\": ";
  print_point_json(state.middle);
  std::cout << ", \"end_nm\": ";
  print_point_json(state.end);
  std::cout << ", \"width_nm\": " << state.width.value()
            << ", \"locked\": " << (state.locked ? "true" : "false") << '}';
}

void print_kicad_change_state(const flux::io::KicadViaChangeState& state) {
  std::cout << "{\"id\": " << json_quote(state.id.value())
            << ", \"net\": " << json_quote(state.net_name) << ", \"position_nm\": ";
  print_point_json(state.position);
  std::cout << ", \"diameter_nm\": " << state.diameter.value()
            << ", \"drill_nm\": " << state.drill.value() << ", \"layers\": [";
  for (std::size_t index = 0U; index < state.layers.size(); ++index) {
    std::cout << (index == 0U ? "" : ", ") << json_quote(state.layers[index].value());
  }
  std::cout << "], \"locked\": " << (state.locked ? "true" : "false") << '}';
}

template <typename State>
void print_kicad_routing_changes(const std::vector<flux::io::KicadRoutingChange<State>>& changes) {
  std::cout << '[';
  for (std::size_t index = 0U; index < changes.size(); ++index) {
    const auto& change = changes[index];
    std::cout << (index == 0U ? "" : ",") << "\n    {\"operation\": "
              << json_quote(kicad_change_operation_name(change.operation)) << ", \"source\": ";
    if (change.source) {
      print_kicad_change_state(*change.source);
    } else {
      std::cout << "null";
    }
    std::cout << ", \"candidate\": ";
    if (change.candidate) {
      print_kicad_change_state(*change.candidate);
    } else {
      std::cout << "null";
    }
    std::cout << '}';
  }
  if (!changes.empty()) {
    std::cout << '\n';
  }
  std::cout << "  ]";
}

void print_kicad_change_set(const flux::io::KicadChangeSetResult& result) {
  std::cout << "{\n  \"ok\": " << (result.ok() ? "true" : "false");
  if (result.change_set) {
    const auto& change_set = *result.change_set;
    std::cout << ",\n  \"version\": " << change_set.version
              << ",\n  \"board_schema\": " << change_set.board_schema
              << ",\n  \"source_revision\": " << json_quote(change_set.source_revision)
              << ",\n  \"source_inventory\": {\n    \"footprints\": [";
    for (std::size_t index = 0U; index < change_set.source_footprints.size(); ++index) {
      const auto& footprint = change_set.source_footprints[index];
      std::cout << (index == 0U ? "" : ",")
                << "\n      {\"id\": " << json_quote(footprint.id.value())
                << ", \"reference\": " << json_quote(footprint.reference) << ", \"state\": ";
      print_kicad_footprint_state(footprint.state);
      std::cout << '}';
    }
    if (!change_set.source_footprints.empty()) {
      std::cout << '\n';
    }
    std::cout << "    ],\n    \"tracks\": [";
    for (std::size_t index = 0U; index < change_set.source_tracks.size(); ++index) {
      std::cout << (index == 0U ? "" : ",\n      ");
      print_kicad_change_state(change_set.source_tracks[index]);
    }
    if (!change_set.source_tracks.empty()) {
      std::cout << '\n';
    }
    std::cout << "    ],\n    \"track_arcs\": [";
    for (std::size_t index = 0U; index < change_set.source_track_arcs.size(); ++index) {
      std::cout << (index == 0U ? "" : ",\n      ");
      print_kicad_change_state(change_set.source_track_arcs[index]);
    }
    if (!change_set.source_track_arcs.empty()) {
      std::cout << '\n';
    }
    std::cout << "    ],\n    \"vias\": [";
    for (std::size_t index = 0U; index < change_set.source_vias.size(); ++index) {
      std::cout << (index == 0U ? "" : ",\n      ");
      print_kicad_change_state(change_set.source_vias[index]);
    }
    if (!change_set.source_vias.empty()) {
      std::cout << '\n';
    }
    std::cout << "    ]\n  },\n  \"footprint_transforms\": [";
    for (std::size_t index = 0U; index < change_set.footprint_transforms.size(); ++index) {
      const auto& change = change_set.footprint_transforms[index];
      std::cout << (index == 0U ? "" : ",") << "\n    {\"id\": " << json_quote(change.id.value())
                << ", \"reference\": " << json_quote(change.reference) << ", \"source\": ";
      print_kicad_footprint_state(change.source);
      std::cout << ", \"candidate\": ";
      print_kicad_footprint_state(change.candidate);
      std::cout << '}';
    }
    if (!change_set.footprint_transforms.empty()) {
      std::cout << '\n';
    }
    std::cout << "  ],\n  \"tracks\": ";
    print_kicad_routing_changes(change_set.tracks);
    std::cout << ",\n  \"track_arcs\": ";
    print_kicad_routing_changes(change_set.track_arcs);
    std::cout << ",\n  \"vias\": ";
    print_kicad_routing_changes(change_set.vias);
  }
  std::cout << ",\n  \"diagnostics\": [";
  for (std::size_t index = 0U; index < result.diagnostics.size(); ++index) {
    const auto& diagnostic = result.diagnostics[index];
    std::cout << (index == 0U ? "" : ",") << "\n    {\"severity\": "
              << json_quote(diagnostic.severity == flux::io::ExportSeverity::error ? "error"
                                                                                   : "warning")
              << ", \"code\": " << json_quote(diagnostic.code)
              << ", \"message\": " << json_quote(diagnostic.message) << ", \"object_ids\": [";
    for (std::size_t object_index = 0U; object_index < diagnostic.object_ids.size();
         ++object_index) {
      std::cout << (object_index == 0U ? "" : ", ")
                << json_quote(diagnostic.object_ids[object_index]);
    }
    std::cout << "]}";
  }
  if (!result.diagnostics.empty()) {
    std::cout << '\n';
  }
  std::cout << "  ]\n}\n";
}

void print_specctra_diagnostics(const std::vector<flux::io::SpecctraDiagnostic>& diagnostics) {
  std::cout << "[";
  for (std::size_t index = 0U; index < diagnostics.size(); ++index) {
    const auto& item = diagnostics[index];
    std::cout << (index == 0U ? "" : ",") << "\n    {\"severity\": "
              << json_quote(item.severity == flux::io::SpecctraSeverity::error ? "error"
                                                                               : "warning")
              << ", \"code\": " << json_quote(item.code)
              << ", \"message\": " << json_quote(item.message) << ", \"line\": " << item.line
              << ", \"column\": " << item.column << "}";
  }
  if (!diagnostics.empty()) {
    std::cout << '\n';
  }
  std::cout << "  ]";
}

void print_specctra_export(const flux::io::SpecctraDsnExportResult& result, bool artifact_written) {
  std::cout << "{\n  \"ok\": " << (result.ok() ? "true" : "false")
            << ",\n  \"artifact_written\": " << (artifact_written ? "true" : "false")
            << ",\n  \"layers\": " << result.exported_layers
            << ",\n  \"pins\": " << result.exported_pins
            << ",\n  \"protected_tracks\": " << result.protected_tracks
            << ",\n  \"protected_vias\": " << result.protected_vias << ",\n  \"diagnostics\": ";
  print_specctra_diagnostics(result.diagnostics);
  std::cout << "\n}\n";
}

void print_specctra_import(const flux::io::SpecctraSesImportResult& result, bool artifact_written,
                           const flux::io::KicadExportResult* export_result = nullptr) {
  std::cout << "{\n  \"ok\": " << (result.ok() && artifact_written ? "true" : "false")
            << ",\n  \"artifact_written\": " << (artifact_written ? "true" : "false")
            << ",\n  \"imported_tracks\": " << result.imported_tracks
            << ",\n  \"imported_vias\": " << result.imported_vias
            << ",\n  \"verified_protected_tracks\": " << result.verified_protected_tracks
            << ",\n  \"verified_protected_vias\": " << result.verified_protected_vias;
  if (result.evaluation) {
    std::cout << ",\n  \"open_connections\": " << result.evaluation->connectivity.open_connections
              << ",\n  \"copper_violations\": " << result.evaluation->copper_drc.violations.size();
  }
  std::cout << ",\n  \"diagnostics\": ";
  print_specctra_diagnostics(result.diagnostics);
  if (export_result != nullptr) {
    std::cout << ",\n  \"kicad_export_ok\": " << (export_result->ok() ? "true" : "false")
              << ",\n  \"kicad_export_diagnostics\": [";
    for (std::size_t index = 0U; index < export_result->diagnostics.size(); ++index) {
      const auto& item = export_result->diagnostics[index];
      std::cout << (index == 0U ? "" : ", ") << json_quote(item.code);
    }
    std::cout << ']';
  }
  std::cout << "\n}\n";
}

[[nodiscard]] std::optional<std::string> read_binary_file(const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    return std::nullopt;
  }
  std::string contents{std::istreambuf_iterator<char>{input}, std::istreambuf_iterator<char>{}};
  if (!input.good() && !input.eof()) {
    return std::nullopt;
  }
  return contents;
}

void print_netclass_snapshot_result(const flux::io::KicadNetClassSnapshotResult& snapshot) {
  const auto errors = static_cast<std::size_t>(std::count_if(
      snapshot.diagnostics.begin(), snapshot.diagnostics.end(), [](const auto& diagnostic) {
        return diagnostic.severity == flux::io::NetClassSnapshotSeverity::error;
      }));
  std::cout << "{\n"
            << "  \"ok\": " << (snapshot.ok() ? "true" : "false") << ",\n"
            << "  \"netclass_snapshot_errors\": " << errors << ",\n"
            << "  \"api_version\": " << json_quote(snapshot.api_version) << ",\n"
            << "  \"applied_classes\": " << snapshot.applied_classes << ",\n"
            << "  \"applied_nets\": " << snapshot.applied_nets
            << ",\n  \"netclass_snapshot_diagnostics\": [";
  for (std::size_t index = 0; index < snapshot.diagnostics.size(); ++index) {
    const auto& diagnostic = snapshot.diagnostics[index];
    std::cout << (index == 0U ? "" : ",") << "\n    {\"severity\": "
              << json_quote(diagnostic.severity == flux::io::NetClassSnapshotSeverity::error
                                ? "error"
                                : "warning")
              << ", \"code\": " << json_quote(diagnostic.code)
              << ", \"message\": " << json_quote(diagnostic.message)
              << ", \"line\": " << diagnostic.line << ", \"column\": " << diagnostic.column
              << ", \"object_ids\": [";
    for (std::size_t object_index = 0; object_index < diagnostic.object_ids.size();
         ++object_index) {
      std::cout << (object_index == 0U ? "" : ", ")
                << json_quote(diagnostic.object_ids[object_index]);
    }
    std::cout << "]}";
  }
  if (!snapshot.diagnostics.empty()) {
    std::cout << '\n';
  }
  std::cout << "  ]\n}\n";
}

struct KicadCommandTail {
  std::optional<std::filesystem::path> netclass_snapshot;
  std::optional<std::filesystem::path> differential_pair_constraints;
  std::optional<std::filesystem::path> routing_bus_constraints;
  std::optional<std::filesystem::path> placement_envelope_constraints;
  std::optional<std::filesystem::path> placement_group_constraints;
  std::optional<std::filesystem::path> placement_region_constraints;
  std::optional<std::filesystem::path> relative_placement_constraints;
  std::optional<std::filesystem::path> placement_move_priorities;
  std::optional<std::filesystem::path> output;
  std::vector<std::string> route_net_names;
  std::vector<std::string> priority_route_net_names;
  std::optional<std::size_t> max_placement_passes;
  std::optional<std::size_t> max_candidate_evaluations;
  std::optional<std::size_t> max_candidate_evaluation_workers;
  std::optional<std::size_t> max_full_routing_candidates_per_wave;
  std::optional<std::size_t> candidate_evaluation_offset;
  std::optional<Nanometres> move_step;
  std::optional<Nanometres> grid_pitch;
  std::optional<Nanometres> via_site_pitch;
  std::optional<std::size_t> via_site_phase_x;
  std::optional<std::size_t> via_site_phase_y;
  std::optional<std::size_t> max_router_iterations;
  std::optional<std::size_t> max_final_routability_feedback_passes;
  std::optional<std::size_t> xpbd_iterations;
  std::optional<Nanometres> xpbd_maximum_step;
  std::optional<std::size_t> max_static_seed_workers;
  std::optional<std::size_t> max_expansions_per_connection;
  std::optional<std::uint32_t> expansion_retry_heuristic_weight_per_mille;
  std::optional<std::uint32_t> present_congestion_multiplier_per_mille;
  std::optional<std::uint64_t> max_present_congestion_scale_per_mille;
  std::optional<std::size_t> present_congestion_escalation_start_iteration;
  std::optional<std::size_t> hard_capacity_start_iteration;
  std::optional<std::size_t> max_hard_capacity_ripup_attempts_per_pass;
  std::optional<std::size_t> max_hard_capacity_ripup_nets_per_attempt;
  std::optional<std::size_t> max_hard_capacity_conflict_set_states_per_attempt;
  std::optional<std::size_t> max_hard_capacity_blocker_reuse_invalidations_per_failed_net;
  std::optional<std::size_t> pin_escape_grid_radius;
  std::optional<std::size_t> max_pin_access_nodes_per_terminal;
  std::optional<Nanometres> local_pin_escape_pitch;
  std::optional<Nanometres> clearance_shadow_cost_quantum;
  std::optional<std::uint32_t> capacity_reserve_per_mille;
  std::optional<std::size_t> max_local_pin_escape_expansions_per_terminal;
  std::optional<Nanometres> terminal_neck_down_width;
  std::optional<Nanometres> max_terminal_neck_down_length;
  std::optional<Nanometres> medial_neck_down_width;
  std::optional<Nanometres> max_medial_neck_down_length;
  std::optional<Nanometres> corridor_neck_down_width;
  std::optional<Nanometres> max_corridor_neck_down_length;
  std::optional<Nanometres> max_voronoi_resource_segment_length;
  std::optional<std::size_t> max_corridor_neck_down_states;
  std::optional<std::size_t> selective_full_reroute_interval;
  std::optional<std::uint64_t> width_reduction_cost_per_grid_step;
  std::optional<std::uint64_t> river_following_cost;
  std::optional<std::uint64_t> former_route_memory_cost;
  std::optional<std::uint64_t> terminal_escape_reservation_cost;
  std::optional<std::uint64_t> terminal_escape_proximity_cost;
  std::optional<std::uint64_t> capacity_reserve_cost;
  std::optional<std::size_t> max_terminal_escape_reservation_resources;
  std::optional<std::string> telemetry_listen;
  std::optional<std::uint32_t> telemetry_rate_hz;
  std::optional<flux::RouterTelemetryLevel> telemetry_level;
  std::optional<std::filesystem::path> telemetry_record;
  bool strict_medial_topology{false};
  bool diagonal_grid_edges{false};
  bool crossing_safe_diagonal_grid_edges{false};
  bool sampled_route_clearance_reservations{false};
  bool direct_medial_edges{false};
  bool negotiate_voronoi_lane_conflicts{false};
  bool diversify_pin_access_rings{false};
  bool multisource_steiner_growth{false};
  bool atomic_complete_net_routing{false};
  bool rollback_failed_hard_capacity_ripup{false};
  bool defer_hard_capacity_ripup_victims_to_pass_tail{false};
  bool protect_recovered_hard_capacity_ripup_nets{false};
  bool selective_congestion_reroute{false};
  bool repair_sampled_conflicts{false};
  bool xpbd_relaxation{false};
  bool xpbd_feedback{false};
  bool xpbd_cpu{false};
  bool reuse_weighted_search_frontier{false};
  bool physical_overflow_objective{false};
  bool physical_overflow_history{false};
  bool physical_envelope_ordering{false};
  bool search_diversification{false};
  bool circular_arc_refinement{false};
  bool differential_pair_skew_tuning{false};
  bool differential_pair_skew_tuning_batch_assignment{false};
  bool enable_swap_moves{false};
  bool enable_group_moves{false};
  bool enable_relative_component_moves{false};
  bool enable_global_routing_guidance{false};
  bool enable_assignment_legalization{false};
  bool tentative_placement_envelopes{false};
  bool suggested_placement_move_priorities{false};
  bool refillable_zone_fills{false};
  bool write_incomplete_preview{false};
  bool diagnostic_lossy_preview{false};
  bool telemetry{true};
};

[[nodiscard]] std::optional<KicadCommandTail> parse_kicad_command_tail(
    int argc, char** argv, int first, bool allow_output, bool allow_placement_moves = false,
    bool allow_global_routing_guidance = false, bool allow_refillable_zones = false,
    bool allow_incomplete_preview = false) {
  KicadCommandTail result;
  for (auto index = first; index < argc; ++index) {
    const std::string_view argument{argv[index]};
    if (argument == "--netclasses") {
      if (result.netclass_snapshot || index + 1 >= argc) {
        return std::nullopt;
      }
      result.netclass_snapshot = std::filesystem::path{argv[++index]};
    } else if (argument == "--route-net") {
      if (index + 1 >= argc || std::string_view{argv[index + 1]}.empty()) {
        return std::nullopt;
      }
      result.route_net_names.emplace_back(argv[++index]);
    } else if (argument == "--route-first") {
      if (index + 1 >= argc || std::string_view{argv[index + 1]}.empty()) {
        return std::nullopt;
      }
      result.priority_route_net_names.emplace_back(argv[++index]);
    } else if (argument == "--repair-sampled-conflicts") {
      if (result.repair_sampled_conflicts) {
        return std::nullopt;
      }
      result.repair_sampled_conflicts = true;
    } else if (argument == "--xpbd-relaxation") {
      result.xpbd_relaxation = true;
    } else if (argument == "--xpbd-feedback") {
      result.xpbd_relaxation = true;
      result.xpbd_feedback = true;
    } else if (argument == "--xpbd-cpu") {
      result.xpbd_relaxation = true;
      result.xpbd_cpu = true;
    } else if (argument == "--xpbd-iterations") {
      if (result.xpbd_iterations || index + 1 >= argc)
        return std::nullopt;
      std::size_t value = 0U;
      const std::string_view token{argv[++index]};
      const auto parsed = std::from_chars(token.data(), token.data() + token.size(), value);
      if (parsed.ec != std::errc{} || parsed.ptr != token.data() + token.size() || value == 0U ||
          value > 4'096U) {
        return std::nullopt;
      }
      result.xpbd_iterations = value;
    } else if (argument == "--xpbd-max-step-nm") {
      if (result.xpbd_maximum_step || index + 1 >= argc)
        return std::nullopt;
      std::int64_t value = 0;
      const std::string_view token{argv[++index]};
      const auto parsed = std::from_chars(token.data(), token.data() + token.size(), value);
      if (parsed.ec != std::errc{} || parsed.ptr != token.data() + token.size() || value <= 0) {
        return std::nullopt;
      }
      result.xpbd_maximum_step = Nanometres{value};
    } else if (argument == "--telemetry") {
      result.telemetry = true;
    } else if (argument == "--no-telemetry") {
      result.telemetry = false;
    } else if (argument == "--telemetry-listen") {
      if (result.telemetry_listen || index + 1 >= argc ||
          std::string_view{argv[index + 1]}.empty()) {
        return std::nullopt;
      }
      result.telemetry_listen = std::string{argv[++index]};
    } else if (argument == "--telemetry-rate") {
      if (result.telemetry_rate_hz || index + 1 >= argc) {
        return std::nullopt;
      }
      std::uint32_t value = 0U;
      const std::string_view token{argv[++index]};
      const auto parsed = std::from_chars(token.data(), token.data() + token.size(), value);
      if (parsed.ec != std::errc{} || parsed.ptr != token.data() + token.size() || value == 0U ||
          value > 1'000U) {
        return std::nullopt;
      }
      result.telemetry_rate_hz = value;
    } else if (argument == "--telemetry-level") {
      if (result.telemetry_level || index + 1 >= argc) {
        return std::nullopt;
      }
      const std::string_view value{argv[++index]};
      if (value == "normal") {
        result.telemetry_level = flux::RouterTelemetryLevel::normal;
      } else if (value == "diagnostic") {
        result.telemetry_level = flux::RouterTelemetryLevel::diagnostic;
      } else if (value == "trace") {
        result.telemetry_level = flux::RouterTelemetryLevel::trace;
      } else {
        return std::nullopt;
      }
    } else if (argument == "--telemetry-record") {
      if (result.telemetry_record || index + 1 >= argc) {
        return std::nullopt;
      }
      result.telemetry_record = std::filesystem::path{argv[++index]};
    } else if (argument == "--diffpairs") {
      if (result.differential_pair_constraints || index + 1 >= argc) {
        return std::nullopt;
      }
      result.differential_pair_constraints = std::filesystem::path{argv[++index]};
    } else if (argument == "--buses") {
      if (result.routing_bus_constraints || index + 1 >= argc) {
        return std::nullopt;
      }
      result.routing_bus_constraints = std::filesystem::path{argv[++index]};
    } else if (argument == "--envelopes") {
      if (result.placement_envelope_constraints || index + 1 >= argc) {
        return std::nullopt;
      }
      result.placement_envelope_constraints = std::filesystem::path{argv[++index]};
    } else if (argument == "--placement-groups") {
      if (result.placement_group_constraints || index + 1 >= argc) {
        return std::nullopt;
      }
      result.placement_group_constraints = std::filesystem::path{argv[++index]};
    } else if (argument == "--placement-regions") {
      if (result.placement_region_constraints || index + 1 >= argc) {
        return std::nullopt;
      }
      result.placement_region_constraints = std::filesystem::path{argv[++index]};
    } else if (argument == "--relative-placements") {
      if (result.relative_placement_constraints || index + 1 >= argc) {
        return std::nullopt;
      }
      result.relative_placement_constraints = std::filesystem::path{argv[++index]};
    } else if (argument == "--placement-move-priorities" && allow_placement_moves) {
      if (result.placement_move_priorities || index + 1 >= argc) {
        return std::nullopt;
      }
      result.placement_move_priorities = std::filesystem::path{argv[++index]};
    } else if ((argument == "--max-placement-passes" || argument == "--max-candidate-evaluations" ||
                argument == "--max-candidate-evaluation-workers" ||
                argument == "--max-full-routing-candidates-per-wave") &&
               allow_placement_moves) {
      auto* target = &result.max_candidate_evaluations;
      if (argument == "--max-placement-passes") {
        target = &result.max_placement_passes;
      } else if (argument == "--max-candidate-evaluation-workers") {
        target = &result.max_candidate_evaluation_workers;
      } else if (argument == "--max-full-routing-candidates-per-wave") {
        target = &result.max_full_routing_candidates_per_wave;
      }
      if (target->has_value() || index + 1 >= argc) {
        return std::nullopt;
      }
      std::size_t value = 0U;
      const std::string_view token{argv[++index]};
      const auto parsed = std::from_chars(token.data(), token.data() + token.size(), value);
      if (parsed.ec != std::errc{} || parsed.ptr != token.data() + token.size() || value == 0U) {
        return std::nullopt;
      }
      *target = value;
    } else if (argument == "--candidate-evaluation-offset" && allow_placement_moves) {
      if (result.candidate_evaluation_offset || index + 1 >= argc) {
        return std::nullopt;
      }
      std::size_t value = 0U;
      const std::string_view token{argv[++index]};
      const auto parsed = std::from_chars(token.data(), token.data() + token.size(), value);
      if (parsed.ec != std::errc{} || parsed.ptr != token.data() + token.size()) {
        return std::nullopt;
      }
      result.candidate_evaluation_offset = value;
    } else if ((argument == "--move-step-nm" && allow_placement_moves) ||
               argument == "--grid-pitch-nm" || argument == "--via-site-pitch-nm" ||
               argument == "--local-pin-escape-pitch-nm" ||
               argument == "--clearance-shadow-cost-quantum-nm" ||
               argument == "--terminal-neck-down-width-nm" ||
               argument == "--max-terminal-neck-down-length-nm" ||
               argument == "--medial-neck-down-width-nm" ||
               argument == "--max-medial-neck-down-length-nm" ||
               argument == "--corridor-neck-down-width-nm" ||
               argument == "--max-corridor-neck-down-length-nm" ||
               argument == "--max-voronoi-resource-segment-length-nm") {
      if (index + 1 >= argc) {
        return std::nullopt;
      }
      std::int64_t value = 0;
      const std::string_view token{argv[++index]};
      const auto parsed = std::from_chars(token.data(), token.data() + token.size(), value);
      if (parsed.ec != std::errc{} || parsed.ptr != token.data() + token.size() || value <= 0) {
        return std::nullopt;
      }
      auto* target = &result.max_corridor_neck_down_length;
      if (argument == "--move-step-nm") {
        target = &result.move_step;
      } else if (argument == "--grid-pitch-nm") {
        target = &result.grid_pitch;
      } else if (argument == "--via-site-pitch-nm") {
        target = &result.via_site_pitch;
      } else if (argument == "--local-pin-escape-pitch-nm") {
        target = &result.local_pin_escape_pitch;
      } else if (argument == "--clearance-shadow-cost-quantum-nm") {
        target = &result.clearance_shadow_cost_quantum;
      } else if (argument == "--terminal-neck-down-width-nm") {
        target = &result.terminal_neck_down_width;
      } else if (argument == "--max-terminal-neck-down-length-nm") {
        target = &result.max_terminal_neck_down_length;
      } else if (argument == "--medial-neck-down-width-nm") {
        target = &result.medial_neck_down_width;
      } else if (argument == "--max-medial-neck-down-length-nm") {
        target = &result.max_medial_neck_down_length;
      } else if (argument == "--corridor-neck-down-width-nm") {
        target = &result.corridor_neck_down_width;
      } else if (argument == "--max-voronoi-resource-segment-length-nm") {
        target = &result.max_voronoi_resource_segment_length;
      }
      if (*target) {
        return std::nullopt;
      }
      *target = Nanometres{value};
    } else if (argument == "--width-reduction-cost-per-grid-step" ||
               argument == "--river-following-cost" || argument == "--former-route-memory-cost" ||
               argument == "--terminal-escape-reservation-cost" ||
               argument == "--terminal-escape-proximity-cost" ||
               argument == "--capacity-reserve-cost") {
      auto* target = &result.width_reduction_cost_per_grid_step;
      if (argument == "--river-following-cost") {
        target = &result.river_following_cost;
      } else if (argument == "--former-route-memory-cost") {
        target = &result.former_route_memory_cost;
      } else if (argument == "--terminal-escape-reservation-cost") {
        target = &result.terminal_escape_reservation_cost;
      } else if (argument == "--terminal-escape-proximity-cost") {
        target = &result.terminal_escape_proximity_cost;
      } else if (argument == "--capacity-reserve-cost") {
        target = &result.capacity_reserve_cost;
      }
      if (target->has_value() || index + 1 >= argc) {
        return std::nullopt;
      }
      std::uint64_t value = 0U;
      const std::string_view token{argv[++index]};
      const auto parsed = std::from_chars(token.data(), token.data() + token.size(), value);
      if (parsed.ec != std::errc{} || parsed.ptr != token.data() + token.size()) {
        return std::nullopt;
      }
      *target = value;
    } else if (argument == "--max-terminal-escape-reservation-resources") {
      if (result.max_terminal_escape_reservation_resources || index + 1 >= argc) {
        return std::nullopt;
      }
      std::size_t value = 0U;
      const std::string_view token{argv[++index]};
      const auto parsed = std::from_chars(token.data(), token.data() + token.size(), value);
      if (parsed.ec != std::errc{} || parsed.ptr != token.data() + token.size()) {
        return std::nullopt;
      }
      result.max_terminal_escape_reservation_resources = value;
    } else if (argument == "--max-present-congestion-scale-per-mille") {
      if (result.max_present_congestion_scale_per_mille || index + 1 >= argc) {
        return std::nullopt;
      }
      std::uint64_t value = 0U;
      const std::string_view token{argv[++index]};
      const auto parsed = std::from_chars(token.data(), token.data() + token.size(), value);
      if (parsed.ec != std::errc{} || parsed.ptr != token.data() + token.size() || value == 0U) {
        return std::nullopt;
      }
      result.max_present_congestion_scale_per_mille = value;
    } else if (argument == "--capacity-reserve-per-mille") {
      if (result.capacity_reserve_per_mille || index + 1 >= argc) {
        return std::nullopt;
      }
      std::uint32_t value = 0U;
      const std::string_view token{argv[++index]};
      const auto parsed = std::from_chars(token.data(), token.data() + token.size(), value);
      if (parsed.ec != std::errc{} || parsed.ptr != token.data() + token.size() || value > 1'000U) {
        return std::nullopt;
      }
      result.capacity_reserve_per_mille = value;
    } else if (argument == "--expansion-retry-heuristic-weight-per-mille" ||
               argument == "--present-congestion-multiplier-per-mille") {
      auto* target = &result.present_congestion_multiplier_per_mille;
      if (argument == "--expansion-retry-heuristic-weight-per-mille") {
        target = &result.expansion_retry_heuristic_weight_per_mille;
      }
      if (*target || index + 1 >= argc) {
        return std::nullopt;
      }
      std::uint32_t value = 0U;
      const std::string_view token{argv[++index]};
      const auto parsed = std::from_chars(token.data(), token.data() + token.size(), value);
      if (parsed.ec != std::errc{} || parsed.ptr != token.data() + token.size() || value == 0U) {
        return std::nullopt;
      }
      *target = value;
    } else if (argument == "--max-router-iterations" ||
               argument == "--max-final-routability-feedback-passes" ||
               argument == "--max-static-seed-workers" || argument == "--max-expansions" ||
               argument == "--present-congestion-escalation-start-iteration" ||
               argument == "--hard-capacity-start-iteration" || argument == "--pin-escape-radius" ||
               argument == "--via-site-phase-x" || argument == "--via-site-phase-y" ||
               argument == "--max-hard-capacity-ripup-attempts-per-pass" ||
               argument == "--max-hard-capacity-ripup-nets-per-attempt" ||
               argument == "--max-hard-capacity-conflict-set-states-per-attempt" ||
               argument == "--max-hard-capacity-blocker-reuse-invalidations-per-failed-net" ||
               argument == "--max-pin-access-nodes" ||
               argument == "--max-local-pin-escape-expansions" ||
               argument == "--max-corridor-neck-down-states" ||
               argument == "--selective-full-reroute-interval") {
      if (index + 1 >= argc) {
        return std::nullopt;
      }
      std::size_t value = 0;
      const std::string_view token{argv[++index]};
      const auto parsed = std::from_chars(token.data(), token.data() + token.size(), value);
      if (parsed.ec != std::errc{} || parsed.ptr != token.data() + token.size() ||
          (value == 0U && argument != "--pin-escape-radius" && argument != "--via-site-phase-x" &&
           argument != "--via-site-phase-y")) {
        return std::nullopt;
      }
      auto* target = &result.max_corridor_neck_down_states;
      if (argument == "--max-router-iterations") {
        target = &result.max_router_iterations;
      } else if (argument == "--max-final-routability-feedback-passes") {
        target = &result.max_final_routability_feedback_passes;
      } else if (argument == "--max-static-seed-workers") {
        target = &result.max_static_seed_workers;
      } else if (argument == "--max-expansions") {
        target = &result.max_expansions_per_connection;
      } else if (argument == "--present-congestion-escalation-start-iteration") {
        target = &result.present_congestion_escalation_start_iteration;
      } else if (argument == "--hard-capacity-start-iteration") {
        target = &result.hard_capacity_start_iteration;
      } else if (argument == "--max-hard-capacity-ripup-attempts-per-pass") {
        target = &result.max_hard_capacity_ripup_attempts_per_pass;
      } else if (argument == "--max-hard-capacity-ripup-nets-per-attempt") {
        target = &result.max_hard_capacity_ripup_nets_per_attempt;
      } else if (argument == "--max-hard-capacity-conflict-set-states-per-attempt") {
        target = &result.max_hard_capacity_conflict_set_states_per_attempt;
      } else if (argument == "--max-hard-capacity-blocker-reuse-invalidations-per-failed-net") {
        target = &result.max_hard_capacity_blocker_reuse_invalidations_per_failed_net;
      } else if (argument == "--pin-escape-radius") {
        target = &result.pin_escape_grid_radius;
      } else if (argument == "--via-site-phase-x") {
        target = &result.via_site_phase_x;
      } else if (argument == "--via-site-phase-y") {
        target = &result.via_site_phase_y;
      } else if (argument == "--max-pin-access-nodes") {
        target = &result.max_pin_access_nodes_per_terminal;
      } else if (argument == "--max-local-pin-escape-expansions") {
        target = &result.max_local_pin_escape_expansions_per_terminal;
      } else if (argument == "--selective-full-reroute-interval") {
        target = &result.selective_full_reroute_interval;
      }
      if (*target) {
        return std::nullopt;
      }
      *target = value;
    } else if (argument == "--diagonal-grid") {
      if (result.diagonal_grid_edges) {
        return std::nullopt;
      }
      result.diagonal_grid_edges = true;
    } else if (argument == "--crossing-safe-diagonal-grid") {
      if (result.crossing_safe_diagonal_grid_edges) {
        return std::nullopt;
      }
      result.crossing_safe_diagonal_grid_edges = true;
    } else if (argument == "--sampled-clearance-reservations") {
      if (result.sampled_route_clearance_reservations) {
        return std::nullopt;
      }
      result.sampled_route_clearance_reservations = true;
    } else if (argument == "--strict-medial") {
      if (result.strict_medial_topology) {
        return std::nullopt;
      }
      result.strict_medial_topology = true;
    } else if (argument == "--direct-medial") {
      if (result.direct_medial_edges) {
        return std::nullopt;
      }
      result.direct_medial_edges = true;
    } else if (argument == "--negotiate-lane-conflicts") {
      if (result.negotiate_voronoi_lane_conflicts) {
        return std::nullopt;
      }
      result.negotiate_voronoi_lane_conflicts = true;
    } else if (argument == "--diversify-pin-access-rings") {
      if (result.diversify_pin_access_rings) {
        return std::nullopt;
      }
      result.diversify_pin_access_rings = true;
    } else if (argument == "--multisource-steiner") {
      if (result.multisource_steiner_growth) {
        return std::nullopt;
      }
      result.multisource_steiner_growth = true;
    } else if (argument == "--atomic-net-trees") {
      if (result.atomic_complete_net_routing) {
        return std::nullopt;
      }
      result.atomic_complete_net_routing = true;
    } else if (argument == "--rollback-failed-hard-capacity-ripup") {
      if (result.rollback_failed_hard_capacity_ripup) {
        return std::nullopt;
      }
      result.rollback_failed_hard_capacity_ripup = true;
    } else if (argument == "--defer-hard-capacity-ripup-victims-to-pass-tail") {
      if (result.defer_hard_capacity_ripup_victims_to_pass_tail) {
        return std::nullopt;
      }
      result.defer_hard_capacity_ripup_victims_to_pass_tail = true;
    } else if (argument == "--protect-recovered-hard-capacity-ripup-nets") {
      if (result.protect_recovered_hard_capacity_ripup_nets) {
        return std::nullopt;
      }
      result.protect_recovered_hard_capacity_ripup_nets = true;
    } else if (argument == "--selective-congestion-reroute") {
      if (result.selective_congestion_reroute) {
        return std::nullopt;
      }
      result.selective_congestion_reroute = true;
    } else if (argument == "--reuse-weighted-search-frontier") {
      if (result.reuse_weighted_search_frontier) {
        return std::nullopt;
      }
      result.reuse_weighted_search_frontier = true;
    } else if (argument == "--physical-overflow-objective") {
      if (result.physical_overflow_objective) {
        return std::nullopt;
      }
      result.physical_overflow_objective = true;
    } else if (argument == "--physical-overflow-history") {
      if (result.physical_overflow_history) {
        return std::nullopt;
      }
      result.physical_overflow_history = true;
    } else if (argument == "--physical-envelope-ordering") {
      if (result.physical_envelope_ordering) {
        return std::nullopt;
      }
      result.physical_envelope_ordering = true;
    } else if (argument == "--search-diversification") {
      if (result.search_diversification) {
        return std::nullopt;
      }
      result.search_diversification = true;
    } else if (argument == "--arcs") {
      if (result.circular_arc_refinement) {
        return std::nullopt;
      }
      result.circular_arc_refinement = true;
    } else if (argument == "--tune-differential-pair-skew") {
      if (result.differential_pair_skew_tuning) {
        return std::nullopt;
      }
      result.differential_pair_skew_tuning = true;
    } else if (argument == "--batch-assign-differential-pair-skew") {
      if (result.differential_pair_skew_tuning_batch_assignment) {
        return std::nullopt;
      }
      result.differential_pair_skew_tuning_batch_assignment = true;
    } else if (argument == "--swaps" && allow_placement_moves) {
      if (result.enable_swap_moves) {
        return std::nullopt;
      }
      result.enable_swap_moves = true;
    } else if (argument == "--group-moves" && allow_placement_moves) {
      if (result.enable_group_moves) {
        return std::nullopt;
      }
      result.enable_group_moves = true;
    } else if (argument == "--relative-component-moves" && allow_placement_moves) {
      if (result.enable_relative_component_moves) {
        return std::nullopt;
      }
      result.enable_relative_component_moves = true;
    } else if (argument == "--routing-guided-global-placement" && allow_global_routing_guidance) {
      if (result.enable_global_routing_guidance) {
        return std::nullopt;
      }
      result.enable_global_routing_guidance = true;
    } else if (argument == "--assignment-global-placement" && allow_global_routing_guidance) {
      if (result.enable_assignment_legalization) {
        return std::nullopt;
      }
      result.enable_assignment_legalization = true;
    } else if (argument == "--tentative-envelopes" && allow_placement_moves) {
      if (result.tentative_placement_envelopes) {
        return std::nullopt;
      }
      result.tentative_placement_envelopes = true;
    } else if (argument == "--suggested-placement-move-priorities" && allow_placement_moves) {
      if (result.suggested_placement_move_priorities) {
        return std::nullopt;
      }
      result.suggested_placement_move_priorities = true;
    } else if (argument == "--refillable-zones" && allow_refillable_zones) {
      if (result.refillable_zone_fills) {
        return std::nullopt;
      }
      result.refillable_zone_fills = true;
    } else if (argument == "--write-incomplete-preview" && allow_incomplete_preview) {
      if (result.write_incomplete_preview) {
        return std::nullopt;
      }
      result.write_incomplete_preview = true;
    } else if (argument == "--diagnostic-lossy-preview" && allow_incomplete_preview) {
      if (result.diagnostic_lossy_preview) {
        return std::nullopt;
      }
      result.diagnostic_lossy_preview = true;
      result.write_incomplete_preview = true;
    } else if (allow_output && !result.output && !argument.starts_with("--")) {
      result.output = std::filesystem::path{argument};
    } else {
      return std::nullopt;
    }
  }
  if (result.terminal_neck_down_width.has_value() !=
      result.max_terminal_neck_down_length.has_value()) {
    return std::nullopt;
  }
  if (result.medial_neck_down_width.has_value() != result.max_medial_neck_down_length.has_value() ||
      (result.medial_neck_down_width && !result.direct_medial_edges)) {
    return std::nullopt;
  }
  if (result.corridor_neck_down_width.has_value() !=
      result.max_corridor_neck_down_length.has_value()) {
    return std::nullopt;
  }
  if (result.enable_group_moves && !result.placement_group_constraints) {
    return std::nullopt;
  }
  if (result.enable_relative_component_moves && !result.relative_placement_constraints) {
    return std::nullopt;
  }
  if (result.tentative_placement_envelopes && result.placement_envelope_constraints) {
    return std::nullopt;
  }
  if (result.suggested_placement_move_priorities && result.placement_move_priorities) {
    return std::nullopt;
  }
  if (result.differential_pair_skew_tuning_batch_assignment) {
    result.differential_pair_skew_tuning = true;
  }
  if (result.diagonal_grid_edges && result.crossing_safe_diagonal_grid_edges) {
    return std::nullopt;
  }
  if (result.refillable_zone_fills && result.differential_pair_skew_tuning) {
    return std::nullopt;
  }
  if (result.reuse_weighted_search_frontier && !result.expansion_retry_heuristic_weight_per_mille) {
    return std::nullopt;
  }
  if (result.physical_overflow_objective && !result.strict_medial_topology &&
      !result.direct_medial_edges) {
    return std::nullopt;
  }
  if (result.physical_overflow_history && !result.strict_medial_topology &&
      !result.direct_medial_edges) {
    return std::nullopt;
  }
  if (result.physical_envelope_ordering && !result.strict_medial_topology &&
      !result.direct_medial_edges) {
    return std::nullopt;
  }
  if (result.hard_capacity_start_iteration && !result.strict_medial_topology &&
      !result.direct_medial_edges) {
    return std::nullopt;
  }
  if (result.max_hard_capacity_ripup_attempts_per_pass.has_value() !=
          result.max_hard_capacity_ripup_nets_per_attempt.has_value() ||
      (result.max_hard_capacity_ripup_attempts_per_pass &&
       (!result.hard_capacity_start_iteration || !result.atomic_complete_net_routing)) ||
      (result.max_hard_capacity_conflict_set_states_per_attempt &&
       !result.max_hard_capacity_ripup_attempts_per_pass) ||
      (result.rollback_failed_hard_capacity_ripup &&
       !result.max_hard_capacity_ripup_attempts_per_pass) ||
      (result.defer_hard_capacity_ripup_victims_to_pass_tail &&
       !result.max_hard_capacity_ripup_attempts_per_pass) ||
      (result.protect_recovered_hard_capacity_ripup_nets &&
       !result.max_hard_capacity_ripup_attempts_per_pass)) {
    return std::nullopt;
  }
  if (result.selective_congestion_reroute && !result.atomic_complete_net_routing) {
    return std::nullopt;
  }
  if (result.selective_full_reroute_interval && !result.selective_congestion_reroute) {
    return std::nullopt;
  }
  if (result.max_hard_capacity_blocker_reuse_invalidations_per_failed_net &&
      (!result.selective_congestion_reroute || !result.hard_capacity_start_iteration)) {
    return std::nullopt;
  }
  if (!result.telemetry && (result.telemetry_listen || result.telemetry_rate_hz ||
                            result.telemetry_level || result.telemetry_record)) {
    return std::nullopt;
  }
  return result;
}

[[nodiscard]] std::unique_ptr<flux::RouterTelemetryService>
make_router_telemetry_service(const KicadCommandTail& tail) {
  if (!tail.telemetry) {
    return {};
  }
  flux::RouterTelemetryConfig config;
  config.publish_rate_hz = tail.telemetry_rate_hz.value_or(10U);
  config.level = tail.telemetry_level.value_or(flux::RouterTelemetryLevel::normal);
  config.record_path = tail.telemetry_record;
  const auto endpoint = tail.telemetry_listen.value_or("127.0.0.1:8765");
  std::string address;
  std::string_view port_token;
  if (!endpoint.empty() && endpoint.front() == '[') {
    const auto close = endpoint.find(']');
    if (close == std::string::npos || close + 2U > endpoint.size() || endpoint[close + 1U] != ':') {
      return {};
    }
    address = endpoint.substr(1U, close - 1U);
    port_token = std::string_view{endpoint}.substr(close + 2U);
  } else {
    const auto separator = endpoint.rfind(':');
    if (separator == std::string::npos) {
      return {};
    }
    address = endpoint.substr(0U, separator);
    port_token = std::string_view{endpoint}.substr(separator + 1U);
  }
  std::uint32_t port = 0U;
  const auto parsed =
      std::from_chars(port_token.data(), port_token.data() + port_token.size(), port);
  if (address.empty() || parsed.ec != std::errc{} ||
      parsed.ptr != port_token.data() + port_token.size() || port == 0U ||
      port > std::numeric_limits<std::uint16_t>::max()) {
    return {};
  }
  config.listen_address = std::move(address);
  config.listen_port = static_cast<std::uint16_t>(port);
  return std::make_unique<flux::RouterTelemetryService>(std::move(config));
}

[[nodiscard]] flux::GlobalRouteResult route_with_optional_sampled_conflict_repair(
    const flux::Board& board, const flux::GlobalRouterConfig& config, bool repair_sampled_conflicts,
    flux::RouterTelemetrySink* telemetry = nullptr) {
  auto result = flux::route_global(board, config, {}, telemetry);
  if (!repair_sampled_conflicts || result.complete() || !result.unrouted_nets.empty() ||
      (result.sampled_overflow == 0U && result.total_voronoi_overflow_nm == 0U &&
       result.clearance_shadow_overflow == 0U) ||
      result.corridor_overflow != 0U) {
    return result;
  }

  struct RepairCone {
    std::vector<flux::NetId> nets;
    std::size_t pressure{0};
    std::tuple<std::string, std::int64_t, std::int64_t> tile;
  };
  struct TileMembers {
    std::set<flux::NetId> nets;
    std::size_t pressure{0};
  };
  const auto pitch = config.grid_pitch.value();
  const auto tile_span = pitch > std::numeric_limits<std::int64_t>::max() / 4 ? pitch : pitch * 4;
  const auto tile_coordinate = [tile_span](std::int64_t coordinate) {
    auto quotient = coordinate / tile_span;
    if (coordinate % tile_span < 0) {
      --quotient;
    }
    return quotient;
  };
  std::map<std::tuple<std::string, std::int64_t, std::int64_t>, TileMembers> tiles;
  const auto add_tile = [&](flux::Point first, flux::Point second, const flux::LayerId& layer,
                            const auto& nets, std::size_t pressure) {
    const auto midpoint_x = std::midpoint(first.x.value(), second.x.value());
    const auto midpoint_y = std::midpoint(first.y.value(), second.y.value());
    auto& tile = tiles[{layer.value(), tile_coordinate(midpoint_x), tile_coordinate(midpoint_y)}];
    tile.nets.insert(nets.begin(), nets.end());
    tile.pressure += pressure;
  };
  for (const auto& conflict : result.clearance_shadow_conflicts) {
    const std::array nets{conflict.first_net, conflict.second_net};
    add_tile(conflict.first.position, conflict.second.position, conflict.first.layer, nets, 1U);
  }
  for (const auto& resource : result.resource_usage) {
    if (resource.congested()) {
      add_tile(resource.first.position, resource.second.position, resource.first.layer,
               resource.nets, resource.demand - resource.capacity);
    }
  }
  for (const auto& resource : result.voronoi_resource_usage) {
    if (resource.congested()) {
      add_tile(resource.start, resource.end, resource.layer, resource.nets, 1U);
    }
  }
  std::vector<RepairCone> repair_cones;
  repair_cones.reserve(tiles.size());
  for (auto& [key, members] : tiles) {
    if (members.nets.size() < 2U) {
      continue;
    }
    repair_cones.push_back(RepairCone{
        std::vector<flux::NetId>(members.nets.begin(), members.nets.end()), members.pressure, key});
  }
  std::stable_sort(repair_cones.begin(), repair_cones.end(), [](const auto& lhs, const auto& rhs) {
    return lhs.pressure != rhs.pressure
               ? lhs.pressure > rhs.pressure
               : std::tie(lhs.nets, lhs.tile) < std::tie(rhs.nets, rhs.tile);
  });
  if (repair_cones.empty()) {
    return result;
  }

  auto repair_config = config;
  repair_config.atomic_complete_net_routing = true;
  repair_config.diversify_pin_access_rings = true;
  repair_config.hard_capacity_start_iteration = 1U;
  repair_config.max_iterations = std::min(config.max_iterations, std::size_t{12U});
  repair_config.selective_congestion_reroute = false;
  repair_config.selective_full_reroute_interval = 0U;
  constexpr std::size_t max_repair_cones = 2U;
  std::set<std::vector<flux::NetId>> attempted;
  std::size_t attempts = 0U;
  std::size_t accepted = 0U;
  std::size_t nets_attempted = 0U;
  for (const auto& cone : repair_cones) {
    if (attempts >= max_repair_cones || cone.nets.empty() || !attempted.insert(cone.nets).second) {
      continue;
    }
    ++attempts;
    nets_attempted += cone.nets.size();
    auto repair_checkpoint = result;
    // The repair makes search stricter only after removing every participant in this attributed
    // spatial cone. Preserved paths remain reserved and the original checkpoint survives every
    // non-improving or incomplete hard-capacity transaction.
    repair_checkpoint.configured_diversify_pin_access_rings = true;
    repair_checkpoint.configured_hard_capacity_start_iteration = 1U;
    auto repaired = flux::route_global_incremental(board, repair_checkpoint, cone.nets,
                                                   repair_config, {}, telemetry);
    if (flux::global_route_checkpoint_better(repaired, result, config.checkpoint_objective)) {
      result = std::move(repaired);
      ++accepted;
    }
  }
  result.post_negotiation_cone_repair_attempts = attempts;
  result.post_negotiation_cone_repairs_accepted = accepted;
  result.post_negotiation_cone_repair_nets_attempted = nets_attempted;
  return result;
}

[[nodiscard]] bool apply_router_tail(const KicadCommandTail& tail, const flux::Board& board,
                                     flux::GlobalRouterConfig& config) {
  std::set<std::string> selected_names;
  for (const auto& name : tail.route_net_names) {
    const auto found = std::find_if(board.nets.begin(), board.nets.end(),
                                    [&name](const flux::Net& net) { return net.name == name; });
    if (!selected_names.insert(name).second || found == board.nets.end()) {
      std::cerr << "--route-net must name each existing source net at most once: " << name << '\n';
      return false;
    }
    config.selected_nets.push_back(found->id);
  }
  std::sort(config.selected_nets.begin(), config.selected_nets.end());
  std::set<std::string> priority_names;
  for (const auto& name : tail.priority_route_net_names) {
    const auto found = std::find_if(board.nets.begin(), board.nets.end(),
                                    [&name](const flux::Net& net) { return net.name == name; });
    if (!priority_names.insert(name).second || found == board.nets.end()) {
      std::cerr << "--route-first must name each existing source net at most once: " << name
                << '\n';
      return false;
    }
    if (!config.selected_nets.empty() &&
        std::find(config.selected_nets.begin(), config.selected_nets.end(), found->id) ==
            config.selected_nets.end()) {
      std::cerr << "--route-first net is outside the --route-net scope: " << name << '\n';
      return false;
    }
    config.priority_nets.push_back(found->id);
  }
  config.use_direct_voronoi_resource_edges = tail.direct_medial_edges;
  config.allow_diagonal_grid_edges = tail.diagonal_grid_edges;
  config.allow_crossing_safe_diagonal_grid_edges = tail.crossing_safe_diagonal_grid_edges;
  if (tail.sampled_route_clearance_reservations) {
    config.reserve_sampled_route_clearance = true;
  }
  config.negotiate_voronoi_lane_conflicts = tail.negotiate_voronoi_lane_conflicts;
  config.diversify_pin_access_rings = tail.diversify_pin_access_rings;
  config.use_multisource_steiner_growth = tail.multisource_steiner_growth;
  config.atomic_complete_net_routing = tail.atomic_complete_net_routing;
  config.rollback_failed_hard_capacity_ripup = tail.rollback_failed_hard_capacity_ripup;
  config.defer_hard_capacity_ripup_victims_to_pass_tail =
      tail.defer_hard_capacity_ripup_victims_to_pass_tail;
  config.protect_recovered_hard_capacity_ripup_nets =
      tail.protect_recovered_hard_capacity_ripup_nets;
  config.selective_congestion_reroute = tail.selective_congestion_reroute;
  if (tail.selective_full_reroute_interval) {
    config.selective_full_reroute_interval = *tail.selective_full_reroute_interval;
  }
  config.reuse_weighted_search_frontier = tail.reuse_weighted_search_frontier;
  config.checkpoint_objective =
      tail.physical_overflow_objective
          ? flux::GlobalRouteCheckpointObjective::dimensioned_physical_overflow
          : flux::GlobalRouteCheckpointObjective::aggregate_overflow;
  config.scale_voronoi_history_by_overflow = tail.physical_overflow_history;
  config.order_routing_groups_by_physical_envelope = tail.physical_envelope_ordering;
  config.enable_search_diversification = tail.search_diversification;
  if (tail.search_diversification) {
    // Large-neighborhood reuse is whole-net transactional. Make the safe prerequisite implicit
    // for the single high-level CLI switch while retaining independent C++ configuration.
    config.atomic_complete_net_routing = true;
  }
  if (tail.grid_pitch) {
    config.grid_pitch = *tail.grid_pitch;
  }
  if (tail.via_site_pitch) {
    config.via_site_pitch = *tail.via_site_pitch;
  }
  if (tail.via_site_phase_x) {
    config.via_site_phase_x = *tail.via_site_phase_x;
  }
  if (tail.via_site_phase_y) {
    config.via_site_phase_y = *tail.via_site_phase_y;
  }
  if (tail.max_router_iterations) {
    config.max_iterations = *tail.max_router_iterations;
  }
  if (tail.max_static_seed_workers) {
    config.max_static_seed_workers = *tail.max_static_seed_workers;
  }
  if (tail.max_expansions_per_connection) {
    config.max_expansions_per_connection = *tail.max_expansions_per_connection;
  }
  if (tail.expansion_retry_heuristic_weight_per_mille) {
    config.expansion_retry_heuristic_weight_per_mille =
        *tail.expansion_retry_heuristic_weight_per_mille;
  }
  if (tail.present_congestion_multiplier_per_mille) {
    config.present_congestion_multiplier_per_mille = *tail.present_congestion_multiplier_per_mille;
  }
  if (tail.max_present_congestion_scale_per_mille) {
    config.max_present_congestion_scale_per_mille = *tail.max_present_congestion_scale_per_mille;
  }
  if (tail.present_congestion_escalation_start_iteration) {
    config.present_congestion_escalation_start_iteration =
        *tail.present_congestion_escalation_start_iteration;
  }
  if (tail.hard_capacity_start_iteration) {
    config.hard_capacity_start_iteration = *tail.hard_capacity_start_iteration;
  }
  if (tail.max_hard_capacity_ripup_attempts_per_pass) {
    config.max_hard_capacity_ripup_attempts_per_pass =
        *tail.max_hard_capacity_ripup_attempts_per_pass;
  }
  if (tail.max_hard_capacity_ripup_nets_per_attempt) {
    config.max_hard_capacity_ripup_nets_per_attempt =
        *tail.max_hard_capacity_ripup_nets_per_attempt;
  }
  if (tail.max_hard_capacity_conflict_set_states_per_attempt) {
    config.max_hard_capacity_conflict_set_states_per_attempt =
        *tail.max_hard_capacity_conflict_set_states_per_attempt;
  }
  if (tail.max_hard_capacity_blocker_reuse_invalidations_per_failed_net) {
    config.max_hard_capacity_blocker_reuse_invalidations_per_failed_net =
        *tail.max_hard_capacity_blocker_reuse_invalidations_per_failed_net;
  }
  if (tail.pin_escape_grid_radius) {
    config.max_pin_escape_grid_radius = *tail.pin_escape_grid_radius;
  }
  if (tail.max_pin_access_nodes_per_terminal) {
    config.max_pin_access_nodes_per_terminal = *tail.max_pin_access_nodes_per_terminal;
  }
  if (tail.local_pin_escape_pitch) {
    config.local_pin_escape_pitch = *tail.local_pin_escape_pitch;
  }
  if (tail.clearance_shadow_cost_quantum) {
    config.clearance_shadow_cost_quantum = *tail.clearance_shadow_cost_quantum;
  }
  if (tail.capacity_reserve_per_mille) {
    config.capacity_reserve_per_mille = *tail.capacity_reserve_per_mille;
  }
  if (tail.max_local_pin_escape_expansions_per_terminal) {
    config.max_local_pin_escape_expansions_per_terminal =
        *tail.max_local_pin_escape_expansions_per_terminal;
  }
  if (tail.terminal_neck_down_width) {
    config.terminal_neck_down_width = *tail.terminal_neck_down_width;
  }
  if (tail.max_terminal_neck_down_length) {
    config.max_terminal_neck_down_length = *tail.max_terminal_neck_down_length;
  }
  if (tail.medial_neck_down_width) {
    config.medial_neck_down_width = *tail.medial_neck_down_width;
  }
  if (tail.max_medial_neck_down_length) {
    config.max_medial_neck_down_length = *tail.max_medial_neck_down_length;
  }
  if (tail.corridor_neck_down_width) {
    config.corridor_neck_down_width = *tail.corridor_neck_down_width;
  }
  if (tail.max_corridor_neck_down_length) {
    config.max_corridor_neck_down_length = *tail.max_corridor_neck_down_length;
  }
  if (tail.max_voronoi_resource_segment_length) {
    config.max_voronoi_resource_segment_length = *tail.max_voronoi_resource_segment_length;
  }
  if (tail.max_corridor_neck_down_states) {
    config.max_corridor_neck_down_states = *tail.max_corridor_neck_down_states;
  }
  if (tail.width_reduction_cost_per_grid_step) {
    config.width_reduction_cost_per_grid_step = *tail.width_reduction_cost_per_grid_step;
  }
  if (tail.river_following_cost) {
    config.river_following_cost = *tail.river_following_cost;
  }
  if (tail.former_route_memory_cost) {
    config.former_route_memory_cost = *tail.former_route_memory_cost;
  }
  if (tail.terminal_escape_reservation_cost) {
    config.terminal_escape_reservation_cost = *tail.terminal_escape_reservation_cost;
  }
  if (tail.terminal_escape_proximity_cost) {
    config.terminal_escape_proximity_cost = *tail.terminal_escape_proximity_cost;
  }
  if (tail.capacity_reserve_cost) {
    config.capacity_reserve_cost = *tail.capacity_reserve_cost;
  }
  if (tail.max_terminal_escape_reservation_resources) {
    config.max_terminal_escape_reservation_resources =
        *tail.max_terminal_escape_reservation_resources;
  }
  return true;
}

void apply_regional_tail(const KicadCommandTail& tail, flux::RegionalRouterConfig& config) {
  config.treat_saved_zone_fills_as_refillable = tail.refillable_zone_fills;
  config.enable_xpbd_relaxation = tail.xpbd_relaxation;
  config.enable_xpbd_feedback = tail.xpbd_feedback;
  config.xpbd.prefer_gpu = !tail.xpbd_cpu;
  if (tail.xpbd_iterations)
    config.xpbd.iterations = *tail.xpbd_iterations;
  if (tail.xpbd_maximum_step)
    config.xpbd.maximum_step_nm = tail.xpbd_maximum_step->value();
}

[[nodiscard]] bool autoroute_tail_is_supported(const KicadCommandTail& tail) {
  return tail.output.has_value() && !tail.placement_envelope_constraints &&
         tail.route_net_names.empty() && tail.priority_route_net_names.empty() &&
         !tail.repair_sampled_conflicts && !tail.placement_group_constraints &&
         !tail.placement_region_constraints && !tail.relative_placement_constraints &&
         !tail.placement_move_priorities && !tail.max_placement_passes &&
         !tail.max_candidate_evaluations && !tail.max_candidate_evaluation_workers &&
         !tail.max_full_routing_candidates_per_wave && !tail.candidate_evaluation_offset &&
         !tail.move_step && !tail.grid_pitch && !tail.via_site_pitch && !tail.via_site_phase_x &&
         !tail.via_site_phase_y && !tail.max_router_iterations && !tail.max_static_seed_workers &&
         !tail.max_expansions_per_connection && !tail.expansion_retry_heuristic_weight_per_mille &&
         !tail.present_congestion_multiplier_per_mille &&
         !tail.max_present_congestion_scale_per_mille &&
         !tail.present_congestion_escalation_start_iteration && !tail.pin_escape_grid_radius &&
         !tail.clearance_shadow_cost_quantum && !tail.capacity_reserve_per_mille &&
         !tail.hard_capacity_start_iteration && !tail.max_hard_capacity_ripup_attempts_per_pass &&
         !tail.max_hard_capacity_ripup_nets_per_attempt &&
         !tail.max_hard_capacity_conflict_set_states_per_attempt &&
         !tail.max_hard_capacity_blocker_reuse_invalidations_per_failed_net &&
         !tail.max_pin_access_nodes_per_terminal && !tail.local_pin_escape_pitch &&
         !tail.max_local_pin_escape_expansions_per_terminal && !tail.terminal_neck_down_width &&
         !tail.max_terminal_neck_down_length && !tail.medial_neck_down_width &&
         !tail.max_medial_neck_down_length && !tail.corridor_neck_down_width &&
         !tail.max_corridor_neck_down_length && !tail.max_voronoi_resource_segment_length &&
         !tail.max_corridor_neck_down_states && !tail.selective_full_reroute_interval &&
         !tail.diagonal_grid_edges && !tail.crossing_safe_diagonal_grid_edges &&
         !tail.sampled_route_clearance_reservations && !tail.width_reduction_cost_per_grid_step &&
         !tail.river_following_cost && !tail.former_route_memory_cost &&
         !tail.terminal_escape_reservation_cost && !tail.terminal_escape_proximity_cost &&
         !tail.capacity_reserve_cost && !tail.max_terminal_escape_reservation_resources &&
         !tail.strict_medial_topology && !tail.direct_medial_edges &&
         !tail.negotiate_voronoi_lane_conflicts && !tail.diversify_pin_access_rings &&
         !tail.search_diversification && !tail.multisource_steiner_growth &&
         !tail.atomic_complete_net_routing && !tail.rollback_failed_hard_capacity_ripup &&
         !tail.selective_congestion_reroute &&
         !tail.defer_hard_capacity_ripup_victims_to_pass_tail &&
         !tail.protect_recovered_hard_capacity_ripup_nets && !tail.reuse_weighted_search_frontier &&
         !tail.physical_overflow_objective && !tail.physical_overflow_history &&
         !tail.physical_envelope_ordering && !tail.circular_arc_refinement &&
         !tail.differential_pair_skew_tuning &&
         !tail.differential_pair_skew_tuning_batch_assignment && !tail.enable_swap_moves &&
         !tail.enable_group_moves && !tail.enable_relative_component_moves &&
         !tail.enable_global_routing_guidance && !tail.enable_assignment_legalization &&
         !tail.tentative_placement_envelopes && !tail.suggested_placement_move_priorities &&
         !tail.refillable_zone_fills;
}

void apply_integrated_tail(const KicadCommandTail& tail, flux::IntegratedOptimizerConfig& config) {
  if (tail.max_placement_passes) {
    config.max_passes = *tail.max_placement_passes;
  }
  if (tail.max_candidate_evaluations) {
    config.max_candidate_evaluations = *tail.max_candidate_evaluations;
  }
  if (tail.max_candidate_evaluation_workers) {
    config.max_candidate_evaluation_workers = *tail.max_candidate_evaluation_workers;
  }
  if (tail.max_full_routing_candidates_per_wave) {
    config.max_full_routing_candidates_per_wave = *tail.max_full_routing_candidates_per_wave;
  }
  if (tail.candidate_evaluation_offset) {
    config.candidate_evaluation_offset = *tail.candidate_evaluation_offset;
  }
  if (tail.move_step) {
    config.move_step = *tail.move_step;
  }
}

[[nodiscard]] flux::GlobalRouterConfig minimal_autoroute_config() {
  flux::GlobalRouterConfig config;
  // The production workflow uses the crossing-capacity-gated 45-degree topology. The core
  // router remains opt-in so diagnostic callers can retain the orthogonal comparison baseline.
  config.allow_crossing_safe_diagonal_grid_edges = true;
  config.max_iterations = 6U;
  config.max_expansions_per_connection = 50'000U;
  config.expansion_retry_heuristic_weight_per_mille = 10'000U;
  config.reuse_weighted_search_frontier = true;
  config.checkpoint_objective = flux::GlobalRouteCheckpointObjective::dimensioned_physical_overflow;
  config.scale_voronoi_history_by_overflow = true;
  config.present_congestion_multiplier_per_mille = 1'050U;
  config.max_present_congestion_scale_per_mille = 10'000U;
  config.present_congestion_escalation_start_iteration = 4U;
  config.use_voronoi_allocation_resources = true;
  config.use_direct_voronoi_resource_edges = true;
  config.require_voronoi_topological_transitions = true;
  config.negotiate_voronoi_lane_conflicts = true;
  config.atomic_complete_net_routing = true;
  config.selective_congestion_reroute = true;
  config.order_routing_groups_by_physical_envelope = true;
  return config;
}

[[nodiscard]] std::optional<Board>
apply_optional_routing_policy(const Board& source,
                              const std::optional<std::filesystem::path>& snapshot_path,
                              const std::optional<std::filesystem::path>& differential_pair_path,
                              const std::optional<std::filesystem::path>& routing_bus_path,
                              const std::optional<std::filesystem::path>& envelope_path,
                              const std::optional<std::filesystem::path>& placement_group_path,
                              const std::optional<std::filesystem::path>& placement_region_path,
                              const std::optional<std::filesystem::path>& relative_placement_path) {
  std::optional<Board> candidate = source;
  if (!snapshot_path) {
    candidate = source;
  } else {
    auto snapshot = flux::io::apply_kicad_netclass_snapshot_file(source, *snapshot_path);
    if (!snapshot.ok()) {
      print_netclass_snapshot_result(snapshot);
      return std::nullopt;
    }
    candidate = std::move(snapshot.board);
  }
  if (differential_pair_path) {
    auto constrained =
        flux::io::apply_differential_pair_constraints_file(*candidate, *differential_pair_path);
    if (!constrained.ok()) {
      const auto errors = static_cast<std::size_t>(
          std::count_if(constrained.diagnostics.begin(), constrained.diagnostics.end(),
                        [](const auto& diagnostic) {
                          return diagnostic.severity == flux::io::ConstraintSeverity::error;
                        }));
      std::cout << "{\n  \"ok\": false,\n  \"differential_pair_constraint_errors\": " << errors
                << ",\n  \"issue_codes\": [";
      for (std::size_t index = 0; index < constrained.diagnostics.size(); ++index) {
        std::cout << (index == 0U ? "" : ", ") << json_quote(constrained.diagnostics[index].code);
      }
      std::cout << "]\n}\n";
      return std::nullopt;
    }
    candidate = std::move(constrained.board);
  }
  if (envelope_path) {
    auto constrained =
        flux::io::apply_placement_envelope_constraints_file(*candidate, *envelope_path);
    if (!constrained.ok()) {
      const auto errors = static_cast<std::size_t>(
          std::count_if(constrained.diagnostics.begin(), constrained.diagnostics.end(),
                        [](const auto& diagnostic) {
                          return diagnostic.severity == flux::io::ConstraintSeverity::error;
                        }));
      std::cout << "{\n  \"ok\": false,\n  \"placement_envelope_errors\": " << errors
                << ",\n  \"issue_codes\": [";
      for (std::size_t index = 0; index < constrained.diagnostics.size(); ++index) {
        std::cout << (index == 0U ? "" : ", ") << json_quote(constrained.diagnostics[index].code);
      }
      std::cout << "]\n}\n";
      return std::nullopt;
    }
    candidate = std::move(constrained.board);
  }
  if (routing_bus_path) {
    auto constrained = flux::io::apply_routing_bus_constraints_file(*candidate, *routing_bus_path);
    if (!constrained.ok()) {
      const auto errors = static_cast<std::size_t>(
          std::count_if(constrained.diagnostics.begin(), constrained.diagnostics.end(),
                        [](const auto& diagnostic) {
                          return diagnostic.severity == flux::io::ConstraintSeverity::error;
                        }));
      std::cout << "{\n  \"ok\": false,\n  \"routing_bus_constraint_errors\": " << errors
                << ",\n  \"issue_codes\": [";
      for (std::size_t index = 0; index < constrained.diagnostics.size(); ++index) {
        std::cout << (index == 0U ? "" : ", ") << json_quote(constrained.diagnostics[index].code);
      }
      std::cout << "]\n}\n";
      return std::nullopt;
    }
    candidate = std::move(constrained.board);
  }
  if (placement_group_path) {
    auto constrained =
        flux::io::apply_placement_group_constraints_file(*candidate, *placement_group_path);
    if (!constrained.ok()) {
      const auto errors = static_cast<std::size_t>(
          std::count_if(constrained.diagnostics.begin(), constrained.diagnostics.end(),
                        [](const auto& diagnostic) {
                          return diagnostic.severity == flux::io::ConstraintSeverity::error;
                        }));
      std::cout << "{\n  \"ok\": false,\n  \"placement_group_constraint_errors\": " << errors
                << ",\n  \"issue_codes\": [";
      for (std::size_t index = 0; index < constrained.diagnostics.size(); ++index) {
        std::cout << (index == 0U ? "" : ", ") << json_quote(constrained.diagnostics[index].code);
      }
      std::cout << "]\n}\n";
      return std::nullopt;
    }
    candidate = std::move(constrained.board);
  }
  if (placement_region_path) {
    auto constrained =
        flux::io::apply_placement_region_constraints_file(*candidate, *placement_region_path);
    if (!constrained.ok()) {
      const auto errors = static_cast<std::size_t>(
          std::count_if(constrained.diagnostics.begin(), constrained.diagnostics.end(),
                        [](const auto& diagnostic) {
                          return diagnostic.severity == flux::io::ConstraintSeverity::error;
                        }));
      std::cout << "{\n  \"ok\": false,\n  \"placement_region_constraint_errors\": " << errors
                << ",\n  \"issue_codes\": [";
      for (std::size_t index = 0; index < constrained.diagnostics.size(); ++index) {
        std::cout << (index == 0U ? "" : ", ") << json_quote(constrained.diagnostics[index].code);
      }
      std::cout << "]\n}\n";
      return std::nullopt;
    }
    candidate = std::move(constrained.board);
  }
  if (relative_placement_path) {
    auto constrained =
        flux::io::apply_relative_placement_constraints_file(*candidate, *relative_placement_path);
    if (!constrained.ok()) {
      const auto errors = static_cast<std::size_t>(
          std::count_if(constrained.diagnostics.begin(), constrained.diagnostics.end(),
                        [](const auto& diagnostic) {
                          return diagnostic.severity == flux::io::ConstraintSeverity::error;
                        }));
      std::cout << "{\n  \"ok\": false,\n  \"relative_placement_constraint_errors\": " << errors
                << ",\n  \"issue_codes\": [";
      for (std::size_t index = 0; index < constrained.diagnostics.size(); ++index) {
        std::cout << (index == 0U ? "" : ", ") << json_quote(constrained.diagnostics[index].code);
      }
      std::cout << "]\n}\n";
      return std::nullopt;
    }
    candidate = std::move(constrained.board);
  }
  return candidate;
}

void print_constraint_result(const flux::io::OrientationConstraintResult& constrained) {
  const auto errors = static_cast<std::size_t>(std::count_if(
      constrained.diagnostics.begin(), constrained.diagnostics.end(), [](const auto& diagnostic) {
        return diagnostic.severity == flux::io::ConstraintSeverity::error;
      }));
  std::cout << "{\n"
            << "  \"ok\": " << (constrained.ok() ? "true" : "false") << ",\n"
            << "  \"constraint_errors\": " << errors << ",\n"
            << "  \"constrained_footprints\": " << constrained.constrained_footprints.size()
            << ",\n  \"constraint_diagnostics\": [";
  for (std::size_t index = 0; index < constrained.diagnostics.size(); ++index) {
    const auto& diagnostic = constrained.diagnostics[index];
    std::cout << (index == 0U ? "" : ",") << "\n    {\"severity\": "
              << json_quote(diagnostic.severity == flux::io::ConstraintSeverity::error ? "error"
                                                                                       : "warning")
              << ", \"code\": " << json_quote(diagnostic.code)
              << ", \"message\": " << json_quote(diagnostic.message)
              << ", \"line\": " << diagnostic.line << ", \"column\": " << diagnostic.column
              << ", \"object_ids\": [";
    for (std::size_t object_index = 0; object_index < diagnostic.object_ids.size();
         ++object_index) {
      std::cout << (object_index == 0U ? "" : ", ")
                << json_quote(diagnostic.object_ids[object_index]);
    }
    std::cout << "]}";
  }
  if (!constrained.diagnostics.empty()) {
    std::cout << '\n';
  }
  std::cout << "  ]\n}\n";
}

[[nodiscard]] flux::IntegratedOptimizerConfig
constrained_optimizer_config(const Board& board,
                             const std::vector<ObjectId>& constrained_footprints, bool refine) {
  flux::IntegratedOptimizerConfig config;
  config.orientation_scope = constrained_footprints;
  config.optimize_routed_quality = refine;
  if (refine) {
    config.max_passes = 32;
    config.max_candidate_evaluations = 2'000;
  }
  std::set<Microdegrees> offsets;
  for (const auto& id : constrained_footprints) {
    const auto footprint =
        std::find_if(board.footprints.begin(), board.footprints.end(),
                     [&id](const Footprint& candidate) { return candidate.id == id; });
    if (footprint == board.footprints.end()) {
      continue;
    }
    for (const auto target : footprint->orientation_constraints.allowed_rotations) {
      const auto offset = Microdegrees{target.value() - footprint->rotation.value()};
      if (offset != Microdegrees{} && offsets.insert(offset).second) {
        config.rotation_offsets.push_back(offset);
      }
    }
    const bool opposite_allowed = footprint->side == flux::BoardSide::front
                                      ? footprint->orientation_constraints.allow_back
                                      : footprint->orientation_constraints.allow_front;
    config.enable_side_flips = config.enable_side_flips || opposite_allowed;
  }
  return config;
}

[[nodiscard]] bool apply_placement_move_priorities(const Board& source,
                                                   const std::optional<std::filesystem::path>& path,
                                                   bool use_suggestions,
                                                   flux::IntegratedOptimizerConfig& config) {
  if (use_suggestions) {
    config.placement_move_priorities = flux::io::make_suggested_placement_move_priorities(source);
    config.suggested_placement_move_priorities = true;
    return true;
  }
  if (!path) {
    return true;
  }
  const auto loaded = flux::io::load_placement_move_priorities_file(source, *path);
  if (!loaded.ok()) {
    const auto errors = static_cast<std::size_t>(std::count_if(
        loaded.diagnostics.begin(), loaded.diagnostics.end(), [](const auto& diagnostic) {
          return diagnostic.severity == flux::io::ConstraintSeverity::error;
        }));
    std::cout << "{\n  \"ok\": false,\n  \"placement_move_priority_errors\": " << errors
              << ",\n  \"issue_codes\": [";
    for (std::size_t index = 0U; index < loaded.diagnostics.size(); ++index) {
      std::cout << (index == 0U ? "" : ", ") << json_quote(loaded.diagnostics[index].code);
    }
    std::cout << "]\n}\n";
    return false;
  }
  config.placement_move_priorities = *loaded.policy;
  return true;
}

[[nodiscard]] int finish_kicad_optimization(const std::filesystem::path& source_path,
                                            std::optional<std::filesystem::path> output_path,
                                            const Board& source,
                                            const flux::IntegratedOptimizationResult& optimized) {
  if (!output_path || !optimized.complete() || !optimized.candidate) {
    print_integrated_result(optimized);
    return optimized.complete() ? EXIT_SUCCESS : 6;
  }
  const auto exported = flux::io::export_kicad_candidate_file(source_path, *output_path, source,
                                                              *optimized.candidate);
  const auto error_count = static_cast<std::size_t>(std::count_if(
      exported.diagnostics.begin(), exported.diagnostics.end(), [](const auto& diagnostic) {
        return diagnostic.severity == flux::io::ExportSeverity::error;
      }));
  std::vector<std::string> issue_codes;
  issue_codes.reserve(exported.diagnostics.size());
  for (const auto& diagnostic : exported.diagnostics) {
    issue_codes.push_back(diagnostic.code);
  }
  print_integrated_result(optimized,
                          ExportSummary{exported.ok(), error_count,
                                        exported.generated_id_map.size(), std::move(issue_codes)});
  return exported.ok() ? EXIT_SUCCESS : 7;
}

[[nodiscard]] int finish_kicad_detail(const std::filesystem::path& source_path,
                                      const std::optional<std::filesystem::path>& output_path,
                                      const Board& source,
                                      const flux::DetailedRouteResult& detailed,
                                      const flux::GlobalRouteResult& global,
                                      std::optional<std::string_view> workflow = std::nullopt,
                                      bool allow_provisional_zone_refill = false,
                                      bool write_incomplete_preview = false,
                                      bool diagnostic_lossy_preview = false) {
  const bool exportable = detailed.complete() || (allow_provisional_zone_refill &&
                                                  detailed.provisional_zone_refill_candidate());
  const bool preview_exportable = write_incomplete_preview && detailed.candidate.has_value();
  if (!output_path || (!exportable && !preview_exportable) || !detailed.candidate) {
    print_detailed_result(detailed, std::nullopt, workflow, &global);
    return exportable ? EXIT_SUCCESS : 5;
  }
  flux::io::KicadExportOptions export_options;
  export_options.allow_lossy_diagnostic_preview = diagnostic_lossy_preview;
  export_options.verify_round_trip = !diagnostic_lossy_preview;
  const auto exported = flux::io::export_kicad_candidate_file(
      source_path, *output_path, source, *detailed.candidate, export_options);
  const auto error_count = static_cast<std::size_t>(std::count_if(
      exported.diagnostics.begin(), exported.diagnostics.end(), [](const auto& diagnostic) {
        return diagnostic.severity == flux::io::ExportSeverity::error;
      }));
  std::vector<std::string> issue_codes;
  issue_codes.reserve(exported.diagnostics.size());
  for (const auto& diagnostic : exported.diagnostics) {
    issue_codes.push_back(diagnostic.code);
  }
  print_detailed_result(detailed,
                        ExportSummary{exported.ok(), error_count, exported.generated_id_map.size(),
                                      std::move(issue_codes), !exportable},
                        workflow, &global);
  if (!exported.ok()) {
    return 7;
  }
  return exportable ? EXIT_SUCCESS : 5;
}

[[nodiscard]] bool run_free_space_demo() {
  const auto board = make_demo_board(false, false);
  const auto rules = flux::resolve_net_routing_rules(board, NetId{"N1"});
  const auto extracted = flux::extract_polygonal_board_free_space(
      board, flux::BoardFreeSpaceExtractionConfig{LayerId{"F.Cu"}, NetId{"N1"}, true});
  flux::FreeSpaceOffsetResult offset;
  if (extracted.complete()) {
    offset = flux::offset_polygonal_free_space(
        extracted.domain, flux::FreeSpaceOffsetConfig{
                              Nanometres{rules.clearance.value() + rules.track_width.value() / 2}});
  }
  std::size_t skeleton_nodes = 0;
  std::size_t skeleton_edges = 0;
  std::size_t skeleton_junctions = 0;
  std::size_t curved_edges_seen = 0;
  std::size_t accepted_curved_edge_chords = 0;
  std::size_t withheld_curved_edges = 0;
  std::size_t peak_lane_capacity = 0;
  bool skeletons_ok = offset.ok();
  if (offset.ok()) {
    for (const auto& region : offset.regions) {
      flux::FreeSpaceDomain domain;
      domain.outline = region.area;
      domain.cutouts = region.holes;
      const auto skeleton = flux::build_free_space_skeleton(
          domain, flux::FreeSpaceSkeletonConfig{Nanometres{}, 200'000U, true});
      skeletons_ok = skeletons_ok && skeleton.ok();
      skeleton_nodes += skeleton.nodes.size();
      skeleton_edges += skeleton.edges.size();
      skeleton_junctions += skeleton.junctions;
      curved_edges_seen += skeleton.curved_edges_seen;
      accepted_curved_edge_chords += skeleton.accepted_curved_edge_chords;
      withheld_curved_edges += skeleton.skipped_curved_edges;
      for (const auto& edge : skeleton.edges) {
        peak_lane_capacity =
            std::max(peak_lane_capacity,
                     flux::skeleton_lane_capacity(edge, rules.track_width, rules.clearance));
      }
    }
  }
  const bool complete = extracted.complete() && offset.ok() && skeletons_ok;
  std::cout << "{\n"
            << "  \"complete\": " << (complete ? "true" : "false") << ",\n"
            << "  \"polygonal_obstacles\": " << extracted.domain.obstacles.size() << ",\n"
            << "  \"board_cutouts\": " << extracted.domain.cutouts.size() << ",\n"
            << "  \"filled_obstacles\": " << extracted.domain.filled_obstacles.size() << ",\n"
            << "  \"conservative_obstacles\": " << extracted.conservative_obstacles.size() << ",\n"
            << "  \"unsupported_obstacles\": " << extracted.unsupported_obstacles.size() << ",\n"
            << "  \"offset_regions\": " << offset.regions.size() << ",\n"
            << "  \"offset_vertices\": " << offset.output_vertices << ",\n"
            << "  \"skeleton_nodes\": " << skeleton_nodes << ",\n"
            << "  \"skeleton_edges\": " << skeleton_edges << ",\n"
            << "  \"skeleton_junctions\": " << skeleton_junctions << ",\n"
            << "  \"curved_edges_seen\": " << curved_edges_seen << ",\n"
            << "  \"accepted_curved_edge_chords\": " << accepted_curved_edge_chords << ",\n"
            << "  \"withheld_curved_edges\": " << withheld_curved_edges << ",\n"
            << "  \"peak_lane_capacity\": " << peak_lane_capacity << "\n"
            << "}\n";
  return complete;
}

[[nodiscard]] bool run_inspect_free_space_kicad(const std::filesystem::path& path,
                                                std::string_view net_selector,
                                                std::string_view layer_selector,
                                                bool shared_domain) {
  const auto imported = flux::io::import_kicad_pcb_file(path);
  if (!imported.ok()) {
    print_import_result(imported);
    return false;
  }
  const auto& board = *imported.board;
  const auto net = std::find_if(board.nets.begin(), board.nets.end(), [net_selector](const Net& n) {
    return n.id.value() == net_selector || n.name == net_selector;
  });
  const auto layer =
      std::find_if(board.layers.begin(), board.layers.end(), [layer_selector](const Layer& l) {
        return l.kind == LayerKind::copper &&
               (l.id.value() == layer_selector || l.name == layer_selector);
      });
  if (net == board.nets.end() || layer == board.layers.end()) {
    std::cerr << "inspect-free-space-kicad requires an existing net name/ID and copper layer\n";
    return false;
  }

  const auto rules = flux::resolve_net_routing_rules(board, net->id);
  const auto extracted = flux::extract_polygonal_board_free_space(
      board, flux::BoardFreeSpaceExtractionConfig{layer->id, net->id, true, !shared_domain});
  flux::FreeSpaceOffsetResult offset;
  if (extracted.complete()) {
    offset = flux::offset_polygonal_free_space(
        extracted.domain,
        flux::FreeSpaceOffsetConfig{
            shared_domain ? Nanometres{0}
                          : Nanometres{rules.clearance.value() +
                                       rules.track_width.value() / static_cast<std::int64_t>(2)}});
  }
  std::size_t skeleton_nodes = 0;
  std::size_t skeleton_edges = 0;
  std::size_t skeleton_junctions = 0;
  std::size_t curved_edges_seen = 0;
  std::size_t accepted_curved_edge_chords = 0;
  std::size_t withheld_curved_edges = 0;
  std::size_t zero_capacity_edges = 0;
  std::size_t peak_lane_capacity = 0;
  bool skeletons_ok = offset.ok();
  if (offset.ok()) {
    for (const auto& region : offset.regions) {
      flux::FreeSpaceDomain domain;
      domain.outline = region.area;
      domain.cutouts = region.holes;
      const auto skeleton = flux::build_free_space_skeleton(
          domain, flux::FreeSpaceSkeletonConfig{Nanometres{}, 200'000U, shared_domain});
      skeletons_ok = skeletons_ok && skeleton.ok();
      skeleton_nodes += skeleton.nodes.size();
      skeleton_edges += skeleton.edges.size();
      skeleton_junctions += skeleton.junctions;
      curved_edges_seen += skeleton.curved_edges_seen;
      accepted_curved_edge_chords += skeleton.accepted_curved_edge_chords;
      withheld_curved_edges += skeleton.skipped_curved_edges;
      for (const auto& edge : skeleton.edges) {
        const auto capacity =
            flux::skeleton_lane_capacity(edge, rules.track_width, rules.clearance);
        zero_capacity_edges += capacity == 0U ? 1U : 0U;
        peak_lane_capacity = std::max(peak_lane_capacity, capacity);
      }
    }
  }
  const bool geometry_complete = extracted.complete() && offset.ok() && skeletons_ok;
  std::cout << "{\n"
            << "  \"geometry_complete\": " << (geometry_complete ? "true" : "false") << ",\n"
            << "  \"source_strict\": " << (imported.strict() ? "true" : "false") << ",\n"
            << "  \"source_lossy\": " << (imported.lossy ? "true" : "false") << ",\n"
            << "  \"shared_domain\": " << (shared_domain ? "true" : "false") << ",\n"
            << "  \"source_revision\": " << json_quote(board.source_revision) << ",\n"
            << "  \"net_id\": " << json_quote(net->id.value()) << ",\n"
            << "  \"net_name\": " << json_quote(net->name) << ",\n"
            << "  \"layer\": " << json_quote(layer->id.value()) << ",\n"
            << "  \"track_width_nm\": " << rules.track_width.value() << ",\n"
            << "  \"clearance_nm\": " << rules.clearance.value() << ",\n"
            << "  \"polygonal_obstacles\": " << extracted.domain.obstacles.size() << ",\n"
            << "  \"board_cutouts\": " << extracted.domain.cutouts.size() << ",\n"
            << "  \"filled_obstacles\": " << extracted.domain.filled_obstacles.size() << ",\n"
            << "  \"conservative_obstacles\": " << extracted.conservative_obstacles.size() << ",\n"
            << "  \"unsupported_obstacles\": " << extracted.unsupported_obstacles.size() << ",\n"
            << "  \"extraction_diagnostic\": " << json_quote(extracted.diagnostic) << ",\n"
            << "  \"offset_regions\": " << offset.regions.size() << ",\n"
            << "  \"offset_vertices\": " << offset.output_vertices << ",\n"
            << "  \"offset_diagnostic\": " << json_quote(offset.diagnostic) << ",\n"
            << "  \"skeleton_nodes\": " << skeleton_nodes << ",\n"
            << "  \"skeleton_edges\": " << skeleton_edges << ",\n"
            << "  \"skeleton_junctions\": " << skeleton_junctions << ",\n"
            << "  \"curved_edges_seen\": " << curved_edges_seen << ",\n"
            << "  \"accepted_curved_edge_chords\": " << accepted_curved_edge_chords << ",\n"
            << "  \"zero_capacity_edges\": " << zero_capacity_edges << ",\n"
            << "  \"withheld_curved_edges\": " << withheld_curved_edges << ",\n"
            << "  \"peak_lane_capacity\": " << peak_lane_capacity << "\n"
            << "}\n";
  return geometry_complete;
}

void print_help() {
  std::cout << "flux-cli 0.128.0\n\n"
               "Usage:\n"
               "  flux-cli version       Print the engine version\n"
               "  flux-cli capabilities --json  Print the pinned feature-parity contract\n"
               "  flux-cli system-info --json  Print production GPU and schema identity\n"
               "  flux-cli demo          Evaluate a valid built-in board\n"
               "  flux-cli demo-invalid  Evaluate an overlapping built-in board\n"
               "  flux-cli route-demo [--strict-medial] [--direct-medial] Globally route the "
               "built-in board\n"
               "  flux-cli free-space-demo Build offset and Voronoi free-space geometry\n"
               "  flux-cli incremental-route-demo Reuse an unaffected route checkpoint\n"
               "  flux-cli detail-demo [--strict-medial] [--direct-medial] [--arcs] Globally "
               "route and materialize built-in copper\n"
               "  flux-cli incremental-detail-demo Reuse unaffected physical copper\n"
               "  flux-cli optimize-demo Run routing-guided placement repair\n"
               "  flux-cli orientation-optimize-demo Run opt-in rotation repair\n"
               "  flux-cli group-optimize-demo Run atomic declared-group routing repair\n"
               "  flux-cli constraint-global-optimize-demo Run atomic constraint-component "
               "global placement\n"
               "  flux-cli routing-guided-global-optimize-demo Run PathFinder-guided global "
               "placement\n"
               "  flux-cli refine-demo Improve an already feasible routed checkpoint\n"
               "  flux-cli global-optimize-demo Run global placement with routed acceptance\n"
               "  flux-cli inspect-kicad BOARD [--tentative-envelopes]\n"
               "  flux-cli changeset-kicad SOURCE CANDIDATE\n"
               "  flux-cli autoroute-kicad BOARD OUTPUT [--netclasses SNAPSHOT] "
               "[--diffpairs SIDECAR] [--buses SIDECAR] [--no-telemetry] "
               "[--telemetry-listen ADDRESS:PORT] [--telemetry-rate HZ] "
               "[--telemetry-level normal|diagnostic|trace] [--telemetry-record FILE]\n"
               "    [--xpbd-relaxation] [--xpbd-feedback] [--xpbd-cpu] "
               "[--xpbd-iterations N] [--xpbd-max-step-nm NM]\n"
               "  flux-cli export-dsn-kicad BOARD OUTPUT.dsn [--netclasses SNAPSHOT] "
               "[--refillable-zones]\n"
               "  flux-cli import-ses-kicad BOARD INPUT.dsn RESULT.ses OUTPUT.kicad_pcb "
               "[--netclasses SNAPSHOT]\n"
               "  flux-cli inspect-free-space-kicad BOARD NET LAYER [--shared-domain]\n"
               "  flux-cli constraints-template-kicad BOARD\n"
               "  flux-cli envelopes-template-kicad BOARD\n"
               "  flux-cli diffpairs-template-kicad BOARD\n"
               "  flux-cli buses-template-kicad BOARD\n"
               "  flux-cli placement-groups-template-kicad BOARD\n"
               "  flux-cli placement-regions-template-kicad BOARD\n"
               "  flux-cli relative-placements-template-kicad BOARD\n"
               "  flux-cli placement-move-priorities-template-kicad BOARD\n"
               "  flux-cli route-kicad BOARD [--netclasses SNAPSHOT] [--diffpairs SIDECAR] "
               "[--route-net NAME]... [--route-first NAME]... [--repair-sampled-conflicts] "
               "[--buses SIDECAR] [--envelopes SIDECAR] [--placement-groups SIDECAR] "
               "[--placement-regions SIDECAR] "
               "[--relative-placements SIDECAR] "
               "[--max-router-iterations N] "
               "[--max-static-seed-workers N] "
               "[--no-telemetry] [--telemetry-listen ADDRESS:PORT] "
               "[--telemetry-rate HZ] [--telemetry-level normal|diagnostic|trace] "
               "[--telemetry-record FILE] "
               "[--grid-pitch-nm NM] [--via-site-pitch-nm NM] "
               "[--via-site-phase-x N] [--via-site-phase-y N] "
               "[--diagonal-grid | --crossing-safe-diagonal-grid] "
               "[--sampled-clearance-reservations] "
               "[--clearance-shadow-cost-quantum-nm NM] "
               "[--capacity-reserve-per-mille N] [--capacity-reserve-cost N] "
               "[--max-expansions N] [--expansion-retry-heuristic-weight-per-mille N] "
               "[--reuse-weighted-search-frontier] "
               "[--physical-overflow-objective] "
               "[--physical-overflow-history] "
               "[--physical-envelope-ordering] "
               "[--search-diversification] "
               "[--present-congestion-multiplier-per-mille N] "
               "[--max-present-congestion-scale-per-mille N] "
               "[--present-congestion-escalation-start-iteration N] "
               "[--pin-escape-radius N] [--max-pin-access-nodes N] "
               "[--local-pin-escape-pitch-nm NM] [--max-local-pin-escape-expansions N] "
               "[--multisource-steiner] [--atomic-net-trees] "
               "[--selective-congestion-reroute] [--selective-full-reroute-interval N] "
               "[--strict-medial] [--direct-medial] [--negotiate-lane-conflicts]\n"
               "  flux-cli detail-kicad BOARD [--netclasses SNAPSHOT] [--diffpairs SIDECAR] "
               "[--buses SIDECAR] [--envelopes SIDECAR] [--placement-groups SIDECAR] "
               "[--placement-regions SIDECAR] "
               "[--relative-placements SIDECAR] "
               "[--strict-medial] [--direct-medial] [--negotiate-lane-conflicts] [--arcs] "
               "[--refillable-zones] [--route-net NAME]... [--route-first NAME]... "
               "[--repair-sampled-conflicts] "
               "[--no-telemetry] [--telemetry-listen ADDRESS:PORT] "
               "[--telemetry-rate HZ] [--telemetry-level normal|diagnostic|trace] "
               "[--telemetry-record FILE] "
               "[--max-final-routability-feedback-passes N] "
               "[--write-incomplete-preview | --diagnostic-lossy-preview] "
               "[--xpbd-relaxation] [--xpbd-feedback] [--xpbd-cpu] "
               "[--xpbd-iterations N] [--xpbd-max-step-nm NM] "
               "[--tune-differential-pair-skew] [--batch-assign-differential-pair-skew] "
               "[OUTPUT]\n"
               "  flux-cli optimize-kicad BOARD [--netclasses SNAPSHOT] [--diffpairs SIDECAR] "
               "[--buses SIDECAR] [--envelopes SIDECAR] [--placement-groups SIDECAR] "
               "[--placement-regions SIDECAR] "
               "[--relative-placements SIDECAR] "
               "[--placement-move-priorities SIDECAR] "
               "[--suggested-placement-move-priorities] "
               "[--strict-medial] [--direct-medial] [--arcs] "
               "[--tune-differential-pair-skew] [--batch-assign-differential-pair-skew] "
               "[--swaps] [--group-moves] "
               "[--relative-component-moves] "
               "[--tentative-envelopes] "
               "[OUTPUT]\n"
               "  flux-cli global-optimize-kicad BOARD [--netclasses SNAPSHOT] "
               "[--diffpairs SIDECAR] [--buses SIDECAR] [--envelopes SIDECAR] "
               "[--placement-groups SIDECAR] [--placement-regions SIDECAR] [--strict-medial] "
               "[--relative-placements SIDECAR] "
               "[--placement-move-priorities SIDECAR] "
               "[--suggested-placement-move-priorities] "
               "[--direct-medial] [--arcs] [--tune-differential-pair-skew] "
               "[--batch-assign-differential-pair-skew] "
               "[--routing-guided-global-placement] "
               "[--assignment-global-placement] "
               "[--swaps] [--group-moves] [--relative-component-moves] "
               "[--tentative-envelopes] [OUTPUT]\n"
               "  flux-cli constrained-optimize-kicad BOARD SIDECAR [--netclasses SNAPSHOT] "
               "[--diffpairs SIDECAR] [--buses SIDECAR] [--envelopes SIDECAR] "
               "[--placement-groups SIDECAR] [--placement-regions SIDECAR] [--strict-medial] "
               "[--relative-placements SIDECAR] "
               "[--placement-move-priorities SIDECAR] "
               "[--suggested-placement-move-priorities] "
               "[--direct-medial] [--arcs] [--tune-differential-pair-skew] "
               "[--batch-assign-differential-pair-skew] "
               "[--swaps] [--group-moves] [--relative-component-moves] "
               "[--tentative-envelopes] [OUTPUT]\n"
               "  flux-cli constrained-refine-kicad BOARD SIDECAR [--netclasses SNAPSHOT] "
               "[--diffpairs SIDECAR] [--buses SIDECAR] [--envelopes SIDECAR] "
               "[--placement-groups SIDECAR] [--placement-regions SIDECAR] [--strict-medial] "
               "[--relative-placements SIDECAR] "
               "[--placement-move-priorities SIDECAR] "
               "[--suggested-placement-move-priorities] "
               "[--direct-medial] [--arcs] [--tune-differential-pair-skew] "
               "[--batch-assign-differential-pair-skew] "
               "[--swaps] [--group-moves] [--relative-component-moves] "
               "[--tentative-envelopes] [OUTPUT]\n"
               "  Placement optimize commands additionally accept [--max-placement-passes N] "
               "[--max-candidate-evaluations N] [--max-candidate-evaluation-workers N] "
               "[--max-full-routing-candidates-per-wave N] "
               "[--candidate-evaluation-offset N] "
               "[--move-step-nm N].\n"
               "  WebSocket telemetry is enabled by default on 127.0.0.1:8765 for all KiCad "
               "routing/optimization commands; use [--no-telemetry] to disable it.\n"
               "  All KiCad route/detail/optimize commands accept [--grid-pitch-nm N] "
               "[--max-router-iterations N] "
               "[--max-static-seed-workers N] "
               "[--max-expansions N] [--expansion-retry-heuristic-weight-per-mille N] "
               "[--reuse-weighted-search-frontier] "
               "[--physical-overflow-objective] "
               "[--physical-overflow-history] "
               "[--physical-envelope-ordering] "
               "[--search-diversification] "
               "[--present-congestion-multiplier-per-mille N] "
               "[--max-present-congestion-scale-per-mille N] "
               "[--present-congestion-escalation-start-iteration N] "
               "[--hard-capacity-start-iteration N] "
               "[--max-hard-capacity-ripup-attempts-per-pass N] "
               "[--max-hard-capacity-ripup-nets-per-attempt N] "
               "[--max-hard-capacity-conflict-set-states-per-attempt N] "
               "[--rollback-failed-hard-capacity-ripup] "
               "[--defer-hard-capacity-ripup-victims-to-pass-tail] "
               "[--protect-recovered-hard-capacity-ripup-nets] "
               "[--max-hard-capacity-blocker-reuse-invalidations-per-failed-net N] "
               "[--pin-escape-radius N] "
               "[--max-pin-access-nodes N] [--diversify-pin-access-rings] "
               "[--local-pin-escape-pitch-nm N] "
               "[--max-local-pin-escape-expansions N] "
               "[--crossing-safe-diagonal-grid] "
               "[--sampled-clearance-reservations] "
               "[--clearance-shadow-cost-quantum-nm N] "
               "[--capacity-reserve-per-mille N] [--capacity-reserve-cost N] "
               "[--max-corridor-neck-down-states N] "
               "[--max-voronoi-resource-segment-length-nm N] "
               "[--width-reduction-cost-per-grid-step N] [--river-following-cost N] "
               "[--former-route-memory-cost N] [--terminal-escape-reservation-cost N] "
               "[--terminal-escape-proximity-cost N] "
               "[--max-terminal-escape-reservation-resources N] "
               "[--multisource-steiner] "
               "[--atomic-net-trees] [--selective-congestion-reroute] "
               "[--selective-full-reroute-interval N]. Terminal-only "
               "controlled width requires both "
               "[--terminal-neck-down-width-nm N] and "
               "[--max-terminal-neck-down-length-nm N].\n"
               "Direct-medial controlled width additionally requires [--direct-medial], "
               "[--medial-neck-down-width-nm N], and "
               "[--max-medial-neck-down-length-nm N]. Sampled-corridor controlled width requires "
               "[--corridor-neck-down-width-nm N] and "
               "[--max-corridor-neck-down-length-nm N] and also enables width-aware shared "
               "medial capacity. Longitudinal resource segmentation requires "
               "[--max-voronoi-resource-segment-length-nm N] and also enables shared medial "
               "capacity. The physical-overflow checkpoint objective and history policy "
               "and physical-envelope ordering require [--strict-medial] or [--direct-medial]. "
               "[--negotiate-lane-conflicts] feeds exact-rejected selected lane bridges into "
               "later PathFinder passes and enables shared medial allocation. "
               "Selective congestion rerouting requires [--atomic-net-trees]; periodic full "
               "rerouting additionally requires [--selective-congestion-reroute].\n";
}

} // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    print_help();
    return EXIT_SUCCESS;
  }
  const std::string_view command{argv[1]};
  if (command == "version" && argc == 2) {
    std::cout << "flux_capacitor 0.128.0 board-schema 18\n";
    return EXIT_SUCCESS;
  }
  if (command == "capabilities" && argc == 3 && std::string_view{argv[2]} == "--json") {
    std::cout << flux::generated::parity_manifest_json;
    if (!flux::generated::parity_manifest_json.ends_with('\n')) {
      std::cout << '\n';
    }
    return EXIT_SUCCESS;
  }
  if (command == "system-info" && argc == 3 && std::string_view{argv[2]} == "--json") {
    const auto device = flux::routing_compute_device_info();
    const char* override_value = std::getenv("FLUX_VULKAN_DEVICE");
    std::cout << "{\n"
              << "  \"engine\": \"flux_capacitor\",\n"
              << "  \"version\": \"0.128.0\",\n"
              << "  \"board_schema\": 18,\n"
              << "  \"production_gpu_required\": true,\n"
              << "  \"gpu_available\": " << (device.available ? "true" : "false") << ",\n"
              << "  \"gpu_device\": " << json_quote(device.device_name) << ",\n"
              << "  \"gpu_device_type\": " << json_quote(device.device_type) << ",\n"
              << "  \"gpu_vendor_id\": " << device.vendor_id << ",\n"
              << "  \"gpu_device_id\": " << device.device_id << ",\n"
              << "  \"gpu_api_version_raw\": " << device.api_version << ",\n"
              << "  \"gpu_driver_version_raw\": " << device.driver_version << ",\n"
              << "  \"gpu_override\": "
              << (override_value == nullptr ? "null" : json_quote(override_value)) << ",\n"
              << "  \"gpu_diagnostic\": " << json_quote(device.diagnostic) << "\n"
              << "}\n";
    return device.available ? EXIT_SUCCESS : 4;
  }
  if ((command == "demo" || command == "demo-invalid") && argc == 2) {
    const auto result = flux::evaluate_board(make_demo_board(command == "demo-invalid"));
    print_result(result);
    return result.validation.ok() ? EXIT_SUCCESS : 2;
  }
  if (command == "route-demo" && argc >= 2 && argc <= 4) {
    bool strict_medial = false;
    bool direct_medial = false;
    for (auto index = 2; index < argc; ++index) {
      const std::string_view argument{argv[index]};
      if (argument == "--strict-medial" && !strict_medial) {
        strict_medial = true;
      } else if (argument == "--direct-medial" && !direct_medial) {
        direct_medial = true;
      } else {
        print_help();
        return EXIT_FAILURE;
      }
    }
    flux::GlobalRouterConfig config;
    config.grid_pitch = Nanometres::from_mm(5.0);
    config.emit_voronoi_corridor_evidence = true;
    config.use_voronoi_pathfinder_guidance = true;
    config.use_voronoi_allocation_resources = true;
    config.require_voronoi_topological_transitions = strict_medial;
    config.use_direct_voronoi_resource_edges = direct_medial;
    const auto result = flux::plan_routes(make_demo_board(false, false),
                                          flux::RoutingPlannerConfig{config, {}, 2U, {}})
                            .coarse_global;
    print_route_result(result);
    return result.complete() ? EXIT_SUCCESS : 3;
  }
  if (command == "free-space-demo" && argc == 2) {
    return run_free_space_demo() ? EXIT_SUCCESS : 8;
  }
  if (command == "incremental-route-demo" && argc == 2) {
    flux::GlobalRouterConfig config;
    config.grid_pitch = Nanometres::from_mm(5.0);
    auto board = make_incremental_route_demo();
    const auto checkpoint = flux::route_global(board, config);
    board.footprints[1].position = mm(70.0, 40.0);
    const auto result = flux::route_global_incremental(board, checkpoint, {NetId{"N1"}}, config);
    print_route_result(result);
    return result.complete() ? EXIT_SUCCESS : 3;
  }
  if (command == "detail-demo" && argc >= 2 && argc <= 5) {
    bool strict_medial = false;
    bool direct_medial = false;
    bool circular_arcs = false;
    for (auto index = 2; index < argc; ++index) {
      const std::string_view argument{argv[index]};
      if (argument == "--strict-medial" && !strict_medial) {
        strict_medial = true;
      } else if (argument == "--direct-medial" && !direct_medial) {
        direct_medial = true;
      } else if (argument == "--arcs" && !circular_arcs) {
        circular_arcs = true;
      } else {
        print_help();
        return EXIT_FAILURE;
      }
    }
    flux::GlobalRouterConfig global_config;
    global_config.grid_pitch = Nanometres::from_mm(5.0);
    global_config.emit_voronoi_corridor_evidence = strict_medial;
    global_config.use_voronoi_pathfinder_guidance = strict_medial;
    global_config.use_voronoi_allocation_resources = strict_medial || direct_medial;
    global_config.require_voronoi_topological_transitions = strict_medial;
    global_config.use_direct_voronoi_resource_edges = direct_medial;
    const auto board = make_demo_board(false, false);
    const auto global =
        flux::plan_routes(board, flux::RoutingPlannerConfig{global_config, {}, 2U, {}})
            .coarse_global;
    flux::DetailedRouterConfig detailed_config;
    detailed_config.topology_locked_medial_materialization = strict_medial;
    detailed_config.circular_arc_refinement = circular_arcs;
    const auto detailed = flux::materialize_detailed_route(board, global, detailed_config);
    print_detailed_result(detailed, std::nullopt, std::nullopt, &global);
    return detailed.complete() ? EXIT_SUCCESS : 5;
  }
  if (command == "incremental-detail-demo" && argc == 2) {
    flux::GlobalRouterConfig global_config;
    global_config.grid_pitch = Nanometres::from_mm(5.0);
    flux::DetailedRouterConfig detailed_config;
    detailed_config.existing_routing =
        flux::ExistingRoutingPolicy::replace_unlocked_for_routed_nets;
    auto board = make_incremental_route_demo();
    const auto global_checkpoint = flux::route_global(board, global_config);
    const auto detailed_checkpoint =
        flux::materialize_detailed_route(board, global_checkpoint, detailed_config);
    board.footprints[1].position = mm(70.0, 40.0);
    const auto global =
        flux::route_global_incremental(board, global_checkpoint, {NetId{"N1"}}, global_config);
    const auto detailed = flux::materialize_detailed_route_incremental(
        board, global, detailed_checkpoint, detailed_config);
    print_detailed_result(detailed);
    return detailed.complete() ? EXIT_SUCCESS : 5;
  }
  if (command == "optimize-demo" && argc == 2) {
    auto board = make_route_conflict_demo();
    flux::IntegratedOptimizerConfig config;
    config.global_router.grid_pitch = Nanometres::from_mm(5.0);
    config.move_step = Nanometres::from_mm(5.0);
    config.max_passes = 1;
    config.max_candidate_evaluations = 2;
    config.allow_diagonal_moves = false;
    flux::PlacementMovePriorityPolicy priorities;
    priorities.source_revision = board.source_revision;
    priorities.order = flux::default_placement_move_priority_order();
    for (const auto& footprint : board.footprints) {
      priorities.footprints.push_back(
          flux::FootprintMovePriority{footprint.id, flux::PlacementMovePriorityClass::connector});
    }
    config.placement_move_priorities = std::move(priorities);
    const auto optimized = flux::optimize_integrated_placement_routing(board, config);
    print_integrated_result(optimized);
    return optimized.complete() ? EXIT_SUCCESS : 6;
  }
  if (command == "orientation-optimize-demo" && argc == 2) {
    flux::IntegratedOptimizerConfig config;
    config.global_router.grid_pitch = Nanometres::from_mm(5.0);
    config.rotation_offsets = {Microdegrees::from_degrees(180.0)};
    config.enable_routing_guided_moves = false;
    config.allow_diagonal_moves = false;
    config.max_passes = 1;
    config.max_candidate_evaluations = 2;
    const auto optimized =
        flux::optimize_integrated_placement_routing(make_rotation_conflict_demo(), config);
    print_integrated_result(optimized);
    return optimized.complete() ? EXIT_SUCCESS : 6;
  }
  if (command == "group-optimize-demo" && argc == 2) {
    flux::IntegratedOptimizerConfig config;
    config.global_router.grid_pitch = Nanometres::from_mm(5.0);
    config.move_step = Nanometres::from_mm(5.0);
    config.enable_group_moves = true;
    config.allow_diagonal_moves = false;
    config.max_passes = 1;
    config.max_candidate_evaluations = 2;
    const auto optimized =
        flux::optimize_integrated_placement_routing(make_group_conflict_demo(), config);
    print_integrated_result(optimized);
    return optimized.complete() ? EXIT_SUCCESS : 6;
  }
  if (command == "constraint-global-optimize-demo" && argc == 2) {
    flux::IntegratedOptimizerConfig config;
    config.enable_global_placement = true;
    config.global_placer.iterations = 30;
    config.global_placer.attraction_strength = 0.5L;
    config.global_placer.source_anchor_weight = 0.05L;
    config.global_placer.placement_grid = Nanometres::from_mm(5.0);
    config.global_router.grid_pitch = Nanometres::from_mm(5.0);
    config.max_passes = 1;
    config.max_candidate_evaluations = 2;
    config.allow_diagonal_moves = false;
    const auto optimized = flux::optimize_integrated_placement_routing(
        make_constraint_component_conflict_demo(), config);
    print_integrated_result(optimized);
    return optimized.complete() ? EXIT_SUCCESS : 6;
  }
  if (command == "routing-guided-global-optimize-demo" && argc == 2) {
    flux::IntegratedOptimizerConfig config;
    config.enable_global_placement = true;
    config.enable_global_routing_guidance = true;
    config.enable_routing_guided_moves = false;
    config.global_router.grid_pitch = Nanometres::from_mm(5.0);
    config.global_router.present_congestion_cost = 0U;
    config.global_router.history_congestion_cost = 0U;
    config.global_router.max_iterations = 6U;
    config.global_placer.iterations = 20U;
    config.global_placer.density_strength = 0.0L;
    config.global_placer.routing_guidance_weight = 4.0L;
    config.global_placer.placement_grid = Nanometres::from_mm(1.0);
    config.max_passes = 1U;
    config.max_candidate_evaluations = 2U;
    const auto optimized =
        flux::optimize_integrated_placement_routing(make_global_routing_guidance_demo(), config);
    print_integrated_result(optimized);
    return optimized.complete() ? EXIT_SUCCESS : 6;
  }
  if (command == "refine-demo" && argc == 2) {
    flux::IntegratedOptimizerConfig config;
    config.global_router.grid_pitch = Nanometres::from_mm(5.0);
    config.move_step = Nanometres::from_mm(20.0);
    config.max_passes = 2;
    config.max_candidate_evaluations = 20;
    config.allow_diagonal_moves = false;
    config.optimize_routed_quality = true;
    const auto optimized =
        flux::optimize_integrated_placement_routing(make_quality_refinement_demo(), config);
    print_integrated_result(optimized);
    return optimized.complete() ? EXIT_SUCCESS : 6;
  }
  if (command == "global-optimize-demo" && argc == 2) {
    flux::IntegratedOptimizerConfig config;
    config.enable_global_placement = true;
    config.global_placer.iterations = 30;
    config.global_placer.attraction_strength = 0.5L;
    config.global_placer.source_anchor_weight = 0.05L;
    config.global_placer.placement_grid = Nanometres::from_mm(5.0);
    config.global_router.grid_pitch = Nanometres::from_mm(5.0);
    config.max_passes = 1;
    config.max_candidate_evaluations = 4;
    config.move_step = Nanometres::from_mm(5.0);
    config.allow_diagonal_moves = false;
    const auto optimized =
        flux::optimize_integrated_placement_routing(make_route_conflict_demo(true), config);
    print_integrated_result(optimized);
    return optimized.complete() ? EXIT_SUCCESS : 6;
  }
  if (command == "export-dsn-kicad" && argc >= 4) {
    const auto tail = parse_kicad_command_tail(argc, argv, 4, false, false, false, true);
    if (!tail || tail->differential_pair_constraints || tail->routing_bus_constraints ||
        tail->placement_envelope_constraints || tail->placement_group_constraints ||
        tail->placement_region_constraints || tail->relative_placement_constraints) {
      std::cerr << "invalid export-dsn-kicad options\n";
      return 2;
    }
    const auto imported = flux::io::import_kicad_pcb_file(std::filesystem::path{argv[2]});
    if (!imported.strict()) {
      print_import_result(imported);
      std::cerr << "export-dsn-kicad requires a strict, lossless import\n";
      return 4;
    }
    const auto source = apply_optional_routing_policy(*imported.board, tail->netclass_snapshot,
                                                      std::nullopt, std::nullopt, std::nullopt,
                                                      std::nullopt, std::nullopt, std::nullopt);
    if (!source) {
      return 4;
    }
    Board dsn_source = *source;
    if (tail->refillable_zone_fills) {
      dsn_source.zones.clear();
    }
    const auto result =
        flux::io::export_specctra_dsn_file(dsn_source, std::filesystem::path{argv[3]});
    print_specctra_export(result, result.ok());
    return result.ok() ? EXIT_SUCCESS : 10;
  }
  if (command == "import-ses-kicad" && argc >= 6) {
    const auto tail = parse_kicad_command_tail(argc, argv, 6, false);
    if (!tail || tail->differential_pair_constraints || tail->routing_bus_constraints ||
        tail->placement_envelope_constraints || tail->placement_group_constraints ||
        tail->placement_region_constraints || tail->relative_placement_constraints) {
      std::cerr << "invalid import-ses-kicad options\n";
      return 2;
    }
    const auto source_path = std::filesystem::path{argv[2]};
    const auto imported = flux::io::import_kicad_pcb_file(source_path);
    if (!imported.strict()) {
      print_import_result(imported);
      std::cerr << "import-ses-kicad requires a strict, lossless import\n";
      return 4;
    }
    const auto source = apply_optional_routing_policy(*imported.board, tail->netclass_snapshot,
                                                      std::nullopt, std::nullopt, std::nullopt,
                                                      std::nullopt, std::nullopt, std::nullopt);
    if (!source) {
      return 4;
    }
    const auto dsn = read_binary_file(std::filesystem::path{argv[3]});
    const auto ses = read_binary_file(std::filesystem::path{argv[4]});
    if (!dsn || !ses) {
      std::cerr << "unable to read the DSN and SES provenance artifacts\n";
      return 10;
    }
    const auto session = flux::io::import_specctra_session(*source, *dsn, *ses);
    if (!session.ok()) {
      print_specctra_import(session, false);
      return 10;
    }
    const auto exported = flux::io::export_kicad_candidate_file(
        source_path, std::filesystem::path{argv[5]}, *imported.board, *session.candidate);
    print_specctra_import(session, exported.ok(), &exported);
    return exported.ok() ? EXIT_SUCCESS : 7;
  }
  if (command == "changeset-kicad" && argc == 4) {
    const auto change_set = flux::io::make_kicad_change_set_file(std::filesystem::path{argv[2]},
                                                                 std::filesystem::path{argv[3]});
    print_kicad_change_set(change_set);
    return change_set.ok() ? EXIT_SUCCESS : 7;
  }
  if (command == "inspect-kicad" && (argc == 3 || argc == 4)) {
    flux::io::KicadImportOptions options;
    if (argc == 4) {
      if (std::string_view{argv[3]} != "--tentative-envelopes" &&
          std::string_view{argv[3]} != "--approximate-courtyards") {
        print_help();
        return EXIT_FAILURE;
      }
      options.derive_tentative_placement_envelopes = true;
    }
    const auto imported = flux::io::import_kicad_pcb_file(std::filesystem::path{argv[2]}, options);
    print_import_result(imported);
    return imported.ok() ? EXIT_SUCCESS : 4;
  }
  if (command == "inspect-free-space-kicad" && (argc == 5 || argc == 6)) {
    const bool shared_domain = argc == 6 && std::string_view{argv[5]} == "--shared-domain";
    if (argc == 6 && !shared_domain) {
      print_help();
      return EXIT_FAILURE;
    }
    return run_inspect_free_space_kicad(std::filesystem::path{argv[2]}, argv[3], argv[4],
                                        shared_domain)
               ? EXIT_SUCCESS
               : 8;
  }
  if (command == "constraints-template-kicad" && argc == 3) {
    const auto imported = flux::io::import_kicad_pcb_file(std::filesystem::path{argv[2]});
    if (!imported.strict()) {
      print_import_result(imported);
      std::cerr << "constraints-template-kicad requires a strict, lossless import\n";
      return 4;
    }
    std::cout << flux::io::make_orientation_constraints_template(*imported.board);
    return EXIT_SUCCESS;
  }
  if (command == "envelopes-template-kicad" && argc == 3) {
    const auto imported = flux::io::import_kicad_pcb_file(std::filesystem::path{argv[2]});
    if (!imported.strict()) {
      print_import_result(imported);
      std::cerr << "envelopes-template-kicad requires a strict, lossless import\n";
      return 4;
    }
    std::cout << flux::io::make_placement_envelope_constraints_template(*imported.board);
    return EXIT_SUCCESS;
  }
  if (command == "diffpairs-template-kicad" && argc == 3) {
    const auto imported = flux::io::import_kicad_pcb_file(std::filesystem::path{argv[2]});
    if (!imported.strict()) {
      print_import_result(imported);
      std::cerr << "diffpairs-template-kicad requires a strict, lossless import\n";
      return 4;
    }
    std::cout << flux::io::make_differential_pair_constraints_template(*imported.board);
    return EXIT_SUCCESS;
  }
  if (command == "buses-template-kicad" && argc == 3) {
    const auto imported = flux::io::import_kicad_pcb_file(std::filesystem::path{argv[2]});
    if (!imported.strict()) {
      print_import_result(imported);
      std::cerr << "buses-template-kicad requires a strict, lossless import\n";
      return 4;
    }
    std::cout << flux::io::make_routing_bus_constraints_template(*imported.board);
    return EXIT_SUCCESS;
  }
  if (command == "placement-groups-template-kicad" && argc == 3) {
    const auto imported = flux::io::import_kicad_pcb_file(std::filesystem::path{argv[2]});
    if (!imported.strict()) {
      print_import_result(imported);
      std::cerr << "placement-groups-template-kicad requires a strict, lossless import\n";
      return 4;
    }
    std::cout << flux::io::make_placement_group_constraints_template(*imported.board);
    return EXIT_SUCCESS;
  }
  if (command == "placement-regions-template-kicad" && argc == 3) {
    const auto imported = flux::io::import_kicad_pcb_file(std::filesystem::path{argv[2]});
    if (!imported.strict()) {
      print_import_result(imported);
      std::cerr << "placement-regions-template-kicad requires a strict, lossless import\n";
      return 4;
    }
    std::cout << flux::io::make_placement_region_constraints_template(*imported.board);
    return EXIT_SUCCESS;
  }
  if (command == "relative-placements-template-kicad" && argc == 3) {
    const auto imported = flux::io::import_kicad_pcb_file(std::filesystem::path{argv[2]});
    if (!imported.strict()) {
      print_import_result(imported);
      std::cerr << "relative-placements-template-kicad requires a strict, lossless import\n";
      return 4;
    }
    std::cout << flux::io::make_relative_placement_constraints_template(*imported.board);
    return EXIT_SUCCESS;
  }
  if (command == "placement-move-priorities-template-kicad" && argc == 3) {
    const auto imported = flux::io::import_kicad_pcb_file(std::filesystem::path{argv[2]});
    if (!imported.strict()) {
      print_import_result(imported);
      std::cerr << "placement-move-priorities-template-kicad requires a strict, lossless import\n";
      return 4;
    }
    std::cout << flux::io::make_placement_move_priorities_template(*imported.board);
    return EXIT_SUCCESS;
  }
  if (command == "autoroute-kicad" && argc >= 4) {
    const auto tail = parse_kicad_command_tail(argc, argv, 3, true);
    if (!tail || !autoroute_tail_is_supported(*tail)) {
      print_help();
      return EXIT_FAILURE;
    }
    const auto source_path = std::filesystem::path{argv[2]};
    const auto imported = flux::io::import_kicad_pcb_file(source_path);
    if (!imported.strict()) {
      print_import_result(imported);
      std::cerr << "autoroute-kicad requires a strict, lossless import\n";
      return 4;
    }
    const auto source = apply_optional_routing_policy(
        *imported.board, tail->netclass_snapshot, tail->differential_pair_constraints,
        tail->routing_bus_constraints, std::nullopt, std::nullopt, std::nullopt, std::nullopt);
    if (!source) {
      return 9;
    }
    auto telemetry = make_router_telemetry_service(*tail);
    if (tail->telemetry && !telemetry) {
      std::cerr << "invalid telemetry WebSocket listen endpoint\n";
      return 11;
    }
    if (telemetry) {
      telemetry->publish_source_board(*source, source_path.string());
    }
    const auto autoroute_config = minimal_autoroute_config();
    auto planner_config = flux::RoutingPlannerConfig{};
    planner_config.global = autoroute_config;
    apply_regional_tail(*tail, planner_config.regional);
    auto routing_plan = flux::plan_routes(*source, planner_config, {}, telemetry.get());
    auto global = routing_plan.coarse_global;
    flux::DetailedRouterConfig detailed_config;
    detailed_config.existing_routing =
        flux::ExistingRoutingPolicy::replace_unlocked_for_routed_nets;
    detailed_config.topology_locked_medial_materialization = true;
    auto detailed = flux::materialize_detailed_route(*source, routing_plan, detailed_config);
    if (global.complete() &&
        detailed.stop_reason == flux::DetailedRouterStopReason::copper_violations) {
      const auto discarded_direct_medial_edges = global.used_direct_voronoi_edges;
      const auto discarded_copper_violations = detailed.copper_drc.violation_count();
      const auto discarded_topology_complete_paths = global.topologically_complete_medial_paths;
      const auto discarded_portal_transitions = global.medial_direct_portal_transitions;
      const auto discarded_accepted_portal_transitions =
          global.accepted_direct_portal_lane_transitions;
      const auto discarded_lane_conflict_passes = global.voronoi_lane_conflict_passes;
      const auto discarded_lane_conflict_observations = global.voronoi_lane_conflict_observations;
      const auto discarded_lane_conflict_history_entries =
          global.voronoi_lane_conflict_history_entries;
      const auto discarded_exact_junction_history_entries =
          global.voronoi_exact_junction_conflict_history_entries;
      const auto discarded_unaccepted_lane_transitions =
          global.unaccepted_selected_voronoi_lane_transitions;
      const auto discarded_materialized_topology_paths =
          detailed.materialized_topology_locked_paths;
      const auto discarded_topology_rejection_violations =
          detailed.topology_locked_rejection_copper_violations;
      const auto discarded_topology_subset_candidates =
          detailed.attempted_topology_locked_subset_repair_candidates;
      const auto discarded_attempted_shared_fanout_groups =
          detailed.attempted_shared_resource_fanout_groups;
      const auto discarded_attempted_shared_fanout_candidates =
          detailed.attempted_shared_resource_fanout_candidates;
      const auto discarded_materialized_shared_fanout_groups =
          detailed.materialized_shared_resource_fanout_groups;
      const auto discarded_synchronized_shared_fanouts =
          detailed.synchronized_shared_resource_fanouts;
      const auto discarded_shared_fanout_fallbacks = detailed.shared_resource_fanout_fallbacks;
      const auto discarded_shared_fanout_failures = detailed.shared_resource_fanout_failures;

      auto sampled_config = autoroute_config;
      sampled_config.use_direct_voronoi_resource_edges = false;
      sampled_config.use_voronoi_allocation_resources = false;
      sampled_config.require_voronoi_topological_transitions = false;
      sampled_config.checkpoint_objective =
          flux::GlobalRouteCheckpointObjective::aggregate_overflow;
      sampled_config.scale_voronoi_history_by_overflow = false;
      sampled_config.order_routing_groups_by_physical_envelope = false;
      sampled_config.negotiate_voronoi_lane_conflicts = false;
      auto sampled_planner_config = flux::RoutingPlannerConfig{};
      sampled_planner_config.global = sampled_config;
      apply_regional_tail(*tail, sampled_planner_config.regional);
      auto sampled_plan = flux::plan_routes(*source, sampled_planner_config, {}, telemetry.get());
      auto sampled_global = sampled_plan.coarse_global;
      auto sampled_detailed_config = detailed_config;
      sampled_detailed_config.topology_locked_medial_materialization = false;
      auto sampled_detailed =
          flux::materialize_detailed_route(*source, sampled_plan, sampled_detailed_config);
      const bool fallback_accepted = sampled_global.complete() && sampled_detailed.complete();
      if (fallback_accepted) {
        global = std::move(sampled_global);
        routing_plan = std::move(sampled_plan);
        detailed = std::move(sampled_detailed);
        detailed.diagnostic =
            "direct-medial copper was rejected; sampled-graph reroute completed legally";
      }
      detailed.direct_medial_sampled_fallback_attempted = true;
      detailed.direct_medial_sampled_fallback_accepted = fallback_accepted;
      detailed.discarded_direct_medial_edges = discarded_direct_medial_edges;
      detailed.discarded_direct_medial_copper_violations = discarded_copper_violations;
      detailed.discarded_direct_medial_topology_complete_paths = discarded_topology_complete_paths;
      detailed.discarded_direct_medial_portal_transitions = discarded_portal_transitions;
      detailed.discarded_accepted_direct_portal_lane_transitions =
          discarded_accepted_portal_transitions;
      detailed.discarded_voronoi_lane_conflict_passes = discarded_lane_conflict_passes;
      detailed.discarded_voronoi_lane_conflict_observations = discarded_lane_conflict_observations;
      detailed.discarded_voronoi_lane_conflict_history_entries =
          discarded_lane_conflict_history_entries;
      detailed.discarded_voronoi_exact_junction_conflict_history_entries =
          discarded_exact_junction_history_entries;
      detailed.discarded_unaccepted_selected_voronoi_lane_transitions =
          discarded_unaccepted_lane_transitions;
      detailed.discarded_materialized_topology_locked_paths = discarded_materialized_topology_paths;
      detailed.discarded_topology_locked_rejection_copper_violations =
          discarded_topology_rejection_violations;
      detailed.discarded_attempted_topology_locked_subset_repair_candidates =
          discarded_topology_subset_candidates;
      detailed.discarded_attempted_shared_resource_fanout_groups =
          discarded_attempted_shared_fanout_groups;
      detailed.discarded_attempted_shared_resource_fanout_candidates =
          discarded_attempted_shared_fanout_candidates;
      detailed.discarded_materialized_shared_resource_fanout_groups =
          discarded_materialized_shared_fanout_groups;
      detailed.discarded_synchronized_shared_resource_fanouts =
          discarded_synchronized_shared_fanouts;
      detailed.discarded_shared_resource_fanout_fallbacks = discarded_shared_fanout_fallbacks;
      detailed.discarded_shared_resource_fanout_failures = discarded_shared_fanout_failures;
    }
    if (telemetry) {
      telemetry->stop();
    }
    return finish_kicad_detail(source_path, tail->output, *source, detailed, global,
                               "autoroute_v1");
  }
  if (command == "route-kicad" && argc >= 3) {
    const auto tail = parse_kicad_command_tail(argc, argv, 3, false, false, false, true);
    if (!tail) {
      print_help();
      return EXIT_FAILURE;
    }
    if (tail->circular_arc_refinement || tail->differential_pair_skew_tuning) {
      print_help();
      return EXIT_FAILURE;
    }
    const auto source_path = std::filesystem::path{argv[2]};
    const auto imported = flux::io::import_kicad_pcb_file(source_path);
    if (!imported.strict()) {
      print_import_result(imported);
      std::cerr << "route-kicad requires a strict, lossless import\n";
      return 4;
    }
    const auto source = apply_optional_routing_policy(
        *imported.board, tail->netclass_snapshot, tail->differential_pair_constraints,
        tail->routing_bus_constraints, tail->placement_envelope_constraints,
        tail->placement_group_constraints, tail->placement_region_constraints,
        tail->relative_placement_constraints);
    if (!source) {
      return 9;
    }
    flux::GlobalRouterConfig config;
    config.use_voronoi_allocation_resources =
        !source->routing_buses.empty() || tail->strict_medial_topology ||
        tail->direct_medial_edges || tail->negotiate_voronoi_lane_conflicts ||
        tail->corridor_neck_down_width.has_value() ||
        tail->max_voronoi_resource_segment_length.has_value();
    config.require_voronoi_topological_transitions = tail->strict_medial_topology;
    config.treat_saved_zone_fills_as_refillable = tail->refillable_zone_fills;
    if (!apply_router_tail(*tail, *source, config)) {
      return 10;
    }
    auto telemetry = make_router_telemetry_service(*tail);
    if (tail->telemetry && !telemetry) {
      std::cerr << "invalid telemetry WebSocket listen endpoint\n";
      return 11;
    }
    if (telemetry) {
      telemetry->publish_source_board(*source, source_path.string());
    }
    flux::RegionalRouterConfig regional_config;
    apply_regional_tail(*tail, regional_config);
    flux::RoutingPlanResult routing_plan;
    if (tail->repair_sampled_conflicts) {
      routing_plan.coarse_global =
          route_with_optional_sampled_conflict_repair(*source, config, true, telemetry.get());
      routing_plan.regional = flux::analyze_regional_conflicts(
          *source, routing_plan.coarse_global, regional_config, {}, telemetry.get());
      routing_plan.paths = routing_plan.regional.spline_candidate_available
                               ? routing_plan.regional.spline_candidate_paths
                               : routing_plan.coarse_global.paths;
      routing_plan.stop_reason =
          routing_plan.coarse_global.complete() && routing_plan.regional.complete()
              ? flux::RoutingPlanStopReason::complete
              : flux::RoutingPlanStopReason::regional_conflicts;
    } else {
      auto planner_config = flux::RoutingPlannerConfig{};
      planner_config.global = config;
      planner_config.regional = regional_config;
      routing_plan = flux::plan_routes(*source, planner_config, {}, telemetry.get());
    }
    std::optional<flux::RouterTelemetryStatistics> telemetry_statistics;
    if (telemetry) {
      telemetry->stop();
      telemetry_statistics = telemetry->statistics();
    }
    print_route_result(routing_plan.coarse_global, &routing_plan.regional,
                       telemetry_statistics ? &*telemetry_statistics : nullptr, &routing_plan);
    return routing_plan.complete() ? EXIT_SUCCESS : 3;
  }
  if (command == "detail-kicad" && argc >= 3) {
    const auto tail = parse_kicad_command_tail(argc, argv, 3, true, false, false, true, true);
    if (!tail) {
      print_help();
      return EXIT_FAILURE;
    }
    if (tail->refillable_zone_fills && !tail->output.has_value()) {
      std::cerr << "--refillable-zones requires a provisional output path\n";
      return EXIT_FAILURE;
    }
    if (tail->write_incomplete_preview && !tail->output.has_value()) {
      std::cerr << "copper preview requires an output path\n";
      return EXIT_FAILURE;
    }
    const auto source_path = std::filesystem::path{argv[2]};
    const auto imported = flux::io::import_kicad_pcb_file(source_path);
    if (!imported.board) {
      print_import_result(imported);
      std::cerr << "detail-kicad could not construct a canonical board projection\n";
      return 4;
    }
    if (!imported.strict() && !tail->diagnostic_lossy_preview) {
      print_import_result(imported);
      std::cerr << "detail-kicad requires a strict, lossless import\n";
      return 4;
    }
    if (tail->diagnostic_lossy_preview) {
      std::cerr << "warning: diagnostic lossy preview is non-authoritative; import, structural "
                   "validation, and DRC failures will not suppress copper export\n";
    }
    const auto source = apply_optional_routing_policy(
        *imported.board, tail->netclass_snapshot, tail->differential_pair_constraints,
        tail->routing_bus_constraints, tail->placement_envelope_constraints,
        tail->placement_group_constraints, tail->placement_region_constraints,
        tail->relative_placement_constraints);
    if (!source) {
      return 9;
    }
    flux::GlobalRouterConfig global_config;
    global_config.allow_invalid_source_for_diagnostic_preview = tail->diagnostic_lossy_preview;
    global_config.use_voronoi_allocation_resources =
        !source->routing_buses.empty() || tail->strict_medial_topology ||
        tail->direct_medial_edges || tail->negotiate_voronoi_lane_conflicts ||
        tail->corridor_neck_down_width.has_value() ||
        tail->max_voronoi_resource_segment_length.has_value();
    global_config.require_voronoi_topological_transitions = tail->strict_medial_topology;
    global_config.treat_saved_zone_fills_as_refillable = tail->refillable_zone_fills;
    global_config.single_lane_direct_voronoi_resources =
        tail->refillable_zone_fills && tail->direct_medial_edges;
    if (!apply_router_tail(*tail, *source, global_config)) {
      return 10;
    }
    auto telemetry = make_router_telemetry_service(*tail);
    if (tail->telemetry && !telemetry) {
      std::cerr << "invalid telemetry WebSocket listen endpoint\n";
      return 11;
    }
    if (telemetry) {
      telemetry->publish_source_board(*source, source_path.string());
    }
    flux::RegionalRouterConfig regional_config;
    apply_regional_tail(*tail, regional_config);
    flux::RoutingPlanResult regional_plan;
    if (tail->repair_sampled_conflicts) {
      regional_plan.coarse_global = route_with_optional_sampled_conflict_repair(
          *source, global_config, true, telemetry.get());
      regional_plan.regional = flux::analyze_regional_conflicts(
          *source, regional_plan.coarse_global, regional_config, {}, telemetry.get());
      regional_plan.paths = regional_plan.regional.spline_candidate_available
                                ? regional_plan.regional.spline_candidate_paths
                                : regional_plan.coarse_global.paths;
      regional_plan.stop_reason =
          regional_plan.coarse_global.complete() && regional_plan.regional.complete()
              ? flux::RoutingPlanStopReason::complete
              : flux::RoutingPlanStopReason::regional_conflicts;
    } else {
      auto planner_config = flux::RoutingPlannerConfig{};
      planner_config.global = global_config;
      planner_config.regional = regional_config;
      regional_plan = flux::plan_routes(*source, planner_config, {}, telemetry.get());
    }
    auto global = regional_plan.coarse_global;
    flux::DetailedRouterConfig detailed_config;
    detailed_config.allow_invalid_source_for_diagnostic_preview =
        tail->diagnostic_lossy_preview;
    detailed_config.existing_routing =
        flux::ExistingRoutingPolicy::replace_unlocked_for_routed_nets;
    detailed_config.defer_saved_zone_fill_clearance_to_refill = tail->refillable_zone_fills;
    detailed_config.pull_tight_45_degrees = !tail->refillable_zone_fills;
    detailed_config.continuous_refinement = !tail->refillable_zone_fills;
    detailed_config.topology_locked_medial_materialization =
        !tail->refillable_zone_fills && tail->strict_medial_topology;
    detailed_config.circular_arc_refinement = tail->circular_arc_refinement;
    detailed_config.differential_pair_skew_tuning =
        !tail->refillable_zone_fills && tail->differential_pair_skew_tuning;
    detailed_config.differential_pair_skew_tuning_batch_assignment =
        !tail->refillable_zone_fills && tail->differential_pair_skew_tuning_batch_assignment;
    auto materialization_global = global;
    const bool previewing_incomplete_global = tail->write_incomplete_preview && !global.complete();
    if (previewing_incomplete_global) {
      // Materialize the best paths only for a review artifact. Keep the original global result
      // authoritative in JSON and for the process exit status.
      materialization_global.stop_reason = flux::GlobalRouterStopReason::complete;
      materialization_global.unrouted_nets.clear();
      materialization_global.total_overflow = 0U;
    }
    auto detailed =
        !previewing_incomplete_global
            ? flux::materialize_detailed_route(*source, regional_plan, detailed_config)
            : flux::materialize_detailed_route(*source, materialization_global, detailed_config);
    if (!previewing_incomplete_global && !tail->repair_sampled_conflicts && !detailed.complete() &&
        !detailed.provisional_zone_refill_candidate() && !detailed.routability_diagnoses.empty()) {
      // Exact copper is the first representation that sees every track/via interaction. Consume
      // each rejection as cost-only knowledge and retry from the immutable source. The accepted
      // transaction remains monotonic, but rejected portfolios also teach later passes so one
      // locally worse alternative cannot trap the feedback loop.
      const auto max_feedback_passes = tail->max_final_routability_feedback_passes.value_or(3U);
      const auto exact_quality = [](const flux::DetailedRouteResult& candidate) {
        return std::tuple{
            candidate.complete() ? 0U : 1U,         candidate.candidate ? 0U : 1U,
            candidate.validation.error_count(),     candidate.connectivity.open_connections,
            candidate.copper_drc.violation_count(), candidate.routability_diagnoses.size()};
      };
      auto feedback_knowledge = regional_plan.routability_knowledge;
      auto diagnoses = detailed.routability_diagnoses;
      std::set<flux::NetId> affected_feedback_nets;
      std::size_t diagnoses_consumed = 0U;
      std::size_t feedback_passes = 0U;
      std::size_t improvements = 0U;
      const auto final_feedback_epoch_iterations = std::max<std::size_t>(
          1U, (global_config.max_iterations + std::max<std::size_t>(1U, max_feedback_passes) - 1U) /
                  std::max<std::size_t>(1U, max_feedback_passes));
      for (; feedback_passes < max_feedback_passes && !diagnoses.empty(); ++feedback_passes) {
        diagnoses_consumed += diagnoses.size();
        for (const auto& diagnosis : diagnoses) {
          affected_feedback_nets.insert(diagnosis.resource.nets.begin(),
                                        diagnosis.resource.nets.end());
        }
        if (telemetry) {
          for (const auto& diagnosis : diagnoses) {
            flux::RouterTelemetryEvent event;
            event.kind = flux::RouterTelemetryEventKind::search_summary;
            event.stable_id = diagnosis.resource.stable_id;
            event.detail =
                "final routability diagnosis pass " + std::to_string(feedback_passes + 1U) + ": " +
                std::string(flux::to_string(diagnosis.kind)) +
                ", relaxation=" + std::string(flux::to_string(diagnosis.suggested_relaxation));
            static_cast<void>(telemetry->try_emit(std::move(event)));
          }
        }
        flux::accumulate_clean_copper_knowledge(feedback_knowledge, detailed);
        flux::accumulate_routability_knowledge(feedback_knowledge, diagnoses);
        auto feedback_global_config = global_config;
        feedback_global_config.max_iterations = final_feedback_epoch_iterations;
        if (feedback_global_config.hard_capacity_start_iteration > 0U) {
          feedback_global_config.hard_capacity_start_iteration = 1U;
        }
        if (feedback_global_config.bidirectional_route_growth_start_iteration > 0U) {
          feedback_global_config.bidirectional_route_growth_start_iteration = 1U;
        }
        if (feedback_global_config.present_congestion_escalation_start_iteration > 0U) {
          feedback_global_config.present_congestion_escalation_start_iteration = 1U;
        }
        feedback_global_config.routability_history = feedback_knowledge.history;
        feedback_global_config.hydraulic_success_history = feedback_knowledge.success_history;
        // Preserve every unaffected route from the complete checkpoint. Exact failures are local
        // physical evidence; throwing away all connected topology made the fresh planner lose
        // nets before it could compare a new copper transaction.
        flux::RoutingPlanResult feedback_plan;
        const std::vector<flux::NetId> affected_nets(affected_feedback_nets.begin(),
                                                     affected_feedback_nets.end());
        feedback_plan.coarse_global =
            flux::route_global_incremental(*source, regional_plan.coarse_global, affected_nets,
                                           feedback_global_config, {}, telemetry.get(), true);
        feedback_plan.regional = flux::analyze_regional_conflicts(
            *source, feedback_plan.coarse_global, regional_config, {}, telemetry.get());
        feedback_plan.paths = feedback_plan.regional.spline_candidate_available
                                  ? feedback_plan.regional.spline_candidate_paths
                                  : feedback_plan.coarse_global.paths;
        const auto connected_topology = feedback_plan.coarse_global.unrouted_nets.empty() &&
                                        !feedback_plan.coarse_global.paths.empty();
        const auto physically_realizable = feedback_plan.regional.complete() &&
                                           (feedback_plan.coarse_global.complete() ||
                                            feedback_plan.regional.spline_candidate_available);
        feedback_plan.stop_reason =
            connected_topology && physically_realizable ? flux::RoutingPlanStopReason::complete
            : !connected_topology ? flux::RoutingPlanStopReason::coarse_global_incomplete
                                  : flux::RoutingPlanStopReason::regional_conflicts;
        auto feedback_diagnoses = flux::probe_global_routability(feedback_plan.coarse_global);
        const auto regional_diagnoses = flux::diagnose_regional_routability(feedback_plan.regional);
        feedback_diagnoses.insert(feedback_diagnoses.end(), regional_diagnoses.begin(),
                                  regional_diagnoses.end());
        flux::accumulate_routability_knowledge(feedback_knowledge, feedback_diagnoses);
        feedback_plan.routability_knowledge = feedback_knowledge;
        if (!feedback_plan.complete()) {
          diagnoses = std::move(feedback_diagnoses);
          continue;
        }
        auto feedback_detailed =
            flux::materialize_detailed_route(*source, feedback_plan, detailed_config);
        // Retain diagnoses learned inside the nested global/regional loop as well as the exact
        // materialization diagnoses consumed at the next outer iteration.
        feedback_knowledge = feedback_plan.routability_knowledge;
        flux::accumulate_clean_copper_knowledge(feedback_knowledge, feedback_detailed);
        diagnoses = feedback_detailed.routability_diagnoses;
        if (exact_quality(feedback_detailed) < exact_quality(detailed)) {
          ++improvements;
          regional_plan = std::move(feedback_plan);
          global = regional_plan.coarse_global;
          detailed = std::move(feedback_detailed);
          if (detailed.complete() || detailed.provisional_zone_refill_candidate()) {
            ++feedback_passes;
            break;
          }
        }
      }
      detailed.final_routability_feedback_attempted = feedback_passes > 0U;
      detailed.final_routability_feedback_selected = improvements > 0U;
      detailed.final_routability_feedback_passes = feedback_passes;
      detailed.final_routability_feedback_improvements = improvements;
      detailed.final_routability_diagnoses_consumed = diagnoses_consumed;
    }
    if (previewing_incomplete_global && detailed.complete()) {
      detailed.stop_reason = flux::DetailedRouterStopReason::incomplete_global_route;
      detailed.diagnostic =
          "review-only preview materialized from an incomplete global-route checkpoint";
    }
    if (telemetry) {
      telemetry->stop();
    }
    return finish_kicad_detail(
        std::filesystem::path{argv[2]}, tail->output, *source, detailed, global,
        tail->diagnostic_lossy_preview
            ? std::optional<std::string_view>{"diagnostic_lossy_preview_v1"}
        : tail->refillable_zone_fills
            ? std::optional<std::string_view>{"refillable_zones_v1"}
            : std::nullopt,
        tail->refillable_zone_fills, tail->write_incomplete_preview,
        tail->diagnostic_lossy_preview);
  }
  if ((command == "optimize-kicad" || command == "global-optimize-kicad") && argc >= 3) {
    const auto tail =
        parse_kicad_command_tail(argc, argv, 3, true, true, command == "global-optimize-kicad");
    if (!tail) {
      print_help();
      return EXIT_FAILURE;
    }
    if (tail->repair_sampled_conflicts) {
      std::cerr << "--repair-sampled-conflicts is supported only by route-kicad and detail-kicad\n";
      return EXIT_FAILURE;
    }
    flux::io::KicadImportOptions import_options;
    import_options.derive_tentative_placement_envelopes = tail->tentative_placement_envelopes;
    const auto imported =
        flux::io::import_kicad_pcb_file(std::filesystem::path{argv[2]}, import_options);
    if (!imported.strict()) {
      print_import_result(imported);
      std::cerr << command << " requires a strict, lossless import\n";
      return 4;
    }
    const auto source = apply_optional_routing_policy(
        *imported.board, tail->netclass_snapshot, tail->differential_pair_constraints,
        tail->routing_bus_constraints, tail->placement_envelope_constraints,
        tail->placement_group_constraints, tail->placement_region_constraints,
        tail->relative_placement_constraints);
    if (!source) {
      return 9;
    }
    auto telemetry = make_router_telemetry_service(*tail);
    if (tail->telemetry && !telemetry) {
      std::cerr << "invalid telemetry WebSocket listen endpoint\n";
      return 11;
    }
    if (telemetry) {
      telemetry->publish_source_board(*source, std::filesystem::path{argv[2]}.string());
    }
    flux::IntegratedOptimizerConfig config;
    apply_integrated_tail(*tail, config);
    apply_regional_tail(*tail, config.regional_router);
    config.enable_global_placement = command == "global-optimize-kicad";
    config.allow_tentative_placement_envelopes = tail->tentative_placement_envelopes;
    config.enable_global_routing_guidance = tail->enable_global_routing_guidance;
    config.global_placer.enable_assignment_legalization = tail->enable_assignment_legalization;
    config.enable_swap_moves = tail->enable_swap_moves;
    config.enable_group_moves = tail->enable_group_moves;
    config.enable_relative_component_moves = tail->enable_relative_component_moves;
    if (!apply_placement_move_priorities(*source, tail->placement_move_priorities,
                                         tail->suggested_placement_move_priorities, config)) {
      return 10;
    }
    config.global_router.use_voronoi_allocation_resources =
        !source->routing_buses.empty() || tail->strict_medial_topology ||
        tail->direct_medial_edges || tail->negotiate_voronoi_lane_conflicts ||
        tail->corridor_neck_down_width.has_value() ||
        tail->max_voronoi_resource_segment_length.has_value();
    config.global_router.require_voronoi_topological_transitions = tail->strict_medial_topology;
    if (!apply_router_tail(*tail, *source, config.global_router)) {
      return 10;
    }
    config.detailed_router.topology_locked_medial_materialization = tail->strict_medial_topology;
    config.detailed_router.circular_arc_refinement = tail->circular_arc_refinement;
    config.detailed_router.differential_pair_skew_tuning = tail->differential_pair_skew_tuning;
    config.detailed_router.differential_pair_skew_tuning_batch_assignment =
        tail->differential_pair_skew_tuning_batch_assignment;
    const auto optimized =
        flux::optimize_integrated_placement_routing(*source, config, {}, telemetry.get());
    if (telemetry) {
      telemetry->stop();
    }
    return finish_kicad_optimization(std::filesystem::path{argv[2]}, tail->output, *source,
                                     optimized);
  }
  if ((command == "constrained-optimize-kicad" || command == "constrained-refine-kicad") &&
      argc >= 4) {
    const auto tail = parse_kicad_command_tail(argc, argv, 4, true, true);
    if (!tail) {
      print_help();
      return EXIT_FAILURE;
    }
    if (tail->repair_sampled_conflicts) {
      std::cerr << "--repair-sampled-conflicts is supported only by route-kicad and detail-kicad\n";
      return EXIT_FAILURE;
    }
    const auto source_path = std::filesystem::path{argv[2]};
    flux::io::KicadImportOptions import_options;
    import_options.derive_tentative_placement_envelopes = tail->tentative_placement_envelopes;
    const auto imported = flux::io::import_kicad_pcb_file(source_path, import_options);
    if (!imported.strict()) {
      print_import_result(imported);
      std::cerr << command << " requires a strict, lossless import\n";
      return 4;
    }
    const auto ruled = apply_optional_routing_policy(
        *imported.board, tail->netclass_snapshot, tail->differential_pair_constraints,
        tail->routing_bus_constraints, tail->placement_envelope_constraints,
        tail->placement_group_constraints, tail->placement_region_constraints,
        tail->relative_placement_constraints);
    if (!ruled) {
      return 9;
    }
    const auto constrained =
        flux::io::apply_orientation_constraints_file(*ruled, std::filesystem::path{argv[3]});
    if (!constrained.ok()) {
      print_constraint_result(constrained);
      return 8;
    }
    auto telemetry = make_router_telemetry_service(*tail);
    if (tail->telemetry && !telemetry) {
      std::cerr << "invalid telemetry WebSocket listen endpoint\n";
      return 11;
    }
    if (telemetry) {
      telemetry->publish_source_board(*constrained.board, source_path.string());
    }
    const bool refine = command == "constrained-refine-kicad";
    auto config = constrained_optimizer_config(*constrained.board,
                                               constrained.constrained_footprints, refine);
    apply_integrated_tail(*tail, config);
    apply_regional_tail(*tail, config.regional_router);
    config.allow_tentative_placement_envelopes = tail->tentative_placement_envelopes;
    config.enable_swap_moves = tail->enable_swap_moves;
    config.enable_group_moves = tail->enable_group_moves;
    config.enable_relative_component_moves = tail->enable_relative_component_moves;
    if (!apply_placement_move_priorities(*constrained.board, tail->placement_move_priorities,
                                         tail->suggested_placement_move_priorities, config)) {
      return 10;
    }
    config.global_router.use_voronoi_allocation_resources =
        !constrained.board->routing_buses.empty() || tail->strict_medial_topology ||
        tail->direct_medial_edges || tail->negotiate_voronoi_lane_conflicts ||
        tail->corridor_neck_down_width.has_value() ||
        tail->max_voronoi_resource_segment_length.has_value();
    config.global_router.require_voronoi_topological_transitions = tail->strict_medial_topology;
    if (!apply_router_tail(*tail, *constrained.board, config.global_router)) {
      return 10;
    }
    config.detailed_router.topology_locked_medial_materialization = tail->strict_medial_topology;
    config.detailed_router.circular_arc_refinement = tail->circular_arc_refinement;
    config.detailed_router.differential_pair_skew_tuning = tail->differential_pair_skew_tuning;
    config.detailed_router.differential_pair_skew_tuning_batch_assignment =
        tail->differential_pair_skew_tuning_batch_assignment;
    const auto optimized = flux::optimize_integrated_placement_routing(*constrained.board, config,
                                                                       {}, telemetry.get());
    if (telemetry) {
      telemetry->stop();
    }
    return finish_kicad_optimization(source_path, tail->output, *constrained.board, optimized);
  }
  print_help();
  return EXIT_FAILURE;
}
