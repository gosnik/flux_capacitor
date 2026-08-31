#include "flux/core/copper_geometry.hpp"
#include "flux/core/detailed_router.hpp"
#include "flux/core/geometry.hpp"
#include "flux/core/global_router.hpp"
#include "flux/core/integrated_optimizer.hpp"
#include "flux/core/navmesh_route.hpp"
#include "flux/core/pns_router.hpp"
#include "flux/core/regional_router.hpp"
#include "flux/core/router_telemetry.hpp"
#include "flux/core/spline_route.hpp"
#include "flux/io/kicad_export.hpp"
#include "flux/io/kicad_import.hpp"
#include "flux/io/placement_move_priorities.hpp"
#include "flux/telemetry/event.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <limits>
#include <numeric>
#include <set>
#include <stdexcept>
#include <stop_token>
#include <string>
#include <thread>
#include <vector>

namespace {

using flux::Board;
using flux::BoardSide;
using flux::Footprint;
using flux::GlobalRouteCheckpointObjective;
using flux::GlobalRouterConfig;
using flux::GlobalRouterStopReason;
using flux::Keepout;
using flux::LayerId;
using flux::LayerKind;
using flux::Microdegrees;
using flux::Nanometres;
using flux::Net;
using flux::NetId;
using flux::ObjectId;
using flux::Point;
using flux::Polygon;

struct TestCase {
  std::string name;
  std::function<void()> run;
};

class CollectingTelemetrySink final : public flux::RouterTelemetrySink {
public:
  [[nodiscard]] bool accepts(flux::RouterTelemetryLevel) const noexcept override { return true; }

  [[nodiscard]] bool try_emit(flux::RouterTelemetryEvent event) noexcept override {
    events.push_back(std::move(event));
    return true;
  }

  std::vector<flux::RouterTelemetryEvent> events;
};

[[noreturn]] void fail(const std::string& message) { throw std::runtime_error(message); }

void require(bool condition, const std::string& message) {
  if (!condition) {
    fail(message);
  }
}

[[nodiscard]] Point mm(double x, double y) {
  return Point{Nanometres::from_mm(x), Nanometres::from_mm(y)};
}

[[nodiscard]] Polygon rectangle(double min_x, double min_y, double max_x, double max_y) {
  return Polygon{{mm(min_x, min_y), mm(max_x, min_y), mm(max_x, max_y), mm(min_x, max_y)}};
}

[[nodiscard]] Polygon local_rectangle(double half_width, double half_height) {
  return rectangle(-half_width, -half_height, half_width, half_height);
}

[[nodiscard]] Board two_pad_board() {
  const LayerId front{"F.Cu"};
  const LayerId back{"B.Cu"};
  const NetId net{"net-1"};
  Board board;
  board.source_revision = "risk-fixture";
  board.outline = rectangle(0.0, 0.0, 100.0, 80.0);
  board.layers = {{front, "F.Cu", LayerKind::copper, 0}, {back, "B.Cu", LayerKind::copper, 1}};
  board.nets = {{net, "SIGNAL", 0}};

  const auto footprint = [&front, &net](std::string id, Point position) {
    Footprint result;
    result.id = ObjectId{id};
    result.reference = id;
    result.position = position;
    result.courtyard = local_rectangle(4.0, 3.0);
    result.pads = {{ObjectId{id + "-pad"}, net, Point{}, {front}}};
    return result;
  };
  board.footprints = {footprint("U1", mm(20.0, 40.0)), footprint("U2", mm(80.0, 40.0))};
  return board;
}

[[nodiscard]] std::string load_minimal_kicad_fixture() {
  const auto path = std::filesystem::path{FLUX_TEST_SOURCE_DIR} / "fixtures/minimal.kicad_pcb";
  std::ifstream input(path, std::ios::binary);
  require(static_cast<bool>(input), "minimal KiCad fixture must be readable");
  return std::string{std::istreambuf_iterator<char>{input}, std::istreambuf_iterator<char>{}};
}

[[nodiscard]] std::string load_tentative_placement_fixture() {
  const auto path =
      std::filesystem::path{FLUX_TEST_SOURCE_DIR} / "fixtures/tentative-placement.kicad_pcb";
  std::ifstream input(path, std::ios::binary);
  require(static_cast<bool>(input), "tentative-placement KiCad fixture must be readable");
  return std::string{std::istreambuf_iterator<char>{input}, std::istreambuf_iterator<char>{}};
}

void units_remain_exact_and_saturating() {
  require(Nanometres::from_mm(1.2345674).value() == 1'234'567,
          "millimetres must quantize to integer nanometres");
  require(Nanometres::from_mm(-0.0000006).value() == -1,
          "negative conversion must round symmetrically");
  require(Microdegrees::from_degrees(-90.0).value() == 270'000'000,
          "angles must normalize into one positive turn");
  require(flux::geometry::transform_point(mm(4.45, -0.25), mm(48.0, 47.0),
                                          Microdegrees::from_degrees(90.0),
                                          BoardSide::front) == mm(47.75, 42.55),
          "positive KiCad footprint rotation must follow its downward-Y board coordinates");
}

void dimensioned_pathfinder_pressure_preserves_units_and_group_fairness() {
  const NetId first{"a-first"};
  const NetId second{"b-second"};
  flux::GlobalRouteResult route;

  flux::RoutingResourceUsage sampled;
  sampled.demand = 4U;
  sampled.capacity = 2U;
  sampled.nets = {first, second};
  route.resource_usage.push_back(sampled);

  flux::RoutingCorridorUsage corridor;
  corridor.id = 1U;
  corridor.demand = 5U;
  corridor.capacity = 2U;
  corridor.nets = {first};
  route.corridor_usage.push_back(corridor);
  auto neck_down = corridor;
  neck_down.id = 2U;
  neck_down.demand = 2U;
  neck_down.capacity = 1U;
  neck_down.nets = {second};
  route.corridor_neck_down_usage.push_back(neck_down);

  flux::RoutingVoronoiResourceUsage shared_physical;
  shared_physical.id = 3U;
  shared_physical.demand_nm = 1'000U;
  shared_physical.capacity_nm = 700U;
  shared_physical.nets = {first, second};
  route.voronoi_resource_usage.push_back(shared_physical);
  auto severe_physical = shared_physical;
  severe_physical.id = 4U;
  severe_physical.demand_nm = 900U;
  severe_physical.capacity_nm = 500U;
  severe_physical.nets = {first};
  route.voronoi_resource_usage.push_back(severe_physical);

  const auto pressure = flux::collect_global_route_net_pressure(route);
  require(pressure.size() == 2U && pressure[0].net == first &&
              pressure[0].legacy_congested_resources == 4U && pressure[0].sampled_overflow == 2U &&
              pressure[0].corridor_overflow == 3U &&
              pressure[0].total_physical_overflow_nm == 700U &&
              pressure[0].peak_physical_overflow_nm == 400U &&
              pressure[0].congested_physical_resources == 2U,
          "per-net pressure must remain stable and unit-separated");

  const auto group = flux::collect_global_route_group_pressure(route, {second, first, second});
  require(group.net == first && group.legacy_congested_resources == 5U &&
              group.sampled_overflow == 2U && group.corridor_overflow == 4U &&
              group.total_physical_overflow_nm == 700U && group.peak_physical_overflow_nm == 400U &&
              group.congested_physical_resources == 2U,
          "a coupled group must count a shared resource once regardless of duplicate members");

  flux::RoutingNetPressure broad;
  broad.legacy_congested_resources = 2U;
  flux::RoutingNetPressure narrow;
  narrow.legacy_congested_resources = 1U;
  narrow.total_physical_overflow_nm = std::numeric_limits<std::uint64_t>::max();
  require(flux::routing_net_pressure_more_severe(
              broad, narrow, GlobalRouteCheckpointObjective::dimensioned_physical_overflow),
          "congestion breadth must prevent magnitude-driven scheduling starvation");

  broad.legacy_congested_resources = narrow.legacy_congested_resources;
  require(flux::routing_net_pressure_more_severe(
              narrow, broad, GlobalRouteCheckpointObjective::dimensioned_physical_overflow),
          "physical excess must resolve equal-breadth pressure without mixing units");

  Board pressure_board;
  Footprint duplicate;
  duplicate.id = ObjectId{"z-duplicate-net-pads"};
  duplicate.pads = {{ObjectId{"duplicate-1"}, first, Point{}, {LayerId{"F.Cu"}}},
                    {ObjectId{"duplicate-2"}, first, Point{}, {LayerId{"F.Cu"}}},
                    {ObjectId{"duplicate-3"}, first, Point{}, {LayerId{"F.Cu"}}}};
  Footprint single;
  single.id = ObjectId{"a-single-net-pad"};
  single.pads = {{ObjectId{"single-1"}, first, Point{}, {LayerId{"F.Cu"}}}};
  pressure_board.footprints = {std::move(duplicate), std::move(single)};
  const auto footprint_pressure = flux::collect_integrated_footprint_pressure(
      pressure_board, route, GlobalRouteCheckpointObjective::dimensioned_physical_overflow);
  require(footprint_pressure.size() == 2U &&
              footprint_pressure[0].footprint == ObjectId{"a-single-net-pad"} &&
              footprint_pressure[0].distinct_connected_nets == 1U &&
              footprint_pressure[1].distinct_connected_nets == 1U &&
              footprint_pressure[0].legacy_pressure == footprint_pressure[1].legacy_pressure &&
              footprint_pressure[0].physical_pressure_nm ==
                  footprint_pressure[1].physical_pressure_nm &&
              footprint_pressure[0].sampled_pressure == footprint_pressure[1].sampled_pressure &&
              footprint_pressure[0].corridor_pressure == footprint_pressure[1].corridor_pressure,
          "a congested net must contribute once per footprint regardless of repeated pads");

  route.physical_obstacle_impacts.push_back(
      flux::RoutingPhysicalObstacleImpact{3U,
                                          ObjectId{"duplicate-1"},
                                          LayerId{"F.Cu"},
                                          mm(1.0, 2.0),
                                          mm(3.0, 2.0),
                                          300U,
                                          {first, second}});
  const auto feedback = flux::collect_integrated_routing_feedback(route);
  const auto blocker_pressure = flux::collect_integrated_footprint_pressure(
      pressure_board, route, GlobalRouteCheckpointObjective::dimensioned_physical_overflow);
  require(feedback.physical_channel_obstacles == route.physical_obstacle_impacts &&
              std::find(feedback.routing_obstacles.begin(), feedback.routing_obstacles.end(),
                        ObjectId{"duplicate-1"}) != feedback.routing_obstacles.end() &&
              blocker_pressure.front().footprint == ObjectId{"z-duplicate-net-pads"} &&
              blocker_pressure.front().bounds_congested_channel &&
              blocker_pressure.front().blocks_route,
          "a source-bound channel limiter must propagate into placement feedback without losing "
          "its physical units");
}

void physical_checkpoint_ordering_is_feasibility_first_and_fail_closed() {
  using Objective = GlobalRouteCheckpointObjective;
  flux::GlobalRouteResult lhs;
  flux::GlobalRouteResult rhs;
  rhs.unrouted_nets = {NetId{"unrouted"}};
  lhs.total_voronoi_overflow_nm = std::numeric_limits<std::uint64_t>::max();
  require(flux::global_route_checkpoint_better(lhs, rhs, Objective::dimensioned_physical_overflow),
          "connectivity must dominate physical overflow");

  lhs = {};
  rhs = {};
  lhs.total_voronoi_overflow_nm = 100U;
  rhs.total_voronoi_overflow_nm = 200U;
  lhs.sampled_overflow = std::numeric_limits<std::size_t>::max();
  require(flux::global_route_checkpoint_better(lhs, rhs, Objective::dimensioned_physical_overflow),
          "exact physical excess must precede sampled overflow without adding their units");

  flux::IntegratedScore closer_to_strict_feasibility;
  flux::IntegratedScore softer_but_farther_from_strict_feasibility;
  closer_to_strict_feasibility.configured_dimensioned_physical_overflow = true;
  softer_but_farther_from_strict_feasibility.configured_dimensioned_physical_overflow = true;
  closer_to_strict_feasibility.best_hard_capacity_unrouted_nets = 5U;
  closer_to_strict_feasibility.total_physical_overflow_nm =
      std::numeric_limits<std::uint64_t>::max();
  softer_but_farther_from_strict_feasibility.best_hard_capacity_unrouted_nets = 6U;
  require(closer_to_strict_feasibility < softer_but_farther_from_strict_feasibility,
          "integrated placement must reduce the best constructive hard-capacity deficit before "
          "optimizing a selected soft checkpoint");

  auto board = two_pad_board();
  GlobalRouterConfig invalid;
  invalid.grid_pitch = Nanometres::from_mm(5.0);
  invalid.checkpoint_objective = Objective::dimensioned_physical_overflow;
  const auto rejected = flux::route_global(board, invalid);
  require(rejected.stop_reason == GlobalRouterStopReason::invalid_input &&
              rejected.diagnostic.find("requires shared Voronoi allocation") != std::string::npos,
          "physical checkpoint selection must fail closed without physical resources");

  invalid = {};
  invalid.grid_pitch = Nanometres::from_mm(5.0);
  invalid.max_voronoi_resource_segment_length = Nanometres::from_mm(5.0);
  const auto unbacked_segmentation = flux::route_global(board, invalid);
  require(unbacked_segmentation.stop_reason == GlobalRouterStopReason::invalid_input &&
              unbacked_segmentation.diagnostic.find("requires shared Voronoi allocation") !=
                  std::string::npos,
          "longitudinal capacity segmentation must fail closed without physical resources");

  invalid.use_voronoi_allocation_resources = true;
  invalid.max_voronoi_allocation_resources = 1U;
  const auto over_budget_segmentation = flux::route_global(board, invalid);
  require(over_budget_segmentation.stop_reason == GlobalRouterStopReason::invalid_input &&
              over_budget_segmentation.diagnostic.find("resource budget") != std::string::npos,
          "longitudinal capacity segmentation must fail closed at the physical resource budget");

  invalid = {};
  invalid.grid_pitch = Nanometres::from_mm(5.0);
  invalid.via_site_pitch = Nanometres::from_mm(7.5);
  const auto invalid_via_lattice = flux::route_global(board, invalid);
  require(invalid_via_lattice.stop_reason == GlobalRouterStopReason::invalid_input &&
              invalid_via_lattice.diagnostic.find("exact positive multiple") != std::string::npos,
          "a decoupled via lattice must remain an exact bounded sublattice of the planar grid");

  invalid = {};
  invalid.grid_pitch = Nanometres::from_mm(5.0);
  invalid.local_pin_escape_pitch = Nanometres::from_mm(3.0);
  const auto invalid_local_mesh = flux::route_global(board, invalid);
  require(invalid_local_mesh.stop_reason == GlobalRouterStopReason::invalid_input &&
              invalid_local_mesh.diagnostic.find("exact divisor") != std::string::npos,
          "a local escape mesh must remain exactly registered to its global portals");

  invalid = {};
  invalid.grid_pitch = Nanometres::from_mm(5.0);
  invalid.atomic_complete_net_routing = true;
  invalid.selective_full_reroute_interval = 2U;
  const auto unbacked_full_refresh = flux::route_global(board, invalid);
  require(unbacked_full_refresh.stop_reason == GlobalRouterStopReason::invalid_input &&
              unbacked_full_refresh.diagnostic.find("requires selective congestion rerouting") !=
                  std::string::npos,
          "periodic full rerouting must fail closed without selective whole-net reuse");

  invalid = {};
  invalid.grid_pitch = Nanometres::from_mm(5.0);
  invalid.enable_search_diversification = true;
  const auto unbacked_search_diversification = flux::route_global(board, invalid);
  require(unbacked_search_diversification.stop_reason == GlobalRouterStopReason::invalid_input &&
              unbacked_search_diversification.diagnostic.find("atomic complete-net routing") !=
                  std::string::npos,
          "large-neighborhood search must fail closed unless every destroyed coupled group is "
          "rerouted atomically");

  flux::RoutabilityDiagnosis physical_failure;
  physical_failure.kind = flux::RoutabilityFailureKind::packing_capacity_exceeded;
  physical_failure.stage = flux::RoutabilityFailureStage::global_probe;
  physical_failure.resource = flux::RoutabilityResourceReference{
      "fixture-channel", LayerId{"F.Cu"}, {mm(40.0, 35.0), mm(60.0, 45.0)}, {NetId{"net-1"}}};
  physical_failure.severity = 3U;
  flux::RoutabilityKnowledge knowledge;
  flux::accumulate_routability_knowledge(knowledge, {physical_failure});
  auto repeated_failure = physical_failure;
  repeated_failure.severity = 2U;
  flux::accumulate_routability_knowledge(knowledge, {repeated_failure});
  require(knowledge.history.size() == 1U && knowledge.history.front().observations == 2U &&
              knowledge.history.front().pressure == 5U,
          "repeated physical failures must accumulate deterministically as bounded cost evidence");

  GlobalRouterConfig feedback_config;
  feedback_config.grid_pitch = Nanometres::from_mm(5.0);
  feedback_config.max_iterations = 1U;
  feedback_config.routability_history = knowledge.history;
  const auto feedback_route = flux::route_global(board, feedback_config);
  require(feedback_route.routability_history_entries == 1U &&
              feedback_route.routability_history_edges > 0U &&
              feedback_route.routability_history_pressure == 5U,
          "global routing must prepare physical history into net-scoped non-authoritative edge "
          "costs before static and negotiated search");

  flux::RegionalRouteResult regional_feedback;
  auto locally_resolved = physical_failure;
  locally_resolved.resource.stable_id = "resolved-window";
  locally_resolved.locally_resolved = true;
  auto unresolved = physical_failure;
  unresolved.resource.stable_id = "unresolved-window";
  regional_feedback.local_routability_diagnoses = {locally_resolved, unresolved};
  const auto escalated = flux::diagnose_regional_routability(regional_feedback);
  require(escalated.size() == 1U && !escalated.front().locally_resolved,
          "a locally discharged spline/PNS diagnosis must not become upstream PathFinder history");

  flux::DetailedRouteResult rejected_detail;
  rejected_detail.candidate = board;
  rejected_detail.stop_reason = flux::DetailedRouterStopReason::connectivity_incomplete;
  rejected_detail.connectivity.nets = {flux::NetConnectivity{NetId{"net-1"}, 2U, 2U, 1U}};
  rejected_detail.connectivity.open_connections = 1U;
  const auto detailed_diagnoses = flux::diagnose_detailed_routability(rejected_detail);
  require(detailed_diagnoses.size() == 1U &&
              detailed_diagnoses.front().kind == flux::RoutabilityFailureKind::topological_trap &&
              detailed_diagnoses.front().stage == flux::RoutabilityFailureStage::final_validation &&
              detailed_diagnoses.front().resource.nets == std::vector<NetId>{NetId{"net-1"}},
          "final connectivity rejection must return an attributed topological diagnosis");
}

void global_to_detailed_route_is_transactional_and_capacity_gated() {
  const auto source = two_pad_board();
  GlobalRouterConfig diversified_config;
  diversified_config.grid_pitch = Nanometres::from_mm(5.0);
  diversified_config.max_iterations = 1U;
  diversified_config.max_pin_escape_grid_radius = 2U;
  diversified_config.max_pin_access_nodes_per_terminal = 32U;
  diversified_config.diversify_pin_access_rings = true;
  diversified_config.hard_capacity_start_iteration = 1U;
  diversified_config.single_lane_direct_voronoi_resources = true;
  diversified_config.priority_nets = {NetId{"net-1"}};
  const auto diversified_route = flux::route_global(source, diversified_config);
  require(diversified_route.configured_diversify_pin_access_rings &&
              diversified_route.configured_single_lane_direct_voronoi_resources &&
              diversified_route.configured_priority_nets == diversified_config.priority_nets &&
              diversified_route.configured_residual_corridor_demand_cost ==
                  diversified_config.residual_corridor_demand_cost &&
              diversified_route.configured_critical_cut_cost ==
                  diversified_config.critical_cut_cost &&
              diversified_route.configured_critical_cut_minimum_future_demand ==
                  diversified_config.critical_cut_minimum_future_demand &&
              diversified_route.residual_corridor_demand_nets == 1U &&
              diversified_route.residual_corridor_demand_resources > 0U &&
              diversified_route.diversified_pin_access_nodes > 0U &&
              diversified_route.terminals_with_diversified_pin_access > 0U &&
              diversified_route.additional_successful_pin_access_rings > 0U &&
              diversified_route.failed_pin_escapes == 0U,
          "bounded later-ring portals must remain exact-legal and observable");
  auto branched_source = source;
  auto third_terminal = source.footprints.front();
  third_terminal.id = ObjectId{"U3-branch"};
  third_terminal.reference = "U3-branch";
  third_terminal.position = mm(20.0, 70.0);
  third_terminal.pads.front().id = ObjectId{"U3-branch-pad"};
  branched_source.footprints.push_back(std::move(third_terminal));
  const auto branched_route = flux::route_global(branched_source, diversified_config);
  flux::DetailedRouterConfig branched_detail_config;
  branched_detail_config.pull_tight_45_degrees = false;
  branched_detail_config.continuous_refinement = false;
  const auto branched_detail =
      flux::materialize_detailed_route(branched_source, branched_route, branched_detail_config);
  require(branched_route.complete() && branched_route.static_seed_complete_trees >= 1U &&
              branched_route.static_seed_tree_joins >= 1U &&
              branched_route.iteration_zero_paths == 2U &&
              branched_detail.connectivity.open_connections == 0U,
          "later branches must attach only to the first terminal portal that acquired physical "
          "access copper: " +
              std::to_string(branched_route.paths.size()) + "/" +
              std::to_string(branched_detail.connectivity.open_connections) + "/" +
              branched_detail.diagnostic);
  auto paired_source = source;
  paired_source.nets.push_back(Net{NetId{"net-2"}, "SIGNAL_N", 0});
  paired_source.footprints[0].position = mm(20.0, 38.0);
  paired_source.footprints[1].position = mm(80.0, 38.0);
  paired_source.footprints[0].courtyard = local_rectangle(0.1, 0.1);
  paired_source.footprints[1].courtyard = local_rectangle(0.1, 0.1);
  auto negative_start = paired_source.footprints[0];
  negative_start.id = ObjectId{"U1N"};
  negative_start.reference = "U1N";
  negative_start.position = mm(20.0, 42.0);
  negative_start.pads[0].id = ObjectId{"U1N-pad"};
  negative_start.pads[0].net = NetId{"net-2"};
  auto negative_end = paired_source.footprints[1];
  negative_end.id = ObjectId{"U2N"};
  negative_end.reference = "U2N";
  negative_end.position = mm(80.0, 42.0);
  negative_end.pads[0].id = ObjectId{"U2N-pad"};
  negative_end.pads[0].net = NetId{"net-2"};
  paired_source.footprints.push_back(std::move(negative_start));
  paired_source.footprints.push_back(std::move(negative_end));
  paired_source.differential_pairs.push_back({flux::DifferentialPairId{"pair-1"}, "PAIR",
                                              NetId{"net-1"}, NetId{"net-2"},
                                              Nanometres::from_mm(0.2), std::nullopt});
  auto paired_config = diversified_config;
  paired_config.use_voronoi_allocation_resources = true;
  auto serial_paired_config = paired_config;
  serial_paired_config.max_static_seed_workers = 1U;
  const auto serial_paired_route = flux::route_global(paired_source, serial_paired_config);
  paired_config.max_static_seed_workers = 2U;
  const auto paired_route = flux::route_global(paired_source, paired_config);
  require(paired_route.complete() && paired_route.static_seed_joint_groups == 1U &&
              paired_route.static_seed_joint_group_candidates > 0U &&
              paired_route.static_seed_joint_groups_withheld == 0U &&
              serial_paired_route.paths == paired_route.paths &&
              serial_paired_route.static_seed_searches == paired_route.static_seed_searches &&
              serial_paired_route.static_seed_search_expansions ==
                  paired_route.static_seed_search_expansions &&
              paired_route.static_seed_tasks_dispatched == 2U &&
              serial_paired_route.static_seed_workers_used == 1U &&
              paired_route.static_seed_workers_used == 2U &&
              serial_paired_route.iteration_zero_seed_priority_evidence ==
                  paired_route.iteration_zero_seed_priority_evidence &&
              paired_route.iteration_zero_seed_priority_evidence.size() == 1U &&
              paired_route.iteration_zero_seed_priority_evidence.front().nets ==
                  std::vector<NetId>{NetId{"net-1"}, NetId{"net-2"}} &&
              paired_route.iteration_zero_staged_priority_qualified &&
              paired_route.iteration_zero_baseline_unrouted_nets ==
                  paired_route.iteration_zero_staged_unrouted_nets &&
              paired_route.iteration_zero_staged_clearance_shadow_conflicts <=
                  paired_route.iteration_zero_baseline_clearance_shadow_conflicts &&
              paired_route.configured_terminal_escape_reservation_cost > 0U &&
              paired_route.terminal_escape_reservation_terminals > 0U &&
              paired_route.terminal_escape_reservation_candidates > 0U &&
              paired_route.terminal_escape_reservation_resources > 0U &&
              paired_route.iteration_zero_trivial_groups +
                      paired_route.iteration_zero_constrained_groups +
                      paired_route.iteration_zero_ordinary_groups +
                      paired_route.iteration_zero_flexible_groups ==
                  1U,
          "declared differential members must enter iteration-zero planning only through a "
          "complete atomic priority-wave/layer-sequence candidate, independent of worker "
          "scheduling: " +
              std::to_string(paired_route.complete()) + "/" +
              std::to_string(paired_route.static_seed_joint_groups) + "/" +
              std::to_string(paired_route.static_seed_joint_group_candidates) + "/" +
              std::to_string(paired_route.static_seed_joint_groups_withheld) + "/" +
              paired_route.diagnostic);
  auto bus_source = paired_source;
  bus_source.differential_pairs.clear();
  bus_source.routing_buses.push_back({flux::RoutingBusId{"bus-1"},
                                      "BUS",
                                      {NetId{"net-1"}, NetId{"net-2"}},
                                      Nanometres::from_mm(0.2)});
  const auto bus_route = flux::route_global(bus_source, paired_config);
  const auto bus_detail =
      flux::materialize_detailed_route(bus_source, bus_route, branched_detail_config);
  require(bus_route.complete() && bus_route.static_seed_joint_groups == 1U &&
              bus_route.static_seed_joint_group_candidates > 0U &&
              bus_route.static_seed_joint_groups_withheld == 0U && bus_detail.complete() &&
              bus_detail.candidate && bus_detail.connectivity.open_connections == 0U &&
              bus_detail.copper_drc.ok(),
          "declared bus members must retain one atomic ordered seed candidate through exact "
          "detailed acceptance: " +
              std::to_string(bus_route.static_seed_joint_group_candidates) + "/" +
              std::to_string(bus_detail.connectivity.open_connections) + "/" +
              std::to_string(bus_detail.copper_drc.violations.size()) + "/" +
              bus_detail.diagnostic);
  auto unassigned_pad_source = source;
  unassigned_pad_source.nets.front().name.clear();
  const auto unassigned_connectivity = flux::analyze_connectivity(unassigned_pad_source);
  require(unassigned_connectivity.open_connections == 0U &&
              unassigned_connectivity.nets.front().pad_count == 2U,
          "empty-name KiCad net-zero pads must remain inventoried but intentionally unconnected");
  auto physically_connected_source = source;
  physically_connected_source.tracks = {
      {ObjectId{"physical-track-a"}, NetId{"net-1"}, LayerId{"F.Cu"}, mm(20.5, 40.0),
       mm(49.8, 40.0), Nanometres::from_mm(0.4), false},
      {ObjectId{"physical-track-b"}, NetId{"net-1"}, LayerId{"F.Cu"}, mm(50.2, 40.0),
       mm(79.5, 40.0), Nanometres::from_mm(0.4), false}};
  const auto physical_connectivity = flux::analyze_connectivity(physically_connected_source);
  const auto preserved_route = flux::route_global(physically_connected_source, diversified_config);
  require(physical_connectivity.open_connections == 0U && preserved_route.complete() &&
              preserved_route.paths.empty(),
          "touching physical copper must connect pads and suppress redundant source-net rerouting");
  auto scoped_config = diversified_config;
  scoped_config.selected_nets = {NetId{"net-1"}};
  scoped_config.allow_diagonal_grid_edges = true;
  const auto explicitly_rerouted = flux::route_global(physically_connected_source, scoped_config);
  require(explicitly_rerouted.complete() && !explicitly_rerouted.paths.empty() &&
              explicitly_rerouted.configured_selected_nets == scoped_config.selected_nets &&
              explicitly_rerouted.configured_diagonal_grid_edges,
          "an exact selected-net scope must be observable and may deliberately replace a "
          "source-complete net while preserving all unselected copper");
  scoped_config.selected_nets = {NetId{"missing"}};
  const auto invalid_scope = flux::route_global(physically_connected_source, scoped_config);
  require(invalid_scope.stop_reason == GlobalRouterStopReason::invalid_input &&
              invalid_scope.diagnostic.find("unique IDs") != std::string::npos,
          "a selected-net scope must fail closed on a non-source net ID");
  GlobalRouterConfig first_ring_config = diversified_config;
  first_ring_config.diversify_pin_access_rings = false;
  const auto policy_changed_route =
      flux::route_global_incremental(source, diversified_route, {}, first_ring_config);
  require(policy_changed_route.reused_nets.empty() && policy_changed_route.reused_paths == 0U &&
              !policy_changed_route.rerouted_nets.empty(),
          "incremental routing must invalidate later-ring paths when access policy changes");

  auto trapped_source = source;
  trapped_source.nets.push_back(Net{NetId{"blocker-net"}, "BLOCKER", 0});
  Footprint blocker;
  blocker.id = ObjectId{"U3"};
  blocker.reference = "U3";
  blocker.position = mm(50.0, 70.0);
  blocker.courtyard = local_rectangle(4.0, 3.0);
  flux::Pad blocker_pad;
  blocker_pad.id = ObjectId{"U3-pad"};
  blocker_pad.net = NetId{"blocker-net"};
  blocker_pad.local_position = mm(-30.0, -30.0);
  blocker_pad.layers = {LayerId{"F.Cu"}};
  blocker_pad.shape = flux::PadShape::rectangle;
  blocker_pad.width = Nanometres::from_mm(36.0);
  blocker_pad.height = Nanometres::from_mm(36.0);
  blocker.pads.push_back(std::move(blocker_pad));
  trapped_source.footprints.push_back(std::move(blocker));
  GlobalRouterConfig trapped_config;
  trapped_config.grid_pitch = Nanometres::from_mm(5.0);
  trapped_config.max_iterations = 1U;
  CollectingTelemetrySink trapped_telemetry;
  const auto trapped_route =
      flux::route_global(trapped_source, trapped_config, {}, &trapped_telemetry);
  require(std::any_of(trapped_route.pin_escape_failures.begin(),
                      trapped_route.pin_escape_failures.end(),
                      [](const auto& failure) {
                        return failure.footprint == ObjectId{"U1"} &&
                               failure.pad == ObjectId{"U1-pad"} && failure.net == NetId{"net-1"} &&
                               failure.position == mm(20.0, 40.0);
                      }),
          "failed pin escape evidence must retain stable component, pad, net, and position IDs");
  require(std::any_of(trapped_telemetry.events.begin(), trapped_telemetry.events.end(),
                      [](const auto& event) {
                        return event.kind == flux::RouterTelemetryEventKind::net_failed &&
                               event.net == NetId{"net-1"} &&
                               event.rejection_reason == "no_access_nodes" &&
                               event.x_nm == mm(20.0, 40.0).x.value() &&
                               event.y_nm == mm(20.0, 40.0).y.value() &&
                               event.detail.find("footprint=U1") != std::string::npos &&
                               event.detail.find("pad=U1-pad") != std::string::npos;
                      }),
          "failed pin escapes must publish a terminal-attributed NetFailed event for the live "
          "viewer");
  const auto trapped_feedback = flux::collect_integrated_routing_feedback(trapped_route);
  require(std::find(trapped_feedback.routing_obstacles.begin(),
                    trapped_feedback.routing_obstacles.end(),
                    ObjectId{"U1"}) != trapped_feedback.routing_obstacles.end() &&
              std::find(trapped_feedback.affected_nets.begin(),
                        trapped_feedback.affected_nets.end(),
                        NetId{"net-1"}) != trapped_feedback.affected_nets.end(),
          "failed escapes and unrouted nets must become integrated placement feedback");

  auto neck_down_source = source;
  neck_down_source.nets.push_back(Net{NetId{"neck-down-blocker-net"}, "BLOCKER", 0});
  neck_down_source.footprints.front().pads.front().local_position = mm(0.0, 0.3);
  neck_down_source.footprints.front().pads.front().width = Nanometres::from_mm(0.1);
  neck_down_source.footprints.front().pads.front().height = Nanometres::from_mm(0.1);
  Footprint neck_down_blocker;
  neck_down_blocker.id = ObjectId{"neck-down-blocker"};
  neck_down_blocker.reference = "U3";
  neck_down_blocker.position = mm(50.0, 70.0);
  neck_down_blocker.courtyard = local_rectangle(1.0, 1.0);
  flux::Pad neck_down_blocker_pad;
  neck_down_blocker_pad.id = ObjectId{"neck-down-blocker-pad"};
  neck_down_blocker_pad.net = NetId{"neck-down-blocker-net"};
  neck_down_blocker_pad.local_position = mm(-29.65, -29.85);
  neck_down_blocker_pad.layers = {LayerId{"F.Cu"}};
  neck_down_blocker_pad.width = Nanometres::from_mm(0.1);
  neck_down_blocker_pad.height = Nanometres::from_mm(0.1);
  neck_down_blocker.pads.push_back(std::move(neck_down_blocker_pad));
  neck_down_source.footprints.push_back(std::move(neck_down_blocker));
  GlobalRouterConfig neck_down_config;
  neck_down_config.grid_pitch = Nanometres::from_mm(5.0);
  neck_down_config.max_iterations = 1U;
  neck_down_config.terminal_neck_down_width = Nanometres::from_mm(0.1);
  neck_down_config.max_terminal_neck_down_length = Nanometres::from_mm(1.0);
  const auto neck_down_route = flux::route_global(neck_down_source, neck_down_config);
  const auto selected_blocker = std::find_if(
      neck_down_route.neck_down_obstacle_impacts.begin(),
      neck_down_route.neck_down_obstacle_impacts.end(), [](const auto& impact) {
        return impact.routed_net == NetId{"net-1"} &&
               impact.obstacle == ObjectId{"neck-down-blocker-pad"} &&
               impact.width_kind == flux::GlobalRouteSegmentWidthKind::terminal_neck_down &&
               impact.selected_width == Nanometres::from_mm(0.1);
      });
  const auto neck_down_feedback = flux::collect_integrated_routing_feedback(neck_down_route);
  require(
      neck_down_route.used_terminal_neck_down_segments > 0U &&
          selected_blocker != neck_down_route.neck_down_obstacle_impacts.end() &&
          neck_down_feedback.neck_down_obstacles == neck_down_route.neck_down_obstacle_impacts &&
          std::find(neck_down_feedback.routing_obstacles.begin(),
                    neck_down_feedback.routing_obstacles.end(),
                    ObjectId{"neck-down-blocker-pad"}) !=
              neck_down_feedback.routing_obstacles.end(),
      "a selected narrow segment must attribute its exact nominal-width blocker to placement: " +
          std::to_string(neck_down_route.used_terminal_neck_down_segments) + "/" +
          std::to_string(neck_down_route.terminal_neck_down_access_nodes) + "/" +
          std::to_string(neck_down_route.failed_pin_escapes) + "/" +
          std::to_string(neck_down_route.neck_down_obstacle_impacts.size()) + "/" +
          std::to_string(neck_down_feedback.neck_down_obstacles.size()) + "/" +
          neck_down_route.diagnostic);

  auto disconnected_source = source;
  Keepout wall;
  wall.id = ObjectId{"routing-wall"};
  wall.area = rectangle(45.0, 0.0, 55.0, 80.0);
  wall.layers = {LayerId{"F.Cu"}, LayerId{"B.Cu"}};
  wall.prohibit.tracks = true;
  wall.prohibit.vias = true;
  disconnected_source.keepouts.push_back(std::move(wall));
  GlobalRouterConfig disconnected_config;
  disconnected_config.grid_pitch = Nanometres::from_mm(5.0);
  disconnected_config.max_iterations = 1U;
  disconnected_config.max_expansions_per_connection = 10'000U;
  const auto disconnected_route = flux::route_global(disconnected_source, disconnected_config);
  const auto disconnected_failure = std::find_if(
      disconnected_route.connection_failures.begin(), disconnected_route.connection_failures.end(),
      [](const auto& failure) {
        return failure.net == NetId{"net-1"} &&
               failure.reason == flux::RoutingConnectionFailureReason::search_frontier_exhausted;
      });
  require(disconnected_failure != disconnected_route.connection_failures.end() &&
              disconnected_failure->attempted_terminals.size() == 1U &&
              disconnected_failure->attempted_terminals.front().footprint == ObjectId{"U2"} &&
              disconnected_failure->connected_terminals.size() == 1U &&
              disconnected_failure->connected_terminals.front().footprint == ObjectId{"U1"} &&
              disconnected_failure->expansions > 0U,
          "a disconnected search must retain its attempted terminal, tree root, and exact reason");
  const auto disconnected_feedback = flux::collect_integrated_routing_feedback(disconnected_route);
  require(std::find(disconnected_feedback.routing_obstacles.begin(),
                    disconnected_feedback.routing_obstacles.end(),
                    ObjectId{"U2"}) != disconnected_feedback.routing_obstacles.end(),
          "the failed connection endpoint must become a first-tier placement target");

  auto via_scarce_source = source;
  Keepout front_wall;
  front_wall.id = ObjectId{"front-layer-routing-wall"};
  front_wall.area = rectangle(45.0, 0.0, 55.0, 80.0);
  front_wall.layers = {LayerId{"F.Cu"}};
  front_wall.prohibit.tracks = true;
  via_scarce_source.keepouts.push_back(std::move(front_wall));
  GlobalRouterConfig via_scarce_config;
  via_scarce_config.grid_pitch = Nanometres::from_mm(5.0);
  via_scarce_config.via_site_pitch = Nanometres::from_mm(50.0);
  via_scarce_config.via_site_phase_x = 8U;
  via_scarce_config.via_site_phase_y = 8U;
  via_scarce_config.max_iterations = 1U;
  const auto via_scarce_route = flux::route_global(via_scarce_source, via_scarce_config);
  require(
      via_scarce_route.complete() &&
          via_scarce_route.iteration_zero_seed_priority_evidence.size() == 1U &&
          via_scarce_route.iteration_zero_seed_priority_evidence.front().primary_via_transitions >
              0U &&
          via_scarce_route.iteration_zero_seed_priority_evidence.front().viable_via_sites > 0U &&
          via_scarce_route.iteration_zero_seed_priority_evidence.front().wave ==
              flux::IterationZeroRoutingWave::constrained,
      "a route with a mandatory scarce layer transition must expose its via-site portfolio "
      "and enter the constrained iteration-zero wave: " +
          std::to_string(via_scarce_route.complete()) + "/" +
          std::to_string(via_scarce_route.iteration_zero_seed_priority_evidence.empty()
                             ? 0U
                             : via_scarce_route.iteration_zero_seed_priority_evidence.front()
                                   .primary_via_transitions) +
          "/" + via_scarce_route.diagnostic);

  auto crossing_source = two_pad_board();
  const NetId crossing_net{"crossing-net"};
  crossing_source.nets.push_back({crossing_net, "CROSSING", 0});
  const auto crossing_footprint = [&crossing_net](std::string id, Point position) {
    Footprint result;
    result.id = ObjectId{id};
    result.reference = id;
    result.position = position;
    result.courtyard = local_rectangle(4.0, 3.0);
    result.pads = {{ObjectId{id + "-pad"}, crossing_net, Point{}, {LayerId{"F.Cu"}}}};
    return result;
  };
  crossing_source.footprints.push_back(crossing_footprint("U3", mm(50.0, 10.0)));
  crossing_source.footprints.push_back(crossing_footprint("U4", mm(50.0, 70.0)));
  GlobalRouterConfig crossing_config;
  crossing_config.grid_pitch = Nanometres::from_mm(5.0);
  crossing_config.max_iterations = 1U;
  crossing_config.max_static_seed_workers = 1U;
  const auto crossing_route = flux::route_global(crossing_source, crossing_config);
  require(
      crossing_route.iteration_zero_seed_priority_evidence.size() == 2U &&
          std::all_of(crossing_route.iteration_zero_seed_priority_evidence.begin(),
                      crossing_route.iteration_zero_seed_priority_evidence.end(),
                      [](const auto& evidence) {
                        return evidence.terminal_crossing_pressure == 1U &&
                               evidence.wave != flux::IterationZeroRoutingWave::trivial;
                      }),
      "crossing terminal MST airwires must be observable and suppress a false trivial seed wave");

  auto refresh_source = source;
  refresh_source.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0}};
  const NetId competing_net{"competing-net"};
  refresh_source.nets.push_back(Net{competing_net, "COMPETING", 0});
  for (std::size_t index = 0U; index < refresh_source.footprints.size(); ++index) {
    refresh_source.footprints[index].courtyard = local_rectangle(4.0, 8.0);
    refresh_source.footprints[index].pads.push_back(
        flux::Pad{ObjectId{"competing-pad-" + std::to_string(index)},
                  competing_net,
                  mm(0.0, 1.0),
                  {LayerId{"F.Cu"}}});
  }
  Keepout upper_channel_wall;
  upper_channel_wall.id = ObjectId{"upper-channel-wall"};
  upper_channel_wall.area = rectangle(0.0, 0.0, 100.0, 35.0);
  upper_channel_wall.layers = {LayerId{"F.Cu"}};
  upper_channel_wall.prohibit.tracks = true;
  upper_channel_wall.prohibit.vias = true;
  auto lower_channel_wall = upper_channel_wall;
  lower_channel_wall.id = ObjectId{"lower-channel-wall"};
  lower_channel_wall.area = rectangle(0.0, 45.0, 100.0, 80.0);
  refresh_source.keepouts = {std::move(upper_channel_wall), std::move(lower_channel_wall)};
  GlobalRouterConfig refresh_config;
  refresh_config.grid_pitch = Nanometres::from_mm(5.0);
  refresh_config.max_iterations = 3U;
  refresh_config.atomic_complete_net_routing = true;
  refresh_config.selective_congestion_reroute = true;
  refresh_config.selective_full_reroute_interval = 1U;
  const auto refreshed_route = flux::route_global(refresh_source, refresh_config);
  require(refreshed_route.configured_selective_full_reroute_interval == 1U &&
              refreshed_route.selective_reroute_passes == 2U &&
              refreshed_route.selective_full_reroute_refreshes == 2U &&
              !refreshed_route.selected_net_pressure.empty(),
          "periodic refresh must discard active reuse on schedule and publish selected pressure: " +
              std::to_string(refreshed_route.configured_selective_full_reroute_interval) + "/" +
              std::to_string(refreshed_route.selective_reroute_passes) + "/" +
              std::to_string(refreshed_route.selective_full_reroute_refreshes) + "/" +
              std::to_string(refreshed_route.selected_net_pressure.size()) + "/" +
              refreshed_route.diagnostic);

  Board hard_capacity_source;
  hard_capacity_source.source_revision = "hard-capacity-risk-fixture";
  hard_capacity_source.outline = rectangle(0.0, 0.0, 100.0, 80.0);
  hard_capacity_source.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0}};
  hard_capacity_source.nets = {{NetId{"hard-a"}, "HARD_A", 0}, {NetId{"hard-b"}, "HARD_B", 0}};
  hard_capacity_source.rules.default_track_width = Nanometres::from_mm(0.35);
  const auto hard_terminal = [](std::string id, const NetId& net, Point position) {
    Footprint footprint;
    footprint.id = ObjectId{id};
    footprint.reference = id;
    footprint.position = position;
    footprint.courtyard = local_rectangle(0.1, 0.1);
    flux::Pad pad;
    pad.id = ObjectId{id + "-pad"};
    pad.net = net;
    pad.layers = {LayerId{"F.Cu"}};
    pad.shape = flux::PadShape::rectangle;
    pad.width = Nanometres::from_mm(0.1);
    pad.height = Nanometres::from_mm(0.1);
    footprint.pads.push_back(std::move(pad));
    return footprint;
  };
  hard_capacity_source.footprints = {hard_terminal("HA1", NetId{"hard-a"}, mm(20.0, 35.0)),
                                     hard_terminal("HA2", NetId{"hard-a"}, mm(80.0, 35.0)),
                                     hard_terminal("HB1", NetId{"hard-b"}, mm(20.0, 45.0)),
                                     hard_terminal("HB2", NetId{"hard-b"}, mm(80.0, 45.0))};
  Keepout hard_upper;
  hard_upper.id = ObjectId{"hard-upper"};
  hard_upper.area = rectangle(30.0, 0.0, 70.0, 39.3);
  hard_upper.layers = {LayerId{"F.Cu"}};
  hard_upper.prohibit.tracks = true;
  hard_upper.prohibit.vias = true;
  auto hard_lower = hard_upper;
  hard_lower.id = ObjectId{"hard-lower"};
  hard_lower.area = rectangle(30.0, 40.7, 70.0, 80.0);
  hard_capacity_source.keepouts = {std::move(hard_upper), std::move(hard_lower)};
  GlobalRouterConfig hard_capacity_config;
  hard_capacity_config.grid_pitch = Nanometres::from_mm(1.0);
  hard_capacity_config.max_iterations = 2U;
  hard_capacity_config.max_expansions_per_connection = 50'000U;
  hard_capacity_config.use_voronoi_allocation_resources = true;
  hard_capacity_config.atomic_complete_net_routing = true;
  hard_capacity_config.hard_capacity_start_iteration = 2U;
  hard_capacity_config.max_hard_capacity_ripup_attempts_per_pass = 2U;
  hard_capacity_config.max_hard_capacity_ripup_nets_per_attempt = 1U;
  hard_capacity_config.max_hard_capacity_conflict_set_states_per_attempt = 16U;
  hard_capacity_config.rollback_failed_hard_capacity_ripup = true;
  hard_capacity_config.defer_hard_capacity_ripup_victims_to_pass_tail = true;
  const auto hard_capacity_route = flux::route_global(hard_capacity_source, hard_capacity_config);
  require(
      hard_capacity_route.configured_hard_capacity_start_iteration == 2U &&
          hard_capacity_route.hard_capacity_passes == 1U &&
          hard_capacity_route.hard_sampled_capacity_rejected_transitions > 0U &&
          hard_capacity_route.hard_voronoi_capacity_rejected_transitions > 0U &&
          hard_capacity_route.hard_capacity_diversified_passes == 1U &&
          hard_capacity_route.hard_capacity_pass_summaries.size() == 1U &&
          hard_capacity_route.hard_capacity_pass_summaries.front().attempted_order.size() == 2U &&
          hard_capacity_route.hard_capacity_pass_summaries.front().unrouted_nets.size() == 1U &&
          hard_capacity_route.hard_capacity_net_failures.size() == 1U &&
          hard_capacity_route.hard_capacity_net_failures.front().failed_passes == 1U &&
          !hard_capacity_route.hard_capacity_blockers.empty() &&
          hard_capacity_route.hard_capacity_blockers.front().rejected_transitions > 0U &&
          hard_capacity_route.configured_max_hard_capacity_ripup_attempts_per_pass == 2U &&
          hard_capacity_route.configured_max_hard_capacity_ripup_nets_per_attempt == 1U &&
          hard_capacity_route.configured_max_hard_capacity_conflict_set_states_per_attempt == 16U &&
          hard_capacity_route.hard_capacity_conflict_set_states > 0U &&
          hard_capacity_route.hard_capacity_joint_ripup_attempts == 0U &&
          hard_capacity_route.configured_rollback_failed_hard_capacity_ripup &&
          hard_capacity_route.hard_capacity_failed_retry_rollbacks == 0U &&
          hard_capacity_route.configured_defer_hard_capacity_ripup_victims_to_pass_tail &&
          hard_capacity_route.hard_capacity_tail_deferred_victims == 2U &&
          hard_capacity_route.hard_capacity_ripup_attempts == 2U &&
          hard_capacity_route.hard_capacity_ripped_nets == 2U &&
          hard_capacity_route.hard_capacity_ripup_recoveries == 2U &&
          hard_capacity_route.hard_capacity_ripup_retained_recoveries == 1U &&
          hard_capacity_route.configured_former_route_memory_cost > 0U &&
          hard_capacity_route.former_route_memory_ripped_paths > 0U &&
          hard_capacity_route.former_route_memory_edges > 0U &&
          hard_capacity_route.hard_capacity_ripup_records.size() == 2U &&
          hard_capacity_route.unrouted_nets.empty() && hard_capacity_route.paths.size() == 2U &&
          hard_capacity_route.total_voronoi_overflow_nm > 0U,
      "a late hard-cap pass must reject aggregate shared-resource overflow while retaining "
      "the earlier checkpoint: " +
          std::to_string(hard_capacity_route.hard_capacity_passes) + "/" +
          std::to_string(hard_capacity_route.hard_sampled_capacity_rejected_transitions) + "/" +
          std::to_string(hard_capacity_route.hard_voronoi_capacity_rejected_transitions) + "/" +
          std::to_string(hard_capacity_route.hard_capacity_diversified_passes) + "/" +
          std::to_string(hard_capacity_route.hard_capacity_pass_summaries.size()) + "/" +
          std::to_string(hard_capacity_route.hard_capacity_net_failures.size()) + "/" +
          std::to_string(hard_capacity_route.hard_capacity_blockers.size()) + "/" +
          std::to_string(hard_capacity_route.hard_capacity_ripup_attempts) + "/" +
          std::to_string(hard_capacity_route.hard_capacity_ripped_nets) + "/" +
          std::to_string(hard_capacity_route.hard_capacity_ripup_recoveries) + "/" +
          std::to_string(hard_capacity_route.hard_capacity_ripup_retained_recoveries) + "/" +
          std::to_string(hard_capacity_route.hard_capacity_ripup_records.size()) + "/" +
          std::to_string(hard_capacity_route.configured_former_route_memory_cost) + "/" +
          std::to_string(hard_capacity_route.former_route_memory_ripped_paths) + "/" +
          std::to_string(hard_capacity_route.former_route_memory_edges) + "/" +
          std::to_string(hard_capacity_route.former_route_memory_penalty_events) + "/" +
          std::to_string(hard_capacity_route.former_route_memory_penalty) + "/" +
          std::to_string(hard_capacity_route.voronoi_resource_usage.size()) + "/" +
          std::to_string(hard_capacity_route.congested_voronoi_resources) + "/" +
          std::to_string(hard_capacity_route.total_voronoi_overflow_nm) + "/" +
          std::to_string(hard_capacity_route.total_overflow) + "/" +
          std::to_string(hard_capacity_route.unrouted_nets.size()) + "/" +
          std::to_string(hard_capacity_route.paths.size()) + "/" + hard_capacity_route.diagnostic);

  const auto& hard_summary = hard_capacity_route.hard_capacity_pass_summaries.front();
  require(!hard_summary.connection_failures.empty() &&
              !hard_summary.connection_failures.front().attempted_terminals.empty(),
          "the strict-capacity fixture must retain an exact attempted terminal for placement "
          "feedback");
  const auto& hard_terminal_evidence =
      hard_summary.connection_failures.front().attempted_terminals.front();
  const auto hard_feedback = flux::collect_integrated_routing_feedback(hard_capacity_route);
  const auto feedback_has = [](const auto& values, const auto& value) {
    return std::find(values.begin(), values.end(), value) != values.end();
  };
  const auto& hard_blocker = hard_capacity_route.hard_capacity_blockers.front();
  const auto hard_pressure = flux::collect_integrated_footprint_pressure(
      hard_capacity_source, hard_capacity_route,
      GlobalRouteCheckpointObjective::dimensioned_physical_overflow);
  const auto attempted_pressure =
      std::find_if(hard_pressure.begin(), hard_pressure.end(), [&](const auto& pressure) {
        return pressure.footprint == hard_terminal_evidence.footprint;
      });
  require(feedback_has(hard_feedback.routing_obstacles, hard_terminal_evidence.footprint) &&
              feedback_has(hard_feedback.routing_obstacles, hard_terminal_evidence.pad) &&
              feedback_has(hard_feedback.affected_nets, hard_blocker.failed_net) &&
              feedback_has(hard_feedback.affected_nets, hard_blocker.blocking_net) &&
              attempted_pressure != hard_pressure.end() && attempted_pressure->blocks_route &&
              attempted_pressure->sampled_pressure > 0U,
          "integrated placement must consume strict failed-net, blocker, and terminal evidence "
          "even when the selected soft checkpoint itself routed every net");

  auto delayed_hard_capacity_config = hard_capacity_config;
  delayed_hard_capacity_config.max_iterations = 7U;
  delayed_hard_capacity_config.hard_capacity_start_iteration = 7U;
  const auto delayed_hard_capacity_route =
      flux::route_global(hard_capacity_source, delayed_hard_capacity_config);
  require(delayed_hard_capacity_route.hard_capacity_passes == 1U &&
              delayed_hard_capacity_route.hard_capacity_pass_summaries.size() == 1U &&
              delayed_hard_capacity_route.hard_capacity_pass_summaries.front().iteration == 7U,
          "soft stagnation must not pre-empt an explicitly scheduled hard-capacity phase: " +
              std::to_string(delayed_hard_capacity_route.iterations) + "/" +
              std::to_string(delayed_hard_capacity_route.hard_capacity_passes) + "/" +
              std::to_string(delayed_hard_capacity_route.hard_capacity_pass_summaries.size()) +
              "/" + delayed_hard_capacity_route.diagnostic);

  auto protected_recovery_config = hard_capacity_config;
  protected_recovery_config.protect_recovered_hard_capacity_ripup_nets = true;
  const auto protected_recovery_route =
      flux::route_global(hard_capacity_source, protected_recovery_config);
  require(
      protected_recovery_route.configured_protect_recovered_hard_capacity_ripup_nets &&
          protected_recovery_route.hard_capacity_protected_recoveries == 1U &&
          protected_recovery_route.hard_capacity_protected_victim_rejections > 0U &&
          protected_recovery_route.hard_capacity_ripup_attempts == 1U &&
          protected_recovery_route.hard_capacity_ripup_recoveries == 1U &&
          protected_recovery_route.hard_capacity_ripup_retained_recoveries == 1U &&
          protected_recovery_route.hard_capacity_pass_summaries.size() == 1U &&
          protected_recovery_route.hard_capacity_pass_summaries.front().unrouted_nets.size() == 1U,
      "same-pass recovery protection must retain the first successful repair instead of "
      "displacing it for an impossible reciprocal retry: " +
          std::to_string(protected_recovery_route.hard_capacity_protected_recoveries) + "/" +
          std::to_string(protected_recovery_route.hard_capacity_protected_victim_rejections) + "/" +
          std::to_string(protected_recovery_route.hard_capacity_ripup_attempts) + "/" +
          std::to_string(protected_recovery_route.hard_capacity_ripup_recoveries) + "/" +
          std::to_string(protected_recovery_route.hard_capacity_ripup_retained_recoveries) + "/" +
          (protected_recovery_route.hard_capacity_pass_summaries.empty()
               ? "0"
               : std::to_string(protected_recovery_route.hard_capacity_pass_summaries.front()
                                    .unrouted_nets.size())) +
          "/" + protected_recovery_route.diagnostic);

  Board rollback_source;
  rollback_source.source_revision = "failed-ripup-rollback-risk-fixture";
  rollback_source.outline = rectangle(0.0, 0.0, 120.0, 40.0);
  rollback_source.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0}};
  rollback_source.nets = {{NetId{"gate-a"}, "GATE_A", 30},
                          {NetId{"gate-b"}, "GATE_B", 20},
                          {NetId{"gate-c"}, "GATE_C", 10}};
  rollback_source.rules.default_track_width = Nanometres::from_mm(0.35);
  rollback_source.footprints = {hard_terminal("GA1", NetId{"gate-a"}, mm(10.0, 10.0)),
                                hard_terminal("GA2", NetId{"gate-a"}, mm(50.0, 10.0)),
                                hard_terminal("GB1", NetId{"gate-b"}, mm(50.0, 30.0)),
                                hard_terminal("GB2", NetId{"gate-b"}, mm(110.0, 30.0)),
                                hard_terminal("GC1", NetId{"gate-c"}, mm(10.0, 20.0)),
                                hard_terminal("GC2", NetId{"gate-c"}, mm(110.0, 20.0))};
  const auto gate_wall = [](std::string id, Polygon area) {
    Keepout wall;
    wall.id = ObjectId{std::move(id)};
    wall.area = std::move(area);
    wall.layers = {LayerId{"F.Cu"}};
    wall.prohibit.tracks = true;
    wall.prohibit.vias = true;
    return wall;
  };
  rollback_source.keepouts = {gate_wall("gate-one-low", rectangle(30.0, 0.0, 31.0, 9.3)),
                              gate_wall("gate-one-high", rectangle(30.0, 10.7, 31.0, 40.0)),
                              gate_wall("gate-two-low", rectangle(70.0, 0.0, 71.0, 29.3)),
                              gate_wall("gate-two-high", rectangle(70.0, 30.7, 71.0, 40.0))};
  auto rollback_config = hard_capacity_config;
  rollback_config.max_hard_capacity_ripup_attempts_per_pass = 1U;
  rollback_config.reorder_nets_by_pathfinder_pressure = false;
  const auto rollback_route = flux::route_global(rollback_source, rollback_config);
  require(
      rollback_route.hard_capacity_ripup_attempts == 1U &&
          rollback_route.hard_capacity_ripup_recoveries == 0U &&
          rollback_route.hard_capacity_failed_retry_rollbacks == 1U &&
          rollback_route.hard_capacity_tail_deferred_victims == 1U &&
          rollback_route.hard_capacity_ripup_records.size() == 1U &&
          rollback_route.hard_capacity_ripup_records.front().rolled_back_after_failed_retry &&
          !rollback_route.hard_capacity_ripup_records.front().retry_completed &&
          rollback_route.hard_capacity_pass_summaries.size() == 1U &&
          rollback_route.hard_capacity_pass_summaries.front().unrouted_nets.size() == 1U &&
          rollback_route.unrouted_nets.empty() && rollback_route.paths.size() == 3U,
      "an unsuccessful one-victim retry must restore both successful gate allocations and retain "
      "only the failed net in strict evidence: " +
          std::to_string(rollback_route.hard_capacity_ripup_attempts) + "/" +
          std::to_string(rollback_route.hard_capacity_ripup_recoveries) + "/" +
          std::to_string(rollback_route.hard_capacity_failed_retry_rollbacks) + "/" +
          std::to_string(rollback_route.hard_capacity_pass_summaries.size()) + "/" +
          (rollback_route.hard_capacity_pass_summaries.empty()
               ? "0"
               : std::to_string(
                     rollback_route.hard_capacity_pass_summaries.front().unrouted_nets.size())) +
          "/" + std::to_string(rollback_route.unrouted_nets.size()) + "/" +
          std::to_string(rollback_route.paths.size()) + "/" + rollback_route.diagnostic);

  auto focused_reuse_config = rollback_config;
  focused_reuse_config.max_iterations = 3U;
  focused_reuse_config.selective_congestion_reroute = true;
  focused_reuse_config.max_hard_capacity_blocker_reuse_invalidations_per_failed_net = 1U;
  const auto focused_reuse_route = flux::route_global(rollback_source, focused_reuse_config);
  require(
      focused_reuse_route.configured_max_hard_capacity_blocker_reuse_invalidations_per_failed_net ==
              1U &&
          focused_reuse_route.hard_capacity_blocker_reuse_invalidations == 1U &&
          focused_reuse_route.selective_retained_net_observations > 0U &&
          focused_reuse_route.hard_capacity_pass_summaries.size() == 2U &&
          focused_reuse_route.hard_capacity_pass_summaries.back().attempted_order.size() == 2U,
      "selective strict reuse must retain the unrelated legal gate route and reroute only the "
      "failed net plus its highest-evidence blocker: " +
          std::to_string(
              focused_reuse_route
                  .configured_max_hard_capacity_blocker_reuse_invalidations_per_failed_net) +
          "/" + std::to_string(focused_reuse_route.hard_capacity_blocker_reuse_invalidations) +
          "/" + std::to_string(focused_reuse_route.selective_retained_net_observations) + "/" +
          std::to_string(focused_reuse_route.hard_capacity_pass_summaries.size()) + "/" +
          (focused_reuse_route.hard_capacity_pass_summaries.empty()
               ? "0"
               : std::to_string(focused_reuse_route.hard_capacity_pass_summaries.back()
                                    .attempted_order.size())) +
          "/" + focused_reuse_route.diagnostic);

  auto holed_zone_source = source;
  holed_zone_source.nets.push_back(Net{NetId{"holed-zone-net"}, "HOLED_ZONE", 0});
  flux::CopperZone holed_zone;
  holed_zone.id = ObjectId{"holed-zone"};
  holed_zone.net = NetId{"holed-zone-net"};
  holed_zone.fills = {
      flux::CopperZoneFill{LayerId{"F.Cu"},
                           rectangle(5.0, 5.0, 95.0, 75.0),
                           {rectangle(10.0, 10.0, 15.0, 15.0), rectangle(45.0, 35.0, 55.0, 45.0),
                            rectangle(85.0, 65.0, 90.0, 70.0)}}};
  holed_zone_source.zones.push_back(std::move(holed_zone));
  GlobalRouterConfig holed_zone_config;
  holed_zone_config.grid_pitch = Nanometres::from_mm(5.0);
  holed_zone_config.max_iterations = 1U;
  const auto holed_zone_route = flux::route_global(holed_zone_source, holed_zone_config);
  require(holed_zone_route.stop_reason != GlobalRouterStopReason::invalid_input &&
              holed_zone_route.zone_hole_broad_phase_rejections > 0U &&
              holed_zone_route.zone_hole_exact_checks > 0U &&
              holed_zone_route.zone_hole_broad_phase_rejections >
                  holed_zone_route.zone_hole_exact_checks,
          "holed-zone routing must retain exact checks while rejecting distant contours broadly");

  GlobalRouterConfig config;
  config.grid_pitch = Nanometres::from_mm(5.0);
  config.use_voronoi_allocation_resources = true;
  config.require_voronoi_topological_transitions = true;
  config.max_voronoi_resource_segment_length = Nanometres::from_mm(5.0);
  const auto global = flux::route_global(source, config);
  require(
      global.complete() && global.unrouted_nets.empty() && global.total_overflow == 0U &&
          global.configured_max_voronoi_resource_segment_length == Nanometres::from_mm(5.0) &&
          global.segmented_voronoi_source_resources > 0U &&
          global.additional_voronoi_segment_resources > 0U &&
          std::all_of(global.voronoi_resource_usage.begin(), global.voronoi_resource_usage.end(),
                      [](const auto& resource) {
                        return resource.start != resource.end &&
                               flux::geometry::point_distance_nm(resource.start, resource.end) <=
                                   static_cast<long double>(Nanometres::from_mm(5.0).value());
                      }),
      "segmented physical routing must produce bounded local resources and a feasible plan");

  const auto detailed = flux::materialize_detailed_route(source, global);
  require(detailed.complete() && detailed.candidate && detailed.added_tracks > 0U &&
              detailed.connectivity.open_connections == 0U && detailed.copper_drc.ok() &&
              source.tracks.empty(),
          "detailed routing must produce checked copper on a copy without mutating the source: " +
              std::to_string(static_cast<int>(detailed.stop_reason)) + "/" +
              std::to_string(detailed.candidate.has_value()) + "/" +
              std::to_string(detailed.added_tracks) + "/" + std::to_string(detailed.added_vias) +
              "/" + std::to_string(detailed.connectivity.open_connections) + "/" +
              std::to_string(detailed.copper_drc.violations.size()));

  auto overfull = global;
  overfull.total_overflow = 1U;
  const auto withheld = flux::materialize_detailed_route(source, overfull);
  require(withheld.stop_reason == flux::DetailedRouterStopReason::incomplete_global_route &&
              !withheld.candidate && withheld.added_tracks == 0U && withheld.added_vias == 0U,
          "abstract overflow must transactionally block copper materialization");

  auto refill_source = two_pad_board();
  refill_source.layers.resize(1U);
  refill_source.nets.push_back(Net{NetId{"plane-net"}, "PLANE", 0});
  flux::CopperZone blocking_fill;
  blocking_fill.id = ObjectId{"saved-plane-fill"};
  blocking_fill.net = NetId{"plane-net"};
  blocking_fill.fills = {
      flux::CopperZoneFill{LayerId{"F.Cu"}, rectangle(45.0, 0.0, 55.0, 80.0), {}}};
  refill_source.zones.push_back(std::move(blocking_fill));
  GlobalRouterConfig immutable_fill_config;
  immutable_fill_config.grid_pitch = Nanometres::from_mm(5.0);
  immutable_fill_config.max_iterations = 2U;
  immutable_fill_config.bidirectional_route_growth_start_iteration = 1U;
  const auto immutable_fill_route = flux::route_global(refill_source, immutable_fill_config);
  auto refillable_config = immutable_fill_config;
  refillable_config.treat_saved_zone_fills_as_refillable = true;
  const auto refillable_route = flux::route_global(refill_source, refillable_config);
  flux::DetailedRouterConfig refillable_detail_config;
  refillable_detail_config.pull_tight_45_degrees = false;
  refillable_detail_config.continuous_refinement = false;
  refillable_detail_config.defer_saved_zone_fill_clearance_to_refill = true;
  const auto refillable_detail =
      flux::materialize_detailed_route(refill_source, refillable_route, refillable_detail_config);
  require(!immutable_fill_route.complete() && refillable_route.complete() &&
              immutable_fill_route.bidirectional_route_growth_enabled &&
              immutable_fill_route.configured_bidirectional_route_growth_start_iteration == 1U &&
              immutable_fill_route.bidirectional_route_growth_attempts > 0U &&
              immutable_fill_route.bidirectional_route_growth_accepted +
                      immutable_fill_route.bidirectional_route_growth_fallbacks ==
                  immutable_fill_route.bidirectional_route_growth_attempts &&
              refillable_detail.provisional_zone_refill_candidate() &&
              !refillable_detail.complete() && refillable_detail.candidate &&
              refillable_detail.connectivity.open_connections == 0U &&
              refillable_detail.copper_drc.ok(),
          "saved fills must remain immutable by default while explicit refill planning can only "
          "publish a provisional externally gated candidate");
}

void obstacle_aware_diagonals_and_curves_remain_capacity_and_drc_gated() {
  Board crossing;
  crossing.source_revision = "crossing-safe-diagonal-risk-fixture";
  crossing.outline = rectangle(0.0, 0.0, 30.0, 30.0);
  crossing.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0}};
  crossing.nets = {{NetId{"diagonal-a"}, "DIAGONAL_A", 2}, {NetId{"diagonal-b"}, "DIAGONAL_B", 1}};
  const auto terminal = [](std::string id, NetId net, Point position) {
    Footprint footprint;
    footprint.id = ObjectId{id};
    footprint.reference = id;
    footprint.position = position;
    footprint.courtyard = local_rectangle(0.4, 0.4);
    footprint.pads = {{ObjectId{id + "-pad"}, std::move(net), Point{}, {LayerId{"F.Cu"}}}};
    return footprint;
  };
  crossing.footprints = {terminal("A1", NetId{"diagonal-a"}, mm(10.0, 10.0)),
                         terminal("A2", NetId{"diagonal-a"}, mm(20.0, 20.0)),
                         terminal("B1", NetId{"diagonal-b"}, mm(20.0, 10.0)),
                         terminal("B2", NetId{"diagonal-b"}, mm(10.0, 20.0))};
  GlobalRouterConfig crossing_config;
  crossing_config.grid_pitch = Nanometres::from_mm(10.0);
  crossing_config.max_iterations = 1U;
  crossing_config.max_pin_escape_grid_radius = 0U;
  crossing_config.max_pin_access_nodes_per_terminal = 1U;
  crossing_config.use_free_space_corridors = false;
  crossing_config.allow_crossing_safe_diagonal_grid_edges = true;
  crossing_config.clearance_shadow_cost_quantum = Nanometres::from_mm(0.05);
  crossing_config.present_congestion_cost = 0U;
  crossing_config.history_congestion_cost = 0U;
  const auto crossed = flux::route_global(crossing, crossing_config);
  const auto shared_cell = std::find_if(
      crossed.resource_usage.begin(), crossed.resource_usage.end(), [](const auto& usage) {
        return usage.kind == flux::RoutingResourceKind::planar_edge && usage.demand == 2U &&
               usage.capacity == 1U;
      });
  require(crossed.configured_crossing_safe_diagonal_grid_edges && crossed.paths.size() == 2U &&
              crossed.sampled_overflow == 1U && crossed.clearance_shadow_overflow > 0U &&
              !crossed.clearance_shadow_conflicts.empty() &&
              std::all_of(crossed.clearance_shadow_conflicts.begin(),
                          crossed.clearance_shadow_conflicts.end(),
                          [](const auto& conflict) {
                            return conflict.shadow_owner_first.has_value() &&
                                   conflict.shadow_owner_second.has_value();
                          }) &&
              crossed.peak_clearance_shadow_history_increment > 1U &&
              crossed.clearance_shadow_history_increment >=
                  crossed.peak_clearance_shadow_history_increment &&
              crossed.residual_conflict_groups.size() == 1U &&
              shared_cell != crossed.resource_usage.end(),
          "opposite exact-legal cell diagonals must be discoverable but share one hard physical "
          "crossing resource whose two negotiated participants retain their own attribution");

  auto diversified_crossing_config = crossing_config;
  diversified_crossing_config.max_iterations = 6U;
  diversified_crossing_config.atomic_complete_net_routing = true;
  diversified_crossing_config.enable_search_diversification = true;
  diversified_crossing_config.global_route_beam_width = 4U;
  diversified_crossing_config.max_large_neighborhood_attempts = 5U;
  diversified_crossing_config.large_neighborhood_stagnation_iterations = 1U;
  diversified_crossing_config.destroy_route_percent = 25U;
  diversified_crossing_config.max_routing_order_variants = 4U;
  diversified_crossing_config.routing_order_diversification_seed = 37U;
  const auto diversified_crossing = flux::route_global(crossing, diversified_crossing_config);
  const auto repeated_diversified_crossing =
      flux::route_global(crossing, diversified_crossing_config);
  require(diversified_crossing.configured_search_diversification &&
              diversified_crossing.configured_global_route_beam_width == 4U &&
              diversified_crossing.global_route_beam_peak > 0U &&
              diversified_crossing.global_route_beam_peak <= 4U &&
              diversified_crossing.temporary_illegal_checkpoints > 0U &&
              diversified_crossing.routing_order_variants_attempted > 0U &&
              diversified_crossing.large_neighborhood_attempts == 5U &&
              diversified_crossing.large_neighborhood_destroyed_nets >= 2U &&
              diversified_crossing.large_neighborhood_operators.size() == 5U &&
              diversified_crossing.large_neighborhood_operators.back() ==
                  "destroy_terminal_via_cluster_and_random_routes" &&
              diversified_crossing.paths == repeated_diversified_crossing.paths &&
              diversified_crossing.large_neighborhood_operators ==
                  repeated_diversified_crossing.large_neighborhood_operators &&
              diversified_crossing.routing_order_variants_attempted ==
                  repeated_diversified_crossing.routing_order_variants_attempted &&
              diversified_crossing.large_neighborhood_destroyed_nets ==
                  repeated_diversified_crossing.large_neighborhood_destroyed_nets,
          "a fixed diversification seed must preserve deterministic beam/order/destruction search "
          "while allowing bounded temporary physical illegality: " +
              std::to_string(diversified_crossing.global_route_beam_peak) + "/" +
              std::to_string(diversified_crossing.temporary_illegal_checkpoints) + "/" +
              std::to_string(diversified_crossing.routing_order_variants_attempted) + "/" +
              std::to_string(diversified_crossing.large_neighborhood_attempts) + "/" +
              std::to_string(diversified_crossing.large_neighborhood_destroyed_nets) + "/" +
              diversified_crossing.diagnostic);

  flux::RegionalRouterConfig terminal_portal_config;
  terminal_portal_config.enable_terminal_portal_templates = true;
  const auto regional = flux::analyze_regional_conflicts(crossing, crossed, terminal_portal_config);
  const auto repeated_regional =
      flux::analyze_regional_conflicts(crossing, crossed, terminal_portal_config);
  flux::RouterTelemetryConfig regional_telemetry_config;
  regional_telemetry_config.queue_capacity = 128U;
  regional_telemetry_config.publish_rate_hz = 1'000U;
  regional_telemetry_config.level = flux::RouterTelemetryLevel::diagnostic;
  flux::RouterTelemetryService regional_telemetry(regional_telemetry_config);
  const auto observed_regional = flux::analyze_regional_conflicts(
      crossing, crossed, terminal_portal_config, {}, &regional_telemetry);
  regional_telemetry.stop();
  const auto regional_telemetry_state = regional_telemetry.state();
  const auto decoded_regional_snapshot =
      flux::telemetry::decode_events(flux::serialize_router_telemetry_snapshot(
          regional_telemetry_state, regional_telemetry.statistics()));
  const auto alternative_terminal_template = std::any_of(
      regional.bundle_assignments.begin(), regional.bundle_assignments.end(),
      [&](const auto& assignment) {
        return std::any_of(
            assignment.templates.begin(), assignment.templates.end(),
            [&](const auto& route_template) {
              const std::array choices{route_template.terminal_entry_choice,
                                       route_template.terminal_exit_choice};
              return std::any_of(choices.begin(), choices.end(), [&](const auto& choice_id) {
                if (!choice_id) {
                  return false;
                }
                const auto choice =
                    std::find_if(regional.terminal_choices.begin(), regional.terminal_choices.end(),
                                 [&](const auto& candidate) { return candidate.id == *choice_id; });
                return choice != regional.terminal_choices.end() &&
                       !choice->selected_global_access && choice->access_path.size() == 2U &&
                       choice->access_path.front() == choice->pad_position &&
                       choice->access_path.back() == choice->portal;
              });
            });
      });
  require(regional == repeated_regional && regional == observed_regional &&
              decoded_regional_snapshot && decoded_regional_snapshot.events.size() == 1U &&
              decoded_regional_snapshot.events.front().dynamic_state_included &&
              decoded_regional_snapshot.events.front().conflicts.size() ==
                  regional.conflict_edges.size() &&
              regional_telemetry_state.total_groups == regional.components.size() &&
              regional_telemetry_state.regional_conflicts == regional.conflict_edges.size() &&
              !regional_telemetry_state.observed_routes.empty() &&
              regional.source_clearance_shadow_conflicts ==
                  crossed.clearance_shadow_conflicts.size() &&
              regional.coalesced_clearance_shadow_samples +
                      regional.unmapped_clearance_shadow_conflicts ==
                  regional.source_clearance_shadow_conflicts &&
              regional.lane_solver_queries > 0U && regional.lane_solver_states > 0U &&
              regional.lane_solver_cache_hits <= regional.lane_solver_queries &&
              regional.unmapped_clearance_shadow_conflicts == 0U && !regional.fragments.empty() &&
              !regional.conflict_edges.empty() && regional.components.size() == 1U &&
              regional.components.front().nets.size() == 2U && alternative_terminal_template &&
              regional.complete() == regional.spline_candidate_available &&
              (!regional.complete() ||
               std::all_of(regional.bundle_assignments.begin(), regional.bundle_assignments.end(),
                           [](const auto& assignment) { return assignment.complete(); })),
          "regional analysis must deterministically attribute every exact clearance-shadow "
          "observation to stable route fragments, expose exact checked terminal-portal templates, "
          "and require an all-window spline candidate before bounded source conflicts count as "
          "regionally complete");

  Board conflict_chain;
  conflict_chain.source_revision = "regional-conflict-chain";
  conflict_chain.outline = rectangle(0.0, 0.0, 100.0, 40.0);
  conflict_chain.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0}};
  for (std::size_t index = 1U; index <= 4U; ++index) {
    conflict_chain.nets.push_back(Net{NetId{"chain-" + std::to_string(index)}, "CHAIN",
                                      static_cast<std::uint32_t>(5U - index)});
  }
  flux::GlobalRouteResult chain_global;
  for (std::size_t index = 1U; index <= 4U; ++index) {
    const auto y = 10.0 + static_cast<double>(index) * 0.1;
    chain_global.paths.push_back(
        flux::GlobalRoutePath{NetId{"chain-" + std::to_string(index)},
                              0U,
                              {{mm(5.0, y), LayerId{"F.Cu"}}, {mm(95.0, y), LayerId{"F.Cu"}}}});
  }
  const auto add_shadow = [&chain_global](std::size_t first_net, std::size_t second_net,
                                          double first_x, double second_x) {
    const auto first_y = 10.0 + static_cast<double>(first_net) * 0.1;
    const auto second_y = 10.0 + static_cast<double>(second_net) * 0.1;
    chain_global.clearance_shadow_conflicts.push_back(flux::RoutingClearanceShadowConflict{
        flux::RoutingClearanceShadowResourceKind::planar_edge,
        {mm(first_x, first_y), LayerId{"F.Cu"}},
        {mm(second_x, first_y), LayerId{"F.Cu"}},
        NetId{"chain-" + std::to_string(first_net)},
        NetId{"chain-" + std::to_string(second_net)},
        Nanometres::from_mm(0.1),
        flux::GlobalRoutePoint{mm(first_x, second_y), LayerId{"F.Cu"}},
        flux::GlobalRoutePoint{mm(second_x, second_y), LayerId{"F.Cu"}}});
  };
  add_shadow(1U, 2U, 10.0, 15.0);
  add_shadow(1U, 2U, 70.0, 75.0);
  add_shadow(2U, 3U, 30.0, 35.0);
  add_shadow(3U, 4U, 50.0, 55.0);
  const auto chain_regional = flux::analyze_regional_conflicts(conflict_chain, chain_global);
  flux::RegionalRouterConfig bounded_retry_config;
  bounded_retry_config.max_residual_retry_bundle_assignments = 1U;
  const auto bounded_retry =
      flux::analyze_regional_conflicts(conflict_chain, chain_global, bounded_retry_config);
  require(chain_regional.source_clearance_shadow_conflicts == 4U &&
              chain_regional.conflict_edges.size() == 4U &&
              chain_regional.components.size() == 1U &&
              chain_regional.biconnected_blocks.size() == 3U &&
              bounded_retry.bundle_assignments.size() > 1U &&
              bounded_retry.residual_conflict_cone_retries == 1U &&
              bounded_retry.residual_conflict_cone_recomputed_assignments == 1U &&
              bounded_retry.residual_conflict_cone_retry_withheld_assignments == 0U &&
              bounded_retry.residual_conflict_cone_reused_assignments > 0U &&
              chain_regional.articulation_fragments.size() == 2U &&
              !chain_regional.windows.empty() && chain_regional.windows.size() <= 3U &&
              std::all_of(chain_regional.windows.begin(), chain_regional.windows.end(),
                          [&](const auto& window) {
                            return !window.conflict_edges.empty() && !window.portals.empty() &&
                                   window.fragments.size() <
                                       chain_regional.components.front().fragments.size();
                          }),
          "a weak net chain must retain distant same-owner conflicts while decomposing into "
          "articulation-bounded physical windows instead of one whole-net repair group");

  flux::RegionalRouteResult split_regional;
  split_regional.configured.max_conflict_fragments_per_window = 3U;
  split_regional.configured.max_nets_per_window = 3U;
  split_regional.configured.window_margin = Nanometres::from_mm(2.0);
  for (std::size_t index = 1U; index <= 4U; ++index) {
    const auto id =
        flux::RegionalRouteFragmentId{NetId{"chain-" + std::to_string(index)}, 0U, 0U, 0U};
    split_regional.fragments.push_back(flux::RegionalRouteFragment{
        id,
        LayerId{"F.Cu"},
        mm(1.0, 5.0 + static_cast<double>(index)),
        mm(99.0, 5.0 + static_cast<double>(index)),
        Nanometres::from_mm(0.2),
        Nanometres::from_mm(0.1),
        {mm(1.0, 5.0 + static_cast<double>(index)), mm(99.0, 5.0 + static_cast<double>(index))}});
  }
  const auto add_partition_edge = [&split_regional](std::size_t first, std::size_t second,
                                                    double x) {
    flux::RegionalConflictEdge edge;
    edge.id = split_regional.conflict_edges.size();
    edge.first = split_regional.fragments[first].id;
    edge.second = split_regional.fragments[second].id;
    edge.layer = LayerId{"F.Cu"};
    edge.conflict_bounds = {mm(x, 8.0), mm(x + 1.0, 12.0)};
    edge.peak_clearance_deficit = Nanometres::from_mm(0.1);
    edge.observed_overlap_length_nm = static_cast<std::uint64_t>(Nanometres::from_mm(1.0).value());
    edge.sample_count = 1U;
    edge.clearance_shadow_sample_count = 1U;
    split_regional.conflict_edges.push_back(std::move(edge));
  };
  add_partition_edge(0U, 1U, 10.0);
  add_partition_edge(1U, 2U, 12.0);
  add_partition_edge(2U, 3U, 70.0);
  add_partition_edge(3U, 0U, 72.0);
  flux::partition_regional_conflicts(conflict_chain, split_regional);
  std::size_t partitioned_edge_count = 0U;
  for (const auto& window : split_regional.windows) {
    partitioned_edge_count += window.conflict_edges.size();
  }
  require(split_regional.biconnected_blocks.size() == 1U &&
              split_regional.geometrically_split_windows == 1U &&
              split_regional.uncuttable_oversized_blocks == 0U &&
              split_regional.windows.size() == 2U && partitioned_edge_count == 4U &&
              std::all_of(split_regional.windows.begin(), split_regional.windows.end(),
                          [](const auto& window) {
                            return window.nets.size() == 3U && window.eligible() &&
                                   !window.portals.empty();
                          }),
          "a spatially separated biconnected cycle must split at a conflict-free geometric cut "
          "without dropping edges or exceeding the bounded three-net window");

  flux::RegionalRouteResult anchored_regional;
  for (std::size_t index = 0U; index < 2U; ++index) {
    const auto id =
        flux::RegionalRouteFragmentId{NetId{"anchor-" + std::to_string(index)}, 0U, 0U, 0U};
    anchored_regional.fragments.push_back(flux::RegionalRouteFragment{
        id,
        LayerId{"F.Cu"},
        mm(5.0, 10.0 + static_cast<double>(index)),
        mm(95.0, 10.0 + static_cast<double>(index)),
        Nanometres::from_mm(0.2),
        Nanometres::from_mm(0.1),
        {mm(5.0, 10.0 + static_cast<double>(index)), mm(95.0, 10.0 + static_cast<double>(index))}});
  }
  flux::RegionalConflictEdge displaced_edge;
  displaced_edge.id = 0U;
  displaced_edge.first = anchored_regional.fragments[0].id;
  displaced_edge.second = anchored_regional.fragments[1].id;
  displaced_edge.layer = LayerId{"F.Cu"};
  displaced_edge.conflict_bounds = {mm(45.0, 30.0), mm(55.0, 31.0)};
  displaced_edge.peak_clearance_deficit = Nanometres::from_mm(0.1);
  displaced_edge.observed_overlap_length_nm =
      static_cast<std::uint64_t>(Nanometres::from_mm(1.0).value());
  displaced_edge.sample_count = 1U;
  anchored_regional.conflict_edges.push_back(std::move(displaced_edge));
  flux::partition_regional_conflicts(conflict_chain, anchored_regional);
  require(anchored_regional.windows.size() == 1U && anchored_regional.windows.front().eligible() &&
              anchored_regional.window_fragment_anchor_expansions == 2U &&
              anchored_regional.peak_window_fragment_anchor_expansion > Nanometres::from_mm(10.0) &&
              anchored_regional.windows.front().portals.size() == 4U,
          "a conflict-derived window must add deterministic nearest-fragment anchors before "
          "portal clipping instead of rejecting attributed physical routes that lie outside the "
          "sample bounds");

  std::vector<flux::RegionalRouteFragment> lane_fragments;
  for (std::size_t index = 0U; index < 3U; ++index) {
    const auto id =
        flux::RegionalRouteFragmentId{NetId{"lane-" + std::to_string(index)}, 0U, 0U, 0U};
    lane_fragments.push_back(flux::RegionalRouteFragment{
        id, LayerId{"F.Cu"}, mm(0.0, 0.0), mm(10.0, 0.0),
        Nanometres::from_mm(index == 1U ? 0.3 : 0.2), Nanometres::from_mm(0.1),
        flux::geometry::AxisAlignedBounds{mm(0.0, 0.0), mm(10.0, 0.0)}});
  }
  std::vector<flux::RegionalRouteFragmentId> lane_order;
  std::transform(lane_fragments.begin(), lane_fragments.end(), std::back_inserter(lane_order),
                 [](const auto& fragment) { return fragment.id; });
  const auto fitting_lanes =
      flux::pack_regional_cross_section(lane_fragments, lane_order, LayerId{"F.Cu"}, mm(0.0, 0.0),
                                        mm(0.0, 1.1), Nanometres::from_mm(1.1));
  const auto blocked_lanes =
      flux::pack_regional_cross_section(lane_fragments, lane_order, LayerId{"F.Cu"}, mm(0.0, 0.0),
                                        mm(0.0, 1.09), Nanometres::from_mm(1.09));
  const auto reversed_walls =
      flux::pack_regional_cross_section(lane_fragments, lane_order, LayerId{"F.Cu"}, mm(0.0, 1.1),
                                        mm(0.0, 0.0), Nanometres::from_mm(1.1));
  require(fitting_lanes.physically_fits && fitting_lanes.placements.size() == 3U &&
              fitting_lanes.packed_width == Nanometres::from_mm(1.1) &&
              !blocked_lanes.physically_fits && reversed_walls == fitting_lanes,
          "regional corridor packing must use heterogeneous copper widths and adjacent physical "
          "clearances rather than a scalar lane count, and reversed route direction must not "
          "reverse the physical lane axis");

  // Stable route IDs deliberately run opposite to their physical bottom-to-top positions. The
  // joint allocator must preserve the incoming geometry rather than treating the ID ordering as
  // a lane topology decision.
  auto geometric_lane_fragments = lane_fragments;
  for (std::size_t index = 0U; index < geometric_lane_fragments.size(); ++index) {
    const auto y = 2.0 - static_cast<double>(index);
    geometric_lane_fragments[index].start = mm(0.0, y);
    geometric_lane_fragments[index].end = mm(10.0, y);
  }
  const auto geometric_lane_order = flux::geometric_regional_cross_section_order(
      geometric_lane_fragments, lane_order, mm(5.0, 0.0), mm(5.0, 2.0));
  const auto reversed_geometric_lane_order = flux::geometric_regional_cross_section_order(
      geometric_lane_fragments, lane_order, mm(5.0, 2.0), mm(5.0, 0.0));
  auto reversed_lane_enumeration = lane_order;
  std::reverse(reversed_lane_enumeration.begin(), reversed_lane_enumeration.end());
  const auto reordered_input_lane_order = flux::geometric_regional_cross_section_order(
      geometric_lane_fragments, reversed_lane_enumeration, mm(5.0, 0.0), mm(5.0, 2.0));
  require(geometric_lane_order == std::vector<flux::RegionalRouteFragmentId>{lane_order[2],
                                                                             lane_order[1],
                                                                             lane_order[0]} &&
              reversed_geometric_lane_order == geometric_lane_order &&
              reordered_input_lane_order == geometric_lane_order,
          "regional lane references must follow physical cross-section intersections, remain "
          "invariant to wall/input enumeration, and use stable IDs only as coincident ties");

  flux::GlobalRouteResult corridor_membership_global;
  flux::GlobalRoutePath corridor_membership_path;
  corridor_membership_path.net = NetId{"diagonal-a"};
  corridor_membership_path.points = {{mm(2.0, 2.0), LayerId{"F.Cu"}},
                                     {mm(8.0, 2.0), LayerId{"F.Cu"}},
                                     {mm(8.0, 20.0), LayerId{"F.Cu"}}};
  flux::RoutingGeometricCorridorRun local_corridor;
  local_corridor.layer = LayerId{"F.Cu"};
  local_corridor.topology_cells = {rectangle(1.0, 1.0, 7.0, 3.0)};
  local_corridor.lane_centerline = {mm(2.0, 2.0), mm(7.0, 2.0)};
  local_corridor.required_envelope_width = Nanometres::from_mm(0.4);
  local_corridor.minimum_physical_cross_section_width = Nanometres::from_mm(2.0);
  local_corridor.lane_capacity = 3U;
  local_corridor.selected_path_preserves_topology = true;
  corridor_membership_path.geometric_corridors.push_back(std::move(local_corridor));
  corridor_membership_global.paths.push_back(std::move(corridor_membership_path));
  const auto local_corridor_regional =
      flux::analyze_regional_conflicts(crossing, corridor_membership_global);
  require(local_corridor_regional.fragments.size() == 2U &&
              local_corridor_regional.channels.size() == 1U &&
              local_corridor_regional.channels.front().occupants ==
                  std::vector<flux::RegionalRouteFragmentId>{
                      local_corridor_regional.fragments.front().id},
          "a retained geometric corridor must own only route fragments that physically traverse "
          "its cells, not every same-layer fragment of the complete connection: fragments=" +
              std::to_string(local_corridor_regional.fragments.size()) + ", channels=" +
              std::to_string(local_corridor_regional.channels.size()) + ", occupants=" +
              std::to_string(local_corridor_regional.channels.empty()
                                 ? 0U
                                 : local_corridor_regional.channels.front().occupants.size()));

  auto bounded_crossed = crossed;
  for (auto& path : bounded_crossed.paths) {
    flux::RoutingGeometricCorridorRun corridor;
    corridor.layer = LayerId{"F.Cu"};
    corridor.topology_cells = {rectangle(5.0, 5.0, 25.0, 25.0)};
    corridor.lane_centerline.reserve(path.points.size());
    for (const auto& point : path.points) {
      corridor.lane_centerline.push_back(point.position);
    }
    corridor.required_envelope_width = Nanometres::from_mm(0.65);
    corridor.minimum_physical_cross_section_width = Nanometres::from_mm(2.0);
    corridor.lane_capacity = 3U;
    corridor.selected_path_preserves_topology = true;
    path.geometric_corridors.push_back(std::move(corridor));
  }
  for (auto& group : bounded_crossed.residual_conflict_groups) {
    group.all_nets_have_preserved_geometric_corridors = true;
    group.peak_geometric_lane_capacity = 3U;
  }
  flux::DetailedRouterConfig bounded_crossed_config;
  bounded_crossed_config.pull_tight_45_degrees = false;
  bounded_crossed_config.continuous_refinement = false;
  const auto direct_bounded_detail =
      flux::materialize_detailed_route(crossing, bounded_crossed, bounded_crossed_config);
  require(
      direct_bounded_detail.residual_conflict_handoff_attempted &&
          direct_bounded_detail.residual_conflict_groups_attempted == 1U &&
          direct_bounded_detail.residual_conflict_nets_attempted == 2U &&
          direct_bounded_detail.residual_joint_candidate_states ==
              bounded_crossed_config.max_residual_joint_candidate_states &&
          !direct_bounded_detail.complete(),
      "a topology-complete bounded residual checkpoint must be handed to the joint "
      "B-spline/PNS transaction after ordinary detailed copper fails, while retaining an "
      "observable exact accept/reject outcome: handoff=" +
          std::to_string(direct_bounded_detail.residual_conflict_handoff_attempted) +
          ", groups=" + std::to_string(direct_bounded_detail.residual_conflict_groups_attempted) +
          ", nets=" + std::to_string(direct_bounded_detail.residual_conflict_nets_attempted) +
          ", states=" + std::to_string(direct_bounded_detail.residual_joint_candidate_states) +
          ", rejected=" +
          std::to_string(direct_bounded_detail.residual_joint_exact_transactions_rejected) +
          ", accepted=" +
          std::to_string(direct_bounded_detail.residual_joint_transaction_accepted) +
          ", diagnostic=" + direct_bounded_detail.diagnostic);
  flux::RoutingPlanResult crossed_plan;
  crossed_plan.coarse_global = bounded_crossed;
  crossed_plan.regional = flux::analyze_regional_conflicts(crossing, bounded_crossed);
  crossed_plan.paths = crossed_plan.regional.spline_candidate_available
                           ? crossed_plan.regional.spline_candidate_paths
                           : bounded_crossed.paths;
  crossed_plan.stop_reason = flux::RoutingPlanStopReason::regional_conflicts;
  const auto crossed_plan_detail =
      flux::materialize_detailed_route(crossing, crossed_plan, bounded_crossed_config);
  require(!crossed_plan_detail.complete(),
          "a regional B-spline proposal must remain one atomic transaction and cannot bypass "
          "exact materialization when its selected topology still forces a crossing");

  auto stacked_via_board = crossing;
  stacked_via_board.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0},
                              {LayerId{"In1.Cu"}, "In1.Cu", LayerKind::copper, 1},
                              {LayerId{"B.Cu"}, "B.Cu", LayerKind::copper, 2}};
  flux::GlobalRouteResult stacked_via_global;
  stacked_via_global.paths = {flux::GlobalRoutePath{NetId{"diagonal-a"},
                                                    0U,
                                                    {{mm(10.0, 10.0), LayerId{"F.Cu"}},
                                                     {mm(10.0, 10.0), LayerId{"In1.Cu"}},
                                                     {mm(10.0, 10.0), LayerId{"B.Cu"}},
                                                     {mm(20.0, 20.0), LayerId{"B.Cu"}}}},
                              flux::GlobalRoutePath{NetId{"diagonal-b"},
                                                    0U,
                                                    {{mm(20.0, 10.0), LayerId{"F.Cu"}},
                                                     {mm(20.0, 10.0), LayerId{"In1.Cu"}},
                                                     {mm(20.0, 10.0), LayerId{"B.Cu"}},
                                                     {mm(10.0, 20.0), LayerId{"B.Cu"}}}}};
  for (auto& path : stacked_via_global.paths) {
    flux::RoutingGeometricCorridorRun corridor;
    corridor.layer = LayerId{"B.Cu"};
    corridor.topology_cells = {rectangle(5.0, 5.0, 25.0, 25.0)};
    corridor.lane_centerline = {path.points[path.points.size() - 2U].position,
                                path.points.back().position};
    corridor.required_envelope_width = Nanometres::from_mm(0.65);
    corridor.minimum_physical_cross_section_width = Nanometres::from_mm(2.0);
    corridor.lane_capacity = 3U;
    corridor.selected_path_preserves_topology = true;
    path.geometric_corridors.push_back(std::move(corridor));
  }
  stacked_via_global.clearance_shadow_conflicts.push_back(flux::RoutingClearanceShadowConflict{
      flux::RoutingClearanceShadowResourceKind::planar_edge,
      {mm(10.0, 10.0), LayerId{"B.Cu"}},
      {mm(20.0, 20.0), LayerId{"B.Cu"}},
      NetId{"diagonal-a"},
      NetId{"diagonal-b"},
      Nanometres::from_mm(0.1),
      flux::GlobalRoutePoint{mm(20.0, 10.0), LayerId{"B.Cu"}},
      flux::GlobalRoutePoint{mm(10.0, 20.0), LayerId{"B.Cu"}}});
  flux::RegionalRouterConfig stacked_via_config;
  stacked_via_config.max_alternative_template_checks = 1U;
  stacked_via_config.max_same_layer_detour_checks = 1U;
  const auto stacked_via_regional =
      flux::analyze_regional_conflicts(stacked_via_board, stacked_via_global, stacked_via_config);
  const auto selected_stacked_via_regions = static_cast<std::size_t>(
      std::count_if(stacked_via_regional.via_region_evidence.begin(),
                    stacked_via_regional.via_region_evidence.end(),
                    [](const auto& evidence) { return evidence.selected_global_candidates > 0U; }));
  require(selected_stacked_via_regions == 2U &&
              std::count_if(stacked_via_regional.via_choices.begin(),
                            stacked_via_regional.via_choices.end(),
                            [](const auto& choice) {
                              return choice.selected_global_site && choice.span.size() == 3U;
                            }) == 2,
          "each maximal same-coordinate layer stack must be one physical regional via variable "
          "with its complete span, matching detailed materialization");
  require(std::all_of(stacked_via_regional.fragments.begin(), stacked_via_regional.fragments.end(),
                      [](const auto& fragment) { return fragment.begins_at_terminal; }),
          "a planar run after a stacked via at the connection endpoint must retain terminal "
          "provenance so selected-topology relocation requires explicit pad fanout geometry");

  auto multilayer_crossing = crossing;
  multilayer_crossing.layers.push_back({LayerId{"B.Cu"}, "B.Cu", LayerKind::copper, 1});
  multilayer_crossing.nets.push_back(Net{NetId{"via-site-blocker"}, "VIA_SITE_BLOCKER", 0});
  for (std::size_t index = 0U; index < crossing.footprints.size(); ++index) {
    const auto center = crossing.footprints[index].position;
    multilayer_crossing.tracks.push_back(flux::TrackSegment{
        ObjectId{"via-site-blocker-" + std::to_string(index)}, NetId{"via-site-blocker"},
        LayerId{"B.Cu"},
        Point{Nanometres{center.x.value() - Nanometres::from_mm(0.05).value()}, center.y},
        Point{Nanometres{center.x.value() + Nanometres::from_mm(0.05).value()}, center.y},
        Nanometres::from_mm(0.2), false});
  }
  auto multilayer_global = bounded_crossed;
  flux::RegionalRouterConfig multilayer_regional_config;
  multilayer_regional_config.navmesh_cell_pitch = Nanometres::from_mm(1.0);
  multilayer_regional_config.max_navmesh_cells_per_template = 20'000U;
  multilayer_regional_config.max_navmesh_expansions_per_template = 20'000U;
  auto multilayer_plan = flux::RoutingPlanResult{};
  multilayer_plan.coarse_global = multilayer_global;
  multilayer_plan.regional = flux::analyze_regional_conflicts(
      multilayer_crossing, multilayer_global, multilayer_regional_config);
  multilayer_plan.paths = multilayer_plan.regional.spline_candidate_available
                              ? multilayer_plan.regional.spline_candidate_paths
                              : multilayer_global.paths;
  multilayer_plan.stop_reason = flux::RoutingPlanStopReason::regional_conflicts;
  const auto selected_alternative = std::any_of(
      multilayer_plan.regional.bundle_assignments.begin(),
      multilayer_plan.regional.bundle_assignments.end(), [&](const auto& assignment) {
        return std::any_of(assignment.selected_templates.begin(),
                           assignment.selected_templates.end(), [&](const auto template_id) {
                             return template_id < assignment.templates.size() &&
                                    !assignment.templates[template_id].selected_global_topology;
                           });
      });
  const auto selected_relocated_via = std::any_of(
      multilayer_plan.regional.bundle_assignments.begin(),
      multilayer_plan.regional.bundle_assignments.end(), [&](const auto& assignment) {
        return std::any_of(
            assignment.selected_templates.begin(), assignment.selected_templates.end(),
            [&](const auto template_id) {
              if (template_id >= assignment.templates.size()) {
                return false;
              }
              const auto& route_template = assignment.templates[template_id];
              if (route_template.selected_global_topology) {
                return false;
              }
              const auto fragment = std::find_if(
                  multilayer_plan.regional.fragments.begin(),
                  multilayer_plan.regional.fragments.end(),
                  [&](const auto& candidate) { return candidate.id == route_template.fragment; });
              if (fragment == multilayer_plan.regional.fragments.end()) {
                return false;
              }
              return std::any_of(route_template.via_choices.begin(),
                                 route_template.via_choices.end(), [&](const auto choice_id) {
                                   const auto choice =
                                       std::find_if(multilayer_plan.regional.via_choices.begin(),
                                                    multilayer_plan.regional.via_choices.end(),
                                                    [&](const auto& candidate) {
                                                      return candidate.id == choice_id;
                                                    });
                                   return choice != multilayer_plan.regional.via_choices.end() &&
                                          choice->site != fragment->start &&
                                          choice->site != fragment->end;
                                 });
            });
      });
  const auto multilayer_detail = flux::materialize_detailed_route(
      multilayer_crossing, multilayer_plan, bounded_crossed_config);
  const auto retained_via_choices =
      std::accumulate(multilayer_plan.regional.via_region_evidence.begin(),
                      multilayer_plan.regional.via_region_evidence.end(), std::size_t{0},
                      [](const auto total, const auto& evidence) {
                        return total + evidence.candidate_choices.size();
                      });
  const auto bounded_via_portfolio =
      !multilayer_plan.regional.via_region_evidence.empty() &&
      retained_via_choices == multilayer_plan.regional.via_choices.size() &&
      std::all_of(multilayer_plan.regional.via_region_evidence.begin(),
                  multilayer_plan.regional.via_region_evidence.end(),
                  [&](const auto& evidence) {
                    return !evidence.candidate_choices.empty() &&
                           evidence.candidate_choices.size() <=
                               multilayer_regional_config.max_via_sites_per_region;
                  }) &&
      std::any_of(multilayer_plan.regional.via_region_evidence.begin(),
                  multilayer_plan.regional.via_region_evidence.end(),
                  [](const auto& evidence) { return evidence.candidate_choices.size() > 1U; });
  const auto stable_via_assignment_order = std::all_of(
      multilayer_plan.regional.bundle_assignments.begin(),
      multilayer_plan.regional.bundle_assignments.end(), [](const auto& assignment) {
        const std::set<std::size_t> unique(assignment.via_assignment_region_order.begin(),
                                           assignment.via_assignment_region_order.end());
        return unique.size() == assignment.via_assignment_region_order.size();
      });
  const auto complete_two_via_plan =
      std::any_of(multilayer_plan.regional.multilayer_route_plans.begin(),
                  multilayer_plan.regional.multilayer_route_plans.end(), [](const auto& plan) {
                    return plan.via_transitions.size() >= 2U &&
                           plan.layer_runs.size() == plan.via_transitions.size() + 1U;
                  });
  require(multilayer_plan.regional.accepted_alternative_templates > 0U &&
              selected_alternative && selected_relocated_via &&
              bounded_via_portfolio && stable_via_assignment_order &&
              complete_two_via_plan &&
              multilayer_plan.regional.complete_via_dependent_routes ==
                  multilayer_plan.regional.multilayer_route_plans.size() &&
              multilayer_plan.regional.planned_via_transitions >= 2U &&
              multilayer_plan.regional.spline_candidate_available &&
              multilayer_detail.residual_conflict_handoff_attempted &&
              multilayer_detail.residual_conflict_groups_attempted > 0U &&
              multilayer_detail.residual_conflict_nets_attempted == 2U &&
              multilayer_detail.residual_joint_groups_optimized ==
                  multilayer_plan.regional.bundle_assignments.size() &&
              multilayer_detail.residual_joint_transaction_accepted &&
              multilayer_detail.residual_joint_exact_transactions_rejected == 0U &&
              std::all_of(multilayer_plan.regional.bundle_assignments.begin(),
                          multilayer_plan.regional.bundle_assignments.end(),
                          [](const auto& assignment) {
                            return std::all_of(
                                assignment.spline_realizations.begin(),
                                assignment.spline_realizations.end(), [](const auto& realization) {
                                  return realization.quantized_mutually_legal;
                                });
                          }) &&
              std::any_of(multilayer_plan.paths.begin(), multilayer_plan.paths.end(),
                          [](const auto& path) {
                            return std::adjacent_find(
                                       path.points.begin(), path.points.end(),
                                       [](const auto& first, const auto& second) {
                                         return first.position == second.position &&
                                                first.layer != second.layer;
                                       }) != path.points.end();
                          }) &&
              multilayer_detail.complete() && multilayer_detail.added_vias >= 2U,
          "a bounded crossing window must accept attributed sampled overflow, jointly choose an "
          "exact-checked alternate layer and relocated via site, "
          "realize it as B-spline-derived copper with endpoint vias, and commit only after "
          "whole-board connectivity and DRC; alternatives=" +
              std::to_string(multilayer_plan.regional.accepted_alternative_templates) +
              " selected=" + std::to_string(selected_alternative) + " spline=" +
              std::to_string(multilayer_plan.regional.spline_candidate_available) +
              " relocated=" + std::to_string(selected_relocated_via) + " via_regions=" +
              std::to_string(multilayer_plan.regional.via_region_evidence.size()) +
              " retained_vias=" + std::to_string(retained_via_choices) + " detail=" +
              std::to_string(multilayer_detail.complete()) + " diagnostic=" +
              multilayer_detail.diagnostic + " bundles=" +
              std::to_string(multilayer_plan.regional.bundle_assignments.size()) +
              " bundle_stop=" +
              (multilayer_plan.regional.bundle_assignments.empty()
                   ? std::string{"none"}
                   : std::to_string(static_cast<int>(
                         multilayer_plan.regional.bundle_assignments.front().stop_reason))) +
              " realizations=" +
              (multilayer_plan.regional.bundle_assignments.empty()
                   ? std::string{"none"}
                   : std::to_string(multilayer_plan.regional.bundle_assignments.front()
                                        .spline_realizations.size())) +
              " realization_flags=" +
              (multilayer_plan.regional.bundle_assignments.empty()
                   ? std::string{"none"}
                   : [&]() {
                       std::string flags;
                       for (const auto& realization :
                            multilayer_plan.regional.bundle_assignments.front()
                                .spline_realizations) {
                         flags += std::to_string(realization.topology_legal) + "/" +
                                  std::to_string(realization.mutually_legal) + ";";
                       }
                       return flags;
                     }()) +
              " selected_residual=" +
              (multilayer_plan.regional.bundle_assignments.empty()
                   ? std::string{"none"}
                   : std::to_string(multilayer_plan.regional.bundle_assignments.front()
                                        .selected_residual_conflicts)));

  auto multi_transition_board = multilayer_crossing;
  for (auto& layer : multi_transition_board.layers) {
    if (layer.id == LayerId{"B.Cu"}) {
      layer.stack_order = 2U;
    }
  }
  multi_transition_board.layers.push_back({LayerId{"In1.Cu"}, "In1.Cu", LayerKind::copper, 1U});
  flux::GlobalRoutePath multi_transition_path;
  multi_transition_path.net = NetId{"diagonal-a"};
  multi_transition_path.connection_index = 7U;
  multi_transition_path.points = {
      {mm(2.0, 2.0), LayerId{"F.Cu"}},   {mm(4.0, 2.0), LayerId{"F.Cu"}},
      {mm(4.0, 2.0), LayerId{"In1.Cu"}}, {mm(8.0, 2.0), LayerId{"In1.Cu"}},
      {mm(8.0, 2.0), LayerId{"B.Cu"}},   {mm(12.0, 2.0), LayerId{"B.Cu"}},
      {mm(12.0, 2.0), LayerId{"F.Cu"}},  {mm(16.0, 2.0), LayerId{"F.Cu"}},
  };
  const auto multi_transition_plan =
      flux::plan_multilayer_route(multi_transition_board, multi_transition_path);
  require(multi_transition_plan && multi_transition_plan->via_transitions.size() == 3U &&
              multi_transition_plan->layer_runs.size() == 4U &&
              multi_transition_plan->via_transitions.back().span.size() == 3U,
          "multilayer planning must preserve an arbitrary sequence of distinct physical vias and "
          "planar layer runs rather than assuming exactly one via pair");

  auto stale_multilayer_plan = multilayer_plan;
  stale_multilayer_plan.stop_reason = flux::RoutingPlanStopReason::complete;
  stale_multilayer_plan.paths = stale_multilayer_plan.regional.spline_candidate_paths;
  stale_multilayer_plan.regional.multilayer_route_plans.clear();
  const auto stale_multilayer_detail = flux::materialize_detailed_route(
      multilayer_crossing, stale_multilayer_plan, bounded_crossed_config);
  require(!stale_multilayer_detail.complete() && !stale_multilayer_detail.candidate,
          "detailed routing must fail closed when complete multilayer planning evidence no longer "
          "matches the spline candidate path");

  auto explicit_width_global = multilayer_global;
  explicit_width_global.configured_terminal_neck_down_width = Nanometres::from_mm(0.15);
  explicit_width_global.configured_max_terminal_neck_down_length = Nanometres::from_mm(30.0);
  auto& explicit_path = explicit_width_global.paths.front();
  explicit_path.segment_widths.assign(explicit_path.points.size() - 1U, Nanometres::from_mm(0.15));
  explicit_path.segment_width_kinds.assign(explicit_path.segment_widths.size(),
                                           flux::GlobalRouteSegmentWidthKind::terminal_neck_down);
  explicit_path.terminal_neck_down_segments = explicit_path.segment_widths.size();
  explicit_path.terminal_neck_down_length_nm = 0U;
  for (std::size_t edge = 1U; edge < explicit_path.points.size(); ++edge) {
    explicit_path.terminal_neck_down_length_nm +=
        static_cast<std::uint64_t>(std::ceil(flux::geometry::point_distance_nm(
            explicit_path.points[edge - 1U].position, explicit_path.points[edge].position)));
  }
  flux::RoutingPlanResult explicit_width_plan;
  explicit_width_plan.coarse_global = explicit_width_global;
  explicit_width_plan.regional = flux::analyze_regional_conflicts(
      multilayer_crossing, explicit_width_global, multilayer_regional_config);
  explicit_width_plan.paths = explicit_width_plan.regional.spline_candidate_available
                                  ? explicit_width_plan.regional.spline_candidate_paths
                                  : explicit_width_global.paths;
  explicit_width_plan.stop_reason = flux::RoutingPlanStopReason::regional_conflicts;
  const auto routed_explicit = std::find_if(
      explicit_width_plan.paths.begin(), explicit_width_plan.paths.end(), [&](const auto& path) {
        return path.net == explicit_path.net &&
               path.connection_index == explicit_path.connection_index;
      });
  const auto explicit_width_detail = flux::materialize_detailed_route(
      multilayer_crossing, explicit_width_plan, bounded_crossed_config);
  require(
      explicit_width_plan.regional.refittable_terminal_width_fragments > 0U &&
          explicit_width_plan.regional.fixed_resource_width_fragments == 0U &&
          std::all_of(explicit_width_plan.regional.windows.begin(),
                      explicit_width_plan.regional.windows.end(),
                      [](const auto& window) { return window.eligible(); }) &&
          explicit_width_plan.regional.spline_candidate_available &&
          routed_explicit != explicit_width_plan.paths.end() &&
          routed_explicit->segment_widths.size() + 1U == routed_explicit->points.size() &&
          routed_explicit->segment_width_kinds.size() == routed_explicit->segment_widths.size() &&
          routed_explicit->terminal_neck_down_segments ==
              static_cast<std::size_t>(
                  std::count(routed_explicit->segment_width_kinds.begin(),
                             routed_explicit->segment_width_kinds.end(),
                             flux::GlobalRouteSegmentWidthKind::terminal_neck_down)) &&
          routed_explicit->terminal_neck_down_length_nm <=
              static_cast<std::uint64_t>(
                  explicit_width_global.configured_max_terminal_neck_down_length.value()) &&
          explicit_width_detail.complete(),
      "regional spline replacement must preserve an explicit terminal width and its provenance, "
      "recompute the tessellated segment evidence, and pass the exact transaction: fragments=" +
          std::to_string(explicit_width_plan.regional.refittable_terminal_width_fragments) +
          " spline=" + std::to_string(explicit_width_plan.regional.spline_candidate_available) +
          " detail=" + std::to_string(explicit_width_detail.complete()) +
          " diagnostic=" + explicit_width_detail.diagnostic);

  GlobalRouterConfig early_spline_config;
  early_spline_config.grid_pitch = Nanometres::from_mm(5.0);
  early_spline_config.max_iterations = 1U;
  auto early_spline_board = two_pad_board();
  early_spline_board.footprints[1].position = mm(80.0, 60.0);
  early_spline_board.nets.push_back(Net{NetId{"early-pns-obstacle-net"}, "EARLY_PNS", 0});
  early_spline_board.tracks.push_back(flux::TrackSegment{
      ObjectId{"early-pns-obstacle"}, NetId{"early-pns-obstacle-net"}, LayerId{"F.Cu"},
      mm(30.0, 40.1), mm(70.0, 40.1), Nanometres::from_mm(0.25), false});
  flux::OffsetFreeSpaceRegion direct_navmesh_region;
  direct_navmesh_region.area = rectangle(0.5, 0.5, 99.5, 79.5);
  direct_navmesh_region.holes.push_back(rectangle(29.5, 39.5, 70.5, 40.7));
  const flux::NavMeshConfig direct_navmesh_config{Nanometres::from_mm(5.0),
                                                  100'000U,
                                                  100'000U,
                                                  {},
                                                  Nanometres{0},
                                                  1U,
                                                  Nanometres{0},
                                                  std::nullopt};
  const auto direct_navmesh = flux::build_navmesh_funnel_route(
      direct_navmesh_region, mm(20.0, 40.0), mm(80.0, 60.0), direct_navmesh_config);
  const auto prepared_navmesh =
      flux::prepare_navmesh_domain(direct_navmesh_region, direct_navmesh_config);
  const auto reused_navmesh = flux::route_prepared_navmesh(prepared_navmesh, mm(20.0, 40.0),
                                                           mm(80.0, 60.0), direct_navmesh_config);
  const auto flooded_navmesh = flux::route_prepared_navmesh_flood_fill(
      prepared_navmesh, mm(20.0, 40.0), {mm(80.0, 60.0), mm(20.0, 60.0)}, direct_navmesh_config);
  const auto closed_cell = reused_navmesh.corridor[reused_navmesh.corridor.size() / 2U];
  const auto reflooded_navmesh = flux::route_prepared_navmesh_flood_fill(
      prepared_navmesh, mm(20.0, 40.0), {mm(80.0, 60.0)}, direct_navmesh_config, {closed_cell});
  require(direct_navmesh.ok() && prepared_navmesh.ok() && reused_navmesh.ok() &&
              direct_navmesh.corridor == reused_navmesh.corridor &&
              direct_navmesh.centerline == reused_navmesh.centerline &&
              reused_navmesh.used_flood_fill && reused_navmesh.flood_fill_cells_visited > 0U &&
              flooded_navmesh.ok() && flooded_navmesh.used_flood_fill &&
              flooded_navmesh.goal == mm(20.0, 60.0) && reflooded_navmesh.ok() &&
              std::find(reflooded_navmesh.corridor.begin(), reflooded_navmesh.corridor.end(),
                        closed_cell) == reflooded_navmesh.corridor.end() &&
              std::all_of(reused_navmesh.centerline.begin(), reused_navmesh.centerline.end(),
                          [&](const auto point) {
                            return flux::navmesh_corridor_contains(reused_navmesh, point);
                          }),
          "prepared, one-shot, and multi-source flood-fill NavMesh routing must select legal "
          "deterministic topology and centerlines: " +
              direct_navmesh.diagnostic + "/" + reused_navmesh.diagnostic + "/" +
              flooded_navmesh.diagnostic + "/" + reflooded_navmesh.diagnostic);
  const auto preserved_navmesh_topology = flux::validate_navmesh_topology_signature(
      direct_navmesh.topology_signature, direct_navmesh.centerline);
  auto reversed_navmesh_centerline = direct_navmesh.centerline;
  std::reverse(reversed_navmesh_centerline.begin(), reversed_navmesh_centerline.end());
  const auto reversed_navmesh_topology = flux::validate_navmesh_topology_signature(
      direct_navmesh.topology_signature, reversed_navmesh_centerline);
  require(direct_navmesh.topology_signature.valid &&
              !direct_navmesh.topology_signature.gates.empty() &&
              preserved_navmesh_topology.preserved && !reversed_navmesh_topology.preserved,
          "a selected NavMesh corridor must publish a directed gate certificate that accepts its "
          "funnel trace and rejects the reverse obstacle-side traversal");

  const auto prepared_polygon = flux::geometry::prepare_polygon(direct_navmesh_region.area);
  for (const auto point : {mm(0.5, 0.5), mm(50.0, 50.0), mm(100.0, 50.0)}) {
    require(
        flux::geometry::locate_point(prepared_polygon, point) ==
                flux::geometry::locate_point(direct_navmesh_region.area, point) &&
            std::abs(flux::geometry::point_polygon_boundary_distance_nm(point, prepared_polygon) -
                     flux::geometry::point_polygon_boundary_distance_nm(
                         point, direct_navmesh_region.area)) < 1.0e-6L,
        "prepared polygon point location and nearest-boundary distance must remain exact");
  }

  const std::vector<Point> evaluator_points{mm(10.0, 10.0), mm(20.0, 15.0), mm(30.0, 8.0),
                                            mm(40.0, 20.0), mm(50.0, 10.0)};

  // Spline controls describe direction changes, not source sampling density. A straight route
  // with duplicated/subdivided points must remain exactly straight even when a soft clearance
  // gradient would otherwise reward bowing it into open space.
  const std::vector<Point> oversampled_straight{mm(10.0, 10.0), mm(10.0, 10.0), mm(15.0, 10.0),
                                                mm(20.0, 10.0), mm(25.0, 10.0), mm(30.0, 10.0),
                                                mm(30.0, 10.0)};
  auto straight_spline = flux::make_cubic_bspline_route(oversampled_straight);
  require(straight_spline.has_value() && straight_spline->control_points.size() == 5U,
          "straight spline seeds must collapse repeated and forward-collinear samples");
  flux::SplineTrajectoryOptimizationConfig straight_config;
  straight_config.minimum_bend_radius = Nanometres{0};
  straight_config.maximum_corridor_displacement = Nanometres::from_mm(2.0);
  straight_config.samples = 24U;
  straight_config.maximum_iterations = 4U;
  straight_config.maximum_candidate_checks = 128U;
  const auto straight_optimized = flux::optimize_cubic_bspline_trajectory(
      *straight_spline, straight_config, [](flux::SplineVector point) {
        return flux::SplineClearanceProbe{
            std::max(1.0L, 100'000.0L + point.y - 10'000'000.0L), {0.0L, 1.0L}, true};
      });
  const auto straight_samples =
      flux::adaptive_tessellate_cubic_bspline(*straight_spline, Nanometres{1}, 8U, 128U);
  require(straight_optimized.accepted &&
              std::all_of(straight_samples.begin(), straight_samples.end(),
                          [](const auto& sample) {
                            return std::abs(sample.position.y - 10'000'000.0L) < 0.5L;
                          }),
          "trajectory optimization must not introduce a curve into an already legal straight "
          "route solely to improve a soft clearance objective");

  const auto evaluator_spline = flux::make_cubic_bspline_route(evaluator_points);
  require(evaluator_spline.has_value(), "prepared spline evaluator fixture must form a spline");
  flux::PreparedCubicBSplineEvaluator evaluator(evaluator_spline->control_points.size(), 17U);
  require(evaluator.update(*evaluator_spline),
          "prepared spline evaluator must accept its topology");
  for (std::size_t sample = 0U; sample < evaluator.samples().size(); ++sample) {
    const auto parameter = static_cast<long double>(sample) /
                           static_cast<long double>(evaluator.samples().size() - 1U);
    const auto expected = flux::evaluate_cubic_bspline_differential(*evaluator_spline, parameter);
    const auto& actual = evaluator.samples()[sample];
    require(std::abs(expected.position.x - actual.position.x) < 1.0e-6L &&
                std::abs(expected.position.y - actual.position.y) < 1.0e-6L &&
                std::abs(expected.tangent.x - actual.tangent.x) < 1.0e-6L &&
                std::abs(expected.tangent.y - actual.tangent.y) < 1.0e-6L &&
                std::abs(expected.second_derivative.x - actual.second_derivative.x) < 1.0e-6L &&
                std::abs(expected.second_derivative.y - actual.second_derivative.y) < 1.0e-6L,
            "prepared spline samples must match analytical on-demand differentials");
  }
  const auto early_spline = flux::route_global(early_spline_board, early_spline_config);
  const auto retained_geometric_corridors = static_cast<std::size_t>(
      std::count_if(early_spline.paths.begin(), early_spline.paths.end(), [](const auto& path) {
        return std::any_of(path.geometric_corridors.begin(), path.geometric_corridors.end(),
                           [](const auto& corridor) {
                             return !corridor.topology_cells.empty() &&
                                    corridor.lane_centerline.size() >= 2U &&
                                    corridor.minimum_physical_cross_section_width >=
                                        corridor.required_envelope_width &&
                                    corridor.lane_capacity > 0U;
                           });
      }));
  require(early_spline.complete() && early_spline.static_seed_connections == 1U &&
              early_spline.static_seed_searches == 2U &&
              early_spline.static_seed_candidates == 2U &&
              early_spline.iteration_zero_portfolios_evaluated == 4U &&
              early_spline.static_seed_corridor_nodes > 0U &&
              !early_spline.static_seed_budget_exhausted && retained_geometric_corridors > 0U,
          "the early hybrid-graph seed search must retain two exact static topology candidates "
          "around a blocker before considering a copper shove: " +
              std::to_string(early_spline.complete()) + "/" +
              std::to_string(early_spline.static_seed_searches) + "/" +
              std::to_string(early_spline.static_seed_candidates) + "/" +
              std::to_string(early_spline.static_seed_corridor_nodes) + "/" +
              std::to_string(early_spline.static_seed_budget_exhausted) +
              "/geometry=" + std::to_string(retained_geometric_corridors) +
              "/nav=" + std::to_string(early_spline.static_seed_navmesh_routes) + "/" +
              std::to_string(early_spline.static_seed_navmesh_failures) +
              "/traj=" + std::to_string(early_spline.static_seed_trajectory_accepted) + "/" +
              std::to_string(early_spline.static_seed_trajectory_rejected));

  auto curved_source = two_pad_board();
  curved_source.nets.push_back(Net{NetId{"arc-blocker-net"}, "ARC_BLOCKER", 0});
  curved_source.tracks.push_back(
      flux::TrackSegment{ObjectId{"arc-radius-blocker"}, NetId{"arc-blocker-net"}, LayerId{"F.Cu"},
                         mm(49.3, 40.39), mm(49.3, 40.7), Nanometres::from_mm(0.1), true});
  flux::GlobalRouteResult curved_global;
  curved_global.stop_reason = GlobalRouterStopReason::complete;
  flux::GlobalRoutePath curved_path;
  curved_path.net = NetId{"net-1"};
  curved_path.points = {{mm(20.0, 40.0), LayerId{"F.Cu"}},
                        {mm(50.0, 40.0), LayerId{"F.Cu"}},
                        {mm(50.0, 60.0), LayerId{"F.Cu"}},
                        {mm(80.0, 60.0), LayerId{"F.Cu"}},
                        {mm(80.0, 40.0), LayerId{"F.Cu"}}};
  curved_global.paths.push_back(std::move(curved_path));
  flux::DetailedRouterConfig curved_config;
  curved_config.pull_tight_45_degrees = false;
  curved_config.continuous_refinement = false;
  curved_config.circular_arc_refinement = true;
  curved_config.spline_minimum_bend_radius = Nanometres{0};
  curved_config.maximum_circular_arc_radius = Nanometres::from_mm(1.0);
  const auto curved = flux::materialize_detailed_route(curved_source, curved_global, curved_config);
  require(curved.complete() && curved.candidate && curved.fitted_circular_arcs > 0U &&
              curved.attempted_cubic_bspline_runs == 1U && curved.fitted_cubic_bspline_runs == 1U &&
              curved.adaptive_spline_tessellation_samples > curved.fitted_circular_arcs &&
              curved.unrounded_non_junction_bends == 0U && curved.copper_drc.ok(),
          "a bent route must be adaptively fitted from a cubic B-spline to connected DRC-clean "
          "native line/arc copper: " +
              std::to_string(curved.adaptive_spline_tessellation_samples) + "/" +
              std::to_string(curved.fitted_circular_arcs) +
              "/nav=" + std::to_string(curved.spline_navmesh_routes) + "/" +
              std::to_string(curved.spline_navmesh_failures) +
              "/traj=" + std::to_string(curved.spline_trajectory_accepted) + "/" +
              std::to_string(curved.spline_trajectory_initial_violations) + "->" +
              std::to_string(curved.spline_trajectory_final_violations) + "/" +
              curved.spline_navmesh_last_diagnostic + "/" + curved.diagnostic);

  const std::vector<Point> spline_polyline{mm(10.0, 10.0), mm(30.0, 10.0), mm(30.0, 30.0),
                                           mm(50.0, 30.0)};
  auto local_spline = flux::make_cubic_bspline_route(spline_polyline);
  require(local_spline.has_value(), "a bent polyline must initialize a cubic B-spline");
  const auto original_controls = local_spline->control_points;
  const auto original_midpoint = flux::evaluate_cubic_bspline(*local_spline, 0.5L);
  const auto moved_controls = flux::shove_cubic_bspline_control_corridor(
      *local_spline, 0.5L, flux::SplineVector{0.0L, 1.0L}, Nanometres::from_mm(0.5),
      Nanometres::from_mm(1.0));
  const auto moved_midpoint = flux::evaluate_cubic_bspline(*local_spline, 0.5L);
  require(moved_controls > 0U && moved_midpoint.y > original_midpoint.y &&
              local_spline->control_points[0] == original_controls[0] &&
              local_spline->control_points[1] == original_controls[1] &&
              local_spline->control_points[local_spline->control_points.size() - 2U] ==
                  original_controls[original_controls.size() - 2U] &&
              local_spline->control_points.back() == original_controls.back(),
          "basis-weighted spline shove must remain local and preserve endpoint tangent controls");
  const auto differential = flux::evaluate_cubic_bspline_differential(*local_spline, 0.5L);
  require(std::isfinite(differential.tangent.x) && std::isfinite(differential.tangent.y) &&
              differential.bend_radius_nm > 0.0L,
          "analytical B-spline derivatives must expose finite tangent and bend-radius evidence");
  flux::SplineTrajectoryOptimizationConfig elastic_config;
  elastic_config.minimum_bend_radius = Nanometres{0};
  elastic_config.maximum_corridor_displacement = Nanometres::from_mm(5.0);
  elastic_config.samples = 12U;
  elastic_config.maximum_iterations = 1U;
  elastic_config.maximum_candidate_checks = 4U;
  const auto prior_elastic_midpoint = flux::evaluate_cubic_bspline(*local_spline, 0.5L);
  const auto new_end =
      flux::SplineVector{local_spline->control_points.back().x,
                         local_spline->control_points.back().y + Nanometres::from_mm(1.0).value()};
  const auto elastic = flux::deform_cubic_bspline_elastic(
      *local_spline, local_spline->control_points.front(), new_end, elastic_config,
      [](flux::SplineVector) { return flux::SplineClearanceProbe{1'000'000.0L, {}, true}; });
  const auto elastic_midpoint = flux::evaluate_cubic_bspline(*local_spline, 0.5L);
  require(elastic.accepted && elastic.elastic_predeformed_controls > 0U &&
              local_spline->control_points.back() == new_end &&
              elastic_midpoint.y > prior_elastic_midpoint.y &&
              elastic_midpoint.y - prior_elastic_midpoint.y < Nanometres::from_mm(1.0).value(),
          "elastic endpoint deformation must preserve the endpoint constraint and attenuate its "
          "effect through nearby spline controls");

  Board pns_board;
  pns_board.source_revision = "pns-actionable-risk-fixture";
  pns_board.outline = rectangle(0.0, 0.0, 100.0, 80.0);
  pns_board.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0}};
  pns_board.nets = {{NetId{"net-1"}, "INTRUDER", 0},
                    {NetId{"pns-obstacle-net"}, "PNS_OBSTACLE", 0}};
  pns_board.tracks.push_back(flux::TrackSegment{ObjectId{"pns-intruder"}, NetId{"net-1"},
                                                LayerId{"F.Cu"}, mm(30.0, 50.0), mm(70.0, 50.0),
                                                Nanometres::from_mm(0.25), false});
  pns_board.tracks.push_back(flux::TrackSegment{ObjectId{"pns-obstacle"}, NetId{"pns-obstacle-net"},
                                                LayerId{"F.Cu"}, mm(20.0, 50.1), mm(80.0, 50.1),
                                                Nanometres::from_mm(0.25), false});
  const auto dynamic_pns_drc =
      flux::check_dynamic_copper_rules(pns_board, {ObjectId{"pns-intruder"}});
  require(!dynamic_pns_drc.ok() &&
              std::all_of(dynamic_pns_drc.violations.begin(), dynamic_pns_drc.violations.end(),
                          [](const auto& violation) {
                            return std::find(violation.object_ids.begin(),
                                             violation.object_ids.end(),
                                             "pns-intruder") != violation.object_ids.end();
                          }),
          "a dynamic local DRC probe must report exact violations involving its selected copper "
          "without rechecking unrelated source-only pairs");
  const flux::PnsShoveRequest pns_request{{"pns-intruder"},
                                          {"pns-obstacle"},
                                          {},
                                          mm(50.0, 50.0),
                                          0.0L,
                                          1.0L,
                                          Nanometres::from_mm(0.5),
                                          {}};
  const auto pns = flux::pns_push_and_shove(pns_board, pns_request);
  require(pns.accepted() && pns.moved_tracks == 1U && pns.local_splits == 1U &&
              pns.added_dogleg_tracks >= 3U && flux::check_copper_rules(*pns.candidate).ok(),
          "bounded PNS must move an unlocked parallel blocker with connectivity-preserving "
          "doglegs and exact DRC acceptance: " +
              pns.diagnostic);

  Board via_pns_board;
  via_pns_board.source_revision = "pns-via-transaction-risk-fixture";
  via_pns_board.outline = rectangle(0.0, 0.0, 100.0, 80.0);
  via_pns_board.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0},
                          {LayerId{"B.Cu"}, "B.Cu", LayerKind::copper, 1}};
  via_pns_board.nets = {{NetId{"via-intruder"}, "VIA_INTRUDER", 0},
                        {NetId{"via-shoved"}, "VIA_SHOVED", 0}};
  via_pns_board.tracks = {
      flux::TrackSegment{ObjectId{"via-intruder-track"}, NetId{"via-intruder"}, LayerId{"F.Cu"},
                         mm(30.0, 50.0), mm(70.0, 50.0), Nanometres::from_mm(0.25), false},
      flux::TrackSegment{ObjectId{"via-attached-front"}, NetId{"via-shoved"}, LayerId{"F.Cu"},
                         mm(20.0, 50.1), mm(50.0, 50.1), Nanometres::from_mm(0.25), false},
      flux::TrackSegment{ObjectId{"via-attached-back"}, NetId{"via-shoved"}, LayerId{"B.Cu"},
                         mm(50.0, 50.1), mm(80.0, 60.0), Nanometres::from_mm(0.25), false}};
  via_pns_board.vias = {flux::Via{ObjectId{"movable-via"},
                                  NetId{"via-shoved"},
                                  mm(50.0, 50.1),
                                  Nanometres::from_mm(0.8),
                                  Nanometres::from_mm(0.4),
                                  {LayerId{"F.Cu"}, LayerId{"B.Cu"}},
                                  false}};
  const flux::PnsShoveRequest via_pns_request{{"via-intruder-track"},
                                              {"movable-via"},
                                              {},
                                              mm(50.0, 50.0),
                                              0.0L,
                                              1.0L,
                                              Nanometres::from_mm(0.5),
                                              {}};
  const auto via_pns = flux::pns_push_and_shove(via_pns_board, via_pns_request);
  require(via_pns.accepted() && via_pns.moved_vias == 1U && via_pns.moved_tracks == 2U &&
              via_pns.candidate && via_pns.candidate->vias.front().position != mm(50.0, 50.1) &&
              flux::check_copper_rules(*via_pns.candidate).ok(),
          "bounded PNS must move an unlocked via and every attached straight-track endpoint as "
          "one exact transaction: " +
              via_pns.diagnostic);
  flux::GlobalRouteResult shove_global;
  shove_global.stop_reason = GlobalRouterStopReason::complete;
  flux::GlobalRoutingShoveProposal shove_proposal;
  shove_proposal.id = 0U;
  shove_proposal.intruding_net = NetId{"net-1"};
  shove_proposal.affected_nets = {NetId{"pns-obstacle-net"}};
  shove_proposal.replaced_tracks = {ObjectId{"pns-obstacle"}};
  shove_proposal.propagation_depth = pns.propagation_depth;
  const std::set<ObjectId> pns_source_ids{ObjectId{"pns-intruder"}, ObjectId{"pns-obstacle"}};
  for (const auto& track : pns.candidate->tracks) {
    if (!pns_source_ids.contains(track.id)) {
      shove_proposal.replacement_tracks.push_back(track);
    }
  }
  shove_global.selected_shove_proposals.push_back(std::move(shove_proposal));
  const auto shove_detail = flux::materialize_detailed_route(pns_board, shove_global);
  require(shove_detail.complete() && shove_detail.candidate && shove_detail.copper_drc.ok() &&
              std::none_of(shove_detail.candidate->tracks.begin(),
                           shove_detail.candidate->tracks.end(),
                           [](const auto& track) { return track.id == ObjectId{"pns-obstacle"}; }),
          "an iteration-zero PNS delta must remain actionable through detailed transactional "
          "materialization");

  curved_config.defer_saved_zone_fill_clearance_to_refill = true;
  const auto refillable_curve =
      flux::materialize_detailed_route(curved_source, curved_global, curved_config);
  require(refillable_curve.provisional_zone_refill_candidate() && refillable_curve.candidate &&
              refillable_curve.fitted_circular_arcs > 0U &&
              !refillable_curve.fallback_polyline_materialization,
          "a legal curved refill candidate must remain provisional without being mistaken for a "
          "failed curved transaction");
}

void integrated_placement_rejects_proxy_improvement_that_violates_copper_rules() {
  auto board = two_pad_board();
  board.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0}};
  board.footprints.front().locked = true;
  Keepout pad_keepout;
  pad_keepout.id = ObjectId{"pad-only-keepout"};
  pad_keepout.area = rectangle(25.0, 35.0, 40.0, 45.0);
  pad_keepout.layers = {LayerId{"F.Cu"}};
  pad_keepout.prohibit.pads = true;
  board.keepouts.push_back(std::move(pad_keepout));

  flux::IntegratedOptimizerConfig config;
  config.enable_global_placement = true;
  config.global_placer.iterations = 30U;
  config.global_placer.attraction_strength = 0.5L;
  config.global_placer.source_anchor_weight = 0.05L;
  config.global_placer.placement_grid = Nanometres::from_mm(5.0);
  config.global_router.grid_pitch = Nanometres::from_mm(5.0);
  config.max_passes = 1U;
  config.max_candidate_evaluations = 2U;

  const auto result = flux::optimize_integrated_placement_routing(board, config);
  require(result.complete() && result.global_placement && result.global_placement->complete() &&
              result.global_placement->after.hpwl_nm < result.global_placement->before.hpwl_nm,
          "fixture must generate a proxy-improving placement");
  require(!result.global_seed_accepted && result.accepted_moves.empty() && result.candidate &&
              result.candidate->footprints[1].position == board.footprints[1].position &&
              result.score.copper_violations == 0U,
          "exact copper acceptance must reject the proxy improvement and retain the source");

  auto continuation_config = config;
  continuation_config.enable_global_placement = false;
  continuation_config.optimize_routed_quality = true;
  continuation_config.max_candidate_evaluations = 1U;
  continuation_config.candidate_evaluation_offset = 2U;
  const auto continuation =
      flux::optimize_integrated_placement_routing(two_pad_board(), continuation_config);
  require(continuation.configured_candidate_evaluation_offset == 2U &&
              continuation.candidate_evaluations_skipped == 2U &&
              continuation.candidate_evaluations == 1U && continuation.accepted_moves.empty() &&
              continuation.stop_reason == flux::IntegratedOptimizerStopReason::evaluation_limit,
          "a deterministic candidate offset must skip only the certified prefix and must not "
          "claim a standalone local optimum");

  auto serial_wave_config = continuation_config;
  serial_wave_config.candidate_evaluation_offset = 0U;
  serial_wave_config.max_candidate_evaluations = 6U;
  serial_wave_config.max_candidate_evaluation_workers = 1U;
  serial_wave_config.max_full_routing_candidates_per_wave = 2U;
  const auto serial_wave =
      flux::optimize_integrated_placement_routing(two_pad_board(), serial_wave_config);
  auto parallel_wave_config = serial_wave_config;
  parallel_wave_config.max_candidate_evaluation_workers = 4U;
  const auto parallel_wave =
      flux::optimize_integrated_placement_routing(two_pad_board(), parallel_wave_config);
  const bool same_candidate_transforms =
      serial_wave.candidate && parallel_wave.candidate &&
      serial_wave.candidate->footprints.size() == parallel_wave.candidate->footprints.size() &&
      std::equal(serial_wave.candidate->footprints.begin(), serial_wave.candidate->footprints.end(),
                 parallel_wave.candidate->footprints.begin(), [](const auto& lhs, const auto& rhs) {
                   return lhs.id == rhs.id && lhs.position == rhs.position &&
                          lhs.rotation == rhs.rotation && lhs.side == rhs.side;
                 });
  require(serial_wave.score == parallel_wave.score &&
              serial_wave.stop_reason == parallel_wave.stop_reason && same_candidate_transforms &&
              serial_wave.accepted_moves.size() == parallel_wave.accepted_moves.size() &&
              serial_wave.candidate_evaluations == parallel_wave.candidate_evaluations &&
              serial_wave.candidate_evaluation_tasks_dispatched ==
                  parallel_wave.candidate_evaluation_tasks_dispatched &&
              serial_wave.candidate_evaluation_waves > 0U &&
              serial_wave.peak_candidate_evaluation_workers == 1U &&
              parallel_wave.peak_candidate_evaluation_workers > 1U,
          "candidate-wave reduction must retain the same exact winner and score independently of "
          "worker completion order");
}

void cancellation_and_kicad_export_boundaries_fail_closed() {
  const auto board = two_pad_board();
  std::stop_source stop;
  stop.request_stop();
  require(flux::route_global(board, {}, stop.get_token()).stop_reason ==
              GlobalRouterStopReason::cancelled,
          "routing must honor cancellation before mutation");
  require(flux::optimize_integrated_placement_routing(board, {}, stop.get_token()).stop_reason ==
              flux::IntegratedOptimizerStopReason::cancelled,
          "integrated optimization must honor cancellation before mutation");

  GlobalRouterConfig telemetry_route_config;
  telemetry_route_config.grid_pitch = Nanometres::from_mm(5.0);
  telemetry_route_config.max_iterations = 2U;
  const auto unobserved_route = flux::route_global(board, telemetry_route_config);
  flux::RouterTelemetryConfig telemetry_config;
  telemetry_config.queue_capacity = 64U;
  telemetry_config.publish_rate_hz = 1'000U;
  flux::RouterTelemetryService telemetry(telemetry_config);
  const auto observed_route = flux::route_global(board, telemetry_route_config, {}, &telemetry);
  telemetry.stop();
  const auto observed_statistics = telemetry.statistics();
  const auto observed_state = telemetry.state();
  const auto serialized_snapshot =
      flux::serialize_router_telemetry_snapshot(observed_state, observed_statistics);
  const auto decoded_snapshot = flux::telemetry::decode_events(serialized_snapshot);
  require(unobserved_route.stop_reason == observed_route.stop_reason &&
              unobserved_route.paths == observed_route.paths &&
              unobserved_route.unrouted_nets == observed_route.unrouted_nets &&
              unobserved_route.total_overflow == observed_route.total_overflow &&
              unobserved_route.total_cost == observed_route.total_cost &&
              observed_statistics.produced == observed_statistics.consumed &&
              observed_statistics.produced >= 4U && observed_statistics.dropped == 0U &&
              observed_state.routing_finished &&
              observed_state.observed_routes.size() == observed_route.paths.size() &&
              decoded_snapshot && decoded_snapshot.events.size() == 1U &&
              decoded_snapshot.events.front().type == flux::telemetry::EventType::snapshot &&
              decoded_snapshot.events.front().metrics.has_value() &&
              decoded_snapshot.events.front().metrics->route_cost == observed_state.route_cost &&
              decoded_snapshot.events.front().routes.size() == observed_route.paths.size() &&
              !decoded_snapshot.events.front().routes.front().points.empty(),
          "a disabled/enabled telemetry observer must preserve deterministic routing bytes and "
          "drain a viewer-decodable versioned final snapshot without becoming routing state");

  flux::RouterTelemetryEvent queued_event;
  queued_event.kind = flux::RouterTelemetryEventKind::net_queued;
  queued_event.net = NetId{"net-1"};
  queued_event.stable_id = "pair:gate";
  queued_event.priority_stage = "constrained";
  queued_event.explicit_priority = 7U;
  queued_event.terminal_portals = 1U;
  queued_event.corridor_alternatives = 2U;
  queued_event.via_transitions = 1U;
  queued_event.via_alternatives = 1U;
  queued_event.via_contention = 3U;
  queued_event.estimated_interference = 4U;
  queued_event.physical_envelope_nm = 500'000U;
  queued_event.topology_alternatives = 2U;
  queued_event.layer_alternatives = 1U;
  queued_event.priority_ordinal = 9U;
  const auto decoded_priority =
      flux::telemetry::decode_events(flux::serialize_router_telemetry_event(queued_event));
  require(decoded_priority && decoded_priority.events.size() == 1U &&
              decoded_priority.events.front().type == flux::telemetry::EventType::net_queued &&
              decoded_priority.events.front().active_net == std::optional<std::string>{"net-1"} &&
              decoded_priority.events.front().priorities.size() == 1U &&
              decoded_priority.events.front().priorities.front().group == "pair:gate" &&
              decoded_priority.events.front().priorities.front().staged_class == "constrained",
          "routing-priority telemetry must use the viewer protocol and retain stable Flux IDs");

  flux::RouterTelemetryConfig overflow_config;
  overflow_config.queue_capacity = 1U;
  overflow_config.publish_rate_hz = 1U;
  flux::RouterTelemetryService overflow_telemetry(overflow_config);
  std::vector<std::thread> producers;
  for (std::size_t producer = 0U; producer < 4U; ++producer) {
    producers.emplace_back([&overflow_telemetry, producer]() {
      for (std::size_t event = 0U; event < 2'000U; ++event) {
        flux::RouterTelemetryEvent observation;
        observation.kind = flux::RouterTelemetryEventKind::route_candidate_selected;
        observation.stable_id = std::to_string(producer) + ":" + std::to_string(event);
        static_cast<void>(overflow_telemetry.try_emit(std::move(observation)));
      }
    });
  }
  for (auto& producer : producers) {
    producer.join();
  }
  overflow_telemetry.stop();
  const auto overflow_statistics = overflow_telemetry.statistics();
  require(overflow_statistics.dropped > 0U &&
              overflow_statistics.produced == overflow_statistics.consumed &&
              overflow_statistics.produced + overflow_statistics.dropped == 8'000U,
          "the bounded MPSC telemetry queue must drop under contention without losing accepted "
          "events or waiting for its consumer");

  auto validation_board = board;
  flux::CopperZone zone;
  zone.id = ObjectId{"static-zone"};
  zone.net = NetId{"net-1"};
  zone.name = "Static zone";
  zone.fills = {flux::CopperZoneFill{
      LayerId{"F.Cu"}, rectangle(5.0, 5.0, 15.0, 15.0), {rectangle(7.0, 7.0, 9.0, 9.0)}}};
  validation_board.zones.push_back(std::move(zone));
  const auto baseline = flux::make_board_validation_baseline(validation_board);
  auto placement_candidate = validation_board;
  placement_candidate.footprints[1].position = mm(75.0, 40.0);
  const auto ordinary_validation = flux::validate_board(placement_candidate);
  const auto reused_validation = flux::validate_board_candidate(placement_candidate, baseline);
  const auto issue_identity = [](const flux::ValidationReport& lhs,
                                 const flux::ValidationReport& rhs) {
    if (lhs.issues.size() != rhs.issues.size()) {
      return false;
    }
    for (std::size_t index = 0U; index < lhs.issues.size(); ++index) {
      if (lhs.issues[index].severity != rhs.issues[index].severity ||
          lhs.issues[index].code != rhs.issues[index].code ||
          lhs.issues[index].object_ids != rhs.issues[index].object_ids) {
        return false;
      }
    }
    return true;
  };
  require(reused_validation.reused_static_zone_geometry &&
              issue_identity(ordinary_validation, reused_validation.validation),
          "an exact unchanged-zone baseline must preserve canonical candidate validation");
  placement_candidate.zones.front().fills.front().holes.front().vertices.front() = mm(4.0, 4.0);
  const auto changed_zone_validation =
      flux::validate_board_candidate(placement_candidate, baseline);
  require(!changed_zone_validation.reused_static_zone_geometry &&
              std::any_of(changed_zone_validation.validation.issues.begin(),
                          changed_zone_validation.validation.issues.end(),
                          [](const auto& issue) { return issue.code == "invalid_zone_fill_hole"; }),
          "changed static geometry must fall back to full validation and retain exact rejection");

  const auto contents = load_minimal_kicad_fixture();
  const auto imported = flux::io::import_kicad_pcb(contents);
  require(imported.strict(), "the canonical KiCad fixture must import without loss");
  auto rotated_pad_contents = contents;
  const auto footprint_at = rotated_pad_contents.find("(at 80 40 180)");
  require(footprint_at != std::string::npos, "rotation fixture footprint must remain identifiable");
  rotated_pad_contents.replace(footprint_at, std::string{"(at 80 40 180)"}.size(), "(at 80 40 90)");
  const auto last_pad = rotated_pad_contents.rfind("(pad \"1\" smd circle");
  require(last_pad != std::string::npos, "rotation fixture pad must remain identifiable");
  rotated_pad_contents.replace(last_pad + std::string{"(pad \"1\" smd "}.size(),
                               std::string{"circle"}.size(), "rect");
  const auto last_pad_at = rotated_pad_contents.find("(at 0 0)", last_pad);
  require(last_pad_at != std::string::npos,
          "rotation fixture pad position must remain identifiable");
  rotated_pad_contents.replace(last_pad_at, std::string{"(at 0 0)"}.size(), "(at 0 0 90)");
  const auto last_pad_size = rotated_pad_contents.find("(size 2 2)", last_pad);
  require(last_pad_size != std::string::npos, "rotation fixture pad size must remain identifiable");
  rotated_pad_contents.replace(last_pad_size, std::string{"(size 2 2)"}.size(), "(size 1 3)");
  const auto rotated_pad_import = flux::io::import_kicad_pcb(rotated_pad_contents);
  require(rotated_pad_import.strict() &&
              rotated_pad_import.board->footprints.back().pads.front().rotation == Microdegrees{} &&
              flux::point_to_pad_copper_distance_nm(
                  mm(81.4, 40.0), flux::copper_pad_shapes(*rotated_pad_import.board).back()) ==
                  0.0L,
          "a KiCad board-file pad angle must be converted to local orientation before the "
          "footprint transform is applied exactly once");
  const auto priority_template = flux::io::make_placement_move_priorities_template(*imported.board);
  const auto priorities =
      flux::io::load_placement_move_priorities(*imported.board, priority_template);
  require(priorities.ok() && priorities.policy && priorities.policy->footprints.size() == 2U &&
              std::all_of(
                  priorities.policy->footprints.begin(), priorities.policy->footprints.end(),
                  [](const auto& assignment) {
                    return assignment.priority_class == flux::PlacementMovePriorityClass::connector;
                  }),
          "the source-bound priority template must completely classify J prefixes connector-last");
  const auto suggested_priorities =
      flux::io::make_suggested_placement_move_priorities(*imported.board);
  require(suggested_priorities.source_revision == imported.board->source_revision &&
              suggested_priorities.order == flux::default_placement_move_priority_order() &&
              suggested_priorities.footprints == priorities.policy->footprints,
          "in-memory prefix suggestions must match the deterministic review template exactly");
  auto stale_priority_board = *imported.board;
  stale_priority_board.source_revision += ":stale";
  require(flux::io::load_placement_move_priorities(stale_priority_board, priority_template)
              .has_errors(),
          "a placement-move priority sidecar must reject a stale source revision");
  flux::IntegratedOptimizerConfig stale_priority_config;
  stale_priority_config.placement_move_priorities = *priorities.policy;
  stale_priority_config.placement_move_priorities->source_revision += ":stale";
  const auto stale_priority_result =
      flux::optimize_integrated_placement_routing(*imported.board, stale_priority_config);
  require(
      stale_priority_result.stop_reason == flux::IntegratedOptimizerStopReason::invalid_config &&
          stale_priority_result.diagnostic.find("different source revision") != std::string::npos,
      "the core optimizer must retain and enforce the policy's source binding");
  auto moved = *imported.board;
  moved.footprints.front().position = mm(25.0, 40.0);
  require(flux::io::export_kicad_candidate(contents, *imported.board, moved).ok(),
          "a supported source-bound move must export");

  const auto stale = flux::io::export_kicad_candidate(contents + "\n", *imported.board, moved);
  require(!stale.ok() && std::any_of(stale.diagnostics.begin(), stale.diagnostics.end(),
                                     [](const auto& diagnostic) {
                                       return diagnostic.code == "stale_source_snapshot";
                                     }),
          "changed source bytes must reject a stale candidate");

  auto unsupported = *imported.board;
  unsupported.outline.vertices.front() = mm(1.0, 1.0);
  const auto rejected = flux::io::export_kicad_candidate(contents, *imported.board, unsupported);
  require(!rejected.ok() && std::any_of(rejected.diagnostics.begin(), rejected.diagnostics.end(),
                                        [](const auto& diagnostic) {
                                          return diagnostic.code == "unsupported_design_change";
                                        }),
          "unsupported board changes must be rejected rather than silently dropped");

  const auto tentative_contents = load_tentative_placement_fixture();
  flux::io::KicadImportOptions tentative_options;
  tentative_options.derive_tentative_placement_envelopes = true;
  const auto tentative = flux::io::import_kicad_pcb(tentative_contents, tentative_options);
  require(
      tentative.strict() && tentative.board && tentative.board->footprints.size() == 2U &&
          std::all_of(tentative.board->footprints.begin(), tentative.board->footprints.end(),
                      [](const Footprint& footprint) {
                        return footprint.has_tentative_placement_envelope() &&
                               !footprint.has_exact_placement_envelope();
                      }) &&
          tentative.board->footprints.front().courtyard.vertices.front() == mm(-5.25, -4.25),
      "opt-in tentative geometry must include Fab/Silk bounds and retain provisional provenance");

  flux::IntegratedOptimizerConfig tentative_config;
  const auto tentative_rejected =
      flux::optimize_integrated_placement_routing(*tentative.board, tentative_config);
  require(!tentative_rejected.complete() &&
              tentative_rejected.diagnostic.find("explicit tentative envelope opt-in") !=
                  std::string::npos,
          "integrated placement must reject tentative geometry by default");

  tentative_config.allow_tentative_placement_envelopes = true;
  tentative_config.placement_move_priorities =
      flux::io::make_suggested_placement_move_priorities(*tentative.board);
  tentative_config.suggested_placement_move_priorities = true;
  tentative_config.enable_global_placement = true;
  tentative_config.max_passes = 1U;
  tentative_config.max_candidate_evaluations = 2U;
  const auto tentative_accepted =
      flux::optimize_integrated_placement_routing(*tentative.board, tentative_config);
  require(tentative_accepted.complete() && tentative_accepted.tentative_placement_envelopes == 2U &&
              tentative_accepted.human_placement_review_required &&
              tentative_accepted.configured_placement_move_priorities &&
              tentative_accepted.suggested_placement_move_priorities &&
              tentative_accepted.placement_move_priority_review_required &&
              tentative_accepted.configured_max_passes == 1U &&
              tentative_accepted.configured_max_candidate_evaluations == 2U,
          "explicit tentative placement must remain feasible and require post-process review");

  auto overlapping_tentative = *tentative.board;
  overlapping_tentative.footprints.back().position =
      overlapping_tentative.footprints.front().position;
  require(!flux::validate_board(overlapping_tentative).ok(),
          "the tentative legalization fixture must begin with an illegal overlap");
  flux::GlobalPlacerConfig tentative_placer_config;
  tentative_placer_config.iterations = 1U;
  const auto overlapping_rejected =
      flux::place_global(overlapping_tentative, tentative_placer_config);
  require(!overlapping_rejected.complete(),
          "global placement must reject an invalid tentative source without explicit opt-in");
  tentative_placer_config.allow_tentative_placement_envelopes = true;
  auto structurally_invalid_tentative = overlapping_tentative;
  structurally_invalid_tentative.footprints.front().pads.front().net = flux::NetId{"missing-net"};
  const auto structurally_invalid_result =
      flux::place_global(structurally_invalid_tentative, tentative_placer_config);
  require(!structurally_invalid_result.complete() &&
              !structurally_invalid_result.repairable_tentative_source_accepted,
          "tentative source legalization must not whitelist an unrelated structural error");
  const auto overlapping_legalized =
      flux::place_global(overlapping_tentative, tentative_placer_config);
  require(overlapping_legalized.complete() && overlapping_legalized.candidate &&
              overlapping_legalized.repairable_tentative_source_accepted &&
              overlapping_legalized.repairable_tentative_source_errors > 0U &&
              flux::validate_board(*overlapping_legalized.candidate).ok(),
          "explicit tentative placement must repair source overlap before publishing a proposal");

  auto tentatively_moved = *tentative.board;
  tentatively_moved.footprints.front().position = mm(25.0, 40.0);
  require(flux::io::export_kicad_candidate(tentative_contents, *tentative.board, tentatively_moved)
              .ok(),
          "non-serialized tentative geometry must round-trip against the unchanged KiCad source");
}

} // namespace

int main() {
  const std::vector<TestCase> tests{
      {"units_remain_exact_and_saturating", units_remain_exact_and_saturating},
      {"dimensioned_pathfinder_pressure_preserves_units_and_group_fairness",
       dimensioned_pathfinder_pressure_preserves_units_and_group_fairness},
      {"physical_checkpoint_ordering_is_feasibility_first_and_fail_closed",
       physical_checkpoint_ordering_is_feasibility_first_and_fail_closed},
      {"global_to_detailed_route_is_transactional_and_capacity_gated",
       global_to_detailed_route_is_transactional_and_capacity_gated},
      {"obstacle_aware_diagonals_and_curves_remain_capacity_and_drc_gated",
       obstacle_aware_diagonals_and_curves_remain_capacity_and_drc_gated},
      {"integrated_placement_rejects_proxy_improvement_that_violates_copper_rules",
       integrated_placement_rejects_proxy_improvement_that_violates_copper_rules},
      {"cancellation_and_kicad_export_boundaries_fail_closed",
       cancellation_and_kicad_export_boundaries_fail_closed},
  };

  std::size_t passed = 0U;
  for (const auto& test : tests) {
    try {
      test.run();
      ++passed;
      std::cout << "PASS " << test.name << '\n';
    } catch (const std::exception& error) {
      std::cerr << "FAIL " << test.name << ": " << error.what() << '\n';
    }
  }
  std::cout << passed << '/' << tests.size() << " tests passed\n";
  return passed == tests.size() ? 0 : 1;
}
