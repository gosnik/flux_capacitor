#include "flux/core/copper_geometry.hpp"
#include "flux/core/detailed_router.hpp"
#include "flux/core/drc.hpp"
#include "flux/core/geometry.hpp"
#include "flux/core/global_router.hpp"
#include "flux/core/hydraulic_path.hpp"
#include "flux/core/integrated_optimizer.hpp"
#include "flux/core/navmesh_route.hpp"
#include "flux/core/pns_router.hpp"
#include "flux/core/regional_router.hpp"
#include "flux/core/router_telemetry.hpp"
#include "flux/core/routing_compute.hpp"
#include "flux/core/routing_rules.hpp"
#include "flux/core/spline_route.hpp"
#include "flux/io/kicad_export.hpp"
#include "flux/io/kicad_import.hpp"
#include "flux/io/kicad_netclass_snapshot.hpp"
#include "flux/io/kicad_refill.hpp"
#include "flux/io/placement_move_priorities.hpp"
#include "flux/io/work_plan.hpp"
#include "flux/io/work_plan_executor.hpp"
#include "flux/io/work_plan_router.hpp"
#include "flux/telemetry/event.hpp"

#include <algorithm>
#include <barrier>
#include <chrono>
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

void source_bound_board_rules_control_geometry_and_hole_clearance() {
  auto source = two_pad_board();
  const std::string snapshot = R"((flux_kicad_netclass_snapshot
  (version 3)
  (source_revision "risk-fixture")
  (project_revision "sha256:project-fixture")
  (api_version "10.0.4-test")
  (board_rules
    (minimum_clearance_nm 250000)
    (minimum_track_width_nm 300000)
    (minimum_via_diameter_nm 400000)
    (minimum_via_annular_width_nm 150000)
    (minimum_through_drill_nm 300000)
    (copper_to_edge_clearance_nm 350000)
    (hole_clearance_nm 200000)
    (hole_to_hole_clearance_nm 250000)
  )
  (class
    (name "Default")
    (source_priority 2147483647)
    (clearance_nm 100000)
    (track_width_nm 200000)
    (via_diameter_nm 400000)
    (via_drill_nm 100000)
    (diff_pair_track_width_nm 200000)
    (diff_pair_gap_nm 150000)
    (diff_pair_via_gap_nm 200000)
  )
  (net (name "SIGNAL") (classes))
))";
  const auto applied = flux::io::apply_kicad_netclass_snapshot(source, snapshot);
  require(applied.ok() && applied.applied_board_rules && applied.board &&
              applied.project_revision == "sha256:project-fixture",
          "a version-3 KiCad snapshot must atomically bind project fabrication rules");
  const auto resolved = flux::resolve_net_routing_rules(*applied.board, NetId{"net-1"});
  require(resolved.clearance == Nanometres{250000} &&
              resolved.track_width == Nanometres{300000} &&
              resolved.via_drill == Nanometres{300000} &&
              resolved.via_diameter == Nanometres{600000} &&
              applied.board->rules.copper_to_edge_clearance == Nanometres{350000},
          "project floors must raise generated copper dimensions without rewriting class values");

  auto board = *applied.board;
  board.vias = {
      {ObjectId{"hole-a"}, NetId{"net-1"}, mm(40.0, 40.0), Nanometres{600000},
       Nanometres{300000}, {LayerId{"F.Cu"}, LayerId{"B.Cu"}}, false},
      {ObjectId{"hole-b"}, NetId{"net-1"}, mm(40.5, 40.0), Nanometres{600000},
       Nanometres{300000}, {LayerId{"F.Cu"}, LayerId{"B.Cu"}}, false}};
  const auto whole = flux::check_copper_rules(board);
  const auto has_hole_violation = [](const auto& report) {
    return std::any_of(report.violations.begin(), report.violations.end(), [](const auto& item) {
      return item.kind == flux::CopperViolationKind::hole_to_hole_clearance;
    });
  };
  require(has_hole_violation(whole) &&
              has_hole_violation(
                  flux::check_dynamic_copper_rules(board, {ObjectId{"hole-b"}})),
          "hole-to-hole clearance must remain mechanical across same-net vias in whole and "
          "dynamic exact DRC");

  const flux::DrillShape oval{ObjectId{"oval"}, NetId{}, mm(0.0, 0.0), Nanometres{1'000'000},
                              Nanometres{400'000}, Microdegrees{0},
                              flux::DrillOwnerKind::pad};
  const flux::DrillShape circle{ObjectId{"circle"}, NetId{}, mm(0.7, 0.0), Nanometres{300'000},
                                Nanometres{300'000}, Microdegrees{0},
                                flux::DrillOwnerKind::via};
  require(std::abs(flux::drill_to_drill_boundary_distance_nm(oval, circle) - 50'000.0L) < 1.0L,
          "oval drill spacing must use the exact capsule boundary rather than a bounding circle");
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

  flux::GlobalRouteResult physical_history_route;
  flux::RoutingVoronoiResourceUsage physical_history;
  physical_history.id = 5U;
  physical_history.layer = LayerId{"F.Cu"};
  physical_history.start = mm(1.0, 1.0);
  physical_history.end = mm(2.0, 1.0);
  physical_history.demand_nm = 1'330'000U;
  physical_history.capacity_nm = 700'000U;
  physical_history.nets = {first};
  physical_history_route.voronoi_resource_usage.push_back(physical_history);
  const auto physical_history_diagnoses = flux::probe_global_routability(physical_history_route);
  require(physical_history_diagnoses.size() == 1U &&
              physical_history_diagnoses.front().severity == 13U &&
              physical_history_diagnoses.front().clearance_deficit == Nanometres{} &&
              physical_history_diagnoses.front().planned_capacity == 1'330'000U &&
              physical_history_diagnoses.front().observed_capacity == 700'000U,
          "physical-medial overflow must enter routability history in clearance quanta, not raw "
          "nanometres that overwhelm negotiated PathFinder cost");

  flux::GlobalRouteResult lane_transition_route;
  flux::RoutingVoronoiResourceUsage first_lane_resource;
  first_lane_resource.id = 7U;
  first_lane_resource.layer = LayerId{"F.Cu"};
  first_lane_resource.start = mm(1.0, 1.0);
  first_lane_resource.end = mm(2.0, 1.0);
  first_lane_resource.nets = {first};
  flux::RoutingVoronoiResourceUsage second_lane_resource = first_lane_resource;
  second_lane_resource.id = 8U;
  second_lane_resource.start = mm(2.0, 1.0);
  second_lane_resource.end = mm(2.0, 2.0);
  second_lane_resource.nets = {second};
  lane_transition_route.voronoi_resource_usage = {first_lane_resource, second_lane_resource};
  flux::RoutingVoronoiLaneTransition rejected_transition;
  rejected_transition.first_resource_id = first_lane_resource.id;
  rejected_transition.second_resource_id = second_lane_resource.id;
  rejected_transition.net = first;
  rejected_transition.layer = LayerId{"F.Cu"};
  rejected_transition.medial_junction = mm(2.0, 1.0);
  rejected_transition.first_lane_endpoint = mm(1.8, 1.0);
  rejected_transition.second_lane_endpoint = mm(2.0, 1.2);
  lane_transition_route.voronoi_lane_transitions.push_back(rejected_transition);
  const auto lane_transition_diagnoses = flux::probe_global_routability(lane_transition_route);
  require(lane_transition_diagnoses.size() == 1U &&
              lane_transition_diagnoses.front().suggested_relaxation ==
                  flux::RoutabilityRelaxation::change_lane_order &&
              !lane_transition_diagnoses.front().contributes_to_history &&
              lane_transition_diagnoses.front().resource.nets == std::vector{first, second},
          "an unaccepted packed-lane bridge must expand incremental feedback to every physical "
          "occupant of its two medial resources, including a blocker without a shadow overlap");

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

  GlobalRouterConfig fine_planar_lattice;
  fine_planar_lattice.grid_pitch = Nanometres::from_mm(0.5);
  fine_planar_lattice.via_site_pitch = Nanometres{0};
  fine_planar_lattice.max_iterations = 1U;
  const auto derived_via_lattice = flux::route_global(board, fine_planar_lattice);
  require(derived_via_lattice.stop_reason != GlobalRouterStopReason::invalid_input &&
              derived_via_lattice.configured_via_site_pitch == Nanometres{0},
          "an omitted via pitch must derive a rule-safe coarse sublattice instead of invalidating "
          "a finer planar route grid");

  invalid = {};
  invalid.grid_pitch = Nanometres::from_mm(5.0);
  invalid.capacity_reserve_per_mille = 1'001U;
  const auto invalid_capacity_reserve = flux::route_global(board, invalid);
  require(invalid_capacity_reserve.stop_reason == GlobalRouterStopReason::invalid_input &&
              invalid_capacity_reserve.diagnostic.find("capacity reserve") != std::string::npos,
          "capacity-reserve pressure must remain bounded and fail closed above one full resource");

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
  rejected_detail.candidate->tracks.push_back(
      flux::TrackSegment{ObjectId{"clean-feedback-track"}, NetId{"net-1"}, LayerId{"F.Cu"},
                         mm(10.0, 10.0), mm(20.0, 20.0), Nanometres::from_mm(0.2), false});
  flux::accumulate_clean_copper_knowledge(knowledge, rejected_detail);
  flux::accumulate_clean_copper_knowledge(knowledge, rejected_detail);
  require(knowledge.success_history.size() == 1U &&
              knowledge.success_history.front().observations == 2U &&
              knowledge.success_history.front().strength == 2U &&
              knowledge.success_history.front().start == mm(10.0, 10.0) &&
              knowledge.success_history.front().end == mm(20.0, 20.0),
          "clean quantized copper must accumulate a bounded directional success prior separately "
          "from failure pressure");
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
              paired_route.initial_pathfinder_order ==
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
          "complete atomic priority-wave/layer-sequence candidate and remain contiguous in the "
          "first negotiated PathFinder pass, independent of worker "
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
  // Two ordinary seeds plus the mandatory hydraulic family; RRT is disabled below.
  hard_capacity_config.max_static_seed_candidates_per_connection = 3U;
  hard_capacity_config.use_static_seed_rrt_connect_alternatives = false;
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
          hard_capacity_route.hard_capacity_soft_cut_probes > 0U &&
          hard_capacity_route.hard_capacity_soft_cut_routes > 0U &&
          hard_capacity_route.hard_capacity_soft_cut_blockers > 0U &&
          hard_capacity_route.hard_capacity_joint_ripup_attempts == 0U &&
          hard_capacity_route.configured_rollback_failed_hard_capacity_ripup &&
          hard_capacity_route.hard_capacity_failed_retry_rollbacks == 0U &&
          hard_capacity_route.configured_defer_hard_capacity_ripup_victims_to_pass_tail &&
          hard_capacity_route.hard_capacity_tail_deferred_victims == 2U &&
          hard_capacity_route.hard_capacity_ripup_attempts == 2U &&
          hard_capacity_route.hard_capacity_ripped_nets == 2U &&
          hard_capacity_route.hard_capacity_ripup_recoveries == 2U &&
          hard_capacity_route.hard_capacity_ripup_retained_recoveries == 1U &&
          hard_capacity_route.configured_capacity_reserve_per_mille ==
              hard_capacity_config.capacity_reserve_per_mille &&
          hard_capacity_route.configured_capacity_reserve_cost ==
              hard_capacity_config.capacity_reserve_cost &&
          hard_capacity_route.capacity_reserve_penalty_events > 0U &&
          hard_capacity_route.capacity_reserve_penalty > 0U &&
          hard_capacity_route.configured_former_route_memory_cost > 0U &&
          hard_capacity_route.former_route_memory_ripped_paths > 0U &&
          hard_capacity_route.former_route_memory_edges > 0U &&
          hard_capacity_route.former_route_memory_nodes > 0U &&
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
          std::to_string(hard_capacity_route.paths.size()) + "/" +
          std::to_string(hard_capacity_route.capacity_reserve_penalty_events) + "/" +
          std::to_string(hard_capacity_route.corridor_capacity_reserve_penalty_events) + "/" +
          std::to_string(hard_capacity_route.physical_capacity_reserve_penalty_events) + "/" +
          std::to_string(hard_capacity_route.clearance_shadow_reserve_penalty_events) + "/" +
          std::to_string(hard_capacity_route.capacity_reserve_penalty) + "/" +
          hard_capacity_route.diagnostic);

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

  auto unfilled_source = refill_source;
  unfilled_source.zones.front().fills.clear();
  unfilled_source.zones.front().requires_refill = true;
  const auto unfilled_validation = flux::validate_board(unfilled_source);
  const auto unfilled_route = flux::route_global(unfilled_source, refillable_config);
  const auto unfilled_default_detail =
      flux::materialize_detailed_route(unfilled_source, unfilled_route);
  const auto unfilled_provisional_detail = flux::materialize_detailed_route(
      unfilled_source, unfilled_route, refillable_detail_config);
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
  require(unfilled_validation.ok() && unfilled_validation.warning_count() == 1U &&
              unfilled_route.complete() &&
              unfilled_default_detail.stop_reason ==
                  flux::DetailedRouterStopReason::invalid_input &&
              !unfilled_default_detail.candidate &&
              unfilled_provisional_detail.provisional_zone_refill_candidate() &&
              unfilled_provisional_detail.candidate &&
              unfilled_provisional_detail.connectivity.open_connections == 0U &&
              unfilled_provisional_detail.copper_drc.ok(),
          "an unfilled source zone must remain a valid immutable marker while only the explicit "
          "refill workflow can publish provisional copper");

  auto contradictory_zone_source = unfilled_source;
  contradictory_zone_source.zones.front().fills = {
      flux::CopperZoneFill{LayerId{"F.Cu"}, rectangle(45.0, 0.0, 55.0, 80.0), {}}};
  const auto contradictory_validation = flux::validate_board(contradictory_zone_source);
  require(!contradictory_validation.ok() &&
              std::any_of(contradictory_validation.issues.begin(),
                          contradictory_validation.issues.end(), [](const auto& issue) {
                            return issue.code == "contradictory_zone_refill_state";
                          }),
          "saved physical copper and a requires-refill marker must be mutually exclusive");
}

void quintic_ph_geometry_preserves_invariants_and_exact_offsets() {
  using flux::SplineVector;
  const auto mm_value = [](long double value) {
    return value * static_cast<long double>(Nanometres::from_mm(1.0).value());
  };
  const std::vector<SplineVector> parameter_points{
      {0.0L, 0.0L}, {mm_value(1.0L), 0.0L}, {mm_value(5.0L), 0.0L}};
  const auto parameters = flux::select_quintic_ph_parameters(parameter_points, 0.5L);
  require(parameters.size() == parameter_points.size() && parameters.front() == 0.0L &&
              parameters.back() == 1.0L && parameters[1] > 0.0L && parameters[1] < 0.5L,
          "centripetal parameter selection must retain order and de-emphasize a long chord");

  const auto segment = flux::interpolate_quintic_ph_segment(
      {0.0L, 0.0L}, {mm_value(10.0L), mm_value(4.0L)}, {mm_value(9.0L), 0.0L},
      {mm_value(7.0L), mm_value(4.0L)});
  require(segment.has_value() && segment->bezier.front() == SplineVector{0.0L, 0.0L} &&
              std::hypotl(segment->bezier.back().x - mm_value(10.0L),
                          segment->bezier.back().y - mm_value(4.0L)) < 0.5L &&
              segment->cusp_count == 0U &&
              flux::quintic_ph_arc_length(*segment) >= std::hypotl(mm_value(10.0L), mm_value(4.0L)),
          "PH Hermite interpolation must prefer a cusp-free endpoint-exact branch and expose "
          "exact polynomial length");
  for (std::size_t coefficient = 0U; coefficient < segment->hodograph_power.size(); ++coefficient) {
    const auto scale = static_cast<long double>(coefficient + 1U);
    require(
        std::hypotl(segment->position_power[coefficient + 1U].x * scale -
                        segment->hodograph_power[coefficient].x,
                    segment->position_power[coefficient + 1U].y * scale -
                        segment->hodograph_power[coefficient].y) < 1.0e-3L,
        "cached position power coefficients must differentiate to cached w-squared coefficients");
  }
  for (std::size_t coefficient = 0U; coefficient < segment->speed_power.size(); ++coefficient) {
    require(std::abs(segment->arc_length_power[coefficient + 1U] *
                         static_cast<long double>(coefficient + 1U) -
                     segment->speed_power[coefficient]) < 1.0e-3L,
            "cached exact arc-length power coefficients must differentiate to cached speed");
  }
  for (const auto parameter : {0.0L, 0.17L, 0.5L, 0.83L, 1.0L}) {
    const auto offset = flux::make_quintic_ph_offset(*segment, mm_value(0.25L));
    const auto base_route = flux::make_quintic_ph_route({mm(0.0, 0.0), mm(10.0, 4.0)});
    require(offset.regular && base_route.has_value(),
            "a regular PH quintic must have a rational offset");
    // Evaluate the same authoritative segment directly through a one-segment route.
    auto route = *base_route;
    route.segments = {*segment};
    route.segments.front().route_parameter_start = 0.0L;
    route.segments.front().route_parameter_end = 1.0L;
    const auto point = flux::evaluate_quintic_ph(route, parameter);
    const auto differential = flux::evaluate_quintic_ph_differential(route, parameter);
    const auto evaluate_vector_power = [parameter](const auto& coefficients) {
      SplineVector value{};
      for (auto iterator = coefficients.rbegin(); iterator != coefficients.rend(); ++iterator) {
        value.x = value.x * parameter + iterator->x;
        value.y = value.y * parameter + iterator->y;
      }
      return value;
    };
    const auto preimage = evaluate_vector_power(segment->preimage_power);
    const auto hodograph = evaluate_vector_power(segment->hodograph_power);
    require(
        std::hypotl(hodograph.x - (preimage.x * preimage.x - preimage.y * preimage.y),
                    hodograph.y - 2.0L * preimage.x * preimage.y) < 1.0e-3L &&
            std::hypotl(differential.tangent.x - hodograph.x,
                        differential.tangent.y - hodograph.y) < 1.0e-3L,
        "hot differential evaluation must read the prepared quadratic-w/quartic-w-squared cache");
    const auto offset_point = flux::evaluate_quintic_ph_offset(offset, parameter);
    require(std::abs(std::hypotl(offset_point.x - point.x, offset_point.y - point.y) -
                     mm_value(0.25L)) < 1.0e-4L,
            "degree-nine rational PH offset evaluation must preserve exact signed distance");
    require(point.x >= segment->bounds_minimum.x - 1.0e-3L &&
                point.x <= segment->bounds_maximum.x + 1.0e-3L &&
                point.y >= segment->bounds_minimum.y - 1.0e-3L &&
                point.y <= segment->bounds_maximum.y + 1.0e-3L,
            "cached algebraic PH extrema must bound every evaluated curve position");
  }
  const auto extrema = flux::quintic_ph_curvature_extrema(*segment);
  require(extrema.size() == segment->curvature_extremum_count && extrema.size() >= 2U &&
              extrema.front().parameter == 0.0L && extrema.back().parameter == 1.0L,
          "curvature analysis must include endpoints and every isolated interior extremum");

  const std::vector<flux::SplineSample> shallow_samples{
      {0.0L, {mm_value(53.0L), mm_value(66.31L)}},
      {0.5L, {mm_value(45.009929L), mm_value(65.546615L)}},
      {1.0L, {mm_value(39.51749L), mm_value(64.989416L)}}};
  const auto shallow_pieces = flux::fit_spline_samples_to_lines_and_arcs(
      shallow_samples, Nanometres::from_mm(0.02), 16U, 16U);
  require(shallow_pieces.size() == 1U &&
              shallow_pieces.front().kind == flux::SplineFittedPieceKind::segment,
          "near-collinear PH samples must collapse to one tolerance-bounded line instead of a "
          "huge-radius KiCad arc");

  auto chain =
      flux::make_quintic_ph_route({mm(0.0, 0.0), mm(8.0, 0.0), mm(14.0, 5.0), mm(22.0, 5.0)});
  require(chain.has_value() && chain->tangent_continuous && chain->segments.size() == 3U,
          "multi-segment PH interpolation must construct a C1 chain");
  for (std::size_t join = 1U; join < chain->segments.size(); ++join) {
    const auto& left = chain->segments[join - 1U].bezier;
    const auto& right = chain->segments[join].bezier;
    const auto left_span = chain->segments[join - 1U].route_parameter_end -
                           chain->segments[join - 1U].route_parameter_start;
    const auto right_span =
        chain->segments[join].route_parameter_end - chain->segments[join].route_parameter_start;
    const auto left_tangent = SplineVector{5.0L * (left[5].x - left[4].x) / left_span,
                                           5.0L * (left[5].y - left[4].y) / left_span};
    const auto right_tangent = SplineVector{5.0L * (right[1].x - right[0].x) / right_span,
                                            5.0L * (right[1].y - right[0].y) / right_span};
    require(std::hypotl(left_tangent.x - right_tangent.x, left_tangent.y - right_tangent.y) < 0.1L,
            "adjacent PH Bézier segments must share their endpoint derivative exactly");
  }
  const auto inflections =
      std::accumulate(chain->segments.begin(), chain->segments.end(), std::size_t{0U},
                      [](auto count, const auto& candidate) {
                        return count + flux::quintic_ph_inflections(candidate).size();
                      });
  require(inflections <= chain->segments.size(),
          "isolated PH inflections must be explicitly bounded rather than inferred from samples");

  const auto horizontal = flux::interpolate_quintic_ph_segment(
      {0.0L, 0.0L}, {mm_value(10.0L), 0.0L}, {mm_value(10.0L), 0.0L}, {mm_value(10.0L), 0.0L});
  const auto vertical = flux::interpolate_quintic_ph_segment(
      {mm_value(5.0L), -mm_value(5.0L)}, {mm_value(5.0L), mm_value(5.0L)}, {0.0L, mm_value(10.0L)},
      {0.0L, mm_value(10.0L)});
  require(horizontal && vertical, "orthogonal PH intersection fixtures must interpolate");
  flux::PHOffsetIntersectionStats offset_stats;
  const auto intersections = flux::intersect_quintic_ph_offsets(
      flux::make_quintic_ph_offset(*horizontal, 0.0L),
      flux::make_quintic_ph_offset(*vertical, 0.0L), Nanometres{10}, 18U, 64U, 1'000'000U,
      &offset_stats);
  require(std::any_of(intersections.begin(), intersections.end(),
                      [&](const auto& intersection) {
                        return std::hypotl(intersection.position.x - mm_value(5.0L),
                                           intersection.position.y) <= 10.0L;
                      }),
          "bounded rational-offset intersection must recover an exact transverse crossing");
  require(offset_stats.subdivision_nodes > 0U && offset_stats.isolated_leaf_pairs > 0U &&
              !offset_stats.node_budget_exhausted,
          "rational-offset certificates must expose subdivision work and budget exhaustion");
  flux::PHOffsetIntersectionStats exhausted_offset_stats;
  static_cast<void>(flux::intersect_quintic_ph_offsets(
      flux::make_quintic_ph_offset(*horizontal, 0.0L),
      flux::make_quintic_ph_offset(*vertical, 0.0L), Nanometres{10}, 18U, 64U, 1U,
      &exhausted_offset_stats));
  require(exhausted_offset_stats.node_budget_exhausted,
          "bounded rational-offset isolation must explicitly report an exhausted node budget");
  const auto displaced_intersections = flux::intersect_quintic_ph_offsets(
      flux::make_quintic_ph_offset(*horizontal, mm_value(0.2L)),
      flux::make_quintic_ph_offset(*vertical, -mm_value(0.3L)), Nanometres{10});
  require(std::any_of(displaced_intersections.begin(), displaced_intersections.end(),
                      [&](const auto& intersection) {
                        return std::hypotl(intersection.position.x - mm_value(5.3L),
                                           intersection.position.y - mm_value(0.2L)) <= 10.0L;
                      }),
          "non-zero degree-nine PH offsets must intersect at their displaced physical envelopes");
  const auto horizontal_hulls = flux::subdivide_quintic_ph_offset_bounds(
      flux::make_quintic_ph_offset(*horizontal, 0.0L), 4U, Nanometres{10});
  const auto far_vertical_hulls = flux::subdivide_quintic_ph_offset_bounds(
      flux::make_quintic_ph_offset(*vertical, mm_value(20.0L)), 4U, Nanometres{10});
  const auto hulls_overlap = [](const auto& a, const auto& b) {
    return a.maximum_x_nm >= b.minimum_x_nm && b.maximum_x_nm >= a.minimum_x_nm &&
           a.maximum_y_nm >= b.minimum_y_nm && b.maximum_y_nm >= a.minimum_y_nm;
  };
  require(horizontal_hulls.size() == 16U && far_vertical_hulls.size() == 16U &&
              std::none_of(horizontal_hulls.begin(), horizontal_hulls.end(), [&](const auto& a) {
                return std::any_of(far_vertical_hulls.begin(), far_vertical_hulls.end(),
                                   [&](const auto& b) { return hulls_overlap(a, b); });
              }),
          "fixed-depth rational-offset hulls must conservatively reject separated curves");

  flux::PHInterpolationConfig c2;
  c2.continuity = flux::PHJoinContinuity::curvature;
  const auto c2_line = flux::make_quintic_ph_route({mm(0.0, 0.0), mm(5.0, 0.0), mm(10.0, 0.0)}, c2);
  require(c2_line && c2_line->curvature_continuous,
          "explicit higher-order PH joining must certify a curvature-continuous case");

  auto cell_crossing = flux::make_quintic_ph_route({mm(0.0, 0.0), mm(10.0, 0.0), mm(20.0, 0.0)});
  Polygon left_cell;
  left_cell.vertices = {mm(0.0, -1.0), mm(10.0, -1.0), mm(10.0, 1.0), mm(0.0, 1.0)};
  Polygon right_cell;
  right_cell.vertices = {mm(10.0, -1.0), mm(20.0, -1.0), mm(20.0, 1.0), mm(10.0, 1.0)};
  require(cell_crossing && flux::set_quintic_ph_corridor(*cell_crossing, {left_cell, right_cell}),
          "a PH route must retain a multi-cell certified corridor");
  const auto cell_certificate = flux::certify_quintic_ph_continuous_envelope(
      *cell_crossing, Nanometres{0}, [](flux::SplineVector) {
        return flux::SplineClearanceProbe{std::numeric_limits<long double>::infinity(), {}, true};
      });
  require(cell_certificate.legal && cell_certificate.corridor_violations == 0U,
          "a PH hull crossing a shared NavMesh cell boundary must be certified by the cell union");
  const auto certified_cell_boundary = [](flux::SplineVector) {
    return flux::SplineClearanceProbe{0.0L, {}, true};
  };
  const auto redundantly_inflated_certificate = flux::certify_quintic_ph_continuous_envelope(
      *cell_crossing, Nanometres{0}, certified_cell_boundary);
  flux::SplineContinuousEnvelopeConfig certified_corridor_config;
  certified_corridor_config.corridor_certifies_clearance = true;
  const auto certified_corridor_certificate = flux::certify_quintic_ph_continuous_envelope(
      *cell_crossing, Nanometres{0}, certified_cell_boundary, certified_corridor_config);
  require(!redundantly_inflated_certificate.legal && certified_corridor_certificate.legal,
          "a full-envelope-certified NavMesh cell must prove its contained PH hull without "
          "subtracting the copper clearance a second time");

  auto nearly_straight =
      flux::make_quintic_ph_route({Point{Nanometres{43'000'000}, Nanometres{44'000'000}},
                                   Point{Nanometres{42'853'109}, Nanometres{42'531'095}},
                                   Point{Nanometres{42'846'891}, Nanometres{42'468'905}},
                                   Point{Nanometres{42'700'000}, Nanometres{41'000'000}}});
  flux::SplineShapeCertificationConfig nearly_straight_shape;
  nearly_straight_shape.minimum_bend_radius = Nanometres::from_mm(0.25);
  const auto nearly_straight_certificate =
      nearly_straight ? flux::certify_quintic_ph_shape(*nearly_straight, nearly_straight_shape)
                      : flux::SplineShapeCertificate{};
  require(
      nearly_straight && nearly_straight_certificate.bend_radius_violations == 0U,
      "quantized nearly-collinear corridor points must not create artificial tight PH bends: " +
          std::to_string(static_cast<double>(nearly_straight_certificate.minimum_bend_radius_nm)));

  auto rubber_band = flux::make_quintic_ph_route(
      {mm(0.0, 0.0), mm(5.0, 1.0), mm(10.0, -1.0), mm(15.0, 1.0), mm(20.0, 0.0)});
  require(rubber_band.has_value(), "rubber-band fixture must form a PH chain");
  const auto control_roughness = [](const flux::QuinticPHRoute& route) {
    long double value = 0.0L;
    for (std::size_t point = 1U; point + 1U < route.control_points.size(); ++point) {
      const auto second_x = route.control_points[point - 1U].x -
                            2.0L * route.control_points[point].x +
                            route.control_points[point + 1U].x;
      const auto second_y = route.control_points[point - 1U].y -
                            2.0L * route.control_points[point].y +
                            route.control_points[point + 1U].y;
      value += std::hypotl(second_x, second_y);
    }
    return value;
  };
  const auto initial_roughness = control_roughness(*rubber_band);
  flux::SplineTrajectoryOptimizationConfig rubber_config;
  rubber_config.minimum_bend_radius = Nanometres{0};
  rubber_config.control_step = Nanometres::from_mm(0.25);
  rubber_config.maximum_corridor_displacement = Nanometres::from_mm(2.0);
  rubber_config.maximum_iterations = 32U;
  rubber_config.maximum_candidate_checks = 4'096U;
  rubber_config.weights.elastic_anchor = 0.02L;
  rubber_config.weights.control_complexity = 1.0L;
  const auto rubber_optimized =
      flux::optimize_quintic_ph_trajectory(*rubber_band, rubber_config, {});
  const auto final_roughness = control_roughness(*rubber_band);
  require(rubber_optimized.accepted && rubber_optimized.accepted_control_moves > 0U &&
              final_roughness < initial_roughness * 0.65L,
          "whole-chain PH optimization must pull a sampled zigzag taut instead of only rounding "
          "its corners: roughness=" +
              std::to_string(static_cast<double>(initial_roughness)) + "->" +
              std::to_string(static_cast<double>(final_roughness)) +
              ", moves=" + std::to_string(rubber_optimized.accepted_control_moves));
  const auto rubber_samples =
      flux::adaptive_tessellate_quintic_ph(*rubber_band, Nanometres{20'000}, 16U, 4'096U);
  const auto rubber_pieces =
      flux::fit_spline_samples_to_lines_and_arcs(rubber_samples, Nanometres{20'000}, 64U, 4'096U);
  for (std::size_t piece = 1U; piece < rubber_pieces.size(); ++piece) {
    if (rubber_pieces[piece - 1U].kind != flux::SplineFittedPieceKind::segment ||
        rubber_pieces[piece].kind != flux::SplineFittedPieceKind::segment)
      continue;
    const auto& before = rubber_pieces[piece - 1U];
    const auto& after = rubber_pieces[piece];
    const auto first_x = before.end.x.value() - before.start.x.value();
    const auto first_y = before.end.y.value() - before.start.y.value();
    const auto second_x = after.end.x.value() - after.start.x.value();
    const auto second_y = after.end.y.value() - after.start.y.value();
    require(static_cast<long double>(first_x) * second_y ==
                static_cast<long double>(first_y) * second_x,
            "PH line/arc fitting must not recreate a polygonal bend as consecutive non-collinear "
            "straight chords");
  }
  std::vector<flux::SplineSample> long_flat_samples;
  for (std::size_t sample = 0U; sample < 130U; ++sample) {
    const auto parameter = static_cast<long double>(sample) / 129.0L;
    long_flat_samples.push_back(
        {parameter,
         flux::SplineVector{parameter * static_cast<long double>(Nanometres::from_mm(20.0).value()),
                            10'000.0L * std::sin(std::numbers::pi_v<long double> * parameter)}});
  }
  const auto long_flat_pieces =
      flux::fit_spline_samples_to_lines_and_arcs(long_flat_samples, Nanometres{20'000}, 8U, 256U);
  require(long_flat_pieces.size() == 1U &&
              long_flat_pieces.front().kind == flux::SplineFittedPieceKind::segment,
          "the arc sample bound must not fragment one complete tolerance-flat PH span into "
          "differently directed straight chords");

  auto gpu_rubber_band = flux::make_quintic_ph_route(
      {mm(0.0, 0.0), mm(5.0, 1.0), mm(10.0, -1.0), mm(15.0, 1.0), mm(20.0, 0.0)});
  auto gpu_rubber_config = rubber_config;
  gpu_rubber_config.prefer_gpu_preconditioner = true;
  gpu_rubber_config.gpu_precondition_iterations = 32U;
  gpu_rubber_config.prefer_gpu_polishing = true;
  gpu_rubber_config.gpu_polish_shortlist = 4U;
  gpu_rubber_config.exact_candidate_workers = 4U;
  const auto gpu_rubber_optimized =
      flux::optimize_quintic_ph_trajectory(*gpu_rubber_band, gpu_rubber_config, {});
  require(gpu_rubber_optimized.gpu_precondition_attempted &&
              (!flux::routing_compute_gpu_available() ||
               gpu_rubber_optimized.gpu_precondition_used_gpu) &&
              gpu_rubber_optimized.gpu_precondition_accepted &&
              gpu_rubber_optimized.gpu_precondition_legal &&
              (!flux::routing_compute_gpu_available() ||
               (gpu_rubber_optimized.gpu_polish_dispatches > 0U &&
                gpu_rubber_optimized.gpu_polish_exact_shortlist_checks > 0U &&
                gpu_rubber_optimized.gpu_polish_exact_shortlist_checks <
                    gpu_rubber_optimized.gpu_polish_candidates)) &&
              gpu_rubber_optimized.parallel_exact_waves > 0U &&
              gpu_rubber_optimized.parallel_exact_checks >=
                  gpu_rubber_optimized.parallel_exact_waves * 2U &&
              gpu_rubber_optimized.accepted &&
              gpu_rubber_band->control_points.front() == SplineVector{0.0L, 0.0L} &&
              gpu_rubber_band->control_points.back() == SplineVector{mm_value(20.0L), 0.0L},
          "whole-chain PH preconditioning must use the resident Vulkan XPBD path when available, "
          "preserve fixed endpoints, and remain subject to CPU trajectory acceptance");
}

void constrained_ph_rebound_avoids_inter_sample_obstacle() {
  auto spline = flux::make_quintic_ph_route(
      std::vector<Point>{mm(10.0, 10.0), mm(20.0, 10.0), mm(30.0, 10.0)});
  require(spline.has_value(), "continuous-envelope fixture must form a quintic PH curve");
  Polygon corridor;
  corridor.vertices = {mm(9.0, 8.0), mm(31.0, 8.0), mm(31.0, 12.0), mm(9.0, 12.0)};
  require(flux::set_quintic_ph_corridor(*spline, {corridor}),
          "the spline must retain its selected convex free-space corridor");
  const auto clearance = [](flux::SplineVector point) {
    const auto dx = point.x - static_cast<long double>(Nanometres::from_mm(20.0).value());
    const auto dy = point.y - static_cast<long double>(Nanometres::from_mm(10.25).value());
    const auto distance = std::hypotl(dx, dy);
    const auto radius = static_cast<long double>(Nanometres::from_mm(0.45).value());
    return flux::SplineClearanceProbe{distance - radius,
                                      distance > 0.0L
                                          ? flux::SplineVector{dx / distance, dy / distance}
                                          : flux::SplineVector{0.0L, -1.0L},
                                      distance >= radius};
  };
  const auto initial =
      flux::certify_quintic_ph_continuous_envelope(*spline, Nanometres{0}, clearance);
  flux::SplineTrajectoryOptimizationConfig config;
  config.minimum_bend_radius = Nanometres{0};
  config.control_step = Nanometres::from_mm(0.25);
  config.maximum_corridor_displacement = Nanometres::from_mm(1.5);
  config.clearance_influence_distance = Nanometres::from_mm(0.4);
  config.samples = 8U;
  config.maximum_iterations = 16U;
  config.maximum_candidate_checks = 192U;
  const auto optimized = flux::optimize_quintic_ph_trajectory(*spline, config, clearance);
  const auto final =
      flux::certify_quintic_ph_continuous_envelope(*spline, Nanometres{0}, clearance);
  flux::PreparedQuinticPHEvaluator rebound_samples(*spline, config.samples);
  require(rebound_samples.update(*spline), "optimized rebound must remain evaluable");
  std::size_t sampled_clearance_failures = 0U;
  long double minimum_sample_speed = std::numeric_limits<long double>::infinity();
  for (const auto& sample : rebound_samples.samples()) {
    sampled_clearance_failures += clearance(sample.position).represented_legal ? 0U : 1U;
    minimum_sample_speed =
        std::min(minimum_sample_speed, std::hypotl(sample.tangent.x, sample.tangent.y));
  }
  require(!initial.legal && optimized.accepted && final.legal &&
              flux::certify_quintic_ph_shape(*spline).legal,
          "continuous collision attribution must produce a smooth corridor-preserving rebound: " +
              std::to_string(initial.violations()) + "/" + std::to_string(optimized.accepted) +
              "/" + std::to_string(optimized.initial_violations) + "->" +
              std::to_string(optimized.final_violations) +
              "(shape=" + std::to_string(optimized.final_shape_violations) +
              ",continuous=" + std::to_string(optimized.final_continuous_violations) + ")/" +
              std::to_string(final.violations()) +
              "/moves=" + std::to_string(optimized.accepted_control_moves) +
              "/sample-clearance=" + std::to_string(sampled_clearance_failures) +
              "/min-speed=" + std::to_string(static_cast<double>(minimum_sample_speed)));
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
  // The broad fixture repeats regional analysis for determinism and telemetry. Keep the optional
  // terminal topology portfolio out of this deliberately impossible one-layer crossing; its
  // production-scale fallback is exercised by the routing corpus instead.
  terminal_portal_config.enable_terminal_portal_templates = false;
  terminal_portal_config.max_same_layer_detour_checks = 0U;
  CollectingTelemetrySink regional_candidate_telemetry;
  const auto regional = flux::analyze_regional_conflicts(crossing, crossed, terminal_portal_config,
                                                         {}, &regional_candidate_telemetry);
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
  const auto first_candidate_spline = std::find_if(
      regional_candidate_telemetry.events.begin(), regional_candidate_telemetry.events.end(),
      [](const auto& event) {
        return event.kind == flux::RouterTelemetryEventKind::route_candidate_selected &&
               event.route_geometry == flux::RouterTelemetryRouteGeometry::quintic_ph &&
               event.spline_control_points.size() >= 6U;
      });
  const auto spline_phase_completed = std::find_if(
      regional_candidate_telemetry.events.begin(), regional_candidate_telemetry.events.end(),
      [](const auto& event) {
        return event.kind == flux::RouterTelemetryEventKind::search_summary &&
               event.detail.find("regional spline realization completed") != std::string::npos;
      });
  require(regional == repeated_regional && regional == observed_regional &&
              decoded_regional_snapshot && decoded_regional_snapshot.events.size() == 1U &&
              decoded_regional_snapshot.events.front().dynamic_state_included &&
              decoded_regional_snapshot.events.front().conflicts.size() ==
                  regional.conflict_edges.size() &&
              regional_telemetry_state.total_groups == regional.components.size() &&
              regional_telemetry_state.regional_conflicts == regional.conflict_edges.size() &&
              !regional_telemetry_state.observed_routes.empty() &&
              first_candidate_spline != regional_candidate_telemetry.events.end() &&
              spline_phase_completed != regional_candidate_telemetry.events.end() &&
              first_candidate_spline < spline_phase_completed &&
              std::any_of(
                  regional_telemetry_state.observed_routes.begin(),
                  regional_telemetry_state.observed_routes.end(),
                  [](const auto& route) {
                    return route.route_geometry == flux::RouterTelemetryRouteGeometry::quintic_ph &&
                           route.spline_degree == 5U && route.spline_control_points.size() >= 6U;
                  }) &&
              regional.source_clearance_shadow_conflicts ==
                  crossed.clearance_shadow_conflicts.size() &&
              regional.spline_interpolation_gpu_batches > 0U &&
              regional.spline_interpolation_gpu_fallbacks <
                  regional.spline_interpolation_gpu_batches &&
              regional.spline_interpolation_gpu_candidates >=
                  regional.spline_interpolation_exact_candidates &&
              regional.spline_interpolation_exact_candidates <=
                  regional.spline_interpolation_gpu_batches * 2U &&
              regional.spline_geometry_cache_hits > 0U &&
              regional.spline_offset_broadphase_batches > 0U &&
              static_cast<std::uint64_t>(regional.spline_realization_us) > 0U &&
              regional.coalesced_clearance_shadow_samples +
                      regional.unmapped_clearance_shadow_conflicts ==
                  regional.source_clearance_shadow_conflicts &&
              regional.lane_solver_queries > 0U && regional.lane_solver_states > 0U &&
              regional.lane_solver_cache_hits <= regional.lane_solver_queries &&
              regional.joint_spline_candidates_generated >=
                  regional.joint_spline_active_candidates_retained &&
              regional.joint_spline_fallback_candidates_promoted <=
                  regional.joint_spline_fallback_candidates_retained &&
              regional.joint_spline_fallback_candidates_remaining <=
                  regional.joint_spline_fallback_candidates_retained &&
              regional.joint_spline_fallback_candidates_promoted +
                      regional.joint_spline_fallback_candidates_remaining <=
                  regional.joint_spline_fallback_candidates_retained &&
              regional.joint_spline_candidates_evaluated <=
                  regional.joint_spline_active_candidates_retained +
                      regional.joint_spline_fallback_candidates_promoted &&
              regional.joint_spline_fallback_windows_recovered <=
                  regional.joint_spline_fallback_candidates_promoted &&
              regional.unmapped_clearance_shadow_conflicts == 0U && !regional.fragments.empty() &&
              !regional.conflict_edges.empty() && regional.components.size() == 1U &&
              regional.components.front().nets.size() == 2U &&
              regional.complete() == regional.spline_candidate_available &&
              (!regional.complete() ||
               std::all_of(regional.bundle_assignments.begin(), regional.bundle_assignments.end(),
                           [](const auto& assignment) { return assignment.complete(); })),
          "regional analysis must deterministically attribute every exact clearance-shadow "
          "observation to stable route fragments, "
          "retain a bounded compact active/fallback spline portfolio, and require an all-window "
          "spline candidate before bounded source conflicts count as regionally complete: " +
              std::to_string(regional.joint_spline_candidates_generated) + "/" +
              std::to_string(regional.joint_spline_active_candidates_retained) + "/" +
              std::to_string(regional.joint_spline_fallback_candidates_retained) + "/" +
              std::to_string(regional.joint_spline_fallback_candidates_promoted) + "/" +
              std::to_string(regional.joint_spline_fallback_candidates_remaining) + "/" +
              std::to_string(regional.joint_spline_candidates_evaluated));

  flux::RoutingPlannerConfig deferred_regional_config;
  deferred_regional_config.global = crossing_config;
  deferred_regional_config.max_routability_feedback_retries = 0U;
  deferred_regional_config.regional.max_nets_per_window = 1U;
  const auto deferred_regional_plan = flux::plan_routes(crossing, deferred_regional_config);
  require(
      !deferred_regional_plan.complete() &&
          deferred_regional_plan.regional.diagnostic !=
              "regional realization deferred until every coarse connection is routed" &&
          deferred_regional_plan.feedback_probes.size() == 1U &&
          deferred_regional_plan.feedback_probes.front().pass == 0U &&
          deferred_regional_plan.feedback_probes.front().candidates.size() == 1U &&
          deferred_regional_plan.feedback_probes.front().candidates.front().regional_consulted &&
          deferred_regional_plan.feedback_probes.front().history_entries_after_global >=
              deferred_regional_plan.feedback_probes.front().history_entries_before,
      "a complete coarse route must hand a spatial conflict component to regional window "
      "partitioning rather than rejecting its entire global connected component, and every "
      "outer physical-feedback probe must publish its history and regional hand-off state");

  auto broad_feedback_config = deferred_regional_config;
  broad_feedback_config.max_routability_feedback_retries = 1U;
  const auto broad_feedback_plan = flux::plan_routes(crossing, broad_feedback_config);
  const auto broad_feedback_candidate = [&]() -> const flux::RoutingPlanFeedbackCandidate* {
    for (const auto& probe : broad_feedback_plan.feedback_probes) {
      const auto found =
          std::find_if(probe.candidates.begin(), probe.candidates.end(),
                       [](const auto& candidate) { return candidate.history_informed_global; });
      if (found != probe.candidates.end()) {
        return &*found;
      }
    }
    return nullptr;
  }();
  require(broad_feedback_candidate != nullptr &&
              broad_feedback_candidate->feedback_fell_back_to_full_global &&
              !broad_feedback_candidate->incremental_global &&
              broad_feedback_candidate->feedback_affected_nets >= 2U &&
              broad_feedback_plan.global_feedback_epochs ==
                  broad_feedback_plan.feedback_probes.size() &&
              broad_feedback_plan.cumulative_global_iterations > 0U &&
              broad_feedback_plan.retained_global_portfolio_peak > 0U &&
              broad_feedback_plan.retained_global_portfolios_evaluated > 0U &&
              broad_feedback_plan.intermediate_regional_probes > 0U &&
              std::all_of(broad_feedback_plan.feedback_probes.begin(),
                          broad_feedback_plan.feedback_probes.end(),
                          [](const auto& probe) {
                            return probe.global_epoch_iterations > 0U &&
                                   probe.cumulative_global_iterations > 0U &&
                                   probe.retained_global_portfolios > 0U;
                          }),
          "a feedback cone which covers essentially every net must re-run global allocation "
          "rather than preserving a token subset of incompatible routes, and short global "
          "epochs must retain observable topology breadth across physical probes");

  flux::RoutabilityKnowledge typed_history;
  const flux::RoutabilityResourceReference shared_resource{
      "same-cell", LayerId{"F.Cu"}, {mm(40.0, 40.0), mm(41.0, 41.0)}, {NetId{"net-1"}}};
  flux::accumulate_routability_knowledge(typed_history,
                                         {{flux::RoutabilityFailureKind::packing_capacity_exceeded,
                                           flux::RoutabilityFailureStage::regional_discrete,
                                           shared_resource,
                                           2U,
                                           {},
                                           std::nullopt,
                                           std::nullopt,
                                           flux::RoutabilityRelaxation::change_lane_order,
                                           "lane",
                                           false},
                                          {flux::RoutabilityFailureKind::packing_capacity_exceeded,
                                           flux::RoutabilityFailureStage::regional_discrete,
                                           shared_resource,
                                           3U,
                                           {},
                                           std::nullopt,
                                           std::nullopt,
                                           flux::RoutabilityRelaxation::change_corridor,
                                           "corridor",
                                           false}});
  require(typed_history.history.size() == 2U &&
              typed_history.history.front().suggested_relaxation !=
                  typed_history.history.back().suggested_relaxation,
          "routability history must not collapse lane-order and corridor-change failures into "
          "one actionless scalar pressure entry");

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
  flux::RegionalRouterConfig chain_config;
  // This fixture isolates spatial conflict-graph decomposition; terminal topology expansion is
  // orthogonal and would spend production continuous-routing budgets on every synthetic block.
  chain_config.enable_terminal_portal_templates = false;
  chain_config.max_same_layer_detour_checks = 0U;
  chain_config.enable_spline_realization = false;
  const auto chain_regional =
      flux::analyze_regional_conflicts(conflict_chain, chain_global, chain_config);
  require(chain_regional.source_clearance_shadow_conflicts == 4U &&
              chain_regional.conflict_edges.size() == 4U &&
              chain_regional.components.size() == 1U &&
              chain_regional.biconnected_blocks.size() == 3U &&
              chain_regional.articulation_fragments.size() == 2U &&
              !chain_regional.windows.empty() && chain_regional.windows.size() <= 3U &&
              std::all_of(chain_regional.windows.begin(), chain_regional.windows.end(),
                          [&](const auto& window) {
                            return !window.conflict_edges.empty() && !window.portals.empty() &&
                                   window.fragments.size() <
                                       chain_regional.components.front().fragments.size();
                          }),
          "a weak net chain must retain distant same-owner conflicts while decomposing into "
          "articulation-bounded physical windows instead of one whole-net repair group: " +
              std::to_string(chain_regional.source_clearance_shadow_conflicts) + "/" +
              std::to_string(chain_regional.conflict_edges.size()) + "/" +
              std::to_string(chain_regional.components.size()) + "/" +
              std::to_string(chain_regional.biconnected_blocks.size()) + "/" +
              std::to_string(chain_regional.articulation_fragments.size()) + "/" +
              std::to_string(chain_regional.windows.size()));

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
  const auto local_corridor_regional = flux::analyze_regional_conflicts(
      crossing, corridor_membership_global, terminal_portal_config);
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
      "quintic PH curve/PNS transaction after ordinary detailed copper fails, while retaining an "
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
  crossed_plan.regional =
      flux::analyze_regional_conflicts(crossing, bounded_crossed, terminal_portal_config);
  crossed_plan.paths = crossed_plan.regional.spline_candidate_available
                           ? crossed_plan.regional.spline_candidate_paths
                           : bounded_crossed.paths;
  crossed_plan.stop_reason = flux::RoutingPlanStopReason::regional_conflicts;
  const auto crossed_plan_detail =
      flux::materialize_detailed_route(crossing, crossed_plan, bounded_crossed_config);
  require(
      !crossed_plan_detail.complete(),
      "a regional quintic PH curve proposal must remain one atomic transaction and cannot bypass "
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
  stacked_via_config.max_same_layer_detour_checks = 0U;
  stacked_via_config.enable_terminal_portal_templates = false;
  stacked_via_config.enable_spline_realization = false;
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
  multilayer_regional_config.max_same_layer_detour_checks = 0U;
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
  auto shared_access_budget_config = multilayer_regional_config;
  shared_access_budget_config.max_access_assignment_states = 1U;
  shared_access_budget_config.enable_spline_realization = false;
  const auto shared_access_budget_regional = flux::analyze_regional_conflicts(
      multilayer_crossing, multilayer_global, shared_access_budget_config);
  const auto shared_access_budget_is_bounded =
      !shared_access_budget_regional.bundle_assignments.empty() &&
      std::all_of(shared_access_budget_regional.bundle_assignments.begin(),
                  shared_access_budget_regional.bundle_assignments.end(),
                  [&](const auto& assignment) {
                    return assignment.partial_access_assignment_states <=
                           shared_access_budget_config.max_access_assignment_states;
                  });
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
  const auto selected_multilayer_solution = selected_alternative && selected_relocated_via &&
                                            complete_two_via_plan &&
                                            multilayer_plan.regional.planned_via_transitions >= 2U;
  const auto selected_direct_ph_solution = !selected_alternative && !selected_relocated_via &&
                                           multilayer_plan.regional.spline_candidate_available &&
                                           multilayer_detail.complete();
  require(multilayer_regional_config.enable_terminal_portal_templates &&
              shared_access_budget_is_bounded &&
              multilayer_plan.regional.accepted_alternative_templates > 0U &&
              (selected_multilayer_solution || selected_direct_ph_solution) &&
              bounded_via_portfolio && stable_via_assignment_order &&
              multilayer_plan.regional.complete_via_dependent_routes ==
                  multilayer_plan.regional.multilayer_route_plans.size() &&
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
              (selected_direct_ph_solution ||
               std::any_of(multilayer_plan.paths.begin(), multilayer_plan.paths.end(),
                           [](const auto& path) {
                             return std::adjacent_find(
                                        path.points.begin(), path.points.end(),
                                        [](const auto& first, const auto& second) {
                                          return first.position == second.position &&
                                                 first.layer != second.layer;
                                        }) != path.points.end();
                           })) &&
              multilayer_detail.complete() &&
              (selected_direct_ph_solution || multilayer_detail.added_vias >= 2U),
          "a bounded crossing window must accept attributed sampled overflow, retain exact-checked "
          "alternate layer/via states, and select either clean direct PH geometry or a relocated "
          "multilayer realization, "
          "realize it as quintic PH curve-derived copper with endpoint vias, and commit only after "
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
                                  std::to_string(realization.shape_legal) + "/" +
                                  std::to_string(realization.mutually_legal) + "/" +
                                  std::to_string(realization.quantized_mutually_legal) + "/" +
                                  std::to_string(realization.centerline.size()) + "/" +
                                  std::to_string(realization.shape_self_intersections) + "/" +
                                  std::to_string(realization.shape_cusps) + "/" +
                                  std::to_string(realization.shape_control_station_reversals) +
                                  "/" + std::to_string(realization.shape_station_reversals) + "/" +
                                  std::to_string(realization.shape_excess_turn_violations) + "/" +
                                  std::to_string(realization.shape_unintended_inflections) + "/" +
                                  std::to_string(realization.shape_sample_budget_exhausted) + "/" +
                                  std::to_string(static_cast<double>(
                                      realization.shape_total_turn_radians)) + "/" +
                                  std::to_string(static_cast<double>(
                                      realization.shape_anchor_turn_radians)) + "/" +
                                  std::to_string(static_cast<double>(
                                      realization.interpolation_parameter_exponent)) + "/" +
                                  std::to_string(static_cast<double>(
                                      realization.interpolation_tangent_scale)) + ";";
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

  if (selected_multilayer_solution) {
    auto stale_multilayer_plan = multilayer_plan;
    stale_multilayer_plan.stop_reason = flux::RoutingPlanStopReason::complete;
    stale_multilayer_plan.paths = stale_multilayer_plan.regional.spline_candidate_paths;
    stale_multilayer_plan.regional.multilayer_route_plans.clear();
    const auto stale_multilayer_detail = flux::materialize_detailed_route(
        multilayer_crossing, stale_multilayer_plan, bounded_crossed_config);
    require(
        !stale_multilayer_detail.complete() && !stale_multilayer_detail.candidate,
        "detailed routing must fail closed when selected multilayer planning evidence no longer "
        "matches the spline candidate path");
  }

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
  // The mandatory portfolio retains shortest, diversified, RRT, and hydraulic discovery. RRT is
  // disabled here but its prepared-NavMesh behavior has a separate assertion.
  early_spline_config.max_static_seed_candidates_per_connection = 4U;
  early_spline_config.use_static_seed_rrt_connect_alternatives = false;
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
                                                  std::nullopt,
                                                  false};
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
  const auto rrt_navmesh = flux::route_prepared_navmesh_rrt_connect(
      prepared_navmesh, mm(20.0, 40.0), mm(80.0, 60.0), direct_navmesh_config,
      flux::NavMeshRrtConnectConfig{20'000U, 17U, 0U, 8U});
  flux::HydraulicPathConfig hydraulic_flow;
  hydraulic_flow.relaxation_iterations = 128U;
  hydraulic_flow.droplet_batches = 2U;
  hydraulic_flow.droplets_per_batch = 128U;
  hydraulic_flow.maximum_steps_per_droplet = 128U;
  hydraulic_flow.prefer_gpu = true;
  flux::NavMeshHydraulicConfig hydraulic_config;
  hydraulic_config.flow = hydraulic_flow;
  hydraulic_config.opened_guard_rings = 1U;
  const auto hydraulic_navmesh = flux::route_prepared_navmesh_hydraulic(
      prepared_navmesh, mm(20.0, 40.0), mm(80.0, 60.0), direct_navmesh_config, hydraulic_config);
  hydraulic_flow.prefer_gpu = false;
  hydraulic_config.flow = hydraulic_flow;
  const auto hydraulic_cpu_navmesh = flux::route_prepared_navmesh_hydraulic(
      prepared_navmesh, mm(20.0, 40.0), mm(80.0, 60.0), direct_navmesh_config, hydraulic_config);
  const std::vector<flux::RoutingComputeSegment> compute_segments{
      {0, 0, 1'000'000, 0, 50'000, 0U, 0U},
      {500'000, -500'000, 500'000, 500'000, 50'000, 0U, 1U},
      {2'000'000, 0, 2'000'000, 0, 50'000, 0U, 2U},
      {2'075'000, 0, 2'075'000, 0, 50'000, 0U, 3U}};
  std::vector<std::pair<std::uint32_t, std::uint32_t>> compute_pairs{{0U, 1U}, {0U, 2U}, {2U, 3U}};
  while (compute_pairs.size() < 256U) {
    compute_pairs.push_back(compute_pairs[compute_pairs.size() % 3U]);
  }
  const auto compute_conflicts_gpu =
      flux::discover_routing_segment_conflicts(compute_segments, compute_pairs, 1'024, true);
  const auto compute_conflicts_cpu =
      flux::discover_routing_segment_conflicts(compute_segments, compute_pairs, 1'024, false);
  std::vector<flux::RoutingComputePoint> compute_proximity_points;
  std::vector<flux::RoutingComputeSegment> compute_proximity_segments;
  for (std::size_t index = 0U; index < 64U; ++index) {
    compute_proximity_points.push_back(flux::RoutingComputePoint{
        static_cast<std::int64_t>(index) * 100'000, index % 2U == 0U ? 0 : 1'000'000, 10'000, 0U,
        std::numeric_limits<std::uint32_t>::max()});
    compute_proximity_segments.push_back(
        flux::RoutingComputeSegment{static_cast<std::int64_t>(index) * 100'000, -100'000,
                                    static_cast<std::int64_t>(index) * 100'000, 100'000, 10'000, 0U,
                                    static_cast<std::uint32_t>(index)});
  }
  const auto compute_proximity_gpu = flux::probe_routing_proximity(
      compute_proximity_points, compute_proximity_segments, 1'024, true);
  const auto compute_proximity_cpu = flux::probe_routing_proximity(
      compute_proximity_points, compute_proximity_segments, 1'024, false);
  flux::RoutingComputeWavefrontInput compute_wavefront;
  compute_wavefront.width = 64U;
  compute_wavefront.height = 64U;
  compute_wavefront.traversable.assign(64U * 64U, 1U);
  for (std::size_t y = 0U; y < 63U; ++y) {
    if (y != 31U)
      compute_wavefront.traversable[y * 64U + 32U] = 0U;
  }
  compute_wavefront.sources = {0U};
  compute_wavefront.maximum_passes = 256U;
  const auto compute_wavefront_gpu = flux::compute_routing_wavefront(compute_wavefront, true);
  const auto compute_wavefront_cpu = flux::compute_routing_wavefront(compute_wavefront, false);
  const auto compute_clearance_gpu = flux::compute_routing_clearance_field(
      compute_wavefront.width, compute_wavefront.height, compute_wavefront.traversable, 256U, true);
  const auto compute_clearance_cpu =
      flux::compute_routing_clearance_field(compute_wavefront.width, compute_wavefront.height,
                                            compute_wavefront.traversable, 256U, false);
  flux::RoutingComputeSplineBatchInput compute_spline;
  compute_spline.control_points_nm = {
      {0, 0},         {1'000'000, 0},         {1'000'000, 1'000'000}, {2'000'000, 1'000'000},
      {0, 2'000'000}, {1'000'000, 2'000'000}, {1'000'000, 3'000'000}, {2'000'000, 3'000'000}};
  compute_spline.sample_count = 128U;
  compute_spline.candidate_count = 2U;
  for (std::size_t candidate = 0U; candidate < compute_spline.candidate_count; ++candidate) {
    for (std::size_t sample = 0U; sample < compute_spline.sample_count; ++sample) {
      const auto parameter = static_cast<long double>(sample) /
                             static_cast<long double>(compute_spline.sample_count - 1U);
      const auto inverse = 1.0L - parameter;
      compute_spline.basis_coefficients.insert(
          compute_spline.basis_coefficients.end(),
          {inverse * inverse * inverse, 3.0L * inverse * inverse * parameter,
           3.0L * inverse * parameter * parameter, parameter * parameter * parameter});
    }
  }
  const auto compute_spline_gpu = flux::evaluate_routing_spline_batch(compute_spline, true);
  const auto compute_spline_cpu = flux::evaluate_routing_spline_batch(compute_spline, false);
  std::vector<flux::RoutingComputeLinearCandidate> compute_candidates(64U);
  for (std::size_t candidate = 0U; candidate < compute_candidates.size(); ++candidate) {
    compute_candidates[candidate].terms = {
        static_cast<std::uint32_t>(candidate), 2U, 3U, 4U, 5U, 6U, 7U, 8U};
  }
  const auto compute_scores_gpu = flux::score_routing_compute_candidates(
      compute_candidates, {1U, 10U, 100U, 1'000U, 10'000U, 100'000U, 1'000'000U, 10'000'000U},
      true);
  const auto compute_scores_cpu = flux::score_routing_compute_candidates(
      compute_candidates, {1U, 10U, 100U, 1'000U, 10'000U, 100'000U, 1'000'000U, 10'000'000U},
      false);
  const auto spline_batch_matches =
      compute_spline_gpu.ok() && compute_spline_cpu.ok() &&
      compute_spline_gpu.positions_nm.size() == compute_spline_cpu.positions_nm.size() &&
      std::equal(compute_spline_gpu.positions_nm.begin(), compute_spline_gpu.positions_nm.end(),
                 compute_spline_cpu.positions_nm.begin(),
                 [](const auto& gpu, const auto& cpu) {
                   return std::abs(gpu.first - cpu.first) <= 512.0L &&
                          std::abs(gpu.second - cpu.second) <= 512.0L;
                 }) &&
      compute_spline_cpu.positions_nm.size() == 256U &&
      std::abs(compute_spline_cpu.positions_nm[128U].second - 2'000'000.0L) <= 0.5L;
  require(
      direct_navmesh.ok() && prepared_navmesh.ok() && reused_navmesh.ok() &&
          direct_navmesh.corridor == reused_navmesh.corridor &&
          direct_navmesh.centerline == reused_navmesh.centerline &&
          reused_navmesh.used_flood_fill && reused_navmesh.flood_fill_cells_visited > 0U &&
          flooded_navmesh.ok() && flooded_navmesh.used_flood_fill &&
          flooded_navmesh.goal == mm(20.0, 60.0) && reflooded_navmesh.ok() &&
          std::find(reflooded_navmesh.corridor.begin(), reflooded_navmesh.corridor.end(),
                    closed_cell) == reflooded_navmesh.corridor.end() &&
          rrt_navmesh.ok() && rrt_navmesh.used_rrt_connect &&
          rrt_navmesh.rrt_connect_iterations > 0U &&
          flux::validate_navmesh_topology_signature(rrt_navmesh.topology_signature,
                                                    rrt_navmesh.centerline)
              .preserved &&
          hydraulic_navmesh.ok() && hydraulic_navmesh.used_hydraulic_erosion &&
          hydraulic_navmesh.hydraulic_direct_polygon_discovery &&
          hydraulic_navmesh.hydraulic_field_cell_pitch.value() ==
              direct_navmesh_config.cell_pitch.value() / 2 &&
          hydraulic_navmesh.hydraulic_discovery_cells > prepared_navmesh.cells.size() &&
          hydraulic_navmesh.hydraulic_exact_certified_chords > 0U &&
          hydraulic_navmesh.hydraulic_relaxation_passes > 0U &&
          hydraulic_navmesh.hydraulic_droplet_steps > 0U &&
          flux::validate_navmesh_topology_signature(hydraulic_navmesh.topology_signature,
                                                    hydraulic_navmesh.centerline)
              .preserved &&
          hydraulic_cpu_navmesh.ok() &&
          hydraulic_cpu_navmesh.centerline == hydraulic_navmesh.centerline &&
          hydraulic_cpu_navmesh.hydraulic_flow_field == hydraulic_navmesh.hydraulic_flow_field &&
          hydraulic_cpu_navmesh.hydraulic_erosion_field ==
              hydraulic_navmesh.hydraulic_erosion_field &&
          hydraulic_cpu_navmesh.hydraulic_heightmap == hydraulic_navmesh.hydraulic_heightmap &&
          hydraulic_cpu_navmesh.hydraulic_free_space_field ==
              hydraulic_navmesh.hydraulic_free_space_field &&
          hydraulic_navmesh.hydraulic_heightmap.size() ==
              hydraulic_navmesh.hydraulic_field_width * hydraulic_navmesh.hydraulic_field_height &&
          std::count_if(hydraulic_navmesh.hydraulic_free_space_field.begin(),
                        hydraulic_navmesh.hydraulic_free_space_field.end(),
                        [](std::uint16_t coverage) { return coverage > 0U; }) >
              hydraulic_navmesh.hydraulic_erosion_field.size() / 2U &&
          std::all_of(reused_navmesh.centerline.begin(), reused_navmesh.centerline.end(),
                      [&](const auto point) {
                        return flux::navmesh_corridor_contains(reused_navmesh, point);
                      }) &&
          compute_conflicts_gpu.ok() && compute_conflicts_cpu.ok() &&
          compute_conflicts_gpu.possible_conflicts == compute_conflicts_cpu.possible_conflicts &&
          compute_conflicts_cpu.possible_conflicts[0] == 1U &&
          compute_conflicts_cpu.possible_conflicts[1] == 0U &&
          compute_conflicts_cpu.possible_conflicts[2] == 1U && compute_proximity_gpu.ok() &&
          compute_proximity_cpu.ok() &&
          compute_proximity_gpu.queries.size() == compute_proximity_cpu.queries.size() &&
          std::equal(compute_proximity_gpu.queries.begin(), compute_proximity_gpu.queries.end(),
                     compute_proximity_cpu.queries.begin(),
                     [](const auto& gpu, const auto& cpu) {
                       return gpu.possible_conflict == cpu.possible_conflict &&
                              std::abs(gpu.conservative_margin_nm - cpu.conservative_margin_nm) <=
                                  512;
                     }) &&
          compute_wavefront_gpu.ok() && compute_wavefront_cpu.ok() &&
          compute_wavefront_gpu.distance == compute_wavefront_cpu.distance &&
          compute_clearance_gpu.ok() && compute_clearance_cpu.ok() &&
          compute_clearance_gpu.signed_distance == compute_clearance_cpu.signed_distance &&
          spline_batch_matches && compute_scores_gpu.ok() && compute_scores_cpu.ok() &&
          compute_scores_gpu.scores == compute_scores_cpu.scores,
      "prepared, one-shot, and multi-source flood-fill NavMesh routing must select legal "
      "deterministic topology and centerlines: " +
          direct_navmesh.diagnostic + "/" + reused_navmesh.diagnostic + "/" +
          flooded_navmesh.diagnostic + "/" + reflooded_navmesh.diagnostic + "/" +
          hydraulic_navmesh.diagnostic);
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

  const auto topology_spline = flux::make_topology_preserving_quintic_ph_route(
      direct_navmesh.centerline, Nanometres::from_mm(0.1));
  require(topology_spline.has_value() && topology_spline->tangent_continuous &&
              topology_spline->segments.size() + 1U == topology_spline->control_points.size() &&
              topology_spline->node_parameters.size() == topology_spline->control_points.size(),
          "a topology-preserving seed must be an explicit tangent-continuous quintic PH chain");
  auto knot_refined_spline = *topology_spline;
  const auto refined_start = flux::evaluate_quintic_ph(knot_refined_spline, 0.0L);
  const auto refined_end = flux::evaluate_quintic_ph(knot_refined_spline, 1.0L);
  const std::array refined_shape_parameters{0.19L, 0.37L, 0.61L, 0.83L};
  std::array<flux::SplineVector, refined_shape_parameters.size()> refined_shape_before{};
  for (std::size_t sample = 0U; sample < refined_shape_parameters.size(); ++sample) {
    refined_shape_before[sample] =
        flux::evaluate_quintic_ph(knot_refined_spline, refined_shape_parameters[sample]);
  }
  const auto knot_control_count = knot_refined_spline.control_points.size();
  const auto refined_shape_preserved = [&] {
    for (std::size_t sample = 0U; sample < refined_shape_parameters.size(); ++sample) {
      const auto after =
          flux::evaluate_quintic_ph(knot_refined_spline, refined_shape_parameters[sample]);
      if (std::hypotl(after.x - refined_shape_before[sample].x,
                      after.y - refined_shape_before[sample].y) > 1.0L)
        return false;
    }
    return true;
  };
  require(flux::insert_quintic_ph_interpolation_node(knot_refined_spline, 0.37L) &&
              knot_refined_spline.control_points.size() == knot_control_count + 1U &&
              knot_refined_spline.tangent_continuous &&
              flux::evaluate_quintic_ph(knot_refined_spline, 0.0L) == refined_start &&
              flux::evaluate_quintic_ph(knot_refined_spline, 1.0L) == refined_end &&
              refined_shape_preserved(),
          "adaptive PH refinement must exactly subdivide one span while retaining its shape, "
          "endpoints, and C1 continuity");
  const auto topology_samples = flux::adaptive_tessellate_quintic_ph(
      *topology_spline, Nanometres::from_mm(0.005), 12U, 2'048U);
  std::vector<Point> topology_polyline;
  topology_polyline.reserve(topology_samples.size());
  for (const auto& sample : topology_samples) {
    const auto point =
        Point{Nanometres{static_cast<std::int64_t>(std::llround(sample.position.x))},
              Nanometres{static_cast<std::int64_t>(std::llround(sample.position.y))}};
    if (topology_polyline.empty() || topology_polyline.back() != point) {
      topology_polyline.push_back(point);
    }
  }
  const auto topology_spline_check = flux::validate_navmesh_topology_signature(
      direct_navmesh.topology_signature, topology_polyline);
  require(topology_spline_check.preserved && flux::certify_quintic_ph_shape(*topology_spline).legal,
          "the rounded piecewise-cubic seed must preserve the selected directed NavMesh gates "
          "without loops or stationary reversals: " +
              topology_spline_check.diagnostic);

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
  auto straight_spline = flux::make_quintic_ph_route(oversampled_straight);
  require(straight_spline.has_value() && straight_spline->control_points.size() == 2U,
          "straight spline seeds must collapse repeated and forward-collinear samples");
  flux::SplineTrajectoryOptimizationConfig straight_config;
  straight_config.minimum_bend_radius = Nanometres{0};
  straight_config.maximum_corridor_displacement = Nanometres::from_mm(2.0);
  straight_config.samples = 24U;
  straight_config.maximum_iterations = 4U;
  straight_config.maximum_candidate_checks = 128U;
  const auto straight_optimized = flux::optimize_quintic_ph_trajectory(
      *straight_spline, straight_config, [](flux::SplineVector point) {
        return flux::SplineClearanceProbe{
            std::max(1.0L, 100'000.0L + point.y - 10'000'000.0L), {0.0L, 1.0L}, true};
      });
  const auto straight_samples =
      flux::adaptive_tessellate_quintic_ph(*straight_spline, Nanometres{1}, 8U, 128U);
  require(straight_optimized.accepted &&
              std::all_of(straight_samples.begin(), straight_samples.end(),
                          [](const auto& sample) {
                            return std::abs(sample.position.y - 10'000'000.0L) < 0.5L;
                          }),
          "trajectory optimization must not introduce a curve into an already legal straight "
          "route solely to improve a soft clearance objective");

  const auto evaluator_spline = flux::make_quintic_ph_route(evaluator_points);
  require(evaluator_spline.has_value(), "prepared spline evaluator fixture must form a spline");
  flux::PreparedQuinticPHEvaluator evaluator(*evaluator_spline, 17U);
  require(evaluator.update(*evaluator_spline),
          "prepared spline evaluator must accept its topology");
  for (std::size_t sample = 0U; sample < evaluator.samples().size(); ++sample) {
    const auto parameter = static_cast<long double>(sample) /
                           static_cast<long double>(evaluator.samples().size() - 1U);
    const auto expected = flux::evaluate_quintic_ph_differential(*evaluator_spline, parameter);
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
                                    corridor.topology_signature.valid &&
                                    corridor.lane_centerline.size() >= 2U &&
                                    corridor.minimum_physical_cross_section_width >=
                                        corridor.required_envelope_width &&
                                    corridor.lane_capacity > 0U;
                           });
      }));
  require(early_spline.complete() && early_spline.static_seed_connections == 1U &&
              early_spline.static_seed_searches == 4U &&
              early_spline.static_seed_candidates >= 3U &&
              early_spline.iteration_zero_portfolios_evaluated > 0U &&
              early_spline.static_seed_hydraulic_attempts > 0U &&
              early_spline.static_seed_hydraulic_accepted +
                      early_spline.static_seed_hydraulic_failures ==
                  early_spline.static_seed_hydraulic_attempts &&
              early_spline.static_seed_corridor_nodes > 0U &&
              !early_spline.static_seed_budget_exhausted && retained_geometric_corridors > 0U,
          "the early hybrid-graph seed search must evaluate its complete mandatory portfolio, "
          "including hydraulic erosion, around a blocker before considering a copper shove: " +
              std::to_string(early_spline.complete()) + "/" +
              std::to_string(early_spline.static_seed_searches) + "/" +
              std::to_string(early_spline.static_seed_candidates) + "/" +
              std::to_string(early_spline.static_seed_corridor_nodes) + "/" +
              std::to_string(early_spline.static_seed_budget_exhausted) +
              "/geometry=" + std::to_string(retained_geometric_corridors) +
              "/nav=" + std::to_string(early_spline.static_seed_navmesh_routes) + "/" +
              std::to_string(early_spline.static_seed_navmesh_failures) +
              "/hydraulic=" + std::to_string(early_spline.static_seed_hydraulic_attempts) + "/" +
              std::to_string(early_spline.static_seed_hydraulic_accepted) + "/" +
              std::to_string(early_spline.static_seed_hydraulic_failures) +
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
  const auto curved_seed = flux::make_quintic_ph_route(std::vector<Point>{
      mm(20.0, 40.0), mm(50.0, 40.0), mm(50.0, 60.0), mm(80.0, 60.0), mm(80.0, 40.0)});
  require(curved_seed.has_value(), "curved materialization fixture must form a spline seed");
  const auto curved_seed_shape = flux::certify_quintic_ph_shape(*curved_seed);
  flux::DetailedRouterConfig curved_config;
  curved_config.pull_tight_45_degrees = false;
  curved_config.continuous_refinement = false;
  curved_config.circular_arc_refinement = true;
  curved_config.spline_minimum_bend_radius = Nanometres{0};
  curved_config.maximum_circular_arc_radius = Nanometres::from_mm(1.0);
  const auto curved = flux::materialize_detailed_route(curved_source, curved_global, curved_config);
  require(curved.complete() && curved.candidate && curved.fitted_circular_arcs > 0U &&
              curved.attempted_quintic_ph_runs > 0U &&
              curved.adaptive_spline_tessellation_samples > curved.fitted_circular_arcs &&
              curved.unrounded_non_junction_bends == 0U && curved.copper_drc.ok(),
          "a bent route must be adaptively fitted from a quintic PH curve to connected DRC-clean "
          "native line/arc copper: " +
              std::to_string(curved.adaptive_spline_tessellation_samples) + "/" +
              std::to_string(curved.fitted_circular_arcs) +
              "/runs=" + std::to_string(curved.attempted_quintic_ph_runs) + "/" +
              std::to_string(curved.fitted_quintic_ph_runs) +
              "/complete=" + std::to_string(curved.complete()) +
              "/nav=" + std::to_string(curved.spline_navmesh_routes) + "/" +
              std::to_string(curved.spline_navmesh_failures) +
              "/traj=" + std::to_string(curved.spline_trajectory_accepted) + "/" +
              std::to_string(curved.spline_trajectory_initial_violations) + "->" +
              std::to_string(curved.spline_trajectory_final_violations) + "/" +
              std::to_string(curved.spline_trajectory_initial_shape_violations) + "->" +
              std::to_string(curved.spline_trajectory_final_shape_violations) + "/continuous=" +
              std::to_string(curved.spline_trajectory_initial_continuous_violations) + "->" +
              std::to_string(curved.spline_trajectory_final_continuous_violations) + "/" +
              std::to_string(curved_seed_shape.self_intersections) + "/" +
              std::to_string(curved_seed_shape.cusps) + "/" +
              std::to_string(curved_seed_shape.station_reversals) + "/" +
              std::to_string(curved_seed_shape.excess_turn_violations) + "/" +
              curved.spline_navmesh_last_diagnostic + "/" + curved.diagnostic);

  const std::vector<Point> spline_polyline{mm(10.0, 10.0), mm(30.0, 10.0), mm(30.0, 30.0),
                                           mm(50.0, 30.0)};
  auto local_spline = flux::make_quintic_ph_route(spline_polyline);
  require(local_spline.has_value(), "a bent polyline must initialize a quintic PH curve");
  const auto original_controls = local_spline->control_points;
  const auto original_midpoint = flux::evaluate_quintic_ph(*local_spline, 0.5L);
  const auto moved_controls = flux::shove_quintic_ph_interpolation_corridor(
      *local_spline, 0.5L, flux::SplineVector{0.0L, 1.0L}, Nanometres::from_mm(0.5),
      Nanometres::from_mm(1.0));
  const auto moved_midpoint = flux::evaluate_quintic_ph(*local_spline, 0.5L);
  require(moved_controls > 0U && moved_midpoint.y > original_midpoint.y &&
              local_spline->control_points[0] == original_controls[0] &&
              local_spline->control_points.back() == original_controls.back() &&
              local_spline->tangent_continuous,
          "PH interpolation-node shove must remain local, preserve endpoints, and rebuild C1");
  const auto differential = flux::evaluate_quintic_ph_differential(*local_spline, 0.5L);
  require(std::isfinite(differential.tangent.x) && std::isfinite(differential.tangent.y) &&
              differential.bend_radius_nm > 0.0L,
          "analytical quintic PH curve derivatives must expose finite tangent and bend-radius "
          "evidence");
  flux::SplineTrajectoryOptimizationConfig elastic_config;
  elastic_config.minimum_bend_radius = Nanometres{0};
  elastic_config.maximum_corridor_displacement = Nanometres::from_mm(5.0);
  elastic_config.samples = 12U;
  elastic_config.maximum_iterations = 1U;
  elastic_config.maximum_candidate_checks = 4U;
  const auto prior_elastic_midpoint = flux::evaluate_quintic_ph(*local_spline, 0.5L);
  const auto new_end =
      flux::SplineVector{local_spline->control_points.back().x,
                         local_spline->control_points.back().y + Nanometres::from_mm(1.0).value()};
  const auto elastic = flux::deform_quintic_ph_elastic(
      *local_spline, local_spline->control_points.front(), new_end, elastic_config,
      [](flux::SplineVector) { return flux::SplineClearanceProbe{1'000'000.0L, {}, true}; });
  const auto elastic_midpoint = flux::evaluate_quintic_ph(*local_spline, 0.5L);
  require(elastic.accepted && elastic.elastic_predeformed_controls > 0U &&
              local_spline->control_points.back() == new_end &&
              elastic_midpoint.y > prior_elastic_midpoint.y &&
              elastic_midpoint.y - prior_elastic_midpoint.y < Nanometres::from_mm(1.0).value(),
          "elastic endpoint deformation must preserve the endpoint constraint and attenuate its "
          "effect through nearby spline controls");

  // Shape validity is a hard route property, not a soft objective. An obstacle-free curve that
  // loops or reverses along its source corridor must therefore be rejected before materialization.
  flux::QuinticPHRoute looping_spline;
  for (const auto point : {mm(0.0, 0.0), mm(1.0, 0.0), mm(3.0, 3.0), mm(1.0, 3.0), mm(3.0, -3.0),
                           mm(5.0, 0.0), mm(6.0, 0.0)}) {
    looping_spline.control_points.push_back(flux::SplineVector{
        static_cast<long double>(point.x.value()), static_cast<long double>(point.y.value())});
  }
  for (std::int64_t x = 0; x <= 6; ++x) {
    looping_spline.corridor_anchors.push_back(
        flux::SplineVector{static_cast<long double>(Nanometres::from_mm(x).value()), 0.0L});
  }
  require(flux::rebuild_quintic_ph_route(looping_spline),
          "looping PH fixture must still be algebraically constructible");
  const auto looping_shape = flux::certify_quintic_ph_shape(looping_spline);
  require(!looping_shape.legal && looping_shape.violations() > 0U &&
              looping_shape.control_station_reversals > 0U,
          "a looping or corridor-reversing quintic PH curve must fail hard shape certification");
  for (const auto& smooth_seed :
       std::vector<std::vector<Point>>{{mm(0.0, 0.0), mm(10.0, 0.0), mm(20.0, 10.0)},
                                       {mm(0.0, 0.0), mm(10.0, 0.0), mm(10.0, 10.0)},
                                       {mm(0.0, 0.0), mm(10.0, 0.0), mm(3.0, 7.0)}}) {
    const auto smooth_spline = flux::make_quintic_ph_route(smooth_seed);
    require(smooth_spline.has_value() && flux::certify_quintic_ph_shape(*smooth_spline).legal,
            "ordinary acute, right, and obtuse route turns must remain simple forward curves");
  }

  // Equal endpoint motion is a rigid translation component. It must not bow the middle of the
  // route as the previous independent cubic endpoint falloffs did.
  auto translated_spline = flux::make_quintic_ph_route(
      std::vector<Point>{mm(10.0, 40.0), mm(20.0, 40.0), mm(30.0, 40.0)});
  require(translated_spline.has_value(), "translation fixture must initialize a quintic PH curve");
  const auto translated_before = translated_spline->control_points;
  const auto translation =
      flux::SplineVector{static_cast<long double>(Nanometres::from_mm(1.0).value()),
                         static_cast<long double>(Nanometres::from_mm(2.0).value())};
  auto translation_config = elastic_config;
  translation_config.maximum_iterations = 1U;
  translation_config.maximum_candidate_checks = 4U;
  const auto translated = flux::deform_quintic_ph_elastic(
      *translated_spline,
      flux::SplineVector{translated_before.front().x + translation.x,
                         translated_before.front().y + translation.y},
      flux::SplineVector{translated_before.back().x + translation.x,
                         translated_before.back().y + translation.y},
      translation_config,
      [](flux::SplineVector) { return flux::SplineClearanceProbe{1.0e12L, {}, true}; });
  require(translated.accepted &&
              std::equal(translated_before.begin(), translated_before.end(),
                         translated_spline->control_points.begin(),
                         [&](const auto& before, const auto& after) {
                           return std::abs(after.x - before.x - translation.x) < 0.5L &&
                                  std::abs(after.y - before.y - translation.y) < 0.5L;
                         }) &&
              flux::certify_quintic_ph_shape(*translated_spline).legal,
          "equal endpoint displacement must rigidly translate every control and retain a simple "
          "forward curve");

  // A large local collision displacement may be rejected, but can never be committed as a loop,
  // cusp, station reversal, or sub-radius bend.
  auto guarded_shove = flux::make_quintic_ph_route(
      std::vector<Point>{mm(0.0, 50.0), mm(1.0, 50.0), mm(2.0, 50.0), mm(3.0, 50.0)});
  require(guarded_shove.has_value(), "guarded shove fixture must initialize a quintic PH curve");
  const auto guarded_before = *guarded_shove;
  auto guarded_config = elastic_config;
  guarded_config.minimum_bend_radius = Nanometres::from_mm(0.1);
  guarded_config.maximum_corridor_displacement = Nanometres::from_mm(2.0);
  guarded_config.maximum_iterations = 4U;
  guarded_config.maximum_candidate_checks = 128U;
  const auto guarded_result = flux::deform_quintic_ph_elastic_at(
      *guarded_shove, 0.5L, flux::SplineVector{0.0L, 1.0L}, Nanometres::from_mm(2.0),
      guarded_config,
      [](flux::SplineVector) { return flux::SplineClearanceProbe{1.0e12L, {}, true}; });
  flux::SplineShapeCertificationConfig guarded_shape_config;
  guarded_shape_config.minimum_bend_radius = guarded_config.minimum_bend_radius;
  const auto guarded_shape = flux::certify_quintic_ph_shape(*guarded_shove, guarded_shape_config);
  require(guarded_shape.legal && (guarded_result.accepted ||
                                  guarded_shove->control_points == guarded_before.control_points),
          "a rejected shove must roll back, while an accepted shove must retain exact shape "
          "legality and bend radius");

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
              decoded_snapshot.events.front().message == "Routing finished" &&
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

  flux::RouterTelemetryEvent global_stage_finished;
  global_stage_finished.kind = flux::RouterTelemetryEventKind::global_routing_finished;
  global_stage_finished.stable_id = "global-routing-stage";
  global_stage_finished.detail = "global routing stage completed: checkpoint retained";
  const auto decoded_global_stage =
      flux::telemetry::decode_events(flux::serialize_router_telemetry_event(global_stage_finished));
  require(
      decoded_global_stage && decoded_global_stage.events.size() == 1U &&
          decoded_global_stage.events.front().type == flux::telemetry::EventType::search_summary &&
          decoded_global_stage.events.front().message.find("stage completed") != std::string::npos,
      "a nested global-router completion must remain a stage summary rather than marking the "
      "complete routing session finished");

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
  auto unfilled_zone_contents = contents;
  const auto board_close = unfilled_zone_contents.rfind(')');
  require(board_close != std::string::npos,
          "unfilled-zone fixture must retain the root closing delimiter");
  unfilled_zone_contents.insert(
      board_close,
      "  (zone (layer \"B.Cu\") (uuid \"unfilled-zone-test\") (hatch edge 0.5)\n"
      "    (polygon (pts (xy 20 20) (xy 30 20) (xy 30 30) (xy 20 30))))\n");
  const auto unfilled_zone_import = flux::io::import_kicad_pcb(unfilled_zone_contents);
  const auto imported_unfilled_zone =
      unfilled_zone_import.board
          ? std::find_if(unfilled_zone_import.board->zones.begin(),
                         unfilled_zone_import.board->zones.end(), [](const auto& zone) {
                           return zone.id == ObjectId{"unfilled-zone-test"};
                         })
          : std::vector<flux::CopperZone>::const_iterator{};
  require(unfilled_zone_import.strict() && unfilled_zone_import.board &&
              imported_unfilled_zone != unfilled_zone_import.board->zones.end() &&
              imported_unfilled_zone->requires_refill && imported_unfilled_zone->fills.empty() &&
              imported_unfilled_zone->net == NetId{"kicad-net-0"},
          "a netless KiCad zone without saved fill must retain strict source-bound refill state");
  const auto unfilled_zone_export = flux::io::export_kicad_candidate(
      unfilled_zone_contents, *unfilled_zone_import.board, *unfilled_zone_import.board);
  const auto unfilled_zone_reimport =
      unfilled_zone_export.contents
          ? flux::io::import_kicad_pcb(*unfilled_zone_export.contents)
          : flux::io::KicadImportResult{};
  require(unfilled_zone_export.ok() && unfilled_zone_export.contents &&
              unfilled_zone_export.contents->find("unfilled-zone-test") != std::string::npos &&
              unfilled_zone_reimport.strict() && unfilled_zone_reimport.board &&
              std::any_of(unfilled_zone_reimport.board->zones.begin(),
                          unfilled_zone_reimport.board->zones.end(), [](const auto& zone) {
                            return zone.id == ObjectId{"unfilled-zone-test"} &&
                                   zone.requires_refill && zone.fills.empty();
                          }),
          "an unchanged unfilled zone must retain exact source syntax and canonical refill state "
          "through source-preserving export");
  auto named_net_contents = contents;
  const auto erase_once = [&named_net_contents](std::string_view value) {
    const auto position = named_net_contents.find(value);
    require(position != std::string::npos, "named-net fixture source syntax must be identifiable");
    named_net_contents.erase(position, value.size());
  };
  const auto replace_all = [&named_net_contents](std::string_view from, std::string_view to) {
    auto position = named_net_contents.find(from);
    while (position != std::string::npos) {
      named_net_contents.replace(position, from.size(), to);
      position = named_net_contents.find(from, position + to.size());
    }
  };
  erase_once("  (net 0 \"\")\n");
  erase_once("  (net 1 \"SIGNAL\")\n");
  replace_all("(net 1 \"SIGNAL\")", "(net \"SIGNAL\")");
  replace_all("(net 1)", "(net \"SIGNAL\")");
  replace_all("(net 0)", "(net \"\")");
  const auto named_net_import = flux::io::import_kicad_pcb(named_net_contents);
  require(named_net_import.strict() && named_net_import.board &&
              std::any_of(named_net_import.board->nets.begin(), named_net_import.board->nets.end(),
                          [](const auto& net) {
                            return net.id == NetId{"kicad-net-name:SIGNAL"} &&
                                   net.name == "SIGNAL";
                          }),
          "a KiCad 10 board without an ordinal net table must derive stable named-net identity");
  auto named_net_candidate = *named_net_import.board;
  named_net_candidate.tracks.push_back(
      {ObjectId{"named-net-added-track"}, NetId{"kicad-net-name:SIGNAL"}, LayerId{"B.Cu"},
       mm(25.0, 35.0), mm(30.0, 35.0), Nanometres::from_mm(0.25), false});
  const auto named_net_export = flux::io::export_kicad_candidate(
      named_net_contents, *named_net_import.board, named_net_candidate);
  require(named_net_export.ok() && named_net_export.contents &&
              named_net_export.contents->find("(net \"SIGNAL\")") != std::string::npos,
          "new KiCad 10 copper must retain quoted net-name references through strict round trip");
  auto differential_suffix_contents = contents;
  const auto signal_net = differential_suffix_contents.find("(net 1 \"SIGNAL\")");
  require(signal_net != std::string::npos,
          "differential-suffix fixture must retain the canonical net declaration");
  differential_suffix_contents.replace(
      signal_net, std::string{"(net 1 \"SIGNAL\")"}.size(),
      "(net 1 \"DATA_P\")\n  (net 2 \"DATA_N\")\n  (net 3 \"Net-(U5A-+)\")\n"
      "  (net 4 \"Net-(U5A--)\")");
  const auto differential_suffix_import = flux::io::import_kicad_pcb(differential_suffix_contents);
  require(differential_suffix_import.strict() &&
              differential_suffix_import.board->differential_pairs.size() == 1U &&
              differential_suffix_import.board->differential_pairs.front().positive_net ==
                  NetId{"kicad-net-1"} &&
              differential_suffix_import.board->differential_pairs.front().negative_net ==
                  NetId{"kicad-net-2"},
          "KiCad differential-pair suffix parsing must stop at punctuation and must not couple "
          "autogenerated Net-(...) pin names");
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
  auto oval_drill_contents = contents;
  const auto oval_pad = oval_drill_contents.rfind("(pad \"1\" smd circle");
  require(oval_pad != std::string::npos,
          "oval-drill fixture pad declaration must remain identifiable");
  oval_drill_contents.replace(oval_pad + std::string{"(pad \"1\" "}.size(),
                              std::string{"smd circle"}.size(), "np_thru_hole oval");
  const auto oval_layers = oval_drill_contents.find(
      "(layers \"F.Cu\" \"F.Paste\" \"F.Mask\")", oval_pad);
  require(oval_layers != std::string::npos,
          "oval-drill fixture pad layers must remain identifiable");
  oval_drill_contents.replace(
      oval_layers, std::string{"(layers \"F.Cu\" \"F.Paste\" \"F.Mask\")"}.size(),
      "(drill oval 0.6 1.4 (offset 0.2 0))\n      (layers \"*.Cu\" \"*.Mask\")");
  const auto oval_drill_import = flux::io::import_kicad_pcb(oval_drill_contents);
  const auto oval_drill_shapes = oval_drill_import.board
                                     ? flux::copper_pad_shapes(*oval_drill_import.board)
                                     : std::vector<flux::CopperPadShape>{};
  require(oval_drill_import.strict() && oval_drill_import.board &&
              !oval_drill_import.board->footprints.back().pads.front().plated &&
              oval_drill_import.board->footprints.back().pads.front().drill_width ==
                  Nanometres::from_mm(0.6) &&
              oval_drill_import.board->footprints.back().pads.front().drill_height ==
                  Nanometres::from_mm(1.4) &&
              oval_drill_import.board->footprints.back().pads.front().drill_offset ==
                  mm(0.2, 0.0) &&
              !oval_drill_shapes.empty() &&
              oval_drill_shapes.back().shape == flux::PadShape::oval &&
              oval_drill_shapes.back().width == Nanometres::from_mm(0.6) &&
              oval_drill_shapes.back().height == Nanometres::from_mm(1.4) &&
              oval_drill_shapes.back().position == mm(79.8, 40.0),
          "oval and offset non-plated drill geometry must retain its exact local dimensions and "
          "world-space routing obstacle");
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

  auto lossy_contents = contents;
  const auto keepout_start = lossy_contents.find("(keepout");
  const auto keepout_end = lossy_contents.find("\n    )", keepout_start);
  require(keepout_start != std::string::npos && keepout_end != std::string::npos,
          "KiCad fixture keepout must remain identifiable");
  auto keepout_rule = lossy_contents.find("not_allowed", keepout_start);
  while (keepout_rule != std::string::npos && keepout_rule < keepout_end) {
    lossy_contents.replace(keepout_rule, std::string{"not_allowed"}.size(), "allowed");
    keepout_rule = lossy_contents.find("not_allowed", keepout_rule + 1U);
  }
  const auto lossy_import = flux::io::import_kicad_pcb(lossy_contents);
  require(lossy_import.board && !lossy_import.strict(),
          "empty keepout syntax must retain a non-strict canonical projection: board=" +
              std::to_string(lossy_import.board.has_value()) +
              ", strict=" + std::to_string(lossy_import.strict()) +
              ", lossy=" + std::to_string(lossy_import.lossy) +
              ", diagnostics=" + std::to_string(lossy_import.diagnostics.size()));
  const auto qualified_lossy_export =
      flux::io::export_kicad_candidate(lossy_contents, *lossy_import.board, *lossy_import.board);
  require(!qualified_lossy_export.ok() &&
              std::any_of(qualified_lossy_export.diagnostics.begin(),
                          qualified_lossy_export.diagnostics.end(), [](const auto& diagnostic) {
                            return diagnostic.code == "source_not_strict";
                          }),
          "ordinary export must continue to reject a lossy source projection");
  flux::io::KicadExportOptions preview_options;
  preview_options.verify_round_trip = false;
  preview_options.allow_lossy_diagnostic_preview = true;
  const auto diagnostic_lossy_export = flux::io::export_kicad_candidate(
      lossy_contents, *lossy_import.board, *lossy_import.board, preview_options);
  require(diagnostic_lossy_export.ok() &&
              std::any_of(diagnostic_lossy_export.diagnostics.begin(),
                          diagnostic_lossy_export.diagnostics.end(), [](const auto& diagnostic) {
                            return diagnostic.code == "lossy_diagnostic_preview" &&
                                   diagnostic.severity == flux::io::ExportSeverity::warning;
                          }),
          "explicit diagnostic export must write a visibly non-qualified lossy artifact");

  auto invalid_contents = contents;
  const auto first_footprint = invalid_contents.find("(footprint");
  const auto first_pad_size = invalid_contents.find("(size 2 2)", first_footprint);
  require(first_pad_size != std::string::npos, "KiCad fixture pad must remain identifiable");
  invalid_contents.replace(first_pad_size, std::string{"(size 2 2)"}.size(), "(size 0 2)");
  const auto invalid_import = flux::io::import_kicad_pcb(invalid_contents);
  require(invalid_import.strict() && invalid_import.board &&
              !flux::validate_board(*invalid_import.board).ok(),
          "zero-width imported pad must isolate canonical structural validation");
  const auto qualified_invalid_export = flux::io::export_kicad_candidate(
      invalid_contents, *invalid_import.board, *invalid_import.board);
  require(!qualified_invalid_export.ok() &&
              std::any_of(qualified_invalid_export.diagnostics.begin(),
                          qualified_invalid_export.diagnostics.end(), [](const auto& diagnostic) {
                            return diagnostic.code == "invalid_source_snapshot";
                          }),
          "ordinary export must continue to reject a structurally invalid projection");
  const auto diagnostic_invalid_export = flux::io::export_kicad_candidate(
      invalid_contents, *invalid_import.board, *invalid_import.board, preview_options);
  require(diagnostic_invalid_export.ok() &&
              std::any_of(diagnostic_invalid_export.diagnostics.begin(),
                          diagnostic_invalid_export.diagnostics.end(), [](const auto& diagnostic) {
                            return diagnostic.code == "invalid_diagnostic_preview" &&
                                   diagnostic.severity == flux::io::ExportSeverity::warning;
                          }),
          "diagnostic export must preserve structural-invalid evidence without suppressing bytes");

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

void routing_xpbd_joint_relaxation_preserves_endpoints_and_reports_stress() {
  const std::vector<flux::RoutingXpbdParticle> particles{
      {-2'000'000, -1'000'000, 200'000, 0.0L, 0U, 0U, flux::routing_xpbd_endpoint, -2'000'000,
       -1'000'000, 100'000},
      {0, 0, 200'000, 1.0L, 0U, 0U, 0U, 0, 0, 1'200'000},
      {2'000'000, -1'000'000, 200'000, 0.0L, 0U, 0U, flux::routing_xpbd_endpoint, 2'000'000,
       -1'000'000, 100'000},
      {-2'000'000, 1'000'000, 200'000, 0.0L, 1U, 0U, flux::routing_xpbd_endpoint, -2'000'000,
       1'000'000, 100'000},
      {0, 0, 200'000, 1.0L, 1U, 0U, 0U, 0, 0, 1'200'000},
      {2'000'000, 1'000'000, 200'000, 0.0L, 1U, 0U, flux::routing_xpbd_endpoint, 2'000'000,
       1'000'000, 100'000}};
  const std::vector<flux::RoutingXpbdSegment> segments{{0U, 1U, 0U, 0U, 200'000, 2'236'068},
                                                       {1U, 2U, 0U, 0U, 200'000, 2'236'068},
                                                       {3U, 4U, 1U, 0U, 200'000, 2'236'068},
                                                       {4U, 5U, 1U, 0U, 200'000, 2'236'068}};
  flux::RoutingXpbdConfig config;
  config.iterations = 64U;
  config.maximum_step_nm = 100'000;
  config.bend_compliance = 0.01L;
  config.pressure_width = 32U;
  config.pressure_height = 32U;
  config.pressure_origin_x_nm = -4'000'000;
  config.pressure_origin_y_nm = -4'000'000;
  config.pressure_pitch_nm = 250'000;
  config.record_iteration_metrics = true;
  config.prefer_gpu = false;
  const auto cpu = flux::relax_routing_xpbd(particles, segments, {}, {}, config);
  config.prefer_gpu = true;
  const auto gpu = flux::relax_routing_xpbd(particles, segments, {}, {}, config);
  require(cpu.ok() && gpu.ok() &&
              cpu.positions_nm.front() ==
                  std::pair<std::int64_t, std::int64_t>{-2'000'000, -1'000'000} &&
              cpu.positions_nm[2] == std::pair<std::int64_t, std::int64_t>{2'000'000, -1'000'000} &&
              cpu.final.total_residual_nm < cpu.initial.total_residual_nm &&
              cpu.constraints.stretch == segments.size() && cpu.constraints.bend == 2U &&
              cpu.constraints.trace_clearance > 0U && cpu.converged &&
              cpu.pressure_field.size() == 32U * 32U &&
              gpu.positions_nm.front() == cpu.positions_nm.front() &&
              gpu.positions_nm[2] == cpu.positions_nm[2] &&
              gpu.positions_nm[1] != gpu.positions_nm[4] &&
              std::abs(gpu.positions_nm[1].first) < 4'000'000 &&
              std::abs(gpu.positions_nm[1].second) < 4'000'000 &&
              std::abs(gpu.positions_nm[4].first) < 4'000'000 &&
              std::abs(gpu.positions_nm[4].second) < 4'000'000 &&
              gpu.initial.total_residual_nm > 0.0L &&
              gpu.final.total_residual_nm <= gpu.initial.total_residual_nm && gpu.converged &&
              gpu.constraints.trace_clearance == gpu.candidate_pairs && gpu.timings.solve_us > 0U &&
              (!gpu.used_gpu ||
               (gpu.used_gpu_broadphase && !gpu.broadphase_fallback && gpu.broadphase_cells > 0U &&
                gpu.broadphase_pair_emissions > 0U && gpu.used_gpu_pressure_field &&
                gpu.pressure_field.size() == 32U * 32U &&
                gpu.iteration_metrics.size() == config.iterations &&
                gpu.particle_stress_by_type.size() == particles.size() &&
                gpu.stress.trace_clearance_nm > 0.0L)),
          "joint XPBD relaxation must keep terminal particles fixed, reduce inter-route stress, "
          "publish bounded pressure, and preserve endpoint semantics on CPU and Vulkan paths: " +
              cpu.diagnostic + "/" + gpu.diagnostic + "; cpu=" +
              std::to_string(cpu.initial.violations) + "->" + std::to_string(cpu.final.violations) +
              ", " + std::to_string(static_cast<double>(cpu.final.maximum_residual_nm)) + "; gpu=" +
              std::to_string(gpu.initial.violations) + "->" + std::to_string(gpu.final.violations) +
              ", " + std::to_string(static_cast<double>(gpu.final.maximum_residual_nm)));

  const auto require_trace_pack = [&](std::string_view name,
                                      const std::vector<std::int64_t>& endpoint_y,
                                      const std::vector<std::int64_t>& interior_y) {
    std::vector<flux::RoutingXpbdParticle> pack_particles;
    std::vector<flux::RoutingXpbdSegment> pack_segments;
    for (std::size_t route = 0U; route < endpoint_y.size(); ++route) {
      const auto first = static_cast<std::uint32_t>(pack_particles.size());
      for (std::size_t control = 0U; control < 5U; ++control) {
        const auto x = -2'000'000 + static_cast<std::int64_t>(control) * 1'000'000;
        const auto endpoint = control == 0U || control == 4U;
        const auto y = endpoint ? endpoint_y[route] : interior_y[route];
        pack_particles.push_back({x, y, 100'000, endpoint ? 0.0L : 1.0L,
                                  static_cast<std::uint32_t>(route), 0U,
                                  endpoint ? flux::routing_xpbd_endpoint : 0U, x, y, 1'500'000});
        if (control > 0U) {
          pack_segments.push_back({first + static_cast<std::uint32_t>(control - 1U),
                                   first + static_cast<std::uint32_t>(control),
                                   static_cast<std::uint32_t>(route), 0U, 100'000, 1'000'000,
                                   100'000});
        }
      }
    }
    auto pack_config = config;
    pack_config.iterations = 512U;
    pack_config.maximum_step_nm = 100'000;
    pack_config.prefer_gpu = false;
    const auto cpu_pack =
        flux::relax_routing_xpbd(pack_particles, pack_segments, {}, {}, pack_config);
    pack_config.prefer_gpu = true;
    const auto gpu_pack =
        flux::relax_routing_xpbd(pack_particles, pack_segments, {}, {}, pack_config);
    require(cpu_pack.ok() && gpu_pack.ok() && cpu_pack.initial.violations > 0U &&
                gpu_pack.initial.violations > 0U && cpu_pack.converged && gpu_pack.converged,
            "joint XPBD trace packing must resolve " + std::string(name) +
                ": cpu=" + std::to_string(cpu_pack.initial.violations) + "->" +
                std::to_string(cpu_pack.final.violations) +
                ", gpu=" + std::to_string(gpu_pack.initial.violations) + "->" +
                std::to_string(gpu_pack.final.violations));
  };
  require_trace_pack("parallel traces that are too close", {-500'000, 500'000},
                     {-100'000, 100'000});
  require_trace_pack("three traces competing in one neck", {-700'000, 0, 700'000},
                     {-150'000, 0, 150'000});

  const std::vector<flux::RoutingXpbdParticle> obstacle_particles{
      {-2'000'000, 0, 100'000, 0.0L, 0U, 0U, flux::routing_xpbd_endpoint, -2'000'000, 0, 1'500'000},
      {-1'000'000, 0, 100'000, 1.0L, 0U, 0U, 0U, -1'000'000, 0, 1'500'000},
      {0, 0, 100'000, 1.0L, 0U, 0U, 0U, 0, 0, 1'500'000},
      {1'000'000, 0, 100'000, 1.0L, 0U, 0U, 0U, 1'000'000, 0, 1'500'000},
      {2'000'000, 0, 100'000, 0.0L, 0U, 0U, flux::routing_xpbd_endpoint, 2'000'000, 0, 1'500'000}};
  const std::vector<flux::RoutingXpbdSegment> obstacle_route{
      {0U, 1U, 0U, 0U, 100'000, 1'000'000, 100'000},
      {1U, 2U, 0U, 0U, 100'000, 1'000'000, 100'000},
      {2U, 3U, 0U, 0U, 100'000, 1'000'000, 100'000},
      {3U, 4U, 0U, 0U, 100'000, 1'000'000, 100'000}};
  const std::vector<flux::RoutingXpbdObstacleSegment> obstacle{
      {0, -500'000, 0, 500'000, 200'000, 0U}};
  config.prefer_gpu = false;
  config.iterations = 96U;
  config.bend_compliance = 0.05L;
  const auto obstacle_cpu =
      flux::relax_routing_xpbd(obstacle_particles, obstacle_route, obstacle, {}, config);
  config.prefer_gpu = true;
  const auto obstacle_gpu =
      flux::relax_routing_xpbd(obstacle_particles, obstacle_route, obstacle, {}, config);
  require(obstacle_cpu.ok() && obstacle_gpu.ok() && obstacle_cpu.converged &&
              obstacle_gpu.converged &&
              (!flux::routing_compute_gpu_available() ||
               (obstacle_gpu.used_gpu && obstacle_gpu.stress.obstacle_nm > 0.0L)) &&
              obstacle_cpu.final.total_residual_nm < obstacle_cpu.initial.total_residual_nm,
          "CPU and Vulkan XPBD segment/obstacle constraints must remove a finite static-geometry "
          "penetration and attribute obstacle stress: " +
              obstacle_cpu.diagnostic + "/" + obstacle_gpu.diagnostic +
              "; violations=" + std::to_string(obstacle_cpu.final.violations) + "/" +
              std::to_string(obstacle_gpu.final.violations));

  // One bounded synthetic matrix covers the risky static-geometry shapes from Stage B without
  // proliferating independent unit fixtures: rectangular penetration, narrow passage, concave
  // corner, and a fixed-pad escape whose first movable controls must bend around a blocker.
  const auto require_static_case =
      [&](std::string_view name, const std::vector<flux::RoutingXpbdParticle>& case_particles,
          const std::vector<flux::RoutingXpbdSegment>& case_segments,
          const std::vector<flux::RoutingXpbdObstacleSegment>& case_obstacles) {
        auto case_config = config;
        case_config.iterations = 1'024U;
        case_config.maximum_step_nm = 100'000;
        case_config.bend_compliance = 0.05L;
        case_config.prefer_gpu = false;
        const auto cpu_case = flux::relax_routing_xpbd(case_particles, case_segments,
                                                       case_obstacles, {}, case_config);
        case_config.prefer_gpu = true;
        const auto gpu_case = flux::relax_routing_xpbd(case_particles, case_segments,
                                                       case_obstacles, {}, case_config);
        require(cpu_case.ok() && gpu_case.ok() && cpu_case.converged && gpu_case.converged &&
                    cpu_case.final.total_residual_nm < cpu_case.initial.total_residual_nm &&
                    gpu_case.final.total_residual_nm < gpu_case.initial.total_residual_nm,
                "XPBD static-geometry case must converge on CPU and Vulkan: " + std::string(name) +
                    "; cpu=" + std::to_string(cpu_case.initial.violations) + "->" +
                    std::to_string(cpu_case.final.violations) +
                    ", gpu=" + std::to_string(gpu_case.initial.violations) + "->" +
                    std::to_string(gpu_case.final.violations) + ", residual=" +
                    std::to_string(static_cast<double>(cpu_case.final.maximum_residual_nm)) + "/" +
                    std::to_string(static_cast<double>(gpu_case.final.maximum_residual_nm)) +
                    ", controls=" + std::to_string(cpu_case.positions_nm[1].first) + ":" +
                    std::to_string(cpu_case.positions_nm[1].second) + "," +
                    std::to_string(cpu_case.positions_nm[2].first) + ":" +
                    std::to_string(cpu_case.positions_nm[2].second) + "," +
                    std::to_string(cpu_case.positions_nm[3].first) + ":" +
                    std::to_string(cpu_case.positions_nm[3].second) +
                    "; gpu_controls=" + std::to_string(gpu_case.positions_nm[1].first) + ":" +
                    std::to_string(gpu_case.positions_nm[1].second) + "," +
                    std::to_string(gpu_case.positions_nm[2].first) + ":" +
                    std::to_string(gpu_case.positions_nm[2].second) + "," +
                    std::to_string(gpu_case.positions_nm[3].first) + ":" +
                    std::to_string(gpu_case.positions_nm[3].second) +
                    "; diagnostics=" + cpu_case.diagnostic + "/" + gpu_case.diagnostic);
      };
  const std::vector<flux::RoutingXpbdObstacleSegment> rectangle_obstacle{
      {-300'000, -500'000, 300'000, -500'000, 0, 0U, std::numeric_limits<std::uint32_t>::max(), 0,
       -600'000},
      {300'000, -500'000, 300'000, 500'000, 0, 0U, std::numeric_limits<std::uint32_t>::max(),
       1'000'000, 0},
      {300'000, 500'000, -300'000, 500'000, 0, 0U, std::numeric_limits<std::uint32_t>::max(), 0,
       600'000},
      {-300'000, 500'000, -300'000, -500'000, 0, 0U, std::numeric_limits<std::uint32_t>::max(),
       -1'000'000, 0}};
  require_static_case("rectangular penetration", obstacle_particles, obstacle_route,
                      rectangle_obstacle);
  auto passage_particles = obstacle_particles;
  passage_particles[1].y_nm = 250'000;
  passage_particles[2].y_nm = 250'000;
  passage_particles[3].y_nm = 250'000;
  const std::vector<flux::RoutingXpbdObstacleSegment> passage_obstacles{
      {-1'500'000, 450'000, 1'500'000, 450'000, 0, 0U},
      {-1'500'000, -450'000, 1'500'000, -450'000, 0, 0U}};
  require_static_case("narrow passage", passage_particles, obstacle_route, passage_obstacles);
  const std::vector<flux::RoutingXpbdObstacleSegment> concave_obstacle{
      {-250'000, -700'000, -250'000, 500'000, 0, 0U, std::numeric_limits<std::uint32_t>::max(),
       -1'200'000, 0},
      {-250'000, 500'000, 600'000, 500'000, 0, 0U, std::numeric_limits<std::uint32_t>::max(), 0,
       850'000},
      {600'000, 500'000, 600'000, 100'000, 0, 0U, std::numeric_limits<std::uint32_t>::max(),
       400'000, 0}};
  require_static_case("concave corner", obstacle_particles, obstacle_route, concave_obstacle);
  const std::vector<flux::RoutingXpbdObstacleSegment> pad_escape_obstacle{
      {-1'150'000, -400'000, -1'150'000, 400'000, 0, 0U, std::numeric_limits<std::uint32_t>::max(),
       0, 800'000}};
  require_static_case("fixed-pad escape", obstacle_particles, obstacle_route, pad_escape_obstacle);

  const std::vector<flux::RoutingXpbdParticle> terrain_particles{
      {-2'000'000, 0, 100'000, 0.0L, 0U, 0U, flux::routing_xpbd_endpoint, -2'000'000, 0, 100'000},
      {0, 0, 100'000, 1.0L, 0U, 0U, 0U, 0, 0, 2'000'000},
      {2'000'000, 0, 100'000, 0.0L, 0U, 0U, flux::routing_xpbd_endpoint, 2'000'000, 0, 100'000}};
  const std::vector<flux::RoutingXpbdSegment> terrain_segments{
      {0U, 1U, 0U, 0U, 100'000, 2'000'000, 100'000}, {1U, 2U, 0U, 0U, 100'000, 2'000'000, 100'000}};
  flux::RoutingXpbdTerrainField terrain;
  terrain.width = 9U;
  terrain.height = 9U;
  terrain.origin_x_nm = -4'000'000;
  terrain.origin_y_nm = -4'000'000;
  terrain.pitch_nm = 1'000'000;
  terrain.pressure.resize(terrain.width * terrain.height);
  for (std::size_t y = 0U; y < terrain.height; ++y) {
    for (std::size_t x = 0U; x < terrain.width; ++x)
      terrain.pressure[y * terrain.width + x] = static_cast<float>(y * 10U);
  }
  config.iterations = 32U;
  config.maximum_step_nm = 50'000;
  config.terrain_gain = 0.05L;
  config.prefer_gpu = false;
  const auto terrain_cpu =
      flux::relax_routing_xpbd(terrain_particles, terrain_segments, {}, terrain, config);
  config.prefer_gpu = true;
  const auto terrain_gpu =
      flux::relax_routing_xpbd(terrain_particles, terrain_segments, {}, terrain, config);
  require(terrain_cpu.ok() && terrain_gpu.ok() && terrain_cpu.positions_nm[1].second < 0 &&
              terrain_gpu.positions_nm[1].second < 0 &&
              terrain_cpu.positions_nm.front() == terrain_gpu.positions_nm.front(),
          "CPU and Vulkan XPBD must sample the same routing-terrain gradient without moving fixed "
          "terminals");

  auto layered_particles = terrain_particles;
  for (const auto& particle : terrain_particles) {
    auto copy = particle;
    copy.route = 1U;
    copy.layer = 1U;
    layered_particles.push_back(copy);
  }
  auto layered_segments = terrain_segments;
  for (const auto& segment : terrain_segments) {
    auto copy = segment;
    copy.first_particle += 3U;
    copy.second_particle += 3U;
    copy.route = 1U;
    copy.layer = 1U;
    layered_segments.push_back(copy);
  }
  auto layered_terrain = terrain;
  layered_terrain.layer_count = 2U;
  const auto terrain_plane = layered_terrain.width * layered_terrain.height;
  layered_terrain.pressure.assign(terrain_plane * layered_terrain.layer_count, 0.0F);
  for (std::size_t y = 0U; y < layered_terrain.height; ++y) {
    for (std::size_t x = 0U; x < layered_terrain.width; ++x)
      layered_terrain.pressure[y * layered_terrain.width + x] = static_cast<float>(y * 10U);
  }
  config.prefer_gpu = false;
  const auto layered_cpu =
      flux::relax_routing_xpbd(layered_particles, layered_segments, {}, layered_terrain, config);
  config.prefer_gpu = true;
  const auto layered_gpu =
      flux::relax_routing_xpbd(layered_particles, layered_segments, {}, layered_terrain, config);
  const auto layered_gpu_repeated =
      flux::relax_routing_xpbd(layered_particles, layered_segments, {}, layered_terrain, config);
  require(layered_cpu.ok() && layered_gpu.ok() && layered_gpu_repeated.ok() &&
              layered_cpu.positions_nm[1].second < 0 && layered_cpu.positions_nm[4].second == 0 &&
              layered_gpu.positions_nm[1].second < 0 && layered_gpu.positions_nm[4].second == 0 &&
              layered_cpu.pressure_layer_count == 2U && layered_gpu.pressure_layer_count == 2U &&
              layered_cpu.pressure_field.size() == 32U * 32U * 2U &&
              layered_gpu.pressure_field.size() == 32U * 32U * 2U &&
              (!flux::routing_compute_gpu_available() ||
               (layered_gpu.used_gpu && layered_gpu.used_gpu_pressure_field &&
                layered_gpu_repeated.resident_input_buffer_reuses >= 4U)),
          "XPBD terrain, stress diffusion, and resident static inputs must remain isolated by "
          "copper layer on CPU and Vulkan paths: cpu_y=" +
              std::to_string(layered_cpu.positions_nm[1].second) + "/" +
              std::to_string(layered_cpu.positions_nm[4].second) +
              ", gpu_y=" + std::to_string(layered_gpu.positions_nm[1].second) + "/" +
              std::to_string(layered_gpu.positions_nm[4].second) +
              ", layers=" + std::to_string(layered_cpu.pressure_layer_count) + "/" +
              std::to_string(layered_gpu.pressure_layer_count) +
              ", cells=" + std::to_string(layered_cpu.pressure_field.size()) + "/" +
              std::to_string(layered_gpu.pressure_field.size()) +
              ", reuse=" + std::to_string(layered_gpu_repeated.resident_input_buffer_reuses) +
              ", diag=" + layered_cpu.diagnostic + "/" + layered_gpu.diagnostic + "/" +
              layered_gpu_repeated.diagnostic);

  flux::RegionalRouteResult feedback;
  feedback.configured.enable_xpbd_feedback = true;
  flux::RegionalRoutingWindow feedback_window;
  feedback_window.id = 7U;
  feedback_window.bounds = {mm(40.0, 35.0), mm(60.0, 45.0)};
  feedback_window.layers = {LayerId{"F.Cu"}};
  feedback_window.nets = {NetId{"net-1"}, NetId{"n-b"}};
  feedback.windows.push_back(feedback_window);
  flux::RegionalXpbdInvocation feedback_invocation;
  feedback_invocation.window_id = feedback_window.id;
  feedback_invocation.final_violations = 2U;
  feedback_invocation.final_peak_residual = Nanometres{75'000};
  feedback_invocation.topology_escape_required = true;
  feedback.xpbd_invocations.push_back(feedback_invocation);
  feedback.local_routability_diagnoses.push_back(flux::RoutabilityDiagnosis{
      flux::RoutabilityFailureKind::shove_deadlock, flux::RoutabilityFailureStage::local_pns,
      flux::RoutabilityResourceReference{"regional-window:7:xpbd-cell:2:3",
                                         LayerId{"F.Cu"},
                                         {mm(40.0, 35.0), mm(60.0, 45.0)},
                                         feedback_window.nets},
      100U, Nanometres{75'000}, std::nullopt, std::nullopt,
      flux::RoutabilityRelaxation::change_corridor, "diffused XPBD constraint-pressure cell", false,
      true});
  const auto feedback_diagnoses = flux::diagnose_regional_routability(feedback);
  flux::RoutabilityKnowledge knowledge;
  flux::accumulate_routability_knowledge(knowledge, feedback_diagnoses);
  require(std::any_of(knowledge.history.begin(), knowledge.history.end(),
                      [](const auto& entry) {
                        return entry.kind == flux::RoutabilityFailureKind::shove_deadlock &&
                               entry.resource.bounds.contains(mm(50.0, 40.0)) &&
                               entry.pressure > 0U;
                      }),
          "diffused XPBD stress must enter the existing bounded spatial routability history");
  GlobalRouterConfig baseline_config;
  baseline_config.grid_pitch = Nanometres::from_mm(5.0);
  baseline_config.max_iterations = 1U;
  const auto baseline_route = flux::route_global(two_pad_board(), baseline_config);
  auto pressured_config = baseline_config;
  pressured_config.routability_history = knowledge.history;
  const auto pressured_route = flux::route_global(two_pad_board(), pressured_config);
  const auto pressure_region = rectangle(40.0, 35.0, 60.0, 45.0);
  const auto pressure_region_crossings = [&](const flux::GlobalRouteResult& route) {
    std::size_t crossings = 0U;
    for (const auto& path : route.paths) {
      for (std::size_t point = 1U; point < path.points.size(); ++point) {
        if (path.points[point - 1U].layer == path.points[point].layer &&
            flux::geometry::segment_polygon_copper_distance_nm(path.points[point - 1U].position,
                                                               path.points[point].position,
                                                               pressure_region) <= 0.0L) {
          ++crossings;
        }
      }
    }
    return crossings;
  };
  const auto baseline_pressure_crossings = pressure_region_crossings(baseline_route);
  const auto pressured_pressure_crossings = pressure_region_crossings(pressured_route);
  require(baseline_route.unrouted_nets.empty() && pressured_route.unrouted_nets.empty() &&
              baseline_route.paths != pressured_route.paths &&
              baseline_pressure_crossings > pressured_pressure_crossings &&
              pressured_route.routability_history_edges > 0U &&
              pressured_route.routability_history_pressure >= 100U,
          "diffused XPBD pressure must alter a subsequent connected PathFinder route while "
          "moving it away from the stressed region and remaining a non-authoritative cost: " +
              std::to_string(baseline_pressure_crossings) + "->" +
              std::to_string(pressured_pressure_crossings));
}

void hydraulic_polygon_discovery_projects_into_coarse_navmesh() {
  flux::OffsetFreeSpaceRegion region;
  region.area = rectangle(0.5, 0.5, 99.5, 79.5);
  region.holes.push_back(rectangle(29.5, 39.5, 70.5, 40.7));
  const flux::NavMeshConfig navmesh_config{Nanometres::from_mm(5.0),
                                           100'000U,
                                           100'000U,
                                           {},
                                           Nanometres{0},
                                           1U,
                                           Nanometres{0},
                                           std::nullopt};
  const auto prepared = flux::prepare_navmesh_domain(region, navmesh_config);
  flux::NavMeshHydraulicConfig hydraulic_config;
  hydraulic_config.flow.relaxation_iterations = 128U;
  hydraulic_config.flow.droplet_batches = 2U;
  hydraulic_config.flow.droplets_per_batch = 128U;
  hydraulic_config.flow.maximum_steps_per_droplet = 128U;
  hydraulic_config.flow.prefer_gpu = true;
  hydraulic_config.downstream_feedback.push_back(flux::NavMeshHydraulicTerrainDeposit{
      flux::geometry::AxisAlignedBounds{mm(45.0, 45.0), mm(55.0, 55.0)}, mm(45.0, 45.0),
      mm(55.0, 55.0), true, 7U, 3U});
  const auto gpu = flux::route_prepared_navmesh_hydraulic(prepared, mm(20.0, 40.0), mm(80.0, 60.0),
                                                          navmesh_config, hydraulic_config);
  hydraulic_config.flow.prefer_gpu = false;
  const auto cpu = flux::route_prepared_navmesh_hydraulic(prepared, mm(20.0, 40.0), mm(80.0, 60.0),
                                                          navmesh_config, hydraulic_config);
  require(gpu.ok() && gpu.used_hydraulic_gpu && gpu.hydraulic_direct_polygon_discovery &&
              gpu.hydraulic_field_cell_pitch.value() == navmesh_config.cell_pitch.value() / 2 &&
              gpu.hydraulic_discovery_cells > prepared.cells.size() &&
              gpu.hydraulic_exact_certified_chords > 0U &&
              flux::validate_navmesh_topology_signature(gpu.topology_signature, gpu.centerline)
                  .preserved &&
              cpu.ok() && cpu.centerline == gpu.centerline &&
              cpu.hydraulic_heightmap == gpu.hydraulic_heightmap &&
              cpu.hydraulic_flow_field == gpu.hydraulic_flow_field &&
              cpu.hydraulic_erosion_field == gpu.hydraulic_erosion_field &&
              cpu.hydraulic_free_space_field == gpu.hydraulic_free_space_field &&
              cpu.hydraulic_downstream_pressure_field == gpu.hydraulic_downstream_pressure_field &&
              cpu.hydraulic_success_prior_field == gpu.hydraulic_success_prior_field &&
              gpu.hydraulic_maximum_downstream_pressure == 7U &&
              gpu.hydraulic_maximum_success_prior == 3U,
          "hydraulic routing must discover a fine polygonal free-space river before exact "
          "certification and coarse NavMesh resource projection: " +
              gpu.diagnostic + "/" + cpu.diagnostic);
}

void hydraulic_gpu_fuses_clearance_and_batches_independent_requests() {
  constexpr std::size_t width = 48U;
  constexpr std::size_t height = 32U;
  std::vector<flux::HydraulicPathInput> inputs(4U);
  for (std::size_t index = 0U; index < inputs.size(); ++index) {
    auto& input = inputs[index];
    input.width = width;
    input.height = height;
    input.walkable.assign(width * height, 1U);
    for (std::size_t y = 6U; y < 26U; ++y) {
      if (y != 13U + index)
        input.walkable[y * width + 24U] = 0U;
    }
    input.source = (4U + index) * width + 3U;
    input.target = (26U - index) * width + 44U;
  }
  flux::HydraulicPathConfig cpu_config;
  cpu_config.prefer_gpu = false;
  cpu_config.relaxation_iterations = 96U;
  cpu_config.droplet_batches = 2U;
  cpu_config.droplets_per_batch = 64U;
  cpu_config.maximum_steps_per_droplet = 96U;
  std::vector<flux::HydraulicPathResult> expected;
  for (const auto& input : inputs)
    expected.push_back(flux::solve_hydraulic_erosion_path(input, cpu_config));

  auto gpu_config = cpu_config;
  gpu_config.prefer_gpu = true;
  std::vector<flux::HydraulicPathResult> actual(inputs.size());
  std::vector<std::jthread> workers;
  std::barrier ready(static_cast<std::ptrdiff_t>(inputs.size()));
  for (std::size_t index = 0U; index < inputs.size(); ++index) {
    workers.emplace_back([&, index] {
      ready.arrive_and_wait();
      actual[index] = flux::solve_hydraulic_erosion_path(inputs[index], gpu_config);
    });
  }
  workers.clear();
  for (std::size_t index = 0U; index < inputs.size(); ++index) {
    require(actual[index].ok() && actual[index].used_gpu && actual[index].gpu_batch_size == 4U &&
                actual[index].used_gpu_clearance_field && expected[index].ok() &&
                actual[index].riverbed == expected[index].riverbed &&
                actual[index].obstacle_clearance == expected[index].obstacle_clearance &&
                actual[index].static_potential == expected[index].static_potential &&
                actual[index].erosion == expected[index].erosion &&
                actual[index].accumulated_flow == expected[index].accumulated_flow,
            "concurrent hydraulic requests must retain deterministic CPU-equivalent clearance, "
            "potential, erosion, flow, and riverbed results: " +
                actual[index].diagnostic);
  }
  const auto reused = flux::solve_hydraulic_erosion_path(inputs.front(), gpu_config);
  require(reused.ok() && reused.gpu_clearance_cache_hit &&
              reused.gpu_persistent_buffer_reuses == 15U && reused.gpu_buffer_growths == 0U,
          "a repeated hydraulic domain must reuse every capacity-grown GPU buffer");

  flux::RoutingComputeViaCandidateInput via_filter;
  via_filter.clearance_nm = 100'000;
  via_filter.top_k = 8U;
  for (std::size_t index = 0U; index < 128U; ++index) {
    flux::RoutingComputeViaCandidate candidate;
    candidate.x_nm = static_cast<std::int64_t>(index) * 1'000'000;
    candidate.y_nm = 5'000'000;
    candidate.radius_nm = 250'000;
    candidate.runup_radius_nm = 100'000;
    candidate.owner = 1U;
    candidate.layer_mask = std::uint64_t{1U} << 1U;
    candidate.runup_count = 1U;
    candidate.runup_origins_nm[0] = {candidate.x_nm, 0};
    candidate.runup_layers[0] = 1U;
    candidate.corridor_occupancy_pressure = index >= 64U ? 1U : 0U;
    via_filter.candidates.push_back(candidate);
  }
  for (std::size_t index = 0U; index < 16U; ++index) {
    const auto x = static_cast<std::int64_t>(index) * 1'000'000;
    via_filter.obstacles.push_back(
        flux::RoutingComputeSegment{x, 4'900'000, x, 5'100'000, 250'000, 1U, 2U});
  }
  const auto via_cpu = flux::filter_routing_via_candidates(via_filter, false);
  const auto via_gpu = flux::filter_routing_via_candidates(via_filter, true);
  require(via_cpu.ok() && via_gpu.ok() && via_gpu.used_gpu &&
              via_cpu.shortlist.size() == via_gpu.shortlist.size() &&
              std::equal(via_cpu.shortlist.begin(), via_cpu.shortlist.end(),
                         via_gpu.shortlist.begin(),
                         [](const auto& lhs, const auto& rhs) {
                           return lhs.candidate == rhs.candidate &&
                                  (lhs.rejection_bits == 0U) == (rhs.rejection_bits == 0U);
                         }) &&
              std::count_if(via_gpu.rejection_bitmap.begin(), via_gpu.rejection_bitmap.end(),
                            [](auto bits) { return bits != 0U; }) >= 16,
          "regional GPU via filtering must retain the deterministic legal top-K while rejecting "
          "obstacle/run-up conflicts before exact regional checks");
}

void adaptive_navmesh_recovers_clearance_clipped_passage() {
  flux::FreeSpaceDomain heterogeneous_domain;
  heterogeneous_domain.outline = rectangle(0.0, 0.0, 20.0, 10.0);
  heterogeneous_domain.obstacles = {rectangle(4.0, 4.0, 5.0, 5.0), rectangle(12.0, 4.0, 13.0, 5.0)};
  flux::FreeSpaceOffsetConfig heterogeneous_config{Nanometres::from_mm(0.1)};
  heterogeneous_config.obstacle_clearances = {Nanometres::from_mm(0.2), Nanometres::from_mm(2.0)};
  const auto heterogeneous =
      flux::offset_polygonal_free_space(heterogeneous_domain, heterogeneous_config);
  const auto represented_free = [&](Point point) {
    return std::any_of(heterogeneous.regions.begin(), heterogeneous.regions.end(),
                       [&](const auto& candidate) {
                         if (flux::geometry::locate_point(candidate.area, point) ==
                             flux::geometry::PointLocation::outside)
                           return false;
                         return std::none_of(candidate.holes.begin(), candidate.holes.end(),
                                             [&](const auto& hole) {
                                               return flux::geometry::locate_point(hole, point) !=
                                                      flux::geometry::PointLocation::outside;
                                             });
                       });
  };
  require(heterogeneous.ok() && represented_free(mm(6.0, 4.5)) && !represented_free(mm(14.0, 4.5)),
          "heterogeneous free-space preparation must preserve room beside a low-clearance "
          "obstacle while reserving the larger envelope of a high-clearance obstacle: " +
              heterogeneous.diagnostic);

  flux::OffsetFreeSpaceRegion region;
  region.area = rectangle(0.0, 0.0, 10.0, 10.0);
  region.holes.push_back(rectangle(4.2, 0.1, 5.8, 8.8));
  flux::NavMeshConfig coarse;
  coarse.cell_pitch = Nanometres::from_mm(2.0);
  coarse.maximum_cells = 20'000U;
  coarse.maximum_search_expansions = 20'000U;
  coarse.corridor_guard_rings = 1U;
  coarse.adaptive_clearance_refinement = false;
  const auto coarse_domain = flux::prepare_navmesh_domain(region, coarse);
  const auto coarse_route =
      flux::route_prepared_navmesh(coarse_domain, mm(2.0, 5.0), mm(8.0, 5.0), coarse);

  auto adaptive = coarse;
  adaptive.adaptive_clearance_refinement = true;
  adaptive.maximum_refinement_depth = 4U;
  adaptive.minimum_refined_cell_pitch = Nanometres::from_mm(0.125);
  const auto adaptive_domain = flux::prepare_navmesh_domain(region, adaptive);
  const auto adaptive_route =
      flux::route_prepared_navmesh(adaptive_domain, mm(2.0, 5.0), mm(8.0, 5.0), adaptive);
  flux::RoutingComputeGraphWavefrontInput graph;
  graph.node_count = 64U * 64U;
  graph.maximum_passes = 8'192U;
  graph.sources = {0U};
  graph.offsets.reserve(graph.node_count + 1U);
  for (std::size_t node = 0U; node < graph.node_count; ++node) {
    graph.offsets.push_back(static_cast<std::uint32_t>(graph.edges.size()));
    const auto x = node % 64U;
    const auto y = node / 64U;
    if (x + 1U < 64U)
      graph.edges.push_back({static_cast<std::uint32_t>(node + 1U), 1U});
    if (x > 0U)
      graph.edges.push_back({static_cast<std::uint32_t>(node - 1U), 1U});
    if (y + 1U < 64U)
      graph.edges.push_back({static_cast<std::uint32_t>(node + 64U), 1U});
    if (y > 0U)
      graph.edges.push_back({static_cast<std::uint32_t>(node - 64U), 1U});
  }
  graph.offsets.push_back(static_cast<std::uint32_t>(graph.edges.size()));
  const auto graph_gpu = flux::compute_routing_graph_wavefront(graph, true);
  const auto graph_cpu = flux::compute_routing_graph_wavefront(graph, false);
  require(!coarse_route.ok() && adaptive_route.ok() && adaptive_domain.refined_parent_cells > 0U &&
              adaptive_domain.refined_leaf_cells > 0U &&
              adaptive_domain.maximum_realized_refinement_depth > 0U &&
              adaptive_route.portals.size() + 1U == adaptive_route.corridor.size() &&
              std::all_of(adaptive_route.centerline.begin(), adaptive_route.centerline.end(),
                          [&](Point point) {
                            return flux::navmesh_corridor_contains(adaptive_route, point);
                          }) &&
              flux::validate_navmesh_topology_signature(adaptive_route.topology_signature,
                                                        adaptive_route.centerline)
                  .preserved &&
              graph_gpu.ok() && graph_gpu.used_gpu && graph_cpu.ok() &&
              graph_gpu.distance == graph_cpu.distance,
          "adaptive NavMesh must retain exact mixed-size portals through a passage discarded by "
          "the coarse lattice: coarse=" +
              coarse_route.diagnostic + " adaptive=" + adaptive_route.diagnostic +
              " leaves=" + std::to_string(adaptive_domain.refined_leaf_cells));
}

void advanced_routing_compute_shortlists_preserve_cpu_authority() {
  flux::RoutingComputeQuantizedCandidateInput quantized;
  quantized.candidate_count = 16U;
  quantized.points_per_candidate = 4U;
  quantized.routes = {{0U, 2U, 0U, 0U, 100'000}, {2U, 2U, 0U, 1U, 100'000}};
  quantized.clearance_nm = 100'000;
  quantized.top_k = 16U;
  for (std::size_t candidate = 0U; candidate < quantized.candidate_count; ++candidate) {
    const auto y = static_cast<long double>(candidate) * 50'000.0L;
    quantized.candidate_points_nm.insert(quantized.candidate_points_nm.end(),
                                         {{0.0L, 0.0L},
                                          {1'000'000.0L, 0.0L},
                                          {500'000.0L, -500'000.0L + y},
                                          {500'000.0L, 500'000.0L + y}});
  }
  const auto quantized_cpu = flux::score_quantized_routing_candidates(quantized, false);
  const auto quantized_gpu = flux::score_quantized_routing_candidates(quantized, true);

  flux::RoutingComputeAssignmentInput assignments;
  assignments.maximum_states = 4'096U;
  assignments.top_k = 32U;
  assignments.option_costs.resize(24U);
  std::iota(assignments.option_costs.begin(), assignments.option_costs.end(), 0U);
  for (std::uint32_t variable = 0U; variable < 6U; ++variable)
    assignments.variable_domains.push_back(
        {variable * 4U, variable * 4U + 1U, variable * 4U + 2U, variable * 4U + 3U});
  assignments.incompatible_options = {{0U, 4U}, {1U, 9U}, {7U, 22U}};
  const auto assignment_cpu = flux::evaluate_routing_assignments(assignments, false);
  const auto assignment_gpu = flux::evaluate_routing_assignments(assignments, true);

  flux::RoutingComputeLaneOrderInput lane;
  lane.widths_nm.assign(7U, 100'000U);
  lane.clearances_nm = {30'000U, 40'000U, 20'000U, 50'000U, 25'000U, 35'000U, 30'000U};
  lane.reference_position = {6U, 5U, 4U, 3U, 2U, 1U, 0U};
  lane.coupled_rank.assign(7U, std::numeric_limits<std::uint32_t>::max());
  lane.usable_width_nm = 1'500'000U;
  const auto lane_cpu = flux::solve_routing_lane_order(lane, false);
  const auto lane_gpu = flux::solve_routing_lane_order(lane, true);

  flux::RoutingComputeOrientationInput orientation;
  orientation.bit_count = 8U;
  orientation.top_k = 16U;
  orientation.bit_flip_costs.assign(8U, 1U);
  orientation.forbidden_clauses = {{1U, 1U}, {6U, 6U}};
  const auto orientation_cpu = flux::evaluate_routing_orientations(orientation, false);
  const auto orientation_gpu = flux::evaluate_routing_orientations(orientation, true);

  std::vector<flux::RoutingComputeCopperPrimitive> copper{
      {flux::RoutingComputeCopperKind::segment, 0, 0, 1'000'000, 0, 100'000, 0U, 0U},
      {flux::RoutingComputeCopperKind::circle, 500'000, 50'000, 500'000, 50'000, 100'000, 0U, 1U}};
  std::vector<std::pair<std::uint32_t, std::uint32_t>> copper_pairs(256U, {0U, 1U});
  const auto copper_cpu =
      flux::assess_simple_copper_pairs(copper, copper_pairs, 100'000, 2'048, false);
  const auto copper_gpu =
      flux::assess_simple_copper_pairs(copper, copper_pairs, 100'000, 2'048, true);

  flux::RoutingComputeBatchedGraphWavefrontInput graph;
  graph.node_count = 2'048U;
  graph.maximum_passes = 2'048U;
  graph.offsets.reserve(graph.node_count + 1U);
  for (std::size_t node = 0U; node < graph.node_count; ++node) {
    graph.offsets.push_back(static_cast<std::uint32_t>(graph.edges.size()));
    if (node > 0U)
      graph.edges.push_back({static_cast<std::uint32_t>(node - 1U), 1U});
    if (node + 1U < graph.node_count)
      graph.edges.push_back({static_cast<std::uint32_t>(node + 1U), 1U});
  }
  graph.offsets.push_back(static_cast<std::uint32_t>(graph.edges.size()));
  graph.source_groups = {{0U}, {graph.node_count - 1U}};
  const auto graph_cpu = flux::compute_batched_routing_graph_wavefront(graph, false);
  const auto graph_gpu = flux::compute_batched_routing_graph_wavefront(graph, true);

  std::vector<flux::RoutingComputeArcCandidate> arcs(4U);
  for (std::size_t index = 0U; index < arcs.size(); ++index) {
    arcs[index].points_nm = {
        {{0, 0}, {100'000, 0}, {200'000, 100'000}, {300'000, 200'000}, {400'000, 200'000}}};
    arcs[index].radius_nm = static_cast<std::int64_t>(index + 1U) * 100'000;
    arcs[index].copper_radius_nm = 50'000;
  }
  const auto arcs_cpu = flux::score_routing_arc_candidates(arcs, {}, 100'000, 4U, false);
  const auto arcs_gpu = flux::score_routing_arc_candidates(arcs, {}, 100'000, 4U, true);

  std::vector<flux::RoutingComputeNegotiatedResource> resources(256U, {3U, 1U, 7U, 2U});
  const auto resources_cpu = flux::reduce_negotiated_routing_resources(resources, false);
  const auto resources_gpu = flux::reduce_negotiated_routing_resources(resources, true);

  std::vector<flux::RoutingComputePrimitiveBounds> offset_hulls;
  offset_hulls.reserve(64U);
  for (std::size_t owner = 0U; owner < 2U; ++owner) {
    for (std::size_t index = 0U; index < 32U; ++index) {
      const auto x = static_cast<std::int64_t>(index) * 100'000;
      const auto shift = owner == 0U ? 0 : 25'000;
      offset_hulls.push_back({x + shift, 0, x + shift + 50'000, 50'000, 0U,
                              static_cast<std::uint32_t>(owner)});
    }
  }
  const auto hulls_cpu =
      flux::discover_routing_broadphase_pairs(offset_hulls, 100'000, 1'024U, false);
  const auto hulls_gpu =
      flux::discover_routing_broadphase_pairs(offset_hulls, 100'000, 1'024U, true);

  require(
      quantized_cpu.ok() && quantized_gpu.ok() && quantized_gpu.used_gpu &&
          quantized_cpu.shortlist.front().candidate == quantized_gpu.shortlist.front().candidate &&
          assignment_cpu.ok() && assignment_gpu.ok() && assignment_gpu.used_gpu &&
          assignment_cpu.shortlist.front().options == assignment_gpu.shortlist.front().options &&
          lane_cpu.ok() && lane_gpu.ok() && lane_gpu.used_gpu && lane_cpu.physically_fits &&
          lane_gpu.physically_fits && orientation_cpu.states == orientation_gpu.states &&
          orientation_gpu.used_gpu && copper_cpu.ok() && copper_gpu.ok() && copper_gpu.used_gpu &&
          copper_cpu.pairs.front().possible_conflict ==
              copper_gpu.pairs.front().possible_conflict &&
          graph_cpu.ok() && graph_gpu.ok() && graph_gpu.used_gpu &&
          graph_cpu.distance == graph_gpu.distance && arcs_cpu.ok() && arcs_gpu.ok() &&
          arcs_gpu.used_gpu &&
          arcs_cpu.shortlist.front().candidate == arcs_gpu.shortlist.front().candidate &&
          hulls_cpu.ok() && hulls_gpu.ok() && hulls_gpu.used_gpu &&
          hulls_cpu.candidate_pairs == hulls_gpu.candidate_pairs &&
          resources_cpu.overflow == resources_gpu.overflow && resources_gpu.used_gpu,
      "advanced GPU shortlists/reductions must agree with their deterministic CPU authority");
}

void physical_copper_t_junction_is_preserved_during_ph_realization() {
  auto source = two_pad_board();
  auto branch_footprint = source.footprints.front();
  branch_footprint.id = ObjectId{"U3"};
  branch_footprint.reference = "U3";
  branch_footprint.position = mm(50.0, 60.0);
  branch_footprint.pads.front().id = ObjectId{"U3-pad"};
  source.footprints.push_back(std::move(branch_footprint));

  flux::GlobalRouteResult global;
  global.stop_reason = GlobalRouterStopReason::complete;
  flux::GlobalRoutePath main;
  main.net = NetId{"net-1"};
  main.connection_index = 0U;
  main.points = {{mm(20.0, 40.0), LayerId{"F.Cu"}},
                 {mm(50.0, 40.0), LayerId{"F.Cu"}},
                 {mm(50.0, 60.0), LayerId{"F.Cu"}}};
  flux::GlobalRoutePath branch;
  branch.net = NetId{"net-1"};
  branch.connection_index = 1U;
  branch.points = {{mm(80.0, 40.0), LayerId{"F.Cu"}}, {mm(50.0, 41.0), LayerId{"F.Cu"}}};
  global.paths = {std::move(main), std::move(branch)};

  flux::DetailedRouterConfig config;
  config.pull_tight_45_degrees = false;
  config.continuous_refinement = false;
  config.circular_arc_refinement = true;
  config.spline_minimum_bend_radius = Nanometres{0};
  config.maximum_circular_arc_radius = Nanometres::from_mm(1.0);
  const auto detailed = flux::materialize_detailed_route(source, global, config);
  require(detailed.complete() && detailed.candidate &&
              detailed.connectivity.open_connections == 0U &&
              detailed.unrounded_non_junction_bends == 0U,
          "an exact same-net branch ending inside an unsplit carrier must become a fixed PH "
          "attachment boundary before the adjacent non-junction corner is rounded: " +
              detailed.diagnostic);

  // A terminal is itself the third electrical incidence at a two-track centreline meeting. The
  // PH run must remain anchored there, while the final quality audit must not confuse that
  // intentional terminal junction with a free-space polygonal bend.
  auto terminal_global = global;
  terminal_global.paths[1].points.back().position = mm(50.0, 60.0);
  const auto terminal = flux::materialize_detailed_route(source, terminal_global, config);
  require(terminal.complete() && terminal.candidate &&
              terminal.connectivity.open_connections == 0U &&
              terminal.unrounded_non_junction_bends == 0U,
          "two same-net tracks meeting at a pad centre must be classified as an anchored terminal "
          "junction rather than an unrealized free-space PH bend: " +
              terminal.diagnostic);
}

void same_net_vertical_features_are_reused_as_transition_portals() {
  auto via_board = two_pad_board();
  via_board.footprints.back().pads.front().layers = {LayerId{"B.Cu"}};
  const auto via_position = mm(50.3, 40.2);
  via_board.vias.push_back(flux::Via{ObjectId{"existing-net-via"},
                                     NetId{"net-1"},
                                     via_position,
                                     Nanometres::from_mm(0.8),
                                     Nanometres::from_mm(0.4),
                                     {LayerId{"F.Cu"}, LayerId{"B.Cu"}},
                                     true});
  GlobalRouterConfig config;
  config.grid_pitch = Nanometres::from_mm(5.0);
  config.via_site_pitch = Nanometres::from_mm(100.0);
  config.via_site_phase_x = 19U;
  config.via_site_phase_y = 19U;
  config.max_iterations = 2U;
  const auto global = flux::route_global(via_board, config);
  const auto exact_transition =
      std::any_of(global.paths.begin(), global.paths.end(), [&](const auto& path) {
        for (std::size_t index = 1U; index < path.points.size(); ++index) {
          if (path.points[index - 1U].position == via_position &&
              path.points[index].position == via_position &&
              path.points[index - 1U].layer != path.points[index].layer) {
            return true;
          }
        }
        return false;
      });
  require(global.complete() && global.reused_existing_via_transitions > 0U && exact_transition,
          "an off-grid same-net via must become a preferred exact transition portal when the "
          "ordinary via lattice has no usable site: " +
              global.diagnostic);

  flux::DetailedRouterConfig detailed_config;
  detailed_config.pull_tight_45_degrees = false;
  detailed_config.continuous_refinement = false;
  detailed_config.circular_arc_refinement = false;
  const auto detailed = flux::materialize_detailed_route(via_board, global, detailed_config);
  require(detailed.complete() && detailed.candidate && detailed.added_vias == 0U &&
              detailed.candidate->vias.size() == 1U,
          "materialization must retain a covering locked same-net via instead of adding "
          "concentric copper: " +
              detailed.diagnostic);

  auto pad_board = two_pad_board();
  pad_board.footprints.back().pads.front().layers = {LayerId{"B.Cu"}};
  Footprint through_hole;
  through_hole.id = ObjectId{"J1"};
  through_hole.reference = "J1";
  through_hole.position = via_position;
  through_hole.courtyard = local_rectangle(3.0, 2.0);
  flux::Pad plated_pad{
      ObjectId{"J1-pad"}, NetId{"net-1"}, Point{}, {LayerId{"F.Cu"}, LayerId{"B.Cu"}}};
  plated_pad.width = Nanometres::from_mm(1.6);
  plated_pad.height = Nanometres::from_mm(1.2);
  plated_pad.shape = flux::PadShape::oval;
  plated_pad.drill_width = Nanometres::from_mm(0.6);
  plated_pad.drill_height = Nanometres::from_mm(0.6);
  plated_pad.drill_offset = mm(0.2, 0.0);
  plated_pad.plated = true;
  through_hole.pads.push_back(std::move(plated_pad));
  pad_board.footprints.push_back(std::move(through_hole));
  const auto pad_hole_position = mm(50.5, 40.2);
  const auto global_pad_route = flux::route_global(pad_board, config);
  const auto exact_pad_transition =
      std::any_of(global_pad_route.paths.begin(), global_pad_route.paths.end(), [&](const auto& p) {
        for (std::size_t index = 1U; index < p.points.size(); ++index) {
          if (p.points[index - 1U].position == pad_hole_position &&
              p.points[index].position == pad_hole_position &&
              p.points[index - 1U].layer != p.points[index].layer) {
            return true;
          }
        }
        return false;
      });
  require(global_pad_route.complete() &&
              global_pad_route.reused_component_pad_transitions > 0U && exact_pad_transition,
          "a plated pad must expose its offset physical hole, rather than its copper anchor, as "
          "the reusable global layer-transition portal: " +
              global_pad_route.diagnostic);
  flux::GlobalRouteResult pad_route;
  pad_route.stop_reason = GlobalRouterStopReason::complete;
  flux::GlobalRoutePath path;
  path.net = NetId{"net-1"};
  path.points = {{mm(20.0, 40.0), LayerId{"F.Cu"}},
                 {pad_hole_position, LayerId{"F.Cu"}},
                 {pad_hole_position, LayerId{"B.Cu"}},
                 {mm(80.0, 40.0), LayerId{"B.Cu"}}};
  pad_route.paths.push_back(std::move(path));
  const auto through_hole_detailed =
      flux::materialize_detailed_route(pad_board, pad_route, detailed_config);
  require(through_hole_detailed.complete() && through_hole_detailed.candidate &&
              through_hole_detailed.added_vias == 0U &&
              through_hole_detailed.candidate->vias.empty(),
          "a plated through-hole component pad must provide the vertical connection without a "
          "synthetic via: " +
              through_hole_detailed.diagnostic);
}

void kicad_zone_refill_adapter_is_bounded_and_fail_closed() {
#ifdef FLUX_TEST_PYTHON_EXECUTABLE
  const auto nonce = static_cast<std::uint64_t>(
      std::chrono::steady_clock::now().time_since_epoch().count());
  const auto root = std::filesystem::temp_directory_path() /
                    ("flux-kicad-refill-test-" + std::to_string(nonce));
  std::error_code error;
  require(std::filesystem::create_directory(root, error) && !error,
          "zone-refill test must create an isolated temporary directory");
  struct Cleanup {
    std::filesystem::path path;
    ~Cleanup() {
      std::error_code ignored;
      std::filesystem::remove_all(path, ignored);
    }
  } cleanup{root};
  const auto provisional = root / "provisional.kicad_pcb";
  {
    std::ofstream stream(provisional, std::ios::binary);
    stream << "provisional-board\n";
  }
  const auto passing_helper = root / "passing refill helper.py";
  {
    std::ofstream stream(passing_helper, std::ios::binary);
    stream << R"PY(import json
import pathlib
import shutil
import sys
arguments = dict(zip(sys.argv[1::2], sys.argv[2::2]))
shutil.copyfile(pathlib.Path(arguments["--board"]), pathlib.Path(arguments["--output"]))
pathlib.Path(arguments["--report"]).write_text(json.dumps({"ok": True}), encoding="utf-8")
print("non-JSON diagnostic before the dedicated report")
)PY";
  }
  const auto passing_directory = root / "passing";
  require(std::filesystem::create_directory(passing_directory, error) && !error,
          "zone-refill test must create a passing diagnostic directory");
  const auto passing_output = passing_directory / "refilled.kicad_pcb";
  const auto passing = flux::io::run_kicad_zone_refill(
      {std::filesystem::path{FLUX_TEST_PYTHON_EXECUTABLE}, passing_helper, provisional,
       passing_output, passing_directory, std::chrono::seconds{5}, 4U * 1024U,
       1U * 1024U * 1024U});
  require(passing.run && passing.passed && passing.exit_code == 0 &&
              std::filesystem::is_regular_file(passing_output),
          "a shell-free bounded refill adapter must accept an explicit successful JSON/artifact "
          "transaction: " + passing.diagnostic);

  const auto failing_helper = root / "failing refill helper.py";
  {
    std::ofstream stream(failing_helper, std::ios::binary);
    stream << "import json, sys\nprint(json.dumps({'ok': False, 'error': 'intentional'}))\n"
              "sys.exit(5)\n";
  }
  const auto failing_directory = root / "failing";
  error.clear();
  require(std::filesystem::create_directory(failing_directory, error) && !error,
          "zone-refill test must create a failing diagnostic directory");
  const auto failing = flux::io::run_kicad_zone_refill(
      {std::filesystem::path{FLUX_TEST_PYTHON_EXECUTABLE}, failing_helper, provisional,
       failing_directory / "must-not-exist.kicad_pcb", failing_directory,
       std::chrono::seconds{5}, 4U * 1024U, 1U * 1024U * 1024U});
  require(failing.run && !failing.passed && failing.exit_code == 5,
          "a nonzero refill helper result must remain a failed transaction");

  const auto unavailable_directory = root / "unavailable";
  error.clear();
  require(std::filesystem::create_directory(unavailable_directory, error) && !error,
          "zone-refill test must create an unavailable diagnostic directory");
  const auto unavailable = flux::io::run_kicad_zone_refill(
      {std::filesystem::path{FLUX_TEST_PYTHON_EXECUTABLE}, root / "missing-helper.py",
       provisional, unavailable_directory / "must-not-exist.kicad_pcb",
       unavailable_directory, std::chrono::seconds{5}, 4U * 1024U,
       1U * 1024U * 1024U});
  require(!unavailable.run && !unavailable.passed && unavailable.exit_code == -1,
          "an unavailable refill helper must fail before a child process is launched");
#endif
}

void work_plan_round_trip_is_strict_source_bound_and_non_authoritative() {
  const auto board = two_pad_board();
  auto plan = flux::io::make_flux_work_plan_template(board, "board.kicad_pcb");
  plan.mode = flux::io::WorkPlanMode::force_reroute_selected;
  flux::io::WorkPlanStep route;
  route.id = "route-signals";
  route.kind = flux::io::WorkPlanStepKind::route;
  route.selectors.exact_nets = {"SIGNAL"};
  route.options = {{"arcs", true},
                   {"max_router_iterations", std::uint64_t{30U}},
                   {"priority_nets", std::vector<std::string>{"SIGNAL"}},
                   {"signed_test", std::int64_t{-1}},
                   {"strategy", std::string{"scarcity"}}};
  flux::io::WorkPlanStep validation;
  validation.id = "validate";
  validation.kind = flux::io::WorkPlanStepKind::validation;
  validation.depends_on = {route.id};
  plan.steps = {route, validation};

  const auto serialized = flux::io::serialize_flux_work_plan(plan);
  const auto parsed = flux::io::parse_flux_work_plan(serialized, &board);
  require(parsed.ok() && parsed.plan && *parsed.plan == plan &&
              flux::io::serialize_flux_work_plan(*parsed.plan) == serialized,
          "FluxWorkPlan must round-trip byte-stably without changing typed options");
  require(flux::io::required_flux_work_plan_capabilities(plan) ==
              std::vector<std::string>{"routing.single-ended", "routing.scope",
                                       "verification.diagnostics"},
          "ordered step kinds must produce stable capability preflight IDs");

  auto stale = board;
  stale.source_revision += "-changed";
  const auto stale_result = flux::io::parse_flux_work_plan(serialized, &stale);
  require(
      !stale_result.ok() &&
          std::any_of(stale_result.diagnostics.begin(), stale_result.diagnostics.end(),
                      [](const auto& diagnostic) { return diagnostic.code == "stale_work_plan"; }),
      "a saved plan must fail closed against another source revision");

  auto type_confused = serialized;
  const auto enabled = type_confused.find("\"enabled\":true");
  require(enabled != std::string::npos, "serialized plan must contain typed telemetry state");
  type_confused.replace(enabled, std::string{"\"enabled\":true"}.size(), "\"enabled\":\"true\"");
  const auto confused_result = flux::io::parse_flux_work_plan(type_confused, &board);
  require(!confused_result.ok() &&
              std::any_of(confused_result.diagnostics.begin(), confused_result.diagnostics.end(),
                          [](const auto& diagnostic) {
                            return diagnostic.code == "invalid_work_plan_boolean";
                          }),
          "the plan parser must reject JSON type confusion rather than coerce strings");

  auto scoped_board = two_pad_board();
  const NetId signal_a{"a"};
  const NetId signal_b{"b"};
  const NetId power{"p"};
  const NetId auxiliary{"x"};
  const flux::NetClassId power_class{"power-class"};
  scoped_board.nets = {{signal_a, "SIG_A", 0},
                       {signal_b, "SIG_B", 0},
                       {power, "PWR", 0, {power_class}},
                       {auxiliary, "AUX", 0}};
  flux::NetClass net_class;
  net_class.id = power_class;
  net_class.name = "Power";
  scoped_board.net_classes = {net_class};
  scoped_board.footprints[0].pads = {
      {ObjectId{"u1-a"}, signal_a, Point{}, {LayerId{"F.Cu"}}},
      {ObjectId{"u1-b"}, signal_b, Point{}, {LayerId{"F.Cu"}}}};
  scoped_board.footprints[1].pads = {
      {ObjectId{"u2-a"}, signal_a, Point{}, {LayerId{"F.Cu"}}},
      {ObjectId{"u2-p"}, power, Point{}, {LayerId{"F.Cu"}}}};
  Footprint third;
  third.id = ObjectId{"U3"};
  third.reference = "U3";
  third.position = mm(50.0, 15.0);
  third.courtyard = local_rectangle(4.0, 3.0);
  third.pads = {{ObjectId{"u3-b"}, signal_b, Point{}, {LayerId{"F.Cu"}}},
                {ObjectId{"u3-p"}, power, Point{}, {LayerId{"F.Cu"}}},
                {ObjectId{"u3-x"}, auxiliary, Point{}, {LayerId{"F.Cu"}}}};
  scoped_board.footprints.push_back(third);
  scoped_board.placement_groups = {
      {flux::PlacementGroupId{"core-id"}, "core", {ObjectId{"U1"}, ObjectId{"U2"}}}};
  scoped_board.tracks = {
      {ObjectId{"track-a"}, signal_a, LayerId{"F.Cu"}, mm(1.0, 1.0), mm(2.0, 1.0),
       Nanometres::from_mm(0.25), false},
      {ObjectId{"track-p-locked"}, power, LayerId{"F.Cu"}, mm(3.0, 1.0), mm(4.0, 1.0),
       Nanometres::from_mm(0.25), true},
      {ObjectId{"track-x"}, auxiliary, LayerId{"F.Cu"}, mm(5.0, 1.0), mm(6.0, 1.0),
       Nanometres::from_mm(0.25), false}};
  scoped_board.track_arcs = {{ObjectId{"arc-p"}, power, LayerId{"F.Cu"}, mm(7.0, 1.0),
                              mm(7.5, 1.5), mm(8.0, 1.0), Nanometres::from_mm(0.25), false}};
  scoped_board.vias = {{ObjectId{"via-p"}, power, mm(9.0, 1.0), Nanometres::from_mm(0.6),
                        Nanometres::from_mm(0.3), {LayerId{"F.Cu"}, LayerId{"B.Cu"}}, false},
                       {ObjectId{"via-x-locked"}, auxiliary, mm(10.0, 1.0),
                        Nanometres::from_mm(0.6), Nanometres::from_mm(0.3),
                        {LayerId{"F.Cu"}, LayerId{"B.Cu"}}, true}};
  flux::CopperZone immutable_zone;
  immutable_zone.id = ObjectId{"zone-p"};
  immutable_zone.net = power;
  scoped_board.zones = {immutable_zone};

  auto scoped_plan = flux::io::make_flux_work_plan_template(scoped_board, "scoped.kicad_pcb");
  scoped_plan.mode = flux::io::WorkPlanMode::force_reroute_selected;
  flux::io::WorkPlanStep scoped_route;
  scoped_route.id = "scoped-route";
  scoped_route.kind = flux::io::WorkPlanStepKind::route;
  scoped_route.selectors.include_globs = {"SIG_*", "A*"};
  scoped_route.selectors.exclude_globs = {"SIG_B"};
  scoped_route.selectors.net_classes = {"Power"};
  scoped_route.selectors.placement_groups = {"core"};
  scoped_route.selectors.group_scope = flux::io::WorkPlanSelectorScope::internal;
  scoped_route.options = {{"atomic_complete_net_routing", true},
                          {"capacity_reserve_cost", std::uint64_t{777U}},
                          {"capacity_reserve_per_mille", std::uint64_t{300U}},
                          {"circular_arc_refinement", true},
                          {"clearance_shadow_cost_quantum_nm", std::uint64_t{10'000U}},
                          {"corridor_neck_down_width_nm", std::uint64_t{130'000U}},
                          {"crossing_safe_diagonal_grid_edges", false},
                          {"defer_hard_capacity_ripup_victims_to_pass_tail", true},
                          {"diagonal_grid_edges", true},
                          {"differential_pair_skew_tuning_batch_assignment", true},
                          {"expansion_retry_heuristic_weight_per_mille",
                           std::uint64_t{2'500U}},
                          {"former_route_memory_cost", std::uint64_t{1'777U}},
                          {"grid_pitch_nm", std::uint64_t{250'000U}},
                          {"hard_capacity_start_iteration", std::uint64_t{3U}},
                          {"local_pin_escape_pitch_nm", std::uint64_t{50'000U}},
                          {"max_corridor_neck_down_length_nm", std::uint64_t{700'000U}},
                          {"max_corridor_neck_down_states", std::uint64_t{12'345U}},
                          {"max_hard_capacity_blocker_reuse_invalidations_per_failed_net",
                           std::uint64_t{2U}},
                          {"max_hard_capacity_conflict_set_states_per_attempt",
                           std::uint64_t{64U}},
                          {"max_hard_capacity_ripup_attempts_per_pass", std::uint64_t{2U}},
                          {"max_hard_capacity_ripup_nets_per_attempt", std::uint64_t{3U}},
                          {"max_final_routability_feedback_passes", std::uint64_t{7U}},
                          {"max_local_pin_escape_expansions_per_terminal",
                           std::uint64_t{23'456U}},
                          {"max_medial_neck_down_length_nm", std::uint64_t{600'000U}},
                          {"max_pin_access_nodes_per_terminal", std::uint64_t{24U}},
                          {"max_post_negotiation_sampled_conflict_repair_cones",
                           std::uint64_t{6U}},
                          {"max_present_congestion_scale_per_mille",
                           std::uint64_t{4'800U}},
                          {"max_router_iterations", std::uint64_t{42U}},
                          {"max_terminal_escape_reservation_resources",
                           std::uint64_t{1'234U}},
                          {"max_terminal_neck_down_length_nm", std::uint64_t{500'000U}},
                          {"max_voronoi_resource_segment_length_nm",
                           std::uint64_t{2'000'000U}},
                          {"medial_neck_down_width_nm", std::uint64_t{140'000U}},
                          {"physical_envelope_ordering", true},
                          {"physical_overflow_history", true},
                          {"pin_escape_grid_radius", std::uint64_t{5U}},
                          {"post_negotiation_sampled_conflict_repair", true},
                          {"present_congestion_escalation_start_iteration",
                           std::uint64_t{3U}},
                          {"present_congestion_multiplier_per_mille",
                           std::uint64_t{1'200U}},
                          {"priority_nets", std::vector<std::string>{"PWR", "SIG_A"}},
                          {"protect_recovered_hard_capacity_ripup_nets", true},
                          {"reuse_weighted_search_frontier", true},
                          {"river_following_cost", std::uint64_t{333U}},
                          {"rollback_failed_hard_capacity_ripup", true},
                          {"sampled_route_clearance_reservations", true},
                          {"search_diversification", true},
                          {"selective_congestion_reroute", true},
                          {"selective_full_reroute_interval", std::uint64_t{5U}},
                          {"terminal_escape_proximity_cost", std::uint64_t{66U}},
                          {"terminal_escape_reservation_cost", std::uint64_t{555U}},
                          {"terminal_neck_down_width_nm", std::uint64_t{150'000U}},
                          {"via_site_phase_x", std::uint64_t{1U}},
                          {"via_site_phase_y", std::uint64_t{3U}},
                          {"via_site_pitch_nm", std::uint64_t{1'000'000U}},
                          {"width_reduction_cost_per_grid_step", std::uint64_t{444U}},
                          {"xpbd_feedback", true},
                          {"xpbd_iterations", std::uint64_t{17U}},
                          {"xpbd_maximum_step_nm", std::uint64_t{80'000U}},
                          {"xpbd_prefer_gpu", true}};
  scoped_plan.steps = {scoped_route};
  const auto expansion = flux::io::expand_flux_work_plan(scoped_board, scoped_plan);
  require(expansion.ok() && expansion.steps.size() == 1U &&
              expansion.steps.front().nets == std::vector<NetId>{signal_a, power, auxiliary} &&
              expansion.steps.front().footprints ==
                  std::vector<ObjectId>{ObjectId{"U1"}, ObjectId{"U2"}} &&
              expansion.steps.front().wildcard_only_nets == std::vector<NetId>{auxiliary} &&
              expansion.copper_authority.reroutable_nets ==
                  std::vector<NetId>{signal_a, power} &&
              expansion.copper_authority.removable_tracks ==
                  std::vector<ObjectId>{ObjectId{"track-a"}} &&
              expansion.copper_authority.removable_track_arcs ==
                  std::vector<ObjectId>{ObjectId{"arc-p"}} &&
              expansion.copper_authority.removable_vias ==
                  std::vector<ObjectId>{ObjectId{"via-p"}} &&
              expansion.copper_authority.immutable_tracks ==
                  std::vector<ObjectId>{ObjectId{"track-p-locked"}, ObjectId{"track-x"}} &&
              expansion.copper_authority.immutable_vias ==
                  std::vector<ObjectId>{ObjectId{"via-x-locked"}} &&
              expansion.copper_authority.immutable_zones ==
                  std::vector<ObjectId>{ObjectId{"zone-p"}} &&
              expansion.selected_net_mask == std::vector<std::uint8_t>({1U, 0U, 1U, 1U}) &&
              expansion.reroutable_net_mask == std::vector<std::uint8_t>({1U, 0U, 1U, 0U}),
          "selector expansion must union exact physical scopes, apply exclusions last, expose "
          "device-ready masks, and keep wildcard-only or locked source copper immutable");
  const auto compiled = flux::io::compile_flux_work_plan_routes(scoped_board, scoped_plan);
  require(compiled.ok() && compiled.route_steps.size() == 1U &&
              compiled.route_steps.front().selected_nets ==
                  std::vector<NetId>{signal_a, power, auxiliary} &&
              compiled.route_steps.front().priority_nets == std::vector<NetId>{power, signal_a} &&
              compiled.route_steps.front().configuration.planner.global.grid_pitch ==
                  Nanometres{250'000} &&
              compiled.route_steps.front().configuration.planner.global.via_site_pitch ==
                  Nanometres{1'000'000} &&
              compiled.route_steps.front().configuration.planner.global.via_site_phase_x == 1U &&
              compiled.route_steps.front().configuration.planner.global.via_site_phase_y == 3U &&
              compiled.route_steps.front()
                      .configuration.planner.global.max_pin_escape_grid_radius == 5U &&
              compiled.route_steps.front()
                      .configuration.planner.global.max_pin_access_nodes_per_terminal == 24U &&
              compiled.route_steps.front().configuration.planner.global.local_pin_escape_pitch ==
                  Nanometres{50'000} &&
              compiled.route_steps.front()
                      .configuration.planner.global
                      .max_local_pin_escape_expansions_per_terminal == 23'456U &&
              compiled.route_steps.front().configuration.planner.global.terminal_neck_down_width ==
                  Nanometres{150'000} &&
              compiled.route_steps.front()
                      .configuration.planner.global.max_terminal_neck_down_length ==
                  Nanometres{500'000} &&
              compiled.route_steps.front().configuration.planner.global.medial_neck_down_width ==
                  Nanometres{140'000} &&
              compiled.route_steps.front()
                      .configuration.planner.global.max_medial_neck_down_length ==
                  Nanometres{600'000} &&
              compiled.route_steps.front().configuration.planner.global.corridor_neck_down_width ==
                  Nanometres{130'000} &&
              compiled.route_steps.front()
                      .configuration.planner.global.max_corridor_neck_down_length ==
                  Nanometres{700'000} &&
              compiled.route_steps.front()
                      .configuration.planner.global.max_corridor_neck_down_states == 12'345U &&
              compiled.route_steps.front()
                      .configuration.planner.global.max_voronoi_resource_segment_length ==
                  Nanometres{2'000'000} &&
              compiled.route_steps.front().configuration.planner.global.max_iterations == 42U &&
              compiled.route_steps.front().max_final_routability_feedback_passes == 7U &&
              compiled.route_steps.front().post_negotiation_sampled_conflict_repair &&
              compiled.route_steps.front()
                      .max_post_negotiation_sampled_conflict_repair_cones == 6U &&
              compiled.route_steps.front().configuration.planner.global.allow_diagonal_grid_edges &&
              !compiled.route_steps.front()
                   .configuration.planner.global.allow_crossing_safe_diagonal_grid_edges &&
              compiled.route_steps.front()
                  .configuration.planner.global.reserve_sampled_route_clearance &&
              compiled.route_steps.front()
                      .configuration.planner.global.expansion_retry_heuristic_weight_per_mille ==
                  2'500U &&
              compiled.route_steps.front()
                  .configuration.planner.global.reuse_weighted_search_frontier &&
              compiled.route_steps.front()
                      .configuration.planner.global.present_congestion_multiplier_per_mille ==
                  1'200U &&
              compiled.route_steps.front()
                      .configuration.planner.global.max_present_congestion_scale_per_mille ==
                  4'800U &&
              compiled.route_steps.front()
                      .configuration.planner.global
                      .present_congestion_escalation_start_iteration == 3U &&
              compiled.route_steps.front()
                  .configuration.planner.global.scale_voronoi_history_by_overflow &&
              compiled.route_steps.front()
                  .configuration.planner.global.order_routing_groups_by_physical_envelope &&
              compiled.route_steps.front()
                      .configuration.planner.global.clearance_shadow_cost_quantum ==
                  Nanometres{10'000} &&
              compiled.route_steps.front().configuration.planner.global.capacity_reserve_per_mille ==
                  300U &&
              compiled.route_steps.front().configuration.planner.global.capacity_reserve_cost ==
                  777U &&
              compiled.route_steps.front().configuration.planner.global.river_following_cost ==
                  333U &&
              compiled.route_steps.front()
                      .configuration.planner.global.terminal_escape_reservation_cost == 555U &&
              compiled.route_steps.front()
                      .configuration.planner.global.terminal_escape_proximity_cost == 66U &&
              compiled.route_steps.front()
                      .configuration.planner.global
                      .max_terminal_escape_reservation_resources == 1'234U &&
              compiled.route_steps.front()
                      .configuration.planner.global.width_reduction_cost_per_grid_step == 444U &&
              compiled.route_steps.front()
                      .configuration.planner.global.hard_capacity_start_iteration == 3U &&
              compiled.route_steps.front()
                      .configuration.planner.global.max_hard_capacity_ripup_attempts_per_pass ==
                  2U &&
              compiled.route_steps.front()
                      .configuration.planner.global.max_hard_capacity_ripup_nets_per_attempt ==
                  3U &&
              compiled.route_steps.front()
                      .configuration.planner.global
                      .max_hard_capacity_conflict_set_states_per_attempt == 64U &&
              compiled.route_steps.front()
                      .configuration.planner.global
                      .max_hard_capacity_blocker_reuse_invalidations_per_failed_net == 2U &&
              compiled.route_steps.front()
                      .configuration.planner.global.rollback_failed_hard_capacity_ripup &&
              compiled.route_steps.front()
                      .configuration.planner.global
                      .defer_hard_capacity_ripup_victims_to_pass_tail &&
              compiled.route_steps.front()
                      .configuration.planner.global.protect_recovered_hard_capacity_ripup_nets &&
              compiled.route_steps.front().configuration.planner.global
                      .selective_full_reroute_interval == 5U &&
              compiled.route_steps.front().configuration.planner.global.former_route_memory_cost ==
                  1'777U &&
              compiled.route_steps.front().configuration.planner.global
                  .enable_search_diversification &&
              compiled.route_steps.front().configuration.planner.global.atomic_complete_net_routing &&
              compiled.route_steps.front().configuration.planner.global
                  .selective_congestion_reroute &&
              compiled.route_steps.front().configuration.planner.regional.enable_xpbd_relaxation &&
              compiled.route_steps.front().configuration.planner.regional.enable_xpbd_feedback &&
              compiled.route_steps.front().configuration.planner.regional.xpbd.iterations == 17U &&
              compiled.route_steps.front().configuration.planner.regional.xpbd.maximum_step_nm ==
                  80'000 &&
              compiled.route_steps.front().configuration.planner.regional.xpbd.prefer_gpu &&
              compiled.route_steps.front().configuration.detailed.circular_arc_refinement &&
              compiled.route_steps.front().configuration.detailed.differential_pair_skew_tuning &&
              compiled.route_steps.front()
                  .configuration.detailed.differential_pair_skew_tuning_batch_assignment &&
              compiled.route_steps.front().configuration.detailed.existing_routing ==
                  flux::ExistingRoutingPolicy::replace_unlocked_for_routed_nets &&
              compiled.route_steps.front().configuration.detailed.existing_routing_replacement_scope ==
                  std::optional<std::vector<NetId>>{std::vector<NetId>{signal_a, power}},
          "route options must compile into the shared autoroute_v1 configuration without giving "
          "wildcard-only nets source-copper replacement authority");

  auto invalid_recovery_topology_plan = scoped_plan;
  invalid_recovery_topology_plan.steps.front().options["strict_medial_topology"] = false;
  invalid_recovery_topology_plan.steps.front().options["direct_medial_edges"] = false;
  const auto invalid_recovery_topology =
      flux::io::compile_flux_work_plan_routes(scoped_board, invalid_recovery_topology_plan);
  require(!invalid_recovery_topology.ok() &&
              std::any_of(invalid_recovery_topology.diagnostics.begin(),
                          invalid_recovery_topology.diagnostics.end(), [](const auto& diagnostic) {
                            return diagnostic.code ==
                                   "invalid_work_plan_route_option_dependency";
                          }),
          "hard-capacity WorkPlan recovery must fail closed without a represented medial "
          "topology");

  auto incomplete_recovery_bounds_plan = scoped_plan;
  incomplete_recovery_bounds_plan.steps.front().options.erase(
      "max_hard_capacity_ripup_nets_per_attempt");
  const auto incomplete_recovery_bounds =
      flux::io::compile_flux_work_plan_routes(scoped_board, incomplete_recovery_bounds_plan);
  require(!incomplete_recovery_bounds.ok() &&
              std::any_of(incomplete_recovery_bounds.diagnostics.begin(),
                          incomplete_recovery_bounds.diagnostics.end(), [](const auto& diagnostic) {
                            return diagnostic.code ==
                                   "invalid_work_plan_route_option_dependency";
                          }),
          "hard-capacity WorkPlan recovery must require both bounded attempt and victim counts");

  auto invalid_search_pressure_plan = scoped_plan;
  invalid_search_pressure_plan.steps.front()
      .options["expansion_retry_heuristic_weight_per_mille"] = std::uint64_t{0U};
  const auto invalid_search_pressure =
      flux::io::compile_flux_work_plan_routes(scoped_board, invalid_search_pressure_plan);
  require(!invalid_search_pressure.ok() &&
              std::any_of(invalid_search_pressure.diagnostics.begin(),
                          invalid_search_pressure.diagnostics.end(), [](const auto& diagnostic) {
                            return diagnostic.code ==
                                   "invalid_work_plan_route_option_dependency";
                          }),
          "weighted-frontier WorkPlan search must fail closed without a retry weight");

  auto incomplete_neck_down_plan = scoped_plan;
  incomplete_neck_down_plan.steps.front().options.erase("max_terminal_neck_down_length_nm");
  const auto incomplete_neck_down =
      flux::io::compile_flux_work_plan_routes(scoped_board, incomplete_neck_down_plan);
  require(!incomplete_neck_down.ok() &&
              std::any_of(incomplete_neck_down.diagnostics.begin(),
                          incomplete_neck_down.diagnostics.end(), [](const auto& diagnostic) {
                            return diagnostic.code ==
                                   "invalid_work_plan_route_option_dependency";
                          }),
          "terminal neck-down WorkPlan geometry must require a paired physical length bound");

  auto orphan_sampled_repair_budget_plan = scoped_plan;
  orphan_sampled_repair_budget_plan.steps.front()
      .options["post_negotiation_sampled_conflict_repair"] = false;
  const auto orphan_sampled_repair_budget =
      flux::io::compile_flux_work_plan_routes(scoped_board, orphan_sampled_repair_budget_plan);
  require(!orphan_sampled_repair_budget.ok() &&
              std::any_of(orphan_sampled_repair_budget.diagnostics.begin(),
                          orphan_sampled_repair_budget.diagnostics.end(),
                          [](const auto& diagnostic) {
                            return diagnostic.code ==
                                   "invalid_work_plan_route_option_dependency";
                          }),
          "a WorkPlan sampled-conflict repair budget must not be accepted when the repair "
          "transaction is disabled");

  auto refill_plan = scoped_plan;
  refill_plan.steps.front().options["refillable_zone_fills"] = true;
  refill_plan.steps.front().options["differential_pair_skew_tuning"] = false;
  refill_plan.steps.front().options["differential_pair_skew_tuning_batch_assignment"] = false;
  const auto refill_compile =
      flux::io::compile_flux_work_plan_routes(scoped_board, refill_plan);
  require(refill_compile.ok() && refill_compile.route_steps.size() == 1U &&
              refill_compile.route_steps.front().refillable_zone_fills &&
              refill_compile.route_steps.front()
                  .configuration.planner.global.treat_saved_zone_fills_as_refillable &&
              refill_compile.route_steps.front()
                  .configuration.planner.regional.treat_saved_zone_fills_as_refillable &&
              refill_compile.route_steps.front()
                  .configuration.detailed.defer_saved_zone_fill_clearance_to_refill &&
              !refill_compile.route_steps.front()
                   .configuration.detailed.pull_tight_45_degrees &&
              !refill_compile.route_steps.front()
                   .configuration.detailed.continuous_refinement &&
              !refill_compile.route_steps.front()
                   .configuration.detailed.topology_locked_medial_materialization,
          "refill-required WorkPlan routing must compile the same conservative provisional "
          "configuration as the direct KiCad workflow");
  auto nonterminal_refill_plan = refill_plan;
  flux::io::WorkPlanStep post_refill_validation;
  post_refill_validation.id = "post-refill-validation";
  post_refill_validation.kind = flux::io::WorkPlanStepKind::validation;
  post_refill_validation.depends_on = {refill_plan.steps.front().id};
  nonterminal_refill_plan.steps.push_back(post_refill_validation);
  const auto nonterminal_refill_compile =
      flux::io::compile_flux_work_plan_routes(scoped_board, nonterminal_refill_plan);
  require(!nonterminal_refill_compile.ok() &&
              std::any_of(nonterminal_refill_compile.diagnostics.begin(),
                          nonterminal_refill_compile.diagnostics.end(),
                          [](const auto& diagnostic) {
                            return diagnostic.code ==
                                   "invalid_work_plan_route_option_dependency";
                          }),
          "a refill-required route must terminate the private plan because later steps would "
          "observe stale saved-fill geometry");

  auto diagnostic_preview_plan = scoped_plan;
  diagnostic_preview_plan.mode = flux::io::WorkPlanMode::preview;
  diagnostic_preview_plan.steps.front().options["retain_incomplete_diagnostic_preview"] = true;
  const auto diagnostic_preview_compile =
      flux::io::compile_flux_work_plan_routes(scoped_board, diagnostic_preview_plan);
  require(diagnostic_preview_compile.ok() &&
              diagnostic_preview_compile.route_steps.size() == 1U &&
              diagnostic_preview_compile.route_steps.front()
                  .retain_incomplete_diagnostic_preview,
          "the diagnostic incomplete-copper transaction must compile only as explicit preview "
          "policy");
  auto mutating_diagnostic_preview_plan = diagnostic_preview_plan;
  mutating_diagnostic_preview_plan.mode = flux::io::WorkPlanMode::new_routing;
  const auto mutating_diagnostic_preview_compile =
      flux::io::compile_flux_work_plan_routes(scoped_board, mutating_diagnostic_preview_plan);
  require(!mutating_diagnostic_preview_compile.ok() &&
              std::any_of(mutating_diagnostic_preview_compile.diagnostics.begin(),
                          mutating_diagnostic_preview_compile.diagnostics.end(),
                          [](const auto& diagnostic) {
                            return diagnostic.code ==
                                   "invalid_work_plan_route_option_dependency";
                          }),
          "an incomplete diagnostic candidate must never compile in a mutating WorkPlan mode");
  auto refill_diagnostic_preview_plan = diagnostic_preview_plan;
  refill_diagnostic_preview_plan.steps.front().options["refillable_zone_fills"] = true;
  refill_diagnostic_preview_plan.steps.front().options["differential_pair_skew_tuning"] = false;
  refill_diagnostic_preview_plan.steps.front()
      .options["differential_pair_skew_tuning_batch_assignment"] = false;
  const auto refill_diagnostic_preview_compile =
      flux::io::compile_flux_work_plan_routes(scoped_board, refill_diagnostic_preview_plan);
  require(!refill_diagnostic_preview_compile.ok(),
          "incomplete diagnostic copper must not enter a zone-refill transaction");

  auto wildcard_plan = scoped_plan;
  wildcard_plan.steps.front().selectors.net_classes.clear();
  wildcard_plan.steps.front().selectors.placement_groups.clear();
  const auto wildcard_expansion = flux::io::expand_flux_work_plan(scoped_board, wildcard_plan);
  require(!wildcard_expansion.ok() &&
              std::any_of(wildcard_expansion.diagnostics.begin(),
                          wildcard_expansion.diagnostics.end(), [](const auto& diagnostic) {
                            return diagnostic.code == "wildcard_only_force_reroute_scope";
                          }),
          "glob selection must never infer authority to remove pre-existing copper");

  auto allowlisted_plan = scoped_plan;
  allowlisted_plan.mode = flux::io::WorkPlanMode::allowlisted_ripup;
  allowlisted_plan.ripup_allowlist = {"AUX"};
  const auto allowlisted = flux::io::expand_flux_work_plan(scoped_board, allowlisted_plan);
  require(allowlisted.ok() &&
              allowlisted.copper_authority.reroutable_nets == std::vector<NetId>{auxiliary} &&
              allowlisted.copper_authority.removable_tracks ==
                  std::vector<ObjectId>{ObjectId{"track-x"}} &&
              allowlisted.copper_authority.immutable_vias ==
                  std::vector<ObjectId>{ObjectId{"via-p"}, ObjectId{"via-x-locked"}},
          "an exact allowlist may expose only unlocked copper on named nets while locked copper "
          "remains immutable");
  const auto compiled_allowlisted =
      flux::io::compile_flux_work_plan_routes(scoped_board, allowlisted_plan);
  require(compiled_allowlisted.ok() && compiled_allowlisted.route_steps.size() == 1U &&
              compiled_allowlisted.route_steps.front().selected_nets ==
                  std::vector<NetId>{signal_a, power, auxiliary} &&
              compiled_allowlisted.route_steps.front()
                      .configuration.detailed.existing_routing_replacement_scope ==
                  std::optional<std::vector<NetId>>{std::vector<NetId>{auxiliary}},
          "allowlisted blockers must join the planned route set while only their exact unlocked "
          "copper enters replacement authority");

  auto unsupported_option_plan = scoped_plan;
  unsupported_option_plan.steps.front().options["not_a_router_option"] = true;
  const auto unsupported =
      flux::io::compile_flux_work_plan_routes(scoped_board, unsupported_option_plan);
  require(!unsupported.ok() &&
              std::any_of(unsupported.diagnostics.begin(), unsupported.diagnostics.end(),
                          [](const auto& diagnostic) {
                            return diagnostic.code == "unsupported_work_plan_route_option";
                          }),
          "unknown route options must fail compilation instead of being ignored");

  auto coupled_board = two_pad_board();
  const NetId pair_positive{"net-1"};
  const NetId pair_negative{"net-2"};
  coupled_board.nets = {{pair_positive, "PAIR_P", 0}, {pair_negative, "PAIR_N", 0}};
  coupled_board.footprints[0].position = mm(20.0, 38.0);
  coupled_board.footprints[1].position = mm(80.0, 38.0);
  coupled_board.footprints[0].courtyard = local_rectangle(0.1, 0.1);
  coupled_board.footprints[1].courtyard = local_rectangle(0.1, 0.1);
  auto negative_start = coupled_board.footprints[0];
  negative_start.id = ObjectId{"U1N-work-plan"};
  negative_start.reference = "U1N-work-plan";
  negative_start.position = mm(20.0, 42.0);
  negative_start.pads.front().id = ObjectId{"U1N-work-plan-pad"};
  negative_start.pads.front().net = pair_negative;
  auto negative_end = coupled_board.footprints[1];
  negative_end.id = ObjectId{"U2N-work-plan"};
  negative_end.reference = "U2N-work-plan";
  negative_end.position = mm(80.0, 42.0);
  negative_end.pads.front().id = ObjectId{"U2N-work-plan-pad"};
  negative_end.pads.front().net = pair_negative;
  coupled_board.footprints.push_back(std::move(negative_start));
  coupled_board.footprints.push_back(std::move(negative_end));
  coupled_board.tracks = {
      {ObjectId{"pair-p-track"}, pair_positive, LayerId{"F.Cu"}, mm(20.0, 38.0),
       mm(80.0, 38.0), Nanometres::from_mm(0.25), false},
      {ObjectId{"pair-n-track"}, pair_negative, LayerId{"F.Cu"}, mm(20.0, 42.0),
       mm(80.0, 42.0), Nanometres::from_mm(0.25), false}};
  coupled_board.differential_pairs = {
      {flux::DifferentialPairId{"pair-1"}, "PAIR", pair_positive, pair_negative,
       Nanometres::from_mm(0.2), std::nullopt}};

  auto pair_plan = flux::io::make_flux_work_plan_template(coupled_board, "pair.kicad_pcb");
  pair_plan.mode = flux::io::WorkPlanMode::force_reroute_selected;
  flux::io::WorkPlanStep pair_step;
  pair_step.id = "route-pair";
  pair_step.kind = flux::io::WorkPlanStepKind::differential_pair;
  pair_step.selectors.exact_nets = {"PAIR_P", "PAIR_N"};
  pair_step.options = {{"checkpoint_objective", std::string{"aggregate-overflow"}},
                       {"crossing_safe_diagonal_grid_edges", false},
                       {"direct_medial_edges", false},
                       {"diversify_pin_access_rings", true},
                       {"grid_pitch_nm", std::uint64_t{5'000'000U}},
                       {"interleave_routability_feedback", false},
                       {"max_pin_access_nodes_per_terminal", std::uint64_t{32U}},
                       {"max_router_iterations", std::uint64_t{2U}},
                       {"negotiate_voronoi_lane_conflicts", false},
                       {"physical_envelope_ordering", false},
                       {"physical_overflow_history", false},
                       {"pin_escape_grid_radius", std::uint64_t{2U}},
                       {"routability_feedback_retries", std::uint64_t{0U}},
                       {"strict_medial_topology", false},
                       {"topology_locked_medial_materialization", false}};
  pair_plan.steps = {pair_step};
  const auto pair_compile = flux::io::compile_flux_work_plan_routes(coupled_board, pair_plan);
  require(pair_compile.ok() && pair_compile.route_steps.size() == 1U &&
              pair_compile.route_steps.front().kind ==
                  flux::io::WorkPlanStepKind::differential_pair &&
              pair_compile.route_steps.front().selected_nets ==
                  std::vector<NetId>{pair_positive, pair_negative} &&
              pair_compile.expansion.copper_authority.reroutable_nets ==
                  std::vector<NetId>{pair_positive, pair_negative} &&
              pair_compile.expansion.copper_authority.removable_tracks ==
                  std::vector<ObjectId>{ObjectId{"pair-n-track"}, ObjectId{"pair-p-track"}},
          "a differential-pair step must compile as one atomic route transaction with exact "
          "selected-copper authority");

  auto connected_pair_plan = pair_plan;
  connected_pair_plan.mode = flux::io::WorkPlanMode::new_routing;
  const auto connected_pair_execution =
      flux::io::execute_flux_work_plan(coupled_board, connected_pair_plan);
  require(connected_pair_execution.ok() && connected_pair_execution.whole_board_accepted &&
              connected_pair_execution.steps.size() == 1U &&
              connected_pair_execution.steps.front().kind ==
                  flux::io::WorkPlanStepKind::differential_pair &&
              connected_pair_execution.steps.front().status ==
                  flux::io::WorkPlanStepExecutionStatus::completed,
          "an already-connected declared pair must execute through the coupled adapter without "
          "mutating authoritative copper");
  auto open_pair_board = coupled_board;
  open_pair_board.tracks.clear();
  const auto open_pair_execution =
      flux::io::execute_flux_work_plan(open_pair_board, connected_pair_plan);
  require(open_pair_execution.ok() && open_pair_execution.whole_board_accepted &&
              open_pair_execution.candidate && open_pair_execution.steps.size() == 1U &&
              open_pair_execution.steps.front().status ==
                  flux::io::WorkPlanStepExecutionStatus::completed &&
              open_pair_execution.steps.front().global_unrouted_nets == 0U &&
              !open_pair_execution.candidate->tracks.empty(),
          "an open declared pair must complete global, regional, detailed, connectivity, and DRC "
          "acceptance as one coupled WorkPlan transaction: " +
              flux::io::serialize_flux_work_plan_execution_summary(open_pair_execution));

  auto invalid_pair_board = coupled_board;
  invalid_pair_board.nets.push_back({NetId{"ordinary"}, "ORDINARY", 0});
  auto invalid_pair_plan =
      flux::io::make_flux_work_plan_template(invalid_pair_board, "pair.kicad_pcb");
  flux::io::WorkPlanStep invalid_pair_step;
  invalid_pair_step.id = "invalid-pair";
  invalid_pair_step.kind = flux::io::WorkPlanStepKind::differential_pair;
  invalid_pair_step.selectors.exact_nets = {"ORDINARY"};
  invalid_pair_plan.steps = {invalid_pair_step};
  const auto invalid_pair_compile =
      flux::io::compile_flux_work_plan_routes(invalid_pair_board, invalid_pair_plan);
  require(!invalid_pair_compile.ok() &&
              std::any_of(invalid_pair_compile.diagnostics.begin(),
                          invalid_pair_compile.diagnostics.end(), [](const auto& diagnostic) {
                            return diagnostic.code ==
                                   "invalid_work_plan_differential_pair_scope";
                          }),
          "a differential-pair step must reject ordinary nets instead of degrading to a generic "
          "route");

  auto bus_board = coupled_board;
  bus_board.differential_pairs.clear();
  bus_board.routing_buses = {
      {flux::RoutingBusId{"bus-1"}, "PAIR_BUS", {pair_positive, pair_negative},
       Nanometres::from_mm(0.5)}};
  auto bus_plan = flux::io::make_flux_work_plan_template(bus_board, "bus.kicad_pcb");
  bus_plan.mode = flux::io::WorkPlanMode::force_reroute_selected;
  flux::io::WorkPlanStep bus_step;
  bus_step.id = "route-bus";
  bus_step.kind = flux::io::WorkPlanStepKind::bus;
  bus_step.selectors.exact_nets = {"PAIR_P", "PAIR_N"};
  bus_step.options = pair_step.options;
  bus_plan.steps = {bus_step};
  const auto bus_compile = flux::io::compile_flux_work_plan_routes(bus_board, bus_plan);
  require(bus_compile.ok() && bus_compile.route_steps.size() == 1U &&
              bus_compile.route_steps.front().kind == flux::io::WorkPlanStepKind::bus &&
              bus_compile.expansion.copper_authority.reroutable_nets ==
                  std::vector<NetId>{pair_positive, pair_negative},
          "an ordered-bus step must compile as one atomic route transaction with exact "
          "selected-copper authority");
  auto connected_bus_plan = bus_plan;
  connected_bus_plan.mode = flux::io::WorkPlanMode::new_routing;
  const auto connected_bus_execution =
      flux::io::execute_flux_work_plan(bus_board, connected_bus_plan);
  require(connected_bus_execution.ok() && connected_bus_execution.whole_board_accepted &&
              connected_bus_execution.steps.size() == 1U &&
              connected_bus_execution.steps.front().kind == flux::io::WorkPlanStepKind::bus &&
              connected_bus_execution.steps.front().status ==
                  flux::io::WorkPlanStepExecutionStatus::completed,
          "an already-connected declared bus must execute through the coupled adapter without "
          "mutating authoritative copper");
  auto open_bus_board = bus_board;
  open_bus_board.tracks.clear();
  const auto open_bus_execution =
      flux::io::execute_flux_work_plan(open_bus_board, connected_bus_plan);
  require(open_bus_execution.ok() && open_bus_execution.whole_board_accepted &&
              open_bus_execution.candidate && open_bus_execution.steps.size() == 1U &&
              open_bus_execution.steps.front().status ==
                  flux::io::WorkPlanStepExecutionStatus::completed &&
              open_bus_execution.steps.front().global_unrouted_nets == 0U &&
              !open_bus_execution.candidate->tracks.empty(),
          "an open declared bus must complete global, regional, detailed, connectivity, and DRC "
          "acceptance as one coupled WorkPlan transaction: " +
              flux::io::serialize_flux_work_plan_execution_summary(open_bus_execution));

  auto connected = two_pad_board();
  connected.tracks = {{ObjectId{"source-track"}, NetId{"net-1"}, LayerId{"F.Cu"},
                       mm(20.0, 40.0), mm(80.0, 40.0), Nanometres::from_mm(0.25), false}};
  auto executable_plan = flux::io::make_flux_work_plan_template(connected, "board.kicad_pcb");
  executable_plan.mode = flux::io::WorkPlanMode::force_reroute_selected;
  flux::io::WorkPlanStep executable_route;
  executable_route.id = "reroute";
  executable_route.kind = flux::io::WorkPlanStepKind::route;
  executable_route.selectors.exact_nets = {"SIGNAL"};
  executable_route.options = {{"grid_pitch_nm", std::uint64_t{1'000'000U}},
                              {"max_expansions_per_connection", std::uint64_t{100'000U}},
                              {"max_router_iterations", std::uint64_t{4U}}};
  flux::io::WorkPlanStep executable_validation;
  executable_validation.id = "validate";
  executable_validation.kind = flux::io::WorkPlanStepKind::validation;
  executable_validation.depends_on = {"reroute"};
  executable_plan.steps = {executable_route, executable_validation};
  CollectingTelemetrySink work_plan_telemetry;
  const auto execution =
      flux::io::execute_flux_work_plan(connected, executable_plan, {}, &work_plan_telemetry);
  require(execution.ok() && execution.whole_board_accepted && execution.candidate_ready_for_export &&
              execution.candidate && execution.steps.size() == 2U &&
              execution.steps.front().status ==
                  flux::io::WorkPlanStepExecutionStatus::completed &&
              execution.steps.front().replacement_nets == std::vector<NetId>{NetId{"net-1"}} &&
              std::none_of(execution.candidate->tracks.begin(), execution.candidate->tracks.end(),
                           [](const auto& track) {
                             return track.id == ObjectId{"source-track"};
                           }) &&
              connected.tracks.size() == 1U && connected.tracks.front().id == ObjectId{"source-track"} &&
              flux::io::serialize_flux_work_plan_execution_summary(execution) ==
                  flux::io::serialize_flux_work_plan_execution_summary(execution) &&
              std::any_of(work_plan_telemetry.events.begin(), work_plan_telemetry.events.end(),
                          [](const auto& event) {
                            return event.kind == flux::RouterTelemetryEventKind::search_summary &&
                                   event.stable_id == "work-plan-step:reroute" &&
                                   event.detail == "Work plan step completed: reroute";
                          }),
          "work-plan execution must reroute on a copy, replace only exact unlocked source copper, "
          "and commit a deterministic exact-clean checkpoint through validation");

  auto unrelated_open = connected;
  const NetId unrelated_net{"unrelated-open"};
  unrelated_open.nets.push_back({unrelated_net, "UNRELATED_OPEN", 0});
  auto unrelated_start = unrelated_open.footprints.front();
  unrelated_start.id = ObjectId{"U3-unrelated"};
  unrelated_start.reference = "U3-unrelated";
  unrelated_start.position = mm(20.0, 20.0);
  unrelated_start.pads.front().id = ObjectId{"U3-unrelated-pad"};
  unrelated_start.pads.front().net = unrelated_net;
  auto unrelated_end = unrelated_start;
  unrelated_end.id = ObjectId{"U4-unrelated"};
  unrelated_end.reference = "U4-unrelated";
  unrelated_end.position = mm(80.0, 20.0);
  unrelated_end.pads.front().id = ObjectId{"U4-unrelated-pad"};
  unrelated_open.footprints.push_back(std::move(unrelated_start));
  unrelated_open.footprints.push_back(std::move(unrelated_end));
  auto route_only_plan = executable_plan;
  route_only_plan.steps.pop_back();
  const auto unrelated_open_execution =
      flux::io::execute_flux_work_plan(unrelated_open, route_only_plan);
  require(unrelated_open_execution.ok() && !unrelated_open_execution.whole_board_accepted &&
              !unrelated_open_execution.candidate_ready_for_export &&
              unrelated_open_execution.final_acceptance.open_connections > 0U,
          "selected-net step success must not authorize publication while an unrelated whole-board "
          "connection remains open");

  auto preview_plan = executable_plan;
  preview_plan.mode = flux::io::WorkPlanMode::preview;
  const auto preview = flux::io::execute_flux_work_plan(connected, preview_plan);
  require(preview.ok() && preview.preview && !preview.candidate_ready_for_export &&
              preview.steps.front().selected_nets.empty() && preview.candidate &&
              preview.candidate->tracks == connected.tracks &&
              preview.candidate->footprints == connected.footprints,
          "preview mode must observe an already complete selected net without replacing or "
          "export-authorizing its source copper");

  auto placement_source = two_pad_board();
  Footprint placement_bystander;
  placement_bystander.id = ObjectId{"BYSTANDER"};
  placement_bystander.reference = "BYSTANDER";
  placement_bystander.position = mm(50.0, 15.0);
  placement_bystander.courtyard = local_rectangle(2.0, 2.0);
  placement_source.footprints.push_back(placement_bystander);
  auto placement_plan =
      flux::io::make_flux_work_plan_template(placement_source, "placement.kicad_pcb");
  placement_plan.mode = flux::io::WorkPlanMode::new_routing;
  flux::io::WorkPlanStep placement_step;
  placement_step.id = "place-and-route";
  placement_step.kind = flux::io::WorkPlanStepKind::placement;
  placement_step.selectors.footprints = {"U1", "U2"};
  placement_step.selectors.group_scope = flux::io::WorkPlanSelectorScope::touching;
  placement_step.options = {
      {"checkpoint_objective", std::string{"aggregate-overflow"}},
      {"crossing_safe_diagonal_grid_edges", false},
      {"direct_medial_edges", false},
      {"enable_incremental_routing", false},
      {"enable_routing_guided_moves", false},
      {"grid_pitch_nm", std::uint64_t{5'000'000U}},
      {"max_candidate_evaluations", std::uint64_t{2U}},
      {"max_candidate_evaluation_workers", std::uint64_t{1U}},
      {"max_expansions_per_connection", std::uint64_t{100'000U}},
      {"max_full_routing_candidates_per_wave", std::uint64_t{1U}},
      {"max_placement_passes", std::uint64_t{1U}},
      {"max_router_iterations", std::uint64_t{2U}},
      {"negotiate_voronoi_lane_conflicts", false},
      {"physical_envelope_ordering", false},
      {"physical_overflow_history", false},
      {"strict_medial_topology", false},
      {"topology_locked_medial_materialization", false}};
  placement_plan.steps = {placement_step};
  const auto placement_compile =
      flux::io::compile_flux_work_plan_routes(placement_source, placement_plan);
  require(placement_compile.ok() && placement_compile.route_steps.size() == 1U &&
              placement_compile.route_steps.front().placement &&
              placement_compile.route_steps.front().selected_footprints ==
                  std::vector<ObjectId>{ObjectId{"U1"}, ObjectId{"U2"}} &&
              placement_compile.route_steps.front().selected_nets ==
                  std::vector<NetId>{NetId{"net-1"}} &&
              placement_compile.route_steps.front().placement->max_passes == 1U &&
              placement_compile.route_steps.front()
                      .placement->max_candidate_evaluations == 2U &&
              placement_compile.route_steps.front()
                      .configuration.detailed.existing_routing_replacement_scope ==
                  std::optional<std::vector<NetId>>{std::vector<NetId>{}},
          "a placement step must compile selected footprints and all touching nets through the "
          "shared router plus bounded integrated-optimizer configuration");
  CollectingTelemetrySink placement_telemetry;
  const auto placement_execution = flux::io::execute_flux_work_plan(
      placement_source, placement_plan, {}, &placement_telemetry);
  const bool bystander_preserved = placement_execution.candidate &&
      std::any_of(placement_execution.candidate->footprints.begin(),
                  placement_execution.candidate->footprints.end(),
                  [&placement_bystander](const auto& footprint) {
                    return footprint.id == ObjectId{"BYSTANDER"} &&
                           footprint.position == placement_bystander.position &&
                           !footprint.locked;
                  });
  require(placement_execution.ok() && placement_execution.whole_board_accepted &&
              placement_execution.candidate_ready_for_export && placement_execution.candidate &&
              placement_execution.steps.size() == 1U &&
              placement_execution.steps.front().status ==
                  flux::io::WorkPlanStepExecutionStatus::completed &&
              placement_execution.steps.front().placement_attempted &&
              placement_execution.steps.front().placement_candidate_evaluations > 0U &&
              placement_execution.steps.front().placement_full_route_evaluations > 0U &&
              bystander_preserved &&
              placement_source.tracks.empty() && !placement_execution.candidate->tracks.empty() &&
              std::any_of(placement_telemetry.events.begin(), placement_telemetry.events.end(),
                          [](const auto& event) {
                            return event.stable_id == "work-plan-step:place-and-route" &&
                                   event.total_groups == 2U && event.search_expansions > 0U;
                          }),
          "placement execution must route on a private board, preserve unselected transforms and "
          "locks, and commit only an exact accepted integrated checkpoint: " +
              flux::io::serialize_flux_work_plan_execution_summary(placement_execution));

  auto reviewed_placement_plan = placement_plan;
  reviewed_placement_plan.policies = {{"placement-move-priorities",
                                       "reviewed-priorities.policy",
                                       std::string(64U, '0'),
                                       placement_source.source_revision}};
  flux::PlacementMovePriorityPolicy reviewed_priorities;
  reviewed_priorities.source_revision = placement_source.source_revision;
  reviewed_priorities.order = flux::default_placement_move_priority_order();
  for (const auto& footprint : placement_source.footprints) {
    reviewed_priorities.footprints.push_back(
        {footprint.id, flux::PlacementMovePriorityClass::other});
  }
  flux::io::WorkPlanLoadResult loaded_placement;
  loaded_placement.plan = reviewed_placement_plan;
  loaded_placement.policy_board = placement_source;
  loaded_placement.placement_move_priorities = reviewed_priorities;
  loaded_placement.applied_policy_kinds = {"placement-move-priorities"};
  const auto reviewed_placement_execution =
      flux::io::execute_loaded_flux_work_plan(loaded_placement);
  require(reviewed_placement_execution.ok() && reviewed_placement_execution.steps.size() == 1U &&
              reviewed_placement_execution.steps.front()
                  .placement_move_priorities_configured,
          "the one-read reviewed move-priority policy must reach the integrated placement "
          "optimizer rather than being discarded after identity validation");

  auto unsafe_placement_plan = placement_plan;
  unsafe_placement_plan.steps.front().options["allow_tentative_placement_envelopes"] = true;
  require(!flux::io::compile_flux_work_plan_routes(placement_source, unsafe_placement_plan).ok(),
          "tentative placement envelopes must remain preview-only in a saved WorkPlan");
  auto ignored_placement_option_plan = placement_plan;
  ignored_placement_option_plan.steps.front().options["refillable_zone_fills"] = true;
  require(!flux::io::compile_flux_work_plan_routes(placement_source,
                                                    ignored_placement_option_plan)
               .ok(),
          "a placement step must reject route-transaction wrappers that its optimizer would not "
          "execute");
  auto unprojected_placement_plan = placement_plan;
  unprojected_placement_plan.steps.front().selectors.group_scope =
      flux::io::WorkPlanSelectorScope::selected;
  require(!flux::io::compile_flux_work_plan_routes(placement_source,
                                                    unprojected_placement_plan)
               .ok(),
          "placement must project every selected footprint to touching nets before execution");
  auto route_with_placement_option = executable_plan;
  route_with_placement_option.steps.front().options["max_placement_passes"] =
      std::uint64_t{2U};
  require(!flux::io::compile_flux_work_plan_routes(connected, route_with_placement_option).ok(),
          "placement-only configuration must not be silently accepted by an ordinary route step");

  auto refill_source = two_pad_board();
  refill_source.layers.resize(1U);
  refill_source.nets.push_back(Net{NetId{"work-plan-plane"}, "WORK_PLAN_PLANE", 0});
  flux::CopperZone refill_zone;
  refill_zone.id = ObjectId{"work-plan-saved-fill"};
  refill_zone.net = NetId{"work-plan-plane"};
  refill_zone.fills = {
      flux::CopperZoneFill{LayerId{"F.Cu"}, rectangle(45.0, 0.0, 55.0, 80.0), {}}};
  refill_source.zones.push_back(std::move(refill_zone));
  auto refill_execution_plan =
      flux::io::make_flux_work_plan_template(refill_source, "refill.kicad_pcb");
  refill_execution_plan.mode = flux::io::WorkPlanMode::new_routing;
  flux::io::WorkPlanStep refill_route;
  refill_route.id = "route-before-refill";
  refill_route.kind = flux::io::WorkPlanStepKind::route;
  refill_route.selectors.exact_nets = {"SIGNAL"};
  refill_route.options = {{"checkpoint_objective", std::string{"aggregate-overflow"}},
                          {"crossing_safe_diagonal_grid_edges", false},
                          {"direct_medial_edges", false},
                          {"grid_pitch_nm", std::uint64_t{5'000'000U}},
                          {"interleave_routability_feedback", false},
                          {"max_router_iterations", std::uint64_t{2U}},
                          {"negotiate_voronoi_lane_conflicts", false},
                          {"physical_envelope_ordering", false},
                          {"physical_overflow_history", false},
                          {"refillable_zone_fills", true},
                          {"routability_feedback_retries", std::uint64_t{0U}},
                          {"strict_medial_topology", false}};
  refill_execution_plan.steps = {refill_route};
  const auto provisional_refill =
      flux::io::execute_flux_work_plan(refill_source, refill_execution_plan);
  require(provisional_refill.ok() && !provisional_refill.whole_board_accepted &&
              provisional_refill.zone_refill_required &&
              provisional_refill.candidate_ready_for_zone_refill &&
              !provisional_refill.candidate_ready_for_export &&
              provisional_refill.candidate && provisional_refill.steps.size() == 1U &&
              provisional_refill.steps.front().status ==
                  flux::io::WorkPlanStepExecutionStatus::completed &&
              provisional_refill.steps.front().provisional_zone_refill_required,
          "a refill-required WorkPlan must retain the exact provisional candidate privately and "
          "authorize only the external refill transaction: " +
              flux::io::serialize_flux_work_plan_execution_summary(provisional_refill));

  Board partial_preview_source;
  partial_preview_source.source_revision = "partial-preview-fixture";
  partial_preview_source.outline = rectangle(0.0, 0.0, 100.0, 80.0);
  partial_preview_source.layers = {
      {LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0U}};
  const NetId preview_easy{"preview-easy"};
  const NetId preview_blocked{"preview-blocked"};
  partial_preview_source.nets = {{preview_easy, "PREVIEW_EASY", 0},
                                 {preview_blocked, "PREVIEW_BLOCKED", 0}};
  const auto preview_terminal = [](std::string id, NetId net, Point position) {
    Footprint footprint;
    footprint.id = ObjectId{id};
    footprint.reference = id;
    footprint.position = position;
    footprint.courtyard = local_rectangle(2.0, 2.0);
    footprint.pads = {
        {ObjectId{id + "-pad"}, net, Point{}, {LayerId{"F.Cu"}}}};
    return footprint;
  };
  partial_preview_source.footprints = {
      preview_terminal("E1", preview_easy, mm(10.0, 20.0)),
      preview_terminal("E2", preview_easy, mm(30.0, 20.0)),
      preview_terminal("B1", preview_blocked, mm(10.0, 60.0)),
      preview_terminal("B2", preview_blocked, mm(90.0, 60.0))};
  partial_preview_source.keepouts = {
      {ObjectId{"preview-wall"}, rectangle(45.0, 0.0, 55.0, 80.0),
       {LayerId{"F.Cu"}}, flux::KeepoutRules{true, true, false, false, false},
       "preview wall"}};
  auto incomplete_preview_plan = flux::io::make_flux_work_plan_template(
      partial_preview_source, "partial-preview.kicad_pcb");
  incomplete_preview_plan.mode = flux::io::WorkPlanMode::preview;
  flux::io::WorkPlanStep incomplete_preview_route;
  incomplete_preview_route.id = "incomplete-preview";
  incomplete_preview_route.kind = flux::io::WorkPlanStepKind::route;
  incomplete_preview_route.selectors.exact_nets = {"PREVIEW_EASY", "PREVIEW_BLOCKED"};
  incomplete_preview_route.options = {
      {"checkpoint_objective", std::string{"aggregate-overflow"}},
      {"crossing_safe_diagonal_grid_edges", false},
      {"direct_medial_edges", false},
      {"grid_pitch_nm", std::uint64_t{5'000'000U}},
      {"interleave_routability_feedback", false},
      {"max_expansions_per_connection", std::uint64_t{100'000U}},
      {"max_router_iterations", std::uint64_t{2U}},
      {"negotiate_voronoi_lane_conflicts", false},
      {"physical_envelope_ordering", false},
      {"physical_overflow_history", false},
      {"retain_incomplete_diagnostic_preview", true},
      {"routability_feedback_retries", std::uint64_t{0U}},
      {"strict_medial_topology", false},
      {"topology_locked_medial_materialization", false}};
  incomplete_preview_plan.steps = {incomplete_preview_route};
  CollectingTelemetrySink incomplete_preview_telemetry;
  const auto incomplete_preview_execution = flux::io::execute_flux_work_plan(
      partial_preview_source, incomplete_preview_plan, {}, &incomplete_preview_telemetry);
  require(!incomplete_preview_execution.ok() && incomplete_preview_execution.preview &&
              incomplete_preview_execution.candidate_is_incomplete_diagnostic_preview &&
              !incomplete_preview_execution.candidate_ready_for_export &&
              !incomplete_preview_execution.candidate_ready_for_zone_refill &&
              incomplete_preview_execution.candidate &&
              !incomplete_preview_execution.candidate->tracks.empty() &&
              partial_preview_source.tracks.empty() &&
              incomplete_preview_execution.steps.size() == 1U &&
              incomplete_preview_execution.steps.front().status ==
                  flux::io::WorkPlanStepExecutionStatus::failed &&
              incomplete_preview_execution.steps.front()
                  .incomplete_diagnostic_preview_attempted &&
              incomplete_preview_execution.steps.front()
                  .incomplete_diagnostic_preview_available &&
              incomplete_preview_execution.steps.front().incomplete_diagnostic_preview_paths >
                  0U &&
              std::any_of(incomplete_preview_telemetry.events.begin(),
                          incomplete_preview_telemetry.events.end(), [](const auto& event) {
                            return event.stable_id ==
                                   "work-plan-diagnostic-preview:incomplete-preview";
                          }),
          "an incomplete preview must retain partial copper and telemetry without changing failure "
          "or publication authority: " +
              flux::io::serialize_flux_work_plan_execution_summary(
                  incomplete_preview_execution));

  std::stop_source cancelled_source;
  cancelled_source.request_stop();
  const auto cancelled = flux::io::execute_flux_work_plan(
      connected, executable_plan, cancelled_source.get_token());
  require(!cancelled.ok() && !cancelled.steps.empty() &&
              cancelled.steps.front().status ==
                  flux::io::WorkPlanStepExecutionStatus::cancelled &&
              cancelled.candidate && cancelled.candidate->tracks == connected.tracks &&
              cancelled.candidate->footprints == connected.footprints,
          "a pre-requested stop token must cancel before routing and preserve the source checkpoint");
}

} // namespace

int main(int argc, char** argv) {
  const std::vector<TestCase> tests{
      {"units_remain_exact_and_saturating", units_remain_exact_and_saturating},
      {"source_bound_board_rules_control_geometry_and_hole_clearance",
       source_bound_board_rules_control_geometry_and_hole_clearance},
      {"dimensioned_pathfinder_pressure_preserves_units_and_group_fairness",
       dimensioned_pathfinder_pressure_preserves_units_and_group_fairness},
      {"physical_checkpoint_ordering_is_feasibility_first_and_fail_closed",
       physical_checkpoint_ordering_is_feasibility_first_and_fail_closed},
      {"routing_xpbd_joint_relaxation_preserves_endpoints_and_reports_stress",
       routing_xpbd_joint_relaxation_preserves_endpoints_and_reports_stress},
      {"global_to_detailed_route_is_transactional_and_capacity_gated",
       global_to_detailed_route_is_transactional_and_capacity_gated},
      {"constrained_ph_rebound_avoids_inter_sample_obstacle",
       constrained_ph_rebound_avoids_inter_sample_obstacle},
      {"quintic_ph_geometry_preserves_invariants_and_exact_offsets",
       quintic_ph_geometry_preserves_invariants_and_exact_offsets},
      {"obstacle_aware_diagonals_and_curves_remain_capacity_and_drc_gated",
       obstacle_aware_diagonals_and_curves_remain_capacity_and_drc_gated},
      {"hydraulic_polygon_discovery_projects_into_coarse_navmesh",
       hydraulic_polygon_discovery_projects_into_coarse_navmesh},
      {"hydraulic_gpu_fuses_clearance_and_batches_independent_requests",
       hydraulic_gpu_fuses_clearance_and_batches_independent_requests},
      {"adaptive_navmesh_recovers_clearance_clipped_passage",
       adaptive_navmesh_recovers_clearance_clipped_passage},
      {"advanced_routing_compute_shortlists_preserve_cpu_authority",
       advanced_routing_compute_shortlists_preserve_cpu_authority},
      {"physical_copper_t_junction_is_preserved_during_ph_realization",
       physical_copper_t_junction_is_preserved_during_ph_realization},
      {"same_net_vertical_features_are_reused_as_transition_portals",
       same_net_vertical_features_are_reused_as_transition_portals},
      {"kicad_zone_refill_adapter_is_bounded_and_fail_closed",
       kicad_zone_refill_adapter_is_bounded_and_fail_closed},
      {"work_plan_round_trip_is_strict_source_bound_and_non_authoritative",
       work_plan_round_trip_is_strict_source_bound_and_non_authoritative},
      {"integrated_placement_rejects_proxy_improvement_that_violates_copper_rules",
       integrated_placement_rejects_proxy_improvement_that_violates_copper_rules},
      {"cancellation_and_kicad_export_boundaries_fail_closed",
       cancellation_and_kicad_export_boundaries_fail_closed},
  };

  std::size_t passed = 0U;
  std::size_t selected = 0U;
  for (const auto& test : tests) {
    if (argc > 1 && test.name != argv[1]) {
      continue;
    }
    ++selected;
    try {
      test.run();
      ++passed;
      std::cout << "PASS " << test.name << '\n';
    } catch (const std::exception& error) {
      std::cerr << "FAIL " << test.name << ": " << error.what() << '\n';
    }
  }
  std::cout << passed << '/' << selected << " tests passed\n";
  return selected > 0U && passed == selected ? 0 : 1;
}
