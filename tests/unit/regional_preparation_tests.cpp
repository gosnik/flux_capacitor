#include "flux/core/geometry.hpp"
#include "flux/core/global_router.hpp"
#include "flux/core/regional_router.hpp"
#include "../../src/regional_source_index.hpp"

#include <cstdlib>
#include <iostream>
#include <stop_token>

namespace {
using namespace flux;

[[noreturn]] void fail(const char* message) { std::cerr << message << '\n'; std::exit(1); }
void require(bool condition, const char* message) { if (!condition) fail(message); }

Board fixture_board() {
  const LayerId front{"F.Cu"};
  const NetId net{"net-1"};
  Board board;
  board.source_revision = "regional-preparation-fixture";
  const auto rectangle = [](double min_x, double min_y, double max_x, double max_y) {
    return Polygon{{{Nanometres::from_mm(min_x), Nanometres::from_mm(min_y)},
                    {Nanometres::from_mm(max_x), Nanometres::from_mm(min_y)},
                    {Nanometres::from_mm(max_x), Nanometres::from_mm(max_y)},
                    {Nanometres::from_mm(min_x), Nanometres::from_mm(max_y)}}};
  };
  board.outline = rectangle(0, 0, 100, 80);
  board.layers = {{front, "F.Cu", LayerKind::copper, 0}};
  board.nets = {{net, "SIGNAL", 0}};
  const auto footprint = [&](std::string id, Point position) {
    Footprint fp;
    fp.id = ObjectId{id}; fp.reference = id; fp.position = position;
    fp.courtyard = rectangle(-3, -3, 3, 3);
    fp.pads = {{ObjectId{id + "-pad"}, net, Point{}, {front}}};
    return fp;
  };
  board.footprints = {footprint("U1", {Nanometres::from_mm(20), Nanometres::from_mm(40)}),
                      footprint("U2", {Nanometres::from_mm(80), Nanometres::from_mm(40)})};
  return board;
}

void preparation_reuse_and_invalidation_are_semantically_safe() {
  const Board board = fixture_board();
  GlobalRouterConfig global_config;
  global_config.max_iterations = 2U;
  global_config.grid_pitch = Nanometres::from_mm(5.0);
  global_config.max_pin_escape_grid_radius = 2U;
  global_config.max_pin_access_nodes_per_terminal = 32U;
  const auto global = route_global(board, global_config);
  require(global.complete() && !global.paths.empty(), "fixture must produce real routed paths");
  RegionalRouterConfig regional_config;
  regional_config.max_physical_scope_retries = 0U;
  const auto preparation = prepare_regional_routing(board, global, regional_config);
  require(preparation && preparation->build_count() == 1U &&
              preparation->retained_bytes_estimate().value_or(0U) != 0U,
          "prewarm must retain inspectable bounded preparation state");
  const auto fresh = analyze_regional_conflicts(board, global, regional_config);
  const auto reused = analyze_regional_conflicts(board, global, regional_config, {}, nullptr, {}, preparation);
  require(preparation->reuse_count() != 0U && !fresh.fragments.empty() && fresh.fragments == reused.fragments &&
              fresh.conflict_edges == reused.conflict_edges &&
              fresh.native_scene_conflicts == reused.native_scene_conflicts,
          "same-board preparation reuse must preserve regional evidence");

  Board changed = board;
  // Keep the revision string unchanged: equality must notice actual geometry changes.
  changed.outline = Polygon{{{Nanometres::from_mm(0), Nanometres::from_mm(0)},
                             {Nanometres::from_mm(90), Nanometres::from_mm(0)},
                             {Nanometres::from_mm(90), Nanometres::from_mm(80)},
                             {Nanometres::from_mm(0), Nanometres::from_mm(80)}}};
  const auto changed_global = route_global(changed, global_config);
  const auto invalidated = analyze_regional_conflicts(changed, changed_global, regional_config,
                                                       {}, nullptr, {}, preparation);
  const auto changed_fresh = analyze_regional_conflicts(changed, changed_global, regional_config);
  require(preparation->build_count() == 2U && invalidated.fragments == changed_fresh.fragments &&
              invalidated.conflict_edges == changed_fresh.conflict_edges,
          "a changed board must invalidate cached preparation rather than reuse stale geometry");

  // Copper feedback can change without changing the imported source revision. Reuse must match
  // a cold preparation after both laying and ripping copper, not merely after outline edits.
  changed.nets.push_back({NetId{"blocker"}, "BLOCKER", 0});
  changed.tracks.push_back({ObjectId{"fixed-blocker"}, NetId{"blocker"}, LayerId{"F.Cu"},
                            {Nanometres::from_mm(50), Nanometres::from_mm(10)},
                            {Nanometres::from_mm(50), Nanometres::from_mm(70)},
                            Nanometres::from_mm(0.5)});
  const auto laid = analyze_regional_conflicts(changed, changed_global, regional_config,
                                               {}, nullptr, {}, preparation);
  const auto laid_fresh = analyze_regional_conflicts(changed, changed_global, regional_config);
  require(preparation->build_count() == 3U && laid.fragments == laid_fresh.fragments &&
              laid.conflict_edges == laid_fresh.conflict_edges &&
              laid.native_scene_conflicts == laid_fresh.native_scene_conflicts,
          "new copper must invalidate source and navigation certificates");
  changed.tracks.clear();
  const auto ripped = analyze_regional_conflicts(changed, changed_global, regional_config,
                                                 {}, nullptr, {}, preparation);
  const auto ripped_fresh = analyze_regional_conflicts(changed, changed_global, regional_config);
  require(preparation->build_count() == 4U && ripped.fragments == ripped_fresh.fragments &&
              ripped.conflict_edges == ripped_fresh.conflict_edges &&
              ripped.native_scene_conflicts == ripped_fresh.native_scene_conflicts,
          "ripped copper must invalidate source and navigation certificates");
}

void bounded_physical_prefix_precedes_broad_portfolio_and_preserves_gates() {
  const LayerId front{"F.Cu"};
  const LayerId back{"B.Cu"};
  const auto rectangle = [](double min_x, double min_y, double max_x, double max_y) {
    return Polygon{{{Nanometres::from_mm(min_x), Nanometres::from_mm(min_y)},
                    {Nanometres::from_mm(max_x), Nanometres::from_mm(min_y)},
                    {Nanometres::from_mm(max_x), Nanometres::from_mm(max_y)},
                    {Nanometres::from_mm(min_x), Nanometres::from_mm(max_y)}}};
  };
  Board board;
  board.source_revision = "regional-physical-prefix";
  board.outline = rectangle(0.0, 0.0, 100.0, 40.0);
  board.layers = {{front, "F.Cu", LayerKind::copper, 0U},
                  {back, "B.Cu", LayerKind::copper, 1U}};
  const NetId first{"cache-a"};
  const NetId second{"cache-b"};
  board.nets = {{first, "CACHE_A", 0U}, {second, "CACHE_B", 1U}};
  const auto endpoint = [&](std::string id, const NetId& net, Point position) {
    Footprint footprint;
    footprint.id = ObjectId{id};
    footprint.reference = id;
    footprint.position = position;
    footprint.courtyard = rectangle(-2.0, -2.0, 2.0, 2.0);
    footprint.pads = {{ObjectId{id + "-pad"}, net, Point{}, {front}}};
    return footprint;
  };
  board.footprints = {endpoint("CA1", first, {Nanometres::from_mm(10.0), Nanometres::from_mm(20.0)}),
                      endpoint("CA2", first, {Nanometres::from_mm(90.0), Nanometres::from_mm(20.0)}),
                      endpoint("CB1", second, {Nanometres::from_mm(50.0), Nanometres::from_mm(10.0)}),
                      endpoint("CB2", second, {Nanometres::from_mm(50.0), Nanometres::from_mm(30.0)})};
  GlobalRouteResult global;
  global.paths = {{first, 0U, {{{Nanometres::from_mm(10.0), Nanometres::from_mm(20.0)}, front},
                               {{Nanometres::from_mm(40.0), Nanometres::from_mm(20.0)}, front},
                               {{Nanometres::from_mm(60.0), Nanometres::from_mm(20.0)}, front},
                               {{Nanometres::from_mm(90.0), Nanometres::from_mm(20.0)}, front}}},
                  {second, 0U, {{{Nanometres::from_mm(50.0), Nanometres::from_mm(10.0)}, front},
                                {{Nanometres::from_mm(50.0), Nanometres::from_mm(30.0)}, front}}}};
  for (auto& path : global.paths) {
    RoutingGeometricCorridorRun corridor;
    corridor.layer = front;
    corridor.topology_cells = {rectangle(8.0, 18.0, 92.0, 22.5)};
    corridor.lane_centerline = {path.points.front().position, path.points.back().position};
    corridor.required_envelope_width = Nanometres::from_mm(0.4);
    corridor.minimum_physical_cross_section_width = Nanometres::from_mm(1.0);
    corridor.lane_capacity = 1U;
    corridor.selected_path_preserves_topology = true;
    path.geometric_corridors.push_back(std::move(corridor));
  }
  global.clearance_shadow_conflicts.push_back(
      {RoutingClearanceShadowResourceKind::planar_edge,
       {{Nanometres::from_mm(45.0), Nanometres::from_mm(20.0)}, front},
       {{Nanometres::from_mm(55.0), Nanometres::from_mm(20.0)}, front}, first, second,
       Nanometres::from_mm(0.15),
       GlobalRoutePoint{{Nanometres::from_mm(50.0), Nanometres::from_mm(15.0)}, front},
       GlobalRoutePoint{{Nanometres::from_mm(50.0), Nanometres::from_mm(25.0)}, front}});

  RegionalRouterConfig config;
  config.max_same_layer_detour_checks = 0U;
  config.max_physical_scope_retries = 0U;
  config.max_alternative_template_checks = 16U;
  config.max_early_alternative_template_checks = 2U;
  config.max_bundle_search_states = 4'096U;
  config.max_access_assignment_states = 512U;
  config.max_lane_order_states = 4'096U;
  config.max_early_bundle_search_states = 64U;
  config.max_early_access_assignment_states = 32U;
  config.max_early_lane_order_states = 64U;
  const auto preparation = prepare_regional_routing(board, global, config);
  std::vector<RegionalDecisionPhase> phases;
  std::optional<RegionalRouterConfig> physical_prefix_config;
  struct CommittedWindowEvent {
    RegionalExperimentMetadata metadata;
    std::shared_ptr<const RegionalRouteResult> checkpoint;
  };
  std::vector<CommittedWindowEvent> committed_windows;
  const auto first_pass = analyze_regional_conflicts(
      board, global, config, {}, nullptr,
      [&phases, &physical_prefix_config, &committed_windows](const RegionalDecisionPoint& point) {
        phases.push_back(point.phase);
        if (point.phase == RegionalDecisionPhase::physical_feedback_started && point.checkpoint)
          physical_prefix_config = point.checkpoint->configured;
        if (point.phase == RegionalDecisionPhase::experiment_window_completed) {
          require(point.checkpoint && point.experiment.has_value(),
                  "a committed regional window must publish coherent experiment metadata");
          committed_windows.push_back({*point.experiment, point.checkpoint});
        }
      }, preparation);
  const auto physical = std::find(phases.begin(), phases.end(),
                                  RegionalDecisionPhase::physical_feedback_started);
  const auto portfolio = std::find(phases.begin(), phases.end(),
                                   RegionalDecisionPhase::template_batch_completed);
  require(physical != phases.end() &&
              (portfolio == phases.end() || physical < portfolio) && physical_prefix_config &&
              physical_prefix_config->max_bundle_search_states == 64U &&
              physical_prefix_config->max_access_assignment_states == 32U &&
              physical_prefix_config->max_lane_order_states == 64U,
          "bounded prefix must reach physical feedback before broad portfolio enumeration");
  require(!committed_windows.empty(),
          "the bounded physical prefix must yield after committing each window assignment");
  for (const auto& event : committed_windows) {
    require(event.metadata.window_id &&
                event.metadata.batch_ordinal < event.checkpoint->bundle_assignments.size(),
            "a committed-window yield must identify an assignment already owned by its checkpoint");
    const auto& assignment =
        event.checkpoint->bundle_assignments[event.metadata.batch_ordinal];
    require(assignment.window_id == *event.metadata.window_id &&
                event.metadata.dependencies.owned_fragment_count >=
                    event.metadata.dependencies.owned_fragments.size() &&
                event.metadata.dependencies.conflict_edge_count >=
                    event.metadata.dependencies.conflict_edges.size(),
            "committed-window metadata must be bounded and match its stable assignment state");
    const auto candidate_available =
        assignment.complete() && !assignment.spline_candidate_shortlist.empty();
    require(event.metadata.candidate_available == candidate_available,
            "committed-window availability must describe the committed discrete assignment");
    require(!event.metadata.access_infeasible ||
                (!event.metadata.dependencies.access_budget_exhausted &&
                 assignment.partial_forward_domain_wipeouts != 0U),
            "access infeasibility must never turn a budget-exhausted search into a proven wipeout");
  }

  RegionalRouteResult locally_clean_but_globally_conflicted;
  locally_clean_but_globally_conflicted.spline_candidate_available = true;
  locally_clean_but_globally_conflicted.native_scene_evaluated = true;
  locally_clean_but_globally_conflicted.native_scene_complete = true;
  locally_clean_but_globally_conflicted.native_scene_conflicts = 0U;
  locally_clean_but_globally_conflicted.native_scene_total_conflicts = 2U;
  locally_clean_but_globally_conflicted.native_scene_open_connections = 1U;
  auto fully_legal = locally_clean_but_globally_conflicted;
  fully_legal.native_scene_total_conflicts = 0U;
  fully_legal.native_scene_open_connections = 0U;
  require(regional_checkpoint_better(fully_legal, locally_clean_but_globally_conflicted),
          "whole-scene conflicts and opens must prevent a locally attributed zero from winning "
          "the physical checkpoint comparison");
  auto fewer_native_opens = fully_legal;
  fewer_native_opens.native_scene_total_conflicts = 7U;
  auto more_native_opens = fully_legal;
  more_native_opens.native_scene_open_connections = 1U;
  require(regional_checkpoint_better(fewer_native_opens, more_native_opens),
          "native full-scene opens must outrank native conflict counts in checkpoint selection");

  std::stop_source cancel_prefix;
  std::vector<RegionalDecisionPhase> cancelled_phases;
  const auto cancelled = analyze_regional_conflicts(
      board, global, config, cancel_prefix.get_token(), nullptr,
      [&cancel_prefix, &cancelled_phases](const RegionalDecisionPoint& point) {
        cancelled_phases.push_back(point.phase);
        if (point.phase == RegionalDecisionPhase::spline_candidate_started)
          cancel_prefix.request_stop();
      });
  require(cancelled.stop_reason == RegionalRouterStopReason::cancelled &&
              std::find(cancelled_phases.begin(), cancelled_phases.end(),
                        RegionalDecisionPhase::template_batch_completed) == cancelled_phases.end(),
          "cancellation during the physical prefix must return without enumerating the broad "
          "template portfolio");
}

void stopped_prewarm_returns_without_working_the_source_set() {
  const Board board = fixture_board();
  const GlobalRouteResult global;
  std::stop_source stop;
  stop.request_stop();
  const auto preparation = prepare_regional_routing(board, global, {}, stop.get_token());
  require(preparation && preparation->build_count() == 1U && preparation->reuse_count() == 0U,
          "a pre-cancelled prewarm must return its empty preparation without reuse");
}

void source_index_preserves_touching_obstacles_and_source_order() {
  FreeSpaceDomain source;
  source.outline = fixture_board().outline;
  const auto box = [](std::int64_t x, std::int64_t y) {
    return Polygon{{{Nanometres{x}, Nanometres{y}}, {Nanometres{x + 100}, Nanometres{y}},
                     {Nanometres{x + 100}, Nanometres{y + 100}},
                     {Nanometres{x}, Nanometres{y + 100}}}};
  };
  for (std::int64_t i = 0; i < 80; ++i) {
    // Reverse x order exercises BVH reordering and stable restoration of source order.
    source.obstacles.push_back(box((79 - i) * 100, (i % 7) * 100));
  }
  source.cutouts.push_back(box(300, 300));
  CopperZoneFill fill;
  fill.area = box(500, 300);
  source.filled_obstacles.push_back(fill);
  const detail::RegionalSourceIndex index(source);
  for (std::int64_t x = -100; x < 8200; x += 100) {
    const geometry::AxisAlignedBounds window{{Nanometres{x}, Nanometres{200}},
                                             {Nanometres{x + 300}, Nanometres{500}}};
    std::vector<std::pair<unsigned, std::size_t>> expected;
    const auto append = [&](const Polygon& polygon, unsigned kind, std::size_t i) {
      if (geometry::prepare_polygon(polygon).bounds.overlaps(window)) expected.emplace_back(kind, i);
    };
    for (std::size_t i = 0; i < source.cutouts.size(); ++i) append(source.cutouts[i], 0U, i);
    for (std::size_t i = 0; i < source.obstacles.size(); ++i) append(source.obstacles[i], 1U, i);
    for (std::size_t i = 0; i < source.filled_obstacles.size(); ++i)
      append(source.filled_obstacles[i].area, 2U, i);
    std::vector<std::pair<unsigned, std::size_t>> actual;
    for (const auto& entry : index.query(window)) actual.emplace_back(entry.kind, entry.index);
    require(actual == expected, "source BVH must retain touching geometry and original order");
  }
}

void channel_overlap_batching_preserves_stable_membership() {
  const LayerId front{"F.Cu"};
  const auto rectangle = [](double min_x, double min_y, double max_x, double max_y) {
    return Polygon{{{Nanometres::from_mm(min_x), Nanometres::from_mm(min_y)},
                    {Nanometres::from_mm(max_x), Nanometres::from_mm(min_y)},
                    {Nanometres::from_mm(max_x), Nanometres::from_mm(max_y)},
                    {Nanometres::from_mm(min_x), Nanometres::from_mm(max_y)}}};
  };

  // This bypasses global search deliberately: the regional channel builder consumes retained
  // geometric corridors. Eight disjoint source cells leave eight eligible index candidates for
  // the final oversized cell, whose stable first exact overlap must remain channel zero.
  Board board;
  board.source_revision = "regional-channel-overlap-batch";
  board.outline = rectangle(0.0, 0.0, 100.0, 40.0);
  board.layers = {{front, "F.Cu", LayerKind::copper, 0}};
  GlobalRouteResult global;
  for (std::size_t index = 0U; index < 8U; ++index) {
    const auto net = NetId{"channel-" + std::to_string(index)};
    const auto x = 10.0 + static_cast<double>(index) * 10.0;
    const auto y = 10.0 + static_cast<double>(index);
    board.nets.push_back({net, net.value(), 0U});
    RoutingGeometricCorridorRun corridor;
    corridor.layer = front;
    corridor.topology_cells = {rectangle(x, y - 0.25, x + 1.0, y + 0.25)};
    corridor.lane_centerline = {{Nanometres::from_mm(x), Nanometres::from_mm(y)},
                                {Nanometres::from_mm(x + 1.0), Nanometres::from_mm(y)}};
    corridor.required_envelope_width = Nanometres::from_mm(0.4);
    corridor.minimum_physical_cross_section_width = Nanometres::from_mm(1.0);
    corridor.lane_capacity = 1U;
    corridor.selected_path_preserves_topology = true;
    GlobalRoutePath path;
    path.net = net;
    path.points = {{corridor.lane_centerline.front(), front}, {corridor.lane_centerline.back(), front}};
    path.geometric_corridors.push_back(std::move(corridor));
    global.paths.push_back(std::move(path));
  }
  const NetId broad_net{"channel-broad"};
  board.nets.push_back({broad_net, "channel-broad", 0U});
  RoutingGeometricCorridorRun broad;
  broad.layer = front;
  broad.topology_cells = {rectangle(2.0, 5.0, 98.0, 25.0)};
  broad.lane_centerline = {{Nanometres::from_mm(2.0), Nanometres::from_mm(24.0)},
                           {Nanometres::from_mm(98.0), Nanometres::from_mm(24.0)}};
  broad.required_envelope_width = Nanometres::from_mm(0.4);
  broad.minimum_physical_cross_section_width = Nanometres::from_mm(1.0);
  broad.lane_capacity = 1U;
  broad.selected_path_preserves_topology = true;
  GlobalRoutePath broad_path;
  broad_path.net = broad_net;
  broad_path.points = {{broad.lane_centerline.front(), front}, {broad.lane_centerline.back(), front}};
  broad_path.geometric_corridors.push_back(std::move(broad));
  global.paths.push_back(std::move(broad_path));

  RegionalRouterConfig serial_config;
  serial_config.enable_terminal_portal_templates = false;
  serial_config.enable_spline_realization = false;
  serial_config.max_same_layer_detour_checks = 0U;
  serial_config.max_physical_scope_retries = 0U;
  serial_config.max_spline_cpu_workers = 1U;
  auto batched_config = serial_config;
  batched_config.max_spline_cpu_workers = 4U;

  const auto serial = analyze_regional_conflicts(board, global, serial_config);
  const auto batched = analyze_regional_conflicts(board, global, batched_config);
  require(serial.channels.size() == 8U && serial.channels.front().topology_cells.size() == 2U &&
              serial.channels.front().occupants.size() == 2U,
          "fixture must merge the broad corridor into the stable first source channel");
  require(batched.candidate_pipeline_stats.channel_exact_overlap_checks >= 8U &&
              batched.candidate_pipeline_stats.channel_exact_overlap_checks >
                  serial.candidate_pipeline_stats.channel_exact_overlap_checks,
          "the broad cell must drive at least eight exact immutable channel-overlap checks");
  require(serial.channels == batched.channels,
          "serial and bounded-batch channel reduction must preserve topology and occupants");
}
} // namespace

int main() {
  preparation_reuse_and_invalidation_are_semantically_safe();
  bounded_physical_prefix_precedes_broad_portfolio_and_preserves_gates();
  stopped_prewarm_returns_without_working_the_source_set();
  source_index_preserves_touching_obstacles_and_source_order();
  channel_overlap_batching_preserves_stable_membership();
  return 0;
}
