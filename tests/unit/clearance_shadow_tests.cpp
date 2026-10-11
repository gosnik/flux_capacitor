#include "flux/core/global_router.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using flux::Board;
using flux::Footprint;
using flux::GlobalRoutePath;
using flux::GlobalRouterConfig;
using flux::LayerId;
using flux::LayerKind;
using flux::Nanometres;
using flux::Net;
using flux::NetId;
using flux::ObjectId;
using flux::Pad;
using flux::Point;
using flux::Polygon;

struct TestCase {
  std::string name;
  std::function<void()> run;
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

[[nodiscard]] Footprint terminal(std::string id, NetId net, Point position) {
  Footprint footprint;
  footprint.id = ObjectId{id};
  footprint.reference = id;
  footprint.position = position;
  footprint.courtyard = local_rectangle(0.01, 0.01);
  Pad pad;
  pad.id = ObjectId{id + "-pad"};
  pad.net = std::move(net);
  pad.layers = {LayerId{"F.Cu"}};
  pad.shape = flux::PadShape::rectangle;
  pad.width = Nanometres::from_mm(0.01);
  pad.height = Nanometres::from_mm(0.01);
  footprint.pads.push_back(std::move(pad));
  return footprint;
}

[[nodiscard]] GlobalRouterConfig shadow_config(bool advisory = false) {
  GlobalRouterConfig config;
  config.grid_pitch = Nanometres::from_mm(0.65);
  config.max_iterations = 1U;
  config.max_pin_escape_grid_radius = 0U;
  config.max_pin_access_nodes_per_terminal = 1U;
  config.use_free_space_corridors = false;
  config.use_navmesh_funnel_spline_seeds = false;
  config.allow_diagonal_grid_edges = false;
  config.allow_crossing_safe_diagonal_grid_edges = true;
  config.clearance_shadow_cost_quantum = Nanometres::from_mm(0.05);
  config.present_congestion_cost = 0U;
  config.history_congestion_cost = 0U;
  config.clearance_shadows_advisory_only = advisory;
  return config;
}

[[nodiscard]] Board diagonal_shadow_board(bool crossing = false) {
  Board board;
  board.source_revision = crossing ? "clearance-shadow-crossing" : "clearance-shadow-legal";
  board.outline = rectangle(0.0, 0.0, 2.6, 2.6);
  board.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0}};
  board.nets = {{NetId{"diagonal-a"}, "A", 0}, {NetId{"owner-b"}, "B", 0}};
  board.rules.default_track_width = Nanometres::from_mm(0.25);
  board.rules.default_clearance = Nanometres::from_mm(0.20);
  board.rules.copper_to_edge_clearance = Nanometres{0};

  // Keep the test within one 0.65 mm grid cell.  In the legal fixture, B's nearest point to A is
  // (0.65, 1.30), whose distance from A's chord is 0.65 / sqrt(2) = 0.459619... mm, above the
  // 0.25 + 0.20 mm copper-envelope requirement.  The crossing fixture uses the other chord.
  board.footprints = {
      terminal("A1", NetId{"diagonal-a"}, mm(0.65, 0.65)),
      terminal("A2", NetId{"diagonal-a"}, mm(1.30, 1.30)),
      terminal("B1", NetId{"owner-b"}, crossing ? mm(0.65, 1.30) : mm(0.65, 1.30)),
      terminal("B2", NetId{"owner-b"}, crossing ? mm(1.30, 0.65) : mm(0.65, 1.95)),
  };
  return board;
}

[[nodiscard]] Board corridor_packing_board() {
  Board board;
  board.source_revision = "clearance-shadow-corridor-packing";
  board.outline = rectangle(0.0, 0.0, 3.25, 3.9);
  board.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0}};
  board.nets = {{NetId{"corridor-a"}, "A", 0}, {NetId{"corridor-b"}, "B", 0}};
  board.rules.default_track_width = Nanometres::from_mm(0.25);
  board.rules.default_clearance = Nanometres::from_mm(0.20);
  board.rules.copper_to_edge_clearance = Nanometres{0};
  // Keep source pads away from the central sampled corridor. The frozen paths below deliberately
  // overlap only as planning centrelines, so capacity-qualified corridor accounting—not pad
  // access geometry—owns the assertion.
  board.footprints = {
      terminal("A1", NetId{"corridor-a"}, mm(0.65, 0.65)),
      terminal("A2", NetId{"corridor-a"}, mm(2.60, 0.65)),
      terminal("B1", NetId{"corridor-b"}, mm(0.65, 3.25)),
      terminal("B2", NetId{"corridor-b"}, mm(2.60, 3.25)),
  };
  return board;
}

[[nodiscard]] GlobalRouterConfig corridor_packing_config() {
  auto config = shadow_config();
  config.use_free_space_corridors = true;
  return config;
}

[[nodiscard]] bool has_direct_chord(const flux::GlobalRouteResult& routed, const NetId& net,
                                    Point first, Point second) {
  return std::any_of(routed.paths.begin(), routed.paths.end(), [&](const GlobalRoutePath& path) {
    return path.net == net && path.points.size() == 2U && path.points.front().position == first &&
           path.points.back().position == second;
  });
}

[[nodiscard]] GlobalRoutePath frozen_path(NetId net, Point first, Point second,
                                           Nanometres width = Nanometres{}) {
  GlobalRoutePath path;
  path.net = std::move(net);
  path.points = {{first, LayerId{"F.Cu"}}, {second, LayerId{"F.Cu"}}};
  path.segment_widths = {width};
  path.segment_width_kinds = {flux::GlobalRouteSegmentWidthKind::nominal};
  return path;
}

[[nodiscard]] GlobalRoutePath frozen_polyline(NetId net, std::vector<Point> points,
                                               Nanometres width = Nanometres{}) {
  GlobalRoutePath path;
  path.net = std::move(net);
  path.points.reserve(points.size());
  for (const auto point : points) {
    path.points.push_back({point, LayerId{"F.Cu"}});
  }
  if (points.size() > 1U) {
    path.segment_widths.assign(points.size() - 1U, width);
    path.segment_width_kinds.assign(points.size() - 1U,
                                    flux::GlobalRouteSegmentWidthKind::nominal);
  }
  return path;
}

[[nodiscard]] flux::GlobalRouteResult frozen_checkpoint(
    const Board& board, const GlobalRouterConfig& config, std::vector<GlobalRoutePath> paths) {
  // Start from a real result so every configuration fingerprint is populated, then substitute
  // exact externally frozen geometry.  preserve_unaffected_provisional_congestion is the
  // established API for replaying that geometry without introducing synthetic resource usage.
  auto checkpoint = flux::route_global(board, config);
  checkpoint.paths = std::move(paths);
  checkpoint.resource_usage.clear();
  checkpoint.corridor_usage.clear();
  checkpoint.corridor_neck_down_usage.clear();
  checkpoint.voronoi_resource_usage.clear();
  checkpoint.unrouted_nets.clear();
  return checkpoint;
}

[[nodiscard]] flux::GlobalRouteResult frozen_checkpoint_with_sampled_usage(
    const Board& board, const GlobalRouterConfig& config, std::vector<GlobalRoutePath> paths) {
  auto checkpoint = frozen_checkpoint(board, config, std::move(paths));
  for (const auto& path : checkpoint.paths) {
    for (std::size_t index = 1U; index < path.points.size(); ++index) {
      const auto& first = path.points[index - 1U];
      const auto& second = path.points[index];
      if (first.layer != second.layer) {
        continue;
      }
      checkpoint.resource_usage.push_back(flux::RoutingResourceUsage{
          flux::RoutingResourceKind::planar_edge, first, second, 1U, config.edge_capacity,
          {path.net}});
    }
  }
  return checkpoint;
}

[[nodiscard]] std::string path_summary(const flux::GlobalRouteResult& routed) {
  std::string summary{"paths=" + std::to_string(routed.paths.size())};
  for (const auto& path : routed.paths) {
    summary += " [" + path.net.value() + ":" + std::to_string(path.points.size());
    for (const auto& point : path.points) {
      summary += " (" + std::to_string(point.position.x.value()) + "," +
                 std::to_string(point.position.y.value()) + ")";
    }
    summary += "]";
  }
  return summary;
}

[[nodiscard]] std::string route_summary(const flux::GlobalRouteResult& routed) {
  auto summary = path_summary(routed) + ", reused=" + std::to_string(routed.reused_paths) +
         ", rerouted=" + std::to_string(routed.rerouted_nets.size()) +
         ", invalidated=" + std::to_string(routed.invalidated_reuse_nets.size()) +
         ", shadows=" + std::to_string(routed.clearance_shadow_overflow) +
         ", obligations=" + std::to_string(routed.unresolved_corridor_packing.size()) +
         ", diagnostic=" + routed.diagnostic;
  return summary;
}

void legal_frozen_owner_does_not_cast_a_capacity_alias_shadow() {
  const auto board = diagonal_shadow_board();
  const auto config = shadow_config();
  const auto checkpoint = frozen_checkpoint(
      board, config,
      {frozen_path(NetId{"diagonal-a"}, mm(0.65, 0.65), mm(1.30, 1.30)),
       frozen_path(NetId{"owner-b"}, mm(0.65, 1.30), mm(0.65, 1.95))});

  const auto rebuilt = flux::route_global_incremental(
      board, checkpoint, {}, config, {}, nullptr, true);
  require(rebuilt.reused_nets == std::vector<NetId>{NetId{"diagonal-a"}, NetId{"owner-b"}} &&
              rebuilt.reused_paths == 2U && rebuilt.rerouted_nets.empty() &&
              has_direct_chord(rebuilt, NetId{"diagonal-a"}, mm(0.65, 0.65), mm(1.30, 1.30)) &&
              rebuilt.clearance_shadow_overflow == 0U && rebuilt.clearance_shadow_conflicts.empty(),
          "a frozen, legally separated owner must not poison the selected diagonal through its "
          "opposite-cell capacity alias: " + route_summary(rebuilt));

  auto advisory_config = shadow_config(true);
  const auto advisory_checkpoint = frozen_checkpoint(
      board, advisory_config,
      {frozen_path(NetId{"diagonal-a"}, mm(0.65, 0.65), mm(1.30, 1.30)),
       frozen_path(NetId{"owner-b"}, mm(0.65, 1.30), mm(0.65, 1.95))});
  const auto advisory = flux::route_global_incremental(
      board, advisory_checkpoint, {}, advisory_config, {}, nullptr, true);
  require(advisory.configured_clearance_shadows_advisory_only &&
              advisory.clearance_shadow_overflow == 0U && advisory.clearance_shadow_conflicts.empty(),
          "advisory shadows must use the same exact legal-distance predicate");
}

void actual_crossing_diagonals_remain_shadow_conflicts_in_both_modes() {
  const auto board = diagonal_shadow_board(true);
  for (const bool advisory : {false, true}) {
    const auto config = shadow_config(advisory);
    const auto checkpoint = frozen_checkpoint(
        board, config,
        {frozen_path(NetId{"diagonal-a"}, mm(0.65, 0.65), mm(1.30, 1.30)),
         frozen_path(NetId{"owner-b"}, mm(0.65, 1.30), mm(1.30, 0.65))});
    const auto rebuilt = flux::route_global_incremental(
        board, checkpoint, {}, config, {}, nullptr, true);
    require(has_direct_chord(rebuilt, NetId{"diagonal-a"}, mm(0.65, 0.65), mm(1.30, 1.30)) &&
                has_direct_chord(rebuilt, NetId{"owner-b"}, mm(0.65, 1.30), mm(1.30, 0.65)) &&
                rebuilt.clearance_shadow_overflow > 0U &&
                !rebuilt.clearance_shadow_conflicts.empty() &&
                rebuilt.unresolved_corridor_packing.empty(),
            "actual opposite-diagonal crossings must remain physical clearance-shadow conflicts "
            "in both enforced and advisory modes: " + path_summary(rebuilt));
  }
}

void selected_neck_down_width_controls_shadow_qualification() {
  const auto board = diagonal_shadow_board();
  const auto config = shadow_config();
  // The owner begins 0.565 mm above the first diagonal endpoint: its closest centreline distance
  // is 0.3995 mm.  That is legal for 0.25 mm copper beside a selected 0.05 mm neck-down at
  // 0.20 mm clearance (0.35 mm requirement), but illegal if the selected segment is charged at
  // its 0.25 mm nominal width (0.45 mm requirement).
  const auto narrow_checkpoint = frozen_checkpoint(
      board, config,
      {frozen_path(NetId{"diagonal-a"}, mm(0.65, 0.65), mm(1.30, 1.30)),
       frozen_path(NetId{"owner-b"}, mm(0.65, 1.215), mm(0.65, 1.95),
                   Nanometres::from_mm(0.05))});
  const auto narrow = flux::route_global_incremental(
      board, narrow_checkpoint, {}, config, {}, nullptr, true);
  auto nominal_checkpoint = narrow_checkpoint;
  nominal_checkpoint.paths.back().segment_widths = {Nanometres::from_mm(0.25)};
  const auto nominal = flux::route_global_incremental(
      board, nominal_checkpoint, {}, config, {}, nullptr, true);
  require(narrow.reused_paths == 2U && narrow.clearance_shadow_overflow == 0U &&
              narrow.clearance_shadow_conflicts.empty() && nominal.reused_paths == 2U &&
              nominal.clearance_shadow_overflow > 0U && !nominal.clearance_shadow_conflicts.empty(),
          "clearance shadows must use the selected neck-down width, while the nominal-width "
          "control remains a physical conflict: narrow=" + route_summary(narrow) +
              ", nominal=" + route_summary(nominal));
}

void long_direct_chord_uses_exact_midsegment_shadow_geometry() {
  const auto board = diagonal_shadow_board();
  const auto config = shadow_config();
  const auto long_path = frozen_path(NetId{"diagonal-a"}, mm(0.65, 0.65), mm(1.95, 0.65));
  const auto crossing_owner = frozen_path(NetId{"owner-b"}, mm(1.30, 0.0), mm(1.30, 1.30));
  const auto crossing_checkpoint = frozen_checkpoint(board, config, {long_path, crossing_owner});
  const auto crossing = flux::route_global_incremental(
      board, crossing_checkpoint, {}, config, {}, nullptr, true);
  require(crossing.reused_paths == 2U &&
              has_direct_chord(crossing, NetId{"diagonal-a"}, mm(0.65, 0.65), mm(1.95, 0.65)) &&
              crossing.clearance_shadow_overflow > 0U &&
              !crossing.clearance_shadow_conflicts.empty(),
          "a retained multi-cell direct chord must detect a foreign chord crossing at its middle, "
          "rather than relying on endpoint-local sampled shadows: " + route_summary(crossing));

  const auto legal_owner = frozen_path(NetId{"owner-b"}, mm(1.30, 1.30), mm(1.30, 1.95));
  const auto legal_checkpoint = frozen_checkpoint(board, config, {long_path, legal_owner});
  const auto legal = flux::route_global_incremental(
      board, legal_checkpoint, {}, config, {}, nullptr, true);
  require(legal.reused_paths == 2U &&
              has_direct_chord(legal, NetId{"diagonal-a"}, mm(0.65, 0.65), mm(1.95, 0.65)) &&
              legal.clearance_shadow_overflow == 0U && legal.clearance_shadow_conflicts.empty(),
          "the exact long-chord check must preserve a genuinely legal offset owner: " +
              route_summary(legal));
}

void same_corridor_overlap_becomes_a_packing_obligation() {
  const auto board = corridor_packing_board();
  const auto config = corridor_packing_config();
  const auto checkpoint = frozen_checkpoint_with_sampled_usage(
      board, config,
      {frozen_polyline(NetId{"corridor-a"},
                       {mm(0.65, 1.30), mm(1.30, 1.30), mm(1.95, 1.30), mm(2.60, 1.30)}),
       frozen_polyline(NetId{"corridor-b"}, {mm(1.30, 1.30), mm(1.95, 1.30)})});
  const auto rebuilt = flux::route_global_incremental(
      board, checkpoint, {}, config, {}, nullptr, true);
  require(rebuilt.unresolved_corridor_packing.size() == 1U &&
              !rebuilt.corridor_packing_obligation_budget_exhausted &&
              rebuilt.unresolved_corridor_packing.front().witness.first_net == NetId{"corridor-a"} &&
              rebuilt.unresolved_corridor_packing.front().witness.second_net == NetId{"corridor-b"} &&
              rebuilt.unresolved_corridor_packing.front().witness.first.layer == LayerId{"F.Cu"},
          "same-corridor selected copper overlap must produce one packing obligation; ordinary "
          "cross-corridor shadow witnesses remain independently attributable: " +
              route_summary(rebuilt));
}

void legally_separated_corridor_widths_do_not_create_a_packing_obligation() {
  const auto board = corridor_packing_board();
  const auto config = corridor_packing_config();
  const auto checkpoint = frozen_checkpoint_with_sampled_usage(
      board, config,
      {frozen_polyline(NetId{"corridor-a"},
                       {mm(0.65, 0.65), mm(1.30, 0.65), mm(1.95, 0.65), mm(2.60, 0.65)},
                       Nanometres::from_mm(0.05)),
       frozen_polyline(NetId{"corridor-b"},
                       {mm(0.65, 1.30), mm(1.30, 1.30), mm(1.95, 1.30), mm(2.60, 1.30)},
                       Nanometres::from_mm(0.05))});
  const auto rebuilt = flux::route_global_incremental(
      board, checkpoint, {}, config, {}, nullptr, true);
  require(rebuilt.clearance_shadow_conflicts.empty() && rebuilt.clearance_shadow_overflow == 0U &&
              rebuilt.unresolved_corridor_packing.empty() &&
              !rebuilt.corridor_packing_obligation_budget_exhausted,
          "selected narrow traces with legal exact separation must not become corridor packing "
          "obligations: " + route_summary(rebuilt));
}

void corridor_packing_obligation_budget_bounds_distinct_net_pairs() {
  auto board = corridor_packing_board();
  board.nets.push_back(Net{NetId{"corridor-c"}, "C", 0});
  board.footprints.push_back(terminal("C1", NetId{"corridor-c"}, mm(1.30, 0.65)));
  board.footprints.push_back(terminal("C2", NetId{"corridor-c"}, mm(1.95, 0.65)));
  auto config = corridor_packing_config();
  config.max_clearance_shadow_conflict_resources = 1U;
  const auto checkpoint = frozen_checkpoint_with_sampled_usage(
      board, config,
      {frozen_polyline(NetId{"corridor-a"},
                       {mm(0.65, 1.30), mm(1.30, 1.30), mm(1.95, 1.30), mm(2.60, 1.30)}),
       frozen_polyline(NetId{"corridor-b"}, {mm(1.30, 1.30), mm(1.95, 1.30)}),
       frozen_polyline(NetId{"corridor-c"},
                       {mm(0.65, 1.30), mm(1.30, 1.30), mm(1.95, 1.30), mm(2.60, 1.30)})});
  const auto rebuilt = flux::route_global_incremental(
      board, checkpoint, {}, config, {}, nullptr, true);
  require(rebuilt.unresolved_corridor_packing.size() == 1U &&
              rebuilt.corridor_packing_obligation_budget_exhausted,
          "the deferred-packing evidence cap must bound multiple distinct same-corridor net "
          "pairs independently of ordinary shadow witnesses: " + route_summary(rebuilt));
}

} // namespace

int main() {
  const std::vector<TestCase> tests{
      {"legal_frozen_owner_does_not_cast_a_capacity_alias_shadow",
       legal_frozen_owner_does_not_cast_a_capacity_alias_shadow},
      {"actual_crossing_diagonals_remain_shadow_conflicts_in_both_modes",
       actual_crossing_diagonals_remain_shadow_conflicts_in_both_modes},
      {"selected_neck_down_width_controls_shadow_qualification",
       selected_neck_down_width_controls_shadow_qualification},
      {"long_direct_chord_uses_exact_midsegment_shadow_geometry",
       long_direct_chord_uses_exact_midsegment_shadow_geometry},
      {"same_corridor_overlap_becomes_a_packing_obligation",
       same_corridor_overlap_becomes_a_packing_obligation},
      {"legally_separated_corridor_widths_do_not_create_a_packing_obligation",
       legally_separated_corridor_widths_do_not_create_a_packing_obligation},
      {"corridor_packing_obligation_budget_bounds_distinct_net_pairs",
       corridor_packing_obligation_budget_bounds_distinct_net_pairs},
  };
  std::size_t failures = 0U;
  for (const auto& test : tests) {
    try {
      test.run();
      std::cout << "PASS " << test.name << '\n';
    } catch (const std::exception& error) {
      ++failures;
      std::cerr << "FAIL " << test.name << ": " << error.what() << '\n';
    }
  }
  return failures == 0U ? 0 : 1;
}
