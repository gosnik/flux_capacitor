#include "flux/core/detailed_router.hpp"
#include "flux/core/geometry.hpp"
#include "flux/core/navmesh_route.hpp"

#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <stdexcept>

namespace {
using namespace flux;
[[nodiscard]] Point mm(double x, double y) { return {Nanometres::from_mm(x), Nanometres::from_mm(y)}; }
void require(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
[[nodiscard]] Board board() {
  Board result;
  result.outline = Polygon{{mm(0, 0), mm(30, 0), mm(30, 30), mm(0, 30)}};
  result.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0U}, {LayerId{"B.Cu"}, "B.Cu", LayerKind::copper, 1U}};
  result.nets = {{NetId{"scaffold"}, "scaffold", 1U}};
  result.rules.default_track_width = Nanometres::from_mm(0.2);
  result.rules.default_clearance = Nanometres::from_mm(0.2);
  return result;
}
void add_pad(Board& value, std::string id, Point position) {
  Footprint footprint;
  footprint.placement_envelope_source = PlacementEnvelopeSource::absent;
  footprint.id = ObjectId{"footprint-" + id};
  footprint.reference = id;
  footprint.position = position;
  footprint.side = BoardSide::front;
  Pad pad;
  pad.id = ObjectId{"pad-" + id};
  pad.net = NetId{"scaffold"};
  pad.layers = {LayerId{"F.Cu"}};
  pad.shape = PadShape::circle;
  pad.width = Nanometres::from_mm(1.0);
  pad.height = Nanometres::from_mm(1.0);
  footprint.pads.push_back(std::move(pad));
  value.footprints.push_back(std::move(footprint));
}
void materializes_complete_scaffold_without_regional_plan() {
  auto source = board();
  GlobalRoutePath path;
  path.net = NetId{"scaffold"}; path.legal_scaffold = true;
  path.points = {{mm(2, 2), LayerId{"F.Cu"}}, {mm(2, 8), LayerId{"F.Cu"}},
                 {mm(12, 8), LayerId{"F.Cu"}}, {mm(12, 8), LayerId{"B.Cu"}},
                 {mm(12, 18), LayerId{"B.Cu"}}, {mm(24, 18), LayerId{"B.Cu"}}};
  RoutingPlanResult plan;
  plan.legal_scaffold_complete = true; plan.paths = {path};
  plan.coarse_global.stop_reason = GlobalRouterStopReason::complete;
  DetailedRouterConfig config;
  config.pull_tight_45_degrees = true; config.continuous_refinement = true;
  config.residual_bend_repair = false;
  const auto detailed = materialize_detailed_route(source, plan, config);
  require(detailed.complete() && detailed.candidate && detailed.copper_drc.ok(), "complete certified scaffold must materialize without regional evidence");
  require(detailed.candidate->vias.size() == 1U && detailed.candidate->vias.front().position == mm(12, 8), "certified scaffold must retain exact via site");
  require(detailed.candidate->tracks.size() == 4U && detailed.candidate->tracks.front().start == mm(2, 2) && detailed.candidate->tracks.front().end == mm(2, 8) && detailed.candidate->tracks.back().start == mm(12, 18) && detailed.candidate->tracks.back().end == mm(24, 18), "certified scaffold must retain terminal approach directions and planar topology");
}
void final_drc_epsilon_preserves_strict_materialization_by_default() {
  auto source = board();
  source.nets.push_back({NetId{"near-left"}, "near-left", 2U});
  source.nets.push_back({NetId{"near-right"}, "near-right", 3U});
  Footprint footprint;
  footprint.id = ObjectId{"near-threshold-source"};
  footprint.placement_envelope_source = PlacementEnvelopeSource::absent;
  footprint.position = mm(20, 20);
  Pad left;
  left.id = ObjectId{"near-left-pad"};
  left.net = NetId{"near-left"};
  left.layers = {LayerId{"F.Cu"}};
  left.shape = PadShape::circle;
  left.width = left.height = Nanometres{650'000};
  Pad right = left;
  right.id = ObjectId{"near-right-pad"};
  right.net = NetId{"near-right"};
  right.local_position = {Nanometres{849'872}, Nanometres{0}};
  footprint.pads = {left, right};
  source.footprints.push_back(std::move(footprint));
  GlobalRoutePath path;
  path.net = NetId{"scaffold"};
  path.legal_scaffold = true;
  path.points = {{mm(2, 2), LayerId{"F.Cu"}}, {mm(2, 8), LayerId{"F.Cu"}},
                 {mm(12, 8), LayerId{"F.Cu"}}};
  RoutingPlanResult plan;
  plan.legal_scaffold_complete = true;
  plan.paths = {path};
  plan.coarse_global.stop_reason = GlobalRouterStopReason::complete;
  DetailedRouterConfig strict;
  strict.circular_arc_refinement = true;
  strict.residual_bend_repair = false;
  const auto strict_result = materialize_detailed_route(source, plan, strict);
  DetailedRouterConfig parity = strict;
  parity.final_copper_drc.clearance_epsilon = Nanometres{500};
  const auto parity_result = materialize_detailed_route(source, plan, parity);
  require(strict_result.stop_reason == DetailedRouterStopReason::copper_violations &&
              !strict_result.complete() && parity_result.complete() && parity_result.candidate &&
              parity_result.copper_drc.ok() &&
              (!parity_result.candidate->track_arcs.empty() || !parity_result.candidate->track_beziers.empty()) &&
              !parity_result.fallback_polyline_materialization,
          "only explicit final KiCad DRC parity may retain curved materialization across a bounded "
          "immutable pad shortfall");
}

void explicit_nominal_widths_match_implicit_certified_scaffold() {
  const auto source = board();
  GlobalRoutePath implicit;
  implicit.net = NetId{"scaffold"}; implicit.legal_scaffold = true;
  implicit.points = {{mm(2, 2), LayerId{"F.Cu"}}, {mm(2, 8), LayerId{"F.Cu"}},
                     {mm(12, 8), LayerId{"F.Cu"}}, {mm(12, 8), LayerId{"B.Cu"}},
                     {mm(12, 18), LayerId{"B.Cu"}}, {mm(24, 18), LayerId{"B.Cu"}}};
  auto explicit_nominal = implicit;
  explicit_nominal.segment_widths.assign(implicit.points.size() - 1U, source.rules.default_track_width);
  explicit_nominal.segment_width_kinds.assign(implicit.points.size() - 1U, GlobalRouteSegmentWidthKind::nominal);
  const auto plan_for = [](GlobalRoutePath path) {
    RoutingPlanResult plan;
    plan.legal_scaffold_complete = true; plan.paths = {std::move(path)};
    plan.coarse_global.stop_reason = GlobalRouterStopReason::complete;
    return plan;
  };
  DetailedRouterConfig config;
  config.pull_tight_45_degrees = true; config.continuous_refinement = true;
  config.residual_bend_repair = false;
  const auto implicit_result = materialize_detailed_route(source, plan_for(implicit), config);
  const auto explicit_result = materialize_detailed_route(source, plan_for(explicit_nominal), config);
  require(implicit_result.complete() && explicit_result.complete() && implicit_result.candidate && explicit_result.candidate &&
              implicit_result.copper_drc.ok() && explicit_result.copper_drc.ok(),
          "explicit nominal certified widths must materialize like implicit nominal widths");
  require(implicit_result.candidate->tracks == explicit_result.candidate->tracks &&
              implicit_result.candidate->vias == explicit_result.candidate->vias &&
              implicit_result.candidate->track_arcs == explicit_result.candidate->track_arcs &&
              implicit_result.candidate->track_beziers == explicit_result.candidate->track_beziers,
          "nominal width metadata must preserve certified multilayer copper and topology");
}
void malformed_certified_width_metadata_is_rejected() {
  const auto source = board();
  GlobalRoutePath path;
  path.net = NetId{"scaffold"}; path.legal_scaffold = true;
  path.points = {{mm(2, 2), LayerId{"F.Cu"}}, {mm(2, 8), LayerId{"F.Cu"}},
                 {mm(12, 8), LayerId{"F.Cu"}}, {mm(12, 8), LayerId{"B.Cu"}},
                 {mm(12, 18), LayerId{"B.Cu"}}, {mm(24, 18), LayerId{"B.Cu"}}};
  path.segment_widths = {source.rules.default_track_width};
  path.segment_width_kinds = {GlobalRouteSegmentWidthKind::nominal};
  RoutingPlanResult plan;
  plan.legal_scaffold_complete = true; plan.paths = {path};
  plan.coarse_global.stop_reason = GlobalRouterStopReason::complete;
  const auto detailed = materialize_detailed_route(source, plan, {});
  require(!detailed.complete() && detailed.stop_reason == DetailedRouterStopReason::invalid_path,
          "misaligned certified width metadata must remain invalid");
}
void unbacked_neck_down_metadata_is_rejected() {
  const auto source = board();
  for (int policy = 0; policy < 3; ++policy) {
    GlobalRoutePath path;
    path.net = NetId{"scaffold"}; path.legal_scaffold = true;
    path.points = {{mm(2, 2), LayerId{"F.Cu"}}, {mm(12, 2), LayerId{"F.Cu"}}};
    if (policy != 0) {
      path.segment_widths = {Nanometres{0}};
      path.segment_width_kinds = {GlobalRouteSegmentWidthKind::nominal};
    }
    if (policy == 2) path.segment_width_kinds.front() = GlobalRouteSegmentWidthKind::terminal_neck_down;
    else path.terminal_neck_down_segments = 1U;
    RoutingPlanResult plan;
    plan.legal_scaffold_complete = true; plan.paths = {path};
    plan.coarse_global.stop_reason = GlobalRouterStopReason::complete;
    const auto detailed = materialize_detailed_route(source, plan, {});
    require(!detailed.complete() && detailed.stop_reason == DetailedRouterStopReason::invalid_path,
            "neck-down metadata without explicit narrowed widths must remain invalid");
  }
}
void invalid_certified_layer_transition_is_rejected() {
  const auto source = board();
  GlobalRoutePath path;
  path.net = NetId{"scaffold"}; path.legal_scaffold = true;
  path.points = {{mm(2, 2), LayerId{"F.Cu"}}, {mm(2, 8), LayerId{"F.Cu"}},
                 {mm(12, 8), LayerId{"F.Cu"}}, {mm(12.1, 8), LayerId{"B.Cu"}},
                 {mm(12, 18), LayerId{"B.Cu"}}, {mm(24, 18), LayerId{"B.Cu"}}};
  RoutingPlanResult plan;
  plan.legal_scaffold_complete = true; plan.paths = {path};
  plan.coarse_global.stop_reason = GlobalRouterStopReason::complete;
  const auto detailed = materialize_detailed_route(source, plan, {});
  require(!detailed.complete() && detailed.stop_reason == DetailedRouterStopReason::invalid_path,
          "non-coincident certified layer transitions must remain invalid");
}
void failed_curve_fit_retains_scaffold_and_unrelated_copper() {
  auto source = board();
  source.nets.push_back({NetId{"unrelated"}, "unrelated", 2U});
  GlobalRoutePath scaffold;
  scaffold.net = NetId{"scaffold"}; scaffold.legal_scaffold = true;
  scaffold.points = {{mm(2, 2), LayerId{"F.Cu"}}, {mm(2, 10), LayerId{"F.Cu"}}, {mm(12, 10), LayerId{"F.Cu"}}, {mm(12, 10), LayerId{"B.Cu"}}, {mm(12, 20), LayerId{"B.Cu"}}, {mm(24, 20), LayerId{"B.Cu"}}};
  GlobalRoutePath unrelated;
  unrelated.net = NetId{"unrelated"}; unrelated.legal_scaffold = true;
  unrelated.points = {{mm(2, 26), LayerId{"F.Cu"}}, {mm(24, 26), LayerId{"F.Cu"}}};
  RoutingPlanResult plan;
  plan.legal_scaffold_complete = true; plan.paths = {scaffold, unrelated};
  plan.coarse_global.stop_reason = GlobalRouterStopReason::complete;
  DetailedRouterConfig config;
  config.pull_tight_45_degrees = true; config.continuous_refinement = true;
  config.circular_arc_refinement = true;
  config.spline_minimum_bend_radius = Nanometres::from_mm(100.0);
  config.spline_control_corridor = Nanometres::from_mm(0.1);
  config.retry_failed_spline_routes_after_commits = false;
  config.residual_bend_repair = false;
  const auto detailed = materialize_detailed_route(source, plan, config);
  require(!detailed.complete() && detailed.candidate && detailed.copper_drc.ok() && detailed.unrounded_non_junction_bends > 0U, "forced local curve-fit failure must retain a DRC-clean scaffold candidate");
  const auto& candidate = *detailed.candidate;
  const auto has_track = [&candidate](NetId net, LayerId layer, Point start, Point end) {
    return std::any_of(candidate.tracks.begin(), candidate.tracks.end(), [&](const TrackSegment& track) { return track.net == net && track.layer == layer && track.start == start && track.end == end; });
  };
  require(candidate.vias.size() == 1U && candidate.vias.front().position == mm(12, 10), "failed curve fit must retain the certified via site");
  require(has_track(NetId{"scaffold"}, LayerId{"F.Cu"}, mm(2, 2), mm(2, 10)) && has_track(NetId{"scaffold"}, LayerId{"B.Cu"}, mm(12, 20), mm(24, 20)), "failed curve fit must retain scaffold terminal escape directions");
  require(has_track(NetId{"unrelated"}, LayerId{"F.Cu"}, mm(2, 26), mm(24, 26)) && candidate.tracks.size() == 5U && candidate.track_arcs.empty() && candidate.track_beziers.empty(), "a failed local scaffold fit must not mutate unrelated copper or replace topology");
}
void residual_bend_repair_uses_exact_fillet_after_ph_rejection() {
  auto source = board();
  GlobalRoutePath path;
  path.net = NetId{"scaffold"}; path.legal_scaffold = true;
  path.points = {{mm(2, 2), LayerId{"F.Cu"}}, {mm(2, 10), LayerId{"F.Cu"}},
                 {mm(12, 16), LayerId{"F.Cu"}}};
  RoutingPlanResult plan;
  plan.legal_scaffold_complete = true; plan.paths = {path};
  plan.coarse_global.stop_reason = GlobalRouterStopReason::complete;
  DetailedRouterConfig config;
  config.circular_arc_refinement = true;
  config.spline_minimum_bend_radius = Nanometres::from_mm(100.0);
  config.spline_control_corridor = Nanometres::from_mm(0.1);
  config.retry_failed_spline_routes_after_commits = false;
  config.max_residual_bend_repair_runs = 1U;
  const auto detailed = materialize_detailed_route(source, plan, config);
  require(detailed.complete() && detailed.candidate && detailed.copper_drc.ok() &&
              detailed.unrounded_non_junction_bends == 0U &&
              detailed.residual_bend_repair_accepted == 1U &&
              detailed.candidate->track_arcs.size() == 1U,
          "a legal straight residual after PH rejection must commit through the exact fillet transaction");
  const auto& arc = detailed.candidate->track_arcs.front();
  const auto circle = geometry::circular_arc_parameters(arc.start, arc.middle, arc.end);
  require(circle.has_value(), "accepted residual arc must have valid quantized circle geometry");
  const auto check_join = [&](Point endpoint) {
    const auto segment = std::find_if(detailed.candidate->tracks.begin(), detailed.candidate->tracks.end(),
        [&](const auto& track) { return track.start == endpoint || track.end == endpoint; });
    require(segment != detailed.candidate->tracks.end(), "arc endpoint must meet an actual segment");
    const auto other = segment->start == endpoint ? segment->end : segment->start;
    const long double dx = static_cast<long double>(other.x.value()) - endpoint.x.value();
    const long double dy = static_cast<long double>(other.y.value()) - endpoint.y.value();
    const long double rx = static_cast<long double>(endpoint.x.value()) - circle->center_x_nm;
    const long double ry = static_cast<long double>(endpoint.y.value()) - circle->center_y_nm;
    const auto scale = std::hypot(dx, dy) * std::hypot(rx, ry);
    require(scale > 0.0L && std::abs(dx * rx + dy * ry) / scale <= 0.035L,
            "accepted oblique fillet must remain tangent after integer quantization");
  };
  check_join(arc.start);
  check_join(arc.end);
}
void residual_bend_batches_resume_without_charging_collinear_points() {
  const auto source = board();
  GlobalRoutePath path;
  path.net = NetId{"scaffold"}; path.legal_scaffold = true;
  path.points = {{mm(2, 2), LayerId{"F.Cu"}}, {mm(2, 4), LayerId{"F.Cu"}},
                 {mm(2, 6), LayerId{"F.Cu"}}, {mm(2, 8), LayerId{"F.Cu"}},
                 {mm(8, 8), LayerId{"F.Cu"}}, {mm(8, 14), LayerId{"F.Cu"}},
                 {mm(14, 14), LayerId{"F.Cu"}}};
  RoutingPlanResult plan;
  plan.legal_scaffold_complete = true; plan.paths = {path};
  plan.coarse_global.stop_reason = GlobalRouterStopReason::complete;
  DetailedRouterConfig config;
  config.circular_arc_refinement = true;
  config.spline_minimum_bend_radius = Nanometres::from_mm(100.0);
  config.spline_control_corridor = Nanometres::from_mm(0.1);
  config.retry_failed_spline_routes_after_commits = false;
  config.max_residual_bend_repair_runs = 1U;
  config.max_residual_bend_repair_corners = 1U;
  config.max_residual_bend_repair_batches = 2U;
  const auto limited = materialize_detailed_route(source, plan, config);
  require(!limited.complete() && limited.candidate && limited.copper_drc.ok() &&
              limited.residual_bend_repair_accepted == 2U &&
              limited.residual_bend_repair_batches == 2U &&
              limited.residual_bend_repair_budget_exhausted &&
              limited.unrounded_non_junction_bends == 1U,
          "bounded residual cursor must spend two batches on two real bends, not collinear points");
  config.max_residual_bend_repair_batches = 3U;
  const auto complete = materialize_detailed_route(source, plan, config);
  require(complete.complete() && complete.candidate && complete.copper_drc.ok() &&
              complete.residual_bend_repair_accepted == 3U &&
              complete.residual_bend_repair_batches == 3U &&
              !complete.residual_bend_repair_budget_exhausted &&
              complete.unrounded_non_junction_bends == 0U,
          "residual cursor must resume within a chain and reach the last bend");
}
void residual_bend_batches_resume_across_chains() {
  const auto source = board();
  RoutingPlanResult plan;
  plan.legal_scaffold_complete = true;
  plan.coarse_global.stop_reason = GlobalRouterStopReason::complete;
  for (std::size_t index = 0; index < 3U; ++index) {
    GlobalRoutePath path;
    path.net = NetId{"scaffold"}; path.legal_scaffold = true; path.connection_index = index;
    const auto y = 2.0 + 8.0 * static_cast<double>(index);
    path.points = {{mm(2, y), LayerId{"F.Cu"}}, {mm(8, y), LayerId{"F.Cu"}},
                   {mm(8, y + 4.0), LayerId{"F.Cu"}}};
    plan.paths.push_back(path);
  }
  DetailedRouterConfig config;
  config.circular_arc_refinement = true;
  config.spline_minimum_bend_radius = Nanometres::from_mm(100.0);
  config.spline_control_corridor = Nanometres::from_mm(0.1);
  config.retry_failed_spline_routes_after_commits = false;
  config.max_residual_bend_repair_runs = 1U;
  config.max_residual_bend_repair_batches = 3U;
  const auto detailed = materialize_detailed_route(source, plan, config);
  require(detailed.complete() && detailed.copper_drc.ok() &&
              detailed.residual_bend_repair_accepted == 3U &&
              detailed.residual_bend_repair_batches == 3U &&
              detailed.residual_bend_repair_scanned_runs == 3U &&
              detailed.unrounded_non_junction_bends == 0U,
          "residual run batches must advance to later chains without restarting a prefix");
}
void residual_bend_repair_rejects_blocked_corner_without_mutating_copper() {
  auto source = board();
  source.nets.push_back({NetId{"blocker"}, "blocker", 2U});
  source.tracks.push_back({ObjectId{"blocked-corner"}, NetId{"blocker"}, LayerId{"F.Cu"},
                           mm(2.35, 9.65), mm(3.5, 9.65), Nanometres::from_mm(0.05), true});
  GlobalRoutePath path;
  path.net = NetId{"scaffold"}; path.legal_scaffold = true;
  path.points = {{mm(2, 2), LayerId{"F.Cu"}}, {mm(2, 10), LayerId{"F.Cu"}},
                 {mm(12, 10), LayerId{"F.Cu"}}};
  RoutingPlanResult plan;
  plan.legal_scaffold_complete = true; plan.paths = {path};
  plan.coarse_global.stop_reason = GlobalRouterStopReason::complete;
  DetailedRouterConfig config;
  config.circular_arc_refinement = true;
  // The full-radius fillet is blocked; a sufficiently small backtracked radius would be legal.
  config.circular_arc_backtracking_steps = 0U;
  config.spline_minimum_bend_radius = Nanometres::from_mm(100.0);
  config.spline_control_corridor = Nanometres::from_mm(0.1);
  config.retry_failed_spline_routes_after_commits = false;
  const auto detailed = materialize_detailed_route(source, plan, config);
  require(!detailed.complete() && detailed.candidate && detailed.copper_drc.ok() &&
              detailed.unrounded_non_junction_bends == 1U &&
              detailed.residual_bend_repair_accepted == 0U &&
              detailed.residual_bend_repair_rejected == 1U &&
              detailed.candidate->track_arcs.empty(),
          "a blocked residual bend must remain unrounded instead of committing unsafe copper");
  require(std::any_of(detailed.candidate->tracks.begin(), detailed.candidate->tracks.end(),
                      [](const auto& track) { return track.id == ObjectId{"blocked-corner"}; }),
          "residual repair must preserve locked obstacle copper");
}
void residual_bend_repair_preserves_existing_routing() {
  auto source = board();
  source.tracks = {
      {ObjectId{"source-first"}, NetId{"scaffold"}, LayerId{"F.Cu"}, mm(2, 2), mm(2, 10),
       Nanometres::from_mm(0.2), false},
      {ObjectId{"source-second"}, NetId{"scaffold"}, LayerId{"F.Cu"}, mm(2, 10), mm(12, 10),
       Nanometres::from_mm(0.2), false}};
  source.nets.push_back({NetId{"new-route"}, "new-route", 2U});
  GlobalRoutePath path;
  path.net = NetId{"new-route"}; path.legal_scaffold = true;
  path.points = {{mm(2,25), LayerId{"F.Cu"}}, {mm(12,25), LayerId{"F.Cu"}}};
  RoutingPlanResult plan;
  plan.legal_scaffold_complete = true; plan.paths = {path};
  plan.coarse_global.stop_reason = GlobalRouterStopReason::complete;
  DetailedRouterConfig config;
  config.circular_arc_refinement = true;
  config.existing_routing = ExistingRoutingPolicy::preserve;
  const auto detailed = materialize_detailed_route(source, plan, config);
  require(detailed.candidate && std::all_of(source.tracks.begin(), source.tracks.end(), [&](const auto& original) {
                return std::find(detailed.candidate->tracks.begin(), detailed.candidate->tracks.end(), original) !=
                       detailed.candidate->tracks.end();
              }) &&
              detailed.candidate->track_arcs.empty() && detailed.residual_bend_repair_attempted == 0U,
          "residual repair must not modify preserved source routing");
}
void residual_bend_repair_handles_pad_anchored_closed_chain() {
  auto source = board();
  Footprint footprint;
  footprint.id = ObjectId{"pad-anchor"};
  footprint.reference = "J1";
  footprint.position = mm(2, 2);
  footprint.courtyard = Polygon{{mm(-0.5,-0.5), mm(0.5,-0.5), mm(0.5,0.5), mm(-0.5,0.5)}};
  Pad pad;
  pad.id = ObjectId{"pad-anchor-1"};
  pad.net = NetId{"scaffold"};
  pad.layers = {LayerId{"F.Cu"}};
  pad.width = Nanometres::from_mm(1.0);
  pad.height = Nanometres::from_mm(1.0);
  footprint.pads = {pad};
  source.footprints.push_back(std::move(footprint));
  GlobalRoutePath path;
  path.net = NetId{"scaffold"}; path.legal_scaffold = true;
  path.points = {{mm(2, 2), LayerId{"F.Cu"}}, {mm(2, 8), LayerId{"F.Cu"}},
                 {mm(8, 8), LayerId{"F.Cu"}}, {mm(2, 2), LayerId{"F.Cu"}}};
  RoutingPlanResult plan;
  plan.legal_scaffold_complete = true; plan.paths = {path};
  plan.coarse_global.stop_reason = GlobalRouterStopReason::complete;
  DetailedRouterConfig config;
  config.circular_arc_refinement = true;
  config.spline_minimum_bend_radius = Nanometres::from_mm(100.0);
  config.spline_control_corridor = Nanometres::from_mm(0.1);
  config.retry_failed_spline_routes_after_commits = false;
  const auto detailed = materialize_detailed_route(source, plan, config);
  require(detailed.complete() && detailed.candidate && detailed.copper_drc.ok() &&
              detailed.residual_bend_repair_accepted > 0U &&
              detailed.unrounded_non_junction_bends == 0U,
          "a pad-anchored closed chain must repair interior bends while retaining its seam junction");
}
void residual_bend_repair_skips_floating_closed_chain() {
  auto source = board();
  GlobalRoutePath path;
  path.net = NetId{"scaffold"}; path.legal_scaffold = true;
  path.points = {{mm(2, 2), LayerId{"F.Cu"}}, {mm(2, 8), LayerId{"F.Cu"}},
                 {mm(8, 8), LayerId{"F.Cu"}}, {mm(2, 2), LayerId{"F.Cu"}}};
  RoutingPlanResult plan;
  plan.legal_scaffold_complete = true; plan.paths = {path};
  plan.coarse_global.stop_reason = GlobalRouterStopReason::complete;
  DetailedRouterConfig config;
  config.circular_arc_refinement = true;
  config.spline_minimum_bend_radius = Nanometres::from_mm(100.0);
  config.spline_control_corridor = Nanometres::from_mm(0.1);
  config.retry_failed_spline_routes_after_commits = false;
  const auto detailed = materialize_detailed_route(source, plan, config);
  require(detailed.candidate && detailed.residual_bend_repair_attempted == 0U &&
              detailed.candidate->track_arcs.empty() && detailed.unrounded_non_junction_bends > 0U,
          "a floating closed chain must keep its ambiguous audited seam untouched");
}
void generated_tail_cleanup_prunes_pad_rooted_interior_spur() {
  auto source = board();
  add_pad(source, "left", mm(2, 8));
  add_pad(source, "right", mm(12, 8));
  GlobalRoutePath trunk;
  trunk.net = NetId{"scaffold"}; trunk.legal_scaffold = true;
  trunk.points = {{mm(2, 8), LayerId{"F.Cu"}}, {mm(7, 8), LayerId{"F.Cu"}},
                  {mm(12, 8), LayerId{"F.Cu"}}};
  GlobalRoutePath spur;
  spur.net = NetId{"scaffold"}; spur.legal_scaffold = true;
  spur.points = {{mm(7, 8), LayerId{"F.Cu"}}, {mm(7, 12), LayerId{"F.Cu"}}};
  RoutingPlanResult plan;
  plan.legal_scaffold_complete = true; plan.paths = {trunk, spur};
  plan.coarse_global.stop_reason = GlobalRouterStopReason::complete;
  DetailedRouterConfig config;
  config.circular_arc_refinement = false;
  config.residual_bend_repair = false;
  config.enable_generated_tail_cleanup = false;
  const auto before = materialize_detailed_route(source, plan, config);
  require(before.complete() && before.candidate && before.candidate->tracks.size() == 3U,
          "disabled cleanup must retain generated T spur");
  config.enable_generated_tail_cleanup = true;
  const auto after = materialize_detailed_route(source, plan, config);
  require(after.complete() && after.candidate && after.candidate->tracks.size() == 2U &&
              after.generated_tail_cleanup_removed_chains == 1U &&
              after.generated_tail_cleanup_removed_tracks == 1U,
          "cleanup must remove only the pad-rooted bare interior spur");
  require(analyze_connectivity(*after.candidate).open_connections == 0U,
          "cleanup must retain required pad connectivity");
}
void generated_tail_cleanup_preserves_padless_component() {
  const auto source = board();
  GlobalRoutePath path;
  path.net = NetId{"scaffold"}; path.legal_scaffold = true;
  path.points = {{mm(3, 3), LayerId{"F.Cu"}}, {mm(8, 3), LayerId{"F.Cu"}}};
  RoutingPlanResult plan;
  plan.legal_scaffold_complete = true; plan.paths = {path};
  plan.coarse_global.stop_reason = GlobalRouterStopReason::complete;
  DetailedRouterConfig config;
  config.circular_arc_refinement = false;
  config.residual_bend_repair = false;
  const auto detailed = materialize_detailed_route(source, plan, config);
  require(detailed.complete() && detailed.candidate && detailed.candidate->tracks.size() == 1U &&
              detailed.generated_tail_cleanup_removed_tracks == 0U,
          "padless generated copper must fail closed rather than be pruned");
}

[[nodiscard]] Board tail_fixture() {
  auto source = board();
  Footprint footprint;
  footprint.placement_envelope_source = PlacementEnvelopeSource::absent;
  footprint.id = ObjectId{"tail-pads"}; footprint.reference = "J-tail";
  std::size_t index = 0U;
  for (const auto position : {mm(2, 10), mm(10, 10), mm(6, 6)}) {
    Pad pad;
    pad.id = ObjectId{"tail-pad-" + std::to_string(index++)};
    pad.net = NetId{"scaffold"}; pad.layers = {LayerId{"F.Cu"}};
    pad.local_position = position;
    pad.width = Nanometres::from_mm(1); pad.height = Nanometres::from_mm(1);
    footprint.pads.push_back(pad);
  }
  source.footprints.push_back(footprint);
  return source;
}
[[nodiscard]] RoutingPlanResult tail_plan(bool via = false) {
  GlobalRoutePath trunk;
  trunk.net = NetId{"scaffold"}; trunk.legal_scaffold = true;
  trunk.points = {{mm(2, 10), LayerId{"F.Cu"}}, {mm(10, 10), LayerId{"F.Cu"}}};
  auto branch = trunk; branch.connection_index = 1U;
  branch.points = {{mm(6, 6), LayerId{"F.Cu"}}, {mm(6, 14), LayerId{"F.Cu"}}};
  auto floating = trunk; floating.connection_index = 2U;
  floating.points = {{mm(6, 14), LayerId{"B.Cu"}}, {mm(8, 14), LayerId{"B.Cu"}}};
  if (via) {
    branch.points.push_back({mm(6, 14), LayerId{"B.Cu"}});
    branch.points.push_back({mm(8, 14), LayerId{"B.Cu"}});
  }
  RoutingPlanResult plan;
  plan.legal_scaffold_complete = true;
  plan.paths = via ? std::vector<GlobalRoutePath>{trunk, branch}
                   : std::vector<GlobalRoutePath>{trunk, branch, floating};
  plan.coarse_global.stop_reason = GlobalRouterStopReason::complete;
  return plan;
}
[[nodiscard]] bool has_tail_tip(const Board& candidate) {
  return std::any_of(candidate.tracks.begin(), candidate.tracks.end(), [](const auto& track) {
    return track.layer == LayerId{"F.Cu"} &&
           (track.start == mm(6, 14) || track.end == mm(6, 14));
  });
}
void generated_tail_cleanup_retains_required_interior_t_and_other_layer() {
  const auto source = tail_fixture();
  const auto plan = tail_plan();
  DetailedRouterConfig config;
  config.circular_arc_refinement = false;
  config.enable_generated_tail_cleanup = false;
  const auto before = materialize_detailed_route(source, plan, config);
  require(before.complete() && before.candidate && has_tail_tip(*before.candidate),
          "disabled cleanup must reproduce the bare generated tail");
  config.enable_generated_tail_cleanup = true;
  const auto after = materialize_detailed_route(source, plan, config);
  require(after.complete() && after.candidate && after.copper_drc.ok() &&
              analyze_connectivity(*after.candidate).open_connections == 0U &&
              !has_tail_tip(*after.candidate) && after.generated_tail_cleanup_removed_chains == 1U,
          "cleanup must split interior T contact and remove only its spare tail");
  require(std::any_of(after.candidate->tracks.begin(), after.candidate->tracks.end(), [](const auto& track) {
            return track.layer == LayerId{"B.Cu"} && track.start == mm(6,14) && track.end == mm(8,14);
          }), "coincident opposite-layer padless copper must remain untouched");
}
void generated_tail_cleanup_preserves_via_attachment_and_budget() {
  const auto source = tail_fixture();
  DetailedRouterConfig config;
  config.circular_arc_refinement = false;
  const auto result = materialize_detailed_route(source, tail_plan(true), config);
  require(result.complete() && result.candidate && !result.candidate->vias.empty() &&
              copper_probes_share_component(*result.candidate, NetId{"scaffold"},
                  {mm(2,10), {LayerId{"F.Cu"}}}, {mm(6,14), {LayerId{"B.Cu"}}}),
          "cleanup must preserve the entire pad-to-via attachment, not merely the via-contact edge");
  config.enable_generated_tail_cleanup = false;
  const auto disabled = materialize_detailed_route(source, tail_plan(), config);
  config.enable_generated_tail_cleanup = true;
  config.max_generated_tail_cleanup_candidates = 0U;
  const auto bounded = materialize_detailed_route(source, tail_plan(), config);
  require(bounded.complete() && bounded.candidate && has_tail_tip(*bounded.candidate) &&
              bounded.generated_tail_cleanup_removed_tracks == 0U && disabled.candidate &&
              bounded.candidate->tracks == disabled.candidate->tracks,
          "zero cleanup allowance must leave generated topology intact");
  config.max_generated_tail_cleanup_candidates = 256U;
  config.max_generated_tail_cleanup_split_tracks = 3U;
  const auto split_bounded = materialize_detailed_route(source, tail_plan(), config);
  require(split_bounded.complete() && split_bounded.candidate && disabled.candidate &&
              split_bounded.generated_tail_cleanup_budget_exhausted &&
              split_bounded.generated_tail_cleanup_removed_tracks == 0U &&
              split_bounded.candidate->tracks == disabled.candidate->tracks,
          "exhausted split-output budget must retain original tracks atomically");
}
void generated_tail_cleanup_preserves_finite_width_via_contact() {
  auto source = tail_fixture();
  source.footprints.front().pads.resize(1U);
  auto plan = tail_plan();
  plan.paths.resize(2U);
  plan.paths[0].points = {{mm(2,10), LayerId{"F.Cu"}}, {mm(8,10), LayerId{"F.Cu"}}};
  // Copper overlaps by 0.05mm, but the two centerlines never intersect.  Pad-only
  // connectivity would miss the orphaned via if the apparent bare horizontal tail vanished.
  plan.paths[1].points = {{mm(6,10.15), LayerId{"F.Cu"}}, {mm(6,14), LayerId{"F.Cu"}},
                         {mm(6,14), LayerId{"B.Cu"}}};
  DetailedRouterConfig config;
  config.circular_arc_refinement = false;
  const auto result = materialize_detailed_route(source, plan, config);
  require(result.complete() && result.candidate && result.copper_drc.ok() &&
              copper_probes_share_component(*result.candidate, NetId{"scaffold"},
                  {mm(2,10), {LayerId{"F.Cu"}}}, {mm(6,14), {LayerId{"B.Cu"}}}),
          ("finite-width interior contact must retain pad-to-via component membership; complete=" +
           std::to_string(result.complete()) + "; candidate=" + std::to_string(result.candidate.has_value()) +
           "; drc=" + std::to_string(result.copper_drc.ok()) +
           "; removed=" + std::to_string(result.generated_tail_cleanup_removed_tracks) +
           "; diagnostic=" + result.diagnostic).c_str());
}
void generated_tail_cleanup_preserves_source_curve_net() {
  auto source = tail_fixture();
  source.track_arcs.push_back({ObjectId{"source-arc"}, NetId{"scaffold"}, LayerId{"F.Cu"},
                              mm(20,20), mm(21,21), mm(20,22), Nanometres::from_mm(0.2), false});
  DetailedRouterConfig config;
  config.circular_arc_refinement = false;
  config.existing_routing = ExistingRoutingPolicy::preserve;
  const auto result = materialize_detailed_route(source, tail_plan(), config);
  require(result.complete() && result.candidate && has_tail_tip(*result.candidate) &&
              result.generated_tail_cleanup_removed_tracks == 0U &&
              std::find(result.candidate->track_arcs.begin(), result.candidate->track_arcs.end(),
                        source.track_arcs.front()) != result.candidate->track_arcs.end(),
          "source curved copper must exclude ambiguous cleanup on its net");
}
void spline_navigation_cache_preserves_parallel_copper() {
  const auto source = board();
  RoutingPlanResult plan;
  plan.legal_scaffold_complete = true;
  plan.coarse_global.stop_reason = GlobalRouterStopReason::complete;
  for (std::size_t index = 0; index < 3U; ++index) {
    GlobalRoutePath path;
    path.net = NetId{"scaffold"}; path.legal_scaffold = true;
    path.connection_index = index;
    const LayerId layer{index == 2U ? "B.Cu" : "F.Cu"};
    const double x = index == 1U ? 12.0 : 2.0;
    path.points = {{mm(x, 2), layer}, {mm(x, 6), layer}, {mm(x + 4, 6), layer}};
    plan.paths.push_back(std::move(path));
  }
  DetailedRouterConfig config;
  config.circular_arc_refinement = true;
  config.enable_generated_tail_cleanup = false;
  config.prefer_gpu_spline_trajectory = false;
  config.max_spline_trajectory_iterations = 1U;
  config.max_spline_trajectory_candidate_checks = 16U;
  config.retry_failed_spline_routes_after_commits = false;
  config.spline_route_workers = 1U;
  config.reuse_spline_navigation_preparation = false;
  config.gpu_materialization_batches = false;
  const auto uncached = materialize_detailed_route(source, plan, config);
  config.reuse_spline_navigation_preparation = true;
  config.gpu_materialization_batches = true;
  config.spline_route_workers = 3U;
  const auto cached = materialize_detailed_route(source, plan, config);
  require(uncached.candidate && cached.candidate && uncached.copper_drc.ok() && cached.copper_drc.ok(),
          "cached and uncached curve realization must both retain legal copper");
  require(uncached.stop_reason == cached.stop_reason &&
              uncached.candidate->tracks == cached.candidate->tracks &&
              uncached.candidate->track_arcs == cached.candidate->track_arcs &&
              uncached.candidate->track_beziers == cached.candidate->track_beziers &&
              uncached.candidate->vias == cached.candidate->vias,
          "snapshot preparation reuse and parallel workers must preserve exact output");
  require(cached.spline_navigation_preparation_hits == 1U &&
              cached.spline_navigation_preparation_builds == 2U &&
              uncached.spline_navigation_preparation_builds == 3U &&
              uncached.spline_navigation_preparation_hits == 0U,
          "same-net same-layer runs must share preparation without mixing copper layers");
  auto distinct_source = source;
  auto distinct_plan = plan;
  for (std::size_t index = 1U; index < distinct_plan.paths.size(); ++index) {
    const auto name = "distinct-" + std::to_string(index);
    distinct_source.nets.push_back({NetId{name}, name, 1U});
    distinct_plan.paths[index].net = NetId{name};
  }
  const auto unique = materialize_detailed_route(distinct_source, distinct_plan, config);
  require(unique.candidate && unique.copper_drc.ok() &&
              unique.spline_navigation_preparation_hits == 0U &&
              unique.spline_navigation_preparation_builds == 3U,
          "single-consumer keys must not retain cached geometry or share across nets");
}
void navmesh_screening_preparation_cache_is_exact_and_bounded() {
  OffsetFreeSpaceRegion region;
  region.area = Polygon{{mm(0, 0), mm(12, 0), mm(12, 12), mm(0, 12)}};
  NavMeshConfig first;
  first.cell_pitch = Nanometres::from_mm(1.0);
  first.maximum_cells = 1'000U;
  first.topology_guide = {mm(1, 1), mm(11, 1)};
  first.maximum_guide_distance = Nanometres::from_mm(2.0);
  NavMeshScreeningContext context(20'000U, 1U, 500U);
  const auto initial = context.prepare(region, first);
  const auto repeated = context.prepare(region, first);
  const auto after_repeat = context.stats();
  require(initial && initial == repeated && after_repeat.prepared_domain_misses == 1U &&
              after_repeat.prepared_domain_hits == 1U && after_repeat.geometry_misses == 1U,
          "identical preparation must reuse the exact immutable lattice and its geometry base");
  auto guided = first;
  guided.topology_guide = {mm(1, 1), mm(11, 11)};
  const auto changed_guide = context.prepare(region, guided);
  const auto after_guide = context.stats();
  require(changed_guide && changed_guide != initial && after_guide.prepared_domain_misses == 2U &&
              after_guide.geometry_hits >= 1U && after_guide.prepared_domain_evictions == 1U &&
              after_guide.retained_prepared_cells <= 500U,
          "guide changes must rebuild the lattice while reusing bounded static geometry");
  auto changed_region = region;
  changed_region.holes.push_back(Polygon{{mm(5, 5), mm(7, 5), mm(7, 7), mm(5, 7)}});
  const auto changed = context.prepare(changed_region, first);
  require(changed && changed != changed_guide && !query_prepared_navmesh_point(*changed, mm(6, 6)).inside &&
              query_prepared_navmesh_point(*initial, mm(6, 6)).inside,
          "region identity must invalidate both prepared lattice and static geometry caches");
}

} // namespace
int main() { final_drc_epsilon_preserves_strict_materialization_by_default(); navmesh_screening_preparation_cache_is_exact_and_bounded(); spline_navigation_cache_preserves_parallel_copper(); generated_tail_cleanup_preserves_finite_width_via_contact(); generated_tail_cleanup_retains_required_interior_t_and_other_layer(); generated_tail_cleanup_preserves_via_attachment_and_budget(); generated_tail_cleanup_preserves_source_curve_net(); materializes_complete_scaffold_without_regional_plan(); explicit_nominal_widths_match_implicit_certified_scaffold(); malformed_certified_width_metadata_is_rejected(); unbacked_neck_down_metadata_is_rejected(); invalid_certified_layer_transition_is_rejected(); failed_curve_fit_retains_scaffold_and_unrelated_copper(); residual_bend_repair_uses_exact_fillet_after_ph_rejection(); residual_bend_batches_resume_without_charging_collinear_points(); residual_bend_batches_resume_across_chains(); residual_bend_repair_rejects_blocked_corner_without_mutating_copper(); residual_bend_repair_preserves_existing_routing(); residual_bend_repair_handles_pad_anchored_closed_chain(); generated_tail_cleanup_prunes_pad_rooted_interior_spur(); generated_tail_cleanup_preserves_padless_component(); return EXIT_SUCCESS; }
