#include "flux/core/detailed_router.hpp"
#include "flux/core/geometry.hpp"

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
} // namespace
int main() { materializes_complete_scaffold_without_regional_plan(); explicit_nominal_widths_match_implicit_certified_scaffold(); malformed_certified_width_metadata_is_rejected(); unbacked_neck_down_metadata_is_rejected(); invalid_certified_layer_transition_is_rejected(); failed_curve_fit_retains_scaffold_and_unrelated_copper(); residual_bend_repair_uses_exact_fillet_after_ph_rejection(); residual_bend_repair_rejects_blocked_corner_without_mutating_copper(); residual_bend_repair_preserves_existing_routing(); residual_bend_repair_handles_pad_anchored_closed_chain(); residual_bend_repair_skips_floating_closed_chain(); return EXIT_SUCCESS; }
