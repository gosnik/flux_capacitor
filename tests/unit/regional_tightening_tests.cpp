#include "flux/core/regional_router.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {
using namespace flux;

[[noreturn]] void fail(const char* message) { std::cerr << message << '\n'; std::exit(1); }
void require(bool condition, const char* message) { if (!condition) fail(message); }
Point mm(double x, double y) { return {Nanometres::from_mm(x), Nanometres::from_mm(y)}; }
Polygon rectangle(double x0, double y0, double x1, double y1) {
  return {{mm(x0, y0), mm(x1, y0), mm(x1, y1), mm(x0, y1)}};
}
} // namespace

struct Fixture {
  Board board;
  GlobalRouteResult global;
  RegionalRouteResult regional;
};

Fixture fixture(bool protect_detour, bool shortcut_obstacle = false) {
  const LayerId layer{"F.Cu"};
  const NetId net{"signal"};
  Fixture result;
  result.board.outline = rectangle(0, 0, 100, 80);
  result.board.layers = {{layer, "F.Cu", LayerKind::copper, 0U}};
  result.board.nets = {{net, "SIGNAL", 0U}, {NetId{"blocker"}, "BLOCKER", 1U}};
  if (shortcut_obstacle) {
    result.board.tracks.push_back(
        {ObjectId{"blocker-track"}, NetId{"blocker"}, layer, mm(50, 35), mm(50, 45),
         Nanometres::from_mm(.25), false});
  }
  for (const auto& [id, position] : {std::pair{"U1", mm(20, 40)}, {"U2", mm(80, 40)}}) {
    Footprint footprint;
    footprint.id = ObjectId{id}; footprint.reference = id; footprint.position = position;
    footprint.courtyard = rectangle(-2, -2, 2, 2);
    footprint.pads = {{ObjectId{std::string(id) + "-pad"}, net, Point{}, {layer}}};
    result.board.footprints.push_back(std::move(footprint));
  }
  result.global.paths = {{net, 0U, {{mm(20, 40), layer}, {mm(80, 40), layer}}}};
  RegionalRouteFragment fragment{{net, 0U, 0U, 0U}, layer, mm(20, 40), mm(80, 40),
                                 Nanometres::from_mm(.25), Nanometres::from_mm(.2)};
  fragment.begins_at_terminal = fragment.ends_at_terminal = true;
  result.regional.fragments = {fragment};
  RegionalRoutingWindow window; window.id = 0U; window.fragments = {fragment.id};
  window.owned_fragments = {fragment.id}; result.regional.windows = {window};
  const auto ph = make_quintic_ph_route({mm(20, 40), mm(50, 60), mm(80, 40)});
  require(ph.has_value(), "PH fixture must build");
  RegionalSplineRealization realization;
  realization.fragment = fragment.id; realization.replacement_end = fragment.end; realization.layer = layer;
  realization.width = fragment.width; realization.ph_route = std::make_shared<const QuinticPHRoute>(*ph);
  realization.native_fit_policy = {};
  realization.native_pieces = fit_quintic_ph_to_cubic_copper(*ph, realization.native_fit_policy);
  for (const auto& sample : adaptive_tessellate_quintic_ph(*ph, Nanometres::from_mm(.02), 10U, 512U))
    realization.centerline.push_back({Nanometres{static_cast<std::int64_t>(std::llround(sample.position.x))},
                                        Nanometres{static_cast<std::int64_t>(std::llround(sample.position.y))}});
  if (protect_detour) realization.protected_anchors = {mm(50, 60)};
  realization.native_copper_legal = realization.mutually_legal = realization.quantized_mutually_legal = true;
  RegionalBundleAssignment assignment; assignment.window_id = 0U;
  assignment.stop_reason = RegionalBundleStopReason::complete;
  assignment.spline_realizations = {std::move(realization)};
  result.regional.bundle_assignments = {std::move(assignment)};
  result.regional.spline_candidate_available = true;
  result.regional.native_scene_evaluated = result.regional.native_scene_complete = true;
  result.regional.validated_copper_scene.data = std::make_shared<const RegionalCopperSceneData>(
      RegionalCopperSceneData{result.board, result.board, result.global.paths, {}, false});
  return result;
}

int main() {
  auto accepted = fixture(false);
  require(tighten_regional_spline_candidate(accepted.board, accepted.global, accepted.regional,
                                            accepted.regional.configured),
          "a removable legal PH detour must tighten through native and connectivity gates");
  require(accepted.regional.spline_tightening_routes_accepted == 1U &&
              accepted.regional.spline_tightening_exact_length_after_nm <
                  accepted.regional.spline_tightening_exact_length_before_nm &&
              accepted.regional.validated_copper_scene.data &&
              validate_regional_native_ownership(accepted.regional),
          "accepted tightening must shorten exact PH length and publish coherent owned copper");
  auto vetoed = fixture(true);
  require(!tighten_regional_spline_candidate(vetoed.board, vetoed.global, vetoed.regional,
                                             vetoed.regional.configured) &&
              vetoed.regional.bundle_assignments.front().spline_realizations.front().ph_route->control_points.size() == 3U,
          "a protected junction anchor must veto tightening and retain the original realization");
  auto blocked = fixture(false, true);
  blocked.regional.configured.enable_paired_obstacle_supports = false;
  require(!tighten_regional_spline_candidate(blocked.board, blocked.global, blocked.regional,
                                             blocked.regional.configured) &&
              blocked.regional.bundle_assignments.front().spline_realizations.front().ph_route->control_points.size() == 3U,
          "full native copper assessment must veto a shortcut through foreign copper");
  auto paired = fixture(false, true);
  require(tighten_regional_spline_candidate(paired.board, paired.global, paired.regional,
                                           paired.regional.configured),
          "paired supports must shorten a legal bulge around real foreign copper");
  const auto& paired_route = paired.regional.bundle_assignments.front().spline_realizations.front();
  require(paired_route.ph_route->control_points.size() >= 4U &&
              paired.regional.spline_tightening_exact_length_after_nm <
                  paired.regional.spline_tightening_exact_length_before_nm &&
              paired.regional.validated_copper_scene.data &&
              validate_regional_native_ownership(paired.regional),
          "paired obstacle supports must survive coherent native reconstruction and validation");
  return 0;
}
