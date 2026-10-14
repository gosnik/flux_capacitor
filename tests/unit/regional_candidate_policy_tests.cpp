#include "flux/core/regional_candidate_policy.hpp"

#include <array>
#include <cstdlib>
#include <limits>
#include <iostream>

namespace {
using namespace flux;
[[noreturn]] void fail(const char* message) { std::cerr << message << '\n'; std::exit(1); }
void require(const bool value, const char* message) { if (!value) fail(message); }
Point nm(const std::int64_t x, const std::int64_t y) { return {Nanometres{x}, Nanometres{y}}; }

struct Fixture {
  Board board;
  RegionalRouteResult regional;
  RegionalBundleAssignment assignment;
  LayerId layer{"F.Cu"};
  RegionalRouteFragmentId first{NetId{"a"}, 0U, 0U, 0U};
  RegionalRouteFragmentId second{NetId{"b"}, 0U, 0U, 0U};
  Fixture() {
    board.layers = {{layer, "F.Cu", LayerKind::copper, 0U}};
    assignment.window_id = 1U;
    assignment.templates = {{0U, first, layer}, {1U, second, layer}};
    assignment.selected_templates = {0U, 1U};
  }
  void fragments(Point a, Point b, bool fixed = true) {
    RegionalRouteFragment left{first, layer, a, nm(1000, 0), Nanometres{100}, Nanometres{50}};
    RegionalRouteFragment right{second, layer, b, nm(1000, 1000), Nanometres{100}, Nanometres{50}};
    left.begins_at_terminal = right.begins_at_terminal = fixed;
    regional.fragments = {left, right};
  }
};
} // namespace

int main() {
  require(regional_assignment_is_no_good_certificate(
              RegionalBundleStopReason::candidate_preflight_infeasible),
          "a monotonic regional preflight certificate must authorize a hard no-good");
  require(!regional_assignment_is_no_good_certificate(
              RegionalBundleStopReason::spline_realization_failed),
          "a failed PH/native realization must remain candidate-specific evidence");
  require(!regional_assignment_is_no_good_certificate(
              RegionalBundleStopReason::state_budget_exhausted),
          "regional search-budget exhaustion must remain unknown");
  require(!regional_assignment_is_no_good_certificate(
              RegionalBundleStopReason::spline_realization_failed, true),
          "a shape-sample budget exhaustion must never become a hard no-good");
  { Fixture fixture;
    fixture.assignment.first_realization_rejection = RegionalRealizationRejection{
        RegionalRealizationRejectionStage::topology_seed_failed, fixture.assignment.window_id,
        std::size_t{0U}, fixture.first, fixture.layer, nm(10, 20), true,
        {{"clearance", {"generated-route", "fixed-track"}, Nanometres{100}, 25.0L}}};
    const RegionalSplineCandidateSummary summary{
        0U, {}, {}, {}, {}, 0U, 0U, 0U, 0U, 0U,
        fixture.assignment.first_realization_rejection, false, false};
    require(summary.first_realization_rejection &&
                summary.first_realization_rejection->stage ==
                    RegionalRealizationRejectionStage::topology_seed_failed &&
                summary.first_realization_rejection->fragment == fixture.first &&
                summary.first_realization_rejection->geometry == nm(10, 20) &&
                summary.first_realization_rejection->native_violations.size() == 1U &&
                summary.first_realization_rejection->native_violations.front().code == "clearance",
            "candidate summaries must retain typed realization and native-copper evidence"); }
  { Fixture fixture; fixture.fragments(nm(0, 0), nm(100, 0));
    const auto preflight = preflight_regional_candidate(fixture.board, fixture.regional, fixture.assignment);
    require(preflight.definitively_infeasible() &&
                regional_assignment_is_no_good_certificate(
                    RegionalBundleStopReason::candidate_preflight_infeasible),
            "overlapping fixed terminal envelopes must produce a no-good certificate"); }
  { Fixture fixture; fixture.fragments(nm(0, 0), nm(0, 0));
    fixture.regional.fragments[1].id.net = fixture.first.net;
    fixture.regional.fragments[1].id.run_index = 1U;
    fixture.assignment.templates[1].fragment = fixture.regional.fragments[1].id;
    const auto preflight = preflight_regional_candidate(fixture.board, fixture.regional, fixture.assignment);
    require(!preflight.definitively_infeasible(), "same-net branches must remain allowed"); }
  { Fixture fixture; fixture.fragments(nm(0, 0), nm(150, 0));
    const auto preflight = preflight_regional_candidate(fixture.board, fixture.regional, fixture.assignment);
    require(!preflight.definitively_infeasible(), "clearance-envelope tangency must remain feasible"); }
  { Fixture fixture; fixture.fragments(nm(0, 0), nm(0, 0), false);
    RegionalSplineRealization left; left.fragment = fixture.first; left.layer = fixture.layer;
    left.centerline = {nm(0, 0), nm(10, 10)};
    RegionalSplineRealization right; right.fragment = fixture.second; right.layer = fixture.layer;
    right.centerline = {nm(0, 10), nm(10, 0)};
    fixture.assignment.spline_realizations = {left, right};
    const auto preflight = preflight_regional_candidate(fixture.board, fixture.regional, fixture.assignment);
    require(!preflight.definitively_infeasible(), "mutable centreline crossings must not reject"); }
  { Fixture fixture; fixture.fragments(nm(0, 0), nm(1000, 0));
    RegionalCrossSection section; section.id = 7U; section.layer = fixture.layer; section.usable_width = Nanometres{250};
    section.channel_id = 4U; section.ordered_occupants = {fixture.first, fixture.second};
    fixture.assignment.packed_cross_sections = {section};
    const auto preflight = preflight_regional_candidate(fixture.board, fixture.regional, fixture.assignment);
    require(preflight.definitively_infeasible(), "cross-section whose best packing cannot fit must reject"); }
  { Fixture fixture; fixture.fragments(nm(0, 0), nm(1000, 0));
    auto first = fixture.assignment; first.selected_residual_conflicts = 1U;
    auto second = fixture.assignment; second.selected_residual_conflicts = 1U;
    const std::array assignments{first, second};
    const auto schedule = schedule_regional_candidates(fixture.board, fixture.regional, assignments);
    require(schedule.size() == 2U && schedule[0].assignment_index == 0U && schedule[1].assignment_index == 1U,
            "equal scheduling keys must retain input order"); }
  { Fixture fixture; fixture.fragments(nm(0, 0), nm(100, 0));
    fixture.assignment.templates[1].routing_layer = LayerId{"B.Cu"};
    require(!preflight_regional_candidate(fixture.board, fixture.regional, fixture.assignment).definitively_infeasible(),
            "changed-layer assignment must not be rejected using original layer"); }
  { Fixture fixture; fixture.fragments(nm(std::numeric_limits<std::int64_t>::min(), 0),
                                      nm(std::numeric_limits<std::int64_t>::max(), 0));
    require(!preflight_regional_candidate(fixture.board, fixture.regional, fixture.assignment).definitively_infeasible(),
            "extreme coordinate separation overflowed"); }
  { Fixture fixture; fixture.fragments(nm(0, 0), nm(1000, 0));
    auto first = fixture.assignment;
    auto second = fixture.assignment;
    second.templates[0].id = 100U;
    second.templates[1].id = 101U;
    const std::array assignments{first, second};
    RegionalCandidateSchedulingState history;
    record_regional_candidate_evaluation(history, regional_candidate_schedule_signature(first), {10U, 0U, false});
    const auto scheduled = schedule_regional_candidates(fixture.board, fixture.regional, assignments, &history);
    require(scheduled[0].assignment_index == 1U,
            "untried candidate family must precede a known conflicting family");
    // A good nearby family should precede a worse nearby family independent of original order.
    record_regional_candidate_evaluation(history, regional_candidate_schedule_signature(second), {1U, 1U, false});
    const auto improved = schedule_regional_candidates(fixture.board, fixture.regional, assignments, &history);
    require(improved[0].assignment_index == 1U, "physical candidate progress did not affect scheduling"); }
  return 0;
}
