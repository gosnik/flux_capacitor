#include "flux/core/copper_scaffold.hpp"
#include "flux/core/drc.hpp"
#include "../../src/scaffold_joint_displacement.hpp"
#include "../../src/scaffold_connectivity_guard.hpp"
#include <limits>

#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <stop_token>
#include <utility>
#include <vector>

namespace {
using namespace flux;

[[noreturn]] void fail(const char* message) { throw std::runtime_error(message); }
void require(bool value, const char* message) { if (!value) fail(message); }
[[nodiscard]] Point mm(double x, double y) {
  return {Nanometres::from_mm(x), Nanometres::from_mm(y)};
}

[[nodiscard]] GlobalRoutePath demand(NetId net, Point start, Point end) {
  GlobalRoutePath path;
  path.net = std::move(net);
  path.points = {{start, LayerId{"F.Cu"}}, {end, LayerId{"F.Cu"}}};
  return path;
}

struct CoupledPassageFixture {
  Board board;
  TrackSegment locked_source;
  GlobalRoutePath lower;
  GlobalRoutePath upper;
  GlobalRoutePath incoming;

  CoupledPassageFixture()
      : locked_source{ObjectId{"joint-locked-source"}, NetId{"source"}, LayerId{"F.Cu"},
                      mm(3, 2.3), mm(5, 2.3), Nanometres::from_mm(0.2), true},
        lower(demand(NetId{"lower"}, mm(0.1, 0.9), mm(19.9, 0.9))),
        upper(demand(NetId{"upper"}, mm(0.1, 1.7), mm(19.9, 1.7))),
        incoming(demand(NetId{"incoming"}, mm(10, 0.6), mm(10, 2.0))) {
    board.outline = Polygon{{mm(0, 0), mm(20, 0), mm(20, 2.6), mm(0, 2.6)}};
    board.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0U},
                    {LayerId{"B.Cu"}, "B.Cu", LayerKind::copper, 1U}};
    board.nets = {{NetId{"lower"}, "lower", 1U}, {NetId{"upper"}, "upper", 2U},
                  {NetId{"incoming"}, "incoming", 3U}, {NetId{"source"}, "source", 4U}};
    board.rules.default_track_width = Nanometres::from_mm(0.2);
    board.rules.default_clearance = Nanometres::from_mm(0.2);
    // The via cannot fit on either side of either original owner.  The incoming connection is
    // therefore blocked until both generated owners change together; moving only one leaves the
    // other full-width barrier intact.
    board.rules.default_via_diameter = Nanometres::from_mm(2.0);
    board.rules.default_via_drill = Nanometres::from_mm(0.5);
    board.rules.copper_to_edge_clearance = Nanometres{0};
    board.tracks.push_back(locked_source);
  }

  [[nodiscard]] std::vector<GlobalRoutePath> requests() const {
    return {lower, upper, incoming};
  }
};

[[nodiscard]] CopperScaffoldConfig joint_config() {
  CopperScaffoldConfig config;
  config.grid_pitch = Nanometres::from_mm(0.1);
  config.maximum_cells = 20'000U;
  config.maximum_expansions_per_connection = 100'000U;
  // Isolate the new pre-exchange stage: no single-owner PNS or legacy causal/component repair
  // is allowed to make this fixture pass incidentally.
  config.maximum_victim_exchanges = 0U;
  config.maximum_displacement_attempts = 0U;
  config.maximum_coupled_victims = 0U;
  config.maximum_coupled_exchange_attempts = 0U;
  config.maximum_joint_passage_candidates = 4U;
  config.maximum_joint_blocker_owners = 3U;
  config.maximum_joint_search_expansions = 20'000U;
  config.maximum_joint_particles = 256U;
  config.maximum_joint_iterations = 32U;
  return config;
}

[[nodiscard]] bool contains_track(const Board& board, const TrackSegment& track) {
  return std::find(board.tracks.begin(), board.tracks.end(), track) != board.tracks.end();
}

[[nodiscard]] const GlobalRoutePath* path_for(const CopperScaffoldResult& result, const NetId& net) {
  const auto found = std::find_if(result.paths.begin(), result.paths.end(), [&](const auto& path) {
    return path.net == net;
  });
  return found == result.paths.end() ? nullptr : &*found;
}

void coupled_generated_blockers_require_the_joint_stage() {
  const CoupledPassageFixture scene;
  const auto source_snapshot = scene.board;
  auto disabled = joint_config();
  disabled.maximum_joint_passage_candidates = 0U;
  const auto without_joint = construct_copper_scaffold(scene.board, scene.requests(), disabled);
  require(!without_joint.complete && without_joint.counters.joint_group_attempts == 0U &&
              scene.board == source_snapshot && contains_track(without_joint.copper, scene.locked_source) &&
              check_copper_rules(without_joint.copper).ok(),
          "the blocked passage must remain incomplete when all joint recovery is disabled");

  const auto result = construct_copper_scaffold(scene.board, scene.requests(), joint_config());
  if (!result.complete) std::cerr << result.diagnostic << " candidates=" << result.counters.joint_passage_candidates
      << " attempts=" << result.counters.joint_group_attempts << " xpbd=" << result.counters.joint_xpbd_invocations
      << " avbd=" << result.counters.joint_avbd_invocations << " pns=" << result.counters.joint_pns_attempts
      << " rejected=" << result.counters.joint_exact_rejections << std::endl;
  require(result.complete && result.satisfied_indices.size() == scene.requests().size() &&
              result.counters.joint_passage_candidates > 0U && result.counters.joint_group_attempts > 0U &&
              result.counters.joint_group_accepted == 1U && result.counters.joint_xpbd_invocations > 0U &&
              result.counters.joint_avbd_invocations > 0U,
          "the two generated full-width blockers must be admitted as one successful joint passage group");
  const auto* rebuilt_lower = path_for(result, NetId{"lower"});
  const auto* rebuilt_upper = path_for(result, NetId{"upper"});
  require(rebuilt_lower && rebuilt_upper &&
              std::any_of(rebuilt_lower->points.begin(), rebuilt_lower->points.end(), [&](const auto& p) {
                return p.position.y != scene.lower.points.front().position.y; }) &&
              std::any_of(rebuilt_upper->points.begin(), rebuilt_upper->points.end(), [&](const auto& p) {
                return p.position.y != scene.upper.points.front().position.y; }),
          "both blocker owners must change; a one-owner displacement cannot solve this passage");
  require(scene.board == source_snapshot && contains_track(result.copper, scene.locked_source) &&
              check_copper_rules(result.copper).ok(),
          "joint commit must preserve immutable source copper and publish only exact DRC-clean copper");

  const auto replayed = construct_copper_scaffold(scene.board, scene.requests(), joint_config(), {}, &result);
  require(replayed.complete && replayed.paths == result.paths && replayed.copper == result.copper &&
              check_copper_rules(replayed.copper).ok(),
          "the atomic joint result must be retainable and replay exactly without mutating the source board");
}

void joint_caps_and_failed_transactions_publish_only_the_prior_legal_subset() {
  const CoupledPassageFixture scene;
  const auto source_snapshot = scene.board;
  auto owner_cap = joint_config();
  owner_cap.maximum_joint_blocker_owners = 1U;
  const auto capped = construct_copper_scaffold(scene.board, scene.requests(), owner_cap);
  require(!capped.complete && scene.board == source_snapshot && contains_track(capped.copper, scene.locked_source) &&
              check_copper_rules(capped.copper).ok(),
          "an insufficient joint-owner cap must fail closed without moving locked or source copper");

  auto exhausted = joint_config();
  exhausted.maximum_joint_particles = 1U;
  const auto failed = construct_copper_scaffold(scene.board, scene.requests(), exhausted);
  require(!failed.complete && failed.counters.joint_budget_exhausted > 0U && scene.board == source_snapshot &&
              contains_track(failed.copper, scene.locked_source) && check_copper_rules(failed.copper).ok(),
          "a bounded failed joint transaction must leave only its prior exact legal subset");
}

void cancellation_before_joint_commit_keeps_the_published_owners_immutable() {
  const CoupledPassageFixture scene;
  const auto source_snapshot = scene.board;
  std::stop_source stop;
  CopperScaffoldResult checkpoint;
  const auto result = construct_copper_scaffold(
      scene.board, scene.requests(), joint_config(), stop.get_token(), nullptr,
      [&](const CopperScaffoldResult& published) {
        checkpoint = published;
        if (published.paths.size() == 2U) stop.request_stop();
      });
  require(!result.complete && result.counters.cancelled && result.paths == checkpoint.paths &&
              result.copper == checkpoint.copper && scene.board == source_snapshot &&
              contains_track(result.copper, scene.locked_source) && check_copper_rules(result.copper).ok(),
          "cancellation before a joint commit must retain the last exact legal checkpoint unchanged");
}
void aggregate_counts_cannot_hide_a_split_pad_group() {
  const NetId net{"branch"};
  ConnectivityReport before;
  before.pad_components = {{ObjectId{"a"}, net, 0U}, {ObjectId{"b"}, net, 0U},
                           {ObjectId{"c"}, net, 1U}, {ObjectId{"d"}, net, 1U}};
  auto swapped = before;
  swapped.pad_components[1].component = 1U;
  swapped.pad_components[2].component = 0U;
  require(!preserves_existing_pad_components(before, swapped),
          "unchanged group counts must not conceal a split and membership swap");
  auto merged = before;
  for (auto& pad : merged.pad_components) pad.component = 7U;
  require(preserves_existing_pad_components(before, merged),
          "a new same-net connection may merge groups without losing old connectivity");
}

void dense_obstacles_exhaust_work_before_relaxation() {
  CoupledPassageFixture scene;
  for (std::size_t i = 0; i < 100U; ++i) {
    const auto x = 0.2 + 0.19 * static_cast<double>(i);
    scene.board.tracks.push_back({ObjectId{"dense-" + std::to_string(i)}, NetId{"source"}, LayerId{"F.Cu"},
                                 mm(x, 1.1), mm(x, 1.4), Nanometres::from_mm(0.05), true});
  }
  ScaffoldJointDisplacementConfig cap;
  cap.maximum_contacts = 256U;
  const auto exhausted = solve_scaffold_joint_displacement(
      scene.board, {scene.incoming, scene.lower, scene.upper}, cap);
  require(exhausted.budget_exhausted && !exhausted.paths && exhausted.xpbd_iterations == 0U,
          "dense immutable obstacles must consume a work budget before iterative relaxation");
}

void adapter_bounds_and_refinement_fallback_are_explicit() {
  const CoupledPassageFixture scene;
  const std::vector<GlobalRoutePath> paths{scene.incoming, scene.lower, scene.upper};
  ScaffoldJointDisplacementConfig overflow;
  overflow.maximum_contacts = std::numeric_limits<std::size_t>::max();
  const auto rejected = solve_scaffold_joint_displacement(scene.board, paths, overflow);
  require(!rejected.paths && !rejected.ok(), "overflowing contact budgets must reject before allocation");
  ScaffoldJointDisplacementConfig fallback;
  fallback.avbd.iterations = 0U;
  const auto proposal = solve_scaffold_joint_displacement(scene.board, paths, fallback);
  require(proposal.ok() && proposal.paths && !proposal.refinement_diagnostic.empty(),
          "AVBD refinement failure must preserve a nonfatal bounded XPBD proposal");
}
} // namespace

int main() {
  aggregate_counts_cannot_hide_a_split_pad_group();
  dense_obstacles_exhaust_work_before_relaxation();
  adapter_bounds_and_refinement_fallback_are_explicit();
  coupled_generated_blockers_require_the_joint_stage();
  joint_caps_and_failed_transactions_publish_only_the_prior_legal_subset();
  cancellation_before_joint_commit_keeps_the_published_owners_immutable();
}
