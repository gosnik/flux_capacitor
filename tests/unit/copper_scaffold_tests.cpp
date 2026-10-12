#include "flux/core/copper_scaffold.hpp"
#include "flux/core/drc.hpp"
#include "flux/core/routing_kernel.hpp"
#include "../../src/scaffold_via_history.hpp"
#include "../../src/scaffold_access.hpp"
#include "../../src/scaffold_collision.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>

namespace {
using namespace flux;
[[noreturn]] void fail(const char* message) { throw std::runtime_error(message); }
void require(bool value, const char* message) { if (!value) fail(message); }
[[nodiscard]] Point mm(double x, double y) { return {Nanometres::from_mm(x), Nanometres::from_mm(y)}; }
[[nodiscard]] Board fixture() {
  Board board;
  board.outline = Polygon{{mm(0,0), mm(20,0), mm(20,20), mm(0,20)}};
  board.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0U},
                  {LayerId{"B.Cu"}, "B.Cu", LayerKind::copper, 1U}};
  board.nets = {{NetId{"a"}, "a", 1U}, {NetId{"b"}, "b", 2U}};
  board.rules.default_track_width = Nanometres::from_mm(0.2);
  board.rules.default_clearance = Nanometres::from_mm(0.2);
  return board;
}
[[nodiscard]] GlobalRoutePath request(NetId net, Point a, LayerId al, Point b, LayerId bl) {
  GlobalRoutePath path; path.net = std::move(net); path.points = {{a, std::move(al)}, {b, std::move(bl)}};
  return path;
}
void via_histories_are_canonical_immutable_and_bounded() {
  ScaffoldViaHistory histories(1023U, 4096U);
  std::size_t forward = 0U, reverse = 0U;
  for (std::uint64_t i = 0U; i < 40U; ++i) {
    const auto next = histories.insert(forward, i * 7U);
    require(next.has_value(), "more than sixteen via sites must remain representable");
    forward = *next;
  }
  for (std::uint64_t i = 40U; i > 0U; --i) reverse = *histories.insert(reverse, (i - 1U) * 7U);
  require(forward == reverse, "equivalent via sets must share state identity regardless of visit order");
  const auto old_count = histories.node_count();
  require(histories.insert(forward, 7U) == forward && histories.node_count() == old_count,
          "revisiting a drilled position must reuse its exact history root without allocation");
  const auto extended = histories.insert(forward, 1023U);
  require(extended && !histories.contains(forward, 1023U) && histories.contains(*extended, 1023U),
          "inserting a via must preserve histories held by other search labels");
  ScaffoldViaHistory bounded(1023U, 6U);
  require(!bounded.insert(0U, 1U) && bounded.node_count() <= 6U,
          "history exhaustion must fail closed within the configured node allowance");
  ScaffoldViaHistory atomic(15U, 16U);
  const auto first = atomic.insert(0U, 0U);
  require(first.has_value() && atomic.insert(*first, 1U).has_value(), "initial capped histories must fit");
  const auto branch = atomic.insert(*first, 2U);
  require(branch.has_value(), "branched capped history must fit");
  const auto before_failure = atomic.node_count();
  require(!atomic.insert(*branch, 15U) && atomic.node_count() == before_failure,
          "failed insertion must roll back interned nodes instead of consuming future allowance");
  require(atomic.insert(*branch, 1U).has_value(),
          "a prefix-sharing candidate must remain insertable after a rejected disjoint candidate");
  ScaffoldViaHistory disabled(1023U, 0U);
  require(!disabled.insert(0U, 1U) && disabled.node_count() == 0U,
          "zero history allowance must not allocate sentinel nodes");
}
void accepts_legal_supplied_topology() {
  const auto board = fixture();
  const auto result = construct_copper_scaffold(board, {request(NetId{"a"}, mm(2,2), LayerId{"F.Cu"}, mm(18,2), LayerId{"F.Cu"})});
  require(result.complete && result.paths.size() == 1U, "legal direct route must be retained");
  require(check_copper_rules(result.copper).ok(), "published scaffold must pass copper DRC");
}
void finds_a_legal_detour_when_the_supplied_line_is_blocked() {
  auto board = fixture();
  board.tracks.push_back({ObjectId{"blocker"}, NetId{"b"}, LayerId{"F.Cu"}, mm(10,5), mm(10,15),
                          Nanometres::from_mm(0.8), false});
  CopperScaffoldConfig config;
  config.grid_pitch = Nanometres::from_mm(0.5);
  config.maximum_cells = 2'000U;
  const auto result = construct_copper_scaffold(
      board, {request(NetId{"a"}, mm(2,10), LayerId{"F.Cu"}, mm(18,10), LayerId{"F.Cu"})}, config);
  require(result.complete && result.paths.front().points.size() > 2U,
          "blocked direct line must receive a complete detour");
  require(check_copper_rules(result.copper).ok(), "detour must be DRC legal");
}
void supplies_a_legal_via_transition() {
  const auto board = fixture();
  const auto result = construct_copper_scaffold(board, {request(NetId{"a"}, mm(2,2), LayerId{"F.Cu"}, mm(18,2), LayerId{"B.Cu"})});
  require(result.complete && result.copper.vias.size() == 1U, "layer transition must materialize a via");
  require(check_copper_rules(result.copper).ok(), "via scaffold must be DRC legal");
}
void cancellation_never_publishes_illegal_copper() {
  const auto board = fixture();
  std::stop_source stop; stop.request_stop();
  const auto result = construct_copper_scaffold(board, {request(NetId{"a"}, mm(2,2), LayerId{"F.Cu"}, mm(18,18), LayerId{"F.Cu"})}, {}, stop.get_token());
  require(!result.complete && result.paths.empty() && result.counters.cancelled, "cancel must retain no partial candidate");
  require(check_copper_rules(result.copper).ok(), "cancel must not publish illegal copper");
}
void malformed_or_mismatched_retained_paths_are_not_reused() {
  const auto board = fixture();
  GlobalRoutePath malformed; malformed.net = NetId{"a"};
  CopperScaffoldResult retained; retained.paths.push_back(malformed);
  const auto result = construct_copper_scaffold(board, {malformed}, {}, {}, &retained);
  require(!result.complete && result.paths.empty() && result.missing_indices == std::vector<std::size_t>{0U},
          "malformed request must fail without publishing copper");
}
void duplicate_connection_identity_is_rejected_without_publication() {
  const auto board = fixture();
  const auto first = request(NetId{"a"}, mm(2,2), LayerId{"F.Cu"}, mm(18,2), LayerId{"F.Cu"});
  const auto result = construct_copper_scaffold(board, {first, first});
  require(!result.complete && result.paths.empty() && result.missing_indices.size() == 2U,
          "duplicate identity must not create ambiguous provisional copper");
}
void failed_causal_exchange_rolls_back_the_legal_subset() {
  auto board = fixture();
  // A thin single-layer channel makes the second connection meet the first at its only usable
  // crossing. The deliberately bounded frontier reaches that blocker, attempts its causal
  // exchange, and cannot rebuild the displaced connection; the original legal copper survives.
  board.layers.resize(1U);
  board.outline = Polygon{{mm(0,0), mm(20,0), mm(20,2), mm(0,2)}};
  CopperScaffoldConfig config;
  config.grid_pitch = Nanometres::from_mm(0.1);
  config.maximum_expansions_per_connection = 50U;
  config.maximum_victim_exchanges = 1U;
  config.maximum_joint_passage_candidates = 0U;
  config.maximum_displacement_attempts = 0U;
  const auto horizontal = request(NetId{"a"}, mm(2,1), LayerId{"F.Cu"}, mm(18,1), LayerId{"F.Cu"});
  const auto vertical = request(NetId{"b"}, mm(10,0.6), LayerId{"F.Cu"}, mm(10,1.4), LayerId{"F.Cu"});
  const auto source_snapshot = board;
  const auto result = construct_copper_scaffold(board, {horizontal, vertical}, config);
  require(!result.complete && result.counters.victim_exchanges_attempted > 0U &&
              result.paths.size() == 1U && result.paths.front() == horizontal && board == source_snapshot,
          "failed exchange must retain the previously legal connection");
  require(result.requests == std::vector<GlobalRoutePath>{horizontal, vertical} &&
              result.request_evidence.size() == 2U &&
              result.request_evidence[1].request_identity == result.request_identities[1] &&
              result.request_evidence[1].termination != CopperScaffoldSearchTermination::not_attempted &&
              result.counters.exchange_rollbacks > 0U && !result.exchange_rollbacks.empty(),
          "failed repair must preserve canonical demands and explain its bounded rollback");
  require(check_copper_rules(result.copper).ok(), "rollback must retain only legal copper");

  // Regional admission rechecks retained copper with search disabled.  It must not erase the
  // earlier frontier owners simply because that recheck did no work in the identical scene.
  auto admission = config;
  admission.maximum_expansions_per_connection = 0U;
  admission.maximum_victim_exchanges = 0U;
  admission.maximum_coupled_exchange_attempts = 0U;
  const auto rechecked = construct_copper_scaffold(board, {horizontal, vertical}, admission, {}, &result);
  require(rechecked.paths == result.paths && rechecked.copper == result.copper &&
              rechecked.request_evidence == result.request_evidence &&
              rechecked.exchange_rollbacks == result.exchange_rollbacks,
          "zero-budget admission must retain causal evidence for an identical checked subset");
}
void causal_exchange_preserves_survivors_and_rebuilds_victim() {
  auto board = fixture();
  board.outline = Polygon{{mm(0,0), mm(20,0), mm(20,2.6), mm(0,2.6)}};
  board.rules.default_via_diameter = Nanometres::from_mm(2.0);
  board.rules.default_via_drill = Nanometres::from_mm(0.5);
  board.rules.copper_to_edge_clearance = Nanometres{0};
  board.nets.push_back({NetId{"unrelated"}, "unrelated", 3U});
  const TrackSegment unrelated{ObjectId{"fixed-unrelated"}, NetId{"unrelated"}, LayerId{"F.Cu"},
                               mm(5,2.3), mm(7,2.3), Nanometres::from_mm(0.2), true};
  board.tracks.push_back(unrelated);
  CopperScaffoldConfig config;
  config.grid_pitch = Nanometres::from_mm(0.1);
  config.maximum_cells = 20'000U;
  config.maximum_expansions_per_connection = 100'000U;
  config.maximum_victim_exchanges = 1U;
  config.maximum_joint_passage_candidates = 0U;
  config.maximum_displacement_attempts = 0U;
  // The first route seals the only F.Cu crossing. The incoming endpoints lie on opposite sides
  // of it and the large through-via cannot be placed in either half-channel. Once the victim is
  // removed, the incoming route is straight and the victim can detour or use B.Cu.
  const auto victim = request(NetId{"a"}, mm(2,1.3), LayerId{"F.Cu"}, mm(18,1.3), LayerId{"F.Cu"});
  const auto incoming = request(NetId{"b"}, mm(10,1.1), LayerId{"F.Cu"}, mm(10,1.5), LayerId{"F.Cu"});
  auto survivor = request(NetId{"unrelated"}, mm(12,2.3), LayerId{"F.Cu"}, mm(14,2.3), LayerId{"F.Cu"});
  std::vector<TrackSegment> published_survivor;
  const auto result = construct_copper_scaffold(board, {survivor, victim, incoming}, config, {}, nullptr,
      [&](const CopperScaffoldResult& checkpoint) {
        if (published_survivor.empty()) for (const auto& track : checkpoint.copper.tracks)
          if (track.net == NetId{"unrelated"} && track.id != unrelated.id)
            published_survivor.push_back(track);
      });
  if (!result.complete) throw std::runtime_error(result.diagnostic +
      "; exchanges=" + std::to_string(result.counters.victim_exchanges_attempted) +
      "; expansions=" + std::to_string(result.counters.expansions) +
      "; rejected_goals=" + std::to_string(result.counters.goal_rejections));
  require(!published_survivor.empty(), "survivor must be published before the exchange");
  for (const auto& track : published_survivor)
    require(std::find(result.copper.tracks.begin(), result.copper.tracks.end(), track) != result.copper.tracks.end(),
            "exchange must preserve published survivor geometry and object IDs verbatim");
  require(result.counters.survivor_tracks_preserved >= published_survivor.size(),
          "accepted exchange must report preserved survivor copper");
  require(result.complete && result.paths.size() == 3U &&
              result.counters.victim_exchanges_accepted > 0U,
          "blocked connection must be admitted by a successful causal victim exchange");
  require(check_copper_rules(result.copper).ok(), "accepted exchange must publish DRC-clean copper");
  const auto rebuilt = std::find_if(result.paths.begin(), result.paths.end(), [](const auto& path) {
    return path.net == NetId{"a"};
  });
  require(rebuilt != result.paths.end() && *rebuilt != victim,
          "displaced victim must be rebuilt around the admitted connection");
  const auto preserved = std::find_if(result.copper.tracks.begin(), result.copper.tracks.end(),
                                      [](const auto& track) { return track.id == ObjectId{"fixed-unrelated"}; });
  require(preserved != result.copper.tracks.end() && *preserved == unrelated,
          "successful exchange must leave unrelated source copper unchanged");
}
void bounded_pns_displacement_moves_only_attributed_generated_copper() {
  for (const bool rotated : {false, true}) {
  auto board = fixture();
  board.layers.resize(1U);
  board.outline = Polygon{{mm(0,0), mm(20,0), mm(20,3), mm(0,3)}};
  board.rules.copper_to_edge_clearance = Nanometres{0};
  TrackSegment protected_source{ObjectId{"protected-source"}, NetId{"b"}, LayerId{"F.Cu"},
                                      mm(2,2.6), mm(4,2.6), Nanometres::from_mm(0.2), false};
  board.tracks.push_back(protected_source);
  CopperScaffoldConfig config;
  config.grid_pitch = Nanometres::from_mm(0.2);
  config.maximum_expansions_per_connection = 8U;
  config.maximum_joint_passage_candidates = 0U;
  config.maximum_displacement_attempts = 1U;
  config.maximum_displacement_expansions = 200U;
  config.maximum_displacement_candidate_checks = 16U;
  config.maximum_displacement_depth = 2U;
  auto victim = request(NetId{"a"}, mm(2,1.5), LayerId{"F.Cu"}, mm(18,1.5), LayerId{"F.Cu"});
  auto incoming = request(NetId{"b"}, mm(10,1.0), LayerId{"F.Cu"}, mm(10,2.0), LayerId{"F.Cu"});
  victim.points.insert(victim.points.begin() + 1, {mm(5,1.5), LayerId{"F.Cu"}});
  victim.segment_widths.assign(2U, Nanometres::from_mm(0.2));
  victim.segment_width_kinds.assign(2U, GlobalRouteSegmentWidthKind::nominal);
  if (rotated) {
    const auto rotate = [](Point point) { return Point{point.y, Nanometres::from_mm(20) - point.x}; };
    for (auto& point : board.outline.vertices) point = rotate(point);
    protected_source.start = rotate(protected_source.start);
    protected_source.end = rotate(protected_source.end);
    board.tracks.front() = protected_source;
    for (auto& point : victim.points) point.position = rotate(point.position);
    for (auto& point : incoming.points) point.position = rotate(point.position);
  }
  auto disabled = config;
  disabled.maximum_displacement_attempts = 0U;
  disabled.maximum_victim_exchanges = 0U;
  disabled.maximum_coupled_exchange_attempts = 0U;
  const auto no_shove = construct_copper_scaffold(board, {victim, incoming}, disabled);
  require(!no_shove.complete && no_shove.counters.displacement_attempts == 0U &&
              std::find(no_shove.copper.tracks.begin(), no_shove.copper.tracks.end(), protected_source) !=
                  no_shove.copper.tracks.end(),
          "zero displacement budget must leave protected source copper untouched");
  const auto result = construct_copper_scaffold(board, {victim, incoming}, config);
  const auto diagnostic = std::string{"PNS passage: complete="} + std::to_string(result.complete) +
      "; attempts=" + std::to_string(result.counters.displacement_attempts) +
      "; accepted=" + std::to_string(result.counters.displacement_accepted) +
      "; searchfail=" + std::to_string(result.counters.displacement_search_failures) +
      "; shovefail=" + std::to_string(result.counters.displacement_shove_rejections) +
      "; validationfail=" + std::to_string(result.counters.displacement_validation_rejections) +
      "; reconstructionfail=" + std::to_string(result.counters.displacement_reconstruction_rejections);
  require(result.complete && result.counters.displacement_accepted == 1U &&
              result.counters.displacement_moved_tracks > 0U, diagnostic.c_str());
  require(std::find(result.copper.tracks.begin(), result.copper.tracks.end(), protected_source) !=
              result.copper.tracks.end() && check_copper_rules(result.copper).ok(),
          "PNS displacement must preserve immutable source copper and publish exact DRC-clean copper");
  const auto moved = std::find_if(result.paths.begin(), result.paths.end(), [](const auto& path) {
    return path.net == NetId{"a"};
  });
  require(moved != result.paths.end() && moved->points.size() > 2U,
          "the moved owner must retain reconstructed dogleg geometry for later reuse");
  require(std::any_of(result.copper.tracks.begin(), result.copper.tracks.end(), [&](const auto& track) {
            return track.net == victim.net && track.start == victim.points[0].position &&
                   track.end == victim.points[1].position;
          }), "the uncollided piece of a multi-segment owner must survive displacement unchanged");
  const auto rehydrated = construct_copper_scaffold(board, {victim, incoming}, config, {}, &result);
  require(rehydrated.complete && rehydrated.paths == result.paths &&
              check_copper_rules(rehydrated.copper).ok(),
          "retained reconstruction must preserve the PNS-regenerated owner path");
  }
}
void coupled_exchange_rebuilds_two_causal_victims_and_preserves_survivor() {
  auto board = fixture();
  board.outline = Polygon{{mm(0,0), mm(20,0), mm(20,2.6), mm(0,2.6)}};
  board.rules.default_via_diameter = Nanometres::from_mm(2.0);
  board.rules.default_via_drill = Nanometres::from_mm(0.5);
  board.rules.copper_to_edge_clearance = Nanometres{0};
  board.nets.push_back({NetId{"survivor"}, "survivor", 3U});
  const TrackSegment fixed{ObjectId{"fixed-coupled-survivor"}, NetId{"survivor"}, LayerId{"F.Cu"},
                           mm(3,2.3), mm(5,2.3), Nanometres::from_mm(0.2), true};
  board.tracks.push_back(fixed);
  CopperScaffoldConfig config;
  config.grid_pitch = Nanometres::from_mm(0.1);
  config.maximum_cells = 20'000U;
  config.maximum_expansions_per_connection = 100'000U;
  config.maximum_victim_exchanges = 2U;
  config.maximum_joint_passage_candidates = 0U;
  // Allow three-victim components. This fixture has exactly two causal blockers, so it verifies
  // that the generalized component transaction keeps the smaller causal set and does not pull
  // unrelated copper into the repair.
  config.maximum_coupled_victims = 3U;
  config.maximum_coupled_exchange_attempts = 4U;
  const auto survivor = request(NetId{"survivor"}, mm(12,2.3), LayerId{"F.Cu"}, mm(14,2.3), LayerId{"F.Cu"});
  const auto lower = request(NetId{"a"}, mm(2,0.9), LayerId{"F.Cu"}, mm(18,0.9), LayerId{"F.Cu"});
  const auto upper = request(NetId{"b"}, mm(2,1.7), LayerId{"F.Cu"}, mm(18,1.7), LayerId{"F.Cu"});
  auto incoming = request(NetId{"survivor"}, mm(10,0.6), LayerId{"F.Cu"}, mm(10,2.0), LayerId{"F.Cu"});
  incoming.connection_index = 1U;
  const auto result = construct_copper_scaffold(board, {survivor, lower, upper, incoming}, config);
  if (!result.complete) throw std::runtime_error(result.diagnostic +
      "; coupled=" + std::to_string(result.counters.coupled_victim_exchanges_attempted));
  require(result.counters.coupled_victim_exchanges_accepted == 1U &&
              result.counters.coupled_victim_exchanges_attempted > 0U &&
              result.counters.coupled_component_victims_accepted == 2U &&
              result.counters.coupled_component_victims_attempted >= 2U,
          "two causal barriers must require the bounded coupled exchange after singleton failures");
  require(std::find(result.copper.tracks.begin(), result.copper.tracks.end(), fixed) != result.copper.tracks.end() &&
              result.satisfied_indices.size() == 4U && check_copper_rules(result.copper).ok(),
          "coupled commit must retain unrelated copper and every previously satisfied demand");
}
void three_victim_component_succeeds_only_when_the_component_bound_allows_it() {
  auto board = fixture();
  board.layers.resize(1U);
  board.outline = Polygon{{mm(0,0), mm(20,0), mm(20,3.6), mm(0,3.6)}};
  board.rules.copper_to_edge_clearance = Nanometres{0};
  board.nets.push_back({NetId{"c"}, "c", 3U});
  board.nets.push_back({NetId{"survivor"}, "survivor", 4U});
  const TrackSegment fixed{ObjectId{"fixed-three-victim-survivor"}, NetId{"survivor"}, LayerId{"F.Cu"},
                           mm(15,3.4), mm(16,3.4), Nanometres::from_mm(0.2), true};
  board.tracks.push_back(fixed);
  CopperScaffoldConfig config;
  config.grid_pitch = Nanometres::from_mm(0.1);
  config.maximum_cells = 20'000U;
  config.maximum_expansions_per_connection = 50'000U;
  config.maximum_victim_exchanges = 3U;
  config.maximum_joint_passage_candidates = 0U;
  config.maximum_coupled_exchange_attempts = 3U;
  config.maximum_coupled_expansions = 1'000'000U;
  const auto survivor = request(NetId{"survivor"}, mm(12,3.4), LayerId{"F.Cu"}, mm(14,3.4), LayerId{"F.Cu"});
  const auto u_demand = [](NetId net, double lower, double upper) {
    GlobalRoutePath path;
    path.net = std::move(net);
    path.points = {{mm(0.1, lower), LayerId{"F.Cu"}}, {mm(19.9, lower), LayerId{"F.Cu"}},
                   {mm(19.9, upper), LayerId{"F.Cu"}}, {mm(0.1, upper), LayerId{"F.Cu"}}};
    return path;
  };
  // Each supplied U is legal but seals the full width. Once all three are removed, the incoming
  // connection is straight and every victim has a short, mutually legal left-edge replacement.
  const auto lower = u_demand(NetId{"a"}, 0.6, 1.0);
  const auto middle = u_demand(NetId{"b"}, 1.6, 2.0);
  const auto upper = u_demand(NetId{"c"}, 2.6, 3.0);
  auto incoming = request(NetId{"survivor"}, mm(10,0.2), LayerId{"F.Cu"}, mm(10,3.4), LayerId{"F.Cu"});
  incoming.connection_index = 1U;
  const std::vector<GlobalRoutePath> demands{survivor, lower, middle, upper, incoming};
  auto pair_bound = config;
  pair_bound.maximum_coupled_victims = 2U;
  const auto pairs = construct_copper_scaffold(board, demands, pair_bound);
  auto triple_bound = config;
  triple_bound.maximum_coupled_victims = 3U;
  const auto triples = construct_copper_scaffold(board, demands, triple_bound);
  if (pairs.complete || !triples.complete || triples.counters.coupled_component_victims_accepted != 3U ||
      triples.counters.coupled_component_victims_attempted < 5U)
    throw std::runtime_error("three-victim component: pairs_complete=" + std::to_string(pairs.complete) +
        " triples_complete=" + std::to_string(triples.complete) +
        " pair_missing=" + std::to_string(pairs.missing_indices.size()) +
        " triple_missing=" + std::to_string(triples.missing_indices.size()) +
        " triple_attempted_victims=" + std::to_string(triples.counters.coupled_component_victims_attempted) +
        " triple_accepted_victims=" + std::to_string(triples.counters.coupled_component_victims_accepted));
  require(std::find(triples.copper.tracks.begin(), triples.copper.tracks.end(), fixed) != triples.copper.tracks.end() &&
              triples.satisfied_indices.size() == demands.size() && check_copper_rules(triples.copper).ok(),
          "three-victim commit must preserve unrelated copper and publish only DRC-clean routes");
}
void callback_cancellation_publishes_only_legal_checkpoint() {
  const auto board = fixture();
  const auto first = request(NetId{"a"}, mm(2,2), LayerId{"F.Cu"}, mm(18,2), LayerId{"F.Cu"});
  const auto second = request(NetId{"b"}, mm(2,18), LayerId{"F.Cu"}, mm(18,18), LayerId{"F.Cu"});
  std::stop_source stop;
  std::size_t callbacks = 0U;
  CopperScaffoldResult checkpoint;
  const auto result = construct_copper_scaffold(
      board, {first, second}, {}, stop.get_token(), nullptr,
      [&](const CopperScaffoldResult& published) {
        ++callbacks;
        checkpoint = published;
        if (published.paths.size() == 1U) stop.request_stop();
      });
  require(callbacks == 1U && result.counters.cancelled && !result.complete &&
              result.paths == std::vector<GlobalRoutePath>{first} &&
              result.missing_indices == std::vector<std::size_t>{1U},
          "callback cancellation must retain exactly the first legal checkpoint");
  require(checkpoint.paths == result.paths && checkpoint.missing_indices == std::vector<std::size_t>{1U} &&
              check_copper_rules(checkpoint.copper).ok() && check_copper_rules(result.copper).ok(),
          "published cancellation checkpoint and final result must contain only legal retained copper");
}
void existing_component_is_a_completed_transaction_without_new_copper() {
  auto board = fixture();
  board.tracks.push_back({ObjectId{"existing"}, NetId{"a"}, LayerId{"F.Cu"},
                          mm(2,2), mm(18,2), Nanometres::from_mm(0.2), false});
  const auto path = request(NetId{"a"}, mm(3,2), LayerId{"F.Cu"}, mm(17,2), LayerId{"F.Cu"});
  const auto result = construct_copper_scaffold(board, {path});
  require(result.complete && result.paths.empty() && result.copper == board &&
              result.request_count == 1U && result.satisfied_indices == std::vector<std::size_t>{0U} &&
              result.counters.routed_connections == 1U,
          "existing component connectivity must count without redundant copper");
}
void retained_branch_can_attach_to_component_interior() {
  auto board = fixture();
  board.tracks.push_back({ObjectId{"source"}, NetId{"a"}, LayerId{"F.Cu"},
                          mm(2,2), mm(18,2), Nanometres::from_mm(0.2), false});
  const auto original = request(NetId{"a"}, mm(2,2), LayerId{"F.Cu"}, mm(15,8), LayerId{"F.Cu"});
  const auto branch = request(NetId{"a"}, mm(15,2), LayerId{"F.Cu"}, mm(15,8), LayerId{"F.Cu"});
  CopperScaffoldResult retained; retained.paths.push_back(branch);
  const auto result = construct_copper_scaffold(board, {original}, {}, {}, &retained);
  require(result.complete && result.paths == std::vector<GlobalRoutePath>{branch},
          "valid retained branch must not be rejected for using a different source attachment");
  require(check_copper_rules(result.copper).ok(), "component branch must remain DRC clean");
  board.tracks.clear();
  const auto stale = construct_copper_scaffold(board, {original}, {}, {}, &retained);
  require(stale.complete && stale.paths != std::vector<GlobalRoutePath>{branch},
          "stale branch must not satisfy a request after its source component disappears");
}
void component_geometry_precedes_disconnected_pad_fanout() {
  auto board = fixture();
  // Foreign same-net tracks appear first in board order and must be filtered before the cap.
  for (std::size_t index = 0U; index < 40U; ++index)
    board.tracks.push_back({ObjectId{"far-track-" + std::to_string(index)}, NetId{"a"}, LayerId{"F.Cu"},
                            mm(1.0 + static_cast<double>(index) * 0.4, 12.0),
                            mm(1.2 + static_cast<double>(index) * 0.4, 12.0),
                            Nanometres::from_mm(0.2), false});
  board.tracks.push_back({ObjectId{"source"}, NetId{"a"}, LayerId{"F.Cu"},
                          mm(2,2), mm(18,2), Nanometres::from_mm(0.2), false});
  // More same-net terminal pads than the bounded candidate list, each on a separate component.
  // The useful projected point (15,2) must survive their board-order fanout.
  for (std::size_t index = 0U; index < 40U; ++index) {
    Footprint footprint;
    footprint.id = ObjectId{"far-pad-footprint-" + std::to_string(index)};
    footprint.position = mm(1.0 + static_cast<double>(index) * 0.4, 15.0);
    footprint.pads.push_back({ObjectId{"far-pad-" + std::to_string(index)}, NetId{"a"}, Point{},
                              {LayerId{"F.Cu"}}});
    board.footprints.push_back(std::move(footprint));
  }
  const auto path = request(NetId{"a"}, mm(2,2), LayerId{"F.Cu"}, mm(15,8), LayerId{"F.Cu"});
  const auto access = scaffold_connection_access(board, path, 4U);
  require(std::find(access.starts.begin(), access.starts.end(),
                    GlobalRoutePoint{mm(15,2), LayerId{"F.Cu"}}) != access.starts.end(),
          "bounded component access must retain a same-component track projection before "
          "disconnected same-net pads");
}
void search_uses_a_component_branch_instead_of_fixed_endpoint() {
  auto board = fixture();
  board.layers.resize(1U);
  board.tracks.push_back({ObjectId{"source"}, NetId{"a"}, LayerId{"F.Cu"},
                          mm(2,2), mm(18,2), Nanometres::from_mm(0.2), false});
  board.tracks.push_back({ObjectId{"obstacle"}, NetId{"b"}, LayerId{"F.Cu"},
                          mm(8,3), mm(8,9), Nanometres::from_mm(0.2), false});
  const auto path = request(NetId{"a"}, mm(2,2), LayerId{"F.Cu"}, mm(15,8), LayerId{"F.Cu"});
  CopperScaffoldConfig config; config.grid_pitch = Nanometres::from_mm(0.5);
  const auto result = construct_copper_scaffold(board, {path}, config);
  require(result.complete && result.paths.size() == 1U &&
              result.paths.front().points.front().position == mm(15,2),
          "blocked fixed endpoint route should branch from the nearest component interior");
  require(check_copper_rules(result.copper).ok(), "searched component branch must pass DRC");
}
void rejected_goal_assembly_does_not_end_connection_search() {
  auto board = fixture();
  board.rules.hole_to_hole_clearance = Nanometres::from_mm(3.8);
  // An F.Cu moat permits a via at its center but blocks every planar lattice approach.
  // Every candidate therefore reaches exactly the same goal/source/arrival state. The
  // cheapest first via (at x=8) violates pair drill spacing; moving it to x=7.5 is legal.
  const std::vector<Point> ring{mm(11.4,9.4), mm(12.6,9.4), mm(12.6,10.6), mm(11.4,10.6)};
  for (std::size_t i = 0U; i < ring.size(); ++i)
    board.tracks.push_back({ObjectId{"moat" + std::to_string(i)}, NetId{"b"}, LayerId{"F.Cu"},
                            ring[i], ring[(i + 1U) % ring.size()], Nanometres::from_mm(0.2), false});
  const auto path = request(NetId{"a"}, mm(8,10), LayerId{"F.Cu"}, mm(12,10), LayerId{"F.Cu"});
  CopperScaffoldConfig config; config.grid_pitch = Nanometres::from_mm(0.5);
  config.progressive_attachment_search = true;
  config.maximum_attachment_states = 1U;
  config.initial_attachment_states = 1U;
  config.expanded_attachment_states = 1U;
  config.maximum_goal_attempts = 27U;
  config.maximum_expansions_per_connection = 100'000U;
  const auto result = construct_copper_scaffold(board, {path}, config);
  require(result.complete && result.counters.goal_rejections > 0U &&
              result.counters.goal_rejections < config.maximum_goal_attempts &&
              result.counters.duplicate_rejected_assemblies > 0U &&
              result.counters.expansions <= config.maximum_expansions_per_connection,
          ("a rejected assembly must leave different via histories searchable without replenishing "
          "the shared goal or expansion budget: complete=" + std::to_string(result.complete) +
          " rejected=" + std::to_string(result.counters.goal_rejections) +
          " duplicates=" + std::to_string(result.counters.duplicate_rejected_assemblies) +
          " expansions=" + std::to_string(result.counters.expansions)).c_str());
  require(check_copper_rules(result.copper).ok(), "recovered goal assembly must pass full copper DRC");
}
void compact_search_preserves_component_attachment_provenance_within_shared_budget() {
  auto board = fixture();
  board.layers.resize(1U);
  board.tracks.push_back({ObjectId{"compact-source"}, NetId{"a"}, LayerId{"F.Cu"},
                          mm(2,2), mm(18,2), Nanometres::from_mm(0.2), false});
  board.tracks.push_back({ObjectId{"compact-obstacle"}, NetId{"b"}, LayerId{"F.Cu"},
                          mm(8,3), mm(8,9), Nanometres::from_mm(0.2), false});
  const auto demand = request(NetId{"a"}, mm(2,2), LayerId{"F.Cu"}, mm(15,8), LayerId{"F.Cu"});
  CopperScaffoldConfig config;
  config.grid_pitch = Nanometres::from_mm(0.5);
  config.maximum_expansions_per_connection = 200U;
  config.compact_search_expansions = 80U;
  const auto result = construct_copper_scaffold(board, {demand}, config);
  require(result.complete && result.paths.size() == 1U &&
              result.paths.front().points.front() == GlobalRoutePoint{mm(15,2), LayerId{"F.Cu"}} &&
              result.counters.compact_search_attempts == 1U &&
              result.counters.compact_search_expansions <= config.compact_search_expansions &&
              result.counters.expansions <= config.maximum_expansions_per_connection,
          "compact search must retain the chosen source attachment and share the connection budget");
  require(scaffold_request_connected(result.copper, demand) && check_copper_rules(result.copper).ok(),
          "compact-source reconstruction must pass exact connectivity and copper legality");

  auto disabled = config;
  disabled.compact_search_expansions = 0U;
  const auto baseline = construct_copper_scaffold(board, {demand}, disabled);
  require(baseline.complete && baseline.counters.compact_search_attempts == 0U &&
              baseline.counters.compact_search_expansions == 0U &&
              scaffold_request_connected(baseline.copper, demand) && check_copper_rules(baseline.copper).ok(),
          "zero compact budget must retain the provenance-rich baseline search");
}
void compact_search_falls_back_when_via_history_changes_goal_legality() {
  auto board = fixture();
  board.rules.hole_to_hole_clearance = Nanometres::from_mm(3.8);
  // This F.Cu moat forces a two-via connection. The optimistic node-only pass
  // can reach the same goal without retaining the drilled-site history; the
  // rich fallback must retain it to reject the first close via pair and find
  // the alternative legal placement.
  const std::vector<Point> ring{mm(11.4,9.4), mm(12.6,9.4), mm(12.6,10.6), mm(11.4,10.6)};
  for (std::size_t i = 0U; i < ring.size(); ++i)
    board.tracks.push_back({ObjectId{"compact-moat" + std::to_string(i)}, NetId{"b"}, LayerId{"F.Cu"},
                            ring[i], ring[(i + 1U) % ring.size()], Nanometres::from_mm(0.2), false});
  const auto demand = request(NetId{"a"}, mm(8,10), LayerId{"F.Cu"}, mm(12,10), LayerId{"F.Cu"});
  CopperScaffoldConfig config;
  config.grid_pitch = Nanometres::from_mm(0.5);
  config.maximum_attachment_states = 1U;
  config.maximum_goal_attempts = 256U;
  config.maximum_expansions_per_connection = 100'000U;
  config.compact_search_expansions = 128U;
  const auto result = construct_copper_scaffold(board, {demand}, config);
  require(result.complete && result.counters.compact_search_attempts >= 1U &&
              result.counters.compact_search_attempts <= 3U &&
              result.counters.compact_search_fallbacks == result.counters.compact_search_attempts &&
              result.counters.compact_search_successes == 0U && result.counters.goal_rejections > 0U &&
              result.counters.expansions <= config.maximum_expansions_per_connection,
          "path-dependent via legality must fall through compact dominance to rich-state search");
  require(scaffold_request_connected(result.copper, demand) && check_copper_rules(result.copper).ok(),
          "rich fallback must publish only an exactly legal via realization");
}
void compact_search_cancellation_records_started_work() {
  auto board = fixture();
  board.layers.resize(1U);
  board.tracks.push_back({ObjectId{"closed-wall"}, NetId{"b"}, LayerId{"F.Cu"},
                          mm(10,0), mm(10,20), Nanometres::from_mm(0.2), true});
  const auto demand = request(NetId{"a"}, mm(2,10), LayerId{"F.Cu"}, mm(18,10), LayerId{"F.Cu"});
  CopperScaffoldConfig config;
  config.maximum_victim_exchanges = 0U;
  config.maximum_joint_passage_candidates = 0U;
  config.maximum_displacement_attempts = 0U;
  config.maximum_coupled_exchange_attempts = 0U;
  config.progressive_attachment_search = true;
  config.initial_attachment_states = 1U;
  config.expanded_attachment_states = 8U;
  CopperScaffoldResult result;
  std::stop_source search_stop;
  RoutingKernel kernel([&](std::stop_token) {
    result = construct_copper_scaffold(board, {demand}, config, search_stop.get_token());
  });
  require(kernel.advance({2048U, std::chrono::seconds{10}}) == RoutingKernelStatus::yielded,
          "closed passage must retain an active bounded search");
  search_stop.request_stop();
  while (kernel.advance({2048U, std::chrono::seconds{10}}) != RoutingKernelStatus::finished) {}
  require(result.counters.attachment_search_probes > 0U &&
              result.counters.compact_search_expansions > 0U && result.counters.cancelled &&
              result.request_evidence.size() == 1U &&
              result.request_evidence.front().termination == CopperScaffoldSearchTermination::cancelled,
          ("cancel evidence: compact=" + std::to_string(result.counters.compact_search_expansions) +
           "; cancelled=" + std::to_string(result.counters.cancelled) +
           "; evidence=" + std::to_string(result.request_evidence.size()) +
           "; termination=" + (result.request_evidence.empty() ? std::string{"none"} :
               std::string{to_string(result.request_evidence.front().termination)})).c_str());
  require(result.paths.empty() && result.copper == board,
          "cancelled compact search must not publish candidate copper");
}
void disconnected_padded_components_are_not_counted_as_connected() {
  auto board = fixture();
  for (int i = 0; i < 2; ++i) {
    Footprint footprint; footprint.id = ObjectId{"fp" + std::to_string(i)};
    footprint.position = mm(i == 0 ? 2 : 18, 2);
    Pad pad; pad.id = ObjectId{"pad" + std::to_string(i)}; pad.net = NetId{"a"};
    pad.layers = {LayerId{"F.Cu"}}; pad.plated = false; pad.width = Nanometres::from_mm(1);
    pad.height = Nanometres::from_mm(1); footprint.pads.push_back(pad);
    board.footprints.push_back(footprint);
  }
  const auto path = request(NetId{"a"}, mm(2,2), LayerId{"F.Cu"}, mm(18,2), LayerId{"F.Cu"});
  const auto result = construct_copper_scaffold(board, {path});
  require(result.complete && result.paths.size() == 1U && !result.copper.tracks.empty(),
          "two components already containing pads still require a connecting route");
}
void pad_membership_does_not_accept_copper_on_a_wrong_coarse_layer() {
  auto board = fixture();
  for (int i = 0; i < 2; ++i) {
    Footprint footprint; footprint.id = ObjectId{"fp" + std::to_string(i)};
    footprint.position = mm(i == 0 ? 2 : 18, 2);
    Pad pad; pad.id = ObjectId{"pad" + std::to_string(i)}; pad.net = NetId{"a"};
    pad.layers = {LayerId{"F.Cu"}}; pad.plated = false; pad.width = Nanometres::from_mm(1);
    pad.height = Nanometres::from_mm(1); footprint.pads.push_back(pad);
    board.footprints.push_back(footprint);
  }
  const auto wrong_layer = request(NetId{"a"}, mm(2,2), LayerId{"B.Cu"}, mm(18,2), LayerId{"B.Cu"});
  const auto result = construct_copper_scaffold(board, {wrong_layer});
  require(result.complete && !result.paths.empty() &&
              result.paths.front().points.front().layer == LayerId{"F.Cu"} &&
              result.paths.front().points.back().layer == LayerId{"F.Cu"},
          "a legal B.Cu line under F.Cu pads must not satisfy their connection");
  // A same-net B pad directly under the source F pad is a different physical component.
  auto back = board.footprints.front(); back.id = ObjectId{"back-fp"};
  back.pads.front().id = ObjectId{"back-pad"}; back.pads.front().layers = {LayerId{"B.Cu"}};
  board.footprints.push_back(back);
  board.tracks.push_back({ObjectId{"back-wire"}, NetId{"a"}, LayerId{"B.Cu"},
                          mm(2,2), mm(18,8), Nanometres::from_mm(0.2), false});
  const auto across = request(NetId{"a"}, mm(2,2), LayerId{"F.Cu"}, mm(18,8), LayerId{"B.Cu"});
  const auto isolated = construct_copper_scaffold(board, {across});
  require(isolated.complete && !isolated.paths.empty() && !isolated.copper.vias.empty(),
          "coincident opposite-side same-net pads must not create a through connection");
}
void same_net_on_separate_layers_does_not_prove_connectivity() {
  auto board = fixture();
  board.tracks.push_back({ObjectId{"front"}, NetId{"a"}, LayerId{"F.Cu"},
                          mm(2,2), mm(18,2), Nanometres::from_mm(0.2), false});
  board.tracks.push_back({ObjectId{"back"}, NetId{"a"}, LayerId{"B.Cu"},
                          mm(2,2), mm(18,2), Nanometres::from_mm(0.2), false});
  const auto path = request(NetId{"a"}, mm(3,2), LayerId{"F.Cu"}, mm(17,2), LayerId{"B.Cu"});
  const auto result = construct_copper_scaffold(board, {path});
  require(result.complete && !result.paths.empty() && !result.copper.vias.empty(),
          "coincident same-net copper on different layers needs a physical transition");
}
void fine_pitch_rotated_pad_edge_access_reaches_the_scaffold_grid() {
  auto board = fixture();
  board.layers.resize(1U);
  board.rules.default_track_width = Nanometres::from_mm(0.25);
  Footprint footprint;
  footprint.id = ObjectId{"fine-pitch-footprint"};
  footprint.position = mm(5.05, 5.05);
  Pad pad;
  pad.id = ObjectId{"fine-pitch-pad"}; pad.net = NetId{"a"}; pad.layers = {LayerId{"F.Cu"}};
  pad.shape = PadShape::rectangle; pad.width = Nanometres::from_mm(0.3);
  pad.height = Nanometres::from_mm(1.5); pad.rotation = Microdegrees{90'000'000};
  footprint.pads.push_back(pad);
  for (const auto offset : {-0.5, 0.5}) {
    auto neighbor = pad;
    neighbor.id = ObjectId{offset < 0 ? "lower-pad" : "upper-pad"};
    neighbor.net = NetId{"b"}; neighbor.local_position = mm(0, offset);
    footprint.pads.push_back(neighbor);
  }
  board.footprints.push_back(footprint);
  const auto demand = request(NetId{"a"}, mm(8.05, 8.05), LayerId{"F.Cu"}, mm(5.05, 5.05), LayerId{"F.Cu"});
  const auto center_only = scaffold_connection_access(board, demand, 1U);
  require(center_only.goals == std::vector<GlobalRoutePoint>{{mm(5.05, 5.05), LayerId{"F.Cu"}}},
          "existing exact center must remain the first bounded attachment anchor");
  const ScaffoldCollisionIndex collision(board);
  for (int x = 48; x <= 52; ++x) for (int y = 48; y <= 52; ++y) {
    TrackSegment probe{ObjectId{"probe"}, NetId{"a"}, LayerId{"F.Cu"}, mm(5.05, 5.05),
                       mm(static_cast<double>(x) / 10.0, static_cast<double>(y) / 10.0),
                       board.rules.default_track_width, false};
    require(!collision.check_track(probe).ok(),
            "every center-only grid attachment must be blocked by the fine-pitch neighbors");
  }
  const auto access = scaffold_connection_access(board, demand, 8U);
  require(access.goals.size() > 1U, "rotated pad edge escapes must supplement the existing center");
  CopperScaffoldConfig config;
  config.grid_pitch = Nanometres::from_mm(0.1); config.attachment_radius_cells = 2U;
  config.progressive_attachment_search = true;
  config.initial_attachment_states = 1U;
  config.expanded_attachment_states = 8U;
  config.maximum_expansions_per_connection = 5'000U;
  const auto result = construct_copper_scaffold(board, {demand}, config);
  require(result.complete && check_copper_rules(result.copper).ok() &&
              result.counters.searched_connections > 0U &&
              result.counters.attachment_search_rounds > 1U &&
              result.counters.expansions <= config.maximum_expansions_per_connection,
          "a failed center-only prefix must widen to a DRC-clean searched pad-edge escape without "
          "multiplying the connection expansion budget");

  auto eager_direct = config;
  eager_direct.progressive_attachment_search = false;
  const auto unchanged_access = construct_copper_scaffold(board, {demand}, eager_direct);
  require(unchanged_access.complete && unchanged_access.counters.outward_attachment_probes == 0U,
          "legal direct attachment sets must not compete with new outward exits in eager search");

  // Tighten the fixture so every old attachment (including pad-edge anchors) is
  // blocked. The only new freedom is an outward stub and its shifted local lattice.
  auto narrow = board;
  for (auto& p : narrow.footprints.front().pads) p.height = Nanometres::from_mm(1.55);
  const auto narrow_access = scaffold_connection_access(narrow, demand, 32U);
  const ScaffoldCollisionIndex narrow_collision(narrow);
  for (const auto& anchor : narrow_access.goals) {
    const auto gx = anchor.position.x.value() / 100'000;
    const auto gy = anchor.position.y.value() / 100'000;
    for (int dx = -1; dx <= 1; ++dx) for (int dy = -1; dy <= 1; ++dy) {
      const Point point{Nanometres{(gx + dx) * 100'000}, Nanometres{(gy + dy) * 100'000}};
      TrackSegment probe{ObjectId{"old-attachment-probe"}, NetId{"a"}, anchor.layer,
                         anchor.position, point, narrow.rules.default_track_width, false};
      require(!narrow_collision.check_track(probe).ok(),
              "every direct pad-anchor attachment must fail in the shifted-lattice microcase");
    }
  }
  for (const bool compact : {true, false}) for (const bool reverse : {false, true}) {
    auto stub_config = config;
    stub_config.attachment_radius_cells = 1U;
    stub_config.progressive_attachment_search = false;
    stub_config.maximum_attachment_states = 128U;
    stub_config.compact_search_expansions = compact ? 2'048U : 0U;
    auto stub_demand = demand;
    if (reverse) std::reverse(stub_demand.points.begin(), stub_demand.points.end());
    const auto escaped = construct_copper_scaffold(narrow, {stub_demand}, stub_config);
    require(escaped.complete && escaped.counters.outward_attachment_probes > 0U &&
                escaped.counters.outward_attachments > 0U &&
                check_copper_rules(escaped.copper).ok(),
            "source and target outward stubs must survive compact and rich reconstruction "
            "when all original attachments are blocked");
  }

}
void progressive_attachment_search_avoids_eager_attachment_probes() {
  auto board = fixture();
  board.layers.resize(1U);
  board.tracks.push_back({ObjectId{"progressive-blocker"}, NetId{"b"}, LayerId{"F.Cu"},
                          mm(10,5), mm(10,15), Nanometres::from_mm(0.8), false});
  const auto demand = request(NetId{"a"}, mm(2,10), LayerId{"F.Cu"},
                              mm(18,10), LayerId{"F.Cu"});
  CopperScaffoldConfig progressive;
  progressive.progressive_attachment_search = true;
  progressive.grid_pitch = Nanometres::from_mm(0.5);
  progressive.attachment_radius_cells = 2U;
  progressive.maximum_attachment_states = 32U;
  progressive.initial_attachment_states = 1U;
  progressive.expanded_attachment_states = 8U;
  progressive.maximum_expansions_per_connection = 8'000U;
  progressive.compact_search_expansions = 2'048U;
  const auto staged = construct_copper_scaffold(board, {demand}, progressive);

  auto eager_config = progressive;
  eager_config.progressive_attachment_search = false;
  const auto eager = construct_copper_scaffold(board, {demand}, eager_config);
  require(staged.complete && eager.complete && staged.paths.front().points.size() > 2U &&
              check_copper_rules(staged.copper).ok() && check_copper_rules(eager.copper).ok(),
          "the probe comparison must use a searched, DRC-clean detour rather than direct copper");
  require(staged.counters.attachment_search_rounds == 1U &&
              staged.counters.attachment_search_early_successes == 1U &&
              staged.counters.attachment_search_probes < eager.counters.attachment_search_probes,
          "an easy searched connection must finish from the first attachment prefix with fewer probes "
          "than eager attachment generation");
  require(staged.counters.expansions <= progressive.maximum_expansions_per_connection &&
              eager.counters.expansions <= eager_config.maximum_expansions_per_connection,
          "attachment strategy must retain the original total expansion budget");
}
void progressive_attachment_limits_fail_closed_and_clamp_zero_initial_prefix() {
  auto board = fixture();
  board.layers.resize(1U);
  board.tracks.push_back({ObjectId{"attachment-limit-blocker"}, NetId{"b"}, LayerId{"F.Cu"},
                          mm(10,5), mm(10,15), Nanometres::from_mm(0.8), false});
  const auto demand = request(NetId{"a"}, mm(2,10), LayerId{"F.Cu"},
                              mm(18,10), LayerId{"F.Cu"});
  CopperScaffoldConfig disabled;
  disabled.grid_pitch = Nanometres::from_mm(0.5);
  disabled.maximum_attachment_states = 0U;
  const auto rejected = construct_copper_scaffold(board, {demand}, disabled);
  require(!rejected.complete && rejected.paths.empty() && rejected.copper == board &&
              rejected.request_evidence.size() == 1U &&
              rejected.request_evidence.front().termination == CopperScaffoldSearchTermination::attachment_exhausted,
          "a zero attachment-state limit must fail closed after direct copper is rejected");

  CopperScaffoldConfig zero_initial;
  zero_initial.progressive_attachment_search = true;
  zero_initial.grid_pitch = Nanometres::from_mm(0.5);
  zero_initial.maximum_attachment_states = 8U;
  zero_initial.initial_attachment_states = 0U;
  zero_initial.expanded_attachment_states = 0U;
  zero_initial.maximum_attachment_probes_per_round = 1U;
  zero_initial.maximum_expansions_per_connection = 8'000U;
  zero_initial.compact_search_expansions = 2'048U;
  const auto clamped = construct_copper_scaffold(board, {demand}, zero_initial);
  require(clamped.complete && clamped.counters.attachment_search_probes > 0U &&
              clamped.counters.expansions <= zero_initial.maximum_expansions_per_connection &&
              check_copper_rules(clamped.copper).ok(),
          "a zero initial prefix must clamp to a bounded first probe and retain total search limits");
}

} // namespace
int main() {
  via_histories_are_canonical_immutable_and_bounded();
  pad_membership_does_not_accept_copper_on_a_wrong_coarse_layer();
  disconnected_padded_components_are_not_counted_as_connected();
  rejected_goal_assembly_does_not_end_connection_search();
  compact_search_preserves_component_attachment_provenance_within_shared_budget();
  compact_search_falls_back_when_via_history_changes_goal_legality();
  compact_search_cancellation_records_started_work();
  component_geometry_precedes_disconnected_pad_fanout();
  search_uses_a_component_branch_instead_of_fixed_endpoint();
  existing_component_is_a_completed_transaction_without_new_copper();
  retained_branch_can_attach_to_component_interior();
  same_net_on_separate_layers_does_not_prove_connectivity();
  fine_pitch_rotated_pad_edge_access_reaches_the_scaffold_grid();
  progressive_attachment_search_avoids_eager_attachment_probes();
  progressive_attachment_limits_fail_closed_and_clamp_zero_initial_prefix();
  accepts_legal_supplied_topology();
  finds_a_legal_detour_when_the_supplied_line_is_blocked();
  supplies_a_legal_via_transition();
  cancellation_never_publishes_illegal_copper();
  malformed_or_mismatched_retained_paths_are_not_reused();
  duplicate_connection_identity_is_rejected_without_publication();
  failed_causal_exchange_rolls_back_the_legal_subset();
  causal_exchange_preserves_survivors_and_rebuilds_victim();
  bounded_pns_displacement_moves_only_attributed_generated_copper();
  coupled_exchange_rebuilds_two_causal_victims_and_preserves_survivor();
  three_victim_component_succeeds_only_when_the_component_bound_allows_it();
  callback_cancellation_publishes_only_legal_checkpoint();
}
