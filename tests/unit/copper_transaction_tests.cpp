#include "flux/core/copper_transaction.hpp"
#include "flux/core/drc.hpp"

#include <algorithm>
#include <set>
#include <cstdlib>
#include <iostream>

namespace {
using namespace flux;
[[noreturn]] void fail(const char* message) { std::cerr << message << '\n'; std::exit(1); }
void require(bool value, const char* message) { if (!value) fail(message); }
Point mm(double x, double y) { return {Nanometres::from_mm(x), Nanometres::from_mm(y)}; }
Board fixture() {
  Board board;
  board.outline = Polygon{{mm(0, 0), mm(20, 0), mm(20, 20), mm(0, 20)}};
  board.layers = {{LayerId{"F.Cu"}, "F.Cu", LayerKind::copper, 0U}};
  board.nets = {{NetId{"a"}, "a"}, {NetId{"b"}, "b"}};
  board.rules.default_clearance = Nanometres::from_mm(0.2);
  board.tracks = {
      {ObjectId{"a"}, NetId{"a"}, LayerId{"F.Cu"}, mm(2, 4), mm(8, 4), Nanometres::from_mm(0.2), false},
      {ObjectId{"b"}, NetId{"b"}, LayerId{"F.Cu"}, mm(2, 8), mm(8, 8), Nanometres::from_mm(0.2), false}};
  return board;
}
auto signature(const CopperDrcReport& report) {
  std::set<std::pair<std::string, std::vector<std::string>>> result;
  for (auto violation : report.violations) {
    std::sort(violation.object_ids.begin(), violation.object_ids.end());
    result.emplace(violation.code, std::move(violation.object_ids));
  }
  return result;
}
void parity_and_pruning() {
  auto base = fixture();
  base.layers.push_back({LayerId{"B.Cu"}, "B.Cu", LayerKind::copper, 1U});
  base.track_arcs.push_back({ObjectId{"arc"}, NetId{"b"}, LayerId{"F.Cu"},
                            mm(5, 10), mm(7, 8), mm(9, 10), Nanometres::from_mm(.2), false});
  base.vias.push_back({ObjectId{"via"}, NetId{"b"}, mm(12, 10), Nanometres::from_mm(.6),
                       Nanometres::from_mm(.3), {LayerId{"F.Cu"}, LayerId{"B.Cu"}}, false});
  base.track_beziers.push_back({ObjectId{"curve"}, NetId{"b"}, LayerId{"F.Cu"},
      mm(12, 4), mm(14, 4.1), mm(16, 3.9), mm(18, 4), Nanometres::from_mm(.2), false});
  CopperTransactionValidator validator(base);
  bool pruned = false;
  for (double x = 1; x < 18; x += 2.3) for (double y = 1; y < 18; y += 2.7) {
    auto candidate = base;
    candidate.tracks.front().start = mm(x, y);
    candidate.tracks.front().end = mm(x + .7, y + .3);
    auto incremental = validator.validate(candidate);
    require(!incremental.used_full_fallback && incremental.structural.ok(), "valid geometry must use incremental path");
    require(signature(incremental.copper) == signature(check_dynamic_copper_rules(candidate, {ObjectId{"a"}})),
            "incremental moved copper DRC must match complete-board dynamic DRC");
    pruned = pruned || incremental.neighbour_objects < 4U;
    // A second changed object must be checked against the first even though neither is indexed.
    candidate.tracks.back().start = mm(x, y + .1);
    candidate.tracks.back().end = mm(x + .7, y + .1);
    incremental = validator.validate(candidate);
    require(signature(incremental.copper) == signature(check_dynamic_copper_rules(candidate, {ObjectId{"a"}, ObjectId{"b"}})),
            "changed-changed copper pairs must not depend on the baseline index");
  }
  require(pruned, "distant unchanged copper must be spatially pruned");
  auto candidate = base;
  candidate.vias.front().position = mm(6, 8);
  require(signature(validator.validate(candidate).copper) ==
              signature(check_dynamic_copper_rules(candidate, {ObjectId{"via"}})),
          "via movement must preserve exact hole and copper interaction checks");
  candidate = base;
  candidate.tracks.front().net = NetId{"b"};
  candidate.tracks.front().layer = LayerId{"B.Cu"};
  const auto relabelled = validator.validate(candidate);
  require(relabelled.affected_nets == 2U && signature(relabelled.copper) ==
              signature(check_dynamic_copper_rules(candidate, {ObjectId{"a"}})),
          "same-ID net/layer changes must invalidate both old and new connectivity scopes");
}
void connected_deletion_and_replacement() {
  auto base = fixture();
  Footprint footprint; footprint.id = ObjectId{"pads"};
  footprint.courtyard = Polygon{{mm(1, 3), mm(9, 3), mm(9, 5), mm(1, 5)}};
  footprint.pads = {{ObjectId{"p1"}, NetId{"a"}, mm(2, 4), {LayerId{"F.Cu"}}},
                    {ObjectId{"p2"}, NetId{"a"}, mm(8, 4), {LayerId{"F.Cu"}}}};
  base.footprints.push_back(footprint);
  CopperTransactionValidator validator(base);
  auto candidate = base; candidate.tracks.erase(candidate.tracks.begin());
  const auto removed = validator.validate(candidate);
  if (removed.used_full_fallback || removed.preserves_pad_components ||
      removed.after_connectivity.open_connections <= removed.before_connectivity.open_connections) {
    std::cerr << "deletion fallback=" << removed.used_full_fallback << " preserved=" << removed.preserves_pad_components
              << " opens=" << removed.before_connectivity.open_connections << "->" << removed.after_connectivity.open_connections << '\n';
    for (const auto& issue : removed.structural.issues) std::cerr << issue.message << '\n';
  }
  require(!removed.used_full_fallback && !removed.preserves_pad_components &&
              removed.after_connectivity.open_connections > removed.before_connectivity.open_connections,
          "deleting a connecting span must detect the split of an existing pad component");
  candidate.tracks.push_back({ObjectId{"replacement"}, NetId{"a"}, LayerId{"F.Cu"},
                              mm(2, 4), mm(8, 4), Nanometres::from_mm(.2), false});
  const auto replaced = validator.validate(candidate);
  require(replaced.preserves_pad_components && replaced.connectivity_cache_hits == 1U &&
              replaced.after_connectivity.open_connections == 0U && replaced.affected_nets == 1U,
          "equivalent new-ID copper must preserve components and reuse only affected-net baseline connectivity");
  candidate = base; candidate.footprints.front().position = mm(1, 0);
  require(validator.validate(candidate).used_full_fallback, "moving pads must invalidate static geometry");
  candidate = base; candidate.tracks.push_back(candidate.tracks.front());
  require(validator.validate(candidate).used_full_fallback && !validator.validate(candidate).structural.ok(),
          "duplicate canonical IDs must fail closed through full validation");
}
void differential_pair_deletion() {
  auto base = fixture();
  base.differential_pairs = {{DifferentialPairId{"pair"}, "pair", NetId{"a"}, NetId{"b"},
                              std::nullopt, Nanometres::from_mm(.1)}};
  CopperTransactionValidator validator(base);
  auto candidate = base; candidate.tracks.erase(candidate.tracks.begin());
  const auto removed = validator.validate(candidate);
  require(!removed.used_full_fallback && !removed.copper.ok() &&
              signature(removed.copper) == signature(check_copper_rules(candidate)),
          "deletion-only changes must still validate full affected differential-pair length/skew");
}
void incremental_matches_full_dynamic_drc() {
  const auto base = fixture();
  CopperTransactionValidator validator(base);
  auto candidate = base;
  candidate.tracks.front().end = mm(8, 8);
  const auto incremental = validator.validate(candidate);
  const auto full = check_copper_rules(candidate);
  require(!incremental.used_full_fallback && !incremental.copper.ok() && !full.ok(),
          "changed copper must retain its foreign neighbour and report the same illegal interaction");
}
void bezier_neighbour_is_not_lost_by_the_broad_phase() {
  auto base = fixture();
  base.track_beziers = {{ObjectId{"curve"}, NetId{"b"}, LayerId{"F.Cu"}, mm(2, 6), mm(4, 6.05),
                         mm(6, 5.95), mm(8, 6), Nanometres::from_mm(0.2), false}};
  CopperTransactionValidator validator(base);
  auto candidate = base;
  candidate.tracks.front().start = mm(2, 5.7);
  candidate.tracks.front().end = mm(8, 5.7);
  const auto report = validator.validate(candidate);
  require(!report.used_full_fallback && !report.copper.ok(),
          "near-clearance Bezier must remain in the incremental candidate neighbourhood");
}
void deletion_keeps_connectivity_validation_and_static_change_falls_back() {
  const auto base = fixture();
  CopperTransactionValidator validator(base);
  auto removed = base;
  removed.tracks.erase(removed.tracks.begin());
  const auto deletion = validator.validate(removed);
  require(!deletion.used_full_fallback && deletion.changed_objects == 1U &&
              deletion.affected_nets == 1U,
          "deletion must still be represented as an affected incremental transaction");
  auto changed_rule = base;
  changed_rule.rules.default_clearance = Nanometres::from_mm(0.3);
  const auto fallback = validator.validate(changed_rule);
  require(fallback.used_full_fallback,
          "a static rule change must bypass the stale incremental index and use full validation");
}
}
int main() {
  parity_and_pruning();
  connected_deletion_and_replacement();
  differential_pair_deletion();
  incremental_matches_full_dynamic_drc();
  bezier_neighbour_is_not_lost_by_the_broad_phase();
  deletion_keeps_connectivity_validation_and_static_change_falls_back();
}
