#pragma once

#include "flux/core/board.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <stop_token>
#include <string>
#include <vector>

namespace flux {

// Bounded HC-CBS-inspired composition for *static* finite regional assignments.  It composes
// already generated alternatives; it does not model time or claim optimality.  All returned
// assignments remain proposals until the regional access, packing, and copper gates accept them.
struct RegionalHcSemanticContext {
  // These are canonical, equality-compared serializations supplied by the caller.  They must
  // contain the complete board copper/obstacle scene, rules, endpoint geometry, access
  // constraints, relevant options, and stable candidate geometry identities respectively.
  // Fingerprints are intentionally not an identity or collision guard.
  // The complete canonical board is compared by value for exact cache reuse. Keeping it behind
  // shared ownership stores one immutable scene per cache entry rather than copying it for each
  // child alternative. `board_copper` is supplemental caller evidence for adapters without a
  // canonical board snapshot.
  std::shared_ptr<const Board> board_scene;
  std::string board_copper;
  std::string rules;
  std::string terminal_geometry;
  std::string access_constraints;
  // Any additional input consulted by the compatibility callback must be serialized here. The
  // compositor revalidates cached rows in all cases, so this is a semantic guard rather than a
  // promise that an opaque callback remains unchanged.
  std::string compatibility_constraints;
  std::string options;
  std::vector<std::string> candidate_geometry;

  bool operator==(const RegionalHcSemanticContext& other) const;
  [[nodiscard]] bool same_candidate_geometry(const RegionalHcSemanticContext& other) const;
  [[nodiscard]] bool same_endpoints_and_rules(const RegionalHcSemanticContext& other) const;
};

struct RegionalHcAlternative {
  // Stable geometry identity, never an ordinal from a regenerated domain.
  std::string identity;
  std::uint64_t cost{0U};
};

struct RegionalHcChildGroup {
  std::string identity;
  std::vector<RegionalHcAlternative> alternatives;
};

struct RegionalHcCompositionBounds {
  std::size_t max_nodes{256U};
  std::size_t max_frontier{256U};
  std::size_t max_retained_per_group{4U};
  std::size_t max_results{16U};
  std::size_t max_cache_entries{8U};
  std::size_t max_compatibility_checks{4'096U};
  std::size_t max_input_alternatives{4'096U};
  std::size_t max_input_groups{64U};
  std::size_t max_candidate_identity_bytes{1U << 20U};
  std::size_t max_cache_bytes{4U << 20U};
};

enum class RegionalHcCompositionOutcome { complete, infeasible, unknown };
enum class RegionalHcCacheDisposition {
  rebuilt,
  exact_reused,
  stale_revalidated,
  stale_rebuilt,
  invalidated
};

struct RegionalHcCompositionStats {
  std::size_t retained_children{0U};
  std::size_t interaction_edges{0U};
  std::size_t nodes_expanded{0U};
  std::size_t frontier_peak{0U};
  std::size_t compatibility_checks{0U};
  std::size_t compatibility_rejections{0U};
  std::size_t stale_candidates_seen{0U};
  std::size_t stale_candidates_revalidated{0U};
  std::size_t stale_candidates_rejected{0U};
  std::size_t seed_assignments_generated{0U};
  std::size_t seed_assignments_admitted{0U};
  std::size_t cache_hits{0U};
  std::size_t cache_misses{0U};
  bool node_bound_hit{false};
  bool frontier_bound_hit{false};
  bool memory_bound_hit{false};
  bool child_bound_hit{false};
  bool compatibility_bound_hit{false};
  bool cancelled{false};
  RegionalHcCacheDisposition cache_disposition{RegionalHcCacheDisposition::rebuilt};
  bool operator==(const RegionalHcCompositionStats&) const = default;
};

struct RegionalHcCompositionResult {
  // Each row contains an alternative index for every input child group, in input order.
  std::vector<std::vector<std::size_t>> assignments;
  RegionalHcCompositionOutcome outcome{RegionalHcCompositionOutcome::unknown};
  RegionalHcCompositionStats stats;
};

using RegionalHcCompatibility =
    std::function<bool(const RegionalHcAlternative&, const RegionalHcAlternative&)>;

// Thread-safe bounded retention. Failed searches and no-good proofs are deliberately never
// retained: a finite/bounded composition failure is not a permanent-copper infeasibility proof.
class RegionalHcCompositionCache {
public:
  RegionalHcCompositionCache();
  ~RegionalHcCompositionCache();
  RegionalHcCompositionCache(const RegionalHcCompositionCache&) = delete;
  RegionalHcCompositionCache& operator=(const RegionalHcCompositionCache&) = delete;

private:
  struct Entry;
  std::mutex mutex_;
  std::vector<Entry> entries_;
  friend RegionalHcCompositionResult compose_regional_hc_assignments(
      const RegionalHcSemanticContext&, const std::vector<RegionalHcChildGroup>&,
      const RegionalHcCompositionBounds&, const RegionalHcCompatibility&,
      RegionalHcCompositionCache*, std::stop_token);
};

[[nodiscard]] RegionalHcCompositionResult compose_regional_hc_assignments(
    const RegionalHcSemanticContext& context, const std::vector<RegionalHcChildGroup>& groups,
    const RegionalHcCompositionBounds& bounds, const RegionalHcCompatibility& compatible,
    RegionalHcCompositionCache* cache = nullptr, std::stop_token stop_token = {});

// Conservative retained-memory estimate for a copied canonical scene. `limit + 1` means the
// estimate reached the caller's admission limit; it is never used as an RSS measurement.
[[nodiscard]] std::size_t estimate_regional_hc_scene_bytes(const Board& board, std::size_t limit);

} // namespace flux
