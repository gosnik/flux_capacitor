#pragma once

#include "flux/core/board.hpp"
#include "flux/core/global_router.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <stop_token>
#include <string>
#include <string_view>
#include <vector>

namespace flux {

// A deliberately conservative, straight-copper bridge between topology planning and continuous
// realization.  It only publishes complete connections that pass the same dynamic DRC used for
// ordinary proposed copper.  The returned points retain the selected topology; medial evidence
// is cleared because this search is independent of the coarse allocation graph.
struct CopperScaffoldConfig {
  bool enabled{true};
  Nanometres grid_pitch{Nanometres::from_mm(0.1)};
  std::size_t maximum_cells{4'000'000U};
  std::size_t maximum_expansions_per_connection{100'000U};
  std::size_t maximum_victim_exchanges{4U};
  // Retain soft-obstacle passages and legalize a connected group atomically. Static/source
  // copper stays hard; these caps apply per failed request and never authorize a DRC bypass.
  std::size_t maximum_joint_passage_candidates{4U};
  std::size_t maximum_joint_blocker_owners{3U};
  std::size_t maximum_joint_search_expansions{20'000U};
  std::size_t maximum_joint_particles{256U};
  std::size_t maximum_joint_iterations{32U};

  // A small transactional PNS passage probe runs before destructive causal exchange.  It may
  // temporarily omit only attributed generated owners, but commits only an exact full-scene
  // legal shove result.  Zero disables this recovery.
  std::size_t maximum_displacement_attempts{4U};
  std::size_t maximum_displacement_expansions{20'000U};
  std::size_t maximum_displacement_candidate_checks{16U};
  std::size_t maximum_displacement_depth{4U};
  Nanometres maximum_displacement{Nanometres::from_mm(2.0)};
  // Causal component repair is attempted after singleton repair.  Values below two disable it;
  // larger values bound the number of displaced requests in one atomic transaction (internally
  // capped at six to keep combination enumeration bounded).
  std::size_t maximum_coupled_victims{3U};
  std::size_t maximum_coupled_exchange_attempts{8U};
  std::size_t maximum_coupled_expansions{200'000U};
  // A rebound coupled transaction may restore pad components that are not represented by its
  // original coarse requests.  This bounds deterministic, physical pad-to-pad supplement
  // demands; zero leaves the existing transaction behavior unchanged.
  std::size_t maximum_preservation_demands{2U};
  std::size_t maximum_failure_evidence{32U};
  std::size_t maximum_component_access_points{32U};
  std::size_t maximum_attachment_states{128U};
  std::size_t attachment_radius_cells{2U};
  // After every nominal direct and outward attachment fails, a single-pad terminal may use its
  // physical minor dimension for a short, exact-DRC-checked escape.  The maze itself remains at
  // the net's nominal width.  Coupled nets are excluded.
  bool enable_terminal_neckdown{true};
  Nanometres maximum_terminal_neckdown_length{Nanometres::from_mm(2.0)};
  // Start attachment search from small, stable prefixes and widen only after a bounded search
  // fails.  The raw probe cap bounds each cursor advance when few grid candidates are legal.
  // Opt-in until progressive choices preserve benchmark completion and performance.
  bool progressive_attachment_search{false};
  std::size_t initial_attachment_states{8U};
  std::size_t expanded_attachment_states{32U};
  std::size_t maximum_attachment_probes_per_round{256U};
  std::size_t maximum_goal_attempts{64U};
  // After a full forward maze failure, a reversed bounded probe may contribute only sampled
  // exhausted-frontier blocker evidence to the existing causal repair ordering. It never
  // publishes a path or establishes infeasibility; zero disables the diagnostic.
  std::size_t maximum_terminal_pocket_probes{8U};
  std::size_t maximum_terminal_pocket_expansions{2'000U};
  // Optional optimistic node-dominance pass before the provenance-rich search. Zero disables it.
  std::size_t compact_search_expansions{25'000U};
  std::size_t maximum_via_history_nodes{262'144U};
  std::uint64_t planar_step_cost{10U};
  std::uint64_t diagonal_step_cost{14U};
  std::uint64_t via_cost{40U};

  bool operator==(const CopperScaffoldConfig&) const = default;
};

enum class CopperScaffoldSearchTermination {
  not_attempted,
  already_connected,
  direct_route_accepted,
  search_succeeded,
  invalid_request,
  cancelled,
  no_copper_layers,
  missing_board_limits,
  cell_budget_exceeded,
  domain_build_failed,
  attachment_exhausted,
  expansion_budget_exhausted,
  goal_attempt_budget_exhausted,
  search_exhausted,
};

[[nodiscard]] constexpr std::string_view to_string(CopperScaffoldSearchTermination value) {
  switch (value) {
  case CopperScaffoldSearchTermination::not_attempted: return "not_attempted";
  case CopperScaffoldSearchTermination::already_connected: return "already_connected";
  case CopperScaffoldSearchTermination::direct_route_accepted: return "direct_route_accepted";
  case CopperScaffoldSearchTermination::search_succeeded: return "search_succeeded";
  case CopperScaffoldSearchTermination::invalid_request: return "invalid_request";
  case CopperScaffoldSearchTermination::cancelled: return "cancelled";
  case CopperScaffoldSearchTermination::no_copper_layers: return "no_copper_layers";
  case CopperScaffoldSearchTermination::missing_board_limits: return "missing_board_limits";
  case CopperScaffoldSearchTermination::cell_budget_exceeded: return "cell_budget_exceeded";
  case CopperScaffoldSearchTermination::domain_build_failed: return "domain_build_failed";
  case CopperScaffoldSearchTermination::attachment_exhausted: return "attachment_exhausted";
  case CopperScaffoldSearchTermination::expansion_budget_exhausted: return "expansion_budget_exhausted";
  case CopperScaffoldSearchTermination::goal_attempt_budget_exhausted: return "goal_attempt_budget_exhausted";
  case CopperScaffoldSearchTermination::search_exhausted: return "search_exhausted";
  }
  return "unknown";
}

// Evidence is intentionally request-domain based rather than tied to transient scaffold IDs.
// `request_identity` is the canonical identity also present in `request_identities`.
struct CopperScaffoldRequestEvidence {
  std::string request_identity;
  NetId net;
  std::size_t connection_index{0U};
  CopperScaffoldSearchTermination termination{CopperScaffoldSearchTermination::not_attempted};
  std::vector<ObjectId> blocker_objects;
  std::vector<NetId> blocker_nets;
  // Exact physical access telemetry for the primary bounded scaffold search.
  bool access_evaluated{false};
  bool source_endpoint_contained_by_copper{false};
  bool target_endpoint_contained_by_copper{false};
  std::size_t source_component_anchors{0U};
  std::size_t target_component_anchors{0U};
  std::size_t source_legal_attachments{0U};
  std::size_t target_legal_attachments{0U};

  bool operator==(const CopperScaffoldRequestEvidence&) const = default;
};

enum class CopperScaffoldExchangeRollbackReason {
  incoming_unroutable,
  victim_unroutable,
  connectivity_not_preserved,
  cancelled,
};

[[nodiscard]] constexpr std::string_view to_string(CopperScaffoldExchangeRollbackReason value) {
  switch (value) {
  case CopperScaffoldExchangeRollbackReason::incoming_unroutable: return "incoming_unroutable";
  case CopperScaffoldExchangeRollbackReason::victim_unroutable: return "victim_unroutable";
  case CopperScaffoldExchangeRollbackReason::connectivity_not_preserved: return "connectivity_not_preserved";
  case CopperScaffoldExchangeRollbackReason::cancelled: return "cancelled";
  }
  return "unknown";
}

struct CopperScaffoldExchangeRollback {
  std::string request_identity;
  std::vector<std::string> victim_request_identities;
  CopperScaffoldExchangeRollbackReason reason{
      CopperScaffoldExchangeRollbackReason::incoming_unroutable};
  std::vector<ObjectId> blocker_objects;
  std::vector<NetId> blocker_nets;

  std::vector<std::string> disconnected_request_identities;
  bool pad_components_split{false};

  bool operator==(const CopperScaffoldExchangeRollback&) const = default;
};

struct CopperScaffoldCounters {
  std::size_t searched_connections{0U};
  std::size_t routed_connections{0U};
  std::size_t expansions{0U};
  std::size_t compact_search_expansions{0U};
  std::size_t compact_search_attempts{0U};
  std::size_t compact_search_successes{0U};
  std::size_t compact_search_fallbacks{0U};
  std::size_t candidate_rejections{0U};
  std::size_t published_tracks{0U};
  std::size_t published_vias{0U};
  std::size_t victim_exchanges_attempted{0U};
  std::size_t victim_exchanges_accepted{0U};
  std::size_t joint_passage_candidates{0U};
  std::size_t joint_group_attempts{0U};
  std::size_t joint_group_accepted{0U};
  std::size_t joint_search_expansions{0U};
  std::size_t joint_xpbd_invocations{0U};
  std::size_t joint_avbd_invocations{0U};
  std::size_t joint_pns_attempts{0U};
  std::size_t joint_exact_rejections{0U};
  std::size_t joint_budget_exhausted{0U};
  std::size_t displacement_attempts{0U};
  std::size_t displacement_accepted{0U};
  std::size_t displacement_search_expansions{0U};
  std::size_t displacement_search_failures{0U};
  std::size_t displacement_shove_rejections{0U};
  std::size_t displacement_validation_rejections{0U};
  std::size_t displacement_reconstruction_rejections{0U};
  std::size_t displacement_budget_exhausted{0U};
  std::size_t displacement_moved_tracks{0U};
  std::size_t displacement_moved_vias{0U};
  std::size_t coupled_victim_exchanges_attempted{0U};
  std::size_t coupled_victim_exchanges_accepted{0U};
  // Sum of victim counts over attempted/accepted coupled transactions. These distinguish a
  // three-victim component from three pair alternatives in compact telemetry.
  std::size_t coupled_component_victims_attempted{0U};
  std::size_t coupled_component_victims_accepted{0U};
  // Searches stopped by the shared component budget are inconclusive, rather than evidence that
  // the displaced component is infeasible.
  std::size_t coupled_expansion_budget_exhaustions{0U};
  // A failed exchange victim may be rebound from a now-obsolete selected-coarse branch contact
  // to a proved producer pad. Both counts describe only fully bounded transaction-local tries.
  std::size_t transactional_terminal_rebind_attempts{0U};
  std::size_t transactional_terminal_rebinds_accepted{0U};
  std::size_t preservation_repair_attempts{0U};
  std::size_t preservation_demands_accepted{0U};
  std::size_t preservation_repair_rollbacks{0U};
  std::size_t exchange_rollbacks{0U};
  std::size_t exchange_rollbacks_incoming_unroutable{0U};
  std::size_t exchange_rollbacks_victim_unroutable{0U};
  std::size_t exchange_rollbacks_connectivity_not_preserved{0U};
  std::size_t exchange_rollbacks_cancelled{0U};
  std::size_t goal_rejections{0U};
  std::size_t duplicate_rejected_assemblies{0U};
  std::size_t peak_via_history_nodes{0U};
  std::size_t via_history_budget_exhaustions{0U};
  std::size_t source_attachments{0U};
  std::size_t target_attachments{0U};
  std::size_t terminal_pocket_probes{0U};
  std::size_t terminal_pocket_probe_expansions{0U};
  std::size_t terminal_pockets_exhausted{0U};
  std::size_t attachment_search_rounds{0U};
  std::size_t attachment_search_probes{0U};
  std::size_t outward_attachment_probes{0U};
  std::size_t outward_attachments{0U};
  std::size_t terminal_neckdown_probes{0U};
  std::size_t terminal_neckdown_attachments{0U};
  std::size_t attachment_search_early_successes{0U};
  std::uint64_t access_generation_microseconds{0U};
  std::uint64_t domain_build_microseconds{0U};
  std::uint64_t attachment_generation_microseconds{0U};
  std::uint64_t edge_validation_microseconds{0U};
  std::uint64_t exact_goal_validation_microseconds{0U};
  std::uint64_t search_microseconds{0U};
  std::uint64_t terminal_pocket_probe_microseconds{0U};
  std::size_t component_connections{0U};
  std::size_t survivor_tracks_preserved{0U};
  std::size_t survivor_vias_preserved{0U};
  std::size_t completed_transactions{0U};
  bool cancelled{false};
};

// Diagnostic proposal only: blocked passages are never published as legal paths. They are
// retained for bounded retries, always re-attributed and revalidated against the current scene.
struct CopperScaffoldBlockedPassage {
  std::size_t request_index{0U};
  GlobalRoutePath path;
  std::vector<std::size_t> blocker_owners;
  long double clearance_deficit_nm{0};
  long double displacement_space_nm{0};
  long double movement_cost_nm{0};
};

struct CopperScaffoldResult {
  // Stable canonical physical demands. Global-route alternatives may change their interiors,
  // but this domain remains available to later re-planning and failure attribution.
  std::vector<CopperScaffoldBlockedPassage> blocked_passages;
  std::vector<GlobalRoutePath> requests;
  std::vector<GlobalRoutePath> paths;
  Board copper;
  bool complete{false};
  CopperScaffoldCounters counters;
  std::string diagnostic;
  std::vector<std::size_t> missing_indices;
  std::vector<std::size_t> satisfied_indices;
  // Canonical identities in request-index order. They scope retained partial progress: indices
  // are comparable only when this request domain is identical.
  std::vector<std::string> request_identities;
  std::vector<CopperScaffoldRequestEvidence> request_evidence;
  std::vector<CopperScaffoldExchangeRollback> exchange_rollbacks;
  std::size_t request_count{0U};
};

// `retained`, when supplied, is treated as a legal provisional subset produced for this same
// source board. Its paths are revalidated before reuse; mismatched requests are ignored.
[[nodiscard]] CopperScaffoldResult construct_copper_scaffold(
    const Board& board, const std::vector<GlobalRoutePath>& requests,
    const CopperScaffoldConfig& config = {}, std::stop_token stop_token = {},
    const CopperScaffoldResult* retained = nullptr,
    const std::function<void(const CopperScaffoldResult&)>& publish = {});

} // namespace flux
