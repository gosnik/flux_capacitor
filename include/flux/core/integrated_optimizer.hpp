#pragma once

#include "flux/core/detailed_router.hpp"
#include "flux/core/global_placer.hpp"
#include "flux/core/global_router.hpp"
#include "flux/core/placement_move_priority.hpp"

#include <compare>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stop_token>
#include <string>
#include <vector>

namespace flux {

struct IntegratedOptimizerConfig {
  GlobalRouterConfig global_router;
  DetailedRouterConfig detailed_router;
  GlobalPlacerConfig global_placer;
  Nanometres move_step{Nanometres::from_mm(5.0)};
  Nanometres routing_feedback_margin{Nanometres::from_mm(1.0)};
  std::size_t max_passes{4};
  std::size_t max_candidate_evaluations{200};
  // Candidate screening is memory-heavy because each worker owns a board snapshot and routing
  // search state. Keep the default deliberately below typical hardware concurrency.
  std::size_t max_candidate_evaluation_workers{6};
  // Incremental screening may mis-rank candidates relative to a fresh route. Recheck this bounded
  // deterministic shortlist before exact acceptance.
  std::size_t max_full_routing_candidates_per_wave{4};
  // Resume a deterministic local neighborhood after an externally certified prefix. Skipped
  // candidates are never treated as evaluated by this result, so an offset run cannot by itself
  // claim that the complete neighborhood is a local optimum.
  std::size_t candidate_evaluation_offset{0};
  std::size_t max_routing_guided_targets_per_footprint{16};
  std::size_t max_swap_partners_per_footprint{8};
  std::size_t max_group_translation_proposals_per_group{24};
  std::size_t max_relative_component_translation_proposals{24};
  std::size_t max_global_routing_guides{256};
  std::size_t max_orientation_proposals_per_footprint{8};
  std::size_t full_routing_verification_interval{8};
  std::size_t max_pareto_checkpoints{8};
  // Relative alternatives applied to each footprint's current rotation. Empty by default so
  // designer-selected orientations are preserved unless the caller explicitly opts in.
  std::vector<Microdegrees> rotation_offsets;
  // Empty means all unlocked footprints. A non-empty list scopes only orientation proposals;
  // translation and hard constraint behavior are unchanged.
  std::vector<ObjectId> orientation_scope;
  // Optional complete source-bound policy. Search starts with the first class and cumulatively
  // unlocks later classes only when the current wave has no improving exactly accepted move.
  std::optional<PlacementMovePriorityPolicy> placement_move_priorities;
  // True only when the policy was generated from reference-prefix suggestions for a preview run.
  // It does not weaken policy validation and makes post-process classification review mandatory.
  bool suggested_placement_move_priorities{false};
  bool allow_diagonal_moves{true};
  bool enable_side_flips{false};
  bool enable_global_placement{false};
  // Permit explicitly provisional pad/Fab/Silk placement bounds. Every resulting placement must
  // remain marked for post-process human mechanical review.
  bool allow_tentative_placement_envelopes{false};
  // Feed bounded spatial evidence from the routed source checkpoint into the analytical global
  // placement proposal. The routed seed still requires fresh exact acceptance.
  bool enable_global_routing_guidance{false};
  bool enable_routing_guided_moves{true};
  // Evaluate atomic position exchanges between unlocked footprints. Disabled by default because
  // pairwise proposals can consume substantially more routed candidate evaluations.
  bool enable_swap_moves{false};
  // Translate explicitly declared placement groups as rigid atomic transactions.
  bool enable_group_moves{false};
  // Atomically translate the fixed-point closure of explicit relative constraints and any
  // intersecting rigid placement groups.
  bool enable_relative_component_moves{false};
  bool enable_incremental_routing{true};
  // Continue improving route cost after the first fully connected, DRC-clean checkpoint.
  // Disabled by default so repair workflows retain their fast feasibility stop.
  bool optimize_routed_quality{false};
};

// Lexicographic feasibility-first score. The default uses aggregate overflow; the opt-in physical
// objective preserves nanometre, sampled, and corridor dimensions. Explicitly authorized width
// loss follows hard acceptance fields and precedes actual materialized length. Abstract route
// cost, source-orientation, and displacement remain later tie-breakers.
struct IntegratedScore {
  std::size_t placement_violations{0};
  std::size_t global_route_failures{0};
  std::size_t unrouted_nets{0};
  std::size_t failed_pin_escapes{0};
  // Minimum unrouted-net count among retained constructive hard-capacity passes. This precedes
  // soft congestion dimensions so placement cannot improve an overflowed checkpoint by making
  // the closest strictly feasible allocation worse.
  std::size_t best_hard_capacity_unrouted_nets{0};
  bool configured_dimensioned_physical_overflow{false};
  std::uint64_t total_physical_overflow_nm{0};
  std::uint64_t peak_physical_overflow_nm{0};
  std::size_t sampled_overflow{0};
  std::size_t corridor_overflow{0};
  std::size_t congested_physical_resources{0};
  std::size_t global_overflow{0};
  std::size_t copper_violations{0};
  std::size_t open_connections{0};
  std::uint64_t width_reduction_cost{0};
  std::uint64_t routed_length_nm{0};
  std::uint64_t route_cost{0};
  std::size_t side_changes{0};
  std::uint64_t rotation_change_microdegrees{0};
  std::uint64_t displacement_nm{0};

  [[nodiscard]] std::strong_ordering operator<=>(const IntegratedScore& rhs) const;
  [[nodiscard]] bool operator==(const IntegratedScore& rhs) const;
};

enum class PlacementMoveKind {
  global_seed,
  rotation_repair,
  side_flip_repair,
  orientation_repair,
  routing_guided_repair,
  neck_down_obstacle_repair,
  physical_channel_obstacle_repair,
  swap_repair,
  group_repair,
  relative_component_repair,
  local_repair
};

struct PlacementMoveMember {
  ObjectId footprint;
  Point from;
  Point to;

  bool operator==(const PlacementMoveMember&) const = default;
};

struct IntegratedRoutingFeedback {
  std::vector<ObjectId> routing_obstacles;
  std::vector<NetId> affected_nets;
  std::vector<RoutingResourceUsage> routing_pressure;
  std::vector<RoutingCorridorUsage> routing_corridor_pressure;
  std::vector<RoutingVoronoiResourceUsage> routing_physical_pressure;
  std::vector<RoutingNeckDownObstacleImpact> neck_down_obstacles;
  std::vector<RoutingPhysicalObstacleImpact> physical_channel_obstacles;

  bool operator==(const IntegratedRoutingFeedback&) const = default;
};

// Deterministic routing-pressure rank for one footprint. A net contributes at most once even when
// several pads on the footprint share it; pad multiplicity is connectivity, not extra congestion.
struct IntegratedFootprintPressure {
  ObjectId footprint;
  bool bounds_congested_channel{false};
  bool blocks_route{false};
  bool terminates_affected_net{false};
  std::size_t legacy_pressure{0};
  std::uint64_t physical_pressure_nm{0};
  std::size_t sampled_pressure{0};
  std::size_t corridor_pressure{0};
  std::size_t distinct_connected_nets{0};

  bool operator==(const IntegratedFootprintPressure&) const = default;
};

struct PlacementMove {
  ObjectId footprint;
  Point from;
  Point to;
  Microdegrees from_rotation;
  Microdegrees to_rotation;
  BoardSide from_side{BoardSide::front};
  BoardSide to_side{BoardSide::front};
  std::optional<ObjectId> partner_footprint;
  std::optional<Point> partner_from;
  std::optional<Point> partner_to;
  std::optional<PlacementGroupId> placement_group;
  std::vector<RelativePlacementConstraintId> relative_constraints;
  std::vector<PlacementGroupId> constraint_groups;
  std::vector<PlacementMoveMember> companion_moves;
  IntegratedScore before;
  IntegratedScore after;
  std::vector<ObjectId> routing_obstacles;
  std::vector<NetId> affected_nets;
  std::vector<RoutingResourceUsage> routing_pressure;
  std::vector<RoutingCorridorUsage> routing_corridor_pressure;
  std::vector<RoutingVoronoiResourceUsage> routing_physical_pressure;
  std::vector<RoutingNeckDownObstacleImpact> neck_down_obstacles;
  std::vector<RoutingPhysicalObstacleImpact> physical_channel_obstacles;
  // Highest tier required by every footprint moved in this atomic transaction.
  std::optional<PlacementMovePriorityClass> priority_class;
  PlacementMoveKind kind{PlacementMoveKind::local_repair};
};

struct PlacementMovePriorityMetrics {
  PlacementMovePriorityClass priority_class{PlacementMovePriorityClass::other};
  std::size_t footprints{0};
  std::size_t proposals{0};
  std::size_t candidate_evaluations{0};
  std::size_t accepted_moves{0};
};

// A fully materialized, connectivity- and DRC-clean trade-off certified by a fresh full route.
struct ParetoCheckpoint {
  Board candidate;
  IntegratedScore score;
  std::size_t full_route_evaluation{0};
};

enum class IntegratedOptimizerStopReason {
  complete,
  no_improving_move,
  pass_limit,
  evaluation_limit,
  invalid_config,
  cancelled
};

struct IntegratedOptimizationResult {
  std::optional<Board> candidate;
  IntegratedScore score;
  GlobalRouteResult global_route;
  DetailedRouteResult detailed_route;
  std::optional<GlobalPlacementResult> global_placement;
  std::vector<PlacementMove> accepted_moves;
  std::vector<ParetoCheckpoint> pareto_archive;
  std::size_t pareto_candidates_discarded{0};
  std::size_t configured_max_passes{0};
  std::size_t configured_max_candidate_evaluations{0};
  std::size_t configured_max_candidate_evaluation_workers{0};
  std::size_t configured_max_full_routing_candidates_per_wave{0};
  std::size_t configured_candidate_evaluation_offset{0};
  std::size_t candidate_evaluations_skipped{0};
  Nanometres configured_move_step;
  std::size_t candidate_evaluations{0};
  std::size_t candidate_evaluation_waves{0};
  std::size_t candidate_evaluation_tasks_dispatched{0};
  std::size_t peak_candidate_evaluation_workers{0};
  std::size_t shortlisted_full_route_evaluations{0};
  std::size_t board_candidate_validations{0};
  std::size_t reused_static_zone_validations{0};
  std::size_t incremental_route_evaluations{0};
  std::size_t full_route_evaluations{0};
  std::size_t checkpoint_paths_reused{0};
  std::size_t incremental_detailed_evaluations{0};
  std::size_t checkpoint_tracks_reused{0};
  std::size_t checkpoint_vias_reused{0};
  std::size_t detailed_reuse_fallbacks{0};
  std::size_t tentative_placement_envelopes{0};
  bool human_placement_review_required{false};
  bool configured_placement_move_priorities{false};
  bool suggested_placement_move_priorities{false};
  bool placement_move_priority_review_required{false};
  std::size_t placement_priority_tiers_unlocked{0};
  std::size_t placement_priority_escalations{0};
  std::vector<PlacementMovePriorityMetrics> placement_priority_metrics;
  std::vector<IntegratedFootprintPressure> footprint_pressure;
  std::size_t neck_down_obstacle_move_proposals{0};
  std::size_t neck_down_obstacle_candidate_evaluations{0};
  std::size_t neck_down_obstacle_moves_accepted{0};
  std::size_t physical_channel_obstacle_move_proposals{0};
  std::size_t physical_channel_obstacle_candidate_evaluations{0};
  std::size_t physical_channel_obstacle_moves_accepted{0};
  bool global_seed_accepted{false};
  IntegratedOptimizerStopReason stop_reason{IntegratedOptimizerStopReason::invalid_config};
  std::string diagnostic;

  [[nodiscard]] bool complete() const {
    return stop_reason == IntegratedOptimizerStopReason::complete;
  }
  [[nodiscard]] bool feasible() const { return detailed_route.complete(); }
};

// Collect deterministic, unit-separated routing pressure for move provenance and UI explanation.
[[nodiscard]] IntegratedRoutingFeedback
collect_integrated_routing_feedback(const GlobalRouteResult& route);

// Rank every footprint using stable route evidence. Blocking objects precede affected terminals,
// then the active aggregate/dimensioned pressure tuple and stable footprint ID.
[[nodiscard]] std::vector<IntegratedFootprintPressure> collect_integrated_footprint_pressure(
    const Board& board, const GlobalRouteResult& route,
    GlobalRouteCheckpointObjective objective = GlobalRouteCheckpointObjective::aggregate_overflow);

// Bounded deterministic placement/routing optimization driven by actual global routing
// and detailed copper checks. An optional global seed and all local repairs are proposals;
// the source board is immutable and locked footprints never move.
[[nodiscard]] IntegratedOptimizationResult
optimize_integrated_placement_routing(const Board& source,
                                      const IntegratedOptimizerConfig& config = {},
                                      std::stop_token stop_token = {},
                                      RouterTelemetrySink* telemetry = nullptr);

} // namespace flux
