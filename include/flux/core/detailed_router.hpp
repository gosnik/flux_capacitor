#pragma once

#include "flux/core/connectivity.hpp"
#include "flux/core/drc.hpp"
#include "flux/core/global_router.hpp"
#include "flux/core/regional_router.hpp"
#include "flux/core/validation.hpp"

#include <cstddef>
#include <optional>
#include <stop_token>
#include <string>
#include <vector>

namespace flux {

enum class ExistingRoutingPolicy { preserve, replace_unlocked_for_routed_nets };

struct DetailedRouterConfig {
  // Diagnostic-only counterpart to the global-router escape hatch. The source validation gate is
  // bypassed so a candidate can be materialized for inspection; final validation, connectivity,
  // and copper DRC are still measured and prevent the result from becoming complete.
  bool allow_invalid_source_for_diagnostic_preview{false};
  Nanometres track_width{Nanometres{0}};
  Nanometres via_diameter{Nanometres{0}};
  Nanometres via_drill{Nanometres{0}};
  ExistingRoutingPolicy existing_routing{ExistingRoutingPolicy::preserve};
  // Null selects every net materialized by the global plan (the direct CLI behavior). A present
  // vector is an exact replacement authority and may be empty. It must be unique, source-valid,
  // and a subset of routed nets. Locked objects remain immutable in either case.
  std::optional<std::vector<NetId>> existing_routing_replacement_scope;
  // Check detailed copper without saved zone-fill polygons, then return a provisional result that
  // requires external KiCad refill, strict re-import, connectivity, and DRC before acceptance.
  bool defer_saved_zone_fill_clearance_to_refill{false};
  // New copper must use horizontal, vertical or 45-degree sections. Existing protected copper
  // remains immutable; failed exact-clearance conversion prevents completion.
  bool restrict_straight_track_angles{false};
  bool pull_tight_45_degrees{true};
  bool continuous_refinement{true};
  bool topology_locked_medial_materialization{false};
  bool circular_arc_refinement{false};
  // Remove bounded, generated straight-copper leaf chains that terminate at no physical
  // pad/via/source attachment before curve fitting.  Exact validation and connectivity guards
  // own every deletion; protected, coupled, locked and source routing is excluded.
  bool enable_generated_tail_cleanup{true};
  std::size_t max_generated_tail_cleanup_candidates{256};
  std::size_t max_generated_tail_cleanup_split_tracks{4096};
  // After the PH/PNS realization has committed, make one serial, residual-only pass over
  // remaining eligible straight-track bends using the same exact fillet transaction. This never
  // changes the final bend audit or touches locked/protected copper.
  bool residual_bend_repair{true};
  // Each residual batch visits at most this many frozen chains and eligible corners. Batches
  // resume their deterministic cursor, so skipped geometry never repeatedly starves later bends.
  std::size_t max_residual_bend_repair_batches{32};
  std::size_t max_residual_bend_repair_runs{32};
  std::size_t max_residual_bend_repair_corners{128};
  bool differential_pair_skew_tuning{false};
  bool differential_pair_skew_tuning_batch_assignment{false};
  // Permit a topology-complete global plan with only bounded, explicitly grouped residual
  // resource conflicts to enter detailed spline/PNS repair. This never relaxes final validation,
  // connectivity, or copper DRC.
  bool bounded_residual_conflict_refinement{true};
  std::size_t max_residual_conflict_groups{8};
  std::size_t max_residual_conflict_nets_per_group{16};
  std::size_t max_residual_sampled_overflow{32};
  std::size_t max_residual_clearance_shadow_overflow{32};
  std::size_t max_residual_joint_candidate_states{2};
  std::size_t max_residual_joint_optimization_sweeps{2};
  std::size_t continuous_refinement_iterations{12};
  std::size_t continuous_backtracking_steps{8};
  std::size_t max_continuous_exact_candidate_checks{4096};
  // Try the maximum legal-radius fillet first, then halve the radius this many times when exact
  // represented copper checks reject it. Zero retains one maximum-radius attempt.
  std::size_t circular_arc_backtracking_steps{8};
  // Curved routing uses an open clamped quintic PH curve, adaptive tessellation/native arc fitting,
  // collision-envelope attribution, bounded PNS shove, and a control-corridor refit. Allocation
  // remains polyline based and every quantized result remains transactional.
  Nanometres spline_tessellation_tolerance{Nanometres::from_mm(0.02)};
  Nanometres spline_arc_fit_tolerance{Nanometres::from_mm(0.02)};
  long double spline_max_join_angle_radians{0.035L};
  Nanometres spline_control_corridor{Nanometres::from_mm(2.0)};
  Nanometres spline_shove_step{Nanometres::from_mm(0.1)};
  // Detailed continuous realization needs sub-global-grid corridor geometry; otherwise a legal
  // 0.5 mm allocation jog can collapse or disconnect when represented by one equally sized cell.
  Nanometres spline_navmesh_cell_pitch{Nanometres::from_mm(0.25)};
  Nanometres spline_minimum_bend_radius{Nanometres::from_mm(0.25)};
  std::size_t max_spline_navmesh_cells_per_run{100'000};
  std::size_t max_spline_navmesh_search_expansions_per_run{100'000};
  std::size_t spline_trajectory_samples{48};
  std::size_t max_spline_trajectory_iterations{24};
  std::size_t max_spline_trajectory_candidate_checks{4096};
  bool prefer_gpu_spline_trajectory{true};
  // After topology/clearance optimization, make one bounded attempt to promote the selected
  // tangent-continuous PH chain to curvature continuity before fitting native KiCad copper.
  // Failure preserves the already legal G1 chain; it never withholds an otherwise valid route.
  bool prefer_g2_ph_splines{true};
  std::size_t spline_gpu_precondition_iterations{32};
  std::size_t spline_gpu_polish_shortlist{4};
  // Exact candidates within one coordinate-descent wave are deliberately serial: recursive
  // certificates have severe long-tail imbalance. Parallelism belongs at the independent route
  // realization boundary where each task has enough work to amortize scheduling and board copies.
  std::size_t spline_exact_candidate_workers{1};
  std::size_t spline_route_workers{4};
  // One bounded retry of still-unrounded chains after neighbouring curve transactions commit.
  // No-progress passes stop immediately; accepted copper remains immutable within each batch.
  bool retry_failed_spline_routes_after_commits{true};
  std::size_t spline_max_tessellation_depth{10};
  std::size_t max_spline_samples_per_run{4096};
  std::size_t max_spline_fitted_pieces_per_run{4096};
  std::size_t max_spline_arc_samples{24};
  std::size_t max_spline_shove_iterations{12};
  std::size_t max_spline_candidate_checks{256};
  std::size_t max_pns_propagation_depth{8};
  std::size_t max_pns_candidate_checks{64};
  Nanometres maximum_pns_displacement{Nanometres::from_mm(2.0)};
  Nanometres continuous_trust_region{Nanometres{0}};
  Nanometres maximum_circular_arc_radius{Nanometres::from_mm(1.0)};
  Nanometres differential_pair_skew_tuning_step{Nanometres::from_mm(0.05)};
  Nanometres maximum_differential_pair_skew_tuning_amplitude{Nanometres::from_mm(5.0)};
  Nanometres minimum_differential_pair_skew_tuning_lead{Nanometres::from_mm(0.5)};
  Nanometres minimum_differential_pair_skew_tuning_spacing{Nanometres::from_mm(0.5)};
  std::size_t max_circular_arcs{4096};
  std::size_t max_circular_arc_candidate_checks{16'384};
  std::size_t max_differential_pair_skew_tuning_candidates{4096};
  std::size_t max_differential_pair_skew_tuning_periods{8};
  std::size_t max_differential_pair_skew_tuning_assignment_candidates_per_pair{8};
  std::size_t max_differential_pair_skew_tuning_assignment_states{4096};
  std::size_t max_topology_locked_terminal_access_points{4096};
  std::size_t max_coupled_terminal_fanout_candidates{16};
  std::size_t max_shared_resource_fanout_candidates{64};
  std::size_t max_topology_locked_subset_repair_candidates{16};

  bool operator==(const DetailedRouterConfig&) const = default;
};

enum class DetailedRouterStopReason {
  complete,
  invalid_input,
  incomplete_global_route,
  invalid_path,
  structural_violations,
  connectivity_incomplete,
  copper_violations,
  zone_refill_required,
  cancelled
};

struct SharedResourceFanoutFailureMember {
  NetId net;
  Point start_anchor;
  Point start_lane;
  Point end_lane;
  Point end_anchor;

  bool operator==(const SharedResourceFanoutFailureMember&) const = default;
};

struct SharedResourceFanoutFailure {
  std::size_t resource_id{0};
  LayerId layer;
  Point start;
  Point end;
  std::vector<NetId> nets;
  std::vector<NetId> conflicting_nets;
  std::vector<NetId> fallback_nets;
  std::vector<SharedResourceFanoutFailureMember> members;

  bool operator==(const SharedResourceFanoutFailure&) const = default;
};

struct DetailedRouteResult {
  bool reused_validated_regional_scene{false};
  std::optional<Board> candidate;
  ValidationReport validation;
  ConnectivityReport connectivity;
  CopperDrcReport copper_drc;
  std::vector<RoutabilityDiagnosis> routability_diagnoses;
  bool final_routability_feedback_attempted{false};
  bool final_routability_feedback_selected{false};
  std::size_t final_routability_feedback_passes{0};
  std::size_t final_routability_feedback_improvements{0};
  std::size_t final_routability_diagnoses_consumed{0};
  std::size_t final_routability_local_geometry_passes{0};
  std::size_t final_routability_regional_passes{0};
  std::size_t final_routability_global_passes{0};
  std::size_t final_routability_repeated_failure_escalations{0};
  std::size_t final_routability_incremental_detailed_passes{0};
  std::size_t added_tracks{0};
  std::size_t added_track_arcs{0};
  std::size_t added_track_beziers{0};
  std::size_t unrounded_non_junction_bends{0};
  std::size_t unrounded_straight_track_bends{0};
  std::size_t unrounded_curve_tangent_bends{0};
  std::size_t residual_bend_repair_attempted{0};
  std::size_t residual_bend_repair_accepted{0};
  std::size_t residual_bend_repair_rejected{0};
  std::size_t residual_bend_repair_batches{0};
  std::size_t residual_bend_repair_scanned_runs{0};
  std::size_t residual_bend_repair_scanned_points{0};
  std::size_t residual_bend_repair_run_budget_exhaustions{0};
  std::size_t residual_bend_repair_corner_budget_exhaustions{0};
  bool residual_bend_repair_budget_exhausted{false};
  std::size_t generated_tail_cleanup_candidates{0};
  std::size_t generated_tail_cleanup_removed_chains{0};
  std::size_t generated_tail_cleanup_removed_tracks{0};
  std::size_t generated_tail_cleanup_skipped_protected{0};
  bool generated_tail_cleanup_budget_exhausted{false};
  std::size_t spline_committed_context_retry_tasks{0};
  std::size_t spline_committed_context_retry_commits{0};
  std::size_t added_vias{0};
  std::size_t removed_tracks{0};
  std::size_t removed_track_arcs{0};
  std::size_t removed_track_beziers{0};
  std::size_t removed_vias{0};
  std::size_t reused_tracks{0};
  std::size_t reused_track_arcs{0};
  std::size_t reused_track_beziers{0};
  std::size_t reused_vias{0};
  std::size_t straight_angle_candidate_checks{0};
  std::size_t straight_angle_segments_converted{0};
  std::size_t straight_angle_segments_unresolved{0};
  std::size_t pulled_tight_corners{0};
  std::size_t continuously_refined_points{0};
  std::size_t continuous_refinement_iterations{0};
  std::size_t attempted_continuous_refinement_candidates{0};
  std::size_t accepted_continuous_refinement_candidates{0};
  std::size_t rejected_continuous_refinement_candidates{0};
  std::size_t withheld_continuous_refinement_candidates{0};
  std::size_t materialized_terminal_neck_down_paths{0};
  std::size_t materialized_terminal_neck_down_segments{0};
  std::size_t materialized_medial_neck_down_paths{0};
  std::size_t materialized_medial_neck_down_segments{0};
  std::size_t materialized_corridor_neck_down_paths{0};
  std::size_t materialized_corridor_neck_down_segments{0};
  std::size_t width_aware_simplified_points{0};
  std::size_t width_aware_pulled_tight_paths{0};
  std::size_t width_aware_pulled_tight_corners{0};
  std::size_t width_aware_continuously_refined_paths{0};
  std::size_t width_aware_continuously_refined_points{0};
  std::size_t attempted_topology_locked_paths{0};
  std::size_t materialized_topology_locked_paths{0};
  std::size_t materialized_multilayer_topology_locked_paths{0};
  std::size_t topology_locked_path_fallbacks{0};
  std::size_t topology_locked_rejection_copper_violations{0};
  std::string topology_locked_rejection_first_violation;
  std::size_t preserved_topology_locked_terminal_access_paths{0};
  std::size_t attempted_coupled_terminal_fanout_groups{0};
  std::size_t attempted_coupled_terminal_fanout_candidates{0};
  std::size_t materialized_coupled_terminal_fanout_groups{0};
  std::size_t synchronized_coupled_terminal_fanouts{0};
  std::size_t coupled_terminal_fanout_fallbacks{0};
  std::size_t attempted_shared_resource_fanout_groups{0};
  std::size_t attempted_shared_resource_fanout_candidates{0};
  std::size_t materialized_shared_resource_fanout_groups{0};
  std::size_t synchronized_shared_resource_fanouts{0};
  std::size_t shared_resource_fanout_fallbacks{0};
  std::vector<SharedResourceFanoutFailure> shared_resource_fanout_failures;
  std::size_t attempted_coupled_multilayer_groups{0};
  std::size_t materialized_coupled_multilayer_groups{0};
  std::size_t synchronized_coupled_via_arrays{0};
  std::size_t coupled_multilayer_group_fallbacks{0};
  std::size_t attempted_circular_arc_corners{0};
  std::size_t attempted_circular_arc_candidates{0};
  std::size_t rejected_circular_arc_candidates{0};
  std::size_t fitted_circular_arcs{0};
  std::size_t fitted_cubic_beziers{0};
  std::size_t radius_reduced_circular_arcs{0};
  std::size_t withheld_circular_arc_corners{0};
  bool circular_arc_candidate_budget_exhausted{false};
  std::size_t width_aware_circular_arc_paths{0};
  std::size_t width_aware_fitted_circular_arcs{0};
  std::size_t attempted_quintic_ph_runs{0};
  std::size_t fitted_quintic_ph_runs{0};
  std::size_t spline_g2_promotion_attempts{0};
  std::size_t spline_g2_promotions{0};
  std::size_t spline_g2_fallbacks{0};
  std::uint64_t spline_g2_promotion_microseconds{0U};
  std::size_t regional_native_spline_runs_materialized{0};
  std::size_t regional_native_track_segments_materialized{0};
  std::size_t regional_native_track_arcs_materialized{0};
  std::size_t regional_native_track_beziers_materialized{0};
  std::size_t adaptive_spline_tessellation_samples{0};
  std::size_t spline_fitted_track_segments{0};
  std::size_t spline_collision_envelopes{0};
  std::size_t spline_control_point_shoves{0};
  std::size_t spline_refits{0};
  std::size_t spline_navmesh_routes{0};
  std::size_t spline_navmesh_failures{0};
  std::size_t spline_navmesh_refined_parents{0};
  std::size_t spline_navmesh_refined_leaves{0};
  std::size_t spline_navmesh_rejected_leaves{0};
  std::size_t spline_navmesh_gpu_wavefront_routes{0};
  std::size_t spline_navmesh_gpu_wavefront_cpu_fallbacks{0};
  std::size_t spline_navmesh_certificate_fallbacks{0};
  std::size_t spline_navmesh_topology_signatures{0};
  std::size_t spline_navmesh_topology_gates{0};
  std::size_t spline_navmesh_topology_rejections{0};
  std::size_t spline_funnel_points{0};
  std::size_t spline_seed_candidates{0};
  std::size_t spline_seed_exact_legal{0};
  std::size_t spline_run_aligned_seed_candidates{0};
  std::size_t spline_run_aligned_seed_exact_legal{0};
  std::size_t spline_seed_portfolio_improvements{0};
  std::size_t spline_trajectory_attempts{0};
  std::size_t spline_trajectory_accepted{0};
  std::size_t spline_trajectory_control_moves{0};
  std::size_t spline_trajectory_candidate_checks{0};
  std::size_t spline_paired_obstacle_support_attempts{0};
  std::size_t spline_paired_obstacle_supports_inserted{0};
  std::size_t spline_paired_obstacle_support_recoveries{0};
  std::size_t spline_multistart_attempts{0};
  std::size_t spline_multistart_recoveries{0};
  std::size_t spline_gpu_precondition_attempts{0};
  std::size_t spline_gpu_precondition_dispatches{0};
  std::size_t spline_gpu_precondition_cpu_fallbacks{0};
  std::size_t spline_gpu_precondition_accepted{0};
  std::size_t spline_gpu_precondition_legal{0};
  std::uint64_t spline_gpu_precondition_solve_us{0U};
  std::size_t spline_gpu_polish_batches{0};
  std::size_t spline_gpu_polish_dispatches{0};
  std::size_t spline_gpu_polish_cpu_fallbacks{0};
  std::size_t spline_gpu_polish_candidates{0};
  std::size_t spline_gpu_polish_exact_shortlist_checks{0};
  std::size_t spline_parallel_exact_waves{0};
  std::size_t spline_parallel_exact_checks{0};
  std::size_t spline_parallel_route_tasks{0};
  std::size_t spline_parallel_route_workers{0};
  std::size_t spline_parallel_route_candidates{0};
  std::size_t spline_parallel_route_commits{0};
  std::size_t spline_parallel_route_merge_rejections{0};
  std::size_t spline_trajectory_initial_violations{0};
  std::size_t spline_trajectory_final_violations{0};
  std::size_t spline_trajectory_initial_shape_violations{0};
  std::size_t spline_trajectory_final_shape_violations{0};
  std::size_t spline_trajectory_final_self_intersections{0};
  std::size_t spline_trajectory_final_cusps{0};
  std::size_t spline_trajectory_final_control_station_reversals{0};
  std::size_t spline_trajectory_final_station_reversals{0};
  std::size_t spline_trajectory_final_excess_turn_violations{0};
  std::size_t spline_trajectory_final_unintended_inflections{0};
  std::size_t spline_trajectory_final_bend_radius_violations{0};
  std::size_t spline_trajectory_initial_continuous_violations{0};
  std::size_t spline_trajectory_final_continuous_violations{0};
  std::size_t spline_continuous_hulls_checked{0};
  std::size_t spline_continuous_envelope_rejections{0};
  std::size_t spline_continuous_budget_exhaustions{0};
  std::size_t spline_final_validation_rejections{0};
  std::size_t spline_final_copper_rejections{0};
  std::size_t spline_final_connectivity_rejections{0};
  std::string spline_navmesh_last_diagnostic;
  std::string spline_navmesh_last_topology_diagnostic;
  std::string spline_last_materialization_diagnostic;
  std::vector<std::string> spline_route_failure_details;
  std::size_t pns_shove_attempts{0};
  std::size_t pns_shove_candidates{0};
  std::size_t accepted_pns_shoves{0};
  std::size_t pns_moved_tracks{0};
  std::size_t pns_moved_vias{0};
  std::size_t pns_added_dogleg_tracks{0};
  bool residual_conflict_handoff_attempted{false};
  std::size_t residual_conflict_groups_attempted{0};
  std::size_t residual_conflict_nets_attempted{0};
  std::size_t residual_sampled_overflow_attempted{0};
  std::size_t residual_clearance_shadow_overflow_attempted{0};
  std::size_t residual_joint_candidate_states{0};
  std::size_t residual_joint_groups_optimized{0};
  std::size_t residual_joint_spline_routes{0};
  std::size_t residual_joint_optimization_sweeps{0};
  std::size_t residual_joint_control_moves{0};
  std::size_t residual_joint_exact_transactions_rejected{0};
  bool residual_joint_transaction_accepted{false};
  bool regional_candidate_ranker_requested{false};
  bool regional_candidate_ranker_loaded{false};
  std::size_t regional_candidate_ranker_trees{0};
  std::size_t regional_candidate_ranker_groups_scored{0};
  std::size_t regional_candidate_ranker_candidates_scored{0};
  std::size_t regional_candidate_ranker_candidates_reordered{0};
  std::size_t regional_candidate_ranker_training_rows_written{0};
  std::size_t regional_candidate_ranker_training_rows_failed{0};
  std::string regional_candidate_ranker_diagnostic;
  bool spline_candidate_budget_exhausted{false};
  bool pns_candidate_budget_exhausted{false};
  std::size_t attempted_differential_pair_skew_tuning_pairs{0};
  std::size_t attempted_differential_pair_skew_tuning_candidates{0};
  std::size_t tuned_differential_pairs{0};
  std::size_t withheld_differential_pair_skew_tuning_pairs{0};
  std::size_t assessed_differential_pair_skew_tuning_regions{0};
  std::size_t assessed_any_direction_differential_pair_skew_tuning_regions{0};
  std::size_t withheld_differential_pair_skew_tuning_regions{0};
  std::size_t withheld_differential_pair_skew_tuning_free_space_candidates{0};
  std::size_t withheld_differential_pair_skew_tuning_quantization_candidates{0};
  std::size_t maximum_differential_pair_skew_tuning_period_capacity{0};
  std::size_t materialized_differential_pair_skew_tuning_periods{0};
  std::size_t materialized_any_direction_differential_pair_skew_tuning_patterns{0};
  std::size_t legal_differential_pair_skew_tuning_candidates{0};
  std::size_t differential_pair_skew_tuning_assignment_states{0};
  std::size_t rejected_differential_pair_skew_tuning_assignment_conflicts{0};
  std::uint64_t added_differential_pair_skew_tuning_length_nm{0};
  std::vector<NetId> reused_nets;
  std::vector<NetId> rematerialized_nets;
  std::vector<NetId> invalidated_reuse_nets;
  bool fallback_full_materialization{false};
  bool fallback_orthogonal_materialization{false};
  bool fallback_discrete_materialization{false};
  bool continuous_refinement_budget_exhausted{false};
  bool differential_pair_skew_tuning_budget_exhausted{false};
  bool differential_pair_skew_tuning_assignment_attempted{false};
  bool differential_pair_skew_tuning_assignment_budget_exhausted{false};
  bool fallback_topology_locked_materialization{false};
  bool fallback_topology_locked_subset_materialization{false};
  std::size_t attempted_topology_locked_subset_repair_nets{0};
  std::size_t attempted_topology_locked_subset_repair_candidates{0};
  std::size_t retained_topology_locked_subset_paths{0};
  bool fallback_polyline_materialization{false};
  // A production workflow may discard a complete hybrid global plan whose detailed copper fails
  // whole-board DRC, then retry once with only the sampled graph. The legacy direct-medial field
  // names remain wire-compatible. These fields describe that outer transaction;
  // materialize_detailed_route never sets them itself.
  bool direct_medial_sampled_fallback_attempted{false};
  bool direct_medial_sampled_fallback_accepted{false};
  std::size_t discarded_direct_medial_edges{0};
  std::size_t discarded_direct_medial_copper_violations{0};
  std::size_t discarded_direct_medial_topology_complete_paths{0};
  std::size_t discarded_direct_medial_portal_transitions{0};
  std::size_t discarded_accepted_direct_portal_lane_transitions{0};
  std::size_t discarded_voronoi_lane_conflict_passes{0};
  std::size_t discarded_voronoi_lane_conflict_observations{0};
  std::size_t discarded_voronoi_lane_conflict_history_entries{0};
  std::size_t discarded_voronoi_exact_junction_conflict_history_entries{0};
  std::size_t discarded_unaccepted_selected_voronoi_lane_transitions{0};
  std::size_t discarded_materialized_topology_locked_paths{0};
  std::size_t discarded_topology_locked_rejection_copper_violations{0};
  std::size_t discarded_attempted_topology_locked_subset_repair_candidates{0};
  std::size_t discarded_attempted_shared_resource_fanout_groups{0};
  std::size_t discarded_attempted_shared_resource_fanout_candidates{0};
  std::size_t discarded_materialized_shared_resource_fanout_groups{0};
  std::size_t discarded_synchronized_shared_resource_fanouts{0};
  std::size_t discarded_shared_resource_fanout_fallbacks{0};
  std::vector<SharedResourceFanoutFailure> discarded_shared_resource_fanout_failures;
  DetailedRouterConfig materialized_config;
  DetailedRouterStopReason stop_reason{DetailedRouterStopReason::invalid_input};
  std::string diagnostic;

  [[nodiscard]] bool complete() const { return stop_reason == DetailedRouterStopReason::complete; }
  [[nodiscard]] bool provisional_zone_refill_candidate() const {
    return stop_reason == DetailedRouterStopReason::zone_refill_required && candidate.has_value();
  }
};

// Converts a completed global plan into simplified physical copper on a board copy. Optional
// deterministic 45-degree pull-tight, bounded exact-clearance continuous centerline refinement,
// topology-locked packed medial lanes, optional post-allocation quintic PH curve/PNS/native-arc
// fitting, and bounded polyline multi-period differential-pair skew tuning in a quantized local
// frame are accepted only after exact validation/connectivity/DRC. Illegal candidates fall back
// transactionally to the conventional polyline. The source is never mutated.
[[nodiscard]] DetailedRouteResult
materialize_detailed_route(const Board& source, const GlobalRouteResult& global,
                           const DetailedRouterConfig& config = {},
                           std::stop_token stop_token = {});

// A bounded regional spline plan may cross the detailed-router gate with planning-only conflicts
// cleared on a private copy. The ordinary complete-board validation, connectivity, and copper DRC
// transaction remains the only acceptance authority.
[[nodiscard]] DetailedRouteResult
materialize_detailed_route(const Board& source, const RoutingPlanResult& plan,
                           const DetailedRouterConfig& config = {},
                           std::stop_token stop_token = {});

// Reuses per-net clean physical copper for global.reused_nets from a prior detailed
// checkpoint candidate, rematerializes the affected paths, and validates the combined
// transaction. If reuse makes the candidate illegal, all paths are rematerialized.
[[nodiscard]] DetailedRouteResult materialize_detailed_route_incremental(
    const Board& source, const GlobalRouteResult& global, const DetailedRouteResult& checkpoint,
    const DetailedRouterConfig& config = {}, std::stop_token stop_token = {});

// Regional counterpart of incremental detailed materialization.  Complete clean copper for
// unchanged nets is retained from checkpoint, while the selected regional PH/layer/via paths are
// rematerialized and the combined board passes the same whole-transaction gates.  Planning-only
// overflow is cleared only on the private candidate after the complete regional evidence has been
// verified exactly as in materialize_detailed_route(RoutingPlanResult).
[[nodiscard]] DetailedRouteResult materialize_detailed_route_incremental(
    const Board& source, const RoutingPlanResult& plan, const DetailedRouteResult& checkpoint,
    const DetailedRouterConfig& config = {}, std::stop_token stop_token = {});

} // namespace flux
