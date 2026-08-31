#pragma once

#include "flux/core/geometry.hpp"
#include "flux/core/global_router.hpp"
#include "flux/core/spline_route.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <stop_token>
#include <string>
#include <vector>

namespace flux {

// Positively bounded derived-state policy for the regional physical-packing stage. The first
// implementation publishes conflict evidence only; later milestones consume these same bounds for
// window partitioning, route-template enumeration, bundle search, and exact transactions.
struct RegionalRouterConfig {
  bool treat_saved_zone_fills_as_refillable{false};
  Nanometres window_margin{Nanometres::from_mm(2.0)};
  std::size_t max_conflict_fragments_per_window{64};
  std::size_t max_nets_per_window{12};
  std::size_t max_templates_per_fragment{4};
  std::size_t max_terminal_choices_per_terminal{4};
  // Diagnostic/test override. Production leaves this false until portal portfolio growth has a
  // tighter joint-search representation and a measured dense-board benefit.
  bool enable_terminal_portal_templates{false};
  std::size_t max_via_regions_per_fragment{4};
  std::size_t max_via_sites_per_region{4};
  std::size_t max_selected_via_site_candidate_checks{16'384};
  std::size_t max_selected_via_site_checks_per_region{256};
  std::size_t max_access_assignment_states{512};
  std::size_t max_access_blockers_per_window{16};
  std::size_t max_layer_sequences_per_fragment{3};
  Nanometres navmesh_cell_pitch{Nanometres::from_mm(0.25)};
  std::size_t max_navmesh_cells_per_template{100'000};
  std::size_t max_navmesh_expansions_per_template{100'000};
  std::size_t max_alternative_template_checks{384};
  // A second, explicitly bounded topology wave is reserved for a window whose complete ordinary
  // portfolio has no jointly legal access/lane state. It explores only blocker-attributed
  // fragments and never enlarges the ordinary portfolio globally.
  std::size_t max_joint_repair_windows{512};
  std::size_t max_joint_repair_fragments_per_window{8};
  std::size_t max_joint_alternative_template_checks{1'024};
  std::size_t max_joint_templates_per_fragment{12};
  std::size_t max_joint_via_site_ring{4};
  // Conflict-bracketing layer overpasses are deliberately independent of the established
  // endpoint-alternative portfolio. They are attempted only as a bounded regional follow-up and
  // therefore cannot consume or reorder the ordinary/joint template budgets.
  std::size_t max_local_layer_bypass_checks{512};
  std::size_t max_local_layer_bypasses_per_fragment{4};
  // The discrete joint solver retains only this many complete-window states for authoritative
  // continuous realization. The first spline-legal state wins in discrete-objective order.
  std::size_t max_joint_spline_candidates{4};
  // Bounded semantic failure records produced while reducing the joint spline shortlist.
  std::size_t max_local_routability_diagnoses{4'096};
  // Reuse existing exact-prevalidated portfolios for only the highest-pressure windows. The
  // ADR 0145 expanded window is included in this total rather than added on top of it.
  std::size_t max_spline_portfolio_windows{512};
  std::size_t max_same_layer_detour_checks{1'024};
  std::size_t max_bundle_search_states{100'000};
  std::size_t max_lane_order_states{100'000};
  std::size_t max_spline_sweeps{8};
  std::size_t max_adaptive_pair_shove_checks_per_window{16};
  std::size_t max_quantization_feedback_iterations{2};
  std::size_t max_quantized_segment_pair_checks_per_window{2'000'000};
  // One fallback decomposition may combine a failed articulation window with the windows that
  // currently own its participating fragments. The ordinary decomposition is retained unless
  // this bounded conflict-cone transaction is feasibility-first better.
  std::size_t max_residual_conflict_cone_windows{8};
  // Do not duplicate an already broad board-wide assignment portfolio for one residual retry.
  // A future incremental cone transaction may replace this guard after it remaps every referenced
  // channel, terminal/via choice, and spline realization atomically.
  std::size_t max_residual_retry_bundle_assignments{384};
  std::size_t max_exact_transactions{64};

  bool operator==(const RegionalRouterConfig&) const = default;
};

// Stable within one selected global checkpoint. A fragment ordinal is derived by walking one
// net/connection path in point order and splitting at every layer transition. It is derived
// routing evidence and is never persisted in the canonical KiCad board.
struct RegionalRouteFragmentId {
  NetId net;
  std::size_t connection_index{0};
  std::size_t run_index{0};
  std::size_t fragment_index{0};

  auto operator<=>(const RegionalRouteFragmentId&) const = default;
};

struct RegionalRouteFragment {
  RegionalRouteFragmentId id;
  LayerId layer;
  Point start;
  Point end;
  Nanometres width;
  Nanometres clearance;
  geometry::AxisAlignedBounds bounds;
  bool begins_at_terminal{false};
  bool ends_at_terminal{false};
  bool coupled_member{false};
  bool explicit_width{false};
  GlobalRouteSegmentWidthKind width_kind{GlobalRouteSegmentWidthKind::nominal};

  bool operator==(const RegionalRouteFragment&) const = default;
};

// A spatially contiguous set of exact clearance-shadow observations between the same two selected
// route fragments. Physical deficit remains a diagnostic/partition weight; it does not authorize
// copper or mix nanometres into the global sampled-capacity dimension.
struct RegionalConflictEdge {
  std::size_t id{0};
  RegionalRouteFragmentId first;
  RegionalRouteFragmentId second;
  LayerId layer;
  geometry::AxisAlignedBounds conflict_bounds;
  Nanometres peak_clearance_deficit;
  std::uint64_t observed_overlap_length_nm{0};
  std::size_t sample_count{0};
  std::size_t clearance_shadow_sample_count{0};
  std::size_t capacity_sample_count{0};
  std::uint8_t evidence_mask{0};
  std::size_t discrete_excess{0};
  std::uint64_t physical_excess_nm{0};
  std::vector<ObjectId> attributed_obstacles;

  bool operator==(const RegionalConflictEdge&) const = default;
};

struct RegionalConflictComponent {
  std::size_t id{0};
  std::vector<RegionalRouteFragmentId> fragments;
  std::vector<std::size_t> conflict_edges;
  std::vector<NetId> nets;

  bool operator==(const RegionalConflictComponent&) const = default;
};

struct RegionalConflictBlock {
  std::size_t id{0};
  std::vector<RegionalRouteFragmentId> fragments;
  std::vector<std::size_t> conflict_edges;
  std::vector<RegionalRouteFragmentId> articulation_fragments;
  std::vector<NetId> nets;
  geometry::AxisAlignedBounds conflict_bounds;
  std::uint64_t observed_overlap_length_nm{0};
  Nanometres peak_clearance_deficit;

  bool operator==(const RegionalConflictBlock&) const = default;
};

struct RegionalBoundaryPortal {
  std::size_t id{0};
  RegionalRouteFragmentId fragment;
  Point position;
  LayerId layer;
  SplineVector inward_tangent;
  Nanometres width;

  bool operator==(const RegionalBoundaryPortal&) const = default;
};

enum class RegionalWindowFailureReason {
  none,
  oversized_fragment_set,
  oversized_net_set,
  uncuttable_fragment,
  coupled_atom_split
};

struct RegionalRoutingWindow {
  std::size_t id{0};
  geometry::AxisAlignedBounds bounds;
  std::vector<LayerId> layers;
  std::vector<RegionalRouteFragmentId> fragments;
  std::vector<RegionalRouteFragmentId> articulation_fragments;
  std::vector<RegionalRouteFragmentId> owned_fragments;
  std::vector<RegionalBoundaryPortal> portals;
  std::vector<std::size_t> conflict_edges;
  std::vector<NetId> nets;
  std::uint64_t observed_overlap_length_nm{0};
  Nanometres peak_clearance_deficit;
  RegionalWindowFailureReason failure_reason{RegionalWindowFailureReason::none};

  [[nodiscard]] bool eligible() const {
    return failure_reason == RegionalWindowFailureReason::none;
  }
  bool operator==(const RegionalRoutingWindow&) const = default;
};

struct RegionalLanePlacement {
  RegionalRouteFragmentId fragment;
  std::int64_t center_offset_nm{0};

  bool operator==(const RegionalLanePlacement&) const = default;
};

struct RegionalCrossSection {
  std::size_t id{0};
  LayerId layer;
  Point first_wall;
  Point second_wall;
  Nanometres usable_width;
  // Physical order in which the selected source centre-lines cross this wall. This is the
  // topology-preserving reference used by the regional allocator; stable fragment IDs remain
  // deterministic identities, not a proxy for a lane position.
  std::vector<RegionalRouteFragmentId> geometric_reference_order;
  std::vector<RegionalRouteFragmentId> ordered_occupants;
  std::vector<RegionalLanePlacement> placements;
  Nanometres packed_width;
  std::size_t stable_order_inversions{0};
  std::size_t selected_order_inversions{0};
  bool physically_fits{false};

  bool operator==(const RegionalCrossSection&) const = default;
};

struct RegionalTopologyLink {
  Point first;
  Point second;
  Nanometres radius;

  bool operator==(const RegionalTopologyLink&) const = default;
};

struct RegionalChannel {
  std::size_t id{0};
  LayerId layer;
  std::vector<Polygon> topology_cells;
  std::vector<NavMeshPortal> portals;
  // Directed, selected funnel gates.  Empty is explicit "cell-union-only evidence" for older
  // global corridor producers that did not retain a NavMesh route certificate.
  NavMeshTopologySignature topology_signature;
  std::vector<RegionalTopologyLink> endpoint_links;
  std::vector<RegionalCrossSection> cross_sections;
  std::vector<std::size_t> adjacent_channels;
  std::vector<RegionalRouteFragmentId> occupants;
  bool conservative_source_evidence{true};

  bool operator==(const RegionalChannel&) const = default;
};

struct RegionalViaRunup {
  LayerId layer;
  Point anchor;
  std::vector<Point> centerline;

  bool operator==(const RegionalViaRunup&) const = default;
};

struct RegionalViaChoice {
  std::size_t id{0};
  std::size_t region_id{0};
  NetId net;
  Point site;
  std::vector<LayerId> span;
  Nanometres diameter;
  Nanometres drill;
  std::vector<RegionalViaRunup> attached_runups;
  bool selected_global_site{false};
  bool alternative_search_deferred{false};

  bool operator==(const RegionalViaChoice&) const = default;
};

// Immutable derived evidence for one bounded via-choice region. It makes transition scarcity and
// physical contention visible before bundle realization; it is not canonical copper and cannot
// authorize a via without the existing exact transaction.
struct RegionalViaRegionEvidence {
  std::size_t region_id{0};
  NetId net;
  std::vector<LayerId> span;
  std::vector<std::size_t> candidate_choices;
  std::size_t selected_global_candidates{0};
  std::size_t competing_regions{0};
  std::size_t conflicting_candidate_pairs{0};
  std::size_t terminal_escape_conflicts{0};
  std::size_t overlapping_span_regions{0};
  bool contested{false};
  bool alternative_search_deferred{false};

  bool operator==(const RegionalViaRegionEvidence&) const = default;
};

struct RegionalTerminalChoice {
  std::size_t id{0};
  ObjectId footprint;
  ObjectId pad;
  NetId net;
  Point pad_position;
  Point portal;
  LayerId layer;
  std::vector<Point> access_path;
  Nanometres width;
  std::optional<std::size_t> via_choice;
  bool selected_global_access{false};

  bool operator==(const RegionalTerminalChoice&) const = default;
};

struct RegionalRouteTemplate {
  std::size_t id{0};
  RegionalRouteFragmentId fragment;
  LayerId routing_layer;
  std::vector<std::size_t> entry_portals;
  std::vector<std::size_t> exit_portals;
  std::vector<std::size_t> channel_sequence;
  std::vector<LayerId> layer_sequence;
  std::vector<std::size_t> via_choices;
  std::optional<std::size_t> entry_via_choice;
  std::optional<std::size_t> exit_via_choice;
  std::vector<std::size_t> avoided_conflict_edges;
  std::vector<Point> topology_centerline;
  // The route class selected by this discrete template.  A B-spline/PNS realization may deform
  // inside it but must ask the bounded regional template search for any corridor-side change.
  NavMeshTopologySignature topology_signature;
  std::optional<std::size_t> terminal_entry_choice;
  std::optional<std::size_t> terminal_exit_choice;
  std::uint64_t lower_bound_length_nm{0};
  std::uint64_t topology_cost{0};
  bool selected_global_topology{true};
  bool exact_static_checked{false};

  bool operator==(const RegionalRouteTemplate&) const = default;
};

enum class RegionalBundleStopReason {
  complete,
  missing_corridor_topology,
  no_fitting_lane_order,
  window_ineligible,
  state_budget_exhausted,
  spline_realization_failed
};

enum class RegionalAccessBlockerKind { via_pair, terminal_via };

// Bounded diagnostic evidence from the partial template/access CSP. Choice IDs reference the
// immutable derived arrays in RegionalRouteResult; they are not canonical board object IDs.
struct RegionalAccessBlocker {
  RegionalAccessBlockerKind kind{RegionalAccessBlockerKind::via_pair};
  std::optional<std::size_t> first_via_choice;
  std::optional<std::size_t> second_via_choice;
  std::optional<std::size_t> terminal_choice;
  std::size_t observations{0};

  bool operator==(const RegionalAccessBlocker&) const = default;
};

struct RegionalSplineRealization {
  RegionalRouteFragmentId fragment;
  Point replacement_end;
  LayerId layer;
  std::vector<Point> centerline;
  // Final optimized cubic controls retained for diagnostic telemetry. Canonical copper still uses
  // the separately quantized centerline/materialization transaction.
  std::vector<Point> control_points;
  Nanometres width;
  GlobalRouteSegmentWidthKind width_kind{GlobalRouteSegmentWidthKind::nominal};
  std::optional<Point> entry_via_site;
  std::optional<Point> exit_via_site;
  std::size_t optimization_sweeps{0};
  std::size_t accepted_control_moves{0};
  std::uint64_t topology_signature{0U};
  std::size_t topology_gate_count{0U};
  bool topology_legal{false};
  bool topology_gate_sequence_legal{false};
  bool endpoint_topology_legal{false};
  bool interior_topology_legal{false};
  bool mutually_legal{false};
  bool quantized_mutually_legal{false};

  bool operator==(const RegionalSplineRealization&) const = default;
};

struct RegionalBundleSelection {
  std::vector<std::size_t> templates;
  std::vector<std::size_t> via_choices;
  std::vector<std::size_t> via_assignment_region_order;
  std::vector<RegionalCrossSection> packed_cross_sections;
  std::size_t via_alternative_switches{0};
  std::size_t template_conflicts{0};
  std::size_t residual_conflicts{0};

  bool operator==(const RegionalBundleSelection&) const = default;
};

struct RegionalBundleAssignment {
  std::size_t window_id{0};
  std::vector<RegionalRouteTemplate> templates;
  std::vector<std::size_t> selected_templates;
  std::vector<std::size_t> selected_via_choices;
  std::vector<std::size_t> via_assignment_region_order;
  std::vector<RegionalCrossSection> packed_cross_sections;
  std::vector<RegionalSplineRealization> spline_realizations;
  std::vector<RegionalBundleSelection> spline_candidate_shortlist;
  std::vector<RegionalAccessBlocker> partial_access_blockers;
  std::size_t lane_order_states{0};
  std::size_t bundle_search_states{0};
  std::size_t partial_access_pruned_states{0};
  std::size_t partial_access_assignment_states{0};
  std::size_t partial_access_assignment_exhaustions{0};
  std::size_t partial_terminal_pair_pruned_states{0};
  std::size_t partial_via_pair_pruned_states{0};
  std::size_t partial_terminal_via_pruned_states{0};
  std::size_t partial_lane_capacity_pruned_states{0};
  std::size_t access_assignment_states{0};
  std::size_t selected_template_conflicts{0};
  std::size_t selected_residual_conflicts{0};
  std::size_t coupled_rejected_states{0};
  std::size_t access_rejected_states{0};
  std::size_t invalid_access_rejected_states{0};
  std::size_t terminal_pair_rejected_states{0};
  std::size_t via_pair_rejected_states{0};
  std::size_t terminal_via_rejected_states{0};
  std::size_t packing_rejected_states{0};
  std::size_t selected_via_alternative_switches{0};
  std::size_t access_assignment_exhaustions{0};
  std::size_t spline_candidate_rank{0};
  bool joint_spline_candidate{false};
  RegionalBundleStopReason stop_reason{RegionalBundleStopReason::window_ineligible};

  [[nodiscard]] bool complete() const { return stop_reason == RegionalBundleStopReason::complete; }
  bool operator==(const RegionalBundleAssignment&) const = default;
};

enum class RegionalRouterStopReason { no_conflicts, conflicts_detected, invalid_input, cancelled };

// Complete end-to-end multilayer routing evidence. A route may contain any positive number of
// physical via transitions; it is deliberately not represented as a special-case via pair.
// Consecutive layer changes at one coordinate form one through/blind/buried via span, while a
// planar run between two sites separates them into distinct physical vias. This remains derived
// planning state: detailed materialization, connectivity, and copper DRC are authoritative.
struct RegionalPlannedLayerRun {
  LayerId layer;
  std::vector<Point> centerline;

  bool operator==(const RegionalPlannedLayerRun&) const = default;
};

struct RegionalPlannedViaTransition {
  Point site;
  std::vector<LayerId> span;

  bool operator==(const RegionalPlannedViaTransition&) const = default;
};

struct RegionalMultilayerRoutePlan {
  NetId net;
  std::size_t connection_index{0};
  std::vector<RegionalPlannedLayerRun> layer_runs;
  std::vector<RegionalPlannedViaTransition> via_transitions;

  bool operator==(const RegionalMultilayerRoutePlan&) const = default;
};

struct RegionalRouteResult {
  RegionalRouterConfig configured;
  std::vector<RegionalRouteFragment> fragments;
  std::vector<RegionalConflictEdge> conflict_edges;
  std::vector<RegionalConflictComponent> components;
  std::vector<RegionalConflictBlock> biconnected_blocks;
  std::vector<RegionalRouteFragmentId> articulation_fragments;
  std::vector<RegionalRoutingWindow> windows;
  std::vector<RegionalChannel> channels;
  std::vector<RegionalTerminalChoice> terminal_choices;
  std::size_t selected_terminal_portal_choices{0};
  std::vector<RegionalViaChoice> via_choices;
  std::vector<RegionalViaRegionEvidence> via_region_evidence;
  std::vector<RegionalBundleAssignment> bundle_assignments;
  std::vector<RoutabilityDiagnosis> local_routability_diagnoses;
  std::vector<GlobalRoutePath> spline_candidate_paths;
  std::vector<RegionalMultilayerRoutePlan> multilayer_route_plans;
  std::size_t complete_via_dependent_routes{0};
  std::size_t planned_layer_runs{0};
  std::size_t planned_via_transitions{0};
  bool spline_candidate_available{false};
  bool spline_candidate_conflicting_articulation{false};
  std::size_t source_clearance_shadow_conflicts{0};
  std::size_t coalesced_clearance_shadow_samples{0};
  std::size_t unmapped_clearance_shadow_conflicts{0};
  std::size_t source_sampled_capacity_conflicts{0};
  std::size_t source_corridor_capacity_conflicts{0};
  std::size_t source_physical_capacity_conflicts{0};
  std::size_t unmapped_capacity_conflicts{0};
  std::size_t oversized_windows{0};
  std::size_t uncuttable_windows{0};
  std::size_t window_fragment_anchor_expansions{0};
  Nanometres peak_window_fragment_anchor_expansion;
  std::size_t geometrically_split_windows{0};
  std::size_t coupled_window_merges{0};
  std::size_t uncuttable_oversized_blocks{0};
  std::size_t fragments_without_channel_topology{0};
  std::size_t selected_global_narrow_corridor_fallbacks{0};
  std::size_t selected_lane_cross_sections{0};
  std::size_t selected_geometric_lane_reorders{0};
  std::size_t avoided_lane_order_inversions{0};
  std::size_t lane_solver_queries{0};
  std::size_t lane_solver_cache_hits{0};
  std::size_t lane_solver_states{0};
  std::size_t lane_solver_pruned_states{0};
  std::size_t lane_solver_analytical_rejections{0};
  std::size_t lane_solver_budget_exhaustions{0};
  std::size_t partial_lane_capacity_pruned_states{0};
  std::size_t refittable_terminal_width_fragments{0};
  std::size_t fixed_resource_width_fragments{0};
  std::size_t via_site_candidate_checks{0};
  std::size_t via_site_diversity_rejections{0};
  std::size_t contested_via_regions{0};
  std::size_t peak_via_region_contention{0};
  bool selected_via_site_candidate_budget_exhausted{false};
  bool via_site_candidate_budget_exhausted{false};
  std::size_t alternative_template_checks{0};
  std::size_t accepted_alternative_templates{0};
  std::size_t joint_repair_windows_attempted{0};
  std::size_t joint_repair_windows_with_expanded_portfolio{0};
  std::size_t joint_repair_windows_completed{0};
  std::vector<std::size_t> joint_repair_window_ids;
  std::vector<std::size_t> joint_repair_completed_window_ids;
  std::size_t joint_repair_template_checks{0};
  std::size_t joint_repair_accepted_templates{0};
  bool joint_repair_template_budget_exhausted{false};
  std::size_t local_layer_bypass_checks{0};
  std::size_t accepted_local_layer_bypasses{0};
  bool local_layer_bypass_budget_exhausted{false};
  std::size_t joint_spline_candidates_evaluated{0};
  std::size_t joint_spline_candidates_legal{0};
  std::size_t joint_spline_windows_recovered{0};
  std::vector<std::size_t> joint_spline_recovered_window_ids;
  std::size_t joint_spline_failed_fallback_improvements{0};
  std::size_t diagnosis_directed_spline_selections{0};
  std::size_t diagnosis_directed_template_changes{0};
  std::size_t diagnosis_directed_via_changes{0};
  std::size_t diagnosis_directed_lane_order_changes{0};
  std::size_t locally_resolved_routability_diagnoses{0};
  std::size_t dropped_local_routability_diagnoses{0};
  std::vector<std::size_t> joint_spline_failed_improved_window_ids;
  std::size_t spline_portfolio_windows{0};
  std::vector<std::size_t> spline_portfolio_window_ids;
  std::size_t alternative_domain_rejections{0};
  std::size_t alternative_navmesh_rejections{0};
  std::size_t alternative_runup_rejections{0};
  std::size_t alternative_via_span_rejections{0};
  std::size_t alternative_topology_cell_rejections{0};
  std::size_t alternative_priority_windows{0};
  std::size_t alternative_residual_priority_windows{0};
  std::size_t alternative_residual_followup_checks{0};
  std::size_t same_layer_detour_checks{0};
  std::size_t accepted_same_layer_detours{0};
  bool alternative_template_budget_exhausted{false};
  std::size_t spline_short_realization_rejections{0};
  std::size_t spline_topology_rejections{0};
  std::size_t spline_topology_endpoint_rejections{0};
  std::size_t spline_topology_interior_rejections{0};
  std::size_t spline_topology_gate_signatures{0};
  std::size_t spline_topology_gates{0};
  std::size_t spline_topology_gate_rejections{0};
  std::size_t spline_mutual_clearance_rejections{0};
  std::size_t spline_lane_targets{0};
  std::size_t spline_multi_target_fragments{0};
  std::size_t spline_merged_runs{0};
  std::size_t spline_merged_fragments{0};
  std::size_t spline_joint_shove_attempts{0};
  std::size_t spline_joint_shove_accepted{0};
  std::size_t spline_adaptive_pair_shove_attempts{0};
  std::size_t spline_adaptive_pair_shove_accepted{0};
  std::size_t spline_adaptive_pair_windows_improved{0};
  std::size_t spline_adaptive_pair_windows_legalized{0};
  std::size_t quantized_segment_pair_checks{0};
  std::size_t quantized_clearance_conflicts{0};
  std::size_t quantization_feedback_attempts{0};
  std::size_t quantization_feedback_accepted{0};
  std::size_t quantization_feedback_windows_legalized{0};
  std::size_t quantization_check_budget_exhausted_windows{0};
  Nanometres peak_quantized_clearance_deficit;
  std::size_t residual_conflict_cone_retries{0};
  std::size_t residual_conflict_cones_built{0};
  std::size_t residual_conflict_cone_windows_merged{0};
  std::size_t residual_conflict_cone_reused_assignments{0};
  std::size_t residual_conflict_cone_recomputed_assignments{0};
  std::size_t residual_conflict_cone_retry_withheld_assignments{0};
  bool residual_conflict_cone_selected{false};
  std::uint64_t total_observed_overlap_length_nm{0};
  Nanometres peak_clearance_deficit;
  RegionalRouterStopReason stop_reason{RegionalRouterStopReason::invalid_input};
  std::string diagnostic;

  [[nodiscard]] bool complete() const {
    return spline_candidate_available ||
           (stop_reason == RegionalRouterStopReason::no_conflicts &&
            unmapped_clearance_shadow_conflicts == 0U && unmapped_capacity_conflicts == 0U);
  }

  bool operator==(const RegionalRouteResult&) const = default;
};

enum class RoutingPlanStopReason {
  complete,
  coarse_global_incomplete,
  regional_conflicts,
  invalid_input,
  cancelled
};

struct RoutingPlannerConfig {
  GlobalRouterConfig global;
  RegionalRouterConfig regional;
  // Number of bounded upstream replans after the initial global allocation. Zero restores the
  // single-pass boundary. Each retry consumes typed physical diagnoses as cost-only history.
  std::size_t max_routability_feedback_retries{2U};
  RoutabilityProbeConfig routability_probe;

  bool operator==(const RoutingPlannerConfig&) const = default;
};

// The first-class boundary between coarse PathFinder state and regional physical packing. Paths
// initially mirror coarse_global.paths; a later regional transaction replaces only governed
// fragments while preserving this result contract.
struct RoutingPlanResult {
  GlobalRouteResult coarse_global;
  RegionalRouteResult regional;
  std::vector<GlobalRoutePath> paths;
  RoutabilityKnowledge routability_knowledge;
  std::size_t global_feedback_passes{0U};
  std::size_t cheap_physical_probes{0U};
  std::size_t regional_feedback_passes{0U};
  bool routability_feedback_budget_exhausted{false};
  RoutingPlanStopReason stop_reason{RoutingPlanStopReason::invalid_input};
  std::string diagnostic;

  [[nodiscard]] bool complete() const { return stop_reason == RoutingPlanStopReason::complete; }
};

// Converts exact selected-route clearance shadows into deterministic segment-level conflict
// evidence. Every source shadow must either map to a fragment edge or increment the explicit
// unmapped count; evidence is never silently dropped.
[[nodiscard]] RegionalRouteResult
analyze_regional_conflicts(const Board& board, const GlobalRouteResult& global,
                           const RegionalRouterConfig& config = {}, std::stop_token stop_token = {},
                           RouterTelemetrySink* telemetry = nullptr);

// Deterministically decomposes the fragment conflict graph into biconnected blocks and bounded
// physical windows. Every conflict edge belongs to exactly one block and one initial window.
void partition_regional_conflicts(const Board& board, RegionalRouteResult& regional,
                                  std::stop_token stop_token = {});

// Packs a fixed occupant order using heterogeneous physical widths and adjacent maximum
// clearances. A successful result is centered in the cross section and carries signed lane-center
// offsets; it remains derived evidence rather than copper.
[[nodiscard]] RegionalCrossSection
pack_regional_cross_section(const std::vector<RegionalRouteFragment>& fragments,
                            const std::vector<RegionalRouteFragmentId>& order, LayerId layer,
                            Point first_wall, Point second_wall, Nanometres usable_width);

// Orders occupants by their physical intersection with a canonicalized channel cross section.
// Parallel or non-intersecting source fragments fall back to midpoint projection on the same
// wall axis. Stable IDs are used only to resolve geometrically coincident positions.
[[nodiscard]] std::vector<RegionalRouteFragmentId>
geometric_regional_cross_section_order(const std::vector<RegionalRouteFragment>& fragments,
                                       const std::vector<RegionalRouteFragmentId>& occupants,
                                       Point first_wall, Point second_wall);

// Parses one complete point/layer path into ordered planar runs and physical via transitions.
// Invalid, disconnected, degenerate, or unknown-layer transition sequences fail closed. A
// same-site stack such as F.Cu -> In1.Cu -> B.Cu is one physical via span; leaving that site and
// transitioning again creates another via. Paths without a layer transition return nullopt.
[[nodiscard]] std::optional<RegionalMultilayerRoutePlan>
plan_multilayer_route(const Board& board, const GlobalRoutePath& path);

// Runs the existing global topology/capacity router, then publishes regional physical-conflict
// evidence without changing the selected paths. Regional repair is introduced behind this API.
[[nodiscard]] RoutingPlanResult plan_routes(const Board& board,
                                            const RoutingPlannerConfig& config = {},
                                            std::stop_token stop_token = {},
                                            RouterTelemetrySink* telemetry = nullptr);

} // namespace flux
