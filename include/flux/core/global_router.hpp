#pragma once

#include "flux/core/board.hpp"
#include "flux/core/navmesh_route.hpp"
#include "flux/core/routability_knowledge.hpp"

#include <cstddef>
#include <cstdint>
#include <stop_token>
#include <string>
#include <vector>

namespace flux {

class RouterTelemetrySink;

enum class GlobalRouteCheckpointObjective { aggregate_overflow, dimensioned_physical_overflow };

struct GlobalRouterConfig {
  // Empty routes every incomplete named net. A non-empty exact ID set scopes routing and treats
  // all other source copper as immutable obstacles. Coupled pairs/buses must be selected whole.
  std::vector<NetId> selected_nets;
  // Explicit first-to-last scheduling tier. Listed nets remain ahead of ordinary dynamic
  // pressure ordering; every hard routing and acceptance gate remains unchanged.
  std::vector<NetId> priority_nets;
  Nanometres grid_pitch{Nanometres::from_mm(1.0)};
  // Zero uses every planar node as a via site. An explicit value must be an exact multiple of
  // grid_pitch and large enough for resolved via-to-via spacing.
  Nanometres via_site_pitch{Nanometres{0}};
  std::size_t via_site_phase_x{0};
  std::size_t via_site_phase_y{0};
  // Opt-in non-crossing triangulated 45-degree sampled edges. The grid spacing gate accounts for
  // the closer parallel-diagonal separation; initial allocation remains polyline based.
  bool allow_diagonal_grid_edges{false};
  // Opt-in both diagonals in every sampled cell. The two crossing chords share one cell-capacity
  // resource, so they improve obstacle-aware topology without authorizing unlike-net crossings.
  // This is mutually exclusive with the checkerboard triangulation above.
  bool allow_crossing_safe_diagonal_grid_edges{false};
  // Exact-distance physical shadows for already selected sampled polylines, pad-access stubs,
  // and through-via transitions. Conflicting edge/node shadows are negotiated resources: soft
  // passes pay present/history cost and hard passes expose attributed blockers for bounded rip-up.
  bool reserve_sampled_route_clearance{true};
  std::size_t max_clearance_shadow_conflict_resources{100'000};
  // A positive value normalizes physical clearance deficit into bounded present/history pressure.
  // Zero retains one pressure unit per conflict. Conflict count remains a separate feasibility
  // dimension; this option only distinguishes a near miss from a deep overlap during negotiation.
  Nanometres clearance_shadow_cost_quantum{Nanometres{0}};
  std::size_t edge_capacity{1};
  std::size_t max_iterations{30};
  std::size_t max_expansions_per_connection{500'000};
  // Zero disables selective weighted-A* retry. Enabled values are per-mille and must exceed 1000.
  std::uint32_t expansion_retry_heuristic_weight_per_mille{0};
  // Re-key the ordinary search's live frontier instead of restarting its weighted retry.
  bool reuse_weighted_search_frontier{false};
  // The dimensioned policy requires shared Voronoi allocation resources. The legacy aggregate
  // remains the default so existing route selection and bytes do not change.
  GlobalRouteCheckpointObjective checkpoint_objective{
      GlobalRouteCheckpointObjective::aggregate_overflow};
  // Scale shared-medial history by conservative minimum-envelope excess quanta.
  bool scale_voronoi_history_by_overflow{false};
  std::size_t max_pin_escape_grid_radius{3};
  std::size_t max_pin_access_nodes_per_terminal{16};
  // Optional fine 45-degree mesh used only between exact pad positions and coarse global portals.
  // It must divide the global pitch exactly; global allocation and via sites remain on the coarse
  // lattice so this does not weaken inter-route capacity assumptions.
  Nanometres local_pin_escape_pitch{Nanometres{0}};
  std::size_t max_local_pin_escape_expansions_per_terminal{20'000};
  // Add one bounded exact-checked quarter-pitch escape alternative when different nets share a
  // nearest coarse portal. The ordinary nearest portal remains available and authoritative.
  bool allocate_contended_terminal_fanout{true};
  // Opt-in: retain exact-legal egress portals from later rings until the existing per-terminal
  // node bound, instead of stopping after the first successful Chebyshev ring.
  bool diversify_pin_access_rings{false};
  // Opt-in: seed and grow each multi-terminal tree from terminals with fewer legal access portals
  // before less constrained terminals. Stable footprint/pad IDs break equal-access ties.
  // Both values must be positive to enable terminal-only variable-width access. The nominal-width
  // grid portal remains an exact hard gate; allocation beyond it is unchanged.
  Nanometres terminal_neck_down_width{Nanometres{0}};
  Nanometres max_terminal_neck_down_length{Nanometres{0}};
  // Both values must be positive to allow ordinary, uncoupled nets to traverse a certified direct
  // medial chord at an explicit narrower width. Nominal-width portal disks remain hard gates.
  Nanometres medial_neck_down_width{Nanometres{0}};
  Nanometres max_medial_neck_down_length{Nanometres{0}};
  // Width-mode A* may enter a conservatively extracted sampled corridor at this explicit width.
  // The total contiguous narrow run is bounded and both transitions require nominal-width disks.
  Nanometres corridor_neck_down_width{Nanometres{0}};
  Nanometres max_corridor_neck_down_length{Nanometres{0}};
  std::size_t max_corridor_neck_down_states{5'000'000};
  std::size_t max_free_space_edges{2'000'000};
  std::uint64_t step_cost{100};
  std::uint64_t via_cost{800};
  std::uint64_t present_congestion_cost{2'000};
  std::uint64_t history_congestion_cost{500};
  // Non-authoritative physical-routing diagnoses from an earlier outer-loop pass. These are
  // rasterized once into net-scoped graph-edge costs; they cannot block or authorize copper.
  std::vector<RoutabilityHistoryEntry> routability_history;
  std::uint64_t routability_history_cost{500};
  std::size_t max_routability_history_edges{500'000};
  std::uint64_t corridor_present_cost{400};
  std::uint64_t corridor_history_cost{200};
  // A bounded, non-authoritative negotiated-routing preference for an empty parallel lane next
  // to already routed same-layer copper. The discount is capped at the corridor's existing-use
  // cost, so it cannot make a path cheaper than its physical base length or relax capacity/DRC.
  // Zero disables generic river-following while retaining declared bus/pair cohesion policies.
  std::uint64_t river_following_cost{200};
  // Pass-scoped soft memory left by a hard-capacity rip-up.  It discourages the failed net from
  // immediately claiming a victim's former sampled edge while the victim is being rebuilt.  This
  // is neither occupancy nor history: it expires with the PathFinder pass and cannot reject a
  // legal transition.  Zero disables the memory cost.
  std::uint64_t former_route_memory_cost{1'000};
  // Preserve the final currently viable coarse-portal/local-stub option for an unrouted
  // terminal.  The reservation is a bounded, non-authoritative future-demand cue: it charges a
  // candidate only when its clearance envelope would consume that terminal's last currently
  // available access candidate.  It never blocks a graph transition or replaces exact terminal
  // access, detailed routing, connectivity, or DRC validation.  Zero disables the cue.
  std::uint64_t terminal_escape_reservation_cost{5'000};
  std::size_t max_terminal_escape_reservation_resources{200'000};
  // Forward-looking, non-authoritative cost.  A candidate pays when taking a corridor would
  // leave less capacity than the conservative demand that remains unavoidable for unrouted
  // nets.  Zero disables the lookahead without changing ordinary negotiated legality.
  std::uint64_t residual_corridor_demand_cost{600};
  // Protect topological cut resources whose unavoidable future demand leaves at most one lane of
  // slack. This is a strategic reservation cost, not a capacity or DRC rule: zero disables it.
  std::uint64_t critical_cut_cost{2'000};
  std::size_t critical_cut_minimum_future_demand{2};
  // Passes before the configured start use 1000. Later passes multiply and clamp the scale.
  std::uint32_t present_congestion_multiplier_per_mille{1'000};
  std::uint64_t max_present_congestion_scale_per_mille{1'000'000};
  std::size_t present_congestion_escalation_start_iteration{2};
  std::uint64_t corridor_centering_cost{20};
  // Before negotiated allocation, retain bounded exact hybrid-graph topology alternatives for
  // every ordinary connection. These are mandatory planning evidence, not copper.
  std::size_t max_static_seed_candidates_per_connection{2};
  std::size_t max_static_seed_searches{200'000};
  std::size_t max_static_seed_reprojection_searches{200'000};
  // Zero selects min(hardware concurrency, eligible NetTasks). A positive value is a runtime-only
  // cap; stable task slots and ordered reduction keep routing results independent of this value.
  std::size_t max_static_seed_workers{0};
  std::uint64_t static_seed_alternative_edge_penalty{2'000};
  std::uint64_t static_seed_corridor_departure_penalty{2'000};
  Nanometres global_spline_tessellation_tolerance{Nanometres::from_mm(0.05)};
  Nanometres global_spline_control_corridor{Nanometres::from_mm(2.0)};
  Nanometres global_spline_shove_step{Nanometres::from_mm(0.1)};
  // Build a clearance-expanded polygonal cell corridor and funnel centerline before fitting each
  // same-layer spline run. The corridor is a hard topology constraint during optimization.
  bool use_navmesh_funnel_spline_seeds{true};
  // Zero uses grid_pitch. A positive pitch controls only the polygonal navigation decomposition.
  Nanometres global_navmesh_cell_pitch{Nanometres{0}};
  std::size_t max_global_navmesh_cells_per_seed{100'000};
  std::size_t max_global_navmesh_search_expansions_per_seed{100'000};
  Nanometres global_spline_minimum_bend_radius{Nanometres::from_mm(0.25)};
  std::size_t global_spline_trajectory_samples{48};
  std::size_t max_global_spline_trajectory_iterations{8};
  std::size_t max_global_spline_trajectory_candidate_checks{512};
  std::size_t global_spline_max_tessellation_depth{10};
  std::size_t max_global_spline_samples_per_seed{1024};
  std::size_t max_global_spline_shove_iterations{1};
  std::size_t max_static_seed_corridor_cells{5'000'000};
  std::size_t max_global_pns_propagation_depth{8};
  std::size_t max_global_pns_candidate_checks{64};
  Nanometres maximum_global_pns_displacement{Nanometres::from_mm(2.0)};
  // Cost per grid-equivalent step of normalized explicit-width loss. Zero retains measurement
  // while disabling the routing and placement preference for nominal-width alternatives.
  std::uint64_t width_reduction_cost_per_grid_step{100};
  bool use_free_space_corridors{true};
  bool emit_voronoi_corridor_evidence{false};
  bool use_voronoi_pathfinder_guidance{false};
  bool use_voronoi_allocation_resources{false};
  // Zero preserves one capacity resource per retained skeleton edge. A positive value splits
  // ordinary (non-direct) medial edges into deterministic local cross-section intervals.
  Nanometres max_voronoi_resource_segment_length{Nanometres{0}};
  bool require_voronoi_topological_transitions{false};
  bool use_direct_voronoi_resource_edges{false};
  // Conventional materialization cannot realize several packed lanes on one direct chord.
  // Reserve every direct transition as a sampled capacity-one edge when that fallback is known
  // to be authoritative.
  bool single_lane_direct_voronoi_resources{false};
  // Grow each multi-terminal net by one bounded multi-source search from all remaining legal
  // terminal portals to the current tree. Original terminal indices remain stable path identities.
  bool use_multisource_steiner_growth{false};
  // Later negotiated passes may grow a connection from both endpoint access regions and solve a
  // bounded legal meeting region. The combined path is still materialized and DRC-checked as one
  // ordinary polyline; a failed bidirectional attempt falls back to normal one-sided A*.
  bool use_bidirectional_route_growth{true};
  std::size_t bidirectional_route_growth_start_iteration{2};
  std::size_t max_bidirectional_meeting_nodes{2};
  std::size_t max_bidirectional_meeting_expansions{5'000};
  // Present capacity is committed only when every terminal of a routed net has connected.
  // Incomplete attempts are rolled back as one whole-net transaction.
  bool atomic_complete_net_routing{false};
  // Between incomplete passes, preserve revalidated complete trees outside every congested
  // sampled, corridor, and shared-physical resource. Requires atomic complete-net routing.
  bool selective_congestion_reroute{false};
  // When selective rerouting is enabled, discard active whole-net reuse before every Nth next
  // pass while preserving history, ordering, and the best checkpoint. Zero disables refreshes.
  std::size_t selective_full_reroute_interval{0};
  // Zero keeps negotiated soft capacity for every pass. A positive pass index rejects transitions
  // that would exceed sampled edge/node, corridor, or selected-width physical capacity. Earlier
  // best checkpoints remain eligible.
  std::size_t hard_capacity_start_iteration{0};
  // A failed singleton net may displace already completed singleton blockers from the same pass,
  // retry first, and defer those victims. Both bounds are zero to disable and positive together.
  std::size_t max_hard_capacity_ripup_attempts_per_pass{0};
  std::size_t max_hard_capacity_ripup_nets_per_attempt{0};
  // Zero retains individual blocker ranking. A positive value bounds jointly sufficient victim
  // sets retained and scored for each failed-net repair attempt.
  std::size_t max_hard_capacity_conflict_set_states_per_attempt{0};
  // Restore the exact pre-rip-up route state when the failed net's immediate retry is still
  // incomplete. Victims are deferred only after a successful retry.
  bool rollback_failed_hard_capacity_ripup{false};
  // Requeue repair victims at the pass tail instead of immediately after the failed-net retry.
  // Every victim remains mandatory and may still finish unrouted.
  bool defer_hard_capacity_ripup_victims_to_pass_tail{false};
  // After a failed net is recovered by rip-up, do not select that recovered target as another
  // repair's victim during the same pass. Alternate eligible blockers may still be displaced.
  bool protect_recovered_hard_capacity_ripup_nets{false};
  // On a selective pass after hard-capacity failure, invalidate at most this many highest-evidence
  // blocker nets for each failed net in addition to the failed nets themselves. Zero disables.
  std::size_t max_hard_capacity_blocker_reuse_invalidations_per_failed_net{0};
  std::uint64_t voronoi_centering_cost{10};
  std::uint64_t voronoi_resource_present_cost{400};
  std::uint64_t voronoi_resource_history_cost{200};
  std::uint64_t differential_pair_resource_separation_cost{300};
  std::uint64_t routing_bus_resource_separation_cost{300};
  std::size_t max_voronoi_evidence_edges{200'000};
  std::size_t max_voronoi_guidance_cells{5'000'000};
  std::size_t max_voronoi_allocation_resources{200'000};
  std::size_t max_voronoi_lane_transitions{500'000};
  std::size_t max_voronoi_transition_clearance_checks{2'000'000};
  std::size_t max_voronoi_lane_orientation_resources_per_junction{12};
  std::size_t max_voronoi_lane_orientation_states{100'000};
  // Feed exact-rejected route-selected lane bridges back into later PathFinder passes. Direct
  // history is scoped to a net, resource, and snapped portal; ordinary history is scoped to a net
  // and exact-adjacent resource pair. Neither owns capacity or authorizes copper.
  bool negotiate_voronoi_lane_conflicts{false};
  std::uint64_t voronoi_lane_conflict_history_cost{2'000};
  std::size_t max_voronoi_lane_conflict_history_entries{200'000};
  bool reorder_nets_by_pathfinder_pressure{true};
  // Opt-in bounded escape policy. Soft PathFinder passes may already retain provisional
  // congestion/clearance illegality; this policy keeps several diverse checkpoints and, after
  // stagnation, progressively destroys a larger routing neighborhood before rebuilding it.
  bool enable_search_diversification{false};
  std::size_t global_route_beam_width{8};
  std::size_t max_large_neighborhood_attempts{5};
  std::size_t large_neighborhood_stagnation_iterations{2};
  std::size_t destroy_route_percent{5};
  std::size_t max_routing_order_variants{8};
  std::uint64_t routing_order_diversification_seed{0x4f1bbcdc6762c2d3ULL};
  // Within equal priority/completion tiers, schedule larger resolved width-plus-clearance groups
  // before dynamic pressure and stable-ID tie-breaks.
  bool order_routing_groups_by_physical_envelope{false};
  bool respect_existing_copper{true};
  // Opt-in planning only: saved KiCad zone fills are omitted as obstacles because the caller will
  // refill them after export. This never makes an unrefilled detailed candidate acceptable.
  bool treat_saved_zone_fills_as_refillable{false};
};

struct GlobalRoutePoint {
  Point position;
  LayerId layer;

  bool operator==(const GlobalRoutePoint&) const = default;
};

struct GlobalRouteMedialTransitionEdge {
  GlobalRoutePoint first;
  GlobalRoutePoint second;

  bool operator==(const GlobalRouteMedialTransitionEdge&) const = default;
};

struct GlobalRoutingShoveProposal {
  std::size_t id{0};
  NetId intruding_net;
  std::vector<NetId> affected_nets;
  std::vector<ObjectId> replaced_tracks;
  std::vector<TrackSegment> replacement_tracks;
  std::vector<ObjectId> replaced_vias;
  std::vector<Via> replacement_vias;
  std::uint64_t displacement_cost_nm{0};
  std::size_t propagation_depth{0};

  bool operator==(const GlobalRoutingShoveProposal&) const = default;
};

enum class GlobalRouteSegmentWidthKind {
  nominal,
  terminal_neck_down,
  medial_neck_down,
  corridor_neck_down
};

// Continuous free-space evidence retained from one selected NavMesh corridor. The cell polygons
// and portals are the topology guard; lane_centerline is the optimized B-spline tessellation, not
// a rasterized PathFinder surrogate. Widths describe conservative source-space copper envelopes.
struct RoutingGeometricCorridorRun {
  LayerId layer;
  std::vector<Polygon> topology_cells;
  std::vector<NavMeshPortal> portals;
  std::vector<Point> lane_centerline;
  Nanometres required_envelope_width;
  Nanometres minimum_physical_cross_section_width;
  std::size_t lane_capacity{0};
  bool selected_path_preserves_topology{false};

  bool operator==(const RoutingGeometricCorridorRun&) const = default;
};

struct GlobalRoutePath {
  NetId net;
  std::size_t connection_index{0};
  std::vector<GlobalRoutePoint> points;
  std::vector<std::size_t> medial_resource_sequence;
  // Aligned with transitions between consecutive medial resources. A point identifies an exact
  // snapped direct-edge portal; null retains ordinary shared-skeleton endpoint adjacency.
  std::vector<std::optional<Point>> medial_transition_portals;
  // Aligned with the same transitions. An entry is the exact selected sampled/grid edge across
  // which the compressed resource label changed; null means no adjacent sampled edge was proven.
  std::vector<std::optional<GlobalRouteMedialTransitionEdge>> medial_transition_grid_edges;
  std::size_t medial_topology_breaks{0};
  std::size_t medial_direct_portal_transitions{0};
  std::size_t medial_unmapped_nodes{0};
  std::size_t medial_interior_unmapped_nodes{0};
  std::size_t direct_medial_edges{0};
  std::size_t direct_medial_grid_steps_bypassed{0};
  // Aligned with point-to-point edges when non-empty. Zero means the net's resolved nominal width;
  // a positive value is an explicitly authorized width. Kinds retain its policy provenance.
  std::vector<Nanometres> segment_widths;
  std::vector<GlobalRouteSegmentWidthKind> segment_width_kinds;
  std::vector<RoutingGeometricCorridorRun> geometric_corridors;
  std::size_t terminal_neck_down_segments{0};
  std::uint64_t terminal_neck_down_length_nm{0};
  std::size_t medial_neck_down_segments{0};
  std::uint64_t medial_neck_down_length_nm{0};
  std::size_t corridor_neck_down_segments{0};
  std::uint64_t corridor_neck_down_length_nm{0};

  [[nodiscard]] bool medial_topology_complete() const {
    return !medial_resource_sequence.empty() && medial_topology_breaks == 0U &&
           medial_interior_unmapped_nodes == 0U;
  }

  bool operator==(const GlobalRoutePath&) const = default;
};

enum class GlobalRouterStopReason {
  complete,
  stagnated,
  iteration_limit,
  invalid_input,
  cancelled
};

struct RoutingObstacleImpact {
  NetId routed_net;
  ObjectId obstacle;
  std::size_t rejected_edges{0};

  bool operator==(const RoutingObstacleImpact&) const = default;
};

// Exact route-selected evidence that a narrower segment passed where the resolved nominal width
// was blocked by a represented object. Board-boundary-only blockages have no movable object and
// are deliberately omitted.
struct RoutingNeckDownObstacleImpact {
  NetId routed_net;
  ObjectId obstacle;
  GlobalRouteSegmentWidthKind width_kind{GlobalRouteSegmentWidthKind::nominal};
  LayerId layer;
  Point start;
  Point end;
  Nanometres selected_width;

  bool operator==(const RoutingNeckDownObstacleImpact&) const = default;
};

struct PinEscapeFailure {
  ObjectId footprint;
  ObjectId pad;
  NetId net;
  Point position;
  std::vector<ObjectId> nominal_center_blockers;
  std::vector<ObjectId> neck_down_center_blockers;
  bool nominal_center_board_blocked{false};
  bool neck_down_center_board_blocked{false};

  bool operator==(const PinEscapeFailure&) const = default;
};

struct RoutingTerminalEvidence {
  ObjectId footprint;
  ObjectId pad;
  Point position;

  bool operator==(const RoutingTerminalEvidence&) const = default;
};

enum class RoutingConnectionFailureReason {
  no_access_nodes,
  search_frontier_exhausted,
  expansion_limit,
  unmatched_multisource_terminal
};

// Evidence from the selected PathFinder checkpoint. Attempted terminals are the unconnected side
// of the failed search; connected terminals identify the already built net tree.
struct RoutingConnectionFailure {
  NetId net;
  std::size_t completed_connections{0};
  bool multisource{false};
  bool direct_edge_fallback_attempted{false};
  RoutingConnectionFailureReason reason{RoutingConnectionFailureReason::search_frontier_exhausted};
  std::vector<RoutingTerminalEvidence> attempted_terminals;
  std::vector<RoutingTerminalEvidence> connected_terminals;
  std::size_t expansions{0};
  std::size_t topology_rejected_edges{0};

  bool operator==(const RoutingConnectionFailure&) const = default;
};

// Constructive-pass evidence is retained separately from the selected feasibility-first
// checkpoint. It explains which schedule was attempted and which nets could not be connected
// without exceeding a represented capacity.
struct RoutingHardCapacityPass {
  std::size_t iteration{0};
  std::vector<NetId> attempted_order;
  std::vector<NetId> unrouted_nets;
  std::vector<RoutingConnectionFailure> connection_failures;

  bool operator==(const RoutingHardCapacityPass&) const = default;
};

struct RoutingHardCapacityNetFailure {
  NetId net;
  std::size_t failed_passes{0};

  bool operator==(const RoutingHardCapacityNetFailure&) const = default;
};

struct RoutingHardCapacityBlocker {
  NetId failed_net;
  NetId blocking_net;
  std::size_t rejected_transitions{0};

  bool operator==(const RoutingHardCapacityBlocker&) const = default;
};

struct RoutingHardCapacityRipupAttempt {
  std::size_t iteration{0};
  NetId failed_net;
  std::vector<NetId> victims;
  bool retry_completed{false};
  bool retained_at_pass_end{false};
  bool rolled_back_after_failed_retry{false};

  bool operator==(const RoutingHardCapacityRipupAttempt&) const = default;
};

enum class RoutingResourceKind { node, planar_edge, via_edge, medial_edge };

struct RoutingResourceUsage {
  RoutingResourceKind kind{RoutingResourceKind::node};
  GlobalRoutePoint first;
  GlobalRoutePoint second;
  std::size_t demand{0};
  std::size_t capacity{0};
  std::vector<NetId> nets;

  [[nodiscard]] bool congested() const { return demand > capacity; }
  bool operator==(const RoutingResourceUsage&) const = default;
};

enum class RoutingClearanceShadowResourceKind { planar_edge, terminal_or_via_node };

// A selected centreline occupies a sampled edge/node lying inside another selected route's exact
// clearance shadow. The ordered net pair and physical deficit make sub-grid incompatibility a
// stable negotiated resource rather than a late, unattributed detailed-DRC surprise.
struct RoutingClearanceShadowConflict {
  RoutingClearanceShadowResourceKind kind{RoutingClearanceShadowResourceKind::planar_edge};
  GlobalRoutePoint first;
  GlobalRoutePoint second;
  NetId first_net;
  NetId second_net;
  Nanometres clearance_deficit;
  // Occupied edge or terminal/via portal that cast the shadow. These are absent only when a
  // non-lattice access fragment cannot be bound to a selected portal. They make both sides of the
  // negotiated conflict independently actionable.
  std::optional<GlobalRoutePoint> shadow_owner_first;
  std::optional<GlobalRoutePoint> shadow_owner_second;

  bool operator==(const RoutingClearanceShadowConflict&) const = default;
};

enum class RoutingCorridorOrientation { horizontal, vertical };

// A deterministic constant-cross-section segment of free space. Capacity is the number of
// distinct grid lanes in that cross-section; it never authorizes coincident physical traces.
struct RoutingCorridorUsage {
  std::size_t id{0};
  LayerId layer;
  RoutingCorridorOrientation orientation{RoutingCorridorOrientation::horizontal};
  Point first_lane_start;
  Point last_lane_end;
  Point sampled_medial_start;
  Point sampled_medial_end;
  std::size_t demand{0};
  std::size_t capacity{0};
  Nanometres cross_section_span;
  bool neck_down{false};
  std::vector<NetId> nets;

  [[nodiscard]] bool congested() const { return demand > capacity; }
  bool operator==(const RoutingCorridorUsage&) const = default;
};

// Net/layer-specific, exact-rechecked medial edge produced from conservatively offset free space.
// This is bounded advisory evidence until terminal access and junction resources are integrated
// into the production PathFinder graph.
struct RoutingVoronoiCorridorEvidence {
  std::size_t id{0};
  NetId net;
  LayerId layer;
  Point start;
  Point end;
  Nanometres minimum_boundary_clearance;
  std::size_t capacity{0};
  bool conservative_domain{false};
  bool curved_bisector_chord{false};

  bool operator==(const RoutingVoronoiCorridorEvidence&) const = default;
};

// Shared physical medial resource. Capacity and demand are conservative cross-section envelopes
// in nanometres, so heterogeneous net widths can negotiate one channel without coincident tracks.
struct RoutingVoronoiResourceUsage {
  std::size_t id{0};
  LayerId layer;
  Point start;
  Point end;
  Nanometres channel_width;
  std::uint64_t demand_nm{0};
  std::uint64_t capacity_nm{0};
  bool conservative_domain{false};
  std::vector<NetId> nets;
  bool curved_bisector_chord{false};

  [[nodiscard]] bool congested() const { return demand_nm > capacity_nm; }
  bool operator==(const RoutingVoronoiResourceUsage&) const = default;
};

// Source-bound physical objects whose represented polygon contributes the conservative width
// certificate of an over-capacity shared channel. Outline/cutout limitations remain observable
// through the resource itself but deliberately do not fabricate a movable owner.
struct RoutingPhysicalObstacleImpact {
  std::size_t resource_id{0};
  ObjectId obstacle;
  LayerId layer;
  Point start;
  Point end;
  std::uint64_t overflow_nm{0};
  std::vector<NetId> routed_nets;

  bool operator==(const RoutingPhysicalObstacleImpact&) const = default;
};

// Per-net participation in negotiated congestion. Discrete lane excess and physical nanometre
// excess remain separate; the legacy resource count preserves aggregate-objective ordering.
struct RoutingNetPressure {
  NetId net;
  std::size_t legacy_congested_resources{0};
  std::size_t sampled_overflow{0};
  std::size_t corridor_overflow{0};
  std::uint64_t total_physical_overflow_nm{0};
  std::uint64_t peak_physical_overflow_nm{0};
  std::size_t congested_physical_resources{0};
  std::size_t clearance_shadow_overflow{0};

  bool operator==(const RoutingNetPressure&) const = default;
};

// Connected component of nets that still share one or more over-capacity planning resources.
// Resource indices bind the exact selected checkpoint evidence. The group is a bounded detailed
// repair problem, never permission to accept the planning overlap as copper.
struct RoutingResidualConflictGroup {
  std::size_t id{0};
  std::vector<NetId> nets;
  std::vector<std::size_t> sampled_resource_indices;
  std::vector<std::size_t> corridor_resource_ids;
  std::vector<std::size_t> physical_resource_ids;
  std::vector<std::size_t> clearance_shadow_resource_indices;
  std::size_t sampled_overflow{0};
  std::size_t corridor_overflow{0};
  std::uint64_t physical_overflow_nm{0};
  std::size_t clearance_shadow_overflow{0};
  std::size_t peak_geometric_lane_capacity{0};
  std::size_t path_count{0};
  std::size_t multilayer_path_count{0};
  bool all_nets_have_preserved_geometric_corridors{false};

  bool operator==(const RoutingResidualConflictGroup&) const = default;
};

// Deterministic cross-section packing proposal. Offsets and shifted segments are advisory until
// the detailed router exact-rechecks a continuous parallel trace against the physical board.
struct RoutingVoronoiLaneAssignment {
  std::size_t resource_id{0};
  NetId net;
  std::int64_t signed_offset_nm{0};
  Nanometres track_width;
  Point shifted_start;
  Point shifted_end;
  bool differential_pair_member{false};
  std::optional<RoutingBusId> routing_bus;
  bool static_clearance_checked{false};

  bool operator==(const RoutingVoronoiLaneAssignment&) const = default;
};

// Route-selected bridge proposal between packed lane chords at one medial junction. A miter trims
// or extends both chords to their shifted-line intersection; absence means a direct bridge.
// Static and mutual clearance are separate because neither proves a complete routed topology.
struct RoutingVoronoiLaneTransition {
  std::size_t first_resource_id{0};
  std::size_t second_resource_id{0};
  NetId net;
  LayerId layer;
  Point medial_junction;
  Point first_lane_endpoint;
  Point second_lane_endpoint;
  Nanometres track_width;
  bool differential_pair_member{false};
  std::optional<RoutingBusId> routing_bus;
  bool static_clearance_checked{false};
  bool mutual_clearance_checked{false};
  std::optional<Point> miter;
  bool direct_portal_bridge{false};

  bool operator==(const RoutingVoronoiLaneTransition&) const = default;
};

// Iteration-zero scheduling evidence derived from already prepared terminal access and static
// route portfolios. The wave changes only provisional seed assembly; ordinary PathFinder
// negotiation and every physical/exact acceptance gate remain authoritative.
enum class IterationZeroRoutingWave { trivial, constrained, ordinary, flexible };

struct RoutingSeedPriorityEvidence {
  std::string group;
  std::vector<NetId> nets;
  IterationZeroRoutingWave wave{IterationZeroRoutingWave::ordinary};
  std::uint32_t explicit_priority{0};
  std::size_t minimum_terminal_portals{0};
  std::size_t viable_tree_candidates{0};
  std::size_t viable_corridor_candidates{0};
  std::size_t topology_alternatives{0};
  std::size_t layer_alternatives{0};
  std::size_t primary_via_transitions{0};
  std::size_t viable_via_sites{0};
  std::size_t via_assignment_alternatives{0};
  std::size_t via_site_contention{0};
  // Number of other routing groups whose terminal-MST airwire crosses this group's airwire
  // forest. This is a topology-pressure estimate used only to order initial seed allocation;
  // it neither reserves capacity nor establishes a routing conflict.
  std::size_t terminal_crossing_pressure{0};
  std::size_t estimated_interference{0};
  std::uint64_t physical_envelope_nm{0};
  std::size_t primary_route_steps{0};
  // Saturating product of independent escape/layer/corridor/via choices. Smaller values are
  // scheduled first within the constrained wave. Zero means no complete routing freedom.
  std::uint64_t routing_freedom_score{0};
  bool touches_scarce_resource{false};

  bool operator==(const RoutingSeedPriorityEvidence&) const = default;
};

struct GlobalRouteResult {
  std::vector<GlobalRoutePath> paths;
  std::vector<NetId> unrouted_nets;
  std::vector<RoutingConnectionFailure> connection_failures;
  std::vector<NetId> configured_selected_nets;
  std::vector<NetId> configured_priority_nets;
  Nanometres configured_grid_pitch{Nanometres{0}};
  Nanometres configured_via_site_pitch{Nanometres{0}};
  std::size_t configured_via_site_phase_x{0};
  std::size_t configured_via_site_phase_y{0};
  bool configured_diagonal_grid_edges{false};
  bool configured_crossing_safe_diagonal_grid_edges{false};
  bool configured_sampled_route_clearance_reservations{false};
  std::size_t configured_max_clearance_shadow_conflict_resources{0};
  Nanometres configured_clearance_shadow_cost_quantum;
  bool configured_single_lane_direct_voronoi_resources{false};
  std::size_t configured_max_static_seed_candidates_per_connection{0};
  std::size_t configured_max_static_seed_searches{0};
  std::size_t configured_max_static_seed_reprojection_searches{0};
  std::size_t configured_max_static_seed_workers{0};
  std::uint64_t configured_static_seed_alternative_edge_penalty{0};
  std::uint64_t configured_static_seed_corridor_departure_penalty{0};
  std::uint64_t configured_residual_corridor_demand_cost{0};
  std::uint64_t configured_critical_cut_cost{0};
  std::size_t configured_critical_cut_minimum_future_demand{0};
  std::uint64_t configured_river_following_cost{0};
  std::size_t river_following_edges{0};
  std::uint64_t river_following_cost_discount{0};
  std::uint64_t configured_former_route_memory_cost{0};
  std::size_t former_route_memory_ripped_paths{0};
  std::size_t former_route_memory_edges{0};
  std::size_t former_route_memory_penalty_events{0};
  std::uint64_t former_route_memory_penalty{0};
  std::uint64_t configured_terminal_escape_reservation_cost{0};
  std::size_t configured_max_terminal_escape_reservation_resources{0};
  std::size_t terminal_escape_reservation_terminals{0};
  std::size_t terminal_escape_reservation_candidates{0};
  std::size_t terminal_escape_reservation_resources{0};
  std::size_t terminal_escape_reservation_withheld_candidates{0};
  std::size_t terminal_escape_reservation_penalty_events{0};
  std::uint64_t terminal_escape_reservation_penalty{0};
  std::size_t residual_corridor_demand_nets{0};
  std::size_t residual_corridor_demand_resources{0};
  std::size_t residual_corridor_demand_penalty_events{0};
  std::uint64_t residual_corridor_demand_penalty{0};
  std::size_t critical_cut_resources{0};
  std::size_t critical_cut_reserved_demand{0};
  std::size_t critical_cut_penalty_events{0};
  std::uint64_t critical_cut_penalty{0};
  Nanometres configured_global_spline_tessellation_tolerance;
  Nanometres configured_global_spline_control_corridor;
  Nanometres configured_global_spline_shove_step;
  bool configured_navmesh_funnel_spline_seeds{false};
  Nanometres configured_global_navmesh_cell_pitch;
  Nanometres configured_global_spline_minimum_bend_radius;
  std::size_t configured_max_global_navmesh_cells_per_seed{0};
  std::size_t configured_max_global_navmesh_search_expansions_per_seed{0};
  std::size_t configured_global_spline_trajectory_samples{0};
  std::size_t configured_max_global_spline_trajectory_iterations{0};
  std::size_t configured_max_global_spline_trajectory_candidate_checks{0};
  std::size_t configured_global_spline_max_tessellation_depth{0};
  std::size_t configured_max_global_spline_samples_per_seed{0};
  std::size_t configured_max_global_spline_shove_iterations{0};
  std::size_t configured_max_static_seed_corridor_cells{0};
  std::size_t configured_max_global_pns_propagation_depth{0};
  std::size_t configured_max_global_pns_candidate_checks{0};
  Nanometres configured_maximum_global_pns_displacement;
  std::size_t static_seed_connections{0};
  std::size_t static_seed_tasks_dispatched{0};
  std::size_t static_seed_workers_used{0};
  std::size_t static_seed_searches{0};
  std::size_t static_seed_candidates{0};
  std::size_t static_seed_search_expansions{0};
  std::size_t static_seed_penalty_reruns{0};
  std::size_t static_seed_deduplicated_candidates{0};
  std::size_t static_seed_via_bearing_candidates{0};
  std::size_t static_seed_direct_medial_candidates{0};
  std::size_t static_seed_terminals_without_seed{0};
  std::size_t static_seed_distinct_layer_sequences{0};
  std::size_t static_seed_distinct_topology_signatures{0};
  std::size_t static_seed_corridor_nodes{0};
  std::size_t static_seed_spline_samples{0};
  std::size_t static_seed_navmesh_domains{0};
  std::size_t static_seed_navmesh_routes{0};
  std::size_t static_seed_navmesh_cells{0};
  std::size_t static_seed_navmesh_search_expansions{0};
  std::size_t static_seed_flood_fill_routes{0};
  std::size_t static_seed_flood_fill_cells_visited{0};
  std::size_t static_seed_navmesh_failures{0};
  std::size_t static_seed_navmesh_certificate_domains{0};
  std::size_t static_seed_funnel_points{0};
  std::size_t static_seed_trajectory_attempts{0};
  std::size_t static_seed_trajectory_accepted{0};
  std::size_t static_seed_trajectory_control_moves{0};
  std::size_t static_seed_trajectory_candidate_checks{0};
  std::size_t static_seed_trajectory_rejected{0};
  std::size_t negotiated_spline_guided_searches{0};
  std::size_t negotiated_spline_corridor_departures{0};
  std::size_t static_seed_collision_envelopes{0};
  std::size_t static_seed_control_shoves{0};
  std::size_t static_seed_refits{0};
  std::size_t static_seed_refined_candidates{0};
  std::size_t static_seed_reprojection_searches{0};
  std::size_t static_seed_reprojections_accepted{0};
  std::size_t static_seed_reprojections_reverted{0};
  std::size_t static_seed_corridor_departures{0};
  std::size_t static_seed_complete_trees{0};
  std::size_t static_seed_partial_trees{0};
  std::size_t static_seed_alternative_trees{0};
  std::size_t static_seed_tree_joins{0};
  std::size_t static_seed_growth_reorderings{0};
  std::size_t static_seed_joint_groups{0};
  std::size_t static_seed_joint_group_candidates{0};
  std::size_t static_seed_joint_groups_withheld{0};
  bool static_seed_budget_exhausted{false};
  std::vector<GlobalRoutingShoveProposal> selected_shove_proposals;
  std::size_t global_pns_proposal_attempts{0};
  std::size_t global_pns_eligible_collisions{0};
  std::size_t global_pns_ineligible_collisions{0};
  std::size_t global_pns_candidate_checks{0};
  std::size_t global_pns_local_splits{0};
  std::size_t global_pns_proposals_accepted{0};
  std::size_t iteration_zero_paths{0};
  std::size_t iteration_zero_unrouted_nets{0};
  std::size_t iteration_zero_total_overflow{0};
  std::size_t iteration_zero_sampled_overflow{0};
  std::size_t iteration_zero_corridor_overflow{0};
  std::size_t iteration_zero_portfolios_evaluated{0};
  std::size_t iteration_zero_candidate_index{0};
  std::size_t iteration_zero_seed_history_increments{0};
  bool iteration_zero_selected{false};
  bool iteration_zero_occupancy_aware_portfolio_selected{false};
  bool iteration_zero_staged_priority_portfolio_selected{false};
  bool iteration_zero_staged_priority_qualified{false};
  std::size_t iteration_zero_baseline_unrouted_nets{0};
  std::size_t iteration_zero_staged_unrouted_nets{0};
  std::size_t iteration_zero_baseline_total_overflow{0};
  std::size_t iteration_zero_staged_total_overflow{0};
  std::size_t iteration_zero_baseline_clearance_shadow_conflicts{0};
  std::size_t iteration_zero_staged_clearance_shadow_conflicts{0};
  std::vector<RoutingSeedPriorityEvidence> iteration_zero_seed_priority_evidence;
  std::size_t iteration_zero_trivial_groups{0};
  std::size_t iteration_zero_constrained_groups{0};
  std::size_t iteration_zero_ordinary_groups{0};
  std::size_t iteration_zero_flexible_groups{0};
  std::size_t iterations{0};
  std::size_t selected_iteration{0};
  std::size_t total_overflow{0};
  std::size_t sampled_overflow{0};
  std::size_t corridor_overflow{0};
  std::size_t clearance_shadow_overflow{0};
  std::size_t routability_history_entries{0};
  std::size_t routability_history_edges{0};
  std::uint64_t routability_history_pressure{0};
  bool routability_history_budget_exhausted{false};
  std::size_t clearance_shadow_history_increment{0};
  std::size_t peak_clearance_shadow_history_increment{0};
  bool clearance_shadow_owner_history_phase_attempted{false};
  std::size_t clearance_shadow_owner_history_passes{0};
  bool clearance_shadow_conflict_budget_exhausted{false};
  std::uint64_t total_cost{0};
  GlobalRouteCheckpointObjective configured_checkpoint_objective{
      GlobalRouteCheckpointObjective::aggregate_overflow};
  bool configured_scale_voronoi_history_by_overflow{false};
  std::uint64_t voronoi_history_quantum_nm{0};
  std::size_t total_voronoi_history_increment{0};
  std::size_t peak_voronoi_history_increment{0};
  std::uint32_t configured_present_congestion_multiplier_per_mille{1'000};
  std::uint64_t configured_max_present_congestion_scale_per_mille{1'000'000};
  std::size_t configured_present_congestion_escalation_start_iteration{2};
  std::uint64_t selected_present_congestion_scale_per_mille{1'000};
  std::uint64_t selected_effective_present_congestion_cost{0};
  std::uint64_t selected_effective_corridor_present_cost{0};
  std::uint64_t selected_effective_voronoi_resource_present_cost{0};
  std::size_t configured_max_expansions_per_connection{0};
  std::uint32_t configured_expansion_retry_heuristic_weight_per_mille{0};
  std::size_t path_search_invocations{0};
  std::size_t path_search_expansions{0};
  std::size_t expansion_limited_searches{0};
  std::size_t weighted_search_retries{0};
  std::size_t weighted_search_recoveries{0};
  bool configured_reuse_weighted_search_frontier{false};
  std::size_t weighted_frontier_continuations{0};
  std::size_t weighted_frontier_recoveries{0};
  std::size_t weighted_frontier_states_reused{0};
  std::size_t weighted_frontier_continuation_limits{0};
  std::size_t blocked_search_edges{0};
  std::size_t board_edge_rejections{0};
  std::size_t zone_hole_broad_phase_rejections{0};
  std::size_t zone_hole_exact_checks{0};
  std::size_t failed_pin_escapes{0};
  std::vector<PinEscapeFailure> pin_escape_failures;
  std::size_t pin_access_nodes{0};
  Nanometres configured_local_pin_escape_pitch{Nanometres{0}};
  std::size_t configured_max_local_pin_escape_expansions_per_terminal{0};
  std::size_t local_pin_escape_paths{0};
  std::size_t local_pin_escape_expansions{0};
  std::size_t terminals_with_local_pin_escape{0};
  std::size_t same_net_terminal_fanout_paths{0};
  std::size_t terminals_with_same_net_fanout{0};
  bool configured_contended_terminal_fanout{false};
  std::size_t contended_terminals{0};
  std::size_t contention_triggered_local_paths{0};
  std::size_t adaptive_pin_access_nodes{0};
  std::size_t terminals_with_adaptive_pin_access{0};
  bool configured_diversify_pin_access_rings{false};
  std::size_t diversified_pin_access_nodes{0};
  std::size_t terminals_with_diversified_pin_access{0};
  std::size_t additional_successful_pin_access_rings{0};
  std::size_t isolated_pin_access_nodes_rejected{0};
  std::size_t terminals_with_isolated_pin_access_rejections{0};
  std::size_t terminal_neck_down_access_nodes{0};
  std::size_t terminals_with_neck_down_access{0};
  std::size_t coupled_terminals_ineligible_for_neck_down{0};
  Nanometres configured_terminal_neck_down_width;
  Nanometres configured_max_terminal_neck_down_length;
  std::size_t used_terminal_neck_down_segments{0};
  std::uint64_t total_terminal_neck_down_length_nm{0};
  Nanometres configured_medial_neck_down_width;
  Nanometres configured_max_medial_neck_down_length;
  std::size_t medial_neck_down_edges_available{0};
  std::size_t used_medial_neck_down_segments{0};
  std::uint64_t total_medial_neck_down_length_nm{0};
  Nanometres configured_corridor_neck_down_width;
  Nanometres configured_max_corridor_neck_down_length;
  std::size_t corridor_neck_down_search_states{0};
  std::size_t coupled_nets_ineligible_for_corridor_neck_down{0};
  std::size_t used_corridor_neck_down_segments{0};
  std::uint64_t total_corridor_neck_down_length_nm{0};
  std::uint64_t configured_width_reduction_cost_per_grid_step{0};
  std::uint64_t total_equivalent_width_reduction_length_nm{0};
  std::uint64_t total_width_reduction_cost{0};
  std::size_t topology_rejected_search_edges{0};
  std::size_t configured_hard_capacity_start_iteration{0};
  std::size_t hard_capacity_passes{0};
  std::size_t post_negotiation_cone_repair_attempts{0};
  std::size_t post_negotiation_cone_repairs_accepted{0};
  std::size_t post_negotiation_cone_repair_nets_attempted{0};
  std::size_t hard_sampled_capacity_rejected_transitions{0};
  std::size_t hard_corridor_capacity_rejected_transitions{0};
  std::size_t hard_voronoi_capacity_rejected_transitions{0};
  std::size_t hard_capacity_diversified_passes{0};
  std::size_t hard_capacity_diversified_net_positions{0};
  std::vector<RoutingHardCapacityPass> hard_capacity_pass_summaries;
  std::vector<RoutingHardCapacityNetFailure> hard_capacity_net_failures;
  std::vector<RoutingHardCapacityBlocker> hard_capacity_blockers;
  std::size_t configured_max_hard_capacity_ripup_attempts_per_pass{0};
  std::size_t configured_max_hard_capacity_ripup_nets_per_attempt{0};
  std::size_t hard_capacity_ripup_attempts{0};
  std::size_t hard_capacity_ripped_nets{0};
  std::size_t hard_capacity_ripup_recoveries{0};
  std::size_t hard_capacity_ripup_retained_recoveries{0};
  std::vector<RoutingHardCapacityRipupAttempt> hard_capacity_ripup_records;
  std::size_t configured_max_hard_capacity_conflict_set_states_per_attempt{0};
  std::size_t hard_capacity_conflict_set_states{0};
  std::size_t hard_capacity_joint_ripup_attempts{0};
  std::size_t hard_capacity_conflict_sets_withheld{0};
  bool configured_rollback_failed_hard_capacity_ripup{false};
  std::size_t hard_capacity_failed_retry_rollbacks{0};
  bool configured_defer_hard_capacity_ripup_victims_to_pass_tail{false};
  std::size_t hard_capacity_tail_deferred_victims{0};
  bool configured_protect_recovered_hard_capacity_ripup_nets{false};
  std::size_t hard_capacity_protected_recoveries{0};
  std::size_t hard_capacity_protected_victim_rejections{0};
  std::size_t configured_max_hard_capacity_blocker_reuse_invalidations_per_failed_net{0};
  std::size_t hard_capacity_blocker_reuse_invalidations{0};
  std::size_t medial_topology_breaks{0};
  std::size_t medial_direct_portal_transitions{0};
  std::size_t medial_unmapped_nodes{0};
  std::size_t medial_interior_unmapped_nodes{0};
  std::size_t topologically_complete_medial_paths{0};
  std::size_t direct_voronoi_edges_available{0};
  std::size_t direct_voronoi_edges_withheld{0};
  std::size_t used_direct_voronoi_edges{0};
  std::size_t direct_voronoi_grid_steps_bypassed{0};
  std::size_t direct_voronoi_search_fallbacks{0};
  bool multisource_steiner_growth_enabled{false};
  std::size_t multisource_steiner_connections{0};
  std::size_t multisource_steiner_reordered_connections{0};
  std::size_t multisource_steiner_deferred_inaccessible_terminals{0};
  std::size_t multisource_steiner_fixed_order_fallbacks{0};
  std::size_t multisource_steiner_fixed_order_recoveries{0};
  bool bidirectional_route_growth_enabled{false};
  std::size_t configured_bidirectional_route_growth_start_iteration{0};
  std::size_t configured_max_bidirectional_meeting_nodes{0};
  std::size_t configured_max_bidirectional_meeting_expansions{0};
  std::size_t bidirectional_route_growth_attempts{0};
  std::size_t bidirectional_route_growth_meeting_nodes{0};
  std::size_t bidirectional_route_growth_accepted{0};
  std::size_t bidirectional_route_growth_fallbacks{0};
  bool atomic_complete_net_routing_enabled{false};
  bool configured_selective_congestion_reroute{false};
  std::size_t configured_selective_full_reroute_interval{0};
  std::size_t selective_reroute_passes{0};
  std::size_t selective_full_reroute_refreshes{0};
  std::size_t selective_retained_net_observations{0};
  std::size_t selective_retained_path_observations{0};
  std::size_t selective_conflict_net_observations{0};
  std::size_t selective_coupled_group_expansions{0};
  std::size_t rolled_back_incomplete_net_trees{0};
  std::size_t rolled_back_incomplete_net_paths{0};
  std::uint64_t rolled_back_incomplete_net_cost{0};
  std::vector<RoutingObstacleImpact> obstacle_impacts;
  std::vector<RoutingNeckDownObstacleImpact> neck_down_obstacle_impacts;
  std::vector<RoutingResourceUsage> resource_usage;
  std::vector<RoutingClearanceShadowConflict> clearance_shadow_conflicts;
  std::vector<RoutingCorridorUsage> corridor_usage;
  std::vector<RoutingCorridorUsage> corridor_neck_down_usage;
  std::vector<RoutingVoronoiCorridorEvidence> voronoi_corridor_evidence;
  std::vector<RoutingVoronoiResourceUsage> voronoi_resource_usage;
  std::vector<RoutingPhysicalObstacleImpact> physical_obstacle_impacts;
  std::vector<RoutingVoronoiLaneAssignment> voronoi_lane_assignments;
  std::vector<RoutingVoronoiLaneTransition> voronoi_lane_transitions;
  std::size_t voronoi_domains_attempted{0};
  std::size_t voronoi_domains_accepted{0};
  std::size_t voronoi_domains_rejected{0};
  std::size_t voronoi_conservative_obstacles{0};
  std::size_t voronoi_evidence_curved_edge_chords{0};
  std::size_t voronoi_withheld_curved_edges{0};
  std::size_t voronoi_guided_nets{0};
  std::size_t voronoi_guidance_seed_nodes{0};
  std::size_t congested_voronoi_resources{0};
  std::size_t width_reduced_voronoi_allocations{0};
  std::uint64_t total_voronoi_demand_reduction_nm{0};
  std::size_t voronoi_allocation_curved_edge_chords{0};
  std::size_t voronoi_allocation_withheld_curved_edges{0};
  Nanometres configured_max_voronoi_resource_segment_length;
  std::size_t segmented_voronoi_source_resources{0};
  std::size_t additional_voronoi_segment_resources{0};
  std::size_t direct_voronoi_resources_retained_atomic{0};
  std::uint64_t total_voronoi_overflow_nm{0};
  std::uint64_t peak_voronoi_overflow_nm{0};
  std::uint64_t peak_voronoi_demand_nm{0};
  std::size_t coupled_differential_pair_resources{0};
  std::size_t coupled_routing_bus_resources{0};
  std::size_t packed_voronoi_resources{0};
  std::size_t static_clearance_checked_voronoi_lanes{0};
  std::size_t accepted_voronoi_lane_transitions{0};
  std::size_t unaccepted_selected_voronoi_lane_transitions{0};
  std::size_t direct_portal_lane_transitions{0};
  std::size_t accepted_direct_portal_lane_transitions{0};
  std::size_t unaccepted_direct_portal_lane_transitions{0};
  bool configured_negotiate_voronoi_lane_conflicts{false};
  std::uint64_t configured_voronoi_lane_conflict_history_cost{0};
  std::size_t configured_max_voronoi_lane_conflict_history_entries{0};
  std::size_t voronoi_lane_conflict_passes{0};
  std::size_t voronoi_lane_conflict_observations{0};
  std::size_t voronoi_lane_conflict_history_entries{0};
  std::size_t voronoi_exact_junction_conflict_history_entries{0};
  std::size_t voronoi_selected_junction_edge_conflict_history_entries{0};
  std::size_t voronoi_lane_conflict_resource_history_entries{0};
  std::size_t voronoi_lane_conflict_history_increments{0};
  std::size_t voronoi_lane_conflict_history_withheld{0};
  std::size_t voronoi_lane_conflict_scheduled_reroute_nets{0};
  std::size_t voronoi_lane_conflict_coupled_group_expansions{0};
  std::size_t unselected_voronoi_lane_transitions{0};
  std::size_t reoriented_voronoi_resources{0};
  std::size_t voronoi_lane_orientation_states{0};
  std::size_t solved_voronoi_lane_orientation_components{0};
  std::size_t withheld_voronoi_lane_orientation_components{0};
  std::size_t largest_voronoi_lane_orientation_component_resources{0};
  std::size_t solved_voronoi_lane_orientation_junctions{0};
  std::size_t withheld_voronoi_lane_orientation_junctions{0};
  std::size_t mitered_voronoi_lane_transitions{0};
  std::size_t withheld_voronoi_lane_transitions{0};
  std::size_t withheld_voronoi_transition_clearance_checks{0};
  std::size_t congested_resources{0};
  std::size_t congested_corridors{0};
  std::size_t congested_corridor_neck_downs{0};
  std::size_t neck_down_corridors{0};
  std::size_t peak_corridor_demand{0};
  std::size_t peak_corridor_neck_down_demand{0};
  std::size_t peak_resource_demand{0};
  std::size_t pathfinder_reordered_iterations{0};
  std::size_t pathfinder_reordered_nets{0};
  std::size_t pathfinder_peak_net_pressure{0};
  std::size_t pathfinder_peak_net_sampled_overflow{0};
  std::size_t pathfinder_peak_net_corridor_overflow{0};
  std::uint64_t pathfinder_peak_net_total_physical_overflow_nm{0};
  std::uint64_t pathfinder_peak_net_physical_overflow_nm{0};
  std::size_t pathfinder_peak_net_congested_physical_resources{0};
  std::vector<RoutingNetPressure> selected_net_pressure;
  std::vector<RoutingResidualConflictGroup> residual_conflict_groups;
  bool configured_physical_envelope_ordering{false};
  std::uint64_t peak_net_routing_envelope_nm{0};
  std::uint64_t peak_routing_group_envelope_nm{0};
  std::size_t reused_paths{0};
  std::vector<NetId> last_pathfinder_reroute_order;
  bool configured_search_diversification{false};
  std::size_t configured_global_route_beam_width{0};
  std::size_t configured_max_large_neighborhood_attempts{0};
  std::size_t configured_large_neighborhood_stagnation_iterations{0};
  std::size_t configured_destroy_route_percent{0};
  std::size_t configured_max_routing_order_variants{0};
  std::uint64_t configured_routing_order_diversification_seed{0};
  std::size_t global_route_beam_peak{0};
  std::size_t global_route_beam_candidates_retained{0};
  std::size_t routing_order_variants_attempted{0};
  std::size_t temporary_illegal_checkpoints{0};
  std::size_t large_neighborhood_attempts{0};
  std::size_t large_neighborhood_destroyed_nets{0};
  std::size_t large_neighborhood_coupled_expansions{0};
  std::vector<std::string> large_neighborhood_operators;
  std::vector<NetId> reused_nets;
  std::vector<NetId> rerouted_nets;
  std::vector<NetId> invalidated_reuse_nets;
  GlobalRouterStopReason stop_reason{GlobalRouterStopReason::invalid_input};
  std::string diagnostic;

  [[nodiscard]] bool complete() const { return stop_reason == GlobalRouterStopReason::complete; }
};

// Authoritative deterministic checkpoint ordering used by global routing and available to the
// integrated placement loop. The dimensioned policy changes intermediate selection only; route
// completion still requires every legacy overflow category to reach zero.
[[nodiscard]] bool global_route_checkpoint_better(
    const GlobalRouteResult& lhs, const GlobalRouteResult& rhs,
    GlobalRouteCheckpointObjective objective = GlobalRouteCheckpointObjective::aggregate_overflow);

// Collect stable, saturating, unit-separated congestion participation for diagnostics and bounded
// experiments. Production reroute order deliberately retains legacy resource participation.
[[nodiscard]] std::vector<RoutingNetPressure>
collect_global_route_net_pressure(const GlobalRouteResult& result);

// Collect one pressure record for a coupled routing group. A congested resource shared by
// multiple selected members contributes once, preventing group cardinality from inflating pressure.
[[nodiscard]] RoutingNetPressure
collect_global_route_group_pressure(const GlobalRouteResult& result,
                                    const std::vector<NetId>& group_nets);

// Pure experimental comparator. Production routing does not call it because equal-budget board
// evidence rejected dimensioned serial scheduling. Equal pressure is deliberately unordered.
[[nodiscard]] bool routing_net_pressure_more_severe(
    const RoutingNetPressure& lhs, const RoutingNetPressure& rhs,
    GlobalRouteCheckpointObjective objective = GlobalRouteCheckpointObjective::aggregate_overflow);

// Deterministic planning-stage global routing. Paths reserve represented copper and
// board-edge clearance, but only detailed materialization plus DRC can accept them.
[[nodiscard]] GlobalRouteResult route_global(const Board& board,
                                             const GlobalRouterConfig& config = {},
                                             std::stop_token stop_token = {},
                                             RouterTelemetrySink* telemetry = nullptr);

// Reuses compatible checkpoint paths outside affected_nets. Any preserved path blocked by
// the current board is deterministically promoted into the affected cone and rerouted.
// preserve_unaffected_provisional_congestion is for an unchanged-board exact-feedback
// transaction: it retains crossing-safe/conflicted paths outside the explicit repair cone.
// Leave it false for board/placement changes. Detailed routing and DRC remain required.
[[nodiscard]] GlobalRouteResult
route_global_incremental(const Board& board, const GlobalRouteResult& checkpoint,
                         const std::vector<NetId>& affected_nets,
                         const GlobalRouterConfig& config = {}, std::stop_token stop_token = {},
                         RouterTelemetrySink* telemetry = nullptr,
                         bool preserve_unaffected_provisional_congestion = false);

} // namespace flux
