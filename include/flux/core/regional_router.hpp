#pragma once

#include <array>

#include "flux/core/geometry.hpp"
#include "flux/core/global_router.hpp"
#include "flux/core/quantized_copper_oracle.hpp"
#include "flux/core/routing_compute.hpp"
#include "flux/core/routing_avbd.hpp"
#include "flux/core/regional_hc_composition.hpp"
#include "flux/core/spline_route.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <stop_token>
#include <string>
#include <utility>
#include <vector>

namespace flux {

// Performance observations are intentionally excluded from semantic route-result equality.
// Repeated routing must compare deterministically even though wall-clock samples cannot.
struct RegionalPerformanceMicroseconds {
  constexpr RegionalPerformanceMicroseconds(std::uint64_t microseconds = 0U)
      : value(microseconds), accumulated_nanoseconds(microseconds * 1'000U) {}
  std::uint64_t value{0U};
  // Keep sub-microsecond samples until they are reported. `value` remains the public `_us`
  // quantity consumed by existing JSON/telemetry, while this accumulator prevents a sequence
  // of short calls from being rounded down independently.
  std::uint64_t accumulated_nanoseconds{0U};
  void add_nanoseconds(std::uint64_t amount) {
    accumulated_nanoseconds += amount;
    value = accumulated_nanoseconds / 1'000U;
  }
  RegionalPerformanceMicroseconds& operator+=(std::uint64_t amount) {
    add_nanoseconds(amount * 1'000U);
    return *this;
  }
  RegionalPerformanceMicroseconds& operator+=(const RegionalPerformanceMicroseconds& amount) {
    add_nanoseconds(amount.accumulated_nanoseconds);
    return *this;
  }
  [[nodiscard]] constexpr operator std::uint64_t() const { return value; }
  constexpr bool operator==(const RegionalPerformanceMicroseconds&) const { return true; }
};

// Stable only within one selected global checkpoint, never canonical board identity.
struct RegionalRouteFragmentId {
  NetId net;
  std::size_t connection_index{0};
  std::size_t run_index{0};
  std::size_t fragment_index{0};
  auto operator<=>(const RegionalRouteFragmentId&) const = default;
};

struct RegionalFrozenCopperConflict {
  RegionalRouteFragmentId candidate;
  RegionalRouteFragmentId blocker;
  LayerId layer;
  Nanometres clearance_deficit;
  std::string code;
  bool operator==(const RegionalFrozenCopperConflict&) const = default;
};

// A failed physical scaffold demand can seed a bounded regional solve even when no
// selected-global conflict edge exists. The blocker IDs are verified immutable board geometry;
// they bound the repair window but never become movable regional route fragments.
struct RegionalPhysicalRepairRequest {
  NetId net;
  std::size_t connection_index{0U};
  // Count before stale object IDs are filtered against the current immutable Board.
  std::size_t configured_blocker_objects{0U};
  std::vector<ObjectId> blocker_objects;

  bool operator==(const RegionalPhysicalRepairRequest&) const = default;
};

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
  // Terminal escape is a first-class bounded regional decision.  The alternatives are exact
  // static-checked before publication and the leaf access CSP selects them atomically with vias
  // and lanes; disabling this is retained only as a diagnostic baseline.
  bool enable_terminal_portal_templates{true};
  // Search layer cells and certified via zones together before the legacy via-pair portfolio.
  bool enable_layered_navmesh{true};
  // Endpoint alternatives must be routed through prepared free space rather than represented by
  // an unchecked straight chord. This is a complete-run budget shared by deduplicated
  // (fragment,start-portal,end-portal) searches. Zero retains only exact-checked rebinds within
  // the already selected corridor and disables fresh topology work.
  std::size_t max_terminal_navmesh_template_checks{2'048};
  std::size_t max_via_regions_per_fragment{4};
  std::size_t max_via_sites_per_region{4};
  std::size_t max_selected_via_site_candidate_checks{16'384};
  std::size_t max_selected_via_site_checks_per_region{256};
  // Per-choice terminal/via-access CSP bound. Exhaustion is unknown, never an infeasibility
  // proof that may prune other topology choices.
  std::size_t max_access_assignment_states{2'048};
  bool enable_bundle_forward_checking{true};
  // Bounded HC-CBS-inspired composition is a static finite-assignment seed portfolio. It never
  // certifies copper and can be ablated without changing the existing exhaustive master.
  bool enable_bounded_hc_composition{true};
  std::size_t max_hc_composition_nodes{256U};
  std::size_t max_hc_composition_frontier{256U};
  std::size_t max_hc_composition_children_per_group{4U};
  std::size_t max_hc_composition_results{16U};
  std::size_t max_hc_composition_compatibility_checks{4'096U};
  std::size_t max_hc_composition_cache_bytes{4U << 20U};
  // Copies of this config share a small retained portfolio cache across preparation rebuilds.
  std::shared_ptr<RegionalHcCompositionCache> hc_composition_cache{
      std::make_shared<RegionalHcCompositionCache>()};
  // Experimental propagation of the existing aggregate capacity and nominated-via identity
  // constraints into future domains. No new physical acceptance rule is introduced.
  bool enable_bundle_resource_forward_checking{false};
  // Experimental local finite-section demand. Disabled until shared same-net branch
  // occupancy is handled consistently and frontier benchmarks demonstrate an improvement.
  bool local_channel_occupancy{false};
  // Count exactly identical same-net template copper once, retaining every branch owner.
  bool shared_channel_copper{false};
  // Experimental cut demand, not a physical-clearance certificate. Requires local occupancy.
  bool centerline_channel_cuts{false};
  std::size_t max_access_blockers_per_window{16};
  std::size_t max_layer_sequences_per_fragment{3};
  Nanometres navmesh_cell_pitch{Nanometres::from_mm(0.25)};
  std::size_t max_navmesh_cells_per_template{100'000};
  std::size_t max_navmesh_expansions_per_template{100'000};
  std::size_t max_alternative_template_checks{384};
  // Evaluate a small deterministic ordinary-template prefix through the physical realization
  // pipeline before spending the broad portfolio budget. The prepared free-space/domain cache is
  // retained by RegionalRoutingPreparation, so a later full pass resumes source preparation.
  bool enable_early_physical_feedback{true};
  std::size_t max_early_alternative_template_checks{8U};
  // The first exact physical probe owns only the highest-pressure regional windows. Unselected
  // windows remain global provisional geometry in the assembled scene and cannot be accepted
  // from this prefix; this cap only guarantees early diagnostic feedback.
  std::size_t max_early_physical_windows{2U};
  std::size_t max_early_bundle_search_states{1'024U};
  std::size_t max_early_access_assignment_states{128U};
  std::size_t max_early_lane_order_states{1'024U};
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
  std::size_t max_joint_spline_candidates{8};
  // Stable prefix from which the ordinary diversity beam is selected. Fallback capacity must
  // not silently change normal active routing; states beyond this prefix are late-stage reserve.
  std::size_t max_joint_spline_active_pool_candidates{72};
  // Compact complete discrete states outside the active continuous beam are retained for bounded
  // conflict-directed fallback waves. They carry portals/corridors/lanes/layers/vias, not spline
  // samples or copper, and therefore preserve bounded diversity without eager geometry cost.
  std::size_t max_joint_spline_fallback_candidates{512};
  // Re-rank a residual conflict cone's remaining reserve after every failed wave. Ordinary
  // regional analysis retains one wave; deeper exploration happens only after the cone has been
  // spatially isolated, preventing hundreds of useful states from being discarded without
  // multiplying continuous work across every ordinary window.
  std::size_t max_joint_spline_fallback_waves{3};
  // Optional, advisory learning-to-rank experiment. The model may only reorder already retained
  // candidates; it cannot prune a state or bypass any physical/exact acceptance gate. An invalid
  // model preserves the deterministic heuristic order. Training observations are append-only
  // JSONL and are emitted only for candidates that actually receive continuous realization.
  std::string candidate_ranker_model_path;
  std::string candidate_ranker_training_data_path;
  // A merged residual cone may require every contiguous fragment of one same-net/layer run to
  // take the same alternate layer/via template. Keep a small protected coverage set so the
  // ordinary low-cost bundle pool cannot discard that atomic route-run state before spline/DRC.
  std::size_t max_residual_cone_template_coverage_states{8};
  // After an exact continuous/copper shortlist proves that a finite regional domain is
  // insufficient, build new topology/layer/via/terminal columns only for the attributed failing
  // runs. Each window is re-solved as an exact bounded discrete master with lazy no-good clauses
  // from the failed physical states. Zero disables this search-and-repair wave.
  std::size_t max_conflict_directed_topology_windows{16};
  std::size_t max_conflict_directed_fragments_per_window{8};
  // A failed exact regional state may request new same-layer topology from a hydraulic field
  // confined to its attributed conflict cone. Two signed lateral priors expose opposite obstacle
  // sides while the anisotropic metric preserves progress along the failed route run. These are
  // ordinary topology columns: every accepted proposal still passes the existing exact static,
  // PH/copper, connectivity, and DRC gates. Zero checks disables the feature.
  std::size_t max_confined_anisotropic_erosion_checks{32};
  std::size_t max_confined_anisotropic_candidates_per_fragment{2};
  std::uint32_t confined_anisotropy_per_mille{2'500U};
  std::uint32_t confined_erosion_blocker_pressure{16U};
  std::uint32_t confined_erosion_lane_prior{2'000U};
  HydraulicPathConfig confined_anisotropic_erosion_flow{.relaxation_iterations = 256U,
                                                         .droplet_batches = 3U,
                                                         .droplets_per_batch = 512U,
                                                         .maximum_steps_per_droplet = 256U};
  // Bounded semantic failure records produced while reducing the joint spline shortlist.
  std::size_t max_local_routability_diagnoses{4'096};
  // Reuse existing exact-prevalidated portfolios for only the highest-pressure windows. The
  // ADR 0145 expanded window is included in this total rather than added on top of it.
  std::size_t max_spline_portfolio_windows{512};
  // Zero disables same-layer detour construction for diagnostic/decomposition-only runs.
  std::size_t max_same_layer_detour_checks{1'024};
  std::size_t max_same_layer_detour_offsets{3};
  std::size_t max_bundle_search_states{100'000};
  std::size_t max_lane_order_states{100'000};
  // Direct regional callers may stop after the discrete portfolio. The planner disables
  // realization in straight mode; every output mode still requires final physical copper DRC.
  bool enable_spline_realization{true};
  // Realize selected lanes as integer H/V/45 copper without PH interpolation.
  bool enable_straight_realization{false};
  // After the selected straight transaction and its retained source-topology fallback both fail
  // whole-scene copper checks, replay this many existing discrete portfolio alternatives.
  // Singles and conflict-linked two-window transactions share this exact-check budget. Zero
  // preserves the selected-only baseline. This does not generate topology or relax packing.
  std::size_t max_straight_portfolio_checks{0U};
  // Connected multi-window transactions share the existing portfolio budget.
  std::size_t max_straight_joint_windows{4U};
  // Materialize each final PH window candidate immediately as KiCad-native arcs/segments and
  // use represented copper DRC as the mutual/static legality authority. Continuous PH envelopes
  // remain topology/fairness guidance and a failure diagnostic, not a second copper model.
  // Decide the final regional window using the native arcs/segments Flux would emit. Expensive
  // continuous PH offsets remain available to diagnose a rejected native scene.
  bool native_arc_materialization_authority{true};
  // One exact polyline guide may be considered for a native-illegal PH candidate. The guide is
  // diagnostic input to a fresh PH fit and is never accepted as final copper.
  std::size_t max_native_polyline_guide_retries{1U};
  // Experimental conservative local check: retain every global fragment not owned by the
  // current window. Other windows may eventually move that copper, so this is opt-in.
  bool frozen_copper_context{false};
  bool expand_frozen_copper_scope{false};
  // Experimental scheduling: enlarge ownership before expensive same-scope retries when
  // ordinary native certification has found a new movable outside blocker. A required region
  // exceeding the configured local size bounds returns its global-replan diagnosis immediately.
  bool prioritize_frozen_scope_expansion{false};
  // Exact fragment identities from this checkpoint, not nearest-net/layer approximations.
  std::vector<RegionalFrozenCopperConflict> frozen_copper_scope_witnesses;
  NativeSplineFitConfig native_fit;
  // Independent regional windows use private mutable geometry state and are reduced in stable
  // window order. Alternatives within one window may run in bounded batches, with stable
  // shortlist reduction. Keep this bounded because each worker owns substantial PH/DRC scratch.
  std::size_t max_spline_window_workers{4U};
  // Optional memory-sensitive cap on leases from the process-wide CPU worker budget. Zero uses
  // the host-derived shared budget. Inner exact candidates remain serial under a window lease.
  std::size_t max_spline_cpu_workers{0U};
  // A window worker copies mutable regional/DRC state. A non-zero scratch budget limits the
  // number of such copies using this conservative per-worker estimate. This is an estimate of
  // copied route/DRC scratch rather than measured RSS, so unusually dense boards should lower
  // the budget or raise the estimate.
  std::size_t max_spline_window_scratch_bytes{256U * 1024U * 1024U};
  std::size_t estimated_spline_window_scratch_bytes{64U * 1024U * 1024U};
  // Bounded speculative evaluation of alternatives for one immutable regional copper snapshot.
  // Results are reduced in shortlist order after each batch, so later batches are never started
  // after the existing exact-legal portfolio cap is met.
  std::size_t max_spline_candidate_workers{2U};
  // Exact move proposals inside one window are immutable and reduced in their original slot
  // order. Four inner workers preserve deterministic neighborhood selection while allowing the
  // single dense window on the critical path to use the host after outer windows finish.
  std::size_t max_spline_exact_candidate_workers{4U};
  // Keep a small legal portfolio per window so post-realization selection can compare exact PH
  // length and curvature rather than inheriting the first legal discrete state.
  std::size_t max_legal_spline_candidates_per_window{2U};
  // A post-realization pass may remove redundant PH interpolation anchors from a small number of
  // already DRC-clean routes. Every proposal is rebuilt into native copper and rechecked as the
  // complete regional scene before it can replace the incumbent.
  bool enable_paired_obstacle_supports{true};
  std::size_t max_post_realization_tighten_routes{8U};
  std::size_t max_post_realization_tighten_removals{8U};
  std::size_t max_post_realization_tighten_candidate_checks{128U};
  // Joint physical relaxation is part of the normal regional realization loop. Every result is
  // still re-fitted and accepted only by the existing topology, continuous-envelope,
  // quantization, connectivity and DRC gates. A caller may disable it explicitly for controlled
  // comparison, but production routing must not silently skip the physical repair/feedback pass.
  bool enable_xpbd_relaxation{true};
  bool enable_xpbd_feedback{true};
  RoutingXpbdConfig xpbd;
  // A small CPU contact-coloring refinement is attempted only after a non-converged XPBD
  // proposal. Exact PH/native/DRC gates remain the only publication authority.
  bool enable_avbd_relaxation{true};
  RoutingAvbdConfig avbd{12U, 8'192U, 2'048U};
  // Slow-loop physical history sampled as a soft terrain potential by the next regional XPBD
  // pass. This is derived, bounded cost evidence only; exact corridor/topology/DRC gates remain
  // authoritative and an empty vector restores a history-free relaxation baseline.
  std::vector<RoutabilityHistoryEntry> xpbd_history;
  // Current exact copper failures for these unchanged global paths. Unlike historical costs,
  // these witnesses define repair scope; callers must clear them after global replanning.
  std::vector<RoutabilityDiagnosis> physical_conflict_witnesses;
  // Explicit failed-demand repair scopes. Unlike physical_conflict_witnesses, these are
  // bounded search evidence, not exact-copper conflict proofs.
  std::vector<RegionalPhysicalRepairRequest> physical_scaffold_requests;
  // Resolve native/provisional cross-window obligations before publishing a regional transaction.
  std::size_t max_physical_scope_retries{2U};
  // Fixed/via failures stop before PH portfolios. Mutable in-window pair conflicts may instead
  // enter the joint constructive solver when the selected cross-sections physically fit.
  bool require_legal_linear_scaffold{true};
  std::size_t max_xpbd_obstacle_segments_per_window{32'768U};
  // Subdivide the exact PH preimage at adaptive curve samples before constructing XPBD
  // particles. This makes the GPU contact polyline cover the same curved geometry inspected by
  // the continuous regional gate while keeping the per-route solver state bounded.
  std::size_t max_xpbd_interpolation_nodes_per_route{64U};
  // The early physical prefix uses this smaller proxy before any later portfolio expansion.
  std::size_t max_early_xpbd_interpolation_nodes_per_route{8U};
  std::size_t max_spline_sweeps{8};
  std::size_t max_adaptive_pair_shove_checks_per_window{16};
  std::size_t max_quantization_feedback_iterations{2};
  // Each failed continuous-to-copper conversion may test this many bounded two-route elastic
  // displacements before returning its diagnosis to the regional/global loops.
  std::size_t max_quantization_feedback_candidates_per_iteration{32};
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

  // Inclusive original path-edge range, retained when forward-collinear edges merge.
  std::size_t first_source_segment{0U};
  std::size_t last_source_segment{0U};

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

  std::size_t exact_copper_sample_count{0};

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

// Outcome of the bounded straight-copper replay domain.  This deliberately says nothing about
// global routability: a retained domain can be exhausted while topology outside the region is
// still movable, and a shared exact-check cap leaves the answer unknown.
enum class RegionalStraightSearchOutcome {
  not_run,
  found_legal_candidate,
  retained_fixed_pool_exhausted,
  budget_or_scope_unknown,
  cancelled
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
  bool residual_conflict_cone{false};
  std::size_t residual_stub_fragments{0};

  [[nodiscard]] bool eligible() const {
    return failure_reason == RegionalWindowFailureReason::none;
  }
  bool operator==(const RegionalRoutingWindow&) const = default;
};

struct RegionalLanePlacement {
  RegionalRouteFragmentId fragment;
  std::int64_t center_offset_nm{0};

  auto operator<=>(const RegionalLanePlacement&) const = default;
};

struct RegionalCrossSection {
  std::size_t id{0};
  std::size_t channel_id{0};
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

// A global geometric corridor can overlap several regional channels.  Keep its directed route
// certificate attached to the exact fragment it describes instead of assigning one arbitrary
// obstacle-side sequence to every occupant of the merged channel.
struct RegionalFragmentTopologyBinding {
  RegionalRouteFragmentId fragment;
  NavMeshTopologySignature signature;
  std::vector<Point> centerline;

  bool operator==(const RegionalFragmentTopologyBinding&) const = default;
};

struct RegionalChannel {
  std::size_t id{0};
  LayerId layer;
  std::vector<Polygon> topology_cells;
  std::vector<NavMeshPortal> portals;
  // Directed, selected funnel gates.  Empty is explicit "cell-union-only evidence" for older
  // global corridor producers that did not retain a NavMesh route certificate.
  NavMeshTopologySignature topology_signature;
  std::vector<RegionalFragmentTopologyBinding> fragment_topology_bindings;
  std::vector<RegionalTopologyLink> endpoint_links;
  std::vector<RegionalCrossSection> cross_sections;
  std::vector<std::size_t> adjacent_channels;
  std::vector<RegionalRouteFragmentId> occupants;
  bool conservative_source_evidence{true};
  // Alternative generators append channels to the shared inventory. Their occupant identifies
  // the candidate's route, not membership in the retained global template's corridor.
  bool alternative_template_only{false};

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
  // The route class selected by this discrete template.  A quintic PH curve/PNS realization may
  // deform inside it but must ask the bounded regional template search for any corridor-side
  // change.
  NavMeshTopologySignature topology_signature;
  std::optional<std::size_t> terminal_entry_choice;
  std::optional<std::size_t> terminal_exit_choice;
  std::uint64_t lower_bound_length_nm{0};
  std::uint64_t topology_cost{0};
  bool selected_global_topology{true};
  bool exact_static_checked{false};

  bool operator==(const RegionalRouteTemplate&) const = default;
};

// Cross sections sharing the same longitudinal support form one demand scope.
// Template indices refer to the supplied portfolio vector, not template.id. A
// template can occupy several scopes; missing geometric evidence retains the
// conservative whole-channel scope. This is an allocation model, not a DRC proof.
struct RegionalChannelDemandScope {
  std::size_t channel_id{0U};
  std::vector<std::size_t> cross_section_indices;
  std::vector<std::size_t> template_indices;
  Nanometres maximum_width;
  bool localized{false};
};

// Exact whole-polyline copper identities, independent of selection order. Unknown
// geometry stays unique. Returned representatives are portfolio vector indices.
[[nodiscard]] std::vector<std::size_t> regional_template_copper_aliases(
    const std::vector<RegionalRouteFragment>& fragments,
    const std::vector<RegionalRouteTemplate>& templates, bool share_identical = true);

[[nodiscard]] std::vector<RegionalChannelDemandScope> regional_channel_demand_scopes(
    const std::vector<RegionalRouteFragment>& fragments,
    const std::vector<RegionalChannel>& channels,
    const std::vector<RegionalRouteTemplate>& templates, bool local_occupancy = true,
    bool centerline_cuts = false);

enum class RegionalBundleStopReason {
  complete,
  missing_corridor_topology,
  no_fitting_lane_order,
  candidate_preflight_infeasible,
  window_ineligible,
  state_budget_exhausted,
  spline_realization_failed
};

// The first failed gate while turning a discrete regional template into its PH route.  This is
// deliberately candidate-local evidence: none of these failures authorizes a topology no-good.
enum class RegionalRealizationRejectionStage {
  invalid_cross_section,
  missing_fragment,
  degenerate_fragment,
  missing_window,
  invalid_template,
  missing_channel_topology,
  topology_seed_failed,
  sampled_route_too_short,
  route_topology_illegal,
  route_shape_illegal,
  native_copper_illegal,
  quantized_copper_illegal
};

// Exact dynamic-copper witnesses retained only for a rejected regional candidate. Object IDs are
// DRC identities (including generated copper), so they explain a failure without becoming a
// mutable no-good clause.
struct RegionalNativeCopperViolation {
  std::string code;
  std::vector<std::string> object_ids;
  Nanometres required_clearance;
  long double actual_distance_nm{0.0L};

  bool operator==(const RegionalNativeCopperViolation&) const = default;
};

struct RegionalRealizationRejection {
  RegionalRealizationRejectionStage stage{RegionalRealizationRejectionStage::invalid_template};
  std::size_t window_id{0U};
  std::optional<std::size_t> template_id;
  std::optional<RegionalRouteFragmentId> fragment;
  std::optional<LayerId> layer;
  std::optional<Point> geometry;
  bool owned{false};
  std::vector<RegionalNativeCopperViolation> native_violations;

  bool operator==(const RegionalRealizationRejection&) const = default;
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

// Bounded topology search diagnostics. Choices contain local option indices in the original
// fragment-domain order. These candidates still require lane, continuous and native validation.
struct RegionalBundleChoicePortfolio {
  std::vector<std::vector<std::size_t>> choices;
  std::vector<RegionalAccessBlocker> access_blockers;
  std::vector<RegionalFrozenCopperConflict> frozen_copper_conflicts;
  std::size_t states{0};
  std::size_t access_pruned_states{0};
  std::size_t access_assignment_states{0};
  std::size_t access_assignment_exhaustions{0};
  std::size_t terminal_pair_pruned_states{0};
  std::size_t via_pair_pruned_states{0};
  std::size_t terminal_via_pruned_states{0};
  std::size_t frozen_copper_via_pruned_states{0};
  std::size_t lane_capacity_pruned_states{0};
  std::size_t run_layer_pruned_states{0};
  std::size_t coupled_pruned_states{0};
  std::size_t forward_domain_wipeouts{0};
  std::size_t pairwise_access_checks{0};
  std::size_t fixed_geometry_compatibility_checks{0};
  std::size_t fixed_geometry_compatibility_rejections{0};
  RegionalPerformanceMicroseconds access_selection_us;
  std::size_t access_selection_invocations{0U};
  // Cache scope is one generate_consistent_bundle_choices() call, whose board, regional
  // snapshot, window, templates and access-state budget are immutable.
  std::size_t access_selection_cache_hits{0U};
  std::size_t access_selection_cache_misses{0U};
  std::size_t access_selection_cache_avoided_calls{0U};
  std::size_t access_selection_cache_entries_peak{0U};
  std::size_t access_selection_cache_capacity_skipped_insertions{0U};
  std::size_t forward_capacity_rejections{0};
  std::size_t forward_via_identity_rejections{0};
  RegionalHcCompositionStats hc_composition;
  bool exhausted{false};
};

// A regional result is a value snapshot.  PH routes are retained through shared ownership to
// avoid copying their prepared corridor certificates, but the allocation itself is not part of
// that snapshot's identity: repeated deterministic analysis necessarily builds fresh routes.
// Keep pointer-like access for the realization and compare the retained immutable route by value.
struct RegionalPHRouteReference {
  std::shared_ptr<const QuinticPHRoute> value;

  RegionalPHRouteReference() = default;
  RegionalPHRouteReference(std::shared_ptr<const QuinticPHRoute> route) : value(std::move(route)) {}
  RegionalPHRouteReference& operator=(std::shared_ptr<const QuinticPHRoute> route) {
    value = std::move(route);
    return *this;
  }
  [[nodiscard]] explicit operator bool() const { return static_cast<bool>(value); }
  [[nodiscard]] const QuinticPHRoute* operator->() const { return value.get(); }
  [[nodiscard]] const QuinticPHRoute& operator*() const { return *value; }
  [[nodiscard]] const QuinticPHRoute* get() const { return value.get(); }
  void reset() { value.reset(); }
  bool operator==(const RegionalPHRouteReference& other) const {
    return value == other.value || (value && other.value && *value == *other.value);
  }
};

struct RegionalSplineRealization {
  RegionalRouteFragmentId fragment;
  std::optional<std::size_t> owner_window_id{};
  Point replacement_end;
  LayerId layer;
  std::vector<Point> centerline;
  // Flattened six-controls-per-segment quintic PH Bézier polygons retained for diagnostic
  // telemetry. Canonical copper still uses the separately quantized centerline transaction.
  std::vector<Point> control_points;
  // Immutable PH design route retained with the emitted copper.  Post-realization operations may
  // only clone this certificate: it carries the prepared clearance-expanded corridor needed to
  // prove that a tightened curve remains in the selected topology.
  RegionalPHRouteReference ph_route;
  NavMeshTopologySignature topology_certificate;
  // Terminals, selected via sites, and explicit regional junctions that coincide with PH
  // interpolation nodes. These are hard anchors for post-realization tightening.
  std::vector<Point> protected_anchors;
  Nanometres certified_minimum_bend_radius{Nanometres{0}};
  // The first physical realization of this PH curve. These pieces are exact KiCad primitives and
  // are checked together in the regional window before the candidate can be selected.
  std::vector<SplineFittedPiece> native_pieces;
  NativeSplineFitConfig native_fit_policy;
  Nanometres width;
  GlobalRouteSegmentWidthKind width_kind{GlobalRouteSegmentWidthKind::nominal};
  std::optional<Point> entry_via_site;
  std::optional<Point> exit_via_site;
  std::size_t optimization_sweeps{0};
  std::size_t accepted_control_moves{0};
  std::uint64_t topology_signature{0U};
  std::size_t topology_gate_count{0U};
  std::size_t merged_fragments{1U};
  bool topology_legal{false};
  bool topology_gate_sequence_legal{false};
  bool endpoint_topology_legal{false};
  bool interior_topology_legal{false};
  // Non-authoritative reason from the selected NavMesh homotopy certificate. This is retained
  // with the spline so telemetry can distinguish an obstacle/cell escape from a gate-order error.
  std::string topology_diagnostic;
  bool shape_legal{false};
  bool shape_sample_budget_exhausted{false};
  std::size_t shape_self_intersections{0U};
  std::size_t shape_cusps{0U};
  std::size_t shape_control_station_reversals{0U};
  std::size_t shape_station_reversals{0U};
  std::size_t shape_excess_turn_violations{0U};
  std::size_t shape_unintended_inflections{0U};
  long double shape_total_turn_radians{0.0L};
  long double shape_anchor_turn_radians{0.0L};
  long double interpolation_parameter_exponent{0.5L};
  long double interpolation_tangent_scale{1.0L};
  bool mutually_legal{false};
  bool quantized_mutually_legal{false};
  bool native_copper_legal{false};
  bool continuous_envelope_evaluated{false};
  bool continuous_envelope_legal{false};
  bool rational_offset_evaluated{false};
  bool rational_offset_intersection{false};

  // Segment-only transactions carry no PH/continuous-envelope certificate.
  bool straight_realization{false};

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
  // Original discrete-objective order and optional advisory learned score. These do not
  // participate in physical feasibility and remain attached only for experiment diagnostics.
  std::size_t candidate_heuristic_rank{0};
  double candidate_learned_score{0.0};
  bool candidate_learned_score_available{false};

  bool operator==(const RegionalBundleSelection&) const = default;
};

// Compact post-realization record for a bounded joint spline candidate. It preserves the
// selected discrete template IDs and failure profile, not mutable spline geometry, so a regional
// diagnosis can tell whether a promising layer/corridor combination was actually tested.
struct RegionalSplineCandidateSummary {
  std::size_t rank{0};
  std::vector<std::size_t> templates;
  std::vector<std::size_t> via_choices;
  std::vector<std::size_t> via_assignment_region_order;
  std::vector<RegionalCrossSection> packed_cross_sections;
  std::size_t short_realizations{0};
  std::size_t topology_rejections{0};
  std::size_t shape_rejections{0};
  std::size_t mutual_clearance_rejections{0};
  std::size_t quantized_clearance_rejections{0};
  std::optional<RegionalRealizationRejection> first_realization_rejection;
  // Only a monotonic structural certificate can create a conflict-directed no-good. A failed
  // continuous realization remains candidate-specific evidence: another deformation in the
  // same corridor/lane assignment may still be legal.
  bool no_good_certified{false};
  bool complete{false};

  bool operator==(const RegionalSplineCandidateSummary&) const = default;
};

struct RegionalBundleAssignment {
  std::size_t window_id{0};
  std::vector<RegionalRouteTemplate> templates;
  std::vector<std::size_t> selected_templates;
  std::vector<std::size_t> selected_via_choices;
  // Candidate-local relocations produced by physical relaxation. These copies deliberately do
  // not alter RegionalRouteResult::via_choices because alternatives may be evaluated in parallel.
  std::vector<RegionalViaChoice> relaxed_via_choices;
  std::vector<std::size_t> via_assignment_region_order;
  std::vector<RegionalCrossSection> packed_cross_sections;
  std::vector<RegionalSplineRealization> spline_realizations;
  // First pre-realization failure for this candidate. Retaining it prevents empty realization
  // vectors from being reported as an unexplained "spline_realization_failed".
  std::optional<RegionalRealizationRejection> first_realization_rejection;
  // Discrete access-search evidence survives native spline candidate changes.
  std::vector<RegionalFrozenCopperConflict> frozen_copper_conflicts;
  // Native copper assessment is specific to the currently evaluated spline candidate.
  std::vector<RegionalFrozenCopperConflict> native_frozen_copper_conflicts;
  std::vector<RegionalBundleSelection> spline_candidate_shortlist;
  std::vector<RegionalBundleSelection> spline_candidate_reserve;
  std::vector<RegionalAccessBlocker> partial_access_blockers;
  std::size_t lane_order_states{0};
  std::size_t bundle_search_states{0};
  std::size_t partial_access_pruned_states{0};
  std::size_t partial_access_assignment_states{0};
  std::size_t partial_access_assignment_exhaustions{0};
  std::size_t partial_terminal_pair_pruned_states{0};
  std::size_t partial_via_pair_pruned_states{0};
  std::size_t partial_terminal_via_pruned_states{0};
  std::size_t partial_frozen_copper_via_pruned_states{0};
  std::size_t partial_lane_capacity_pruned_states{0};
  std::size_t partial_run_layer_pruned_states{0};
  std::size_t partial_coupled_pruned_states{0};
  std::size_t partial_forward_domain_wipeouts{0};
  std::size_t partial_pairwise_access_checks{0};
  std::size_t access_assignment_states{0};
  std::size_t selected_template_conflicts{0};
  std::size_t selected_residual_conflicts{0};
  std::size_t coupled_rejected_states{0};
  std::size_t access_rejected_states{0};
  std::size_t invalid_access_rejected_states{0};
  std::size_t terminal_pair_rejected_states{0};
  std::size_t via_pair_rejected_states{0};
  std::size_t terminal_via_rejected_states{0};
  std::size_t frozen_copper_via_rejected_states{0};
  std::size_t packing_rejected_states{0};
  std::size_t selected_via_alternative_switches{0};
  std::size_t access_assignment_exhaustions{0};
  std::size_t spline_candidate_rank{0};
  std::size_t spline_candidate_heuristic_rank{0};
  double spline_candidate_learned_score{0.0};
  bool spline_candidate_learned_score_available{false};
  // Aggregate result of the bounded continuous shortlist before it is reduced to the best
  // legal, or least-illegal diagnostic, candidate. These values explain why a local cone did
  // not commit without retaining every trial spline geometry.
  std::size_t spline_candidates_evaluated{0};
  std::size_t spline_candidates_legal{0};
  std::size_t spline_candidate_min_topology_rejections{0};
  std::size_t spline_candidate_min_shape_rejections{0};
  std::size_t spline_candidate_min_mutual_clearance_rejections{0};
  std::size_t spline_candidate_min_quantized_clearance_rejections{0};
  std::vector<RegionalSplineCandidateSummary> spline_candidate_summaries;
  std::size_t residual_cone_template_coverage_states{0};
  std::size_t residual_cone_template_coverage_coupled_rejections{0};
  std::size_t residual_cone_template_coverage_access_rejections{0};
  std::size_t residual_cone_template_coverage_packing_rejections{0};
  std::size_t spline_fallback_waves{0};
  // An XPBD failure is not just another failed smoothness candidate. Once bounded deformation
  // cannot remove the mutual physical conflicts, the selected discrete state must become a lazy
  // no-good and the regional master must change corridor, layer, via, or terminal topology.
  bool xpbd_attempted{false};
  bool xpbd_topology_escape_required{false};
  bool joint_spline_candidate{false};
  // A private candidate worker has completed this assignment. The ordered parent reducer must
  // consume the resulting exact state without realizing it a second time.
  bool spline_candidate_realized{false};
  RegionalBundleStopReason stop_reason{RegionalBundleStopReason::window_ineligible};

  [[nodiscard]] bool complete() const { return stop_reason == RegionalBundleStopReason::complete; }
  bool operator==(const RegionalBundleAssignment&) const = default;
};

// Returns stable, deduplicated window IDs eligible for one regional spline tier. Tier zero is the
// active shortlist and includes every published assignment. A positive tier includes only failed
// assignments with an unconsumed reserve whose fallback-wave ordinal has not yet been reached.
// This pure selector is shared by the parallel executor and deterministic scheduler tests.
[[nodiscard]] std::vector<std::size_t> select_regional_spline_tier_windows(
    const std::vector<RegionalBundleAssignment>& assignments, std::size_t fallback_wave);

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

// One bounded residual-cone retry.  This is diagnostic state only: the ordinary regional
// checkpoint remains authoritative until a complete cone transaction strictly improves it, and
// canonical copper/connectivity/DRC still decide the eventual board commit.  Keeping this record
// makes a failed cone distinguishable from a cone that was legal but not feasibility-first better.
struct RegionalResidualFragmentFailure {
  RegionalRouteFragmentId fragment;
  std::size_t merged_fragments{1U};

  auto operator<=>(const RegionalResidualFragmentFailure&) const = default;
};

struct RegionalResidualConflictConeAttempt {
  std::size_t root_window_id{0};
  std::vector<std::size_t> source_window_ids;
  std::size_t fragment_count{0};
  std::size_t net_count{0};
  std::size_t assignment_count{0};
  std::size_t incomplete_assignments{0};
  std::size_t topology_rejections{0};
  std::size_t mutual_clearance_rejections{0};
  std::size_t quantized_clearance_rejections{0};
  std::size_t spline_candidates_evaluated{0};
  std::size_t spline_candidates_legal{0};
  std::size_t spline_merged_runs{0};
  std::size_t spline_merged_fragments{0};
  std::vector<RegionalResidualFragmentFailure> topology_rejected_fragments;
  std::vector<RegionalResidualFragmentFailure> mutual_clearance_rejected_fragments;
  std::vector<RegionalResidualFragmentFailure> quantized_clearance_rejected_fragments;
  std::size_t template_coverage_states{0};
  std::size_t template_coverage_retained{0};
  std::size_t template_coverage_coupled_rejections{0};
  std::size_t template_coverage_access_rejections{0};
  std::size_t template_coverage_packing_rejections{0};
  std::size_t stub_fragments{0};
  bool complete{false};
  bool selected{false};

  bool operator==(const RegionalResidualConflictConeAttempt&) const = default;
};

struct RegionalXpbdInvocation {
  std::size_t window_id{0U};
  std::size_t candidate_rank{0U};
  std::size_t route_count{0U};
  std::size_t particle_count{0U};
  std::size_t segment_count{0U};
  std::size_t obstacle_segment_count{0U};
  std::size_t terrain_cells{0U};
  std::size_t terrain_layers{0U};
  std::size_t pressure_layers{0U};
  std::size_t candidate_pairs{0U};
  std::size_t broadphase_cells{0U};
  std::size_t broadphase_pair_emissions{0U};
  std::size_t resident_input_buffer_reuses{0U};
  std::size_t unresolved_trace_pairs{0U};
  std::vector<NetId> nets;
  struct NetStress {
    NetId net;
    RoutingXpbdStressBreakdown stress;
    Point worst_location;
    long double worst_stress_nm{0.0L};

    bool operator==(const NetStress&) const = default;
  };
  std::vector<NetStress> net_stress;
  struct CorridorStress {
    std::size_t channel_id{0U};
    long double accumulated_stress_nm{0.0L};
    long double peak_stress_nm{0.0L};

    bool operator==(const CorridorStress&) const = default;
  };
  std::vector<CorridorStress> corridor_stress;
  RoutingXpbdStressBreakdown stress;
  RoutingXpbdConstraintCounts constraints;
  RoutingXpbdTimings timings;
  std::size_t iterations{0U};
  std::size_t initial_violations{0U};
  std::size_t final_violations{0U};
  Nanometres initial_peak_residual;
  Nanometres final_peak_residual;
  std::uint64_t initial_total_residual_nm{0U};
  std::uint64_t final_total_residual_nm{0U};
  bool used_gpu{false};
  bool used_gpu_broadphase{false};
  bool broadphase_fallback{false};
  bool used_gpu_pressure_field{false};
  bool cpu_fallback{false};
  bool converged{false};
  bool stalled{false};
  bool accepted{false};
  // The adaptive centreline/envelope probe may miss a transverse crossing that the rational PH
  // offset certificate finds. Such a crossing must trigger XPBD and participate in its acceptance
  // objective instead of being discovered only after the deformation stage has been skipped.
  bool rational_offset_triggered{false};
  bool topology_escape_required{false};
  bool pair_budget_exhausted{false};
  std::string diagnostic;

  bool operator==(const RegionalXpbdInvocation&) const = default;
};

// A physical-scope retry is an exploration, not an implicit replacement for the last assembled
// scene.  Keep the compact evidence needed to explain which scope became the checkpoint.
struct RegionalPhysicalScopeAttempt {
  std::size_t retry_index{0U};
  bool selected{false};
  bool native_scene_evaluated{false};
  bool native_scene_complete{false};
  bool straight_scene_evaluated{false};
  std::size_t incomplete_assignments{0U};
  std::size_t open_connections{0U};
  std::size_t physical_errors{0U};

  bool operator==(const RegionalPhysicalScopeAttempt&) const = default;
};

// Measurements for the bounded discrete candidate pipeline. These counters describe work before
// continuous/straight realization and deliberately remain separate from spline/XPBD telemetry.
struct RegionalCandidatePipelineStats {
  RegionalPerformanceMicroseconds channel_construction_us;
  RegionalPerformanceMicroseconds alternative_domain_us;
  RegionalPerformanceMicroseconds bundle_assignment_us;
  RegionalPerformanceMicroseconds bundle_search_us;
  // Exclusive to generate_consistent_bundle_choices(): wall time after subtracting access-CSP
  // calls nested entirely inside that invocation. It is not an assignment-level subtraction.
  RegionalPerformanceMicroseconds bundle_search_excluding_access_us;
  RegionalPerformanceMicroseconds lane_packing_us;
  RegionalPerformanceMicroseconds access_selection_us;
  std::size_t channel_construction_invocations{0U};
  std::size_t channel_prepared_cells{0U};
  std::size_t channel_spatial_index_queries{0U};
  std::size_t channel_exact_overlap_checks{0U};
  std::size_t alternative_domain_requests{0U};
  std::size_t bundle_assignment_invocations{0U};
  std::size_t bundle_search_invocations{0U};
  std::size_t lane_packing_invocations{0U};
  std::size_t access_selection_invocations{0U};
  std::size_t alternative_source_cache_hits{0U};
  std::size_t alternative_source_cache_misses{0U};
  std::size_t alternative_geometry_cache_hits{0U};
  std::size_t alternative_geometry_cache_misses{0U};
  std::size_t alternative_navigation_cache_hits{0U};
  std::size_t alternative_navigation_cache_misses{0U};
  std::size_t alternative_route_cache_hits{0U};
  std::size_t alternative_route_cache_misses{0U};
  std::size_t alternative_cache_evictions{0U};
  // Shallow container/payload estimate for the bounded alternative-domain cache, not process RSS
  // or a total heap-retention measurement.
  std::size_t alternative_cache_entries_peak{0U};
  std::size_t alternative_cache_shallow_bytes_estimate_peak{0U};
  std::size_t access_selection_cache_hits{0U};
  std::size_t access_selection_cache_misses{0U};
  std::size_t access_selection_cache_avoided_calls{0U};
  std::size_t access_selection_cache_entries_peak{0U};
  std::size_t access_selection_cache_capacity_skipped_insertions{0U};
  std::size_t fixed_geometry_compatibility_checks{0U};
  std::size_t fixed_geometry_compatibility_rejections{0U};
  std::size_t candidate_schedule_observations{0U};
  std::size_t candidate_schedule_reorders{0U};
  std::size_t candidate_preflight_checked{0U};
  std::size_t candidate_preflight_definitive_rejections{0U};
  std::size_t candidate_preflight_fixed_endpoint_rejections{0U};
  std::size_t candidate_preflight_packing_rejections{0U};
  std::size_t layered_navmesh_graph_nodes{0U};
  std::size_t layered_navmesh_planar_edges{0U};
  std::size_t layered_navmesh_via_zones{0U};
  std::size_t layered_navmesh_certified_transitions{0U};
  std::size_t layered_navmesh_rejected_spans{0U};
  std::size_t layered_navmesh_preparation_bound_hits{0U};
  std::size_t layered_navmesh_search_expansions{0U};
  std::size_t layered_navmesh_search_bound_hits{0U};
  std::size_t layered_navmesh_selected_proposals{0U};

  bool operator==(const RegionalCandidatePipelineStats&) const = default;
};

// Immutable exact scene shared by regional certification and detailed handoff. Source and
// route snapshots are structural identity checks; imported revision strings alone are not
// sufficient when placement, rules or protected copper are edited in memory.
struct RegionalCopperSceneData {
  Board source;
  Board scene;
  std::vector<GlobalRoutePath> paths;
  std::vector<RegionalSplineRealization> manifest;
  bool refillable_zone_fills{false};
  bool operator==(const RegionalCopperSceneData&) const = default;
};

struct RegionalCopperSceneArtifact {
  std::shared_ptr<const RegionalCopperSceneData> data;
  bool operator==(const RegionalCopperSceneArtifact& other) const {
    return data == other.data || (data && other.data && *data == *other.data);
  }
};

struct RegionalRouteResult {
  RegionalRouterConfig configured;
  RegionalCandidatePipelineStats candidate_pipeline_stats;
  RegionalHcCompositionStats hc_composition_stats;
  std::vector<RegionalRouteFragment> fragments;
  std::vector<RegionalConflictEdge> conflict_edges;
  std::vector<RegionalConflictComponent> components;
  std::vector<RegionalConflictBlock> biconnected_blocks;
  std::vector<RegionalRouteFragmentId> articulation_fragments;
  std::vector<RegionalRoutingWindow> windows;
  std::size_t source_physical_repair_requests{0U};
  std::size_t admitted_physical_repair_requests{0U};
  std::size_t configured_physical_repair_blockers{0U};
  std::size_t resolved_physical_repair_blockers{0U};
  std::vector<RegionalChannel> channels;
  std::vector<RegionalTerminalChoice> terminal_choices;
  std::size_t selected_terminal_portal_choices{0};
  std::size_t terminal_navmesh_template_checks{0};
  std::size_t accepted_terminal_corridor_rebinds{0};
  std::size_t accepted_terminal_navmesh_templates{0};
  std::size_t rejected_terminal_navmesh_templates{0};
  bool terminal_navmesh_template_budget_exhausted{false};
  std::vector<RegionalViaChoice> via_choices;
  std::vector<RegionalViaRegionEvidence> via_region_evidence;
  std::vector<RegionalBundleAssignment> bundle_assignments;
  std::vector<RoutabilityDiagnosis> local_routability_diagnoses;
  std::vector<GlobalRoutePath> spline_candidate_paths;
  // Ownership-resolved publication list, assembled with spline_candidate_paths. Boundary
  // context from neighboring windows must never become a second copper writer.
  std::vector<RegionalSplineRealization> native_realization_manifest;
  bool native_realization_manifest_available{false};
  RegionalCopperSceneArtifact validated_copper_scene;
  std::size_t physical_scope_retries{0U};
  std::size_t physical_scope_retries_explored{0U};
  std::size_t selected_physical_scope_retry{0U};
  std::vector<RegionalPhysicalScopeAttempt> physical_scope_attempts;
  std::size_t linear_scaffold_checks{0U};
  std::size_t linear_scaffold_rejections{0U};
  std::size_t straight_scene_trials{0U};
  bool straight_source_fallback_selected{false};
  std::size_t straight_scene_portfolio_checks{0U};
  // Geometry alternatives share the portfolio budget but are measured separately.
  std::size_t straight_scene_multi_window_checks{0U};
  std::size_t straight_scene_multi_window_improvements{0U};
  std::size_t straight_scene_geometry_checks{0U};
  std::size_t straight_scene_geometry_improvements{0U};
  std::size_t straight_scene_joint_portfolio_checks{0U};
  std::size_t straight_scene_portfolio_improvements{0U};
  // Family-level evidence for deterministic bounded scheduling.  Elapsed time is diagnostic
  // only; ordering uses the observed lexicographic error reduction.
  std::size_t straight_scene_geometry_attempts{0U};
  std::size_t straight_scene_geometry_error_reduction{0U};
  RegionalPerformanceMicroseconds straight_scene_geometry_elapsed_us;
  std::size_t straight_scene_portfolio_attempts{0U};
  std::size_t straight_scene_portfolio_error_reduction{0U};
  RegionalPerformanceMicroseconds straight_scene_portfolio_elapsed_us;
  // Native conflicts that also involved a provisional continuation.  They signal an unknown
  // mutable context, rather than proving that the blocker itself is outside the scope.
  std::size_t straight_scene_provisional_context_conflicts{0U};
  RegionalStraightSearchOutcome straight_scene_search_outcome{
      RegionalStraightSearchOutcome::not_run};
  std::size_t straight_scene_preview_checks{0U};
  std::size_t straight_scene_reserve_candidates{0U};
  std::size_t straight_scene_pool_states{0U};
  bool straight_scene_pool_complete{false};
  std::array<std::size_t, 4U> repair_family_attempts{};
  std::array<std::size_t, 4U> repair_family_error_reduction{};
  std::array<std::size_t, 4U> repair_family_work_units{};
  std::array<RegionalPerformanceMicroseconds, 4U> repair_family_elapsed_us{};
  std::size_t straight_collapsed_via_adapters{0U};
  std::size_t straight_scene_drc_violations{0U};
  std::size_t straight_scene_open_connections{0U};
  std::map<std::string, std::size_t> straight_scene_drc_categories;
  std::size_t native_scene_conflicts{0U};
  std::size_t native_scene_fixed_conflicts{0U};
  // Whole assembled scene truth, including violations unrelated to a native replacement. The
  // local counters above remain repair attribution only and must never certify progress alone.
  std::size_t native_scene_total_conflicts{0U};
  std::size_t native_scene_open_connections{0U};
  // Bounded post-realization PH tightening evidence. Exact arc length is recorded in nanometres
  // for the accepted replacement route; zero means no replacement was committed.
  std::size_t spline_tightening_routes_examined{0U};
  std::size_t spline_tightening_routes_accepted{0U};
  std::uint64_t spline_tightening_exact_length_before_nm{0U};
  std::uint64_t spline_tightening_exact_length_after_nm{0U};
  // Set only after the assembled native/static/via scene has reached copper-rule assessment.
  // Completeness records whether the manifest covered every published route independently.
  bool native_scene_evaluated{false};
  bool native_scene_complete{false};
  // Straight scenes are evaluated only when the oracle both ran and materialized every path.
  bool straight_scene_evaluated{false};
  std::size_t native_arc_fit_failures{0U};
  std::size_t native_polyline_guide_checks{0U};
  std::size_t native_polyline_guide_exact_legal{0U};
  std::size_t native_polyline_guide_ph_accepted{0U};
  std::vector<RegionalMultilayerRoutePlan> multilayer_route_plans;
  std::size_t complete_via_dependent_routes{0};
  std::size_t planned_layer_runs{0};
  std::size_t planned_via_transitions{0};
  std::size_t stitched_multilayer_runs{0};
  bool spline_candidate_available{false};
  bool spline_candidate_conflicting_articulation{false};
  std::string spline_candidate_diagnostic;
  std::size_t source_clearance_shadow_conflicts{0};
  std::size_t source_packing_obligations{0};
  std::size_t unmapped_packing_obligations{0};
  bool packing_obligation_budget_exhausted{false};
  std::size_t source_exact_copper_witnesses{0};
  std::size_t mapped_exact_copper_pairs{0};
  std::size_t unmapped_exact_copper_pairs{0};
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
  std::size_t lane_precedence_rejected_states{0};
  std::size_t lane_solver_queries{0};
  std::size_t lane_solver_cache_hits{0};
  std::size_t lane_solver_states{0};
  std::size_t lane_solver_pruned_states{0};
  std::size_t lane_solver_analytical_rejections{0};
  std::size_t lane_solver_budget_exhaustions{0};
  std::size_t partial_lane_capacity_pruned_states{0};
  std::size_t frozen_copper_via_pruned_states{0};
  std::size_t frozen_copper_via_rejected_states{0};
  std::size_t refittable_terminal_width_fragments{0};
  std::size_t fixed_resource_width_fragments{0};
  std::size_t via_site_candidate_checks{0};
  std::size_t via_site_diversity_rejections{0};
  std::size_t via_site_gpu_filter_batches{0};
  std::size_t via_site_gpu_candidates{0};
  std::size_t via_site_gpu_shortlisted{0};
  std::size_t via_site_gpu_rejected{0};
  std::size_t via_site_gpu_cpu_fallbacks{0};
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
  std::size_t ordinary_spline_candidates_evaluated{0};
  std::size_t ordinary_spline_candidates_legal{0};
  bool candidate_ranker_requested{false};
  bool candidate_ranker_loaded{false};
  std::size_t candidate_ranker_trees{0};
  std::size_t candidate_ranker_groups_scored{0};
  std::size_t candidate_ranker_candidates_scored{0};
  std::size_t candidate_ranker_candidates_reordered{0};
  std::size_t candidate_ranker_training_rows_written{0};
  std::size_t candidate_ranker_training_rows_failed{0};
  std::string candidate_ranker_diagnostic;
  std::size_t joint_spline_candidates_generated{0};
  std::size_t joint_spline_active_candidates_retained{0};
  std::size_t joint_spline_fallback_candidates_retained{0};
  std::size_t joint_spline_fallback_candidates_dropped{0};
  std::size_t joint_spline_fallback_candidates_promoted{0};
  std::size_t joint_spline_fallback_promotion_waves{0};
  std::size_t joint_spline_fallback_candidates_remaining{0};
  std::size_t joint_spline_fallback_windows_recovered{0};
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
  std::size_t spline_continuous_hulls_checked{0};
  std::size_t spline_continuous_envelope_rejections{0};
  std::size_t spline_continuous_budget_exhaustions{0};
  std::size_t spline_lane_targets{0};
  std::size_t spline_multi_target_fragments{0};
  std::size_t spline_merged_runs{0};
  std::size_t spline_merged_fragments{0};
  std::size_t spline_interpolation_portfolio_candidates{0};
  std::size_t spline_interpolation_portfolio_offset_rejections{0};
  std::size_t spline_interpolation_portfolio_accepted{0};
  std::size_t spline_interpolation_portfolio_windows_improved{0};
  // The initial topology-seed PH portfolio uses the same candidate-major GPU ranker as every
  // later interpolation sweep. These counters keep that production path independently visible.
  std::size_t spline_topology_seed_gpu_batches{0};
  std::size_t spline_topology_seed_gpu_fallbacks{0};
  std::size_t spline_topology_seed_gpu_candidates{0};
  std::size_t spline_topology_seed_exact_candidates{0};
  std::size_t spline_topology_seed_construction_rejections{0};
  std::size_t spline_topology_seed_corridor_rejections{0};
  std::size_t spline_topology_seed_cell_rejections{0};
  std::size_t spline_topology_seed_gate_rejections{0};
  std::size_t spline_topology_seed_shape_rejections{0};
  std::size_t spline_interpolation_gpu_batches{0};
  std::size_t spline_interpolation_gpu_fallbacks{0};
  std::size_t spline_interpolation_gpu_candidates{0};
  std::size_t spline_interpolation_exact_candidates{0};
  std::size_t spline_geometry_cache_hits{0};
  std::size_t spline_geometry_cache_misses{0};
  std::size_t spline_window_tasks{0};
  std::size_t spline_window_workers{0};
  std::size_t spline_candidate_parallel_batches{0};
  std::size_t spline_candidate_parallel_tasks{0};
  std::size_t spline_candidate_parallel_workers{0};
  std::size_t spline_candidate_parallel_discarded{0};
  std::size_t spline_incremental_segments_reused{0};
  std::size_t spline_incremental_segments_rebuilt{0};
  std::size_t spline_route_certificate_cache_hits{0};
  std::size_t spline_route_certificate_cache_misses{0};
  std::size_t spline_centerline_certificate_pairs{0};
  std::size_t spline_centerline_separation_proofs{0};
  std::size_t spline_centerline_collision_proofs{0};
  std::size_t spline_centerline_ambiguous_pairs{0};
  std::size_t spline_centerline_certificate_nodes{0};
  std::size_t spline_centerline_simd_batches{0};
  std::size_t spline_centerline_simd_lanes{0};
  std::size_t spline_centerline_multiprecision_comparisons{0};
  std::size_t spline_offset_broadphase_batches{0};
  std::size_t spline_offset_broadphase_gpu_batches{0};
  std::size_t spline_offset_broadphase_rejections{0};
  std::size_t spline_offset_exact_pairs{0};
  std::size_t spline_offset_subdivision_nodes{0};
  std::size_t spline_offset_isolated_leaf_pairs{0};
  std::size_t spline_offset_node_budget_exhaustions{0};
  RegionalPerformanceMicroseconds spline_interpolation_rank_us;
  RegionalPerformanceMicroseconds spline_interpolation_exact_us;
  RegionalPerformanceMicroseconds spline_adaptive_assessment_us;
  RegionalPerformanceMicroseconds spline_offset_broadphase_us;
  RegionalPerformanceMicroseconds spline_offset_exact_us;
  RegionalPerformanceMicroseconds spline_realization_us;
  std::size_t spline_joint_shove_attempts{0};
  std::size_t spline_joint_shove_accepted{0};
  std::size_t spline_adaptive_pair_shove_attempts{0};
  std::size_t spline_adaptive_pair_shove_accepted{0};
  std::size_t spline_adaptive_pair_windows_improved{0};
  std::size_t spline_adaptive_pair_windows_legalized{0};
  std::size_t quantized_segment_pair_checks{0};
  std::size_t quantized_clearance_conflicts{0};
  std::size_t native_arc_windows_checked{0};
  std::size_t native_arc_pieces_checked{0};
  std::size_t native_arc_copper_violations{0};
  std::size_t native_arc_legal_windows{0};
  std::size_t quantization_feedback_attempts{0};
  std::size_t quantization_feedback_accepted{0};
  std::size_t quantization_feedback_windows_legalized{0};
  std::size_t quantization_check_budget_exhausted_windows{0};
  Nanometres peak_quantized_clearance_deficit;
  std::size_t residual_conflict_cone_retries{0};
  std::size_t residual_conflict_cones_built{0};
  std::size_t residual_conflict_cone_windows_merged{0};
  std::size_t residual_conflict_cone_stub_fragments{0};
  std::size_t residual_conflict_cone_reused_assignments{0};
  std::size_t residual_conflict_cone_recomputed_assignments{0};
  std::size_t residual_conflict_cone_retry_withheld_assignments{0};
  bool residual_conflict_cone_selected{false};
  std::size_t residual_cone_template_coverage_states{0};
  std::size_t residual_cone_template_coverage_retained{0};
  std::size_t conflict_directed_topology_cores{0};
  std::size_t conflict_directed_topology_fragments{0};
  std::size_t conflict_directed_master_reruns{0};
  std::size_t conflict_directed_columns_generated{0};
  std::size_t conflict_directed_no_good_clauses{0};
  std::size_t conflict_directed_no_good_rejections{0};
  // Candidate failures withheld from no-good learning because the bounded probe did not prove
  // their physical infeasibility.  This makes unknown-versus-infeasible visible in telemetry.
  std::size_t conflict_directed_no_good_unknown_withheld{0};
  std::size_t conflict_directed_windows_recovered{0};
  std::size_t conflict_directed_windows_exhausted{0};
  std::size_t confined_anisotropic_erosion_checks{0};
  std::size_t confined_anisotropic_erosion_accepted{0};
  std::size_t confined_anisotropic_erosion_rejected{0};
  std::size_t confined_anisotropic_erosion_gpu_routes{0};
  std::size_t confined_anisotropic_erosion_cpu_routes{0};
  std::size_t confined_anisotropic_erosion_exact_chords{0};
  std::vector<RegionalResidualConflictConeAttempt> residual_conflict_cone_attempts;
  std::vector<RegionalXpbdInvocation> xpbd_invocations;
  std::size_t xpbd_windows_attempted{0U};
  // Multi-route candidates which were already mutually legal under the adaptive continuous
  // envelope probe do not need a physics solve. Keeping this observable prevents dense regional
  // portfolios from silently spending most of their budget on no-op GPU dispatches.
  std::size_t xpbd_unconflicted_candidates_skipped{0U};
  std::size_t xpbd_windows_improved{0U};
  std::size_t xpbd_subset_assessments{0U};
  std::size_t xpbd_subset_repairs_retained{0U};
  std::size_t xpbd_subset_groups_retained{0U};

  std::size_t avbd_windows_attempted{0U};
  std::size_t avbd_proposals_improved{0U};
  std::size_t avbd_windows_improved{0U};
  std::size_t avbd_contacts{0U};
  std::size_t avbd_duplicate_reductions{0U};
  std::size_t avbd_colors{0U};
  std::size_t avbd_local_block_solves{0U};
  std::size_t avbd_duals{0U};
  std::size_t avbd_penalty_updates{0U};
  std::size_t avbd_maximum_degree{0U};
  std::size_t avbd_moved_rigid_owners{0U};
  std::size_t avbd_capacity_rejections{0U};
  std::uint64_t avbd_solve_us{0U};

  std::size_t xpbd_windows_legalized{0U};
  std::size_t xpbd_gpu_invocations{0U};
  std::size_t xpbd_cpu_fallbacks{0U};
  std::size_t xpbd_topology_rejections{0U};
  std::size_t xpbd_pressure_cells{0U};
  std::size_t xpbd_terrain_cells_sampled{0U};
  std::size_t xpbd_obstacle_segments{0U};
  std::size_t xpbd_obstacle_segment_budget_exhaustions{0U};
  std::size_t xpbd_constraints{0U};
  std::size_t xpbd_broadphase_cells{0U};
  std::size_t xpbd_broadphase_pair_emissions{0U};
  std::size_t xpbd_resident_input_buffer_reuses{0U};
  std::size_t xpbd_gpu_broadphase_invocations{0U};
  std::size_t xpbd_broadphase_fallbacks{0U};
  std::size_t xpbd_gpu_pressure_field_invocations{0U};
  std::size_t xpbd_unresolved_trace_pairs{0U};
  RoutingXpbdStressBreakdown xpbd_stress;
  std::size_t xpbd_initial_violations{0U};
  std::size_t xpbd_final_violations{0U};
  Nanometres xpbd_peak_initial_residual;
  Nanometres xpbd_peak_final_residual;
  std::uint64_t xpbd_broadphase_us{0U};
  std::uint64_t xpbd_constraint_generation_us{0U};
  std::uint64_t xpbd_solve_us{0U};
  std::uint64_t xpbd_stress_reduction_us{0U};
  std::uint64_t xpbd_readback_us{0U};
  long double xpbd_peak_pressure{0.0L};
  std::uint64_t total_observed_overlap_length_nm{0};
  Nanometres peak_clearance_deficit;
  RegionalRouterStopReason stop_reason{RegionalRouterStopReason::invalid_input};
  std::string diagnostic;

  [[nodiscard]] bool packing_proven() const {
    return (native_scene_evaluated && native_scene_complete &&
            native_scene_total_conflicts == 0U && native_scene_open_connections == 0U) ||
           (straight_scene_evaluated && straight_scene_open_connections == 0U &&
            straight_scene_drc_violations == 0U);
  }
  [[nodiscard]] std::size_t unresolved_packing_obligations() const {
    return packing_proven() ? 0U : source_packing_obligations;
  }
  [[nodiscard]] bool complete() const {
    if ((source_packing_obligations != 0U || packing_obligation_budget_exhausted) &&
        !packing_proven()) return false;
    if (spline_candidate_available) {
      const bool native_legal = native_scene_evaluated && native_scene_complete &&
                                native_scene_total_conflicts == 0U &&
                                native_scene_open_connections == 0U;
      const bool straight_legal = straight_scene_evaluated &&
                                  straight_scene_open_connections == 0U &&
                                  straight_scene_drc_violations == 0U;
      return unmapped_exact_copper_pairs == 0U && (native_legal || straight_legal);
    }
    return unmapped_exact_copper_pairs == 0U &&
           stop_reason == RegionalRouterStopReason::no_conflicts &&
           unmapped_clearance_shadow_conflicts == 0U && unmapped_capacity_conflicts == 0U;
  }

  bool operator==(const RegionalRouteResult&) const = default;
};

// Observer-only, coherent pause points for a managed routing kernel. A null checkpoint marks
// progress inside a bounded search without copying its evolving frontier.
enum class RegionalDecisionPhase {
  pre_scene,
  fragments_prepared,
  conflicts_partitioned,
  window_prepared,
  // One regional window has committed its bounded discrete assignment.  A managed kernel may
  // pause here: the checkpoint already owns the assignment and no mutable window frontier is
  // required to resume the following window.
  experiment_window_completed,
  template_batch_completed,
  lane_search_progress,
  spline_candidate_started,
  spline_phase_started,
  spline_candidate_completed,
  // First bounded candidate is about to enter native/straight physical assessment. This is the
  // explicit preparation-to-repair budget boundary for resumable routing sessions.
  physical_feedback_started,
  scene_evaluated,
  before_scope_retry,
  after_scope_retry
};

// The engine needs a small, stable explanation of a completed local experiment to decide
// whether to resume geometry work or change topology.  IDs refer to the accompanying immutable
// RegionalRouteResult checkpoint.  The vectors are deliberately capped by the producer so this
// observer record cannot become a second copy of the regional search state.
enum class RegionalExperimentStage { window_assignment_committed };

struct RegionalExperimentDependencies {
  std::size_t owned_fragment_count{0U};
  std::size_t conflict_edge_count{0U};
  std::vector<RegionalRouteFragmentId> owned_fragments;
  std::vector<std::size_t> conflict_edges;
  std::vector<std::size_t> selected_templates;
  std::vector<RegionalAccessBlocker> access_blockers;
  bool access_budget_exhausted{false};

  bool operator==(const RegionalExperimentDependencies&) const = default;
};

struct RegionalExperimentMetadata {
  RegionalExperimentStage stage{RegionalExperimentStage::window_assignment_committed};
  std::optional<std::size_t> window_id;
  std::size_t batch_ordinal{0U};
  bool candidate_available{false};
  // True only when the completed, non-exhausted assignment has a forward access-domain
  // wipeout. A search budget exhaustion remains unknown rather than being labelled infeasible.
  bool access_infeasible{false};
  RegionalExperimentDependencies dependencies;

  bool operator==(const RegionalExperimentMetadata&) const = default;
};

struct RegionalDecisionPoint {
  RegionalDecisionPhase phase{RegionalDecisionPhase::pre_scene};
  std::shared_ptr<const RegionalRouteResult> checkpoint;
  bool scene_evaluated{false};
  std::string diagnostic;
  std::optional<RegionalExperimentMetadata> experiment;
};

using RegionalDecisionObserver = std::function<void(const RegionalDecisionPoint&)>;

class RegionalRoutingPreparation {
public:
  // One analysis holds this object's cache lease across observer pause points. Concurrent live
  // routing kernels must each own a distinct preparation; sharing one is intentionally
  // serialized to preserve cache and board-snapshot consistency.
  RegionalRoutingPreparation();
  ~RegionalRoutingPreparation();
  RegionalRoutingPreparation(const RegionalRoutingPreparation&) = delete;
  RegionalRoutingPreparation& operator=(const RegionalRoutingPreparation&) = delete;
  [[nodiscard]] std::uint64_t build_count() const;
  [[nodiscard]] std::uint64_t reuse_count() const;
  // Conservative retained cache estimate for admission telemetry.  It is a snapshot, not an
  // allocator accounting guarantee. Returns unavailable while a live analysis owns the mutable
  // cache lease, so telemetry cannot block a parked continuation.
  [[nodiscard]] std::optional<std::size_t> retained_bytes_estimate() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  friend std::shared_ptr<RegionalRoutingPreparation> prepare_regional_routing(
      const Board&, const GlobalRouteResult&, const RegionalRouterConfig&, std::stop_token);
  friend RegionalRouteResult analyze_regional_conflicts(
      const Board&, const GlobalRouteResult&, const RegionalRouterConfig&, std::stop_token,
      RouterTelemetrySink*, RegionalDecisionObserver, std::shared_ptr<RegionalRoutingPreparation>);
};

[[nodiscard]] std::shared_ptr<RegionalRoutingPreparation> prepare_regional_routing(
    const Board& board, const GlobalRouteResult& global, const RegionalRouterConfig& config,
    std::stop_token stop_token = {});

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
  // Number of bounded upstream replans after the initial global allocation. Two passes establish
  // a physical checkpoint; two further passes let residual regional lane/via/spline diagnoses
  // influence a fresh global allocation instead of being discovered only on the final pass.
  // Zero restores the single-pass boundary. Each retry consumes typed physical diagnoses as
  // cost-only history.
  std::size_t max_routability_feedback_retries{4U};
  RoutabilityProbeConfig routability_probe;
  // Split the caller's existing global PathFinder iteration budget across the initial pass and
  // feedback retries.  Zero retains the old whole-wave behaviour.  A short epoch lets physical
  // envelope, regional lane/via/escape, and hydraulic feedback influence the next PathFinder
  // allocation before all configured iterations have been consumed.
  bool interleave_routability_feedback{true};
  // Opt in to reserving the penultimate feedback epoch for an assembled straight-copper scene.
  // This spends the existing final global epoch consuming that exact regional evidence.
  bool enable_assembled_straight_feedback{false};
  // Retain distinct outer-loop global topologies for regional comparison.  This is separate from
  // the global router's pass-local beam: entries survive a physical-feedback epoch and therefore
  // preserve alternatives that become attractive only after downstream evidence arrives.
  std::size_t retained_global_portfolios{4U};
  // Only this many retained topologies receive bounded regional realization in an intermediate
  // epoch.  The final epoch always uses the full regional configuration on the best portfolio.
  std::size_t intermediate_regional_portfolios{2U};
  // Intermediate regional work is intentionally a probe rather than a second full detail pass.
  // It is deferred while sampled/corridor resources or selected lane transitions still need
  // ordinary PathFinder/rip-up negotiation.  Once those are clean these bounds preserve
  // lane/terminal/via failure information while leaving PH/copper realization and the configured
  // full budgets for the selected checkpoint.
  std::size_t intermediate_regional_bundle_states{20'000U};
  std::size_t intermediate_regional_exact_transactions{8U};
  std::size_t max_stagnant_feedback_epochs{4U};
  // KRT-like fine-grid feasibility is sampled between global allocation and regional PH
  // realization. It is an exact represented-copper diagnostic, never an acceptance shortcut.
  // Kept after the established aggregate fields so existing policy initializers remain stable.
  QuantizedCopperOracleConfig quantized_copper_oracle;

  // Unset preserves the legacy feedback-dependent policy. Explicit values let experiments
  // hold physical-conflict policy fixed while changing the number of feedback epochs.
  std::optional<bool> clearance_shadows_advisory;
  // Return exact provisional-copper failures upstream before regional search in intermediate
  // epochs. The final epoch still performs full realization; an illegal seed is not proof
  // that no legal regional deformation exists. Disabled until benchmark qualification.
  bool oracle_first_feedback{false};
  // Default full repair policy: certify the initial seed before further negotiation, activate
  // compatible global operators and physical push/shove on every epoch. Failed physical
  // realisations feed conflict-directed upstream rerouting before another local scope rebuild.
  // Never changes widths, clearance, ownership, or acceptance predicates.
  bool full_strength_first_pass{true};
  // Curved realization is the default; callers can explicitly request straight copper.
  bool curved_tracks{true};

  bool operator==(const RoutingPlannerConfig&) const = default;
};

// One global candidate inspected by a bounded outer feedback probe.  The candidate is a copy of
// authoritative global state, but these fields are only a diagnostic summary used to explain
// why the full routing-plan comparator retained or discarded it.
struct RoutingPlanFeedbackCandidate {
  bool history_informed_global{false};
  // A feedback pass may retain the previous global checkpoint and re-route only the exact nets
  // attributed by its physical diagnoses.  These are observability fields: they cannot change
  // the global or regional acceptance predicates.
  bool incremental_global{false};
  bool feedback_fell_back_to_full_global{false};
  std::size_t feedback_affected_nets{0U};
  std::size_t global_reused_nets{0U};
  std::size_t global_rerouted_nets{0U};
  std::size_t global_unrouted_nets{0U};
  std::size_t global_total_overflow{0U};
  std::size_t global_sampled_overflow{0U};
  std::size_t global_corridor_overflow{0U};
  std::size_t global_clearance_shadow_conflicts{0U};
  std::size_t global_unaccepted_lane_transitions{0U};
  bool coarse_topology_complete{false};
  bool regional_consulted{false};
  bool regional_spline_realization_enabled{false};
  bool regional_straight_realization_enabled{false};
  bool regional_straight_scene_evaluated{false};
  std::size_t regional_straight_scene_violations{0U};
  // A discrete-only regional probe may establish a plausible lane/escape/layer/via assignment,
  // but it is not authoritative physical completion.  The planner immediately promotes such a
  // state through the ordinary PH/copper-aware regional transaction before it can terminate an
  // interleaved feedback session.
  bool provisional_plan_complete{false};
  bool exact_promotion_attempted{false};
  bool exact_promotion_succeeded{false};
  std::size_t regional_incomplete_bundle_assignments{0U};
  std::size_t regional_spline_candidates_evaluated{0U};
  std::size_t regional_diagnoses{0U};
  bool plan_complete{false};

  bool operator==(const RoutingPlanFeedbackCandidate&) const = default;
};

// One bounded outer-loop observation.  This is diagnostic state only: it records whether
// non-authoritative physical history materially changes a subsequent global allocation, without
// becoming another routing representation or affecting the feasibility comparator.
struct RoutingPlanFeedbackProbe {
  std::size_t pass{0U};
  std::size_t history_entries_before{0U};
  std::uint64_t history_pressure_before{0U};
  std::size_t global_diagnoses{0U};
  bool quantized_copper_oracle_evaluated{false};
  bool quantized_copper_oracle_feasible{false};
  std::size_t quantized_copper_oracle_diagnoses{0U};
  std::size_t quantized_copper_oracle_violations{0U};
  std::size_t quantized_copper_oracle_opens{0U};
  std::uint64_t quantized_copper_oracle_us{0U};
  std::size_t history_entries_after_global{0U};
  std::uint64_t history_pressure_after_global{0U};
  std::vector<RoutingPlanFeedbackCandidate> candidates;
  std::size_t history_entries_after_regional{0U};
  std::uint64_t history_pressure_after_regional{0U};
  // A physically selected checkpoint normally seeds the next cone.  A topology-incomplete,
  // history-informed branch with an unaccepted packed-lane transition is the bounded exception:
  // it is retained only as a repair seed so the exact transition occupants can be reconsidered.
  bool next_feedback_uses_repair_branch{false};
  std::size_t next_feedback_affected_nets{0U};
  std::size_t global_epoch_iterations{0U};
  std::size_t cumulative_global_iterations{0U};
  std::size_t retained_global_portfolios{0U};
  bool intermediate_regional_probe{false};

  bool operator==(const RoutingPlanFeedbackProbe&) const = default;
};

// The first-class boundary between coarse PathFinder state and regional physical packing. Paths
// initially mirror coarse_global.paths; a later regional transaction replaces only governed
// fragments while preserving this result contract.
struct RoutingPlanResult {
  // Whole-board scaffold certification is independent of regional portfolio completion.
  // The exact oracle must also remain feasible before this can authorize materialization.
  bool legal_scaffold_complete{false};
  GlobalRouteResult coarse_global;
  RegionalRouteResult regional;
  QuantizedCopperOracleResult quantized_copper_oracle;
  std::vector<GlobalRoutePath> paths;
  RoutabilityKnowledge routability_knowledge;
  std::vector<RoutingPlanFeedbackProbe> feedback_probes;
  std::size_t global_feedback_passes{0U};
  std::size_t cheap_physical_probes{0U};
  std::size_t regional_feedback_passes{0U};
  std::size_t global_feedback_epochs{0U};
  std::size_t cumulative_global_iterations{0U};
  RegionalCandidatePipelineStats cumulative_candidate_pipeline_stats;
  std::size_t retained_global_portfolio_peak{0U};
  std::size_t retained_global_portfolios_evaluated{0U};
  std::size_t intermediate_regional_probes{0U};
  std::size_t provisional_exact_promotions_attempted{0U};
  std::size_t provisional_exact_promotions_succeeded{0U};
  std::size_t discarded_portfolio_diagnoses{0U};
  bool routability_feedback_budget_exhausted{false};
  // An empty deferred assessment is unknown, not evidence of zero regional failures.
  bool regional_evaluated{false};
  RoutingPlanStopReason stop_reason{RoutingPlanStopReason::invalid_input};
  std::string diagnostic;

  [[nodiscard]] bool complete() const { return stop_reason == RoutingPlanStopReason::complete; }
};

// Replays the bounded template allocator over an existing derived regional scene. Disabling
// forward checking reproduces the earlier identity/capacity-only search for differential tests.
[[nodiscard]] RegionalBundleChoicePortfolio enumerate_regional_bundle_choices(
    const Board& board, const RegionalRouteResult& regional, const RegionalRoutingWindow& window,
    const std::vector<RegionalRouteTemplate>& templates,
    const std::vector<std::vector<std::size_t>>& templates_by_fragment,
    std::size_t maximum_states, bool forward_checking = true);

// Converts exact selected-route clearance shadows into deterministic segment-level conflict
// evidence. Every source shadow must either map to a fragment edge or increment the explicit
// unmapped count; evidence is never silently dropped.
[[nodiscard]] RegionalRouteResult
analyze_regional_conflicts(const Board& board, const GlobalRouteResult& global,
                           const RegionalRouterConfig& config = {}, std::stop_token stop_token = {},
                           RouterTelemetrySink* telemetry = nullptr,
                           RegionalDecisionObserver observer = {},
                           std::shared_ptr<RegionalRoutingPreparation> preparation = nullptr);

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

// Assesses the owned native manifest together with every provisional planar run and physical
// via. Every represented conflict blocks handoff and supplies a repair obligation. Provisional
// tracks remain movable, but their movement must be resolved before this transaction is accepted.
void assess_regional_native_scene(const Board& board, RegionalRouteResult& regional);
// Bounded post-realization tightening hook. It preserves selected topology, terminal/via/junction
// anchors and accepts a replacement only after the ordinary whole-scene native assessment.
[[nodiscard]] bool tighten_regional_spline_candidate(
    const Board& board, const GlobalRouteResult& global, RegionalRouteResult& regional,
    const RegionalRouterConfig& config, std::stop_token stop_token = {},
    RouterTelemetrySink* telemetry = nullptr);

// Feasibility-first ordering for retaining regional scope checkpoints.  Unknown or unassembled
// scenes never borrow zero-valued physical counters from an evaluated scene.
[[nodiscard]] bool regional_checkpoint_better(const RegionalRouteResult& lhs,
                                               const RegionalRouteResult& rhs);
// Realizes the selected discrete assignment and accepts only an exact whole-scene copper proof.
void realize_regional_straight_candidate(const Board& board, const GlobalRouteResult& global,
                                         RegionalRouteResult& regional,
                                         std::stop_token stop_token = {});
[[nodiscard]] bool validate_regional_native_ownership(const RegionalRouteResult& regional);
// Diagnostic scene only, never accepted copper. Excludes owned fragments, not entire nets.
// Unknown ownership fails closed; source is unchanged. Proposed vias are not included here.
[[nodiscard]] std::optional<Board> build_regional_frozen_copper_context(
    const Board& source, const RegionalRouteResult& regional, std::size_t window_id,
    std::map<std::string, RegionalRouteFragmentId>* provenance = nullptr);
// Adds resolved candidate vias to a diagnostic clone, using assembly dimensions/reuse rules.
// Shared vias retain every attached fragment owner. Invalid spans fail closed.
[[nodiscard]] std::optional<Board> build_regional_candidate_via_context(
    const Board& source,
    const std::vector<std::pair<RegionalRouteFragmentId, RegionalViaChoice>>& selected,
    std::map<std::string, std::vector<RegionalRouteFragmentId>>& provenance);
// Copper-envelope guidance only; exact native/hole/DRC validation remains mandatory.
[[nodiscard]] SplineClearanceProbe regional_candidate_via_clearance(
    const NetId& net, const LayerId& layer, Nanometres width, Nanometres clearance,
    SplineVector position, const std::vector<RegionalViaChoice>& vias,
    Nanometres via_net_clearance);
// Only retained incomplete assignments contribute; stable pair deduplication bounds retries.
[[nodiscard]] bool append_regional_frozen_scope_witnesses(
    const RegionalRouteResult& regional, RegionalRouterConfig& config);
[[nodiscard]] bool regional_frozen_scope_retry_pending(
    const RegionalRouteResult& regional, const RegionalRouterConfig& config,
    std::size_t remaining_scope_retries);

// Runs the existing global topology/capacity router, then publishes regional physical-conflict
// evidence without changing the selected paths. Regional repair is introduced behind this API.
[[nodiscard]] RoutingPlanResult plan_routes(const Board& board,
                                            const RoutingPlannerConfig& config = {},
                                            std::stop_token stop_token = {},
                                            RouterTelemetrySink* telemetry = nullptr);

// Aggregates summed counters/timers and peak cache observations across engine actions.
void accumulate_regional_candidate_pipeline_stats(RegionalCandidatePipelineStats& total,
                                                 const RegionalCandidatePipelineStats& increment);

// Explicit reproducibility control; ordinary callers use the state-machine coordinator.
[[nodiscard]] RoutingPlanResult plan_routes_legacy(const Board& board,
                                            const RoutingPlannerConfig& config = {},
                                            std::stop_token stop_token = {},
                                            RouterTelemetrySink* telemetry = nullptr);

} // namespace flux
