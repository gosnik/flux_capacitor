add_executable(flux_unit_tests unit/core_tests.cpp)
target_link_libraries(flux_unit_tests PRIVATE flux::core)
target_compile_features(flux_unit_tests PRIVATE cxx_std_20)
target_compile_definitions(
  flux_unit_tests PRIVATE FLUX_TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_test(NAME flux_unit_tests COMMAND flux_unit_tests)

add_executable(flux_telemetry_viewer_tests unit/telemetry_viewer_tests.cpp)
target_link_libraries(flux_telemetry_viewer_tests PRIVATE flux::core flux::telemetry)
target_compile_features(flux_telemetry_viewer_tests PRIVATE cxx_std_20)
add_test(NAME flux_telemetry_viewer_tests COMMAND flux_telemetry_viewer_tests)

add_executable(
  flux_telemetry_live_client_tests
  unit/telemetry_live_client_tests.cpp
  ${PROJECT_SOURCE_DIR}/apps/flux-viewer/live_client.cpp)
target_include_directories(
  flux_telemetry_live_client_tests PRIVATE ${PROJECT_SOURCE_DIR}/apps/flux-viewer)
target_compile_features(flux_telemetry_live_client_tests PRIVATE cxx_std_20)
target_compile_definitions(flux_telemetry_live_client_tests PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(
  flux_telemetry_live_client_tests PRIVATE flux::core flux::telemetry Threads::Threads)
add_test(NAME flux_telemetry_live_client_tests COMMAND flux_telemetry_live_client_tests)
set_tests_properties(flux_telemetry_live_client_tests PROPERTIES RUN_SERIAL TRUE)

find_package(Python3 COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
  add_test(
    NAME flux_kicad_netclass_bridge_tests
    COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/python/test_kicad_netclass_bridge.py")
  set_tests_properties(flux_kicad_netclass_bridge_tests PROPERTIES
    ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1")
  add_test(
    NAME flux_kicad_candidate_bridge_tests
    COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/python/test_kicad_candidate_bridge.py")
  set_tests_properties(flux_kicad_candidate_bridge_tests PROPERTIES
    ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1")
endif()

function(flux_add_cli_test name pass_expression)
  add_test(NAME "${name}" COMMAND flux-cli ${ARGN})
  set_tests_properties("${name}" PROPERTIES PASS_REGULAR_EXPRESSION "${pass_expression}")
endfunction()

find_program(KICAD_CLI_EXECUTABLE kicad-cli)
find_program(INKSCAPE_EXECUTABLE inkscape PATHS /snap/bin)
set(FLUX_FIXTURES "${CMAKE_CURRENT_SOURCE_DIR}/fixtures")
if(Python3_Interpreter_FOUND AND KICAD_CLI_EXECUTABLE AND INKSCAPE_EXECUTABLE)
  add_test(
    NAME flux_cli_routing_corpus_unlocked
    COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/python/verify_routing_corpus.py"
            --flux-cli $<TARGET_FILE:flux-cli>
            --corpus ${FLUX_FIXTURES}/routing-corpus
            --artifacts ${CMAKE_CURRENT_BINARY_DIR}/routing-corpus-artifacts
            --kicad-cli ${KICAD_CLI_EXECUTABLE}
            --inkscape ${INKSCAPE_EXECUTABLE})
  set_tests_properties(
    flux_cli_routing_corpus_unlocked PROPERTIES
    LABELS "routing-corpus;real-board"
    TIMEOUT 7200)
endif()
flux_add_cli_test(flux_cli_demo "\"valid\": true" demo)
flux_add_cli_test(flux_cli_version "board-schema 18" version)
flux_add_cli_test(flux_cli_route_demo "\"complete\": true" route-demo)
flux_add_cli_test(
  flux_cli_strict_medial_route_demo
  "\"topologically_complete_medial_paths\": 1"
  route-demo --strict-medial)
flux_add_cli_test(
  flux_cli_direct_medial_route_demo
  "\"used_direct_voronoi_edges\": [1-9][0-9]*"
  route-demo --strict-medial --direct-medial)
flux_add_cli_test(
  flux_cli_free_space_demo "\"skeleton_edges\": [1-9][0-9]*" free-space-demo)
flux_add_cli_test(
  flux_cli_incremental_route_demo "\"reused_paths\": 1" incremental-route-demo)
flux_add_cli_test(flux_cli_detail_demo "\"copper_violations\": 0" detail-demo)
flux_add_cli_test(
  flux_cli_strict_medial_detail_demo
  "\"materialized_multilayer_topology_locked_paths\": 1"
  detail-demo --strict-medial)
flux_add_cli_test(
  flux_cli_incremental_detail_demo "\"reused_tracks\": 1" incremental-detail-demo)
flux_add_cli_test(
  flux_cli_optimize_demo "\"kind\": \"routing_guided_repair\"" optimize-demo)
flux_add_cli_test(
  flux_cli_optimize_demo_pareto "\"pareto_archive_size\": 1" optimize-demo)
flux_add_cli_test(
  flux_cli_optimize_demo_connector_last
  "\"placement_priority_tiers_unlocked\": 8"
  optimize-demo)
flux_add_cli_test(
  flux_cli_orientation_optimize_demo "\"kind\": \"rotation_repair\""
  orientation-optimize-demo)
flux_add_cli_test(
  flux_cli_group_optimize_demo "\"placement_group\": \"motion-stage\""
  group-optimize-demo)
flux_add_cli_test(
  flux_cli_constraint_global_optimize_demo
  "\"global_atomically_legalized_components\": 1"
  constraint-global-optimize-demo)
flux_add_cli_test(
  flux_cli_constraint_global_optimize_provenance
  "\"relative_constraints\": \\[\"stage-anchor\"\\]"
  constraint-global-optimize-demo)
flux_add_cli_test(
  flux_cli_routing_guided_global_optimize_demo
  "\"global_routing_guided_footprints\": 1"
  routing-guided-global-optimize-demo)
flux_add_cli_test(
  flux_cli_routing_guided_global_optimize_acceptance
  "\"global_seed_accepted\": true"
  routing-guided-global-optimize-demo)
flux_add_cli_test(flux_cli_refine_demo "\"feasible\": true" refine-demo)
flux_add_cli_test(
  flux_cli_global_optimize_demo "\"global_seed_accepted\": true" global-optimize-demo)
flux_add_cli_test(
  flux_cli_inspect_kicad "\"net_classes\": 0"
  inspect-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb")
flux_add_cli_test(
  flux_cli_inspect_kicad_cutouts "\"board_cutouts\": 0"
  inspect-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb")
flux_add_cli_test(
  flux_cli_inspect_free_space_kicad "\"shared_domain\": true"
  inspect-free-space-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb" SIGNAL F.Cu --shared-domain)
flux_add_cli_test(
  flux_cli_constraints_template_kicad "flux_orientation_constraints"
  constraints-template-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb")
flux_add_cli_test(
  flux_cli_envelopes_template_kicad "flux_placement_envelopes"
  envelopes-template-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb")
flux_add_cli_test(
  flux_cli_diffpairs_template_kicad "flux_differential_pair_constraints"
  diffpairs-template-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb")
flux_add_cli_test(
  flux_cli_buses_template_kicad "flux_routing_bus_constraints"
  buses-template-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb")
flux_add_cli_test(
  flux_cli_placement_groups_template_kicad "flux_placement_group_constraints"
  placement-groups-template-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb")
flux_add_cli_test(
  flux_cli_placement_regions_template_kicad "flux_placement_region_constraints"
  placement-regions-template-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb")
flux_add_cli_test(
  flux_cli_relative_placements_template_kicad "flux_relative_placement_constraints"
  relative-placements-template-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb")
flux_add_cli_test(
  flux_cli_placement_move_priorities_template_kicad
  "tiers resistor capacitor transistor other misc_ic mcu soc connector"
  placement-move-priorities-template-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb")
flux_add_cli_test(
  flux_cli_constrained_optimize_kicad "\"complete\": true"
  constrained-optimize-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb"
  "${FLUX_FIXTURES}/minimal.orientation_constraints")
flux_add_cli_test(
  flux_cli_route_kicad "\"complete\": true"
  route-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb")
flux_add_cli_test(
  flux_cli_route_kicad_unrouted_net_ids "\"unrouted_net_ids\": \\[\\]"
  route-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb")
flux_add_cli_test(
  flux_cli_route_kicad_bounded "\"iterations\": 1"
  route-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb" --route-net SIGNAL
  --max-router-iterations 1 --max-expansions 10000 --pin-escape-radius 0
  --max-pin-access-nodes 1)
flux_add_cli_test(
  flux_cli_route_kicad_multisource_steiner
  "\"multisource_steiner_growth_enabled\": true"
  route-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb" --multisource-steiner)
flux_add_cli_test(
  flux_cli_route_kicad_atomic_net_trees
  "\"atomic_complete_net_routing_enabled\": true"
  route-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb" --atomic-net-trees)
flux_add_cli_test(
  flux_cli_route_kicad_selective_congestion_reroute
  "\"configured_selective_full_reroute_interval\": 2"
  route-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb" --atomic-net-trees
  --selective-congestion-reroute --selective-full-reroute-interval 2)
add_test(
  NAME flux_cli_route_kicad_rejects_unbacked_selective_congestion_reroute
  COMMAND flux-cli route-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb"
          --selective-congestion-reroute)
set_tests_properties(
  flux_cli_route_kicad_rejects_unbacked_selective_congestion_reroute PROPERTIES WILL_FAIL TRUE)
add_test(
  NAME flux_cli_route_kicad_rejects_duplicate_selective_congestion_reroute
  COMMAND flux-cli route-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb" --atomic-net-trees
          --selective-congestion-reroute --selective-congestion-reroute)
set_tests_properties(
  flux_cli_route_kicad_rejects_duplicate_selective_congestion_reroute PROPERTIES WILL_FAIL TRUE)
flux_add_cli_test(
  flux_cli_route_kicad_weighted_search_retry
  "\"configured_expansion_retry_heuristic_weight_per_mille\": 2000"
  route-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb"
  --max-expansions 10000 --expansion-retry-heuristic-weight-per-mille 2000)
flux_add_cli_test(
  flux_cli_route_kicad_reused_weighted_frontier
  "\"configured_reuse_weighted_search_frontier\": true"
  route-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb"
  --max-expansions 10000 --expansion-retry-heuristic-weight-per-mille 2000
  --reuse-weighted-search-frontier)
flux_add_cli_test(
  flux_cli_route_kicad_physical_overflow_objective
  "\"configured_protect_recovered_hard_capacity_ripup_nets\": true"
  route-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb" --direct-medial
  --physical-overflow-objective --hard-capacity-start-iteration 2 --atomic-net-trees
  --max-hard-capacity-ripup-attempts-per-pass 2
  --max-hard-capacity-ripup-nets-per-attempt 1
  --max-hard-capacity-conflict-set-states-per-attempt 16
  --rollback-failed-hard-capacity-ripup
  --defer-hard-capacity-ripup-victims-to-pass-tail
  --protect-recovered-hard-capacity-ripup-nets
  --selective-congestion-reroute
  --max-hard-capacity-blocker-reuse-invalidations-per-failed-net 1)
add_test(
  NAME flux_cli_route_kicad_rejects_unbacked_physical_overflow_objective
  COMMAND flux-cli route-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb"
          --physical-overflow-objective)
set_tests_properties(
  flux_cli_route_kicad_rejects_unbacked_physical_overflow_objective PROPERTIES WILL_FAIL TRUE)
add_test(
  NAME flux_cli_route_kicad_rejects_duplicate_physical_overflow_objective
  COMMAND flux-cli route-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb" --direct-medial
          --physical-overflow-objective --physical-overflow-objective)
set_tests_properties(
  flux_cli_route_kicad_rejects_duplicate_physical_overflow_objective PROPERTIES WILL_FAIL TRUE)
flux_add_cli_test(
  flux_cli_route_kicad_physical_overflow_history
  "\"configured_physical_overflow_history\": true"
  route-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb" --direct-medial
  --physical-overflow-history)
add_test(
  NAME flux_cli_route_kicad_rejects_unbacked_physical_overflow_history
  COMMAND flux-cli route-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb"
          --physical-overflow-history)
set_tests_properties(
  flux_cli_route_kicad_rejects_unbacked_physical_overflow_history PROPERTIES WILL_FAIL TRUE)
add_test(
  NAME flux_cli_route_kicad_rejects_duplicate_physical_overflow_history
  COMMAND flux-cli route-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb" --direct-medial
          --physical-overflow-history --physical-overflow-history)
set_tests_properties(
  flux_cli_route_kicad_rejects_duplicate_physical_overflow_history PROPERTIES WILL_FAIL TRUE)
flux_add_cli_test(
  flux_cli_route_kicad_physical_envelope_ordering
  "\"configured_physical_envelope_ordering\": true"
  route-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb" --direct-medial
  --physical-envelope-ordering)
add_test(
  NAME flux_cli_route_kicad_rejects_unbacked_physical_envelope_ordering
  COMMAND flux-cli route-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb"
          --physical-envelope-ordering)
set_tests_properties(
  flux_cli_route_kicad_rejects_unbacked_physical_envelope_ordering PROPERTIES WILL_FAIL TRUE)
add_test(
  NAME flux_cli_route_kicad_rejects_duplicate_physical_envelope_ordering
  COMMAND flux-cli route-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb" --direct-medial
          --physical-envelope-ordering --physical-envelope-ordering)
set_tests_properties(
  flux_cli_route_kicad_rejects_duplicate_physical_envelope_ordering PROPERTIES WILL_FAIL TRUE)
flux_add_cli_test(
  flux_cli_route_kicad_present_cost_escalation
  "\"configured_present_congestion_escalation_start_iteration\": 3"
  route-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb"
  --present-congestion-multiplier-per-mille 2000
  --max-present-congestion-scale-per-mille 100000
  --present-congestion-escalation-start-iteration 3)
flux_add_cli_test(
  flux_cli_route_kicad_diversified_pin_access
  "\"configured_diversify_pin_access_rings\": true"
  route-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb"
  --pin-escape-radius 2 --max-pin-access-nodes 32 --diversify-pin-access-rings)
flux_add_cli_test(
  flux_cli_route_kicad_terminal_neck_down
  "\"configured_terminal_neck_down_width_nm\": 100000"
  route-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb" --route-net SIGNAL
  --terminal-neck-down-width-nm 100000 --max-terminal-neck-down-length-nm 1000000)
flux_add_cli_test(
  flux_cli_route_kicad_medial_neck_down
  "\"configured_medial_neck_down_width_nm\": 100000"
  route-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb" --direct-medial
  --medial-neck-down-width-nm 100000 --max-medial-neck-down-length-nm 100000000)
flux_add_cli_test(
  flux_cli_route_kicad_corridor_neck_down
  "\"configured_width_reduction_cost_per_grid_step\": 0"
  route-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb"
  --corridor-neck-down-width-nm 100000 --max-corridor-neck-down-length-nm 100000000
  --max-corridor-neck-down-states 5000000 --width-reduction-cost-per-grid-step 0)
flux_add_cli_test(
  flux_cli_route_kicad_longitudinal_medial_resources
  "\"configured_max_voronoi_resource_segment_length_nm\": 1000000"
  route-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb"
  --max-voronoi-resource-segment-length-nm 1000000)
flux_add_cli_test(
  flux_cli_detail_kicad "\"complete\": true"
  detail-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb")
flux_add_cli_test(
  flux_cli_detail_kicad_width_aware_metrics
  "\"width_aware_circular_arc_paths\": 0"
  detail-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb")
flux_add_cli_test(
  flux_cli_detail_kicad_with_netclasses "\"complete\": true"
  detail-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb"
  --netclasses "${FLUX_FIXTURES}/minimal.netclasses"
  --diffpairs "${FLUX_FIXTURES}/minimal.diffpairs"
  --buses "${FLUX_FIXTURES}/minimal.buses")
flux_add_cli_test(
  flux_cli_detail_kicad_with_skew_tuning
  "\"configured_max_differential_pair_skew_tuning_periods\": 8"
  detail-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb"
  --diffpairs "${FLUX_FIXTURES}/minimal.diffpairs" --tune-differential-pair-skew)
flux_add_cli_test(
  flux_cli_detail_kicad_with_batch_skew_tuning
  "\"configured_differential_pair_skew_tuning_batch_assignment\": true"
  detail-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb"
  --diffpairs "${FLUX_FIXTURES}/minimal.diffpairs"
  --batch-assign-differential-pair-skew)
flux_add_cli_test(
  flux_cli_detail_kicad_with_any_direction_skew_metrics
  "\"assessed_any_direction_differential_pair_skew_tuning_regions\": 0"
  detail-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb"
  --diffpairs "${FLUX_FIXTURES}/minimal.diffpairs" --tune-differential-pair-skew)

add_test(
  NAME flux_cli_incomplete_preview_is_written_but_fails_closed
  COMMAND "${CMAKE_COMMAND}"
          -DFLUX_CLI=$<TARGET_FILE:flux-cli>
          -DSOURCE_BOARD=${FLUX_FIXTURES}/routing-corpus/pt2399_delay.kicad_pcb
          -DPREVIEW_BOARD=${CMAKE_CURRENT_BINARY_DIR}/incomplete-preview.kicad_pcb
          -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/verify_incomplete_preview.cmake)

set(FLUX_AUTOROUTE_BOARD "${CMAKE_CURRENT_BINARY_DIR}/minimal-autorouted.kicad_pcb")
set(FLUX_AUTOROUTE_WORKFLOW_BOARD
    "${CMAKE_CURRENT_BINARY_DIR}/minimal-workflow-autorouted.kicad_pcb")
set(FLUX_PLACE_ROUTE_WORKFLOW_BOARD
    "${CMAKE_CURRENT_BINARY_DIR}/minimal-workflow-place-routed.kicad_pcb")
add_test(
  NAME flux_kicad_autoroute_prepare
  COMMAND "${CMAKE_COMMAND}" -E rm -f
          "${FLUX_AUTOROUTE_BOARD}" "${FLUX_AUTOROUTE_WORKFLOW_BOARD}"
          "${FLUX_PLACE_ROUTE_WORKFLOW_BOARD}")
set_tests_properties(flux_kicad_autoroute_prepare PROPERTIES FIXTURES_SETUP kicad_autoroute)
add_test(
  NAME flux_cli_autoroute_kicad_export
  COMMAND flux-cli autoroute-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb"
          "${FLUX_AUTOROUTE_BOARD}"
          --netclasses "${FLUX_FIXTURES}/minimal.netclasses")
set_tests_properties(
  flux_cli_autoroute_kicad_export PROPERTIES
  FIXTURES_REQUIRED kicad_autoroute
  PASS_REGULAR_EXPRESSION "\"workflow\": \"autoroute_v1\"")
if(Python3_Interpreter_FOUND)
  add_test(
    NAME flux_kicad_autoroute_workflow
    COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/../plugins/kicad/route_board.py"
            --board "${FLUX_FIXTURES}/minimal.kicad_pcb"
            --output "${FLUX_AUTOROUTE_WORKFLOW_BOARD}"
            --flux-cli $<TARGET_FILE:flux-cli>
            --netclasses "${FLUX_FIXTURES}/minimal.netclasses"
            --skip-external-drc)
  set_tests_properties(
    flux_kicad_autoroute_workflow PROPERTIES
    FIXTURES_REQUIRED kicad_autoroute
    PASS_REGULAR_EXPRESSION "\"ok\": true"
    ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1")
  add_test(
    NAME flux_kicad_candidate_bridge_current_schema
    COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/../plugins/kicad/apply_candidate.py"
            --board "${FLUX_FIXTURES}/minimal.kicad_pcb"
            --candidate "${FLUX_AUTOROUTE_BOARD}"
            --flux-cli $<TARGET_FILE:flux-cli> --validate-only)
  set_tests_properties(
    flux_kicad_candidate_bridge_current_schema PROPERTIES
    DEPENDS flux_cli_autoroute_kicad_export
    FIXTURES_REQUIRED kicad_autoroute
    PASS_REGULAR_EXPRESSION "\"mode\": \"validate-only\""
    ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1")
  add_test(
    NAME flux_kicad_place_route_workflow
    COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/../plugins/kicad/route_board.py"
            --board "${FLUX_FIXTURES}/minimal.kicad_pcb"
            --output "${FLUX_PLACE_ROUTE_WORKFLOW_BOARD}"
            --flux-cli $<TARGET_FILE:flux-cli>
            --netclasses "${FLUX_FIXTURES}/minimal.netclasses"
            --integrated-placement --max-placement-passes 1
            --max-candidate-evaluations 4 --skip-external-drc)
  set_tests_properties(
    flux_kicad_place_route_workflow PROPERTIES
    FIXTURES_REQUIRED kicad_autoroute
    PASS_REGULAR_EXPRESSION "\"ok\": true"
    ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1")
endif()
add_test(
  NAME flux_kicad_autoroute_cleanup
  COMMAND "${CMAKE_COMMAND}" -E rm -f
          "${FLUX_AUTOROUTE_BOARD}" "${FLUX_AUTOROUTE_WORKFLOW_BOARD}"
          "${FLUX_PLACE_ROUTE_WORKFLOW_BOARD}")
set_tests_properties(flux_kicad_autoroute_cleanup PROPERTIES FIXTURES_CLEANUP kicad_autoroute)

flux_add_cli_test(
  flux_cli_optimize_kicad "\"configured_move_step_nm\": 1000000"
  optimize-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb" --move-step-nm 1000000)
flux_add_cli_test(
  flux_cli_optimize_kicad_dimensioned_physical_score
  "\"configured_dimensioned_physical_overflow\": true"
  optimize-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb" --direct-medial
  --physical-overflow-objective)
flux_add_cli_test(
  flux_cli_optimize_kicad_with_swaps "\"complete\": true"
  optimize-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb" --swaps)
flux_add_cli_test(
  flux_cli_optimize_kicad_with_placement_move_priorities
  "\"configured_placement_move_priorities\": true"
  optimize-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb"
  --placement-move-priorities "${FLUX_FIXTURES}/minimal.placement_move_priorities")
flux_add_cli_test(
  flux_cli_optimize_kicad_with_group_moves "\"complete\": true"
  optimize-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb"
  --placement-groups "${FLUX_FIXTURES}/minimal.placement_groups" --group-moves)
flux_add_cli_test(
  flux_cli_optimize_kicad_with_placement_region "\"complete\": true"
  optimize-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb"
  --placement-regions "${FLUX_FIXTURES}/minimal.placement_regions")
flux_add_cli_test(
  flux_cli_optimize_kicad_with_relative_placement "\"complete\": true"
  optimize-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb"
  --relative-placements "${FLUX_FIXTURES}/minimal.relative_placements")
flux_add_cli_test(
  flux_cli_optimize_kicad_with_relative_distance_rotation "\"complete\": true"
  optimize-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb"
  --relative-placements "${FLUX_FIXTURES}/minimal.relative_placements_v2")
flux_add_cli_test(
  flux_cli_optimize_kicad_with_relative_component_moves "\"complete\": true"
  optimize-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb"
  --relative-placements "${FLUX_FIXTURES}/minimal.relative_placements"
  --relative-component-moves)
flux_add_cli_test(
  flux_cli_optimize_kicad_with_netclasses "\"complete\": true"
  optimize-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb"
  --netclasses "${FLUX_FIXTURES}/minimal.netclasses")
flux_add_cli_test(
  flux_cli_global_optimize_kicad_with_routing_guidance "\"complete\": true"
  global-optimize-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb"
  --routing-guided-global-placement)
flux_add_cli_test(
  flux_cli_global_optimize_kicad_with_assignment
  "\"global_assignment_legalization_attempted\": true"
  global-optimize-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb"
  --assignment-global-placement)
flux_add_cli_test(
  flux_cli_global_optimize_kicad_with_tentative_envelopes
  "\"placement_move_priority_review_required\": true"
  global-optimize-kicad "${FLUX_FIXTURES}/tentative-placement.kicad_pcb"
  --tentative-envelopes --suggested-placement-move-priorities
  --max-placement-passes 1 --max-candidate-evaluations 2)

set(FLUX_EXPORTED_BOARD "${CMAKE_CURRENT_BINARY_DIR}/minimal-optimized.kicad_pcb")
set(FLUX_EXPORTED_DRC "${CMAKE_CURRENT_BINARY_DIR}/minimal-optimized-drc.json")
add_test(
  NAME flux_kicad_export_prepare
  COMMAND "${CMAKE_COMMAND}" -E rm -f "${FLUX_EXPORTED_BOARD}" "${FLUX_EXPORTED_DRC}")
set_tests_properties(flux_kicad_export_prepare PROPERTIES FIXTURES_SETUP kicad_export)
add_test(
  NAME flux_cli_optimize_kicad_export
  COMMAND flux-cli optimize-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb"
          --netclasses "${FLUX_FIXTURES}/minimal.netclasses" "${FLUX_EXPORTED_BOARD}")
set_tests_properties(
  flux_cli_optimize_kicad_export PROPERTIES
  FIXTURES_REQUIRED kicad_export
  PASS_REGULAR_EXPRESSION "\"artifact_written\": true")
add_test(
  NAME flux_cli_changeset_kicad
  COMMAND flux-cli changeset-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb"
          "${FLUX_EXPORTED_BOARD}")
set_tests_properties(
  flux_cli_changeset_kicad PROPERTIES
  DEPENDS flux_cli_optimize_kicad_export
  FIXTURES_REQUIRED kicad_export
  PASS_REGULAR_EXPRESSION "\"source_inventory\"")
add_test(
  NAME flux_cli_kicad_export_refuses_overwrite
  COMMAND flux-cli optimize-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb"
          --netclasses "${FLUX_FIXTURES}/minimal.netclasses" "${FLUX_EXPORTED_BOARD}")
set_tests_properties(
  flux_cli_kicad_export_refuses_overwrite PROPERTIES
  DEPENDS flux_cli_optimize_kicad_export
  FIXTURES_REQUIRED kicad_export
  WILL_FAIL TRUE)
add_test(
  NAME flux_kicad_export_cleanup
  COMMAND "${CMAKE_COMMAND}" -E rm -f "${FLUX_EXPORTED_BOARD}" "${FLUX_EXPORTED_DRC}")
set_tests_properties(flux_kicad_export_cleanup PROPERTIES FIXTURES_CLEANUP kicad_export)

set(FLUX_SPECCTRA_DSN "${CMAKE_CURRENT_BINARY_DIR}/specctra-open.dsn")
set(FLUX_SPECCTRA_BOARD "${CMAKE_CURRENT_BINARY_DIR}/specctra-open-routed.kicad_pcb")
set(FLUX_SPECCTRA_DRC "${CMAKE_CURRENT_BINARY_DIR}/specctra-open-routed-drc.json")
add_test(
  NAME flux_specctra_prepare
  COMMAND "${CMAKE_COMMAND}" -E rm -f
          "${FLUX_SPECCTRA_DSN}" "${FLUX_SPECCTRA_BOARD}" "${FLUX_SPECCTRA_DRC}")
set_tests_properties(flux_specctra_prepare PROPERTIES FIXTURES_SETUP specctra_exchange)
add_test(
  NAME flux_cli_export_dsn_kicad
  COMMAND flux-cli export-dsn-kicad "${FLUX_FIXTURES}/specctra-open.kicad_pcb"
          "${FLUX_SPECCTRA_DSN}")
set_tests_properties(
  flux_cli_export_dsn_kicad PROPERTIES
  FIXTURES_REQUIRED specctra_exchange
  PASS_REGULAR_EXPRESSION "\"artifact_written\": true")
add_test(
  NAME flux_cli_export_dsn_refuses_overwrite
  COMMAND flux-cli export-dsn-kicad "${FLUX_FIXTURES}/specctra-open.kicad_pcb"
          "${FLUX_SPECCTRA_DSN}")
set_tests_properties(
  flux_cli_export_dsn_refuses_overwrite PROPERTIES
  DEPENDS flux_cli_export_dsn_kicad
  FIXTURES_REQUIRED specctra_exchange
  WILL_FAIL TRUE)
add_test(
  NAME flux_cli_import_ses_kicad
  COMMAND flux-cli import-ses-kicad "${FLUX_FIXTURES}/specctra-open.kicad_pcb"
          "${FLUX_SPECCTRA_DSN}" "${FLUX_FIXTURES}/specctra-open.ses"
          "${FLUX_SPECCTRA_BOARD}")
set_tests_properties(
  flux_cli_import_ses_kicad PROPERTIES
  DEPENDS flux_cli_export_dsn_kicad
  FIXTURES_REQUIRED specctra_exchange
  PASS_REGULAR_EXPRESSION "\"open_connections\": 0")
add_test(
  NAME flux_specctra_cleanup
  COMMAND "${CMAKE_COMMAND}" -E rm -f
          "${FLUX_SPECCTRA_DSN}" "${FLUX_SPECCTRA_BOARD}" "${FLUX_SPECCTRA_DRC}")
set_tests_properties(flux_specctra_cleanup PROPERTIES FIXTURES_CLEANUP specctra_exchange)

set(FLUX_GLOBAL_EXPORTED_BOARD
    "${CMAKE_CURRENT_BINARY_DIR}/minimal-global-optimized.kicad_pcb")
set(FLUX_GLOBAL_EXPORTED_DRC
    "${CMAKE_CURRENT_BINARY_DIR}/minimal-global-optimized-drc.json")
add_test(
  NAME flux_kicad_global_export_prepare
  COMMAND "${CMAKE_COMMAND}" -E rm -f
          "${FLUX_GLOBAL_EXPORTED_BOARD}" "${FLUX_GLOBAL_EXPORTED_DRC}")
set_tests_properties(flux_kicad_global_export_prepare PROPERTIES
  FIXTURES_SETUP kicad_global_export)
add_test(
  NAME flux_cli_global_optimize_kicad_export
  COMMAND flux-cli global-optimize-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb"
          "${FLUX_GLOBAL_EXPORTED_BOARD}")
set_tests_properties(
  flux_cli_global_optimize_kicad_export PROPERTIES
  FIXTURES_REQUIRED kicad_global_export
  PASS_REGULAR_EXPRESSION "\"artifact_written\": true")
add_test(
  NAME flux_kicad_global_export_cleanup
  COMMAND "${CMAKE_COMMAND}" -E rm -f
          "${FLUX_GLOBAL_EXPORTED_BOARD}" "${FLUX_GLOBAL_EXPORTED_DRC}")
set_tests_properties(flux_kicad_global_export_cleanup PROPERTIES
  FIXTURES_CLEANUP kicad_global_export)

set(FLUX_CONSTRAINED_EXPORTED_BOARD
    "${CMAKE_CURRENT_BINARY_DIR}/minimal-constrained-refined.kicad_pcb")
set(FLUX_CONSTRAINED_EXPORTED_DRC
    "${CMAKE_CURRENT_BINARY_DIR}/minimal-constrained-refined-drc.json")
add_test(
  NAME flux_kicad_constrained_export_prepare
  COMMAND "${CMAKE_COMMAND}" -E rm -f
          "${FLUX_CONSTRAINED_EXPORTED_BOARD}" "${FLUX_CONSTRAINED_EXPORTED_DRC}")
set_tests_properties(flux_kicad_constrained_export_prepare PROPERTIES
  FIXTURES_SETUP kicad_constrained_export)
add_test(
  NAME flux_cli_constrained_refine_kicad_export
  COMMAND flux-cli constrained-refine-kicad "${FLUX_FIXTURES}/minimal.kicad_pcb"
          "${FLUX_FIXTURES}/minimal.orientation_constraints"
          "${FLUX_CONSTRAINED_EXPORTED_BOARD}")
set_tests_properties(
  flux_cli_constrained_refine_kicad_export PROPERTIES
  FIXTURES_REQUIRED kicad_constrained_export
  PASS_REGULAR_EXPRESSION "\"artifact_written\": true")
add_test(
  NAME flux_kicad_constrained_export_cleanup
  COMMAND "${CMAKE_COMMAND}" -E rm -f
          "${FLUX_CONSTRAINED_EXPORTED_BOARD}" "${FLUX_CONSTRAINED_EXPORTED_DRC}")
set_tests_properties(flux_kicad_constrained_export_cleanup PROPERTIES
  FIXTURES_CLEANUP kicad_constrained_export)

set(FLUX_ARC_DETOUR_SOURCE "${CMAKE_CURRENT_BINARY_DIR}/arc-detour-source.kicad_pcb")
set(FLUX_ARC_DETOUR_BOARD "${CMAKE_CURRENT_BINARY_DIR}/arc-detour-candidate.kicad_pcb")
set(FLUX_ARC_DETOUR_DRC "${CMAKE_CURRENT_BINARY_DIR}/arc-detour-drc.json")
add_test(
  NAME flux_kicad_arc_detour_prepare
  COMMAND "${CMAKE_COMMAND}"
          -DINPUT=${FLUX_FIXTURES}/minimal.kicad_pcb
          -DOUTPUT=${FLUX_ARC_DETOUR_SOURCE}
          -DCANDIDATE=${FLUX_ARC_DETOUR_BOARD}
          -DDRC_REPORT=${FLUX_ARC_DETOUR_DRC}
          -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/make_arc_detour_fixture.cmake)
set_tests_properties(flux_kicad_arc_detour_prepare PROPERTIES
  FIXTURES_SETUP kicad_arc_detour)
add_test(
  NAME flux_cli_detail_kicad_arc_export
  COMMAND flux-cli detail-kicad "${FLUX_ARC_DETOUR_SOURCE}" --arcs
          "${FLUX_ARC_DETOUR_BOARD}")
set_tests_properties(
  flux_cli_detail_kicad_arc_export PROPERTIES
  FIXTURES_REQUIRED kicad_arc_detour
  PASS_REGULAR_EXPRESSION "\"fitted_circular_arcs\": [1-9][0-9]*")
add_test(
  NAME flux_kicad_arc_export_is_native
  COMMAND "${CMAKE_COMMAND}" -DINPUT=${FLUX_ARC_DETOUR_BOARD}
          -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/assert_native_track_arc.cmake)
set_tests_properties(
  flux_kicad_arc_export_is_native PROPERTIES
  DEPENDS flux_cli_detail_kicad_arc_export
  FIXTURES_REQUIRED kicad_arc_detour)
add_test(
  NAME flux_kicad_arc_detour_cleanup
  COMMAND "${CMAKE_COMMAND}" -E rm -f
          "${FLUX_ARC_DETOUR_SOURCE}" "${FLUX_ARC_DETOUR_BOARD}" "${FLUX_ARC_DETOUR_DRC}")
set_tests_properties(flux_kicad_arc_detour_cleanup PROPERTIES
  FIXTURES_CLEANUP kicad_arc_detour)

set(FLUX_MULTILAYER_SOURCE "${CMAKE_CURRENT_BINARY_DIR}/multilayer-source.kicad_pcb")
set(FLUX_MULTILAYER_BOARD "${CMAKE_CURRENT_BINARY_DIR}/multilayer-candidate.kicad_pcb")
set(FLUX_MULTILAYER_DRC "${CMAKE_CURRENT_BINARY_DIR}/multilayer-drc.json")
add_test(
  NAME flux_kicad_multilayer_prepare
  COMMAND "${CMAKE_COMMAND}"
          -DINPUT=${FLUX_FIXTURES}/minimal.kicad_pcb
          -DOUTPUT=${FLUX_MULTILAYER_SOURCE}
          -DCANDIDATE=${FLUX_MULTILAYER_BOARD}
          -DDRC_REPORT=${FLUX_MULTILAYER_DRC}
          -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/make_multilayer_fixture.cmake)
set_tests_properties(flux_kicad_multilayer_prepare PROPERTIES
  FIXTURES_SETUP kicad_multilayer)
add_test(
  NAME flux_cli_detail_kicad_multilayer_export
  COMMAND flux-cli detail-kicad "${FLUX_MULTILAYER_SOURCE}" --strict-medial
          "${FLUX_MULTILAYER_BOARD}")
set_tests_properties(
  flux_cli_detail_kicad_multilayer_export PROPERTIES
  FIXTURES_REQUIRED kicad_multilayer
  PASS_REGULAR_EXPRESSION "\"materialized_multilayer_topology_locked_paths\": 1")
add_test(
  NAME flux_kicad_multilayer_cleanup
  COMMAND "${CMAKE_COMMAND}" -E rm -f
          "${FLUX_MULTILAYER_SOURCE}" "${FLUX_MULTILAYER_BOARD}" "${FLUX_MULTILAYER_DRC}")
set_tests_properties(flux_kicad_multilayer_cleanup PROPERTIES
  FIXTURES_CLEANUP kicad_multilayer)

if(KICAD_CLI_EXECUTABLE)
  add_test(
    NAME flux_kicad_fixture_drc
    COMMAND "${KICAD_CLI_EXECUTABLE}" pcb drc
            --output "${CMAKE_CURRENT_BINARY_DIR}/minimal-drc.json"
            --format json --severity-error --exit-code-violations
            "${FLUX_FIXTURES}/minimal.kicad_pcb")
  add_test(
    NAME flux_kicad_exported_drc
    COMMAND "${KICAD_CLI_EXECUTABLE}" pcb drc
            --output "${FLUX_EXPORTED_DRC}" --format json --severity-error
            --exit-code-violations "${FLUX_EXPORTED_BOARD}")
  set_tests_properties(flux_kicad_exported_drc PROPERTIES
    DEPENDS flux_cli_optimize_kicad_export
    FIXTURES_REQUIRED kicad_export)
  add_test(
    NAME flux_kicad_global_exported_drc
    COMMAND "${KICAD_CLI_EXECUTABLE}" pcb drc
            --output "${FLUX_GLOBAL_EXPORTED_DRC}" --format json --severity-error
            --exit-code-violations "${FLUX_GLOBAL_EXPORTED_BOARD}")
  set_tests_properties(flux_kicad_global_exported_drc PROPERTIES
    DEPENDS flux_cli_global_optimize_kicad_export
    FIXTURES_REQUIRED kicad_global_export)
  add_test(
    NAME flux_kicad_constrained_exported_drc
    COMMAND "${KICAD_CLI_EXECUTABLE}" pcb drc
            --output "${FLUX_CONSTRAINED_EXPORTED_DRC}" --format json --severity-error
            --exit-code-violations "${FLUX_CONSTRAINED_EXPORTED_BOARD}")
  set_tests_properties(flux_kicad_constrained_exported_drc PROPERTIES
    DEPENDS flux_cli_constrained_refine_kicad_export
    FIXTURES_REQUIRED kicad_constrained_export)
  add_test(
    NAME flux_kicad_arc_exported_drc
    COMMAND "${KICAD_CLI_EXECUTABLE}" pcb drc
            --output "${FLUX_ARC_DETOUR_DRC}" --format json --severity-error
            --exit-code-violations "${FLUX_ARC_DETOUR_BOARD}")
  set_tests_properties(flux_kicad_arc_exported_drc PROPERTIES
    DEPENDS flux_kicad_arc_export_is_native
    FIXTURES_REQUIRED kicad_arc_detour)
  add_test(
    NAME flux_kicad_multilayer_exported_drc
    COMMAND "${KICAD_CLI_EXECUTABLE}" pcb drc
            --output "${FLUX_MULTILAYER_DRC}" --format json --severity-error
            --exit-code-violations "${FLUX_MULTILAYER_BOARD}")
  set_tests_properties(flux_kicad_multilayer_exported_drc PROPERTIES
    DEPENDS flux_cli_detail_kicad_multilayer_export
    FIXTURES_REQUIRED kicad_multilayer)
  add_test(
    NAME flux_kicad_specctra_imported_drc
    COMMAND "${KICAD_CLI_EXECUTABLE}" pcb drc
            --output "${FLUX_SPECCTRA_DRC}" --format json --severity-error
            --exit-code-violations "${FLUX_SPECCTRA_BOARD}")
  set_tests_properties(flux_kicad_specctra_imported_drc PROPERTIES
    DEPENDS flux_cli_import_ses_kicad
    FIXTURES_REQUIRED specctra_exchange)
endif()
