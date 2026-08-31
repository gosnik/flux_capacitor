if(NOT DEFINED FLUX_CLI OR NOT DEFINED SOURCE_BOARD OR NOT DEFINED PREVIEW_BOARD)
  message(FATAL_ERROR "FLUX_CLI, SOURCE_BOARD, and PREVIEW_BOARD are required")
endif()

file(REMOVE "${PREVIEW_BOARD}")
execute_process(
  COMMAND "${FLUX_CLI}" detail-kicad "${SOURCE_BOARD}"
          --max-expansions 1 --max-router-iterations 1
          --write-incomplete-preview "${PREVIEW_BOARD}"
  RESULT_VARIABLE result
  OUTPUT_VARIABLE output
  ERROR_VARIABLE error)

if(NOT result EQUAL 5)
  message(FATAL_ERROR "incomplete preview returned ${result}: ${error}")
endif()
if(NOT EXISTS "${PREVIEW_BOARD}")
  message(FATAL_ERROR "incomplete preview artifact was not written")
endif()
if(NOT output MATCHES "\"artifact_written\": true" OR
   NOT output MATCHES "\"artifact_preview_only\": true" OR
   NOT output MATCHES "\"global_complete\": false")
  message(FATAL_ERROR "incomplete preview JSON did not retain fail-closed evidence: ${output}")
endif()
file(REMOVE "${PREVIEW_BOARD}")
