cmake_minimum_required(VERSION 3.25)

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

project(
  flux_capacitor
  VERSION 0.128.0
  DESCRIPTION "Integrated PCB placement and routing engine"
  LANGUAGES CXX)

option(FLUX_BUILD_TESTS "Build flux_capacitor tests" ON)
option(FLUX_BUILD_VIEWER "Build the native ImGui router telemetry viewer" OFF)
option(FLUX_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
set(FLUX_VIEWER_IMGUI_SOURCE_DIR "" CACHE PATH
    "Existing Dear ImGui source tree; empty fetches the pinned upstream release")

find_package(Boost 1.74 REQUIRED)
find_package(Threads REQUIRED)

add_library(flux_core
  src/board_free_space.cpp
  src/connectivity.cpp
  src/copper_geometry.cpp
  src/detailed_router.cpp
  src/differential_pair_constraints.cpp
  src/drc.cpp
  src/evaluate.cpp
  src/free_space_offset.cpp
  src/free_space_skeleton.cpp
  src/geometry.cpp
  src/global_placer.cpp
  src/global_router.cpp
  src/integrated_optimizer.cpp
  src/navmesh_route.cpp
  src/keepout_geometry.cpp
  src/kicad_export.cpp
  src/kicad_import.cpp
  src/kicad_netclass_snapshot.cpp
  src/orientation_constraints.cpp
  src/placement_move_priorities.cpp
  src/placement_group_constraints.cpp
  src/placement_region_constraints.cpp
  src/pns_router.cpp
  src/regional_router.cpp
  src/relative_placement_constraints.cpp
  src/routability_knowledge.cpp
  src/placement_envelope_constraints.cpp
  src/routing_rules.cpp
  src/routing_bus_constraints.cpp
  src/router_telemetry.cpp
  src/sexpr.cpp
  src/specctra.cpp
  src/spline_route.cpp
  src/validation.cpp)
add_library(flux::core ALIAS flux_core)

target_compile_features(flux_core PUBLIC cxx_std_20)
target_include_directories(
  flux_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(flux_core PRIVATE Boost::headers Threads::Threads)
target_compile_definitions(flux_core PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)

add_library(flux_telemetry
  src/telemetry_event.cpp
  src/telemetry_state.cpp
  src/telemetry_trace.cpp)
add_library(flux::telemetry ALIAS flux_telemetry)
target_compile_features(flux_telemetry PUBLIC cxx_std_20)
target_include_directories(
  flux_telemetry
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(flux_telemetry PRIVATE Boost::headers)
target_link_libraries(flux_core PRIVATE flux::telemetry)
if(WIN32)
  target_link_libraries(flux_core PRIVATE ws2_32)
endif()
target_link_libraries(flux_core PRIVATE flux::telemetry)

if(MSVC)
  target_compile_options(flux_core PRIVATE /W4 /permissive-)
  if(FLUX_WARNINGS_AS_ERRORS)
    target_compile_options(flux_core PRIVATE /WX)
  endif()
else()
  target_compile_options(
    flux_core
    PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
  if(FLUX_WARNINGS_AS_ERRORS)
    target_compile_options(flux_core PRIVATE -Werror)
  endif()
endif()

if(MSVC)
  target_compile_options(flux_telemetry PRIVATE /W4 /permissive-)
  if(FLUX_WARNINGS_AS_ERRORS)
    target_compile_options(flux_telemetry PRIVATE /WX)
  endif()
else()
  target_compile_options(
    flux_telemetry
    PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
  if(FLUX_WARNINGS_AS_ERRORS)
    target_compile_options(flux_telemetry PRIVATE -Werror)
  endif()
endif()

add_executable(flux-cli apps/flux-cli/main.cpp)
target_link_libraries(flux-cli PRIVATE flux::core)
target_compile_features(flux-cli PRIVATE cxx_std_20)

if(FLUX_BUILD_VIEWER)
  find_package(OpenGL REQUIRED)
  find_package(SDL2 REQUIRED)

  if(FLUX_VIEWER_IMGUI_SOURCE_DIR)
    set(flux_imgui_source_dir "${FLUX_VIEWER_IMGUI_SOURCE_DIR}")
  else()
    include(FetchContent)
    FetchContent_Declare(
      flux_imgui_upstream
      GIT_REPOSITORY https://github.com/ocornut/imgui.git
      GIT_TAG v1.92.5
      GIT_SHALLOW TRUE)
    FetchContent_MakeAvailable(flux_imgui_upstream)
    set(flux_imgui_source_dir "${flux_imgui_upstream_SOURCE_DIR}")
  endif()

  if(NOT EXISTS "${flux_imgui_source_dir}/imgui.cpp" OR
     NOT EXISTS "${flux_imgui_source_dir}/backends/imgui_impl_sdl2.cpp")
    message(FATAL_ERROR
      "FLUX_BUILD_VIEWER requires a complete Dear ImGui source tree; got "
      "'${flux_imgui_source_dir}'")
  endif()

  add_library(flux_viewer_imgui STATIC
    ${flux_imgui_source_dir}/imgui.cpp
    ${flux_imgui_source_dir}/imgui_draw.cpp
    ${flux_imgui_source_dir}/imgui_tables.cpp
    ${flux_imgui_source_dir}/imgui_widgets.cpp
    ${flux_imgui_source_dir}/misc/cpp/imgui_stdlib.cpp
    ${flux_imgui_source_dir}/backends/imgui_impl_opengl3.cpp
    ${flux_imgui_source_dir}/backends/imgui_impl_sdl2.cpp)
  if(EXISTS "${flux_imgui_source_dir}/imgui_stacklayout.cpp")
    target_sources(flux_viewer_imgui PRIVATE ${flux_imgui_source_dir}/imgui_stacklayout.cpp)
  endif()
  target_include_directories(
    flux_viewer_imgui
    PUBLIC ${flux_imgui_source_dir} ${flux_imgui_source_dir}/backends)
  target_link_libraries(flux_viewer_imgui PUBLIC SDL2::SDL2 OpenGL::GL)

  add_executable(flux-viewer
    apps/flux-viewer/main.cpp
    apps/flux-viewer/live_client.cpp
    apps/flux-viewer/viewer_app.cpp)
  target_compile_features(flux-viewer PRIVATE cxx_std_20)
  target_compile_definitions(flux-viewer PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
  if(MSVC)
    target_compile_options(flux-viewer PRIVATE /W4 /permissive-)
    if(FLUX_WARNINGS_AS_ERRORS)
      target_compile_options(flux-viewer PRIVATE /WX)
    endif()
  else()
    target_compile_options(
      flux-viewer PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
    if(FLUX_WARNINGS_AS_ERRORS)
      target_compile_options(flux-viewer PRIVATE -Werror)
    endif()
  endif()
  target_link_libraries(flux-viewer PRIVATE
    flux::telemetry flux_viewer_imgui SDL2::SDL2 OpenGL::GL Threads::Threads)
  if(WIN32)
    target_link_libraries(flux-viewer PRIVATE ws2_32)
  endif()
  if(TARGET SDL2::SDL2main)
    target_link_libraries(flux-viewer PRIVATE SDL2::SDL2main)
  endif()
  install(TARGETS flux-viewer RUNTIME DESTINATION bin)
endif()

if(FLUX_BUILD_TESTS)
  include(CTest)
  add_subdirectory(tests)
endif()
