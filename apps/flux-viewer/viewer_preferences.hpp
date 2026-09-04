#pragma once

#include <filesystem>
#include <string>

namespace flux::viewer {

struct OverlayPreferences {
  bool source_board{true};
  bool source_footprints{true};
  bool source_copper{true};
  bool source_zones_keepouts{true};
  bool committed_routes{true};
  bool current_routes{true};
  bool retained_coarse_routes{true};
  bool river_routes{true};
  bool rrt_routes{true};
  bool hydraulic_routes{true};
  bool hydraulic_fields{true};
  bool ph_curves{true};
  bool ph_controls{false};
  bool rip_up_activity{true};
  bool congestion{true};
  bool clearance_shadows{true};
  bool occupancy{true};
  bool priorities{false};
  bool via_candidates{true};
  bool selected_vias{true};
  bool contested_vias{true};
  bool rejected_vias{false};
  bool grid{true};
  std::string layer;

  bool operator==(const OverlayPreferences&) const = default;
};

struct ViewerPreferences {
  float ui_scale{1.0F};
  float activity_panel_height{220.0F};
  float replay_speed{1.0F};
  std::string endpoint{"ws://127.0.0.1:8765/telemetry"};
  std::string telemetry_level{"normal"};
  double telemetry_rate_hz{10.0};
  std::filesystem::path trace_directory;
  std::filesystem::path record_directory;
  OverlayPreferences overlays;

  bool operator==(const ViewerPreferences&) const = default;
};

[[nodiscard]] std::filesystem::path standard_viewer_config_path();

// A missing file is a successful load of defaults. Malformed individual keys are ignored and
// diagnosed without discarding the remaining valid preferences.
[[nodiscard]] bool load_viewer_preferences(const std::filesystem::path& path,
                                           ViewerPreferences& preferences, std::string& warning);
[[nodiscard]] bool save_viewer_preferences(const std::filesystem::path& path,
                                           const ViewerPreferences& preferences,
                                           std::string& error);

} // namespace flux::viewer
