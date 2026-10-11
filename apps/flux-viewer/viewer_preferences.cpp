#include "viewer_preferences.hpp"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <system_error>

namespace flux::viewer {
namespace {

constexpr float minimum_ui_scale = 0.75F;
constexpr float maximum_ui_scale = 3.0F;

std::string trim(std::string value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos)
    return {};
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1U);
}

bool parse_bool(std::string value, bool& result) {
  value = trim(std::move(value));
  if (value == "true" || value == "1") {
    result = true;
    return true;
  }
  if (value == "false" || value == "0") {
    result = false;
    return true;
  }
  return false;
}

bool parse_string(const std::string& value, std::string& result) {
  std::istringstream input(value);
  input >> std::quoted(result);
  return static_cast<bool>(input);
}

template <typename Number> bool parse_number(const std::string& value, Number& result) {
  std::istringstream input(value);
  input >> result;
  return static_cast<bool>(input) && input.peek() == std::char_traits<char>::eof();
}

void write_bool(std::ostream& output, std::string_view key, bool value) {
  output << key << '=' << (value ? "true" : "false") << '\n';
}

void write_string(std::ostream& output, std::string_view key, const std::string& value) {
  output << key << '=' << std::quoted(value) << '\n';
}

} // namespace

std::filesystem::path standard_viewer_config_path() {
#if defined(_WIN32)
  if (const char* app_data = std::getenv("APPDATA"); app_data != nullptr && *app_data != '\0') {
    return std::filesystem::path{app_data} / "Flux" / "RouterTelemetryViewer" / "viewer.conf";
  }
#elif defined(__APPLE__)
  if (const char* home = std::getenv("HOME"); home != nullptr && *home != '\0') {
    return std::filesystem::path{home} / "Library" / "Application Support" / "Flux" /
           "RouterTelemetryViewer" / "viewer.conf";
  }
#else
  if (const char* config = std::getenv("XDG_CONFIG_HOME"); config != nullptr && *config != '\0') {
    return std::filesystem::path{config} / "flux" / "router-telemetry-viewer" / "viewer.conf";
  }
  if (const char* home = std::getenv("HOME"); home != nullptr && *home != '\0') {
    return std::filesystem::path{home} / ".config" / "flux" / "router-telemetry-viewer" /
           "viewer.conf";
  }
#endif
  return std::filesystem::temp_directory_path() / "flux-router-telemetry-viewer.conf";
}

bool load_viewer_preferences(const std::filesystem::path& path, ViewerPreferences& preferences,
                             std::string& warning) {
  warning.clear();
  std::error_code error;
  if (!std::filesystem::exists(path, error)) {
    if (error) {
      warning = "could not inspect viewer preferences: " + error.message();
      return false;
    }
    return true;
  }
  std::ifstream input(path);
  if (!input) {
    warning = "could not open viewer preferences: " + path.string();
    return false;
  }

  std::size_t invalid_values = 0U;
  std::string line;
  while (std::getline(input, line)) {
    const auto separator = line.find('=');
    if (separator == std::string::npos || line.empty() || line.front() == '#')
      continue;
    const auto key = trim(line.substr(0U, separator));
    const auto value = line.substr(separator + 1U);
    bool valid = true;
    if (key == "ui_scale")
      valid = parse_number(value, preferences.ui_scale);
    else if (key == "activity_panel_height")
      valid = parse_number(value, preferences.activity_panel_height);
    else if (key == "replay_speed")
      valid = parse_number(value, preferences.replay_speed);
    else if (key == "endpoint")
      valid = parse_string(value, preferences.endpoint);
    else if (key == "telemetry_level")
      valid = parse_string(value, preferences.telemetry_level);
    else if (key == "telemetry_rate_hz")
      valid = parse_number(value, preferences.telemetry_rate_hz);
    else if (key == "trace_directory") {
      std::string decoded;
      valid = parse_string(value, decoded);
      if (valid)
        preferences.trace_directory = decoded;
    } else if (key == "record_directory") {
      std::string decoded;
      valid = parse_string(value, decoded);
      if (valid)
        preferences.record_directory = decoded;
    } else if (key == "overlay.layer")
      valid = parse_string(value, preferences.overlays.layer);
#define FLUX_PARSE_OVERLAY(name)                                                                   \
  else if (key == "overlay." #name) valid = parse_bool(value, preferences.overlays.name)
    FLUX_PARSE_OVERLAY(source_board);
    FLUX_PARSE_OVERLAY(source_footprints);
    FLUX_PARSE_OVERLAY(source_copper);
    FLUX_PARSE_OVERLAY(resulting_copper);
    FLUX_PARSE_OVERLAY(source_zones_keepouts);
    FLUX_PARSE_OVERLAY(committed_routes);
    FLUX_PARSE_OVERLAY(current_routes);
    FLUX_PARSE_OVERLAY(retained_coarse_routes);
    FLUX_PARSE_OVERLAY(river_routes);
    FLUX_PARSE_OVERLAY(rrt_routes);
    FLUX_PARSE_OVERLAY(hydraulic_routes);
    FLUX_PARSE_OVERLAY(hydraulic_fields);
    FLUX_PARSE_OVERLAY(ph_curves);
    FLUX_PARSE_OVERLAY(ph_controls);
    FLUX_PARSE_OVERLAY(rip_up_activity);
    FLUX_PARSE_OVERLAY(congestion);
    FLUX_PARSE_OVERLAY(clearance_shadows);
    FLUX_PARSE_OVERLAY(occupancy);
    FLUX_PARSE_OVERLAY(priorities);
    FLUX_PARSE_OVERLAY(via_candidates);
    FLUX_PARSE_OVERLAY(selected_vias);
    FLUX_PARSE_OVERLAY(contested_vias);
    FLUX_PARSE_OVERLAY(rejected_vias);
    FLUX_PARSE_OVERLAY(active_region);
    FLUX_PARSE_OVERLAY(grid);
#undef FLUX_PARSE_OVERLAY
    else {
      continue;
    }
    if (!valid)
      ++invalid_values;
  }
  preferences.ui_scale = std::clamp(preferences.ui_scale, minimum_ui_scale, maximum_ui_scale);
  preferences.activity_panel_height =
      std::clamp(preferences.activity_panel_height, 100.0F, 2'000.0F);
  preferences.replay_speed = std::clamp(preferences.replay_speed, 0.25F, 8.0F);
  preferences.telemetry_rate_hz = std::clamp(preferences.telemetry_rate_hz, 1.0, 60.0);
  if (preferences.telemetry_level != "normal" && preferences.telemetry_level != "diagnostic" &&
      preferences.telemetry_level != "trace") {
    preferences.telemetry_level = "normal";
    ++invalid_values;
  }
  if (invalid_values != 0U) {
    warning = "ignored " + std::to_string(invalid_values) + " invalid viewer preference value(s)";
  }
  return true;
}

bool save_viewer_preferences(const std::filesystem::path& path,
                             const ViewerPreferences& preferences, std::string& error) {
  error.clear();
  std::error_code filesystem_error;
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path(), filesystem_error);
    if (filesystem_error) {
      error = "could not create viewer configuration directory: " + filesystem_error.message();
      return false;
    }
  }
  auto temporary = path;
  temporary += ".tmp";
  std::ofstream output(temporary, std::ios::trunc);
  if (!output) {
    error = "could not write viewer preferences: " + temporary.string();
    return false;
  }
  output << "# Flux Router Telemetry Viewer preferences\nversion=1\n";
  output << "ui_scale=" << std::clamp(preferences.ui_scale, minimum_ui_scale, maximum_ui_scale)
         << '\n';
  output << "activity_panel_height=" << preferences.activity_panel_height << '\n';
  output << "replay_speed=" << preferences.replay_speed << '\n';
  write_string(output, "endpoint", preferences.endpoint);
  write_string(output, "telemetry_level", preferences.telemetry_level);
  output << "telemetry_rate_hz=" << preferences.telemetry_rate_hz << '\n';
  write_string(output, "trace_directory", preferences.trace_directory.string());
  write_string(output, "record_directory", preferences.record_directory.string());
  write_string(output, "overlay.layer", preferences.overlays.layer);
#define FLUX_WRITE_OVERLAY(name) write_bool(output, "overlay." #name, preferences.overlays.name)
  FLUX_WRITE_OVERLAY(source_board);
  FLUX_WRITE_OVERLAY(source_footprints);
  FLUX_WRITE_OVERLAY(source_copper);
  FLUX_WRITE_OVERLAY(resulting_copper);
  FLUX_WRITE_OVERLAY(source_zones_keepouts);
  FLUX_WRITE_OVERLAY(committed_routes);
  FLUX_WRITE_OVERLAY(current_routes);
  FLUX_WRITE_OVERLAY(retained_coarse_routes);
  FLUX_WRITE_OVERLAY(river_routes);
  FLUX_WRITE_OVERLAY(rrt_routes);
  FLUX_WRITE_OVERLAY(hydraulic_routes);
  FLUX_WRITE_OVERLAY(hydraulic_fields);
  FLUX_WRITE_OVERLAY(ph_curves);
  FLUX_WRITE_OVERLAY(ph_controls);
  FLUX_WRITE_OVERLAY(rip_up_activity);
  FLUX_WRITE_OVERLAY(congestion);
  FLUX_WRITE_OVERLAY(clearance_shadows);
  FLUX_WRITE_OVERLAY(occupancy);
  FLUX_WRITE_OVERLAY(priorities);
  FLUX_WRITE_OVERLAY(via_candidates);
  FLUX_WRITE_OVERLAY(selected_vias);
  FLUX_WRITE_OVERLAY(contested_vias);
  FLUX_WRITE_OVERLAY(rejected_vias);
  FLUX_WRITE_OVERLAY(active_region);
  FLUX_WRITE_OVERLAY(grid);
#undef FLUX_WRITE_OVERLAY
  output.flush();
  if (!output) {
    error = "could not flush viewer preferences: " + temporary.string();
    return false;
  }
  output.close();
  std::filesystem::rename(temporary, path, filesystem_error);
  if (filesystem_error) {
    filesystem_error.clear();
    std::filesystem::copy_file(temporary, path, std::filesystem::copy_options::overwrite_existing,
                               filesystem_error);
    if (!filesystem_error)
      std::filesystem::remove(temporary, filesystem_error);
  }
  if (filesystem_error) {
    error = "could not publish viewer preferences: " + filesystem_error.message();
    return false;
  }
  return true;
}

} // namespace flux::viewer
