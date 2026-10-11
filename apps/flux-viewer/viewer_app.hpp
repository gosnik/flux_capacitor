#pragma once

#include "flux/telemetry/state.hpp"
#include "flux/telemetry/trace.hpp"
#include "live_client.hpp"
#include "viewer_preferences.hpp"

#include <imgui.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace flux::viewer {

struct LaunchOptions {
  std::string endpoint{"ws://127.0.0.1:8765/telemetry"};
  telemetry::Level level{telemetry::Level::normal};
  double rate_hz{10.0};
  std::filesystem::path replay_path;
  std::filesystem::path record_path;
  bool connect_on_start{true};
  std::shared_ptr<ViewerPreferences> preferences;
  std::filesystem::path preferences_path;
};

class ViewerApp {
public:
  explicit ViewerApp(LaunchOptions options);
  ~ViewerApp();

  void tick();
  void draw();
  [[nodiscard]] bool wants_exit() const { return wants_exit_; }

private:
  struct FileBrowserEntry {
    std::filesystem::path path;
    std::string name;
    bool directory{false};
  };

  struct FileBrowserState {
    std::string title;
    std::filesystem::path directory;
    std::string directory_input;
    std::filesystem::path selected;
    std::vector<FileBrowserEntry> entries;
    std::string save_name;
    std::string status;
    bool open{false};
    bool open_requested{false};
    bool save_mode{false};
  };

  struct CanvasView {
    double pixels_per_mm{24.0};
    double center_x_nm{0.0};
    double center_y_nm{0.0};
    ImVec2 pan{};
    bool fit_requested{true};
  };

  enum class SourceMode { live, replay };

  void connect_live();
  void disconnect_live();
  void load_trace(const std::filesystem::path& path);
  void start_recording(const std::filesystem::path& path);
  void stop_recording();
  void draw_menu_bar();
  void draw_source_panel();
  void draw_overlay_panel();
  void draw_legend_panel();
  void draw_board_canvas();
  void draw_metrics_panel();
  void draw_activity_panel();
  void open_file_browser(bool save_mode);
  void refresh_file_browser();
  void draw_file_browser();
  void draw_status_bar();
  void advance_replay(double elapsed_seconds);
  void seek_replay(std::size_t position);
  void update_preferences();

  [[nodiscard]] telemetry::LiveState& mutable_state();
  [[nodiscard]] const telemetry::LiveState& state() const;
  [[nodiscard]] const telemetry::LiveState& overlay_state() const;
  void sync_overlay_history_to_source();
  [[nodiscard]] std::optional<std::uint64_t> freshness_age_us(
      const std::optional<std::uint64_t>& event_time_us, bool metrics) const;

  LaunchOptions options_;
  SourceMode mode_{SourceMode::live};
  LiveClient client_;
  telemetry::LiveState live_state_;
  telemetry::LiveSessionTimeline live_timeline_;
  telemetry::Trace trace_;
  telemetry::Replay replay_;
  telemetry::LiveStateHistory overlay_history_;
  telemetry::TraceWriter recorder_;
  OverlayPreferences overlays_;
  CanvasView view_;
  std::string endpoint_;
  std::string trace_path_;
  std::string record_path_;
  std::string ui_status_;
  FileBrowserState file_browser_;
  bool replay_playing_{false};
  bool overlay_follow_source_{true};
  float replay_speed_{1.0F};
  float activity_panel_height_{220.0F};
  double replay_accumulator_us_{0.0};
  std::uint64_t replay_clock_us_{0};
  std::uint64_t live_connection_generation_{0};
  // Kept across automatic WebSocket reconnects so the first authoritative source identity can
  // distinguish a continuation of the current board from a genuinely new board.
  std::string live_board_identity_;
  std::chrono::steady_clock::time_point last_tick_{std::chrono::steady_clock::now()};
  std::optional<std::uint64_t> live_metrics_event_time_us_;
  std::optional<std::uint64_t> live_geometry_event_time_us_;
  std::optional<std::chrono::steady_clock::time_point> live_metrics_received_at_;
  std::optional<std::chrono::steady_clock::time_point> live_geometry_received_at_;
  bool wants_exit_{false};
};

} // namespace flux::viewer
