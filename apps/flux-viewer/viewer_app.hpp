#pragma once

#include "flux/telemetry/state.hpp"
#include "flux/telemetry/trace.hpp"
#include "live_client.hpp"

#include <imgui.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

namespace flux::viewer {

struct LaunchOptions {
  std::string endpoint{"ws://127.0.0.1:8765/telemetry"};
  telemetry::Level level{telemetry::Level::normal};
  double rate_hz{10.0};
  std::filesystem::path replay_path;
  std::filesystem::path record_path;
  bool connect_on_start{true};
};

class ViewerApp {
public:
  explicit ViewerApp(LaunchOptions options);
  ~ViewerApp();

  void tick();
  void draw();
  [[nodiscard]] bool wants_exit() const { return wants_exit_; }

private:
  struct OverlaySettings {
    bool source_board{true};
    bool source_footprints{true};
    bool source_copper{true};
    bool source_zones_keepouts{true};
    bool committed_routes{true};
    bool current_routes{true};
    bool bspline_curves{true};
    bool bspline_controls{false};
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
  void draw_board_canvas();
  void draw_metrics_panel();
  void draw_activity_panel();
  void draw_path_modal();
  void draw_status_bar();
  void advance_replay(double elapsed_seconds);

  [[nodiscard]] telemetry::LiveState& mutable_state();
  [[nodiscard]] const telemetry::LiveState& state() const;

  LaunchOptions options_;
  SourceMode mode_{SourceMode::live};
  LiveClient client_;
  telemetry::LiveState live_state_;
  telemetry::Trace trace_;
  telemetry::Replay replay_;
  telemetry::TraceWriter recorder_;
  OverlaySettings overlays_;
  CanvasView view_;
  std::string endpoint_;
  std::string trace_path_;
  std::string record_path_;
  std::string ui_status_;
  std::string path_modal_title_;
  std::string path_modal_value_;
  bool path_modal_open_{false};
  bool path_modal_save_{false};
  bool replay_playing_{false};
  float replay_speed_{1.0F};
  double replay_accumulator_us_{0.0};
  std::uint64_t replay_clock_us_{0};
  std::uint64_t live_connection_generation_{0};
  std::chrono::steady_clock::time_point last_tick_{std::chrono::steady_clock::now()};
  bool wants_exit_{false};
};

} // namespace flux::viewer
