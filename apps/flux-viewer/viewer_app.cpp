#include "viewer_app.hpp"

#include <misc/cpp/imgui_stdlib.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <limits>
#include <set>
#include <string_view>

namespace flux::viewer {
namespace {

using telemetry::ConflictKind;
using telemetry::EventType;
using telemetry::Point;
using telemetry::RouteGeometry;
using telemetry::RouteState;
using telemetry::ViaState;

constexpr ImU32 color(float red, float green, float blue, float alpha = 1.0F) {
  return IM_COL32(static_cast<int>(red * 255.0F), static_cast<int>(green * 255.0F),
                  static_cast<int>(blue * 255.0F), static_cast<int>(alpha * 255.0F));
}

ImU32 layer_color(std::string_view layer, float alpha = 1.0F) {
  static constexpr std::array<ImVec4, 8> palette{{
      {0.30F, 0.78F, 1.00F, 1.0F},
      {1.00F, 0.48F, 0.27F, 1.0F},
      {0.45F, 0.88F, 0.48F, 1.0F},
      {0.95F, 0.76F, 0.27F, 1.0F},
      {0.74F, 0.55F, 1.00F, 1.0F},
      {0.26F, 0.86F, 0.78F, 1.0F},
      {1.00F, 0.43F, 0.66F, 1.0F},
      {0.72F, 0.78F, 0.86F, 1.0F},
  }};
  std::uint64_t hash = 1469598103934665603ULL;
  for (const char character : layer) {
    hash ^= static_cast<unsigned char>(character);
    hash *= 1099511628211ULL;
  }
  const ImVec4 selected = palette[static_cast<std::size_t>(hash % palette.size())];
  return ImGui::ColorConvertFloat4ToU32(ImVec4(selected.x, selected.y, selected.z, alpha));
}

bool is_milestone(EventType type) {
  switch (type) {
  case EventType::routing_started:
  case EventType::routing_finished:
  case EventType::net_completed:
  case EventType::net_failed:
  case EventType::route_committed:
  case EventType::route_ripped_up:
  case EventType::via_assigned:
  case EventType::pathfinder_iteration_completed:
  case EventType::checkpoint_selected:
  case EventType::best_checkpoint_updated:
  case EventType::placement_proposal_accepted:
  case EventType::placement_proposal_rejected:
    return true;
  default:
    return false;
  }
}

std::uint64_t replay_time_us(const std::vector<telemetry::Event>& events, std::size_t index) {
  if (index >= events.size()) {
    return 0;
  }
  if (events[index].monotonic_time_us != 0U) {
    return events[index].monotonic_time_us;
  }
  constexpr std::uint64_t legacy_event_interval_us = 100'000;
  return static_cast<std::uint64_t>(index) * legacy_event_interval_us;
}

void help_marker(const char* text) {
  ImGui::SameLine();
  ImGui::TextDisabled("(?)");
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("%s", text);
  }
}

void add_dashed_line(ImDrawList* draw, ImVec2 first, ImVec2 second, ImU32 line_color,
                     float thickness, float dash_length = 7.0F) {
  const float dx = second.x - first.x;
  const float dy = second.y - first.y;
  const float length = std::sqrt(dx * dx + dy * dy);
  if (length < 0.1F) {
    return;
  }
  const float unit_x = dx / length;
  const float unit_y = dy / length;
  for (float offset = 0.0F; offset < length; offset += dash_length * 2.0F) {
    const float end = std::min(length, offset + dash_length);
    draw->AddLine(ImVec2(first.x + unit_x * offset, first.y + unit_y * offset),
                  ImVec2(first.x + unit_x * end, first.y + unit_y * end), line_color, thickness);
  }
}

std::vector<Point> tessellate_source_arc(const telemetry::SourcePrimitive& primitive) {
  if (primitive.points.size() != 3U) {
    return primitive.points;
  }
  const auto& first = primitive.points[0];
  const auto& middle = primitive.points[1];
  const auto& last = primitive.points[2];
  const auto ax = static_cast<long double>(first.x_nm);
  const auto ay = static_cast<long double>(first.y_nm);
  const auto bx = static_cast<long double>(middle.x_nm);
  const auto by = static_cast<long double>(middle.y_nm);
  const auto cx = static_cast<long double>(last.x_nm);
  const auto cy = static_cast<long double>(last.y_nm);
  const auto denominator = 2.0L * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
  if (std::abs(denominator) < 0.5L) {
    return primitive.points;
  }
  const auto first_norm = ax * ax + ay * ay;
  const auto middle_norm = bx * bx + by * by;
  const auto last_norm = cx * cx + cy * cy;
  const auto center_x =
      (first_norm * (by - cy) + middle_norm * (cy - ay) + last_norm * (ay - by)) / denominator;
  const auto center_y =
      (first_norm * (cx - bx) + middle_norm * (ax - cx) + last_norm * (bx - ax)) / denominator;
  constexpr long double pi = 3.141592653589793238462643383279502884L;
  constexpr long double full_turn = 2.0L * pi;
  const auto start_angle = std::atan2(ay - center_y, ax - center_x);
  const auto middle_angle = std::atan2(by - center_y, bx - center_x);
  const auto end_angle = std::atan2(cy - center_y, cx - center_x);
  const auto positive_delta = [&](long double start, long double end) {
    auto delta = std::fmod(end - start, full_turn);
    return delta < 0.0L ? delta + full_turn : delta;
  };
  const auto counterclockwise_span = positive_delta(start_angle, end_angle);
  const auto middle_counterclockwise = positive_delta(start_angle, middle_angle);
  const auto span = middle_counterclockwise <= counterclockwise_span + 1.0e-12L
                        ? counterclockwise_span
                        : -positive_delta(end_angle, start_angle);
  const auto radius = std::hypotl(ax - center_x, ay - center_y);
  const auto estimated_segments =
      static_cast<std::size_t>(std::ceil(std::abs(span) * radius / 250'000.0L));
  const auto segments = std::clamp<std::size_t>(estimated_segments, 8U, 128U);
  std::vector<Point> result;
  result.reserve(segments + 1U);
  for (std::size_t index = 0U; index <= segments; ++index) {
    const auto parameter = static_cast<long double>(index) / static_cast<long double>(segments);
    const auto angle = start_angle + span * parameter;
    result.push_back(Point{
        static_cast<std::int64_t>(std::llround(center_x + radius * std::cos(angle))),
        static_cast<std::int64_t>(std::llround(center_y + radius * std::sin(angle))), first.layer});
  }
  return result;
}

std::vector<Point> tessellate_route_quintic_ph(const telemetry::Route& route) {
  constexpr std::size_t degree = 5U;
  constexpr std::size_t controls_per_segment = degree + 1U;
  const auto count = route.control_points.size();
  if (route.geometry != RouteGeometry::quintic_ph || route.spline_degree != degree ||
      count < controls_per_segment || count % controls_per_segment != 0U) {
    return route.points;
  }
  struct Coordinate {
    long double x{0.0L};
    long double y{0.0L};
  };
  const auto evaluate_segment = [&](std::size_t segment, long double parameter) {
    std::array<Coordinate, degree + 1U> work;
    for (std::size_t index = 0U; index <= degree; ++index) {
      const auto& control = route.control_points[segment * controls_per_segment + index];
      work[index] = Coordinate{static_cast<long double>(control.x_nm),
                               static_cast<long double>(control.y_nm)};
    }
    for (std::size_t level = 1U; level <= degree; ++level) {
      for (std::size_t index = 0U; index + level <= degree; ++index) {
        work[index].x = (1.0L - parameter) * work[index].x + parameter * work[index + 1U].x;
        work[index].y = (1.0L - parameter) * work[index].y + parameter * work[index + 1U].y;
      }
    }
    const auto low = static_cast<long double>(std::numeric_limits<std::int64_t>::min());
    const auto high = static_cast<long double>(std::numeric_limits<std::int64_t>::max());
    return Point{static_cast<std::int64_t>(std::llround(std::clamp(work[0].x, low, high))),
                 static_cast<std::int64_t>(std::llround(std::clamp(work[0].y, low, high))),
                 route.control_points.front().layer};
  };

  const auto ph_segments = count / controls_per_segment;
  const auto samples_per_segment = std::clamp<std::size_t>(256U / ph_segments, 16U, 48U);
  std::vector<Point> result;
  result.reserve(ph_segments * samples_per_segment + 1U);
  for (std::size_t segment = 0U; segment < ph_segments; ++segment) {
    for (std::size_t index = segment == 0U ? 0U : 1U; index <= samples_per_segment; ++index) {
      const auto parameter = static_cast<long double>(index) /
                             static_cast<long double>(samples_per_segment);
      const auto point = evaluate_segment(segment, parameter);
      if (result.empty() || result.back() != point) result.push_back(point);
    }
  }
  return result.size() >= 2U ? result : route.points;
}

struct Bounds {
  double minimum_x{std::numeric_limits<double>::max()};
  double minimum_y{std::numeric_limits<double>::max()};
  double maximum_x{std::numeric_limits<double>::lowest()};
  double maximum_y{std::numeric_limits<double>::lowest()};

  void include(const Point& point) {
    minimum_x = std::min(minimum_x, static_cast<double>(point.x_nm));
    minimum_y = std::min(minimum_y, static_cast<double>(point.y_nm));
    maximum_x = std::max(maximum_x, static_cast<double>(point.x_nm));
    maximum_y = std::max(maximum_y, static_cast<double>(point.y_nm));
  }
  [[nodiscard]] bool valid() const { return minimum_x <= maximum_x && minimum_y <= maximum_y; }
};

Bounds state_bounds(const telemetry::LiveState& state) {
  Bounds bounds;
  for (const auto& [id, board] : state.source_boards()) {
    (void)id;
    if (!board) {
      continue;
    }
    for (const auto& primitive : board->primitives) {
      for (const Point& point : primitive.points) {
        bounds.include(point);
      }
      if (primitive.kind == "pad" && !primitive.points.empty()) {
        const auto& center = primitive.points.front();
        const auto extent = std::max(primitive.width_nm, primitive.height_nm);
        const auto radius = static_cast<std::int64_t>(extent / 2U + extent % 2U);
        bounds.include(Point{center.x_nm - radius, center.y_nm - radius, center.layer});
        bounds.include(Point{center.x_nm + radius, center.y_nm + radius, center.layer});
      }
    }
  }
  for (const auto& [id, route] : state.routes()) {
    (void)id;
    for (const Point& point : route.points) {
      bounds.include(point);
    }
    for (const Point& point : route.control_points) {
      bounds.include(point);
    }
  }
  for (const auto& [id, via] : state.vias()) {
    (void)id;
    bounds.include(via.position);
  }
  for (const auto& [id, conflict] : state.conflicts()) {
    (void)id;
    bounds.include(conflict.location);
  }
  for (const auto& [id, resource] : state.resources()) {
    (void)id;
    bounds.include(resource.start);
    bounds.include(resource.end);
  }
  for (const auto& [id, field] : state.hydraulic_fields()) {
    (void)id;
    const auto extent_x = static_cast<std::int64_t>(
        field.cell_pitch_nm * field.stride * field.width);
    const auto extent_y = static_cast<std::int64_t>(
        field.cell_pitch_nm * field.stride * field.height);
    bounds.include(Point{field.origin_x_nm, field.origin_y_nm, field.layer});
    bounds.include(Point{field.origin_x_nm + extent_x, field.origin_y_nm + extent_y, field.layer});
  }
  return bounds;
}

const char* connection_label(ConnectionState state) {
  switch (state) {
  case ConnectionState::connected:
    return "CONNECTED";
  case ConnectionState::connecting:
    return "CONNECTING";
  case ConnectionState::failed:
    return "FAILED";
  case ConnectionState::disconnected:
    return "DISCONNECTED";
  }
  return "UNKNOWN";
}

ImVec4 connection_color(ConnectionState state) {
  switch (state) {
  case ConnectionState::connected:
    return {0.38F, 0.88F, 0.55F, 1.0F};
  case ConnectionState::connecting:
    return {0.95F, 0.75F, 0.28F, 1.0F};
  case ConnectionState::failed:
    return {1.00F, 0.38F, 0.34F, 1.0F};
  case ConnectionState::disconnected:
    return {0.62F, 0.67F, 0.72F, 1.0F};
  }
  return {1.0F, 1.0F, 1.0F, 1.0F};
}

void metric_row(const char* label, std::uint64_t value, const char* suffix = "") {
  ImGui::TableNextRow();
  ImGui::TableSetColumnIndex(0);
  ImGui::TextDisabled("%s", label);
  ImGui::TableSetColumnIndex(1);
  ImGui::Text("%llu%s", static_cast<unsigned long long>(value), suffix);
}

std::string lowercase(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });
  return value;
}

void draw_history_plot(const std::vector<telemetry::IterationMetrics>& history) {
  if (history.empty()) {
    ImGui::TextDisabled("Iteration history arrives at completed Pathfinder passes.");
    return;
  }
  std::vector<float> overflow;
  std::vector<float> unrouted;
  overflow.reserve(history.size());
  unrouted.reserve(history.size());
  for (const telemetry::IterationMetrics& metrics : history) {
    overflow.push_back(static_cast<float>(metrics.sampled_overflow + metrics.corridor_overflow +
                                          metrics.clearance_shadow_conflicts));
    unrouted.push_back(static_cast<float>(metrics.unrouted_nets));
  }
  ImGui::PlotLines("Overflow / conflicts", overflow.data(), static_cast<int>(overflow.size()), 0,
                   nullptr, 0.0F, std::numeric_limits<float>::max(), ImVec2(-1.0F, 58.0F));
  ImGui::PlotLines("Unrouted nets", unrouted.data(), static_cast<int>(unrouted.size()), 0, nullptr,
                   0.0F, std::numeric_limits<float>::max(), ImVec2(-1.0F, 48.0F));
}

} // namespace

ViewerApp::ViewerApp(LaunchOptions options)
    : options_(std::move(options)), endpoint_(options_.endpoint),
      trace_path_(options_.replay_path.string()), record_path_(options_.record_path.string()) {
  if (options_.preferences) {
    overlays_ = options_.preferences->overlays;
    replay_speed_ = options_.preferences->replay_speed;
    activity_panel_height_ = options_.preferences->activity_panel_height;
  }
  if (!record_path_.empty()) {
    start_recording(record_path_);
  }
  if (!trace_path_.empty()) {
    load_trace(trace_path_);
  } else if (options_.connect_on_start) {
    connect_live();
  }
}

ViewerApp::~ViewerApp() {
  disconnect_live();
  stop_recording();
  update_preferences();
}

void ViewerApp::update_preferences() {
  if (!options_.preferences) return;
  options_.preferences->overlays = overlays_;
  options_.preferences->activity_panel_height = activity_panel_height_;
  options_.preferences->replay_speed = replay_speed_;
  options_.preferences->endpoint = endpoint_;
  options_.preferences->telemetry_level = std::string(telemetry::to_string(options_.level));
  options_.preferences->telemetry_rate_hz = options_.rate_hz;
  std::string error;
  if (!save_viewer_preferences(options_.preferences_path, *options_.preferences, error)) {
    ui_status_ = std::move(error);
  }
}

telemetry::LiveState& ViewerApp::mutable_state() {
  return mode_ == SourceMode::replay ? replay_.state() : live_state_;
}

const telemetry::LiveState& ViewerApp::state() const {
  return mode_ == SourceMode::replay ? replay_.state() : live_state_;
}

const telemetry::LiveState& ViewerApp::overlay_state() const {
  return overlay_history_.size() == 0U ? state() : overlay_history_.state();
}

void ViewerApp::sync_overlay_history_to_source() {
  if (!overlay_follow_source_) {
    return;
  }
  const auto source_position =
      mode_ == SourceMode::replay ? replay_.position() : overlay_history_.size();
  overlay_history_.seek(source_position);
}

void ViewerApp::connect_live() {
  mode_ = SourceMode::live;
  replay_playing_ = false;
  live_state_.clear();
  overlay_history_.clear();
  overlay_follow_source_ = true;
  (void)client_.connect(endpoint_, options_.level, options_.rate_hz);
  ui_status_ = "Connecting to live telemetry";
  view_.fit_requested = true;
}

void ViewerApp::disconnect_live() { client_.disconnect(); }

void ViewerApp::load_trace(const std::filesystem::path& path) {
  std::string error;
  telemetry::Trace loaded;
  if (!loaded.load(path, error)) {
    ui_status_ = std::move(error);
    return;
  }
  disconnect_live();
  trace_ = std::move(loaded);
  replay_.set_trace(&trace_);
  overlay_history_.clear();
  for (const auto& event : trace_.events()) {
    overlay_history_.append(event);
  }
  overlay_history_.seek(0U);
  overlay_follow_source_ = true;
  mode_ = SourceMode::replay;
  replay_playing_ = false;
  replay_accumulator_us_ = 0.0;
  replay_clock_us_ = replay_time_us(trace_.events(), 0U);
  trace_path_ = path.string();
  if (options_.preferences) {
    options_.preferences->trace_directory =
        std::filesystem::is_directory(path) ? path : path.parent_path();
  }
  ui_status_ = "Loaded " + std::to_string(trace_.events().size()) + " telemetry events";
  view_.fit_requested = true;
}

void ViewerApp::start_recording(const std::filesystem::path& path) {
  std::string error;
  if (recorder_.open(path, error)) {
    record_path_ = path.string();
    if (options_.preferences) options_.preferences->record_directory = path.parent_path();
    ui_status_ = "Recording live events to " + record_path_;
  } else {
    ui_status_ = std::move(error);
  }
}

void ViewerApp::stop_recording() {
  if (recorder_.is_open()) {
    recorder_.close();
    ui_status_ = "Telemetry recording stopped";
  }
}

void ViewerApp::tick() {
  const auto now = std::chrono::steady_clock::now();
  const double elapsed = std::chrono::duration<double>(now - last_tick_).count();
  last_tick_ = now;

  const ConnectionStatus connection = client_.status();
  if (connection.connection_generation != live_connection_generation_) {
    live_connection_generation_ = connection.connection_generation;
    live_state_.clear();
    if (mode_ == SourceMode::live) {
      overlay_history_.clear();
      overlay_follow_source_ = true;
    }
    view_.fit_requested = true;
    ui_status_ = live_connection_generation_ == 1U ? "Connected to Flux telemetry"
                                                   : "Connected to a new Flux telemetry instance";
  }
  for (telemetry::Event& event : client_.drain()) {
    live_state_.apply(event);
    if (mode_ == SourceMode::live) {
      overlay_history_.append(event);
    }
    if (recorder_.is_open() && !recorder_.append(event)) {
      ui_status_ = recorder_.error();
    }
  }
  if (mode_ == SourceMode::replay && replay_playing_) {
    advance_replay(elapsed);
  }
  sync_overlay_history_to_source();
}

void ViewerApp::advance_replay(double elapsed_seconds) {
  if (replay_.position() >= replay_.size()) {
    replay_playing_ = false;
    return;
  }
  replay_accumulator_us_ += elapsed_seconds * replay_speed_ * 1'000'000.0;
  const double target_time = static_cast<double>(replay_clock_us_) + replay_accumulator_us_;
  while (replay_.position() < trace_.events().size()) {
    const std::size_t next_index = replay_.position();
    if (next_index != 0U &&
        static_cast<double>(replay_time_us(trace_.events(), next_index)) > target_time) {
      break;
    }
    (void)replay_.step();
  }
  if (replay_.position() >= replay_.size()) {
    replay_playing_ = false;
  }
  sync_overlay_history_to_source();
}

void ViewerApp::seek_replay(std::size_t position) {
  replay_playing_ = false;
  replay_.seek(std::min(position, replay_.size()));
  replay_clock_us_ = replay_.position() == 0U
                         ? replay_time_us(trace_.events(), 0U)
                         : replay_time_us(trace_.events(), replay_.position() - 1U);
  replay_accumulator_us_ = 0.0;
  view_.fit_requested = true;
  sync_overlay_history_to_source();
}

void ViewerApp::draw_menu_bar() {
  if (!ImGui::BeginMenuBar()) {
    return;
  }
  if (ImGui::BeginMenu("File")) {
    if (ImGui::MenuItem("Open trace...", "Ctrl+O")) {
      open_file_browser(false);
    }
    if (ImGui::MenuItem("Record live events...", nullptr, false, !recorder_.is_open())) {
      open_file_browser(true);
    }
    if (ImGui::MenuItem("Stop recording", nullptr, false, recorder_.is_open())) {
      stop_recording();
    }
    ImGui::Separator();
    if (ImGui::MenuItem("Exit")) {
      wants_exit_ = true;
    }
    ImGui::EndMenu();
  }
  if (ImGui::BeginMenu("View")) {
    if (ImGui::MenuItem("Fit routing state", "F")) {
      view_.fit_requested = true;
    }
    ImGui::MenuItem("Grid", nullptr, &overlays_.grid);
    ImGui::EndMenu();
  }
  ImGui::TextDisabled("  Flux Router Telemetry Viewer");
  ImGui::EndMenuBar();
}

void ViewerApp::draw_status_bar() {
  const ConnectionStatus connection = client_.status();
  if (mode_ == SourceMode::live) {
    ImGui::TextColored(connection_color(connection.state), "%s",
                       connection_label(connection.state));
    ImGui::SameLine();
    ImGui::TextDisabled("%s", connection.message.c_str());
  } else {
    ImGui::TextColored(ImVec4(0.42F, 0.78F, 1.0F, 1.0F), "REPLAY");
    ImGui::SameLine();
    ImGui::TextDisabled("%s", trace_path_.c_str());
  }
  ImGui::SameLine();
  ImGui::Text("seq %llu  iteration %zu", static_cast<unsigned long long>(state().sequence()),
              state().iteration());
  if (recorder_.is_open()) {
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(1.0F, 0.32F, 0.28F, 1.0F), "REC");
  }
}

void ViewerApp::draw_source_panel() {
  ImGui::TextUnformatted("Source");
  ImGui::Separator();
  if (ImGui::RadioButton("Live", mode_ == SourceMode::live)) {
    connect_live();
  }
  ImGui::SameLine();
  if (ImGui::RadioButton("Replay", mode_ == SourceMode::replay)) {
    mode_ = SourceMode::replay;
  }

  ImGui::SetNextItemWidth(-1.0F);
  ImGui::InputTextWithHint("##endpoint", "ws://127.0.0.1:8765/telemetry", &endpoint_);
  const ConnectionStatus connection = client_.status();
  if (connection.state == ConnectionState::connected ||
      connection.state == ConnectionState::connecting) {
    if (ImGui::Button("Disconnect", ImVec2(-1.0F, 0.0F))) {
      disconnect_live();
    }
  } else if (ImGui::Button("Connect live", ImVec2(-1.0F, 0.0F))) {
    connect_live();
  }

  int level = static_cast<int>(options_.level);
  if (ImGui::Combo("Detail", &level, "Normal milestones\0Diagnostic\0Trace debugging\0")) {
    options_.level = static_cast<telemetry::Level>(level);
  }
  help_marker(
      "The viewer waits for Flux and reconnects automatically across process restarts. Normal "
      "reports routing milestones, not individual A* node expansions. Reconnect after changing "
      "detail so the producer can negotiate the new level.");
  float rate = static_cast<float>(options_.rate_hz);
  if (ImGui::SliderFloat("Rate", &rate, 1.0F, 60.0F, "%.0f Hz")) {
    options_.rate_hz = static_cast<double>(rate);
  }

  if (!state().source_boards().empty()) {
    ImGui::Spacing();
    ImGui::TextUnformatted("KiCad source scene");
    ImGui::Separator();
    for (const auto& [id, board] : state().source_boards()) {
      if (!board) {
        continue;
      }
      ImGui::PushID(id.c_str());
      ImGui::TextWrapped("%s", board->path.empty() ? id.c_str() : board->path.c_str());
      ImGui::TextDisabled("schema %u  %zu layers  %zu nets  %zu objects", board->board_schema,
                          board->layers.size(), board->nets.size(), board->primitives.size());
      if (!board->revision.empty() && ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Source revision: %s", board->revision.c_str());
      }
      ImGui::PopID();
    }
  }

  ImGui::Spacing();
  ImGui::TextUnformatted("Trace");
  ImGui::Separator();
  ImGui::TextWrapped("%s", trace_path_.empty() ? "No trace selected" : trace_path_.c_str());
  if (ImGui::Button("Browse trace...", ImVec2(-1.0F, 0.0F))) {
    open_file_browser(false);
  }
  if (!trace_path_.empty() && ImGui::Button("Reload selected trace", ImVec2(-1.0F, 0.0F))) {
    load_trace(trace_path_);
  }

  if (mode_ == SourceMode::replay && !trace_.empty()) {
    if (ImGui::Button(replay_playing_ ? "Pause" : "Play")) {
      replay_playing_ = !replay_playing_;
      if (replay_playing_ && replay_.position() >= replay_.size()) {
        replay_.reset();
      }
      replay_clock_us_ = replay_.position() == 0U
                             ? replay_time_us(trace_.events(), 0U)
                             : replay_time_us(trace_.events(), replay_.position() - 1U);
      replay_accumulator_us_ = 0.0;
    }
    ImGui::SameLine();
    if (ImGui::Button("Step")) {
      replay_playing_ = false;
      (void)replay_.step();
    }
    ImGui::SameLine();
    if (ImGui::Button("Next milestone")) {
      replay_playing_ = false;
      while (replay_.position() < trace_.events().size()) {
        const bool milestone = is_milestone(trace_.events()[replay_.position()].type);
        (void)replay_.step();
        if (milestone) {
          break;
        }
      }
    }
    const std::uint64_t original_position = replay_.position();
    std::uint64_t position = original_position;
    const std::uint64_t maximum = replay_.size();
    const std::uint64_t minimum = 0;
    bool seek_requested =
        ImGui::SliderScalar("Event", ImGuiDataType_U64, &position, &minimum, &maximum, "%llu");
    if (ImGui::IsItemFocused()) {
      if (ImGui::IsKeyPressed(ImGuiKey_LeftArrow, true)) {
        position = original_position == 0U ? 0U : original_position - 1U;
        seek_requested = true;
      } else if (ImGui::IsKeyPressed(ImGuiKey_RightArrow, true)) {
        position = std::min(maximum, original_position + 1U);
        seek_requested = true;
      }
    }
    if (seek_requested) {
      seek_replay(static_cast<std::size_t>(position));
    }
    ImGui::SliderFloat("Speed", &replay_speed_, 0.25F, 8.0F, "%.2fx");
    ImGui::TextDisabled("%zu / %zu events", replay_.position(), replay_.size());
    sync_overlay_history_to_source();
  }

  ImGui::Spacing();
  ImGui::TextUnformatted("Capture");
  ImGui::Separator();
  ImGui::TextWrapped("%s", record_path_.empty() ? "No recording file selected"
                                                : record_path_.c_str());
  if (!recorder_.is_open()) {
    if (ImGui::Button("Select recording file...", ImVec2(-1.0F, 0.0F))) {
      open_file_browser(true);
    }
  } else if (ImGui::Button("Stop recording", ImVec2(-1.0F, 0.0F))) {
    stop_recording();
  }
}

void ViewerApp::draw_overlay_panel() {
  ImGui::TextUnformatted("Overlays");
  ImGui::Separator();
  if (overlay_history_.size() != 0U) {
    ImGui::TextUnformatted("Overlay history");
    bool follow = overlay_follow_source_;
    if (ImGui::Checkbox("Follow live / replay", &follow)) {
      overlay_follow_source_ = follow;
      sync_overlay_history_to_source();
    }
    const std::uint64_t minimum = 0U;
    const std::uint64_t maximum = overlay_history_.size();
    std::uint64_t position = overlay_history_.position();
    ImGui::SetNextItemWidth(-1.0F);
    if (ImGui::SliderScalar("##overlay-timeline", ImGuiDataType_U64, &position, &minimum, &maximum,
                            "%llu")) {
      overlay_history_.seek(static_cast<std::size_t>(position));
      const auto source_position =
          mode_ == SourceMode::replay ? replay_.position() : overlay_history_.size();
      overlay_follow_source_ = position == source_position;
      view_.fit_requested = true;
    }
    if (position == 0U) {
      ImGui::TextDisabled("Initial state (oldest -> newest)");
    } else if (const auto* event =
                   overlay_history_.event(static_cast<std::size_t>(position - 1U))) {
      ImGui::TextDisabled("seq %llu  %.3fs  %s", static_cast<unsigned long long>(event->sequence),
                          static_cast<double>(event->monotonic_time_us) / 1'000'000.0,
                          telemetry::to_string(event->type).data());
    }
    ImGui::Separator();
  }
  ImGui::Checkbox("Source board outline", &overlays_.source_board);
  ImGui::Checkbox("Source footprints / pads", &overlays_.source_footprints);
  ImGui::Checkbox("Existing source copper", &overlays_.source_copper);
  ImGui::Checkbox("Source zones / keepouts", &overlays_.source_zones_keepouts);
  ImGui::Separator();
  ImGui::Checkbox("Committed routes", &overlays_.committed_routes);
  ImGui::Checkbox("Current / provisional", &overlays_.current_routes);
  ImGui::Checkbox("Retained coarse alternatives", &overlays_.retained_coarse_routes);
  ImGui::Checkbox("River alternatives", &overlays_.river_routes);
  ImGui::Checkbox("RRT-Connect alternatives", &overlays_.rrt_routes);
  ImGui::Checkbox("Hydraulic erosion alternatives", &overlays_.hydraulic_routes);
  ImGui::Checkbox("Hydraulic erosion fields", &overlays_.hydraulic_fields);
  ImGui::Checkbox("Quintic PH curves", &overlays_.ph_curves);
  ImGui::Checkbox("PH Bézier control polygons", &overlays_.ph_controls);
  ImGui::Checkbox("Rip-up / reroute", &overlays_.rip_up_activity);
  ImGui::Checkbox("Congestion conflicts", &overlays_.congestion);
  ImGui::Checkbox("Clearance shadows", &overlays_.clearance_shadows);
  ImGui::Checkbox("Corridor / resource occupancy", &overlays_.occupancy);
  ImGui::Checkbox("Priority scores", &overlays_.priorities);
  ImGui::Checkbox("Via candidates", &overlays_.via_candidates);
  ImGui::Checkbox("Selected vias", &overlays_.selected_vias);
  ImGui::Checkbox("Contested sites", &overlays_.contested_vias);
  ImGui::Checkbox("Rejected via sites", &overlays_.rejected_vias);
  ImGui::Checkbox("Grid", &overlays_.grid);

  const auto& displayed = overlay_state();
  std::set<std::string> layers;
  for (const auto& [id, board] : displayed.source_boards()) {
    (void)id;
    if (!board) {
      continue;
    }
    for (const auto& layer : board->layers) {
      layers.insert(layer.id);
    }
  }
  for (const auto& [id, route] : displayed.routes()) {
    (void)id;
    for (const Point& point : route.points) {
      if (!point.layer.empty())
        layers.insert(point.layer);
    }
    for (const Point& point : route.control_points) {
      if (!point.layer.empty())
        layers.insert(point.layer);
    }
  }
  for (const auto& [id, via] : displayed.vias()) {
    (void)id;
    if (!via.position.layer.empty())
      layers.insert(via.position.layer);
    if (!via.to_layer.empty())
      layers.insert(via.to_layer);
  }
  for (const auto& [id, resource] : displayed.resources()) {
    (void)id;
    if (!resource.layer.empty())
      layers.insert(resource.layer);
  }
  ImGui::SetNextItemWidth(-1.0F);
  if (ImGui::BeginCombo("Layer",
                        overlays_.layer.empty() ? "All layers" : overlays_.layer.c_str())) {
    if (ImGui::Selectable("All layers", overlays_.layer.empty())) {
      overlays_.layer.clear();
    }
    for (const std::string& layer : layers) {
      if (ImGui::Selectable(layer.c_str(), overlays_.layer == layer)) {
        overlays_.layer = layer;
      }
    }
    ImGui::EndCombo();
  }
  if (ImGui::Button("Fit state", ImVec2(-1.0F, 0.0F))) {
    view_.fit_requested = true;
  }
}

void ViewerApp::draw_board_canvas() {
  const telemetry::LiveState& displayed = overlay_state();
  const ImVec2 canvas_origin = ImGui::GetCursorScreenPos();
  ImVec2 canvas_size = ImGui::GetContentRegionAvail();
  canvas_size.x = std::max(canvas_size.x, 100.0F);
  canvas_size.y = std::max(canvas_size.y, 100.0F);
  ImGui::InvisibleButton("routing-canvas", canvas_size,
                         ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
  const bool hovered = ImGui::IsItemHovered();
  ImDrawList* draw = ImGui::GetWindowDrawList();
  const ImVec2 canvas_end(canvas_origin.x + canvas_size.x, canvas_origin.y + canvas_size.y);
  draw->AddRectFilled(canvas_origin, canvas_end, color(0.035F, 0.052F, 0.070F));
  draw->AddRect(canvas_origin, canvas_end, color(0.18F, 0.25F, 0.31F));
  draw->PushClipRect(canvas_origin, canvas_end, true);

  const Bounds bounds = state_bounds(displayed);
  if (view_.fit_requested && bounds.valid()) {
    const double width_mm = std::max(0.1, (bounds.maximum_x - bounds.minimum_x) / 1'000'000.0);
    const double height_mm = std::max(0.1, (bounds.maximum_y - bounds.minimum_y) / 1'000'000.0);
    view_.pixels_per_mm =
        std::clamp(0.88 * std::min(static_cast<double>(canvas_size.x) / width_mm,
                                   static_cast<double>(canvas_size.y) / height_mm),
                   0.02, 2000.0);
    view_.center_x_nm = (bounds.minimum_x + bounds.maximum_x) * 0.5;
    view_.center_y_nm = (bounds.minimum_y + bounds.maximum_y) * 0.5;
    view_.pan = {};
    view_.fit_requested = false;
  }

  if (hovered && ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
    const ImVec2 delta = ImGui::GetIO().MouseDelta;
    view_.pan.x += delta.x;
    view_.pan.y += delta.y;
  }
  if (hovered && ImGui::GetIO().MouseWheel != 0.0F) {
    const double old_scale = view_.pixels_per_mm;
    const double factor = std::pow(1.18, static_cast<double>(ImGui::GetIO().MouseWheel));
    view_.pixels_per_mm = std::clamp(old_scale * factor, 0.02, 2000.0);
    const ImVec2 mouse = ImGui::GetIO().MousePos;
    const ImVec2 center(canvas_origin.x + canvas_size.x * 0.5F + view_.pan.x,
                        canvas_origin.y + canvas_size.y * 0.5F + view_.pan.y);
    const float ratio = static_cast<float>(view_.pixels_per_mm / old_scale);
    view_.pan.x -= (mouse.x - center.x) * (ratio - 1.0F);
    view_.pan.y -= (mouse.y - center.y) * (ratio - 1.0F);
  }

  const auto screen = [&](const Point& point) {
    const double x_mm = (static_cast<double>(point.x_nm) - view_.center_x_nm) / 1'000'000.0;
    const double y_mm = (static_cast<double>(point.y_nm) - view_.center_y_nm) / 1'000'000.0;
    return ImVec2(canvas_origin.x + canvas_size.x * 0.5F + view_.pan.x +
                      static_cast<float>(x_mm * view_.pixels_per_mm),
                  canvas_origin.y + canvas_size.y * 0.5F + view_.pan.y -
                      static_cast<float>(y_mm * view_.pixels_per_mm));
  };
  const auto layer_visible = [&](const std::string& layer) {
    return overlays_.layer.empty() || layer == overlays_.layer;
  };
  const auto primitive_layer_visible = [&](const telemetry::SourcePrimitive& primitive) {
    return overlays_.layer.empty() || primitive.layers.empty() ||
           std::find(primitive.layers.begin(), primitive.layers.end(), overlays_.layer) !=
               primitive.layers.end();
  };
  const auto draw_closed_points = [&](const std::vector<Point>& points, ImU32 line_color,
                                      float width) {
    if (points.size() < 2U) {
      return;
    }
    std::vector<ImVec2> vertices;
    vertices.reserve(points.size());
    for (const auto& point : points) {
      vertices.push_back(screen(point));
    }
    draw->AddPolyline(vertices.data(), static_cast<int>(vertices.size()), line_color,
                      ImDrawFlags_Closed, width);
  };
  const auto rotated_pad_point = [](const Point& center, long double local_x, long double local_y,
                                    std::int64_t microdegrees) {
    constexpr long double pi = 3.141592653589793238462643383279502884L;
    const auto angle = static_cast<long double>(microdegrees) * pi / 180'000'000.0L;
    const auto cosine = std::cos(angle);
    const auto sine = std::sin(angle);
    return Point{static_cast<std::int64_t>(std::llround(static_cast<long double>(center.x_nm) +
                                                        local_x * cosine + local_y * sine)),
                 static_cast<std::int64_t>(std::llround(static_cast<long double>(center.y_nm) -
                                                        local_x * sine + local_y * cosine)),
                 center.layer};
  };
  const auto draw_source_pad = [&](const telemetry::SourcePrimitive& primitive) {
    if (primitive.points.empty()) {
      return;
    }
    const auto& center = primitive.points.front();
    const auto pad_color =
        layer_color(primitive.layers.empty() ? std::string_view{"pad"}
                                             : std::string_view{primitive.layers.front()},
                    0.46F);
    const auto outline_color =
        layer_color(primitive.layers.empty() ? std::string_view{"pad"}
                                             : std::string_view{primitive.layers.front()},
                    0.90F);
    const auto half_width = static_cast<long double>(primitive.width_nm) / 2.0L;
    const auto half_height = static_cast<long double>(primitive.height_nm) / 2.0L;
    if (primitive.shape == "circle" || primitive.width_nm == primitive.height_nm) {
      const auto radius = static_cast<float>(std::max(1.5, static_cast<double>(primitive.width_nm) /
                                                               2'000'000.0 * view_.pixels_per_mm));
      draw->AddCircleFilled(screen(center), radius, pad_color, 24);
      draw->AddCircle(screen(center), radius, outline_color, 24, 1.0F);
    } else if (primitive.shape == "oval") {
      const auto horizontal = primitive.width_nm >= primitive.height_nm;
      const auto radius_nm = horizontal ? half_height : half_width;
      const auto run_nm = (horizontal ? half_width : half_height) - radius_nm;
      const auto first =
          rotated_pad_point(center, horizontal ? -run_nm : 0.0L, horizontal ? 0.0L : -run_nm,
                            primitive.rotation_microdegrees);
      const auto second =
          rotated_pad_point(center, horizontal ? run_nm : 0.0L, horizontal ? 0.0L : run_nm,
                            primitive.rotation_microdegrees);
      const auto thickness = static_cast<float>(
          std::max(3.0, static_cast<double>(radius_nm * 2.0L) / 1'000'000.0 * view_.pixels_per_mm));
      draw->AddLine(screen(first), screen(second), pad_color, thickness);
      draw->AddCircle(screen(first), thickness / 2.0F, outline_color, 20, 1.0F);
      draw->AddCircle(screen(second), thickness / 2.0F, outline_color, 20, 1.0F);
    } else {
      std::array<Point, 4> corners{
          rotated_pad_point(center, -half_width, -half_height, primitive.rotation_microdegrees),
          rotated_pad_point(center, half_width, -half_height, primitive.rotation_microdegrees),
          rotated_pad_point(center, half_width, half_height, primitive.rotation_microdegrees),
          rotated_pad_point(center, -half_width, half_height, primitive.rotation_microdegrees)};
      std::array<ImVec2, 4> pixels;
      std::transform(corners.begin(), corners.end(), pixels.begin(), screen);
      draw->AddConvexPolyFilled(pixels.data(), static_cast<int>(pixels.size()), pad_color);
      draw->AddPolyline(pixels.data(), static_cast<int>(pixels.size()), outline_color,
                        ImDrawFlags_Closed, 1.0F);
    }
    const auto drill_width =
        primitive.drill_width_nm > 0U ? primitive.drill_width_nm : primitive.drill_nm;
    const auto drill_height =
        primitive.drill_height_nm > 0U ? primitive.drill_height_nm : primitive.drill_nm;
    if (drill_width > 0U && drill_height > 0U) {
      const auto drill_center = rotated_pad_point(
          center, static_cast<long double>(primitive.drill_offset_x_nm),
          static_cast<long double>(primitive.drill_offset_y_nm), primitive.rotation_microdegrees);
      const auto horizontal = drill_width >= drill_height;
      const auto radius_nm = static_cast<long double>(horizontal ? drill_height : drill_width) / 2.0L;
      const auto run_nm =
          static_cast<long double>(horizontal ? drill_width : drill_height) / 2.0L - radius_nm;
      const auto first = rotated_pad_point(
          drill_center, horizontal ? -run_nm : 0.0L, horizontal ? 0.0L : -run_nm,
          primitive.rotation_microdegrees);
      const auto second = rotated_pad_point(
          drill_center, horizontal ? run_nm : 0.0L, horizontal ? 0.0L : run_nm,
          primitive.rotation_microdegrees);
      const auto thickness = static_cast<float>(std::max(
          2.0, static_cast<double>(radius_nm * 2.0L) / 1'000'000.0 * view_.pixels_per_mm));
      const auto hole_color = color(0.035F, 0.052F, 0.070F);
      const auto drill_outline = color(0.68F, 0.72F, 0.76F);
      draw->AddLine(screen(first), screen(second), hole_color, thickness);
      draw->AddCircleFilled(screen(first), thickness / 2.0F, hole_color, 20);
      draw->AddCircleFilled(screen(second), thickness / 2.0F, hole_color, 20);
      draw->AddCircle(screen(first), thickness / 2.0F, drill_outline, 20, 1.0F);
      draw->AddCircle(screen(second), thickness / 2.0F, drill_outline, 20, 1.0F);
    }
  };

  if (overlays_.grid) {
    double grid_mm = 0.01;
    while (grid_mm * view_.pixels_per_mm < 34.0) {
      grid_mm *= 10.0;
    }
    const float spacing = static_cast<float>(grid_mm * view_.pixels_per_mm);
    const float center_x = canvas_origin.x + canvas_size.x * 0.5F + view_.pan.x;
    const float center_y = canvas_origin.y + canvas_size.y * 0.5F + view_.pan.y;
    const float first_x = canvas_origin.x + std::fmod(center_x - canvas_origin.x, spacing);
    const float first_y = canvas_origin.y + std::fmod(center_y - canvas_origin.y, spacing);
    for (float x = first_x; x < canvas_end.x; x += spacing) {
      draw->AddLine(ImVec2(x, canvas_origin.y), ImVec2(x, canvas_end.y),
                    color(0.17F, 0.22F, 0.27F, 0.55F));
    }
    for (float y = first_y; y < canvas_end.y; y += spacing) {
      draw->AddLine(ImVec2(canvas_origin.x, y), ImVec2(canvas_end.x, y),
                    color(0.17F, 0.22F, 0.27F, 0.55F));
    }
  }

  for (const auto& [board_id, board] : displayed.source_boards()) {
    (void)board_id;
    if (!board) {
      continue;
    }
    for (const auto& primitive : board->primitives) {
      if (!primitive_layer_visible(primitive)) {
        continue;
      }
      if (overlays_.source_board && primitive.kind == "board_outline") {
        draw_closed_points(primitive.points, color(0.72F, 0.78F, 0.82F, 0.95F), 2.2F);
      } else if (overlays_.source_board && primitive.kind == "board_cutout") {
        draw_closed_points(primitive.points, color(0.95F, 0.52F, 0.34F, 0.90F), 1.8F);
      } else if (overlays_.source_zones_keepouts && primitive.kind == "zone") {
        draw_closed_points(
            primitive.points,
            layer_color(primitive.layers.empty() ? "zone" : primitive.layers.front(), 0.32F), 1.2F);
        for (const auto& hole : primitive.holes) {
          draw_closed_points(hole, color(0.95F, 0.65F, 0.20F, 0.55F), 1.0F);
        }
      } else if (overlays_.source_zones_keepouts && primitive.kind == "keepout") {
        draw_closed_points(primitive.points, color(0.96F, 0.26F, 0.72F, 0.68F), 1.6F);
      } else if (overlays_.source_footprints && primitive.kind == "footprint") {
        draw_closed_points(primitive.points, color(0.86F, 0.84F, 0.58F, 0.82F), 1.5F);
        if (!primitive.reference.empty() && !primitive.points.empty() &&
            view_.pixels_per_mm >= 7.0) {
          draw->AddText(screen(primitive.points.front()), color(0.92F, 0.90F, 0.68F, 0.86F),
                        primitive.reference.c_str());
        }
      } else if (overlays_.source_footprints && primitive.kind == "footprint_courtyard_circle" &&
                 primitive.points.size() == 2U) {
        const auto dx = static_cast<double>(primitive.points[1].x_nm - primitive.points[0].x_nm);
        const auto dy = static_cast<double>(primitive.points[1].y_nm - primitive.points[0].y_nm);
        const auto radius =
            static_cast<float>(std::hypot(dx, dy) / 1'000'000.0 * view_.pixels_per_mm);
        draw->AddCircle(screen(primitive.points[0]), radius, color(0.86F, 0.84F, 0.58F, 0.82F), 32,
                        1.5F);
      } else if (overlays_.source_footprints && primitive.kind == "pad") {
        draw_source_pad(primitive);
      } else if (overlays_.source_copper && primitive.kind == "track" &&
                 primitive.points.size() >= 2U) {
        const auto width = static_cast<float>(std::max(
            1.0, static_cast<double>(primitive.width_nm) / 1'000'000.0 * view_.pixels_per_mm));
        draw->AddLine(
            screen(primitive.points[0]), screen(primitive.points[1]),
            layer_color(primitive.layers.empty() ? "track" : primitive.layers.front(), 0.64F),
            width);
      } else if (overlays_.source_copper && primitive.kind == "track_arc" &&
                 primitive.points.size() == 3U) {
        const auto width = static_cast<float>(std::max(
            1.0, static_cast<double>(primitive.width_nm) / 1'000'000.0 * view_.pixels_per_mm));
        const auto arc = tessellate_source_arc(primitive);
        std::vector<ImVec2> pixels;
        pixels.reserve(arc.size());
        for (const auto& point : arc) {
          pixels.push_back(screen(point));
        }
        draw->AddPolyline(
            pixels.data(), static_cast<int>(pixels.size()),
            layer_color(primitive.layers.empty() ? "arc" : primitive.layers.front(), 0.64F),
            ImDrawFlags_None, width);
      } else if (overlays_.source_copper && primitive.kind == "via" && !primitive.points.empty()) {
        const auto radius = static_cast<float>(std::max(
            2.0, static_cast<double>(primitive.diameter_nm) / 2'000'000.0 * view_.pixels_per_mm));
        draw->AddCircleFilled(screen(primitive.points.front()), radius,
                              color(0.76F, 0.72F, 0.60F, 0.65F), 20);
        const auto drill = static_cast<float>(std::max(
            1.0, static_cast<double>(primitive.drill_nm) / 2'000'000.0 * view_.pixels_per_mm));
        draw->AddCircleFilled(screen(primitive.points.front()), drill,
                              color(0.035F, 0.052F, 0.070F), 20);
      }
    }
  }

  if (overlays_.occupancy) {
    for (const auto& [id, resource] : displayed.resources()) {
      (void)id;
      if (!layer_visible(resource.layer)) {
        continue;
      }
      const double utilization = resource.utilization();
      const float width = static_cast<float>(std::clamp(
          static_cast<double>(resource.channel_width_nm) / 1'000'000.0 * view_.pixels_per_mm, 4.0,
          30.0));
      const ImU32 resource_color = utilization > 1.0   ? color(1.0F, 0.20F, 0.12F, 0.30F)
                                   : utilization > 0.8 ? color(1.0F, 0.70F, 0.18F, 0.24F)
                                                       : color(0.24F, 0.62F, 0.82F, 0.16F);
      draw->AddLine(screen(resource.start), screen(resource.end), resource_color, width);
    }
  }

  if (overlays_.hydraulic_fields) {
    for (const auto& [id, field] : displayed.hydraulic_fields()) {
      (void)id;
      if (!layer_visible(field.layer) || field.width == 0U || field.height == 0U ||
          field.stride == 0U || field.cell_pitch_nm == 0U ||
          field.erosion.size() != field.width * field.height ||
          field.flow.size() != field.erosion.size()) {
        continue;
      }
      const bool has_heightmap = field.heightmap.size() == field.erosion.size() &&
                                 field.free_space.size() == field.erosion.size();
      const bool has_feedback =
          field.downstream_pressure.size() == field.erosion.size() &&
          field.success_prior.size() == field.erosion.size();
      const auto cell_nm = static_cast<std::int64_t>(field.cell_pitch_nm * field.stride);
      for (std::size_t y = 0U; y < field.height; ++y) {
        for (std::size_t x = 0U; x < field.width; ++x) {
          const auto index = y * field.width + x;
          const float erosion = static_cast<float>(field.erosion[index]) / 65'535.0F;
          const float flow = static_cast<float>(field.flow[index]) / 65'535.0F;
          const float terrain = has_heightmap
                                    ? static_cast<float>(field.heightmap[index]) / 65'535.0F
                                    : 0.0F;
          const float free_space = has_heightmap
                                       ? static_cast<float>(field.free_space[index]) / 65'535.0F
                                       : (std::max(erosion, flow) > 0.0F ? 1.0F : 0.0F);
          const float pressure = has_feedback
                                     ? static_cast<float>(field.downstream_pressure[index]) /
                                           65'535.0F
                                     : 0.0F;
          const float success = has_feedback
                                    ? static_cast<float>(field.success_prior[index]) / 65'535.0F
                                    : 0.0F;
          const float intensity = std::max(erosion, flow * 0.42F);
          if (free_space <= 0.0F) continue;
          Point first{field.origin_x_nm + static_cast<std::int64_t>(x) * cell_nm,
                      field.origin_y_nm + static_cast<std::int64_t>(y) * cell_nm, field.layer};
          Point last{first.x_nm + cell_nm, first.y_nm + cell_nm, field.layer};
          const auto first_screen = screen(first);
          const auto last_screen = screen(last);
          // Low terrain is the target valley (blue/green); high terrain is uphill (amber).
          // Flow and erosion brighten the carved riverbed without hiding the complete domain.
          // Downstream failures tint the deposited terrain magenta/red. Repeatedly clean copper
          // contributes a bounded green success prior, making learned valleys directly visible.
          const float red = std::clamp(0.05F + terrain * 0.90F + intensity * 0.20F +
                                           pressure * 0.70F,
                                       0.0F, 1.0F);
          const float green = std::clamp(0.42F + terrain * 0.28F + intensity * 0.25F +
                                             success * 0.45F - pressure * 0.22F,
                                         0.0F, 1.0F);
          const float blue = std::clamp(0.88F - terrain * 0.72F + intensity * 0.08F +
                                            pressure * 0.32F,
                                        0.0F, 1.0F);
          const float alpha =
              free_space * (0.09F + intensity * 0.29F + pressure * 0.24F + success * 0.12F);
          draw->AddRectFilled(ImVec2(std::min(first_screen.x, last_screen.x),
                                    std::min(first_screen.y, last_screen.y)),
                              ImVec2(std::max(first_screen.x, last_screen.x),
                                    std::max(first_screen.y, last_screen.y)),
                              color(red, green, blue, alpha));
        }
      }
    }
  }

  for (const auto& [id, conflict] : displayed.conflicts()) {
    (void)id;
    if (!layer_visible(conflict.location.layer)) {
      continue;
    }
    const bool clearance = conflict.kind == ConflictKind::clearance_shadow;
    if ((clearance && !overlays_.clearance_shadows) || (!clearance && !overlays_.congestion)) {
      continue;
    }
    const ImVec2 position = screen(conflict.location);
    const float radius =
        static_cast<float>(std::clamp(std::abs(static_cast<double>(conflict.clearance_deficit_nm)) /
                                          1'000'000.0 * view_.pixels_per_mm,
                                      7.0, 34.0));
    const ImU32 fill =
        clearance ? color(1.0F, 0.35F, 0.22F, 0.24F) : color(1.0F, 0.12F, 0.18F, 0.34F);
    draw->AddCircleFilled(position, radius, fill, 20);
    draw->AddCircle(position, radius,
                    clearance ? color(1.0F, 0.48F, 0.28F) : color(1.0F, 0.18F, 0.22F), 20, 2.0F);
  }

  for (const auto& [id, route] : displayed.routes()) {
    (void)id;
    const bool committed = route.state == RouteState::committed;
    const bool active =
        route.state == RouteState::current || route.state == RouteState::provisional;
    const bool ripped = route.state == RouteState::ripped_up || route.state == RouteState::removed;
    const bool retained = route.lifecycle == "retained_fallback";
    if (retained &&
        (!overlays_.retained_coarse_routes ||
         (route.family == "river" && !overlays_.river_routes) ||
         (route.family == "rrt_connect" && !overlays_.rrt_routes) ||
         (route.family == "hydraulic_erosion" && !overlays_.hydraulic_routes))) {
      continue;
    }
    if ((committed && !overlays_.committed_routes) ||
        (active && !retained && !overlays_.current_routes) ||
        (ripped && !overlays_.rip_up_activity)) {
      continue;
    }
    const bool highlighted = displayed.active_net() && route.net == *displayed.active_net();
    const bool quintic_ph = route.geometry == RouteGeometry::quintic_ph &&
                               route.spline_degree == 5U && route.control_points.size() >= 6U;
    const auto spline_points = quintic_ph && overlays_.ph_curves
                                   ? tessellate_route_quintic_ph(route)
                                   : std::vector<Point>{};
    const auto& display_points = spline_points.empty() ? route.points : spline_points;
    for (std::size_t index = 1; index < display_points.size(); ++index) {
      const Point& first = display_points[index - 1U];
      const Point& second = display_points[index];
      if (!layer_visible(first.layer)) {
        continue;
      }
      const float physical_width = static_cast<float>(static_cast<double>(route.width_nm) /
                                                      1'000'000.0 * view_.pixels_per_mm);
      const float width = std::clamp(physical_width, highlighted ? 3.5F : 2.0F, 12.0F);
      ImU32 route_color = layer_color(first.layer, ripped ? 0.35F : 0.92F);
      if (retained && route.family == "rrt_connect") {
        route_color = color(1.00F, 0.62F, 0.20F, 0.62F);
      } else if (retained && route.family == "hydraulic_erosion") {
        route_color = color(0.20F, 0.95F, 0.48F, 0.70F);
      } else if (retained && route.family == "river") {
        route_color = color(0.20F, 0.72F, 1.00F, 0.68F);
      } else if (retained) {
        route_color = color(0.58F, 0.64F, 0.78F, 0.48F);
      } else if (active) {
        route_color = quintic_ph && overlays_.ph_curves ? color(0.88F, 0.44F, 1.00F)
                      : route.state == RouteState::provisional  ? color(0.20F, 0.94F, 0.92F)
                                                                : color(1.0F, 0.86F, 0.25F);
      } else if (ripped) {
        route_color = color(1.0F, 0.24F, 0.22F, 0.55F);
      }
      if ((route.state == RouteState::provisional || ripped) &&
          !(quintic_ph && overlays_.ph_curves)) {
        add_dashed_line(draw, screen(first), screen(second), route_color, width);
      } else {
        draw->AddLine(screen(first), screen(second), route_color, width);
      }
    }
    if (quintic_ph && overlays_.ph_controls && !route.control_points.empty()) {
      const auto control_color = color(0.96F, 0.72F, 1.00F, 0.48F);
      for (std::size_t index = 1U; index < route.control_points.size(); ++index) {
        if (index % 6U == 0U) continue;
        if (layer_visible(route.control_points[index - 1U].layer)) {
          add_dashed_line(draw, screen(route.control_points[index - 1U]),
                          screen(route.control_points[index]), control_color, 1.0F, 4.0F);
        }
      }
      for (const auto& control : route.control_points) {
        if (layer_visible(control.layer)) {
          draw->AddCircleFilled(screen(control), 2.8F, color(0.98F, 0.80F, 1.00F), 10);
        }
      }
    }
    if (overlays_.priorities && !route.points.empty()) {
      if (const auto priority = displayed.priorities().find(route.net);
          priority != displayed.priorities().end()) {
        const ImVec2 position = screen(route.points.front());
        char label[96];
        std::snprintf(label, sizeof(label), "%s  %.2f  %s", route.net.c_str(),
                      priority->second.final_priority, priority->second.staged_class.c_str());
        draw->AddText(ImVec2(position.x + 6.0F, position.y - 18.0F), color(0.95F, 0.95F, 0.82F),
                      label);
      }
    }
  }

  for (const auto& [id, via] : displayed.vias()) {
    (void)id;
    if (!layer_visible(via.position.layer)) {
      continue;
    }
    const bool show = (via.state == ViaState::candidate && overlays_.via_candidates) ||
                      (via.state == ViaState::selected && overlays_.selected_vias) ||
                      (via.state == ViaState::contested && overlays_.contested_vias) ||
                      (via.state == ViaState::rejected && overlays_.rejected_vias);
    if (!show || via.state == ViaState::removed) {
      continue;
    }
    const ImVec2 position = screen(via.position);
    const float radius = static_cast<float>(std::clamp(
        static_cast<double>(via.diameter_nm) / 2'000'000.0 * view_.pixels_per_mm, 4.0, 12.0));
    if (via.state == ViaState::selected) {
      draw->AddCircleFilled(position, radius, layer_color(via.position.layer));
      draw->AddCircle(position, radius + 2.0F, layer_color(via.to_layer), 18, 2.0F);
    } else if (via.state == ViaState::contested) {
      draw->AddCircle(position, radius + 3.0F, color(1.0F, 0.24F, 0.82F), 18, 3.0F);
      draw->AddCircle(position, radius, color(0.95F, 0.78F, 0.24F), 18, 1.5F);
    } else if (via.state == ViaState::rejected) {
      draw->AddCircle(position, radius, color(0.95F, 0.26F, 0.22F), 16, 1.5F);
      draw->AddLine(ImVec2(position.x - radius, position.y - radius),
                    ImVec2(position.x + radius, position.y + radius), color(0.95F, 0.26F, 0.22F),
                    1.5F);
      draw->AddLine(ImVec2(position.x - radius, position.y + radius),
                    ImVec2(position.x + radius, position.y - radius), color(0.95F, 0.26F, 0.22F),
                    1.5F);
    } else {
      draw->AddCircle(position, radius, color(0.60F, 0.78F, 0.92F, 0.80F), 16, 1.5F);
    }
  }

  if (!bounds.valid()) {
    const char* text = "Waiting for a telemetry snapshot or route event";
    const ImVec2 size = ImGui::CalcTextSize(text);
    draw->AddText(ImVec2(canvas_origin.x + (canvas_size.x - size.x) * 0.5F,
                         canvas_origin.y + (canvas_size.y - size.y) * 0.5F),
                  color(0.55F, 0.64F, 0.70F), text);
  }

  draw->PopClipRect();
  draw->AddText(ImVec2(canvas_origin.x + 10.0F, canvas_origin.y + 8.0F), color(0.67F, 0.74F, 0.79F),
                "Right-drag: pan   Wheel: zoom   Ctrl+wheel: UI scale   F: fit");
}

void ViewerApp::draw_metrics_panel() {
  ImGui::TextUnformatted("Pathfinder / routing metrics");
  ImGui::Separator();
  const telemetry::LiveState& current = state();
  if (current.metrics()) {
    const telemetry::IterationMetrics& metrics = *current.metrics();
    if (ImGui::BeginTable("routing-metrics-grid", 2,
                          ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH |
                              ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_SizingStretchProp)) {
      ImGui::TableSetupColumn("Metric", ImGuiTableColumnFlags_WidthStretch, 1.3F);
      ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthStretch, 1.0F);
      metric_row("Iteration", metrics.iteration);
      metric_row("Nets routed", metrics.routed_nets);
      metric_row("Nets unrouted", metrics.unrouted_nets);
      metric_row("Nets total", metrics.total_nets);
      metric_row("Sampled overflow", metrics.sampled_overflow);
      metric_row("Corridor overflow", metrics.corridor_overflow);
      metric_row("Physical overflow", metrics.physical_overflow_nm, " nm");
      metric_row("Clearance shadows", metrics.clearance_shadow_conflicts);
      metric_row("Regional conflicts", metrics.regional_conflicts);
      metric_row("Route cost", metrics.route_cost);
      metric_row("Search expansions", metrics.search_expansions);
      metric_row("Vias", metrics.vias);
      metric_row("Rip-ups", metrics.rip_ups);
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::TextDisabled("Checkpoint");
      ImGui::TableSetColumnIndex(1);
      ImGui::Text("%llu / best %llu%s",
                  static_cast<unsigned long long>(metrics.selected_checkpoint),
                  static_cast<unsigned long long>(metrics.best_checkpoint),
                  metrics.selected_is_best ? " (selected)" : "");
      ImGui::EndTable();
    }
  } else {
    ImGui::TextDisabled("No iteration summary received.");
  }

  draw_history_plot(current.iteration_history());

  if (ImGui::CollapsingHeader("Routing priority", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::BeginTable("priority-table", 4,
                          ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                              ImGuiTableFlags_ScrollY,
                          ImVec2(0.0F, 150.0F))) {
      ImGui::TableSetupColumn("Net");
      ImGui::TableSetupColumn("Class");
      ImGui::TableSetupColumn("Score");
      ImGui::TableSetupColumn("Via");
      ImGui::TableHeadersRow();
      for (const auto& [net, priority] : current.priorities()) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted(net.c_str());
        ImGui::TableSetColumnIndex(1);
        ImGui::TextUnformatted(priority.staged_class.c_str());
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%.2f", priority.final_priority);
        ImGui::TableSetColumnIndex(3);
        ImGui::Text("%.2f", priority.via_scarcity);
        if (ImGui::IsItemHovered()) {
          ImGui::SetTooltip("explicit %.2f\nterminal %.2f\ncorridor %.2f\ninterference %.2f\n"
                            "width %.2f\ntopologies %zu\nlayers %zu\ntie %llu",
                            priority.explicit_priority, priority.terminal_scarcity,
                            priority.corridor_scarcity, priority.routing_interference,
                            priority.physical_width_demand, priority.topology_alternatives,
                            priority.layer_alternatives,
                            static_cast<unsigned long long>(priority.stable_tie_break));
        }
      }
      ImGui::EndTable();
    }
  }

  if (ImGui::CollapsingHeader("Via sites", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::BeginTable("via-table", 4,
                          ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                              ImGuiTableFlags_ScrollY,
                          ImVec2(0.0F, 145.0F))) {
      ImGui::TableSetupColumn("ID");
      ImGui::TableSetupColumn("Net");
      ImGui::TableSetupColumn("State");
      ImGui::TableSetupColumn("Score");
      ImGui::TableHeadersRow();
      for (const auto& [id, via] : current.vias()) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted(id.c_str());
        ImGui::TableSetColumnIndex(1);
        ImGui::TextUnformatted(via.net.c_str());
        ImGui::TableSetColumnIndex(2);
        ImGui::TextUnformatted(telemetry::to_string(via.state).data());
        ImGui::TableSetColumnIndex(3);
        ImGui::Text("%.2f", via.score);
        if (ImGui::IsItemHovered()) {
          ImGui::SetTooltip(
              "margin %lld nm\ncompeting nets %zu\ncongestion %.2f\nalternatives %zu\n%s",
              static_cast<long long>(via.clearance_margin_nm), via.competing_nets,
              via.local_congestion, via.remaining_alternatives, via.rejection_reason.c_str());
        }
      }
      ImGui::EndTable();
    }
  }

  if (ImGui::CollapsingHeader("Telemetry integrity")) {
    const telemetry::Counters& counters = current.counters();
    if (ImGui::BeginTable("telemetry-metrics-grid", 2,
                          ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH |
                              ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_SizingStretchProp)) {
      ImGui::TableSetupColumn("Metric", ImGuiTableColumnFlags_WidthStretch, 1.3F);
      ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthStretch, 1.0F);
      metric_row("Produced", counters.produced);
      metric_row("Dropped", counters.dropped);
      metric_row("Coalesced", counters.coalesced);
      metric_row("Serialized", counters.serialized);
      metric_row("Snapshots", counters.snapshots);
      metric_row("Observed gaps", current.observed_sequence_gaps());
      const ConnectionStatus connection = client_.status();
      metric_row("Viewer drops", connection.viewer_dropped_events);
      ImGui::EndTable();
    }
  }
}

void ViewerApp::draw_activity_panel() {
  ImGui::TextUnformatted("Routing activity");
  ImGui::SameLine();
  ImGui::TextDisabled("semantic milestones and bounded debug summaries");
  ImGui::Separator();
  if (ImGui::BeginTable("activity-table", 4,
                        ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                            ImGuiTableFlags_ScrollY)) {
    ImGui::TableSetupColumn("Seq", ImGuiTableColumnFlags_WidthFixed, 72.0F);
    ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed, 88.0F);
    ImGui::TableSetupColumn("Event", ImGuiTableColumnFlags_WidthFixed, 190.0F);
    ImGui::TableSetupColumn("Detail", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableHeadersRow();
    for (auto activity_it = state().activity().crbegin(); activity_it != state().activity().crend();
         ++activity_it) {
      const telemetry::Activity& activity = *activity_it;
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::Text("%llu", static_cast<unsigned long long>(activity.sequence));
      ImGui::TableSetColumnIndex(1);
      ImGui::Text("%.3f", static_cast<double>(activity.monotonic_time_us) / 1'000'000.0);
      ImGui::TableSetColumnIndex(2);
      ImGui::TextUnformatted(telemetry::to_string(activity.type).data());
      ImGui::TableSetColumnIndex(3);
      ImGui::TextUnformatted(activity.message.c_str());
    }
    ImGui::EndTable();
  }
}

void ViewerApp::open_file_browser(bool save_mode) {
  file_browser_ = FileBrowserState{};
  file_browser_.save_mode = save_mode;
  file_browser_.title = save_mode ? "Record telemetry trace" : "Open telemetry trace";
  std::filesystem::path initial;
  if (options_.preferences) {
    initial = save_mode ? options_.preferences->record_directory
                        : options_.preferences->trace_directory;
  }
  const auto& current = save_mode ? record_path_ : trace_path_;
  std::error_code error;
  if (initial.empty() && !current.empty()) {
    const std::filesystem::path current_path{current};
    initial = std::filesystem::is_directory(current_path, error) ? current_path
                                                                 : current_path.parent_path();
    error.clear();
  }
  if (initial.empty()) initial = std::filesystem::current_path(error);
  if (initial.empty() || error) initial = std::filesystem::temp_directory_path(error);
  file_browser_.directory = initial;
  file_browser_.directory_input = initial.string();
  if (save_mode) {
    file_browser_.save_name = std::filesystem::path{record_path_}.filename().string();
    if (file_browser_.save_name.empty()) file_browser_.save_name = "flux-telemetry.jsonl";
  }
  file_browser_.open = true;
  file_browser_.open_requested = true;
  refresh_file_browser();
}

void ViewerApp::refresh_file_browser() {
  file_browser_.entries.clear();
  file_browser_.selected.clear();
  std::error_code error;
  auto directory = std::filesystem::absolute(file_browser_.directory, error);
  if (error || !std::filesystem::is_directory(directory, error)) {
    file_browser_.status = "Directory is unavailable: " + file_browser_.directory.string();
    return;
  }
  file_browser_.directory = directory.lexically_normal();
  file_browser_.directory_input = file_browser_.directory.string();
  for (std::filesystem::directory_iterator iterator(file_browser_.directory, error), end;
       !error && iterator != end; iterator.increment(error)) {
    const auto& entry = *iterator;
    const bool directory_entry = entry.is_directory(error);
    if (error) {
      error.clear();
      continue;
    }
    const auto name = entry.path().filename().string();
    if (!directory_entry && lowercase(entry.path().extension().string()) != ".jsonl") continue;
    file_browser_.entries.push_back(FileBrowserEntry{entry.path(), name, directory_entry});
  }
  if (error) {
    file_browser_.status = "Directory listing failed: " + error.message();
    return;
  }
  std::sort(file_browser_.entries.begin(), file_browser_.entries.end(),
            [](const auto& lhs, const auto& rhs) {
              if (lhs.directory != rhs.directory) return lhs.directory > rhs.directory;
              return lowercase(lhs.name) < lowercase(rhs.name);
            });
  file_browser_.status = std::to_string(file_browser_.entries.size()) +
                         " directories and JSONL traces";
}

void ViewerApp::draw_file_browser() {
  if (file_browser_.open_requested) {
    ImGui::OpenPopup(file_browser_.title.c_str());
    file_browser_.open_requested = false;
  }
  if (!file_browser_.open) return;

  ImGui::SetNextWindowSize(ImVec2(780.0F, 540.0F), ImGuiCond_FirstUseEver);
  bool keep_open = true;
  if (!ImGui::BeginPopupModal(file_browser_.title.c_str(), &keep_open,
                              ImGuiWindowFlags_NoSavedSettings)) {
    file_browser_.open = keep_open;
    return;
  }
  std::optional<std::filesystem::path> chosen;
  ImGui::TextUnformatted("Directory");
  ImGui::SameLine();
  ImGui::SetNextItemWidth(std::max(220.0F, ImGui::GetContentRegionAvail().x - 180.0F));
  if (ImGui::InputText("##browser-directory", &file_browser_.directory_input,
                       ImGuiInputTextFlags_EnterReturnsTrue)) {
    file_browser_.directory = file_browser_.directory_input;
    refresh_file_browser();
  }
  ImGui::SameLine();
  if (ImGui::Button("Up")) {
    const auto parent = file_browser_.directory.parent_path();
    if (!parent.empty()) {
      file_browser_.directory = parent;
      refresh_file_browser();
    }
  }
  ImGui::SameLine();
  if (ImGui::Button("Refresh")) refresh_file_browser();
  ImGui::TextDisabled("%s", file_browser_.status.c_str());
  ImGui::Separator();

  const float footer_rows = file_browser_.save_mode ? 2.0F : 1.0F;
  const float footer_height = ImGui::GetFrameHeightWithSpacing() * footer_rows + 18.0F;
  ImGui::BeginChild("file-browser-list", ImVec2(0.0F, std::max(220.0F,
                     ImGui::GetContentRegionAvail().y - footer_height)), true);
  std::optional<std::filesystem::path> navigate_to;
  for (const auto& entry : file_browser_.entries) {
    const bool selected = file_browser_.selected == entry.path;
    const std::string label = std::string(entry.directory ? "[DIR]  " : "       ") + entry.name +
                              "##" + entry.path.string();
    if (ImGui::Selectable(label.c_str(), selected, ImGuiSelectableFlags_AllowDoubleClick)) {
      file_browser_.selected = entry.path;
      if (file_browser_.save_mode && !entry.directory) file_browser_.save_name = entry.name;
      if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
        if (entry.directory) {
          navigate_to = entry.path;
          break;
        } else if (!file_browser_.save_mode) {
          chosen = entry.path;
          break;
        }
      }
    }
  }
  ImGui::EndChild();
  if (navigate_to) {
    file_browser_.directory = *navigate_to;
    refresh_file_browser();
  }
  ImGui::Separator();

  if (file_browser_.save_mode) {
    ImGui::SetNextItemWidth(-1.0F);
    ImGui::InputText("File name", &file_browser_.save_name);
    ImGui::BeginDisabled(file_browser_.save_name.empty());
    if (ImGui::Button("Record here", ImVec2(140.0F, 0.0F))) {
      chosen = file_browser_.directory / file_browser_.save_name;
    }
    ImGui::EndDisabled();
  } else {
    std::error_code selected_error;
    const bool selected_file =
        !file_browser_.selected.empty() &&
        !std::filesystem::is_directory(file_browser_.selected, selected_error) && !selected_error;
    ImGui::BeginDisabled(!selected_file);
    if (ImGui::Button("Open trace", ImVec2(140.0F, 0.0F))) chosen = file_browser_.selected;
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Open this directory", ImVec2(170.0F, 0.0F))) {
      chosen = file_browser_.directory;
    }
  }
  ImGui::SameLine();
  if (ImGui::Button("Cancel", ImVec2(100.0F, 0.0F))) keep_open = false;

  if (chosen) {
    if (file_browser_.save_mode) start_recording(*chosen);
    else load_trace(*chosen);
    keep_open = false;
  }
  if (!keep_open) {
    file_browser_.open = false;
    ImGui::CloseCurrentPopup();
  }
  ImGui::EndPopup();
}

void ViewerApp::draw() {
  if (ImGui::IsKeyPressed(ImGuiKey_F, false) && !ImGui::GetIO().WantTextInput) {
    view_.fit_requested = true;
  }
  if (ImGui::GetIO().KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_O, false) &&
      !ImGui::GetIO().WantTextInput) {
    open_file_browser(false);
  }
  const ImGuiViewport* viewport = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(viewport->WorkPos);
  ImGui::SetNextWindowSize(viewport->WorkSize);
  const ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                                 ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoSavedSettings |
                                 ImGuiWindowFlags_MenuBar;
  ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0F);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0F);
  ImGui::Begin("Flux Router Telemetry", nullptr, flags);
  ImGui::PopStyleVar(2);
  draw_menu_bar();
  draw_status_bar();
  ImGui::Separator();

  const float available_height = ImGui::GetContentRegionAvail().y;
  if (ImGui::BeginTable("main-layout", 3, ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersInnerV,
                        ImVec2(0.0F, available_height))) {
    ImGui::TableSetupColumn("Controls", ImGuiTableColumnFlags_WidthFixed, 265.0F);
    ImGui::TableSetupColumn("Routing state", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableSetupColumn("Metrics", ImGuiTableColumnFlags_WidthFixed, 365.0F);
    ImGui::TableNextRow();

    ImGui::TableSetColumnIndex(0);
    ImGui::BeginChild("controls", ImVec2(0.0F, 0.0F));
    draw_source_panel();
    ImGui::Spacing();
    draw_overlay_panel();
    if (!ui_status_.empty()) {
      ImGui::Separator();
      ImGui::TextWrapped("%s", ui_status_.c_str());
    }
    ImGui::EndChild();

    ImGui::TableSetColumnIndex(1);
    constexpr float splitter_height = 7.0F;
    constexpr float minimum_pane_height = 100.0F;
    activity_panel_height_ = std::clamp(
        activity_panel_height_, minimum_pane_height,
        std::max(minimum_pane_height, available_height - minimum_pane_height - splitter_height));
    const float canvas_height =
        std::max(minimum_pane_height, available_height - activity_panel_height_ - splitter_height);
    ImGui::BeginChild("canvas", ImVec2(0.0F, canvas_height), false,
                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    draw_board_canvas();
    ImGui::EndChild();
    const ImVec2 splitter_start = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton("activity-panel-splitter", ImVec2(-1.0F, splitter_height));
    ImGui::GetWindowDrawList()->AddLine(
        ImVec2(splitter_start.x, splitter_start.y + splitter_height * 0.5F),
        ImVec2(splitter_start.x + ImGui::GetContentRegionAvail().x,
               splitter_start.y + splitter_height * 0.5F),
        ImGui::IsItemHovered() || ImGui::IsItemActive() ? color(0.42F, 0.78F, 1.0F)
                                                        : color(0.18F, 0.25F, 0.31F),
        ImGui::IsItemHovered() || ImGui::IsItemActive() ? 2.0F : 1.0F);
    if (ImGui::IsItemHovered() || ImGui::IsItemActive()) {
      ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS);
    }
    if (ImGui::IsItemActive()) {
      activity_panel_height_ = std::clamp(
          activity_panel_height_ - ImGui::GetIO().MouseDelta.y, minimum_pane_height,
          std::max(minimum_pane_height, available_height - minimum_pane_height - splitter_height));
    }
    ImGui::BeginChild("activity", ImVec2(0.0F, activity_panel_height_));
    draw_activity_panel();
    ImGui::EndChild();

    ImGui::TableSetColumnIndex(2);
    ImGui::BeginChild("metrics", ImVec2(0.0F, 0.0F));
    draw_metrics_panel();
    ImGui::EndChild();
    ImGui::EndTable();
  }

  draw_file_browser();
  ImGui::End();
}

} // namespace flux::viewer
