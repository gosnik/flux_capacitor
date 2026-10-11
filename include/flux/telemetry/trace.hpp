#pragma once

#include "flux/telemetry/event.hpp"
#include "flux/telemetry/state.hpp"

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace flux::telemetry {

class Trace {
public:
  static constexpr std::size_t maximum_trace_bytes = 128U * 1024U * 1024U;
  static constexpr std::size_t maximum_line_bytes = 8U * 1024U * 1024U;
  [[nodiscard]] bool load(const std::filesystem::path& path, std::string& error);
  [[nodiscard]] bool load_text(std::string_view text, std::string& error);

  [[nodiscard]] const std::vector<Event>& events() const { return events_; }
  [[nodiscard]] const std::filesystem::path& path() const { return path_; }
  [[nodiscard]] bool empty() const { return events_.empty(); }

private:
  std::filesystem::path path_;
  std::vector<Event> events_;
};

class Replay {
public:
  void set_trace(const Trace* trace);
  void seek(std::size_t event_count);
  bool step();
  void reset();

  [[nodiscard]] const LiveState& state() const { return state_; }
  [[nodiscard]] LiveState& state() { return state_; }
  [[nodiscard]] std::size_t position() const { return position_; }
  [[nodiscard]] std::size_t size() const;

private:
  const Trace* trace_{nullptr};
  std::size_t position_{0};
  LiveState state_;
};

class TraceWriter {
public:
  ~TraceWriter();
  [[nodiscard]] bool open(const std::filesystem::path& path, std::string& error);
  [[nodiscard]] bool append(const Event& event);
  void close();
  [[nodiscard]] bool is_open() const { return stream_.is_open(); }
  [[nodiscard]] const std::string& error() const { return error_; }

private:
  std::ofstream stream_;
  std::string error_;
};

} // namespace flux::telemetry
