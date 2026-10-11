#include "flux/telemetry/trace.hpp"

#include <algorithm>
#include <cctype>
#include <limits>
#include <sstream>
#include <system_error>

namespace flux::telemetry {
namespace {

[[nodiscard]] bool append_trace_line(std::string_view line, const std::filesystem::path& path,
                                     std::size_t line_number, std::vector<Event>& events,
                                     std::size_t& stored_bytes, std::string& error) {
  const bool whitespace_only = std::all_of(line.begin(), line.end(), [](unsigned char ch) {
    return std::isspace(ch) != 0;
  });
  if (whitespace_only) {
    return true;
  }
  DecodeResult decoded = decode_events(line);
  if (!decoded) {
    error = path.string() + ": telemetry JSON line " + std::to_string(line_number) + ": " +
            decoded.error;
    return false;
  }
  for (Event& event : decoded.events) {
    const std::size_t event_bytes = estimated_event_bytes(event);
    if (event_bytes > Trace::maximum_trace_bytes ||
        stored_bytes > Trace::maximum_trace_bytes - event_bytes) {
      error = path.string() + ": telemetry replay exceeds the viewer's 128 MiB storage limit";
      return false;
    }
    stored_bytes += event_bytes;
    events.push_back(std::move(event));
  }
  return true;
}

[[nodiscard]] bool read_bounded_line(std::istream& input, std::string& line, bool& eof,
                                     std::string& error, const std::filesystem::path& path,
                                     std::size_t line_number) {
  line.clear();
  eof = false;
  char character = '\0';
  while (input.get(character)) {
    if (character == '\n') {
      return true;
    }
    if (line.size() == Trace::maximum_line_bytes) {
      error = path.string() + ": telemetry JSON line " + std::to_string(line_number) +
              " exceeds the viewer's 8 MiB line limit";
      return false;
    }
    line.push_back(character);
  }
  if (input.eof()) {
    eof = true;
    return !line.empty();
  }
  error = "could not read telemetry trace: " + path.string();
  return false;
}

[[nodiscard]] bool load_regular_trace(const std::filesystem::path& path,
                                      std::vector<Event>& events, std::size_t& stored_bytes,
                                      std::string& error) {
  try {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
      error = "could not open telemetry trace: " + path.string();
      return false;
    }
    std::string line;
    line.reserve(std::min<std::size_t>(Trace::maximum_line_bytes, 4096U));
    std::size_t line_number = 0U;
    bool eof = false;
    while (!eof) {
      ++line_number;
      if (!read_bounded_line(input, line, eof, error, path, line_number)) {
        if (eof && line.empty()) {
          break;
        }
        return false;
      }
      if (!append_trace_line(line, path, line_number, events, stored_bytes, error)) {
        return false;
      }
    }
    return true;
  } catch (const std::exception& exception) {
    error = "could not read telemetry trace " + path.string() + ": " + exception.what();
    return false;
  }
}

[[nodiscard]] std::uint64_t rebased_value(std::uint64_t value, std::uint64_t first,
                                          std::uint64_t preceding) {
  const auto delta = value >= first ? value - first : 0U;
  const auto base = preceding == std::numeric_limits<std::uint64_t>::max() ? preceding
                                                                           : preceding + 1U;
  return delta > std::numeric_limits<std::uint64_t>::max() - base
             ? std::numeric_limits<std::uint64_t>::max()
             : base + delta;
}

} // namespace

bool Trace::load(const std::filesystem::path& path, std::string& error) {
  std::error_code status_error;
  const auto status = std::filesystem::status(path, status_error);
  if (status_error) {
    error = "could not inspect telemetry path " + path.string() + ": " +
            status_error.message();
    return false;
  }
  if (std::filesystem::is_regular_file(status)) {
    std::vector<Event> loaded;
    std::size_t stored_bytes = 0U;
    if (!load_regular_trace(path, loaded, stored_bytes, error)) {
      return false;
    }
    if (loaded.empty()) {
      error = path.string() + ": telemetry input contains no events";
      return false;
    }
    events_ = std::move(loaded);
    path_ = path;
    error.clear();
    return true;
  }
  if (!std::filesystem::is_directory(status)) {
    error = "telemetry path is neither a regular file nor a directory: " + path.string();
    return false;
  }

  struct DirectoryTrace {
    std::filesystem::path path;
    std::filesystem::file_time_type modified;
  };
  std::vector<DirectoryTrace> files;
  std::error_code directory_error;
  for (std::filesystem::directory_iterator iterator(path, directory_error), end;
       !directory_error && iterator != end; iterator.increment(directory_error)) {
    std::error_code entry_error;
    if (!iterator->is_regular_file(entry_error) || entry_error ||
        iterator->path().extension() != ".jsonl") {
      continue;
    }
    auto modified = iterator->last_write_time(entry_error);
    if (entry_error) {
      modified = std::filesystem::file_time_type::min();
    }
    files.push_back(DirectoryTrace{iterator->path(), modified});
  }
  if (directory_error) {
    error = "could not enumerate telemetry directory " + path.string() + ": " +
            directory_error.message();
    return false;
  }
  std::sort(files.begin(), files.end(), [](const auto& lhs, const auto& rhs) {
    return lhs.modified != rhs.modified ? lhs.modified < rhs.modified : lhs.path < rhs.path;
  });
  if (files.empty()) {
    error = "telemetry directory contains no .jsonl traces: " + path.string();
    return false;
  }

  std::vector<Event> combined;
  std::size_t stored_bytes = 0U;
  for (const auto& file : files) {
    std::vector<Event> part;
    if (!load_regular_trace(file.path, part, stored_bytes, error)) {
      return false;
    }
    if (part.empty()) {
      continue;
    }
    if (!combined.empty()) {
      const auto preceding_sequence = std::max_element(
          combined.begin(), combined.end(),
          [](const auto& lhs, const auto& rhs) { return lhs.sequence < rhs.sequence; });
      const auto preceding_time = std::max_element(
          combined.begin(), combined.end(), [](const auto& lhs, const auto& rhs) {
            return lhs.monotonic_time_us < rhs.monotonic_time_us;
          });
      const auto first_sequence = part.front().sequence;
      const auto first_time = part.front().monotonic_time_us;
      for (auto& event : part) {
        event.sequence = rebased_value(event.sequence, first_sequence,
                                       preceding_sequence->sequence);
        event.monotonic_time_us =
            rebased_value(event.monotonic_time_us, first_time,
                          preceding_time->monotonic_time_us);
      }
    }
    combined.insert(combined.end(), std::make_move_iterator(part.begin()),
                    std::make_move_iterator(part.end()));
  }
  if (combined.empty()) {
    error = "telemetry directory contains no events: " + path.string();
    return false;
  }
  events_ = std::move(combined);
  path_ = path;
  error.clear();
  return true;
}

bool Trace::load_text(std::string_view text, std::string& error) {
  if (text.size() > maximum_line_bytes) {
    error = "telemetry text exceeds the viewer's 8 MiB input limit; use a JSONL trace file";
    return false;
  }
  DecodeResult decoded = decode_events(text);
  if (!decoded) {
    error = std::move(decoded.error);
    return false;
  }
  std::size_t stored_bytes = 0U;
  for (const Event& event : decoded.events) {
    const std::size_t event_bytes = estimated_event_bytes(event);
    if (event_bytes > maximum_trace_bytes || stored_bytes > maximum_trace_bytes - event_bytes) {
      error = "telemetry replay exceeds the viewer's 128 MiB storage limit";
      return false;
    }
    stored_bytes += event_bytes;
  }
  events_ = std::move(decoded.events);
  path_.clear();
  error.clear();
  return true;
}

void Replay::set_trace(const Trace* trace) {
  trace_ = trace;
  reset();
}

void Replay::seek(std::size_t event_count) {
  if (trace_ == nullptr) {
    reset();
    return;
  }
  event_count = std::min(event_count, trace_->events().size());
  if (event_count < position_) {
    state_.clear();
    position_ = 0;
  }
  while (position_ < event_count) {
    state_.apply(trace_->events()[position_]);
    ++position_;
  }
}

bool Replay::step() {
  if (trace_ == nullptr || position_ >= trace_->events().size()) {
    return false;
  }
  state_.apply(trace_->events()[position_]);
  ++position_;
  return true;
}

void Replay::reset() {
  position_ = 0;
  state_.clear();
}

std::size_t Replay::size() const { return trace_ == nullptr ? 0U : trace_->events().size(); }

TraceWriter::~TraceWriter() { close(); }

bool TraceWriter::open(const std::filesystem::path& path, std::string& error) {
  close();
  stream_.open(path, std::ios::binary | std::ios::trunc);
  if (!stream_) {
    error_ = "could not create telemetry trace: " + path.string();
    error = error_;
    return false;
  }
  error_.clear();
  error.clear();
  return true;
}

bool TraceWriter::append(const Event& event) {
  if (!stream_) {
    error_ = "telemetry trace is not open";
    return false;
  }
  stream_ << encode_event(event) << '\n';
  stream_.flush();
  if (!stream_) {
    error_ = "failed while writing telemetry trace";
    close();
    return false;
  }
  return true;
}

void TraceWriter::close() {
  if (stream_.is_open()) {
    stream_.close();
  }
}

} // namespace flux::telemetry
