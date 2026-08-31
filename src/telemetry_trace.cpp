#include "flux/telemetry/trace.hpp"

#include <iterator>
#include <sstream>

namespace flux::telemetry {

bool Trace::load(const std::filesystem::path& path, std::string& error) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    error = "could not open telemetry trace: " + path.string();
    return false;
  }
  const std::string text{std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
  if (!input.eof() && input.fail()) {
    error = "could not read telemetry trace: " + path.string();
    return false;
  }
  if (!load_text(text, error)) {
    return false;
  }
  path_ = path;
  return true;
}

bool Trace::load_text(std::string_view text, std::string& error) {
  DecodeResult decoded = decode_events(text);
  if (!decoded) {
    error = std::move(decoded.error);
    return false;
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
