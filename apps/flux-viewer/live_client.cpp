#include "live_client.hpp"

#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>

#include <algorithm>
#include <chrono>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace flux::viewer {
namespace {

using tcp = boost::asio::ip::tcp;
namespace beast = boost::beast;
namespace websocket = beast::websocket;

constexpr auto reconnect_delay = std::chrono::milliseconds(500);
constexpr auto retry_poll_interval = std::chrono::milliseconds(50);
constexpr auto connect_timeout = std::chrono::seconds(2);

struct Endpoint {
  std::string host;
  std::string port;
  std::string target;
};

Endpoint parse_endpoint(std::string endpoint) {
  constexpr std::string_view scheme = "ws://";
  if (endpoint.starts_with("wss://")) {
    throw std::runtime_error("wss:// is not supported; the Flux telemetry endpoint is local ws://");
  }
  if (endpoint.starts_with(scheme)) {
    endpoint.erase(0, scheme.size());
  }
  const std::size_t slash = endpoint.find('/');
  const std::string authority = endpoint.substr(0, slash);
  const std::string target = slash == std::string::npos ? "/telemetry" : endpoint.substr(slash);
  if (authority.empty()) {
    throw std::runtime_error("telemetry endpoint has no host");
  }

  Endpoint parsed;
  parsed.target = target.empty() ? "/telemetry" : target;
  if (authority.front() == '[') {
    const std::size_t bracket = authority.find(']');
    if (bracket == std::string::npos) {
      throw std::runtime_error("invalid bracketed telemetry host");
    }
    parsed.host = authority.substr(1, bracket - 1U);
    parsed.port = bracket + 1U < authority.size() && authority[bracket + 1U] == ':'
                      ? authority.substr(bracket + 2U)
                      : "8765";
  } else {
    const std::size_t colon = authority.rfind(':');
    if (colon == std::string::npos) {
      parsed.host = authority;
      parsed.port = "8765";
    } else {
      parsed.host = authority.substr(0, colon);
      parsed.port = authority.substr(colon + 1U);
    }
  }
  if (parsed.host.empty() || parsed.port.empty()) {
    throw std::runtime_error("invalid telemetry host or port");
  }
  return parsed;
}

void wait_before_retry(std::stop_token stop) {
  const auto deadline = std::chrono::steady_clock::now() + reconnect_delay;
  while (!stop.stop_requested() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(retry_poll_interval);
  }
}

} // namespace

LiveClient::~LiveClient() { disconnect(); }

bool LiveClient::connect(std::string endpoint, telemetry::Level level, double rate_hz) {
  disconnect();
  if (endpoint.empty()) {
    set_status(ConnectionState::failed, "Telemetry endpoint is empty");
    return false;
  }
  set_status(ConnectionState::connecting, "Connecting to " + endpoint);
  thread_ = std::jthread([this, endpoint = std::move(endpoint), level,
                          rate_hz](std::stop_token stop) { run(stop, endpoint, level, rate_hz); });
  return true;
}

void LiveClient::disconnect() {
  if (thread_.joinable()) {
    thread_.request_stop();
    thread_.join();
  }
  std::lock_guard lock(mutex_);
  pending_.clear();
  pending_bytes_ = 0U;
  status_.state = ConnectionState::disconnected;
  status_.message = "Disconnected";
}

std::vector<telemetry::Event> LiveClient::drain() {
  std::lock_guard lock(mutex_);
  std::vector<telemetry::Event> drained;
  drained.reserve(pending_.size());
  while (!pending_.empty()) {
    drained.push_back(std::move(pending_.front()));
    pending_.pop_front();
  }
  pending_bytes_ = 0U;
  return drained;
}

ConnectionStatus LiveClient::status() const {
  std::lock_guard lock(mutex_);
  return status_;
}

void LiveClient::set_status(ConnectionState state, std::string message) {
  std::lock_guard lock(mutex_);
  status_.state = state;
  status_.message = std::move(message);
}

void LiveClient::mark_connected() {
  std::lock_guard lock(mutex_);
  // A server recovery snapshot is the authoritative replacement for this queue.
  // Never replay incremental events from a prior connection into its new generation.
  pending_.clear();
  pending_bytes_ = 0U;
  ++status_.connection_generation;
  status_.state = ConnectionState::connected;
  status_.message = "Live telemetry connected";
}

bool LiveClient::accept_message(std::string_view message, std::string& error) {
  if (message.size() > maximum_message_bytes) {
    error = "Telemetry message exceeds the viewer's 8 MiB safety limit";
    return false;
  }
  telemetry::DecodeResult decoded = telemetry::decode_events(message);
  if (!decoded) {
    error = "Telemetry decode failed: " + decoded.error;
    return false;
  }

  std::lock_guard lock(mutex_);
  for (telemetry::Event& event : decoded.events) {
    const std::size_t event_bytes = telemetry::estimated_event_bytes(event);
    ++status_.received_events;
    if (event_bytes > maximum_pending_bytes) {
      if (event.type == telemetry::EventType::snapshot) {
        error = "Telemetry recovery snapshot exceeds the viewer's 32 MiB queue limit";
        return false;
      }
      ++status_.viewer_dropped_events;
      continue;
    }
    // Source and dynamic recovery snapshots are independent. Neither can discard the
    // other, or the milestones needed by recorded history, merely by arriving later.
    while (!pending_.empty() &&
           (pending_.size() >= maximum_pending_events ||
            pending_bytes_ > maximum_pending_bytes - event_bytes)) {
      pending_bytes_ -= telemetry::estimated_event_bytes(pending_.front());
      pending_.pop_front();
      ++status_.viewer_dropped_events;
    }
    pending_.push_back(std::move(event));
    pending_bytes_ += event_bytes;
  }
  return true;
}

void LiveClient::run(std::stop_token stop, std::string endpoint, telemetry::Level level,
                     double rate_hz) {
  Endpoint parsed;
  try {
    parsed = parse_endpoint(std::move(endpoint));
  } catch (const std::exception& exception) {
    if (!stop.stop_requested()) {
      set_status(ConnectionState::failed,
                 "Invalid telemetry endpoint: " + std::string(exception.what()));
    }
    return;
  }

  bool reconnecting = false;
  while (!stop.stop_requested()) {
    set_status(ConnectionState::connecting,
               reconnecting ? "Waiting for Flux telemetry; reconnecting automatically"
                            : "Connecting to Flux telemetry");
    std::string retry_reason;
    try {
      boost::asio::io_context context;
      tcp::resolver resolver(context);
      websocket::stream<beast::tcp_stream> socket(context);
      const auto endpoints = resolver.resolve(parsed.host, parsed.port);
      socket.next_layer().expires_after(connect_timeout);
      socket.next_layer().connect(endpoints);
      socket.set_option(websocket::stream_base::timeout::suggested(beast::role_type::client));
      socket.set_option(websocket::stream_base::decorator([](websocket::request_type& request) {
        request.set(boost::beast::http::field::sec_websocket_protocol,
                    std::string(telemetry::protocol_name));
      }));
      socket.handshake(parsed.host + ':' + parsed.port, parsed.target);
      std::ostringstream subscription;
      subscription << "{\"type\":\"subscribe\",\"protocol\":\"" << telemetry::protocol_name
                   << "\",\"level\":\"" << telemetry::to_string(level)
                   << "\",\"rate_hz\":" << std::clamp(rate_hz, 1.0, 60.0) << '}';
      socket.write(boost::asio::buffer(subscription.str()));
      socket.next_layer().expires_never();
      mark_connected();
      reconnecting = true;

      beast::flat_buffer buffer;
      buffer.max_size(maximum_message_bytes);
      bool read_pending = false;
      bool read_failed = false;
      std::function<void()> begin_read;
      begin_read = [&]() {
        if (read_pending || read_failed || stop.stop_requested()) {
          return;
        }
        read_pending = true;
        socket.async_read(buffer, [&](beast::error_code error, std::size_t) {
          read_pending = false;
          if (error) {
            if (error != websocket::error::closed && !stop.stop_requested()) {
              retry_reason = error == websocket::error::buffer_overflow
                                 ? "Telemetry message exceeds the viewer's 8 MiB safety limit"
                                 : error.message();
            }
            read_failed = true;
            return;
          }
          const std::string payload = beast::buffers_to_string(buffer.data());
          buffer.consume(buffer.size());
          if (!accept_message(payload, retry_reason)) {
            read_failed = true;
            return;
          }
          begin_read();
        });
      };
      begin_read();

      while (!stop.stop_requested() && !read_failed) {
        context.run_for(retry_poll_interval);
        context.restart();
      }

      beast::error_code ignored;
      socket.next_layer().socket().shutdown(tcp::socket::shutdown_both, ignored);
      socket.next_layer().socket().close(ignored);
    } catch (const std::exception& exception) {
      if (!stop.stop_requested()) {
        retry_reason = exception.what();
      }
    }

    if (stop.stop_requested()) {
      break;
    }
    const std::string suffix = retry_reason.empty() ? std::string{} : ": " + retry_reason;
    set_status(ConnectionState::connecting,
               "Flux telemetry unavailable" + suffix + "; retrying automatically");
    reconnecting = true;
    wait_before_retry(stop);
  }
}

} // namespace flux::viewer
