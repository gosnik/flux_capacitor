#pragma once

#include "flux/telemetry/event.hpp"

#include <atomic>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace flux::viewer {

enum class ConnectionState { disconnected, connecting, connected, failed };

struct ConnectionStatus {
  ConnectionState state{ConnectionState::disconnected};
  std::string message{"Disconnected"};
  std::uint64_t connection_generation{0};
  std::uint64_t received_events{0};
  std::uint64_t viewer_dropped_events{0};
};

class LiveClient {
public:
  LiveClient() = default;
  ~LiveClient();
  LiveClient(const LiveClient&) = delete;
  LiveClient& operator=(const LiveClient&) = delete;

  [[nodiscard]] bool connect(std::string endpoint, telemetry::Level level, double rate_hz);
  void disconnect();
  [[nodiscard]] std::vector<telemetry::Event> drain();
  [[nodiscard]] ConnectionStatus status() const;

private:
  void run(std::stop_token stop, std::string endpoint, telemetry::Level level, double rate_hz);
  void set_status(ConnectionState state, std::string message);
  void mark_connected();
  [[nodiscard]] bool accept_message(std::string_view message, std::string& error);

  mutable std::mutex mutex_;
  std::deque<telemetry::Event> pending_;
  ConnectionStatus status_;
  std::jthread thread_;
};

} // namespace flux::viewer
