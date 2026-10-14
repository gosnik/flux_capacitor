#pragma once

#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <stop_token>

namespace flux {

struct RoutingKernelBudget {
  std::size_t work_units{256U};
  std::chrono::milliseconds time_slice{50};
};
enum class RoutingKernelStatus { yielded, finished };

// In-memory continuation of an existing routing kernel. Suspension retains its stack, search
// frontier and caches. Only advance() runs it; callers may inspect owned state after advance
// returns. Concurrent advance calls or inspecting state while advance is active are unsupported.
// Budgets are cooperative: a device dispatch or geometry operation runs to its next checkpoint.
class RoutingKernel {
public:
  explicit RoutingKernel(std::function<void(std::stop_token)> task);
  ~RoutingKernel();
  RoutingKernel(const RoutingKernel&) = delete;
  RoutingKernel& operator=(const RoutingKernel&) = delete;
  // Called on the advancing thread, at most twice per second while awaiting a checkpoint.
  // Receives only a synchronized work counter; the running transaction must not be inspected.
  RoutingKernelStatus advance(RoutingKernelBudget budget = {}, std::stop_token stop = {},
                              const std::function<void(std::size_t)>& on_wait = {});
  void request_stop();
  [[nodiscard]] std::stop_token stop_token() const;
  [[nodiscard]] std::size_t work_units() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// No-op outside a managed kernel. Worker-pool children remain bounded by their own existing
// cancellation contracts; they do not inherit the owning thread's suspension context.
void routing_kernel_checkpoint(std::size_t work = 1U);
// A semantic decision boundary always parks the managed continuation, independent of its
// numerical slice budget. The caller can schedule another operation before resuming it.
void routing_kernel_decision_point();
[[nodiscard]] bool routing_kernel_managed_thread();
// Returns cumulative time spent executing the current managed kernel. Time while its
// continuation is parked is excluded. Outside a managed kernel this is the steady-clock
// epoch time, so callers can use the same clock for request-level elapsed budgets.
[[nodiscard]] std::chrono::nanoseconds routing_kernel_active_time();

// Runs independent routing tasks with at most `workers` child threads. In a managed routing
// continuation, child checkpoints park behind a batch-local barrier before the parent reaches
// its own checkpoint; the parent therefore never yields while children can mutate their private
// task state. Callers own any CPU-worker lease and must keep task outputs isolated until return.
// Exceptions are rethrown in stable task-index order after every child has unwound.
void run_bounded_routing_tasks(std::size_t count, std::size_t workers, std::stop_token stop,
                               const std::function<void(std::size_t)>& task);

} // namespace flux
