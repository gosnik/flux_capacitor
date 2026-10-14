#include "flux/core/routing_kernel.hpp"

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <exception>
#include <limits>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

namespace flux {
namespace {
struct KernelCancelled {};
thread_local std::function<void(std::size_t, bool)>* checkpoint_handler = nullptr;
thread_local std::function<std::chrono::nanoseconds()>* active_time_handler = nullptr;
} // namespace

struct RoutingKernel::Impl {
  std::mutex mutex;
  std::condition_variable changed;
  std::stop_source stop;
  bool running{false};
  bool finished{false};
  std::size_t work{0U};
  std::size_t slice_work{0U};
  RoutingKernelBudget budget;
  std::chrono::steady_clock::time_point slice_start;
  std::chrono::steady_clock::time_point active_segment_start;
  std::chrono::steady_clock::duration active_elapsed{};
  bool active_segment_running{false};
  std::exception_ptr error;
  std::thread worker;

  explicit Impl(std::function<void(std::stop_token)> task) {
    worker = std::thread([this, task = std::move(task)] {
      std::function<void(std::size_t, bool)> checkpoint = [this](std::size_t amount, bool decision) {
        std::unique_lock lock(mutex);
        if (stop.stop_requested())
          throw KernelCancelled{};
        work += std::min(amount, std::numeric_limits<std::size_t>::max() - work);
        slice_work += std::min(amount, std::numeric_limits<std::size_t>::max() - slice_work);
        if (!stop.stop_requested() &&
            (decision || slice_work >= budget.work_units ||
             std::chrono::steady_clock::now() - slice_start >= budget.time_slice)) {
          if (active_segment_running) {
            active_elapsed += std::chrono::steady_clock::now() - active_segment_start;
            active_segment_running = false;
          }
          running = false;
          changed.notify_all();
          changed.wait(lock, [this] { return running || stop.stop_requested(); });
          if (stop.stop_requested())
            throw KernelCancelled{};
        }
      };
      {
        std::unique_lock lock(mutex);
        changed.wait(lock, [this] { return running || stop.stop_requested(); });
      }
      checkpoint_handler = &checkpoint;
      std::function<std::chrono::nanoseconds()> active_time = [this] {
        std::lock_guard lock(mutex);
        auto elapsed = active_elapsed;
        if (active_segment_running)
          elapsed += std::chrono::steady_clock::now() - active_segment_start;
        return std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed);
      };
      active_time_handler = &active_time;
      try {
        if (!stop.stop_requested())
          task(stop.get_token());
      } catch (const KernelCancelled&) {
        // Discard the in-flight transaction; the engine retains its last committed checkpoint.
      } catch (...) {
        error = std::current_exception();
      }
      checkpoint_handler = nullptr;
      active_time_handler = nullptr;
      {
        std::lock_guard lock(mutex);
        if (active_segment_running) {
          active_elapsed += std::chrono::steady_clock::now() - active_segment_start;
          active_segment_running = false;
        }
        finished = true;
        running = false;
      }
      changed.notify_all();
    });
  }
};

RoutingKernel::RoutingKernel(std::function<void(std::stop_token)> task)
    : impl_(std::make_unique<Impl>(std::move(task))) {}
RoutingKernel::~RoutingKernel() {
  request_stop();
  {
    std::lock_guard lock(impl_->mutex);
    impl_->running = true;
  }
  impl_->changed.notify_all();
  if (impl_->worker.joinable())
    impl_->worker.join();
}
void RoutingKernel::request_stop() {
  impl_->stop.request_stop();
  // Synchronize with wait's predicate to avoid a missed wakeup.
  {
    std::lock_guard lock(impl_->mutex);
  }
  impl_->changed.notify_all();
}
std::stop_token RoutingKernel::stop_token() const { return impl_->stop.get_token(); }
std::size_t RoutingKernel::work_units() const {
  std::lock_guard lock(impl_->mutex);
  return impl_->work;
}
RoutingKernelStatus RoutingKernel::advance(RoutingKernelBudget budget, std::stop_token stop,
                                           const std::function<void(std::size_t)>& on_wait) {
  std::stop_callback callback(stop, [this] { request_stop(); });
  std::unique_lock lock(impl_->mutex);
  if (!impl_->finished && (impl_->stop.stop_requested() ||
                           (budget.work_units != 0U && budget.time_slice.count() > 0))) {
    impl_->budget = budget;
    impl_->slice_work = 0U;
    impl_->slice_start = std::chrono::steady_clock::now();
    impl_->active_segment_start = impl_->slice_start;
    impl_->active_segment_running = !impl_->stop.stop_requested();
    impl_->running = true;
    impl_->changed.notify_all();
    while (!impl_->changed.wait_for(lock, std::chrono::milliseconds{500},
                                    [this] { return !impl_->running || impl_->finished; })) {
      if (on_wait) {
        const auto work = impl_->work;
        lock.unlock();
        // Observability is never an input to the routing transaction.
        try { on_wait(work); } catch (...) {}
        lock.lock();
      }
    }
  }
  if (impl_->finished && impl_->error)
    std::rethrow_exception(impl_->error);
  return impl_->finished ? RoutingKernelStatus::finished : RoutingKernelStatus::yielded;
}
void routing_kernel_checkpoint(std::size_t work) {
  if (checkpoint_handler != nullptr)
    (*checkpoint_handler)(work, false);
}
void routing_kernel_decision_point() {
  if (checkpoint_handler != nullptr) (*checkpoint_handler)(0U, true);
}
bool routing_kernel_managed_thread() { return checkpoint_handler != nullptr; }
std::chrono::nanoseconds routing_kernel_active_time() {
  if (active_time_handler != nullptr)
    return (*active_time_handler)();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch());
}

void run_bounded_routing_tasks(std::size_t count, std::size_t workers, std::stop_token stop,
                               const std::function<void(std::size_t)>& task) {
  if (count == 0U) return;
  const auto thread_count = std::min(count, workers);
  if (thread_count < 2U || checkpoint_handler == nullptr) {
    if (thread_count < 2U) {
      for (std::size_t index = 0U; index < count; ++index) {
        if (stop.stop_requested()) return;
        task(index);
      }
      return;
    }
    std::atomic<std::size_t> next{0U};
    std::vector<std::exception_ptr> failures(count);
    std::vector<std::jthread> pool;
    pool.reserve(thread_count);
    for (std::size_t worker = 0U; worker < thread_count; ++worker) {
      pool.emplace_back([&](std::stop_token) {
        while (!stop.stop_requested()) {
          const auto index = next.fetch_add(1U);
          if (index >= count) return;
          try { task(index); } catch (...) { failures[index] = std::current_exception(); return; }
        }
      });
    }
    pool.clear();
    for (const auto& failure : failures)
      if (failure) std::rethrow_exception(failure);
    return;
  }

  struct BatchState {
    std::mutex mutex;
    std::condition_variable changed;
    std::atomic<std::size_t> next{0U};
    std::vector<std::exception_ptr> failures;
    std::size_t parked{0U};
    std::size_t completed{0U};
    std::size_t pending_work{0U};
    bool decision_pending{false};
    bool pause_requested{false};
    bool cancelling{false};
    explicit BatchState(std::size_t tasks) : failures(tasks) {}
  } state{count};

  struct ResumeOnExit {
    BatchState& state;
    ~ResumeOnExit() {
      std::lock_guard lock(state.mutex);
      state.cancelling = true;
      state.pause_requested = false;
      state.changed.notify_all();
    }
  };

  auto* const inherited_active_time_handler = active_time_handler;
  const auto child = [&, inherited_active_time_handler](std::stop_token) {
    std::function<void(std::size_t, bool)> child_checkpoint = [&](std::size_t amount, bool decision) {
      std::unique_lock lock(state.mutex);
      if (stop.stop_requested() || state.cancelling) throw KernelCancelled{};
      state.pending_work += std::min(amount, std::numeric_limits<std::size_t>::max() - state.pending_work);
      state.decision_pending = state.decision_pending || decision;
      state.pause_requested = state.pause_requested || decision;
      if (!state.pause_requested) return;
      ++state.parked;
      state.changed.notify_all();
      state.changed.wait(lock, [&] {
        return !state.pause_requested || stop.stop_requested() || state.cancelling;
      });
      --state.parked;
      if (stop.stop_requested() || state.cancelling) throw KernelCancelled{};
    };
    checkpoint_handler = &child_checkpoint;
    active_time_handler = inherited_active_time_handler;
    while (!stop.stop_requested()) {
      const auto index = state.next.fetch_add(1U);
      if (index >= count) break;
      try {
        task(index);
      } catch (const KernelCancelled&) {
        break;
      } catch (...) {
        std::lock_guard lock(state.mutex);
        state.failures[index] = std::current_exception();
        state.cancelling = true;
        state.changed.notify_all();
        break;
      }
    }
    checkpoint_handler = nullptr;
    active_time_handler = nullptr;
    {
      std::lock_guard lock(state.mutex);
      ++state.completed;
    }
    state.changed.notify_all();
  };

  std::vector<std::jthread> pool;
  ResumeOnExit resume_on_exit{state}; // Cancel/release children before pool destruction joins.
  pool.reserve(thread_count);
  for (std::size_t worker = 0U; worker < thread_count; ++worker)
    pool.emplace_back(child);

  while (true) {
    std::unique_lock lock(state.mutex);
    if (state.completed == thread_count || stop.stop_requested() || state.cancelling) break;
    // Keep the interval short enough that a 50ms outer slice has several chances to park a
    // bounded batch, without making every child checkpoint contend on the parent handler.
    state.changed.wait_for(lock, std::chrono::milliseconds{3});
    if (state.completed == thread_count || stop.stop_requested() || state.cancelling) break;
    state.pause_requested = true;
    state.changed.notify_all();
    state.changed.wait(lock, [&] {
      return state.completed == thread_count || stop.stop_requested() || state.cancelling ||
             state.parked + state.completed == thread_count;
    });
    if (state.completed == thread_count || stop.stop_requested() || state.cancelling) break;
    const auto work = std::exchange(state.pending_work, 0U);
    const auto decision = std::exchange(state.decision_pending, false);
    lock.unlock();
    // Only every active child is parked at this point, so yielding the parent cannot leave a
    // mutable candidate transaction running in the background.
    try {
      (*checkpoint_handler)(work, decision);
    } catch (...) {
      std::lock_guard release_lock(state.mutex);
      state.cancelling = true;
      state.pause_requested = false;
      state.changed.notify_all();
      throw;
    }
    lock.lock();
    state.pause_requested = false;
    state.changed.notify_all();
  }
  {
    std::lock_guard lock(state.mutex);
    state.cancelling = state.cancelling || stop.stop_requested();
    state.pause_requested = false;
  }
  state.changed.notify_all();
  pool.clear();
  for (const auto& failure : state.failures)
    if (failure) std::rethrow_exception(failure);
}
} // namespace flux
