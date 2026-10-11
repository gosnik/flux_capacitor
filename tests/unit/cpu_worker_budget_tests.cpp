#include "flux/core/cpu_worker_budget.hpp"

#include <atomic>
#include <barrier>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

namespace {
void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
} // namespace

int main() {
  try {
    // A saturated caller limit must never wait for a nested lease: nested routing falls back to
    // its caller thread, which is what prevents worker-pool deadlocks.
    auto outer = flux::acquire_cpu_workers(1U, 1U);
    require(outer.workers() == 1U, "initial bounded lease was not granted");
    auto nested = flux::acquire_cpu_workers(8U, 1U);
    require(nested.workers() == 0U, "nested exhausted lease must return immediately with zero");
    nested = {};

    // Move assignment transfers ownership exactly once; releasing the moved-to lease restores
    // capacity for later routing work.
    auto moved = std::move(outer);
    require(outer.workers() == 0U && moved.workers() == 1U,
            "move must transfer the only active lease");
    moved = {};
    auto restored = flux::acquire_cpu_workers(1U, 1U);
    require(restored.workers() == 1U, "moved lease did not release capacity");
    restored = {};

    // Concurrent callers share one cap rather than each independently receiving its requested
    // worker count. The barrier holds every successful lease long enough to observe the bound.
    constexpr std::size_t callers = 8U;
    std::barrier rendezvous(static_cast<std::ptrdiff_t>(callers + 1U));
    std::atomic<std::size_t> active{0U};
    std::atomic<std::size_t> peak{0U};
    std::atomic<bool> release{false};
    std::vector<std::jthread> threads;
    threads.reserve(callers);
    for (std::size_t index = 0U; index < callers; ++index) {
      threads.emplace_back([&] {
        auto lease = flux::acquire_cpu_workers(1U, 2U);
        if (lease.workers() != 0U) {
          const auto now = active.fetch_add(lease.workers()) + lease.workers();
          auto observed = peak.load();
          while (observed < now &&
                 !peak.compare_exchange_weak(observed, now)) {
          }
        }
        rendezvous.arrive_and_wait();
        while (!release.load(std::memory_order_acquire)) std::this_thread::yield();
        if (lease.workers() != 0U) active.fetch_sub(lease.workers());
      });
    }
    rendezvous.arrive_and_wait();
    require(peak.load() <= 2U, "concurrent leases exceeded their shared caller cap");
    require(peak.load() > 0U, "concurrent lease test acquired no workers");
    release.store(true, std::memory_order_release);
    threads.clear();
    require(active.load() == 0U, "all concurrent leases must release at scope exit");

    std::cout << "cpu worker budget tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
