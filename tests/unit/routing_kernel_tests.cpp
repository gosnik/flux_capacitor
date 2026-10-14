#include "flux/core/routing_kernel.hpp"

#include <chrono>
#include <atomic>
#include <array>
#include <cstdint>
#include <iostream>
#include <future>
#include <stdexcept>
#include <vector>

using namespace std::chrono_literals;
static void require(bool condition, const char* message) {
  if (!condition)
    throw std::runtime_error(message);
}
int main() {
  try {
    std::vector<int> visited;
    int starts = 0;
    flux::RoutingKernel kernel([&](std::stop_token stop) {
      ++starts;
      std::vector<int> frontier{0, 1, 2, 3, 4, 5, 6};
      for (int node : frontier) {
        if (stop.stop_requested())
          return;
        visited.push_back(node);
        flux::routing_kernel_checkpoint();
      }
    });
    require(kernel.advance({0, 1s}) == flux::RoutingKernelStatus::yielded,
            "zero budget must not execute");
    require(starts == 0, "zero budget started kernel");
    require(kernel.advance({2, 1s}) == flux::RoutingKernelStatus::yielded,
            "work budget should yield");
    require(visited.size() == 2 && kernel.work_units() == 2, "wrong slice work");
    while (kernel.advance({2, 1s}) != flux::RoutingKernelStatus::finished) {
    }
    require(starts == 1 && visited == std::vector<int>({0, 1, 2, 3, 4, 5, 6}),
            "continuation restarted or lost frontier");
    require(kernel.advance() == flux::RoutingKernelStatus::finished, "finished not stable");

    int semantic_steps = 0;
    flux::RoutingKernel semantic([&](std::stop_token) {
      require(flux::routing_kernel_managed_thread(), "managed hook missing");
      ++semantic_steps;
      flux::routing_kernel_decision_point();
      ++semantic_steps;
    });
    require(!flux::routing_kernel_managed_thread(), "managed hook leaked to caller");
    require(semantic.advance({100000U, 1s}) == flux::RoutingKernelStatus::yielded &&
                semantic_steps == 1 && semantic.work_units() == 0U,
            "semantic boundary must yield independently of numerical budget");
    require(semantic.advance({100000U, 1s}) == flux::RoutingKernelStatus::finished &&
                semantic_steps == 2, "semantic resume restarted or lost work");

    std::chrono::nanoseconds active_before_park;
    std::chrono::nanoseconds active_after_resume;
    flux::RoutingKernel active_clock([&](std::stop_token) {
      active_before_park = flux::routing_kernel_active_time();
      flux::routing_kernel_decision_point();
      active_after_resume = flux::routing_kernel_active_time();
    });
    require(active_clock.advance({1'000U, 1s}) == flux::RoutingKernelStatus::yielded,
            "active clock fixture failed to park");
    std::this_thread::sleep_for(80ms);
    require(active_clock.advance({1'000U, 1s}) == flux::RoutingKernelStatus::finished,
            "active clock fixture failed to resume");
    require(active_after_resume >= active_before_park &&
                active_after_resume - active_before_park < 40ms,
            "managed active clock counted a parked interval");

    std::array<std::atomic<std::int64_t>, 2U> child_active{};
    std::chrono::nanoseconds parent_before_children;
    std::chrono::nanoseconds parent_after_children;
    flux::RoutingKernel inherited_clock([&](std::stop_token stop) {
      parent_before_children = flux::routing_kernel_active_time();
      flux::run_bounded_routing_tasks(2U, 2U, stop, [&](std::size_t index) {
        child_active[index].store(flux::routing_kernel_active_time().count(),
                                  std::memory_order_relaxed);
      });
      parent_after_children = flux::routing_kernel_active_time();
    });
    require(inherited_clock.advance({1'000U, 1s}) == flux::RoutingKernelStatus::finished,
            "child active clock fixture did not finish");
    for (const auto& child_time : child_active) {
      const auto value = std::chrono::nanoseconds{child_time.load(std::memory_order_relaxed)};
      require(value >= parent_before_children && value <= parent_after_children,
              "bounded child did not inherit its parent's managed active clock");
    }

    int stopped_work = 0;
    flux::RoutingKernel cancelled([&](std::stop_token stop) {
      while (!stop.stop_requested()) {
        ++stopped_work;
        flux::routing_kernel_checkpoint();
      }
    });
    require(cancelled.advance({1, 1s}) == flux::RoutingKernelStatus::yielded,
            "cancel fixture failed to pause");
    std::stop_source external;
    external.request_stop();
    require(cancelled.advance({1, 1s}, external.get_token()) == flux::RoutingKernelStatus::finished,
            "cancelled continuation did not unwind");
    require(stopped_work == 1, "cancelled kernel performed more work");

    std::promise<void> unwound;
    auto unwind_complete = unwound.get_future();
    flux::RoutingKernel standalone_cancel([&](std::stop_token) {
      struct SignalOnExit {
        std::promise<void>& signal;
        ~SignalOnExit() { signal.set_value(); }
      } signal{unwound};
      flux::routing_kernel_checkpoint();
    });
    require(standalone_cancel.advance({1, 1s}) == flux::RoutingKernelStatus::yielded,
            "standalone cancellation fixture failed to pause");
    standalone_cancel.request_stop();
    require(unwind_complete.wait_for(1s) == std::future_status::ready,
            "request_stop must wake a paused kernel without another advance");

    std::promise<void> release_wait;
    auto waiting = release_wait.get_future();
    flux::RoutingKernel blocked([&](std::stop_token) { waiting.wait(); });
    std::size_t heartbeats = 0U;
    bool no_fabricated_work = true;
    require(blocked.advance({1U, 1s}, {}, [&](std::size_t work) {
              no_fabricated_work = no_fabricated_work && work == 0U;
              if (++heartbeats == 1U) release_wait.set_value();
            }) == flux::RoutingKernelStatus::finished && heartbeats > 0U && no_fabricated_work,
            "non-yielding operation must publish progress without inspecting its transaction");

    flux::RoutingKernel failure([](std::stop_token) { throw std::runtime_error("kernel-error"); });
    bool propagated = false;
    try {
      static_cast<void>(failure.advance());
    } catch (const std::runtime_error& error) {
      propagated = std::string(error.what()) == "kernel-error";
    }
    require(propagated, "kernel exception not propagated");

    // Child routing checkpoints must park as one batch before the managed parent yields. The
    // observed count therefore stays frozen while advance() has returned yielded, and resumes
    // only when the parent continuation is advanced again.
    std::atomic<std::size_t> child_work{0U};
    flux::RoutingKernel bounded([&](std::stop_token stop) {
      flux::run_bounded_routing_tasks(2U, 2U, stop, [&](std::size_t) {
        while (!stop.stop_requested()) {
          child_work.fetch_add(1U, std::memory_order_relaxed);
          flux::routing_kernel_checkpoint();
        }
      });
    });
    require(bounded.advance({1U, 1s}) == flux::RoutingKernelStatus::yielded,
            "bounded child batch must yield through its managed parent");
    const auto parked_work = child_work.load(std::memory_order_relaxed);
    for (std::size_t spin = 0U; spin < 10'000U; ++spin) std::this_thread::yield();
    require(child_work.load(std::memory_order_relaxed) == parked_work,
            "all child work must be parked before the parent reports yielded");
    require(bounded.work_units() > 0U, "parent must aggregate child checkpoint work");
    bounded.request_stop();
    require(bounded.advance({1U, 1s}) == flux::RoutingKernelStatus::finished,
            "cancelling a parked child batch must unwind every child");

    std::atomic<unsigned> child_decisions{0U};
    std::atomic<unsigned> after_child_decision{0U};
    flux::RoutingKernel decision_batch([&](std::stop_token stop) {
      flux::run_bounded_routing_tasks(2U, 2U, stop, [&](std::size_t) {
        ++child_decisions;
        flux::routing_kernel_decision_point();
        ++after_child_decision;
      });
    });
    require(decision_batch.advance({1'000'000U, 1s}) == flux::RoutingKernelStatus::yielded &&
                child_decisions == 2U && after_child_decision == 0U,
            "child semantic decisions must yield even with an unspent numerical budget");
    require(decision_batch.advance({1'000'000U, 1s}) == flux::RoutingKernelStatus::finished &&
                after_child_decision == 2U,
            "resuming a semantic batch boundary must release both children");

    flux::RoutingKernel child_failure([](std::stop_token stop) {
      flux::run_bounded_routing_tasks(2U, 2U, stop, [](std::size_t task) {
        if (task == 0U) throw std::runtime_error("first-child-error");
        while (true) flux::routing_kernel_checkpoint();
      });
    });
    bool child_failure_propagated = false;
    try {
      static_cast<void>(child_failure.advance({100U, 1s}));
    } catch (const std::runtime_error& error) {
      child_failure_propagated = std::string(error.what()) == "first-child-error";
    }
    require(child_failure_propagated,
            "bounded child exceptions must unwind peers and propagate in task order");
    {
      flux::RoutingKernel abandoned([](std::stop_token stop) {
        while (!stop.stop_requested())
          flux::routing_kernel_checkpoint();
      });
      static_cast<void>(abandoned.advance({1, 1s}));
    }
    std::cout << "routing kernel continuation tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
