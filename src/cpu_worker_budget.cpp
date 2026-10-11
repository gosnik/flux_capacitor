#include "flux/core/cpu_worker_budget.hpp"

#include <algorithm>
#include <mutex>
#include <thread>
#include <utility>

namespace flux {
namespace {
struct CpuWorkerBudgetState {
  std::mutex mutex;
  std::size_t leased{0U};
  const std::size_t capacity{std::max<std::size_t>(
      1U, std::thread::hardware_concurrency() > 1U ? std::thread::hardware_concurrency() - 1U
                                                    : 1U)};
};

CpuWorkerBudgetState& budget_state() {
  static CpuWorkerBudgetState state;
  return state;
}
} // namespace

CpuWorkerLease::~CpuWorkerLease() { release(); }
CpuWorkerLease::CpuWorkerLease(CpuWorkerLease&& other) noexcept
    : workers_(std::exchange(other.workers_, 0U)) {}
CpuWorkerLease& CpuWorkerLease::operator=(CpuWorkerLease&& other) noexcept {
  if (this != &other) {
    release();
    workers_ = std::exchange(other.workers_, 0U);
  }
  return *this;
}
void CpuWorkerLease::release() {
  if (workers_ == 0U) return;
  auto& state = budget_state();
  std::lock_guard lock(state.mutex);
  state.leased -= workers_;
  workers_ = 0U;
}
CpuWorkerLease acquire_cpu_workers(std::size_t requested, std::size_t caller_limit) {
  if (requested == 0U) return {};
  auto& state = budget_state();
  std::lock_guard lock(state.mutex);
  const auto capacity = caller_limit == 0U ? state.capacity
                                            : std::min(state.capacity, caller_limit);
  const auto available = capacity > state.leased ? capacity - state.leased : 0U;
  const auto granted = std::min(requested, available);
  state.leased += granted;
  return CpuWorkerLease{granted};
}
} // namespace flux
