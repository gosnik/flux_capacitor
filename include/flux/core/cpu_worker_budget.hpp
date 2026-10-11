#pragma once

#include <cstddef>

namespace flux {

// Process-wide advisory budget for CPU-heavy routing worker threads. Callers retain a lease for
// the lifetime of their worker threads. Acquisition never waits, so nested work falls back to
// fewer workers rather than deadlocking behind the caller.
class CpuWorkerLease {
public:
  CpuWorkerLease() = default;
  ~CpuWorkerLease();
  CpuWorkerLease(const CpuWorkerLease&) = delete;
  CpuWorkerLease& operator=(const CpuWorkerLease&) = delete;
  CpuWorkerLease(CpuWorkerLease&& other) noexcept;
  CpuWorkerLease& operator=(CpuWorkerLease&& other) noexcept;

  [[nodiscard]] std::size_t workers() const { return workers_; }

private:
  friend CpuWorkerLease acquire_cpu_workers(std::size_t requested, std::size_t caller_limit);
  explicit CpuWorkerLease(std::size_t workers) : workers_(workers) {}
  void release();
  std::size_t workers_{0U};
};

// `caller_limit == 0` uses the host-derived process budget. A non-zero value gives a
// memory-sensitive phase an additional cap without changing the global limit.
[[nodiscard]] CpuWorkerLease acquire_cpu_workers(std::size_t requested,
                                                  std::size_t caller_limit = 0U);

} // namespace flux
