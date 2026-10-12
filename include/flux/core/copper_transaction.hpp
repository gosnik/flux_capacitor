#pragma once

#include "flux/core/connectivity.hpp"
#include "flux/core/drc.hpp"
#include "flux/core/validation.hpp"

#include <memory>

namespace flux {

struct CopperTransactionReport {
  ValidationReport structural;
  // Only relationships involving changed copper (plus affected differential-pair rules).
  // This is not a certificate that pre-existing, untouched copper is DRC clean.
  CopperDrcReport copper;
  ConnectivityReport before_connectivity;
  ConnectivityReport after_connectivity;
  bool preserves_pad_components{false};
  bool used_full_fallback{false};
  std::size_t changed_objects{0U};
  std::size_t neighbour_objects{0U};
  std::size_t affected_nets{0U};
  std::size_t connectivity_cache_hits{0U};
};

// One immutable copper revision, reusable for alternative transactions. Changes are inferred
// from complete canonical objects, not a caller-supplied dirty list. Static/rule changes use the
// full validator. Connectivity always uses actual fills unless the caller explicitly supplies a
// fill-cleared board; defer_zone_fill_clearance changes only the DRC view.
// Instances are thread-confined. Discard after accepting a transaction and prepare the new base.
// Final whole-board structural, copper, connectivity and KiCad validation remain mandatory.
class CopperTransactionValidator {
public:
  explicit CopperTransactionValidator(const Board& base, bool defer_zone_fill_clearance = false);
  ~CopperTransactionValidator();
  CopperTransactionValidator(CopperTransactionValidator&&) noexcept;
  CopperTransactionValidator& operator=(CopperTransactionValidator&&) noexcept;
  CopperTransactionValidator(const CopperTransactionValidator&) = delete;
  CopperTransactionValidator& operator=(const CopperTransactionValidator&) = delete;
  [[nodiscard]] CopperTransactionReport validate(const Board& candidate);
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace flux
