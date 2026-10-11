#include "flux/core/repair_subset.hpp"
#include <iostream>
#include <stdexcept>

void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
int main() {
  try {
    // Restoring B introduces a new collision for A; independent C remains a useful legal move.
    auto checked = flux::retain_safe_repair_subset(3, 8, [](const auto& mask) {
      flux::RepairSubsetAssessment r;
      if (mask[1]) r.blocked_groups.push_back(1);
      else if (mask[0]) r.blocked_groups.push_back(0);
      r.strictly_improved = mask[2];
      return r;
    });
    require(checked.accepted && checked.retained == std::vector<bool>({false,false,true}) &&
            checked.assessments == 3, "survivors were not rechecked after restoring obstacles");
    auto bounded = flux::retain_safe_repair_subset(3, 2, [](const auto& mask) {
      return flux::RepairSubsetAssessment{{mask[1] ? 1U : 0U}, true};
    });
    require(!bounded.accepted, "unverified survivor accepted at budget exhaustion");
    auto unchanged = flux::retain_safe_repair_subset(2, 4, [](const auto&) {
      return flux::RepairSubsetAssessment{{}, false};
    });
    require(!unchanged.accepted, "non-improvement accepted");
    // Groups represent all same-net/junction dependent moves: they leave or survive together.
    auto atomic = flux::retain_safe_repair_subset(2, 4, [](const auto& mask) {
      return flux::RepairSubsetAssessment{mask[0] ? std::vector<std::size_t>{0} :
                                                   std::vector<std::size_t>{}, true};
    });
    require(atomic.accepted && !atomic.retained[0] && atomic.retained[1],
            "dependent group was partially retained");
    // Two moves can only survive together; removing an unrelated failed move must preserve
    // their joint legality. The oracle sees the simultaneous retained scene, not sequential moves.
    auto pair = flux::retain_safe_repair_subset(3, 4, [](const auto& mask) {
      flux::RepairSubsetAssessment r;
      if (mask[2]) r.blocked_groups.push_back(2);
      if (mask[0] != mask[1]) r.blocked_groups.push_back(mask[0] ? 0U : 1U);
      r.strictly_improved = mask[0] && mask[1];
      return r;
    });
    require(pair.accepted && pair.retained == std::vector<bool>({true,true,false}),
            "mutually dependent safe moves were lost with an unrelated blocked move");
    auto zero = flux::retain_safe_repair_subset(2, 0, [](const auto&) {
      throw std::runtime_error("zero budget invoked oracle");
      return flux::RepairSubsetAssessment{};
    });
    require(!zero.accepted, "zero budget accepted");
    std::cout << "safe repair subset tests passed\n";
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
