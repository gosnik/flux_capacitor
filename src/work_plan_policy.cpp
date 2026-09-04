#include "flux/io/work_plan.hpp"

#include "flux/io/differential_pair_constraints.hpp"
#include "flux/io/kicad_netclass_snapshot.hpp"
#include "flux/io/orientation_constraints.hpp"
#include "flux/io/placement_envelope_constraints.hpp"
#include "flux/io/placement_group_constraints.hpp"
#include "flux/io/placement_move_priorities.hpp"
#include "flux/io/placement_region_constraints.hpp"
#include "flux/io/relative_placement_constraints.hpp"
#include "flux/io/routing_bus_constraints.hpp"

#include <openssl/evp.h>

#include <algorithm>
#include <array>
#include <fstream>
#include <map>
#include <memory>
#include <set>

namespace flux::io {
namespace {

constexpr std::uintmax_t maximum_policy_bytes = 64U * 1024U * 1024U;

enum class PolicyKind {
  netclasses,
  differential_pairs,
  placement_envelopes,
  routing_buses,
  placement_groups,
  placement_regions,
  relative_placements,
  orientation_constraints,
  placement_move_priorities,
};

struct LoadedPolicy {
  std::size_t index{0U};
  PolicyKind kind{PolicyKind::netclasses};
  std::string kind_name;
  std::string contents;
};

[[nodiscard]] std::optional<PolicyKind> parse_kind(std::string_view value) {
  static const std::map<std::string_view, PolicyKind> kinds{
      {"kicad-netclass-snapshot", PolicyKind::netclasses},
      {"differential-pairs", PolicyKind::differential_pairs},
      {"placement-envelopes", PolicyKind::placement_envelopes},
      {"routing-buses", PolicyKind::routing_buses},
      {"placement-groups", PolicyKind::placement_groups},
      {"placement-regions", PolicyKind::placement_regions},
      {"relative-placements", PolicyKind::relative_placements},
      {"orientation-constraints", PolicyKind::orientation_constraints},
      {"placement-move-priorities", PolicyKind::placement_move_priorities},
  };
  const auto found = kinds.find(value);
  return found == kinds.end() ? std::nullopt : std::optional<PolicyKind>{found->second};
}

[[nodiscard]] std::optional<std::string> sha256(std::string_view contents) {
  const auto deleter = [](EVP_MD_CTX* context) { EVP_MD_CTX_free(context); };
  std::unique_ptr<EVP_MD_CTX, decltype(deleter)> context(EVP_MD_CTX_new(), deleter);
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int size = 0U;
  if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1 ||
      EVP_DigestUpdate(context.get(), contents.data(), contents.size()) != 1 ||
      EVP_DigestFinal_ex(context.get(), digest.data(), &size) != 1 || size != 32U) {
    return std::nullopt;
  }
  constexpr char hexadecimal[] = "0123456789abcdef";
  std::string result(size * 2U, '0');
  for (std::size_t index = 0U; index < size; ++index) {
    result[index * 2U] = hexadecimal[digest[index] >> 4U];
    result[index * 2U + 1U] = hexadecimal[digest[index] & 0x0fU];
  }
  return result;
}

void diagnostic(WorkPlanPolicyApplicationResult& result, WorkPlanDiagnosticSeverity severity,
                std::string code, std::string message, std::size_t index) {
  result.diagnostics.push_back(
      {severity, std::move(code), std::move(message), "/policies/" + std::to_string(index)});
}

void append_constraint_diagnostics(WorkPlanPolicyApplicationResult& result,
                                   const std::vector<ConstraintDiagnostic>& diagnostics,
                                   std::size_t index) {
  for (const auto& item : diagnostics) {
    diagnostic(result,
               item.severity == ConstraintSeverity::error ? WorkPlanDiagnosticSeverity::error
                                                          : WorkPlanDiagnosticSeverity::warning,
               item.code,
               item.message + " (line " + std::to_string(item.line) + ", column " +
                   std::to_string(item.column) + ")",
               index);
  }
}

void append_netclass_diagnostics(WorkPlanPolicyApplicationResult& result,
                                 const std::vector<NetClassSnapshotDiagnostic>& diagnostics,
                                 std::size_t index) {
  for (const auto& item : diagnostics) {
    diagnostic(result,
               item.severity == NetClassSnapshotSeverity::error
                   ? WorkPlanDiagnosticSeverity::error
                   : WorkPlanDiagnosticSeverity::warning,
               item.code,
               item.message + " (line " + std::to_string(item.line) + ", column " +
                   std::to_string(item.column) + ")",
               index);
  }
}

template <typename Result>
[[nodiscard]] bool accept_board_result(WorkPlanPolicyApplicationResult& aggregate, Result&& applied,
                                       Board& board, std::size_t index) {
  append_constraint_diagnostics(aggregate, applied.diagnostics, index);
  if (!applied.ok()) {
    return false;
  }
  board = std::move(*applied.board);
  return true;
}

} // namespace

bool WorkPlanPolicyApplicationResult::has_errors() const {
  return std::any_of(diagnostics.begin(), diagnostics.end(), [](const auto& item) {
    return item.severity == WorkPlanDiagnosticSeverity::error;
  });
}

WorkPlanPolicyApplicationResult
apply_flux_work_plan_policies(const Board& source, const FluxWorkPlan& plan,
                              const std::filesystem::path& policy_base_directory) {
  WorkPlanPolicyApplicationResult result;
  std::vector<LoadedPolicy> loaded;
  loaded.reserve(plan.policies.size());
  std::set<PolicyKind> seen;
  for (std::size_t index = 0U; index < plan.policies.size(); ++index) {
    const auto& policy = plan.policies[index];
    const auto kind = parse_kind(policy.kind);
    if (!kind) {
      diagnostic(result, WorkPlanDiagnosticSeverity::error, "unsupported_work_plan_policy_kind",
                 "policy kind has no strict source-bound adapter", index);
      continue;
    }
    if (!seen.insert(*kind).second) {
      diagnostic(result, WorkPlanDiagnosticSeverity::error, "duplicate_work_plan_policy_kind",
                 "a policy kind may be bound at most once", index);
      continue;
    }
    auto path = std::filesystem::path{policy.path};
    if (path.is_relative()) {
      path = policy_base_directory / path;
    }
    path = path.lexically_normal();
    std::error_code error;
    if (!std::filesystem::is_regular_file(path, error) || error) {
      diagnostic(result, WorkPlanDiagnosticSeverity::error, "work_plan_policy_read_failed",
                 "bound policy path is not a readable regular file", index);
      continue;
    }
    const auto size = std::filesystem::file_size(path, error);
    if (error || size > maximum_policy_bytes) {
      diagnostic(result, WorkPlanDiagnosticSeverity::error, "work_plan_policy_budget_exceeded",
                 "bound policy file exceeds the 64 MiB identity-check limit", index);
      continue;
    }
    std::ifstream input(path, std::ios::binary);
    std::string contents{std::istreambuf_iterator<char>{input}, std::istreambuf_iterator<char>{}};
    if (input.bad() || contents.size() != size) {
      diagnostic(result, WorkPlanDiagnosticSeverity::error, "work_plan_policy_read_failed",
                 "unable to read the complete bound policy file", index);
      continue;
    }
    const auto digest = sha256(contents);
    if (!digest) {
      diagnostic(result, WorkPlanDiagnosticSeverity::error, "work_plan_policy_read_failed",
                 "unable to hash bound policy file", index);
      continue;
    }
    if (*digest != policy.sha256) {
      diagnostic(result, WorkPlanDiagnosticSeverity::error, "stale_work_plan_policy_content",
                 "bound policy file content does not match its recorded SHA-256", index);
      continue;
    }
    loaded.push_back({index, *kind, policy.kind, std::move(contents)});
  }
  if (result.has_errors()) {
    return result;
  }

  std::stable_sort(loaded.begin(), loaded.end(),
                   [](const auto& lhs, const auto& rhs) { return lhs.kind < rhs.kind; });
  Board board = source;
  for (const auto& policy : loaded) {
    bool accepted = false;
    switch (policy.kind) {
    case PolicyKind::netclasses: {
      auto applied = apply_kicad_netclass_snapshot(board, policy.contents);
      append_netclass_diagnostics(result, applied.diagnostics, policy.index);
      if (applied.ok()) {
        board = std::move(*applied.board);
        accepted = true;
      }
      break;
    }
    case PolicyKind::differential_pairs:
      accepted = accept_board_result(
          result, apply_differential_pair_constraints(board, policy.contents), board, policy.index);
      break;
    case PolicyKind::placement_envelopes:
      accepted =
          accept_board_result(result, apply_placement_envelope_constraints(board, policy.contents),
                              board, policy.index);
      break;
    case PolicyKind::routing_buses:
      accepted = accept_board_result(result, apply_routing_bus_constraints(board, policy.contents),
                                     board, policy.index);
      break;
    case PolicyKind::placement_groups:
      accepted = accept_board_result(
          result, apply_placement_group_constraints(board, policy.contents), board, policy.index);
      break;
    case PolicyKind::placement_regions:
      accepted = accept_board_result(
          result, apply_placement_region_constraints(board, policy.contents), board, policy.index);
      break;
    case PolicyKind::relative_placements:
      accepted =
          accept_board_result(result, apply_relative_placement_constraints(board, policy.contents),
                              board, policy.index);
      break;
    case PolicyKind::orientation_constraints:
      accepted = accept_board_result(result, apply_orientation_constraints(board, policy.contents),
                                     board, policy.index);
      break;
    case PolicyKind::placement_move_priorities: {
      auto applied = load_placement_move_priorities(board, policy.contents);
      append_constraint_diagnostics(result, applied.diagnostics, policy.index);
      accepted = applied.ok();
      if (accepted) {
        result.placement_move_priorities = std::move(applied.policy);
      }
      break;
    }
    }
    if (!accepted) {
      return result;
    }
    result.applied_policy_kinds.push_back(policy.kind_name);
  }
  result.board = std::move(board);
  return result;
}

} // namespace flux::io
