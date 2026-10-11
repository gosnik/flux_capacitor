#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace flux {

// Stable, pre-realization feature contract shared by Flux and the offline XGBoost trainer.
// Changing the order or meaning requires a new model schema version.
inline constexpr std::size_t regional_candidate_feature_count = 18U;
using RegionalCandidateFeatureVector =
    std::array<double, regional_candidate_feature_count>;

[[nodiscard]] const std::array<std::string_view, regional_candidate_feature_count>&
regional_candidate_feature_names();

// A compact inference-only gradient-boosted tree ensemble. The accepted JSON is deliberately a
// Flux-owned export format rather than XGBoost's internal serialization, so the router has no
// training-library/runtime dependency and rejects feature-schema drift explicitly.
class RegionalCandidateRanker {
public:
  [[nodiscard]] static std::optional<RegionalCandidateRanker>
  load_file(const std::filesystem::path& path, std::string& diagnostic);
  [[nodiscard]] static std::optional<RegionalCandidateRanker>
  load_json(std::string_view contents, std::string& diagnostic);

  [[nodiscard]] double score(const RegionalCandidateFeatureVector& features) const;
  [[nodiscard]] std::size_t tree_count() const { return trees_.size(); }

private:
  struct Node {
    std::size_t feature{0U};
    double threshold{0.0};
    std::size_t yes{0U};
    std::size_t no{0U};
    std::size_t missing{0U};
    double leaf{0.0};
    bool is_leaf{false};
  };
  struct Tree {
    std::vector<Node> nodes;
  };

  double base_score_{0.0};
  std::vector<Tree> trees_;
};

struct RegionalCandidateTrainingObservation {
  std::string board_revision;
  std::string query_id;
  std::size_t window_id{0U};
  std::size_t heuristic_rank{0U};
  std::size_t evaluation_rank{0U};
  RegionalCandidateFeatureVector features{};
  bool model_score_available{false};
  double model_score{0.0};
  bool complete{false};
  std::size_t short_realizations{0U};
  std::size_t topology_rejections{0U};
  std::size_t shape_rejections{0U};
  std::size_t mutual_clearance_rejections{0U};
  std::size_t quantized_clearance_rejections{0U};
  std::uint64_t realization_us{0U};
};

// Appends one independently valid JSON object under a process-wide lock. Collection is opt-in,
// diagnostic-only, and cannot affect candidate acceptance.
[[nodiscard]] bool append_regional_candidate_training_observation(
    const std::filesystem::path& path, const RegionalCandidateTrainingObservation& observation,
    std::string& diagnostic);

} // namespace flux
