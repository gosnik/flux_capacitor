#include "flux/core/regional_candidate_ranker.hpp"

#include <boost/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <mutex>
#include <set>
#include <sstream>
#include <system_error>

namespace flux {
namespace {

namespace json = boost::json;

constexpr std::size_t maximum_model_bytes = 16U * 1024U * 1024U;
constexpr std::size_t maximum_trees = 4'096U;
constexpr std::size_t maximum_nodes_per_tree = 65'536U;

const json::value* member(const json::object& object, std::string_view name) {
  const auto found = object.find(name);
  return found == object.end() ? nullptr : &found->value();
}

std::optional<std::size_t> unsigned_value(const json::value* value) {
  if (value == nullptr)
    return std::nullopt;
  if (value->is_uint64() && value->as_uint64() <= std::numeric_limits<std::size_t>::max())
    return static_cast<std::size_t>(value->as_uint64());
  if (value->is_int64() && value->as_int64() >= 0 &&
      static_cast<std::uint64_t>(value->as_int64()) <=
          std::numeric_limits<std::size_t>::max())
    return static_cast<std::size_t>(value->as_int64());
  return std::nullopt;
}

std::optional<double> finite_number(const json::value* value) {
  if (value == nullptr || !value->is_number())
    return std::nullopt;
  const auto result = value->to_number<double>();
  return std::isfinite(result) ? std::optional<double>{result} : std::nullopt;
}

template <typename Tree>
bool validate_tree(const Tree& tree, std::string& diagnostic) {
  if (tree.nodes.empty()) {
    diagnostic = "regional candidate ranker contains an empty tree";
    return false;
  }
  enum class Visit : unsigned char { unseen, active, complete };
  std::vector<Visit> visits(tree.nodes.size(), Visit::unseen);
  const auto visit = [&](const auto& self, std::size_t index) -> bool {
    if (index >= tree.nodes.size()) {
      diagnostic = "regional candidate ranker tree references an out-of-range child";
      return false;
    }
    if (visits[index] == Visit::active) {
      diagnostic = "regional candidate ranker tree contains a cycle";
      return false;
    }
    if (visits[index] == Visit::complete)
      return true;
    visits[index] = Visit::active;
    const auto& node = tree.nodes[index];
    if (!node.is_leaf &&
        (!self(self, node.yes) || !self(self, node.no) || !self(self, node.missing))) {
      return false;
    }
    visits[index] = Visit::complete;
    return true;
  };
  if (!visit(visit, 0U))
    return false;
  if (std::ranges::any_of(visits, [](const auto state) { return state == Visit::unseen; })) {
    diagnostic = "regional candidate ranker tree contains unreachable nodes";
    return false;
  }
  return true;
}

} // namespace

const std::array<std::string_view, regional_candidate_feature_count>&
regional_candidate_feature_names() {
  static constexpr std::array<std::string_view, regional_candidate_feature_count> names{
      "heuristic_rank",
      "template_count",
      "via_count",
      "cross_section_count",
      "via_alternative_switches",
      "template_conflicts",
      "residual_conflicts",
      "alternative_template_count",
      "selected_global_template_count",
      "distinct_layer_count",
      "total_channel_steps",
      "total_layer_transitions",
      "total_topology_cost",
      "total_lower_bound_length_mm",
      "minimum_lane_slack_mm",
      "contested_via_count",
      "maximum_via_competitors",
      "bundle_search_states",
  };
  return names;
}

std::optional<RegionalCandidateRanker>
RegionalCandidateRanker::load_file(const std::filesystem::path& path, std::string& diagnostic) {
  diagnostic.clear();
  std::error_code error;
  const auto size = std::filesystem::file_size(path, error);
  if (error || size == 0U || size > maximum_model_bytes) {
    diagnostic = error ? "could not inspect regional candidate ranker model: " + error.message()
                       : "regional candidate ranker model has an invalid bounded size";
    return std::nullopt;
  }
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    diagnostic = "could not open regional candidate ranker model";
    return std::nullopt;
  }
  std::string contents(static_cast<std::size_t>(size), '\0');
  input.read(contents.data(), static_cast<std::streamsize>(contents.size()));
  if (!input || input.gcount() != static_cast<std::streamsize>(contents.size())) {
    diagnostic = "could not read complete regional candidate ranker model";
    return std::nullopt;
  }
  return load_json(contents, diagnostic);
}

std::optional<RegionalCandidateRanker>
RegionalCandidateRanker::load_json(std::string_view contents, std::string& diagnostic) {
  diagnostic.clear();
  boost::system::error_code error;
  const auto document = json::parse(contents, error);
  if (error || !document.is_object()) {
    diagnostic = "regional candidate ranker model is not valid JSON object";
    return std::nullopt;
  }
  const auto& root = document.as_object();
  const auto schema = unsigned_value(member(root, "schema_version"));
  if (!schema || *schema != 1U) {
    diagnostic = "regional candidate ranker model has an unsupported schema version";
    return std::nullopt;
  }
  const auto* names_value = member(root, "feature_names");
  if (names_value == nullptr || !names_value->is_array() ||
      names_value->as_array().size() != regional_candidate_feature_count) {
    diagnostic = "regional candidate ranker model feature schema has the wrong size";
    return std::nullopt;
  }
  const auto& expected_names = regional_candidate_feature_names();
  for (std::size_t index = 0U; index < expected_names.size(); ++index) {
    const auto& value = names_value->as_array()[index];
    if (!value.is_string() || value.as_string() != expected_names[index]) {
      diagnostic = "regional candidate ranker model feature schema does not match Flux";
      return std::nullopt;
    }
  }
  const auto base_score = finite_number(member(root, "base_score"));
  const auto* trees_value = member(root, "trees");
  if (!base_score || trees_value == nullptr || !trees_value->is_array() ||
      trees_value->as_array().empty() || trees_value->as_array().size() > maximum_trees) {
    diagnostic = "regional candidate ranker model has invalid ensemble metadata";
    return std::nullopt;
  }

  RegionalCandidateRanker result;
  result.base_score_ = *base_score;
  result.trees_.reserve(trees_value->as_array().size());
  for (const auto& tree_value : trees_value->as_array()) {
    if (!tree_value.is_object()) {
      diagnostic = "regional candidate ranker tree is not an object";
      return std::nullopt;
    }
    const auto* nodes_value = member(tree_value.as_object(), "nodes");
    if (nodes_value == nullptr || !nodes_value->is_array() || nodes_value->as_array().empty() ||
        nodes_value->as_array().size() > maximum_nodes_per_tree) {
      diagnostic = "regional candidate ranker tree has invalid node storage";
      return std::nullopt;
    }
    Tree tree;
    tree.nodes.resize(nodes_value->as_array().size());
    std::vector<bool> populated(tree.nodes.size(), false);
    for (const auto& node_value : nodes_value->as_array()) {
      if (!node_value.is_object()) {
        diagnostic = "regional candidate ranker node is not an object";
        return std::nullopt;
      }
      const auto& object = node_value.as_object();
      const auto id = unsigned_value(member(object, "id"));
      if (!id || *id >= tree.nodes.size() || populated[*id]) {
        diagnostic = "regional candidate ranker node ID is invalid or duplicated";
        return std::nullopt;
      }
      auto& node = tree.nodes[*id];
      populated[*id] = true;
      if (const auto leaf = finite_number(member(object, "leaf"))) {
        node.leaf = *leaf;
        node.is_leaf = true;
        continue;
      }
      const auto feature = unsigned_value(member(object, "feature"));
      const auto threshold = finite_number(member(object, "threshold"));
      const auto yes = unsigned_value(member(object, "yes"));
      const auto no = unsigned_value(member(object, "no"));
      const auto missing = unsigned_value(member(object, "missing"));
      if (!feature || *feature >= regional_candidate_feature_count || !threshold || !yes || !no ||
          !missing) {
        diagnostic = "regional candidate ranker split node is incomplete";
        return std::nullopt;
      }
      node.feature = *feature;
      node.threshold = *threshold;
      node.yes = *yes;
      node.no = *no;
      node.missing = *missing;
    }
    if (std::ranges::any_of(populated, std::logical_not{})) {
      diagnostic = "regional candidate ranker node IDs are not dense";
      return std::nullopt;
    }
    if (!validate_tree(tree, diagnostic))
      return std::nullopt;
    result.trees_.push_back(std::move(tree));
  }
  return result;
}

double RegionalCandidateRanker::score(const RegionalCandidateFeatureVector& features) const {
  auto result = static_cast<long double>(base_score_);
  for (const auto& tree : trees_) {
    std::size_t index = 0U;
    for (std::size_t steps = 0U; steps < tree.nodes.size(); ++steps) {
      const auto& node = tree.nodes[index];
      if (node.is_leaf) {
        result += static_cast<long double>(node.leaf);
        break;
      }
      const auto value = features[node.feature];
      index = !std::isfinite(value) ? node.missing : value < node.threshold ? node.yes : node.no;
    }
  }
  const auto maximum = static_cast<long double>(std::numeric_limits<double>::max());
  return static_cast<double>(std::clamp(result, -maximum, maximum));
}

bool append_regional_candidate_training_observation(
    const std::filesystem::path& path, const RegionalCandidateTrainingObservation& observation,
    std::string& diagnostic) {
  static std::mutex output_mutex;
  std::scoped_lock lock(output_mutex);
  diagnostic.clear();
  std::ofstream output(path, std::ios::binary | std::ios::app);
  if (!output) {
    diagnostic = "could not append regional candidate training data";
    return false;
  }
  json::object document;
  document["schema_version"] = 1;
  document["board_revision"] = observation.board_revision;
  document["query_id"] = observation.query_id;
  document["window_id"] = observation.window_id;
  document["heuristic_rank"] = observation.heuristic_rank;
  document["evaluation_rank"] = observation.evaluation_rank;
  json::object features;
  const auto& names = regional_candidate_feature_names();
  for (std::size_t index = 0U; index < names.size(); ++index)
    features[names[index]] = observation.features[index];
  document["features"] = std::move(features);
  document["model_score_available"] = observation.model_score_available;
  if (observation.model_score_available)
    document["model_score"] = observation.model_score;
  document["complete"] = observation.complete;
  document["short_realizations"] = observation.short_realizations;
  document["topology_rejections"] = observation.topology_rejections;
  document["shape_rejections"] = observation.shape_rejections;
  document["mutual_clearance_rejections"] = observation.mutual_clearance_rejections;
  document["quantized_clearance_rejections"] = observation.quantized_clearance_rejections;
  document["realization_us"] = observation.realization_us;
  output << json::serialize(document) << '\n';
  if (!output) {
    diagnostic = "could not write complete regional candidate training observation";
    return false;
  }
  return true;
}

} // namespace flux
