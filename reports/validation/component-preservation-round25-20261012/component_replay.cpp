// Offline proof experiment: no nearest-pad inference and no router source changes.
#include "flux/core/copper_transaction.hpp"
#include "../../../src/scaffold_terminal_rebind.hpp"

#define main round22_embedded_main
#include "../exchange-evidence-round22-20261012/replay_joint_repair.cpp"
#undef main

namespace {

std::vector<GlobalRoutePath> all_requests(const value& global) {
  const auto& evidence = global.as_object().at("result").as_object().at("copper_scaffold").as_object()
      .at("request_evidence").as_array();
  std::vector<GlobalRoutePath> requests; requests.reserve(evidence.size());
  for (const auto& item : evidence) {
    const auto& record = item.as_object(); auto request = parse_identity(text(record.at("request_identity")));
    if (request.net.value() != text(record.at("net")) || request.connection_index != record.at("connection_index").as_int64())
      throw std::runtime_error("canonical request identity does not match evidence metadata");
    requests.push_back(std::move(request));
  }
  return requests;
}

CopperScaffoldConfig bounded_config() {
  CopperScaffoldConfig config;
  config.maximum_expansions_per_connection = 50'000U;
  config.compact_search_expansions = 25'000U;
  config.maximum_victim_exchanges = 0U;
  config.maximum_joint_passage_candidates = 0U;
  config.maximum_displacement_attempts = 0U;
  config.maximum_coupled_victims = 1U;
  return config;
}

void json_point(std::ostream& out, const GlobalRoutePoint& point) {
  out << "{\"x_nm\":" << point.position.x.value() << ",\"y_nm\":" << point.position.y.value()
      << ",\"layer\":"; json_string(out, point.layer.value()); out << '}';
}
void json_local(std::ostream& out, const CopperScaffoldResult& result) {
  out << "{\"complete\":" << (result.complete ? "true" : "false") << ",\"satisfied\":";
  json_indices(out, result.satisfied_indices); out << ",\"missing\":"; json_indices(out, result.missing_indices);
  out << ",\"evidence\":[";
  for (std::size_t i = 0; i < result.request_evidence.size(); ++i) {
    if (i) out << ','; const auto& evidence = result.request_evidence[i];
    out << "{\"termination\":"; json_string(out, std::string(to_string(evidence.termination)));
    out << ",\"blockers\":"; json_blockers(out, evidence);
    out << ",\"attachments\":"; json_attachments(out, evidence); out << '}';
  }
  out << "]}";
}

struct BranchScenario { const char* name; std::size_t incoming; };

}  // namespace

int main(int argc, char** argv) {
  if (argc != 4) return 2;
  try {
    const auto imported = io::import_kicad_pcb_file(argv[1]);
    if (!imported.ok()) throw std::runtime_error("cannot import PCB");
    const auto global = read_global_result(argv[2]);
    const auto requests = all_requests(global);
    if (requests.size() <= 91U) throw std::runtime_error("global evidence does not contain required requests");
    const auto copper = latest_generated_primitives(argv[3]);
    Board saved = *imported.board; for (const auto& primitive : copper) add_generated(saved, primitive);
    const auto baseline_drc = check_copper_rules(saved);
    const std::array<BranchScenario, 2U> scenarios{{{"incoming-44", 44U}, {"incoming-91", 91U}}};
    std::cout << "{\"outcomes\":[";
    for (std::size_t ordinal = 0; ordinal < scenarios.size(); ++ordinal) {
      if (ordinal) std::cout << ','; const auto& scenario = scenarios[ordinal];
      Board ripped = saved; erase_owner(ripped, 11U); erase_owner(ripped, 19U);
      const auto incoming = construct_copper_scaffold(ripped, {requests.at(scenario.incoming)}, bounded_config());
      const auto eleven = construct_copper_scaffold(incoming.copper, {requests.at(11U)}, bounded_config());
      const auto failed_nineteen = construct_copper_scaffold(eleven.copper, {requests.at(19U)}, bounded_config());

      CopperScaffoldResult retained;
      retained.requests = requests;
      retained.copper = eleven.copper;
      retained.request_count = requests.size();
      retained.missing_indices = {19U};
      retained.satisfied_indices = {11U};
      retained.request_evidence.resize(requests.size());
      retained.request_evidence[19U] = failed_nineteen.request_evidence.at(0U);
      retained.request_evidence[19U].request_identity = text(global.as_object().at("result").as_object()
          .at("copper_scaffold").as_object().at("request_evidence").as_array().at(19U).as_object().at("request_identity"));
      retained.request_evidence[19U].net = requests.at(19U).net;
      retained.request_evidence[19U].connection_index = requests.at(19U).connection_index;
      const auto rebound = derive_scaffold_terminal_rebinds(retained.copper, retained, {},
                                                            "round24-saved-copper-replay");
      const auto matching = std::find_if(rebound.second.begin(), rebound.second.end(), [](const auto& proof) {
        return proof.request_index == 19U;
      });
      CopperScaffoldResult retry;
      std::optional<CopperTransactionReport> transaction;
      if (matching != rebound.second.end()) {
        CopperTransactionValidator validator(saved);
        retry = construct_copper_scaffold(eleven.copper, {rebound.first.at(19U)}, bounded_config());
        transaction = validator.validate(retry.copper);
      }
      const auto final_board = matching != rebound.second.end() ? retry.copper : eleven.copper;
      const auto before_components = analyze_connectivity(saved);
      const auto after_components = analyze_connectivity(final_board);
      std::map<std::pair<std::string,std::size_t>,std::map<std::size_t,std::vector<std::string>>> partitions;
      for(const auto& pad:before_components.pad_components) {
        const auto found=std::find_if(after_components.pad_components.begin(),after_components.pad_components.end(),[&](const auto& p){return p.net==pad.net && p.pad==pad.pad;});
        if(found!=after_components.pad_components.end()) partitions[{pad.net.value(),pad.component}][found->component].push_back(pad.pad.value());
      }
      const auto drc = check_copper_rules(final_board);
      const auto new_drc = new_drc_violations(baseline_drc, drc);
      std::cout << "{\"scenario\":"; json_string(std::cout, scenario.name);
      std::cout << ",\"incoming\":"; json_local(std::cout, incoming);
      std::cout << ",\"victim_11\":"; json_local(std::cout, eleven);
      std::cout << ",\"failed_19\":"; json_local(std::cout, failed_nineteen);
      std::cout << ",\"rebind_proven\":" << (matching != rebound.second.end() ? "true" : "false");
      if (matching != rebound.second.end()) {
        std::cout << ",\"proof\":"; json_string(std::cout, matching->proof);
        std::cout << ",\"original_endpoint\":"; json_point(std::cout, matching->original_endpoint);
        std::cout << ",\"proven_pad_terminal\":"; json_point(std::cout, matching->terminal);
        std::cout << ",\"rebound_19\":"; json_local(std::cout, retry);
        std::cout << ",\"existing_pad_components_preserved\":"
                  << (transaction->preserves_pad_components ? "true" : "false");
      }
      std::cout << ",\"split_components\":[";
      bool first_split=true;
      for(const auto& [key,groups]:partitions) if(groups.size()>1) {
        if(!first_split)std::cout<<','; first_split=false;
        std::cout<<"{\"net\":";json_string(std::cout,key.first);std::cout<<",\"pad_groups\":[";
        bool first_group=true;for(const auto& [component,pads]:groups) {if(!first_group)std::cout<<',';first_group=false;std::cout<<'[';bool first_pad=true;for(const auto& pad:pads){if(!first_pad)std::cout<<',';first_pad=false;json_string(std::cout,pad);}std::cout<<']';}std::cout<<"]}";
      }
      std::cout<<']';
      std::cout << ",\"new_exact_drc_violations\":"; json_drc_violations(std::cout, new_drc);
      std::cout << '}';
    }
    std::cout << "]}\n";
  } catch (const std::exception& error) {
    std::cout << "{\"error\":"; json_string(std::cout, error.what()); std::cout << "}\n"; return 1;
  }
}
