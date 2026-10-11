"""Summarize preserved corpus runs without mistaking missing DRC for a pass."""
import argparse
from collections import Counter
import json
from pathlib import Path


def telemetry_events(path):
    # Diagnostic geometry can make these recordings hundreds of MB. Stream them rather
    # than holding both the complete text and its split-line copy during a benchmark.
    with path.open() as stream:
        for line in stream:
            try:
                yield json.loads(line)
            except ValueError:
                continue  # A timeout may leave a partially serialized trailing event.


def regional_portfolio_metrics(message):
    """Decode diagnostic key/value counters without treating discrete feasibility as DRC."""
    result = {}
    for field in message.removeprefix("regional window portfolio: ").split(", "):
        key, separator, value = field.partition("=")
        if separator:
            try:
                value = int(value)
            except ValueError:
                pass
            result[key] = value
    return result


def summarize(root, *, allow_incomplete=False):
    metadata = json.loads((root / "run-metadata.json").read_text())
    outcome_path = root / "run-result.json"
    if allow_incomplete and not outcome_path.exists():
        outcome = {"status": "not_reported", "complete": False,
                   "detail": "No run-result.json; termination cause and final outcome are unknown."}
    else:
        outcome = json.loads(outcome_path.read_text())
    records = []
    for path in sorted(root.glob("tier-*/global-result.json")):
        record = json.loads(path.read_text())
        result = record["result"]
        records.append({
            "artifact": str(path),
            "duration_seconds": record.get("duration_seconds"),
            "metrics": {key: value for key, value in result.items()
                        if not isinstance(value, (dict, list))},
            "quantized_copper_oracle": result.get("quantized_copper_oracle"),
            "feedback_probes": result.get("routability_feedback_probes"),
            "detailed_artifacts": [str(p) for p in path.parent.glob("*detail*json")],
            "drc_artifacts": [str(p) for p in path.parent.rglob("*drc*.json")],
            "final_drc_status": "not_evaluated" if not list(path.parent.rglob("*drc*.json"))
                                else "inspect_drc_artifacts",
        })
    details = []
    for path in sorted(root.glob("tier-*/detail-result.json")):
        record = json.loads(path.read_text())
        result = record["result"]
        details.append({
            "artifact": str(path),
            "duration_seconds": record.get("duration_seconds"),
            "metrics": {key: value for key, value in result.items()
                        if not isinstance(value, (dict, list))},
            "structured_diagnostics": {key: value for key, value in result.items()
                                       if isinstance(value, (dict, list))},
        })
    epochs = []
    for path in sorted(root.glob("tier-*/telemetry/*-global.jsonl")):
        # Also usable after the subprocess timeout, when no final JSON was published.
        events = []
        regional_events = []
        bundle_portfolios = []
        bundle_contexts = []
        context_occurrences = Counter()
        counts = Counter()
        last_snapshot = None
        for event in telemetry_events(path):
            counts[event.get("kind", event.get("type", "unknown"))] += 1
            if "metrics" in event:
                last_snapshot = {"monotonic_time_us": event.get("monotonic_time_us"),
                                 "metrics": event["metrics"]}
            message = event.get("message", "")
            group = event.get("active_group") or ""
            if message.startswith("interleaved routing epoch"):
                events.append({"monotonic_time_us": event.get("monotonic_time_us"),
                               "message": message})
            if event.get("kind") == "SearchSummary" and (
                    group.startswith("regional-analysis:") or
                    group == "regional-frozen-scope-escalation"):
                regional_events.append({"monotonic_time_us": event.get("monotonic_time_us"),
                                        "message": message})
            if group.startswith("regional-window-portfolio:"):
                bundle_portfolios.append({
                    "monotonic_time_us": event.get("monotonic_time_us"),
                    "group": group, "metrics": regional_portfolio_metrics(message)})
            if group.startswith("regional-bundle-context:"):
                context_occurrences[group] += 1
                context = json.loads(message)
                bundle_contexts.append({
                    "monotonic_time_us": event.get("monotonic_time_us"),
                    "group": group, "occurrence": context_occurrences[group],
                    "schema": context["schema"], "phase": context["phase"],
                    "window": context["window"], "source_revision": context["source_revision"],
                    "templates": len(context["templates"]),
                    "domain_sizes": [len(domain) for domain in context["domains"]],
                    "nets": context["nets"]})
        epochs.append({"artifact": str(path), "messages": [e["message"] for e in events],
                       "events": events, "event_counts": dict(counts),
                       "regional_progress": regional_events,
                       "regional_bundle_portfolios": bundle_portfolios,
                       "regional_bundle_contexts": bundle_contexts,
                       "non_authoritative_last_snapshot": last_snapshot})
    return {"root": str(root), "metadata": metadata, "outcome": outcome,
            "planning_records": records, "detailed_records": details,
            "epoch_telemetry": epochs}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", nargs="+", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--experiment", default="iteration-budget-1-4-8")
    parser.add_argument("--note", action="append", default=[])
    parser.add_argument("--allow-incomplete", action="store_true",
                        help="Preserve unfinished telemetry with an explicitly unknown outcome.")
    args = parser.parse_args()
    report = {"schema": 1, "experiment": args.experiment,
              "notes": args.note,
              "runs": [summarize(root, allow_incomplete=args.allow_incomplete) for root in args.runs]}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        json.dump(report, stream, indent=2)
        stream.write("\n")


if __name__ == "__main__":
    main()
