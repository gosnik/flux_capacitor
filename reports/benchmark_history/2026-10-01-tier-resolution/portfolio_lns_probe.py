"""Bounded exact-copper portfolio experiment; cannot qualify a routing tier.

Only complete retained whole-net trees are replaced. All other selected paths
stay fixed, and every trial rechecks connectivity and copper under source rules.
"""
import argparse
import copy
import hashlib
import itertools
import json
from pathlib import Path
import subprocess
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--board", required=True, type=Path)
    parser.add_argument("--telemetry", required=True, type=Path)
    parser.add_argument("--sequence", required=True, type=int)
    parser.add_argument("--oracle", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--max-checks", type=int, default=150)
    parser.add_argument("--max-rounds", type=int, default=3)
    args = parser.parse_args()
    if args.max_checks < 1 or args.max_rounds < 1:
        parser.error("positive search bounds required")
    args.output.mkdir(parents=True, exist_ok=False)
    with args.telemetry.open() as source:
        snapshot = next(json.loads(line) for line in source
                        if '"snapshot"' in line and
                        (event := json.loads(line)).get("type") == "snapshot" and
                        event.get("sequence") == args.sequence)
    selected = {route["id"]: route for route in snapshot["routes"]
                if route["id"].startswith("global-route:") and
                route["family"] == "selected" and route["lifecycle"] == "active"}
    expected = {}
    for route in selected.values():
        expected.setdefault(route["net"], set()).add(int(route["id"].rsplit(":", 1)[1]))
    portfolios = {}
    for route in snapshot["routes"]:
        if not route["id"].startswith("coarse:") or route["lifecycle"] != "retained_fallback":
            continue
        _, connection, slot = route["id"].rsplit(":", 2)
        portfolios.setdefault(route["net"], {}).setdefault(int(slot), {})[int(connection)] = route
    for net, slots in portfolios.items():
        for slot in list(slots):
            if set(slots[slot]) != expected.get(net, set()) or len(slots[slot]) == 0:
                del slots[slot]
        # The same topology can appear under multiple seed families/slots.
        seen = set()
        for slot in list(slots):
            signature = json.dumps([(c, r["points"]) for c, r in sorted(slots[slot].items())], sort_keys=True)
            if signature in seen:
                del slots[slot]
            seen.add(signature)
    started = time.monotonic()
    checks = 0
    rows = []

    def probe(scene, label):
        nonlocal checks
        path = args.output / "trial.jsonl"
        path.write_text(json.dumps(scene) + "\n")
        command = [str(args.oracle.resolve()), str(args.board.resolve()), str(path.resolve()), str(args.sequence)]
        completed = subprocess.run(command, capture_output=True, text=True, check=True, timeout=60)
        result = json.loads(completed.stdout.splitlines()[-1])
        if result["pitch_nm"] != 1 or result["paths"] != len(selected):
            raise RuntimeError("oracle coverage or lattice changed")
        checks += 1
        row = {"check": checks, "label": label, **result}
        rows.append(row)
        with (args.output / "trials.jsonl").open("a") as stream:
            stream.write(json.dumps(row) + "\n")
        return result

    def objective(result):
        valid = result["malformed"] == 0 and result["materialized"] == result["paths"]
        return (not valid, result["opens"], result["drc_violations"], result["vias"])

    incumbent = snapshot
    proof = probe(incumbent, "baseline")
    before = proof
    changes = []
    for round_index in range(args.max_rounds):
        improved = False
        pairs = []
        for group in proof["largest_pair_groups"]:
            first, second, layer = group["net_pair_and_layer"].split(":")
            if (first, second) not in pairs and portfolios.get(first) and portfolios.get(second):
                pairs.append((first, second))
        for first, second in pairs:
            candidate_scene, candidate_proof, candidate_choice = incumbent, proof, None
            for first_slot, second_slot in itertools.product(sorted(portfolios[first]), sorted(portfolios[second])):
                if checks >= args.max_checks:
                    break
                scene = copy.deepcopy(incumbent)
                for route in scene["routes"]:
                    if route["id"] not in selected or route["net"] not in (first, second):
                        continue
                    connection = int(route["id"].rsplit(":", 1)[1])
                    replacement = portfolios[route["net"]][first_slot if route["net"] == first else second_slot][connection]
                    route["points"] = replacement["points"]
                trial = probe(scene, {"round": round_index, "nets": [first, second],
                                      "slots": [first_slot, second_slot]})
                if objective(trial) < objective(candidate_proof):
                    candidate_scene, candidate_proof = scene, trial
                    candidate_choice = {"nets": [first, second], "slots": [first_slot, second_slot]}
            if candidate_choice is not None:
                changes.append({"round": round_index, **candidate_choice,
                                "before_errors": proof["drc_violations"],
                                "after_errors": candidate_proof["drc_violations"],
                                "opens": candidate_proof["opens"]})
                incumbent, proof = candidate_scene, candidate_proof
                improved = True
                (args.output / "incumbent.jsonl").write_text(json.dumps(incumbent) + "\n")
            if checks >= args.max_checks:
                break
        if not improved or checks >= args.max_checks:
            break
    summary = {"schema": 1, "authoritative": False, "tier_qualified": False,
               "algorithm": "bounded exact-copper complete-net portfolio neighborhood",
               "board": str(args.board), "sequence": args.sequence,
               "board_sha256": hashlib.sha256(args.board.read_bytes()).hexdigest(),
               "project_sha256": hashlib.sha256(args.board.with_suffix(".kicad_pro").read_bytes()).hexdigest(),
               "oracle_sha256": hashlib.sha256(args.oracle.read_bytes()).hexdigest(),
               "checks": checks, "seconds": time.monotonic() - started,
               "before": before, "after": proof, "changes": changes,
               "limitations": "Coarse snapshot only; no resource reallocation, regional native fitting, straight-angle proof or KiCad DRC qualification."}
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({"checks": checks, "seconds": summary["seconds"],
                      "before_errors": before["drc_violations"], "after_errors": proof["drc_violations"],
                      "opens": proof["opens"], "accepted_changes": len(changes)}, indent=2))


if __name__ == "__main__":
    main()
