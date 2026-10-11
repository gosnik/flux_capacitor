#!/usr/bin/env python3
"""Train and export Flux's advisory regional candidate ranker.

XGBoost is a training-only dependency. The output is a bounded Flux-owned JSON tree ensemble that
the C++ router evaluates directly; no Python or XGBoost library is used while routing.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable


FEATURE_NAMES = [
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
]


def relevance(row: dict[str, Any]) -> int:
    """Encode Flux's downstream feasibility order as graded ranking relevance."""
    if row["complete"]:
        return 4
    if any(row[name] for name in ("short_realizations", "shape_rejections")):
        return 0
    if row["topology_rejections"]:
        return 1
    if row["mutual_clearance_rejections"]:
        return 2
    if row["quantized_clearance_rejections"]:
        return 3
    return 3


def read_rows(paths: Iterable[Path]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for path in paths:
        with path.open("r", encoding="utf-8") as stream:
            for line_number, line in enumerate(stream, 1):
                if not line.strip():
                    continue
                row = json.loads(line)
                if row.get("schema_version") != 1:
                    raise ValueError(f"{path}:{line_number}: unsupported observation schema")
                features = row.get("features")
                if not isinstance(features, dict) or set(features) != set(FEATURE_NAMES):
                    raise ValueError(f"{path}:{line_number}: feature schema mismatch")
                values = [float(features[name]) for name in FEATURE_NAMES]
                if not all(math.isfinite(value) for value in values):
                    raise ValueError(f"{path}:{line_number}: non-finite feature")
                row["feature_vector"] = values
                row["relevance"] = relevance(row)
                rows.append(row)
    if not rows:
        raise ValueError("no training observations were found")
    return rows


def usable_groups(rows: list[dict[str, Any]]) -> list[list[dict[str, Any]]]:
    grouped: dict[str, list[dict[str, Any]]] = {}
    for row in rows:
        grouped.setdefault(str(row["query_id"]), []).append(row)
    groups = [
        sorted(group, key=lambda row: int(row["evaluation_rank"]))
        for group in grouped.values()
        if len(group) >= 2 and len({row["relevance"] for row in group}) >= 2
    ]
    if not groups:
        raise ValueError("training requires a query with at least two differently labeled candidates")
    return sorted(groups, key=lambda group: str(group[0]["query_id"]))


def validation_board(revision: str, fraction: float) -> bool:
    digest = hashlib.sha256(revision.encode("utf-8")).digest()
    bucket = int.from_bytes(digest[:8], "big") / float(1 << 64)
    return bucket < fraction


def make_matrix(xgb: Any, groups: list[list[dict[str, Any]]]) -> Any:
    import numpy as np

    flat = [row for group in groups for row in group]
    matrix = xgb.DMatrix(
        np.asarray([row["feature_vector"] for row in flat], dtype=np.float32),
        label=np.asarray([row["relevance"] for row in flat], dtype=np.float32),
        feature_names=FEATURE_NAMES,
    )
    matrix.set_group([len(group) for group in groups])
    return matrix


def flatten_tree(root: dict[str, Any]) -> dict[str, Any]:
    nodes: dict[int, dict[str, Any]] = {}

    def visit(node: dict[str, Any]) -> None:
        node_id = int(node["nodeid"])
        if "leaf" in node:
            nodes[node_id] = {"id": node_id, "leaf": float(node["leaf"])}
            return
        split = str(node["split"])
        if split in FEATURE_NAMES:
            feature = FEATURE_NAMES.index(split)
        elif split.startswith("f") and split[1:].isdigit():
            feature = int(split[1:])
        else:
            raise ValueError(f"unrecognized XGBoost split feature {split!r}")
        nodes[node_id] = {
            "id": node_id,
            "feature": feature,
            "threshold": float(node["split_condition"]),
            "yes": int(node["yes"]),
            "no": int(node["no"]),
            "missing": int(node["missing"]),
        }
        for child in node.get("children", []):
            visit(child)

    visit(root)
    original_ids = sorted(nodes)
    remap = {node_id: index for index, node_id in enumerate(original_ids)}
    ordered = []
    for original_id in original_ids:
        node = dict(nodes[original_id])
        node["id"] = remap[original_id]
        for edge in ("yes", "no", "missing"):
            if edge in node:
                node[edge] = remap[node[edge]]
        ordered.append(node)
    return {"nodes": ordered}


def score_exported(model: dict[str, Any], values: list[float]) -> float:
    score = float(model["base_score"])
    for tree in model["trees"]:
        nodes = tree["nodes"]
        index = 0
        while "leaf" not in nodes[index]:
            node = nodes[index]
            value = values[node["feature"]]
            index = node["missing"] if not math.isfinite(value) else (
                node["yes"] if value < node["threshold"] else node["no"]
            )
        score += nodes[index]["leaf"]
    return score


def top_one_accuracy(groups: list[list[dict[str, Any]]], scores: list[float]) -> float:
    correct = 0
    offset = 0
    for group in groups:
        group_scores = scores[offset : offset + len(group)]
        selected = max(range(len(group)), key=lambda index: (group_scores[index], -index))
        correct += group[selected]["relevance"] == max(row["relevance"] for row in group)
        offset += len(group)
    return correct / len(groups)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("inputs", nargs="+", type=Path, help="Flux observation JSONL files")
    parser.add_argument("--output", required=True, type=Path, help="exported Flux model JSON")
    parser.add_argument("--rounds", type=int, default=160)
    parser.add_argument("--max-depth", type=int, default=5)
    parser.add_argument("--learning-rate", type=float, default=0.05)
    parser.add_argument("--validation-board-fraction", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=20260905)
    args = parser.parse_args()
    if args.rounds <= 0 or args.max_depth <= 0 or not 0.0 < args.learning_rate <= 1.0:
        parser.error("invalid bounded training parameters")
    if not 0.0 <= args.validation_board_fraction < 1.0:
        parser.error("validation fraction must be in [0, 1)")

    try:
        import numpy as np
        import xgboost as xgb
    except ImportError as error:
        parser.error(f"training requires numpy and xgboost: {error}")

    groups = usable_groups(read_rows(args.inputs))
    train = [
        group
        for group in groups
        if not validation_board(str(group[0]["board_revision"]), args.validation_board_fraction)
    ]
    validation = [group for group in groups if group not in train]
    if not train:
        train, validation = groups, []

    train_matrix = make_matrix(xgb, train)
    evaluations = [(train_matrix, "train")]
    validation_matrix = make_matrix(xgb, validation) if validation else None
    if validation_matrix is not None:
        evaluations.append((validation_matrix, "board_validation"))
    parameters = {
        "objective": "rank:ndcg",
        "eval_metric": "ndcg@1",
        "tree_method": "hist",
        "max_depth": args.max_depth,
        "eta": args.learning_rate,
        "min_child_weight": 0.0,
        "subsample": 0.9,
        "colsample_bytree": 0.9,
        "seed": args.seed,
        "nthread": 1,
    }
    booster = xgb.train(parameters, train_matrix, args.rounds, evals=evaluations, verbose_eval=False)
    trees = [flatten_tree(json.loads(tree)) for tree in booster.get_dump(dump_format="json")]
    train_margin = booster.predict(train_matrix, output_margin=True)
    tree_only = [
        score_exported({"base_score": 0.0, "trees": trees}, row["feature_vector"])
        for group in train
        for row in group
    ]
    residuals = train_margin - np.asarray(tree_only)
    base_score = float(residuals[0])
    if not np.allclose(residuals, base_score, rtol=1e-6, atol=1e-6):
        raise RuntimeError("exported tree ensemble does not reproduce XGBoost margins")

    model = {
        "schema_version": 1,
        "producer": "tools/train_regional_candidate_ranker.py",
        "xgboost_version": xgb.__version__,
        "objective": "rank:ndcg",
        "feature_names": FEATURE_NAMES,
        "base_score": base_score,
        "trees": trees,
    }
    exported_train_scores = [
        score_exported(model, row["feature_vector"]) for group in train for row in group
    ]
    if not np.allclose(exported_train_scores, train_margin, rtol=1e-5, atol=1e-5):
        raise RuntimeError("final exported model failed inference equivalence")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(model, separators=(",", ":")) + "\n", encoding="utf-8")
    report = {
        "training_groups": len(train),
        "training_rows": sum(map(len, train)),
        "training_top1": top_one_accuracy(train, exported_train_scores),
        "validation_groups": len(validation),
        "trees": len(trees),
        "output": str(args.output),
    }
    if validation_matrix is not None:
        validation_scores = booster.predict(validation_matrix, output_margin=True).tolist()
        report["validation_top1"] = top_one_accuracy(validation, validation_scores)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
