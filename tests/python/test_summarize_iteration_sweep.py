"""An interrupted recording must never acquire a fabricated routing verdict."""
import json
from pathlib import Path
import tempfile
import unittest

from summarize_iteration_sweep import summarize


class IncompleteRunTests(unittest.TestCase):
    def test_missing_outcome_requires_explicit_opt_in(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "run-metadata.json").write_text(json.dumps({"revision": "test"}))
            with self.assertRaises(FileNotFoundError):
                summarize(root)
            result = summarize(root, allow_incomplete=True)
            self.assertEqual(result["outcome"]["status"], "not_reported")
            self.assertFalse(result["outcome"]["complete"])
            self.assertEqual(result["planning_records"], [])

    def test_opt_in_preserves_actual_outcome(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "run-metadata.json").write_text("{}")
            outcome = {"status": "failed", "reason": "planning timeout"}
            (root / "run-result.json").write_text(json.dumps(outcome))
            self.assertEqual(summarize(root, allow_incomplete=True)["outcome"], outcome)

    def test_malformed_outcome_is_not_treated_as_missing(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "run-metadata.json").write_text("{}")
            (root / "run-result.json").write_text("{")
            with self.assertRaises(json.JSONDecodeError):
                summarize(root, allow_incomplete=True)

    def test_bundle_search_metrics_and_replay_occurrences_remain_diagnostic(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "run-metadata.json").write_text("{}")
            telemetry = root / "tier-1-synthetic" / "telemetry"
            telemetry.mkdir(parents=True)
            context = {"schema": 1, "source_revision": "test", "phase": "residual-cone",
                       "window": 2, "templates": [{}, {}], "domains": [[0, 1]], "nets": ["n"]}
            event = {"kind": "SearchSummary", "active_group": "regional-bundle-context:residual-cone:2",
                     "message": json.dumps(context)}
            portfolio = {"kind": "SearchSummary", "active_group": "regional-window-portfolio:2",
                         "message": "regional window portfolio: complete_states=0, bundle_states=100000, "
                                    "bundle_search_us=1200, generated_choices=40, packing_rejected=40, "
                                    "phase=residual-cone, stop=4"}
            (telemetry / "synthetic-global.jsonl").write_text(
                "\n".join(json.dumps(e) for e in [event, event, portfolio]) + "\n{")
            result = summarize(root, allow_incomplete=True)
            trace = result["epoch_telemetry"][0]
            self.assertEqual([x["occurrence"] for x in trace["regional_bundle_contexts"]], [1, 2])
            self.assertEqual(trace["regional_bundle_contexts"][0]["domain_sizes"], [2])
            metrics = trace["regional_bundle_portfolios"][0]["metrics"]
            self.assertEqual(metrics["complete_states"], 0)
            self.assertEqual(metrics["generated_choices"], 40)
            self.assertEqual(metrics["packing_rejected"], 40)
            self.assertEqual(metrics["phase"], "residual-cone")
            self.assertEqual(result["outcome"]["status"], "not_reported")


if __name__ == "__main__":
    unittest.main()
