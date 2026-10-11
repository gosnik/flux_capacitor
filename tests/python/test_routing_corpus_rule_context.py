#!/usr/bin/env python3
"""The final KiCad gate must use the source project, including renamed outputs."""

import json
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import verify_routing_corpus as corpus


class RuleContextTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="flux-rule-context-")
        self.addCleanup(self.temporary.cleanup)
        root = Path(self.temporary.name)
        self.source = root / "source.kicad_pcb"
        self.candidate = root / "renamed.kicad_pcb"
        self.report = root / "drc.json"
        self.source.write_text("source board", encoding="utf-8")
        self.candidate.write_text("candidate board", encoding="utf-8")
        self.project = b'{"board":{"design_settings":{"rules":{"min_track_width":0.7}}}}\n'
        self.source.with_suffix(".kicad_pro").write_bytes(self.project)

    def test_expanded_physical_requests_require_a_complete_detailed_plan(self):
        expected = {"global_paths": 50}
        physical = {"paths": 51, "routing_plan_complete": True,
                    "copper_scaffold": {"complete": True, "request_count": 51,
                        "satisfied_connections": 51, "missing_connections": 0,
                        "physical_open_connections": 0, "physical_unrouted_nets": 0}}
        self.assertEqual(corpus.path_inventory_acceptance(physical, expected, "detailed_complete"),
                         "expanded_physical_demand")
        self.assertIsNone(corpus.path_inventory_acceptance(physical, expected, "global_complete"))
        self.assertIsNone(corpus.path_inventory_acceptance(
            dict(physical, routing_plan_complete=False), expected, "detailed_complete"))
        for change in ({"complete": False}, {"request_count": 49},
                       {"satisfied_connections": 50}, {"missing_connections": 1},
                       {"physical_open_connections": 1}, {"physical_unrouted_nets": 1}):
            with self.subTest(change=change):
                broken = dict(physical, copper_scaffold=dict(physical["copper_scaffold"], **change))
                self.assertIsNone(corpus.path_inventory_acceptance(broken, expected, "detailed_complete"))
        self.assertIsNone(corpus.path_inventory_acceptance({"paths": 51}, expected, "detailed_complete"))
        self.assertEqual(corpus.path_inventory_acceptance({"paths": 50}, expected, "global_complete"),
                         "exact_global_path_count")

    def test_straight_angle_audit_checks_exported_decimal_copper(self):
        self.candidate.write_text("""(kicad_pcb
          (segment (start 1.1 2) (end 3.1 2) (width 0.25) (layer "F.Cu"))
          (segment (end 3.1 4) (start 3.1 2) (width 0.25) (layer "F.Cu"))
          (segment (start 3.1 4) (end 3.100001 4.000001) (width 0.25) (layer "F.Cu")))""")
        self.assertTrue(corpus.audit_straight_copper_angles(self.candidate)["passed"])
        with self.candidate.open("a") as out:
            out.write('(segment (start 0 0) (end 1 1.000001) (width 0.25) (layer "F.Cu"))')
        audit = corpus.audit_straight_copper_angles(self.candidate)
        self.assertFalse(audit["passed"])
        self.assertEqual(audit["angles"]["other"], 1)

    def test_drc_sees_original_project_and_custom_rules_before_launch(self):
        rules = b'(version 1)\n(rule "hole spacing" (constraint hole_to_hole (min 0.25mm)))\n'
        self.source.with_suffix(".kicad_dru").write_bytes(rules)

        def run(command, **kwargs):
            self.assertEqual(self.candidate.with_suffix(".kicad_pro").read_bytes(), self.project)
            self.assertEqual(self.candidate.with_suffix(".kicad_dru").read_bytes(), rules)
            self.assertIn("--exit-code-violations", command)
            self.assertIn("--severity-error", command)
            return subprocess.CompletedProcess(command, 0, "", "")

        with patch.object(corpus.subprocess, "run", side_effect=run) as launch:
            corpus.require_kicad_drc(self.candidate, self.report, Path("kicad-cli"), self.source)
        launch.assert_called_once()
        context = json.loads(self.report.with_suffix(".context.json").read_text())
        self.assertEqual(len(context["rule_context"]), 2)
        self.assertEqual(context["candidate_sha256"], corpus.sha256(self.candidate))
        self.assertEqual(self.source.with_suffix(".kicad_pro").read_bytes(), self.project)

    def test_missing_project_fails_closed_without_running_kicad(self):
        self.source.with_suffix(".kicad_pro").unlink()
        with patch.object(corpus.subprocess, "run") as launch:
            with self.assertRaisesRegex(RuntimeError, "missing source KiCad project"):
                corpus.require_kicad_drc(self.candidate, self.report, Path("kicad-cli"), self.source)
        launch.assert_not_called()

    def test_written_preview_cannot_bypass_detailed_curve_completion(self):
        expected = {"min_track_arcs": 1, "max_unrounded_non_junction_bends": 0}
        legal = {"complete": True, "validation_errors": 0, "open_connections": 0,
                 "copper_violations": 0, "added_track_arcs": 2,
                 "unrounded_non_junction_bends": 0, "artifact_written": True}
        self.assertTrue(corpus.detailed_copper_ready_for_final_drc(legal, expected, self.candidate))
        for changes in ({"complete": False}, {"unrounded_non_junction_bends": 11},
                        {"added_track_arcs": 0}, {"open_connections": 1},
                        {"copper_violations": 1}, {"artifact_written": False}):
            with self.subTest(changes=changes):
                preview = dict(legal, **changes, diagnostic_kicad_drc_passed=True)
                self.assertFalse(corpus.detailed_copper_ready_for_final_drc(
                    preview, expected, self.candidate))
        self.assertFalse(corpus.detailed_copper_ready_for_final_drc(
            legal, expected, self.candidate.parent / "missing.kicad_pcb"))

    def test_straight_copper_requires_no_arcs_and_all_legality_gates(self):
        expected = {"max_track_arcs": 0}
        legal = {"complete": True, "validation_errors": 0, "open_connections": 0,
                 "copper_violations": 0, "added_track_arcs": 0,
                 "unrounded_non_junction_bends": 17, "artifact_written": True}
        self.assertTrue(corpus.detailed_copper_ready_for_final_drc(legal, expected, self.candidate))
        for change in ({"added_track_arcs": 1}, {"open_connections": 1},
                       {"copper_violations": 1}, {"complete": False}, {"validation_errors": 1}):
            self.assertFalse(corpus.detailed_copper_ready_for_final_drc(
                dict(legal, **change), expected, self.candidate))

    def test_mismatched_candidate_project_is_not_overwritten(self):
        staged = self.candidate.with_suffix(".kicad_pro")
        staged.write_bytes(b"unrelated project")
        with self.assertRaisesRegex(RuntimeError, "hash mismatch"):
            corpus.stage_kicad_rule_context(self.source, self.candidate)
        self.assertEqual(staged.read_bytes(), b"unrelated project")

    def test_unexpected_candidate_custom_rules_are_rejected(self):
        self.candidate.with_suffix(".kicad_dru").write_bytes(b"unexpected override")
        with self.assertRaisesRegex(RuntimeError, "unexpected candidate rule context"):
            corpus.stage_kicad_rule_context(self.source, self.candidate)

    def test_identical_staged_project_is_idempotent(self):
        first = corpus.stage_kicad_rule_context(self.source, self.candidate)
        self.assertEqual(corpus.stage_kicad_rule_context(self.source, self.candidate), first)
        self.assertFalse(self.candidate.with_suffix(".kicad_dru").exists())

    def test_external_rejection_is_not_converted_to_a_pass(self):
        with patch.object(corpus.subprocess, "run", return_value=
                          subprocess.CompletedProcess([], 5, "Found 208 violations", "")):
            with self.assertRaisesRegex(RuntimeError, "KiCad DRC rejected"):
                corpus.require_kicad_drc(self.candidate, self.report, Path("kicad-cli"), self.source)
        self.assertTrue(self.report.with_suffix(".context.json").is_file())


if __name__ == "__main__":
    unittest.main()
