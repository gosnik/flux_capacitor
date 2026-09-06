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
