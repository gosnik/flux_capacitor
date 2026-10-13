"""Exhausting routing must return a retained preview without starting another curve fit."""
import json
import re
import subprocess
import sys
import tempfile
from pathlib import Path


def main():
    cli, fixture = map(Path, sys.argv[1:3])
    with tempfile.TemporaryDirectory(prefix="flux-retained-preview-") as directory:
        root = Path(directory)
        source = root / "source.kicad_pcb"
        # The fixture has one straight segment connecting two pads. Leave its keepout,
        # footprints and outline intact, and require the engine to produce the connection.
        text, removed = re.subn(r"\n  \(segment\n.*?\n  \)", "", fixture.read_text(), flags=re.S)
        assert removed == 1, "fixture must have one removable copper connection"
        source.write_text(text)
        provisional_output = root / "provisional.kicad_pcb"
        provisional = subprocess.run(
            [str(cli.resolve()), "detail-kicad", str(fixture.resolve()),
             str(provisional_output), "--refillable-zones", "--routing-timeout-ms", "60000"],
            capture_output=True, text=True, timeout=90)
        provisional_document = json.loads(provisional.stdout)
        assert provisional.returncode == 5, provisional.stderr + provisional.stdout
        assert provisional_document["provisional_complete"] is True
        assert provisional_document["complete"] is False
        assert provisional_document["artifact_preview_only"] is True
        assert provisional_document["zone_refill_required"] is True
        assert provisional_output.exists()
        assert "(filled_polygon" not in provisional_output.read_text()
        for actions, expect_preview in [(1, False), (2, True)]:
            output = root / f"preview-{actions}.kicad_pcb"
            result = subprocess.run(
                [str(cli.resolve()), "detail-kicad", str(source), str(output),
                 "--refillable-zones", "--write-incomplete-preview", "--curved-tracks",
                 "--routing-max-actions", str(actions), "--routing-timeout-ms", "60000"],
                capture_output=True, text=True, timeout=90)
            document = json.loads(result.stdout)
            assert result.returncode == 5, result.stderr + result.stdout
            assert document["routing_engine_outcome"] == "exhausted", document
            assert document["complete"] is False and document["provisional_complete"] is False
            assert document["candidate_is_retained_scaffold_preview"] is expect_preview
            assert output.exists() is expect_preview
            if expect_preview:
                assert document["candidate_final_validation_performed"] is False
                for metric in ["open_connections", "copper_violations", "validation_errors"]:
                    assert document[metric] is None, (metric, document[metric])
                assert document["added_tracks"] > 0
                assert "unassessed" in document["diagnostic"]
    print("retained preview exhaustion checks passed")


if __name__ == "__main__":
    main()
