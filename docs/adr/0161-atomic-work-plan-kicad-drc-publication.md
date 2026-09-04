# ADR 0161: Atomic WorkPlan publication after external KiCad DRC

## Status

Accepted.

## Context

The WorkPlan core executor already required canonical validation, connectivity, represented copper
DRC, and source-preserving export. The CLI then wrote the requested output directly and left
`external_kicad_drc_run` false. That contradicted the repository's rule that KiCad is the final DRC
authority and allowed an internally accepted candidate to appear at its final path before external
verification.

KiCad DRC must see the source project's rule context. It must also run without a command shell so a
board path, executable path, or report path cannot become command syntax.

## Decision

For a non-preview `execute-work-plan-kicad` invocation, create one private staging directory beside
the requested output. Copy matching source `.kicad_pro` and `.kicad_dru` regular files into that
directory and export the candidate under the source board's original filename. The final output is
not created at this point.

Resolve KiCad from an explicit `FLUX_KICAD_CLI` path or from `kicad-cli` on `PATH`. An explicit but
invalid path fails closed and cannot fall back silently. A `.AppImage` is invoked with
`--appimage-extract-and-run kicad-cli`; a normal executable receives the `pcb drc` arguments
directly. Boost.Process passes an argument vector without a shell. Standard output, standard error,
and the JSON report remain inside the staging directory. The process is bounded to fifteen minutes
and observes the WorkPlan stop token.

Require exit code zero and a bounded parseable JSON report. Only then atomically rename the staged
board to the requested path. An unavailable executable, launch error, timeout, cancellation,
nonzero exit status, absent/oversized/invalid report, or rename failure withholds publication and
sets `candidate_ready_for_export` false. The deterministic execution result records whether DRC ran,
whether it passed, its exit code, executable identity, and diagnostic; CLI JSON separately records
whether the output was published. The staging directory is removed on every path.

## Consequences

A final WorkPlan filename now means source-preserving export and external KiCad DRC both succeeded.
Tests use executable paths containing spaces and cover pass, violation, and unavailable-tool states;
the real KiCad 10 AppImage command is also exercised against the minimal board. Preview plans remain
non-mutating and do not invoke DRC.

This gate does not refill zones or claim that Flux's analytic DRC implements every KiCad rule.
Refill-required candidates remain governed by the separate refill workflow, and capability-partial
plans remain blocked before execution.
