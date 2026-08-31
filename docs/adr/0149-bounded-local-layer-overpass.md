# ADR 0149: Bounded local layer overpasses

## Status

Accepted after ADR 0148.

## Context

An alternate-layer regional template previously searched via sites around the two endpoints of a
complete straight fragment. That can move much more copper than a local physical conflict
requires, consume scarce terminal escape area, and miss the ordinary PCB repair of entering
another layer immediately before a crossing and returning immediately after it.

The first implementation attempt put conflict-local sites ahead of the established endpoint
portfolio. On the isolated ESC this reduced accepted topology alternatives and regressed complete
regional windows from 272 to 262. A local repair must not consume or reorder the portfolio whose
fallback is already proven.

## Decision

For a single-conflict regional window, project the attributed conflict bounds onto each eligible
owned route fragment. Generate a bounded sequence of via pairs immediately before and after that
projected interval, using the larger of the NavMesh pitch and the complete via-diameter-plus-
clearance envelope as the longitudinal step.

The two sites must form a non-empty forward run. The source-layer run-ups, complete via spans, and
alternate-layer centerline must independently pass the existing represented free-space checks.
The alternate-layer section uses the prepared NavMesh, micromouse flood, funnel, and B-spline
pipeline. Materialization retains the unchanged original-layer copper outside the two vias.

Local overpasses have a separate 512-check board-wide budget and reserve at most four additional
templates per fragment. The established ordinary and joint endpoint portfolios are generated
first under their unchanged limits. The overpass wave therefore cannot reduce their search space,
alter their counters, or become authoritative merely by being shorter. The complete regional
spline shortlist and exact whole-board transaction remain the acceptance boundary.

CLI diagnostics publish attempted, accepted, and exhausted local-overpass counters.

## Consequences

On the unchanged isolated ESC checkpoint, the router still connects all 320 coarse paths with no
unmapped physical conflicts. It performs 20 local-overpass checks and accepts 12 templates. The
qualified result improves from 272/284 to 273/284 complete regional windows, removes the last
discrete no-fit window, and reduces retained mutual-clearance failures from 27 to 26. Runtime is
6:35.19 and peak RSS is 874,368 KiB. The board remains withheld because 11 windows still fail
continuous realization.

The remaining failures identify articulation-window coordination as the next implementation
boundary. Independent local stencils are no longer the appropriate remedy: residual windows must
be combined into bounded conflict cones so every participating movable fragment is optimized once
against all of its incident physical conflicts.
