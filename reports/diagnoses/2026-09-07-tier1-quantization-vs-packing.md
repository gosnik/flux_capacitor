# Tier 1: artificial quantization conflicts plus real allocation collisions

The 600-second `bm_svf` probe did not finish regional repair and produced no
accepted copper. The hard-capacity iteration routed only 19 of 25 nets; the
subsequently selected fallback contains all 78 paths but is physically congested.

Replaying exactly snapshot 587 under the unchanged original project separates
static-obstacle failures from inter-route packing. The 100 micrometre diagnostic
grid yields 930 raw violations, including 65 board-edge and ten pad clearances.
The exact integer-coordinate allocation has none of those edge/pad violations,
but still has 853 violations: 752 track/track, 71 track/via, 28 hole and two
via/via clearances. All three tested pitches retain zero opens and 22 generated
vias. These are straight-scaffold diagnostics, not KiCad-approved routing.

The first emitted witness—net 4 on F.Cu at (50.7, 92.6) mm, 51.536 micrometres
short of pad clearance—is therefore introduced by diagnostic rounding. It is
not evidence of a bad source pad escape or a spline failure. Rounding also hides
some real track/via reports. The probe currently sends both kinds of evidence
upstream into persistent history and regional scope selection.

The 64 logged physical events are capped samples of 261 coalesced diagnoses,
not the size of the complete conflict graph. At 557 seconds, expanded regional
window 0 contains eleven nets and no complete discrete state. Increasing spline
iterations cannot establish feasibility for an unassigned regional bundle.

Rank the smallest experiment first: match default feedback to actual native
copper precision, preserving explicit coarse diagnostic probes and all DRC rules.
Next, investigate the remaining exact conflict components and their layer/via
alternatives. A known-good external original-project routing proves the board is
feasible, not that these selected corridor combinations can be legalized.

Machine-readable evidence and hashes: the sibling JSON file. Tier 0 remains the
mandatory lower-tier guard; Tier 1 is not promoted.
