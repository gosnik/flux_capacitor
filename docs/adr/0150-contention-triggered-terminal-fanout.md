# ADR 0150: Contention-triggered terminal fanout

## Status

Accepted after ADR 0149.

## Context

The isolated ESC's remaining global allocation is dominated by physical clearance shadows rather
than disconnected nets. A controlled quarter-grid local-escape run reduced total overflow from
940 to 521, but applying the fine mesh to every terminal spends work where the nearest coarse
portal is not contested. Broad later-ring diversification would also change the established
nearest-successful-ring policy.

## Decision

Before filtering terminal access, build an immutable mapping from each nearest coarse portal to
the distinct nets that may use it. When at least two nets share a portal, retain the ordinary
nearest exact-legal access and additionally search for one bounded exact-checked local escape on a
quarter-pitch mesh (or half-pitch when quarter pitch is not integral). Global nodes, via sites,
capacity, and PathFinder remain on the coarse lattice.

The local search uses the existing per-terminal expansion bound, represented obstacle predicates,
nominal-width portal egress gate, and optional explicit neck-down policy. It never replaces the
nearest access. Incremental checkpoint reuse records whether contention-triggered fanout was
enabled. JSON exposes contested-terminal and generated-path counts.

## Consequences

On the isolated ESC, 58 contested terminals receive 58 alternatives using 4,024 local
expansions. All 320 global paths remain present. Total overflow falls from 940 to 543, sampled
overflow from 131 to 64, clearance-shadow conflicts from 809 to 479, and regional windows from
284 to 199. The ordinary regional solve completes 184 windows and leaves 15 continuous failures.
Runtime is 4:58.31 with 613,452 KiB peak RSS. This is planning evidence, not accepted copper.

