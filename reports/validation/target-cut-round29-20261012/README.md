# Tier2 target-cut diagnostic

Compiled before Round29 production edits, from Round27 routing implementation (`a24444c`, repository HEAD `b413814`). The diagnostic explicitly reverses only the incoming request and manually removes each recorded victim set before rebuilding it. This isolates feasibility; it does not demonstrate automatic victim selection.

The saved source is Round27 final Tier2 copper and canonical request evidence. Eleven bounded cases were checked against original incumbent pad groups and baseline internal DRC. Six transactions leave zero physical opens and preserve all original pad connections, including singleton owner97 (9,328 expansions) and singleton90 (20,538). Group90+98 satisfies its explicit requests but loses a pad connection and is not accepted as success. See `results.json` and `verification.json`.

Reproduce against the recorded baseline implementation. Later production direction selection can change the experiment and must not be silently substituted. This is provisional straight scaffold evidence, not final curved routing or external KiCad certification.
