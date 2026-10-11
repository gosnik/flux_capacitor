# Retained source-compatibility boards

The previous routing Tiers 2–4 remain here with their original manifest entries,
project settings, normalized references, licenses and hash checks. They have
8 / 3 / 17 known structural blockers and are retained for importer/geometry
regression work. Their replacement routing inputs are documented in
`../alternatives/README.md`. No source rules or placements were edited.

Run the existing verifier with `--corpus tests/fixtures/routing-corpus/compatibility
--level all --preflight-only` and a fresh artifact destination. A successful
compatibility preflight checks the recorded allowances; it does not mean these
boards are physically valid or routed.
