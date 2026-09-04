#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd -- "$script_dir"

artifacts=${FLUX_ARTIFACTS:-"build/tiers-check-$(date +%Y%m%d-%H%M%S)"}

exec python3 tests/python/verify_routing_corpus.py \
    --flux-cli build/dev/flux-cli \
    --corpus tests/fixtures/routing-corpus \
    --level unlocked \
    --artifacts "$artifacts" \
    --telemetry-level diagnostic \
    --failed-route-png-mode both \
    --max-failed-route-png-results 8 \
    "$@"
