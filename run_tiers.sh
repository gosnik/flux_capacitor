#!/bin/bash
python3 tests/python/verify_routing_corpus.py \
    --flux-cli build/dev/flux-cli \
    --corpus tests/fixtures/routing-corpus \
    --level unlocked \
    --artifacts "build/tiers-check-$(date +%Y%m%d-%H%M%S)" \
    --telemetry-level diagnostic \
    --failed-route-png-mode both \
    --max-failed-route-png-results 8
