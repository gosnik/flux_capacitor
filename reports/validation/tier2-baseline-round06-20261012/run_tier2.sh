#!/usr/bin/env bash
set -u
cd /home/adam/qtbits/flux_capacitor
root=reports/validation/tier2-baseline-round06-20261012
python3 "$root/freeze.py" --verify || exit 2
board=ft2232h_jtag
tier=tier2
  /usr/bin/time -v -o "$root/$tier-resource.txt" python3 tests/python/verify_routing_corpus.py \
    --flux-cli "$root/flux-cli" --corpus tests/fixtures/routing-corpus --level all --board "$board" \
    --artifacts "$root/$tier" --track-geometry curved --routing-timeout-seconds 900 \
    --planning-timeout-seconds 960 --detailed-timeout-seconds 960 --iteration-budget 30 --telemetry-level diagnostic \
    --telemetry-record-dir "$root/$tier/telemetry" --failed-route-png-dir "$root/$tier/failed-route-png" \
    --failed-route-png-mode both --max-failed-route-png-results 4 > "$root/$tier.log" 2>&1
  status=$?
  echo "$status" > "$root/$tier-exit.txt"
