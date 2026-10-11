#!/usr/bin/env bash
set -u
cd /home/adam/qtbits/flux_capacitor
root=reports/validation/residual-batches-round05-20261012
python3 "$root/freeze.py" --verify || exit 2
tier=${1:?tier0 or tier1 required}
case "$tier" in
  tier0) board=pt2399_delay ;;
  tier1) board=bm_svf ;;
  *) exit 2 ;;
esac
  /usr/bin/time -v -o "$root/$tier-resource.txt" python3 tests/python/verify_routing_corpus.py \
    --flux-cli "$root/flux-cli" --corpus tests/fixtures/routing-corpus --level all --board "$board" \
    --artifacts "$root/$tier" --track-geometry curved --routing-timeout-seconds 900 \
    --planning-timeout-seconds 960 --detailed-timeout-seconds 960 --telemetry-level diagnostic \
    --telemetry-record-dir "$root/$tier/telemetry" --failed-route-png-dir "$root/$tier/failed-route-png" \
    --failed-route-png-mode both --max-failed-route-png-results 4 > "$root/$tier.log" 2>&1
  status=$?
  echo "$status" > "$root/$tier-exit.txt"
