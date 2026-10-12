#!/usr/bin/env bash
set -u
cd /home/adam/qtbits/flux_capacitor
root=reports/validation/constrained-endpoint-round29-20261012
python3 "$root/freeze.py" --verify || exit 2
tier=${1:?tier0, tier1, tier2 or tier3 required}
extra_args=()
case "$tier" in
  tier0) board=pt2399_delay ;;
  tier1) board=bm_svf ;;
  tier2) board=ft2232h_jtag; extra_args=(--iteration-budget 30) ;;
  tier3) board=gb_nintendo_power; extra_args=(--iteration-budget 30 --route-locked) ;;
  *) exit 2 ;;
esac
  /usr/bin/time -v -o "$root/$tier-resource.txt" python3 tests/python/verify_routing_corpus.py \
    --flux-cli "$root/flux-cli" --corpus tests/fixtures/routing-corpus --level all --board "$board" \
    "${extra_args[@]}" --artifacts "$root/$tier" --track-geometry curved --routing-timeout-seconds 900 \
    --planning-timeout-seconds 960 --detailed-timeout-seconds 960 --telemetry-level diagnostic \
    --telemetry-record-dir "$root/$tier/telemetry" --failed-route-png-dir "$root/$tier/failed-route-png" \
    --failed-route-png-mode both --max-failed-route-png-results 4 > "$root/$tier.log" 2>&1
  status=$?
  echo "$status" > "$root/$tier-exit.txt"
  exit "$status"
