#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd -- "$script_dir"

suite=${1:-}
if [[ -z "$suite" ]]; then
    echo "usage: $0 per-change|frontier|release [tier-run options...]" >&2
    exit 2
fi
shift

case "$suite" in
    per-change) routing_level=qualified ;;
    frontier) routing_level=unlocked ;;
    release) routing_level=all ;;
    *)
        echo "unknown verification suite: $suite" >&2
        exit 2
        ;;
esac

unit_runner=build/dev/tests/flux_unit_tests
if [[ ! -x "$unit_runner" || ! -x build/dev/flux-cli ]]; then
    echo "build/dev is not ready; run: cmake --build build/dev -j4" >&2
    exit 2
fi

"$unit_runner" global_to_detailed_route_is_transactional_and_capacity_gated
"$unit_runner" hydraulic_polygon_discovery_projects_into_coarse_navmesh
"$unit_runner" same_net_vertical_features_are_reused_as_transition_portals
ctest --test-dir build/dev --output-on-failure -L risk

if [[ -z "${FLUX_ARTIFACTS:-}" ]]; then
    export FLUX_ARTIFACTS="build/suite-${suite}-$(date +%Y%m%d-%H%M%S)"
fi
exec ./run_tiers.sh --level "$routing_level" "$@"
