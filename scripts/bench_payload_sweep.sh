#!/usr/bin/env bash
# Fast-cadence payload sweep over all six directed MPSL board pairs.
# Usage: scripts/bench_payload_sweep.sh [SECS]
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SECS="${1:-25}"
PAYLOADS=(1 4 8 16 32)

for payload in "${PAYLOADS[@]}"; do
  echo "== fast cadence payload ${payload}B: build =="
  CADENCE_PROBE=1 CADENCE_HOLD=1 \
    THUNDERS_BENCH_PAYLOAD_BYTES="$payload" \
    "$ROOT/scripts/bench.sh" build-mpsl

  echo "== fast cadence payload ${payload}B: six directions =="
  CADENCE_PROBE=1 CADENCE_HOLD=1 \
    THUNDERS_BENCH_PAYLOAD_BYTES="$payload" \
    THUNDERS_BENCH_PAYLOAD_SUFFIX=1 \
    "$ROOT/scripts/bench.sh" run-mpsl "$SECS"
done

python3 "$ROOT/scripts/bench_payload_parse.py" "$ROOT/bench/logs"
