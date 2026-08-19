#!/usr/bin/env bash
# Cold-start cadence boundary grid. Runs every attempt instead of stopping at
# the first survivor and preserves per-attempt logs through scripts/bench.sh.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CENTRAL="${1:-52840}"
PERIPHERAL="${2:-5340}"
PAYLOAD="${3:-8}"
SECS="${4:-120}"
ATTEMPTS="${5:-20}"
OUT="$ROOT/bench/cadence-grid-${CENTRAL}-${PERIPHERAL}-p${PAYLOAD}.txt"

: > "$OUT"
for STEP in 25 10 5; do
  echo "=== step=${STEP}us pair=${CENTRAL}->${PERIPHERAL} payload=${PAYLOAD}B attempts=${ATTEMPTS} ===" | tee -a "$OUT"
  CADENCE_PROBE=1 CADENCE_HOLD=1 \
    THUNDERS_BENCH_PAYLOAD_BYTES="$PAYLOAD" \
    THUNDERS_CADENCE_STEP_US="$STEP" \
    "$ROOT/scripts/bench.sh" build-mpsl >/dev/null
  BENCH_ALL_ATTEMPTS=1 BENCH_ATTEMPTS="$ATTEMPTS" \
    CADENCE_PROBE=1 CADENCE_HOLD=1 \
    THUNDERS_BENCH_PAYLOAD_BYTES="$PAYLOAD" \
    THUNDERS_BENCH_PAYLOAD_SUFFIX=1 \
    THUNDERS_CADENCE_STEP_US="$STEP" \
    "$ROOT/scripts/bench.sh" run-pair "$CENTRAL" "$PERIPHERAL" mpsl "$SECS" \
    | tee -a "$OUT" || true
done

echo "wrote $OUT"
