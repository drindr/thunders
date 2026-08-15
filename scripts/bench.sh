#!/usr/bin/env bash
# thunders bench matrix: build every board/backend/role firmware and run the
# full directed-pair matrix over the attached probes, capturing the RTT logs
# for bench_parse.py.
#
# The probes (see README "Building & flashing"):
#   nRF52840   DAPLink 0d28:0204-3:0700...   chip nRF52840_xxAA
#   nRF5340    DAPLink 0d28:0204-3:1304...   chip nRF5340_xxAA (net core)
#   nRF54LM20  J-Link 1366:1069              chip nRF54LM20A
#
# Usage:
#   scripts/bench.sh build                        # build all 12 ELFs
#   scripts/bench.sh run [SECS]                   # full matrix (default 30 s/run)
#   scripts/bench.sh run-pair C P BACKEND [SECS]  # one run, e.g. 52840 lm20 mpsl
#
# The bench firmware reports (5 s windows):
#   BENCH C  - central: reverse-link loss, payload bandwidth, app RTT
#   BENCH P  - peripheral: forward-link loss (seq gaps), slot rate
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN="$ROOT/bench/bin"
LOGS="$ROOT/bench/logs"
mkdir -p "$BIN" "$LOGS"

BOARDS=(52840 5340 lm20)
BACKENDS=(bare mpsl)
declare -A CHIP=( [52840]=nRF52840_xxAA [5340]=nRF5340_xxAA [lm20]=nRF54LM20A )
declare -A PROBE=(
  [52840]="0d28:0204-3:0700000100440055360000054e534d4ca5a5a5a597969908"
  [5340]="0d28:0204-3:13040003001100e10465599500004fca0000000097969921"
  [lm20]="1366:1069"
)
declare -A TRIPLE=( [52840]=thumbv7em-none-eabihf [5340]=thumbv8m.main-none-eabi [lm20]=thumbv8m.main-none-eabihf )
declare -A EXAMPLE=( [52840]=nrf52840 [5340]=nrf5340 [lm20]=nrf54lm20 )
declare -A EXTRA=( [5340]="--allow-erase-all" [lm20]="--speed 100" )
# The RTT control-block scan region (the RAM where defmt-rtt lives; the
# 5340 net core's RAM is at 0x21000000, the others at 0x20000000).
declare -A SCAN=( [52840]="0x20000000..0x20040000" [5340]="0x21000000..0x21010000" [lm20]="0x20000000..0x20100000" )

build_one() {
  local board=$1 backend=$2 role=$3
  local d="$ROOT/examples/${EXAMPLE[$board]}/$backend"
  local td="$ROOT/bench-target/$board-$backend"
  # Explicit --no-default-features: the 5340 examples default to the app-core
  # "host" integration, which the standalone bench must NOT include (the
  # mailbox RAM is secure without the host and the net core faults on it).
  local feats=(--no-default-features)
  if [ "$role" = peripheral ]; then
    feats+=(--features peripheral)
  else
    feats+=(--features central)
  fi
  ( cd "$d" && cargo build --release "${feats[@]}" --target "${TRIPLE[$board]}" --target-dir "$td" )
  cp "$td/${TRIPLE[$board]}/release/thunders-$backend" "$BIN/$board-$backend-$role.elf"
  echo "built $board/$backend/$role"
}

build_all() {
  for board in "${BOARDS[@]}"; do
    for backend in "${BACKENDS[@]}"; do
      build_one "$board" "$backend" central
      build_one "$board" "$backend" peripheral
    done
  done
}

run_pair() {
  local c=$1 p=$2 backend=$3 secs=${4:-30}
  local run="${c}-${p}-${backend}"
  local celf="$BIN/$c-$backend-central.elf" pelf="$BIN/$p-$backend-peripheral.elf"
  [ -f "$celf" ] || { echo "missing $celf - run 'scripts/bench.sh build'"; exit 1; }
  [ -f "$pelf" ] || { echo "missing $pelf"; exit 1; }

  echo "== run $run (${secs}s, ${backend}) =="
  # The 5340 net core is debug-locked: every flash needs the erase-all
  # permission (which also wipes the app core). The bench firmware is
  # standalone (no host mailbox), so nothing else needs deploying - the
  # role-correct ELF is flashed directly by the run below.

  # The peripheral boots first and holds its slot session open; the central
  # starts after it, so the link forms from the first slot. setsid: the
  # peripheral probe-rs runs in its own process group so it can be reaped
  # cleanly when the central finishes.
  setsid timeout -s INT $((secs + 20)) probe-rs run --chip "${CHIP[$p]}" --probe "${PROBE[$p]}" \
    ${EXTRA[$p]:-} --scan-region "${SCAN[$p]}" "$pelf" > "$LOGS/$run.peripheral.log" 2>&1 &
  local ppid=$!
  sleep 4
  timeout -s INT "$secs" probe-rs run --chip "${CHIP[$c]}" --probe "${PROBE[$c]}" \
    ${EXTRA[$c]:-} --scan-region "${SCAN[$c]}" "$celf" > "$LOGS/$run.central.log" 2>&1 || true
  kill -- -"$ppid" 2>/dev/null || true
  wait "$ppid" 2>/dev/null || true
  echo "captured $LOGS/$run.central.log + $LOGS/$run.peripheral.log"
  sleep 2
}

run_all() {
  local secs=${1:-30}
  local pairs=("52840 5340" "52840 lm20" "5340 52840" "5340 lm20" "lm20 52840" "lm20 5340")
  for backend in "${BACKENDS[@]}"; do
    for pair in "${pairs[@]}"; do
      run_pair $pair "$backend" "$secs"
    done
  done
}

probe_check() {
  echo "== probe check =="
  local list
  list="$(probe-rs list 2>&1 || true)"
  echo "$list" | sed -n '1,12p'
  if echo "$list" | grep -q "(inaccessible)"; then
    echo "ERROR: some probes are still inaccessible." >&2
    echo "Run '/flashdev add all' in this session (or re-plug the probes) and retry." >&2
    return 1
  fi
}

case "${1:-}" in
  build) build_all ;;
  run) probe_check || exit 1; run_all "${2:-30}" ;;
  run-pair) [ $# -ge 4 ] || { echo "usage: $0 run-pair C P BACKEND [SECS]"; exit 1; }; probe_check || exit 1; run_pair "$2" "$3" "$4" "${5:-30}" ;;
  *) echo "usage: $0 {build|run [SECS]|run-pair C P BACKEND [SECS]}"; exit 1 ;;
esac
