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
# Radio mode: THUNDERS_RADIO_MODE=1m (default 2m). 1M improves link margin
# at the cost of throughput; build and run with the same mode for both roles.
#
# The bench firmware reports (5 s windows):
#   BENCH C  - central: reverse-link loss, payload bandwidth, app RTT
#   BENCH P  - peripheral: forward-link loss (seq gaps), slot rate
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# Selectable radio mode: THUNDERS_RADIO_MODE=1m scripts/bench.sh build/run.
# Both roles and both boards of a run must use the same mode.
RADIO_MODE="${THUNDERS_RADIO_MODE:-2m}"
# Optional asymmetric schedules: THUNDERS_RATIO=844|622|422.
RATIO="${THUNDERS_RATIO:-}"
PAYLOAD_BYTES="${THUNDERS_BENCH_PAYLOAD_BYTES:-8}"
PAYLOAD_SUFFIX="${THUNDERS_BENCH_PAYLOAD_SUFFIX:-0}"
CADENCE_STEP_US="${THUNDERS_CADENCE_STEP_US:-25}"
LFCLK_52840="${THUNDERS_52840_LFCLK:-rc}"
CADENCE_MODE="${THUNDERS_CADENCE_MODE:-safe}"
case "$CADENCE_MODE" in
  safe|fast) ;;
  *) echo "unsupported THUNDERS_CADENCE_MODE=$CADENCE_MODE (use safe,fast)" >&2; exit 2 ;;
esac
case "$LFCLK_52840" in
  rc|xtal) ;;
  *) echo "unsupported THUNDERS_52840_LFCLK=$LFCLK_52840 (use rc,xtal)" >&2; exit 2 ;;
esac
case "$CADENCE_STEP_US" in
  5|10|25) ;;
  *) echo "unsupported THUNDERS_CADENCE_STEP_US=$CADENCE_STEP_US (use 5,10,25)" >&2; exit 2 ;;
esac
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
  if [ "$RADIO_MODE" = "1m" ]; then
    feats+=(--features radio-1m)
  fi
  if [ "$board" = "52840" ] && [ "$LFCLK_52840" = "xtal" ]; then
    feats+=(--features lfxo)
  fi
  if [ "$CADENCE_MODE" = "fast" ]; then
    feats+=(--features cadence-fast)
  fi
  if [ "${CADENCE_PROBE:-0}" = "1" ] && [ "$backend" = "mpsl" ]; then
    feats+=(--features cadence-probe)
  fi
  if [ "${CADENCE_HOLD:-0}" = "1" ] && [ "$backend" = "mpsl" ]; then
    feats+=(--features cadence-hold)
  fi
  if [ "$backend" = "mpsl" ]; then
    case "$CADENCE_STEP_US" in
      5) feats+=(--features cadence-step-5) ;;
      10) feats+=(--features cadence-step-10) ;;
    esac
  fi
  if [ "$backend" = "mpsl" ]; then
    case "$PAYLOAD_BYTES" in
      1) feats+=(--features payload-1) ;;
      4) feats+=(--features payload-4) ;;
      8) ;;
      16) feats+=(--features payload-16) ;;
      32) feats+=(--features payload-32) ;;
      *) echo "unsupported THUNDERS_BENCH_PAYLOAD_BYTES=$PAYLOAD_BYTES (use 1,4,8,16,32)" >&2; exit 2 ;;
    esac
  fi
  case "$RATIO" in
    844) feats+=(--features ratio-8-4-4) ;;
    622) feats+=(--features ratio-6-2-2) ;;
    422) feats+=(--features ratio-4-2-2) ;;
  esac
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

build_mpsl() {
  for board in "${BOARDS[@]}"; do
    build_one "$board" mpsl central
    build_one "$board" mpsl peripheral
  done
}

run_pair() {
  local c=$1 p=$2 backend=$3 secs=${4:-30}
  local mode_suffix=""
  if [ "$RADIO_MODE" = "1m" ]; then
    mode_suffix="-1m"
  fi
  local lfclk_suffix=""
  if { [ "$c" = "52840" ] || [ "$p" = "52840" ]; } && [ "$LFCLK_52840" = "xtal" ]; then
    lfclk_suffix="-lfxo"
  fi
  local ratio_suffix=""
  if [ -n "$RATIO" ]; then
    ratio_suffix="-r$RATIO"
  fi
  local payload_suffix=""
  if [ "$PAYLOAD_SUFFIX" = "1" ]; then
    payload_suffix="-p${PAYLOAD_BYTES}"
  fi
  local cadence_suffix=""
  if [ "${CADENCE_PROBE:-0}" = "1" ] && [ "$CADENCE_STEP_US" != "25" ]; then
    cadence_suffix="-s${CADENCE_STEP_US}"
  fi
  local cadence_mode_suffix=""
  if [ "$CADENCE_MODE" = "fast" ]; then
    cadence_mode_suffix="-fast"
  fi
  local run="${c}-${p}-${backend}${mode_suffix}${lfclk_suffix}${ratio_suffix}${payload_suffix}${cadence_mode_suffix}${cadence_suffix}"
  local celf="$BIN/$c-$backend-central.elf" pelf="$BIN/$p-$backend-peripheral.elf"
  [ -f "$celf" ] || { echo "missing $celf - run 'scripts/bench.sh build'"; exit 1; }
  [ -f "$pelf" ] || { echo "missing $pelf"; exit 1; }

  local attempts="${BENCH_ATTEMPTS:-3}"
  local all_attempts="${BENCH_ALL_ATTEMPTS:-0}"
  local successes=0
  for attempt in $(seq 1 "$attempts"); do
    echo "== run $run (${secs}s, ${backend}, attempt $attempt) =="
    # The 5340 net core is debug-locked: every flash needs the erase-all
    # permission (which also wipes the app core). The bench firmware is
    # standalone (no host mailbox), so nothing else needs deploying - the
    # role-correct ELF is flashed directly by the run below.

    # The peripheral boots first and holds its slot session open; the central
    # starts after it, so the link forms from the first slot. setsid: the
    # peripheral probe-rs runs in its own process group so it can be reaped
    # cleanly when the central finishes.
    setsid timeout -s INT $((secs + 120)) probe-rs run --chip "${CHIP[$p]}" --probe "${PROBE[$p]}" \
      ${EXTRA[$p]:-} --scan-region "${SCAN[$p]}" "$pelf" > "$LOGS/$run.peripheral.log" 2>&1 &
    local ppid=$!
    # Wait for the peripheral to actually reach BENCH READY before starting
    # the central. The LM20 flash path can take ~15s; a fixed short sleep
    # starts the central too early and makes the first acquisition a race.
    local waited=0
    until grep -q "BENCH READY" "$LOGS/$run.peripheral.log" 2>/dev/null; do
      if ! kill -0 "$ppid" 2>/dev/null; then
        break
      fi
      sleep 1
      waited=$((waited + 1))
      if [ "$waited" -ge 60 ]; then
        echo "timed out waiting for $run peripheral BENCH READY" >&2
        break
      fi
    done
    # Start the central in its own session too. Its timeout covers the flash
    # time plus the requested measurement window; the window itself only
    # starts after the central prints BENCH READY, otherwise a 14s LM20 flash
    # would silently eat most (or all) of a short SECS run.
    setsid timeout -s INT $((secs + 120)) probe-rs run --chip "${CHIP[$c]}" --probe "${PROBE[$c]}" \
      ${EXTRA[$c]:-} --scan-region "${SCAN[$c]}" "$celf" > "$LOGS/$run.central.log" 2>&1 &
    local cpid=$!
    local c_ready=0
    local c_waited=0
    until grep -q "BENCH READY" "$LOGS/$run.central.log" 2>/dev/null; do
      if ! kill -0 "$cpid" 2>/dev/null; then
        break
      fi
      sleep 1
      c_waited=$((c_waited + 1))
      if [ "$c_waited" -ge 90 ]; then
        echo "timed out waiting for $run central BENCH READY" >&2
        break
      fi
    done
    if grep -q "BENCH READY" "$LOGS/$run.central.log" 2>/dev/null; then
      c_ready=1
      local measured=0
      local stable_deadline="${BENCH_STABLE_DEADLINE_SECS:-$secs}"
      if [ "${CADENCE_PROBE:-0}" = "1" ] && [ "$stable_deadline" -lt "$secs" ]; then
        while [ "$measured" -lt "$stable_deadline" ] && \
              ! grep -q "CADENCE STABLE" "$LOGS/$run.central.log" 2>/dev/null; do
          sleep 1
          measured=$((measured + 1))
        done
      fi
      if [ "${CADENCE_PROBE:-0}" != "1" ] || \
         grep -q "CADENCE STABLE" "$LOGS/$run.central.log" 2>/dev/null || \
         [ "$stable_deadline" -ge "$secs" ]; then
        sleep $((secs - measured))
      else
        echo "no Stable within ${stable_deadline}s, ending attempt early"
      fi
    fi
    kill -INT -- -"$cpid" 2>/dev/null || true
    for _ in 1 2 3 4 5; do
      kill -0 "$cpid" 2>/dev/null || break
      sleep 1
    done
    kill -KILL -- -"$cpid" 2>/dev/null || true
    wait "$cpid" 2>/dev/null || true

    # Reap the peripheral cleanly and without an unbounded wait: TERM first,
    # then KILL if a probe-rs process ignores it.
    kill -TERM -- -"$ppid" 2>/dev/null || true
    for _ in 1 2 3 4 5; do
      kill -0 "$ppid" 2>/dev/null || break
      sleep 1
    done
    kill -KILL -- -"$ppid" 2>/dev/null || true
    wait "$ppid" 2>/dev/null || true
    echo "captured $LOGS/$run.central.log + $LOGS/$run.peripheral.log (central_ready=$c_ready)"
    if [ "$attempts" -gt 1 ]; then
      cp "$LOGS/$run.central.log" "$LOGS/$run.attempt${attempt}.central.log"
      cp "$LOGS/$run.peripheral.log" "$LOGS/$run.attempt${attempt}.peripheral.log"
    fi

    # The LM20 boot is intermittent (RtcDriver::init HardFault / Firmware
    # exited unexpectedly). Retry once when either side did not produce a
    # single BENCH line.
    if grep -q "Firmware exited unexpectedly" "$LOGS/$run.peripheral.log" "$LOGS/$run.central.log" 2>/dev/null; then
      echo "detected firmware crash, retrying"
      sleep 2
      continue
    fi
    if ! grep -q "BENCH READY" "$LOGS/$run.peripheral.log" 2>/dev/null || \
       ! grep -q "BENCH READY" "$LOGS/$run.central.log" 2>/dev/null; then
      echo "detected missing BENCH READY, retrying"
      sleep 2
      continue
    fi
    if [ "${CADENCE_PROBE:-0}" = "1" ] && \
       ! grep -q "CADENCE STABLE" "$LOGS/$run.central.log" 2>/dev/null; then
      echo "detected cadence negotiation failure, retrying"
      sleep 2
      continue
    fi
    # BENCH READY is not enough: an acquisition race can leave both sides
    # printing empty windows for the whole run. A valid matrix row must have
    # delivered at least one payload in each direction.
    if ! grep -q "BENCH C .*rx=[1-9]" "$LOGS/$run.central.log" 2>/dev/null || \
       ! grep -q "BENCH P .*rx=[1-9]" "$LOGS/$run.peripheral.log" 2>/dev/null; then
      echo "detected empty link (no data in either direction), retrying"
      sleep 2
      continue
    fi
    successes=$((successes + 1))
    echo "CADENCE ATTEMPT run=$run stable=1 attempt=$attempt/$attempts"
    if [ "$all_attempts" != "1" ]; then
      echo "CADENCE YIELD run=$run successes=1 attempts=$attempt"
      return 0
    fi
  done
  echo "CADENCE YIELD run=$run successes=$successes attempts=$attempts"
  if [ "$successes" -gt 0 ]; then
    return 0
  fi
  echo "run $run failed after retries" >&2
  return 1
}

run_all() {
  local secs=${1:-30}
  local pairs=("52840 5340" "52840 lm20" "5340 52840" "5340 lm20" "lm20 52840" "lm20 5340")
  for backend in "${BACKENDS[@]}"; do
    for pair in "${pairs[@]}"; do
      run_pair $pair "$backend" "$secs" || echo "run $pair failed - continuing matrix" >&2
    done
  done
}

run_mpsl() {
  local secs=${1:-30}
  local pairs=("52840 5340" "52840 lm20" "5340 52840" "5340 lm20" "lm20 52840" "lm20 5340")
  for pair in "${pairs[@]}"; do
    run_pair $pair mpsl "$secs" || echo "run $pair mpsl failed - continuing matrix" >&2
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
  build-mpsl) build_mpsl ;;
  run) probe_check || exit 1; run_all "${2:-30}" ;;
  run-mpsl) probe_check || exit 1; run_mpsl "${2:-30}" ;;
  run-pair) [ $# -ge 4 ] || { echo "usage: $0 run-pair C P BACKEND [SECS]"; exit 1; }; probe_check || exit 1; run_pair "$2" "$3" "$4" "${5:-30}" ;;
  *) echo "usage: $0 {build|build-mpsl|run [SECS]|run-mpsl [SECS]|run-pair C P BACKEND [SECS]}"; exit 1 ;;
esac
