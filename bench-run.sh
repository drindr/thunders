#!/bin/bash
# usage: bench-run.sh <name> <rx-cmd> <tx-cmd> [tx2-cmd]
# Runs RX in background, TX(s) in background, captures 30s, kills all.
NAME=$1; RX=$2; TX=$3; TX2=$4
rm -f bench-logs/$NAME-rx.log bench-logs/$NAME-tx.log bench-logs/$NAME-tx2.log
eval "$RX" > bench-logs/$NAME-rx.log 2>&1 &
RXPID=$!
sleep 4
eval "$TX" > bench-logs/$NAME-tx.log 2>&1 &
TXPID=$!
[ -n "$TX2" ] && { eval "$TX2" > bench-logs/$NAME-tx2.log 2>&1 & TX2PID=$!; }
sleep 32
kill $RXPID $TXPID $TX2PID 2>/dev/null
sleep 2
kill -9 $RXPID $TXPID $TX2PID 2>/dev/null
echo "== $NAME done"
