#!/bin/bash
LOG_DIR="/tmp/carla_roboos_logs"
[ ! -f "$LOG_DIR/pids.txt" ] && echo "No processes running" && exit 1
read PID1 PID2 PID3 PID4 PID5 PID6 < $LOG_DIR/pids.txt
for i in 1 2 3 4 5 6; do
  eval pid=\$PID$i
  ps -p $pid >/dev/null 2>&1 && echo "✅ Terminal $i (PID: $pid)" || echo "❌ Terminal $i (PID: $pid)"
done
