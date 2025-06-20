#!/bin/bash
set -e

RUN_TIME=${RUN_TIME:-300}
FREQ=${FREQ:-10}
NUM_PAIRS=${NUM_PAIRS:-2}

echo "Starte alle Services für $RUN_TIME Sekunden mit freq=$FREQ, num_pairs=$NUM_PAIRS"

setsid lttng destroy -a &
setsid lttng-sessiond --daemonize &
setsid lttng-relayd -d &

sleep 1

setsid ros2 run tracer structuralLO &
pid_structural=$!

setsid ros2 run tracer continuous &
pid_continuous=$!

setsid bash -c "echo starting in 3s && sleep 3 && ros2 run datamgmt datamgmt" &
pid_datamgmt=$!

setsid ros2 launch evaluationpkg evaluation_launch.launch.py frequency:=$FREQ num_nodes:=$NUM_PAIRS 1>/dev/null &
pid_eval=$!

setsid process-exporter --config.path .devcontainer/configs/process-exporter.yml &
pid_proc_exp=$!

sleep $RUN_TIME

echo "5 Minuten vorbei, beende alle Prozesse..."

kill -- -$pid_structural
kill -- -$pid_continuous
kill -- -$pid_datamgmt
kill -- -$pid_eval
kill -- -$pid_proc_exp

wait $pid_structural || true
wait $pid_continuous || true
wait $pid_datamgmt || true
wait $pid_eval || true
wait $pid_proc_exp || true

echo "Alle Prozesse beendet."
