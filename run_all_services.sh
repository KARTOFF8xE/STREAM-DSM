#!/bin/bash
set -e

RUN_TIME=${RUN_TIME:-300}
FREQ=${FREQ:-10}
NUM_PAIRS=${NUM_PAIRS:-2}

echo "Starting all Processes; runtime: ${RUN_TIME}s with freq=${FREQ}Hz, num_pairs=$NUM_PAIRS"

setsid lttng-sessiond --daemonize
setsid lttng-relayd -d

source /opt/ros/rolling/setup.bash
source /testspace/ws/install/setup.bash
setsid ros2 run tracer structuralLO &
pid_structural=$!

setsid ros2 run tracer continuous &
pid_continuous=$!

setsid bash -c "echo starting in 3s && sleep 3 && ros2 run datamgmt datamgmt" &
pid_datamgmt=$!

setsid ros2 launch evaluationpkg evaluation_listener.launch.py frequency:=$FREQ num_nodes:=$NUM_PAIRS & # 1>/dev/null &
pid_eval=$!

sleep 10

setsid ros2 launch evaluationpkg evaluation_talker.launch.py frequency:=$FREQ num_nodes:=$NUM_PAIRS & # 1>/dev/null &
pid_eval=$!

setsid process-exporter --config.path ./configs/process-exporter.yml &
pid_proc_exp=$!

for ((i=RUN_TIME; i>=0; i--)); do
    echo -ne "\r\033[K"
    echo -ne "\rFinishing test (num_pairs: $NUM_PAIRS freq: ${FREQ}Hz) in ${i}s"
    sleep 1
done

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

echo "Terminated all processes."
