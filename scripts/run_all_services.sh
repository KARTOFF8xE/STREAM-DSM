#!/bin/bash
set -e

RUN_TIME=${RUN_TIME:-300}
FREQ=${FREQ:-10}
NUM_PAIRS=${NUM_PAIRS:-2}

source /opt/ros/rolling/setup.bash
source /testspace/ws/install/setup.bash

csv_file="latencies/times.csv"
if [ ! -f "$csv_file" ]; then
    echo "frequency,num_pairs,type,start_time,stop_time" > "$csv_file"
fi

pid_structural=-1
pid_continuous=-1
pid_datamgmt=-1
if [[ $TEST_TYPE == TRACE ]]; then
    echo "Starting all Processes (TRACE); runtime: ${RUN_TIME}s with freq=${FREQ}Hz, num_pairs=$NUM_PAIRS"
    setsid lttng-sessiond --daemonize
    setsid lttng-relayd -d

    setsid ros2 run tracer structuralLO &
    pid_structural=$!

    setsid ros2 run tracer continuous &
    pid_continuous=$!

    setsid bash -c "echo starting in 3s && sleep 3 && ros2 run datamgmt datamgmt" &
    pid_datamgmt=$!
    sleep 10
else 
    echo "Starting all Processes (BASE); runtime: ${RUN_TIME}s with freq=${FREQ}Hz, num_pairs=$NUM_PAIRS"
fi

setsid process-exporter --config.path ./configs/process-exporter.yml &
pid_proc_exp=$!

setsid ros2 launch evaluationpkg evaluation_listener.launch.py frequency:=$FREQ num_nodes:=$NUM_PAIRS test_type:=$TEST_TYPE & # 1>/dev/null &
pid_listeners=$!

sleep 10

setsid ros2 launch evaluationpkg evaluation_talker.launch.py frequency:=$FREQ num_nodes:=$NUM_PAIRS test_type:=$TEST_TYPE & # 1>/dev/null &
pid_talkers=$!

sleep 10

start_time=$(date '+%Y-%m-%d %H:%M:%S')
for ((i=RUN_TIME; i>=0; i--)); do
    echo -ne "\r\033[K"
    echo -ne "\rFinishing test (num_pairs: $NUM_PAIRS freq: ${FREQ}Hz) in ${i}s"
    sleep 1
done
stop_time=$(date '+%Y-%m-%d %H:%M:%S')
echo "${FREQ},${NUM_PAIRS},${TEST_TYPE},${start_time},${stop_time}" >> "$csv_file"

if [[ $TEST_TYPE == TRACE ]]; then
    kill -- -$pid_structural || true
    kill -- -$pid_continuous || true
    kill -- -$pid_datamgmt || true

    wait $pid_structural || true
    wait $pid_continuous || true
    wait $pid_datamgmt || true
fi

kill -- -$pid_listeners
kill -- -$pid_talkers
echo pid_proc_exp: $pid_proc_exp
kill -- -$pid_proc_exp

wait $pid_listeners || true
wait $pid_talkers || true
wait $pid_proc_exp || true


echo "Terminated all processes."
