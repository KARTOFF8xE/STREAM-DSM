#!/bin/bash
set -e

CONTAINER_NAME="testspace"

frequency_arr=(0 10 20 30 40 50 60 70 80 90 100)
num_pairs_arr=(0 10 20 30 40)
test_period=60
pause_period=30

for num_pairs in "${num_pairs_arr[@]}"; do
  for freq in "${frequency_arr[@]}"; do
    echo "Start test with num_pairs=$num_pairs und freq=$freq Hz"

    docker run --rm \
      --name test_run_${num_pairs}_${freq} \
      -e FREQ=$freq \
      -e NUM_PAIRS=$num_pairs \
      -e RUN_TIME=$test_period \
      -v ./latencies:/testspace/latencies \
      --network host \
      --entrypoint /testspace/run_all_services.sh \
      $CONTAINER_NAME

    echo "Finalized test with num_pairs=$num_pairs freq=$freq."

    for ((i=RUN_TIME; i>=0; i--)); do
    echo -ne "\r\033[K"
    echo -ne "\rStarting next test (num_pairs: $num_pairs freq: ${freq}Hz) in ${i}s"
    sleep 1
done
  done
done

echo "Finished tests."
