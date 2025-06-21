#!/bin/bash
set -e

CONTAINER_NAME="testspace"

frequency_arr=(0 10 20 30 40 50 60 70 80 90 100)
num_pairs_arr=(0 10 20 30 40)
test_period=450
pause_period=300

for num_pairs in "${num_pairs_arr[@]}"; do
  for freq in "${frequency_arr[@]}"; do
    # trace
    echo "Start test (trace) with num_pairs=$num_pairs und freq=$freq Hz"

    docker run --rm \
      --name test_run_${num_pairs}_${freq} \
      -e FREQ=$freq \
      -e NUM_PAIRS=$num_pairs \
      -e RUN_TIME=$test_period \
      -e TEST_TYPE=TRACE \
      -v ./latencies:/testspace/latencies \
      --network host \
      --entrypoint /testspace/run_all_services.sh \
      $CONTAINER_NAME

    echo "Finalized test (trace) with num_pairs=$num_pairs freq=$freq."

    for ((i=pause_period; i>=0; i--)); do
      echo -ne "\r\033[K"
      echo -ne "\rStarting next test (base) (num_pairs: $num_pairs freq: ${freq}Hz) in ${i}s"
    done

    # base
    echo "Start test (base) with num_pairs=$num_pairs und freq=$freq Hz"
    docker run --rm \
      --name test_run_${num_pairs}_${freq} \
      -e FREQ=$freq \
      -e NUM_PAIRS=$num_pairs \
      -e RUN_TIME=$test_period \
      -e TEST_TYPE=BASE \
      -v ./latencies:/testspace/latencies \
      --network host \
      --entrypoint /testspace/run_all_services.sh \
      $CONTAINER_NAME

    echo "Finalized test (base) with num_pairs=$num_pairs freq=$freq."

    for ((i=pause_period; i>=0; i--)); do
      echo -ne "\r\033[K"
      echo -ne "\rStarting next test (trace) (num_pairs: $num_pairs freq: ${freq}Hz) in ${i}s"
      sleep 1
    done
  done
done

echo "Finished tests."
