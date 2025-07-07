#!/bin/bash
set -e

CONTAINER_NAME="testspace"

frequency_arr=(100 90 80 70 60 50 40 30 20 10 0)
num_pairs_arr=(40 30 20 10 0)
test_period=600
pause_period=300

for num_pairs in "${num_pairs_arr[@]}"; do
  for freq in "${frequency_arr[@]}"; do
    # trace
    echo -e "\e[36mStart test (trace) with num_pairs=$num_pairs und freq=$freq Hz\e[0m"

    docker run --rm \
      --name testrun \
      -e FREQ=$freq \
      -e NUM_PAIRS=$num_pairs \
      -e RUN_TIME=$test_period \
      -e TEST_TYPE=TRACE \
      -v ./latencies:/testspace/latencies \
      --shm-size=1g \
      --network host \
      --entrypoint /testspace/run_all_services.sh \
      $CONTAINER_NAME

    echo -e "\e[34mFinalized test (trace) with num_pairs=$num_pairs freq=$freq.\e[0m"

    for ((i=pause_period; i>=0; i--)); do
      echo -ne "\r\033[K"
      echo -ne "\rStarting next test (base) (num_pairs: $num_pairs freq: ${freq}Hz) in ${i}s"
      sleep 1
    done

    # base
    echo -e "\e[32mStart test (base) with num_pairs=$num_pairs und freq=$freq Hz\e[0m"
    docker run --rm \
      --name testrun \
      -e FREQ=$freq \
      -e NUM_PAIRS=$num_pairs \
      -e RUN_TIME=$test_period \
      -e TEST_TYPE=BASE \
      -v ./latencies:/testspace/latencies \
      --shm-size=1g \
      --network host \
      --entrypoint /testspace/run_all_services.sh \
      $CONTAINER_NAME

    echo -e "\e[33mFinalized test (base) with num_pairs=$num_pairs freq=$freq.\e[0m"

    for ((i=pause_period; i>=0; i--)); do
      echo -ne "\r\033[K"
      echo -ne "\rStarting next test (trace) (num_pairs: $num_pairs freq: ${freq}Hz) in ${i}s"
      sleep 1
    done
  done
done

echo "Finished tests."
