#!/bin/bash
set -e

# Dein Containername
CONTAINER_NAME="diplarbeitcontainer_devcontainer-devcontainer"

# Parameter-Arrays
frequency_arr=(10 100)
num_pairs_arr=(2 20)

# Testzeit in Sekunden
test_period=300

# Optional Pause zwischen Tests
pause_period=10

for num_pairs in "${num_pairs_arr[@]}"; do
  for freq in "${frequency_arr[@]}"; do
    echo "Starte Test mit num_pairs=$num_pairs und freq=$freq Hz"

    docker run --rm \
      --name test_run_${num_pairs}_${freq} \
      -e FREQ=$freq \
      -e NUM_PAIRS=$num_pairs \
      -e RUN_TIME=$test_period \
      --entrypoint /home/ubuntu/stream/run_all_services.sh \
      $CONTAINER_NAME

    echo "Test mit num_pairs=$num_pairs freq=$freq beendet."

    echo "Pause für $pause_period Sekunden vor nächstem Test..."
    sleep $pause_period
  done
done

echo "Alle Tests abgeschlossen."
