#!/bin/zsh

declare -a frequency_arr=(10 100)
declare -a num_pairs_arr=(2 20)

test_period=300
pause_period=300
output_file="evaluation/times.csv"
mkdir -p evaluation

if [[ ! -f "$output_file" ]]; then
    echo "freq,num_nodes,starttimestamp,stoptimestamp" > "$output_file"
fi

start_process() {
    local freq=$1
    local num_pairs=$2

    echo -ne "\rLaunching: $num_pairs Nodes with ${freq}Hz per Talker"

    local start_time=$(date "+%Y-%m-%d %H:%M:%S")

    setsid bash -c "ros2 launch evaluationpkg evaluation_launch.launch.py frequency:=$freq num_nodes:=$num_pairs" 1>/dev/null &
    pid=$!

    for ((i=test_period; i>=0; i--)); do
        echo -ne "\rFinishing test (num_pairs: $num_pairs freq: ${freq}Hz) in ${i}s"
        sleep 1
        echo -ne "\r\033[K"
    done

    echo -ne "\rStopping launched process (PID: $pid)"

    pgid=$(ps -o pgid= -p $pid | tr -d ' ')

    if [[ -n "$pgid" ]]; then
        kill -TERM -"$pgid" 2>/dev/null
        sleep 1
        kill -KILL -"$pgid" 2>/dev/null
    fi

    local stop_time=$(date "+%Y-%m-%d %H:%M:%S")
    echo "$freq,$num_pairs,\"$start_time\",\"$stop_time\"" >> "$output_file"

    # sudo sync && sudo sh -c 'echo 3 > /d'
}

clear
for num_pairs in "${num_pairs_arr[@]}"; do
    for freq in "${frequency_arr[@]}"; do
        echo -ne "\033[H"
        echo "======== Number of PubSub pairs: ${num_pairs}; Frequency: ${freq}Hz ========"
        for ((i=pause_period; i>=0; i--)); do
            echo -ne "\rPreparing test (num_pairs: $num_pairs freq: ${freq}Hz) in ${i}s"
            sleep 1
            echo -ne "\r\033[K"
        done

        start_process "$freq" "$num_pairs"
    done
done

echo "Test run completed."
