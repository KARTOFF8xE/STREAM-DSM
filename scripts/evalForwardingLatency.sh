#!/bin/bash
set -e

pkill talker
sleep 1
timestamp=$(date +%s%N)
setsid ros2 run cpp_pubsub talker &>/dev/null &
timestamp=$(($timestamp + 2 * 3600 * 1000 * 1000 * 1000))
echo "${timestamp}"
echo "${timestamp}" >> "started"


# extract_timestamps() {
#     local input_file="$1"
#     while IFS= read -r line; do
#         if [[ $line =~ \[([0-9]{2}:[0-9]{2}:[0-9]{2}\.[0-9]{9})\] ]]; then
#             ts="${BASH_REMATCH[1]}"
#             hour=${ts:0:2}
#             min=${ts:3:2}
#             sec=${ts:6:2}
#             nsec=${ts:9:9}
#             today=$(date +%Y-%m-%d)
#             full="${today} ${hour}:${min}:${sec}"
#             epoch=$(date -d "$full" +%s)
#             echo "${epoch}.${nsec}"
#         fi
#     done < "$input_file"
# }
