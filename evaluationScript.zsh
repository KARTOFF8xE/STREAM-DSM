#!/bin/zsh

echo "Start evaluation"
echo "Dashboards to Metrics can be found here: http://localhost:3000/dashboards"
for i in {1..16}; do
  echo "Running iteration $i"
  echo "  Launching..."
  ros2 launch cpp_pubsub double_launch.launch.py &
  PID_A=$!

  sleep 300
  echo "  Stopping... (PID: $PID_A)..."
  kill $PID_A 2>/dev/null

  sleep 60
done
