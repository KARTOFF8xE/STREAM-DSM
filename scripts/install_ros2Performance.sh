#!/bin/bash

mkdir -p ~/stream/performance_ws/src
cd ~/stream/performance_ws/src
git clone https://github.com/irobot-ros/ros2-performance
cd ros2-performance
git submodule update --init --recursive
cd ../..
colcon build

echo "to start ros2 performance"
echo "source ~/stream/performance_ws/install/setup.bash"
echo "cd ~/stream/performance_ws/install/irobot_benchmark/lib/irobot_benchmark"
echo "./irobot_benchmark topology/sierra_nevada.json"