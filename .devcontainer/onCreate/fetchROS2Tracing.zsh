#!/bin/zsh

git config --global --add safe.directory /workspaces/DiplArbeitContainer/ws/src/ros2_tracing
cd /workspaces/DiplArbeitContainer/ws/src || exit
git clone https://github.com/ros2/ros2_tracing.git
cd ros2_tracing
git checkout rolling