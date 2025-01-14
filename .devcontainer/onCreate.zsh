#!/bin/zsh

./.devcontainer/fetchFastDDS.zsh
./.devcontainer/fetchROS2Tracing.zsh

cd /workspaces/DiplArbeitContainer/ws
source /opt/ros/humble/setup.zsh
colcon build --cmake-args -DFASTDDS_STATISTICS=ON --symlink-install