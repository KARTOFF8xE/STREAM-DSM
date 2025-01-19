#!/bin/zsh

# ./.devcontainer/fetchFastDDS.zsh
# ./.devcontainer/fetchROS2Tracing.zsh
# ./.devcontainer/fetchAndBuildBabelTrace.zsh

./.devcontainer/onCreate/*.zsh

cd /workspaces/DiplArbeitContainer/ws
source /opt/ros/humble/setup.zsh
colcon build --cmake-args -DFASTDDS_STATISTICS=ON --symlink-install