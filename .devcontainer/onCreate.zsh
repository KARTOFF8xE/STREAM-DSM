#!/bin/zsh

for file in ./.devcontainer/onCreate/*.zsh
do
    echo "\e[34;1mexecuting ./$file\e[0m"
    ./$file
done

# ./.devcontainer/onCreate/setupEverything.zsh
cd /workspaces/DiplArbeitContainer/ws
source /opt/ros/humble/setup.zsh
colcon build --cmake-args -DFASTDDS_STATISTICS=ON --symlink-install