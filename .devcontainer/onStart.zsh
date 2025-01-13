#!/bin/zsh

declare -a sources=(
    "/opt/ros/humble/setup.zsh"
    "/workspaces/DiplArbeitContainer/ws/cpp_pubsub/install/setup.zsh"
    "/workspaces/DiplArbeitContainer/ws/ros2_tracing/install/setup.zsh"
    "/workspaces/DiplArbeitContainer/Fast-DDS/install/setup.zsh"
    )

for source in "${sources[@]}"
do
    echo "source $source" | sudo tee -a ~/.zshrc > /dev/null
done