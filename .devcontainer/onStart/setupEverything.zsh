#!/bin/zsh

declare -a sources=(
    "/opt/ros/humble/setup.zsh"
    "/workspaces/DiplArbeitContainer/ws/install/setup.zsh"
    "/usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh"
    )

for source in "${sources[@]}"
do
    echo "source $source" | sudo tee -a ~/.zshrc > /dev/null
done