#!/bin/zsh

wsPath="/home/Georg/stream"

declare -a sources=(
    "/opt/ros/rolling/setup.zsh"
    "${wsPath}/ws/install/setup.zsh"
    "/usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh"
    )

for source in "${sources[@]}"
do
    echo "source $source" | sudo tee -a ~/.zshrc > /dev/null
done

sudo chmod -R 777 ${wsPath}
sudo chown Georg:Georg -R ${wsPath}
git config --global core.filemode false