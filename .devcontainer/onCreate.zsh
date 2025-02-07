#!/bin/zsh

for file in ./.devcontainer/onCreate/*.zsh
do
    echo "\e[34;1mexecuting ./$file\e[0m"
    ./$file
done

git config --global --add safe.directory /workspaces/DiplArbeitContainer