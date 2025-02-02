#!/bin/zsh

counter=$(apt list --upgradable 2>/dev/null | wc -l)
((counter=$counter-1))
echo "There are \e[1m${counter}\e[0m new updates available."