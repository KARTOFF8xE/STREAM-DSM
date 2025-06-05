#!/bin/zsh

SESSION="evaluation"

tmux new-session -d -s $SESSION -n demo
tmux select-layout -t $SESSION tiled
tmux split-window -h -t $SESSION

tmux select-pane -t 0
tmux send-keys -t $SESSION 'for i in {600..0}; do clear; echo "starting evaluation in ${i}s"; sleep 1; done && ./evaluationScript.zsh' C-m
tmux select-pane -t 1
tmux send-keys -t $SESSION 'process-exporter --config.path ~/stream/.devcontainer/configs/process-exporter.yml' C-m

tmux attach-session -t $SESSION