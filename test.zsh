#!/bin/zsh

tmux new-session -d -s "testSession"

tmux send-keys -t testSession:0.0 'ros2 run tracer babel' C-m

tmux split-window -h
tmux send-keys -t testSession:0.1 'ros2 run datamgmt datamgmt' C-m

tmux split-window -v
tmux send-keys -t testSession:0.2 'ros2 run cpp_pubsub talker' C

tmux select-pane -t testSession:0.0
tmux split-window -v
tmux send-keys -t testSession:0.3 'ros2 run primary primary' C

tmux attach -t testSession
