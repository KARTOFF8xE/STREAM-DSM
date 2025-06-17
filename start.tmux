#!/bin/zsh

SESSION="STREAM"

tmux new-session -d -s $SESSION -n demo

# Panel 0:

tmux select-pane -t 0
# tmux split-window -h -t $SESSION
tmux split-window -v -t $SESSION
tmux split-window -v -t $SESSION
tmux split-window -h -t $SESSION
# tmux split-window -h -t $SESSION
# tmux split-window -h -t $SESSION

tmux select-layout -t $SESSION tiled

cd ~/stream
lttng destroy -a
lttng-sessiond --daemonize
lttng-relayd -d
rm -rf /tmp/continuous_traces
rm -rf /tmp/structural_traces
sleep 1

tmux select-pane -t 0
tmux send-keys -t $SESSION 'clear && ros2 run tracer structuralLO' C-m
tmux select-pane -t 1
tmux send-keys -t $SESSION 'clear && ros2 run tracer continuous' C-m
tmux select-pane -t 2
tmux send-keys -t $SESSION 'clear && echo "starting in 3s" && sleep 3 && clear && ros2 run datamgmt datamgmt' C-m
tmux select-pane -t 3
# tmux send-keys -t $SESSION '' C-m
# tmux select-pane -t 4
# tmux send-keys -t $SESSION '' C-m
# tmux select-pane -t 5
tmux send-keys -t $SESSION 'watch -n 1 ipcs -' C-m
tmux split-window -v -t $SESSION
tmux send-keys -t $SESSION 'process-exporter --config.path .devcontainer/configs/process-exporter.yml' C-m
tmux select-pane -t 2

# tmux send-keys -t $SESSION 'watch -n 1 ipcs -q' C-m
# Split horizontal
# tmux send-keys -t $SESSION 'echo C' C-m

# Split vertical (lower panel)
# tmux select-pane -t 2
# tmux send-keys -t $SESSION 'echo D' C-m

# tmux select-pane -t 3
# Split vertical (lower panel)
#tmux send-keys -t $SESSION 'echo B' C-m

# Split vertical (upper panel)
# tmux select-pane -t 0
# tmux select-pane -t 4
# tmux send-keys -t $SESSION 'echo A' C-m




tmux attach-session -t $SESSION
