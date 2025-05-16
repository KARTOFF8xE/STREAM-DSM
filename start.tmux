#!/bin/zsh

SESSION="zmq_demo"

tmux new-session -d -s $SESSION -n demo

# Panel 0:

tmux select-pane -t 0
tmux split-window -h -t $SESSION
tmux split-window -v -t $SESSION
tmux split-window -h -t $SESSION
tmux split-window -h -t $SESSION
tmux split-window -h -t $SESSION

tmux select-layout -t $SESSION tiled


tmux select-pane -t 0
tmux send-keys -t $SESSION 'lttng destroy || clear && ./lttng_startsession.sh && ros2 run tracer babel' C-m
tmux select-pane -t 1
tmux send-keys -t $SESSION 'for i in {5..0}; do clear && echo $i && sleep 1s; done && clear && ros2 run datamgmt datamgmt' C-m
tmux select-pane -t 2
tmux send-keys -t $SESSION '' C-m
tmux select-pane -t 3
tmux send-keys -t $SESSION '' C-m
tmux select-pane -t 4
tmux send-keys -t $SESSION '' C-m
tmux select-pane -t 5
tmux send-keys -t $SESSION 'watch -n 1 ipcs -' C-m


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