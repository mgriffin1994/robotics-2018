#! /bin/bash

SESSION="Nao"

tmux -2 new-session -d -s $SESSION

# Set up all windows for Nao connection
tmux new-window -t $SESSION:0 -n 'Code'
tmux split-pane -v
tmux split-pane -h
tmux split-pane -h
tmux select-pane -t 1
tmux split-pane -h

# Tool
tmux select-pane -t 1;
tmux send-keys "bin/tool" C-m;

# Nao windows
tmux select-pane -t 2;
tmux send-keys "snw" C-m;
tmux select-pane -t 3;
tmux send-keys "snw" C-m;
tmux select-pane -t 4;
tmux send-keys "snw" C-m;
sleep 1;

# Naoqi window
tmux select-pane -t 4;
tmux send-keys "nao stop" C-m;
sleep 2;
tmux send-keys "naoqi" C-m;
sleep 12;

# Motion window
tmux select-pane -t 3;
tmux send-keys "bin/motion" C-m;
sleep 2;

# Vision window
tmux select-pane -t 2;
tmux send-keys "bin/vision" C-m;

# Set default window and resize
tmux select-pane -t 0
tmux resize-pane -D 15

# Connect to tmux session
tmux -2 attach-session -t $SESSION
