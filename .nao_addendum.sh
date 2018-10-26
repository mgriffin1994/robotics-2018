# Common Nao bindings for the shell and tmux

alias send='~/nao/trunk/build/copy_robot vision 10.202.16.61'
alias send_all='~/nao/trunk/build/copy_robot all 10.202.16.61'
alias compile='bin/compile vision --fast && bin/compile tool --fast && send'
alias compile_all='bin/compile all --fast && send_all'
alias configs='bin/copy_robot configs 10.202.16.61'
alias sne='ssh nao@10.0.1.61'
alias naossh='ssh nao@10.0.1.61'
alias snw='ssh nao@10.202.16.61'
alias kick='bin/copy_robot kick 10.202.16.61'
alias fastcompileall='bin/compile all --fast && send_all'
alias ns='./nao_start.sh'

function nao_connect() {
    tmux select-pane -t 2;
    tmux send-keys "snw" C-m;
    tmux select-pane -t 3;
    tmux send-keys "snw" C-m;
    tmux select-pane -t 4;
    tmux send-keys "snw" C-m;
    tmux select-pane -t 0;
}

function tmux_kill() {
    tmux kill-session -t "Nao";
}

function nao_disconnect() {
    tmux select-pane -t 1;
    tmux send-keys C-c;

    tmux select-pane -t 2;
    tmux send-keys C-c;
    sleep 0.1;
    tmux send-keys "exit" C-m;

    tmux select-pane -t 3;
    tmux send-keys C-c;
    sleep 0.1;
    tmux send-keys "exit" C-m;

    tmux select-pane -t 4;
    tmux send-keys C-c;
    sleep 0.1;
    tmux send-keys "exit" C-m;

    tmux select-pane -t 0;
}

function nao_restart() {
    tmux select-pane -t 2;
    tmux send-keys C-c;

    tmux select-pane -t 3;
    tmux send-keys C-c;

    tmux select-pane -t 4;
    tmux send-keys C-c;

    # Naoqi window
    sleep 0.1;
    tmux select-pane -t 4;
    tmux send-keys "naoqi" C-m;
    sleep 10;

    # Motion window
    tmux select-pane -t 3;
    tmux send-keys "bin/motion" C-m;
    sleep 1;

    # Vision window
    tmux select-pane -t 2;
    tmux send-keys "bin/vision" C-m;

    # Go back to default
    tmux select-pane -t 0;
}
