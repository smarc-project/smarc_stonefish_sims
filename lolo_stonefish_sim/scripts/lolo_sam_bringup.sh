#!/usr/bin/env bash

SIM_SESSION=core_sim
tmux -2 new-session -d -s $SIM_SESSION -n "roscore"
tmux set-option -g default-shell /bin/bash
tmux new-window -t $SIM_SESSION:1 -n "simulator"
tmux new-window -t $SIM_SESSION:2 -n 'lolo_gui'
tmux new-window -t $SIM_SESSION:3 -n 'sam_gui'
tmux new-window -t $SIM_SESSION:4 -n 'lolo_mission'
tmux new-window -t $SIM_SESSION:5 -n 'sam_mission'

tmux select-window -t $SIM_SESSION:0
tmux send-keys "roscore" C-m

tmux select-window -t $SIM_SESSION:1
tmux send-keys "mon launch lolo_stonefish_sim lolo_sam_simulator.launch --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

tmux select-window -t $ROBOT_SESSION:2
tmux send-keys "mon launch lolo_webgui_native native_webgui.launch with_rosapi:=false --name=$(tmux display-message -p 'p#I_#W')" C-m

tmux select-window -t $ROBOT_SESSION:3
tmux send-keys "mon launch sam_webgui_native native_webgui.launch --name=$(tmux display-message -p 'p#I_#W')" C-m

tmux select-window -t $ROBOT_SESSION:4
tmux send-keys "mon launch lolo_stonefish_sim mission.launch imc_src:=40 imc_bridge_port:=6002 --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

tmux select-window -t $ROBOT_SESSION:5
tmux send-keys "mon launch sam_stonefish_sim mission.launch imc_src:=41 imc_bridge_port:=6003 --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

# Set default window
tmux select-window -t $SIM_SESSION:1
# Attach to session
tmux -2 attach-session -t $SIM_SESSION
