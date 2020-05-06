SESSION=sam_bringup
ROBOT_NAME=sam1

# Biograd
UTM_ZONE=33
UTM_BAND=N
LATITUDE=43.93183
LONGITUDE=15.44264
# ADD other environments, do not just replace above


tmux -2 new-session -d -s $SESSION

tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'sam_gui'
tmux new-window -t $SESSION:2 -n 'sam_sim'
tmux new-window -t $SESSION:3 -n 'sam_static_ctrl'
tmux new-window -t $SESSION:4 -n 'sam_dyn_ctrl'
tmux new-window -t $SESSION:5 -n 'sam_mission'

tmux select-window -t $SESSION:0
tmux send-keys "roscore" C-m

tmux select-window -t $SESSION:1
# tmux send-keys "rosrun flexxros sam_controls.py _display_ip:=localhost" C-m
tmux send-keys "mon launch flexxros sam_controls.launch robot_name:=$ROBOT_NAME display_ip:=localhost --name=$(tmux display-message -p 'p#I_#W')" C-m

tmux select-window -t $SESSION:2
tmux send-keys "mon launch sam_stonefish_sim base_simulator.launch with_teleop:=false robot_name:=$ROBOT_NAME latitude:=$LATITUDE longitude:=$LONGITUDE --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

tmux select-window -t $SESSION:3
tmux send-keys "mon launch sam_basic_controllers static_controllers.launch robot_name:=$ROBOT_NAME --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

tmux select-window -t $SESSION:4
tmux send-keys "mon launch sam_basic_controllers dynamic_controllers.launch robot_name:=$ROBOT_NAME --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

tmux select-window -t $SESSION:5
tmux send-keys "mon launch sam_stonefish_sim mission.launch robot_name:=$ROBOT_NAME utm_zone:=$UTM_ZONE utm_band:=$UTM_BAND --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION
