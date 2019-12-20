SESSION=sam_bringup
# This is the workspace containing the ros packages that are needed

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
tmux send-keys "rosrun flexxros sam_controls.py _display_ip:=localhost" C-m

tmux select-window -t $SESSION:2
tmux send-keys "mon launch sam_stonefish_sim base_simulator.launch with_teleop:=false --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

tmux select-window -t $SESSION:3
tmux send-keys "mon launch sam_basic_controllers static_controllers.launch --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

tmux select-window -t $SESSION:4
tmux send-keys "mon launch sam_basic_controllers dynamic_controllers.launch --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

tmux select-window -t $SESSION:5
tmux send-keys "mon launch sam_stonefish_sim mission.launch --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION
