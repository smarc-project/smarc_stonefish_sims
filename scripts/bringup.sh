SESSION=sam_bringup
# This is the workspace containing the ros packages that are needed

tmux -2 new-session -d -s $SESSION

tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'sam_gui'
tmux new-window -t $SESSION:2 -n 'sam_sim'
tmux new-window -t $SESSION:3 -n 'sam_att_ctrl'

tmux select-window -t $SESSION:0
tmux send-keys "roscore" C-m

tmux select-window -t $SESSION:1
tmux send-keys "rosrun flexxros sam_controls.py" C-m

tmux select-window -t $SESSION:2
tmux send-keys "mon launch sam_stonefish_sim base_simulator.launch hd_model:=true with_teleop:=false --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

tmux select-window -t $SESSION:3
tmux send-keys "mon launch sam_stonefish_sim attitude_controllers.launch --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION
