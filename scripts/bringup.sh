SIM_SESSION=core_sim

# by default, no numbering on anything
ROBOT_NAME=sam

NUM_ROBOTS=1


# Dont forget to change environment_file in base_simulator too
# Biograd
UTM_ZONE=33
UTM_BAND=T
LATITUDE=43.93183
LONGITUDE=15.44264
# ADD other environments, do not just replace above

# localhost for simulation, unless the simulation is done on
# a different computer, no need to change these
NEPTUS_IP=127.0.0.1
SAM_IP=127.0.0.1
# The ports might change if multiple sams are launched, but these
# will always be the 'first' of many
BRIDGE_PORT=6002
WEBGUI_PORT=8097


# the scenario and environment that will be loaded in the simulation
# it includes the world map, auvs, where the auvs are etc.
SCENARIO="sam_biograd_hd"
# and it is found here
SAM_STONEFISH_SIM_PATH="$(rospack find sam_stonefish_sim)"
SCENARIO_DESC=$SAM_STONEFISH_SIM_PATH/data/scenarios/"$SCENARIO".scn
# if we need more than 1 sam, we need to change the scenario file to one that will
# spawn the needed number of sams.
# and since these scenario files do not support looping or anything, we
# need to have separate ones with different numbers of sams in them.
# Differentiate between different number of sams with an appended sam count.
# sam_biograd_hd.scn should have 1 sam
# sam_biograd_hd_2.scn should have 2 sams in it etc.
if [ $NUM_ROBOTS -gt 1 ] #spaces important
then 
	ENVIRONMENT_FILE=$SAM_STONEFISH_SIM_PATH/data/scenarios/"$SCENARIO"_"$NUM_ROBOTS".scn
fi
# ALSO
# take care that in the scenario files, robot_name <arg>s follow the same naming scheme
# as done here in the loop below.



# Main simulation, has its own session and does not loop over num robots
tmux -2 new-session -d -s $SIM_SESSION
tmux new-window -t $SIM_SESSION:0 -n "roscore"
tmux new-window -t $SIM_SESSION:1 -n "base_simulator"

tmux select-window -t $SIM_SESSION:0
tmux send-keys "roscore" C-m

tmux select-window -t $SIM_SESSION:1
tmux send-keys "mon launch sam_stonefish_sim base_simulator.launch robot_name:=$ROBOT_NAME latitude:=$LATITUDE longitude:=$LONGITUDE scenario_description:=$SCENARIO_DESC --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m



# ADD ANY LAUNCHES THAT NEED TO BE LAUNCHED ONLY ONCE, EVEN WHEN THERE ARE 10 SAMS HERE



echo "Started sim"

# we will name tmux sessions the same as robot name
SESSION=sam_bringup
# if one robot, launch everything in the same session
ROBOT_SESSION="$SIM_SESSION"

# seq ranges are inclusive both sides
for ROBOT_NUM in $(seq 1 $NUM_ROBOTS)
do

	# lets not fiddle with the robot_name or ports at all if there is just one
	# this makes sure this for loop is transparent when there is 1 sam only
	if [ $NUM_ROBOTS -gt 1 ] #spaces important
	then 
		ROBOT_SESSION="${SESSION}_${ROBOT_NUM}"
		echo $ROBOT_SESSION
		ROBOT_NAME="${ROBOT_NAME}_${ROBOT_NUM}"
		echo $ROBOT_NAME
		# increment from the default port by 1 for every _extra_ robot
		let WEBGUI_PORT=WEBGUI_PORT+ROBOT_NUM-1
		let BRIDGE_PORT=BRIDGE_PORT+ROBOT_NUM-1
	fi

	# Single SAM launch

	# a new session for the robot-related stuff
	tmux -2 new-session -d -s $ROBOT_SESSION
	echo "Launched new session: $ROBOT_SESSION"

	tmux new-window -t $ROBOT_SESSION:2 -n 'sam_gui'
	tmux new-window -t $ROBOT_SESSION:3 -n 'sam_sim_extras'
	tmux new-window -t $ROBOT_SESSION:4 -n 'sam_static_ctrl'
	tmux new-window -t $ROBOT_SESSION:5 -n 'sam_dyn_ctrl'
	tmux new-window -t $ROBOT_SESSION:6 -n 'sam_mission'

	tmux select-window -t $ROBOT_SESSION:2
	tmux send-keys "mon launch flexxros sam_controls.launch robot_name:=$ROBOT_NAME display_ip:=localhost display_port:=$WEBGUI_PORT --name=$(tmux display-message -p 'p#I_#W')" C-m

	tmux select-window -t $ROBOT_SESSION:3
	tmux send-keys "mon launch sam_stonefish_sim base_simulator_extras.launch with_teleop:=false robot_name:=$ROBOT_NAME --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

	tmux select-window -t $ROBOT_SESSION:4
	tmux send-keys "mon launch sam_basic_controllers static_controllers.launch robot_name:=$ROBOT_NAME --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

	tmux select-window -t $ROBOT_SESSION:5
	tmux send-keys "mon launch sam_basic_controllers dynamic_controllers.launch robot_name:=$ROBOT_NAME --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

	tmux select-window -t $ROBOT_SESSION:6
	tmux send-keys "mon launch sam_stonefish_sim mission.launch robot_name:=$ROBOT_NAME utm_zone:=$UTM_ZONE utm_band:=$UTM_BAND bridge_port:=$BRIDGE_PORT neptus_addr:=$NEPTUS_IP bridge_addr:=$SAM_IP --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m




	# ADD NEW LAUNCHES THAT ARE SPECIFIC TO ONE SAM HERE




done


# Set default window
tmux select-window -t $SIM_SESSION:1
# Attach to session
tmux -2 attach-session -t $SIM_SESSION
