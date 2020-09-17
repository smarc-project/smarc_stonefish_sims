#!/usr/bin/env bash

NUM_ROBOTS=1
# the scenario and environment that will be loaded in the simulation
# it includes the world map, auvs, where the auvs are etc.
# SCENARIO="sam_biograd_hd"
SCENARIO="sam_biograd"

MIN_ALTITUDE=5
MAX_DEPTH=20

SIM_SESSION=core_sim
# by default, no numbering for one robot
ROBOT_BASE_NAME=sam

# ADD other environments to the list here
# the initial position of the robots are defined in the scenario files
case "$SCENARIO" in
	"sam_asko")
		# asko 
		UTM_ZONE=33
		UTM_BAND=V
		LATITUDE=58.811480
		LONGITUDE=17.596177
		CAR_DEPTH=10
		;;
	"sam_biograd")
		# Biograd
		UTM_ZONE=33
		UTM_BAND=T
		LATITUDE=43.93183
		LONGITUDE=15.44264
		;;
	*)
		echo "UNKNOWN SCENARIO!"
		exit 1
esac


# localhost for simulation, unless the simulation is done on
# a different computer, no need to change these
NEPTUS_IP=127.0.0.1
SAM_IP=127.0.0.1
# The ports might change if multiple sams are launched, but these
# will always be the 'first' of many
BRIDGE_PORT=6002
WEBGUI_PORT=8097

# number of sim steps per frame
SIMULATION_RATE=50
echo "Sim rate set to: $SIMULATION_RATE with $NUM_ROBOTS robots"

# visual fidelity of the sim, changes mostly when gpu power is not enough
# check nvidia-smi, see if it pins to 100%
GFX_QUALITY="high" # high/medium/low

SAM_STONEFISH_SIM_PATH="$(rospack find sam_stonefish_sim)"
#SCENARIO_DESC=$SAM_STONEFISH_SIM_PATH/data/scenarios/"$SCENARIO".scn
CONFIG_FILE="${SAM_STONEFISH_SIM_PATH}/config/${SCENARIO}.yaml"
SCENARIO_DESC="${SAM_STONEFISH_SIM_PATH}/data/scenarios/default.scn"

# if we need more than 1 sam, we need to change the scenario file to one that will
# spawn the needed number of sams.
# and since these scenario files do not support looping or anything, we
# need to have separate ones with different numbers of sams in them.
# Differentiate between different number of sams with an appended sam count.
# sam_biograd_hd.scn should have 1 sam
# sam_biograd_hd_2.scn should have 2 sams in it etc.
# take care that in the scenario files, robot_name <arg>s follow the same naming scheme
# as done here in the loop below.
if [ $NUM_ROBOTS -gt 1 ] #spaces important
then 
	#SCENARIO_DESC=$SAM_STONEFISH_SIM_PATH/data/scenarios/"$SCENARIO"_"$NUM_ROBOTS".scn
    CONFIG_FILE="${SAM_STONEFISH_SIM_PATH}/config/${SCENARIO}_${NUM_ROBOTS}_auvs.yaml"
	SCENARIO_DESC="${SAM_STONEFISH_SIM_PATH}/data/scenarios/default_${NUM_ROBOTS}_auvs.scn"
fi
# check if the scenario file exists.
if [ -f "$SCENARIO_DESC" ]
then
	echo "Using scenario: $SCENARIO_DESC"
else
	echo "Scenario not found: $SCENARIO_DESC"
	echo "Did you forget to create one for this number of robots?"
	exit 1
fi

if [ -f "$CONFIG_FILE" ]
then
	echo "Using sim config: $CONFIG_FILE"
else
	echo "Sim config not found: $CONFIG_FILE"
	echo "Did you forget to create one for this number of robots?"
	exit 1
fi

# ros mon can create gigantic core dumps. I had well over 4Gb of dumps happen.
# this cmd will limit system-wide core dumps to a tiny amount. uncomment if
# you need the core dumps for some reason.
# there is currently no way to configure rosmon only.
# see: https://github.com/xqms/rosmon/issues/107
ulimit -c 1

# Main simulation, has its own session and does not loop over num robots
tmux -2 new-session -d -s $SIM_SESSION -n "roscore"
tmux set-option -g default-shell /bin/bash
tmux new-window -t $SIM_SESSION:1 -n "base_simulator"

tmux select-window -t $SIM_SESSION:0
tmux send-keys "roscore" C-m

tmux select-window -t $SIM_SESSION:1
tmux send-keys "mon launch sam_stonefish_sim base_simulator.launch config_file:=$CONFIG_FILE scenario_description:=$SCENARIO_DESC simulation_rate:=$SIMULATION_RATE graphics_quality:=$GFX_QUALITY --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m


# ADD ANY LAUNCHES THAT NEED TO BE LAUNCHED ONLY ONCE, EVEN WHEN THERE ARE 10 SAMS HERE

SESSION=sam_bringup
# if one robot, launch everything in the same session
ROBOT_SESSION="$SIM_SESSION"
# and with the same robot_name without numbering
ROBOT_NAME="$ROBOT_BASE_NAME"
IMC_SRC=40

# seq ranges are inclusive both sides
for ROBOT_NUM in $(seq 1 $NUM_ROBOTS)
do

	# lets not fiddle with the robot_name or ports at all if there is just one
	# this makes sure this for loop is transparent when there is 1 sam only
	if [ $NUM_ROBOTS -gt 1 ] #spaces important
	then 
		ROBOT_SESSION="${SESSION}_${ROBOT_NUM}"
		# echo $ROBOT_SESSION
		ROBOT_NAME="${ROBOT_BASE_NAME}_${ROBOT_NUM}"
		# echo $ROBOT_NAME
		# increment from the default port by 1 for every _extra_ robot
		let WEBGUI_PORT=WEBGUI_PORT+ROBOT_NUM-1
		let BRIDGE_PORT=BRIDGE_PORT+ROBOT_NUM-1
		let IMC_SRC=ROBOT_NUM+40
	fi

	# Single SAM launch

	# a new session for the robot-related stuff
	# this will throw a 'duplicate session' warning if num_robots=1
	# but that can be safely ignored
	tmux -2 new-session -d -s $ROBOT_SESSION
	# echo "Launched new session: $ROBOT_SESSION"

	tmux new-window -t $ROBOT_SESSION:2 -n 'sam_gui'
	tmux new-window -t $ROBOT_SESSION:3 -n 'sam_sim_extras'
	tmux new-window -t $ROBOT_SESSION:4 -n 'sam_dr'
	tmux new-window -t $ROBOT_SESSION:5 -n 'sam_static_ctrl'
	tmux new-window -t $ROBOT_SESSION:6 -n 'sam_dyn_ctrl'
	tmux new-window -t $ROBOT_SESSION:7 -n 'sam_mission'
	tmux new-window -t $ROBOT_SESSION:8 -n 'sam_detection'


	tmux select-window -t $ROBOT_SESSION:2
	if [ $ROBOT_NUM -gt 1 ] #spaces important
	then 
		tmux send-keys "mon launch flexxros sam_controls.launch robot_name:=$ROBOT_NAME display_ip:=localhost display_port:=$WEBGUI_PORT --name=${ROBOT_NAME}_$(tmux display-message -p 'p#I_#W')"
    else
		tmux send-keys "mon launch flexxros sam_controls.launch robot_name:=$ROBOT_NAME display_ip:=localhost display_port:=$WEBGUI_PORT --name=${ROBOT_NAME}_$(tmux display-message -p 'p#I_#W')" C-m
	fi

	tmux select-window -t $ROBOT_SESSION:3
	tmux send-keys "mon launch sam_stonefish_sim base_simulator_extras.launch with_teleop:=false robot_name:=$ROBOT_NAME --name=${ROBOT_NAME}_$(tmux display-message -p 'p#I_#W') --no-start" C-m

	tmux select-window -t $ROBOT_SESSION:4
	tmux send-keys "mon launch sam_dead_reckoning dual_ekf_test.launch robot_name:=$ROBOT_NAME --name=${ROBOT_NAME}_$(tmux display-message -p 'p#I_#W') --no-start" #C-m

	tmux select-window -t $ROBOT_SESSION:5
	tmux send-keys "mon launch sam_basic_controllers static_controllers.launch robot_name:=$ROBOT_NAME --name=${ROBOT_NAME}_$(tmux display-message -p 'p#I_#W') --no-start" C-m

	tmux select-window -t $ROBOT_SESSION:6
	tmux send-keys "mon launch sam_basic_controllers dynamic_controllers.launch robot_name:=$ROBOT_NAME --name=${ROBOT_NAME}_$(tmux display-message -p 'p#I_#W') --no-start" C-m

	tmux select-window -t $ROBOT_SESSION:7
	tmux send-keys "mon launch sam_mission mission.launch robot_name:=$ROBOT_NAME utm_zone:=$UTM_ZONE utm_band:=$UTM_BAND bridge_port:=$BRIDGE_PORT neptus_addr:=$NEPTUS_IP bridge_addr:=$SAM_IP imc_system_name:=$ROBOT_NAME imc_src:=$IMC_SRC max_depth:=$MAX_DEPTH min_altitude:=$MIN_ALTITUDE --name=${ROBOT_NAME}_$(tmux display-message -p 'p#I_#W') --no-start" C-m

	tmux select-window -t $ROBOT_SESSION:8
	tmux send-keys "mon launch sam_camera_config sam_detection.launch robot_name:=$ROBOT_NAME car_depth:=$CAR_DEPTH --name=${ROBOT_NAME}_$(tmux display-message -p 'p#I_#W') --no-start" #C-m

	# ADD NEW LAUNCHES THAT ARE SPECIFIC TO ONE SAM HERE
done


# Set default window
tmux select-window -t $SIM_SESSION:1
# Attach to session
tmux -2 attach-session -t $SIM_SESSION
