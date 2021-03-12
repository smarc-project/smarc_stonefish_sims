#!/usr/bin/env bash

NUM_ROBOTS=1
# the scenario and environment that will be loaded in the simulation
# it includes the world map, auvs, where the auvs are etc.
# SCENARIO="sam_biograd_hd"
SCENARIO="biograd_world"

MIN_ALTITUDE=5
MAX_DEPTH=20

SIM_SESSION=core_sim
# by default, no numbering for one robot
ROBOT_BASE_NAME=sam

# localhost for simulation, unless the simulation is done on
# a different computer, no need to change these
NEPTUS_IP=127.0.0.1
SAM_IP=127.0.0.1
# The ports might change if multiple sams are launched, but these
# will always be the 'first' of many
IMC_BRIDGE_PORT=6002

# number of sim steps per frame
SIMULATION_RATE=300
echo "Sim rate set to: $SIMULATION_RATE with $NUM_ROBOTS robots"

# visual fidelity of the sim, changes mostly when gpu power is not enough
# check nvidia-smi, see if it pins to 100%
GFX_QUALITY="low" # high/medium/low

# ADD other environments to the list here
# the initial position of the robots are defined in the scenario files
case "$SCENARIO" in
	"asko_world")
		# asko 
		UTM_ZONE=33
		UTM_BAND=V
		LATITUDE=58.811480
		LONGITUDE=17.596177
		CAR_DEPTH=10
		;;
	"biograd_world")
		# Biograd
		UTM_ZONE=33
		UTM_BAND=T
		LATITUDE=43.93183
		LONGITUDE=15.44264
		;;
	"algae_world")
		# Algae farm
		UTM_ZONE=33
		UTM_BAND=T
		LATITUDE=43.93183
		LONGITUDE=15.44264
		;;
	*)
		echo "UNKNOWN SCENARIO!"
		exit 1
esac

SAM_STONEFISH_SIM_PATH="$(rospack find sam_stonefish_sim)"

# use the private sims package if available
SAM_PRIVATE_SIMS_PATH="$(rospack find sam_private_sims)"
if [ -z "$SAM_PRIVATE_SIMS_PATH" ]
then
	echo "Did not find sam_private_sims, using sam_stonefish_sim instead"
	SIM_PKG="sam_stonefish_sim"
    SIM_PKG_PATH=$SAM_STONEFISH_SIM_PATH
else
	echo "Found sam_private_sims, using that"
	SIM_PKG="sam_private_sims"
    SIM_PKG_PATH=$SAM_PRIVATE_SIMS_PATH
fi

SMARC_STONEFISH_WORLDS_PATH="$(rospack find smarc_stonefish_worlds)"

#SCENARIO_DESC=$SAM_STONEFISH_SIM_PATH/data/scenarios/"$SCENARIO".scn
#CONFIG_FILE="${SAM_STONEFISH_SIM_PATH}/config/${SCENARIO}.yaml"
WORLD_CONFIG="${SCENARIO}.yaml"
WORLD_CONFIG_FILE="${SMARC_STONEFISH_WORLDS_PATH}/config/${WORLD_CONFIG}"
ROBOT_CONFIG="sam_no_payload_sensors.yaml"
ROBOT_CONFIG_FILE="${SIM_PKG_PATH}/config/${ROBOT_CONFIG}"
SCENARIO_DESC="${SMARC_STONEFISH_WORLDS_PATH}/data/scenarios/default.scn"

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
	#CONFIG_FILE="${SAM_STONEFISH_SIM_PATH}/config/${SCENARIO}_${NUM_ROBOTS}_auvs.yaml"
	SCENARIO_DESC="${SMARC_STONEFISH_WORLDS_PATH}/data/scenarios/default_${NUM_ROBOTS}_auvs.scn"
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

if [ -f "$WORLD_CONFIG_FILE" ]
then
	echo "Using config file: $WORLD_CONFIG_FILE"
else
	echo "Config not found: $WORLD_CONFIG_FILE"
	exit 1
fi

if [ -f "$ROBOT_CONFIG_FILE" ]
then
	echo "Using config file: $ROBOT_CONFIG_FILE"
else
	echo "Config not found: $ROBOT_CONFIG_FILE"
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
tmux new-window -t $SIM_SESSION:1 -n "simulator"

tmux select-window -t $SIM_SESSION:0
tmux send-keys "roscore" C-m

tmux select-window -t $SIM_SESSION:1
tmux send-keys "mon launch smarc_stonefish_worlds stonefish.launch robot_config_file:=$ROBOT_CONFIG_FILE world_config_file:=$WORLD_CONFIG_FILE scenario_description:=$SCENARIO_DESC simulation_rate:=$SIMULATION_RATE graphics_quality:=$GFX_QUALITY --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m


# ADD ANY LAUNCHES THAT NEED TO BE LAUNCHED ONLY ONCE, EVEN WHEN THERE ARE 10 SAMS HERE

BASE_SESSION=robot
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
		ROBOT_SESSION="${BASE_SESSION}_${ROBOT_NUM}"
		ROBOT_NAME="${ROBOT_BASE_NAME}_${ROBOT_NUM}"
		# increment from the default port by 1 for every _extra_ robot
		let WEBGUI_PORT=WEBGUI_PORT+ROBOT_NUM-1
		let IMC_BRIDGE_PORT=IMC_BRIDGE_PORT+ROBOT_NUM-1
		let IMC_SRC=ROBOT_NUM+40
	fi

	# Single SAM launch

	# a new session for the robot-related stuff
	# this will throw a 'duplicate session' warning if num_robots=1
	# but that can be safely ignored
	tmux -2 new-session -d -s $ROBOT_SESSION
	# echo "Launched new session: $ROBOT_SESSION"

	tmux new-window -t $ROBOT_SESSION:2 -n 'gui'
	tmux new-window -t $ROBOT_SESSION:3 -n 'robot_bridge'
	tmux new-window -t $ROBOT_SESSION:4 -n 'mission'

	tmux select-window -t $ROBOT_SESSION:2
    tmux send-keys "mon launch sam_webgui_native native_webgui.launch namespace:=$ROBOT_NAME --name=${ROBOT_NAME}_$(tmux display-message -p 'p#I_#W')"
	if [ $ROBOT_NUM -eq 1 ] #spaces important
	then 
		tmux send-keys C-m
	fi

	tmux select-window -t $ROBOT_SESSION:3
	tmux send-keys "mon launch sam_stonefish_sim robot_bridge.launch robot_name:=$ROBOT_NAME --name=${ROBOT_NAME}_$(tmux display-message -p 'p#I_#W') --no-start" C-m

	tmux select-window -t $ROBOT_SESSION:4
	tmux send-keys "mon launch sam_stonefish_sim mission.launch robot_name:=$ROBOT_NAME bridge_port:=$IMC_BRIDGE_PORT neptus_addr:=$NEPTUS_IP bridge_addr:=$SAM_IP imc_system_name:=$ROBOT_NAME imc_src:=$IMC_SRC max_depth:=$MAX_DEPTH min_altitude:=$MIN_ALTITUDE --name=${ROBOT_NAME}_$(tmux display-message -p 'p#I_#W') --no-start" C-m

	# ADD NEW LAUNCHES THAT ARE SPECIFIC TO ONE SAM HERE
done


# Set default window
tmux select-window -t $SIM_SESSION:1
# Attach to session
tmux -2 attach-session -t $SIM_SESSION
