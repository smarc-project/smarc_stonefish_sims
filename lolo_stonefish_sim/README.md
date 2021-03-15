# lolo_stonefish_sim

Lolo AUV simulation based on Stonefish

## Installing

If you are running Ubuntu 18.04 with ROS Melodic, the easiest way is to
[install the SMARC package repository](https://github.com/smarc-project/rosinstall/blob/master/README.md#binary-install).
When done, install with `sudo apt install ros-melodic-lolo-stonefish-sim`.

## Running

### Base sim with teleop

Make sure `roscore` is running and then run
```
roslaunch lolo_stonefish_sim simulator.launch
```
to start the simulator . If you select the small window,
you can control the vehicle with `w` for forwards, `s` for stop
and the arrow keys for controlling the direction.

### Mission planning with neptus

First, start the smiulator without teleop
```
roslaunch lolo_stonefish_sim simulator.launch with_teleop:=false
```
and then, in another terminal, run
```
roslaunch lolo_stonefish_sim mission.launch
```
to start everything that's needed for the simulator and
interfacing with neptus. Some guidance for how to add LOLO
to neptus can be found [here](https://github.com/smarc-project/imc_ros_bridge).
