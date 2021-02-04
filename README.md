# sam_stonefish_sim
![CI](https://github.com/smarc-project/sam_stonefish_sim/workflows/CI/badge.svg?branch=noetic-devel) [![license](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

Simlation of the SAM AUV based on the [Stonefish simulator](https://github.com/patrykcieslak/stonefish).

## Installing

If you are running Ubuntu 18.04 with ROS Melodic, the easiest way is to
[install the SMARC package repository](https://github.com/smarc-project/rosinstall/blob/master/README.md#binary-install).
When done, install with `sudo apt install ros-melodic-sam-stonefish-sim`.
If you are running another distro, follow the build instructions below.


## Running

### Base sim with teleop

Make sure `roscore` is running and then run
```
roslaunch sam_stonefish_sim simulator.launch
```
to start the simulator . If you select the small window,
you can control the vehicle with `w` for forwards, `s` for stop
and the arrow keys for controlling the direction.

### Mission planning with neptus

First, start the smiulator without teleop
```
roslaunch sam_stonefish_sim simulator.launch with_teleop:=false
```
and then, in another terminal, run
```
roslaunch sam_stonefish_sim mission.launch
```
to start everything that's needed for the simulator and
interfacing with neptus. Some guidance for how to add SAM
to neptus can be found [here](https://github.com/smarc-project/imc_ros_bridge).

## Building

Let us start by installing the dependencies.
```
sudo apt install libsdl2-dev libglew-dev libfreetype6-dev tmux python3-vcstool ros-${ROS_DISTRO}-py-trees-ros ros-${ROS_DISTRO}-pid
```
If you are on Ubuntu 16.04, you need to use vim (or other editor) to open the file
```
sudo vim /usr/lib/x86_64-linux-gnu/cmake/SDL2/sdl2-config.cmake
```
and then remove space after "-lSDL2 ". On 18.04 this is not needed.

### Installing with vcstool

Using the `python3-vcstool` dependency, we can install all of
the packages outlined below with one simple command in our
`catkin_ws/src` folder. If you only have access to the public
repos (i.e. you're not a member of the SMaRC project), use the command:
```
curl https://raw.githubusercontent.com/nilsbore/sam_stonefish_sim/master/rosinstall/sam_sim.rosinstall | vcs import --recursive
```
Otherwise, if you have access to `https://gitr.sys.kth.se/smarc-project`, issue:
```
curl https://raw.githubusercontent.com/nilsbore/sam_stonefish_sim/master/rosinstall/sam_sim_private.rosinstall | vcs import --recursive --w 1
```
in your `catkin_ws/src` folder. This may take a few minutes, so be patient.

You can now go to your `catkin_ws` and use `catkin_make -DCMAKE_BUILD_TYPE=Release`
to build all of the packages needed for the simulation. No need for the build and clone
steps below. Instead, skip to the [running section](https://github.com/nilsbore/sam_stonefish_sim#running).

### Minimum requirements (manual)

Now go into the catkin workspace where you want to build the simulation,
or create a new workspace. In the `src` folder, execute the commands,
```
git clone https://bitbucket.org/iquarobotics/cola2_msgs.git
git clone https://github.com/smarc-project/sam_common.git
git clone https://github.com/smarc-project/sam_stonefish_sim.git
git clone https://github.com/nilsbore/stonefish_ros.git
cd stonefish_ros
git submodule update --init
```
This is everything you need if you just want to run the base sim (see below).

### Compiling

When you fetched what you want, run `catkin_make -DCMAKE_BUILD_TYPE=Release` and make sure to
source your workspace `devel/setup.bash` wherever you want to run this.
