# sam_stonefish_sim

Prototype for simulating the SAM auv in stonefish. This is a work in progress
and support for the full SAM vehicle is not there yet. Right now we support
the following actuator commands:
* `/sam/core/lcg_cmd` - `sam_msgs/PercentStamped` sets position of LCG weight 0-100%
* `/sam/core/rpm_cmd` - `sam_msgs/ThrusterRPMs` sets rpm of the two thrusters, forward is positive
* `/sam/core/thrust_vector_cmd` - `sam_msgs/ThrusterAngles` sets vertical and horizontal angle of thrusters

## Building

Let us start by installing the dependencies.
```
sudo apt install libsdl2-dev ros-kinetic-py-trees-ros
```
If you are on Ubuntu 16.04, you need to use vim (or other editor) to open the file
```
sudo vim /usr/lib/x86_64-linux-gnu/cmake/SDL2/sdl2-config.cmake
```
and then remove space after "-lSDL2 ". On 18.04 this is not needed.

### Minimum requirements

Now go into the catkin workspace where you want to build the simulation,
or create a new workspace. In the `src` folder, execute the commands,
```
git clone https://bitbucket.org/iquarobotics/cola2_msgs.git
git clone https://github.com/nilsbore/sam_stonefish_sim.git
git clone https://github.com/nilsbore/stonefish_ros.git
cd stonefish_ros
git submodule update --init
```
This is everything you need if you just want to run the base sim (see below).
In addition, if you want to be able to use teleop, you unfortunately still need the
`sam_common` package listed below though. If content, skip to the next step.

### Requirements for planner

**NOTE**: you need access to KTH gitr for this step.
These packages are needed if you want to run the full stack including
mission planning with Neptus and controlling the vehicle to follow the plan.
```
git clone https://gitr.sys.kth.se/smarc-project/sam_common.git
git clone https://github.com/smarc-project/imc_ros_bridge.git
https://github.com/smarc-project/smarc_missions.git
```

### Compiling

When you fetched what you want, run `catkin_make -DCMAKE_BUILD_TYPE=Release` and make sure to
source your workspace `devel/setup.bash` wherever you want to run this.

## Running

### Base sim with teleop

Just run
```
roslaunch sam_stonefish_sim ros_simulator.launch
```
and control the vehicle like normal, with `w` for forwards, `s` for stop
and the arrow keys for controlling the direction.

### Mission planning with neptus

Run
```
roslaunch sam_stonefish_sim sam_simulator.launch
```
to start everything that's needed for the simulator and
interfacing with neptus. Some guidance for how to add SAM
to neptus can be found [here](https://github.com/smarc-project/imc_ros_bridge).
It is basically the same as the [BTS tutorial](https://github.com/smarc-project/smarc_scenarios/tree/master/bts_tutorial)
and I think @KKalem is working on more extensive documentation for that that
would be just as relevant here.
