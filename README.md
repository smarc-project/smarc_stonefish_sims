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
sudo apt install libsdl2-dev python3-vcstool ros-${ROS_DISTRO}-py-trees-ros
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
in your `catkin_ws/src` folder.

You can now go to your `catkin_ws` and use `catkin_make -DCMAKE_BUILD_TYPE=Release`
to build all of the packages needed for the simulation. No need for the build and clone
steps below. Instead, skip to the [running section](https://github.com/nilsbore/sam_stonefish_sim#running).

### Minimum requirements (manual)

Now go into the catkin workspace where you want to build the simulation,
or create a new workspace. In the `src` folder, execute the commands,
```
git clone https://bitbucket.org/iquarobotics/cola2_msgs.git
git clone https://github.com/smarc-project/sam_common.git
git clone https://github.com/nilsbore/sam_stonefish_sim.git
git clone https://github.com/nilsbore/stonefish_ros.git
cd stonefish_ros
git submodule update --init
```
This is everything you need if you just want to run the base sim (see below).

### Requirements for planner (manual)

These packages are needed if you want to run the full stack including
mission planning with Neptus and controlling the vehicle to follow the plan.
```
git clone https://github.com/smarc-project/imc_ros_bridge.git
git clone https://github.com/smarc-project/smarc_missions.git
```

### Compiling

When you fetched what you want, run `catkin_make -DCMAKE_BUILD_TYPE=Release` and make sure to
source your workspace `devel/setup.bash` wherever you want to run this.

## Running

### Base sim with teleop

Just run
```
roslaunch sam_stonefish_sim base_simulator.launch
```
and control the vehicle like normal, with `w` for forwards, `s` for stop
and the arrow keys for controlling the direction.

### Mission planning with neptus

Run
```
roslaunch sam_stonefish_sim mission_simulator.launch
```
to start everything that's needed for the simulator and
interfacing with neptus. Some guidance for how to add SAM
to neptus can be found [here](https://github.com/smarc-project/imc_ros_bridge).
It is basically the same as the [BTS tutorial](https://github.com/smarc-project/smarc_scenarios/tree/master/bts_tutorial)
and I think @KKalem is working on more extensive documentation for that that
would be just as relevant here.

### Robot-like interface

The robot interface also requires `sam_common`, see above.
Apart from that, you will need [flexxros](https://github.com/nilsbore/flexxros),
including dependencies, and also the private `sam_controllers` package:
```
pip3 install flexx rospkg
pip3 install tornado==5
sudo apt install ros-kinetic-rosmon
git clone -b new_topics https://github.com/nilsbore/flexxros.git
git clone https://gitr.sys.kth.se/smarc-project/sam_controllers.git
```
You then have to make sure to source your workspace in your `~/.bashrc`, i.e. something like:
```
source /path/to/catkin_ws/devel/setup.bash
```
You will now be able to run the robot interface, including controllers with
```
rosrun sam_stonefish_sim bringup.sh
```
Go to `localhost:8097` in your web browser and start the nodes there and steer the actuators!
