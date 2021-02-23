# ExoForce Force Control Package

## Setup
*Tested on Ubuntu 20.04*
- install python>=3.6
- install [ROS Noetic Ninjemys](http://wiki.ros.org/noetic/Installation/Ubuntu)
- install [Phidget22 libraries](https://www.phidgets.com/docs/OS_-_Linux#Quick_Downloads)
```bash
# Creating ROS workspace
mkdir -p force_ws/src

# Cloning repos to src directory
cd force_ws/src
git clone https://github.com/Roboy/roboy_communication.git -b roboy3 # For the WS20 we need the following branch e0b1c2de7cd2084c87208fb10f6a004a3f7666fb
git clone https://github.com/Roboy/bulletroboy.git

# Building packages in workspace
cd force_ws
source /opt/ros/noetic/setup.bash
catkin_make

# Sourcing workspace overlay (ROS Wiki recommends opening a new terminal before this step)
source /opt/ros/noetic/setup.bash
cd force_ws
. devel/setup.bash

# Install python packages
pip3 install -r requirements.txt
```

## Usage
To be able to run any node after the package is built, you need to source the workspace overlay.

Run the force control node.
```bash
./force_ws/src/exoforce_force_control/src/force_control.py
```
