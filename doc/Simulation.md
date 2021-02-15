# Run a ROS application with catkin and simulation in Gazebo

This document describes how to set up the environment to run a CORC app with ROS support and a Gazebo simulation. It is assumed that the target and host are the same (no cross-compilation).

This simulation is written for X2 only, and is supported on Ubuntu 18.04 and with ROS Melodic. 

> Note: It is possible to run this on Ubuntu 20.04 with ROS Noetic, however, installation instructions will be different (installing the Noetic rather than Melodic packages below)

## Dependencies

### Install ROS and Catkin tools
If you haven't already installed ROS on your system, first install ROS Melodic.

Update your Debian package index:
```bash
$ sudo apt update
```

Install ROS Melodic Desktop-Full:
```bash
$ sudo apt install ros-melodic-desktop-full
```

Install Catkin tools:
```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
$ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install python-catkin-tools
```

> Note: On Ubuntu 20.04/ROS Noetic, `python3-catkin-tools` is required instead

Install ros_control:
```bash
$ sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers
```

The following packages are also required for the simulation. If you don't have access to them, email `baris.kucuktabak@u.northwestern.edu`

* [x2_description](https://github.com/emekBaris/x2_description)
* [cob_gazebo_plugins](https://github.com/emekBaris/cob_gazebo_plugins)

## Build

Create a catkin workspace if you don't have one yet:
```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin build
```

Clone CORC and the required packages into your workspace:
```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/UniMelbHumanRoboticsLab/CANOpenRobotController.git
$ git clone https://github.com/emekBaris/x2_description.git
$ git clone https://github.com/emekBaris/cob_gazebo_plugins.git
```

Make sure `USE_ROS` flag is set to `ON` in CMakeLists.txt:
```set(USE_ROS ON)```

If you will test on the real robot:
```set(NO_ROBOT OFF)```

If you would like to do a simulation: 
```set(NO_ROBOT ON)```

Set your state machine that uses ROS (e.g., X2DemoMachine):
```set (STATE_MACHINE_NAME "X2DemoMachine")```

Build CORC:
```bash
$ cd ~/catkin_ws
$ catkin build CORC
$ source devel/setup.bash
```

> Note: On Ubuntu 20.04/ROS Noetic, we have found that you need to comment out the `#include <cob_gazebo_ros_control/hwi_switch_robot_hw_sim.h>` and `#include <gazebo_ros_control/gazebo_ros_control_plugin.h>` lines in `cob_gazebo_plugins/cob_gazebo_ros_control/include/cob_gazebo_ros_control/hwi_switch_robot_hw_sim.h` before running `catkin build CORC` once, before uncommenting them and running them again. We are not sure why at this stage.

## Run
On real robot:
```bash
$ roslaunch CORC x2_real.launch
```

Simulation:
```bash
$ roslaunch CORC x2_sim.launch
```

