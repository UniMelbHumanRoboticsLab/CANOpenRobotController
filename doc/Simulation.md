# Run a ROS application with catkin and simulation in Gazebo

This document describes how to set up the environment to run a CORC app with ROS support and a Gazebo simulation. It is assumed that the target and host are the same (no cross-compilation).

Currently, the simulation capabilities are implemented only for X2.
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
$ git clone git@github.com:UniMelbHumanRoboticsLab/CANOpenRobotController.git
$ git clone git@github.com:emekBaris/x2_description.git
$ git clone git@github.com:emekBaris/cob_gazebo_plugins.git
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

## Run
On real robot:
```bash
$ roslaunch CORC x2_real.launch
```

Simulation:
```bash
$ roslaunch CORC x2_sim.launch
```

