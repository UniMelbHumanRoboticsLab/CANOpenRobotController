# Run a ROS application

This document describes how to setup the environment to run a CORC app with ROS support. It is assumed that target and host is the same(no cross-compilation).

## Install ROS and Catkin tools
If you haven't already installed ROS on your system. First install ROS Melodic.

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

## Create your catkin workspace
```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin build
```

## Clone CORC in your workspace
```bash
$ cd ~/catkin_ws/src
$ git clone git@github.com:UniMelbHumanRoboticsLab/CANOpenRobotController.git
```

## Build and Run the node

First make sure `USE_ROS` flag is set to `ON` in CMakeLists.txt:

```set(USE_ROS ON)```

and in CMakeLists.txt set your state machine that uses ROS (e.g., X2DemoMachine):

```set (STATE_MACHINE_NAME "X2DemoMachine")```

Build CORC:
```bash
$ cd ~/catkin_ws
$ catkin build CORC
$ source devel/setup.bash
```

Run X2DemoMachine:
```bash
$ rosrun CORC X2DemoMachine_APP
```


