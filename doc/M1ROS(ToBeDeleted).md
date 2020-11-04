# Run a ROS application with catkin and simulation in Gazebo

This document describes how to set up the environment to run a CORC app with ROS support. It is assumed that the target and host are the same (no cross-compilation).

This is written for M1 only, and is supported on Ubuntu 18.04 and with ROS Melodic. 

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
$ git clone github.com/ywen3/CANOpenRobotController.git
```

Make sure `USE_ROS` flag is set to `ON` in CMakeLists.txt:
```set(USE_ROS ON)```

If you will test on the real robot:
```set(NO_ROBOT OFF)```

Set your state machine that uses ROS (e.g., M1DemoMachine):
```set (STATE_MACHINE_NAME "M1DemoMachine")```

Build CORC:
```bash
$ cd ~/catkin_ws
$ catkin build CORC
$ source devel/setup.bash
```

## Run
On real robot:
```bash
$ roscore
```
On a different terminal:
```bash
$ rosrun CORC M1DemoMachine_APP
```

#### Verify the node is created and Joint states are published
You should see m1_node after running:
```bash
$ rosnode list 
```
You should see joint pos/vel/torque:
```bash
$ rostopic echo /m1/joint_states
```

