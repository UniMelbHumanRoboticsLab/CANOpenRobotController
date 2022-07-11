# Run a ROS application with catkin and simulation in Gazebo

This document describes how to control multiple robots simultaneously.

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

## Build

Create a catkin workspace if you don't have one yet:
```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin build
```

Clone CORC and the required packages into your workspace. If you don't have access, email `baris.kucuktabak@u.northwestern.edu`
```bash
$ cd ~/catkin_ws/src
$ git clone github.com/ywen3/CANOpenRobotController.git
$ git clone git@github.com:emekBaris/multi_robot_interaction.git
$ git checkout devel/M1_ROS
```

Make sure `USE_ROS` flag is set to `ON` in CMakeLists.txt:
```set(USE_ROS ON)```

Set your state machine that uses ROS (e.g., M1DemoMachine):
```set (STATE_MACHINE_NAME "M1DemoMachine")```

Build CORC and `multi_robot_interaction` package:
```bash
$ cd ~/catkin_ws
$ catkin build CORC
$ catkin build multi_robot_interaction
$ source devel/setup.bash
```
`multi_robot_interaction` is not declared as dependency, yet. So, it should be built separetely.

## Run
In `multi_m1_real.launch`, set robot's namespaces and CAN devices. Then, execute:
```bash
$ roslaunch CORC multi_m1_real.launch
```



