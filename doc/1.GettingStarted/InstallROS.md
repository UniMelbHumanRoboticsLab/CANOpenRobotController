## Install ROS and Catkin tools
We suggest installing either ROS Melodic or ROS Noetic depending on your Ubuntu version. While Melodic is compatible with
 Ubuntu 18.04, Noetic is compatible with Ubuntu 20.04.

Update your Debian package index:
```bash
$ sudo apt update
```

Install ROS Melodic or Noetic Desktop-Full:
```bash
$ sudo apt install ros-<melodic or noetic>-desktop-full
```

Install Catkin tools for:
```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
$ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install python-catkin-tools (For melodic)
$ sudo apt-get install python3-catkin-tools (For noetic)
```

Install ros_control:
```bash
$ sudo apt-get install ros-<melodic or noetic>-ros-control ros-<melodic or noetic>-ros-controllers
```