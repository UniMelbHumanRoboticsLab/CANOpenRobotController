# Cross-compile and run a ROS application

This document describes how to cross-compile and setup the environment to run a CORC app with ROS support. It assumes that you have a deployment target and a host machine.

It assumes that you have ROS installed on both the host and target machines. For the target, the debian/ubuntu package `ros-base-dev` can be used.

## ROS environment setup
In order to run on both host and target, ROS should be configured. We assume that roscore will run on the host machine.
On the host:
```bash
	$ export ROS_MASTER_UI=http://[host_ip]:11311
	$ export ROS_HOSTNAME=[host_hostname]
	$ export ROS_IP=[host_ip]
```
On the target:
```bash
	$ export ROS_MASTER_UI=http://[host_ip]:11311
	$ export ROS_HOSTNAME=[target_hostname]
	$ export ROS_IP=[target_ip]
```
>These can be added to the .bashrc of the host and target respectivelly to be automatically setup.

>If the hostnames are not defined, edit the /etc/hosts files (see ROS [documentation](http://wiki.ros.org/ROS/Tutorials/MultipleRemoteMachines) or this [example](https://github.com/mktk1117/six_wheel_robot/wiki/Communication-between-Raspberry-Pi-and-PC-\(ROS\)) for more details).


## Cross-compilation
In order to cross-compile CORC with ROS support from the host machine, you first need to create a sysroot: a copy of the basic filesystem of the target. To do so, create a folder named `corc-target-sysroot`. Using scp or sftp, copy the following folders from the target to this folder:
```bash
	/usr/
	/lib/
```
>It is recommended to do this on a minimal system to reduce the size of these folders. Typically graphical packages (xserver etc...) which may be installed by default on the BB distribution can be uninstall before.

Once the files are copied, in a terminal, set the directory of this folder:
```bash
	$ export CORC_CMAKE_SYSROOT=/path/to/my/folder/corc-target-sysroot
```

Edit the `CMakeFileLists.txt` to select the app to build and ensure that the line `set(USE_ROS ON)` is NOT commented and set to ON.

Cross-compile by passing the sysroot path argument to cmake (and the toolchain argument). In the root CANOpenRobotController:
```bash
	$ rm -r build && mkdir build && cd build/ && cmake -DCMAKE_SYSROOT=/path/to/my/folder/corc-target-sysroot -DCMAKE_TOOLCHAIN_FILE=../armhf.cmake ..
	$ make -j8
```
Voila !

## Run the app
On the host machine, start ROS:
```bash
	$ roscore
```
In a second terminal, start rqt to visualise the active ROS graph:
```bash
	$ rqt
```
Plugins->Introspection->Node Graph.

Once copied on the target, the APP can then be run normally, simply adding an extra command line parameter to specify the address of the master node:
```bash
	$ sudo X2DemoMachineApp __master:=$ROS_MASTER_URI
```

You should now see in rqt the nodes corresponding to the CORC app.