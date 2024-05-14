# ROS2 Application - Instructions

Minimal instructions to compile and run a CORC application as a ROS2 node able to publish the robot state.

This page assumes you have an Ubuntu system with ROS2 Humble installed (may work with other ROS2 versions but not tested). See [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) for ROS2 installation instructions.
Testing the application also assumes you have an ExoMotus X2 Exoskeleton.


> Note : these instructions are for ROS2. See [here](AdvancedSimulationAndHardwareTesting.md) for a ROS1 example.

## Compilation


```bash
$ source /opt/ros/humble/setup.bash
```

In CORC root directory, rename ros2 package configuration file
```bash
$ mv package.ros2.xml package.xml
```


Edit the CMAKEFileList.txt to select a ROS2 state machine:

```
...
...
#include(src/apps/X2DemoMachine/app.cmake)
include(src/apps/X2ROS2DemoMachine/app.cmake)
#include(src/apps/LoggingDevice/app.cmake)
#include(../myStateMachineApp/app.cmake) ## example only, need to be defined

# Comment to use actual hardware, uncomment for a nor robot (virtual) app
set(NO_ROBOT OFF)

...
...
```

In the state machine folder, open the `app.cmake` and ensure it is configured to use ROS2:
```
################################## USER FLAGS ##################################

## Which platform (robot) is the state machine using?
## this is the correspondig folder name in src/hardware/platform to use
set(PLATFORM X2)

## Compile for ROS 2
set(ROS 2)

################################################################################
...

```


From the **CANOpenController parent folder**, compile the package:
```bash
$ colcon build
$ source install/setup.bash
```


## Run the node 
```bash
$ ros2 run CORC X2ROS2DemoMachine_APP
```

Here CORC is the package name and the CORC app name is the executable name.

See the content of the `src/apps/X2ROS2DemoMachine` folder to see how to interact with ROS2 from within CORC.
