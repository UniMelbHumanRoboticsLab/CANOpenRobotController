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


Edit the CMAKEFileList.txt to set ROS2 option and to select a ROS2 state machine:

```
...
...
#set (STATE_MACHINE_NAME "ExoTestMachine")
#set (STATE_MACHINE_NAME "M1DemoMachine")
#set (STATE_MACHINE_NAME "M1DemoMachineROS")
#set (STATE_MACHINE_NAME "M2DemoMachine")
#set (STATE_MACHINE_NAME "M3DemoMachine")
#set (STATE_MACHINE_NAME "X2DemoMachine")
set (STATE_MACHINE_NAME "X2ROS2DemoMachine")
#set (STATE_MACHINE_NAME "LoggingDevice")

# Use this if your state machine code folder is not in CORC 'src/apps/' folder.
# Can be a relative or absolute path.
#set (STATE_MACHINE_PATH "../")

# Comment to use actual hardware, uncomment for a nor robot (virtual) app
set(NO_ROBOT OFF)

# ROS Flag: set to 0 for a no ROS stateMachine, 1 for ROS 1 (use catkin build) and 2 for ROS2 (use colcon build)
# Remember to rename select appropriate package.xml too
set(ROS 2)
...
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