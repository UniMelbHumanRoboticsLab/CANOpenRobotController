# CANOpen Robot Controller (CORC) Project

CORC is a free and open source robotic development software stack, written in C++. The project has been under development at the University of Melbourne in partnership with Fourier Intelligence for use with their X2 exoskeleton hardware. The project was developed to run on a Beaglebone Black connected to an X2 Exoskeleton, however, the software is designed to be extensible to any embedded Linux and CANopen enabled Robotic platform.

> Note (12/5/2020): At this stage (due to issues associated with COVID-19), this software has not been tested on physical hardware. 

## The CANOpen Robot Controller project includes:

- An extensible framework to represent multibody robotic systems.
- Event driven state machine to develop custom applications. 
- Implementation of [CANopen Socket](https://github.com/CANopenNode/CANopenSocket) to provide an interface between CAN enabled embedded Linux system and CANopen-based motor drivers/sensors.
- Documentation
- Functional application examples.

### Project Overview

The code is structured into 3 levels:

1. **CANopen Communications Level:** Provides the CAN-level communications, providing the mechanisms for the sending and receiving of PDO and SDO messages
2. **The Robot Level:** Defines the components of the Robot to be controlled, including the joints, associated drives, and input devices
3. **The State Machine and Trajectory Generator:** Defines the high level logic for the device, and the trajectories to be used. 

Whilst the code can be modified at any level, this structure is designed to provide a degree of modularity. The CANopen Communications level should not need to be changed. The Robot level should only change with respect if the robot to be controlled changes. This is loosely enforced by the source code folder structure - the files which should not need modification are placed in the `src/libs` folder, and the remainder are placed in the `src/apps` folder. Due to this, thus there are base classes in the `libs` folder which are derived in the `apps` folder. 

## Getting started with CORC

The following instructions detail the building and testing of a simple test state machine for the X2 Exoskeleton (ExoTestMachine.cpp). This state machine simply simulates an exoskeleton which can move between sitting and standing postures, running in position control, triggered by keyboard events. 

### Before you start
These instructions assume that you have a suitable test platform, workbench environment. 

Specifically, these instructions have been written for (and tested on) a BeagleBone Black running Debian Jessie 8.11 [Firmware](http://beagleboard.org/latest-images). Theoretically, this can be built on other distributions and Linux platforms as well, but they have not yet been tested.

Workbench build environments for most platforms can be found [here](https://exoembedded.readthedocs.io/en/latest/workbench/)

### How to get the Project

Clone the project from git repository:
​
\$ git clone https://github.com/UniMelb-Human-Robotics-Lab/CANOpenRobotController
​
This repository includes all the sources files required for this example. 

### Building ExoTestMachine

$ cd <CANOpenRobotController_directory>
    $ make exe
​
The makefile is configured to compile an executable `EXO_ROBOT_2020` using the `arm-linux-gnueabihf-g++` compiler. Note that this requires an appropriately configured workbench environment (see Section "Before you start").

### Transferring files to the Linux platform

The recommended method of transferring files to the BeagleBone is FTP. 

Using an FTP Client (if you do not have one - or a preferred client, [FileZilla](https://filezilla-project.org/) is reasonable), connect to the BeagleBone. By default, when the BeagleBone is connected to a computer using USB, it is configured to:
  
  > IP: 192.168.7.2 (Windows) or 192.168.6.2 (OSX)
  >​
  > Username: debian
  > ​
  > Password: temppwd

Using the client, transfer the built executable in `build/EXO_ROBOT_2020`, along with the contents of the `initRobot` folder, to the BeagleBone.

> Note: The `initRobot` folder contains scripts for setting up the CAN interfaces that CORC uses for communication

You will need to change the permissions of the executables to executable. You can do this using the the `chmod +x` command. e.g.

$ chmod +x EXO_ROBOT_2020

This must be repeated for the `.sh` scripts as well. 

## Run Virtual ExoTestMachine

To run the ExoTestMachine, open your preferred terminal window and SSH into the the BeagleBone ([tutorial](https://elinux.org/Beagleboard:Terminal_Shells)), using the same username and password, e.g:

$ ssh debian@192.168.7.2

Initialize Virutal CAN device to bind to and run candump([candump manpage](https://manpages.debian.org/testing/can-utils/candump.1.en.html)) on the VCAN interface - this initialises a virtual CAN interface, and prints the contents of the bus to the terminal window. 

```bash
  cd initRobot
  ./initVCAN
```​

> Note: This can be changed to use a non-virtual CAN interface with some minor modifications to the code.

SSH into the BeagleBone in a second terminal window to launch the application ​

```bash
  cd build
  sudo ./EXO_APP_2020
```
> Note: Superuser privileges (`sudo`) are required due to the use of real time threads in the application. 

The first terminal one should display CAN messages on VCAN from the `EXO_APP_2020` application output. On startup init PDO messaging should be sent and appear as follows:
  ​
```bash
vcan0 704 [1] 00
vcan0 184 [2] 00 00 # PDO message
vcan0 704 [1] 05
```
​
Follow terminal instructions using your keyboard in the second terminal instance to run through test stateMachine.​

```bash
==================================
 WELCOME TO THE TEST STATE MACHINE
==================================
==================================
 PRESS S to start program
==================================
​
```

The first terminal instance (running candump) should display PDO messages corresponding to changes to the commanded motor positions as follows:

```bash
vcan0  301   [8]  2F 60 60 00 00 00 00 00 #
vcan0  302   [8]  80 60 60 00 00 00 00 00 #
vcan0  303   [8]  2F 60 60 00 00 00 00 00 #
vcan0  304   [8]  80 60 60 00 00 00 00 00 #
```

> Note these TPDO messages are configured in the Object Dictionary, with configuration SDOs to be sent during the Robot Initialisation stage. Details for the default set of PDOs can be found in the drive.h source files.
​

<!-- ## Run ExoTestMachine on X2
​
Running on the X2 Robot is very similar to the virutal CAN set up, with one adjustment and -->

​

<!-- ### TODO: Event Driven State machine
explain me -->

​

## Developer Information

- Detailed documentation: https://exoembedded.readthedocs.io/en/latest/
- Source code documentation: https://capstonealex.github.io/exo/index.html
- Project Repository: https://github.com/capstonealex/exo
- CANopen Socket: https://github.com/CANopenNode/CANopenSocket

Feel free to contact fong.j[at]unimelb.edu.au with questions or suggestions for continuing development.

## License

​
Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at
​
http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.
​
This program is distributed in the hope that it will be us

<!-- ## Maintainers -->
