# CANOpen Robot Controller (CORC) Project

CORC is a free and open source robotic development software stack. The project has been under development at the University of Melbourne in partnership with Fourier Intelligence for use with their X2 exoskeleton hardware. The work itself has been developed to run on a Beaglebone Black connected to the CAN bus of the X2 Exoskeleton, however, the software has been designed to be extensible to any embedded Linux and CANopen enabled Robotic platform.

**Note (12/5/2020):** At this stage, this software has not been tested on physical hardware. 

## The CANOpen Robot Controller project includes:

- An extensible framework to represent multibody robotic systems.
- Event driven state machine to develop custom applications. 
- Implementation of [CANopen Socket](https://github.com/CANopenNode/CANopenSocket) to provide an interface between CAN enabled embedded Linux system and CANopen-based motor drivers/sensors.
- Documentation
- Functional application examples.

### Project Overview

The code is structured into 3 levels:

1. CANopen Communications Level
  - This provides the CAN-level communications, providing the mechanisms for the sending and receiving of PDO and SDO messages
2. The Robot Level
  - This defines the components of the Robot to be controlled, including the joints, associated drives, and input devices
3. The State Machine and Trajectory Generator
  - This defines the high level logic for the device, and the trajectories to be used. 

Whilst the code can be modified at any level, this structure is designed to provide a degree of modularity. The CANopen Communications level should not need to be changed, except potentially for advanced reasons (e.g. changes to the Object Dictionary). The Robot level should only change with respect if the robot to be controlled changes. This is somewhat represented by the source code folder structure - the files which should not need modification are placed in the `src/libs` folder, and the remainder are placed in the `src/apps` folder. The code is written in C++, and takes advantage of the object-oriented nature of the language to enable this structure - thus there are base classes in the `libs` folder which are derived in the `apps` folder. 

## Getting started with CORC

The following instructions detail the building and testing of a simple test state machine (ExoTestMachine.cpp), the source code can be found in the apps folder of the root directory.

The current recommended and tested environmen is Debian Jessie 8.11 on a BeagleBone Black [Firmware](http://beagleboard.org/latest-images). Theoretically, this can be built on other distributions and Linux platforms as well, but they have not yet been tested.

Workbench build environments for most platforms can be found [here](https://exoembedded.readthedocs.io/en/latest/workbench/)

### How to get the Project

​
Clone the project from git repository(CANopenSocket used in robot homing sequence):
​
\$ git clone https://github.com/UniMelb-Human-Robotics-Lab/CANOpenRobotController
​

### Build ExoTestMachine

​
$ cd <CANOpenRobotController_directory>
    $ make exe
​

### Transfer files to Linux platform

​

- Download [FileZilla](https://filezilla-project.org/) and connect to BeagleBone.
  ​
  > Username: debian
  > ​
  > Password: temppwd
  > ​
  > Transfer executable(build/EXO_ROBOT_2020) and initRobot folder.
  > ​

## Run Virtual ExoTestMachine

​

- SSH into BeagleBone ([tutorial](https://elinux.org/Beagleboard:Terminal_Shells)) or similar Linux board, use the same user and password as above.
- Initialize Virutal CAN device to bind to and run candump([candump manpage](https://manpages.debian.org/testing/can-utils/candump.1.en.html)) on the VCAN interface.
  ​

```bash
  cd initRobot
  ./initVCAN
```

​

- Launch second instance of terminal and ssh into device to Launch application
  ​

```bash
  cd build
  sudo ./EXO_APP_2020
```

​

- Terminal one should display CAN message output on VCAN from our EXO_APP application output. On startup init PDO messaging should be sent and appear as follows:
  ​

```bash
vcan0 704 [1] 00
vcan0 184 [2] 00 00 # PDO message
vcan0 704 [1] 05
```

​
Follow terminal instructions using your keyboard to run through test stateMachine.
​

```bash
==================================
 WELCOME TO THE TEST STATE MACHINE
==================================
==================================
 PRESS S to start program
==================================
​
```

​
Notice that terminal one(candump) sends out messages corresponding with application movement commands as follows:
Specifics of CAN messages can be found in the drive.h source files.
​

```bash
vcan0  601   [8]  2F 60 60 00 01 00 00 00 #initi position control SDO
vcan0  764   [1]  05
vcan0  601   [8]  80 60 60 00 00 00 04 05 #initi position control SDO
```

​

<!-- ## Run ExoTestMachine on X2
​
Running on the X2 Robot is very similar to the virutal CAN set up, with one adjustment and -->

​

<!-- ### TODO: Event Driven State machine
explain me -->

​

## Developer Information

​

- Detailed documentation: https://exoembedded.readthedocs.io/en/latest/
- Source code documentation: https://capstonealex.github.io/exo/index.html
- Project Repository: https://github.com/capstonealex/exo
- CANopen Socket: https://github.com/CANopenNode/CANopenSocket
  ​

## License

​
Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at
​
http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.
​
This program is distributed in the hope that it will be us

<!-- ## Maintainers -->
