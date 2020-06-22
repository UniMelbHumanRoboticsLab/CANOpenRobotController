# CANOpen Robot Controller (CORC) Project

CORC is a free and open source robotic development software stack, written in C++.

The project has been under development at the University of Melbourne in partnership with Fourier Intelligence. The project was developed to run on an X2 Exoskeleton powered by a Beaglebone Black, however, the software is designed to be extensible to any embedded Linux and CANopen enabled Robotic platform.

> Note (12/5/2020): At this stage, this software has not been tested on physical hardware due to lab access limitations due to COVID-19.

## The CANOpen Robot Controller project includes:

- An extensible framework to represent multibody robotic systems.
- An event driven state machine to develop custom applications (see [here](doc/StateMachine.md))
- An implementation of [CANopen Socket](https://github.com/CANopenNode/CANopenSocket) to provide an interface between CAN enabled embedded Linux system and CANopen-based motor drivers/sensors.
- [Documentation](https://unimelb-human-robotics-lab.github.io//CANOpenRobotController/index.html)
- Functional application examples.

### Project Overview

The code is structured into 3 levels:

1. **CANopen Communications Level:** Provides the CAN-level communications, providing the mechanisms for the sending and receiving of PDO and SDO messages
2. **The Robot Level:** Defines the components of the Robot to be controlled, including the joints, associated drives, and input devices
3. **The Application Layer:** Defines the high level logic for the device, based on the implementation of a State Machine.

Whilst the code can be modified at any level, this structure is designed to provide a degree of modularity. The CANopen Communications level should not need to be changed. The Robot level should only change with respect if the robot to be controlled changes. This is loosely enforced by the source code folder structure - the files which should not need modification are placed in the `src/core` folder, and the remainder are placed in the `src/apps` and `src/hardware` folders. Note that in addition to the CANopen Communication code, the `src/core` folder also includes base classes which are derived from in the `src/apps` and `src/hardware` folders. 

## Getting started with CORC

The following instructions detail the building and testing of a simple test state machine for the X2 Exoskeleton (ExoTestMachine.cpp). This state machine simply simulates an exoskeleton which can move between sitting and standing postures, running in position control, triggered by keyboard events.

### Before you start

These instructions assume that you have a suitable test platform (i.e. a Target), and a workbench environment (i.e. a Host).

Specifically, the target that these instructions have been written for (and tested on) is a BeagleBone Black running Debian Stretch 9.5 [Firmware](http://beagleboard.org/latest-images). Theoretically, this can be built on other distributions and Linux platforms as well, but they have not yet been tested.

These instructions have been tested on a number of different host workbench build environments, but instructions on setting these up can be found [here](https://exoembedded.readthedocs.io/en/latest/workbench/). We recommend trying option A if possible.

### How to get the Project

On the host, clone the project from git repository:
​

```bash
$ git clone https://github.com/UniMelb-Human-Robotics-Lab/CANOpenRobotController
```

This repository includes all the sources files required for this example.

### Building ExoTestMachine

On the host, generate a cross-compiled executable (suitable for running on a Beaglebone Black) using the following commands:
```bash
$ mkdir build
$ cd build
$ cmake -DCMAKE_TOOLCHAIN_FILE=../armhf.cmake ..
$ make
```
> Note: To run on a Windows machine, you may need to also add the `-G "Unix Makefiles` flag to the `cmake` command (i.e. `cmake -G "Unix Makefiles -DCMAKE_TOOLCHAIN_FILE=../armhf.cmake ..`). This forces the Unix Makefile format, rather than the default nmake behaviour on Windows. 

or in short:
```bash
$ mkdir build && cd build/ && cmake -DCMAKE_TOOLCHAIN_FILE=../armhf.cmake ..
```
> Note that this requires an appropriately configured toolchain (`arm-linux-gnueabihf-` toolchain). See "Before you start" to setup a proper workbench if required.

Alternatively, if your host is a Linux-based machine, you can build the executable for local test as follows:

```bash
$ cd <CANOpenRobotController_directory/build>
$ cmake ../
$ make
```
> Note: there are also some additional build rules to build additional tests, which are still to be completed

The makefile is configured to compile an executable `ExoTestMachine_APP` using the default C/C++ compilers. 


### Building a custom application with a custom state machine

Derive the StateMachine class to a custom one (see [here for details](doc/StateMachine.md)), named `MyCustomStateMachine`, in files named `MyCustomStateMachine.h/cpp`. Place them in a dedicated subfolder in the apps folder.

Edit CMakeLists.txt and change the entry `set (STATE_MACHINE_NAME "ExoTestMachine")` with `set (STATE_MACHINE_NAME "MyCustomStateMachine")`.

That's it, simply use the same build procedure.

### Transferring files to the Linux platform

The recommended method of transferring files to the BeagleBone is FTP.

Using an FTP Client on the Host (if you do not have one - or a preferred client, [FileZilla](https://filezilla-project.org/) is reasonable), connect to the target (the BeagleBone). By default, when the BeagleBone is connected to a computer using USB, it is configured to:

- **IP Address:** 192.168.7.2 (Windows) or 192.168.6.2 (OSX and Linux)
- **Username:** debian
- **Password:** temppwd

On the host, using the FTP client, transfer the build executable in `build/ExoTestMachine_APP`, along with the contents of the `initRobot` folder, to the Beaglebone.

Alternatively, you can use the [script/uploadBB.sh](script/uploadBB.sh) to automatically upload the content of the script folder and the build/\*APP to the BeagleBone.

> Note: The `initRobot` folder contains scripts for setting up the CAN interfaces that CORC uses for communication

## Run Virtual ExoTestMachine

To run the ExoTestMachine, open your preferred terminal window and SSH into the the BeagleBone ([tutorial](https://elinux.org/Beagleboard:Terminal_Shells)). This will provide terminal access to the target, on the host. This can be done using the same username and password, e.g:

```bash
$ ssh debian@192.168.7.2
```

At this point, you will need to change the permissions of the executables to executable. You can do this using the the `chmod +x` command on the target. e.g.

```bash
$ chmod +x ExoTestMachine_APP
```

This must be repeated for the `.sh` scripts as well.

Initialize the Virtual CAN device to set up, bind to and run candump ([candump manpage](https://manpages.debian.org/testing/can-utils/candump.1.en.html)) on the VCAN interface.

```bash
$  cd initRobot
$  ./initVCAN
```

This initialises a virtual CAN interface, and prints the contents of the bus to the terminal window.

> Note: This can be changed to use a non-virtual CAN interface, but this requires some minor changes to the code before compilation, and the use of the `X2_startCAN.sh` script instead.

SSH into the BeagleBone in a second terminal window to launch the application:

```bash
$$  cd build
$$  sudo ./ExoTestMachine_APP
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
vcan0  301   [8]  2F 60 60 00 00 00 00 00
vcan0  302   [8]  80 60 60 00 00 00 00 00
vcan0  303   [8]  2F 60 60 00 00 00 00 00
vcan0  304   [8]  80 60 60 00 00 00 00 00
```

> Note these TPDO messages are configured in the Object Dictionary, with configuration SDOs to be sent during the Robot Initialisation stage. Details for the default set of PDOs can be found in the drive.h source files.

As the simulated device is in position control, these messages should print whenever the system moves from sit to stand or stand to sit, with no messages transmitted whilst the device is stationary. CORC has provisions for velocity and torque control, but these are not included in this example for ease of simulation.

​<!-- ## Run ExoTestMachine on X2
​
Running on the X2 Robot is very similar to the virutal CAN set up, with one adjustment and -->

<!-- ### TODO: Event Driven State machine
explain me -->

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
