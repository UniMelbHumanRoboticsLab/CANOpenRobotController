# ![CORC logo](doc/img/Corc-logo.png) CANOpen Robot Controller ![GitHub Release](https://img.shields.io/github/v/release/UniMelbHumanRoboticsLab/CANOpenRobotController) [![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

CORC is a free and open source robotic development software stack, written in C++.

The project was initiated at the University of Melbourne in partnership with Fourier Intelligence, however has welcomed (and continues to welcome) collaborators from all institutions. The project was initially developed to run on an ExoMotus X2 Exoskeleton powered by a Beaglebone Black, however, the software is designed to be extensible to any embedded Linux and CANopen enabled robotic platform. More generally the software stacks provides way to build a real-time Linux controller for robots designed around a CAN bus. The repository currently includes suitable code and examples for the ArmMotus M1, M2, M2 Pro and M3 (EMU) rehabilitation devices, as well as the FIT-HV waist exoskeleton from ULS Robotics. It can run on either Linux desktop or laptop computers as well as Linux based SBCs.

## What is CORC?

The project includes:

- A framework to represent multibody rigid robotic systems.
- An event driven state machine to develop custom applications (see [here](doc/3.Software/CustomApplication.md)).
- An implementation of [CANopen Socket](https://github.com/CANopenNode/CANopenSocket) to provide an interface between CAN enabled embedded Linux system and CANopen-based motor drivers and common sensors.
- Documentation (this page and associated ones and code Doxygen).
- Functional application examples. 


The code is structured into 3 levels:

1. **The CANopen Communications Level:** Provides the CAN-level communications, providing the mechanisms for the sending and receiving of PDO and SDO messages
2. **The Robot Level:** Defines the components of the Robot to be controlled, including the joints, associated drives, and input devices
3. **The Application Layer:** Defines the high level logic for the device, based on the implementation of a State Machine.

Whilst the code can be modified at any level, this structure is designed to provide a degree of modularity. The CANopen Communications level should not need to be changed. The Robot level should only change with respect if the robot to be controlled changes. This is loosely enforced by the source code folder structure - the files which should not need modification are placed in the `src/core` folder, and the remainder are placed in the `src/apps` and `src/hardware` folders. Note that in addition to the CANopen Communication code, the `src/core` folder also includes base classes which are derived from in the `src/apps` and `src/hardware` folders. 

## What CORC is not?

 - **Safe on its own:** While there are limits and safeties built-in CORC at various levels, CORC also gives full access to developers to the control of their hadware. As such it is the developer responsibility to ensure their robot is safe to use.
 - **A ready to use robot controller:** Demo machines for each device are provided as examples to get you started but are not final applications in any way. CORC requires some C++ development to get what you want.
 - **Clean:** The code base is functional and tested on all the hardware supported but it is not the best example of clean code. Syntax and style are not necessarily consistent accross the code base and some code might be redundant or unclear.

## Getting started with CORC

See the detailed documentation [here](doc/1.GettingStarted/GettingStarted.md) with all you need to get started and a detailed list of available examples.


## Next steps and specific documentation

### Hardware changes and CAN-USB adapters
See [here](doc/2.Hardware/ModifyingDevice.md) for information on required hardware modifications to get CORC controlling your device. See [this page](doc/2.Hardware/USBCANadapters.md) for notes on tested USB-CAN adapters and [this page](doc/2.Hardware/BBUse.md) for some notes and useful information on using BeagleBones. 

### Generating the code documentation
Before starting to program with CORC it is highly recommended to generate the Doxygen documentation of the code. You can simply run `doxygen Doxyfile` in the root folder. This will generate an HTML documentation in the `doc/html` folder.

### Building a custom application with a custom state machine
See [this detailed explanation](doc/3.Software/CustomApplication.md) for instructions to customise an application or derive your own.

### Communication with external programs (network communication)
See [here](doc/3.Software/NetworkCommunication.md) for instructions and examples on using libFLNL for communication with program outside of CORC (such as Unity UI, Python scripts etc...) over a network communication.

### Logging system (spdlog)
CORC relies on [spdlog](https://github.com/gabime/spdlog) for both general logging (terminal and in file) and for data logging.
See [here](doc/3.Software/Logging.md) for more info on using the logging system.

### ROS Support
See [here](doc/1.GettingStarted/AdvancedSimulationAndHardwareTesting.md) for instructions on how to build and run a CORC app with ROS support.

### List of supported sensors
A list of available drivers for IO and sensors (IMU, F/T sensors...) is available on [this page](doc/2.Hardware/InputsList.md).


## Developer resources

- CANopen Socket: https://github.com/CANopenNode/CANopenSocket
- CANopen CiA 402 (motor drive standard) resources: https://www.can-cia.org/can-knowledge/canopen/cia402/ and https://doc.synapticon.com/software/40/object_dict/all_objects/index.html#all-objects


## Questions and contributions

For problem or technical questions you can [raise an issue](https://github.com/UniMelbHumanRoboticsLab/CANOpenRobotController/issues). Please contact vcrocher[at]unimelb.edu.au with questions or suggestions for continuing development, or if you wish to be more involved in the planning/organisation of CORC.

If you find CORC useful and wish to reference it in your publications, please cite:

Fong, J. et al. (2022). CANopen Robot Controller (CORC): An Open Software Stack for Human Robot Interaction Development. In: Moreno, J.C., Masood, J., Schneider, U., Maufroy, C., Pons, J.L. (eds) Wearable Robotics: Challenges and Trends. WeRob 2020. Biosystems & Biorobotics, vol 27. Springer, Cham. https://doi.org/10.1007/978-3-030-69547-7_47



## Contributors
The following individuals have made contributions to CORC:

- William Campbell
- Vincent Crocher
- Emek Barış Küçüktabak 
- Justin Fong
- Yue Wen
- Tim Haswell
- Xinliang Guo
- Benjamin von Snarski
- Hao Yu
- Zebin Huang
- Mingrui Sun


## License
Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 .
Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.

