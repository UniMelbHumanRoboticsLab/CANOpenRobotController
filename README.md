# CANOpen Robot Controller (CORC) Project

CORC is a free and open source robotic development software stack, written in C++.

The project was initiated at the University of Melbourne in partnership with Fourier Intelligence, however has welcomed (and continues to welcome) collaborators from all institutions. The project was initially developed to run on an ExoMotus X2 Exoskeleton powered by a Beaglebone Black, however, the software is designed to be extensible to any embedded Linux and CANopen enabled Robotic platform. The repository currently also includes code which has been run on the ArmMotus M1, M2 and M3 (EMU) rehabilitation devices, and using desktop or laptop Ubuntu installations.

## The CANOpen Robot Controller project includes:

- An extensible framework to represent multibody rigid robotic systems.
- An event driven state machine to develop custom applications (see [here](doc/3.Software/CustomApplication.md)).
- An implementation of [CANopen Socket](https://github.com/CANopenNode/CANopenSocket) to provide an interface between CAN enabled embedded Linux system and CANopen-based motor drivers/sensors.
- Documentation (this page and associated ones and code Doxygen).
- Functional application examples. 

### Project Overview

The code is structured into 3 levels:

1. **The CANopen Communications Level:** Provides the CAN-level communications, providing the mechanisms for the sending and receiving of PDO and SDO messages
2. **The Robot Level:** Defines the components of the Robot to be controlled, including the joints, associated drives, and input devices
3. **The Application Layer:** Defines the high level logic for the device, based on the implementation of a State Machine.

Whilst the code can be modified at any level, this structure is designed to provide a degree of modularity. The CANopen Communications level should not need to be changed. The Robot level should only change with respect if the robot to be controlled changes. This is loosely enforced by the source code folder structure - the files which should not need modification are placed in the `src/core` folder, and the remainder are placed in the `src/apps` and `src/hardware` folders. Note that in addition to the CANopen Communication code, the `src/core` folder also includes base classes which are derived from in the `src/apps` and `src/hardware` folders. 

## Getting started with CORC
See the detailed document [here](doc/1.GettingStarted/GettingStarted.md) 

## Next Steps
### Building a custom application with a custom state machine
See [this detailed explanation](doc/3.Software/CustomApplication.md) for instructions to customise an application or derive your own.

### Logging system (spdlog)
CORC relies on [spdlog](https://github.com/gabime/spdlog) for both general logging (terminal and in file) and for data logging.
See [here](doc/3.Software/Logging.md) for more info on using the logging system.

### ROS Support
See [here](doc/1.GettingStarted/AdvancedSimulationAndHardwareTesting.md) for instructions on how to build and run a CORC app with ROS support.

### Network communication
See [here](doc/3.Software/NetworkCommunication.md) for instructions on using libFLNL for communication.

### CAN-USB adapters
See [this page](doc/2.Hardware/USBCANadapters.md) for notes on tested USB-CAN adapters.

### Generating the code documentation
To generate the Doxygen documentation of the code you can simply run `doxygen Doxyfile` in the root folder. This will generate an HTML documentation in the `doc/html` folder.

## Developer Information

- CANopen Socket: https://github.com/CANopenNode/CANopenSocket
- CANopen CiA 402 (motor drive standard) ressources: https://www.can-cia.org/can-knowledge/canopen/cia402/ and https://doc.synapticon.com/software/40/object_dict/all_objects/index.html#all-objects


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

Please contact fong.j[at]unimelb.edu.au with questions or suggestions for continuing development, or if you wish to be more involved in the planning/organisation of CORC.

## License
Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 .
Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.

