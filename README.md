# <img src="doc/img/Corc-logo.png" width="600"> 
# CANOpen Robot Controller [![GitHub Release](https://img.shields.io/github/v/release/UniMelbHumanRoboticsLab/CANOpenRobotController)](https://github.com/UniMelbHumanRoboticsLab/CANOpenRobotController/releases/) [![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

CORC is a free and open source robotic development software stack, written in C++.

The project was initiated at the University of Melbourne in partnership with Fourier Intelligence, however has welcomed (and continues to welcome) collaborators from all institutions. The project was initially developed to run on an [ExoMotus X2 Exoskeleton](doc/1.GettingStarted/AdvancedSimulationAndHardwareTesting.md) powered by a Beaglebone Black, however, the software is designed to be extensible to any embedded Linux and CANopen enabled robotic platform. More generally the software stacks provides way to build a real-time Linux controller for robots designed around a CAN bus. The repository currently includes suitable code and examples for the ArmMotus [M1](doc/1.GettingStarted/GSM1DemoMachine.md), [M2](doc/1.GettingStarted/GSM2DemoMachine.md), [M2 Pro](doc/1.GettingStarted/GSM2DemoMachine.md) and [M3 (EMU)](doc/1.GettingStarted/GSM3DemoMachine.md) rehabilitation devices, as well as the [FIT-HV waist exoskeleton from ULS Robotics](doc/1.GettingStarted/GSFITHVDemoMachine.md). It can run on either Linux desktop or laptop computers as well as Linux based SBCs.

## :heavy_plus_sign: What is CORC?

The project includes:

- A framework to represent multibody rigid robotic systems.
- An event driven state machine to develop custom applications (see [here](doc/3.Software/CustomApplication.md)).
- An implementation of [CANopen Socket](https://github.com/CANopenNode/CANopenSocket) to provide an interface between CAN enabled embedded Linux system and CANopen-based motor drivers and common sensors.
- Documentation (this page and associated ones and code Doxygen).
- Functional application examples. 



<table style="border:0px">
<tr style="border:0px">
<td style="border:0px;vertical-align: top;" width="70%">
The code (src folder) is structured into 3 levels:

1. **The Core Level:** Provides all common CORC functionalities. This includes CAN-level communications, generic Robot, Joint and Drive classes and generic StateMachine functionalities.
2. **The Hardware Level:** Provides "drivers" for the supported hardware. This includes definition of the various platforms (robots) supported but also motor drives (Copley, EPOS...) and various IOs.
3. **The Application Layer:** Defines the high level logic for the device, based on the implementation of a State Machine. 

Whilst the code can be modified at any level, this structure is designed to provide a degree of modularity. The Core level should not need to be changed. The Hardware level should only change with respect if the robot to be controlled changes. This is enforced by the source code folder structure - the files which should not need modification are placed in the `src/core` folder, and the remainder are placed in the `src/apps` and `src/hardware` folders.

Detailled folders description is provided on the diagram on the right.
<br>
</td>
<td style="border:0px;vertical-align: top;" width="30%">
Detailled project folders description (over for tooltips):<br>
 <br>
<ul><li><code title="CANOpenRobotController">CANOpenRobotController</code><ul><li><code title="YAML configuration files used by the different robot platforms to dynamically load some important parameters.">config</code></li></ul><ul><li><code title="Markdown main documentation. After using Doxigen you can also access the Doxygen documentation in html/index.html.">doc</code></li></ul><ul><li><code title="ROS and ROS2 launch files if using a ROS/ROS2 stateMachine.">launch</code></li></ul><ul><li><code title="External libraries used (and compiled) by CORC. Some are installed as git submodules.">lib</code></li></ul><ul><li><code title="ROS and ROS2 msg files if using a ROS stateMachine.">msg</code></li></ul><ul><li><code title="Set of useful scripts to initiate CAN or virtual CAN interfaces, upload files to BeagleBone or create a new state machine (app).">script</code></li></ul><ul><li><details open><summary><code title="Source files organised in three levels: 1)apps: contains application code (examples) and is where you can define your main logic; 2) hardware: contains platform (robot) and IO drivers to be modified/added to if you use non-supported hardware or new hardware functionalities; 3)core: with core CORC logic, likely not to be modified.">src</code></summary><ul><li><details><summary><code title="All the demo/example apps provided with CORC. Each app is a dedicated StateMachine with its own states. See the Get started pages for details. Your custom apps can also be placed in a different, external folder, for easier maintenance.">apps</code></summary><ul><li><code title="The very basic app example to start with to test CORC, the compilation process...">ExoTestMachine</code></li></ul><ul><li><code title="The EMU (M3) state machine example.">M3DemoMachine</code></li></ul><ul><li><code title="A template used by the createStateMachine script to ease the creation of a new custom StateMachine (app).">StateMachineTemplate</code></li></ul><ul><li><code title="other examples are available, check the Get started pages.">...</code></li></ul></details></li></ul><ul><li><details><summary><code title="Core code defining the main logic and common functionalities used by all CORC. Should very likely not be modified.">core</code></summary><ul><li><code title="The low-level CANOpen stack. Should very likely not be modified.">CANopen</code></li></ul><ul><li><code title="Convenience classes to manage network communication via FLNL. Should very likely not be modified.">network</code></li></ul><ul><li><code title="The base robot and joint classes. Should very likely not be modified.">robot</code></li></ul><ul><li><code title="The core/base StateMachine classes. Should very likely not be modified.">stateMachine</code></li></ul></details></li></ul><ul><li><details><summary><code title="All the different hardware drivers and interaction code, including the various supported robots definition, motor drives and other IOs.">hardware</code></summary><ul><li><code title="Motor drives classes. Include dedicated classes for Copley, Kinco and EPOS CANOpen based motor drives.">drives</code></li></ul><ul><li><code title="Drivers for various IOs (IMUs, encoders, keyboard...). See dedicated document page on supported IOs.">IO</code></li></ul><ul><li><details><summary><code title="Classes for each supported platform (robot) and their respective Joints where appropriate. Add or modify only if you need a new platform or new platform functionality (e.g. new sensor...).">platforms</code></summary><ul><li><code title="Classes for the FIT-HV waist exoskeleton (ULS robotics).">FITHVExo</code></li></ul><ul><li><code title="Classes for an example logging robot: only logging state.">LoggingRobot</code></li></ul><ul><li><code title="Classes for the ArmMotus M1 (Fourier Intelligence).">M1</code></li></ul><ul><li><code title="Classes for the ArmMotus M2 (Fourier Intelligence).">M2</code></li></ul><ul><li><code title="Classes for the ArmMotus M2 Pro (Fourier Intelligence).">M2P</code></li></ul><ul><li><code title="Classes for the ArmMotus EMU (Fourier Intelligence).">M3</code></li></ul><ul><li><code title="Classes for the X2 exoskeleton (Fourier Intelligence).">X2</code></li></ul></details></li></ul></details></li></ul></details></li></ul></li></ul>
</td>
</tr>
</table>

## :heavy_minus_sign: What CORC is not?

 - **Safe on its own:** While there are limits and safeties built-in CORC at various levels, CORC also gives full access to developers to the control of their hadware. As such it is the developer responsibility to ensure their robot is safe to use.
 - **A ready to use robot controller:** Demo machines for each device are provided as examples to get you started but are not final applications in any way. CORC requires some C++ development to get what you want.
 - **Clean:** The code base is functional and tested on all the hardware supported but it is not the best example of clean code. Syntax and style are not necessarily consistent accross the code base and some code might be redundant or unclear.

## :rocket: Getting started with CORC

See the detailed documentation [here](doc/1.GettingStarted/GettingStarted.md) with all you need to get started and a detailed list of available examples.


## :book: Next steps and specific documentation

### Hardware changes and CAN-USB adapters
See [here](doc/2.Hardware/ModifyingDevice.md) for information on required hardware modifications to get CORC controlling your device. See [this page](doc/2.Hardware/USBCANadapters.md) for notes on tested USB-CAN adapters and [this page](doc/2.Hardware/BBUse.md) for some notes and useful information on using BeagleBones. 

###  Generating the code documentation
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


### Developer resources

- CANopen Socket: https://github.com/CANopenNode/CANopenSocket
- CANopen CiA 402 (motor drive standard) resources: https://www.can-cia.org/can-knowledge/canopen/cia402/ and https://doc.synapticon.com/software/40/object_dict/all_objects/index.html#all-objects


## :grey_question: Questions and contributions

For problem or technical questions you can [raise an issue](https://github.com/UniMelbHumanRoboticsLab/CANOpenRobotController/issues). Please contact vcrocher[at]unimelb.edu.au with questions or suggestions for continuing development, or if you wish to be more involved in the planning/organisation of CORC.

If you find CORC useful and wish to reference it in your publications, please cite:

Fong, J. et al. (2022). CANopen Robot Controller (CORC): An Open Software Stack for Human Robot Interaction Development. In: Moreno, J.C., Masood, J., Schneider, U., Maufroy, C., Pons, J.L. (eds) Wearable Robotics: Challenges and Trends. WeRob 2020. Biosystems & Biorobotics, vol 27. Springer, Cham. https://doi.org/10.1007/978-3-030-69547-7_47



## :people_holding_hands: Contributors
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


## :balance_scale: License
Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 .
Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.

