# First Steps

There are a number of getting started programs and setups, depending on your intended and currently available hardware, and final eventual goal. Key decision points to be made are:


## Choice of Deployment Environment
CORC is flexible in that it can run on any computer with a Controller Area Network (CAN) interface running a Linux Distribution. However, there are two typical deployment scenarios:

1. An a dedicated embedded computer (such as a Beaglebone Black) to run CORC and solely act as the robot controller
2. On a desktop or laptop computer running Linux, connected to the robotic device via a USB-CAN interface

Examples of use of a dedicated embedded computer are when the robotic device is mobile (such as an assistive exoskeleton) or when separation between the robot controller and the user interface is desired (such as on deployment-ready rehabilitation devices). Alternatively, you may choose to use a desktop computer if you wish to run more powerful control algorithms on your device, or if your development is more experimental in nature. With appropriate choice of software packages, it is also possible to transition deployment from a desktop computer to an embedded computer. 

CORC has been well tested on Beagle Bones and dedicated setup instructions are available on [this page](../2.Hardware/BBUse.md). 

Instructions specific to cross-compiling and deploying on a different target (such as a BeagleBone) are tagged with **[DEPLOY-REMOTE]** whereas instructions specific to this setup are tagged with **[DEPLOY-LOCAL]**. 

> Note that cross-compiling a ROS app is theoretically possible but not described in this documentation.
> Note that it is not recommended to run CORC in a Virtual Machine due to latency and limited performance. While this works well for development, testing and simulation (typically to go through those tutorials), this won't give good performances if controlling an actual robot. If you develop under a VM consider using a Beagle Bone or similar SBC. 


## Choice of Development Environment
Independent of the deployment environment, a choice may also be made regarding the development environment. While theoretically possible to develop and cross-compile CORC from Windows it is highly recommended and easier to use a Linux system as development environment. 

Your development system may or may not be the same as your deployment computer. If you can consider using a Linux Virtual Machine (VM) as the development environement, it is not recommended to run CORC (except for quick simulations tests) on a VM: the latency and poor performance makes it not suitable to control a real robot.

See [Linux Workbench Setup page](InstallLinux.md) for instructions to setup a Linux environement with the necessary tools (essentially git, C/C++ compiler and can-utils).

> Note: Old instructions for developping under Windows are available [here](InstallWindows.md) for reference but not recommended. Instead, it is possible to use a Virtual Machine running Ubuntu and cross-compile for a Beagle Bone or similar Single Board Computer. 


## Code documentation
As you walk through the examples below, you may want to generate the code documentation. CORC code is documented using [Doxygen](https://www.doxygen.nl/). You can generate the documentation by running `doxygen Doxyfile` in the root folder. This will generate an HTML documentation in the `doc/html` folder: simply open doc/html/index.html in your browser. 
While not necessary to run the examples, it is highly recommended as soon as you start developping yourself (e.g. modifying code, developping your own state machine etc...).


## Getting the Project
On your development computer, clone the project from git repository. You can do this using the command line by first navigating to an appropriate folder, and typing the command:
```bash
$ git clone --recursive https://github.com/UniMelbHumanRoboticsLab/CANOpenRobotController
```

This repository includes all the sources files required for this example.

> Note: the `--recursive option` is required as external libraries (Eigen, spdlog...) are installed as git submodule (directly from their own repository).


## Hardware modifications and equipment

It is recommended to start by the first example below which runs in simulation only (without the need to be connected to a real robot) before attempting to connect to actual hardware. 

Once ready to use CORC with your actual hardware, you will need to 1) prepare a CAN interface (either via a [USB-CAN interface](doc/2.Hardware/USBCANadapters.md) if running on a laptop/desktop or CAN cape [if using a BeagleBone](../2.Hardware/BBUse.md) 2) get access to your device internal CAN bus and replace its existing controller. See [here](../2.Hardware/ModifyingDevice.md) for some more information.


# First Example: Basic Simulation - Exoskeleton
CORC by default contains a number of example programs, which a new user can choose from to get hands-on with the toolbox for their first application. However, we first recommend you run the Basic Simulation- Exoskeleton example program to gain familiarity with the CORC Environment.  

This example is the most simple of those offered, requiring access only to a Linux-based computer (either actual or Virtual Machine).

The functionality of this example is limited to transitioning between sitting and standing with an exoskeleton. 

This example does not include any visualisation of the robot, with feedback to the user only occuring through command line print statements. 

### Requirements
1. A development machine running Windows or Linux
2. A deployment machine running Linux (can be the same as the development machine, or a simpler, single board computer such as a BeagleBone)

### Suggested for
1. A good introduction to CORC, its StateMachine structure and compilation process
2. Initial testing of software for applications not using ROS (e.g. some embedded )
3. Users seeking the simplest possible runnable example 

### [Basic Simulation - Instructions](GSBasicSimulation.md) 
See also the code example in `src/apps/ExoTestMachine`.


# Additional Example Programs
After you have run the Basic Simulation Program, CORC offers a number of additional "Getting Started" programs. It is suggested that you choose one which aligns with your final intended goals. 


## Advanced Simulation and Hardware Testing using ROS - Exoskeleton
This example is a more advanced example requiring no hardware but also allow testing the same code both on hardware and simulation.
It leverages the ExoMotus X2 Exoskeleton from Fourier Intelligence, using ROS and creating a Gazebo for physics simulation and RViz for visualization purposes.

### Requirements
1. A development and deployment machine running Linux, ROS (Melodic or Noetic) and a compatible Gazebo version with ROS.

### Suggested for
1. Users looking to develop advanced user interfaces for their applications
2. Users who want to pre test their development on a physics simulation 
3. Users who wants to benefit from the advatanges of ROS
4. Users with sufficiently powerful computers to generate the 3d visualisations

### [Advanced Simulation and Hardware Testing using ROS - Instructions](AdvancedSimulationAndHardwareTesting.md) 
See also the code example in `src/apps/X2DemoMachine`.


## Hardware Testing - ArmMotus M2 Planar Manipulandum
This example enables simple movements with the ArmMotus M2 System. It use of the FLNL communications library to communicate between the CORC robot controller, and a user interface.

### Requirements
1. Development machine running Windows or Linux
2. ArmMotus M2 Upper Limb Rehabilitation Robot (Fourier Intelligence) - modified to be driven by an appropriate controller 
3. User Interface Machine (can be the same as the development machine)

### Suggested for
1. Users looking to develop for the ArmMotus M2
2. Users looking to develop systems with a separate user interface

### [M2DemoMachine - Instructions](GSM2DemoMachine.md)
See also the code example in `src/apps/M2DemoMachine`.


## Hardware Testing - ArmMotus M3 (aka EMU) 3D Manipulandum
This example enables simple functionalities of ArmMotus M3/EMU System. It shows basic interfaces of the robot, use of the kinematic models, simple examples of position, velocity and impedance controls.

### Requirements
1. Development machine running Linux
2. ArmMotus M3 (Fourier Intelligence) - modified to be driven by an appropriate controller

### Suggested for
1. Users looking to develop for the ArmMotus M3
2. Users looking to develop systems with a separate user interface

### [M3DemoMachine - Instructions](GSM3DemoMachine.md)
See also the code example in `src/apps/M3DemoMachine`.


## Hardware Testing - AnkleMotus M1
This example enables simple movements with the AnkleMotus M1 System.

### Requirements
1. Development machine running Linux
2. AnkleMotus M1  (Fourier Intelligence) - modified to be driven by an appropriate controller

### Suggested for
1. Users looking to develop for the AnkleMotus M1

### [M1DemoMachine - Instructions](GSM1DemoMachine.md)
See also the code example in `src/apps/M1DemoMachine`.


## ROS2 CORC application - Exoskeleton
This example is a an example of the use of a ROS2 CORC state machine.
It leverages the ExoMotus X2 Exoskeleton from Fourier Intelligence, and creates a ROS2 CORC package.

### Requirements
1. A development and deployment machine running Linux Ubuntu and ROS2 (Humble).

### Suggested for
1. Users looking to develop a ROS2 CORC application

### [ROS2 Application - Instructions](ROS2Application.md)
See also the code example in `src/apps/X2ROS2DemoMachine`.



## Hardware Testing - FIT-HV waist exoskeleton
This example enables simple control of the FIT-HV waist exoskeleton. 

### Requirements
1. Development machine running Linux
2. A modified FIT-HV waist exoskeleton

### Suggested for
1. Users looking to develop for FIT-HV exoskeletons or simple examples of CORC state machines.

### [FITHVExoDemoMachine - Instructions](GSFITHVDemoMachine.md)
See also the code example in `src/apps/FITHVExoDemoMachine`.

