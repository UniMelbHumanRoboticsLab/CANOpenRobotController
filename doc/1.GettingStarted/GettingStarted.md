# First Steps

There are a number of getting started programs and setups, depending on your intended and currently available hardware, and final eventual goal. Key decision points to be made are:

## Choice of Deployment Environment
CORC is flexible in that it can run on any computer with a Controller Area Network (CAN) interface running a Linux Distribution. However, there are two typical deployment scenarios:

1. An a dedicated embedded computer (such as a Beaglebone Black), running internally to a robotic device
2. On a desktop or laptop computer, connected to the robotic device via a USB-CAN interface

Examples of use of a dedicated embedded computer are when the robotic device is mobile (such as an assistive exoskeleton) or when separation between the robot controller and the user interface is desired (such as on deployment-ready rehabilitation devices). Alternatively, you may choose to use a desktop computer if you wish to run more powerful control algorithms on your device, or if your development is more experimental in nature. With appropriate choice of software packages, it is also possible to transition deployment from a desktop computer to an embedded computer. 

> Note that cross-compiling a ROS app is theoretically possible but not described in this documentation (contibutions are welcome!).

## Choice of Development Environment
Independent of the deployment environment, a choice may also be made regarding the development environment. If the deployment environment is Linux-based, this can be the same device as your deployment computer (although, development will typically require more computational power than deployment). Alternatively, you may develop on a different computer to the deployment computer. In this case, you have the choice of using either Windows or Linux based environments*, and you will need to cross compile for your deployment environment (i.e. compile on a system which is not used to run the executable). This requires an appropriate toolchain to be installed. 

> *Note, in theory it should be possible to develop on MacOS devices, however, toolchains for compiling are not readily available. As such, it may be better to dual boot, or run a virtual machine. 

## Choice of Hardware
This choice is likely related to the application. CORC is designed for any robotic device which uses CANOpen components, however, due to the resources and researchers of the initial developers, the examples and templates are based on assistive and rehabilitative robotic devices from Fourier Intelligence. As such, if the user has access to such devices, it would be logical to take an example which uses this hardware. 

# First Example: Basic Simulation - Exoskeleton
With this in mind, CORC by default contains a number of example programs, which a new user can choose from to get hands-on with the toolbox for their first application. However, we first recommend you run the Basic Simulation- Exoskeleton example program to gain familiarity with the CORC Environment.  

This example is the most simple of those offered, requiringly access only to a Linux-based computer. Development can occur either on a computer running either Windows or Linux (although the executeable must run on a Linux Machine).

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
This example enables simple functionalities of ArmMotus M3/EMU System. It shows basic interfaces of the robot, use of the kinematic modela, simple examples of position, velocity and impedance controls.

### Requirements
1. Development machine running Windows or Linux
2. ArmMotus M3 (Fourier Intelligence) - modified to be driven by an appropriate controller

### Suggested for
1. Users looking to develop for the ArmMotus M3
2. Users looking to develop systems with a separate user interface

### [M3DemoMachine - Instructions](GSM3DemoMachine.md)
See also the code example in `src/apps/M3DemoMachine`.


## Hardware Testing - AnkleMotus M1
This example enables simple movements with the AnkleMotus M1 System.

### Requirements
1. Development machine running Windows or Linux
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


