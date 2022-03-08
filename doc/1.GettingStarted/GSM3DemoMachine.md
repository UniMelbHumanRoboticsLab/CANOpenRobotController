# Hardware Testing - ArmMotus EMU (aka M3) 3D Manipulandum

This pages introduces the M3DemoMachine, an example CORC app showing the basic use of the M3 3D manipulandum.

The M3 is a 3 DoFs impedance based robot developed by Fourier Intelligence:

![ArmMotus EMU with frames](../img/M3WithFrames.png)
ArmMotus EMU (aka M3) and reference coordinates.

The state machine code can be found in the folder `src/apps/M3DemoMachine`.

It demonstrates the use of:
- The different control modes of M3 (position, velocity, or impedance)
- The use of the different kinematic models of the robot and associated model parameters (loaded from a YAML file)
- The use of a standard joystick as a control input
- The use of the libFLNL comunication library to pusblish the robot state in a Unity software and send commands to the state machine


## Running the state machine

In the CMakeLists.txt select the M3DemoMachine and set the flags for using a real robot without ROS support:

```
#set (STATE_MACHINE_NAME "ExoTestMachine")
#set (STATE_MACHINE_NAME "M1DemoMachine")
#set (STATE_MACHINE_NAME "M2DemoMachine")
set (STATE_MACHINE_NAME "M3DemoMachine")
#set (STATE_MACHINE_NAME "X2DemoMachine")
#set (STATE_MACHINE_NAME "LoggingDevice")

# Comment to use actual hardware, uncomment for a nor robot (virtual) app
set(NO_ROBOT OFF)

# ROS Flag. set ON if you want to use ROS. Else, set OFF.
set(USE_ROS OFF)
```

If you intend to cross-compile for a BeagleBone (Black or AI), run: `$ rm -r build && mkdir build && cd build && cmake -DCMAKE_TOOLCHAIN_FILE=../armhf.cmake ..`

otherwise, to run the state machine locally use: `$ rm -r build && mkdir build && cd build && cmake .. `

Then simply compile the state machine: `$ make`

This should create the application `M3DemoMachine` within the build folder. After initialising the CANbus (using the `initCAN0.sh` or `initCAN1.sh` script) you should be able to run the application, either locally or on the BB (through SSH).

**WARNING:** With an M3 connected on the CAN bus the robot will imediatly start to move after running the application (to go in a calibration pose), ensure the space is clear around the robot.

Once the calibration state is finished, you can circle through the different demo states using the keyboard (key 1) or using the joystick first button.


## RobotM3 structure and interface

### Robot model and parameters (YAML)



The full list of parameters which can be loaded from the YAML configuration file can be seen in the example YAML file `M3_params.yaml`.

The YAML configuration file to use and the corresponding robot name (model) can be selected from the RobotM3 constructor parameters.

### Control methods

The CORC M3 robot model has the following specific methods of interaction:
- Obtaining current **joint state** (as for any CORC robot): `robot->getPosition()`, `robot->getVelocity()`, `robot->getTorque()`.
- **Joint level interaction**: `setJointPosition(VM3 q)`, `setJointVelocity(VM3 q)` and `setJointTorque(VM3 tau)` allow to apply a position, velocity or torque control using an Eigen:vector of length 3. An example of the torque control can be found in the `M3CalibState` state. Note the  use of `robot->initTorqueControl();` in the `entryCode()` method before applying torque control.
- Obtaining current **end-effector state**: `robot->getEndEffPosition()`, `robot->getEndEffVelocity()`, `robot->getEndEffForce()`. Methods are also provided to obtain the filtered velocity and acceleration (obtained through differentiation and low-pass filtering). Additionnaly the pure interaction force at the end-effector, calculated using the robot model and motor torques from wich gravity compensation torques are substracted can be obtained using the `robot->getInteractionForce()` method.
- **End-effector space control** is available using: `setEndEffPosition(VM3 X)`, `setEndEffVelocity(VM3 dX)`, `setEndEffForce(VM3 F)`. These methods rely on the `inverseKinematic()` and robot Jacobian `J()` and assumes that the kinematic parameters loaded from the YAML file are correct and that the robot has been calibrated (see `applyCalibration()`). As for their joints counterparts they require the proper use of the corresponding initTorque/Velocity/Position method beforehand. The command vectors are expressed in the robot base frame as shown on the picture above. An example of the use of the end-effector velocity control is available in the `M3EndEffDemo` state.
- Finally, the method `setEndEffForceWithCompensation(VM3 F, bool friction_comp=true)` can be used to apply an end-effector force in addition to the **robot gravity self-compensation and friction compensation (optional)**. This method relies on the robot model and parameters (masses and friction coefficients) set in the YAML configuration file. The friction compensation uses a Coulomb + viscous friction model.

See the Doxygen page of the `RobotM3` class for a full list of available methods.

## Joystick

The `RobotM3` class include a Joystick input by default. The joystick, if connected, can be used within the different state machine states and transitions.

An example of the use of the first stick of a joystick used as an input can be found in the `M3EndEffDemo` state. The method `robot->joystick->getAxis(i)` returns a value proportional to the position of the stick direction `i`, which is used as a velocity command.

Additionaly, joystick buttons are used in the state machine transition: `M3DemoMachine::GoToNextState` to allow transition by a button press: `OWNER->robot->joystick->isButtonTransition(3)`. Note the use of the ButtonTransition method to avoid capturing repeatdly if the button stay pressed.


## Network communication with libFLNL

The M3DemoMachine app is using libFLNL to publish the robot states and read incoming commands over a TCP/IP connection.

![FLNL communication](../img/FLNLUnity.png)

TODO
