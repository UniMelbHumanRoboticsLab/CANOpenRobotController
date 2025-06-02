# List of available sensors
CORC [IO folder](src/hardware/IO/) contains a number of "drivers" for various types of sensors listed below.  It also contains drivers for generic keyboards and joysticks which can easily be added to any platform.

> Note that only some of them are used in supported platforms (others may have been used or are used in platforms not listed in CORC). 


## Force sensors and load cells
 - Robotous RFT 6 axis force-torque sensors on CAN (tested with RFT80-6A01 sensors). See dedicated class [here](src/hardware/IO/RobotousRFT.h).
 - HX711 chip: a dedicated ADC and amplifier for load cells. See dedicated class [here](src/hardware/IO/HX711.h).

## IMUs
 - [Techniad IMU](https://www.technaid.com/products/inertial-measurement-unit-tech-imu-biomechanichs/): a driver for the CAN based IMU from Technaid is available [here](src/hardware/IO/TechnaidIMU.cpp). See also [X2Robot](src/hardware/platforms/X2/X2Robot.h) platform for an example of use.
 - A Generic driver for standalone IMUs on microcontrollers (tested with Xiao and Teensy with BNO055 IMU). See [dedicated documentation page](2.Hardware/CANIMU.md) for details including links to microcontroller firmwares.
 
## Other IOs
 - BeagleBone DIOs: a copy of iobb library by shabaz is available in [IO folder](src/hardware/IO/) to handle BB GPIOs.
 - Fourier X2 handle/pendant: see [here](src/hardware/IO/FourierHandle.h).

