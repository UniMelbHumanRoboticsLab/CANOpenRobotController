cmake_minimum_required(VERSION 3.3)
project(CORC C CXX)

#To cross-compile for Arm (BeagleBone) use the armhf toolchain:
# cmake -DCMAKE_TOOLCHAIN_FILE=../armhf.cmake ..
# For aarch64 (e.g. Jetson):
# cmake -DCMAKE_TOOLCHAIN_FILE=../aarch64.cmake ..

################################## USER FLAGS ##################################

### Select the application by setting the state machine app.cmake to use
include(../EMUFourierMachine/app.cmake)
#include(src/apps/ExoTestMachine/app.cmake)
#include(src/apps/M1DemoMachine/app.cmake)
#include(src/apps/M1DemoMachineROS/app.cmake)
#include(src/apps/M2DemoMachine/app.cmake)
#include(src/apps/M3DemoMachine/app.cmake)
#include(src/apps/X2DemoMachine/app.cmake)
#include(src/apps/X2ROS2DemoMachine/app.cmake)
#include(src/apps/LoggingDevice/app.cmake)
#include(../myStateMachineApp/app.cmake) ## example only, need to be defined

## Comment to use actual hardware, uncomment for a nor robot (virtual) app
set(NO_ROBOT OFF)

## Select desired logging level (Options: TRACE, DEBUG, INFO, WARN, ERROR, CRITICAL, OFF)
## INFO is the recommended level in normal operation
set(CORC_LOGGING_LEVEL INFO)

################################################################################

## CORC internal cmake logic
include(corc.cmake)
