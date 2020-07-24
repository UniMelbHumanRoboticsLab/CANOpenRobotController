cmake_minimum_required(VERSION 3.3)
project(CORC C CXX)

# ROS Flag. set ON if you want to use ROS. Else, set OFF.
set(USE_ROS OFF)

if(USE_ROS)
    add_definitions(-DUSEROS)
endif()

## Compile as C++11 with pthread
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pthread" )
set(CMAKE_EXPORT_COMPILE_COMMANDS ON )
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

#Flags (Release is the default)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Select the application by setting the state machine to use.
# If your application use ROS, don't forget to change the flag at the top.
#(should be the class name and have corresponding header naming in a corresponding folder)
#set (STATE_MACHINE_NAME "M3DemoMachine")
#set (STATE_MACHINE_NAME "M3Chai")
#set (STATE_MACHINE_NAME "ExoTestMachine")
set (STATE_MACHINE_NAME "X2DemoMachine")

add_definitions(-DSTATE_MACHINE_TYPE=${STATE_MACHINE_NAME})
add_definitions(-DSTATE_MACHINE_INCLUDE="${STATE_MACHINE_NAME}.h")

if(USE_ROS)
# Required ROS packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        sensor_msgs
        )

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES x2
        #  CATKIN_DEPENDS roscpp rospy std_msgs
        #  DEPENDS system_lib
)

#include CATKIN
include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)
endif()

## Get all source and header files (only the target app folder is included)
file(GLOB_RECURSE SOURCES "src/core/*.cpp" "src/core/*.c" "src/hardware/*.cpp" "src/hardware/*.c" "src/apps/${STATE_MACHINE_NAME}/*.c" "src/apps/${STATE_MACHINE_NAME}/*.cpp")
file(GLOB_RECURSE HEADERS "src/core/*.h" "src/hardware/*.h" "src/apps/${STATE_MACHINE_NAME}/*.h")

## Set every folder containing .h file as include directory
set (INCLUDE_DIRS "")
foreach (_headerFile ${HEADERS})
    get_filename_component(_dir ${_headerFile} PATH)
    list (APPEND INCLUDE_DIRS ${_dir})
endforeach()
list(REMOVE_DUPLICATES INCLUDE_DIRS)
## Add libraries headers
list (APPEND INCLUDE_DIRS lib/)

# Executable name: {STATEMACHINENAME}_APP
add_executable(${STATE_MACHINE_NAME}_APP
        ${SOURCES}
        )

target_include_directories(${STATE_MACHINE_NAME}_APP PRIVATE ${INCLUDE_DIRS})

# Set required external packages
find_package(Threads REQUIRED)

# Link non-ROS libraries
target_link_libraries(${STATE_MACHINE_NAME}_APP ${CMAKE_THREAD_LIBS_INIT})

# Link ROS libraries
if(USE_ROS)
    target_link_libraries(${STATE_MACHINE_NAME}_APP ${catkin_LIBRARIES})
endif()