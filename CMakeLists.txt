cmake_minimum_required(VERSION 3.3)
project(CORC C CXX)


## Select the application by setting the state machine to use
#(it should be the class name and have corresponding header naming in a corresponding folder)
# If your application use ROS, don't forget to change the flag bellow to.
#set (STATE_MACHINE_NAME "M3DemoMachine")
#set (STATE_MACHINE_NAME "M3Machining")
#set (STATE_MACHINE_NAME "M3Chai")
#set (STATE_MACHINE_NAME "ExoTestMachine")
set (STATE_MACHINE_NAME "X2DemoMachine")

## Comment to use actual hardware, uncomment for a nor robot (virtual) app
set(NO_ROBOT ON)

## ROS Flag. set ON if you want to use ROS. Else, set OFF or comment
set(USE_ROS ON)

if(USE_ROS)
    add_definitions(-DUSEROS)
endif()



set(CMAKE_SYSROOT /home/vincent/Mbed/bbai-sysroot/)
## Compile as C++11 with pthread
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(USE_ROS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall " )
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -static" )
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)


## Flags (Release is the default)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")


add_definitions(-DSTATE_MACHINE_TYPE=${STATE_MACHINE_NAME})
add_definitions(-DSTATE_MACHINE_INCLUDE="${STATE_MACHINE_NAME}.h")
if(NO_ROBOT)
	add_definitions(-DNOROBOT=1)
endif()


if(USE_ROS)
	## Required ROS packages
	find_package(roscpp REQUIRED)
	find_package(std_msgs REQUIRED)
	message("--${roscpp_INCLUDE_DIRS}--")
	message("--${std_msgs_INCLUDE_DIRS}--")
	set(roscpp_INCLUDE_DIRS "/home/vincent/Mbed/bbai-sysroot/usr/include")
	set(std_msgs_INCLUDE_DIRS "/home/vincent/Mbed/bbai-sysroot/usr/include")
	include_directories(${roscpp_INCLUDE_DIRS})
	include_directories(${std_msgs_INCLUDE_DIRS})
	
	set(roscpp_LIBRARIES "/home/vincent/Mbed/bbai-sysroot/usr/lib/arm-linux-gnueabihf/libroscpp.so;/home/vincent/Mbed/bbai-sysroot/usr/lib/arm-linux-gnueabihf/libpthread.so;/home/vincent/Mbed/bbai-sysroot/usr/lib/arm-linux-gnueabihf/libboost_signals.so;/home/vincent/Mbed/bbai-sysroot/usr/lib/arm-linux-gnueabihf/libboost_filesystem.so;/home/vincent/Mbed/bbai-sysroot/usr/lib/arm-linux-gnueabihf/librosconsole.so;/home/vincent/Mbed/bbai-sysroot/usr/lib/arm-linux-gnueabihf/librosconsole_log4cxx.so;/home/vincent/Mbed/bbai-sysroot/usr/lib/arm-linux-gnueabihf/librosconsole_backend_interface.so;/home/vincent/Mbed/bbai-sysroot/usr/lib/arm-linux-gnueabihf/liblog4cxx.so;/home/vincent/Mbed/bbai-sysroot/usr/lib/arm-linux-gnueabihf/libboost_regex.so;/home/vincent/Mbed/bbai-sysroot/usr/lib/arm-linux-gnueabihf/libroscpp_serialization.so;/home/vincent/Mbed/bbai-sysroot/usr/lib/arm-linux-gnueabihf/librostime.so;/home/vincent/Mbed/bbai-sysroot/usr/lib/arm-linux-gnueabihf/libxmlrpcpp.so;/home/vincent/Mbed/bbai-sysroot/usr/lib/arm-linux-gnueabihf/libcpp_common.so;/home/vincent/Mbed/bbai-sysroot/usr/lib/arm-linux-gnueabihf/libboost_system.so;/home/vincent/Mbed/bbai-sysroot/usr/lib/arm-linux-gnueabihf/libboost_thread.so;/home/vincent/Mbed/bbai-sysroot/usr/lib/arm-linux-gnueabihf/libboost_chrono.so;/home/vincent/Mbed/bbai-sysroot/usr/lib/arm-linux-gnueabihf/libboost_date_time.so;/home/vincent/Mbed/bbai-sysroot/usr/lib/arm-linux-gnueabihf/libboost_atomic.so;/home/vincent/Mbed/bbai-sysroot/usr/lib/arm-linux-gnueabihf/libpthread.so;/home/vincent/Mbed/bbai-sysroot/usr/lib/arm-linux-gnueabihf/libconsole_bridge.so")
	set(std_msgs_LIBRARIES "/home/vincent/Mbed/bbai-sysroot/usr/lib/arm-linux-gnueabihf/libroscpp_serialization.so;/home/vincent/Mbed/bbai-sysroot/usr/lib/arm-linux-gnueabihf/librostime.so;/home/vincent/Mbed/bbai-sysroot/usr/lib/arm-linux-gnueabihf/libcpp_common.so;/home/vincent/Mbed/bbai-sysroot/usr/lib/arm-linux-gnueabihf/libboost_system.so;/home/vincent/Mbed/bbai-sysroot/usr/lib/arm-linux-gnueabihf/libboost_thread.so;/home/vincent/Mbed/bbai-sysroot/usr/lib/arm-linux-gnueabihf/libboost_chrono.so;/home/vincent/Mbed/bbai-sysroot/usr/lib/arm-linux-gnueabihf/libboost_date_time.so;/home/vincent/Mbed/bbai-sysroot/usr/lib/arm-linux-gnueabihf/libboost_atomic.so;/home/vincent/Mbed/bbai-sysroot/usr/lib/arm-linux-gnueabihf/libpthread.so;/home/vincent/Mbed/bbai-sysroot/usr/lib/arm-linux-gnueabihf/libconsole_bridge.so")
	set(ROS_LIBRARIES ${roscpp_LIBRARIES} ${std_msgs_LIBRARIES})
	message("--${roscpp_LIBRARIES}--")
	message("--${std_msgs_LIBRARIES}--")
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



## Executable name: {STATEMACHINENAME}_APP
set (APP_NAME ${STATE_MACHINE_NAME}_APP)
if(NO_ROBOT)
	set (APP_NAME ${APP_NAME}_NOROBOT)
endif()
add_executable(${APP_NAME}
				${SOURCES}
				)

target_include_directories(${APP_NAME} PRIVATE ${INCLUDE_DIRS})

## Set required external packages
find_package(Threads REQUIRED)

## Link non-ROS libraries
target_link_libraries(${APP_NAME} ${CMAKE_THREAD_LIBS_INIT})

## Link ROS libraries
if(USE_ROS)
	 target_link_libraries(${APP_NAME} ${ROS_LIBRARIES})
endif()


message("-----------------------------------------------\nBuilding application ${APP_NAME}\n-----------------------------------------------")
