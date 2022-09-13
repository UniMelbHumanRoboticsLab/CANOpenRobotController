cmake_minimum_required(VERSION 3.3)
project(CORC C CXX)

#To cross-compile for Arm (BeagleBone) use the armhf toolchain:
# cmake -DCMAKE_TOOLCHAIN_FILE=../armhf.cmake ..
#To cross-compile with ROS support prepare a sysroot and pass it to cmake:
# cmake -DCMAKE_SYSROOT=/path/to/sysroot/ -DCMAKE_TOOLCHAIN_FILE=../armhf.cmake ..
# See doc for more details.
################################## USER FLAGS ##################################

## Select the application by setting the state machine to use
#(it should be the class name and have corresponding header naming in a corresponding folder)
# If your application use ROS, don't forget to change the flag bellow too.

set (STATE_MACHINE_NAME "ExoTestMachine")
#set (STATE_MACHINE_NAME "M1DemoMachine")
#set (STATE_MACHINE_NAME "M2DemoMachine")
#set (STATE_MACHINE_NAME "M3DemoMachine")
#set (STATE_MACHINE_NAME "X2DemoMachine")
#set (STATE_MACHINE_NAME "LoggingDevice")

#Use this if your state machine code folder is not in CORC 'src/apps/' folder. 
#Can be a relative or absolute path.
#set (STATE_MACHINE_PATH "../") 

# Comment to use actual hardware, uncomment for a nor robot (virtual) app
set(NO_ROBOT ON)

# ROS Flag. set ON if you want to use ROS. Else, set OFF.
set(USE_ROS OFF)
# Select desired logging level (Options: TRACE, DEBUG, INFO, WARN, ERROR, CRITICAL OFF)
# INFO is the recommended level in normal operation
set(CORC_LOGGING_LEVEL INFO)

################################################################################
#Default path if not set
if(NOT STATE_MACHINE_PATH)
	set (STATE_MACHINE_PATH "src/apps")
endif()

#ROS internal flags
if(USE_ROS)
	add_definitions(-DUSEROS)
endif()

if(NO_ROBOT AND USE_ROS)
	set(SIM ON)
    add_definitions(-DSIM)
endif()

add_definitions(-DSPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_${CORC_LOGGING_LEVEL})

# For Force Plates
#add_definitions(-DFP_BBB)
add_definitions(-DFP_PB)
add_definitions(-DFP_CMDRPDO=0x3E0)
add_definitions(-DFP_STARTTPDO=0x3E1)

#######################

## Compile as C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(USE_ROS)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-int-in-bool-context" )
else()
	if(CMAKE_CROSSCOMPILING)
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-int-in-bool-context -static" )
	else()
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-int-in-bool-context" )
	endif()
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)


## Flags (Release is the default)
if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

## Current state machine (APP) to be build
if(NOT STATE_MACHINE_NAME)
	message(FATAL_ERROR "ERROR: No state machine (APP) selected: Abort.")
else()
	add_definitions(-DSTATE_MACHINE_TYPE=${STATE_MACHINE_NAME})
	add_definitions(-DSTATE_MACHINE_INCLUDE="${STATE_MACHINE_NAME}.h")
endif()
if(NO_ROBOT)
	add_definitions(-DNOROBOT=1)
endif()

## Get all source and header files (only the target app folder is included)
file(GLOB_RECURSE SOURCES "src/core/*.cpp" "src/core/*.c" "src/hardware/*.cpp" "src/hardware/*.c" "${STATE_MACHINE_PATH}/${STATE_MACHINE_NAME}/*.c" "${STATE_MACHINE_PATH}/${STATE_MACHINE_NAME}/*.cpp" "lib/FLNL/src/*.cpp")
file(GLOB_RECURSE HEADERS "src/core/*.h" "src/hardware/*.h" "${STATE_MACHINE_PATH}/${STATE_MACHINE_NAME}/*.h" )

## Set every folder containing .h file as include directory
set (INCLUDE_DIRS "")
foreach (_headerFile ${HEADERS})
	get_filename_component(_dir ${_headerFile} PATH)
	list (APPEND INCLUDE_DIRS ${_dir})
endforeach()
list(REMOVE_DUPLICATES INCLUDE_DIRS)
## Add libraries headers
list (APPEND INCLUDE_DIRS lib/)
list (APPEND INCLUDE_DIRS lib/Eigen/)
list (APPEND INCLUDE_DIRS lib/FLNL/include/)
list (APPEND INCLUDE_DIRS lib/spdlog/include/)

add_subdirectory(lib/yaml-cpp/)


## Hack for Yaml files path (absolute path required for ROS use, see X2Robot::initializeRobotParams)
if(CMAKE_CROSSCOMPILING)
	add_definitions(-DBASE_DIRECTORY=.)
else()
	add_definitions(-DBASE_DIRECTORY=${CMAKE_SOURCE_DIR})
endif()

## Add ROS dependencies
if(USE_ROS)
	#ROS local compile: use catkin
	if(NOT CMAKE_CROSSCOMPILING)
		message("--catkin--")
		# Required ROS packages
		find_package(catkin REQUIRED COMPONENTS
				roscpp
				rospy
				std_msgs
				std_srvs
				sensor_msgs
				geometry_msgs
				dynamic_reconfigure
				message_generation
        )
		if(SIM)
			find_package(catkin REQUIRED COMPONENTS
			controller_manager_msgs
			cob_gazebo_ros_control
			x2_description
			)
		endif()

		generate_dynamic_reconfigure_options(
				config/x2_dynamic_params.cfg
		)

		add_message_files(
				FILES
				X2Array.msg
				X2Acceleration.msg
				X2AccelerationMerge.msg
		)

		generate_messages(
				DEPENDENCIES
				std_msgs
		)



		catkin_package(
				#  INCLUDE_DIRS include
				#  LIBRARIES x2
			  CATKIN_DEPENDS
				roscpp
				rospy
				std_msgs
				std_srvs
				sensor_msgs
				geometry_msgs
				dynamic_reconfigure
				message_runtime
				#  DEPENDS system_lib
		)

		#include CATKIN
		include_directories(${catkin_INCLUDE_DIRS})
		set(ROS_LIBRARIES ${catkin_LIBRARIES})
	#Cross-compile with ROS (w/o catkin)
	else()
		# Required ROS packages
		find_package(roscpp REQUIRED CONFIG ONLY_CMAKE_FIND_ROOT_PATH)
		find_package(std_msgs REQUIRED CONFIG ONLY_CMAKE_FIND_ROOT_PATH)
		find_package(sensor_msgs REQUIRED CONFIG ONLY_CMAKE_FIND_ROOT_PATH)
		find_package(geometry_msgs REQUIRED CONFIG ONLY_CMAKE_FIND_ROOT_PATH)
		# If cross-compiling force search in sysroot
		if(IS_DIRECTORY ${CMAKE_SYSROOT})
			message("-- Using sysroot folder: ${CMAKE_SYSROOT}")
			#Reset include dirs towards sysroot only
			foreach(inc_dir ${roscpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})
				string(REGEX REPLACE "^/usr/" "${CMAKE_SYSROOT}usr/" inc_dir_cross ${inc_dir})
				include_directories(${inc_dir_cross})
			endforeach()
			#Reset libraries towards sysroot only
			foreach(lib_path ${roscpp_LIBRARIES} ${std_msgs_LIBRARIES})
				string(REPLACE "/" ";" lib_path_list ${lib_path}) #Breakdown path in list
				list(REVERSE lib_path_list)
				list(GET lib_path_list 0 lib) #Get last element (library name)
				find_library(new_path ${lib} PATHS ${CMAKE_SYSROOT}) #Force to look for it in actual sysroot path
				list(APPEND ROS_LIBRARIES ${new_path}) #Add it
				unset(new_path CACHE) #Clear variable to allow new search
			endforeach()
		else()
			message(FATAL_ERROR "No sysroot (CMAKE_SYSROOT) defined or not an accessible path")
		endif()
	endif()
endif()


## Executable name: {STATEMACHINENAME}_APP
set (APP_NAME ${STATE_MACHINE_NAME}_APP)
if(NO_ROBOT)
	set (APP_NAME ${APP_NAME}_NOROBOT)
endif()
add_executable(${APP_NAME}
				${SOURCES}
				)

## Includes
target_include_directories(${APP_NAME} PRIVATE ${INCLUDE_DIRS})

## Set required external packages
find_package(Threads REQUIRED)

## Link non-ROS libraries
target_link_libraries(${APP_NAME}
		${CMAKE_THREAD_LIBS_INIT}
		yaml-cpp)

## Link ROS libraries
if(USE_ROS)
	target_link_libraries(${APP_NAME} ${ROS_LIBRARIES})

	# make sure configure headers are built before any node using them
	add_dependencies(${APP_NAME} ${PROJECT_NAME}_gencfg)
endif()


message("-----------------------------------------------\nBuilding application ${APP_NAME}\n-----------------------------------------------")
