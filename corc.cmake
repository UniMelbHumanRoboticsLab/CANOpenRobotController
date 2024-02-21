################################################################################
## Core cmake logic for CORC apps building.
## Do not edit unless you know what you are doing.
##
## See the CMakeLists.txt for general configuration.
##
## See separate app.cmake in the respective stateMachine folders
## for individual app configurations (ROS, platform etc...)
################################################################################

#ROS internal flags
if(ROS GREATER 0)
    add_definitions(-DROS=${ROS})
endif()

#SIM flag is used for ROS1 simulation
if(NO_ROBOT AND (ROS EQUAL 1))
    set(SIM ON)
    add_definitions(-DSIM)
endif()

add_definitions(-DSPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_${CORC_LOGGING_LEVEL})

#######################

## Compile as C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_CROSSCOMPILING)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-int-in-bool-context -static" )
else()
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-int-in-bool-context" )
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)


## Flags (Release is the default)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
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
file(GLOB_RECURSE COMMON_SOURCES    "src/core/*.cpp" "src/core/*.c" 
                                    "src/hardware/IO/*.cpp" "src/hardware/IO/*.c"
                                    "src/hardware/drives/*.cpp" "src/hardware/drives/*.c"
                                    "${STATE_MACHINE_PATH}/*.c" "${STATE_MACHINE_PATH}/*.cpp"
)
file(GLOB TARGET_SOURCES "src/hardware/platforms/${PLATFORM}/*.cpp")

#if(WITH_FOURIER_AIOS)
#    file(GLOB_RECURSE SOURCES_FOURIER "lib/fourier-cpp-sdk/src/*.cpp")
#    list(APPEND COMMON_SOURCES ${SOURCES_FOURIER})
#endif()

file(GLOB_RECURSE HEADERS   "src/core/*.h"
                            "src/hardware/*.h"
                            "${STATE_MACHINE_PATH}/*.h"
)

## Set every folder containing .h file as include directory
set (INCLUDE_DIRS "")
foreach (_headerFile ${HEADERS})
    get_filename_component(_dir ${_headerFile} PATH)
    list (APPEND INCLUDE_DIRS ${_dir})
endforeach()
list(REMOVE_DUPLICATES INCLUDE_DIRS)
## Add libraries' headers
###list (APPEND INCLUDE_DIRS lib/)
list (APPEND INCLUDE_DIRS lib/Eigen/)
list (APPEND INCLUDE_DIRS lib/spdlog/include/)
if(USE_FLNL)
    list (APPEND INCLUDE_DIRS lib/FLNL/include/)
endif()
#if(WITH_FOURIER_AIOS)
#    list (APPEND INCLUDE_DIRS lib/fourier-cpp-sdk/src/)
#    list (APPEND INCLUDE_DIRS lib/fourier-cpp-sdk/fourier/include/)
#endif()

if(USE_FLNL)
    add_subdirectory(lib/FLNL/)
endif()
add_subdirectory(lib/yaml-cpp/)
#if(WITH_FOURIER_AIOS)
#    ## TODO: enforce static library (when available) and ensure underlying cpp also enforce static one (use full name with .a)
#    ## TODO: switch on cross-compile version
#    find_library(FOURIER_LIB fourier HINTS lib/fourier-cpp-sdk/fourier/lib/linux_x86_64)
#endif()

## Hack for Yaml files path (absolute path required for ROS use, see X2Robot::initializeRobotParams)
if(CMAKE_CROSSCOMPILING)
    add_definitions(-DBASE_DIRECTORY=.)
else()
    add_definitions(-DBASE_DIRECTORY=${CMAKE_SOURCE_DIR})
endif()

if(ROS EQUAL 1)
## Add ROS 1 dependencies
    #ROS 1 local compile: use catkin
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
            config/m1_dynamic_params.cfg
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
    
## Add ROS2 dependencies
elseif(ROS EQUAL 2)
    #ROS 2 local compile: use colcon
    message("--colcon--")

    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
endif()


## Executable name: {STATEMACHINENAME}_APP
set (APP_NAME ${STATE_MACHINE_NAME}_APP)
if(NO_ROBOT)
    set (APP_NAME ${APP_NAME}_NOROBOT)
endif()
add_executable( ${APP_NAME}
                ${COMMON_SOURCES}
                ${TARGET_SOURCES}
)

## Includes and internal code link
target_include_directories(${APP_NAME} PUBLIC ${INCLUDE_DIRS})

## Set required external packages
find_package(Threads REQUIRED)

## Link non-ROS libraries
target_link_libraries(  ${APP_NAME}
                        ${CMAKE_THREAD_LIBS_INIT}
                        yaml-cpp
)

if(USE_FLNL)
    target_link_libraries(${APP_NAME} libFLNL)
endif()

#if(WITH_FOURIER_AIOS)
#    target_link_libraries(${APP_NAME} ${FOURIER_LIB})
#endif()

## Link ROS libraries
if(ROS EQUAL 1)
    target_link_libraries(${APP_NAME} ${ROS_LIBRARIES})

    # make sure configure headers are built before any node using them
    add_dependencies(${APP_NAME} ${PROJECT_NAME}_gencfg)

elseif(ROS EQUAL 2)
    include_directories(${INCLUDE_DIRS})

    ament_target_dependencies(${APP_NAME} rclcpp std_msgs sensor_msgs)
    ament_export_dependencies(rclcpp std_msgs sensor_msgs)

    install(TARGETS ${APP_NAME} DESTINATION lib/${PROJECT_NAME})
    install(PROGRAMS script/initCAN0.sh DESTINATION lib/${PROJECT_NAME})
    install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
    install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

    ament_package()
endif()


message("-----------------------------------------------\nBuilding application ${APP_NAME}\n-----------------------------------------------")
