cmake_minimum_required(VERSION 3.5)
project(CORC)


# STATE MACHINE (APP)
set(STATE_MACHINE_NAME "X2DemoMachineROS2")
set(CORC_LOGGING_LEVEL INFO)


# DEFINITIONS
add_definitions(-DUSEROS)
add_definitions(-DSPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_${CORC_LOGGING_LEVEL})
add_definitions(-DFP_PB)
add_definitions(-DFP_CMDRPDO=0x3E0)
add_definitions(-DFP_STARTTPDO=0x3E1)


# CMAKE
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-int-in-bool-context")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT STATE_MACHINE_NAME)
	message(FATAL_ERROR "NO STATE MACHINE (APP) SELECTED")
else()
	add_definitions(-DSTATE_MACHINE_TYPE=${STATE_MACHINE_NAME})
	add_definitions(-DSTATE_MACHINE_INCLUDE="${STATE_MACHINE_NAME}.h")
	add_definitions(-DBASE_DIRECTORY=${CMAKE_SOURCE_DIR})
endif()


# FILES
file(GLOB_RECURSE SOURCES
	"src/core/*.cpp" "src/core/*.c" "src/hardware/*.cpp" "src/hardware/*.c"
	"src/apps/${STATE_MACHINE_NAME}/*.c" "src/apps/${STATE_MACHINE_NAME}/*.cpp"
	"lib/FLNL/src/*.cpp" "lib/spdlog/src/*.cpp"
)
file(GLOB_RECURSE HEADERS "src/core/*.h" "src/hardware/*.h" "src/apps/${STATE_MACHINE_NAME}/*.h")

set(INCLUDE_DIRS "")
foreach(_headerFile ${HEADERS})
	get_filename_component(_dir ${_headerFile} PATH)
	list(APPEND INCLUDE_DIRS ${_dir})
endforeach()

list(REMOVE_DUPLICATES INCLUDE_DIRS)
list(APPEND INCLUDE_DIRS lib/)
list(APPEND INCLUDE_DIRS lib/Eigen/)
list(APPEND INCLUDE_DIRS lib/FLNL/include/)
list(APPEND INCLUDE_DIRS lib/spdlog/include/)
add_subdirectory(lib/yaml-cpp/)


# ROS
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Threads REQUIRED)

set(APP_NAME ${STATE_MACHINE_NAME}_APP)
add_executable(${APP_NAME} ${SOURCES})
target_link_libraries(${APP_NAME} ${CMAKE_THREAD_LIBS_INIT} yaml-cpp)
target_include_directories(${APP_NAME} PUBLIC ${INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

ament_target_dependencies(${APP_NAME} rclcpp std_msgs std_srvs sensor_msgs geometry_msgs)
ament_export_dependencies(rclcpp std_msgs std_srvs sensor_msgs geometry_msgs)

install(TARGETS ${APP_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
