cmake_minimum_required(VERSION 3.3)
project(CORC C CXX)

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

## Get all source and header files
file(GLOB_RECURSE SOURCES "src/*.cpp" "src/*.c")
file(GLOB_RECURSE HEADERS "src/*.h")

## Set every folder containing .h file as include directory
set (INCLUDE_DIRS "")
foreach (_headerFile ${HEADERS})
    get_filename_component(_dir ${_headerFile} PATH)
    list (APPEND INCLUDE_DIRS ${_dir})
endforeach()
list(REMOVE_DUPLICATES INCLUDE_DIRS)
## Add libraries headers
list (APPEND INCLUDE_DIRS lib/)

# Select the application by setting the state machine to use
#(should be the class name and have corresponding header naming)
set (STATE_MACHINE_NAME "M3DemoMachine")
#set (STATE_MACHINE_NAME "ExoTestMachine")

add_definitions(-DSTATE_MACHINE_TYPE=${STATE_MACHINE_NAME})
add_definitions(-DSTATE_MACHINE_INCLUDE="${STATE_MACHINE_NAME}.h")

# Executable name: {STATEMACHINENAME}_APP
add_executable(${STATE_MACHINE_NAME}_APP
        ${SOURCES}
        )

target_include_directories(${STATE_MACHINE_NAME}_APP PRIVATE ${INCLUDE_DIRS})

# Set required external packages
find_package(Threads REQUIRED)

# Link libraries
target_link_libraries(${STATE_MACHINE_NAME}_APP ${CMAKE_THREAD_LIBS_INIT} -static)