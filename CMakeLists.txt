cmake_minimum_required(VERSION 3.3)
project(EXO_APP C CXX)

## Compile as C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pthread" )
set(CMAKE_EXPORT_COMPILE_COMMANDS ON )
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

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

# Executable name: EXO_APP_2020
add_executable(EXO_APP_2020
        ${SOURCES}
        )

target_include_directories(EXO_APP PRIVATE ${INCLUDE_DIRS})

# Set required external packages
find_package(Threads REQUIRED)

# Link libraries
target_link_libraries(EXO_APP ${CMAKE_THREAD_LIBS_INIT} -static)