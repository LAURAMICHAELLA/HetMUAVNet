cmake_minimum_required(VERSION 3.5)
project(gzuav C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(GNUInstallDirs)
add_definitions(-Wall)

add_subdirectory(src/ardupilot)
add_subdirectory(src/libs) # depends on ardupilot for MAVLink headers

add_subdirectory(src/gazebo)
add_subdirectory(src/gzuav)
add_subdirectory(src/gzuavchannel)
add_subdirectory(src/mavmix)

option(WITH_NS3_EXTERNAL_SYNC "Build and install ns-3 external module")
if (WITH_NS3_EXTERNAL_SYNC)
    add_subdirectory(ns-3)
endif (WITH_NS3_EXTERNAL_SYNC)
