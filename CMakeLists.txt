# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(FRCPose)

include_directories("/usr/local/frc/include")
link_directories("/usr/local/frc/lib")
find_package(realsense2 REQUIRED)
add_executable(nt-rs-pose nt-rs-pose.cpp)
set_property(TARGET nt-rs-pose PROPERTY CXX_STANDARD 17)
target_link_libraries(nt-rs-pose ntcore jsoncpp ${realsense2_LIBRARY} ${DEPENDENCIES})
install(TARGETS nt-rs-pose RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
