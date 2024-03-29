# Copyright: (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini <ugo.pattacini@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.5)

# enforce C++11 features
set(CMAKE_CXX_STANDARD 11)

# set the project name
project(TestAssignmentSolveMaze)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/plugins)
set(CMAKE_SHARED_MODULE_PREFIX "")

# add the required cmake packages
find_package(RobotTestingFramework 2.0.0 COMPONENTS DLL)
find_package(YARP 3.5.1 REQUIRED COMPONENTS os robottestingframework)

list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})

# add include directories
include_directories(${CMAKE_SOURCE_DIR}
                    ${RobotTestingFramework_INCLUDE_DIRS})

# add the source codes to build the plugin library
add_library(${PROJECT_NAME} MODULE test.cpp)

# add required libraries
target_link_libraries(${PROJECT_NAME} ${RobotTestingFramework_LIBRARIES} ${YARP_LIBRARIES})
