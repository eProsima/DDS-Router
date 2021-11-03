# Copyright 2021 Proyectos y Sistemas de Mantenimiento SL (eProsima).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

###############################################################################
# CMake build rules for DataBroker Prototype
###############################################################################
cmake_minimum_required(VERSION 3.5)

set(IS_TOP_LEVEL TRUE)
if(PROJECT_SOURCE_DIR)
    set(IS_TOP_LEVEL FALSE)
endif()

# Set CMAKE_BUILD_TYPE to Release by default.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE Release CACHE STRING
        "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
        FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

###############################################################################
# Project
###############################################################################
project(databroker)

set(PROJECT_NAME_STYLED "DataBroker")
set(PROJECT_NAME_LARGE "Fast DataBroker")
string(TOUPPER "${PROJECT_NAME}" PROJECT_NAME_UPPER)
set(${PROJECT_NAME}_DESCRIPTION_SUMMARY "C++ library for use a WAN bridge using DDS protocol")
set(${PROJECT_NAME}_DESCRIPTION "eProsima ${PROJECT_NAME_LARGE} library provides an easy configurable bridge to communicate an internal network with an external one efficiently and transparently.")

message(STATUS "Configuring ${PROJECT_NAME_LARGE}")
# message(STATUS "Version: ${PROJECT_VERSION}")
# message(STATUS "To change the version modify the file configure.ac")

###############################################################################
# Installation paths
###############################################################################
set(BIN_INSTALL_DIR bin/ CACHE PATH "Installation directory for binaries")
set(INCLUDE_INSTALL_DIR include/ CACHE PATH "Installation directory for C++ headers")
set(LIB_INSTALL_DIR lib/ CACHE PATH "Installation directory for libraries")
set(DATA_INSTALL_DIR share/ CACHE PATH "Installation directory for data")

###############################################################################
# Load external eProsima projects.
###############################################################################
find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)

# include thirdparty header-only projects
include_directories(thirdparty/optionparser)

# include external required packages
find_package(yaml-cpp REQUIRED)

###############################################################################
# Compile library.
###############################################################################

# Project sources
file(GLOB_RECURSE SOURCES_FILES
        "src/**/*.c"
        "src/**/*.cpp"
        "src/**/*.cxx"
    )

# Project headers
file(GLOB_RECURSE HEADERS_FILES
        "include/**/*.h"
        "include/**/*.hpp"
        "include/**/*.hxx"
    )

message(STATUS "Sources: ${SOURCES_FILES}")
message(STATUS "Headers: ${HEADERS_FILES}")

add_executable(${PROJECT_NAME}
        ${SOURCES_FILES}
        ${HEADERS_FILES}
    )

target_include_directories(${PROJECT_NAME} PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/types>
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}>)

target_link_libraries(${PROJECT_NAME}
    fastrtps
    fastcdr
    yaml-cpp)

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${BIN_INSTALL_DIR})

###############################################################################
# Test
###############################################################################
option(BUILD_TESTS "Build eProsima Databroker application and documentation tests" OFF)
option(BUILD_APP_TESTS "Build eProsima Databroker application tests" OFF)
option(BUILD_DOCUMENTATION_TESTS "Build eProsima Databroker documentation tests" OFF)

if (BUILD_TESTS)
    set(BUILD_APP_TESTS ON)
    set(BUILD_DOCUMENTATION_TESTS ON)
endif()

if(BUILD_APP_TESTS OR BUILD_DOCUMENTATION_TESTS)
    # CTest needs to be included here, otherwise it is not possible to run the tests from the root
    # of the build directory
    enable_testing()
    include(CTest)
endif()

if (BUILD_APP_TESTS)
    add_subdirectory(test)
endif()
