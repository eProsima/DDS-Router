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
# CMake build rules for DDS Router
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
# Check C++ version
###############################################################################
# Check C++11
include(CheckCXXCompilerFlag)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG OR
        CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    check_cxx_compiler_flag(-std=c++11 SUPPORTS_CXX11)
    if(NOT SUPPORTS_CXX11)
        message(FATAL_ERROR "Compiler doesn't support C++11")
    endif()
endif()

###############################################################################
# Version
###############################################################################
set(PRODUCT_MAJOR_VERSION 0)
set(PRODUCT_MINOR_VERSION 2)
set(PRODUCT_PATCH_VERSION 0)
set(PRODUCT_VERSION_STR ${PRODUCT_MAJOR_VERSION}.${PRODUCT_MINOR_VERSION}.${PRODUCT_PATCH_VERSION})

###############################################################################
# Project
###############################################################################
project(ddsrouter)
set(PROJECT_VERSION_MAJOR "${PRODUCT_MAJOR_VERSION}")
set(PROJECT_VERSION_MINOR "${PRODUCT_MINOR_VERSION}")
set(PROJECT_VERSION_PATCH "${PRODUCT_PATCH_VERSION}")
set(PROJECT_VERSION "${PRODUCT_VERSION_STR}")

set(PROJECT_NAME_STYLED "DDS Router")
set(PROJECT_NAME_LARGE "DDS Router")
string(TOUPPER "${PROJECT_NAME}" PROJECT_NAME_UPPER)
set(${PROJECT_NAME}_DESCRIPTION_SUMMARY "C++ software application for use a WAN bridge using DDS protocol")
set(${PROJECT_NAME}_DESCRIPTION "eProsima ${PROJECT_NAME_LARGE} provides an easy configurable software application to communicate DDS networks efficiently and transparently.")

message(STATUS "Configuring ${PROJECT_NAME_LARGE}")
message(STATUS "Version: ${PROJECT_VERSION}")

###############################################################################
# Load CMake modules
###############################################################################
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake/modules)

###############################################################################
# Installation paths
###############################################################################
set(BIN_INSTALL_DIR bin/ CACHE PATH "Installation directory for binaries")
set(INCLUDE_INSTALL_DIR include/ CACHE PATH "Installation directory for C++ headers")
set(LIB_INSTALL_DIR lib/ CACHE PATH "Installation directory for libraries")
set(DATA_INSTALL_DIR share/ CACHE PATH "Installation directory for data")
if(WIN32)
    set(LICENSE_INSTALL_DIR . CACHE PATH "Installation directory for licenses")
else()
    set(LICENSE_INSTALL_DIR ${DATA_INSTALL_DIR}/${PROJECT_NAME} CACHE PATH "Installation directory for licenses")
endif()

###############################################################################
# Load external eProsima projects.
###############################################################################
find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)

# include thirdparty header-only projects
include_directories(thirdparty/filewatch)
include_directories(thirdparty/optionparser)

# include external required packages
find_package(yaml-cpp REQUIRED)

###############################################################################
# Code coverage
###############################################################################
option(CODE_COVERAGE "Compute the code coverage")

if (CODE_COVERAGE)
    # Warning/Error messages
    if(NOT (CMAKE_BUILD_TYPE STREQUAL "Debug"))
        message(WARNING "Code coverage results with an optimized (non-Debug) build may be misleading")
    endif()

    if("${CMAKE_C_COMPILER_ID}" MATCHES "(Apple)?[Cc]lang" OR "${CMAKE_CXX_COMPILER_ID}" MATCHES "(Apple)?[Cc]lang")
        message(STATUS "Building with llvm Code Coverage Tools")

        # set Flags
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fprofile-instr-generate -fcoverage-mapping")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-instr-generate -fcoverage-mapping")

    elseif(CMAKE_COMPILER_IS_GNUCXX)
        message(STATUS "Building with lcov Code Coverage Tools")

        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs -ftest-coverage")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs -ftest-coverage")
    else()
        message(FATAL_ERROR "Code coverage requires Clang or GCC. Aborting.")
    endif()
endif()

###############################################################################
# Default shared libraries
###############################################################################
# Global flag to cause add_library() to create shared libraries if on.
# If set to true, this will cause all libraries to be built shared
# unless the library was explicitly added as a static library.
option(BUILD_SHARED_LIBS "Create shared libraries by default" ON)

###############################################################################
# Compile library.
###############################################################################
add_subdirectory(src/cpp)

###############################################################################
# Compile application
###############################################################################
# TODO(annapurna): change to default ON when library is ready
option(COMPILE_TOOLS "Compile and install tools" OFF)

if (COMPILE_TOOLS)
    add_subdirectory(tool)
endif()

###############################################################################
# Test
###############################################################################
option(BUILD_TESTS "Build eProsima DDS Router application and documentation tests" OFF)
option(BUILD_APP_TESTS "Build eProsima DDS Router application tests" OFF)
option(BUILD_DOCUMENTATION_TESTS "Build eProsima DDS Router documentation tests" OFF)

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

###############################################################################
# Documentation
###############################################################################
option(BUILD_DOCUMENTATION "Generate documentation" OFF)
if(BUILD_DOCUMENTATION OR BUILD_DOCUMENTATION_TESTS)
    set(BUILD_DOCUMENTATION ON)
    add_subdirectory(docs)
endif()

###############################################################################
# Logging
###############################################################################
# In Debug, log set to ON
if ("Debug" STREQUAL ${CMAKE_BUILD_TYPE})
    option(LOG_INFO "Compile logInfo messages" ON)
else()
    option(LOG_INFO "Compile logInfo messages" OFF)
endif()

if (LOG_INFO)
    message(STATUS "Compiling logInfo messages for DDSRouter")
    target_compile_definitions(${PROJECT_NAME}
        PRIVATE FASTDDS_ENFORCE_LOG_INFO
        PRIVATE HAVE_LOG_NO_INFO=0
        )
endif()

###############################################################################
# Install Resources
###############################################################################

# Install example files
INSTALL(
    DIRECTORY ${PROJECT_SOURCE_DIR}/resources/configurations/examples/
    DESTINATION ${DATA_INSTALL_DIR}/resources/configurations/examples/)

# Install security files
INSTALL(
    DIRECTORY ${PROJECT_SOURCE_DIR}/resources/configurations/security/
    DESTINATION ${DATA_INSTALL_DIR}/resources/configurations/security/)

###############################################################################
# Packaging
###############################################################################
# Install license
install(FILES ${PROJECT_SOURCE_DIR}/LICENSE
    DESTINATION ${LICENSE_INSTALL_DIR}
    COMPONENT licenses
    )
set(CPACK_COMPONENT_LICENSES_HIDDEN 1)

set(CPACK_COMPONENTS_ALL headers)

if(MSVC OR MSVC_IDE)
set(CPACK_COMPONENTS_ALL ${CPACK_COMPONENTS_ALL} libraries_${MSVC_ARCH})
endif()

include(${PROJECT_SOURCE_DIR}/cmake/packaging/eProsimaPackaging.cmake)
