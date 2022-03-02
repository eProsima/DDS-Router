# Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
# CMake build rules for DDS Router Submodule
###############################################################################
cmake_minimum_required(VERSION 3.5)

set(IS_TOP_LEVEL TRUE)
if(PROJECT_SOURCE_DIR)
    set(IS_TOP_LEVEL FALSE)
endif()

###############################################################################
# Set variables from this submodule
###############################################################################
message(STATUS "CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")
include(${CMAKE_SOURCE_DIR}/project_settings.cmake)

###############################################################################
# Set CMake build type
###############################################################################
# Set CMAKE_BUILD_TYPE to Release by default.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE Release CACHE STRING
        "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
        FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

###############################################################################
# In-source build prevention
###############################################################################
# Prevent to build a CMake project in source directory.
# Always use /build or any other auxiliar directory
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR “In-source build detected!”)
endif()
# Thanks to: https://towardsdatascience.com/7-tips-for-clean-cmake-scripts-c8d276587389

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
# Project
###############################################################################
project(${SUBMODULE_PROJECT_NAME} LANGUAGES CXX)
# NOTE: this is required to be before using ${PROJECT_SOURCE_DIR}

# Set variables to configure project
set(DESCRIPTION_SUMMARY ${SUBMODULE_PROJECT_SUMMARY})

###############################################################################
# Load CMake modules
###############################################################################
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake/modules)

###############################################################################
# Project Version
###############################################################################
# Include read_version function
include(${PROJECT_SOURCE_DIR}/cmake/packaging/versionReader.cmake)
# Read version
read_version("VERSION")

# Set STR version
set(PRODUCT_VERSION_STR "${PRODUCT_MAJOR_VERSION}.${PRODUCT_MINOR_VERSION}.${PRODUCT_PATCH_VERSION}")

# Set Project version
set(PROJECT_VERSION_MAJOR "${PRODUCT_MAJOR_VERSION}")
set(PROJECT_VERSION_MINOR "${PRODUCT_MINOR_VERSION}")
set(PROJECT_VERSION_PATCH "${PRODUCT_PATCH_VERSION}")
set(PROJECT_VERSION "${PRODUCT_VERSION_STR}")

set(PROJECT_NAME_STYLED "${PROJECT_NAME}")
set(PROJECT_NAME_LARGE "${PROJECT_NAME}")
string(TOUPPER "${PROJECT_NAME}" PROJECT_NAME_UPPER)
set(${PROJECT_NAME}_DESCRIPTION_SUMMARY "${DESCRIPTION_SUMMARY}")

message(STATUS "Configuring ${PROJECT_NAME_LARGE}")
message(STATUS "Version: ${PROJECT_VERSION}")

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
# Load external projects.
###############################################################################
foreach(DEPENDENCY ${SUBMODULE_PROJECT_FIND_PACKAGES})
    message(STATUS "Looking for project ${PROJECT_NAME} dependency ${DEPENDENCY}.")
    find_package("${DEPENDENCY}" REQUIRED)
endforeach()

# include thirdparty header-only projects
foreach(THIRDPARTY_HEADERONLY_DEPENDENCY ${SUBMODULE_THIRDPARTY_HEADERONLY_PACKAGES})
    message(STATUS
        "Looking for project ${PROJECT_NAME} thirdparty headeronly thirdparty/${THIRDPARTY_HEADERONLY_DEPENDENCY}.")
    include_directories(thirdparty/${THIRDPARTY_HEADERONLY_DEPENDENCY})
endforeach()

###############################################################################
# Code coverage
###############################################################################
option(CODE_COVERAGE "Compute the code coverage" OFF)

if (CODE_COVERAGE)
    # Warning/Error messages
    if(NOT (${CMAKE_BUILD_TYPE} STREQUAL "Debug"))
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
# Test
###############################################################################
option(BUILD_TESTS "Build eProsima ${PROJECT_NAME} library tests" OFF)
option(BUILD_APP_TESTS "Build eProsima ${PROJECT_NAME} library tests" OFF)

if (BUILD_TESTS)
    set(BUILD_APP_TESTS ON)
endif()

if(BUILD_APP_TESTS)
    # CTest needs to be included here, otherwise it is not possible to run the tests from the root
    # of the build directory
    enable_testing()
    include(CTest)
endif()

if(BUILD_APP_TESTS)
    add_subdirectory(test)
endif()

###############################################################################
# Logging
###############################################################################
# In Debug, log set to ON
if("Debug" STREQUAL ${CMAKE_BUILD_TYPE})
    option(LOG_INFO "Compile logInfo messages" ON)
else()
    option(LOG_INFO "Compile logInfo messages" OFF)
endif()

if(LOG_INFO)
    target_compile_definitions(${PROJECT_NAME}
        PRIVATE FASTDDS_ENFORCE_LOG_INFO
        PRIVATE HAVE_LOG_NO_INFO=0
        )
endif()

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

message(STATUS "PROJECT ${PROJECT_NAME} final version ${PROJECT_VERSION}")
