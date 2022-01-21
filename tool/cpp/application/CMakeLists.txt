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
# COMPILE AND INSTALL TOOL EXECUTABLE
###############################################################################

###############################################################################
# CMake Check
###############################################################################
include(CheckCXXCompilerFlag)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG OR
        CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    check_cxx_compiler_flag(-std=c++11 SUPPORTS_CXX11)
    if(NOT SUPPORTS_CXX11)
        message(FATAL_ERROR "Compiler doesn't support C++11")
    endif()
endif()
cmake_minimum_required(VERSION 3.5)

if(NOT CMAKE_VERSION VERSION_LESS 3.0)
    cmake_policy(SET CMP0048 NEW)
endif()

###############################################################################
# Set Executable project and name
###############################################################################
project(ddsrouter-app)
set(EXECUTABLE_NAME ${PROJECT_NAME})

###############################################################################
# Check C++ version
###############################################################################
#Check C++11
include(CheckCXXCompilerFlag)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG OR
        CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    check_cxx_compiler_flag(-std=c++11 SUPPORTS_CXX11)
    if(NOT SUPPORTS_CXX11)
        message(FATAL_ERROR "Compiler doesn't support C++11")
    endif()
endif()

###############################################################################
# Find requirements
###############################################################################
if(NOT fastcdr_FOUND)
    find_package(fastcdr REQUIRED)
endif()

if(NOT fastrtps_FOUND)
    find_package(fastrtps REQUIRED)
endif()

# Even when it is forced in /tool, this cmake could be compiled independently, so this is needed
if(NOT ddsrouter_FOUND)
    find_package(ddsrouter REQUIRED)
endif()



###############################################################################
# Compile
###############################################################################
message(STATUS "Compiling ${EXECUTABLE_NAME}...")

# Get src files
set(EXECUTABLE_SOURCES
        "main.cpp"
        "user_interface/arguments_configuration.cpp"
    )

add_executable(${EXECUTABLE_NAME} ${EXECUTABLE_SOURCES})

# TODO: check if needed
# target_compile_definitions(${EXECUTABLE_NAME} PRIVATE
#     $<$<AND:$<NOT:$<BOOL:${WIN32}>>,$<STREQUAL:"${CMAKE_BUILD_TYPE}","Debug">>:__DEBUG>
#     $<$<BOOL:${INTERNAL_DEBUG}>:__INTERNALDEBUG> # Internal debug activated.
# )

target_link_libraries(${PROJECT_NAME}
        fastrtps
        fastcdr
        ddsrouter
        yaml-cpp
        $<$<BOOL:${WIN32}>:iphlpapi$<SEMICOLON>Shlwapi>
    )

install(TARGETS ${EXECUTABLE_NAME}
    RUNTIME DESTINATION ${BIN_INSTALL_DIR}/app)
