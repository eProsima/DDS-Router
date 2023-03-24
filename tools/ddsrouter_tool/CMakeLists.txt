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

# Done this to set machine architecture and be able to call cmake_utils
enable_language(CXX)

###############################################################################
# Find package cmake_utils
###############################################################################
# Package cmake_utils is required to get every cmake macro needed
find_package(cmake_utils REQUIRED)

###############################################################################
# Project
###############################################################################
# Configure project by info set in project_settings.cmake
# - Load project_settings variables
# - Read version
# - Set installation paths
configure_project()

# Call explictly project
project(
    ${MODULE_NAME}
    VERSION
        ${MODULE_VERSION}
    DESCRIPTION
        ${MODULE_DESCRIPTION}
    LANGUAGES
        CXX
)

###############################################################################
# C++ Project
###############################################################################
# Configure CPP project for dependencies and required flags:
# - Set CMake Build Type
# - Set C++ version
# - Set shared libraries by default
# - Find external packages and thirdparties
# - Activate Code coverage if flag CODE_COVERAGE
# - Activate Address sanitizer build if flag ASAN_BUILD
# - Activate Thread sanitizer build if flag TSAN_BUILD
# - Configure log depending on LOG_INFO flag and CMake type
configure_project_cpp()

# Compile C++ library
compile_tool(
    "${PROJECT_SOURCE_DIR}/src/cpp" # Source directory
)

###############################################################################
# Test
###############################################################################
# Compile tests if CMake options requires it
compile_test_tool(
    "${PROJECT_SOURCE_DIR}/test" # Test directory
)

###############################################################################
# Packaging
###############################################################################
# Install package
eprosima_packaging()
