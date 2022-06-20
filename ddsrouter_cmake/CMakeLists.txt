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
# CMake build rules
###############################################################################
cmake_minimum_required(VERSION 3.5)
cmake_policy(VERSION 3.5...3.13)

###############################################################################
# Project
###############################################################################
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/project/in_source_build_prevention.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/project/load_project_settings.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/project/read_version.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/project/read_commit_hash.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/project/installation_paths.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/project/configure_project.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/project/cmake_options.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/packaging/install_resources.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/packaging/eProsima_packaging.cmake)

# Get all variables for project
configure_project()

# Call explictly project
project(${MODULE_NAME}
        VERSION
            ${MODULE_VERSION}
        DESCRIPTION
            ${MODULE_DESCRIPTION}
    )

###############################################################################
# Install cmake files
###############################################################################
# Install cmake directory and subdirectories
install(
    DIRECTORY
        cmake
        modules
        templates
    DESTINATION
        ${MODULE_DATA_INSTALL_DIR}/${PROJECT_NAME}
)

# Install load_all.cmake file
install(
    FILES
        ${PROJECT_SOURCE_DIR}/loader/load_all.cmake
    DESTINATION
        ${MODULE_DATA_INSTALL_DIR}/${PROJECT_NAME}/loader
)

###############################################################################
# Install test files
###############################################################################
# Install test required files
install(
    DIRECTORY
        test_installer
    DESTINATION
        ${MODULE_DATA_INSTALL_DIR}/${PROJECT_NAME}
)

###############################################################################
# Create CMake package config file
###############################################################################
include(CMakePackageConfigHelpers)
eprosima_packaging()

# Add load_all.cmake to config.cmake so it is included for the package it uses it, and so it addes every cmake module
set(
    PACKAGE_CONFIG_EXTRA_FILES
        "${PROJECT_NAME}/loader/load_all.cmake")

# Configure .in file for -config.cmake file
configure_package_config_file(
        ${PROJECT_SOURCE_DIR}/loader/cmakeonly-Config.cmake.in
        ${PROJECT_BINARY_DIR}/${PROJECT_NAME}/${PROJECT_NAME}-config.cmake
        INSTALL_DESTINATION
            ${MODULE_DATA_INSTALL_DIR}/${PROJECT_NAME}/cmake
        PATH_VARS
            DATA_INSTALL_DIR
    )

# Configure .in file for -config-version.cmake file
write_basic_package_version_file(
        ${PROJECT_BINARY_DIR}/${PROJECT_NAME}/${PROJECT_NAME}-config-version.cmake
        VERSION
            ${PROJECT_VERSION}
        COMPATIBILITY
            SameMajorVersion
    )

# Install -config.cmake and -config-version.cmake files
install(
        FILES
            ${PROJECT_BINARY_DIR}/${PROJECT_NAME}/${PROJECT_NAME}-config.cmake
            ${PROJECT_BINARY_DIR}/${PROJECT_NAME}/${PROJECT_NAME}-config-version.cmake
        DESTINATION
            ${MODULE_DATA_INSTALL_DIR}/${PROJECT_NAME}/cmake
    )
