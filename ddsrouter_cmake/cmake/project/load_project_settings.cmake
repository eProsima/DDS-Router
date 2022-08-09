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
# Load Project Settings macro
###############################################################################

# Load parameters from project_settings file.
# This file must be a .cmake file (default project_settings.cmake) that set (or optionally set)
# variables for the project.
#
# REQUIRED VARIABLES:
# - MODULE_NAME: Name of the project.
#
# OPTIONAL ADVICED VARIABLES:
# - MODULE_SUMMARY          : Project summary    (default MODULE_NAME)
# - MODULE_FIND_PACKAGES    : Packages required to be used in the project
# - MODULE_THIRDPARTY_HEADERONLY    : Thirdparty Headeronly projects required (in /thirdparty directory)
# - MODULE_DEPENDENCIES     : Packages required to be linked by target
#
# OPTIONAL VARIABLES:
# - MODULE_NAME_LARGE       : Project large name (default MODULE_NAME)
# - MODULE_DESCRIPTION      : Project description (default MODULE_SUMMARY)
# - MODULE_MACRO            : String to set macros in project (default MODULE_NAME in UPPERCASE)
#
# TODO
#
# Arguments:
# (optional) PROJECT_SETTINGS_FILE -> File to load Project settings [default: project_settings.cmake]
macro(load_project_settings)

    # Use default file
    set (PROJECT_SETTINGS_FILE ${CMAKE_CURRENT_SOURCE_DIR}/project_settings.cmake)

    # Check if there is an argument, if it is set, modify default.
    set (VARIADIC_ARGS ${ARGN})
    list(LENGTH VARIADIC_ARGS VARIADIC_ARGS_LENGTH)
    if (${VARIADIC_ARGS_LENGTH} GREATER 0)
        # Set PROJECT_SETTINGS_FILE with first value of ARGS
        list(GET VARIADIC_ARGS 0 PROJECT_SETTINGS_FILE)
    endif ()

    # Load project settings
    include(${PROJECT_SETTINGS_FILE})

    # Check MODULE_NAME is defined
    if (NOT DEFINED MODULE_NAME)
        message (FATAL_ERROR "Module name variable MODULE_NAME not defined in ${PROJECT_SETTINGS_FILE}")
    endif()

    #####
    # Module information

    # Set MODULE_TARGET_NAME
    if (NOT MODULE_TARGET_NAME)
        set (MODULE_TARGET_NAME ${MODULE_NAME})
    endif()

    # Set MODULE_NAME_LARGE
    if (NOT MODULE_NAME_LARGE)
        set (MODULE_NAME_LARGE ${MODULE_NAME})
    endif()

    # Set MODULE_SUMMARY
    if (NOT MODULE_SUMMARY)
        set (MODULE_SUMMARY ${MODULE_NAME})
    endif()

    # Set MODULE_DESCRIPTION
    if (NOT MODULE_DESCRIPTION)
        set (MODULE_DESCRIPTION ${MODULE_SUMMARY})
    endif()

    # Set MODULE_MACRO
    if (NOT MODULE_MACRO)
        string (TOUPPER ${MODULE_NAME} MODULE_MACRO)
    endif()

    #####
    # Module dependencies

    # Set MODULE_THIRDPARTY_PATH
    if (NOT MODULE_THIRDPARTY_PATH)
        set (MODULE_THIRDPARTY_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty")
    endif()

    # Set MODULE_FIND_PACKAGES
    if (NOT MODULE_FIND_PACKAGES)
        set (MODULE_FIND_PACKAGES "")
    endif()

    # Set MODULE_DEPENDENCIES
    if (NOT MODULE_DEPENDENCIES)
        set (MODULE_DEPENDENCIES ${MODULE_FIND_PACKAGES})
    endif()

    #####
    # Module version

    # Set MODULE_VERSION_FILE_PATH
    if (NOT MODULE_VERSION_FILE_PATH)
        set (MODULE_VERSION_FILE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../VERSION")
    endif()

    #####
    # Module files

    # Set MODULE_LICENSE_FILE_PATH
    if (NOT MODULE_LICENSE_FILE_PATH)
        set (MODULE_LICENSE_FILE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../LICENSE")
    endif()

    #####
    # Module external options

    # Set MODULE_LICENSE_FILE_PATH
    if (NOT MODULE_CPP_VERSION)
        set(MODULE_CPP_VERSION "C++17")
    endif()

    # Finish macro
    message (STATUS "Loaded project settings from ${PROJECT_SETTINGS_FILE} to project ${MODULE_NAME_LARGE}:")

endmacro()
