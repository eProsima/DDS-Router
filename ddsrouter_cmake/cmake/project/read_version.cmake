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
# Version Reader macro
###############################################################################

# Load version variables from version file.
# This file (default VERSION) must contain the following variables:
# - VERSION_MAJOR x
# - VERSION_MINOR y
# - VERSION_PATCH z
#
# RETURN VARIABLES:
# - MODULE_VERSION_MAJOR    : x
# - MODULE_VERSION_MINOR    : y
# - MODULE_VERSION_PATCH    : z
# - MODULE_VERSION_STRING   : x.y.z
# - MODULE_VERSION          : x.y.z
#
# Arguments:
# (optional) VERSION_FILE_NAME -> File to load version parameters [default: VERSION]
macro(read_version)

    # If version already set, it must not read file
    if( NOT DEFINED MODULE_VERSION_MAJOR OR
        NOT DEFINED MODULE_VERSION_MAJOR OR
        NOT DEFINED MODULE_VERSION_MAJOR)

        # Use default file
        set(VERSION_FILE_NAME ${CMAKE_CURRENT_SOURCE_DIR}/VERSION)

        # Check if there is an argument, if it is set, modify default.
        set(VARIADIC_ARGS ${ARGN})
        list(LENGTH VARIADIC_ARGS VARIADIC_ARGS_LENGTH)
        if (${VARIADIC_ARGS_LENGTH} GREATER 0)
            # Set VERSION_FILE_NAME with first value of ARGS
            list(GET VARIADIC_ARGS 0 VERSION_FILE_NAME)
        endif ()

        # Read version file
        file(READ ${VERSION_FILE_NAME} READ_VERSION_FILE)

        string(REGEX MATCH "VERSION_MAJOR ([0-9]*)" _ ${READ_VERSION_FILE})
        set(MODULE_VERSION_MAJOR ${CMAKE_MATCH_1})

        string(REGEX MATCH "VERSION_MINOR ([0-9]*)" _ ${READ_VERSION_FILE})
        set(MODULE_VERSION_MINOR ${CMAKE_MATCH_1})

        string(REGEX MATCH "VERSION_PATCH ([0-9]*)" _ ${READ_VERSION_FILE})
        set(MODULE_VERSION_PATCH ${CMAKE_MATCH_1})

    endif()

    # VERSION complete string
    if(NOT DEFINED MODULE_VERSION)
        set (MODULE_VERSION ${MODULE_VERSION_MAJOR}.${MODULE_VERSION_MINOR}.${MODULE_VERSION_PATCH})
    endif()

    if(NOT DEFINED MODULE_VERSION_STRING)
        set (MODULE_VERSION_STRING ${MODULE_VERSION})
    endif()

    # Finish macro
    message(STATUS "Read from version file ${VERSION_FILE_NAME} version: ${MODULE_VERSION_MAJOR}.${MODULE_VERSION_MINOR}.${MODULE_VERSION_PATCH}")

endmacro()
