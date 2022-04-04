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
# - MODULE_MAJOR_VERSION    : x
# - MODULE_MINOR_VERSION    : y
# - MODULE_PATCH_VERSION    : z
# - MODULE_VERSION_STRING   : x.y.z
# - MODULE_VERSION          : x.y.z
#
# Arguments:
# (optional) VERSION_FILE_NAME -> File to load version parameters [default: VERSION]
macro(read_version)

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
    set(MODULE_MAJOR_VERSION ${CMAKE_MATCH_1})

    string(REGEX MATCH "VERSION_MINOR ([0-9]*)" _ ${READ_VERSION_FILE})
    set(MODULE_MINOR_VERSION ${CMAKE_MATCH_1})

    string(REGEX MATCH "VERSION_PATCH ([0-9]*)" _ ${READ_VERSION_FILE})
    set(MODULE_PATCH_VERSION ${CMAKE_MATCH_1})

    # VERSION complete string
    set (MODULE_VERSION_STRING ${MODULE_MAJOR_VERSION}.${MODULE_MINOR_VERSION}.${MODULE_PATCH_VERSION})
    set (MODULE_VERSION ${MODULE_VERSION_STRING})

    # Finish macro
    message(STATUS "Read from version file ${VERSION_FILE_NAME} version: ${MODULE_MAJOR_VERSION}.${MODULE_MINOR_VERSION}.${MODULE_PATCH_VERSION}")

endmacro()
