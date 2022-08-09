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
# Configure Gneral CMake options macro
###############################################################################

# Set General CMake options
# This set the CMake options to the default values
# It also avoid warnings when a option is set and not used (typically will happen in projects with subpackages)
#
# CMAKE OPTIONS ALLOWED
#
# - BUILD_TESTS                 : OFF
# - BUILD_DOCS_TESTS            : BUILD_TESTS
# - BUILD_TOOL_TESTS            : BUILD_TESTS
# - BUILD_LIBRARY_TESTS         : BUILD_TESTS
#
# - BUILD_ALL                   : BUILD_TESTS
# - BUILD_DOCS                  : BUILD_DOCS_TESTS
# - BUILD_TOOL                  : ON
# - BUILD_LIBRARY               : ON
#
# - CODE_COVERAGE               : OFF
# - CMAKE_BUILD_TYPE            : Release
#
# - LOG_INFO                    : OFF       // TODO change LOG cmake options to make them smarter
#
# ARGUMENTS:
# NONE
macro(configure_cmake_options)

    # BUILD TEST
    if (NOT DEFINED BUILD_TESTS)
        set(BUILD_TESTS OFF)
    endif()

    if (NOT DEFINED BUILD_DOCS_TESTS)
        set(BUILD_DOCS_TESTS ${BUILD_TESTS})
    endif()

    if (NOT DEFINED BUILD_TOOL_TESTS)
        set(BUILD_TOOL_TESTS ${BUILD_TESTS})
    endif()

    if (NOT DEFINED BUILD_LIBRARY_TESTS)
        set(BUILD_LIBRARY_TESTS ${BUILD_TESTS})
    endif()

    # BUILD
    if (NOT DEFINED BUILD_ALL)
        set(BUILD_ALL ${BUILD_TESTS})
    endif()

    if (NOT DEFINED BUILD_DOCS)
        if (${BUILD_ALL} OR ${BUILD_DOCS_TESTS})
            set(BUILD_DOCS ON)
        else()
            set(BUILD_DOCS OFF)
        endif()
    endif()

    if (NOT DEFINED BUILD_TOOL)
        set(BUILD_TOOL ON)
    endif()

    if (NOT DEFINED BUILD_LIBRARY)
        set(BUILD_LIBRARY ON)
    endif()

    # COVERAGE
    if (NOT DEFINED CODE_COVERAGE)
        set(CODE_COVERAGE OFF)
    endif()

    # CMake type
    if (NOT DEFINED CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE "Release")
    endif()

    # LOG
    # TODO change LOG cmake options to make them smarter
    if (NOT DEFINED LOG_INFO AND "${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
        set(LOG_INFO ON)
    else ()
        set(LOG_INFO CACHE BOOL OFF)
    endif()

endmacro()
