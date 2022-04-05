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
# C++ Version
###############################################################################

# Set C++ project version and check it is available
#
# ARGUMENTS:
# - CPP_VERSION: C++ version to use [int format]
macro(check_cpp CPP_VERSION)

    # C++17
    if("${CPP_VERSION}" STREQUAL "17" OR "${CPP_VERSION}" STREQUAL "C++17")
        check_cpp_17()
    else()
        message(FATAL_ERROR "C++ version ${CPP_VERSION} is not supported yet by this cmake macro")
    endif()

endmacro()

macro(check_cpp_17)

    include(CheckCXXCompilerFlag)
    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG OR
            CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        check_cxx_compiler_flag(-std=c++17 SUPPORTS_CXX17)
        if(NOT SUPPORTS_CXX17)
            message(FATAL_ERROR "Compiler doesn't support C++17")
        endif()
    endif()

    # Finish macro
    message(STATUS "C++17 is supported and used")

endmacro()
