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

# prevent multiple inclusion
if(DEFINED _ddsrouter_cmake_INCLUDED)
  message(FATAL_ERROR "ddsrouter_cmake/cmake/core/load_all.cmake included multiple times")
endif()
set(_ddsrouter_cmake_INCLUDED TRUE)

if(NOT DEFINED ddsrouter_cmake_DIR)
  message(FATAL_ERROR "ddsrouter_cmake_DIR is not set")
endif()

######
# Include functions, methods and modules
# Get every cmake that must be included
file(GLOB_RECURSE
        CMAKE_INCLUSIONS
        "${ddsrouter_cmake_DIR}/**/*.cmake"
    )

# Include functions / macros
foreach(FILENAME ${CMAKE_INCLUSIONS})
    include(${FILENAME})
endforeach()

# Add modules folder so FindPackages file are in CMAKE_MODULE_PATH and could be accessed
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ddsrouter_cmake_DATA_DIR}/ddsrouter_cmake/modules)

######
# Set some useful variables

# Path to the templates for Config.cmake files
set(ddsrouter_cmake_CONFIG_TEMPLATES_PATH "${ddsrouter_cmake_DATA_DIR}/ddsrouter_cmake/templates/cmake/packaging")
set(ddsrouter_cmake_LIBRARY_HEADERS_TEMPLATES_PATH "${ddsrouter_cmake_DATA_DIR}/ddsrouter_cmake/templates/cpp/library")
