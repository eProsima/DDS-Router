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
# In Source Build Prevention
###############################################################################

# Prevent to build a CMake project in source directory.
# Always use /build or any other auxiliar directory
macro(in_source_build_prevention)

    if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
        message(FATAL_ERROR "In-source build detected!")
    endif()
    # Thanks to: https://towardsdatascience.com/7-tips-for-clean-cmake-scripts-c8d276587389

    # Finish macro
    message(STATUS "Checked positively that build is not source path.")

endmacro()
