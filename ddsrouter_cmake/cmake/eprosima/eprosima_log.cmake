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
# eProsima Log
###############################################################################

# Configure eProsima Log by setting Info level if Debug and enforcing Fast DDS log info
macro(configure_eprosima_log)

    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        option(LOG_INFO "Compile logInfo messages" ON)
    else()
        option(LOG_INFO "No Compile logInfo messages" OFF)
    endif()

    if(${LOG_INFO})
        message(STATUS "Compiling logInfo messages (they still need to be activated in order to be seem).")
    else()
        message(STATUS "Not compiling logInfo messages.")
    endif()

endmacro()
