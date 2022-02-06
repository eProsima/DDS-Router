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
# COMPILE AND INSTALL TOOL EXECUTABLE
###############################################################################
set(EXECUTABLE_TARGET discovery-tool)
set(EXECUTABLE_NAME discovery)

###############################################################################
# Find requirements
###############################################################################
if(NOT fastcdr_FOUND)
    find_package(fastcdr REQUIRED)
endif()

if(NOT fastrtps_FOUND)
    find_package(fastrtps REQUIRED)
endif()

# Even when it is forced in /tool, this cmake could be compiled independently, so this is needed
if(NOT ddsrouter_FOUND)
    find_package(ddsrouter REQUIRED)
endif()

###############################################################################
# Compile
###############################################################################
message(STATUS "Compiling ${EXECUTABLE_NAME}...")

# Get src files
set(EXECUTABLE_SOURCES
        "user_interface/arguments_configuration.cpp"
        "main.cpp"
        "yaml/YamlReaderDiscoveryConfiguration.cpp"
    )

add_executable(${EXECUTABLE_TARGET} ${EXECUTABLE_SOURCES})

target_link_libraries(${EXECUTABLE_TARGET}
        fastrtps
        fastcdr
        ddsrouter
        yaml-cpp
        $<$<BOOL:${WIN32}>:iphlpapi$<SEMICOLON>Shlwapi>
    )

install(TARGETS ${EXECUTABLE_TARGET}
    RUNTIME DESTINATION ${BIN_INSTALL_DIR})

set_target_properties(${EXECUTABLE_TARGET} PROPERTIES OUTPUT_NAME "${EXECUTABLE_NAME}")
