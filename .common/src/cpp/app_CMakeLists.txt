# Copyright 2021 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
set(EXECUTABLE_TARGET ${PROJECT_NAME})
set(EXECUTABLE_NAME ddsrouter)

###############################################################################
# Compile
###############################################################################
message(STATUS "Compiling ${EXECUTABLE_NAME}...")

# Get src files
set(EXECUTABLE_SOURCES
        "main.cpp"
        "user_interface/arguments_configuration.cpp"
    )

add_executable(${EXECUTABLE_TARGET} ${EXECUTABLE_SOURCES})

target_link_libraries(${EXECUTABLE_TARGET}
        ${SUBMODULE_PROJECT_DEPENDENCIES}
    )

install(TARGETS ${EXECUTABLE_TARGET}
    RUNTIME DESTINATION ${BIN_INSTALL_DIR})

set_target_properties(${EXECUTABLE_TARGET} PROPERTIES OUTPUT_NAME "${EXECUTABLE_NAME}")

###############################################################################
# Install Resources
###############################################################################

# Install resource files
INSTALL(
        DIRECTORY ${PROJECT_SOURCE_DIR}/resources/
        DESTINATION ${DATA_INSTALL_DIR}/resources/
    )
