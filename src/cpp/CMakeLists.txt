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
# Compilation
###############################################################################

# Get sources
if(MSVC)
    file(GLOB_RECURSE ALL_HEADERS
            ${PROJECT_SOURCE_DIR}/include/**/*.h
            ${PROJECT_SOURCE_DIR}/include/**/*.hpp
            ${PROJECT_SOURCE_DIR}/src/cpp/**/*.h
            ${PROJECT_SOURCE_DIR}/src/cpp/**/*.hpp
            ${PROJECT_SOURCE_DIR}/types/**/*.h
            ${PROJECT_SOURCE_DIR}/types/**/*.hpp
        )
endif(MSVC)

set(${PROJECT_NAME}_SOURCES
        ${ALL_HEADERS}
        DataBroker.cpp
        DataBrokerListener.cpp
        DataBrokerParticipant.cpp
        DataBrokerROSParticipant.cpp
        main.cpp
        types/StdString/StdString.cxx
        types/StdString/StdStringPubSubTypes.cxx
    )

# Add executable
add_executable(${PROJECT_NAME}
        ${${PROJECT_NAME}_SOURCES}
    )

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    )

target_link_libraries(${PROJECT_NAME} fastrtps fastcdr)

###############################################################################
# Installation
###############################################################################
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${BIN_INSTALL_DIR})
