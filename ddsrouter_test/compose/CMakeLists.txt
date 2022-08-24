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

# Name of files to test
set(TESTS
    repeater
    repeater_tcp
    tcp
    udp
    filter_local
    discovery_server
)

foreach(TEST ${TESTS})
    file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/${TEST} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
endforeach()

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/docker-compose.sh
        ${CMAKE_CURRENT_BINARY_DIR}/docker-compose.sh
        COPYONLY)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/execute_and_validate_subscriber.py
        ${CMAKE_CURRENT_BINARY_DIR}/execute_and_validate_subscriber.py
        COPYONLY)

find_program (BASH_PROGRAM bash)

# Populate the tests
foreach(TEST IN LISTS TESTS)

    set(TEST_NAME "tool.application.ddsrouter.compose.${TEST}")

    message(STATUS "Building test ${TEST_NAME}")

    add_test(
            NAME ${TEST_NAME}
            COMMAND ${BASH_PROGRAM} ${CMAKE_CURRENT_BINARY_DIR}/docker-compose.sh
                -t ${TEST_NAME}
                -f ${CMAKE_CURRENT_BINARY_DIR}/${TEST}/compose.yml)

endforeach()
