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
    discovery_server
    dds_rtps
    dds

    rpc/ros2_services_cloud
    rpc/ros2_services_repeater
    rpc/ros2_services_repeater_with_talker
    rpc/ros2_services_correct_target
    rpc/ros2_services_simple
    rpc/ros2_services_without_router
    rpc/ros2_services_trivial
    rpc/ros2_services_trivial_multiple

    transparency/partitions
    transparency/durability
    transparency/ownership

    security/backdoor
    security/backdoor_dds
    security/secure_trespassing
    security/secure_wan

    forwarding_routes/basic_route
    forwarding_routes/parallel_routes
    forwarding_routes/topic_routes
    forwarding_routes/repeater

    remove_unused_entities/disconnect
    remove_unused_entities/disconnect_wan
    remove_unused_entities/persist_diff_domain
    remove_unused_entities/persist_same_domain
    remove_unused_entities/reconnect
    remove_unused_entities/reconnect_wan

    frequency/max-tx-rate/specs
    frequency/max-tx-rate/participant
    frequency/max-rx-rate/specs
    frequency/max-rx-rate/participant
    frequency/downsampling/specs
    frequency/downsampling/participant

    manual_topics/generic
    manual_topics/participants
    manual_topics/precedence

    discovery_trigger/reader
    discovery_trigger/writer
    discovery_trigger/none
    discovery_trigger/any/reader
    discovery_trigger/any/writer
)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_cases DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/docker-compose.sh
        ${CMAKE_CURRENT_BINARY_DIR}/docker-compose.sh
        COPYONLY)

file(
    COPY
        ${CMAKE_CURRENT_SOURCE_DIR}/scripts
    DESTINATION
        ${CMAKE_CURRENT_BINARY_DIR}
    )

find_program (BASH_PROGRAM bash)

# Populate the tests
foreach(TEST IN LISTS TESTS)

    set(TEST_NAME "tool.application.ddsrouter.compose.${TEST}")
    string(REPLACE "/" "_" TEST_NAME ${TEST_NAME})

    message(STATUS "Building test ${TEST_NAME}")

    add_test(
            NAME ${TEST_NAME}
            COMMAND ${BASH_PROGRAM} ${CMAKE_CURRENT_BINARY_DIR}/docker-compose.sh
                -t ${TEST_NAME}
                -f ${CMAKE_CURRENT_BINARY_DIR}/test_cases/${TEST}/compose.yml)

endforeach()

# Set flaky tests as xfail
add_xfail_label(${CMAKE_CURRENT_SOURCE_DIR}/TEST_XFAIL.list)
