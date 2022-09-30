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

################
# DDS Test WAN #
################

set(TEST_NAME
    DDSTestWAN)

set(TEST_SOURCES
    DDSTestWAN.cpp
    ${PROJECT_SOURCE_DIR}/test/blackbox/ddsrouter_core/dds/types/HelloWorld/HelloWorld.cxx
    ${PROJECT_SOURCE_DIR}/test/blackbox/ddsrouter_core/dds/types/HelloWorld/HelloWorldPubSubTypes.cxx
    ${PROJECT_SOURCE_DIR}/test/blackbox/ddsrouter_core/dds/types/HelloWorldKeyed/HelloWorldKeyed.cxx
    ${PROJECT_SOURCE_DIR}/test/blackbox/ddsrouter_core/dds/types/HelloWorldKeyed/HelloWorldKeyedPubSubTypes.cxx)

set(TEST_LIST
    end_to_end_discovery_server_WAN_communication_UDPv4
    end_to_end_initial_peers_WAN_communication_UDPv4

    end_to_end_discovery_server_WAN_communication_UDPv6
    end_to_end_initial_peers_WAN_communication_UDPv6

    end_to_end_discovery_server_WAN_communication_TCPv4
    end_to_end_initial_peers_WAN_communication_TCPv4

    end_to_end_discovery_server_WAN_communication_TCPv6
    end_to_end_initial_peers_WAN_communication_TCPv6

    end_to_end_discovery_server_WAN_communication_TLSv4
    end_to_end_initial_peers_WAN_communication_TLSv4

    end_to_end_discovery_server_WAN_communication_TLSv6
    end_to_end_initial_peers_WAN_communication_TLSv6

    end_to_end_discovery_server_WAN_communication_high_throughput
    end_to_end_initial_peers_WAN_communication_high_throughput
    )


set(TEST_NEEDED_SOURCES
    # TLS security files
    ../../resources/tls/ca.crt
    ../../resources/tls/ddsrouter.crt
    ../../resources/tls/ddsrouter.key
    ../../resources/tls/dh_params.pem
    )

set(TEST_EXTRA_HEADERS
    ${PROJECT_SOURCE_DIR}/test/blackbox/ddsrouter_core/dds/types/HelloWorld
    ${PROJECT_SOURCE_DIR}/test/blackbox/ddsrouter_core/dds/types)

add_blackbox_executable(
    "${TEST_NAME}"
    "${TEST_SOURCES}"
    "${TEST_LIST}"
    "${TEST_NEEDED_SOURCES}"
    "${TEST_EXTRA_HEADERS}")

# Set flaky tests as xfail
add_xfail_label(${CMAKE_CURRENT_SOURCE_DIR}/TEST_XFAIL.list)
