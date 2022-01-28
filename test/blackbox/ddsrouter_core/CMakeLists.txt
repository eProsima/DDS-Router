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
# Trivial Test #
################

set(TEST_NAME
    TrivialTest)

set(TEST_SOURCES
    TrivialTest.cpp)

set(TEST_LIST
    trivial_void_initialization
    trivial_dummy_initialization
    trivial_communication)

set(TEST_NEEDED_SOURCES
    resources/configurations/trivial_test_dummy_configuration.yaml
    resources/configurations/trivial_test_void_configuration.yaml)

add_blackbox_executable(
    "${TEST_NAME}"
    "${TEST_SOURCES}"
    "${TEST_LIST}"
    "${TEST_NEEDED_SOURCES}")

############
# DDS Test #
############

set(TEST_NAME
    DDSTest)

set(TEST_SOURCES
    DDSTest.cpp
    types/HelloWorld/HelloWorld.cxx
    types/HelloWorld/HelloWorldPubSubTypes.cxx
    types/HelloWorldKeyed/HelloWorldKeyed.cxx
    types/HelloWorldKeyed/HelloWorldKeyedPubSubTypes.cxx)

set(TEST_LIST
    simple_initialization
    end_to_end_local_communication
    end_to_end_local_communication_keyed
    end_to_end_WAN_communication_UDPv4
    end_to_end_WAN_communication_UDPv6
    end_to_end_WAN_communication_TCPv4
    end_to_end_WAN_communication_TCPv6
    end_to_end_WAN_communication_TLSv4
    end_to_end_WAN_communication_TLSv6)

set(TEST_NEEDED_SOURCES
    resources/configurations/dds_test_simple_configuration.yaml
    # UDPv4
    resources/configurations/WAN/UDP/IPv4/server.yaml
    resources/configurations/WAN/UDP/IPv4/client.yaml
    resources/configurations/WAN/UDP/IPv4/server-client-A.yaml
    resources/configurations/WAN/UDP/IPv4/server-client-B.yaml
    # UDPv6
    resources/configurations/WAN/UDP/IPv6/server.yaml
    resources/configurations/WAN/UDP/IPv6/client.yaml
    resources/configurations/WAN/UDP/IPv6/server-client-A.yaml
    resources/configurations/WAN/UDP/IPv6/server-client-B.yaml
    # TCPv4
    resources/configurations/WAN/TCP/IPv4/server.yaml
    resources/configurations/WAN/TCP/IPv4/client.yaml
    resources/configurations/WAN/TCP/IPv4/server-client-A.yaml
    resources/configurations/WAN/TCP/IPv4/server-client-B.yaml
    # TCPv6
    resources/configurations/WAN/TCP/IPv6/server.yaml
    resources/configurations/WAN/TCP/IPv6/client.yaml
    resources/configurations/WAN/TCP/IPv6/server-client-A.yaml
    resources/configurations/WAN/TCP/IPv6/server-client-B.yaml
    # TLSv4
    resources/configurations/WAN/TLS/IPv4/server.yaml
    resources/configurations/WAN/TLS/IPv4/client.yaml
    resources/configurations/WAN/TLS/IPv4/server-client-A.yaml
    resources/configurations/WAN/TLS/IPv4/server-client-B.yaml
    # TLSv6
    resources/configurations/WAN/TLS/IPv6/server.yaml
    resources/configurations/WAN/TLS/IPv6/client.yaml
    resources/configurations/WAN/TLS/IPv6/server-client-A.yaml
    resources/configurations/WAN/TLS/IPv6/server-client-B.yaml
    # TLS security files
    resources/configurations/WAN/TLS/security/ca.crt
    resources/configurations/WAN/TLS/security/ddsrouter.crt
    resources/configurations/WAN/TLS/security/ddsrouter.key
    resources/configurations/WAN/TLS/security/dh_params.pem)

set(TEST_EXTRA_HEADERS
    types/HelloWorld
    types/HelloWorldKeyed)

add_blackbox_executable(
    "${TEST_NAME}"
    "${TEST_SOURCES}"
    "${TEST_LIST}"
    "${TEST_NEEDED_SOURCES}"
    "${TEST_EXTRA_HEADERS}")
