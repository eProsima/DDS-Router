# Copyright 2023 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
    DDSTestRepeater)

# Determine Fast DDS version
if ("${fastcdr_VERSION}" VERSION_LESS 2)
    set(DDS_TYPES_VERSION "v1")
else()
    set(DDS_TYPES_VERSION "v2")
endif()

set(TEST_SOURCES
    DDSTestRepeater.cpp
    ${PROJECT_SOURCE_DIR}/test/blackbox/ddsrouter_core/dds/types/${DDS_TYPES_VERSION}/HelloWorld/HelloWorld.cxx
    $<$<STREQUAL:${DDS_TYPES_VERSION},v2>:${PROJECT_SOURCE_DIR}/test/blackbox/ddsrouter_core/dds/types/${DDS_TYPES_VERSION}/HelloWorld/HelloWorldv1.cxx>
    $<$<STREQUAL:${DDS_TYPES_VERSION},v2>:${PROJECT_SOURCE_DIR}/test/blackbox/ddsrouter_core/dds/types/${DDS_TYPES_VERSION}/HelloWorld/HelloWorldCdrAux.ipp>
    ${PROJECT_SOURCE_DIR}/test/blackbox/ddsrouter_core/dds/types/${DDS_TYPES_VERSION}/HelloWorld/HelloWorldPubSubTypes.cxx
    ${PROJECT_SOURCE_DIR}/test/blackbox/ddsrouter_core/dds/types/${DDS_TYPES_VERSION}/HelloWorldKeyed/HelloWorldKeyed.cxx
    $<$<STREQUAL:${DDS_TYPES_VERSION},v2>:${PROJECT_SOURCE_DIR}/test/blackbox/ddsrouter_core/dds/types/${DDS_TYPES_VERSION}/HelloWorldKeyed/HelloWorldKeyedv1.cxx>
    $<$<STREQUAL:${DDS_TYPES_VERSION},v2>:${PROJECT_SOURCE_DIR}/test/blackbox/ddsrouter_core/dds/types/${DDS_TYPES_VERSION}/HelloWorldKeyed/HelloWorldKeyedCdrAux.ipp>
    ${PROJECT_SOURCE_DIR}/test/blackbox/ddsrouter_core/dds/types/${DDS_TYPES_VERSION}/HelloWorldKeyed/HelloWorldKeyedPubSubTypes.cxx)

set(TEST_LIST
        repeater_initial_peers_communication_UDPv4
    )

set(TEST_NEEDED_SOURCES
    )

set(TEST_EXTRA_HEADERS
    ${PROJECT_SOURCE_DIR}/test/blackbox/ddsrouter_core/dds/types/${DDS_TYPES_VERSION}/HelloWorld
    ${PROJECT_SOURCE_DIR}/test/blackbox/ddsrouter_core/dds/types)

add_blackbox_executable(
    "${TEST_NAME}"
    "${TEST_SOURCES}"
    "${TEST_LIST}"
    "${TEST_NEEDED_SOURCES}"
    "${TEST_EXTRA_HEADERS}")
