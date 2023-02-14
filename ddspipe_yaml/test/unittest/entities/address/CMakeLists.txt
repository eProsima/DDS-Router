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

################################
# Yaml GetEntities Adress Test #
################################

set(TEST_NAME YamlGetEntityAddressTest)

set(TEST_SOURCES
        ${PROJECT_SOURCE_DIR}/src/cpp/YamlReader_generic.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/YamlReader_participants.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/YamlReader_types.cpp
        YamlGetEntityAddressTest.cpp
    )

set(TEST_LIST
        get_transport_protocol
        get_ip_version
        get_port
        get_ip
        get_address_ip
        get_address_domain
        get_address_defaults
        ip_and_domain
    )

set(TEST_EXTRA_LIBRARIES
        yaml-cpp
        fastcdr
        fastrtps
        cpp_utils
        ddspipe_core
        ddspipe_participants
    )

add_unittest_executable(
    "${TEST_NAME}"
    "${TEST_SOURCES}"
    "${TEST_LIST}"
    "${TEST_EXTRA_LIBRARIES}")

################################################
# Yaml GetEntities DiscoveryServerAddress Test #
################################################

set(TEST_NAME YamlGetEntityDiscoveryServerAddressTest)

set(TEST_SOURCES
        ${PROJECT_SOURCE_DIR}/src/cpp/YamlReader_generic.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/YamlReader_participants.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/YamlReader_types.cpp
        YamlGetEntityDiscoveryServerAddressTest.cpp
    )

set(TEST_LIST
        get_ds_address
        get_ds_address_negative
    )

set(TEST_EXTRA_LIBRARIES
        yaml-cpp
        fastcdr
        fastrtps
        cpp_utils
        ddspipe_core
        ddspipe_participants
    )

add_unittest_executable(
    "${TEST_NAME}"
    "${TEST_SOURCES}"
    "${TEST_LIST}"
    "${TEST_EXTRA_LIBRARIES}")
