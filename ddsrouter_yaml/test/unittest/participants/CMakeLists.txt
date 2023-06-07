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

#################################################
# Yaml GetConfigurations CommonParticipant Test #
#################################################

set(TEST_NAME YamlGetCommonParticipantConfigurationTest)

set(TEST_SOURCES
        ${PROJECT_SOURCE_DIR}/src/cpp/YamlReaderConfiguration.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/YamlReader_configuration.cpp
        YamlGetCommonParticipantConfigurationTest.cpp
    )

set(TEST_LIST
        get_participant
        get_participant_negative
    )

set(TEST_EXTRA_LIBRARIES
        yaml-cpp
        fastcdr
        fastrtps
        cpp_utils
        ddspipe_core
        ddspipe_participants
        ddspipe_yaml
        ddsrouter_core
    )

add_unittest_executable(
    "${TEST_NAME}"
    "${TEST_SOURCES}"
    "${TEST_LIST}"
    "${TEST_EXTRA_LIBRARIES}")

#################################################
# Yaml GetConfigurations SimpleParticipant Test #
#################################################

set(TEST_NAME YamlGetSimpleParticipantConfigurationTest)

set(TEST_SOURCES
        ${PROJECT_SOURCE_DIR}/src/cpp/YamlReaderConfiguration.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/YamlReader_configuration.cpp
        YamlGetSimpleParticipantConfigurationTest.cpp
    )

set(TEST_LIST
        get_participant
        get_participant_negative
    )

set(TEST_EXTRA_LIBRARIES
        yaml-cpp
        fastcdr
        fastrtps
        cpp_utils
        ddspipe_core
        ddspipe_participants
        ddspipe_yaml
        ddsrouter_core
    )

set(TEST_NEEDED_SOURCES "")

set(TEST_EXTRA_HEADERS
        ${PROJECT_SOURCE_DIR}/test/unittest/TestUtils)

add_unittest_executable(
    "${TEST_NAME}"
    "${TEST_SOURCES}"
    "${TEST_LIST}"
    "${TEST_EXTRA_LIBRARIES}"
    "${TEST_NEEDED_SOURCES}"
    "${TEST_EXTRA_HEADERS}")

##########################################################
# Yaml GetConfigurations DiscoveryServerParticipant Test #
##########################################################

set(TEST_NAME YamlGetDiscoveryServerParticipantConfigurationTest)

set(TEST_SOURCES
        ${PROJECT_SOURCE_DIR}/src/cpp/YamlReaderConfiguration.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/YamlReader_configuration.cpp
        YamlGetDiscoveryServerParticipantConfigurationTest.cpp
    )

set(TEST_LIST
        get_participant_minimum
        get_participant_domain
        get_participant_listening_addresses
        get_participant_connection_addresses
        get_participant_negative
        tls_configuration_client
        tls_configuration_server
        tls_configuration_client_server
        tls_configuration_inactive
        tls_configuration_incorrect_empty
        tls_configuration_incorrect_format
    )

set(TEST_EXTRA_LIBRARIES
        yaml-cpp
        fastcdr
        fastrtps
        cpp_utils
        ddspipe_core
        ddspipe_participants
        ddspipe_yaml
        ddsrouter_core
    )

set(TEST_NEEDED_SOURCES "")

set(TEST_EXTRA_HEADERS
        ${PROJECT_SOURCE_DIR}/test/unittest/TestUtils)

add_unittest_executable(
    "${TEST_NAME}"
    "${TEST_SOURCES}"
    "${TEST_LIST}"
    "${TEST_EXTRA_LIBRARIES}"
    "${TEST_NEEDED_SOURCES}"
    "${TEST_EXTRA_HEADERS}")

##########################################################
# Yaml GetConfigurations InitialPeersParticipant Test #
##########################################################

set(TEST_NAME YamlGetInitialPeersParticipantConfigurationTest)

set(TEST_SOURCES
        ${PROJECT_SOURCE_DIR}/src/cpp/YamlReaderConfiguration.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/YamlReader_configuration.cpp
        YamlGetInitialPeersParticipantConfigurationTest.cpp
    )

set(TEST_LIST
        get_participant_minimum
        get_participant_repeater
    )

set(TEST_EXTRA_LIBRARIES
        yaml-cpp
        fastcdr
        fastrtps
        cpp_utils
        ddspipe_core
        ddspipe_participants
        ddspipe_yaml
        ddsrouter_core
    )

set(TEST_NEEDED_SOURCES "")

set(TEST_EXTRA_HEADERS
        ${PROJECT_SOURCE_DIR}/test/unittest/TestUtils)

add_unittest_executable(
    "${TEST_NAME}"
    "${TEST_SOURCES}"
    "${TEST_LIST}"
    "${TEST_EXTRA_LIBRARIES}"
    "${TEST_NEEDED_SOURCES}"
    "${TEST_EXTRA_HEADERS}")
