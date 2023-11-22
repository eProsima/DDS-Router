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

#####################################
# Yaml DdsRouter Configuration Test #
#####################################

set(TEST_NAME YamlReaderConfigurationTest)

set(TEST_SOURCES
        YamlReaderConfigurationTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/YamlReaderConfiguration.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/YamlReader_configuration.cpp
    )

set(TEST_LIST
        # ddsrouter_configuration_v1_not_supported
        # get_ddsrouter_configuration_v2
        get_ddsrouter_configuration_no_version
        version_negative_cases
        number_of_threads
        remove_unused_entities
        discovery_trigger
        valid_routes
        invalid_routes
        history_depth
        max_tx_rate
        max_rx_rate
        downsampling
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

#####################################
# Yaml DdsRouter Configuration Test #
#####################################

set(TEST_NAME YamlGetConfigurationDdsRouterTest)

set(TEST_SOURCES
        YamlGetConfigurationDdsRouterTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/YamlReaderConfiguration.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/YamlReader_configuration.cpp
    )

set(TEST_LIST
        get_ddsrouter_configuration_trivial
        get_ddsrouter_configuration_ros_case
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
