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
# Yaml DDSRouter Configuration Test #
#####################################

set(TEST_NAME YamlConfigurationDDSRouterTest)

set(TEST_SOURCES
        YamlConfigurationDDSRouterTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/configuration/DDSRouterConfiguration.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/configuration/participant/DiscoveryServerParticipantConfiguration.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/configuration/participant/ParticipantConfiguration.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/configuration/participant/SimpleParticipantConfiguration.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/exception/Exception.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/security/tls/TlsConfiguration.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/address/Address.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/address/DiscoveryServerConnectionAddress.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/participant/ParticipantId.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/participant/ParticipantKind.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/endpoint/DomainId.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/endpoint/Guid.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/endpoint/GuidPrefix.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/topic/FilterTopic.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/topic/RealTopic.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/topic/Topic.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/topic/WildcardTopic.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/utils.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/yaml/implementations/YamlConfigurationEntity.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/yaml/implementations/YamlConfigurationParticipant.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/yaml/YamlConfiguration.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/yaml/YamlConfigurationDDSRouter.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/yaml/YamlManager.cpp
    )

set(TEST_LIST
        get_ddsrouter_configuration_trivial
        get_ddsrouter_configuration_ros_case
    )

set(TEST_EXTRA_LIBRARIES
        fastcdr
        fastrtps
        yaml-cpp
        $<$<BOOL:${WIN32}>:iphlpapi$<SEMICOLON>Shlwapi>
    )

add_unittest_executable(
    "${TEST_NAME}"
    "${TEST_SOURCES}"
    "${TEST_LIST}"
    "${TEST_EXTRA_LIBRARIES}")
