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

#######################
# Participant Factory #
#######################

set(TEST_NAME ParticipantFactoryTest)

set(TEST_SOURCES
        ParticipantFactoryTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/communication/PayloadPool.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/configuration/BaseConfiguration.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/configuration/implementations/DomainId_configuration.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/configuration/implementations/Address_configuration.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/configuration/implementations/GuidPrefix_configuration.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/configuration/implementations/DiscoveryServerConnectionAddress_configuration.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/configuration/ParticipantConfiguration.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/configuration/SimpleParticipantConfiguration.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/configuration/DiscoveryServerParticipantConfiguration.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/configuration/WANParticipantConfiguration.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/dynamic/DiscoveryDatabase.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/exceptions/Exception.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/participant/implementations/auxiliar/DummyParticipant.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/participant/implementations/auxiliar/EchoParticipant.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/participant/implementations/auxiliar/VoidParticipant.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/participant/implementations/rtps/SimpleParticipant.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/participant/implementations/rtps/LocalDiscoveryServerParticipant.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/participant/implementations/rtps/WANParticipant.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/participant/ParticipantFactory.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/reader/implementations/auxiliar/BaseReader.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/reader/implementations/auxiliar/DummyReader.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/reader/implementations/auxiliar/VoidReader.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/reader/implementations/rtps/Reader.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/security/tls/TlsConfiguration.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/security/tls/TlsConfigurationBoth.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/security/tls/TlsConfigurationClient.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/security/tls/TlsConfigurationServer.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/Data.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/address/Address.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/address/DiscoveryServerConnectionAddress.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/endpoint/DomainId.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/endpoint/Endpoint.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/endpoint/Guid.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/endpoint/GuidPrefix.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/endpoint/QoS.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/participant/ParticipantId.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/participant/ParticipantKind.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/topic/RealTopic.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/topic/Topic.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/utils.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/writer/implementations/auxiliar/BaseWriter.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/writer/implementations/auxiliar/DummyWriter.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/writer/implementations/auxiliar/EchoWriter.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/writer/implementations/auxiliar/VoidWriter.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/writer/implementations/rtps/Writer.cpp
    )

set(TEST_LIST
        create_void_participant
        create_echo_participant
        create_dummy_participant
        create_simple_participant
        create_invalid_participant
        remove_participant
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
        "${TEST_EXTRA_LIBRARIES}"
    )
