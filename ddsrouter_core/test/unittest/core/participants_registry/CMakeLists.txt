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

#########################
# Participants Database #
#########################

set(TEST_NAME ParticipantsRegistryTest)

set(TEST_SOURCES
        ParticipantsRegistryTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/communication/payload_pool/TopicPayloadPool.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/configuration/participant/ParticipantConfiguration.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/configuration/participant/SimpleParticipantConfiguration.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/configuration/participant/DiscoveryServerParticipantConfiguration.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/dynamic/DiscoveryDatabase.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/participant/IParticipant.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/participant/rtps/CommonRTPSRouterParticipant.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/participant/rtps/DiscoveryServerParticipant.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/participant/rtps/LocalDiscoveryServerParticipant.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/participant/rtps/SimpleParticipant.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/participant/rtps/WANParticipant.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/participant/auxiliar/BlankParticipant.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/participant/auxiliar/DummyParticipant.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/participant/auxiliar/EchoParticipant.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/reader/auxiliar/BlankReader.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/reader/auxiliar/DummyReader.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/reader/IReader.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/reader/rtps/Reader.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/writer/auxiliar/BlankWriter.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/writer/auxiliar/DummyWriter.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/writer/IWriter.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/writer/rtps/Writer.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/participant/ParticipantId.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/topic/Topic.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/address/Address.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/address/DiscoveryServerConnectionAddress.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/security/tls/TlsConfiguration.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/endpoint/Endpoint.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/endpoint/QoS.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/dds/Guid.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/dds/GuidPrefix.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/dds/Payload.cpp
    )
all_header_sources("${TEST_SOURCES}")

set(TEST_LIST
        add_participant
        pop_participant
    )

set(TEST_EXTRA_LIBRARIES
        fastcdr
        fastrtps
        ddsrouter_utils
        ddsrouter_event
    )

add_unittest_executable(
        "${TEST_NAME}"
        "${TEST_SOURCES}"
        "${TEST_LIST}"
        "${TEST_EXTRA_LIBRARIES}"
    )
