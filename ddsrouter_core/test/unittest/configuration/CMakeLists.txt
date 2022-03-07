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

################################
# DDSRouter Configuration Test #
################################

set(TEST_NAME ConfigurationTest)

set(TEST_SOURCES
        ConfigurationTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/configuration/DDSRouterConfiguration.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/configuration/ParticipantConfiguration.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/participant/ParticipantId.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/participant/ParticipantKind.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/topic/FilterTopic.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/topic/RealTopic.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/topic/Topic.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/topic/WildcardTopic.cpp
    )

set(TEST_LIST
        constructor
        participants_configurations
        real_topics
        allowlist_wildcard
        blocklist_wildcard
        allowlist_and_blocklist
        constructor_fail
        participants_configurations_fail
        real_topics_fail
        allowlist_wildcard_fail
        blocklist_wildcard_fail
    )

set(TEST_EXTRA_LIBRARIES
        ddsrouter_utils
    )

add_unittest_executable(
    "${TEST_NAME}"
    "${TEST_SOURCES}"
    "${TEST_LIST}"
    "${TEST_EXTRA_LIBRARIES}")
