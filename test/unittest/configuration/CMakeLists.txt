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

set(TEST_NAME DatabrokerConfigurationTest)

set(TEST_SOURCES
        DatabrokerConfigurationTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/exceptions/DatabrokerException.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/ParticipantId.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/topic/DatabrokerTopic.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/topic/RealTopic.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/topic/WildcardTopic.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/configuration/DatabrokerConfiguration.cpp
    )

set(TEST_HEADERS
        ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/configuration/DatabrokerConfiguration.hpp
    )

set(TEST_LIST
        constructor
        participants_configurations
        real_topics
        whitelist_wildcard
        blacklist_wildcard
        participants_configurations_fail
        real_topics_fail
        whitelist_wildcard_fail
        blacklist_wildcard_fail
    )

set(TEST_EXTRA_LIBRARIES
        yaml-cpp
    )

add_unittest_executable(
    "${TEST_NAME}"
    "${TEST_SOURCES}"
    "${TEST_HEADERS}"
    "${TEST_LIST}"
    "${TEST_EXTRA_LIBRARIES}")
