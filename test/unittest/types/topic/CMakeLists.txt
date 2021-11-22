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

###################
# Topic #
###################

set(TEST_NAME TopicTest)

set(TEST_SOURCES
        TopicTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/topic/Topic.cpp
    )

set(TEST_HEADERS
        ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/types/topic/Topic.hpp
    )

set(TEST_LIST
        constructor
        equal_operator
        minor_operator
        non_equal_operator
        non_minor_operator
        stdout_operator
    )

set(TEST_EXTRA_LIBRARIES
    )

add_unittest_executable(
        "${TEST_NAME}"
        "${TEST_SOURCES}"
        "${TEST_HEADERS}"
        "${TEST_LIST}"
        "${TEST_EXTRA_LIBRARIES}"
    )

#############
# RealTopic #
#############

set(TEST_NAME RealTopicTest)

set(TEST_SOURCES
        RealTopicTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/topic/RealTopic.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/topic/Topic.cpp
    )

set(TEST_HEADERS
        ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/types/topic/RealTopic.hpp
    )

set(TEST_LIST
        is_real_topic
        is_non_real_topic
    )

set(TEST_EXTRA_LIBRARIES
    )

add_unittest_executable(
        "${TEST_NAME}"
        "${TEST_SOURCES}"
        "${TEST_HEADERS}"
        "${TEST_LIST}"
        "${TEST_EXTRA_LIBRARIES}"
    )

#################
# WildcardTopic #
#################

set(TEST_NAME WildcardTopicTest)

set(TEST_SOURCES
        WildcardTopicTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/topic/RealTopic.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/topic/Topic.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/topic/WildcardTopic.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/utils.cpp
    )

set(TEST_HEADERS
        ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/types/topic/WildcardTopic.hpp
    )

set(TEST_LIST
        matches
        non_matches

        # Not supported (TODO)
        # non_contains_wildcard
        # contains_wildcard
    )

set(TEST_EXTRA_LIBRARIES
        $<$<BOOL:${WIN32}>:iphlpapi$<SEMICOLON>Shlwapi>
    )

add_unittest_executable(
        "${TEST_NAME}"
        "${TEST_SOURCES}"
        "${TEST_HEADERS}"
        "${TEST_LIST}"
        "${TEST_EXTRA_LIBRARIES}"
    )
