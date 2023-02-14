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

#############
# DdsTopic #
#############

set(TEST_NAME DdsTopicTest)

set(TEST_SOURCES
        DdsTopicTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/topic/dds/DdsTopic.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/topic/Topic.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/dds/TopicQoS.cpp
    )

set(TEST_LIST
        is_valid_dds_topic
        is_valid_dds_topic_negative
    )

set(TEST_EXTRA_LIBRARIES
        fastcdr
        fastrtps
        cpp_utils
    )

add_unittest_executable(
        "${TEST_NAME}"
        "${TEST_SOURCES}"
        "${TEST_LIST}"
        "${TEST_EXTRA_LIBRARIES}"
    )

#################
# WildcardDdsFilterTopic #
#################

set(TEST_NAME WildcardDdsFilterTopic)

set(TEST_SOURCES
        WildcardDdsFilterTopicTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/topic/dds/DdsTopic.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/topic/Topic.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/topic/filter/IFilterTopic.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/topic/filter/WildcardDdsFilterTopic.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/dds/TopicQoS.cpp
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
        fastcdr
        fastrtps
        cpp_utils

    )

add_unittest_executable(
        "${TEST_NAME}"
        "${TEST_SOURCES}"
        "${TEST_LIST}"
        "${TEST_EXTRA_LIBRARIES}"
    )
