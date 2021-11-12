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

set(TEST_NAME AllowedTopicListTest)

set(TEST_SOURCES
        AllowedTopicListTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/dynamic/AllowedTopicList.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/exceptions/DDSRouterException.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/topic/DDSRouterTopic.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/topic/WildcardTopic.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/utils.cpp
    )

set(TEST_HEADERS
        ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/dynamic/AllowedTopicList.hpp
    )

set(TEST_LIST
        is_topic_allowed__default_constructor
        is_topic_allowed__empty_list
        is_topic_allowed__simple_blacklist
        is_topic_allowed__complex_blacklist
        is_topic_allowed__simple_whitelist
        is_topic_allowed__complex_whitelist
        is_topic_allowed__simple_whitelist_and_blacklist
        is_topic_allowed__complex_whitelist_and_blacklist
        is_topic_allowed__simple_whitelist_and_blacklist_entangled
        is_topic_allowed__complex_whitelist_and_blacklist_entangled
    )

set(TEST_EXTRA_LIBRARIES
        yaml-cpp
        $<$<BOOL:${WIN32}>:iphlpapi$<SEMICOLON>Shlwapi>
    )

add_unittest_executable(
        "${TEST_NAME}"
        "${TEST_SOURCES}"
        "${TEST_HEADERS}"
        "${TEST_LIST}"
        "${TEST_EXTRA_LIBRARIES}"
    )
