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

#######
# QoS #
#######

set(TEST_NAME QoSTest)

set(TEST_SOURCES
        QoSTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/endpoint/QoS.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/utils.cpp
    )

set(TEST_LIST
        constructor
        durability_getter
        reliability_getter
    )

set(TEST_EXTRA_LIBRARIES
        fastcdr
        fastrtps
        ddsrouter_utils
    )

add_unittest_executable(
        "${TEST_NAME}"
        "${TEST_SOURCES}"
        "${TEST_LIST}"
        "${TEST_EXTRA_LIBRARIES}"
    )

########
# GUID #
########

set(TEST_NAME GuidTest)

set(TEST_SOURCES
        GuidTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/dds/Guid.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/dds/GuidPrefix.cpp
    )

set(TEST_LIST
        is_valid
    )

set(TEST_EXTRA_LIBRARIES
        fastcdr
        fastrtps
        ddsrouter_utils
    )

add_unittest_executable(
        "${TEST_NAME}"
        "${TEST_SOURCES}"
        "${TEST_LIST}"
        "${TEST_EXTRA_LIBRARIES}"
    )

############
# Endpoint #
############

set(TEST_NAME EndpointTest)

set(TEST_SOURCES
        EndpointTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/endpoint/Endpoint.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/dds/Guid.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/dds/GuidPrefix.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/endpoint/QoS.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/topic/RealTopic.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/topic/Topic.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/utils.cpp
    )

set(TEST_LIST
        constructor
        kind_getter
        guid_getter
        qos_getter
        topic_getter
        active_getter
        active_setter
        is_writer
        is_reader
    )

set(TEST_EXTRA_LIBRARIES
        fastcdr
        fastrtps
        ddsrouter_utils
    )

add_unittest_executable(
        "${TEST_NAME}"
        "${TEST_SOURCES}"
        "${TEST_LIST}"
        "${TEST_EXTRA_LIBRARIES}"
    )
