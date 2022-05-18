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

#################
# ParticipantId #
#################

set(TEST_NAME ParticipantIdTest)

set(TEST_SOURCES
        ParticipantIdTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/participant/ParticipantId.cpp
    )

set(TEST_LIST
        default_constructor
        constructor
        invalid
        is_valid_id
        is_valid
        equal_operator
        minor_operator
        is_non_valid_id
        is_non_valid
        non_equal_operator
        non_minor_operator
    )

set(TEST_EXTRA_LIBRARIES
        ddsrouter_utils
    )

add_unittest_executable(
        "${TEST_NAME}"
        "${TEST_SOURCES}"
        "${TEST_LIST}"
        "${TEST_EXTRA_LIBRARIES}"
    )


###################
# ParticipantKind #
###################

set(TEST_NAME ParticipantKindTest)

set(TEST_SOURCES
        ParticipantKindTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/participant/ParticipantKind.cpp
    )

set(TEST_LIST
        int_conversions
        string_conversions
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
