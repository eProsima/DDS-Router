# Copyright 2023 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

############################
# Participant Factory Test #
############################

set(TEST_NAME ParticipantFactoryTest)

set(TEST_SOURCES
        ParticipantFactoryTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/core/ParticipantFactory.cpp
    )

set(TEST_LIST
        create_echo_participant
        create_simple_participant
        create_discovery_server_participant
        create_initial_peers_participant
        create_xml_participant
    )

set(TEST_EXTRA_LIBRARIES
        fastcdr
        fastrtps
        cpp_utils
        ddspipe_core
        ddspipe_participants
    )

add_unittest_executable(
        "${TEST_NAME}"
        "${TEST_SOURCES}"
        "${TEST_LIST}"
        "${TEST_EXTRA_LIBRARIES}"
    )
