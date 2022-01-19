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

########################
# Implementations Test #
########################

set(TEST_NAME
    ImplementationsTest)

set(TEST_SOURCES
        ImplementationsTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/event/LogEventHandler.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/event/LogSevereEventHandler.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/participant/implementations/auxiliar/DummyParticipant.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/participant/implementations/auxiliar/VoidParticipant.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/participant/implementations/rtps/SimpleParticipant.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/participant/implementations/rtps/LocalDiscoveryServerParticipant.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/participant/implementations/rtps/WANParticipant.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/reader/implementations/auxiliar/DummyReader.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/reader/implementations/auxiliar/VoidReader.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/reader/implementations/rtps/Reader.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/writer/implementations/auxiliar/DummyWriter.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/writer/implementations/auxiliar/VoidWriter.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/writer/implementations/rtps/Writer.cpp
        ${PROJECT_SOURCE_DIR}/test/TestUtils/TestLogHandler.cpp
    )

set(TEST_LIST
        pair_implementation
        pair_implementation_with_topic
        all_implementations
        solo_participant_implementation
    )

set(TEST_NEEDED_SOURCES
    )

add_blackbox_executable(
        "${TEST_NAME}"
        "${TEST_SOURCES}"
        "${TEST_LIST}"
        "${TEST_NEEDED_SOURCES}"
    )
