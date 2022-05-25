# Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

set(TEST_NAME TimerTest)

set(TEST_SOURCES
        TimerTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/time/time_utils.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/time/Timer.cpp
    )

set(TEST_LIST
        elapsed
        elapsed_ms
        reset
    )

set(TEST_EXTRA_LIBRARIES
        fastcdr
        fastrtps
    )

############################
# PAUSABLE TIMER TEST
############################

add_unittest_executable(
        "${TEST_NAME}"
        "${TEST_SOURCES}"
        "${TEST_LIST}"
        "${TEST_EXTRA_LIBRARIES}"
    )

    set(TEST_NAME PausableTimerTest)

    set(TEST_SOURCES
            PausableTimerTest.cpp
            ${PROJECT_SOURCE_DIR}/src/cpp/time/time_utils.cpp
            ${PROJECT_SOURCE_DIR}/src/cpp/time/Timer.cpp
            ${PROJECT_SOURCE_DIR}/src/cpp/time/PausableTimer.cpp
        )

    set(TEST_LIST
            create
            repeat_play_pause
            play_and_pause
        )

    set(TEST_EXTRA_LIBRARIES
            fastcdr
            fastrtps
        )

    add_unittest_executable(
            "${TEST_NAME}"
            "${TEST_SOURCES}"
            "${TEST_LIST}"
            "${TEST_EXTRA_LIBRARIES}"
        )
