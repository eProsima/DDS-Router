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

#############################################
# BOOLEAN WAIT HANDLER TEST
#############################################

set(TEST_NAME BooleanWaitHandlerTest)

set(TEST_SOURCES
        BooleanWaitHandlerTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/wait/BooleanWaitHandler.cpp
    )

set(TEST_LIST
        create_wait_handler
        open_close
        wait_for_open
        wait_for_timeout
        wait_for_disable
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

#############################################
# COUNTER WAIT HANDLER TEST
#############################################

set(TEST_NAME CounterWaitHandlerTest)

set(TEST_SOURCES
        CounterWaitHandlerTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/wait/CounterWaitHandler.cpp
    )

set(TEST_LIST
        value_handler
        wait_value
        wait_upper
        wait_lower
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

#############################################
# DOUBLE QUEUE WAIT HANDLER TEST
#############################################

set(TEST_NAME DBQueueWaitHandlerTest)

set(TEST_SOURCES
        DBQueueWaitHandlerTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/wait/CounterWaitHandler.cpp
    )

set(TEST_LIST
        push_pop_one_thread_int
        push_pop_one_thread_string_move # not working
        push_pop_one_thread_string_copy
        push_one_thread_pop_many_int
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
