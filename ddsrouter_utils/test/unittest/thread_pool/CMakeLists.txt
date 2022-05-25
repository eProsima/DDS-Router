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

###################################
# Slot Thread Pool execution Test
###################################

set(TEST_NAME
    slot_thread_pool_test)

set(TEST_SOURCES
        slot_thread_pool_test.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/thread_pool/pool/SlotThreadPool.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/thread_pool/task/TaskId.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/time/Timer.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/time/time_utils.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/wait/IntWaitHandler.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/wait/CounterWaitHandler.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/utils.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/format/Formatter.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/exception/Exception.cpp
    )

set(TEST_LIST
        pool_one_thread_one_slot
        pool_one_thread_n_slots
        pool_n_threads_one_slot
    )

set(TEST_EXTRA_LIBRARIES
        ${MODULE_DEPENDENCIES}
    )

add_unittest_executable(
        "${TEST_NAME}"
        "${TEST_SOURCES}"
        "${TEST_LIST}"
        "${TEST_EXTRA_LIBRARIES}"
    )
