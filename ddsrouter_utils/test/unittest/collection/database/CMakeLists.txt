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

set(TEST_NAME StdWatchDataBaseTest)

set(TEST_SOURCES
        std_watch_data_base_tests.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/exception/Exception.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/Formatter.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/math/math.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/thread/manager/StdThreadPool.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/thread/task/OwnedTask.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/time/time_utils.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/utils.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/wait/CounterWaitHandler.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/wait/IntWaitHandler.cpp
    )

set(TEST_LIST
        basic_functionality_test
        sync_callback_functionality_test
        semisync_callback_functionality_test
        # parametrize
        # add
        # modify
        # delete
        # exist
        # get
        # addition_callback
        # modification_callback
        # deletion_callback
    )

set(TEST_EXTRA_LIBRARIES
        fastcdr
        fastrtps
    )

set(TEST_NEEDED_SOURCES
    )

add_unittest_executable(
        "${TEST_NAME}"
        "${TEST_SOURCES}"
        "${TEST_LIST}"
        "${TEST_EXTRA_LIBRARIES}"
        "${TEST_NEEDED_SOURCES}"
    )
