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
# IManager Specializations Test
###################################

set(TEST_NAME
    ParametrizedThreadManagerTest)

set(TEST_SOURCES
        manager_interface_test.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/exception/Exception.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/Formatter.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/math/math.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/thread/task/OwnedTask.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/thread/manager/SyncManager.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/thread/manager/AsyncManager.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/thread/manager/StdThreadPool.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/time/time_utils.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/time/Timer.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/utils.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/wait/CounterWaitHandler.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/wait/IntWaitHandler.cpp
    )

set(TEST_LIST
        manager_execute
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

###################################
# StdThreadPool Test
###################################

set(TEST_NAME
    StdThreadPoolTest)

set(TEST_SOURCES
        std_thread_pool_test.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/exception/Exception.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/Formatter.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/math/math.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/thread/task/OwnedTask.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/thread/manager/StdThreadPool.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/time/time_utils.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/time/Timer.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/utils.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/wait/CounterWaitHandler.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/wait/IntWaitHandler.cpp
    )

set(TEST_LIST
        pool_1_threads_1_tasks
        pool_1_threads_M_tasks
        pool_N_threads_N_tasks
        pool_N_threads_NM_tasks
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
