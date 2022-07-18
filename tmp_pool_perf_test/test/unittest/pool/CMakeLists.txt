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

set(TEST_NAME ChangePoolPerformanceTest)

set(TEST_SOURCES
        ChangePoolPerformanceTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/FastDdsChangePool/CacheChangePool.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/RefactorChangePool/RefactorCacheChangePool.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/RouterPool/RouterCacheChangePool.cpp
    )

set(TEST_LIST
        refactor_pool
        router_pool
        fastdds_pool
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
