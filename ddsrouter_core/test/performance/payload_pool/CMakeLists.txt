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

####################################
# Payload Pool Performance Test
####################################

set(TEST_NAME
    PayloadPoolPerformanceTest)

set(TEST_SOURCES
    payload_pool_performance_test.cpp)

set(TEST_LIST
    copy_payload_pool
    map_payload_pool
    fast_payload_pool)

set(TEST_NEEDED_SOURCES
    )

set(TEST_EXTRA_HEADERS
    )

add_blackbox_executable(
    "${TEST_NAME}"
    "${TEST_SOURCES}"
    "${TEST_LIST}"
    "${TEST_NEEDED_SOURCES}"
    "${TEST_EXTRA_HEADERS}")
