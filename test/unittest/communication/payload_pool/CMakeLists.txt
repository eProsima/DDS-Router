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

##############################
# Interface PayloadPool Test #
##############################

set(TEST_NAME PayloadPoolTest)

set(TEST_SOURCES
        PayloadPoolTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/communication/payload_pool/PayloadPool.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/communication/payload_pool/CopyPayloadPool.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/communication/payload_pool/MapPayloadPool.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/exceptions/Exception.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/utils.cpp
    )

set(TEST_LIST
        reserve
        release
        reserve_and_release_counter
        get_payload_negative
        get_payload_from_src_negative
        release_payload_negative
    )

set(TEST_EXTRA_LIBRARIES
        fastcdr
        fastrtps
        yaml-cpp
    )

add_unittest_executable(
        "${TEST_NAME}"
        "${TEST_SOURCES}"
        "${TEST_LIST}"
        "${TEST_EXTRA_LIBRARIES}"
    )
