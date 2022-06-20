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

set(TEST_NAME utilsTest)

set(TEST_SOURCES
        utilsTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/exception/Exception.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/Formatter.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/math/math.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/utils.cpp
    )

set(TEST_LIST
        set_of_ptr_contains_int
        set_of_ptr_with_null_contains_int
        set_of_ptr_contains_string
        are_set_of_ptr_equal_int
        to_lowercase
        tsnh_call
        is_file_accessible
        combined_file_permissions
    )

set(TEST_EXTRA_LIBRARIES
        fastcdr
        fastrtps
        $<$<BOOL:${WIN32}>:iphlpapi$<SEMICOLON>Shlwapi>
    )

set(TEST_NEEDED_SOURCES
        resources/exist.test
    )

add_unittest_executable(
        "${TEST_NAME}"
        "${TEST_SOURCES}"
        "${TEST_LIST}"
        "${TEST_EXTRA_LIBRARIES}"
        "${TEST_NEEDED_SOURCES}"
    )
