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

#####################################
# OWNER PTR TEST
#####################################

set(TEST_NAME OwnerPtrTest)

set(TEST_SOURCES
        OwnerPtrTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/exception/Exception.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/Formatter.cpp
    )

set(TEST_LIST
        owner_ptr_default_constructor
        owner_ptr_access_int
        owner_ptr_access_string
        owner_ptr_access_class
        owner_ptr_custom_deleter
        owner_ptr_reset
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

#####################################
# LESSEE PTR TEST
#####################################

set(TEST_NAME LesseePtrTest)

set(TEST_SOURCES
        LesseePtrTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/exception/Exception.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/Formatter.cpp
    )

set(TEST_LIST
        lessee_ptr_string_reset
        lessee_ptr_string_access
        lessee_ptr_string_multiple_access
        lessee_ptr_string_access_after_destroy
        lessee_ptr_string_access_lock_before_destroy
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
