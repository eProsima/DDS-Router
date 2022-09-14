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
# STD MACROS TEST
#####################################

set(TEST_NAME macrosTest)

set(TEST_SOURCES
        macrosTest.cpp
    )

set(TEST_LIST
        stringify
    )

set(TEST_EXTRA_LIBRARIES
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

#####################################
# RECURSIVE MACROS TEST
#####################################

set(TEST_NAME recursiveMacrosTest)

set(TEST_SOURCES
        recursiveMacrosTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/Formatter.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/exception/Exception.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/utils.cpp
    )

set(TEST_LIST
        count_arguments
        apply_macro_for_each
    )

set(TEST_EXTRA_LIBRARIES
        fastcdr
        fastrtps
        $<$<BOOL:${WIN32}>:iphlpapi$<SEMICOLON>Shlwapi>
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

#####################################
# ENUMERATION BUILDER TEST
#####################################

set(TEST_NAME enumerationBuilderMacrosTest)

set(TEST_SOURCES
        enumerationBuilderMacrosTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/Formatter.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/exception/Exception.cpp
    )

set(TEST_LIST
        show_how
        from_string
        to_string
        serializator
        n_values
    )

set(TEST_EXTRA_LIBRARIES
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
