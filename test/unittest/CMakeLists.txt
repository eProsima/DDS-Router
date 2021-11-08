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

# Create an executable for a unittest
# Arguments:
# TEST_NAME -> test name (it will add "_Test" after name)
# TEST_SOURCES -> sources for the test
# TEST_HEADERS -> headers for the test
# TEST_LIST -> test cases implemented in the Test .cpp
# Note: pass the arguments with "" in order to send them as a list. Otherwise they will not be received correctly
function(add_unittest_executable TEST_NAME TEST_SOURCES TEST_HEADERS TEST_LIST)

    message(STATUS "Adding executable test: " ${TEST_NAME})

    set(EXECUTABLE_TEST_NAME ${TEST_NAME}_Test)

    add_executable(${EXECUTABLE_TEST_NAME}
        ${TEST_SOURCES}
        ${TEST_HEADERS})

    if(MSVC)
    target_compile_definitions(${EXECUTABLE_TEST_NAME} PRIVATE _CRT_DECLARE_NONSTDC_NAMES=0)
    endif(MSVC)

    target_include_directories(${EXECUTABLE_TEST_NAME} PRIVATE
    ${GTEST_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
    ${PROJECT_BINARY_DIR}/include
    ${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}
    ${PROJECT_SOURCE_DIR}/src/cpp)

    target_link_libraries( ${EXECUTABLE_TEST_NAME} PUBLIC
            ${GTEST_LIBRARIES}
            $<$<BOOL:${WIN32}>:iphlpapi$<SEMICOLON>Shlwapi> # TODO remove this when new parameter is added to function
        )

    get_win32_path_dependencies(${EXECUTABLE_TEST_NAME} TEST_FRIENDLY_PATH)

    foreach(test_name ${TEST_LIST})
    add_test(NAME ${TEST_NAME}.${test_name}
            COMMAND ${EXECUTABLE_TEST_NAME}
            --gtest_filter=${TEST_NAME}.${test_name}:**/${TEST_NAME}.${test_name}/**)

    if(TEST_FRIENDLY_PATH)
        set_tests_properties(${TEST_NAME}.${test_name} PROPERTIES ENVIRONMENT "PATH=${TEST_FRIENDLY_PATH}")
    endif(TEST_FRIENDLY_PATH)
    endforeach()

endfunction(add_unittest_executable)

add_subdirectory(types)

