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

# Create an executable for a unit test
#
# ARGUMENTS:
# TEST_EXECUTABLE_NAME -> name of the test executable
# TEST_SOURCES -> sources for the test
# TEST_NAME -> test name (name of the class of the test in Test .cpp)
# TEST_LIST -> test cases implemented in the Test .cpp
# TEST_EXTRA_LIBRARIES -> libraries that must be linked to compile the test
# TEST_NEEDED_SOURCES -> source files required to be copies for the test execution
#
# NOTE:
# pass the arguments with "" in order to send them as a list. Otherwise they will not be received correctly
function(add_test_executable TEST_EXECUTABLE_NAME TEST_SOURCES TEST_NAME TEST_LIST TEST_EXTRA_LIBRARIES TEST_NEEDED_SOURCES)

    message(STATUS "Adding executable test: " ${TEST_EXECUTABLE_NAME})

    add_executable(${TEST_EXECUTABLE_NAME}
        ${TEST_SOURCES}
    )

    if(MSVC)
        target_compile_definitions(${TEST_EXECUTABLE_NAME}
            PRIVATE
                _CRT_DECLARE_NONSTDC_NAMES=0
                ${MODULE_MACRO}_SOURCE)
    endif(MSVC)

    target_include_directories(${TEST_EXECUTABLE_NAME} PRIVATE
        ${GTEST_INCLUDE_DIRS}
        ${GMOCK_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
        ${PROJECT_BINARY_DIR}/include
        ${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}
        ${PROJECT_SOURCE_DIR}/src/cpp
        ${ARGV6}  # TEST_EXTRA_HEADERS (EQUAL "" if not provided)
    )

    target_link_libraries(
        ${TEST_EXECUTABLE_NAME} PUBLIC
            ${GTEST_LIBRARIES}
            ${GMOCK_LIBRARIES}
            ${TEST_EXTRA_LIBRARIES})

    get_win32_path_dependencies(${TEST_EXECUTABLE_NAME} TEST_FRIENDLY_PATH)

    if( TEST_LIST )
        # If list of tests is not empty, add each test separatly
        foreach(test_name ${TEST_LIST})
            add_test(NAME ${TEST_NAME}.${test_name}
                    COMMAND ${TEST_EXECUTABLE_NAME}
                    --gtest_filter=${TEST_NAME}**.${test_name}:**/${TEST_NAME}**.${test_name}/**)

            if(TEST_FRIENDLY_PATH)
                set_tests_properties(${TEST_NAME}.${test_name} PROPERTIES ENVIRONMENT "PATH=${TEST_FRIENDLY_PATH}")
            endif(TEST_FRIENDLY_PATH)
        endforeach()
    else()
        # If no tests are provided, create a single test
        message(STATUS "Creating general test ${TEST_NAME}.")
        add_test(NAME ${TEST_NAME}
            COMMAND ${TEST_EXECUTABLE_NAME})
    endif( TEST_LIST )


    target_compile_definitions(${TEST_EXECUTABLE_NAME}
        PRIVATE FASTDDS_ENFORCE_LOG_INFO
        PRIVATE HAVE_LOG_NO_INFO=0)

    # Store file sources needed
    foreach(NEEDED_SOURCE ${TEST_NEEDED_SOURCES})
        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${NEEDED_SOURCE}
            ${CMAKE_CURRENT_BINARY_DIR}/${NEEDED_SOURCE}
            COPYONLY)
    endforeach()

endfunction(add_test_executable)

# Create an executable for a unittest
#
# ARGUMENTS:
# TEST_NAME -> test name (name of the class of the test in Test .cpp)
# TEST_SOURCES -> sources for the test
# TEST_LIST -> test cases implemented in the Test .cpp
# TEST_EXTRA_LIBRARIES -> libraries that must be linked to compile the test
function(add_unittest_executable TEST_NAME TEST_SOURCES TEST_LIST TEST_EXTRA_LIBRARIES)

    add_test_executable(
        "unittest_${TEST_NAME}"
        "${TEST_SOURCES}"
        "${TEST_NAME}"
        "${TEST_LIST}"
        "${TEST_EXTRA_LIBRARIES}"
        "${ARGV4}"                  # TEST_NEEDED_SOURCES (EQUAL "" if not provided)
    )

endfunction(add_unittest_executable)

# Create an executable for a blackbox
#
# ARGUMENTS:
# TEST_NAME -> test name (it will add "_Test" after name)
# TEST_SOURCES -> sources for the test
# TEST_LIST -> test cases implemented in the Test .cpp
# ARGV4 -> extra headers needed for the test (fifth optional argument)
function(add_blackbox_executable TEST_NAME TEST_SOURCES TEST_LIST TEST_NEEDED_SOURCES)

    # Add all cpp files to sources
    all_library_sources("${TEST_SOURCES}")

    # Add all library needed by sources
    set(EXTRA_LIBRARIES
        ${MODULE_DEPENDENCIES})

    # Create test executable
    add_test_executable(
        "blackbox_${TEST_NAME}"
        "${TEST_SOURCES}"
        "${TEST_NAME}"
        "${TEST_LIST}"
        "${EXTRA_LIBRARIES}"
        "${TEST_NEEDED_SOURCES}"
        "${ARGV4}"  # TEST_EXTRA_HEADERS (EQUAL "" if not provided)
    )

endfunction(add_blackbox_executable)
