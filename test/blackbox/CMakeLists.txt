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

# Create an executable for a blackbox
# Arguments:
# TEST_NAME -> test name (it will add "_Test" after name)
# TEST_SOURCES -> sources for the test
# TEST_LIST -> test cases implemented in the Test .cpp
# ARGV4 -> extra headers needed for the test (fifth optional argument)
# ARGV5 -> flaky test list file (sixth optional argument)
# Note: pass the arguments with "" in order to send them as a list. Otherwise they will not be received correctly
function(add_blackbox_executable TEST_NAME TEST_SOURCES TEST_LIST TEST_NEEDED_SOURCES)

    # All directories in project with library files
    set(LIBRARY_DIRECTORIES
        communication
        configuration
        configuration/implementations
        core
        dynamic
        event
        exceptions
        participant
        participant/implementations/auxiliar
        participant/implementations/rtps
        reader/implementations/auxiliar
        reader/implementations/rtps
        types
        types/address
        types/endpoint
        types/participant
        types/topic
        writer/implementations/auxiliar
        writer/implementations/rtps
    )

    # Get all sources of the library except implementations and main
    foreach(SOURCE_DIRECTORY ${LIBRARY_DIRECTORIES})

        unset(SOURCES_FILES)
        file(GLOB_RECURSE SOURCES_FILES
            "${PROJECT_SOURCE_DIR}/src/cpp/${SOURCE_DIRECTORY}/*.c"
            "${PROJECT_SOURCE_DIR}/src/cpp/${SOURCE_DIRECTORY}/*.cpp"
            "${PROJECT_SOURCE_DIR}/src/cpp/${SOURCE_DIRECTORY}/*.cxx"
            )
        list(APPEND TEST_SOURCES ${SOURCES_FILES})

    endforeach()

    # Source every cpp in Test utils

    unset(SOURCES_FILES)
    file(GLOB_RECURSE SOURCES_FILES
        "${PROJECT_SOURCE_DIR}/test/TestUtils/*.c"
        "${PROJECT_SOURCE_DIR}/test/TestUtils/*.cpp"
        "${PROJECT_SOURCE_DIR}/test/TestUtils/*.cxx"
        )
    list(APPEND TEST_SOURCES ${SOURCES_FILES})

    # Add all library needed by sources
    set(EXTRA_LIBRARIES
        yaml-cpp
        fastcdr
        fastrtps
        $<$<BOOL:${WIN32}>:iphlpapi$<SEMICOLON>Shlwapi>)

    # Store file sources needed
    foreach(NEEDED_SOURCE ${TEST_NEEDED_SOURCES})
        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${NEEDED_SOURCE}
            ${CMAKE_CURRENT_BINARY_DIR}/${NEEDED_SOURCE}
            COPYONLY)
    endforeach()

    # Create test executable
    add_test_executable(
        "blackbox_${TEST_NAME}"
        "${TEST_SOURCES}"
        "${TEST_NAME}"
        "${TEST_LIST}"
        "${EXTRA_LIBRARIES}"
        "${ARGV4}"  # TEST_EXTRA_HEADERS (EQUAL "" if not provided)
    )

    # add xfails to flaky tests in XFAIL list files
    foreach(FLAKY_TEST ${ARGV5})
        add_xfail_label(${FLAKY_TEST})
    endforeach()

endfunction(add_blackbox_executable)

add_subdirectory(ddsrouter_core)
add_subdirectory(implementations)
