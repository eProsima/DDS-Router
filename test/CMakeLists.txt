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

include_directories("TestUtils")

include(${PROJECT_SOURCE_DIR}/cmake/common/gtest.cmake)
check_gtest()
check_gmock()

if(WIN32)

    # populates out_var with the value that the PATH environment variable should have
    # in order to cover all target dependencies required for a ctest.
    # Arguments:
    # GOAL_TARGET -> dependencies related target
    # OUTPUT_VARIABLE -> name of the output variable (in parent scope) that receives the new path
    # a third argument can be specify to simplify recursive operation. If it's value is
    # "EXTRA_PATHS" the current PATH will not be included in the output.
    function(get_win32_path_dependencies GOAL_TARGET OUTPUT_VARIABLE)
        unset(LOCAL_PATH)

        get_target_property(LINK_LIBRARIES_ ${GOAL_TARGET} LINK_LIBRARIES)
        if(NOT "${LINK_LIBRARIES_}" STREQUAL "LINK_LIBRARIES_-NOTFOUND")
            foreach(LIBRARY_LINKED ${LINK_LIBRARIES_})
                if(TARGET ${LIBRARY_LINKED})
                    # Check if is a real target or a target interface
                    get_target_property(type ${LIBRARY_LINKED} TYPE)
                    if(NOT type STREQUAL "INTERFACE_LIBRARY")

                        if(LOCAL_PATH)
                            set(LOCAL_PATH "$<TARGET_FILE_DIR:${LIBRARY_LINKED}>;${LOCAL_PATH}")
                        else(LOCAL_PATH)
                            set(LOCAL_PATH "$<TARGET_FILE_DIR:${LIBRARY_LINKED}>")
                        endif(LOCAL_PATH)

                        # Check for dependencies recursively
                        get_win32_path_dependencies(${LIBRARY_LINKED} DEP_WIN_PATH "EXTRA_PATHS")
                        if(DEP_WIN_PATH)
                            set(LOCAL_PATH "${DEP_WIN_PATH};${LOCAL_PATH}")
                        endif(DEP_WIN_PATH)
                        unset(DEP_WIN_PATH)
                    endif()
                    unset(type)
                endif()
            endforeach()
        endif()

        if( (ARGC EQUAL 2) OR NOT (ARGV2 STREQUAL "EXTRA_PATHS"))
            set(LOCAL_PATH "${LOCAL_PATH};$ENV{PATH}")
            list(REMOVE_DUPLICATES LOCAL_PATH)
            string(REPLACE ";" "\\;" LOCAL_PATH "${LOCAL_PATH}")
        endif()

        set(${OUTPUT_VARIABLE} "${LOCAL_PATH}" PARENT_SCOPE)

    endfunction(get_win32_path_dependencies)

else(WIN32)
    # dummy
    function(get_win32_path_dependencies)
    endfunction(get_win32_path_dependencies)
endif(WIN32)

# Create an executable for a test
# Arguments:
# TEST_EXECUTABLE_NAME -> name of the test executable
# TEST_SOURCES -> sources for the test
# TEST_NAME -> test name (name of the class of the test in Test .cpp)
# TEST_LIST -> test cases implemented in the Test .cpp
# TEST_EXTRA_LIBRARIES -> libraries that must be linked to compile the test
# ARGV5 -> extra headers needed for the test (sixth optional argument)
# Note: pass the arguments with "" in order to send them as a list. Otherwise they will not be received correctly
function(add_test_executable TEST_EXECUTABLE_NAME TEST_SOURCES TEST_NAME TEST_LIST TEST_EXTRA_LIBRARIES)

    message(STATUS "Adding executable test: " ${TEST_EXECUTABLE_NAME})

    add_executable(${TEST_EXECUTABLE_NAME}
        ${TEST_SOURCES}
    )

    if(MSVC)
        target_compile_definitions(${TEST_EXECUTABLE_NAME} PRIVATE _CRT_DECLARE_NONSTDC_NAMES=0)
    endif(MSVC)

    target_include_directories(${TEST_EXECUTABLE_NAME} PRIVATE
        ${GTEST_INCLUDE_DIRS}
        ${GMOCK_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
        ${PROJECT_BINARY_DIR}/include
        ${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}
        ${ARGV5}  # TEST_EXTRA_HEADERS (EQUAL "" if not provided)
    )

    target_link_libraries(
        ${TEST_EXECUTABLE_NAME} PUBLIC
            ${GTEST_LIBRARIES}
            ${GMOCK_LIBRARIES}
            ${TEST_EXTRA_LIBRARIES})

    get_win32_path_dependencies(${TEST_EXECUTABLE_NAME} TEST_FRIENDLY_PATH)

    foreach(test_name ${TEST_LIST})
        add_test(NAME ${TEST_NAME}.${test_name}
                COMMAND ${TEST_EXECUTABLE_NAME}
                --gtest_filter=${TEST_NAME}.${test_name}:**/${TEST_NAME}.${test_name}/**)

        if(TEST_FRIENDLY_PATH)
            set_tests_properties(${TEST_NAME}.${test_name} PROPERTIES ENVIRONMENT "PATH=${TEST_FRIENDLY_PATH}")
        endif(TEST_FRIENDLY_PATH)
    endforeach()

    target_compile_definitions(${TEST_EXECUTABLE_NAME}
        PRIVATE FASTDDS_ENFORCE_LOG_INFO
        PRIVATE HAVE_LOG_NO_INFO=0)

endfunction(add_test_executable)

# Return a list of all cpp files required in order to compile test_utils.cpp
# Arguments:
# TEST_SOURCES -> List of source files where new sources will be added
function(sources_for_test_utils TEST_SOURCES)

    set(TEST_UTILS_SOURCES
        ${PROJECT_SOURCE_DIR}/src/cpp/configuration/participant/ParticipantConfiguration.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/configuration/participant/SimpleParticipantConfiguration.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/configuration/participant/DiscoveryServerParticipantConfiguration.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/event/LogEventHandler.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/event/LogSevereEventHandler.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/exceptions/Exception.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/security/tls/TlsConfiguration.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/security/tls/TlsConfigurationBoth.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/security/tls/TlsConfigurationClient.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/security/tls/TlsConfigurationServer.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/Data.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/address/Address.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/address/DiscoveryServerConnectionAddress.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/endpoint/DomainId.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/endpoint/Endpoint.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/endpoint/Guid.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/endpoint/GuidPrefix.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/endpoint/QoS.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/participant/ParticipantId.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/participant/ParticipantKind.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/topic/RealTopic.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/topic/WildcardTopic.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/topic/FilterTopic.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/topic/Topic.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/utils.cpp
        ${PROJECT_SOURCE_DIR}/test/TestUtils/test_utils.cpp
        ${PROJECT_SOURCE_DIR}/test/TestUtils/TestLogHandler.cpp
    )

    set(NEW_TEST_SOURCES "${TEST_SOURCES};${TEST_UTILS_SOURCES}")

    set(TEST_SOURCES ${NEW_TEST_SOURCES} PARENT_SCOPE)

endfunction()

# Return a list of all cpp files required by ddsrouter library
# Arguments:
# TEST_SOURCES -> List of source files where new sources will be added
function(all_library_sources TEST_SOURCES)

    file(GLOB_RECURSE LIBRARY_SOURCES
            "${PROJECT_SOURCE_DIR}/src/cpp/**/*.c"
            "${PROJECT_SOURCE_DIR}/src/cpp/**/*.cpp"
            "${PROJECT_SOURCE_DIR}/src/cpp/**/*.cxx"
            "${PROJECT_SOURCE_DIR}/test/TestUtils/test_utils.cpp"
            "${PROJECT_SOURCE_DIR}/test/TestUtils/TestLogHandler.cpp"
        )
    set(NEW_TEST_SOURCES "${TEST_SOURCES};${LIBRARY_SOURCES}")
    set(TEST_SOURCES ${NEW_TEST_SOURCES} PARENT_SCOPE)

endfunction()

add_subdirectory(blackbox)
add_subdirectory(unittest)

# If tool active, compile tool tests
if (COMPILE_TOOLS)
    add_subdirectory(tool)
endif()
