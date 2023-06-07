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

###########################################################################
# Create tests                                                            #
###########################################################################

find_package(PythonInterp 3 REQUIRED)

# Name of files to test
set(TEST_CONFIGURATIONS
    simple_configuration
    ds_configuration
    complex_configuration_v3
    wan_configuration
)

# Whether one or other signal in test
set(SIGNAL_TEST_ARGS
    "sigint"
    "sigterm"
)

foreach(TEST_CONFIGURATION ${TEST_CONFIGURATIONS})
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/configurations/${TEST_CONFIGURATION}.yaml
        ${CMAKE_CURRENT_BINARY_DIR}/configurations/${TEST_CONFIGURATION}.yaml
        COPYONLY)
endforeach()

# windows auxiliary script to fork test execution
set(PWS_LAUNCHER
    ${CMAKE_CURRENT_SOURCE_DIR}/launcher.ps1
)

# Calculate environment
set(TEST_ENVIRONMENT
    "PATH=$ENV{PATH};$<TARGET_FILE_DIR:${PROJECT_NAME}>")

foreach(PROJECT_DEPENDENCY ${MODULE_DEPENDENCIES})

    message(STATUS "Finding target for libraries for ${PROJECT_DEPENDENCY}")

    if(TARGET ${PROJECT_DEPENDENCY} OR TARGET eprosima::${PROJECT_DEPENDENCY} OR TARGET ${PROJECT_DEPENDENCY}::${PROJECT_DEPENDENCY})
        set(TEST_ENVIRONMENT
            "${TEST_ENVIRONMENT};$<TARGET_FILE_DIR:${PROJECT_DEPENDENCY}>")

    elseif(EXISTS ${PROJECT_DEPENDENCY}_LIBRARY)
        get_filename_component(
            ${PROJECT_DEPENDENCY}_LIBRARY_DIR ${${PROJECT_DEPENDENCY}_LIBRARY} DIRECTORY)
        set(TEST_ENVIRONMENT
            "${TEST_ENVIRONMENT};${${PROJECT_DEPENDENCY}_LIBRARY_DIR}")
        unset(${PROJECT_DEPENDENCY}_LIBRARY_DIR)

    else()

        message(STATUS "${PROJECT_DEPENDENCY} could not be added to TEST_ENVIRONMENT")

    endif()

endforeach()

if(WIN32)

    if(TARGET tinyxml2 OR TARGET tinyxml2::tinyxml2)
        set(TEST_ENVIRONMENT
            "${TEST_ENVIRONMENT};$<TARGET_FILE_DIR:${TINYXML2_LIBRARY}>")
    elseif(EXISTS TINYXML2_LIBRARY)
        get_filename_component(
            TINYXML2_LIBRARY_DIR ${TINYXML2_LIBRARY} DIRECTORY)
        set(TEST_ENVIRONMENT
            "${TEST_ENVIRONMENT};${TINYXML2_LIBRARY_DIR}")
        unset(TINYXML2_LIBRARY_DIR)
    endif()

    if(TARGET yamlcpp OR TARGET yamlcpp::yamlcpp)
        set(TEST_ENVIRONMENT
            "${TEST_ENVIRONMENT};$<TARGET_FILE_DIR:${YAMLCPP_LIBRARY}>")
    elseif(EXISTS YAMLCPP_LIBRARY)
        get_filename_component(
            YAMLCPP_LIBRARY_DIR ${YAMLCPP_LIBRARY} DIRECTORY)
        set(TEST_ENVIRONMENT
            "${TEST_ENVIRONMENT};${YAMLCPP_LIBRARY_DIR}")
        unset(YAMLCPP_LIBRARY_DIR)
    endif()

    string(REPLACE ";" "\\;" TEST_ENVIRONMENT "${TEST_ENVIRONMENT}")

endif(WIN32)

# populate the tests
foreach(TEST IN LISTS TEST_CONFIGURATIONS)

        set(TEST_NAME "tool.application.ddsrouter.${TEST}")

        # CTest has issues with signals on windows. We forked the test
        # execution using an auxiliary powershell script and using only sigint
        if(WIN32)
            add_test(
                    NAME ${TEST_NAME}
                    COMMAND powershell "-File" ${PWS_LAUNCHER}
                            ${PYTHON_EXECUTABLE}
                            ${CMAKE_CURRENT_SOURCE_DIR}/tests.py
                            $<TARGET_FILE:ddsrouter_tool>
                            ${CMAKE_CURRENT_BINARY_DIR}/configurations/${TEST}.yaml
                )

            # Set test properties
            set_tests_properties(
                ${TEST_NAME}
                PROPERTIES
                    ENVIRONMENT "${TEST_ENVIRONMENT}"
                )

        else()

            foreach(SIGNAL_ARG IN LISTS SIGNAL_TEST_ARGS)

                set(TEST_NAME "${TEST_NAME}.${SIGNAL_ARG}")
                add_test(
                        NAME ${TEST_NAME}
                        COMMAND ${PYTHON_EXECUTABLE}
                                ${CMAKE_CURRENT_SOURCE_DIR}/tests.py
                                "--exe" $<TARGET_FILE:ddsrouter_tool>
                                "--config-file" ${CMAKE_CURRENT_BINARY_DIR}/configurations/${TEST}.yaml
                                "--debug"
                                "--signal" ${SIGNAL_ARG}
                    )

                # Set test properties
                set_tests_properties(
                    ${TEST_NAME}
                    PROPERTIES
                        ENVIRONMENT "${TEST_ENVIRONMENT}"
                    )
                endforeach()
        endif()

endforeach()

unset(TEST_ENVIRONMENT)
