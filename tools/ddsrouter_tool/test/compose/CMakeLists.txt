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

###########################################################################
# Create tests                                                            #
###########################################################################


# Name of files to test
set(TESTS
    repeater
)

foreach(TEST ${TESTS})
    file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/${TEST} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/${TEST})
    file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/docker-compose.sh DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/docker-compose.sh)
endforeach()

# # Calculate environment
# set(TEST_ENVIRONMENT
#     "PATH=$ENV{PATH};$<TARGET_FILE_DIR:${PROJECT_NAME}>")

# foreach(PROJECT_DEPENDENCY ${MODULE_DEPENDENCIES})

#     message(STATUS "Finding target for libraries for ${PROJECT_DEPENDENCY}")

#     if(TARGET ${PROJECT_DEPENDENCY} OR TARGET eprosima::${PROJECT_DEPENDENCY} OR TARGET ${PROJECT_DEPENDENCY}::${PROJECT_DEPENDENCY})
#         set(TEST_ENVIRONMENT
#             "${TEST_ENVIRONMENT};$<TARGET_FILE_DIR:${PROJECT_DEPENDENCY}>")

#     elseif(EXISTS ${PROJECT_DEPENDENCY}_LIBRARY)
#         get_filename_component(
#             ${PROJECT_DEPENDENCY}_LIBRARY_DIR ${${PROJECT_DEPENDENCY}_LIBRARY} DIRECTORY)
#         set(TEST_ENVIRONMENT
#             "${TEST_ENVIRONMENT};${${PROJECT_DEPENDENCY}_LIBRARY_DIR}")
#         unset(${PROJECT_DEPENDENCY}_LIBRARY_DIR)

#     else()

#         message(STATUS "${PROJECT_DEPENDENCY} could not be added to TEST_ENVIRONMENT")

#     endif()

# endforeach()

# if(WIN32)

#     if(TARGET tinyxml2 OR TARGET tinyxml2::tinyxml2)
#         set(TEST_ENVIRONMENT
#             "${TEST_ENVIRONMENT};$<TARGET_FILE_DIR:${TINYXML2_LIBRARY}>")
#     elseif(EXISTS TINYXML2_LIBRARY)
#         get_filename_component(
#             TINYXML2_LIBRARY_DIR ${TINYXML2_LIBRARY} DIRECTORY)
#         set(TEST_ENVIRONMENT
#             "${TEST_ENVIRONMENT};${TINYXML2_LIBRARY_DIR}")
#         unset(TINYXML2_LIBRARY_DIR)
#     endif()

#     if(TARGET yamlcpp OR TARGET yamlcpp::yamlcpp)
#         set(TEST_ENVIRONMENT
#             "${TEST_ENVIRONMENT};$<TARGET_FILE_DIR:${YAMLCPP_LIBRARY}>")
#     elseif(EXISTS YAMLCPP_LIBRARY)
#         get_filename_component(
#             YAMLCPP_LIBRARY_DIR ${YAMLCPP_LIBRARY} DIRECTORY)
#         set(TEST_ENVIRONMENT
#             "${TEST_ENVIRONMENT};${YAMLCPP_LIBRARY_DIR}")
#         unset(YAMLCPP_LIBRARY_DIR)
#     endif()

#     string(REPLACE ";" "\\;" TEST_ENVIRONMENT "${TEST_ENVIRONMENT}")

# endif(WIN32)

# Populate the tests
foreach(TEST IN LISTS TESTS)

    set(TEST_NAME "tool.application.ddsrouter.compose.${TEST}")

    add_test(
            NAME ${TEST_NAME}
            COMMAND bash docker-compose.sh -t ${TEST_NAME} -f ${TEST}/compose.yml)
        )

        # # Set test properties
        # set_tests_properties(
        #     ${TEST_NAME}
        #     PROPERTIES
        #         ENVIRONMENT "${TEST_ENVIRONMENT}"
        #     )

endforeach()

unset(TEST_ENVIRONMENT)
