# Copyright 2026 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#################################
# DdsRouter Yaml Validator Test #
#################################

set(TEST_NAME YamlValidatorDdsRouterTest)


set(TEST_SOURCES
        YamlValidatorDdsRouterTest.cpp
    )

set(TEST_LIST
        validation_passed
        validation_failed
    )

set(TEST_EXTRA_LIBRARIES
        yaml-cpp
        fastcdr
        fastdds
        cpp_utils
        ddspipe_core
        ddspipe_participants
        ddspipe_yaml
    )

configure_file(
    "${PROJECT_SOURCE_DIR}/../resources/configurations/ddsrouter_config_schema.json"
    "${CMAKE_CURRENT_BINARY_DIR}/ddsrouter_config_schema.json"
    COPYONLY
)

file(REMOVE_RECURSE "${CMAKE_CURRENT_BINARY_DIR}/valid_config_files_router/")
file(REMOVE_RECURSE "${CMAKE_CURRENT_BINARY_DIR}/invalid_config_files_router/")

file(GLOB_RECURSE TEST_NEEDED_SOURCES
    RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    "invalid_config_files_router/*.yaml"
    "valid_config_files_router/*.yaml"
)

file(GLOB GETTING_STARTED_YAMLS
    "${PROJECT_SOURCE_DIR}/../docs/resources/getting_started/*.yaml"
)
file(COPY ${GETTING_STARTED_YAMLS}
    DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/valid_config_files_router/"
)

file(GLOB EXAMPLES_YAMLS
    "${PROJECT_SOURCE_DIR}/../resources/configurations/examples/*.yaml"
)
file(COPY ${EXAMPLES_YAMLS}
    DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/valid_config_files_router/"
)

add_unittest_executable(
    "${TEST_NAME}"
    "${TEST_SOURCES}"
    "${TEST_LIST}"
    "${TEST_EXTRA_LIBRARIES}"
    "${TEST_NEEDED_SOURCES}"
)

