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
# Yaml DDSRouter Configuration Test #
#####################################

set(TEST_NAME YamlConfigurationDDSRouterTest)

set(TEST_SOURCES
        YamlConfigurationDDSRouterTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/yaml/YamlReader.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/yaml/YamlManager.cpp
    )
all_library_sources("${TEST_SOURCES}")

set(TEST_LIST
        get_ddsrouter_configuration_trivial
        get_ddsrouter_configuration_ros_case
    )

set(TEST_EXTRA_LIBRARIES
        fastcdr
        fastrtps
        yaml-cpp
        $<$<BOOL:${WIN32}>:iphlpapi$<SEMICOLON>Shlwapi>
    )

add_unittest_executable(
    "${TEST_NAME}"
    "${TEST_SOURCES}"
    "${TEST_LIST}"
    "${TEST_EXTRA_LIBRARIES}")
