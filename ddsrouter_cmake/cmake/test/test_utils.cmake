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

# Return a list of all cpp and hpp files required by ddsrouter library
# Arguments:
# TEST_SOURCES -> List of source files where new sources will be added
function(all_library_sources TEST_SOURCES)

    file(GLOB_RECURSE LIBRARY_SOURCES
            "${PROJECT_SOURCE_DIR}/src/cpp/**/*.c"
            "${PROJECT_SOURCE_DIR}/src/cpp/**/*.cpp"
            "${PROJECT_SOURCE_DIR}/src/cpp/**/*.cxx"
            "${PROJECT_SOURCE_DIR}/src/cpp/**/*.h"
            "${PROJECT_SOURCE_DIR}/src/cpp/**/*.hpp"
            "${PROJECT_SOURCE_DIR}/src/cpp/**/*.hxx"
            "${PROJECT_SOURCE_DIR}/test/TestUtils/**/*.cpp"
            "${PROJECT_SOURCE_DIR}/test/TestUtils/*.cpp"
        )
    set(NEW_TEST_SOURCES "${TEST_SOURCES};${LIBRARY_SOURCES}")
    set(TEST_SOURCES ${NEW_TEST_SOURCES} PARENT_SCOPE)

endfunction()

# Return a list of all hpp files required by ddsrouter library
# Arguments:
# TEST_SOURCES -> List of source files where new sources will be added
function(all_header_sources TEST_SOURCES)

    file(GLOB_RECURSE LIBRARY_SOURCES
            "${PROJECT_SOURCE_DIR}/src/cpp/**/*.h"
            "${PROJECT_SOURCE_DIR}/src/cpp/**/*.hpp"
            "${PROJECT_SOURCE_DIR}/src/cpp/**/*.hxx"
        )
    set(NEW_TEST_SOURCES "${TEST_SOURCES};${LIBRARY_SOURCES}")
    set(TEST_SOURCES ${NEW_TEST_SOURCES} PARENT_SCOPE)

endfunction()
