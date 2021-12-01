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
# TEST_NAME -> test name (name of the class of the test in Test .cpp)
# TEST_SOURCES -> sources for the test
# TEST_LIST -> test cases implemented in the Test .cpp
# TEST_EXTRA_LIBRARIES -> libraries that must be linked to compile the test
# Note: pass the arguments with "" in order to send them as a list. Otherwise they will not be received correctly
function(add_unittest_executable TEST_NAME TEST_SOURCES TEST_LIST TEST_EXTRA_LIBRARIES)

    add_test_executable(
        "unittest_${TEST_NAME}"
        "${TEST_SOURCES}"
        "${TEST_NAME}"
        "${TEST_LIST}"
        "${TEST_EXTRA_LIBRARIES}"
    )

endfunction(add_unittest_executable)

add_subdirectory(configuration)
add_subdirectory(dynamic)
add_subdirectory(participant)
add_subdirectory(types)
