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

###############################################################################
# Configure Test module
###############################################################################

# TODO
macro(compile_test _TEST_PATH)

    option(BUILD_TESTS "Build eProsima ${PROJECT_NAME} library tests" OFF)
    option(BUILD_APP_TESTS "Build eProsima ${PROJECT_NAME} app library tests" OFF)

    if (BUILD_TESTS)
        set(BUILD_APP_TESTS ON)
    endif()

    if(BUILD_APP_TESTS)
        # CTest needs to be included here, otherwise it is not possible to run the tests from the root
        # of the build directory
        enable_testing()
        include(CTest)
    endif()

    if(BUILD_APP_TESTS)

        message(STATUS "Compiling ${PROJECT_NAME} tests")

        test_requirements()

        add_subdirectory(${_TEST_PATH})

    endif()

endmacro()
