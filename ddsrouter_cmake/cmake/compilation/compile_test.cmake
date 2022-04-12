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
macro(configure_test_flags)

    option(BUILD_TESTS "Build eProsima ${PROJECT_NAME} library tests" OFF)
    option(BUILD_LIBRARY_TESTS "Build eProsima ${PROJECT_NAME} library tests" OFF)
    option(BUILD_APP_TESTS "Build eProsima ${PROJECT_NAME} app tests" OFF)
    option(BUILD_DOCUMENTATION_TESTS "Build eProsima ${PROJECT_NAME} documentation tests" OFF)

    if (BUILD_TESTS)
        set(BUILD_LIBRARY_TESTS ON)
        set(BUILD_APP_TESTS ON)
        set(BUILD_DOCUMENTATION_TESTS ON)

        enable_testing()
        include(CTest)
    endif()

endmacro()

# TODO
macro(compile_test_library _TEST_PATH)

    configure_test_flags()

    if(BUILD_LIBRARY_TESTS)

        message(STATUS "Compiling ${PROJECT_NAME} library tests")

        test_requirements()

        add_subdirectory(${_TEST_PATH})

    endif()

endmacro()

# TODO
macro(compile_test_documentation _TEST_PATH)

    configure_test_flags()

    if(BUILD_DOCUMENTATION_TESTS)

        message(STATUS "Compiling ${PROJECT_NAME} documentation tests")

        add_subdirectory(${_TEST_PATH})

    endif()

endmacro()

# TODO
macro(compile_test_tool _TEST_PATH)

    configure_test_flags()

    if(BUILD_APP_TESTS)

        message(STATUS "Compiling ${PROJECT_NAME} tool tests")

        test_requirements()

        add_subdirectory(${_TEST_PATH})

    endif()

endmacro()