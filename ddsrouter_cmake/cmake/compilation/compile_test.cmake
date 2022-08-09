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

# This macro configure the CMake variables needed to enable tests
# It must be called only when tests must be built
macro(configure_test_flags)

    enable_testing()
    include(CTest)

endmacro()

# TODO
macro(compile_test_library _TEST_PATH)

    if(BUILD_LIBRARY_TESTS)

        message(STATUS "Compiling ${PROJECT_NAME} library tests")

        configure_test_flags()

        test_requirements()

        add_subdirectory(${_TEST_PATH})

    endif()

endmacro()

# TODO
macro(compile_test_documentation _TEST_PATH)

    if(BUILD_DOCS_TESTS)

        message(STATUS "Compiling ${PROJECT_NAME} documentation tests")

        configure_test_flags()

        add_subdirectory(${_TEST_PATH})

    endif()

endmacro()

# TODO
macro(compile_test_tool _TEST_PATH)

    if(BUILD_TOOL_TESTS)

        message(STATUS "Compiling ${PROJECT_NAME} tool tests")

        configure_test_flags()

        test_requirements()

        add_subdirectory(${_TEST_PATH})

    endif()

endmacro()

macro(compile_test_compose _TEST_PATH)

    if(BUILD_COMPOSE_TESTS)

        message(STATUS "Compiling ${PROJECT_NAME}")

        configure_test_flags()

        add_subdirectory(${_TEST_PATH})

    endif()

endmacro()
