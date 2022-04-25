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
# Compile Sphinx Documentation
###############################################################################

# TODO
macro(compile_documentation)

    if (BUILD_DOCS)

        message(STATUS "Compiling documentation")

        ####################################################################################################
        # Build documentation
        ####################################################################################################

        set(PROJECT_SOURCE_DOCS_DIR ${PROJECT_SOURCE_DIR})
        set(PROJECT_BINARY_DOCS_DIR ${PROJECT_BINARY_DIR}/docs)
        set(DOCS_OUTPUT_HTML_DIR ${PROJECT_BINARY_DOCS_DIR}/html)

        # Create docs directories
        add_custom_target(doc-dirs
            COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_BINARY_DOCS_DIR}
            COMMAND ${CMAKE_COMMAND} -E make_directory ${DOCS_OUTPUT_HTML_DIR}
            COMMENT "Creating documentation directories" VERBATIM)

        ####################################################################################################
        # Build Sphinx documentation
        ####################################################################################################

        # Find sphinx
        find_package(Sphinx REQUIRED)

        set(DOCS_BUILDER html)

        # Generate the sphinx documentation
        add_custom_target(Sphinx ALL
            COMMAND
            ${SPHINX_EXECUTABLE} -b ${DOCS_BUILDER}
            -d "${PROJECT_BINARY_DOCS_DIR}/doctrees"
            ${PROJECT_SOURCE_DOCS_DIR}
            ${DOCS_OUTPUT_HTML_DIR}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating documentation with Sphinx"
            $<$<STREQUAL:"${CMAKE_BUILD_TYPE}","Debug">:-Dtodo_include_todos=1>)

        # Install the generated docs
        install(DIRECTORY ${DOCS_OUTPUT_HTML_DIR}
            DESTINATION docs/${PROJECT_NAME}/sphinx
            COMPONENT monitor-sphinx
            PATTERN ".buildinfo" EXCLUDE)
        set(CPACK_COMPONENT_monitor-sphinx_DISPLAY_NAME "${MODULE_LARGE_NAME}")
        set(CPACK_COMPONENT_monitor-sphinx_DESCRIPTION
            "${MODULE_DESCRIPTION}")
        set(CPACK_COMPONENTS_ALL ${CPACK_COMPONENTS_ALL} ${DOCS_BUILDER})

    endif()

endmacro()
