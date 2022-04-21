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
# Compile C++ Executable Tool
###############################################################################

# TODO
function(compile_tool _SOURCE_PATH)

    if (BUILD_TOOL)

        ###############################################################################
        # Check needed variables
        ###############################################################################
        if(NOT DEFINED MODULE_NAME)
            message(FATAL_ERROR "MODULE_NAME is not defined. Use load_project_settings or configure_project macros to set it")
        endif()

        ###############################################################################
        # Get source files
        ###############################################################################
        # Project sources
        file(
            GLOB_RECURSE ${MODULE_NAME}_SOURCES
                "${_SOURCE_PATH}/*.c"
                "${_SOURCE_PATH}/*.cpp"
                "${_SOURCE_PATH}/*.cxx"
                "${_SOURCE_PATH}/**/*.c"
                "${_SOURCE_PATH}/**/*.cpp"
                "${_SOURCE_PATH}/**/*.cxx"
            )

        ###############################################################################
        # Compile executable
        ###############################################################################

        # Add executable
        add_executable(${MODULE_NAME} ${${MODULE_NAME}_SOURCES})

        # Set name for target
        set_target_properties(
            ${MODULE_NAME}
            PROPERTIES OUTPUT_NAME
                "${MODULE_TARGET_NAME}"
        )

        if(LOG_INFO)
            target_compile_definitions(${MODULE_NAME}
                PRIVATE FASTDDS_ENFORCE_LOG_INFO
                PRIVATE HAVE_LOG_NO_INFO=0
                )
        endif()

        # Link dependent libraries
        target_link_libraries(
            ${MODULE_NAME}
            ${MODULE_DEPENDENCIES}
        )

        # Install
        install(
            TARGETS
                ${MODULE_NAME}
            RUNTIME DESTINATION
                ${BIN_INSTALL_DIR}
        )

    endif()

endfunction()
