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
# Compile C++ Library
###############################################################################

# TODO
function(compile_library _SOURCE_PATH _INCLUDE_PATH)

    if (BUILD_LIBRARY)

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
            GLOB_RECURSE SOURCES_FILES
                "${_SOURCE_PATH}/*.c"
                "${_SOURCE_PATH}/*.cpp"
                "${_SOURCE_PATH}/*.cxx"
                "${_SOURCE_PATH}/**/*.c"
                "${_SOURCE_PATH}/**/*.cpp"
                "${_SOURCE_PATH}/**/*.cxx"
            )

        # Project headers
        if(MSVC)
            file(
                GLOB_RECURSE HEADERS_FILES
                    ${_INCLUDE_PATH}/**/*.h
                    ${_INCLUDE_PATH}/**/*.hpp
                    ${_INCLUDE_PATH}/**/*.ipp
                    ${_SOURCE_PATH}/**/*.h
                    ${_SOURCE_PATH}/**/*.hpp
                    ${_SOURCE_PATH}/**/*.ipp
                )
        endif()

        set(${MODULE_NAME}_SOURCES
            ${SOURCES_FILES}
            ${HEADERS_FILES}
        )

        ###############################################################################
        # Compile library
        ###############################################################################
        # Configure config files for dll and library
        configure_file(
            ${ddsrouter_cmake_LIBRARY_HEADERS_TEMPLATES_PATH}/config.h.in
            ${PROJECT_BINARY_DIR}/include/${MODULE_NAME}/library/config.h)
        configure_file(
            ${ddsrouter_cmake_LIBRARY_HEADERS_TEMPLATES_PATH}/library_dll.h.in
            ${PROJECT_BINARY_DIR}/include/${MODULE_NAME}/library/library_dll.h)
        configure_file(
            ${ddsrouter_cmake_LIBRARY_HEADERS_TEMPLATES_PATH}/eProsima_auto_link.h.in
            ${PROJECT_BINARY_DIR}/include/${MODULE_NAME}/library/eProsima_auto_link.h)

        # Create library
        add_library(${MODULE_NAME} ${${MODULE_NAME}_SOURCES})
        set_target_properties(${MODULE_NAME} PROPERTIES VERSION ${MODULE_VERSION})
        set_target_properties(${MODULE_NAME} PROPERTIES SOVERSION ${MODULE_VERSION_MAJOR})
        # set_target_properties(${MODULE_NAME} PROPERTIES OUTPUT_NAME ${MODULE_TARGET_NAME})

        # Define Public API
        target_compile_definitions(${MODULE_NAME}
            PRIVATE
                ${MODULE_MACRO}_SOURCE
            )

        if(MSVC)
            target_compile_definitions(${MODULE_NAME}
                INTERFACE
                    _CRT_DECLARE_NONSTDC_NAMES=0
                PRIVATE
                    EPROSIMA_USER_DLL_EXPORT
                    WIN32_WINNT=0x0603
                    _CRT_DECLARE_NONSTDC_NAMES=0
                )
        endif(MSVC)


        target_include_directories(
            ${MODULE_NAME}
            PUBLIC
                $<BUILD_INTERFACE:${_INCLUDE_PATH}>
                $<BUILD_INTERFACE:${_INCLUDE_PATH}/${MODULE_NAME}>
                $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
                $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include/${MODULE_NAME}>
                $<BUILD_INTERFACE:${_SOURCE_PATH}>
                $<INSTALL_INTERFACE:include>
            )

        target_link_libraries(${MODULE_NAME}
                ${MODULE_DEPENDENCIES}
            )

        if(MSVC OR MSVC_IDE)
            set_target_properties(${MODULE_NAME} PROPERTIES RELEASE_POSTFIX -${MODULE_VERSION_MAJOR}.${MODULE_VERSION_MINOR})
            set_target_properties(${MODULE_NAME} PROPERTIES MINSIZEREL_POSTFIX m-${MODULE_VERSION})
            set_target_properties(${MODULE_NAME} PROPERTIES RELWITHDEBINFO_POSTFIX rd-${MODULE_VERSION_MAJOR}.${MODULE_VERSION_MINOR})
            set_target_properties(${MODULE_NAME} PROPERTIES DEBUG_POSTFIX d-${MODULE_VERSION_MAJOR}.${MODULE_VERSION_MINOR})

            get_target_property(TARGET_TYPE ${MODULE_NAME} TYPE)
            if(TARGET_TYPE STREQUAL "SHARED_LIBRARY")

                # Export symbols in DLL library
                string(REPLACE "-" "_" PREPROCESS_DYN_LINK "${MODULE_MACRO}_DYN_LINK")
                target_compile_definitions(${MODULE_NAME} PUBLIC ${PREPROCESS_DYN_LINK})

                # Define how to export symbol's info
                set_target_properties(${MODULE_NAME} PROPERTIES
                    PDB_NAME_DEBUG "${MODULE_NAME}d-${MODULE_VERSION_MAJOR}.${MODULE_VERSION_MINOR}"
                    PDB_NAME_RELWITHDEBINFO "${MODULE_NAME}-${MODULE_VERSION_MAJOR}.${MODULE_VERSION_MINOR}"
                    PDB_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/symbols"
                    )
            else()
                # Rename the library to have a "lib" before.
                set_target_properties(${MODULE_NAME} PROPERTIES OUTPUT_NAME lib${MODULE_NAME})
                set_target_properties(${MODULE_NAME} PROPERTIES
                    COMPILE_PDB_NAME_DEBUG "lib${MODULE_NAME}d-${MODULE_VERSION_MAJOR}.${MODULE_VERSION_MINOR}"
                    COMPILE_PDB_NAME_RELWITHDEBINFO "lib${MODULE_NAME}-${MODULE_VERSION_MAJOR}.${MODULE_VERSION_MINOR}"
                    COMPILE_PDB_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/symbols"
                )
            endif()

            if(CMAKE_SYSTEM_NAME STREQUAL "WindowsStore")
                set_target_properties(${MODULE_NAME} PROPERTIES VS_WINRT_COMPONENT "true")
            endif()
        endif()

        ###############################################################################
        # Packaging
        ###############################################################################
        # Install headers
        install(DIRECTORY ${_INCLUDE_PATH}/${MODULE_NAME}
                DESTINATION ${INCLUDE_INSTALL_DIR}
                COMPONENT headers
                FILES_MATCHING
                PATTERN "*.h"
                PATTERN "*.hpp"
                PATTERN "*.ipp"
            )

        # Install config headers
        install(
            FILES
                ${PROJECT_BINARY_DIR}/include/${MODULE_NAME}/library/config.h
                ${PROJECT_BINARY_DIR}/include/${MODULE_NAME}/library/eProsima_auto_link.h
                ${PROJECT_BINARY_DIR}/include/${MODULE_NAME}/library/library_dll.h
            DESTINATION
                ${INCLUDE_INSTALL_DIR}/${MODULE_NAME}/library
            COMPONENT headers
            )

        set(CPACK_COMPONENT_HEADERS_DISPLAY_NAME "C++ Headers" PARENT_SCOPE)
        set(CPACK_COMPONENT_HEADERS_DESCRIPTION "eProsima ${MODULE_NAME_LARGE} C++ Headers" PARENT_SCOPE)

        install(TARGETS ${MODULE_NAME}
            EXPORT ${MODULE_NAME}-targets
            RUNTIME DESTINATION ${BIN_INSTALL_DIR}
            LIBRARY DESTINATION ${LIB_INSTALL_DIR}
            ARCHIVE DESTINATION ${LIB_INSTALL_DIR}
            COMPONENT libraries
            )

        install(EXPORT ${MODULE_NAME}-targets
            DESTINATION ${LIB_INSTALL_DIR}/cmake/${MODULE_NAME}
            COMPONENT cmake
            )

        if(MSVC OR MSVC_IDE)

            foreach(CONFIG ${CMAKE_CONFIGURATION_TYPES})

                # first try dll symbols
                set(PDB_PROPERTY PDB_NAME_${CONFIG})
                string(TOUPPER ${PDB_PROPERTY} PDB_PROPERTY)
                get_target_property(PDB_FILE ${MODULE_NAME} ${PDB_PROPERTY})

                if(PDB_FILE)
                    get_target_property(PDB_DIR ${MODULE_NAME} PDB_OUTPUT_DIRECTORY)
                    set(PDB_FILE "${PDB_DIR}/${CONFIG}/${PDB_FILE}.pdb")
                else()
                    # fallback to static lib symbols
                    set(PDB_PROPERTY COMPILE_PDB_NAME_${CONFIG})
                    string(TOUPPER ${PDB_PROPERTY} PDB_PROPERTY)
                    get_target_property(PDB_FILE ${MODULE_NAME} COMPILE_PDB_NAME_${CONFIG})

                    if(PDB_FILE)
                        get_target_property(PDB_DIR ${MODULE_NAME} COMPILE_PDB_OUTPUT_DIRECTORY)
                        set(PDB_FILE "${PDB_DIR}/${PDB_FILE}.pdb")
                    endif()
                endif()

                if(PDB_FILE)
                    # Install symbols
                    install(FILES ${PDB_FILE}
                        DESTINATION ${LIB_INSTALL_DIR}
                        COMPONENT symbols
                        OPTIONAL
                        )
                endif()

            endforeach()

        endif()

        ###############################################################################
        # Create CMake package config file
        ###############################################################################
        include(CMakePackageConfigHelpers)

        configure_package_config_file(
            ${ddsrouter_cmake_CONFIG_TEMPLATES_PATH}/library-Config.cmake.in
            ${CMAKE_CURRENT_BINARY_DIR}/${MODULE_NAME}-config.cmake
            INSTALL_DESTINATION
                ${MODULE_LIB_INSTALL_DIR}/cmake/${MODULE_NAME}
            PATH_VARS
                BIN_INSTALL_DIR INCLUDE_INSTALL_DIR LIB_INSTALL_DIR DATA_INSTALL_DIR
        )

        write_basic_package_version_file(
            ${CMAKE_CURRENT_BINARY_DIR}/${MODULE_NAME}-config-version.cmake
            VERSION
                ${MODULE_VERSION}
            COMPATIBILITY
                SameMajorVersion
        )

        install(
            FILES
                ${CMAKE_CURRENT_BINARY_DIR}/${MODULE_NAME}-config.cmake
                ${CMAKE_CURRENT_BINARY_DIR}/${MODULE_NAME}-config-version.cmake
            DESTINATION
                ${MODULE_LIB_INSTALL_DIR}/cmake/${MODULE_NAME}
            COMPONENT
                cmake
        )

        set(CPACK_COMPONENT_LIBRARIES_DISPLAY_NAME "Libraries" PARENT_SCOPE)
        set(CPACK_COMPONENT_LIBRARIES_DESCRIPTION "eProsima ${MODULE_NAME_LARGE} libraries" PARENT_SCOPE)

    endif()

endfunction()
