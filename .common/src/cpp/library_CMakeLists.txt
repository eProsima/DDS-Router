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

###############################################################################
# Get source files
###############################################################################
# Project sources
file(GLOB_RECURSE SOURCES_FILES
        "*.c"
        "*.cpp"
        "*.cxx"
    )

# Project headers
if(MSVC)
    file(GLOB_RECURSE HEADERS_FILES
            ${PROJECT_SOURCE_DIR}/include/**/*.h
            ${PROJECT_SOURCE_DIR}/include/**/*.hpp
            ${PROJECT_SOURCE_DIR}/include/**/*.ipp
            ${PROJECT_SOURCE_DIR}/src/cpp/**/*.h
            ${PROJECT_SOURCE_DIR}/src/cpp/**/*.hpp
            ${PROJECT_SOURCE_DIR}/src/cpp/**/*.ipp
        )
endif()

set(${PROJECT_NAME}_SOURCES
        ${SOURCES_FILES}
        ${HEADERS_FILES}
    )

###############################################################################
# Compile library
###############################################################################
# Configure config files for dll and library
configure_file(${PROJECT_SOURCE_DIR}/include/library/config.h.in
    ${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/library/config.h)
configure_file(${PROJECT_SOURCE_DIR}/include/library/library_dll.h.in
    ${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/library/library_dll.h)
configure_file(${PROJECT_SOURCE_DIR}/include/library/eProsima_auto_link.h.in
    ${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/library/eProsima_auto_link.h)

# Create library
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROJECT_VERSION})
set_target_properties(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION_MAJOR})

# Define Public API
target_compile_definitions(${PROJECT_NAME}
    PRIVATE
        ${SUBMODULE_PROJECT_MACROS}_SOURCE
    )

if(MSVC)
    target_compile_definitions(${PROJECT_NAME}
        INTERFACE
            _CRT_DECLARE_NONSTDC_NAMES=0
        PRIVATE
            EPROSIMA_USER_DLL_EXPORT
            WIN32_WINNT=0x0603
            _CRT_DECLARE_NONSTDC_NAMES=0
        )
endif(MSVC)


target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}>
        $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
        $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/cpp>
        $<INSTALL_INTERFACE:include>
    )

target_link_libraries(${PROJECT_NAME}
        ${SUBMODULE_PROJECT_DEPENDENCIES}
    )

if(MSVC OR MSVC_IDE)
    set_target_properties(${PROJECT_NAME} PROPERTIES RELEASE_POSTFIX -${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR})
    set_target_properties(${PROJECT_NAME} PROPERTIES MINSIZEREL_POSTFIX m-${PROJECT_VERSION})
    set_target_properties(${PROJECT_NAME} PROPERTIES RELWITHDEBINFO_POSTFIX rd-${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR})
    set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX d-${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR})

    get_target_property(TARGET_TYPE ${PROJECT_NAME} TYPE)
    if(TARGET_TYPE STREQUAL "SHARED_LIBRARY")

        # Export symbols in DLL library
        string(REPLACE "-" "_" PREPROCESS_DYN_LINK "${PROJECT_NAME_UPPER}_DYN_LINK")
        target_compile_definitions(${PROJECT_NAME} PUBLIC ${PREPROCESS_DYN_LINK})

        # Define how to export symbol's info
        set_target_properties(${PROJECT_NAME} PROPERTIES
            PDB_NAME_DEBUG "${PROJECT_NAME}d-${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}"
            PDB_NAME_RELWITHDEBINFO "${PROJECT_NAME}-${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}"
            PDB_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/symbols"
            )
    else()
        # Rename the library to have a "lib" before.
        set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME lib${PROJECT_NAME})
        set_target_properties(${PROJECT_NAME} PROPERTIES
            COMPILE_PDB_NAME_DEBUG "lib${PROJECT_NAME}d-${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}"
            COMPILE_PDB_NAME_RELWITHDEBINFO "lib${PROJECT_NAME}-${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}"
            COMPILE_PDB_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/symbols"
        )
    endif()

    if("${CMAKE_SYSTEM_NAME}" STREQUAL "WindowsStore")
        set_target_properties(${PROJECT_NAME} PROPERTIES VS_WINRT_COMPONENT "true")
    endif()
endif()

###############################################################################
# Packaging
###############################################################################
# Install headers
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
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
        ${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/library/config.h
        ${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/library/eProsima_auto_link.h
        ${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/library/library_dll.h
    DESTINATION
        ${INCLUDE_INSTALL_DIR}/${PROJECT_NAME}/library
    COMPONENT headers
    )

set(CPACK_COMPONENT_HEADERS_DISPLAY_NAME "C++ Headers" PARENT_SCOPE)
set(CPACK_COMPONENT_HEADERS_DESCRIPTION "eProsima ${PROJECT_NAME_LARGE} C++ Headers" PARENT_SCOPE)

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}-targets
    RUNTIME DESTINATION ${BIN_INSTALL_DIR}
    LIBRARY DESTINATION ${LIB_INSTALL_DIR}
    ARCHIVE DESTINATION ${LIB_INSTALL_DIR}
    COMPONENT libraries
    )

install(EXPORT ${PROJECT_NAME}-targets
    DESTINATION ${LIB_INSTALL_DIR}/cmake/${PROJECT_NAME}
    COMPONENT cmake
    )

if(MSVC OR MSVC_IDE)

    foreach(CONFIG ${CMAKE_CONFIGURATION_TYPES})

        # first try dll symbols
        set(PDB_PROPERTY PDB_NAME_${CONFIG})
        string(TOUPPER ${PDB_PROPERTY} PDB_PROPERTY)
        get_target_property(PDB_FILE ${PROJECT_NAME} ${PDB_PROPERTY})

        if(PDB_FILE)
            get_target_property(PDB_DIR ${PROJECT_NAME} PDB_OUTPUT_DIRECTORY)
            set(PDB_FILE "${PDB_DIR}/${CONFIG}/${PDB_FILE}.pdb")
        else()
            # fallback to static lib symbols
            set(PDB_PROPERTY COMPILE_PDB_NAME_${CONFIG})
            string(TOUPPER ${PDB_PROPERTY} PDB_PROPERTY)
            get_target_property(PDB_FILE ${PROJECT_NAME} COMPILE_PDB_NAME_${CONFIG})

            if(PDB_FILE)
                get_target_property(PDB_DIR ${PROJECT_NAME} COMPILE_PDB_OUTPUT_DIRECTORY)
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

configure_package_config_file(${PROJECT_SOURCE_DIR}/cmake/packaging/Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
    INSTALL_DESTINATION ${LIB_INSTALL_DIR}/cmake/${PROJECT_NAME}
    PATH_VARS BIN_INSTALL_DIR INCLUDE_INSTALL_DIR LIB_INSTALL_DIR
    )
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
    )
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
    DESTINATION ${LIB_INSTALL_DIR}/cmake/${PROJECT_NAME}
    COMPONENT cmake
    )

set(CPACK_COMPONENT_LIBRARIES_DISPLAY_NAME "Libraries" PARENT_SCOPE)
set(CPACK_COMPONENT_LIBRARIES_DESCRIPTION "eProsima ${PROJECT_NAME_LARGE} libraries" PARENT_SCOPE)
