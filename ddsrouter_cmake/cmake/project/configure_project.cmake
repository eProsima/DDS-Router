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
# Configure Project macro
###############################################################################

# Set common variables of project by loading load_project_settings and read_version
#
# REQUIRED:
# - project_settings.cmake  file in workspace
# - VERSION                 file in workspace
#
# RETURN VARIABLES:
# - PROJECT:
#   - MODULE_NAME
#   - MODULE_NAME_LARGE
#   - MODULE_SUMMARY
#   - MODULE_DESCRIPTION
# - VERSION:
#   - MODULE_VERSION_MAJOR
#   - MODULE_VERSION_MINOR
#   - MODULE_VERSION_PATCH
#   - MODULE_VERSION_STRING
#   - MODULE_VERSION
# - PATHS
#   - MODULE_BIN_INSTALL_DIR
#   - MODULE_INCLUDE_INSTALL_DIR
#   - MODULE_LIB_INSTALL_DIR
#   - MODULE_DATA_INSTALL_DIR
#   - MODULE_LICENSE_INSTALL_DIR
#
# NOTE:
# project() function could not be called but from main CMakeLists.txt file
#
# ARGUMENTS:
# NONE
macro(configure_project)

    # Prevent build in source
    in_source_build_prevention()

    # Set variables for project
    load_project_settings()

    # Read version if it is not already set
    if( NOT DEFINED MODULE_VERSION_MAJOR OR
        NOT DEFINED MODULE_VERSION_MINOR OR
        NOT DEFINED MODULE_VERSION_PATCH OR
        NOT DEFINED MODULE_VERSION)

        read_version(${MODULE_VERSION_FILE_PATH})
    endif()

    # Read git commit hash if not already set
    if( NOT DEFINED MODULE_COMMIT_HASH)

        read_commit_hash()
    endif()

    # Set CMake options
    configure_cmake_options()

    # Set install directories
    set_installation_paths()

    # Finishing macro
    message(STATUS "Project ${MODULE_NAME_LARGE} configured by eProsima module configuration.")

endmacro()
