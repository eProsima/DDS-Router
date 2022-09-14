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
# - project() already initialized
# - set projects to find in MODULE_PROJECT_FIND_PACKAGES
# - set thirdparty headeronly in MODULE_THIRDPARTY_HEADERONLY
#
# FUNCTIONALITY:
# - CMake Build Type set
# - CMake C++ version set
# - Set Build Shared Libs as default option
# - Find packages and thirdparty headeronly libraries
# - add subdirectory src/cpp
#
# ARGUMENTS:
# NONE
macro(configure_project_cpp)

    # CMake Build Type set
    set_cmake_build_type()

    # Check C++ version
    check_cpp(${MODULE_CPP_VERSION})

    # Set Build Shared Libs as default option
    option(BUILD_SHARED_LIBS "Create shared libraries by default" ON)

    # Find externals
    find_external_projects(
            "${MODULE_FIND_PACKAGES}"
        )
    find_thirdparties_headeronly(
            "${MODULE_THIRDPARTY_HEADERONLY}"
            "${MODULE_THIRDPARTY_PATH}"
        )

    # Activate coverage it required
    if (CODE_COVERAGE)
        activate_code_coverage()
    endif()

    # Set custom C++ Flags
    custom_cpp_flags()

    # Finish macro
    message(STATUS "C++ Project ${MODULE_NAME_LARGE} configured.")

endmacro()
