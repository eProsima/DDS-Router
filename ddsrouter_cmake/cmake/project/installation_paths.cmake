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
# Installation paths
###############################################################################

# Set installation paths
#
# RETURN VARIABLES:
# - MODULE_BIN_INSTALL_DIR       : Installation path   - bin/
# - MODULE_INCLUDE_INSTALL_DIR   : Include path        - include/
# - MODULE_LIB_INSTALL_DIR       : Library path        - lib/
# - MODULE_DATA_INSTALL_DIR      : Data path           - share/
# - MODULE_LICENSE_INSTALL_DIR   : License path        - /
macro(set_installation_paths)

    # Set MODULE_ variable to installation path
    if (NOT MODULE_BIN_INSTALL_DIR)
        set(MODULE_BIN_INSTALL_DIR bin/ CACHE PATH "Installation directory for binaries")
    endif()

    if (NOT MODULE_INCLUDE_INSTALL_DIR)
        set(MODULE_INCLUDE_INSTALL_DIR include/ CACHE PATH "Installation directory for C++ headers")
    endif()

    if (NOT MODULE_LIB_INSTALL_DIR)
        set(MODULE_LIB_INSTALL_DIR lib/ CACHE PATH "Installation directory for libraries")
    endif()

    if (NOT MODULE_DATA_INSTALL_DIR)
        set(MODULE_DATA_INSTALL_DIR share/ CACHE PATH "Installation directory for data")
    endif()

    if (NOT MODULE_LICENSE_INSTALL_DIR)
        if(WIN32)
            set(MODULE_LICENSE_INSTALL_DIR . CACHE PATH "Installation directory for licenses")
        else()
            set(MODULE_LICENSE_INSTALL_DIR ${MODULE_DATA_INSTALL_DIR}/${MODULE_NAME} CACHE PATH "Installation directory for licenses")
        endif()
    endif()

    # Set common names for installation paths
    set(BIN_INSTALL_DIR ${MODULE_BIN_INSTALL_DIR})
    set(INCLUDE_INSTALL_DIR ${MODULE_INCLUDE_INSTALL_DIR})
    set(LIB_INSTALL_DIR ${MODULE_LIB_INSTALL_DIR})
    set(DATA_INSTALL_DIR ${MODULE_DATA_INSTALL_DIR})
    set(LICENSE_INSTALL_DIR ${MODULE_LICENSE_INSTALL_DIR})

    # Finish macro
    message(STATUS "Installation paths set:")
    message(STATUS " - Set binary path to: ${MODULE_BIN_INSTALL_DIR}")
    message(STATUS " - Set include path to: ${MODULE_INCLUDE_INSTALL_DIR}")
    message(STATUS " - Set library path to: ${MODULE_LIB_INSTALL_DIR}")
    message(STATUS " - Set data path to: ${MODULE_DATA_INSTALL_DIR}")
    message(STATUS " - Set license path to: ${MODULE_LICENSE_INSTALL_DIR}")

endmacro()
