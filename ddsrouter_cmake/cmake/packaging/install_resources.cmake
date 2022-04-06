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
# Install resources
###############################################################################

# TODO comment
macro(install_resources _RESOURCES_PATH _RESOURCES_DESTINATION)

    # Install resource files
    INSTALL(
        DIRECTORY
            ${PROJECT_SOURCE_DIR}/${_RESOURCES_PATH}/
        DESTINATION
            ${DATA_INSTALL_DIR}/${_RESOURCES_DESTINATION}/
    )

endmacro()

# TODO comment
macro(eprosima_install_resources)

    # If MODULE_RESOURCES_PATH defined, use it as source for resource installation
    if(DEFINED MODULE_RESOURCES_PATH)
        install_resources(${MODULE_RESOURCES_PATH} "resources")
    endif()

endmacro()
