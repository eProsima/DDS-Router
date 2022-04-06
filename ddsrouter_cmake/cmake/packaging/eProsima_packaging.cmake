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
# Set common CPACK variables.
###############################################################################

# TODO comment
macro(eprosima_packaging)

    install(FILES ${MODULE_LICENSE_FILE_PATH}
        DESTINATION
            ${MODULE_LICENSE_INSTALL_DIR}
        COMPONENT
            licenses
        )
    set(CPACK_COMPONENT_LICENSES_HIDDEN 1)

    set(CPACK_COMPONENTS_ALL headers)

    if(MSVC OR MSVC_IDE)
        set(CPACK_COMPONENTS_ALL ${CPACK_COMPONENTS_ALL} libraries_${MSVC_ARCH})
    endif()

    set(CPACK_PACKAGE_NAME ${PROJECT_NAME})

    set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "${CPACK_PACKAGE_NAME} - ${${PROJECT_NAME}_DESCRIPTION_SUMMARY}")

    set(CPACK_PACKAGE_DESCRIPTION "${${PROJECT_NAME}_DESCRIPTION}")

    set(CPACK_PACKAGE_VENDOR "eProsima")
    set(CPACK_PACKAGE_CONTACT "eProsima Support <support@eprosima.com>")

    set(CPACK_PACKAGE_VERSION_MAJOR ${MODULE_VERSION_MAJOR})
    set(CPACK_PACKAGE_VERSION_MINOR ${MODULE_VERSION_MINOR})
    set(CPACK_PACKAGE_VERSION_PATCH ${MODULE_VERSION_PATCH})
    set(CPACK_PACKAGE_VERSION ${MODULE_VERSION})

    set(CPACK_COMPONENT_CMAKE_HIDDEN 1)
    set(CPACK_COMPONENTS_ALL ${CPACK_COMPONENTS_ALL} cmake)

    include(CPack)

    eprosima_install_resources()

endmacro()
