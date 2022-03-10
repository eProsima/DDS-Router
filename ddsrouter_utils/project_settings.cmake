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
# Set settings for project ddsrouter_utils
###############################################################################

set(SUBMODULE_PROJECT_NAME
    ddsrouter_utils)

set(SUBMODULE_PROJECT_SUMMARY
    "C++ library for generic useful methods and classes for DDS Router.")

set(SUBMODULE_PROJECT_FIND_PACKAGES
    fastcdr
    fastrtps)

set(SUBMODULE_PROJECT_DEPENDENCIES
    $<$<BOOL:${WIN32}>:iphlpapi$<SEMICOLON>Shlwapi>
    ${SUBMODULE_PROJECT_FIND_PACKAGES})

set(SUBMODULE_PROJECT_MACROS
    DDSROUTER_UTILS)

# set(SUBMODULE_THIRDPARTY_HEADERONLY_PACKAGES )
