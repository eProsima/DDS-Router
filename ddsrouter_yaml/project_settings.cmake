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
# Set settings for project ddsrouter_yaml
###############################################################################

set(SUBMODULE_PROJECT_NAME
    ddsrouter_yaml)

set(SUBMODULE_PROJECT_SUMMARY
    "C++ library to create a DDS Router configuration from a YAML.")

set(SUBMODULE_PROJECT_FIND_PACKAGES
    yaml-cpp
    ddsrouter_utils
    ddsrouter_core)

set(SUBMODULE_PROJECT_DEPENDENCIES
    ${SUBMODULE_PROJECT_FIND_PACKAGES})

set(SUBMODULE_PROJECT_MACROS
    DDSROUTERYAML)

# set(SUBMODULE_THIRDPARTY_HEADERONLY_PACKAGES )
