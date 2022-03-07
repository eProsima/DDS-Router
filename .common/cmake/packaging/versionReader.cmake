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
# Version Reader function
###############################################################################

# Return a list of all cpp files required by ddsrouter library
# Arguments:
# TEST_SOURCES -> List of source files where new sources will be added
# TODO: check if it is needed the arguments with the versions
function(read_version VERSION_FILE_NAME)

    file(READ ${VERSION_FILE_NAME} READ_VERSION_FILE)

    string(REGEX MATCH "VERSION_MAJOR ([0-9]*)" _ ${READ_VERSION_FILE})
    set(PRODUCT_MAJOR_VERSION ${CMAKE_MATCH_1} PARENT_SCOPE)

    string(REGEX MATCH "VERSION_MINOR ([0-9]*)" _ ${READ_VERSION_FILE})
    set(PRODUCT_MINOR_VERSION ${CMAKE_MATCH_1} PARENT_SCOPE)

    string(REGEX MATCH "VERSION_PATCH ([0-9]*)" _ ${READ_VERSION_FILE})
    set(PRODUCT_PATCH_VERSION ${CMAKE_MATCH_1} PARENT_SCOPE)

endfunction()
