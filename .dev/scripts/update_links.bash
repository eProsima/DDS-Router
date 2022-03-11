#!/bin/bash

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

#
# Update the links to common files for every subdirectory in the project
#
# This script allows to create and link every repeated file along the subdirectories with a hard
# link. So if any document is modified, all of them will.
# NOTE: A hard link is used instead of symbolic one for Windows support.
#
# ARGS:
#  -w --workspace  <path>               Parent project dir path
#               (Default: ${pwd})
#  -l --library-dir "<relative_paths>"  Relative subdirectories names from workspace (between brackets)
#                                       for those packages that produce libraries
#               (Default: "ddsrouter_utils ddsrouter_event ddsrouter_core ddsrouter_yaml")
#  -a --app-dir     "<relative_paths>"  Relative subdirectories names from workspace (between brackets)
#                                       for those packages that produce executables
#               (Default: "tools/ddsrouter_tool")
#  -f --force                           Force to update links if files already exist.
#  -u --unlink                          Create as new all .common files without links.
#

WORKSPACE_DIR=$(pwd)
LIBRARY_SUBDIRECTORIES_RELATIVE="ddsrouter_utils ddsrouter_event ddsrouter_core ddsrouter_yaml"
APP_SUBDIRECTORIES_RELATIVE="tools/ddsrouter_tool"
FORCE_UPDATE=false
UNLINK_ARG=false

LIBRARY_CMAKE_NAME="library_CMakeLists.txt"
APP_CMAKE_NAME="app_CMakeLists.txt"
APP_SURPLUS_DIR="include"

# Parse arguments
POSITIONAL=()
while [[ $# -gt 0 ]]
do
    key="$1"
    case $key in
        -w|--workspace)
        WORKSPACE_DIR="$2"
        shift # pass argument
        shift # pass value
        ;;
        -l|--library-dir)
        LIBRARY_SUBDIRECTORIES_RELATIVE="$2"
        shift # past argument
        shift # past value
        ;;
        -a|--app-dir)
        APP_SUBDIRECTORIES_RELATIVE="$2"
        shift # past argument
        shift # past value
        ;;
        -u|--unlink)
        UNLINK_ARG=true
        shift # past argument
        ;;
        -f|--force)
        FORCE_UPDATE=true
        shift # past argument
        ;;
        *)    # unknown option
        POSITIONAL+=("$1") # save it in an array for later
        shift # past argument
        ;;
    esac
done
set -- "${POSITIONAL[@]}" # restore positional parameters

function update_link () {

    local FORCE_ARG=""
    if [ "${FORCE_UPDATE}" == true ]
    then
        FORCE_ARG="--force"
    fi

    local LINK_ARG=""
    if [ "${UNLINK_ARG}" == false ]
    then
        LINK_ARG="--link"
    fi

    local LIBRARY_ARG="${2}"

    for SUBDIR_RELATIVE in ${1}
    do
        echo "------------------------------------"
        echo "Updating subdirectory ${WORKSPACE_DIR}/${SUBDIR_RELATIVE}"

        # Previously remove every already existing file if force is set
        if [ "${FORCE_UPDATE}" == true ]
        then

            # Move to .common dir to get relative paths for every file to remove
            local CURRENT_PWD=$(pwd)
            cd ${WORKSPACE_DIR}/.common
            local EVERY_FILE=$(find . -type f)
            cd ${CURRENT_PWD}

            # For every file found, remove it in subdirectory
            for FILE_IN_SUBDIRECTORY in ${EVERY_FILE}
            do
                # echo "Removing file ${WORKSPACE_DIR}/${SUBDIR_RELATIVE}/${FILE_IN_SUBDIRECTORY}"
                rm -f ${WORKSPACE_DIR}/${SUBDIR_RELATIVE}/${FILE_IN_SUBDIRECTORY}

                # Remove file for src/cpp CMake special file
                rm -f ${WORKSPACE_DIR}/${SUBDIR_RELATIVE}/src/cpp/CMakeLists.txt

            done
        fi

        # Create hard links from any document in .common file in every dir in subdir variable
        cp --archive ${LINK_ARG} ${FORCE_ARG} ${WORKSPACE_DIR}/.common/* ${WORKSPACE_DIR}/${SUBDIR_RELATIVE}

        # Create link for src/cpp CMake special file

        if [ "${LIBRARY_ARG}" == true ]
        then
            cp --archive ${LINK_ARG} --force ${WORKSPACE_DIR}/.common/src/cpp/${LIBRARY_CMAKE_NAME} ${WORKSPACE_DIR}/${SUBDIR_RELATIVE}/src/cpp/CMakeLists.txt
        else
            cp --archive ${LINK_ARG} --force ${WORKSPACE_DIR}/.common/src/cpp/${APP_CMAKE_NAME_CMAKE_NAME} ${WORKSPACE_DIR}/${SUBDIR_RELATIVE}/src/cpp/CMakeLists.txt
            rm -f ${WORKSPACE_DIR}/${APP_SURPLUS_DIR}
        fi

        # Remove extra files
        rm ${FORCE_ARG} ${WORKSPACE_DIR}/${SUBDIR_RELATIVE}/src/cpp/${LIBRARY_CMAKE_NAME}
        rm ${FORCE_ARG} ${WORKSPACE_DIR}/${SUBDIR_RELATIVE}/src/cpp/${APP_CMAKE_NAME}

    done
}

# Executing script
echo
echo "------------------------------------"
echo "Updating links script running"
echo "Workspace: ${WORKSPACE_DIR}"
echo "Subdirectories libraries to update: ${LIBRARY_SUBDIRECTORIES_RELATIVE}"
echo "Subdirectories apps to update: ${APP_SUBDIRECTORIES_RELATIVE}"
echo "Force update: ${FORCE_UPDATE}"
if [ "${UNLINK_ARG}" == true ]
then
    echo "Linking files"
else
    echo "Unlinking files"
fi

update_link "${LIBRARY_SUBDIRECTORIES_RELATIVE}" true
update_link "${APP_SUBDIRECTORIES_RELATIVE}" false
