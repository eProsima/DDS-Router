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
# This script allows to link every repeated file along the subdirectories with a hard link.
# So if any document is modified, all of them will.
# NOTE: A hard link is used instead of symbolic one for Windows support.
#
# ARGS:
#  -w --workspace  <path>               Parent project dir path
#               (Default: ${pwd})
#  -d --dir        "<relative_paths>"   Relative subdirectories names from workspace (between brackets)
#               (Default: "ddsrouter_utils ddsrouter_event ddsrouter ddsrouter_yaml tools/ddsrouter_tool")
#  -f --force                           Force to update links if files already exist.
#

WORKSPACE_DIR=$(pwd)
SUBDIRECTORIES_RELATIVE="ddsrouter_utils ddsrouter_event ddsrouter_core ddsrouter_yaml tools/ddsrouter_tool"
FORCE_UPDATE=false

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
        -d|--dir)
        SUBDIRECTORIES_UPDATE="$2"
        shift # past argument
        shift # past value
        ;;
        -f|--force)
        FORCE_UPDATE=true
        shift # past argument
        shift # past value
        ;;
        *)    # unknown option
        POSITIONAL+=("$1") # save it in an array for later
        shift # past argument
        ;;
    esac
done
set -- "${POSITIONAL[@]}" # restore positional parameters

# download fast
echo
echo "------------------------------------"
echo "Updating links script running"
echo "Workspace: ${WORKSPACE_DIR}"
echo "Subdirectories to update: ${SUBDIRECTORIES_RELATIVE}"
echo "Force update: ${FORCE_UPDATE}"

for SUBDIR_RELATIVE in ${SUBDIRECTORIES_RELATIVE}
do
    echo "------------------------------------"
    echo "Updating subdirectory ${WORKSPACE_DIR}/${SUBDIR_RELATIVE}"

    # Create hard links from any document in .common file in every dir in subdir variable
    if [ "${FORCE_UPDATE}" == true ]
    then
        cp --archive --link --force ${WORKSPACE_DIR}/.common/* ${WORKSPACE_DIR}/${SUBDIR_RELATIVE}
    else
        cp --archive --link ${WORKSPACE_DIR}/.common/* ${WORKSPACE_DIR}/${SUBDIR_RELATIVE}
    fi
done
