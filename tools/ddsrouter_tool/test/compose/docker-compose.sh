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

print_usage()
{
    echo "----------------------------------------------------------------------------------"
    echo "Script that retrieves the exit code of the DDS Router docker compose tests."
    echo "----------------------------------------------------------------------------------"
    echo "REQUIRED ARGUMENTS:"
    echo "   -t --test-name [string]        Name of the DDS Router test."
    echo "   -f --compose-file [filename]   The Docker compose file"
    echo ""
    echo "EXAMPLE: bash docker-compose.sh -t <test_name> -f <path/to/compose.yml>"
    echo ""
    exit ${1}
}

parse_options()
{

    if (($# == 0)) ; then print_usage 1 ; fi
    if [ $? != 0 ] ; then print_usage 1 ; fi

    TEMP=`getopt \
            -o t:f:hd \
            --long test-name:,compose-file:,help,debug \
            -n 'Error' \
            -- "$@"`

    eval set -- "${TEMP}"

    TEST_NAME=
    COMPOSE_FILE=
    DEBUG=false
    while true; do
    case "$1" in
        # Mandatory args
        -t | --test-name ) TEST_NAME="$2"; shift 2;;
        -f | --compose-file ) COMPOSE_FILE="$2"; shift 2;;
        # Optional args
        -h | --help ) print_usage 0; shift ;;
        -d | --debug ) DEBUG=true; shift ;;
        # Wrong args
        -- ) shift; break ;;
        * ) break ;;
    esac
    done

    if [[ ${TEST_NAME} == "" ]]
    then
        echo "----------------------------------------------------------------------------------"
        echo "No test name defined"
        print_usage 1
    fi

    if [[ ${COMPOSE_FILE} == "" ]]
    then
        echo "----------------------------------------------------------------------------------"
        echo "No Docker compose file provided"
        print_usage 1
    fi

    if [[ ! -f "${COMPOSE_FILE}" ]]
    then
        echo "----------------------------------------------------------------------------------"
        echo "-f  --compose-file must specify an existing file"
        print_usage 1
    fi
}

full_path ()
{
    if [[ -f ${1} ]]
    then
        echo "$(realpath ${1})"
    else
        echo "$(dirname ${1})"
    fi
}

main ()
{
    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
    parse_options ${@}
    EXIT_CODE=0

    set -e

    echo "Docker compose file: ${COMPOSE_FILE}"

    docker-compose -f ${COMPOSE_FILE} up

    EXIT_CODE=$(docker-compose -f ${COMPOSE_FILE} ps -q |
        xargs docker inspect -f '{{ .State.ExitCode }}' |
        grep -vx "^0$" | wc -l | tr -d ' ')

    echo "${TEST_NAME} exited with code ${EXIT_CODE}"

    exit ${EXIT_CODE}
}

main ${@}
