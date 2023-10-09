# Copyright 2023 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

import argparse
import re

import log
import validation

DESCRIPTION = """Script to validate the publishers' output"""
USAGE = ('python3 validate_publisher.py -e <path/to/application/executable>')


def parse_options():
    """
    Parse arguments.

    :return: The arguments parsed.
    """
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        add_help=True,
        description=(DESCRIPTION),
        usage=(USAGE)
    )

    required_args = parser.add_argument_group('required arguments')

    required_args.add_argument(
        '-e',
        '--exe',
        type=str,
        required=True,
        help='Path to publisher executable.'
    )

    parser.add_argument(
        '--args',
        type=str,
        default='',
        help='Arguments for executable.'
    )

    parser.add_argument(
        '--timeout',
        type=int,
        default=10,
        help='Time before killing the process.'
    )

    parser.add_argument(
        '--debug',
        action='store_true',
        help='Print test debugging info.'
    )

    parser.add_argument(
        '--delay',
        type=int,
        default=0,
        help='Time to wait before executing the command.'
    )

    parser.add_argument(
        '--disconnects',
        type=int,
        default=0,
        help='Number of times the other participant is expected to disconnect.'
    )

    return parser.parse_args()


def _publisher_command(args):
    """
    Build the command to execute the publisher.

    :param args: Arguments parsed
    :return: Command to execute the publisher
    """
    command = [
        args.exe,
        'publisher']

    command += args.args.split(' ')

    return command


def _publisher_parse_output(stdout, stderr):
    """
    Transform the output of the program in a list of received disconnects.

    :param data: Process stdout
    :return: List of subscribers who have disconnected
    """
    regex = re.compile(r'^Publisher unmatched \[.+\].$')
    lines = stdout.splitlines()
    filtered_data = [line for line in lines if regex.match(line)]

    return filtered_data, stderr


def _publisher_get_retcode_validate(
        disconnects):
    if disconnects != 0:
        return validation.validate_retcode_default

    accept_timeout = lambda retcode: retcode in [
        validation.ReturnCode.SUCCESS,
        validation.ReturnCode.TIMEOUT]

    return accept_timeout


def _publisher_validate(
        stdout_parsed,
        stderr_parsed,
        disconnects):

    # Check default validator
    ret_code = validation.validate_default(stdout_parsed, stderr_parsed)

    if len(stdout_parsed) != disconnects:
        log.logger.error(f'Number of disconnected receivers: \
                         {len(stdout_parsed)}. '
                         f'Expected {disconnects}')

        return validation.ReturnCode.NOT_VALID_DISCONNECTS

    return ret_code


if __name__ == '__main__':

    # Parse arguments
    args = parse_options()

    # Set log level
    if args.debug:
        log.activate_debug()

    command = _publisher_command(args)

    validate_func = (lambda stdout_parsed, stderr_parsed: (
        _publisher_validate(
            stdout_parsed,
            stderr_parsed,
            args.disconnects
            )))

    ret_code = validation.run_and_validate(
        command=command,
        timeout=args.timeout,
        delay=args.delay,
        parse_output_function=_publisher_parse_output,
        validate_output_function=validate_func,
        parse_retcode_function=_publisher_get_retcode_validate(
            args.disconnects),
        timeout_as_error=False)


    log.logger.info(f'Publisher validator exited with code {ret_code}')

    exit(ret_code.value)
