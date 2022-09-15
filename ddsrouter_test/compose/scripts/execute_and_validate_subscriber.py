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

import argparse
import re

import log
import validation

DESCRIPTION = """Script to validate subscribers output"""
USAGE = ('python3 execute_and_validate_subscriber.py '
         '-e <path/to/application/executable> '
         '[-s <samples>] [-t <timeout>] [--domain 1] '
         '[--debug] [--allow-duplicates]')


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
        help='Path to discovery-server executable.'
    )
    parser.add_argument(
        '-s',
        '--samples',
        type=int,
        default=5,
        help='Samples to receive.'
    )
    parser.add_argument(
        '-t',
        '--timeout',
        type=int,
        default=5,
        help='Timeout for the subscriber application.'
    )
    parser.add_argument(
        '--allow-duplicates',
        action='store_true',
        help='Allow receive duplicated data.'
    )
    parser.add_argument(
        '--debug',
        action='store_true',
        help='Print test debugging info.'
    )
    parser.add_argument(
        '--domain',
        type=int,
        default=0,
        help='Domain to execute the subscriber.'
    )

    return parser.parse_args()


def _subscriber_command(args):
    """
    Build the command to execute the subscriber.

    :param args: Arguments parsed
    :return: Command to execute the subscriber
    """
    command = [
        args.exe,
        'subscriber',
        '-s', str(args.samples),
        '-d', str(args.domain),
        '--transport', 'udp']

    return command


def _subscriber_parse_output(stdout, stderr):
    """
    Transform the output of the program in a list of received messages.

    :param stdout: Process stdout
    :param stdout: Process stderr

    :return: (List of received messages , stderr)
    """
    regex = re.compile('^Message\sHelloWorld\s+\d+\sRECEIVED$')
    lines = stdout.splitlines()
    filtered_data = [line for line in lines if regex.match(line)]

    return filtered_data, stderr


def _subscriber_validate_duplicates(stdout_parsed, stderr_parsed):
    if (validation.find_duplicates(stdout_parsed)):
        return validation.ReturnCode.DUPLICATES
    else:
        return validation.validate_default(stdout_parsed, stderr_parsed)


if __name__ == '__main__':

    # Parse arguments
    args = parse_options()

    # Set log level
    if args.debug:
        log.activate_debug()

    command = _subscriber_command(args)

    subscriber_validate_function = None
    if args.allow_duplicates:
        subscriber_validate_function = _subscriber_validate_duplicates
    else:
        subscriber_validate_function = validation.validate_default

    ret_code = validation.run_and_validate(
        command=command,
        timeout=args.timeout,
        parse_output_function=_subscriber_parse_output,
        validate_output_function=subscriber_validate_function)

    print(f'Subscriber validator exited with code {ret_code}')

    exit(ret_code.value)
