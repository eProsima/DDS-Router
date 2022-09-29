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

import log

import validation

DESCRIPTION = """Script to validate servers output"""
USAGE = ('python3 execute_and_validate_server.py '
         '[-s <samples>] [-t <timeout>] [-d]')


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
        help='Timeout for the server application.'
    )
    parser.add_argument(
        '-e',
        '--exe',
        type=str,
        default='/scripts/ros2_nodes/node_main.py',
        help='Timeout for the server application.'
    )
    parser.add_argument(
        '--delay',
        type=float,
        default=0,
        help='Time to wait before starting execution.'
    )
    parser.add_argument(
        '-d',
        '--debug',
        action='store_true',
        help='Print test debugging info.'
    )

    return parser.parse_args()


def _server_command(args):
    """
    Build the command to execute the server.

    :param args: Arguments parsed
    :return: Command to execute the server
    """
    command = [
        'python3', args.exe,
        '--samples', str(args.samples)]

    return command


if __name__ == '__main__':

    # Parse arguments
    args = parse_options()

    # Set log level
    if args.debug:
        log.activate_debug()

    command = _server_command(args)

    ret_code = validation.run_and_validate(
        command=command,
        timeout=args.timeout,
        delay=args.delay,
        parse_output_function=validation.parse_default,
        validate_output_function=validation.validate_default)

    log.logger.info(f'Server validator exited with code {ret_code}')

    exit(ret_code.value)
