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

DESCRIPTION = """Script to validate talkers output"""
USAGE = ('python3 execute_and_validate_talker.py '
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
        '-t',
        '--timeout',
        type=int,
        default=5,
        help='Timeout for the subscriber application.'
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


def _talker_command(args):
    """
    Build the command to execute the talker.

    :param args: Arguments parsed
    :return: Command to execute the talker
    """
    command = [
        'python3',
        '/opt/ros/humble/lib/demo_nodes_py/talker']

    return command


def _talker_parse_output(stdout, stderr):
    """
    Do nothing.

    Dummy method as talker will not parse output.
    """
    return stdout, stderr


def _talker_validate(stdout_parsed, stderr_parsed):
    """
    Do nothing.

    Dummy method as talker will not validate anything.
    """
    return validation.ReturnCode.SUCCESS


if __name__ == '__main__':

    # Parse arguments
    args = parse_options()

    # Set log level
    if args.debug:
        log.activate_debug()

    # Prepare command
    command = _talker_command(args)

    # Run command and validate
    ret_code = validation.run_and_validate(
        command=command,
        timeout=args.timeout,
        delay=args.delay,
        parse_output_function=_talker_parse_output,
        validate_output_function=_talker_validate,
        timeout_as_error=False)

    print(f'talker validator exited with code {ret_code}')

    exit(ret_code.value)
