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

DESCRIPTION = """Script to validate listeners output"""
USAGE = ('python3 execute_and_validate_listener.py '
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
        '--allow-duplicates',
        action='store_true',
        help='Allow receive duplicated data.'
    )
    parser.add_argument(
        '-d',
        '--debug',
        action='store_true',
        help='Print test debugging info.'
    )

    return parser.parse_args()


def _listener_command(args):
    """
    Build the command to execute the listener.

    :param args: Arguments parsed
    :return: Command to execute the listener
    """
    command = [
        'python3',
        '/opt/ros/humble/lib/demo_nodes_py/listener']

    return command


def _listener_parse_output(stdout, stderr):
    """
    Parse message and get only the numbers received.
    """
    head_msg_expected = '[INFO] [1664186953.395023916] [listener]: I heard: [Hello World: '
    head_len = len(head_msg_expected)
    tail_msg_expected = ']'
    tail_len = len(tail_msg_expected)
    inner_msg_expected = '[listener]: I heard: [Hello World:'

    lines = stderr.splitlines()  # INFO traces are printed to stderr
    stdout_lines = []
    stderr_lines = []
    for line in lines:
        if inner_msg_expected in line:
            stdout_lines.append(line[head_len:-tail_len])
        else:
            stderr_lines.append(line)

    return stdout_lines, stderr_lines


def _listener_validate_duplicates(stdout_parsed, stderr_parsed):
    """
    Do nothing.

    Dummy method as listener will not validate anything.
    """
    ret = validation.validate_default(stdout_parsed, stderr_parsed)
    if (ret == validation.ReturnCode.SUCCESS):
        if (validation.find_duplicates(stdout_parsed)):
            return validation.ReturnCode.DUPLICATES
    return ret


if __name__ == '__main__':

    # Parse arguments
    args = parse_options()

    # Set log level
    if args.debug:
        log.activate_debug()

    # Prepare command
    command = _listener_command(args)

    _listener_validate_function = None
    if args.allow_duplicates:
        _listener_validate_function = _listener_validate_duplicates
    else:
        _listener_validate_function = validation.validate_default

    # Run command and validate
    ret_code = validation.run_and_validate(
        command=command,
        timeout=args.timeout,
        delay=args.delay,
        parse_output_function=_listener_parse_output,
        validate_output_function=_listener_validate_function,
        timeout_as_error=False)

    print(f'listener validator exited with code {ret_code}')

    exit(ret_code.value)
