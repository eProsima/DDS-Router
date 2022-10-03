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

DESCRIPTION = """Script to validate clients output"""
USAGE = ('python3 execute_and_validate_client.py '
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
        help='Timeout for the client application.'
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


def _client_command(args):
    """
    Build the command to execute the client.

    :param args: Arguments parsed
    :return: Command to execute the client
    """
    command = [
        'python3', args.exe,
        '--client',
        '--samples', str(args.samples),
        '--wait']

    return command


def _client_parse_output(stdout, stderr):
    """
    Transform the output of the program in a list of received messages.

    :param stdout: Process stdout
    :param stdout: Process stderr

    :return: (List of received messages , stderr)
    """
    head_message_expected = 'Result { '

    lines = stdout.splitlines()

    # Get only lines of format "Result { x,y,z }
    filtered_lines = [
        line.split('$')[-1][len(head_message_expected):-2]
        for line
        in lines
        if head_message_expected in line]

    executions = []
    for line in filtered_lines:
        line = line.split(',')
        executions.append((int(line[0]), int(line[1]), int(line[2])))

    return executions, stderr


def _client_validate(stdout_parsed, stderr_parsed):

    # Check default validator
    ret_code = validation.validate_default(stdout_parsed, stderr_parsed)

    if ret_code != validation.ReturnCode.SUCCESS:
        return ret_code

    # Check that responses are ok
    for execution in stdout_parsed:
        if (execution[0] + execution[1]) != execution[2]:
            return validation.ReturnCode.NOT_VALID_MESSAGES

    return ret_code


if __name__ == '__main__':

    # Parse arguments
    args = parse_options()

    # Set log level
    if args.debug:
        log.activate_debug()

    command = _client_command(args)

    ret_code = validation.run_and_validate(
        command=command,
        timeout=args.timeout,
        delay=args.delay,
        parse_output_function=_client_parse_output,
        validate_output_function=_client_validate)

    log.logger.info(f'Client validator exited with code {ret_code}')

    exit(ret_code.value)
