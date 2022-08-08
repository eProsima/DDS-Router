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
import logging
import re
import signal
import subprocess

DESCRIPTION = """Script to validate subscribers output"""
USAGE = ('python3 validate_subscriber.py -e <path/to/application/executable>'
         ' [-d]')

PIPE = subprocess.PIPE
STDOUT = subprocess.STDOUT
DEVNULL = subprocess.DEVNULL


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
        '-d',
        '--debug',
        action='store_true',
        help='Print test debugging info.'
    )

    return parser.parse_args()


def run(command, timeout):
    """
    Run command with timeout.

    :param command: Command to run in list format
    :param timeout: Timeout for the process
    :return:
        - ret_code - The process exit code
        - stdout - Output of the process
        - stderr - Error output of the process
    """
    ret_code = 0

    proc = subprocess.Popen(command,
                            stdout=subprocess.PIPE,
                            universal_newlines=True)
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        logger.info('Timeout expired. '
                    'Killing subscriber before receiving all samples...')
        ret_code = 1
        proc.send_signal(signal.SIGINT)

    stdout, stderr = proc.communicate()

    return (ret_code, stdout, stderr)


def parse_output(data):
    """
    Transform the output of the program in a list of received messages.

    :param data: Process stdout
    :return: List of received messages
    """
    regex = re.compile('^Message\sHelloWorld\s+\d+\sRECEIVED$')
    lines = data.splitlines()
    filtered_data = [line for line in lines if regex.match(line)]

    return filtered_data


def find_duplicates(data):
    """
    Find duplicates in a list os strings.

    :param data: List of strings
    :return: List of tuples with the index of the duplicated strings
    """
    duplicates = []
    lines_seen = {}

    for idx, line in enumerate(data):
        if line not in lines_seen:
            lines_seen[line] = idx
        else:
            duplicates.append((lines_seen[line], idx))

    if duplicates:
        logger.info('Found duplicated messages')
    else:
        logger.debug('None duplicated messages found')

    return duplicates


def validate(command, samples, timeout, allow_duplicates=False):
    """
    Validate the output of a subscriber run.

    :param command: Command to run in list format
    :param samples: Number of samples to receive
    :param timeout: Timeout for the process
    :param allow_duplicates: Allow duplicated messages (Default: False)
    :return: The exit code
    """
    ret_code, stdout, stderr = run(command, timeout)

    if ret_code:
        logger.error('Subscriber application exited with '
                     f'return code {ret_code}')

        logger.error(f'Subscriber output: \n {stdout}')
        return ret_code
    else:
        data_received = parse_output(stdout)

        logger.debug(f'Subscriber received... \n {data_received}')

        if not allow_duplicates:
            return len(find_duplicates(data_received))

        if len(data_received) < samples:
            logger.error(f'Number of messages received: {len(data_received)}. '
                         f'Expected {samples}')
            return 1

    return 0


if __name__ == '__main__':
    # Parse arguments
    args = parse_options()

    # Create a custom logger
    logger = logging.getLogger('VALIDATION')
    # Create handlers
    l_handler = logging.StreamHandler()
    # Create formatters and add it to handlers
    l_format = '[%(asctime)s][%(name)s][%(levelname)s] %(message)s'
    l_format = logging.Formatter(l_format)
    l_handler.setFormatter(l_format)
    # Add handlers to the logger
    logger.addHandler(l_handler)
    # Set log level
    if args.debug:
        logger.setLevel(logging.DEBUG)
    else:
        logger.setLevel(logging.INFO)

    command = [args.exe, 'subscriber', '-s', str(args.samples)]

    ret_code = validate(command,
                        args.samples,
                        args.timeout,
                        args.allow_duplicates)

    print(f'Subscriber validator exited with code {ret_code}')

    exit(ret_code)
