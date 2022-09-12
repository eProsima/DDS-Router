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
import time
from enum import Enum

DESCRIPTION = """Script to validate subscribers output"""
USAGE = ('python3 validate_subscriber.py -e <path/to/application/executable>')

PIPE = subprocess.PIPE
STDOUT = subprocess.STDOUT
DEVNULL = subprocess.DEVNULL


class ReturnCode(Enum):
    """Enumeration for return codes of this script."""

    SUCCESS = 0
    TIMEOUT = 1
    HARD_TIMEOUT = 2
    DUPLICATES = 3
    MISSING_MESSAGES = 4
    NOT_VALID_MESSAGES = 5


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
        '--args',
        type=str,
        default="",
        help='Arguments for executable .'
    )
    parser.add_argument(
        '-s',
        '--samples',
        type=int,
        default=5,
        help='Samples to receive.'
    )
    parser.add_argument(
        '--timeout',
        type=int,
        default=10,
        help='Time before killing process.'
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
        '--transient',
        action='store_true',
        help='Transient Local Subscriber, so it must receive data from 0.'
    )
    parser.add_argument(
        '--delay',
        type=int,
        default=0,
        help='Time to wait before executing the command.'
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
    ret_code = ReturnCode.SUCCESS

    proc = subprocess.Popen(command,
                            stdout=subprocess.PIPE,
                            universal_newlines=True)
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        logger.error('Timeout expired. '
                     'Killing subscriber before receiving all samples...')
        proc.send_signal(signal.SIGINT)
        ret_code = ReturnCode.TIMEOUT

    # Check whether SIGINT was able to terminate the process
    if proc.poll() is None:
        # SIGINT couldn't terminate the process
        logger.error('SIGINT could not kill process. '
                     'Killing subscriber hardly...')
        proc.kill()
        ret_code = ReturnCode.HARD_TIMEOUT

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


def check_transient(data):
    """
    Check that messages go from 0 to N without gaps.

    :param data: List of strings
    :return: True if transient has been fulfilled, false in case on error
    """
    # Convert every line into just the number
    ini_str_size = len("Message HelloWorld  ")
    end_str_size = len(" RECEIVED")
    numbers_received = [
        line[ini_str_size:-end_str_size]
        for line in data]

    # NOTE: idx starts in 0 and messages start in 1
    for idx, number in enumerate(numbers_received):
        if (idx + 1) != int(number):
            logger.info(
                f'Message received in position {idx+1} is {number}')
            return False

    logger.debug('All messages received and in correct order.')
    return True


def validate(
        command,
        samples,
        timeout,
        allow_duplicates=False,
        transient=False):
    """
    Validate the output of a subscriber run.

    :param command: Command to run in list format
    :param samples: Number of samples to receive
    :param timeout: Timeout for the process
    :param allow_duplicates: Allow duplicated messages (Default: False)
    :return: The exit code
    """
    logger.info(f'Executing command: {command}')

    ret_code, stdout, stderr = run(command, timeout)

    if ret_code != ReturnCode.SUCCESS:
        logger.error('Subscriber application exited with '
                     f'return code {ret_code}')

        logger.error(f'Subscriber output: \n {stdout}')
        logger.error(f'Subscriber stderr output: \n {stderr}')

        return ret_code

    else:

        logger.debug(f'Subscriber output: \n {stdout}')
        logger.debug(f'Subscriber stderr output: \n {stderr}')

        data_received = parse_output(stdout)

        logger.debug(f'Subscriber received... \n {data_received}')

        if not allow_duplicates:
            if find_duplicates(data_received):
                logger.error('Duplicated messages found')
                return ReturnCode.DUPLICATES

        if transient:
            if not check_transient(data_received):
                logger.error('Transient messages incorrect reception.')
                return ReturnCode.MISSING_MESSAGES

        if len(data_received) < samples:
            logger.error(f'Number of messages received: {len(data_received)}. '
                         f'Expected {samples}')
            return ReturnCode.NOT_VALID_MESSAGES

    return ReturnCode.SUCCESS


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

    command = [
        args.exe,
        'subscriber'] + args.args.split(' ')

    # Wait for delay
    time.sleep(args.delay)

    ret_code = validate(command,
                        args.samples,
                        args.timeout,
                        args.allow_duplicates,
                        args.transient)

    logger.info(f'Subscriber validator exited with code {ret_code}')

    exit(ret_code.value)
