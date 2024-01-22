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

import signal
import subprocess
import time
from enum import Enum
from typing import List

import log

import utils


class ReturnCode(Enum):
    """Enumeration for return codes of this script."""

    SUCCESS = 0
    TIMEOUT = 1
    HARD_TIMEOUT = 2
    NOT_VALID_MESSAGES = 3
    COMMAND_FAIL = 4
    STDERR_OUTPUT = 5
    NOT_VALID_MATCHES = 6
    NOT_VALID_UNMATCHES = 7
    FINISHED_TOO_QUICKLY = 8
    FINISHED_TOO_SLOWLY = 9


"""
AUXILIARY GENERIC FUNCTIONS
"""


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
        log.logger.info('Found duplicated messages')
    else:
        log.logger.debug('None duplicated messages found')

    return duplicates


def validate_default(stdout_parsed, stderr_parsed) -> ReturnCode:
    """Validate any data as correct."""
    if stderr_parsed != '' and stderr_parsed != []:
        return ReturnCode.STDERR_OUTPUT
    else:
        return ReturnCode.SUCCESS


def parse_default(stdout, stderr) -> ReturnCode:
    """Return stdout and stderr as they are."""
    return (stdout, stderr)


def validate_retcode_default(retcode: ReturnCode) -> bool:
    """Validate that return code is SUCCESS."""
    return retcode == ReturnCode.SUCCESS


"""
RUN AND VALIDATE FUNCTIONS
"""


def run_command(
        command: 'list[str]',
        timeout: float,
        delay: float = 0,
        timeout_as_error: bool = True):
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

    # Delay
    utils.delay(delay)

    log.logger.debug(f'Running command: {command}')

    proc = subprocess.Popen(command,
                            stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE,
                            universal_newlines=True)

    try:
        proc.wait(timeout=timeout)

    except subprocess.TimeoutExpired:
        if timeout_as_error:
            log.logger.error(
                'Timeout expired. '
                'Killing process before receiving all samples...')
            proc.send_signal(signal.SIGINT)
            ret_code = ReturnCode.TIMEOUT

        else:
            proc.send_signal(signal.SIGINT)

    else:
        if not timeout_as_error:
            log.logger.error('Command finished before expected.')
            ret_code = ReturnCode.COMMAND_FAIL

        # Wait a minimum elapsed time to the signal to be received
        time.sleep(0.2)

    stdout, stderr = proc.communicate()

    # Check whether SIGINT was able to terminate the process
    if proc.poll() is None:
        # SIGINT couldn't terminate the process
        log.logger.error(
            'SIGINT could not kill process. '
            'Killing process hardly...')
        proc.kill()
        ret_code = ReturnCode.HARD_TIMEOUT

    if not stdout:
        stdout = ''
    if not stderr:
        stderr = ''

    return (ret_code, stdout, stderr)


def run_and_validate(
        command: List[str],
        timeout: int,
        parse_output_function,
        validate_output_function,
        delay: float = 0,
        timeout_as_error: bool = True,
        parse_retcode_function=validate_retcode_default,
        min_time: int = 0,
        max_time: int = 0):
    """
    Run the subscriber and validate its output.

    :param command: Command to run in list format
    :param timeout: Timeout for the process
    :param parse_output_function: Function to parse the output of the process
    :param validate_output_function: Function to validate the output of process
    :param timeout_as_error: Whether the timeout reach should be taken as error

    :return: exit code
    """
    starting_time = time.time()

    ret_code, stdout, stderr = run_command(
        command=command,
        timeout=timeout,
        delay=delay,
        timeout_as_error=timeout_as_error)

    finishing_time = time.time()

    elapsed_time = finishing_time - starting_time

    if elapsed_time < min_time:
        log.logger.error(
            'Executable exited before min-time.')

        return ReturnCode.FINISHED_TOO_QUICKLY

    elif max_time > 0 and elapsed_time > max_time:
        log.logger.error(
            'Executable exited after max-time.')

        return ReturnCode.FINISHED_TOO_SLOWLY

    elif not parse_retcode_function(ret_code):
        log.logger.error(
            f'Executable exited with '
            f'return code {ret_code}'
            f'\n stdout output: \n{stdout}'
            f'\n stderr output: \n{stderr}')

        return ret_code

    else:

        log.logger.debug(
            f'Executable execution output:'
            f'\n stdout output: \n{stdout}')

        if stderr != '':
            log.logger.warning(
                f'Executable execution output in stderr:'
                f'\n{stderr}')

        stdout_parsed, stderr_parsed = parse_output_function(stdout, stderr)

        validate_ret_code = validate_output_function(
            stdout_parsed,
            stderr_parsed)

        return validate_ret_code
