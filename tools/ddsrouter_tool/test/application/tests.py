# Copyright 2021 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
"""
Tests for the ddsrouter executable.

Contains a package of system test for ddsrouter tool

Usage: test.py -e <binary_path> -c <config_file_path>

Arguments:

    DDS Router binary path          : -e | --exe binary_path

    DDS Router binary path          : -c | --config-file <config_file_path>

    Run test in Debug mode          : -d | --debug

    Use SIGINT or SIGTERM           : -s | --signal sigint|sigterm
"""

import argparse
import logging
import os
import signal
import subprocess
import sys
import time
from enum import Enum

DESCRIPTION = """Script to execute DDS Router executable test"""
USAGE = ('python3 tests.py -e <path/to/ddsrouter-executable>'
         ' -c config_file_path --signal sigint [-d]')

# Sleep time to let process init and finish
SLEEP_TIME = 1
MAX_SIGNALS_SEND_ITERATIONS = 3


def signal_handler(signum, frame):
    """
    Ignore Signal handler.

    This method is required in Windows to not handle the signal that
    is sent to the subprocess.
    """
    pass


def executable_permission_value():
    """Return executable permissions value depending on the OS."""
    if os.name == 'nt':
        return os.X_OK  # windows
    else:
        return os.EX_OK


def file_exist_and_have_permissions(file_path):
    """Check if a file exists and have executable permissions."""
    if os.access(file_path, executable_permission_value()):
        return file_path
    else:
        return None


def is_linux():
    """Return whether the script is running in a Linux environment."""
    return os.name == 'posix'


def is_windows():
    """Return whether the script is running in a Windows environment."""
    return os.name == 'nt'


class KillingSignalType(Enum):
    """Enumeration for signals used to kill subprocesses."""

    KST_SIGINT = 2
    KST_SIGTERM = 15


def check_terminate_signal(st):
    """Return signal that must be used to kill process otherwise."""
    if st == 'sigterm':
        return KillingSignalType.KST_SIGTERM
    elif st == 'sigint':
        return KillingSignalType.KST_SIGINT
    else:
        raise argparse.ArgumentTypeError(
            f'Invalid value: {st}. It must be <sigint> or <sigterm>')


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
        type=file_exist_and_have_permissions,
        required=True,
        help='Path to discovery-server executable.'
    )
    required_args.add_argument(
        '-c',
        '--config-file',
        type=file_exist_and_have_permissions,
        required=True,
        help='Configuration file path.'
    )
    parser.add_argument(
        '-d',
        '--debug',
        action='store_true',
        help='Print test debugging info.'
    )
    parser.add_argument(
        '-s',
        '--signal',
        type=check_terminate_signal,
        required=True,
        help='<sigint>|<sigterm>: Use SIGINT or SIGTERM to kill process.'
    )
    return parser.parse_args()


def test_ddsrouter_closure(ddsrouter, configuration_file, killing_signal):
    """
    Test that ddsrouter command closes correctly.

    It creates a command line with the executable and the configuration
    file, executes the process and send it a signal after SLEEP_TIME sec.
    If the process has finished before the signal, test fails.
    If the process does not end after sending MAX_SIGNALS_SEND_ITERATIONS
    it is hard killed and the test fails.

    Parameters:
    ddsrouter (path): Path to ddsrouter binary executable
    configuration_file (path): Path to ddsrouter yaml configuration file
    use_sigint (KillingSignalType): Signal to kill subprocesses

    Returns:
    0 if okay, otherwise the return code of the command executed
    """
    command = [ddsrouter, '-c', configuration_file]

    logger.info('Executing command: ' + str(command))

    # this subprocess cannot be executed in shell=True or using bash
    #  because a background script will not broadcast the signals
    #  it receives
    proc = subprocess.Popen(command,
                            stdout=subprocess.PIPE,
                            universal_newlines=True)

    # sleep to let the server run
    time.sleep(SLEEP_TIME)

    # Check whether the process has terminated already
    if not proc.poll() is None:
        # If the process has already exit means something has gone wrong.
        # Capture and print output for traceability and exit with code s1.
        output, err = proc.communicate()
        logger.debug('-----------------------------------------------------')
        logger.error('Command ' + str(command) + ' failed before signal.')
        logger.debug('Command output:')
        logger.debug('Stdout: \n' + str(output))
        logger.debug('Stderr: \n' + str(err))
        logger.debug('-----------------------------------------------------')
        return 1

    # direct this script to ignore SIGINT in case of windows
    if is_windows():
        signal.signal(signal.SIGINT, signal_handler)

    # send signal to process and wait for it to be killed
    lease = 0
    while True:

        if killing_signal == KillingSignalType.KST_SIGTERM:
            # Use SIGTERM instead
            if is_linux():
                proc.send_signal(signal.SIGTERM)
            elif is_windows():
                proc.send_signal(signal.CTRL_BREAK_EVENT)
        else:
            # Use SIGINT (use by default if not SIGTERM)
            if is_linux():
                proc.send_signal(signal.SIGINT)
            elif is_windows():
                proc.send_signal(signal.CTRL_C_EVENT)

        time.sleep(SLEEP_TIME)
        lease += 1

        # Break when signal kills the process or it hangs
        if proc.poll() is None and lease < MAX_SIGNALS_SEND_ITERATIONS:
            logger.debug('Sending signal again. Iterating...')
        else:
            break

    # Check whether SIGINT was able to terminate the process
    if proc.poll() is None:
        # SIGINT couldn't terminate the process. Kill it and exit with code 2
        proc.kill()
        output, err = proc.communicate()
        logger.debug('-----------------------------------------------------')
        logger.debug('Internal DDS Router output:')
        logger.debug('Stdout: \n' + str(output))
        logger.debug('Stderr: \n' + str(err))
        logger.error('Signal could not kill process')
        logger.debug('-----------------------------------------------------')
        return 1

    output, err = proc.communicate()
    logger.debug('-----------------------------------------------------')
    logger.info(
        'Command ' + str(command) + ' finished correctly')
    logger.debug('Command output:')
    logger.debug('Stdout: \n' + str(output))
    logger.debug('Stderr: \n' + str(err))
    logger.debug('-----------------------------------------------------')

    return 0


if __name__ == '__main__':

    args = parse_options()

    # Create a custom logger
    logger = logging.getLogger('SYS_TEST')
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

    if args.exe is None:
        logger.error(
            'Executable binary file does not exist or has no '
            'executable permissions.')
        sys.exit(1)

    if args.config_file is None:
        logger.error(
            'Configuration file does not exist or has no '
            'executable permissions.')
        sys.exit(1)

    sys.exit(
        test_ddsrouter_closure(
            args.exe,           # Path to executable
            args.config_file,   # Configuration file
            args.signal))       # Signal to kill subprocess
