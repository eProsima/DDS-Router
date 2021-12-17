# Copyright 2020 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

    usage: test.py -e <binary_path> -c <config_file>

    binary_path: DDS Router binary path

"""

import argparse
import logging
import os
import signal
import subprocess
import sys
import time

DESCRIPTION = """Script to execute DDS Router executable test"""
USAGE = ('python3 tests.py -e <path/to/ddsrouter-executable>'
         ' [-t LIST[test-name]] [-d]')

# Sleep time to let process init and finish
SLEEP_TIME = 1
MAX_SIGNALS_SEND_ITERATIONS = 3

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
    required_args.add_argument(
        '-c',
        '--config-file',
        type=str,
        required=True,
        help='Configuration file path.'
    )
    parser.add_argument(
        '-d',
        '--debug',
        action='store_true',
        help='Print test debugging info.'
    )
    return parser.parse_args()


def test_ddsrouter_closure(ddsrouter, configuration_file):
    """Test that dsdrouter command closes correctly."""
    command = [ddsrouter, '-c', configuration_file]

    logger.info("Executing command: " + str(command))

    # this subprocess cannot be executed in shell=True or using bash
    #  because a background script will not broadcast the signals
    #  it receives
    proc = subprocess.Popen(command,
                            stdout=subprocess.PIPE,
                            universal_newlines=True
                           )

    # sleep to let the server run
    time.sleep(SLEEP_TIME)

    # Check whether the process has terminated already
    if not proc.poll() is None:
        # If the process has already exit means something has gone wrong.
        # Capture and print output for traceability and exit with code s1.
        output, err = proc.communicate()
        logger.error('Command ' + str(command) + ' failed before signal.')
        logger.debug('Command output:')
        logger.debug('STDOUT: "' + output + '"')
        logger.debug('STDERR: "' + err + '"')
        return 1

    # send SIGINT to process and wait for processing
    lease = 0
    while True:

        if os.name == 'posix':
            proc.send_signal(signal.SIGINT)
        elif os.name == 'nt':
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
        logger.error('Signal could not kill process')
        return 1

    output, err = proc.communicate()
    logger.info(
        'Command ' + str(command) + ' finished correctly')
    logger.debug('Command output:')
    logger.debug('STDOUT: "' + str(output) + '"')
    logger.debug('STDERR: "' + str(err) + '"')

    return 0


if __name__ == '__main__':

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

    sys.exit(test_ddsrouter_closure(args.exe, args.config_file))
