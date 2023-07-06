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
USAGE = ('python3 validate_subscriber.py -e <path/to/application/executable>')


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
        default='',
        help='Arguments for executable.'
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
        type=int,
        default=0,
        help=('Allow receive N first messages duplicated. '
              '(N=0 => none can be duplicated) '
              '(N=-1 => all can be duplicated)[Default = 0]')
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


def _subscriber_command(args):
    """
    Build the command to execute the subscriber.

    :param args: Arguments parsed
    :return: Command to execute the subscriber
    """
    command = [
        args.exe,
        'subscriber']

    command += args.args.split(' ')

    return command


def _subscriber_parse_output(stdout, stderr):
    """
    Transform the output of the program in a list of received messages.

    :param data: Process stdout
    :return: List of received messages
    """
    regex = re.compile(r'^Message\sHelloWorld\s+\d+\sRECEIVED$')
    lines = stdout.splitlines()
    filtered_data = [line for line in lines if regex.match(line)]

    return filtered_data, stderr


def _subscriber_get_retcode_validate(
        samples):
    if samples == 0:
        def accept_timeout(retcode):
            return (
                retcode == validation.ReturnCode.SUCCESS
                or retcode == validation.ReturnCode.TIMEOUT)
        return accept_timeout
    else:
        return validation.validate_retcode_default


def _subscriber_validate(
        stdout_parsed,
        stderr_parsed,
        samples,
        duplicates_allow,
        transient):

    # Check default validator
    ret_code = validation.validate_default(stdout_parsed, stderr_parsed)

    if duplicates_allow != -1:
        duplicated_n = len(find_duplicates(stdout_parsed))
        if duplicated_n > duplicates_allow:
            log.logger.error(
                f'{duplicated_n} duplicated messages found. '
                f'Maximum allowed {duplicates_allow}.')
            return validation.ReturnCode.NOT_VALID_MESSAGES

    if transient:
        if not check_transient(stdout_parsed):
            log.logger.error('Transient messages incorrect reception.')
            return validation.ReturnCode.NOT_VALID_MESSAGES

    if len(stdout_parsed) != samples:
        log.logger.error(f'Number of messages received: {len(stdout_parsed)}. '
                         f'Expected {samples}')
        return validation.ReturnCode.NOT_VALID_MESSAGES

    return ret_code


def find_duplicates(data):
    """
    Find duplicates in a list os strings.

    :param data: List of strings
    :return: List of tuples with the index of the duplicated strings
    """
    # TODO: add more logic so duplicated allow are only first messages
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


def check_transient(data):
    """
    Check that messages go from 0 to N without gaps.

    :param data: List of strings
    :return: True if transient has been fulfilled, false in case on error
    """
    # Convert every line into just the number
    ini_str_size = len('Message HelloWorld  ')
    end_str_size = len(' RECEIVED')
    numbers_received = [
        line[ini_str_size:-end_str_size]
        for line in data]

    # NOTE: idx starts in 0 and messages start in 1
    for idx, number in enumerate(numbers_received):
        if (idx + 1) != int(number):
            log.logger.warn(
                f'Message received in position {idx+1} is {number}')
            return False

    log.logger.debug('All messages received and in correct order.')
    return True


if __name__ == '__main__':

    # Parse arguments
    args = parse_options()

    # Set log level
    # if args.debug:
    if True:
        log.activate_debug()

    command = _subscriber_command(args)

    validate_func = (lambda stdout_parsed, stderr_parsed: (
        _subscriber_validate(
            stdout_parsed=stdout_parsed,
            stderr_parsed=stderr_parsed,
            samples=args.samples,
            duplicates_allow=args.allow_duplicates,
            transient=args.transient
            )))

    ret_code = validation.run_and_validate(
        command=command,
        timeout=args.timeout,
        delay=args.delay,
        parse_output_function=_subscriber_parse_output,
        validate_output_function=validate_func,
        parse_retcode_function=_subscriber_get_retcode_validate(args.samples))

    log.logger.info(f'Subscriber validator exited with code {ret_code}')

    exit(ret_code.value)
