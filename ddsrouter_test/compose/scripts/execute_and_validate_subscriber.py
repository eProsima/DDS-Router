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
USAGE = ('python3 execute_and_validate_subscriber.py -e <path/to/application/executable>')


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
        help='Path to subscriber executable.'
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
        '--transient-local',
        action='store_true',
        help='Transient Local Subscriber, so it must receive data from 0.'
    )
    parser.add_argument(
        '--delay',
        type=int,
        default=0,
        help='Time to wait before executing the command.'
    )
    parser.add_argument(
        '--min-time',
        type=int,
        default=0,
        help='Minimum amount of seconds the command should take before finishing.'
    )
    parser.add_argument(
        '--max-time',
        type=int,
        default=0,
        help='Maximum amount of seconds the command should take before finishing.'
    )

    parser.add_argument(
        '--n-matches',
        type=int,
        help='Number of times the other participant is expected to match.'
    )

    parser.add_argument(
        '--n-unmatches',
        type=int,
        help='Number of times the other participant is expected to unmatch.'
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

    msgs_regex = re.compile(r"Sample: 'Configuration' with index: '\d+' \(\d+ Bytes\) RECEIVED")
    match_regex = re.compile(r'^Subscriber matched.$')
    unmatch_regex = re.compile(r'^Subscriber unmatched.$')

    filtered_data = {'messages': [], 'matches': 0, 'unmatches': 0}

    for line in stdout.splitlines():
        if msgs_regex.match(line):
            filtered_data['messages'].append(line)
        elif match_regex.match(line):
            filtered_data['matches'] += 1

        elif unmatch_regex.match(line):
            filtered_data['unmatches'] += 1

    return filtered_data, stderr


def _subscriber_get_retcode_validate(
        samples):
    if samples is None or samples == 0:
        return lambda retcode: retcode == validation.ReturnCode.SUCCESS or \
                               retcode == validation.ReturnCode.TIMEOUT

    return validation.validate_retcode_default


def _subscriber_validate(
        stdout_parsed,
        stderr_parsed,
        samples,
        duplicates_allow,
        transient,
        n_matches,
        n_unmatches):

    # Check default validator
    ret_code = validation.validate_default(stdout_parsed['messages'], stderr_parsed)

    if duplicates_allow != -1:
        duplicated_n = len(find_duplicates(stdout_parsed['messages']))
        if duplicated_n > duplicates_allow:
            log.logger.error(
                f'{duplicated_n} duplicated messages found. '
                f'Maximum allowed {duplicates_allow}.')
            return validation.ReturnCode.NOT_VALID_MESSAGES

    if transient:
        if not check_transient(stdout_parsed['messages']):
            log.logger.error('Transient messages incorrect reception.')
            return validation.ReturnCode.NOT_VALID_MESSAGES

    if samples is not None and len(stdout_parsed['messages']) != samples:
        log.logger.error(f'Number of messages received: {len(stdout_parsed["messages"])}. '
                         f'Expected {samples}')
        return validation.ReturnCode.NOT_VALID_MESSAGES

    if n_matches is not None and stdout_parsed['matches'] != n_matches:
        log.logger.error(f'Number of matched receivers: {stdout_parsed["matches"]}.'
                         f'Expected {n_matches}')

        return validation.ReturnCode.NOT_VALID_MATCHES

    if n_unmatches is not None and stdout_parsed['unmatches'] != n_unmatches:
        log.logger.error(f'Number of unmatched receivers: {stdout_parsed["unmatches"]}.'
                         f'Expected {n_unmatches}')

        return validation.ReturnCode.NOT_VALID_UNMATCHES

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
    msg_pattern = re.compile(r"'Configuration' with index: '(\d+)'")

    for idx, line in enumerate(data):
        match = msg_pattern.search(line)

        if not match:
            log.logger.warn(
                f'Message received in position {idx + 1} is not valid')
            return False

        message_id = int(match.group(1))

        if idx + 1 != message_id:
            log.logger.warn(
                f'Message received in position {idx + 1} is {message_id}')
            return False

    log.logger.debug('All messages received and in correct order.')
    return True


if __name__ == '__main__':

    # Parse arguments
    args = parse_options()

    # Set log level
    if args.debug:
        log.activate_debug()

    command = _subscriber_command(args)

    timeout_as_error = args.samples is not None and args.samples > 0

    validate_func = (lambda stdout_parsed, stderr_parsed: (
        _subscriber_validate(
            stdout_parsed=stdout_parsed,
            stderr_parsed=stderr_parsed,
            samples=args.samples,
            duplicates_allow=args.allow_duplicates,
            transient=args.transient_local,
            n_matches=args.n_matches,
            n_unmatches=args.n_unmatches
            )))

    ret_code = validation.run_and_validate(
        command=command,
        timeout=args.timeout,
        delay=args.delay,
        parse_output_function=_subscriber_parse_output,
        validate_output_function=validate_func,
        parse_retcode_function=_subscriber_get_retcode_validate(args.samples),
        timeout_as_error=timeout_as_error,
        min_time=args.min_time,
        max_time=args.max_time)

    log.logger.info(f'Subscriber validator exited with code {ret_code}')

    exit(ret_code.value)
