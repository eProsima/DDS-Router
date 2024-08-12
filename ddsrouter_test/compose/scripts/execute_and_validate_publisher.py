# Copyright 2023 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

DESCRIPTION = """Script to validate the publishers' output"""
USAGE = ('python3 execute_and_validate_publisher.py -e <path/to/application/executable>')


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
        help='Path to publisher executable.'
    )

    parser.add_argument(
        '--args',
        type=str,
        default='',
        help='Arguments for executable.'
    )

    parser.add_argument(
        '--timeout',
        type=int,
        default=10,
        help='Time before killing the process.'
    )

    parser.add_argument(
        '--debug',
        action='store_true',
        help='Print test debugging info.'
    )

    parser.add_argument(
        '--delay',
        type=int,
        default=0,
        help='Time to wait before executing the command.'
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


def _publisher_command(args):
    """
    Build the command to execute the publisher.

    :param args: Arguments parsed
    :return: Command to execute the publisher
    """
    command = [
        args.exe,
        'publisher']

    command += args.args.split(' ')

    return command


def _publisher_parse_output(stdout, stderr):
    """
    Transform the output to a list of received matches and unmatches.

    :param data: Process stdout
    :return: List of subscribers who have matched and unmatched
    """
    match_regex = re.compile(r'^Publisher matched.$')
    unmatch_regex = re.compile(r'^Publisher unmatched.$')

    filtered_data = {'matches': 0, 'unmatches': 0}

    for line in stdout.splitlines():
        if match_regex.match(line):
            filtered_data['matches'] += 1

        elif unmatch_regex.match(line):
            filtered_data['unmatches'] += 1

    return filtered_data, stderr


def _publisher_get_retcode_validate():
    return lambda retcode: retcode == validation.ReturnCode.SUCCESS or \
                           retcode == validation.ReturnCode.TIMEOUT


def _publisher_validate(
        stdout_parsed,
        stderr_parsed,
        n_matches,
        n_unmatches):

    # Check default validator
    ret_code = validation.validate_default(stdout_parsed, stderr_parsed)

    if n_matches is not None and stdout_parsed['matches'] != n_matches:
        log.logger.error(f'Number of matched receivers: \
                         {len(stdout_parsed)}. '
                         f'Expected {n_matches}')

        return validation.ReturnCode.NOT_VALID_MATCHES

    if n_unmatches is not None and stdout_parsed['unmatches'] != n_unmatches:
        log.logger.error(f'Number of unmatched receivers: \
                         {len(stdout_parsed)}. '
                         f'Expected {n_unmatches}')

        return validation.ReturnCode.NOT_VALID_UNMATCHES

    return ret_code


if __name__ == '__main__':

    # Parse arguments
    args = parse_options()

    # Set log level
    if args.debug:
        log.activate_debug()

    command = _publisher_command(args)

    validate_func = (lambda stdout_parsed, stderr_parsed: (
        _publisher_validate(
            stdout_parsed,
            stderr_parsed,
            args.n_matches,
            args.n_unmatches
            )))

    ret_code = validation.run_and_validate(
        command=command,
        timeout=args.timeout,
        delay=args.delay,
        parse_output_function=_publisher_parse_output,
        validate_output_function=validate_func,
        parse_retcode_function=_publisher_get_retcode_validate(),
        timeout_as_error=False)

    log.logger.info(f'Publisher validator exited with code {ret_code}')

    exit(ret_code.value)
