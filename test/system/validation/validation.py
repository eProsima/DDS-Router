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
Script implementing the validation-run_test communicate module.

In order to do run_test.py script and validation module works independently,
this script encapsulates the different methods needed from one to the other.
"""
import shared.shared as shared

import validation.CountLinesValidator as clv
import validation.ExitCodeValidation as ecv
import validation.GenerateValidator as genv
import validation.GroundTruthValidator as gtv
import validation.StderrOutputValidation as sov


class ValidatorInput(object):
    """Encapsulate Validator input."""

    def __init__(
        self,
        exit_code=None,
        stderr_lines=None,
        result_file=None
    ):
        """
        Construct Validator Input with fields.

        :param exit_code: exit code from process execution (OK = 0).
        :param stderr_lines: number of stderr lines in process execution.
        :param result_file: output file from process execution.
        """
        self.exit_code = exit_code
        self.stderr_lines = stderr_lines
        self.result_file = result_file
        # OBSERVATION: Add here more fields from process execution
        # in case a new validator requires new info


def get_validators():
    """Get a list of validators implemented."""
    return [
        ecv.ExitCodeValidation,
        clv.CountLinesValidator,
        genv.GenerateValidator,
        gtv.GroundTruthValidator,
        sov.StderrOutputValidation
        # OBSERVATION: Add here the validator constructor for
        # adding a new validator
    ]


def validate_test(
    process_id,
    validation_params,
    validator_input,
    debug=False,
    logger=None
):
    """Validate thread execution.

    It creates all the implemented validators. For those which are in
    validation paramenters check wheter the validator
    input given passes that specific validator. Skip otherwise.

    :param process_id: process unique id.
    :param validation_params: validation parameters.
    :param validator_input: input for validator with fields needed.
    :param debug: Debug flag (Default: False).
    :param logger: logger object.

    :return: True if all validators pass or skiped, false otherwise.
    """

    # Get list of validator constructors
    validators = get_validators()
    final_result = True

    for validator_constructor in validators:
        # Construct each validator
        validator = validator_constructor(
            validator_input,
            validation_params,
            debug)

        # For each validator validates process execution
        result = validator.validate()
        print_result(
            logger,
            f'Result of test {process_id} for validator {validator.name()}',
            result
        )

        # Final result is PASSED if test is OK or SKIP
        if result != shared.ReturnCode.OK and result != shared.ReturnCode.SKIP:
            final_result = False

    return final_result


def print_result(logger, msg, res):
    """Print test result with specific color depending result.

    Print a message in a color depending the result code given. Yellow for
    skip case, Red for fail or error case and default color otherwise.

    :param logger: logger object to print the result
    :param msg: message to print
    :param res: ReturnCode value
    """
    if (type(res) is bool):
        print_result_bool(logger, msg, res)

    color = shared.bcolors.FAIL
    log_func = logger.info

    if res == shared.ReturnCode.SKIP:
        color = shared.bcolors.WARNING
        log_func = logger.debug
    elif res == shared.ReturnCode.OK:
        color = shared.bcolors.OK
    elif res == shared.ReturnCode.ERROR:
        log_func = logger.error

    log_func(
        f'{msg}: '
        f'{color}{res.name}{shared.bcolors.ENDC}')


def print_result_bool(logger, msg, res):
    """Print test result with specific color depending result.

    Print a message in a color depending the result code given. Red if false
    and default color otherwise.

    :param logger: logger object to print the result
    :param msg: message to print
    :param res: boolean value
    """
    if res:
        color = shared.bcolors.OK
        logger.info(
            f'{msg}: '
            f'{color}PASS{shared.bcolors.ENDC}')
    else:
        color = shared.bcolors.FAIL
        logger.info(
            f'{msg}: '
            f'{color}FAIL{shared.bcolors.ENDC}')


def validation_file_name(test_name):
    """Std name for a result snapshot."""
    return test_name + '.snapshot~'
