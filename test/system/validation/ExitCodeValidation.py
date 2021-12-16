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
Script implementing the ExitCodeValidation class.

The ExitCodeValidation validates the test exit code
"""
import shared.shared as shared

import validation.Validator as validator


class ExitCodeValidation(validator.Validator):
    """
    Class to validate an snapshot resulting from a Discovery-Server test.

    Validate the test counting and comparing the number of lines of the
    output snapshot resulted from the test execution and an a priori well
    known output.
    """

    def _validator_tag(self):
        """Return validator's tag in json parameters file."""
        return 'exit_code_validation'

    def _validate(self):
        """Validate the test exit code"""

        try:
            exit_code = self.validator_input_.exit_code
            expected_code = self.validation_params_['expected_exit_code']

        except KeyError as e:
            self.logger.error(e)
            return shared.ReturnCode.ERROR

        self.logger.debug(f'ExitCodeValidation: process exit code:'
                          f'{exit_code}, expected code:'
                          f'{expected_code}')

        val = (expected_code == exit_code)

        return shared.ReturnCode.OK if val else shared.ReturnCode.FAIL
