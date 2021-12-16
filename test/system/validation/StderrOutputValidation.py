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
Script implementing the StderrOutputValidation class.

The StderrOutputValidation validates the test exit code
"""
import shared.shared as shared

import validation.Validator as validator


class StderrOutputValidation(validator.Validator):
    """
    Class to validate an snapshot resulting from a Discovery-Server test.

    Validate the test counting and comparing the number of lines of the
    output snapshot resulted from the test execution and an a priori well
    known output.
    """

    def _validator_tag(self):
        """Return validator's tag in json parameters file."""
        return 'stderr_validation'

    def _validate(self):
        """Validate the test stderr number of lines."""
        try:
            exit_lines_count = self.validator_input_.stderr_lines
            expected_lines = self.validation_params_['err_expected_lines']

        except KeyError as e:
            self.logger.error(e)
            return shared.ReturnCode.ERROR

        self.logger.debug(f'StderrOutputValidation: stderr process exit lines:'
                          f'{exit_lines_count}, expected lines:'
                          f'{expected_lines}')

        val = (exit_lines_count == expected_lines)

        return shared.ReturnCode.OK if val else shared.ReturnCode.FAIL
