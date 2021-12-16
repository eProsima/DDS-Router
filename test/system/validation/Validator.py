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
"""Script implementing the Validator class."""
import logging
from pathlib import Path

import shared.shared as shared

import xmltodict


class Validator(object):
    """
    Class to validate an snapshot resulting from a Discovery-Server test.

    This is the base class of various classes that implement a different type
    of test output validation. Each of the child classes will implement its
    validation mechanism in the _validate() function.
    """

    def __init__(
        self,
        validator_input,
        validation_params,
        debug=False,
        logger=None
    ):
        """
        Build a generic validation object.

        :param validator_input: storage of process execution information.
        :param validation_params: validation parameters.
        :param debug: True/False to activate/deactivate debug logger.
        :param logger: The logging object. VALIDATION if None
            logger is provided.
        """
        self.set_logger(logger, debug)
        self.logger.debug(f'Creating an instance of {self.name()}')
        self.validator_input_ = validator_input
        self.validation_params_ = validation_params

    def set_logger(self, logger, debug):
        """
        Instance the class logger.

        :param logger: The logging object. VALIDATION if None
            logger is provided.
        :param debug: True/False to activate/deactivate debug logger.
        """
        if isinstance(logger, logging.Logger):
            self.logger = logger
        else:
            if isinstance(logger, str):
                self.logger = logging.getLogger(logger)
            else:
                self.logger = logging.getLogger('VALIDATION')

            if not self.logger.hasHandlers():
                l_handler = logging.StreamHandler()
                l_format = '[%(asctime)s][%(name)s][%(levelname)s] %(message)s'
                l_format = logging.Formatter(l_format)
                l_handler.setFormatter(l_format)
                self.logger.addHandler(l_handler)

        if debug:
            self.logger.setLevel(logging.DEBUG)
        else:
            self.logger.setLevel(logging.INFO)

    def parse_xml_snapshot(
        self,
        xml_file_path
    ):
        """
        Read an xml snapshot file and convert it into a dictionary.

        :param xml_file_path: The path to the xml snapshot.

        :return: snapshot in Python dictionaty.
        """
        if (shared.get_file_extension(xml_file_path)
                not in ['.snapshot', '.snapshot~']):
            raise ValueError(
                f'The snapshot file \"{xml_file_path}\" '
                'is not an .snapshot file')
        xml_file_path = Path(xml_file_path).resolve()
        valid_path, xml_file = shared.is_valid_path(xml_file_path)
        if not valid_path:
            raise ValueError(f'NOT valid snapshot path: {xml_file}')

        with open(xml_file_path) as xml_file:
            snapshot_dict = xmltodict.parse(xml_file.read())

        return snapshot_dict

    def validate(self):
        """General validation function.

        It checks if this validator must be used with these parameters.
        In positive case, it will execute the specific subclass validate
        function and return the result. In negative case, it will
        return SKIP.

        :return: ReturnCode depending the validation process.
        """
        # Check whether this validator tag is in parameters
        if self._validator_tag() not in self.validation_params_.keys():
            # If there si not the tag in parameters test is skipped
            res = shared.ReturnCode.SKIP
        else:
            self.validation_params_ = \
                self.validation_params_[self._validator_tag()]
            # Execute validation
            res = self._validate()

        # Print result
        color = shared.bcolors.FAIL
        if res == shared.ReturnCode.SKIP:
            color = shared.bcolors.WARNING
        elif res == shared.ReturnCode.OK:
            color = shared.bcolors.OK

        self.logger.debug(
                f'Result of {self.name()}: '
                f'{color}{res.name}{shared.bcolors.ENDC}')

        return res

    def _validate(self):
        """Implement the specific validation.

        Depending on the validator, this method implements
        the different validation possibilities.

        :return: ReturnCode depending the validation process.
        """
        pass

    def name(self):
        """Return validator's name."""
        return type(self).__name__

    def _validator_tag(self):
        """Return validator's tag in json file."""
        pass
