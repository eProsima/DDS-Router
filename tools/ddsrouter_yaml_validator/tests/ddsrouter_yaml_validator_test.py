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

"""DDS-Router YAML Validator tests."""

import glob
import os

from ddsrouter_yaml_validator.validator import YamlValidator


# Move to project root level
os.chdir(os.path.dirname(os.path.abspath(__file__)) + '/../../..')

SCHEMA_PATH = 'tools/ddsrouter_yaml_validator/ddsrouter_yaml_validator/' \
              'ddsrouter_config_schema.json'

VALID_CONFIGURATION_FILES = [
    glob.glob('resources/configurations/examples/*.yaml'),
    glob.glob('docs/resources/getting_started/*.yaml')]

INVALID_CONFIGURATION_FILES = [
    glob.glob('tools/ddsrouter_yaml_validator/tests/'
              'invalid_configuration_files/*.yaml')]


def test_valid_yamls():
    """Assert given configuration files are valid."""
    validator = YamlValidator()
    for valid_files_list in VALID_CONFIGURATION_FILES:
        for valid_file in valid_files_list:
            assert validator.validate_config_file(
                valid_file, SCHEMA_PATH, logout=False)


def test_invalid_yamls():
    """Assert given configuration files are invalid."""
    validator = YamlValidator()
    for invalid_files_list in INVALID_CONFIGURATION_FILES:
        for invalid_file in invalid_files_list:
            assert not validator.validate_config_file(
                invalid_file, SCHEMA_PATH, logout=False)
