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

import os

from ddsrouter_yaml_validator.validator import YamlValidator


# Move to project root level
os.chdir(os.path.dirname(os.path.abspath(__file__)) + '/../../..')

SCHEMA_PATH = 'tools/ddsrouter_yaml_validator/ddsrouter_yaml_validator/' \
              'ddsrouter_config_schema.json'

VALID_CONFIGURATION_FILES = [
    'resources/configurations/examples',
    'docs/resources/getting_started']

INVALID_CONFIGURATION_FILES = [
    'tools/ddsrouter_yaml_validator/tests/invalid_configuration_files']


def test_valid_yamls():
    """Assert given configuration files are valid."""
    validator = YamlValidator()
    for valid_files_dir in VALID_CONFIGURATION_FILES:
        for root, dirs, files in os.walk(valid_files_dir):
            for file in files:
                file_path = os.path.join(root, file)
                assert validator.validate(file_path, SCHEMA_PATH, logout=False), 'Test error: ' + file_path


def test_invalid_yamls():
    """Assert given configuration files are invalid."""
    validator = YamlValidator()
    for invalid_files_dir in INVALID_CONFIGURATION_FILES:
        assert not validator.validate(
            invalid_files_dir, SCHEMA_PATH, logout=False), 'Negative test error: ' + invalid_files_dir
