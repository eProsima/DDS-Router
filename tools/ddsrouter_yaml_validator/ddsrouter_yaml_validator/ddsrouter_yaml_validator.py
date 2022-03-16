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

"""DDS-Router YAML Validator tool."""

import argparse
import json
import os

import jsonschema

import yaml


class YamlValidator:
    """
    Instances of this class are used to validate DDS-Router YAML configuration\
    files against a specific JSON schema.

    It exposes a static method for this purpose.
    """

    def validate_config_file(self, config_file_path, schema_path, logout=True):
        """Load the files whose paths are given, and perform validation."""
        with open(schema_path) as json_file:
            schema = json.load(json_file)
        with open(config_file_path) as yaml_file:
            config_yaml = yaml.safe_load(yaml_file)
        config_json = json.loads(json.dumps(config_yaml))
        try:
            jsonschema.validate(config_json, schema)
        except jsonschema.exceptions.ValidationError as exception:
            if logout:
                print(exception)
            return False

        if logout:
            print(os.path.basename(config_file_path),
                  'is a valid DDS-Router configuration file.')
        return True


def main(args=None):
    """Parse arguments, create a validator object, and perform validation."""
    parser = argparse.ArgumentParser(
        description='Validate DDS-Router configuration file.'
    )
    parser.add_argument(
        '-c',
        '--config-file',
        required=True,
        help='YAML file used to configure DDS-Router',
    )
    parser.add_argument(
        '-s',
        '--schema',
        default=os.path.join(
            os.environ['COLCON_PREFIX_PATH'],
            'ddsrouter_yaml_validator/share/ddsrouter_yaml_validator/'
            'ddsrouter_config_schema.json',
        ),
        help='JSON schema used to perform validation',
    )
    args = parser.parse_args()

    validator = YamlValidator()
    validator.validate_config_file(args.config_file, args.schema)


if __name__ == '__main__':
    main()
