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

"""DDS-Router YAML Validator class container."""

import json
import os

import jsonschema

import yaml


class YamlValidator:
    """
    Used to validate DDS-Router YAML configuration files against a schema.

    It exposes a static method for this purpose.
    DDS-Router configurations are YAML files, and the schema is in JSON format.
    """

    def validate(self, config_path, schema_path, recursive=True, logout=True):
        """Validate configuration file or files under a directory."""
        if not self.__is_json(schema_path):
            print(
                'The given schema {} is not a JSON file.'.format(
                    os.path.basename(schema_path)
                )
            )
            return

        if os.path.isdir(config_path):
            ret = True
            visited_dirs = []
            for root, dirs, files in os.walk(config_path):
                print(f'Scanning directory {root}')
                if root not in visited_dirs:
                    visited_dirs.append(root)
                    for f in files:
                        if self.__is_yaml(f):
                            ret = ret and self.__validate_config_file(
                                os.path.join(root, f), schema_path, logout)
                    if not recursive:
                        break
            return ret
        else:
            if self.__is_yaml(config_path):
                return self.__validate_config_file(
                    config_path, schema_path, logout)
            else:
                print(
                    'The given config file {} is not a YAML file.'.format(
                        os.path.basename(config_path)))

    def __validate_config_file(self, config_file_path, schema_path,
                               logout=True):
        """Validate DDS-Router configuration file."""
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

    def __is_json(self, file):
        """
        Check if a file is a JSON file.

        :param file: The input file
        :return: True if the file is an JSON file, False if not.
        """
        return os.path.splitext(str(file))[-1] == '.json'

    def __is_yaml(self, file):
        """
        Check if a file is a YAML file.

        :param file: The input file
        :return: True if the file is an YAML file, False if not.
        """
        ext = os.path.splitext(str(file))[-1]
        return ext == '.yaml' or ext == '.yml'
