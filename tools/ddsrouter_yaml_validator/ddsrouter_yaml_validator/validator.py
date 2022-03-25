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
