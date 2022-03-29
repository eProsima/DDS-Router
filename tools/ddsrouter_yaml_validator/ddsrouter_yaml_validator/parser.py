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

"""Argument parser for DDS-Router YAML Validator tool."""

import argparse
import os


class Parser:
    """DDS-Router YAML Validator argument parser."""

    def parse(self):
        """Parse configuration and schema path arguments."""
        parser = argparse.ArgumentParser(
            description='Validate DDS-Router configuration file.'
        )
        parser.add_argument(
            '-c',
            '--config-file',
            required=True,
            help='YAML file or directory with files used to configure'
            'DDS-Router',
        )
        parser.add_argument(
            '-r',
            '--recursive',
            action='store_true',
            default=False,
            help='Whether to perform recursive search when --config-file'
            ' refers to a directory (default: false)',
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
        return parser.parse_args()
