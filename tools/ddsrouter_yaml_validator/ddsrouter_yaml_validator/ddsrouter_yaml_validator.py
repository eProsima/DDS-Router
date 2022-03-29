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

from ddsrouter_yaml_validator.parser import Parser
from ddsrouter_yaml_validator.validator import YamlValidator


def main(args=None):
    """Validate a DDS-Router YAML configuration file."""
    parser = Parser()
    args = parser.parse()

    validator = YamlValidator()
    validator.validate(args.config_file, args.schema, args.recursive)


if __name__ == '__main__':
    main()
