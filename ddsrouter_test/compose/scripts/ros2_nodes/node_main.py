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

import argparse

from AdditionClient import AdditionClient
from AdditionServer import AdditionServer

import rclpy

DESCRIPTION = """Script to Execute a ROS2 Service Node"""
USAGE = ('python3 node_main.py [-s] [-s 15] [-a]')


def parse_options():
    """
    Parse arguments.

    :return: The arguments parsed.
    """
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        add_help=True,
        description=(DESCRIPTION),
        usage=(USAGE)
    )
    required_args = parser.add_argument_group('required arguments')
    required_args.add_argument(
        '-c',
        '--client',
        action='store_true',
        help='Execute Client instead of Server.'
    )
    parser.add_argument(
        '-s',
        '--samples',
        type=int,
        default=10,
        help='Samples to receive.'
    )
    parser.add_argument(
        '-a',
        '--async',
        action='store_true',
        help='Use async calls.'
    )
    parser.add_argument(
        '-w',
        '--wait',
        action='store_true',
        help='Wait for reply or request.'
    )

    return parser.parse_args()


def main():
    """Execute main function."""
    # Parse arguments
    args = parse_options()

    # Initialize ROS2 RCL py
    rclpy.init()

    # Create Server or Client
    node = None
    if args.client:
        node = AdditionClient()
    else:
        node = AdditionServer()

    # Run nodes until finish
    result = node.run(args.samples, args.wait)
    if not result:
        exit(1)

    # Destroy Node and stop ROS2 RCL
    node.destroy_node()
    rclpy.shutdown()

    exit(0)


if __name__ == '__main__':
    main()
