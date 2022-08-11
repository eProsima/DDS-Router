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

"""
TODO
"""

import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts


class AdditionServer(Node):
    """
    TODO
    """

    def __init__(
            self):
        """Construct default Addition Server."""
        super().__init__('AdditionMicroServer')
        self.srv = self.create_service(
            AddTwoInts,
            'addition_service',
            self._addition_service_callback)

        self.samples_replied = 0

        print(
            f'Server Addition created.')

    def _addition_service_callback(
            self,
            request: AddTwoInts.Request,
            response: AddTwoInts.Response):
        """Response to the request received."""
        # Calculate this microservice result
        response.sum = request.a + request.b
        self.samples_replied += 1

        # Log server result
        print(
            f'Request {{ {request.a} + {request.b} = {response.sum} }}')

        # return response
        return response

    def run(self, samples):
        """
        Loop until number of samples has been replied.

        :param samples: Number of samples to reply.

        :return: True if all samples have been replied, False otherwise.
        """
        print(
            f'Running Server Addition for {samples} samples.')

        while self.samples_replied < samples:
            rclpy.spin_once(self)

        return True
