"""
TODO
"""

###
# Required for import ../utils
import inspect
import os
import sys

currentdir = os.path.dirname(
    os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)
###

import random

from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node

from utils import print_with_timestamp, sleep_random_time


class AdditionClient(Node):
    """
    TODO
    """

    def __init__(
            self):
        """
        TODO
        """
        # Call Node constructor
        super().__init__('ClientAddition')

        # Create both clients
        self.addition_client = self.create_client(
            AddTwoInts, 'addition_service')

        print_with_timestamp(
            f'Client Addition created.')

    def run(
            self,
            samples: int,
            wait: bool):
        """
        Loop until number of samples has been replied.

        :param samples: Number of samples to reply.

        :return: True if all samples have been replied, False otherwise.
        """
        print_with_timestamp(
            f'Client waiting for server.')
        while not self.addition_client.wait_for_service(timeout_sec=1.0):
            print_with_timestamp(
                f'Server not available yet...')

        print_with_timestamp(
            f'Running Client Addition for {samples} samples.')

        while samples > 0:
            # Sleep a minimum amount of time
            if wait:
                sleep_random_time(0.1, 0.2)

            # Generate random numbers
            a = random.randint(0, 100)
            b = random.randint(0, 100)

            # Send request
            result = self._send_request_sync(a, b)

            # Log result
            print_with_timestamp(
                f'Result {{ {a},{b},{result} }}')

            # Decrement samples
            samples -= 1

        return True

    def _send_request_sync(
            self,
            a: int,
            b: int) -> int:
        """
        TODO
        """
        # Set request
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        print_with_timestamp(
            f'Request {{ {a},{b} }} sent, waiting for server result.')

        # Send request
        self.future = self.addition_client.call_async(request)

        # Wait for response
        rclpy.spin_until_future_complete(self, self.future)

        # Return result
        return self.future.result().sum
