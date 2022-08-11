"""
TODO
"""
import random

from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node


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

        print(
            f'Client Addition created.')

    def run(self, samples):
        """
        Loop until number of samples has been replied.

        :param samples: Number of samples to reply.

        :return: True if all samples have been replied, False otherwise.
        """
        print(
            f'Client waiting for server.')
        while not self.addition_client.wait_for_service(timeout_sec=1.0):
            print(
                f'Server not available yet...')

        print(
            f'Running Client Addition for {samples} samples.')

        while samples > 0:
            # Generate random numbers
            a = random.randint(0, 100)
            b = random.randint(0, 100)

            # Send request
            result = self._send_request_sync(a, b)

            # Log result
            print(
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

        # Send request
        self.future = self.addition_client.call_async(request)

        print(
            f'Request {{ {a},{b} }} sent, waiting for server result.')

        # Wait for response
        rclpy.spin_until_future_complete(self, self.future)

        # Return result
        return self.future.result().sum
