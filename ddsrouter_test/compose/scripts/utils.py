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

import random
import time


def sleep_random_time(
        min_time_seconds: int,
        max_time_seconds: int):
    """Sleep a random time between min_time_seconds and max_time in seconds."""
    time.sleep(
        min_time_seconds +
        ((max_time_seconds - min_time_seconds) * random.random()))


def print_with_timestamp(
        msg: str):
    """Print a message with a timestamp."""
    print(f'{time.time()}$ {msg}')
    # This has been made by copilot... Respect


def delay(
        time_s: float):
    """Wait for time_s seconds."""
    if (time_s > 0):
        time.sleep(time_s)
