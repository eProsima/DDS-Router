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

import logging

# Create a custom logger
logger = logging.getLogger('VALIDATION')
# Create handlers
l_handler = logging.StreamHandler()
# Create formatters and add it to handlers
l_format = '[%(asctime)s][%(name)s][%(levelname)s] %(message)s'
l_format = logging.Formatter(l_format)
l_handler.setFormatter(l_format)
# Add handlers to the logger
logger.addHandler(l_handler)
logger.setLevel(logging.INFO)


def activate_debug():
    """Activate debug mode."""
    logger.setLevel(logging.DEBUG)
