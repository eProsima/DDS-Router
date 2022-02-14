#!/bin/bash

# Copyright 2021 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

if [[ $1 == "listener" ]]
then
    NODE="listener"
else
    NODE="talker"
fi

SERVER_IP=$2
SERVER_PORT=$3

source /opt/ros/foxy/setup.bash

echo "Starting ${NODE} as client of Discovery Server ${SERVER_IP}:${SERVER_PORT}"
ROS_DISCOVERY_SERVER="${SERVER_IP}:${SERVER_PORT}" ros2 run demo_nodes_cpp ${NODE}
