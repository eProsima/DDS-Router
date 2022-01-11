#!/bin/bash

if [[ $1 == "listener" ]]
then
    NODE="listener"
else
    NODE="talker"
fi

SERVER_IP=$2
SERVER_PORT=$3

# Setup environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

echo "Starting ${NODE} as client of Discovery Server ${SERVER_IP}:${SERVER_PORT}"
ROS_DISCOVERY_SERVER=";${SERVER_IP}:${SERVER_PORT}" ros2 run demo_nodes_cpp ${NODE}
