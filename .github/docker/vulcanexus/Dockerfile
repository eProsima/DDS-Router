ARG docker_image_base
FROM $docker_image_base

LABEL author=raul@eprosima.com

# Avoid interactuation with installation of some package that needs the locale.
ENV TZ=Europe/Madrid
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Avoids using interactions during building
ENV DEBIAN_FRONTEND=noninteractive

# Use a bash shell so it is possigle to run things like `source` (required for colcon builds)
SHELL ["/bin/bash", "-c"]

ARG docker_image_base
RUN echo "Docker Base image used: ${docker_image_base}"

# Update ROS 2 keys (not updated in the base image as of June 2025)
RUN rm /etc/apt/sources.list.d/ros2-latest.list && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install message interface and demo nodes required packages
RUN source "/opt/vulcanexus/$VULCANEXUS_DISTRO/setup.bash" && \
        apt-get update && \
        apt-get install -y \
            ros-$VULCANEXUS_DISTRO-example-interfaces \
            ros-$VULCANEXUS_DISTRO-demo-nodes-cpp \
            ros-$VULCANEXUS_DISTRO-demo-nodes-py

# Copy and use run.bash as entrypoint
COPY run.bash /root/run.bash

RUN chmod +x /root/run.bash

ENTRYPOINT ["/root/run.bash" ]
