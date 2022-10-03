ARG docker_image_base
FROM $docker_image_base

LABEL author=javierparis@eprosima.com

# Avoid interactuation with installation of some package that needs the locale.
ENV TZ=Europe/Madrid
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Avoids using interactions during building
ENV DEBIAN_FRONTEND=noninteractive

# Use a bash shell so it is possigle to run things like `source` (required for colcon builds)
SHELL ["/bin/bash", "-c"]

ARG docker_image_base
RUN echo "Docker Base image used: ${docker_image_base}"

# Install message interface and demo nodes required packages
RUN source "/opt/vulcanexus/$VULCANEXUS_DISTRO/setup.bash" && \
        apt-get update && \
        apt-get install -y \
            ros-humble-example-interfaces \
            ros-humble-example-interfaces \
            ros-humble-demo-nodes-cpp \
            ros-humble-demo-nodes-py

# Copy and use run.bash as entrypoint
COPY run.bash /root/run.bash

RUN chmod +x /root/run.bash

ENTRYPOINT ["/root/run.bash" ]
