FROM ubuntu:focal

WORKDIR /ddsrouter

# Needed for a dependency that force to set timezone
ENV TZ=Europe/Madrid
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Get ubuntu repositories information and install dependencies
RUN apt-get update && apt-get install -y \
        git \
        build-essential \
        cmake \
        libssl-dev \
        libasio-dev \
        libtinyxml2-dev \
        libyaml-cpp-dev \
        wget \
        python3-pip && \
    pip3 install colcon-common-extensions vcstool

# Download and build DDS-Router
RUN mkdir resources && \
    wget https://raw.githubusercontent.com/eProsima/DDS-Router/v0.3.0/ddsrouter.repos && \
    mkdir src && \
    vcs import src < ddsrouter.repos && \
    colcon build --event-handlers=console_direct+ --packages-skip ddsrouter_docs --cmake-args -DLOG_INFO=ON

COPY ./run.bash /
RUN chmod +x /run.bash

ENTRYPOINT ["/run.bash"]
