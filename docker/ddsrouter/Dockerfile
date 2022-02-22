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
COPY ./colcon.meta /ddsrouter
RUN mkdir resources && \
    wget https://raw.githubusercontent.com/eProsima/DDS-Router/v0.2.0/ddsrouter.repos && \
    mkdir src && \
    vcs import src < ddsrouter.repos && \
    # Include track deadlock fix
    cd src/ddsrouter && git checkout 016105978bde17c8d03255dac10ca4de642e1cf8 && cd ../.. && \
    colcon build --event-handlers=console_direct+

COPY ./run.bash /
RUN chmod +x /run.bash

ENTRYPOINT ["/run.bash"]
