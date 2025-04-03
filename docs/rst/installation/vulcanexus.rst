.. include:: ../exports/alias.include
.. include:: ../exports/roles.include

.. _vulcanexus:

################
Vulcanexus Cloud
################

Vulcanexus CLOUD scales and integrates ROS 2 networks located in geographically spaced environments, and enables the deployment of ROS 2 entities in the cloud/edge.

*Vulcanexus* offers the possibility of running from a containerized environment by providing a Docker image which contains *Vulcanexus*'s Desktop installation.
This Docker image can be found in `Vulcanexus's Downloads <https://vulcanexus.org/download>`_.
To run it, first install Docker:

.. code-block:: bash

    sudo apt install docker.io

And then load the image with:

.. code-block:: bash

    docker load -i ubuntu-vulcanexus-jazzy-desktop.tar

*Vulcanexus* Docker image can be run with:

.. code-block:: bash

    xhost local:root
    docker run \
        -it \
        --privileged \
        --net host \
        --ipc host \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        ubuntu-vulcanexus:jazzy-desktop

To run more than one session within the same container, *Vulcanexus* installation must be sourced.
Given a running container, you can open another session by:

.. code-block:: bash

    docker exec -it <running-container-id> bash

Then, within the container, source the *Vulcanexus* installation with:

.. code-block:: bash

    source /opt/vulcanexus/jazzy/setup.bash

To verify that the sourcing was correct, run:

.. code-block:: bash

    echo $VULCANEXUS_HOME

The output should be:

.. code-block:: bash

    /opt/vulcanexus/jazzy

For more information about using DDS Router from *Vulcanexus* installation, please refer to the `Vulcanexus Cloud Tutorials <https://docs.vulcanexus.org/en/latest/rst/tutorials/cloud/cloud_tutorials.html>`_.
