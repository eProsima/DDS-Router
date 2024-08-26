.. include:: ../exports/alias.include
.. include:: ../exports/roles.include

.. _docker:

############
Docker image
############

eProsima distributes a Docker image of |ddsrouter| with Ubuntu 22.04 as base image.
This image launches an instance of |ddsrouter| that is configured using a *YAML* configuration file provided by the user
and shared with the Docker container.
The steps to run |ddsrouter| in a Docker container are explained below.

#.  Download the compressed Docker image in ``.tar`` format from the
    `eProsima Downloads website <https://www.eprosima.com/index.php/downloads-all>`_.
    It is strongly recommended to download the image corresponding to the latest version of |ddsrouter|.

    |br|

#.  Extract the image by executing the following command:

    .. code-block:: bash

        docker load -i ubuntu-ddsrouter\ <version>.tar

    where ``version`` is the downloaded version of |ddsrouter|.

    |br|

#.  Build a |ddsrouter| configuration YAML file on the local machine.
    This will be the |ddsrouter| configuration file that runs inside the Docker container.
    To continue this installation manual, let's use one of the configuration files provided in the
    :ref:`Examples <examples_echo_example>` section.
    Open your preferred text editor and copy a full example from the :ref:`Examples <examples_echo_example>` section
    into the ``/<dds_router_ws>/DDS_ROUTER_CONFIGURATION.yaml`` file, where ``dds_router_ws`` is the path of the
    configuration file.
    To make this accessible from the Docker container we will create a shared volume containing just
    this file. This is explained in next point.

    |br|

#.  Run the Docker container executing the following command:

    .. code-block:: bash

        docker run -it \
            --net=host \
            -v /<dds_router_ws>/DDS_ROUTER_CONFIGURATION.yaml:/root/DDS_ROUTER_CONFIGURATION.yaml \
            ubuntu-ddsrouter:v0.3.0

    It is important to mention that both the path to the configuration file hosted in the local machine and the one
    created in the Docker container must be absolute paths in order to share just one single file as a shared volume.

    After executing the previous command you should be able to see the initialization traces from the |ddsrouter|
    running in the Docker container.
    If you want to terminate the application gracefully, just press ``Ctrl+C`` to stop the execution of |ddsrouter|.
