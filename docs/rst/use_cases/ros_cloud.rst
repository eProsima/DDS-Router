.. include:: ../exports/alias.include

.. _ros_cloud:

####################
ROS 2 and Kubernetes
####################

Apart from plain LAN-to-LAN communication, Cloud environments such as container-oriented platforms have also been
present throughout the |ddsrouter| design phase. In this walk-through example, we will set up both a *Kubernetes*
(|k8s|) network and a local environment in order to establish communication between a pair of ROS nodes, one sending
messages from a LAN (talker) and another one (listener) receiving them in the Cloud. This will be accomplished by having
a |ddsrouter| instance at each side of the communication.

.. figure:: /rst/figures/ddsrouter_overview_wan.png

Local setup
===========
The local instance of |ddsrouter| (local router) only requires to have a
:ref:`Simple Participant <user_manual_participants_simple>`, and a
:ref:`WAN Participant <user_manual_participants_wan>` that will play the client role in the discovery process of remote
participants (see :term:`Initial Peers discovery mechanism <Initial Peers>`).

After having acknowledged each other’s existence through `Simple DDS discovery mechanism <https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/simple.html>`_
(multicast communication), the local participant will start receiving messages published by the ROS 2 talker node, and
will then forward them to the WAN participant. Following, these messages will be sent to another participant hosted on a
|k8s| cluster to which it connects via WAN communication over UDP/IP.

Following is a representation of the above-described scenario:

.. figure:: /rst/figures/k8s_local_router.png


Local router
------------
The configuration file used by the local router will be the following:

.. literalinclude:: ../../resources/use_cases/ros_cloud/local-ddsrouter.yaml
    :language: yaml

Note that the simple participant will be receiving messages sent in DDS domain ``0``. Also note that, due to the choice
of UDP as transport protocol, a listening address with the LAN public IP address needs to be specified for the local WAN
participant, even when behaving as client in the participant discovery process. Make sure that the given port is
reachable from outside this local network by properly configuring port forwarding in your Internet router device.
The connection address points to the remote WAN participant deployed in the |k8s| cluster. For further details on how to
configure WAN communication, please have a look at :ref:`WAN Configuration <user_manual_wan_configuration>` and
:ref:`WAN Participant Configuration Example <user_manual_participants_wan_example>`.

.. note::

    As an alternative, :ref:`TCP transport <tcp_example>` may be used instead of UDP. This has the advantage of not requiring
    to set a listening address in the local router's WAN participant (TCP client), so there is no need to fiddle with
    the configuration of your Internet router device.

To launch the local router, execute:

.. code-block:: bash

    ddsrouter --config-path local-ddsrouter.yaml


Talker
------
This example will make use of *ROS 2 galactic* with ``demo-nodes-cpp`` package installed. If not already present in your
system, you may choose any of the available options to `install ROS galactic <https://docs.ros.org/en/galactic/Installation.html>`_,
or even consider directly using a distributed `Docker image <https://hub.docker.com/_/ros>`_. Just make sure the
resulting environment is prepared to utilize |efastdds| as middleware (see `Working with eProsima Fast DDS <https://docs.ros.org/en/galactic/Installation/DDS-Implementations/Working-with-eProsima-Fast-DDS.html>`_).

Once *ROS 2* is installed, start publishing messages in DDS domain ``0`` by executing:

.. code-block:: bash

    RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run demo_nodes_cpp talker


Kubernetes setup
================
Two different deployments will be used for this example, each in a different |k8s| pod. The |ddsrouter| cloud instance
(cloud router) consists of two participants:

* A :ref:`WAN Participant <user_manual_participants_wan>` that receives the messages coming from our LAN through the
  aforementioned UDP communication channel.
* A :ref:`Local Discovery Server <user_manual_participants_local_discovery_server>` (local DS) that propagates them to a
  ROS 2 listener node hosted in a different |k8s| pod.

The choice of a Local Discovery Server instead of a Simple Participant to communicate with the listener has to do with
the difficulty of enabling multicast routing in cloud environments.

The described scheme is represented in the following figure:

.. figure:: /rst/figures/k8s_cloud_router.png

In addition to the two mentioned deployments, two |k8s| `services <https://kubernetes.io/docs/concepts/services-networking/service/>`_
are required in order to direct dataflow to each of the pods. A LoadBalancer will forward messages reaching the cluster
to the WAN participant of the cloud router, and a ClusterIP service will be in charge of delivering messages from the
local DS to the listener pod. Following are the settings needed to launch these services in |k8s|:

.. literalinclude:: ../../resources/use_cases/ros_cloud/load-balancer-service.yaml
    :language: yaml

.. literalinclude:: ../../resources/use_cases/ros_cloud/local-service.yaml
    :language: yaml

.. note::

    An `Ingress <https://kubernetes.io/docs/concepts/services-networking/ingress/>`_ needs to be configured for the
    LoadBalancer service to make it externally-reachable. In this example we consider the assigned public IP address to
    be ``2.2.2.2``.

The configuration file used for the cloud router will be provided by setting up a `ConfigMap <https://kubernetes.io/docs/concepts/configuration/configmap/>`_:

.. literalinclude:: ../../resources/use_cases/ros_cloud/ConfigMap.yaml
    :language: yaml

Following is a representation of the overall |k8s| cluster configuration:

.. figure:: /rst/figures/k8s_diagram.png


DDS-Router deployment
---------------------
The cloud router is launched from within a Docker image, which uses as configuration file the one hosted in
the previously set up ConfigMap. This Docker image needs to be built and made available to the |k8s| cluster for using
|ddsrouter|, which can be accomplished by providing the following
:download:`Dockerfile <../../resources/use_cases/ros_cloud/Dockerfile_ddsrouter>`. If willing to see log messages in
``STDOUT``, use :download:`Dockerfile <../../resources/use_cases/ros_cloud/Dockerfile_ddsrouter_logon>` instead.
Assuming the name of the generated Docker image is ``ddsrouter:main``, the cloud router will then be deployed with the
following settings:

.. literalinclude:: ../../resources/use_cases/ros_cloud/ddsrouter.yaml
    :language: yaml


Listener deployment
-------------------
A suitable Docker image must also be provided in the context of the cluster in order to use *ROS 2*. We will use
``ros:galactic`` as basis for this image, install ``demo-nodes-cpp``, and include a parser that will allow us to specify
the port and IP address of the local DS. This can be achieved by using the following Dockerfile and entrypoint:

.. literalinclude:: ../../resources/use_cases/ros_cloud/Dockerfile_listener
    :language: Dockerfile

.. literalinclude:: ../../resources/use_cases/ros_cloud/run.bash
    :language: bash

Now, assuming the name of the built image is ``ros2-demo-nodes:galactic``, the listener pod can be deployed by providing
the following configuration:

.. literalinclude:: ../../resources/use_cases/ros_cloud/listener.yaml
    :language: yaml


Once all these components are up and running, communication should have been established between talker and listener
nodes, so that messages finally manage to reach the listener pod and get printed in its ``STDOUT``.

Feel free to interchange the locations of the ROS nodes by slightly modifying the provided configuration files, hosting
the talker in the |k8s| cluster while the listener runs in our LAN.
