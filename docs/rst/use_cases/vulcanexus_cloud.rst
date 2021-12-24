.. include:: ../exports/alias.include

.. _vulcanexus_cloud:

.. warning::

    *Vulcanexus* distribution is not currently available, but it will very soon. To follow this tutorial, a
    *ROS 2 galactic* Docker image may be used instead, :ref:`installing <developer_manual_installation_sources_linux>`
    |ddsrouter| whenever required. Please also take into account that environment variable ``RMW_IMPLEMENTATION`` must
    be exported so as to utilize |fastdds| as middleware in *ROS* (see `Working with eProsima Fast DDS <https://docs.ros.org/en/galactic/Installation/DDS-Implementations/Working-with-eProsima-Fast-DDS.html>`_).

################
Vulcanexus Cloud
################

*Vulcanexus* is an extended *ROS 2* distribution provided by *eProsima* which includes additional tools for an improved
user and development experience, such as *Fast-DDS-Monitor* for monitoring the health of DDS networks, or *micro-ROS*,
a framework aimed at optimizing and extending *ROS 2* toolkit to microcontroller-based devices.

Apart from plain LAN-to-LAN communication, Cloud environments such as container-oriented platforms have also been
present throughout the |ddsrouter| design phase. In this walk-through example, we will set up both a *Kubernetes*
(|k8s|) network and local environment in order to establish communication between a pair of ROS nodes, one sending
messages from a LAN (talker) and another one (listener) receiving them in the Cloud. This will be accomplished by having
a |ddsrouter| instance at each side of the communication.

.. figure:: /rst/figures/ddsrouter_overview_wan.png

Local setup
===========
The local instance of |ddsrouter| (local router) only requires to have a
:ref:`Simple Participant <user_manual_participants_simple>`, and a :ref:`WAN Participant <user_manual_participants_wan>`
that will play the client role in the discovery process of remote participants
(see :term:`Discovery Server discovery mechanism <Discovery Server>`).

After having acknowledged each other’s existence through `Simple DDS discovery mechanism <https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/simple.html>`_
(multicast communication), the local participant will start receiving messages published by the ROS 2 talker node, and
will then forward them to the WAN participant. Following, these messages will be sent to another participant hosted on a
|k8s| cluster to which it connects via WAN communication over UDP/IP.

Following is a representation of the above-described scenario:

.. figure:: /rst/figures/vulcanexus_local.png


Local router
------------
The configuration file used by the local router will be the following:

.. literalinclude:: ../../resources/use_cases/vulcanexus_cloud/local-ddsrouter.yaml
    :language: yaml

Note that the simple participant will be receiving messages sent in DDS domain ``0``. Also note that, due to the choice
of UDP as transport protocol, a listening address with the LAN public IP address needs to be specified for the local WAN
participant, even when behaving as client in the participant discovery process. Make sure that the given port is
reachable from outside this local network by properly configuring port forwarding in your Internet router device.
The connection address points to the remote WAN participant deployed in the |k8s| cluster. For further details on how to
configure WAN communication, please have a look at :ref:`WAN Configuration <user_manual_wan_configuration>`.

To launch the local router from within a Vulcanexus Docker image, execute the following:

.. code-block:: bash

    docker run -it --net=host -v local-ddsrouter.yaml:/tmp/local-ddsrouter.yaml ubuntu-vulcanexus:galactic -r "ddsrouter -c /tmp/local-ddsrouter.yaml"


Talker
------
*ROS 2* demo nodes is not installed in *Vulcanexus* distribution by default, but one can easily create a new Docker
image including this feature by using the following Dockerfile:

.. literalinclude:: ../../resources/use_cases/vulcanexus_cloud/Dockerfile_LAN
    :language: Dockerfile

Create the new image and start publishing messages by executing:

.. code-block:: bash

    docker build -t vulcanexus-demo-nodes:galactic -f Dockerfile .
    docker run -it --net=host vulcanexus-demo-nodes:galactic -r "ros2 run demo_nodes_cpp talker"



Kubernetes setup
================
Two different deployments will be used for this example, each in a different |k8s| pod. The |ddsrouter| cloud instance
(cloud router) consists of two participants; a WAN participant that receives the messages coming from our LAN through
the aforementioned UDP communication channel, and a
:ref:`Local Discovery Server <user_manual_participants_local_discovery_server>` (local DS) that propagates them to a
ROS 2 listener node hosted in a different |k8s| pod. The choice of a Local Discovery Server instead of a Simple
Participant to communicate with the listener has to do with the difficulty of enabling multicast routing in cloud
environments.

The described scheme is represented in the following figure:

.. figure:: /rst/figures/vulcanexus_cloud.png

In addition to the two mentioned deployments, two |k8s| `services <https://kubernetes.io/docs/concepts/services-networking/service/>`_
are required in order to direct dataflow to each of the pods. A LoadBalancer will forward messages reaching the cluster
to the WAN participant of the cloud router, and a ClusterIP service will be in charge of delivering messages from the
local DS to the listener pod. Following are the settings needed to launch these services in |k8s|:

.. literalinclude:: ../../resources/use_cases/vulcanexus_cloud/load-balancer-service.yaml
    :language: yaml

.. literalinclude:: ../../resources/use_cases/vulcanexus_cloud/local-service.yaml
    :language: yaml

.. note::

    An `Ingress <https://kubernetes.io/docs/concepts/services-networking/ingress/>`_ needs to be configured for the
    LoadBalancer service to make it externally-reachable. In this example we consider the assigned public IP address to
    be ``2.2.2.2``.

The configuration file used for the cloud router will be provided by setting up a `ConfigMap <https://kubernetes.io/docs/concepts/configuration/configmap/>`_:

.. literalinclude:: ../../resources/use_cases/vulcanexus_cloud/ConfigMap.yaml
    :language: yaml

Following is represented the overall configuration of our |k8s| cluster:

.. figure:: /rst/figures/vulcanexus_k8s.png


DDS-Router deployment
---------------------
The cloud router is launched from within a Vulcanexus Docker image, which uses as configuration file the one hosted in
the previously set up ConfigMap. The cloud router will be deployed with the following settings:

.. literalinclude:: ../../resources/use_cases/vulcanexus_cloud/ddsrouter.yaml
    :language: yaml


Listener deployment
-------------------
Again, since demo nodes is not installed by default in *Vulcanexus* we have to create a new Docker image adding in this
functionality. The Dockerfile used for the listener will slightly differ from the one utilized to launch a talker in
our LAN, as here we need to specify the port and IP address of the local DS. This can be achieved by using the following
Dockerfile and entrypoint:

.. literalinclude:: ../../resources/use_cases/vulcanexus_cloud/Dockerfile_cloud
    :language: Dockerfile

.. literalinclude:: ../../resources/use_cases/vulcanexus_cloud/run.bash
    :language: bash

As before, to build the extended Docker image it suffices to run:

.. code-block:: bash

    docker build -t vulcanexus-demo-nodes:galactic -f Dockerfile .

Now, the listener pod can be deployed by providing the following configuration:

.. literalinclude:: ../../resources/use_cases/vulcanexus_cloud/listener.yaml
    :language: yaml


Once all these components are up and running, communication should have been established between talker and listener
nodes. Feel free to interchange the locations of the ROS nodes by slightly modifying the provided configuration files,
so that the talker is the one hosted in the |k8s| cluster while the listener runs in our LAN.
