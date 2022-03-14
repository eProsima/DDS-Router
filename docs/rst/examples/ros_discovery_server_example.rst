.. include:: ../exports/alias.include

.. _examples_tos_discovery_server_example:

#############################
ROS2 Discovery Server Example
#############################

In the following snippet we see a yaml file to configure a DDS Router to create a
:ref:`Simple Participant <user_manual_participants_simple>` in domain ``0`` and a
:ref:`Local Discovery Server <user_manual_participants_local_discovery_server>` with ROS 2 configuration.

.. literalinclude:: ../../resources/examples/ros_discovery_server.yaml
    :language: yaml
    :lines: 5-40

Configuration
=============

Allowed Topics
--------------

In this section are the :term:`Topics <Topic>` that the DDS Router will route from
one Participant to the other.
Topic ``HelloWorldTopic`` with datatype ``HelloWorld``,
and ROS 2 topic ``rt/chatter`` with datatype ``std_msgs::msg::dds_::String_`` will be forwarded from
one domain to the other, allowing different DDS domains to interact to each other.

.. literalinclude:: ../../resources/examples/ros_discovery_server.yaml
    :language: yaml
    :lines: 13-15


Simple Participant
------------------

This Participant is configured by a name, a kind and the Domain Id, in this case ``0``.

.. literalinclude:: ../../resources/examples/ros_discovery_server.yaml
    :language: yaml
    :lines: 25-27


Discovery Server Participant
----------------------------

This Participant is configured by a name, a kind and a listening addresses where
Discovery Server will expect metatraffic data from clients.

.. literalinclude:: ../../resources/examples/ros_discovery_server.yaml
    :language: yaml
    :lines: 33-40


Execute example
===============

For a detailed explanation on how to execute the |ddsrouter|, refer to this :ref:`section <user_manual_user_interface>`.
Execute a ROS 2 ``demo_nodes_cpp`` *talker* in domain ``0``:

.. code-block:: bash

    ROS_DOMAIN_ID=0 ros2 run demo_nodes_cpp talker

Execute a ROS 2 ``demo_nodes_cpp`` *listener* using Discovery Server as Discovery Protocol:

.. code-block:: bash

    ROS_DISCOVERY_SERVER=";127.0.0.1:11888" ros2 run demo_nodes_cpp listener

Execute |ddsrouter| with this configuration file (available in
``<path/to/ddsrouter_tool>/share/resources/configurations/examples/ros_discovery_server.yaml``).
Once the |ddsrouter| is running, messages from *talker* in domain 0 will be forwarded by the Router
to the *listener* using Discovery Server, that will print them in ``stdout``.
