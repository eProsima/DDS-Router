.. include:: ../exports/alias.include

.. _examples_wan_example:

###########
WAN Example
###########

In the following snippet we see a yaml file to configure a DDS Router to create a
:ref:`Simple Participant <user_manual_participants_simple>` in domain ``0`` and a
:ref:`WAN Participant <user_manual_participants_wan>`.

.. literalinclude:: ../../resources/examples/wan_server.yaml
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

.. literalinclude:: ../../resources/examples/wan_server.yaml
    :language: yaml
    :lines: 13-17


Simple Participant
------------------

This Participant is configured by a name, a kind and the Domain Id, in this case ``0``.

.. literalinclude:: ../../resources/examples/wan_server.yaml
    :language: yaml
    :lines: 27-29


WAN Participant Server
----------------------

This Participant is configured with a name, a kind and the listening addresses where
it will expect data from other remote WAN Participant Clients.
This Participant act as a Server only to receive the discovery data from other WAN Participants.
Once the connection has been established, the communication will be symmetrical (except in TCP case, in which case
this Participant will work as TCP Server).

.. literalinclude:: ../../resources/examples/wan_server.yaml
    :language: yaml
    :lines: 35-40

WAN Participant Client
----------------------

In order to create a WAN Participant Client, check the configuration file
``<path/to/ddsrouter_tool>/share/resources/configurations/examples/wan_client.yaml``

.. literalinclude:: ../../resources/examples/wan_client.yaml
    :language: yaml
    :lines: 35-44


Execute example
===============

In order to run this example, there must be two different hosts located in different local networks:

* host *H*:sub:`A` with private IP ``192.168.1.2`` connected to network router *R*:sub:`A` with public IP ``1.1.1.1``.
* host *H*:sub:`B` with private IP ``192.168.2.2`` connected to network router *R*:sub:`B` with public IP ``2.2.2.2``.

This example could be run in localhost or with two hosts in the same LAN, but it will not use the WAN
communication features of the |ddsrouter|.

Host *H*:sub:`A`
----------------

This host runs the |ddsrouter| WAN Server, which will wait for other WAN Clients to connect to it.
Execute |ddsrouter| using file ``<path/to/ddsrouter_tool>/share/resources/configurations/examples/wan_server.yaml``.
Remember to change the IP and port on the configuration file to the actual public IP of *R*:sub:`A`, and be sure that
the port forwarding rules are configured in *R*:sub:`A` so *H*:sub:`A` is accessible from the outside.
Check the following :ref:`section <user_manual_wan_configuration>` for further information about how to configure
WAN in |ddsrouter|.
Refer to this :ref:`section <user_manual_user_interface>` for a detailed explanation on how to execute the |ddsrouter|.

First of all, execute a ROS 2 ``demo_nodes_cpp`` *listener* in domain ``0``.
This listener will discover the Simple Participant in the |ddsrouter|, but will not receive any data yet.

.. code-block:: bash

    ROS_DOMAIN_ID=0 ros2 run demo_nodes_cpp listener

Host *H*:sub:`B`
----------------

This host runs the |ddsrouter| WAN Client, which will connect to the previously launched WAN Server.
Execute |ddsrouter| using file ``<path/to/ddsrouter_tool>/share/resources/configurations/examples/wan_client.yaml``.
Remember to change the IPs and ports on the configuration file to the actual public IPs of *R*:sub:`A` and *R*:sub:`B`,
and be sure that port forwarding is configured in *R*:sub:`B` so *H*:sub:`B` is accessible from the outside.

In this case, the Simple Participant is configured to use the Domain Id ``1``,
so execute a ROS 2 ``demo_nodes_cpp`` *talker* in domain ``1``.

.. code-block:: bash

    ROS_DOMAIN_ID=1 ros2 run demo_nodes_cpp talker

Result
------

After executing both |ddsrouter| applications in both hosts, and *talker* and *listener* applications,
the *listener* in *H*:sub:`A` will start receiving and printing data from the *talker* in *H*:sub:`B`.
**You are communicating DDS via WAN**.

Remember that the Participants in every |ddsrouter| could be configured as any :term:`Participant Kind`,
allowing to use local Discovery Server, connect to several domains in the same LAN, connect to several WANs, etc.
Endless Possibilities.
Just remember uncle Ben's words: *with great power comes great responsibility*.
