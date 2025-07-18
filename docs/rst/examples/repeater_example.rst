.. include:: ../exports/alias.include

.. _example_repeater:

################
Repeater Example
################

A |ddsrouter| could work as a **Repeater** to forward messages by the same Participant.
If you are interested in understanding the use case of a Repeater please refer to the following
section :ref:`use_case_repeater`.

Configuration
=============

Allowed Topics
--------------

This section lists the :term:`Topics <Topic>` that the DDS Router will route from
one Participant to the other.
Topic ``HelloWorldTopic`` and ROS 2 topic ``rt/chatter`` will be forwarded from
one domain to the other, allowing different DDS domains to interact with each other.

.. literalinclude:: ../../resources/examples/repeater_server.yaml
    :language: yaml
    :lines: 9-11


Repeater Participant
--------------------

The **Repeater Participant** is the one that will be used to forward messages by the same Participant.
It must be a :ref:`WAN <user_manual_participants_wan>` Participant,
configured with :ref:`WAN configuration <user_manual_wan_configuration>`,
or an :ref:`XML <user_manual_participants_xml>` Participant.

.. literalinclude:: ../../resources/examples/repeater_server.yaml
    :language: yaml
    :lines: 21-27


Client Participants
-------------------

Every *Client* can connect to a Repeater Participant as if it was a normal :code:`WAN` Participant.
The Repeater admits as many edge |ddsrouter| connections as desired, so several |ddsrouter| can use the same
or similar configuration to communicate across it.

.. literalinclude:: ../../resources/examples/repeater_client.yaml
    :language: yaml
    :lines: 29-34


Execute example
===============

Please refer to this :ref:`section <user_manual_user_interface>` for a detailed explanation on how to execute the
|ddsrouter|.

In order to run these examples, there must be three different hosts located in different local networks:

* host *H*:sub:`A` with private IP ``192.168.1.2`` connected to network router *R*:sub:`A` with public IP ``1.1.1.1``.
* host *H*:sub:`B` with private IP ``192.168.2.2`` connected to network router *R*:sub:`B` with public IP ``2.2.2.2``.
* host *H*:sub:`C` with private IP ``192.168.2.3`` connected to network router *R*:sub:`C` with public IP ``3.3.3.3``.

These examples could be run in localhost or with two hosts in the same LAN, but it will not use the WAN
communication features of the |ddsrouter|.

Host *H*:sub:`A`
----------------

This host runs the |ddsrouter| Repeater Server, which will receive data from both edges and redirect the messages
between them.
Execute |ddsrouter| using file
``<path/to/ddsrouter_tool>/share/resources/configurations/examples/repeater.yaml``.
Remember to change the IP and port on the configuration file to the actual public IP of *R*:sub:`A`, and be sure that
the port forwarding rules are configured in *R*:sub:`A` so *H*:sub:`A` is accessible from the outside.


Hosts *H*:sub:`B` and *H*:sub:`C`
---------------------------------

These hosts run the |ddsrouter| WAN Clients, which will connect to the previously launched Repeater Server.
Execute |ddsrouter| using file
``<path/to/ddsrouter_tool>/share/resources/configurations/examples/wan_client.yaml``.
Remember to change the IPs and ports on the configuration file to the actual public IPs of *R*:sub:`A` and *R*:sub:`B`.
**In this example the port forwarding is not required, as the Repeater will allow the communication through it,
and TCP protocol is being used.**.

Execute with ROS 2 demo nodes
-----------------------------

Both clients can execute ROS 2 demo nodes, which will publish and subscribe in topic ``rt/chatter``.
Execute a ``talker`` in one of them and a ``listener`` in the other.
It is recommended to use different ``ROS_DOMAIN_ID`` in each node, so in case both nodes are accessible
(under same network) no loop is created.
In order to do so, change the YAML configuration files to use different domains, and use the following ROS2 commands:

.. code-block:: bash

    # Hb
    ROS_DOMAIN_ID=0 ros2 run demo_nodes_cpp talker

    # Hc
    ROS_DOMAIN_ID=1 ros2 run demo_nodes_cpp listener


Result
------

After executing the three |ddsrouter| applications in hosts, and *talker* and *listener* applications,
the *listener* in *H*:sub:`C` will start receiving and printing data from the *talker* in *H*:sub:`B`.
**You are communicating DDS via WAN**.

.. note::

    If *H*:sub:`B` can access *H*:sub:`C` due to port forwarding or because both are under the same network,
    the ``listener`` will receive duplicated messages, as one of them will arrive from *H*:sub:`B` and the other
    from *H*:sub:`A`.

Execute with Fast DDS HelloWorld Example
----------------------------------------

Both clients can create *Fast DDS* endpoints , which will publish and subscribe in topic ``HelloWorldTopic``.
Execute a ``publisher`` in one of them and a ``subscriber`` in the other.
It is recommended to use different domains in each endpoint, so in case both endpoints are accessible
(under same network) no loop is created.
In order to do so, change the YAML configuration files to use different domains.

Execute a |fastdds| ``configuration`` example  *publisher* in domain ``0``:

.. code-block:: bash

    ./<path/to/fastdds_installation>/share/fastdds/examples/cpp/configuration/bin/configuration publisher --domain 0 --name HelloWorldTopic

Execute a |fastdds| ``configuration`` example *subscriber* in domain ``1``:

.. code-block:: bash

    ./<path/to/fastdds_installation>/share/fastdds/examples/cpp/configuration/bin/configuration subscriber --domain 1 --name HelloWorldTopic

Result
------

After executing the three |ddsrouter| applications in hosts, and *publisher* and *subscriber* applications,
the *subscriber* in *H*:sub:`C` will start receiving and printing data from the *publisher* in *H*:sub:`B`.
**You are communicating DDS via WAN**.

.. note::

    If *H*:sub:`B` can access *H*:sub:`C` due to port forwarding or because both are under the same network,
    the ``subscriber`` will receive duplicated messages, as one of them will arrive from *H*:sub:`B` and the other
    from *H*:sub:`A`.
