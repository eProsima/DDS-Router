.. include:: ../exports/alias.include

.. _getting_started:

################
Project Overview
################

|ddsrouter| is a cross-platform non-graphical application developed by eProsima and powered by Fast DDS
that allows to create a communication bridge that connects two DDS networks that otherwise would be isolated.
The main use case of the |ddsrouter| is to communicate two DDS networks that are physically or virtually separated
and belong to different LANs, allowing the entities of each network to publish and subscribe to local
and remote topics indistinctly.

The |ddsrouter| is an application that internally run :term:`Participants <Participant>`, which are an abstraction of
DDS :term:`DomainParticipants <DomainParticipant>`.
Each one of these Participants is an communication interface, a "door" to a specific DDS network configuration.
These Participants allow the application to connect to different DDS networks at the same time.
Every time one of these Participants receives a message from the DDS network to which they are connected,
they will forward the data and the source of this message through the other Participants.
The |ddsrouter| configuration and the topics in which it operates depends on the initial
:ref:`DDS Router configuration <user_manual_configuration>`.

The following schema represents a |ddsrouter| local use case.
This scenario presents different DDS networks that are isolated one to each other due to the Transport Protocol
(UDP, TCP, etc.), the Discovery Protocol (Simple, Discovery Server, etc.) or the DDS :term:`Domain Id` used
by each DDS entity.
Configuring the |ddsrouter| to have 4 different Participants, each of them configured for one isolated DDS network,
will create internally 4 Participants.
All the data that arrives to one of the Participants will be forwarded through the others, allowing all the machines
to connect to each other independently of their different configurations.
This data transmission will be accomplished without copying the data, as all participants will share the pointer
to the allocated data, successfully achieving a **zero-copy** communication mechanism.

.. figure:: /rst/figures/ddsrouter_overview.png


WAN Communication
=================

To achieve a WAN communication of two networks that work in different LANs requires a running
|ddsrouter| application on each LAN.
The |ddsrouter| deployed will communicate to each other using DDS over WAN,
and will route every message received in LAN to the remote |ddsrouter|.
Once the remote Router receives data, it will transmit it to the local networks to which it is connected.
This way, both DDS networks will behave as if they would belong to the same LAN.

Another important feature is that WAN communications are not limited to a single pair of |ddsrouter|.
The WAN communication may be performed using the
`eProsima Discovery Server discovery mechanism <https://fast-dds.docs.eprosima.com/en/v2.4.1/fastdds/discovery/discovery_server.html#discovery-server>`__
(dynamic discovery over non-multicast networks).
Thus, any DDS Router connected to the same Discovery Servers will works as a standard DDS node, publishing
and subscribing in the shared DDS topics.
This allows to create a non limited and highly scalable decentralized and distributed DDS network.

.. figure:: /rst/figures/ddsrouter_overview_wan.png


Usage Description
=================

The |ddsrouter| is a terminal (non-graphical) application that creates the DDS bridge as long as it is running.
The configuration in *YAML* format is very intuitive and human-readable.
The whole application has been thought to be user-friendly, following a user-oriented design.

* **RUN**: In order to run a |ddsrouter| application, just a *YAML* configuration file is required with the specific
  configurations (see :ref:`section <user_manual_configuration>` to check how to configure a DDS Router)
  (see section :ref:`section <user_manual_user_interface_application_arguments>`
  to check the application supported arguments).
* **INTERACT**: Once the |ddsrouter| application is running, the topics involved in this communication could be
  changed in runtime by just changing the *YAML* configuration file
  (see section :ref:`user_manual_user_interface_reload_topics`
  for more details about re-configuring a running |ddsrouter|).
* **STOP**: To stop the |ddsrouter| just send a `^C` signal to the process, and it will gracefully close the whole
  application
  (see section :ref:`user_manual_user_interface_close_application` for more details on how to close the application).


Common Use Cases
================

The different cases where the |ddsrouter| could be applied are very varied, and would increase as new Participant Kinds
will be added in future releases.
These are most common use cases so far:

.. list-table::
    :header-rows: 1

    *   - Use Case
        - Example

    *   - Communicating two different DDS Domain Ids
        - :ref:`examples_change_domain_example`

    *   - Communicating ROS 2 Discovery Server executions
        - :ref:`examples_tos_discovery_server_example`

    *   - WAN Communication
        - :ref:`examples_wan_example`

.. todo:

    Add links to examples
