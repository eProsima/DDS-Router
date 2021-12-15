.. include:: ../exports/alias.include

.. _getting_started_tutorial:

###############
Getting Started
###############

|ddsrouter| is a cross-platform non-graphic application developed by eProsima and powered by Fast DDS
that allow to create a communication bridge that connects two DDS networks that otherwise would be isolated.
The main use case of the |ddsrouter| is to communicate two DDS networks that are physically or virtually separated
and belong to different LANs, allowing those entities to publish and subscribe to local and remote topics indistinctly.


Project Overview
================

The |ddsrouter| is an application that internally run :term:`Participants <Participant>`, that are an abstraction of
DDS :term:`DomainParticipants <DomainParticipant>`.
Each one of these Participants is an communication interface: a "door" to a specific DDS network configuration.
These Participants allow the application to connect to different DDS networks at the same time.
Every time one of these Participants receives a message from the DDS network it is connected, it will forward the
data and the source of this message through the other Participants.
In which DDS configurations and which topics the |ddsrouter| will work depends on the initial
:ref:`DDS Router configuration <user_manual_configuration>`.

The following schema represents a |ddsrouter| local use case.
In this scenario, we have different DDS networks that are isolated one to each other, due to Transport Protocol
(UDP, TCP, etc.), Discovery Protocol (Simple, Discovery Server, etc.) or by DDS :term:`Domain Id`.
Configuring the |ddsrouter| to have 4 different Participants, each of them using one of the different scenario
configurations, will create internally 4 Participants.
All the data that arrives to one of the Participants will be forwarded through the others, allowing all the machines
to connect to each other independently of their different configurations.

.. figure:: /rst/figures/ddsrouter_overview.png


WAN Communication
-----------------

In order to achieve a WAN communication of two networks that work in different LANs, each LAN requires a running
|ddsrouter| application.
These |ddsrouter| will communicate to each other using DDS over WAN, and will route every message received in LAN
to the remote |ddsrouter|.
Once the remote Router has received it, it will transmit the data to the local networks it is connected with.
This way, both DDS networks will behave as if they would belong to the same LAN.

A great feature as well is that WAN communications is not limited to a single pair of |ddsrouter|.
The communication used in WAN is DDS using Discovery Server (dynamic discovery over non-multicast networks).
Thus, any DDS Router connected to the same Discovery Servers will works as a standard DDS node, publishing
and subscribing in the nodes required.
This allows to create a non limited and hugely scalable decentralized and distributed DDS network.

.. figure:: /rst/figures/ddsrouter_overview_wan.png


Usage Description
=================

The |ddsrouter| is a terminal (non-graphical) application that create the DDS bridge as long as it is running.
The configuration and user interaction with this application has been studied and probed to be very easy to learn
and user friendly.

* **RUN**: In order to run a |ddsrouter| application, just a *YAML* configuration file is required with the specific
  Router configurations (see :ref:`section <user_manual_configuration>` to check how to configure a DDS Router)
  (see section TODO to check the application supported arguments).
* **INTERACT**: Once the |ddsrouter| application is running, the topics involved in this communication could be
  changed in runtime by just changing the *YAML* configuration file
  (see section TODO for more details about re-configuring a running |ddsrouter|)..
* **STOP**: To stop the |ddsrouter| just send a `^C` signal to the process, and it will gracefully close the whole
  application (see section TODO for more details on how to close the application).


.. todo:

    Add links to user interface manual

Common Use Cases
================

The different cases where the |ddsrouter| could be applied are very vary, and would increase as new Participant Types
will be added in future releases.
These are most common use cases so far:

.. list-table::
    :header-rows: 1

    *   - Use Case
        - Example

    *   - Communicating two different DDS Domain Ids
        -

    *   - Communicating ROS 2 Discovery Server executions
        -

    *   - WAN Communication
        -

.. todo:

    Add links to examples
