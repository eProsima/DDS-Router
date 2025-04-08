.. include:: ../../exports/alias.include

.. _user_manual_participants_discovery_server:

############################
Discovery Server Participant
############################

This kind of :term:`Participant` refers to a :term:`Discovery Server` :term:`DomainParticipant`.
This Participant will work as discovery broker for those Participants that connect to it (clients or servers).
It could also connect to one or multiple Discovery Servers to create a Discovery Server Network.


Use case
========

This Participant is designed for two use cases:

1. **Internal DDS Communication via Discovery Server**

   Use this Participant to enable communication within an internal DDS network
   using a Discovery Server. This is especially useful in environments where
   multicast is not supported, or when reducing the amount of meta-traffic
   exchanged during the discovery process is important.

2. **Interconnecting DDS Networks Across LANs and WANs**

   Use this Participant to bridge multiple DDS networks across LANs through WAN
   communication. Each network must run a dedicated instance of ``ddsrouter``,
   which will relay messages between networks based on the topic filters
   configured in each instance.

Kind aliases
============

* ``discovery-server``
* ``local-ds``
* ``ds``
* ``local-discovery-server``

For communicating with other discovery servers through WAN, the following aliases are also available:

* ``wan-discovery-server``
* ``wan-ds``

Configuration
=============

The Discovery Server Participant allows users to configure the standard attributes of a Discovery Server.

* To configure the Discovery Server :term:`GuidPrefix`, check the
  :ref:`Discovery Server GuidPrefix <user_manual_configuration_discovery_server_guidprefix>` configuration section.
* To configure the Discovery Server listening addresses, check the
  :ref:`Listening Addresses <user_manual_configuration_listening_addresses>` configuration section.
* To configure the Discovery Server connection addresses to connect to other Discovery Servers,
  check the
  :ref:`Connection Addresses <user_manual_configuration_connection_addresses>` configuration section.

.. note::

    The network addresses set in *listening-addresses* and *connection-addresses* use ``UDP`` transport by default
    if the transport is not specified in the address configuration.

.. _user_manual_participants_discovery_server_example:

Basic Configuration Example
===========================

Since the :term:`GuidPrefix` parameter is not mandatory, this example shows a basic configuration of a Discovery Server
Participant without setting the :term:`GuidPrefix`.
This Discovery Server listens for clients in *localhost* in ports ``11600`` in ``UDP`` and ``11601`` in ``TCP``.
This example connects the local Discovery Server Participant with a remote Discovery Server listening in IPv6 address
``2001:4860:4860::8888`` and port ``11666``.

.. code-block:: yaml

    - name: discovery_server_participant        # Participant Name = local_discovery_server_participant

      kind: discovery-server

      listening-addresses:                            # Local Discovery Server Listening Addresses
        - ip: 127.0.0.1                               # Use UDP by default
          port: 11600
        - ip: 127.0.0.1
          port: 11601
          transport: tcp                              # Use TCP transport

      connection-addresses:                           # External Discovery Server Listening Addresses
        - ip: 2001:4860:4860::8888                    # Use UDP by default
          port: 11666


Configuration Example with GuidPrefix
=====================================

Discovery Server GuidPrefix by Id
---------------------------------

Configure a Discovery Server setting the GuidPrefix used for ROS 2 deployments with id ``2``
(``44.53.02.5f.45.50.52.4f.53.49.4d.41``).
It listens for clients in *localhost* in ports ``11600`` in ``UDP`` and ``11601`` in ``TCP``.
This example connects the local Discovery Server Participant with a remote Discovery Server listening in IPv6 address
``2001:4860:4860::8888`` and port ``11666``.

.. code-block:: yaml

    - name: discovery_server_participant        # Participant Name = local_discovery_server_participant

      kind: discovery-server

      discovery-server-guid:
        id: 2
        ros-discovery-server: true                    # ROS Discovery Server id => GuidPrefix = 44.53.02.5f.45.50.52.4f.53.49.4d.41

      listening-addresses:                            # Local Discovery Server Listening Addresses
        - ip: 127.0.0.1                               # Use UDP by default
          port: 11600
        - ip: 127.0.0.1
          port: 11601
          transport: tcp                              # Use TCP transport

      connection-addresses:                           # External Discovery Server Listening Addresses
        - ip: 2001:4860:4860::8888                    # Use UDP by default
          port: 11666

Discovery Server GuidPrefix by string
-------------------------------------

Configure a Discovery Server setting the GuidPrefix used for ROS 2 deployments using
``guid: 44.53.02.5f.45.50.52.4f.53.49.4d.41``.
It listens for clients in *localhost* in ports ``11600`` in ``UDP`` and ``11601`` in ``TCP``.
This example connects the local Discovery Server Participant with a remote Discovery Server listening in IPv6 address
``2001:4860:4860::8888`` and port ``11666``.

.. code-block:: yaml

    - name: discovery_server_participant        # Participant Name = local_discovery_server_participant

      kind: discovery-server

      discovery-server-guid:
        guid: "44.53.02.5f.45.50.52.4f.53.49.4d.41"
        ros-discovery-server: true                    # ROS Discovery Server id => GuidPrefix = 44.53.02.5f.45.50.52.4f.53.49.4d.41

      listening-addresses:                            # Local Discovery Server Listening Addresses
        - ip: 127.0.0.1                               # Use UDP by default
          port: 11600
        - ip: 127.0.0.1
          port: 11601
          transport: tcp                              # Use TCP transport

      connection-addresses:                           # External Discovery Server Listening Addresses
        - ip: 2001:4860:4860::8888                    # Use UDP by default
          port: 11666
