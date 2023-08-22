.. include:: ../../exports/alias.include

.. _user_manual_participants_local_discovery_server:

##################################
Local Discovery Server Participant
##################################

This kind of :term:`Participant` refers to a :term:`Discovery Server` :term:`DomainParticipant`.
This Participant will work as discovery broker for those Participants that connect to it (clients or servers).
It could also connect to one or multiple Discovery Servers to create a Discovery Server Network.


Use case
========

Use this Participant in order to communicate an internal DDS network using Discovery Server.
This is highly useful in networks that do not support multicast communication;
or to reduce the number of meta-traffic packets exchanged in discovery,
reducing the network traffic in the discovery process.

Kind aliases
============

* ``discovery-server``
* ``local-ds``
* ``ds``
* ``local-discovery-server``


Configuration
=============

The Local Discovery Server Participant allows users to configure the standard attributes of a Discovery Server.

* To configure the Discovery Server :term:`GuidPrefix`, check the
  :ref:`Domain Id <user_manual_configuration_domain_id>` configuration section.
* To configure the Discovery Server listening addresses, check the
  :ref:`Listening Addresses <user_manual_configuration_listening_addresses>` configuration section.
* To configure the Discovery Server connection addresses to connect to other Discovery Servers,
  check the
  :ref:`Discovery Server Connection Addresses <user_manual_configuration_discovery_server_connection_addresses>` configuration section.

.. note::

    The network addresses set in *listening-addresses* and *connection-addresses* use ``UDP`` transport by default
    if the transport is not specified in the address configuration.

Configuration Example
=====================

Configure a Local Discovery Server setting the GuidPrefix used for ROS 2 deployments with id ``2``
(``44.53.02.5f.45.50.52.4f.53.49.4d.41``).
It listens for clients in *localhost* in ports ``11600`` in ``UDP`` and ``11601`` in ``TCP``.
This example connects the local Discovery Server Participant with a remote Discovery Server listening in IPv6 address
``2001:4860:4860::8888`` and port ``11666`` and configured with ``01.0f.04.00.00.00.00.00.00.00.ca.fe``
Discovery Server GuidPrefix.

.. code-block:: yaml

    - name: local_discovery_server_participant        # Participant Name = local_discovery_server_participant

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
        - discovery-server-guid:
            id: 4                                     # External Discovery Server id => GuidPrefix = 01.0f.04.00.00.00.00.00.00.00.ca.fe
          addresses:
            - ip: 2001:4860:4860::8888                # Use UDP by default
              port: 11666
