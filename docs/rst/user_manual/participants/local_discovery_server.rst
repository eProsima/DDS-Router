.. include:: ../../exports/alias.include

.. _user_manual_participants_local_discovery_server:

##################################
Local Discovery Server Participant
##################################

This type of :term:`Participant` refers to a :term:`Discovery Server` :term:`DomainParticipant`.
This Participant will work as discovery broker for those Participants that connect to it (clients or servers).
It could also connect to one or multiple Discovery Servers to create a Discovery Server Network.


Use case
========

Use this Participant in order to communicate an internal DDS network using Discovery Server.
This is highly useful in networks that do not support multicast communication;
or to reduce the number of meta-traffic packets exchanged in discovery,
reducing the network traffic in the discovery process.

Type aliases
============

* ``discovery-server``
* ``local-ds``
* ``ds``


Configuration
=============

Local Discovery Server Participant allow configure the standard attributes of a Discovery Server.

* To configure the Discovery Server :term:`GuidPrefix`, check the following section
  :ref:`Configuration section <user_manual_configuration_domain_id>`.
* To configure the Discovery Server listening addresses, check the following section
  :ref:`Configuration section <user_manual_configuration_discovery_server_listening_addresses>`.
* To configure the Discovery Server connection addresses to connect with other Discovery Servers,
  check the following section
  :ref:`Configuration section <user_manual_configuration_discovery_server_connection_addresses>`.

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

    local_discovery_server_participant:             # Participant Id = local_discovery_server_participant

        type: "discovery-server"

        id: 2
        ros-discovery-server: true                  # ROS Discovery Server id => GuidPrefix = 44.53.02.5f.45.50.52.4f.53.49.4d.41

        listening-addresses:                        # Local Discovery Server Listening Addresses
        [
            {                                       # Use UDP by default
                ip: "127.0.0.1",
                port: 11600,
            },
            {
                ip: "127.0.0.1",
                port: 11601,
                transport: "tcp"                    # Use TCP transport
            }
        ]

        connection-addresses:                       # External Discovery Server Listening Addresses
        [
            {
                id: 4                               # External Discovery Server id => GuidPrefix = 01.0f.04.00.00.00.00.00.00.00.ca.fe
                addresses:
                [
                    {                               # Use UDP by default
                        ip: "2001:4860:4860::8888",
                        port: 11666,
                    }
                ]
            }
        ]
