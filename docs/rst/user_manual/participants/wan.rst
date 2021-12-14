.. include:: ../../exports/alias.include

.. _user_manual_participants_wan:

###############
WAN Participant
###############

This type of :term:`Participant` refers to a :term:`Discovery Server` :term:`DomainParticipant` that communicates
with other **WAN** Participants in different networks.
This Participant will work as bridge for every Participant working locally in the LAN and any other LAN that has
a |ddsrouter| with an active WAN Participant.

.. warning::

    Do not try to communicate a WAN Participant with any other kind of Participant that is not of type
    WAN Participant.

Use case
========

Use this Participant to communicate an internal DDS network with other LANs through a WAN communication.
Each of the networks to be connected require a running |ddsrouter|, and the messages will be relay from one to
another depending on the topics filtered by each of them.


Type aliases
============

* ``wan``
* ``router``


Configuration
=============

WAN Discovery Server Participant allow configure the standard attributes of a Discovery Server.
To configure the Discovery Server :term:`GuidPrefix`, check the following section
:ref:`Configuration section <user_manual_configuration_domain_id>`.
To configure the Discovery Server listening addresses, check the following section
:ref:`Configuration section <user_manual_configuration_discovery_server_listening_addresses>`.
To configure the Discovery Server connection addresses to connect with other Discovery Servers,
check the following section
:ref:`Configuration section <user_manual_configuration_discovery_server_connection_addresses>`.

.. note::

    The network addresses set in *listening-addresses* and *connection-addresses* use ``UDP`` transport by default
    if the transport is not specified in the address configuration.

WAN Configuration
-----------------

Refer to section :ref:`user_manual_wan_configuration` for detailed explanation on how to correctly configure
the |ddsrouter| for WAN communication.

Configuration Example
=====================

Configure a WAN Discovery Server with GuidPrefix id ``2`` (``01.0f.02.00.00.00.00.00.00.00.ca.fe``).
It listens for clients in public IP ``82.0.0.1`` in port ``11600`` in ``TCP``.
It connects with a remote WAN Participant in IPv6 address ``2001:4860:4860::8888`` and port ``11666`` which Discovery
Server GuidPrefix is ``01.0f.04.00.00.00.00.00.00.00.ca.fe`` using ``UDP`` transport.

.. code-block:: yaml

    wan_participant:             # Participant Id = wan_participant

        type: "wan"

        id: 2                                       # GuidPrefix = 01.0f.02.00.00.00.00.00.00.00.ca.fe

        listening-addresses:                        # WAN Discovery Server Listening Addresses
        [
            {                                       # Use TCP by default
                ip: "82.0.0.1",
                port: 11600,
            }
        ]

        connection-addresses:                       # Another WAN Participant Listening Addresses
        [
            {
                id: 4                               # External Discovery Server id => GuidPrefix = 01.0f.04.00.00.00.00.00.00.00.ca.fe
                addresses:
                [
                    {
                        ip: "2001:4860:4860::8888",
                        port: 11666,
                        transport: udp              # Use UDP transport
                    }
                ]
            }
        ]
