.. include:: ../../exports/alias.include

.. _user_manual_participants_discovery_server_wan:

################################
Discovery Server WAN Participant
################################

This type of :term:`Participant` refers to a :term:`Discovery Server` :term:`DomainParticipant` that communicates
with other **Discovery Server WAN** Participants in different networks.
This Participant will work as bridge for every Participant working locally in the LAN and any other LAN that has
a |ddsrouter| with an active Discovery Server WAN Participant.

.. warning::

    Do not try to communicate a Discovery Server WAN Participant with any other kind of Participant that is not of type
    Discovery Server WAN Participant.

Use case
========

Use this Participant to communicate an internal DDS network with other LANs through a WAN communication.
Each of the networks to be connected require a running |ddsrouter|, and the messages will be relay from one to
another depending on the topics filtered by each of them.


Kind aliases
============

* ``wan-discovery-server``
* ``wan-ds``


Configuration
=============

WAN Discovery Server Participant allow configure the standard attributes of a Discovery Server.

* To configure the Discovery Server :term:`GuidPrefix`, check the following section
  :ref:`Configuration section <user_manual_configuration_domain_id>`.
* To configure the Discovery Server listening addresses, check the following section
  :ref:`Configuration section <user_manual_configuration_listening_addresses>`.
* To configure the Discovery Server connection addresses to connect with other Discovery Servers,
  check the following section
  :ref:`Configuration section <user_manual_configuration_discovery_server_connection_addresses>`.

.. note::

    The network addresses set in *listening-addresses* and *connection-addresses* use ``UDP`` transport by default
    if the transport is not specified in the address configuration.

WAN Configuration
-----------------

Refer to section :ref:`user_manual_wan_configuration` for detailed explanation on how to correctly configure
the |ddsrouter| for WAN communication.

.. _user_manual_participants_discovery_server_wan_example:

Configuration Example
=====================

Configure a WAN Discovery Server with GuidPrefix id ``2`` (``01.0f.02.00.00.00.00.00.00.00.ca.fe``).
It listens for clients in public IP ``82.0.0.1`` in port ``11600`` in ``TCP``.
It connects with a remote WAN Participant in IPv6 address ``2001:4860:4860::8888`` and port ``11666`` which Discovery
Server GuidPrefix is ``01.0f.04.00.00.00.00.00.00.00.ca.fe`` using ``UDP`` transport.

.. code-block:: yaml

    - name: wan_participant                       # Participant Name = wan_participant

      kind: wan-discovery-server

      discovery-server-guid:
        id: 2                                     # GuidPrefix = 01.0f.02.00.00.00.00.00.00.00.ca.fe

      listening-addresses:                        # WAN Discovery Server Listening Addresses
        - ip: 82.0.0.1                            # Use UDP by default
          port: 11600

      connection-addresses:                       # Another WAN Participant Listening Addresses
        - discovery-server-guid:
            id: 4                                 # External Discovery Server id => GuidPrefix = 01.0f.04.00.00.00.00.00.00.00.ca.fe
          addresses:
            - ip: 2001:4860:4860::8888
              port: 11666
              transport: udp                      # Use UDP transport
