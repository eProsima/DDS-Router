.. include:: ../../exports/alias.include

.. _user_manual_participants_wan:

###############
WAN Participant
###############

This type of :term:`Participant` refers to a :term:`Initial Peers` :term:`DomainParticipant` that communicates
with other **WAN** Participants in different networks.
This Participant will work as bridge for every Participant working locally in the LAN and any other LAN that has
a |ddsrouter| with an active WAN Participant.

.. warning::

    Do not try to communicate a WAN Participant with any other kind of Participant that is not of type
    WAN Participant.

Use case
========

Use this Participant to communicate an internal DDS network with other LANs through a WAN communication.
Each of the networks to be connected require a running |ddsrouter|, and the messages will be relayed from one to
another depending on the topics filtered by each of them.


Kind aliases
============

* ``wan``
* ``router``
* ``initial-peers``


Configuration
=============

The WAN Participant allows users to configure the listening and connection addresses (whether it should locate or
be located by other remote Participants):

* To configure the listening addresses, check the
  :ref:`Listening Addresses <user_manual_configuration_listening_addresses>` configuration section.
* To configure the connection addresses to connect with other Wan Participants,
  check the
  :ref:`Initial Peers Connection Addresses <user_manual_configuration_initial_peers_connection_addresses>` configuration section.

.. note::

    The network addresses set in *listening-addresses* and *connection-addresses* use ``UDP`` transport by default
    if the transport is not specified in the address configuration.

Repeater
--------

This Participant allows a tag ``repeater`` to be used as a :term:`TURN` server.
Please refer to section :ref:`use_case_repeater` for more information.

WAN Configuration
-----------------

Refer to section :ref:`user_manual_wan_configuration` for a detailed explanation on how to correctly configure
the |ddsrouter| for WAN communication.

.. _user_manual_participants_wan_example:

Configuration Example
=====================

Configure a WAN Participant.
It listens for clients in public IP ``82.0.0.1`` in port ``11600`` in ``TCP``.
It connects with a remote Participant in IPv6 address ``2001:4860:4860::8888`` and port ``11666``
using ``TCP`` transport.

.. code-block:: yaml

    - name: wan_participant                       # Participant Name = wan_participant

      kind: wan

      listening-addresses:                        # WAN Participant Listening Addresses
        - ip: 82.0.0.1                            # Use UDP by default
          port: 11600

      connection-addresses:                       # Another WAN Participant Listening Addresses
        - ip: 2001:4860:4860::8888
          port: 11666
          transport: tcp                          # Use UDP transport
