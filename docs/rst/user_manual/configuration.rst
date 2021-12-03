.. include:: ../exports/alias.include

.. _user_manual_configuration:

########################
DDS Router Configuration
########################

A |ddsrouter| execution is configured by a *.yaml* single configuration file.
This *.yaml* file contains all the information regarding the |ddsrouter| configuration, as topics filtering,
and :term:`Participant` configurations.

Topic Filtering
===============

..note:

    The |ddsrouter| discovery module is a work in progress.
    Thus, the functionality regarding Topic filtering is still in its early stages.

The |ddsrouter| requires a list of :term:`Topics<Topic>` that will be the ones that will be relay.
The yaml must contain a tag ``allowlist`` which element is a vector (``[]``).
This vector elements are the different Topics that will be transmitted (this requires to create the
:term:`Writers<DataWriter>` and :term:`Readers<DataReader>` for each topic for each Participant).
See Topic section for further information about the topic

.. todo:

    Add link to topic page when created

Example
-------

In this yaml example, the |ddsrouter| will transmit the topic ``rt/chatter`` (default ROS2 topic for ``talker`` and
``listener``) with type name ``std_msgs::msg::dds_::String_``.
It also will transmit the topic  ``HelloWorldTopic`` (default FastDDS topic for ``HelloWorldExample``)
with type name ``HelloWorld``.

.. code-block:: yaml

    allowlist:
    [
        {name: "rt/chatter", type: "std_msgs::msg::dds_::String_"},
        {name: "HelloWorldTopic", type: "HelloWorld"},
    ]

.. note:

    Tag ``allowlist`` must be at yaml base level (it must not be inside any other tag).

Participant Configuration
=========================

At the yaml base level, along with ``allowlist`` tag, there will be the different Participants that will be created,
along with their specific configuration.
Each Participant is identified by a unique :term:`Participant Id` that will be the yaml *key*.
The yaml *value* for this *key* is the configuration for this specific Participant.
There could be any number of Participants, and Participant types could be repeated.

Each Participant has its specific configuration.
Please, refer to :ref:`user_manual_participant_participant_types` in order to see each of the
:term:`Participant Types<Participant Type>` requirements.

.. warning:

    Do not configure two Participants in a way that they can communicate to each other (e.g. two Simple participants
    in the same domain).
    This will lead to an infinite feedback loop for one to each other.

Example
-------

In this yaml example, the |ddsrouter| will create two Simple Participants, one for ``domain 0`` and one for
``domain 1``.
This is a typical use case of Domain bridge.
The topics allowed in the two domains will start communicating to each other .
Be aware that the communication is not P2P, the data must reach the |ddsrouter| and this will forward the data.

.. todo:

    Add link to Simple Participant when page created
    Add link to use case domain swap

The first Participant `Participant0` has ParticipantId *Participant0* and ParticipantType *simple*.
The second Participant has ParticipantId *simple*, and type is not required to be specified as it is get from the
ParticipantId.

.. code-block:: yaml

    Participant0:       # ParticipantId <Participant0>
        type: local     # ParticipantType <simple>
        domain: 3       # DomainId <3>

    simple:             # ParticipantId <simple> ; ParticipantType <simple>
        domain: 6       # DomainId <6>

.. note:

    Type is not case sensitive.
    A Participant called *Simple* would be of type ``simple``.

.. note:

    The ParticipantId is get as ParticipantType when type is not specified.
    If type is explicitly specified, the ParticipantId is not used to get the type.

Network Address
===============

Network Addresses are elements that can be configured for specific Participants.
An Address is set by:
    - *IP*: IP of the host (public IP in case of WAN).
    - *Port*: Port where the Participant is listening.
    - *Transport Protocol*: ``UDP`` or ``TCP``.
    If it is not set, it would be chosen by default depending on the Participant Type.
    - *IP version*: ``v4`` or ``v6``.
    If it is not set, it would be chosen depending on the *IP* string format.

Example
-------

.. code-block:: yaml

    ip: "127.0.0.1",
    port: 11666,
    transport: "tcp",   # | "udp"
    ip-version: "v4",   # | "v6"

Discovery Server GuidPrefix
===========================

A Discovery Server requires a DDS :term:`GuidPrefix` in order to other Participants connect to it.
There are several possibilities for configuring a GuidPrefix.

Discovery Server GuidPrefix by string
-------------------------------------

Using tag ``guid``


Example
-------

.. code-block:: yaml

    ip: "127.0.0.1",
    port: 11666,
    transport: "tcp",   # | "udp"
    ip-version: "v4",   # | "v6"


General Example
===============

.. code-block:: yaml

    # Relay topic rt/chatter and type std_msgs::msg::dds_::String_
    # Relay topic HelloWorldTopic and type HelloWorld

    allowlist:
    [
        {name: "rt/chatter", type: "std_msgs::msg::dds_::String_"},
        {name: "HelloWorldTopic", type: "HelloWorld"},
    ]

    ####################

    # Simple DDS Participant in domain 3

    Participant0:                       # ParticipantId = Participant0

        type: local                     # ParticipantType = simple

        domain: 3                       # DomainId = 3

    ####################

    # Discovery Server DDS Participant with ROS GuidPrefix so a local ROS2 Client could connect to it
    # This Discovery Server will listen in ports 11600 and 11601 in localhost

    simple:                             # ParticipantId = simple

        type: "local-discovery-server"  # ParticipantType = local-discovery-server

        id: 1
        ros-discovery-server: true      # ROS Discovery Server id => GuidPrefix = 44.53.01.5f.45.50.52.4f.53.49.4d.41

        listening-addresses:            # Local Discovery Server Listening Addresses
        [
            {
                ip: "127.0.0.1",        # IP = localhost ; Transport = UDP (by default)
                port: 11600,            # Port = 11600
            },
            {
                ip: "127.0.0.1",        # IP = localhost
                port: 11601,            # Port = 11601
                transport: "UDP",       # Transport = UDP
            }
        ]

    ####################

    # Participant that will communicate with a DDS Router in a different LAN.
    # This Participant will work as the remote DDS Router Client, so it set the connection address of the remote one.

    Wan:                                # ParticipantId = simple ; ParticipantType = wan

        id: 2                           # Internal WAN Discovery Server id => GuidPrefix = 01.0f.02.00.00.00.00.00.00.00.ca.fe

        connection-addresses:           # WAN Discovery Server Connection Addresses
        [
            {
                id: 4                   # External WAN Discovery Server id => GuidPrefix = 01.0f.04.00.00.00.00.00.00.00.ca.fe
                addresses:
                [
                    {
                        ip: "8.8.8.8",          # IP = 8.8.8.8 ; transport = TCP (by default)
                        port: 11666,            # Port = 11666
                    }
                ]
            }
        ]
