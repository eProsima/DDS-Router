.. include:: ../exports/alias.include

.. _user_manual_glossary:

############
Nomenclature
############


DDS Router nomenclature
=======================

.. glossary::

    Payload
        Raw data (no format specified) that is received and sent forward from the DDS Router.


Participant nomenclature
------------------------

.. glossary::

    Participant
        DDS Router communication Interface.
        It is an abstraction of DDS DomainParticipant.

        This term is explained :ref:`here <user_manual_participant_participant>`.

    Participant Name
        Unique identifier of a Participant.

        This term is explained :ref:`here <user_manual_participant_participant_name>`.

    Participant Kind
        Element that identifies a Participant kind.
        It can be set as a `string` that references an alias of an existing Participant Kind.

        This term is explained :ref:`here <user_manual_participant_participant_kind>`.


DDS nomenclature
================

.. glossary::

    DataReader
        DDS element that subscribes to a specific Topic.
        It belongs to one and only one Participant, and it is uniquely identified by a Guid.

        See `Fast DDS documentation <https://fast-dds.docs.eprosima.com/en/v2.4.1/fastdds/dds_layer/subscriber/subscriber.html>`__
        for further information.

    DataWriter
        DDS entity that publishes data in a specific Topic.
        It belongs to one and only one Participant, and it is uniquely identified by a Guid.

        See `Fast DDS documentation <https://fast-dds.docs.eprosima.com/en/v2.4.1/fastdds/dds_layer/publisher/dataWriter/dataWriter.html>`__
        for further information.

    Discovery Server
        Discovery Server Discovery Protocol is a Fast DDS feature that enables a new Discovery mechanism based on a
        Server that filters and distribute the discovery information.
        This is highly recommended in networks where multicast is not available (e.g. WAN).

        See `Fast DDS documentation <https://fast-dds.docs.eprosima.com/en/v2.4.1/fastdds/discovery/discovery_server.html>`__
        for further information.

    Domain Id
        The Domain Id is a virtual partition for DDS networks.
        Only DomainParticipants with the same Domain Id would be able to communicate to each other.
        DomainParticipants  in different Domains will not even discover each other.

        See `Fast DDS documentation <https://fast-dds.docs.eprosima.com/en/v2.4.1//fastdds/dds_layer/domain/domain.html>`__
        for further information.

    DomainParticipant
        A DomainParticipant is the entry point of the application to a DDS Domain.
        Every DomainParticipant is linked to a single domain from its creation, and cannot change such domain.
        It also acts as a factory for Publisher, Subscriber and Topic.

        See `Fast DDS documentation <https://fast-dds.docs.eprosima.com/en/v2.4.1/fastdds/dds_layer/domain/domainParticipant/domainParticipant.html>`__
        for further information.

    Endpoint
        DDS element that publish or subscribes in a specific Topic. Endpoint kinds are *DataWriter* or *DataReader*.

    Guid
        Global Unique Identifier.
        It contains a GuidPrefix and an EntityId.
        The EntityId uniquely identifies sub-entities inside a Participant.
        Identifies uniquely a DDS entity (DomainParticipant, DataWriter or DataReader).

    GuidPrefix
        Global Unique Identifier shared by a Participant and all its sub-entities.
        Identifies uniquely a DDS Participant.

    Initial Peers
        It is a Fast DDS Discovery Protocol that allows the router to send the discovery information directly
        to the Participants configured.
        This is highly recommended for static networks where multicast is not available (e.g. WAN).

        See `Fast DDS documentation <https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/simple.html#initial-peers>`__
        for further information.

    Topic
        DDS isolation abstraction to encapsulate subscriptions and publications.
        Each Topic is uniquely identified by a topic name and a topic type name (name of the data type it transmits).

        See `Fast DDS documentation <https://fast-dds.docs.eprosima.com/en/v2.4.1/fastdds/dds_layer/topic/topic.html>`__
        for further information.
