.. include:: ../exports/alias.include

.. _user_manual_glossary:

########
Glossary
########


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
        It is an abstraction over DDS DomainParticipant.

        This term is explained :ref:`here <user_manual_participant_participant>`.

    Participant Id
        Unique identifier of a Participant.

        This term is explained :ref:`here <user_manual_participant_participant_id>`.

    Participant Type
        Element that identifies a Participant kind.
        It can be set as a `string` that references an alias of an existing Participant Type.

        This term is explained :ref:`here <user_manual_participant_participant_type>`.


DDS nomenclature
================

.. glossary::

    DomainParticipant
        A DomainParticipant is the entry point of the application to a DDS Domain.
        Every DomainParticipant is linked to a single domain from its creation,
        and contains all the Entities related to that domain.
        It also acts as a factory for Publisher, Subscriber and Topic.

        See `Fast DDS documentation <https://fast-dds.docs.eprosima.com/en/v2.4.1/fastdds/dds_layer/domain/domainParticipant/domainParticipant.html>`__
        for further information.

    Topic
        DDS isolation abstraction to encapsulate subscriptions and publications.
        Each Topic is uniquely identified by a topic name and a topic type name (name of the data type it transmits).

        See `Fast DDS documentation <https://fast-dds.docs.eprosima.com/en/v2.4.1/fastdds/dds_layer/topic/topic.html>`__
        for further information.

    Guid
        Global Unique Identifier.
        Identifies uniquely a DDS entity (DomainParticipant, DataWriter or DataReader).

    Endpoint
        DDS element that publish or subscribes in a specific Topic. Endpoint kinds are *DataWriter* or *DataReader*.

    DataWriter
        DDS entity that publish data in a specific Topic.
        It belong to one and only one Participant, and it is uniquely identified by a Guid.

        See `Fast DDS documentation <https://fast-dds.docs.eprosima.com/en/v2.4.1/fastdds/dds_layer/publisher/dataWriter/dataWriter.html>`__
        for further information.

    DataReader
        DDS element that subscribes to a specific Topic.
        It belong to one and only one Participant, and it is uniquely identified by a Guid.

        See `Fast DDS documentation <https://fast-dds.docs.eprosima.com/en/v2.4.1/fastdds/dds_layer/subscriber/subscriber.html>`__
        for further information.
