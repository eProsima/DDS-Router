.. include:: ../../exports/alias.include

.. _user_manual_participants_echo:

################
Echo Participant
################

This :term:`Participant` prints in ``stdout`` every data that is received by the |ddsrouter|.
The resulted logs contain the echo :term:`Participant Id`, the source :term:`Endpoint` :term:`Guid` that has
originally generated the message, the :term:`Topic` where this message has been received,
and the :term:`Payload` (in hexadecimal format) received.
Notice that this Payload is the same that a standard DDS :term:`DataReader` will receive if it is connected to one
of the Participants of the |ddsrouter|.

.. code-block:: bash

    Echo Participant: <participant_id> has received from Endpoint: <endpoint_guid> in topic: <endpoint_topic> the following payload: <payload>

This is an example of a message received by a |ddsrouter| in a Participant connected to a ROS2 ``talker``
and written by an **Echo Participant**:

.. code-block:: bash

    Echo Participant: ParticipantId{echo} has received from Endpoint: 01.0f.b8.a8.2e.69.b1.47.01.00.00.00|0.0.12.3 in topic: Topic{rt/chatter, std_msgs::msg::dds_::String_} the following payload: <Payload{00 01 00 00 0f 00 00 00 48 65 6c 6c 6f 20 57 6f 72 6c 64 3a 20 31 00 00}>

.. note::

    This Participant does not perform any discovery or data reception functionality.


Use case
========

Use this Participant in order to see in ``stdout`` the data that is being relayed by the router.
All the data received by any of the Participants of the router will be printed with timestamp, topic and source guid
along with the payload.


Type aliases
============

* ``echo``

Configuration
=============

Echo Participant does not allow any configuration.

Configuration Example
=====================

.. code-block:: yaml

    echo_participant:       # Participant Id = echo_participant
        type: echo
