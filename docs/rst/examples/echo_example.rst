.. include:: ../exports/alias.include

.. _examples_echo_example:

########################
Echo Participant Example
########################

In the following snippet we see a yaml file to configure a DDS Router to create a
:ref:`Simple Participant <user_manual_participants_simple>` in :term:`Domain Id` ``0`` and an
:ref:`Echo Participant <user_manual_participants_echo>` that will print in ``stdout`` every message get in Domain ``0``.

.. literalinclude:: ../../resources/examples/echo.yaml
    :language: yaml
    :lines: 5-28

Configuration
=============

Allowed Topics
--------------

In this section are the :term:`Topics <Topic>` that the DDS Router will route from
one Participant to the other.
Topic ``HelloWorldTopic`` with datatype ``HelloWorld``,
and ROS 2 topic ``rt/chatter`` with datatype ``std_msgs::msg::dds_::String_`` will be forwarded from
``SimpleParticipant`` to ``EchoParticipant``, that will print the message in ``stdout``.

.. literalinclude:: ../../resources/examples/echo.yaml
    :language: yaml
    :lines: 9-13


Simple Participant
------------------

This Participant is configured by a name, a type and the Domain Id, in this case ``0``.

.. literalinclude:: ../../resources/examples/echo.yaml
    :language: yaml
    :lines: 19-21


Echo Participant
----------------

This Participant does not require further configuration than name and type.

.. literalinclude:: ../../resources/examples/echo.yaml
    :language: yaml
    :lines: 27-28


Execute example
===============

For a detailed explanation on how to execute the |ddsrouter|, refer to this section.

.. todo:

    Link when section exists.

Execute with Fast DDS HelloWorld Example
----------------------------------------

Execute a Fast DDS HelloWorld example:

.. code-block:: bash

    ./DDSHelloWorldExample publisher

Execute |ddsrouter| with this configuration file (available in
``path-to-ddsrouter/src/databroker/resources/configurations/examples/echo.yaml``).
The expected output from the DDS Router, printed by the ``Echo Participant`` is:

.. code-block:: console

    Echo Participant: ParticipantId{EchoParticipant} has received from Endpoint: 01.0f.44.59.e6.de.2a.c8.01.00.00.00|0.0.1.3 in topic: Topic{HelloWorldTopic, HelloWorld} the following payload: <Payload{00 01 00 00 01 00 00 00 0b 00 00 00 48 65 6c 6c 6f 57 6f 72 6c 64 00 00}>
    Echo Participant: ParticipantId{EchoParticipant} has received from Endpoint: 01.0f.44.59.e6.de.2a.c8.01.00.00.00|0.0.1.3 in topic: Topic{HelloWorldTopic, HelloWorld} the following payload: <Payload{00 01 00 00 02 00 00 00 0b 00 00 00 48 65 6c 6c 6f 57 6f 72 6c 64 00 00}>
    ...
    Echo Participant: ParticipantId{EchoParticipant} has received from Endpoint: 01.0f.44.59.e6.de.2a.c8.01.00.00.00|0.0.1.3 in topic: Topic{HelloWorldTopic, HelloWorld} the following payload: <Payload{00 01 00 00 0a 00 00 00 0b 00 00 00 48 65 6c 6c 6f 57 6f 72 6c 64 00 00}>

Execute with ROS 2 demo nodes
-----------------------------

Execute a ROS 2 ``demo_nodes_cpp`` *talker* in default domain ``0``:

.. code-block:: bash

    ros2 run demo_nodes_cpp talker

Execute |ddsrouter| with this configuration file (available in
``path-to-ddsrouter/src/databroker/resources/configurations/examples/echo.yaml``).
The expected output from the DDS Router, printed by the ``Echo Participant`` is:

.. code-block:: console

    Echo Participant: ParticipantId{EchoParticipant} has received from Endpoint: 01.0f.44.59.33.e0.2b.cf.01.00.00.00|0.0.12.3 in topic: Topic{rt/chatter, std_msgs::msg::dds_::String_} the following payload: <Payload{00 01 00 00 0f 00 00 00 48 65 6c 6c 6f 20 57 6f 72 6c 64 3a 20 34 00 00}>
    Echo Participant: ParticipantId{EchoParticipant} has received from Endpoint: 01.0f.44.59.33.e0.2b.cf.01.00.00.00|0.0.12.3 in topic: Topic{rt/chatter, std_msgs::msg::dds_::String_} the following payload: <Payload{00 01 00 00 0f 00 00 00 48 65 6c 6c 6f 20 57 6f 72 6c 64 3a 20 35 00 00}>
    ...
