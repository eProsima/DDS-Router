.. include:: ../exports/alias.include

.. _examples_echo_example:

############
Echo Example
############

The following YAML configuration file configures a DDS Router to create a
:ref:`Simple Participant <user_manual_participants_simple>` in :term:`Domain Id` ``0`` and an
:ref:`Echo Participant <user_manual_participants_echo>` that will print in ``stdout`` every message get in Domain ``0``,
as well as information regarding discovery events.

.. literalinclude:: ../../resources/examples/echo.yaml
    :language: yaml
    :lines: 5-39

Configuration
=============

Allowed Topics
--------------

This section lists the :term:`Topics <Topic>` that the DDS Router will route from
one Participant to the other.
Topic ``HelloWorldTopic`` with datatype ``HelloWorld``,
and ROS 2 topic ``rt/chatter`` with datatype ``std_msgs::msg::dds_::String_`` will be forwarded from
``SimpleParticipant`` to ``EchoParticipant``, that will print the message in ``stdout``.

.. literalinclude:: ../../resources/examples/echo.yaml
    :language: yaml
    :lines: 13-17


Simple Participant
------------------

This Participant is configured with a name, a kind and the Domain Id, which is ``0`` in this case.

.. literalinclude:: ../../resources/examples/echo.yaml
    :language: yaml
    :lines: 27-29


Echo Participant
----------------

This Participant is configured to display information regarding messages received, as well as discovery events.
See :ref:`Echo Participant Configuration <user_manual_participants_echo_configuration>` for more details.

.. literalinclude:: ../../resources/examples/echo.yaml
    :language: yaml
    :lines: 35-39


Execute example
===============

For a detailed explanation on how to execute the |ddsrouter|, refer to this :ref:`section <user_manual_user_interface>`.

.. note::

    Internal entities for a specific topic are only created once a data receiver (Reader/Subscriber) is discovered.
    Hence, for these example to work, either substitute ``allowlist`` for :ref:`builtin-topics <topic_filtering>` in the
    configuration file, or launch a subscriber/listener in the same domain (``0``).

Execute with Fast DDS HelloWorld Example
----------------------------------------

Execute a Fast DDS HelloWorld example:

.. code-block:: bash

    ./DDSHelloWorldExample publisher

Execute |ddsrouter| with this configuration file (available in
``<path/to/ddsrouter_tool>/share/resources/configurations/examples/echo.yaml``).
The expected output from the DDS Router, printed by the ``Echo Participant`` is:

.. code-block:: console

    New endpoint discovered: Endpoint{01.0f.b8.d9.81.30.3d.a7.01.00.00.00|0.0.1.3;writer;DdsTopic{HelloWorldTopic;HelloWorld;Fuzzy{Level(20) TopicQoS{TRANSIENT_LOCAL;RELIABLE;SHARED;depth(5000)}}};SpecificEndpointQoS{Partitions{};OwnershipStrength{0}};Active;ParticipantId{SimpleParticipant}}.
    In Endpoint: 01.0f.b8.d9.81.30.3d.a7.01.00.00.00|0.0.1.3 from Participant: ParticipantId{SimpleParticipant} in topic: DdsTopic{HelloWorldTopic;HelloWorld;Fuzzy{Level(20) TopicQoS{VOLATILE;BEST_EFFORT;SHARED;depth(5000)}}} payload received: Payload{00 01 00 00 01 00 00 00 0b 00 00 00 48 65 6c 6c 6f 57 6f 72 6c 64 00 00} with specific qos: SpecificEndpointQoS{Partitions{};OwnershipStrength{0}}.
    In Endpoint: 01.0f.b8.d9.81.30.3d.a7.01.00.00.00|0.0.1.3 from Participant: ParticipantId{SimpleParticipant} in topic: DdsTopic{HelloWorldTopic;HelloWorld;Fuzzy{Level(20) TopicQoS{VOLATILE;BEST_EFFORT;SHARED;depth(5000)}}} payload received: Payload{00 01 00 00 02 00 00 00 0b 00 00 00 48 65 6c 6c 6f 57 6f 72 6c 64 00 00} with specific qos: SpecificEndpointQoS{Partitions{};OwnershipStrength{0}}.
    ...
    In Endpoint: 01.0f.b8.d9.81.30.3d.a7.01.00.00.00|0.0.1.3 from Participant: ParticipantId{SimpleParticipant} in topic: DdsTopic{HelloWorldTopic;HelloWorld;Fuzzy{Level(20) TopicQoS{VOLATILE;BEST_EFFORT;SHARED;depth(5000)}}} payload received: Payload{00 01 00 00 0a 00 00 00 0b 00 00 00 48 65 6c 6c 6f 57 6f 72 6c 64 00 00} with specific qos: SpecificEndpointQoS{Partitions{};OwnershipStrength{0}}.

Execute with ROS 2 demo nodes
-----------------------------

Execute a ROS 2 ``demo_nodes_cpp`` *talker* in default domain ``0``:

.. code-block:: bash

    ros2 run demo_nodes_cpp talker

Execute |ddsrouter| with this configuration file (available in
``<path/to/ddsrouter_tool>/share/resources/configurations/examples/echo.yaml``).
The expected output from the DDS Router, printed by the ``Echo Participant`` is:

.. code-block:: console

    New endpoint discovered: Endpoint{01.0f.b8.d9.b6.3a.7d.95.01.00.00.00|0.0.1.3;writer;DdsTopic{ros_discovery_info;rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_;Fuzzy{Level(20) TopicQoS{TRANSIENT_LOCAL;RELIABLE;SHARED;depth(5000)}}};SpecificEndpointQoS{Partitions{};OwnershipStrength{0}};Active;ParticipantId{SimpleParticipant}}.
    New endpoint discovered: Endpoint{01.0f.b8.d9.b6.3a.7d.95.01.00.00.00|0.0.2.4;reader;DdsTopic{ros_discovery_info;rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_;Fuzzy{Level(20) TopicQoS{TRANSIENT_LOCAL;RELIABLE;SHARED;depth(5000)}}};SpecificEndpointQoS{Partitions{};OwnershipStrength{0}};Active;ParticipantId{SimpleParticipant}}.
    ...
    New endpoint discovered: Endpoint{01.0f.b8.d9.b6.3a.7d.95.01.00.00.00|0.0.12.3;writer;DdsTopic{rt/chatter;std_msgs::msg::dds_::String_;Fuzzy{Level(20) TopicQoS{VOLATILE;RELIABLE;SHARED;depth(5000)}}};SpecificEndpointQoS{Partitions{};OwnershipStrength{0}};Active;ParticipantId{SimpleParticipant}}.
    In Endpoint: 01.0f.b8.d9.b6.3a.7d.95.01.00.00.00|0.0.12.3 from Participant: ParticipantId{SimpleParticipant} in topic: DdsTopic{rt/chatter;std_msgs::msg::dds_::String_;Fuzzy{Level(0) TopicQoS{VOLATILE;BEST_EFFORT;SHARED;depth(1000)}}} payload received: Payload{00 01 00 00 0f 00 00 00 48 65 6c 6c 6f 20 57 6f 72 6c 64 3a 20 31 00 00} with specific qos: SpecificEndpointQoS{Partitions{};OwnershipStrength{0}}.
    In Endpoint: 01.0f.b8.d9.b6.3a.7d.95.01.00.00.00|0.0.12.3 from Participant: ParticipantId{SimpleParticipant} in topic: DdsTopic{rt/chatter;std_msgs::msg::dds_::String_;Fuzzy{Level(0) TopicQoS{VOLATILE;BEST_EFFORT;SHARED;depth(1000)}}} payload received: Payload{00 01 00 00 0f 00 00 00 48 65 6c 6c 6f 20 57 6f 72 6c 64 3a 20 32 00 00} with specific qos: SpecificEndpointQoS{Partitions{};OwnershipStrength{0}}.
    ...
