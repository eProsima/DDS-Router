.. include:: ../exports/alias.include

.. _examples_echo_example:

############
Echo Example
############

The following YAML configuration file configures a DDS Router to create a
:ref:`Simple Participant <user_manual_participants_simple>` in :term:`Domain Id` ``0`` and an
:ref:`Echo Participant <user_manual_participants_echo>` that will print in ``stdout``
every message received in Domain ``0``, as well as information regarding discovery events.

.. literalinclude:: ../../resources/examples/echo.yaml
    :language: yaml
    :lines: 5-35

Configuration
=============

Allowed Topics
--------------

This section lists the :term:`Topics <Topic>` that the DDS Router will route from
one Participant to the other.
Topic ``hello_world_topic`` (from the *Fast DDS* ``hello_world`` example) and ROS 2 topic ``rt/chatter`` will be forwarded from
``SimpleParticipant`` to ``EchoParticipant``, that will print the message in ``stdout``.

.. literalinclude:: ../../resources/examples/echo.yaml
    :language: yaml
    :lines: 11-13


Simple Participant
------------------

This Participant is configured with a name, a kind and the Domain Id, which is ``0`` in this case.

.. literalinclude:: ../../resources/examples/echo.yaml
    :language: yaml
    :lines: 23-25


Echo Participant
----------------

This Participant is configured to display information regarding messages received, as well as discovery events.
See :ref:`Echo Participant Configuration <user_manual_participants_echo_configuration>` for more details.

.. literalinclude:: ../../resources/examples/echo.yaml
    :language: yaml
    :lines: 31-35


Execute example
===============

For a detailed explanation on how to execute the |ddsrouter|, refer to this :ref:`section <user_manual_user_interface>`.

.. note::

    Internal entities for a specific topic are only created once a data receiver (Reader/Subscriber) is discovered.
    Hence, for this example to work, either substitute ``allowlist`` for :ref:`builtin-topics <topic_filtering>` in the
    configuration file, or launch a subscriber/listener in the same domain (``0``).

Execute with Fast DDS HelloWorld Example
----------------------------------------

Execute a Fast DDS HelloWorld example:

.. code-block:: bash

    ./<path/to/fastdds_installation>/share/fastdds/examples/cpp/hello_world/bin/hello_world publisher

Execute |ddsrouter| with this configuration file (available in
``<path/to/ddsrouter_tool>/share/resources/configurations/examples/echo.yaml``).
The expected output from the DDS Router, printed by the ``Echo Participant`` is:

.. code-block:: console

    New endpoint discovered: Endpoint{01.0f.92.94.d9.98.7e.6b.00.00.00.00|0.0.1.3;writer;Topic{hello_world_topic;HelloWorld;TopicQoS{durability(Fuzzy{Level(SET) TRANSIENT_LOCAL});reliability(Fuzzy{Level(SET) RELIABLE});ownership(Fuzzy{Level(SET) SHARED});depth(Fuzzy{Level(DEFAULT) 5000});max_tx_rate(Fuzzy{Level(DEFAULT) 0});max_rx_rate(Fuzzy{Level(DEFAULT) 0});downsampling(Fuzzy{Level(DEFAULT) 1})}(payload::rtps::v0)};SpecificEndpointQoS{Partitions{};OwnershipStrength{0}};Active;SimpleParticipant}.
    In Endpoint: 01.0f.92.94.d9.98.7e.6b.00.00.00.00|0.0.1.3 from Participant: SimpleParticipant in topic: hello_world_topic payload received: Payload{00 01 00 00 01 00 00 00 0c 00 00 00 48 65 6c 6c 6f 20 77 6f 72 6c 64 00} with specific qos: SpecificEndpointQoS{Partitions{};OwnershipStrength{0}}.
    In Endpoint: 01.0f.92.94.d9.98.7e.6b.00.00.00.00|0.0.1.3 from Participant: SimpleParticipant in topic: hello_world_topic payload received: Payload{00 01 00 00 02 00 00 00 0c 00 00 00 48 65 6c 6c 6f 20 77 6f 72 6c 64 00} with specific qos: SpecificEndpointQoS{Partitions{};OwnershipStrength{0}}.
    ...

Execute with ROS 2 demo nodes
-----------------------------

Execute a ROS 2 ``demo_nodes_cpp`` *talker* in default domain ``0``:

.. code-block:: bash

    ros2 run demo_nodes_cpp talker

Execute |ddsrouter| with this configuration file (available in
``<path/to/ddsrouter_tool>/share/resources/configurations/examples/echo.yaml``).
The ``Echo Participant`` prints the discovery and data traces of topic ``rt/chatter`` (and the other ROS 2 internal topics
discovered), with the same format as in the previous example.
