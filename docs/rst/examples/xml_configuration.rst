.. include:: ../exports/alias.include

.. _examples_xml_configuration:

#############################
XML Participant Configuration
#############################

The following YAML configuration file configures a DDS Router to create a :ref:`Simple Participant <user_manual_participants_simple>` in domain ``0``,
and another participant configured by :ref:`XML <user_manual_participants_xml>` to communicate in domain ``1``.
The result of executing this configuration is similar to the one in :ref:`examples_change_domain_example`.
It generates a bridge between two different domains (``0`` & ``1``).

.. literalinclude:: ../../resources/examples/xml.yaml
    :language: yaml
    :lines: 5-37

Configuration
=============

Fast DDS XML Configuration
--------------------------

|fastdds| supports loading XML configuration files to load profiles.
These profiles are used to configure different DomainParticipants using profile name.
Loading an XML file or setting the raw xml file in the |ddsrouter| YAML configuration file allows to load such profiles.
Here there are the 2 ways to load them.
For more information check :ref:`following section <user_manual_configuration_load_xml>`.

.. literalinclude:: ../../resources/examples/xml.yaml
    :language: yaml
    :lines: 7-17


Simple Participant Domain 0
---------------------------

This Participant is configured with a name, a kind and the Domain Id, which is ``0`` in this case.

.. literalinclude:: ../../resources/examples/xml.yaml
    :language: yaml
    :lines: 27-29


XML Participant Domain 1
------------------------

This Participant is configured with a name, a kind and the XML profile tag that will be used to configure it.
As the XML loaded in this same YAML file configures profile ``custom_participant_configuration`` as default Participant in domain ``1``, this Participant will work as a ``local`` participant in such domain.

.. literalinclude:: ../../resources/examples/xml.yaml
    :language: yaml
    :lines: 35-37


Execute example
===============

Please refer to this :ref:`section <user_manual_user_interface>` for a detailed explanation on how to execute the
|ddsrouter|.

Execute with Fast DDS Configuration Example
-------------------------------------------

Execute a |fastdds| ``configuration`` example  *publisher* in domain ``0``:

.. code-block:: bash

    ./<path/to/fastdds_installation>/share/fastdds/examples/cpp/configuration/bin/configuration publisher --domain 0

Execute a |fastdds| ``configuration`` example *subscriber* in domain ``1``:

.. code-block:: bash

    ./<path/to/fastdds_installation>/share/fastdds/examples/cpp/configuration/bin/configuration subscriber --domain 1

Execute |ddsrouter| with this configuration file (available in ``<path/to/ddsrouter_tool>/share/resources/configurations/examples/xml.yaml``).
Once the |ddsrouter| is running, messages from *publisher* in domain 0 will be forwarded by the Router to the *subscriber* in domain 1, that will print them in ``stdout``.


Execute with ROS 2 demo nodes
-----------------------------

Execute a ROS 2 ``demo_nodes_cpp`` *talker* in domain ``0``:

.. code-block:: bash

    ROS_DOMAIN_ID=0 ros2 run demo_nodes_cpp talker

Execute a ROS 2 ``demo_nodes_cpp`` *listener* in domain ``1``:

.. code-block:: bash

    ROS_DOMAIN_ID=1 ros2 run demo_nodes_cpp listener

Execute |ddsrouter| with this configuration file (available in ``<path/to/ddsrouter_tool>/share/resources/configurations/examples/xml.yaml``).
Once the |ddsrouter| is running, messages from *talker* in domain 0 will be forwarded by the Router to the *listener* in domain 1, that will print them in ``stdout``.
