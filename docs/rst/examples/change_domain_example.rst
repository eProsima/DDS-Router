.. include:: ../exports/alias.include

.. _examples_change_domain_example:

#####################
Change Domain Example
#####################

The following YAML configuration file configures a DDS Router to create two
:ref:`Simple Participants <user_manual_participants_simple>`, one in domain ``0`` and another in domain ``1``.

.. literalinclude:: ../../resources/examples/change_domain.yaml
    :language: yaml
    :lines: 5-29

Configuration
=============

Allowed Topics
--------------

This section lists the :term:`Topics <Topic>` that the DDS Router will route from
one Participant to the other.
Topic ``HelloWorldTopic`` with datatype ``HelloWorld``,
and ROS 2 topic ``rt/chatter`` with datatype ``std_msgs::msg::dds_::String_`` will be forwarded from
one domain to the other, allowing different DDS domains to interact with each other.

.. literalinclude:: ../../resources/examples/change_domain.yaml
    :language: yaml
    :lines: 9-13


Simple Participant Domain 0
---------------------------

This Participant is configured with a name, a type and the Domain Id, which is ``0`` in this case.

.. literalinclude:: ../../resources/examples/change_domain.yaml
    :language: yaml
    :lines: 19-21


Simple Participant Domain 1
---------------------------

This Participant is configured with a name, a type and the Domain Id, which is ``1`` in this case.

.. literalinclude:: ../../resources/examples/change_domain.yaml
    :language: yaml
    :lines: 27-29


Execute example
===============

Please refer to this section for a detailed explanation on how to execute the |ddsrouter|.

.. todo:

    Link when section exists.

.. todo:

    Add link when BasicConfiguration Example is added to Fast DDS (if it happens)

    Execute with Fast DDS HelloWorld Example
    ----------------------------------------

Execute with ROS 2 demo nodes
-----------------------------

Execute a ROS 2 ``demo_nodes_cpp`` *talker* in domain ``0``:

.. code-block:: bash

    ROS_DOMAIN_ID=0 ros2 run demo_nodes_cpp talker

Execute a ROS 2 ``demo_nodes_cpp`` *listener* in domain ``1``:

.. code-block:: bash

    ROS_DOMAIN_ID=1 ros2 run demo_nodes_cpp listener

Execute |ddsrouter| with this configuration file (available in
``<path/to/ddsrouter>/resources/configurations/examples/change_domain.yaml``).
Once the |ddsrouter| is running, messages from *talker* in domain 0 will be forwarded by the Router
to the *listener* in domain 1, that will print them in ``stdout``.
