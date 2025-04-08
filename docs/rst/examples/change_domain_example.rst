.. include:: ../exports/alias.include

.. _examples_change_domain_example:

#####################
Change Domain Example
#####################

The following YAML configuration file configures a DDS Router to create two
:ref:`Simple Participants <user_manual_participants_simple>`, one in domain ``0`` and another in domain ``1``.

.. literalinclude:: ../../resources/examples/change_domain_allowlist.yaml
    :language: yaml
    :lines: 5-31

Configuration
=============

Allowed Topics
--------------

This section lists the :term:`Topics <Topic>` that the DDS Router will route from
one Participant to the other.
Topic ``HelloWorldTopic`` and ROS 2 topic ``rt/chatter`` will be forwarded from
one domain to the other, allowing different DDS domains to interact with each other.

.. literalinclude:: ../../resources/examples/change_domain_allowlist.yaml
    :language: yaml
    :lines: 9-11


Simple Participant Domain 0
---------------------------

This Participant is configured with a name, a kind, and a Domain Id, which is ``0`` in this case.

.. literalinclude:: ../../resources/examples/change_domain_allowlist.yaml
    :language: yaml
    :lines: 21-23


Simple Participant Domain 1
---------------------------

This Participant is configured with a name, a kind, and a Domain Id, which is ``1`` in this case.

.. literalinclude:: ../../resources/examples/change_domain_allowlist.yaml
    :language: yaml
    :lines: 29-31


Execute example
===============

Please refer to section :ref:`User Interface <user_manual_user_interface>` for a detailed explanation on how to execute the
|ddsrouter|.

Execute with Fast DDS Configuration Example
-------------------------------------------

Execute a |fastdds| ``configuration`` example  *publisher* in domain ``0``:

.. code-block:: bash

    ./<path/to/fastdds_installation>/share/fastdds/examples/cpp/configuration/bin/configuration publisher --domain 0 --name HelloWorldTopic

Execute a |fastdds| ``configuration`` example *subscriber* in domain ``1``:

.. code-block:: bash

    ./<path/to/fastdds_installation>/share/fastdds/examples/cpp/configuration/bin/configuration subscriber --domain 1 --name HelloWorldTopic

Execute the |ddsrouter| with this configuration file (available in
``<path/to/ddsrouter_tool>/share/resources/configurations/examples/change_domain_allowlist.yaml``).
Once the |ddsrouter| is running, messages from *publisher* in domain 0 will be forwarded by the Router
to the *subscriber* in domain 1 that will print them in ``stdout``.

There is also available an example without ``allowlist`` (available in
``<path/to/ddsrouter_tool>/share/resources/configurations/examples/change_domain.yaml``).
In this case, the topics of the DDS network are dynamically discovered by the DDS Router.
Since there is no allowlist, the data from all the topics found are forwarded.

Execute with ROS 2 demo nodes
-----------------------------

Execute a ROS 2 ``demo_nodes_cpp`` *talker* in domain ``0``:

.. code-block:: bash

    ROS_DOMAIN_ID=0 ros2 run demo_nodes_cpp talker

Execute a ROS 2 ``demo_nodes_cpp`` *listener* in domain ``1``:

.. code-block:: bash

    ROS_DOMAIN_ID=1 ros2 run demo_nodes_cpp listener

Execute the |ddsrouter| with this configuration file (available in
``<path/to/ddsrouter_tool>/share/resources/configurations/examples/change_domain_allowlist.yaml``).
Once the |ddsrouter| is running, messages from *talker* in domain 0 will be forwarded by the Router
to the *listener* in domain 1 that will print them in ``stdout``.

There is also available an example without ``allowlist`` (available in
``<path/to/ddsrouter_tool>/share/resources/configurations/examples/change_domain.yaml``).
In this case, the topics of the DDS network are dynamically discovered by the DDS Router.
Since there is no allowlist, the data from all the topics found are forwarded.
