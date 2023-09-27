.. include:: ../exports/alias.include

.. _examples_forwarding_routes:

#################
Forwarding Routes
#################

The following YAML configuration file configures a DDS Router to create two :ref:`Simple Participants <user_manual_participants_simple>` in domains ``0`` and ``1``.
It then establishes a generic forwarding route between them, and a topic forwarding route for the topic ``Circle``.

.. note::

    This configuration enables participants in different domains to publish and subscribe to topics, as in :ref:`examples_change_domain_example`.
    The difference is that in this example we will use forwarding routes to define the participants we want to send messages to.

.. literalinclude:: ../../resources/examples/forwarding_routes.yaml
    :language: yaml
    :lines: 5-69

Configuration
=============

Simple Participants
-------------------

These participants are configured with a name, a kind (``local``), and a domain id (``0`` and ``1``).

.. literalinclude:: ../../resources/examples/forwarding_routes.yaml
    :language: yaml
    :lines: 9-27


Generic Forwarding Routes
-------------------------

We define the generic forwarding routes under the tag ``routes``.
This route is configured so that ``SimpleParticipant_1`` subscribes to the data published by ``SimpleParticipant_0``.

.. literalinclude:: ../../resources/examples/forwarding_routes.yaml
    :language: yaml
    :lines: 34-40

This route is configured so that ``SimpleParticipant_1`` does not publish the data it receives.
Thus, a subscriber in domain ``0`` would not receive the data published in domain ``1``.

.. literalinclude:: ../../resources/examples/forwarding_routes.yaml
    :language: yaml
    :lines: 42-45


Topic Forwarding Routes
-----------------------

We define the topic forwarding routes under the tag ``topic-routes`` by declaring the topic's ``name``, ``type``, and ``routes``.

.. literalinclude:: ../../resources/examples/forwarding_routes.yaml
    :language: yaml
    :lines: 48-55

.. note::

    The ``type`` tag is required.
    The topic forwarding route will not be set for a topic with the same ``name`` but a different ``type``.

Then, we declare the route for each participant.

.. warning::

    When there is not a topic forwarding route for a specific topic, the generic forwarding route will be completely ignored and the topic forwarding route will be used instead.

This route is configured so that ``SimpleParticipant_0`` will subscribe to the data published by ``SimpleParticipant_1``.

.. literalinclude:: ../../resources/examples/forwarding_routes.yaml
    :language: yaml
    :lines: 57-63

This route is configured so that ``SimpleParticipant_0`` does not forward the data it receives.

.. literalinclude:: ../../resources/examples/forwarding_routes.yaml
    :language: yaml
    :lines: 65-69


Execute example
===============

Please refer to the :ref:`User Interface <user_manual_user_interface>` section for a detailed explanation on how to execute the
|ddsrouter|.

Execute with Fast DDS Basic Configuration Example
-------------------------------------------------

To check that the generic forwarding routes are working we are going to execute two examples.
In the first one, we will set up a *publisher* in domain ``0``, a *subscriber* in domain ``1``, and check that the subscriber receives the publications.
In the second one, we will set up a *publisher* in domain ``1``, a *subscriber* in domain ``0``, and check that the subscriber does not receive the publications, since there are no generic forwarding routes from domain ``1`` to domain ``0``.

Publish in domain 0 and subscribe in domain 1
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Execute a |fastdds| ``BasicConfigurationExample`` *publisher* in domain ``0``:

.. code-block:: bash

    ./BasicConfigurationExample publisher --domain 0

Execute a |fastdds| ``BasicConfigurationExample`` *subscriber* in domain ``1``:

.. code-block:: bash

    ./BasicConfigurationExample subscriber --domain 1

Execute the |ddsrouter| with the configuration file available at ``<path/to/ddsrouter_tool>/share/resources/configurations/examples/forwarding_routes.yaml``.
Once the |ddsrouter| is running, messages from the *publisher* in domain ``0`` will be forwarded by the |ddsrouter| to the *subscriber* in domain ``1``, that will print them in ``stdout``.

Publish in domain 1 and subscribe in domain 0
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Execute a |fastdds| ``BasicConfigurationExample`` *publisher* in domain ``1``:

.. code-block:: bash

    ./BasicConfigurationExample publisher --domain 1

Execute a |fastdds| ``BasicConfigurationExample`` *subscriber* in domain ``0``:

.. code-block:: bash

    ./BasicConfigurationExample subscriber --domain 0

Execute the |ddsrouter| with this configuration file (available in ``<path/to/ddsrouter_tool>/share/resources/configurations/examples/forwarding_routes.yaml``).
Once the |ddsrouter| is running, nothing should happen, since there are no generic forwarding routes from domain ``1`` to domain ``0``.


Execute with Fast DDS Basic Configuration Example on topic Circle
-----------------------------------------------------------------

To check that the topic forwarding routes are working we are going to execute two examples.
In the first one, we will set up a *publisher* in domain ``1``, a *subscriber* in domain ``0``, and check that the subscriber receives the publications on topic ``Circle``.
In the second one, we will set up a *publisher* in domain ``0``, a *subscriber* in domain ``1``, and check that the subscriber does not receive the publications, since there are no topic forwarding routes from domain ``0`` to domain ``1`` on topic ``Circle``.

Publish in domain 1 and subscribe in domain 0
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Execute a |fastdds| ``BasicConfigurationExample`` *publisher* in domain ``1`` on topic ``Circle``:

.. code-block:: bash

    ./BasicConfigurationExample publisher --domain 1 --topic Circle

Execute a |fastdds| ``BasicConfigurationExample`` *subscriber* in domain ``0`` on topic ``Circle``:

.. code-block:: bash

    ./BasicConfigurationExample subscriber --domain 0 --topic Circle

Execute the |ddsrouter| with the configuration file available at ``<path/to/ddsrouter_tool>/share/resources/configurations/examples/forwarding_routes.yaml``.
Once the |ddsrouter| is running, messages from the *publisher* in domain ``1`` on topic ``Circle`` will be forwarded by the |ddsrouter| to the *subscriber* in domain ``0``, that will print them in ``stdout``.

Publish in domain 0 and subscribe in domain 1
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Execute a |fastdds| ``BasicConfigurationExample`` *publisher* in domain ``0`` on topic ``Circle``:

.. code-block:: bash

    ./BasicConfigurationExample publisher --domain 0 --topic Circle

Execute a |fastdds| ``BasicConfigurationExample`` *subscriber* in domain ``1`` on topic ``Circle``:

.. code-block:: bash

    ./BasicConfigurationExample subscriber --domain 1 --topic Circle

Execute the |ddsrouter| with this configuration file (available in ``<path/to/ddsrouter_tool>/share/resources/configurations/examples/forwarding_routes.yaml``).
Once the |ddsrouter| is running, nothing should happen, since there are no topic forwarding routes on topic ``Circle`` from domain ``0`` to domain ``1``.
