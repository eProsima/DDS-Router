.. include:: ../exports/alias.include

.. _usage_example:

################
Example of usage
################

This example will serve as a hands-on tutorial, aimed at introducing some of the key concepts and features that
|eddsrouter| has to offer.

Two disjoint DDS networks will be bridged by a pair of routers to connect the endpoints
hosted at each of the networks. In particular, two `ShapesDemo <https://www.eprosima.com/index.php/products-all/eprosima-shapes-demo>`_
instances will establish communication after proper configuration and deployment of the two aforementioned routers.

.. figure:: /rst/figures/shapesdemo_general.png

.. note::

    This example applies to both LAN and WAN scenarios. For the WAN case, make sure that public IP addresses are used
    instead of private ones, and that the provided ports are reachable by properly configuring port forwarding in your
    Internet router devices.

Launching ShapesDemo
====================
`ShapesDemo <https://www.eprosima.com/index.php/products-all/eprosima-shapes-demo>`_ is an application that publishes
and subscribes to shapes of different colors and sizes moving on a board. This is nothing more than a graphical tool to
test the correctness of a specific DDS protocol implementation, as well as to prove interoperability with other
implementations.

Let us launch a ShapesDemo instance in one of the DDS networks, and start publishing in topics ``Square``, ``Circle``
and ``Triangle`` with default settings.

.. figure:: /rst/figures/shapesdemo_publisher.png
    :scale: 75 %

Now, run another instance in the other network and subscribe to the same topics chosen in the publisher side (use
default settings).

.. figure:: /rst/figures/shapesdemo_subscriber.png
    :scale: 75 %

.. note::

    If you are trying this example in the LAN scenario, make sure a different DDS domain id is used in one of the
    ShapesDemo instances in order to avoid direct communication between them.

Router configuration
====================
A configuration file is all that is required in order to run a |ddsrouter| instance. In a nutshell, each router will
forward messages if their associated topics match the filters contained in its ``allowlist``. A ``blocklist`` may also
be specified, on its own or in addition to an ``allowlist``, but we will not be covering this here.

Let us first add only the ``Square`` topic:

.. literalinclude:: ../../resources/getting_started/client-ddsrouter.yaml
    :language: yaml
    :lines: 5-6

Apart from selecting on which topics we wish to send/receive data, we must configure as well the participants that will
ultimately perform communication. Each router instance will contain a :ref:`simple <user_manual_participants_simple>`
and a :ref:`WAN <user_manual_participants_wan>` participants. In brief, the simple participants will be in charge of
locally communicating with the corresponding ShapesDemo application, while the WAN participants will be the ones
bridging the connection between the two DDS networks.

.. figure:: /rst/figures/shapesdemo_detailed.png

The only configuration required for simple participants is the DDS ``domain`` identifier.

.. literalinclude:: ../../resources/getting_started/client-ddsrouter.yaml
    :language: yaml
    :lines: 10-12

If launching the two routers in the same LAN, set a different ``domain`` id in one of the two configuration files (same
as the one previously set for ShapesDemo).

The configuration of WAN participants is more complex, and we will not be covering it here in detail. In short, in this
example both WAN participants will communicate through UDP, with one being the client and the other one having the
server role. Both participants are required to have a ``listening address`` (for the UDP case) where they will expect to
receive traffic, and a ``connection address`` in the case of the client, which points at the server's
``listening address``. Refer to :ref:`WAN Participant <user_manual_participants_wan>` and
:ref:`WAN Configuration <user_manual_wan_configuration>` for more information. You may also have a look at
:ref:`WAN Example <examples_wan_example>` for a detailed explanation on how to configure this kind of participant.

Following is an example of client and server configuration files:

.. literalinclude:: ../../resources/getting_started/client-ddsrouter.yaml
    :language: yaml

.. literalinclude:: ../../resources/getting_started/server-ddsrouter.yaml
    :language: yaml


Router execution
================
Now, with the configuration files ready, launching a |ddsrouter| instance is as easy as executing the following command:

.. code-block:: bash

    ddsrouter -c config-file.yaml

After setting up both routers, communication between the two ShapesDemo instances should have been established so the
square shape is now visible in the subscriber's panel.

.. figure:: /rst/figures/shapesdemo_square.png

|ddsrouter| supports the dynamic addition/deletion of topics at runtime (see
:ref:`Reload Topics <user_manual_user_interface_reload_topics>`). Let us test this feature by adding the circle topic to
the allowlist of both routers. Also, by removing the square topic (removing this topic from one of the routers'
allowlist will suffice) the square data should stop reaching the subscriber. Alternatively, the square topic may be
added to the ``blocklist``,  achieving the same effect. See :ref:`Topic Filtering <topic_filtering>` for more details
on allowlisting.

.. code-block:: yaml

    allowlist:
      - name: Circle

After applying these changes, the square should no longer be updated in the subscriber's side (appearing visible but
frozen), while the circle should.

.. figure:: /rst/figures/shapesdemo_circle.png

Please feel free to explore sections :ref:`Examples <examples_echo_example>` and :ref:`Use Cases <ros_cloud>` for more
information on how to configure and set up a router, as well as to discover multiple scenarios where |ddsrouter| may
serve as a useful tool.
