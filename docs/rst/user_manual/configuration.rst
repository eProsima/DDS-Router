.. include:: ../exports/alias.include

.. _user_manual_configuration:

########################
DDS Router Configuration
########################

A |ddsrouter| is configured by a *.yaml* configuration file.
This *.yaml* file contains all the information regarding the |ddsrouter| configuration, such as topics filtering
and :term:`Participants <Participant>` configurations. Configuration files may be easily validated by using the
:ref:`yaml_validator` tool.

.. contents::
    :local:
    :backlinks: none
    :depth: 2

Configuration version
=====================

The YAML Configuration supports a ``version`` value to identify the configuration version to parse the file.
In future releases the YAML format (some key words,
fields, etc.) may change.
This value allows users to keep using the same YAML file with an old configuration format, maintaining compatibility
with future releases.

.. list-table::
    :header-rows: 1

    *   - Configuration Versions
        - String in ``version`` tag
        - |ddsrouter| activation release

    *   - version 2.0
        - ``v2.0``
        - *v0.2.0*

    *   - version 3.0 (default)
        - ``v3.0``
        - *v0.3.0*

**Current configuration version is v3.0**.
This is the configuration version that is described along this page.

.. note::

    The current default version when the tag ``version`` is not set is *v3.0*.

.. warning::

    **Deprecation Warning**.
    In future releases, the tag ``version`` will be mandatory.

.. warning::

    **Deprecation warning**.
    Update to  version `v3.0` since `v1.0` is no longer supported.


.. _thread_configuration:

Specs Configuration
===================

The YAML Configuration supports a ``specs`` **optional** tag that contains certain options related with the
overall configuration of the DDS Router instance to run.
The values available to configure are:

Number of Threads
-----------------

``specs`` supports a ``threads`` **optional** value that allows the user to set a maximum number of threads
for the internal :code:`ThreadPool`.
This ThreadPool allows to limit the number of threads spawned by the application.
This improves the performance of the data transmission between Participants.

This value should be set by each user depending on each system's characteristics.
In case this value is not set, the default number of threads used is :code:`12`.

.. _history_depth_configuration:

Maximum History Depth
---------------------

``specs`` supports a ``max-depth`` **optional** value that configures the history size
of the Fast DDS internal entities.
By default, the depth of every RTPS History instance is :code:`5000`, which sets a constraint on the maximum number of
samples a |ddsrouter| instance can deliver to late joiner Readers configured with ``TRANSIENT_LOCAL``
`DurabilityQosPolicyKind <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/standardQosPolicies.html#durabilityqospolicykind>`_.
Its value should be decreased when the sample size and/or number of created endpoints (increasing with the number of
topics and |ddsrouter| participants) are as big as to cause memory exhaustion issues.
Likewise, one may choose to increase this value if wishing to deliver a greater number of samples to late joiners and
enough memory is available.


.. _user_manual_configuration_load_xml:

Load XML Configuration
======================

Fast DDS supports configuration of its internal entities (:term:`DomainParticipant`, :term:`DataWriter`, etc.) via XML Profiles.
These XML files contain different profiles that set specific QoS, and entities can be created following such profiles.
These XML files can be loaded in the process by their *default file name* or by an environment variable.
Check the `Fast DDS documentation <https://fast-dds.docs.eprosima.com/en/latest/fastdds/xml_configuration/xml_configuration.html>` for more information.

Another way of loading these XML configurations is using the |ddsrouter| yaml configuration.
The YAML Configuration supports a ``xml`` **optional** tag that contains certain options to load Fast DDS XML configurations.
XML configurations are then used to configure an :ref:`XML Participant <user_manual_participants_xml>`.


Load XML Files
--------------

Under the **optional** tag `files`, a list can be set with the names of files to load XML from.

Raw XML
-------

Under the **optional** tag `raw`, an XML configuration (with the same format as an XML file) can be set as a string to be loaded.

.. code-block:: yaml

    xml:

      files:
          - "./xml_configuration.xml"

      raw: |
          <?xml version="1.0" encoding="UTF-8" ?>
          <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles" >
              <participant profile_name="custom_participant_configuration">
                  <domainId>1</domainId>
                  <rtps></rtps>
              </participant>
          </profiles>

.. note::

    The |ddsrouter| does not modify any XML configuration in a :ref:`user_manual_participants_xml`.
    However, there are some QoS that can affect performance.
    These QoS should be configured by the user explicitly.
    Check :ref:`user_manual_participants_xml_profiles`.

.. _topic_filtering:

Built-in Topics
===============

Apart from the dynamic creation of Endpoints in DDS Topics discovered,
the discovery phase can be accelerated by using the builtin topic list (``builtin-topics``).
By defining topics in this list, the DDS router will create the DataWriters and DataReaders in router initialization.
This feature also allows to manually force the QoS of a specific topic, so the entities created in such a topic
will follow the specified QoS and not the one first discovered.

Topic Quality of Service
------------------------

For every topic contained in this list, both ``name`` and ``type`` must be specified and contain no wildcard
characters. The entry ``keyed`` is optional, and defaults to ``false``.
Apart from these values, the tag ``qos`` under each topic allows to configure the following values:

.. list-table::
    :header-rows: 1

    *   - Quality of Service
        - Yaml tag
        - Data type
        - Default value
        - QoS set

    *   - Reliability
        - ``reliability``
        - *bool*
        - ``false``
        - ``RELIABLE`` / ``BEST_EFFORT``

    *   - Durability
        - ``durability``
        - *bool*
        - ``false``
        - ``TRANSIENT_LOCAL`` / ``VOLATILE``

    *   - History Depth
        - ``depth``
        - *integer*
        - :ref:`history_depth_configuration`
        - Writers and Readers History Depth

    *   - Partitions
        - ``partitions``
        - *bool*
        - ``false``
        - Topic with / without partitions

    *   - Ownership
        - ``ownership``
        - *bool*
        - ``false``
        - ``EXCLUSIVE_OWNERSHIP_QOS`` / ``SHARED_OWNERSHIP_QOS``

    *   - Key
        - ``keyed``
        - *bool*
        - ``false``
        - Topic with / without key

The entry ``keyed`` determines whether the corresponding topic is `keyed <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/topic/typeSupport/typeSupport.html#data-types-with-a-key>`_
or not. See the :term:`Topic` section for further information about the topic.


.. code-block:: yaml

    builtin-topics:
      - name: HelloWorldTopic
        type: HelloWorld
        qos:
          reliability: true  # Use QoS RELIABLE
          durability: true   # Use QoS TRANSIENT_LOCAL
          depth: 100         # Use History Depth 100
          partitions: true   # Topic with partitions
          ownership: false   # Use QoS SHARED_OWNERSHIP_QOS
          keyed: false       # Topic without keys


Topic Filtering
===============

|ddsrouter| includes a mechanism to automatically detect which topics are being used in a DDS network.
By automatically detecting these topics, a |ddsrouter| creates internal DDS :term:`Writers<DataWriter>`
and :term:`Readers<DataReader>` for each topic and for each Participant in order to relay the data published on each
discovered topic.

.. note::

    DDS Router entities are created with the QoS of the first Subscriber found in this Topic.

|ddsrouter| allows filtering of DDS :term:`Topics<Topic>`, that is, it allows to define which DDS Topics are going to be
relayed by the application.
This way, it is possible to define a set of rules in |ddsrouter| to filter those data samples the user does not wish to
forward.

It is not mandatory to define such set of rules in the configuration file. In this case, a DDS Router will forward all
the data published under the topics that it automatically discovers within the DDS network to which it connects.

To define these data filtering rules based on the Topics to which they belong, three lists are available:

* Allowed topics list (``allowlist``)
* Block topics list (``blocklist``)
* Builtin topics list (``builtin-topics``)

These three lists of topics listed above are defined by a tag in the *YAML* configuration file, which defines a
*YAML* vector (``[]``).
This vector contains the list of topics for each filtering rule.
Each Topic is determined by its entries ``name`` and ``type``, with only the first one being mandatory.

.. list-table::
    :header-rows: 1

    *   - Topic entries
        - Data type
        - Default value

    *   - ``name``
        - ``string``
        - \-

    *   - ``type``
        - ``string``
        - ``"*"``

.. note::

    Tags ``allowlist``, ``blocklist`` and ``builtin-topics`` must be at yaml base level (it must not be inside any
    other tag).

.. note::

    Placing quotation marks around values in a YAML file is generally optional. However, values containing wildcard
    characters must be enclosed by single or double quotation marks.

Allow topic list (``allowlist``)
--------------------------------
This is the list of topics that |ddsrouter| will forward, i.e. the data published under the topics matching the
expressions in the ``allowlist`` will be relayed by |ddsrouter|.

.. note::

    If no ``allowlist`` is provided, data will be forwarded for all topics (unless filtered out in ``blocklist``).


Block topic list (``blocklist``)
--------------------------------
This is the list of topics that the |ddsrouter| will block, that is, all data published under the topics matching the
filters specified in the ``blocklist`` will be discarded by the |ddsrouter| and therefore will not be relayed.

This list takes precedence over the ``allowlist``.
If a topic matches an expression both in the ``allowlist`` and in the ``blocklist``, the ``blocklist`` takes precedence,
causing the data under this topic to be discarded.


Examples of usage
-----------------

The following is an example of how to use the ``allowlist``, ``blocklist`` and ``builtin-topics`` configurations to
setup the |ddsrouter| filtering rules.

Dynamic topic discovery example
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This example shows how the |ddsrouter| is initially configured to forward the ``rt/chatter`` topic (default ROS 2
topic for ``talker`` and ``listener``) with type name ``std_msgs::msg::dds_::String_``, while the rest of the
topics in the DDS network are expected to be dynamically discovered.
Additionally, two rules are specified in the ``blocklist`` in order to filter out messages of no interest to the user
(in this case ROS2 services related topics).

.. code-block:: yaml

    builtin-topics:
      - name: rt/chatter
        type: std_msgs::msg::dds_::String_

    blocklist:
      - name: "rq/*"
      - name: "rr/*"


Allowlist and blocklist collision
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In the following example, the ``HelloWorldTopic`` topic is both in the ``allowlist`` and (implicitly) in the
``blocklist``, so according to the ``blocklist`` preference rule this topic is blocked.
Moreover, only the topics present in the allowlist are relayed, regardless of whether more topics are dynamically
discovered in the DDS network.
In this case the forwarded topics are ``AllowedTopic1`` with data type ``Allowed``
and ``AllowedTopic2`` regardless of its data type.

.. code-block:: yaml

    allowlist:
      - name: AllowedTopic1
        type: Allowed
      - name: AllowedTopic2
        type: "*"
      - name: HelloWorldTopic
        type: HelloWorld

    blocklist:
      - name: "*"
        type: HelloWorld


Participant Configuration
=========================

At the yaml base level, along with the ``builtin-topics`` tag, there will be the tag ``participants`` to handle an array of :term:`Participant` configurations.
Each Participant is identified by a unique :term:`Participant Name` and requires to set the ``kind`` of the Participant.
There can be any number of Participants, and Participant kinds can be repeated.

Each Participant has its specific configuration.
Please, refer to :ref:`user_manual_participant_participant_kinds` in order to see each of the
:term:`Participant Kinds<Participant Kind>` requirements.

.. warning::

    Do not configure two Participants in a way that they can communicate to each other (e.g. two Simple participants
    in the same domain).
    This will lead to an infinite feedback loop between each other.

In the following configuration example, the |ddsrouter| will create two
:ref:`Simple Participants <user_manual_participants_simple>`, one for ``domain 0`` and one for ``domain 1``.
This is a typical use case of :ref:`DDS Domain bridge <examples_change_domain_example>`.
The topics allowed in the two domains will start communicating to each other.
Note that the communication is not P2P performed between the end-user DDS entities,
i.e. the data must reach the |ddsrouter| and this will forward the data.

.. code-block:: yaml

    participants:              # Tag to introduce the participants configurations array

    ################

      - name: Participant0     # Participant Name = Participant0
        kind: local            # Participant Kind = simple
        domain: 0              # DomainId = 0

    ################

      - name: my_custom_part   # Participant Name = my_custom_part
        kind: simple           # Participant Kind = echo
        domain: 1              # DomainId = 1

The first Participant `Participant0` has Participant Name *Participant0* and is configured to be of the *simple*
Participant Kind, and to communicate locally in domain 0.
The second Participant has Participant Name *simple* and it is configured to be of the *simple* kind and to communicate
locally with domain 1.

.. _user_manual_configuration_domain_id:

Domain Id
---------

Tag ``domain`` configures the :term:`Domain Id` of a specific Participant.
Be aware that some Participants (e.g. Discovery Servers) do not need a Domain Id configuration.

.. code-block:: yaml

    domain: 101


.. _user_manual_configuration_ignore_participant_flags:

Ignore Participant Flags
------------------------

A set of discovery traffic filters can be defined for :ref:`Simple Participants <user_manual_participants_simple>` in order to add an extra level of isolation.
This configuration option can be set through the ``ignore-participant-flags`` tag:

.. code-block:: yaml

    ignore-participant-flags: no_filter                          # No filter (default)
    # or
    ignore-participant-flags: filter_different_host              # Discovery traffic from another host is discarded
    # or
    ignore-participant-flags: filter_different_process           # Discovery traffic from another process on same host is discarded
    # or
    ignore-participant-flags: filter_same_process                # Discovery traffic from own process is discarded
    # or
    ignore-participant-flags: filter_different_and_same_process  # Discovery traffic from own host is discarded

See `Ignore Participant Flags <https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/general_disc_settings.html?highlight=ignore%20flags#ignore-participant-flags>`_ for more information.


.. _user_manual_configuration_custom_transport_descriptors:

Custom Transport Descriptors
----------------------------

By default, :ref:`Simple Participants <user_manual_participants_simple>` are created with enabled `UDP <https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/udp/udp.html>`_ and `Shared Memory <https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/shared_memory/shared_memory.html>`_ transport descriptors.
The use of one or the other for communication will depend on the specific scenario, and whenever both are viable candidates, the most efficient one (Shared Memory Transport) is automatically selected.
However, a user may desire to force the use of one of the two, which can be accomplished via the ``transport`` configuration tag in :ref:`Simple Participants <user_manual_participants_simple>`.

.. code-block:: yaml

    transport: builtin    # UDP & SHM (default)
    # or
    transport: udp        # UDP only
    # or
    transport: shm        # SHM only

.. warning::

    Participants configured with ``transport: shm`` will only communicate with applications using Shared Memory Transport exclusively (with disabled UDP transport).


.. _user_manual_configuration_interface_whitelist:

Interface Whitelist
-------------------

Optional tag ``whitelist-interfaces`` allows to limit the network interfaces used by UDP and TCP transport.
This may be useful to only allow communication within the host (note: same can be done with :ref:`user_manual_configuration_ignore_participant_flags`), or in the WAN scenario one may choose to only communicate through the Ethernet or WiFi interface (when both available).
Example:

.. code-block:: yaml

    whitelist-interfaces:
      - "127.0.0.1"    # Localhost only

See `Interface Whitelist <https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/whitelist.html>`_ for more information.

.. warning::

    When providing an interface whitelist, external participants with which communication is desired must also be configured with interface whitelisting.


.. _user_manual_configuration_repeater:

Repeater Participant
--------------------

The optional tag ``repeater`` configures a :ref:`WAN Participant <user_manual_participants_wan>` as a *Repeater* point.
This means that this Participant will forward all the information received from its Readers to its Writers.

Check the :ref:`use_case_repeater` use case to see how the ``repeater`` Participant attribute is used.

.. code-block:: yaml

    repeater: true

.. note::

    This tag is only supported in configuration versions above v2.0.


.. _user_manual_configuration_network_address:

Network Address
---------------

Network Addresses are elements that can be configured for specific Participants.
An Address is defined by:

* *IP*: IP of the host (public IP in case of WAN communication).
* *Port*: Port where the Participant is listening.
* *External Port*: Public port accessible for external entities (only for TCP).
* *Transport Protocol*: ``UDP`` or ``TCP``.
  If it is not set, it would be chosen by default depending on the Participant Kind.
* *IP version*: ``v4`` or ``v6``.
  If it is not set, it would be chosen depending on the *IP* string format.
* *Domain Name*: Domain's unique name to ask the DNS server for the related IP.
  This field is ignored if ``ip`` is specified.

.. code-block:: yaml

    ip: 127.0.0.1
    port: 11666
    external-port: 11670
    transport: tcp
    ip-version: v4

    ################

    ip: 2001:4860:4860::8844      # Recognized as IPv6
    port: 1616
    transport: udp

    ################

    domain: localhost             # DNS call return value = 127.0.0.1
    port: 33333                   # Uses UDP by default


External Port
-------------

External port is used to configure a Server (Discovery Server or Initial Peers) that runs under a NAT and uses
TCP transport.
This value could be set in a TCP listening address to differentiate the public and the internal port.
**In case this value is not set, the external port is considered to be the same as the internal one.**
If both ports coincide, external and internal port in the network router port forwarding rules must coincide.
For more information, check section :ref:`user_manual_wan_configuration_nat_traversal_port_forwarding_external_port`.


.. _user_manual_configuration_discovery_server_guidprefix:

Discovery Server GuidPrefix
---------------------------

A :term:`Discovery Server` requires a DDS :term:`GuidPrefix` in order for other Participants to connect to it.
Under the ``discovery-server-guid`` tag, there are several possibilities for configuring a GuidPrefix.


Discovery Server GuidPrefix by string
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The GuidPrefix of the Discovery Server can be configured using ``guid`` tag.
Be aware of using the correct format for GuidPrefix.
That is, 12 hexadecimal numbers (lower than ``ff``) separated with ``.``.

.. code-block:: yaml

    discovery-server-guid:
      guid: "1.f.1.0.0.0.0.0.0.0.ca.fe"       # GuidPrefix = 01.0f.01.00.00.00.00.00.00.00.ca.fe


Discovery Server GuidPrefix by Id
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Using tag ``id``, the GuidPrefix will be calculated arbitrarily using a default |ddsrouter| GuidPrefix.
This default GuidPrefix is ``01.0f.<id>.00.00.00.00.00.00.00.ca.fe``.
Default value for ``id`` is ``0``.
This entry is ignored if ``guid`` is specified.

.. code-block:: yaml

    discovery-server-guid:
      id: 13                                  # GuidPrefix = 01.0f.0d.00.00.00.00.00.00.00.ca.fe

.. note::

    In the current version of the |ddsrouter| only ids in the range 0 to 256 are allowed.
    In future releases it would be implemented to allow a wider range of ids.


ROS Discovery Server GuidPrefix
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

There is a specific GuidPrefix for ROS 2 executions which can be used with Fast DDS CLI and
ROS 2 ``ROS_DISCOVERY_SERVER`` environment variable
(`<https://fast-dds.docs.eprosima.com/en/v2.4.1/fastdds/ros2/discovery_server/ros2_discovery_server.html>`__).

The ROS 2 Discovery Server GuidPrefix is set by default to ``44.53.<id>.5f.45.50.52.4f.53.49.4d.41`` where ``<id>``
is the specific id of the Server.
This GuidPrefix also allows an ``id``` value to specify which id is used in the GuidPrefix.
The default value for ``id`` is ``0``.

.. code-block:: yaml

    discovery-server-guid:
      ros-discovery-server: true              # GuidPrefix = 44.53.x.5f.45.50.52.4f.53.49.4d.41
      id: 13                                  # GuidPrefix = 44.53.0d.5f.45.50.52.4f.53.49.4d.41


.. _user_manual_configuration_listening_addresses:

Listening Addresses
-------------------

Tag ``listening-addresses`` configures the network addresses where this Participant is going to
listen for remote Participants.
``listening-addresses`` is *key* for an array of :ref:`Network Addresses <user_manual_configuration_network_address>`.

.. code-block:: yaml

    listening-addresses:
      - ip: 127.0.0.1                # UDP by default
        port: 11667
      - ip: 2001:4860:4860::8844     # Recognized as IPv6
        port: 11666
        external-port: 11668
        transport: tcp


.. _user_manual_configuration_initial_peers_connection_addresses:

Initial Peers Connection Addresses
----------------------------------

Tag ``connection-addresses`` configure a connection with one or multiple remote WAN Participants.
``connection-addresses`` is *key* for an array of :ref:`Network Addresses <user_manual_configuration_network_address>`.

.. code-block:: yaml

    connection-addresses:
      - ip: 127.0.0.1
        port: 11666
      - ip: 2001:4860:4860::8844
        port: 11668
        transport: tcp


.. _user_manual_configuration_discovery_server_connection_addresses:

Discovery Server Connection Addresses
-------------------------------------

Tag ``connection-addresses`` configure a connection with one or multiple remote Discovery Servers.
``connection-addresses`` is the *key* for an array in which each element has a GuidPrefix referencing the Discovery
Server to connect with; and a tag ``addresses`` configuring the addresses of such Discovery Server.
Each element inside ``addresses`` must follow the configuration for :ref:`user_manual_configuration_network_address`.

.. code-block:: yaml

    connection-addresses:
      - discovery-server-guid:
          guid: 44.53.0d.5f.45.50.52.4f.53.49.4d.41
        addresses:
          - ip: 127.0.0.1
            port: 11666
      - discovery-server-guid:
          id: 4
        addresses:
          - ip: 2001:4860:4860::8888
            port: 11667
            transport: tcp
          - ip: 2001:4860:4860::8844
            port: 11668
            transport: tcp


.. _user_manual_configuration_profile:

Profile
-------

Tag ``profile`` set the :term:`QoS Profile` to create a specific Participant.
This profile must match with an existent profile loaded by XML.
It will use such profile for configuring the Participant.

.. code-block:: yaml

    profile: participant_custom_configuration


.. _user_manual_configuration_general_example:


Forwarding Routes
=================

The |ddsrouter| is capable of establishing different internal routes between its participants.
This feature enables users to only send sensible data to a set of participants.

.. note::

    By default, when the tag ``routes`` is not set, every participant receives everything.

Generic Routes
--------------

To configure a custom set of forwarding routes, use the tag ``routes`` followed by the destination participants of each source.


.. note::

    If a participant is not listed as a source, it will send the data it receives to every participant.

.. note::

    If a participant is listed as a source but it is not given any destination participants, it will not send the data it receives to any participant.


Consider the following example with three participants: ``Participant0``, ``Participant1``, and ``Participant2``.

.. code-block:: yaml

  routes:
    - src: Participant0
      dst:
        - Participant2

    - src: Participant1


When data is published:

* Participant ``Participant0`` will only forward the data it receives to participant ``Participant2``.
* Participant ``Participant1`` will not forward the data it receives to any participant, since it does not have any destination participants.
* Participant ``Participant2`` will forward the data it receives to every participant, since it does not have a forwarding route.


.. warning::

    A repeater participant with a route defined must add itself to its route's destinations.


Topic Routes
------------

Besides the generic routes just described, custom routes can also be configured for a specific topic.
To configure a custom set of forwarding routes for a specific topic, use the tag ``topic-routes``.

.. warning::

  Topic routes take precedence over generic routes.
  This means that when data is published on a topic with a topic route configured, the generic routes are ignored and the topic route is used.


Again, consider the following example with three participants: ``Participant0``, ``Participant1``, and ``Participant2``.

.. code-block:: yaml

  topic-routes:
    - name: HelloWorld
      type: HelloWorld
      routes:
        - src: Participant1
          dst:
            - Participant0

        - src: Participant2

When data is published in the topic ``HelloWorld`` with type ``HelloWorld``:

* Participant ``Participant0`` will forward the data it receives to every participant, since it does not have a forwarding route.
* Participant ``Participant1`` will only forward the data it receives to participant ``Participant0``.
* Participant ``Participant2`` will not forward the data it receives to any participant, since it does not have any destination participants.

.. warning::

  When configuring topic routes, the topic's ``type`` tag is required.


General Example
===============

A complete example of all the configurations described on this page can be found below.

.. code-block:: yaml

    # Version Latest
    version: v3.0

    # Specifications
    specs:
      threads: 10
      max-depth: 1000

    # XML configurations to load
    xml:

      # Load this file as Fast DDS XML configuration
      files:
          - "./xml_configuration.xml"

      # Load text as Fast DDS XML configuration
      raw: |
          <?xml version="1.0" encoding="UTF-8" ?>
          <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles" >
              <participant profile_name="custom_participant_configuration">
                  <domainId>1</domainId>
                  <rtps></rtps>
              </participant>
          </profiles>

    # Relay topic rt/chatter and type std_msgs::msg::dds_::String_
    # Relay topic HelloWorldTopic and type HelloWorld

    builtin-topics:

      - name: rt/chatter
        type: std_msgs::msg::dds_::String_

      - name: HelloWorldTopic
        type: HelloWorld
        qos:
          reliability: true
          durability: true

    # Do not allow ROS2 services

    blocklist:
      - name: "rr/*"
      - name: "rq/*"


    participants:

    ####################

    # Simple DDS Participant in domain 3

      - name: Participant0              # Participant Name = Participant0

        kind: local                     # Participant Kind = local (= simple)

        domain: 3                       # DomainId = 3

    ####################

    # Simple DDS Participant in domain 3

      - name: Participant1              # Participant Name = Participant1

        kind: local                     # Participant Kind = local (= simple)

        domain: 7                       # DomainId = 7

    ####################

    # Discovery Server DDS Participant with ROS GuidPrefix so a local ROS 2 Client could connect to it
    # This Discovery Server will listen in ports 11600 and 11601 in localhost

      - name: ServerROS2                # Participant Name = ServerROS2

        kind: local-discovery-server    # Participant Kind = local-discovery-server

        discovery-server-guid:
          id: 1
          ros-discovery-server: true    # ROS Discovery Server id => GuidPrefix = 44.53.01.5f.45.50.52.4f.53.49.4d.41

        listening-addresses:            # Local Discovery Server Listening Addresses
          - ip: 127.0.0.1               # IP = localhost ; Transport = UDP (by default)
            port: 11600                 # Port = 11600
          - ip: 127.0.0.1               # IP = localhost
            port: 11601                 # Port = 11601
            external-port: 11602        # External Port = 11602
            transport: tcp              # Transport = TCP

        connection-addresses:
          - discovery-server-guid:
              id: 2
              ros-discovery-server: true
            addresses:
              - domain: "localhost"
                port: 22000

    ####################

    # Participant that will communicate with a DDS Router in a different LAN.
    # This Participant will work as the remote DDS Router Client, so it sets the connection address of the remote one.

      - name: Wan                       # Participant Name = Wan

        kind: wan-ds                     # Participant Kind = Discovery Server WAN

        discovery-server-guid:
          id: 2                         # Internal WAN Discovery Server id => GuidPrefix = 01.0f.02.00.00.00.00.00.00.00.ca.fe

        connection-addresses:           # WAN Discovery Server Connection Addresses
          - discovery-server-guid:
              id: 4                     # External WAN Discovery Server id => GuidPrefix = 01.0f.04.00.00.00.00.00.00.00.ca.fe
            addresses:
              - ip: 8.8.8.8             # IP = 8.8.8.8
                port: 11666             # Port = 11666
                transport: udp          # Transport = UDP


    ####################

    # Participant that will use a user set configuration via QoS Profile.

      - name: xml_participant                       # Participant Name = xml_participant

        kind: xml

        profile: custom_participant_configuration   # Configure participant with this profile

    # Custom generic forwarding route.

    routes:

      - src: Participant0
        dst:
          - Participant1

    # Custom topic forwarding route.

    topic-routes:

      - name: HelloWorld
        type: HelloWorld
        routes:
          - src: Participant0
            dst:
              - Participant1
