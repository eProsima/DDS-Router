.. include:: ../exports/alias.include
.. include:: ../exports/roles.include

.. _user_manual_configuration:

#############
Configuration
#############

A |ddsrouter| is configured by a *.yaml* configuration file.
This *.yaml* file contains all the information regarding the |ddsrouter| configuration, such as topics filtering and :term:`Participants <Participant>` configurations.

.. contents::
    :local:
    :backlinks: none
    :depth: 2

Configuration version
=====================

The YAML Configuration supports a ``version`` value to identify the configuration version to parse the file.
In future releases the YAML format (some key words, fields, etc.) may change.
This value allows users to keep using the same YAML file with an old configuration format, maintaining compatibility with future releases.

..
    .. list-table::
        :header-rows: 1

        *   - Configuration Versions
            - String in ``version`` tag
            - |ddsrouter| activation release

        *   - version 4.0
            - ``v4.0``
            - *v3.1.0*


**The current configuration version is v5.0**.
This is the configuration version that is described along this page.

.. note::

    The current default version when the tag ``version`` is not set is *v5.0*.

.. warning::

    **Deprecation warning**.
    Version `v4.0` is deprecated and will be removed in a future release, please update to version `v5.0`.

.. _user_manual_configuration_load_xml:

Load XML Configuration
======================

Fast DDS supports configuration of its internal entities (:term:`DomainParticipant`, :term:`DataWriter`, etc.) via XML Profiles.
These XML files contain different profiles that set specific QoS, and entities can be created following such profiles.
These XML files can be loaded in the process by their *default file name* or by an environment variable.
Check the `Fast DDS documentation <https://fast-dds.docs.eprosima.com/en/latest/fastdds/xml_configuration/xml_configuration.html>`_ for more information.

Another way of loading these XML configurations is using the |ddsrouter| yaml configuration.
The YAML Configuration supports a ``xml`` **optional** tag that contains certain options to load Fast DDS XML configurations.
XML configurations are then used to configure an :ref:`XML Participant <user_manual_participants_xml>`.


Load XML Files
--------------

Under the **optional** tag ``files``, a list can be set with the names of files to load XML from.

Raw XML
-------

Under the **optional** tag ``raw``, an XML configuration (with the same format as an XML file) can be set as a string to be loaded.

.. code-block:: yaml

    xml:

      files:
          - "./xml_configuration.xml"

      raw: |
          <?xml version="1.0" encoding="UTF-8" ?>
          <profiles xmlns="http://www.eprosima.com">
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

Topics Configuration
====================

.. _user_manual_configuration_builtin_topics:

Built-in Topics
---------------

The discovery phase can be accelerated by listing topics under the ``builtin-topics`` tag.
The |ddsrouter| will create the DataWriters and DataReaders for these topics in the |ddsrouter| initialization.
The :ref:`Topic QoS <user_manual_configuration_topic_qos>` for these topics can be manually configured with a :ref:`Manual Topic <user_manual_configuration_manual_topics>`, a :ref:`Participant Topic QoS <user_manual_configuration_participant_topic_qos>`, and a :ref:`Specs Topic QoS <user_manual_configuration_specs_topic_qos>`; if a :ref:`Topic QoS <user_manual_configuration_topic_qos>` is not configured, it will take its default value.

The ``builtin-topics`` must specify a ``name`` and ``type`` without wildcard characters.


.. code-block:: yaml

    builtin-topics:
      - name: HelloWorldTopic
        type: HelloWorld

.. _topic_filtering:

Topic Filtering
---------------

The |ddsrouter| automatically detects the topics that are being used in a DDS Network.
The |ddsrouter| then creates internal DDS :term:`Writers<DataWriter>` and :term:`Readers<DataReader>` for each participant in each topic, and forwards the data published on each topic.
The |ddsrouter| allows filtering DDS :term:`Topics<Topic>` to allow users to configure the DDS :term:`Topics<Topic>` that must be forwarded.
These data filtering rules can be configured under the ``allowlist`` and ``blocklist`` tags.
If the ``allowlist`` and ``blocklist`` are not configured, the |ddsrouter| will forward all the data published on the topics it discovers.
If both the ``allowlist`` and ``blocklist`` are configured and a topic appears in both of them, the ``blocklist`` has priority and the topic will be blocked.

Topics are determined by the tags ``name`` (required) and ``type``, both of which accept wildcard characters.

.. note::

    Placing quotation marks around values in a YAML file is generally optional, but values containing wildcard characters do require single or double quotation marks.

Consider the following example:

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

In this example, the data in the topic ``AllowedTopic1`` with type ``Allowed`` and the data in the topic ``AllowedTopic2`` with any type will be forwarded by the |ddsrouter|.
The data in the topic ``HelloWorldTopic`` with type ``HelloWorld`` will be blocked, since the ``blocklist`` is blocking all topics with any name and with type ``HelloWorld``.

.. _user_manual_configuration_topic_qos:

Topic QoS
---------

The following is the set of QoS that are configurable for a topic.
For more information on topics, please read the `Fast DDS Topic <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/topic/topic.html>`_ section.

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

    *   - Ownership
        - ``ownership``
        - *bool*
        - ``false``
        - ``EXCLUSIVE_OWNERSHIP_QOS`` / ``SHARED_OWNERSHIP_QOS``

    *   - Partitions
        - ``partitions``
        - *bool*
        - ``false``
        - Topic with / without partitions

    *   - Key
        - ``keyed``
        - *bool*
        - ``false``
        - Topic with / without `key <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/topic/typeSupport/typeSupport.html#data-types-with-a-key>`_

    *   - History Depth
        - ``history-depth``
        - *unsigned integer*
        - ``5000``
        - :ref:`user_manual_configuration_history_depth`

    *   - Max Transmission Rate
        - ``max-tx-rate``
        - *float*
        - ``0`` (unlimited)
        - :ref:`user_manual_configuration_max_tx_rate`

    *   - Max Reception Rate
        - ``max-rx-rate``
        - *float*
        - ``0`` (unlimited)
        - :ref:`user_manual_configuration_max_rx_rate`

    *   - Downsampling
        - ``downsampling``
        - *unsigned integer*
        - ``1``
        - :ref:`user_manual_configuration_downsampling`

.. warning::

    Manually configuring ``TRANSIENT_LOCAL`` durability may lead to incompatibility issues when the discovered reliability is ``BEST_EFFORT``.
    Please ensure to always configure the ``reliability`` when configuring the ``durability`` to avoid the issue.

.. _user_manual_configuration_history_depth:

History Depth
^^^^^^^^^^^^^

The ``history-depth`` tag configures the history depth of the Fast DDS internal entities.
By default, the depth of every RTPS History instance is :code:`5000`, which sets a constraint on the maximum number of samples a |ddsrouter| instance can deliver to late joiner Readers configured with ``TRANSIENT_LOCAL`` `DurabilityQosPolicyKind <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/standardQosPolicies.html#durabilityqospolicykind>`_.
Its value should be decreased when the sample size and/or number of created endpoints (increasing with the number of topics and |ddsrouter| participants) are big enough to cause memory exhaustion issues.
If enough memory is available, however, the ``history-depth`` could be increased to deliver a greater number of samples to late joiners.

.. _user_manual_configuration_max_tx_rate:

Max Transmission Rate
^^^^^^^^^^^^^^^^^^^^^

The ``max-tx-rate`` tag limits the frequency [Hz] at which samples are sent by discarding messages transmitted before :code:`1/max-tx-rate` seconds have passed since the last sent message.
It only accepts non-negative numbers.
By default it is set to ``0``; it sends samples at an unlimited transmission rate.

.. _user_manual_configuration_max_rx_rate:

Max Reception Rate
^^^^^^^^^^^^^^^^^^

The ``max-rx-rate`` tag limits the frequency [Hz] at which samples are processed by discarding messages received before :code:`1/max-rx-rate` seconds have passed since the last processed message.
It only accepts non-negative numbers.
By default it is set to ``0``; it processes samples at an unlimited reception rate.

.. _user_manual_configuration_downsampling:

Downsampling
^^^^^^^^^^^^^

The ``downsampling`` tag reduces the sampling rate of the received data by only keeping *1* out of every *n* samples received (per topic), where *n* is the value specified under the ``downsampling`` tag.
When the ``max-rx-rate`` tag is also set, downsampling only applies to messages that have passed the ``max-rx-rate`` filter.
It only accepts positive integers.
By default it is set to ``1``; it accepts every message.

.. _user_manual_configuration_manual_topics:

Manual Topics
-------------

A subset of :ref:`Topic QoS <user_manual_configuration_topic_qos>` can be manually configured for a specific topic under the tag ``topics``.
The tag ``topics`` has a required ``name`` tag that accepts wildcard characters.
It also has three optional tags: a ``type`` tag that accepts wildcard characters, a ``qos`` tag with the :ref:`Topic QoS <user_manual_configuration_topic_qos>` that the user wants to manually configure, and a ``participants`` tag that lists the participants to which the configuration applies.
If a ``qos`` is not manually configured, it will get its value by discovery; if the ``participants`` tag is empty or non-existent, the configuration will apply to all participants.

**Example of usage**

.. code-block:: yaml

    topics:
      - name: "temperature/*"
        type: "temperature/types/*"
        qos:
          max-tx-rate: 15
          downsampling: 2
        participants:
          - Participant0
          - Participant1

.. note::

    The :ref:`Topic QoS <user_manual_configuration_topic_qos>` configured in the Manual Topics take precedence over the :ref:`Participant Topic QoS <user_manual_configuration_participant_topic_qos>` and the :ref:`Specs Topic QoS <user_manual_configuration_specs_topic_qos>`.

Specs Configuration
===================

The YAML Configuration supports a ``specs`` **optional** tag that contains certain options related with the overall configuration of the DDS Router instance to run.
The values available to configure are:

.. _thread_configuration:

Number of Threads
-----------------

``specs`` supports a ``threads`` **optional** value that allows the user to set a maximum number of threads
for the internal :code:`ThreadPool`.
This ThreadPool allows to limit the number of threads spawned by the application.
This improves the performance of the data transmission between Participants.

This value should be set by each user depending on each system's characteristics.
In case this value is not set, the default number of threads used is :code:`12`.

.. _user_manual_configuration_remove_unused_entities:

Remove Unused Entities
----------------------

``specs`` supports a ``remove-unused-entities`` **optional** value that configures the deletion of unused internal entities in the |ddsrouter|.
By default, unused internal entities are *not* removed.
Thus, when the |ddsrouter| discovers a Subscriber (by default; see :ref:`Discovery Trigger <user_manual_configuration_discovery_trigger>`), the |ddsrouter| creates entities in all of its participants, and these entities stay up even after the Subscriber disconnects.

At times it can be useful to remove the internal entities that are not being used.
Consider the following example.
Two *DDS Routers* are communicating through a WAN connection, when the last of the external Subscribers to which they are forwarding data disconnects.
By default, the internal entities of the *DDS Routers* would *not* be removed, so the *DDS Routers* would keep consuming bandwidth, even though the data is never read.
By setting the ``remove-unused-entities`` option to ``true``, the internal entities of the |ddsrouter| would be removed, and the *DDS Routers* would stop communicating and free up the bandwidth.

.. warning::
  At the time being, the removal of unused entities is incompatible with the `Transient-Local Durability QoS <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/standardQosPolicies.html#durabilityqospolicy>`_.

.. note::
  The ``remove-unused-entities`` option doesn't apply to :ref:`Built-in Topics <user_manual_configuration_builtin_topics>` since they are created before being discovered by a :term:`Participant`.

.. _user_manual_configuration_discovery_trigger:

Discovery Trigger
-----------------

``specs`` supports a ``discovery-trigger`` **optional** value that configures what type of external entity triggers the creation/removal of entities in the |ddsrouter|.
The possible values for the ``discovery-trigger`` are:

.. list-table::
    :header-rows: 1

    *   - Value
        - Tag
        - Description

    *   - Reader
        - ``reader``
        - The creation/removal of readers triggers the creation/removal of internal entities.

    *   - Writer
        - ``writer``
        - The creation/removal of writers triggers the creation/removal of internal entities.

    *   - Any
        - ``any``
        - The creation/removal of readers or writers triggers the creation/removal of internal entities.

    *   - None
        - ``none``
        - The creation/removal of external readers or writers doesn't trigger the creation/removal of internal entities.

.. warning::
  When the |ddsrouter| creates internal entities triggered by the discovery of a writer (i.e. the ``    `` is either ``writer`` or ``any``), the |ddsrouter| will create its internal entities with the writer's :ref:`Topic QoS <user_manual_configuration_topic_qos>`, and, therefore, the QoS of the communication between the external entities and the |ddsrouter| may differ from the QoS of the communication without the |ddsrouter|.

.. _user_manual_configuration_specs_topic_qos:

QoS
---

``specs`` supports a ``qos`` **optional** tag to configure the default values of the :ref:`Topic QoS <user_manual_configuration_topic_qos>`.

.. note::

    The :ref:`Topic QoS <user_manual_configuration_topic_qos>` configured in ``specs`` can be overwritten by the :ref:`Participant Topic QoS <user_manual_configuration_participant_topic_qos>` and the :ref:`Manual Topics <user_manual_configuration_manual_topics>`.

.. _router_specs_logging:

Logging
-------

``specs`` supports a ``logging`` **optional** tag to configure the |ddsrouter| logs.
Under the ``logging`` tag, users can configure the type of logs to display and filter the logs based on their content and category.
When configuring the verbosity to ``info``, all types of logs, including informational messages, warnings, and errors, will be displayed.
Conversely, setting it to ``warning`` will only show warnings and errors, while choosing ``error`` will exclusively display errors.
By default, the filter allows all errors to be displayed, while selectively permitting warning and informational messages from ``DDSROUTER`` category.

.. code-block:: yaml

    logging:
      verbosity: info
      filter:
        error: "DDSPIPE|DDSROUTER"
        warning: "DDSPIPE|DDSROUTER"
        info: "DDSROUTER"

.. note::

    Configuring the logs via the Command-Line is still active and takes precedence over YAML configuration when both methods are used simultaneously.

.. list-table::
    :header-rows: 1

    *   - Logging
        - Yaml tag
        - Description
        - Data type
        - Default value
        - Possible values

    *   - Verbosity
        - ``verbosity``
        - Show messages of equal |br|
          or higher importance.
        - *enum*
        - ``error``
        - ``info`` / ``warning`` / ``error``

    *   - Filter
        - ``filter``
        - Regex to filter the category  |br|
          or message of the logs.
        - *string*
        - info : ``DDSROUTER`` |br|
          warning : ``DDSROUTER`` |br|
          error : ``""``
        - Regex string

.. note::

    For the logs to function properly, the ``-DLOG_INFO=ON`` compilation flag is required.

The |ddsrouter| prints the logs by default (warnings and errors in the standard error and info traces in the standard output).
The |ddsrouter|, however, can also publish the logs in a DDS topic.
To publish the logs, under the tag ``publish``, set ``enable: true`` and set a ``domain`` and a ``topic-name``.
The type of the logs published is defined as follows:

**LogEntry.idl**

.. code-block:: idl

    const long UNDEFINED = 0x10000000;
    const long SAMPLE_LOST = 0x10000001;
    const long TOPIC_MISMATCH_TYPE = 0x10000002;
    const long TOPIC_MISMATCH_QOS = 0x10000003;

    enum Kind {
      Info,
      Warning,
      Error
    };

    struct LogEntry {
      @key long event;
      Kind kind;
      string category;
      string message;
      string timestamp;
    };

**Example of usage**

.. code-block:: yaml

    logging:
      verbosity: info
      filter:
        error: "DDSPIPE|DDSROUTER"
        warning: "DDSPIPE|DDSROUTER"
        info: "DDSROUTER"
      publish:
        enable: true
        domain: 84
        topic-name: "DdsRouterLogs"
      stdout: true

.. _user_manual_configuration_specs_monitor:

Monitor
-------

``specs`` supports a ``monitor`` **optional** tag to publish internal data from the |ddsrouter|.
If the monitor is enabled, it publishes (and logs under the ``MONITOR_DATA`` :ref:`log filter <router_specs_logging>`) the *DDS Router's* internal data on a ``domain``, under a ``topic-name``, once every ``period`` (in milliseconds).
If the monitor is not enabled, the |ddsrouter| will not collect or publish any data.

.. note::

    The data published is relative to each period.
    The |ddsrouter| will reset its tracked data after publishing it.


In particular, the |ddsrouter| will track the number of messages lost, received, and the message reception rate [Hz] of each topic.
The type of the data published is defined as follows:

**MonitoringTopics.idl**

.. code-block:: idl

    struct DdsTopicData
    {
        string participant_id;
        unsigned long msgs_lost;
        unsigned long msgs_received;
        double msg_rx_rate;
    };

    struct DdsTopic
    {
        string name;
        string type_name;
        boolean type_discovered;
        boolean type_mismatch;
        boolean qos_mismatch;
        sequence<DdsTopicData> data;
    };

    struct MonitoringTopics
    {
        sequence<DdsTopic> topics;
    };

**Example of usage**

.. code-block:: yaml

    monitor:
      topics:
        enable: true
        period: 1000
        domain: 10
        topic-name: "DdsRouterTopicData"

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

In |fastdds| versions previous to *v3.0.0*, a :term:`Discovery Server` requires a DDS :term:`GuidPrefix` in order for other Participants to connect to it.
Although this parameter is no longer mandatory, it is still possible to set it so that a Discovery Server client from an older release may still establish connection with a Discovery Server server from the newer ones.
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


.. _user_manual_configuration_connection_addresses:

Connection Addresses
--------------------

Tag ``connection-addresses`` configures a connection with one or multiple remote WAN Participants (:ref:`WAN participant <user_manual_participants_wan>` case) or Discovery Servers (:ref:`Discovery Server participant <user_manual_participants_discovery_server_wan>` case).
``connection-addresses`` is the *key* for an array of :ref:`Network Addresses <user_manual_configuration_network_address>`.

.. code-block:: yaml

    connection-addresses:
      - ip: 127.0.0.1
        port: 11666
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


.. _user_manual_configuration_participant_topic_qos:

QoS
---

Participants support a ``qos`` **optional** tag to manually configure their :ref:`Topic QoS <user_manual_configuration_topic_qos>`.

.. note::

    The :ref:`Topic QoS <user_manual_configuration_topic_qos>` configured for a Participant can be overwritten by the :ref:`Manual Topics <user_manual_configuration_manual_topics>` but take precedence over the :ref:`Specs Topic QoS <user_manual_configuration_specs_topic_qos>`.

.. _user_manual_configuration_forwarding_routes:

Forwarding Routes
=================

The |ddsrouter| is capable of establishing different internal routes between its participants.
This feature enables users to only forward sensitive data to a set of participants.

.. note::

    By default, when the tag ``routes`` is not set, every participant forwards the data it receives to every other participant.

Generic Routes
--------------

To configure a custom set of forwarding routes, use the tag ``routes`` followed by the destination participants for each source participant.


.. note::

    If a participant is not listed as a source, it will forward the data it receives to every other participant.

.. note::

    If a participant is listed as a source but it is not given any destination participants, it will not forward the data it receives to any participant.


Consider the following example with three participants: ``Participant0``, ``Participant1``, and ``Participant2``.

.. code-block:: yaml

  routes:
    - src: Participant0
      dst:
        - Participant2

    - src: Participant1

* Participant ``Participant0`` will only forward the data it receives to participant ``Participant2``.
* Participant ``Participant1`` will not forward the data it receives to any participant, since it does not have any destination participants.
* Participant ``Participant2`` will forward the data it receives to every participant (``Participant0`` and ``Participant1``), since it does not have a forwarding route.


.. warning::

    A repeater participant with a route defined must add itself to its route's destinations.


Topic Routes
------------

Besides the generic routes just described, custom routes can also be configured for a specific topic (determined by a ``name`` and ``type`` pair).
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

.. _user_manual_configuration_general_example:

General Example
===============

A complete example of all the configurations described on this page can be found below.

.. code-block:: yaml

    # Version Latest
    version: v5.0

    # Specifications
    specs:
      threads: 10
      remove-unused-entities: false
      discovery-trigger: reader

      qos:
        history-depth: 1000
        max-tx-rate: 0
        max-rx-rate: 20
        downsampling: 3

      logging:
        verbosity: info
        filter:
          error: "DDSPIPE|DDSROUTER"
          warning: "DDSPIPE|DDSROUTER"
          info: "DDSROUTER"
        publish:
            enable: true
            domain: 84
            topic-name: "DdsRouterLogs"
        stdout: true

      monitor:
        topics:
          enable: true
          period: 1000
          domain: 10
          topic-name: "DdsRouterTopicStatistics"

    # XML configurations to load
    xml:

      # Load this file as Fast DDS XML configuration
      files:
          - "./xml_configuration.xml"

      # Load text as Fast DDS XML configuration
      raw: |
          <?xml version="1.0" encoding="UTF-8" ?>
          <profiles xmlns="http://www.eprosima.com">
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

    # Manually configure Topic QoS for a set of participants on a topic

    topics:

      - name: "temperature/*"
        type: "temperature/types/*"
        qos:
          max-tx-rate: 15
          downsampling: 2
        participants:
          - Participant0
          - Participant1

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

        qos:

          max-rx-rate: 0                  # Max Reception Rate = 0 (unlimited)

          downsampling: 1                 # Downsampling = 1

    ####################

    # Simple DDS Participant in domain 7

      - name: Participant1              # Participant Name = Participant1

        kind: local                     # Participant Kind = local (= simple)

        domain: 7                       # DomainId = 7

        qos:

          max-rx-rate: 15                 # Max Reception Rate = 15

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
          - ip: 8.8.8.8                 # IP = 8.8.8.8
            port: 11666                 # Port = 11666
            transport: udp              # Transport = UDP


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
          - src: Participant1
            dst:
              - Participant0
