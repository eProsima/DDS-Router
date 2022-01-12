.. include:: ../exports/alias.include

.. _user_manual_configuration:

########################
DDS Router Configuration
########################

A |ddsrouter| is configured by a *.yaml* configuration file.
This *.yaml* file contains all the information regarding the |ddsrouter| configuration, such as topics filtering
and :term:`Participants <Participant>` configurations.


Topic Filtering
===============

.. note::

    The |ddsrouter| topic discovery module is a work in progress.
    Thus, the functionality regarding Topic filtering is still in its early stages.

The |ddsrouter| requires a list of allowed :term:`Topics<Topic>`.
The data transmitted under these topics will be the data relayed by the |ddsrouter|.
The YAML configuration file must contain an ``allowlist`` tag which is a vector (``[]``).
This vector elements contains the Topics that will be forwarded (subsequently the |ddsrouter| creates a
:term:`Writers<DataWriter>` and :term:`Readers<DataReader>` for each topic for each Participant).
Each Topic is determined by its entries ``name``, ``type`` and ``keyed``, with only the first one being mandatory.

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

    *   - ``keyed``
        - ``bool``
        - ``false``

The entry ``keyed`` determines whether the corresponding topic is `keyed <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/topic/typeSupport/typeSupport.html#data-types-with-a-key>`_
or not. See Topic section for further information about the topic.

.. todo:

    Add link to topic page when created

In the following configuration example, the |ddsrouter| will transmit the topic ``rt/chatter`` (default ROS 2 topic for
``talker`` and ``listener``) with type name ``std_msgs::msg::dds_::String_``.
It also will transmit the topic ``HelloWorldTopic`` (default FastDDS topic for ``HelloWorldExample``)
with type name ``HelloWorld``.

.. code-block:: yaml

    allowlist:
      - name: "rt/chatter"
        type: "std_msgs::msg::dds_::String_"
      - name: "HelloWorldTopic"
        type: "HelloWorld"

.. note::

    Tag ``allowlist`` must be at yaml base level (it must not be inside any other tag).


Participant Configuration
=========================

At the yaml base level, along with ``allowlist`` tag, there will be the Participants that will be created,
together with their specific configuration.
Each Participant is identified by a unique :term:`Participant Id` that will be the yaml *key*.
The yaml *value* for this *key* is the configuration for this specific Participant.
There could be any number of Participants, and Participant types could be repeated.

Each Participant has its specific configuration.
Please, refer to :ref:`user_manual_participant_participant_types` in order to see each of the
:term:`Participant Types<Participant Type>` requirements.

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

    Participant0:     # Participant Id = Participant0
      type: local     # Participant Type = simple
      domain: 3       # DomainId = 3

    ################

    simple:           # Participant Id = simple ; Participant Type = simple
      domain: 6       # DomainId = 6

The first Participant `Participant0` has Participant Id *Participant0* and is configured to be of the *simple*
Participant Type.
The second Participant has Participant Id *simple* and it is configured to be of the *simple* type.
That is because the type is not required to be specified as it is get from the Participant Id.

.. note::

    Participant Type is not case sensitive.
    A Participant called *Simple* would be of type ``simple``.

.. note::

    The Participant Id is get as Participant Type when type is not specified.
    If type is explicitly specified, the Participant Id is not used to get the type.


.. _user_manual_configuration_domain_id:

Domain Id
=========

Tag ``domain`` configures the :term:`Domain Id` of a specific Participant.
Be aware that some Participants (e.g. Discovery Servers) does not need a Domain Id configuration.

.. code-block:: yaml

    domain: 101


.. _user_manual_configuration_network_address:

Network Address
===============

Network Addresses are elements that can be configured for specific Participants.
An Address is defined by:

* *IP*: IP of the host (public IP in case of WAN communication).
* *Port*: Port where the Participant is listening.
* *Transport Protocol*: ``UDP`` or ``TCP``.
  If it is not set, it would be chosen by default depending on the Participant Type.
* *IP version*: ``v4`` or ``v6``.
  If it is not set, it would be chosen depending on the *IP* string format.

.. code-block:: yaml

    ip: "127.0.0.1"
    port: 11666
    transport: "tcp"
    ip-version: "v4"

    ################

    ip: "2001:4860:4860::8844"      # Recognized as IPv6
    port: 1616

.. warning::

    ``ip`` field does not currently support DNS names, only well-formed IP addresses.

.. _user_manual_configuration_discovery_server_guidprefix:

Discovery Server GuidPrefix
===========================

A :term:`Discovery Server` requires a DDS :term:`GuidPrefix` in order to other Participants connect to it.
There are several possibilities for configuring a GuidPrefix.


Discovery Server GuidPrefix by string
-------------------------------------

The GuidPrefix of the Discovery Server can be configured using ``guid`` tag.
Be aware of using the correct format for GuidPrefix.
That is, 12 hexadecimal numbers (lower than ``ff``) separated with ``.``.

.. code-block:: yaml

    guid: "1.f.1.0.0.0.0.0.0.0.ca.fe"       # GuidPrefix = 01.0f.01.00.00.00.00.00.00.00.ca.fe


Discovery Server GuidPrefix by Id
---------------------------------

Using tag ``id``, the GuidPrefix will be calculated arbitrarily using a default |ddsrouter| GuidPrefix.
This default GuidPrefix is ``01.0f.<id>.00.00.00.00.00.00.00.ca.fe``.
Default value for ``id`` is ``0``.

.. code-block:: yaml

    id: 13                                  # GuidPrefix = 01.0f.0d.00.00.00.00.00.00.00.ca.fe

.. note::

    In the current version of the |ddsrouter| only ids in the range 0 to 256 are allowed.
    In future releases it would be implemented to allow a wider range of ids.


ROS Discovery Server GuidPrefix
-------------------------------

There is a specific GuidPrefix for ROS 2 executions, so it could be used using Fast DDS CLI and
ROS 2 ``ROS_DISCOVERY_SERVER`` environment variable
(`<https://fast-dds.docs.eprosima.com/en/v2.4.1/fastdds/ros2/discovery_server/ros2_discovery_server.html>`__).

The ROS 2 Discovery Server GuidPrefix is set by default to ``44.53.<id>.5f.45.50.52.4f.53.49.4d.41`` where ``<id>``
is the specific id of the Server.
This GuidPrefix also allow an ``id``` value to specify which id is used in the GuidPrefix.
Default value for ``id`` is ``0``.

.. code-block:: yaml

    ros-discovery-server: true              # GuidPrefix = 44.53.x.5f.45.50.52.4f.53.49.4d.41
    id: 13                                  # GuidPrefix = 44.53.0d.5f.45.50.52.4f.53.49.4d.41


.. _user_manual_configuration_discovery_server_listening_addresses:

Discovery Server Listening Addresses
====================================

Tag ``listening-addresses`` configures the network addresses where the Discovery Server configured is going to
listen for remote clients or servers.
``listening-addresses`` is *key* for an array of :ref:`Network Addresses <user_manual_configuration_network_address>`.

.. code-block:: yaml

    listening-addresses:
      - ip: "127.0.0.1"                # UDP by default
        port: 11667
      - ip: "2001:4860:4860::8844"     # Recognized as IPv6
        port: 11668
        transport: "tcp"


.. _user_manual_configuration_discovery_server_connection_addresses:

Discovery Server Connection Addresses
=====================================

Tag ``connection-addresses`` configure a connection with one or multiple remote Discovery Servers.
``connection-addresses`` is the *key* for an array in which each element has a GuidPrefix referencing the Discovery
Server to connect with; and a tag ``addresses`` configuring the addresses of such Discovery Server.
Each element inside ``addresses`` must follow the configuration for :ref:`user_manual_configuration_network_address`.

.. code-block:: yaml

    connection-addresses:
      - guid: "44.53.0d.5f.45.50.52.4f.53.49.4d.41"
        addresses:
          - ip: "127.0.0.1"
            port: 11666
      - id: 4
        addresses:
          - ip: "2001:4860:4860::8888"
            port: 11667
            transport: "tcp"
          - ip: "2001:4860:4860::8844"
            port: 11668
            transport: "tcp"


.. _user_manual_configuration_general_example:

General Example
===============

A complete example of all the configurations described on this page can be found below.

.. code-block:: yaml

    # Relay topic rt/chatter and type std_msgs::msg::dds_::String_
    # Relay topic HelloWorldTopic and type HelloWorld

    allowlist:
      - name: "rt/chatter"
        type: "std_msgs::msg::dds_::String_"
      - name: "HelloWorldTopic"
        type: "HelloWorld"

    ####################

    # Simple DDS Participant in domain 3

    Participant0:                     # Participant Id = Participant0

      type: local                     # Participant Type = local (= simple)

      domain: 3                       # DomainId = 3

    ####################

    # Discovery Server DDS Participant with ROS GuidPrefix so a local ROS 2 Client could connect to it
    # This Discovery Server will listen in ports 11600 and 11601 in localhost

    ServerROS2:                       # Participant Id = ServerROS2

      type: local-discovery-server    # Participant Type = local-discovery-server

      id: 1
      ros-discovery-server: true      # ROS Discovery Server id => GuidPrefix = 44.53.01.5f.45.50.52.4f.53.49.4d.41

      listening-addresses:            # Local Discovery Server Listening Addresses
        - ip: "127.0.0.1"             # IP = localhost ; Transport = UDP (by default)
          port: 11600                 # Port = 11600
        - ip: "127.0.0.1"             # IP = localhost
          port: 11601                 # Port = 11601
          transport: "udp"            # Transport = UDP

    ####################

    # Participant that will communicate with a DDS Router in a different LAN.
    # This Participant will work as the remote DDS Router Client, so it set the connection address of the remote one.

    Wan:                              # Participant Id = Wan ; if type is not specified, this value is used to determine the Participant Type

      id: 2                           # Internal WAN Discovery Server id => GuidPrefix = 01.0f.02.00.00.00.00.00.00.00.ca.fe

      connection-addresses:           # WAN Discovery Server Connection Addresses
        - id: 4                       # External WAN Discovery Server id => GuidPrefix = 01.0f.04.00.00.00.00.00.00.00.ca.fe
          addresses:
            - ip: "8.8.8.8"           # IP = 8.8.8.8
              port: 11666             # Port = 11666
              transport: "tcp"        # Transport = TCP
