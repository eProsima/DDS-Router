.. include:: ../../exports/alias.include

.. _user_manual_participants_xml:

###############
XML Participant
###############

This type of :term:`Participant` refers to a :term:`DomainParticipant` that uses QoS profiles loaded from XML files to be configured.

|fastdds| supports XML to fully configure a DomainParticipant.
Check the following `documentation <https://fast-dds.docs.eprosima.com/en/latest/fastdds/xml_configuration/domainparticipant.html>`_ for further information on how to configure a DDS DomainParticipant with XML.
For further information regarding how to load XML configuration files to the |ddsrouter|, check the :ref:`user_manual_configuration_load_xml` section.

.. note::

    This kind of Participant is meant for advanced users as XML profiles will overwrite the default internal settings of the DDS Router.


Use case
========

Use this Participant to fully configure a DomainParticipant, its discovery methods, transport options, DDS QoS, etc.
The main use case for this Participant is using **DDS Security** (see `Security <https://fast-dds.docs.eprosima.com/en/stable/fastdds/security/security.html>`_),
which requires XML configuration from the user's side.

.. warning::

    This Participant kind does not support :term:`RPC`.
    Thus services and actions of ROS 2 will not work correctly.


Kind aliases
============

* ``xml``
* ``XML``


Configuration
=============

The XML Participant allows setting a profile name for the internal DomainParticipant of the |ddsrouter|.
Such profile name will be used as the QoS profile when creating the internal DomainParticipant.
The :term:`Domain Id` is also taken from this profile, so the ``domain`` tag is not accepted, and neither is the
``qos`` tag.


.. _user_manual_participants_xml_profiles:

Create a Fast DDS XML Participant profile
-----------------------------------------

The whole DomainParticipant configuration settings must be configured via XML, |ddsrouter| will not configure any attribute or QoS for it.
To configure the profile, check the :ref:`Profile <user_manual_configuration_profile>` configuration section.

However, there are specific QoS that will affect the performance of the |ddsrouter| and that the user is advised to set.
Notice that not setting such QoS will not affect the correct functionality of the application, but may affect its performance.

* ``ignore_local_endpoints`` avoid local matching for this participant's endpoints:

  .. code-block:: xml

      <participant profile_name="ignore_local_endpoints_domainparticipant_xml_profile">
          <rtps>
              <propertiesPolicy>
                  <properties>
                      <property>
                          <name>fastdds.ignore_local_endpoints</name>
                          <value>true</value>
                      </property>
                  </properties>
              </propertiesPolicy>
          </rtps>
      </participant>


.. _user_manual_participants_xml_topic_profiles:

Endpoint profiles
-----------------

When the XML Participant creates a :term:`DataWriter` or :term:`DataReader` for a topic, it looks for a loaded XML ``data_writer`` or ``data_reader`` profile to configure that endpoint.
By default, the |ddsrouter| looks for a profile **whose name matches the topic name**.
If a matching profile is found, the endpoint is configured using that profile's QoS, giving the user control over fields such as history, memory policy, transport, etc.
By default, ``durability``, ``reliability``, ``ownership`` and ``history`` are still taken from the YAML configuration and from discovery; see :ref:`user_manual_participants_xml_endpoint_qos_mode` to use the profile as is.
If no matching profile exists, the endpoint falls back to default QoS with values derived from the YAML configuration and from discovery.

.. note::

    Endpoint profiles are only applied by XML Participants.
    The endpoints of any other Participant kind ignore them, even if a profile whose name matches the topic name is loaded.

.. note::

    Certain QoS are always enforced by the |ddsrouter| regardless of the XML profile:
    ``deadline`` on DataWriters (set to the minimum value so it matches any DataReader),
    ``autodispose_unregistered_instances`` on DataWriters (set to ``false`` to preserve dispose/unregister forwarding semantics),
    and ``expects_inline_qos`` on DataReaders for keyed topics.

The following example loads a ``data_writer`` and a ``data_reader`` profile named ``my_topic`` that will be automatically applied when creating endpoints for a topic of that name:

.. code-block:: xml

    <?xml version="1.0" encoding="UTF-8" ?>
    <profiles xmlns="http://www.eprosima.com">
        <data_writer profile_name="my_topic">
            <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
        </data_writer>
        <data_reader profile_name="my_topic">
            <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
        </data_reader>
    </profiles>

Selecting a profile explicitly
""""""""""""""""""""""""""""""

Instead of relying on the topic name, a specific profile can be selected for a topic with the ``endpoint-profile-name`` tag of the :ref:`Topic QoS <user_manual_configuration_topic_qos>`.
When set, the |ddsrouter| looks up the ``data_writer`` and ``data_reader`` profiles with that name instead of the topic name:

.. code-block:: yaml

    topics:
      - name: "rt/chatter"
        qos:
          endpoint-profile-name: "chatter_profile"
        participants:
          - xml_participant

Like any other Topic QoS, ``endpoint-profile-name`` can be set in the :ref:`Manual Topics <user_manual_configuration_manual_topics>`, the :ref:`Participant Topic QoS <user_manual_configuration_participant_topic_qos>` and the :ref:`Specs Topic QoS <user_manual_configuration_specs_topic_qos>`, with the same precedence among them.
Since XML profiles are loaded for the whole |ddsrouter| process, this is the way to apply different profiles to the same topic in different XML Participants.
If no profile with that name is loaded, the |ddsrouter| does not look for a profile named after the topic; the endpoint falls back to default QoS instead.

.. _user_manual_participants_xml_endpoint_qos_mode:

Endpoint QoS mode
"""""""""""""""""

The ``endpoint-qos-mode`` tag of the XML Participant sets how a matching XML endpoint profile is applied:

.. list-table::
    :header-rows: 1

    *   - Value
        - Behavior

    *   - ``xml-overridable`` (default)
        - The XML profile is used as base.
          ``durability``, ``reliability``, ``ownership`` and ``history`` are taken from the :ref:`Topic QoS <user_manual_configuration_topic_qos>`, with priority:
          YAML configuration (Manual Topics, Participant Topic QoS or Specs Topic QoS) > QoS discovered from the remote endpoints > |ddsrouter| defaults.
          Every other policy keeps the value of the XML profile.

    *   - ``xml-standalone``
        - The XML profile is used as is.
          Neither the YAML configuration nor the discovered QoS override it.

    *   - ``disabled``
        - No XML endpoint profile is looked up, as if none were loaded: the endpoints take their QoS from the :ref:`Topic QoS <user_manual_configuration_topic_qos>`.
          Useful to stop applying the XML endpoint profiles without removing them from the XML configuration.

When no profile is found, the endpoint takes ``durability``, ``reliability``, ``ownership`` and ``history`` from the Topic QoS with ``xml-overridable`` and ``xml-standalone``.
The ``endpoint-qos-mode`` tag is only accepted by XML Participants.
The endpoints of any other Participant kind never look up XML endpoint profiles, so they always take their QoS from the Topic QoS.

.. code-block:: yaml

    - name: xml_participant
      kind: xml
      profile: custom_participant_configuration
      endpoint-qos-mode: xml-standalone          # Use the XML endpoint profiles as is

.. warning::

    With ``xml-standalone``, the endpoints of the XML Participant do not adapt to the QoS of the remote endpoints.
    Make sure the XML profiles are compatible with them; e.g. if the remote DataWriters use ``EXCLUSIVE_OWNERSHIP_QOS``, set it in the ``data_writer`` and ``data_reader`` profiles, otherwise the endpoints of the XML Participant will not match them.


Repeater
--------

This Participant allows a tag ``repeater`` to be used as a Repeater server.
Please refer to section :ref:`use_case_repeater` for more information.

Configuration Example
=====================

Configure a XML Participant that gets all of its QoS from XML profile named ``custom_participant_configuration``.
This XML profile must be previously loaded.
Use |fastdds| or |ddsrouter| support to load XML configuration files as explained in :ref:`this section <user_manual_configuration_load_xml>`.

.. code-block:: yaml

    - name: xml_participant                       # Participant Name = xml_participant

      kind: xml

      profile: custom_participant_configuration   # Configure participant with this profile

      endpoint-qos-mode: xml-overridable          # XML endpoint profiles overridden by the Topic QoS (default)
