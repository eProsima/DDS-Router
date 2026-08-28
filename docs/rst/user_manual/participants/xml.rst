.. include:: ../../exports/alias.include

.. _user_manual_participants_xml:

###############
XML Participant
###############

This type of :term:`Participant` refers to a :term:`DomainParticipant` that uses QoS profiles loaded from XML files to be configured.

|fastdds| supports XML to fully configure a DomainParticipant.
Using XML configuration, users have whole access to the full configuration of a DDS DomainParticipant.
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


.. _user_manual_participants_xml_profiles:

Create a Fast DDS XML Participant profile
-----------------------------------------

The whole DomainParticipant configuration settings must be configured via XML, |ddsrouter| will not configure any attribute or QoS for it.
To configure the profile, check the :ref:`Profile <user_manual_configuration_profile>` configuration section.

However, there are specific QoS that will affect the performance of the |ddsrouter| and that are advisable for the user to set them.
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

Topic-name endpoint profile lookup
-----------------------------------
When the XML Participant creates a DataWriter or DataReader for a topic, it looks for a loaded XML ``data_writer``
or ``data_reader`` profile to configure that endpoint.
By default, the |ddsrouter| looks for a profile **whose name matches the topic name**.
If a matching profile is found, the endpoint is configured using that profile's QoS, giving the user
control over fields such as history, memory policy, transport, etc.
If no matching profile exists, the endpoint falls back to default QoS with values derived from the YAML configuration.

.. note::

    Certain QoS are always enforced by the |ddsrouter| regardless of the XML profile:
    deadline on DataWriters (set to minimum so it matches any reader),
    ``autodispose_unregistered_instances`` on DataWriters (set to ``false`` to preserve dispose/unregister forwarding semantics),
    and ``expects_inline_qos`` on DataReaders for keyed topics.

The following example loads a profile named ``my_topic`` that will be automatically applied when creating
endpoints for a topic of that name:

.. code-block:: xml

    <dds>
        <profiles>
            <data_writer profile_name="my_topic">
                <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
            </data_writer>
            <data_reader profile_name="my_topic">
                <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
            </data_reader>
        </profiles>
    </dds>

Selecting a profile explicitly
"""""""""""""""""""""""""""""""

Instead of relying on the topic name, a specific profile can be selected for a topic with the
``endpoint-profile-name`` tag under that topic's QoS configuration.
When set, the |ddsrouter| looks up the XML profile with that name instead of the topic name:

.. code-block:: yaml

    topics:
      - name: "rt/chatter"
        qos:
          endpoint-profile-name: "my_reader_profile"

Overriding profile QoS from the YAML configuration
"""""""""""""""""""""""""""""""""""""""""""""""""""

When a matching XML profile is applied, the QoS fields explicitly set by the user in the YAML
configuration take precedence over the values in the XML profile.
This behavior is controlled by the ``endpoint-qos-mode`` participant tag, which accepts two values:

* ``xml-overridable`` *(default)*: the XML profile is applied first, and any QoS field explicitly set in
  the YAML configuration overrides the corresponding value from the profile.
* ``xml-standalone``: the XML profile is applied verbatim; YAML QoS does not override it.

.. code-block:: yaml

    - name: xml_participant
      kind: xml
      endpoint-qos-mode: xml-standalone


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
