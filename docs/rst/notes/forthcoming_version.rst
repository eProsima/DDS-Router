.. add orphan tag when new info added to this file

:orphan:

###################
Forthcoming Version
###################

This release includes the following **new features**:

* :ref:`XML Participant <user_manual_participants_xml>` now supports endpoint profiles:
  when creating a :term:`DataWriter` or :term:`DataReader` for a topic, a loaded XML ``data_writer`` or
  ``data_reader`` profile whose name matches the topic name is automatically applied, giving users control over
  QoS fields such as history, memory policy and transport.
  A specific profile can also be selected via the new ``endpoint-profile-name`` Topic QoS tag.
  ``durability``, ``reliability``, ``ownership`` and ``history-depth`` explicitly set in the YAML configuration
  take precedence over the profile.
  For more details, see :ref:`user_manual_participants_xml_topic_profiles`.

This release includes the following **documentation updates**:

* Document endpoint profiles for the :ref:`XML Participant <user_manual_participants_xml>`,
  including QoS fields always enforced by the *DDS Router* regardless of the profile.
* Add *Endpoint profiles* section to the :ref:`user_manual_configuration` page.
