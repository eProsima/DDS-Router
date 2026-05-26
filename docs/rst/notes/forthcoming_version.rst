.. add orphan tag when new info added to this file

:orphan:

###################
Forthcoming Version
###################

This release includes the following **new features**:

* :ref:`XML Participant <user_manual_participants_xml>` now supports topic-name endpoint profile lookup:
  when creating a :term:`DataWriter` or :term:`DataReader` for a topic, a loaded XML profile whose name
  matches the topic name is automatically applied, giving users full control over QoS fields such as
  history, memory policy and transport.
  A new optional ``xml-override`` tag allows the user to force YAML QoS to take precedence over a matching XML profile.
  For more details, see :ref:`user_manual_participants_xml_topic_profiles`.

This release includes the following **documentation updates**:

* Document topic-name endpoint profile lookup for the :ref:`XML Participant <user_manual_participants_xml>`,
  including QoS fields always enforced by the *DDS Router* regardless of the profile.
* Add *Endpoint Profiles* section to the :ref:`user_manual_configuration` page.