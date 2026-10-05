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
  A specific profile can also be selected per topic via the new ``endpoint-profile-name`` tag.
  A new optional ``endpoint-qos-mode`` participant tag controls whether YAML QoS overrides the matching
  XML profile (``xml-overridable``, default) or the profile is applied verbatim (``xml-standalone``).
  For more details, see :ref:`user_manual_participants_xml_topic_profiles`.

This release includes the following **improvements**:

* Reject the configuration tags that are accepted but never applied by the *DDS Router*: ``qos`` and ``filter``
  within ``allowlist`` and ``blocklist``, ``filter`` within ``topics``, ``status`` within ``specs: monitor``, and
  ``transport``, ``ignore-participant-flags`` and ``ros2-easy-mode`` in Discovery Server and WAN Participants.
* Report as configuration errors, when validating the configuration file, some configurations that were accepted
  before but rejected while parsing (e.g. ``specs: threads`` equal to ``0``).
* Accept TLS configurations without ``ca`` or ``password`` tags, matching the TLS validation performed at runtime.

This release includes the following **bugfixes**:

* Report the correct default value and accepted values of ``--log-verbosity`` in the application help text.

This release includes the following **documentation updates**:

* Document topic-name endpoint profile lookup for the :ref:`XML Participant <user_manual_participants_xml>`,
  including QoS fields always enforced by the *DDS Router* regardless of the profile.
* Add *Endpoint Profiles* section to the :ref:`user_manual_configuration` page.
* Warn that the configuration file is validated against a schema and that unknown tags are rejected,
  and remove the page of the no longer existing YAML Validator tool.
* Add tag trees to the :ref:`user_manual_configuration` page, and document the accepted values, default values and
  required tags of every configuration tag, including the ``logging`` and ``monitor`` tags.
* Document the tags accepted by each :ref:`Participant Kind <user_manual_participant_participant_kinds>`.
* The Installation Manual has been merged with the Developer Manual, and the latter is removed.
* Update the Docker image installation instructions to use eProsima's *Fast DDS Suite*.
* Update the examples and use cases to *Fast DDS* 3.
