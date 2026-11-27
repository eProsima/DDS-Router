.. add orphan tag when new info added to this file

:orphan:

###################
Forthcoming Version
###################

Next release will include the following **features**:

* Add the ``endpoint-qos-mode`` tag to the XML Participant to choose how a matching XML endpoint profile is applied
  (see :ref:`Endpoint QoS mode <user_manual_participants_xml_endpoint_qos_mode>`).
  By default (``xml-overridable``), the profile is used as base and its durability, reliability, ownership and history
  are taken from the YAML configuration or from the discovered remote endpoints; ``xml-standalone`` applies the profile
  as is; ``disabled`` does not look up any XML endpoint profile.
