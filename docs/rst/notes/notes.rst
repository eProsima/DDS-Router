.. include:: ../exports/alias.include

.. _release_notes:

.. comment the include of forthcoming when new info is added

.. .. include:: forthcoming_version.rst

##############
Version v2.0.0
##############

This release adds new **Requirements**:

* `DDS Pipe <https://github.com/eProsima/DDS-Pipe>`_ project.

This release has the following **Features**:

* Main functionality has been moved to new repository `DDS Pipe <https://github.com/eProsima/DDS-Pipe>`_ to reuse it in other projects.
* `XML Participant <user_manual_participants_xml>`
* Support for `DDS Security <https://fast-dds.docs.eprosima.com/en/v2.10.1/fastdds/security/security.html>`_.

This release includes the following **Internal Implementation Features**:

* Support :ref:`Interface Whitelisting <user_manual_configuration_interface_whitelist>`.
* Support :ref:`Custom Transport Descriptors <user_manual_configuration_custom_transport_descriptors>` (UDP or Shared Memory only) for Simple Participants.
* Support :ref:`Ignore Participant Flags <user_manual_configuration_ignore_participant_flags>` for Simple Participants.
* Add tests for Dynamic Types to ROS 2.
* New ``CommonParticipant`` class for a Fast DDS Participant wrapper.
* New ``XmlParticipant`` class for a Fast DDS Participant in which the participant is created following a profile name loaded by XML configuration.
* New ``CommonReader`` class for a Fast DDS Data Reader wrapper.
* New ``SimpleReader`` class for a Fast DDS Data Reader that implements abstract ``CommonReader``.
* New ``SpecificQoSReader`` class for a Fast DDS Data Reader with specific QoS policies.
* New ``CommonWriter`` class for a Fast DDS Data Writer wrapper.
* New ``SimpleWriter`` class for a Fast DDS Data Writer that implements abstract ``CommonWriter``.
* New ``QoSSpecificWriter`` class for a Fast DDS Data Writer with specific QoS policies.
* New ``MultiWriter`` class for a Fast DDS Data Writer collection that contains multiple ``QoSSpecificWriter``.

This release includes the following **Bugfixes**:

* Add TSAN fixes.
* Include missing DLLs.
* Resolve Windows warnings.
* Restore default DomainParticipantQoS after creating and enabling ``DynTypesParticipant``.

#################
Previous Versions
#################

.. include:: previous_versions/v1.2.0.rst
.. include:: previous_versions/v1.1.0.rst
.. include:: previous_versions/v1.0.0.rst
.. include:: previous_versions/v0.4.0.rst
.. include:: previous_versions/v0.3.0.rst
.. include:: previous_versions/v0.2.0.rst
.. include:: previous_versions/v0.1.0.rst
