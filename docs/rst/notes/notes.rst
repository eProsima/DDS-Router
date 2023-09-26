.. include:: ../exports/alias.include

.. _release_notes:

.. include:: forthcoming_version.rst

##############
Version v2.0.0
##############

This release adds new **Requirements**:

* `DDS Pipe <https://github.com/eProsima/DDS-Pipe>`_ project.

This release has the following **Features**:

* Main functionality has been moved to new repository `DDS Pipe <https://github.com/eProsima/DDS-Pipe>`_ to reuse it in other projects.
* :ref:`XML Participant <user_manual_participants_xml>`
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

This release includes the following **Dependencies Update**:

.. list-table::
    :header-rows: 1

    *   -
        - Repository
        - Old Version
        - New Version
    *   - Foonathan Memory Vendor
        - `eProsima/foonathan_memory_vendor <https://github.com/eProsima/foonathan_memory_vendor>`_
        - `v1.3.0 <https://github.com/eProsima/foonathan_memory_vendor/releases/tag/v1.3.0>`_
        - `v1.3.1 <https://github.com/eProsima/foonathan_memory_vendor/releases/tag/v1.3.1>`_
    *   - Fast CDR
        - `eProsima/Fast-CDR <https://github.com/eProsima/Fast-CDR>`_
        - `v1.0.27 <https://github.com/eProsima/Fast-CDR/releases/tag/v1.0.27>`_
        - `v1.1.0 <https://github.com/eProsima/Fast-CDR/releases/tag/v1.1.0>`_
    *   - Fast DDS
        - `eProsima/Fast-DDS <https://github.com/eProsima/Fast-DDS>`_
        - `v2.10.1 <https://github.com/eProsima/Fast-DDS/releases/tag/v2.10.1>`_
        - `v2.11.0 <https://github.com/eProsima/Fast-DDS/releases/tag/v2.11.0>`_
    *   - Dev Utils
        - `eProsima/dev-utils <https://github.com/eProsima/dev-utils>`_
        - `v0.3.0 <https://github.com/eProsima/dev-utils/releases/tag/v0.3.0>`_
        - `v0.4.0 <https://github.com/eProsima/dev-utils/releases/tag/v0.4.0>`_
    *   - DDS Pipe
        - `eProsima/DDS-Pipe <https://github.com/eProsima/DDS-Pipe.git>`_
        - `v0.1.0 <https://github.com/eProsima/DDS-Pipe/releases/tag/v0.1.0>`_
        - `v0.2.0 <https://github.com/eProsima/DDS-Pipe/releases/tag/v0.2.0>`_

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
