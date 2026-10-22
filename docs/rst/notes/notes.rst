.. include:: ../exports/alias.include

.. _release_notes:

.. include:: forthcoming_version.rst

##############
Version v3.5.4
##############

This release includes the following **features**:

* :ref:`XML Participant <user_manual_participants_xml>` now supports endpoint profiles:
  when creating a :term:`DataWriter` or :term:`DataReader` for a topic, a loaded XML ``data_writer`` or
  ``data_reader`` profile whose name matches the topic name is automatically applied, giving users control over
  QoS fields such as history, memory policy and transport.
  A specific profile can also be selected via the new ``endpoint-profile-name`` Topic QoS tag.
  ``durability``, ``reliability``, ``ownership`` and ``history-depth`` explicitly set in the YAML configuration
  take precedence over the profile.
  For more details, see :ref:`user_manual_participants_xml_topic_profiles`.

This release includes the following **bugfixes**:

* Report the correct default value and accepted values of ``--log-verbosity`` in the application help text.

This release includes the following **improvements**:

* Run the ``Uncrustify`` and ``Python Linters`` workflows only in PRs
* Remove the nightly CI jobs targeting the End-of-Life 2.x branch
* Align the validation of the configuration file with what the *DDS Router* applies: reject tags that were accepted
  but never applied, report configurations that were previously rejected only while parsing, and accept TLS
  configurations without ``ca`` or ``password`` tags

This release includes the following **documentation updates**:

* Rework the documentation: the configuration page contains now a reference tag tree and more detailed information of
  the tags, and it warns that unknown tags are rejected by schema validation; the Developer Manual is merged into the
  Installation Manual; and examples and use cases are updated to *Fast DDS* 3

This release includes the following **Dependencies Update**:

.. list-table::
    :header-rows: 1

    *   -
        - Repository
        - Old Version
        - New Version
    *   - Foonathan Memory Vendor
        - `eProsima/foonathan_memory_vendor <https://github.com/eProsima/foonathan_memory_vendor>`__
        - `v1.4.1 <https://github.com/eProsima/foonathan_memory_vendor/releases/tag/v1.4.1>`__
        - `v1.4.1 <https://github.com/eProsima/foonathan_memory_vendor/releases/tag/v1.4.1>`__
    *   - Fast CDR
        - `eProsima/Fast-CDR <https://github.com/eProsima/Fast-CDR>`__
        - `v2.3.6 <https://github.com/eProsima/Fast-CDR/releases/tag/v2.3.6>`__
        - `v2.3.6 <https://github.com/eProsima/Fast-CDR/releases/tag/v2.3.6>`__
    *   - Fast DDS
        - `eProsima/Fast-DDS <https://github.com/eProsima/Fast-DDS>`__
        - `v3.6.2 <https://github.com/eProsima/Fast-DDS/releases/tag/v3.6.2>`__
        - `v3.6.2 <https://github.com/eProsima/Fast-DDS/releases/tag/v3.6.2>`__
    *   - Dev Utils
        - `eProsima/dev-utils <https://github.com/eProsima/dev-utils>`__
        - `v1.5.3 <https://github.com/eProsima/dev-utils/releases/tag/v1.5.3>`__
        - `v1.5.4 <https://github.com/eProsima/dev-utils/releases/tag/v1.5.4>`__
    *   - DDS Pipe
        - `eProsima/DDS-Pipe <https://github.com/eProsima/DDS-Pipe>`__
        - `v1.5.3 <https://github.com/eProsima/DDS-Pipe/releases/tag/v1.5.3>`__
        - `v1.5.4 <https://github.com/eProsima/DDS-Pipe/releases/tag/v1.5.4>`__

#################
Previous Versions
#################

.. include:: previous_versions/v3.5.3.rst
.. include:: previous_versions/v3.5.2.rst
.. include:: previous_versions/v3.5.1.rst
.. include:: previous_versions/v3.5.0.rst
.. include:: previous_versions/v3.4.0.rst
.. include:: previous_versions/v3.3.0.rst
.. include:: previous_versions/v3.2.0.rst
.. include:: previous_versions/v3.1.0.rst
.. include:: previous_versions/v3.0.0.rst
.. include:: previous_versions/v2.2.0.rst
.. include:: previous_versions/v2.1.0.rst
.. include:: previous_versions/v2.0.0.rst
.. include:: previous_versions/v1.2.0.rst
.. include:: previous_versions/v1.1.0.rst
.. include:: previous_versions/v1.0.0.rst
.. include:: previous_versions/v0.4.0.rst
.. include:: previous_versions/v0.3.0.rst
.. include:: previous_versions/v0.2.0.rst
.. include:: previous_versions/v0.1.0.rst
