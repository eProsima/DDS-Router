.. include:: ../exports/alias.include

.. _release_notes:

.. include:: forthcoming_version.rst

##############
Version v3.3.0
##############

This release includes the following **features**:

* Support :ref:`repeater <use_case_repeater>` feature in :ref:`XML <user_manual_participants_xml>` participants.
* Support writer-side `SQL-like <https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/topic/contentFilteredTopic/defaultFilter.html#the-default-sql-like-filter>`_ content filtering in :ref:`XML <user_manual_participants_xml>` participants.

This release includes the following **Documentation Updates**:

* Fix documentation version.
* Added TCP documentation clarifications

This release includes the following *ci management updates*:

* Regenerate security certificates & Upgrade testing Vulcanexus image to Jazzy
* Use 'debug' binaries for tsan tests
* Use different Vulcanexus images for each Fast DDS version
* Modify QoS of tests to reduce flakiness
* Remove deprecated windows-2019 runner
* Bump setuptools in /docs in the pip group across 1 directory
* Add test report to docker tests


This release includes the following **Dependencies Update**:

.. list-table::
    :header-rows: 1

    *   -
        - Repository
        - Old Version
        - New Version
    *   - Foonathan Memory Vendor
        - `eProsima/foonathan_memory_vendor <https://github.com/eProsima/foonathan_memory_vendor>`__
        - `v1.3.1 <https://github.com/eProsima/foonathan_memory_vendor/releases/tag/v1.3.1>`__
        - `v1.3.1 <https://github.com/eProsima/foonathan_memory_vendor/releases/tag/v1.3.1>`__
    *   - Fast CDR
        - `eProsima/Fast-CDR <https://github.com/eProsima/Fast-CDR>`__
        - `v2.3.0 <https://github.com/eProsima/Fast-CDR/releases/tag/v2.3.0>`__
        - `v2.3.0 <https://github.com/eProsima/Fast-CDR/releases/tag/v2.3.0>`__
    *   - Fast DDS
        - `eProsima/Fast-DDS <https://github.com/eProsima/Fast-DDS>`__
        - `v3.2.2 <https://github.com/eProsima/Fast-DDS/releases/tag/v3.2.2>`__
        - `v3.3.0 <https://github.com/eProsima/Fast-DDS/releases/tag/v3.3.0>`__
    *   - Dev Utils
        - `eProsima/dev-utils <https://github.com/eProsima/dev-utils>`__
        - `v1.2.0 <https://github.com/eProsima/dev-utils/releases/tag/v1.2.0>`__
        - `v1.3.0 <https://github.com/eProsima/dev-utils/releases/tag/v1.3.0>`__
    *   - DDS Pipe
        - `eProsima/DDS-Pipe <https://github.com/eProsima/DDS-Pipe.git>`__
        - `v1.2.0 <https://github.com/eProsima/DDS-Pipe/releases/tag/v1.2.0>`__
        - `v1.3.0 <https://github.com/eProsima/DDS-Pipe/releases/tag/v1.3.0>`__

#################
Previous Versions
#################

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
