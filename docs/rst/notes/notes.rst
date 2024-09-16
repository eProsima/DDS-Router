.. include:: ../exports/alias.include

.. _release_notes:

.. .. include:: forthcoming_version.rst

##############
Version v3.0.0
##############

This release includes the following **Features**:

* Add support for Fast DDS v3.
* Remove ``publish-type`` tag from :ref:`user_manual_configuration_specs_monitor` configuration.
* Remove ``DiscoveryServerConnectionAdress``.

This release includes the following **Bugfixes**:

* Reset cache changes before returning them to the pool.

This release includes the following **Configuration features**:

* Change the default value of :ref:`user_manual_user_interface_log_filter_argument` to ``DDSROUTER``.
* Change the default value of :ref:`user_manual_user_interface_log_verbosity_argument` to ``error``.

This release includes the following **CI improvements**:

* Upgrade to Ubuntu Noble (24.04).
* Remove Ubuntu Focal (20.04) from the CI.
* Limit permissions and include missing certificates in security tests.
* Add stress tests.
* Reduce the list of flaky Docker tests.

This release includes the following **Dependencies Update**:

.. list-table::
    :header-rows: 1

    *   -
        - Repository
        - Old Version
        - New Version
    *   - Foonathan Memory Vendor
        - `eProsima/foonathan_memory_vendor <https://github.com/eProsima/foonathan_memory_vendor>`_
        - `v1.3.1 <https://github.com/eProsima/foonathan_memory_vendor/releases/tag/v1.3.1>`_
        - `v1.3.1 <https://github.com/eProsima/foonathan_memory_vendor/releases/tag/v1.3.1>`_
    *   - Fast CDR
        - `eProsima/Fast-CDR <https://github.com/eProsima/Fast-CDR>`_
        - `v2.2.0 <https://github.com/eProsima/Fast-CDR/releases/tag/v2.2.0>`_
        - `v2.2.4 <https://github.com/eProsima/Fast-CDR/releases/tag/v2.2.4>`_
    *   - Fast DDS
        - `eProsima/Fast-DDS <https://github.com/eProsima/Fast-DDS>`_
        - `v2.14.0 <https://github.com/eProsima/Fast-DDS/releases/tag/v2.14.0>`_
        - `v3.0.1 <https://github.com/eProsima/Fast-DDS/releases/tag/v3.0.1>`_
    *   - Dev Utils
        - `eProsima/dev-utils <https://github.com/eProsima/dev-utils>`_
        - `v0.6.0 <https://github.com/eProsima/dev-utils/releases/tag/v0.6.0>`_
        - `v1.0.0 <https://github.com/eProsima/dev-utils/releases/tag/v1.0.0>`_
    *   - DDS Pipe
        - `eProsima/DDS-Pipe <https://github.com/eProsima/DDS-Pipe.git>`_
        - `v0.4.0 <https://github.com/eProsima/DDS-Pipe/releases/tag/v0.4.0>`__
        - `v1.0.0 <https://github.com/eProsima/DDS-Pipe/releases/tag/v1.0.0>`__

#################
Previous Versions
#################

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
