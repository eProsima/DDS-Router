.. include:: ../exports/alias.include

.. _release_notes:

.. .. include:: forthcoming_version.rst

##############
Version v2.3.0
##############

.. warning::

    This is the last release of the ``2.x`` series.
    This branch has reached its End-of-Life (EOL) and will receive no further releases, bugfixes or security updates.
    Users are encouraged to migrate to the latest stable version of |eddsrouter|.

This release includes the following **Configuration features**:

* Change the default ``logging`` filter and set the default verbosity to ``error``.

This release includes the following **Bugfixes**:

* Reset cache changes before returning them to the pool.
* Update the security certificates used in the tests.
* Fix the ``duplicate explicit target name`` warning in the documentation.

This release includes the following **Testing improvements**:

* New tests to stress the repeater mode.
* New tests to stress reliability.
* Limit the permissions and include the missing certificates in the security tests.

This release includes the following **CI improvements**:

* Upgrade to Ubuntu Noble (24.04).
* Remove Ubuntu Focal (20.04) from the CI.
* Support multiple versions of Fast DDS in the CI.
* Empty the XTSAN tests list.

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
        - `v2.2.0 <https://github.com/eProsima/Fast-CDR/releases/tag/v2.2.0>`_
    *   - Fast DDS
        - `eProsima/Fast-DDS <https://github.com/eProsima/Fast-DDS>`_
        - `v2.14.0 <https://github.com/eProsima/Fast-DDS/releases/tag/v2.14.0>`_
        - `v2.14.7 <https://github.com/eProsima/Fast-DDS/releases/tag/v2.14.7>`_
    *   - Dev Utils
        - `eProsima/dev-utils <https://github.com/eProsima/dev-utils>`_
        - `v0.6.0 <https://github.com/eProsima/dev-utils/releases/tag/v0.6.0>`_
        - `v0.7.0 <https://github.com/eProsima/dev-utils/releases/tag/v0.7.0>`_
    *   - DDS Pipe
        - `eProsima/DDS-Pipe <https://github.com/eProsima/DDS-Pipe.git>`_
        - `v0.4.0 <https://github.com/eProsima/DDS-Pipe/releases/tag/v0.4.0>`__
        - `v0.5.0 <https://github.com/eProsima/DDS-Pipe/releases/tag/v0.5.0>`__

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
