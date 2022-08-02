.. include:: ../exports/alias.include

.. _release_notes:

.. todo:
    uncomment the include of forthcoming when new info is added
    .. include:: forthcoming_version.rst

.. include:: forthcoming_version.rst

##############
Version v0.4.0
##############

This release includes the following **features**:

* New :ref:`yaml_validator`, a simple tool to assert the correctness of DDS Router configuration files.
* New :ref:`user_manual_user_interface_version_argument` to show the current version of DDS Router.

This release includes the following **improvementes**:

* New :code:`FastPayloalPool` class that will allow unblocking access to shared payloads stored by FastDDS and
  DDS Router.
  This slightly improves the **performance** of the router.

* New :code:`ThreadPool` class for handling parallel processing.
  Limit the number of threads spawned by the DDS Router, improving the performance of the application.
  The user can set-up this parameter on the YAML configuration :ref:`thread_configuration`.

This release includes the following **major changes**:

* New auxiliary package :code:`ddsrouter_cmake` to implement general CMake functions and tools.
* The internal package :code:`ddsrouter_event` is joint within the internal package :code:`ddsrouter_utils`.

This release includes the following **minor changes**:

* Refactor all :code:`enum` to :code:`enum class`.

This release includes the following **Continuous-Integration features**:

* New CI workflow to build dependencies nightly.
  This allows to run the CI faster, as the build is only done for the ddsrouter packages and not the whole fastdds.
* Add :code:`ubuntu 22.04` to platforms in CI.
* Add :code:`windows 19` to platforms in CI.

This release fixes the following **minor bugs**:

* Fix race condition occurred when handling signals (due to concurrent access to *SignalManager* singleton).
* Bug in TLS Configuration initialization when setting server configuration files.
* Set FastDDS DomainParticipants and Readers listeners after creation to avoid race conditions in entities creation.

#################
Previous Versions
#################

.. include:: previous_versions/v0.3.0.rst
.. include:: previous_versions/v0.2.0.rst
.. include:: previous_versions/v0.1.0.rst
