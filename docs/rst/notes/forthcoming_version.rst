
.. _forthcoming_version:

This release includes the following **mayor changes**:

* New DDS Router library that provides the DDS Router features through a C++ API.
* Division of DDS Router application into several packages.

  - `ddsrouter_event`: C++ library which implements System Operating (SO)-dependent signal handlers.
  - `ddsrouter_utils`:  C++ library which implements various utility functions.
  - `ddsrouter_core`: C++ library which implements the DDS Router operation and exports the DDS Router C++ API.
  - `ddsrouter_yaml`:  C++ library to parse the DDS Router *yaml* configuration files.
  - `ddsrouter_tool`: DDS Router end-user application.

Next release includes the following **User Interface features**:

* Upgrade the *yaml* configuration file to version 2.0 which breaks compatibility with version 1.0.
* Support for both version 1.0 and version 2.0 of the *yaml* configuration file, maintaining version 1.0 by default.
* Improve reporting of errors resulting from parsing a malformed *yaml* configuration file.

This release includes the following **Continuous-Integration features**:

* Add tests for the *yaml* parsing library (`ddsrouter_yaml`).
* Specific testing GitHub actions for each DDS Router package for both Windows and Linux platforms.

Next release will fix the following **major bugs**:

* Fix deadlock between Track and Fast DDS Reader mutex.
* Support any size for in and out messages.

This release includes the following **Documentation features**:

* Update all examples of *yaml* configuration files to be consistent with the new *yaml* configuration version.
* High-level repository structure description and developer contribution guidelines.
