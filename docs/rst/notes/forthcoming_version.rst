
###################
Forthcoming Version
###################

Next release will include the following **improvementes**:

* New :code:`FastPayloalPool` class that will allow unblocking access to shared payloads stored by FastDDS and
  DDS Router.
  This slightly improves the **performance** of the router.

* New :code:`ThreadPool` class for handling parallel processing.
  Limit the number of threads spawned by the DDS Router, improving the performance of the application.
  The user can set-up this parameter on the YAML configuration :ref:`thread_configuration`.

Next release will include the following **major changes**:

* New :ref:`yaml_validator`, a simple tool to assert the correctness of DDS Router configuration files.
* The internal package `ddsrouter_event` is joint within the internal package `ddsrouter_utils`.

Next release will fix the following **minor bugs**:

* Fix race condition occurred when handling signals (due to concurrent access to *SignalManager* singleton).
* Bug in TLS Configuration initialization when setting server configuration files.
