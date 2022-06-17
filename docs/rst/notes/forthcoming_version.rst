
###################
Forthcoming Version
###################

Next release will include the following **improvementes**:

* New :code:`FastPayloalPool` class that will allow unblocking access to shared payloads stored by FastDDS and
  DDS Router.
  This slightly improves the **performance** of the router.

Next release will include the following **major changes**:

* New :ref:`yaml_validator`, a simple tool to assert the correctness of DDS Router configuration files.
* The subpackage `ddsrouter_event` is joint within the subpackage `ddsrouter_utils`.

Next release will fix the following **minor bugs**:

* Fix race condition occurred when handling signals (due to concurrent access to *SignalManager* singleton).
