.. include:: ../exports/alias.include

.. _forthcoming_notes:

###################
Forthcoming Version
###################

Next release will include the following **Configuration features**:

* Support TLS configuration and communication.
* Support IPv6 communication via UDP, TCP and TLS.
* Support DNS by given Domain Name in configuration instead of an IP address.
* Support keyed topics.

Next release will include the following **User Interface features**:

* Allow to close the Router application gently with ``SIGTERM`` (``kill``) as with ``SIGINT`` (``^C``).

Next release will include the following **Continuous-Integration features**:

* Add communication tests for UDP, TCP and TLS WAN cases.
* Extend tool test with more complex configurations.
* Erase Flaky tests from CI required passing tests.
* Implement a new class to check that no warning or error logs are produced during test executions.
* Add gMock to test libraries.

Next release will fix the following **major bugs**:

* Fix GUID creation when explicit guid is provided.

Next release will fix the following **minor bugs**:

* Change ``YAML`` example configurations to ``YAML`` format (instead of ``JSON``) fixing
  an issue when blank lines were missing at the end of the file.
* Normalize the error and process exit when failure.
* Fix documentation typos.
