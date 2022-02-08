.. include:: ../exports/alias.include

.. _forthcoming_notes:

###################
Forthcoming Version
###################

Next release will include the following **Configuration features**:

* Support DNS by given Domain Name in configuration instead of an IP address.
* Support keyed topics.
* Support TLS configuration and communication.

Next release will include the following **User Interface features**:

* Allow to close the Router application gently with ``SIGTERM`` (``kill``) as with ``SIGINT`` (``^C``).

Next release will include the following **Continuos-Integration features**:

* Add communication tests for UDP, TCP and TLS WAN cases.
* Extend tool test with more complex configurations.
* Erase Flaky tests from CI required passing tests.
* Implement a new class to check test executions does not produce warning or error logs.
* Add gMock to test libraries.

Next release will fix the following **mayor bugs**:

* Fix GUID creation when explicit guid is provided.

Next release will fix the following **minor bugs**:

* Normalize the error and process exit when failure.
* Fix documentation typos.
