.. include:: ../exports/alias.include

.. _user_manual_user_interface:

##############
User Interface
##############

|eddsrouter| is a user application executed from the command line and configured through a YAML configuration file.

.. contents::
    :local:
    :backlinks: none
    :depth: 1

Source Dependency Libraries
---------------------------

|eddsrouter| depends on |fastdds| ``fastrtps`` and ``fastcdr`` libraries.
In order to correctly execute the Router, make sure that ``fastrtps`` and ``fastcdr`` are properly sourced.

.. code-block:: bash

    source <path-to-fastdds-installation>/install/setup.bash

.. note::

    If Fast DDS has been installed in the system, these libraries would be sourced by default.


.. _user_manual_user_interface_application_arguments:

Application Arguments
---------------------

The |ddsrouter| application supports several input arguments:

.. list-table::
    :header-rows: 1

    *   - Command
        - Option
        - Long option
        - Value
        - Default Value

    *   - :ref:`user_manual_user_interface_help_argument`
        - ``-h``
        - ``--help``
        -
        -

    *   - :ref:`user_manual_user_interface_version_argument`
        - ``-v``
        - ``--version``
        -
        -

    *   - :ref:`user_manual_user_interface_configuration_file_argument`
        - ``-c``
        - ``--config-path``
        - Readable File Path
        - ``./DDS_ROUTER_CONFIGURATION.yaml``

    *   - :ref:`user_manual_user_interface_reload_time_argument`
        - ``-r``
        - ``--reload-time``
        - Unsigned Integer
        - ``0``

    *   - :ref:`user_manual_user_interface_debug_argument`
        - ``-d``
        - ``--debug``
        -
        -

    *   - :ref:`user_manual_user_interface_log_verbosity_argument`
        -
        - ``--log-verbosity``
        - ``info`` ``warning`` ``error``
        - ``warning``

    *   - :ref:`user_manual_user_interface_log_filter_argument`
        -
        - ``--log-filter``
        - String
        - ``"DDSROUTER"``

.. _user_manual_user_interface_help_argument:

Help Argument
^^^^^^^^^^^^^

It shows the usage information of the application.

.. code-block:: console

    Usage: Fast DDS Router
    Connect different DDS networks via DDS through LAN or WAN.
    It will build a communication bridge between the different Participants included in the provided configuration file.
    To stop the execution gracefully use SIGINT (C^) or SIGTERM (kill) signals.
    General options:

    Application help and information.
    -h --help           Print this help message.
    -v --version        Print version, branch and commit hash.

    Application parameters
    -c --config-path    Path to the Configuration File (yaml format) [Default: ./DDS_ROUTER_CONFIGURATION.yaml].
    -r --reload-time    Time period in seconds to reload configuration file. This is needed when FileWatcher functionality is not available (e.g. config file is a symbolic link). Value 0 does not reload file. [Default: 0].
    -t --timeout        Set a maximum time in seconds for the Router to run. Value 0 does not set maximum. [Default: 0].

    Debug options
    -d --debug          Set log verbosity to Info (Using this option with --log-filter and/or --log-verbosity will head to undefined behaviour).
        --log-filter     Set a Regex Filter to filter by category the info and warning log entries. [Default = "DDSROUTER"].
        --log-verbosity  Set a Log Verbosity Level higher or equal the one given. (Values accepted: "info","warning","error" no Case Sensitive) [Default = "warning"].

.. _user_manual_user_interface_version_argument:

Version Argument
^^^^^^^^^^^^^^^^

It shows the current version of the DDS Router and the hash of the last commit of the compiled code.

.. _user_manual_user_interface_configuration_file_argument:

Configuration File Argument
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Please refer to :ref:`user_manual_user_interface_configuration_file` for more information on how to build this
configuration file.


.. _user_manual_user_interface_reload_time_argument:

Reload Time Argument
^^^^^^^^^^^^^^^^^^^^

Set the :ref:`user_manual_user_interface_reload_timer` in **seconds**.

.. _user_manual_user_interface_timeout_argument:

Timeout Argument
^^^^^^^^^^^^^^^^

This argument allow to set a maximum time while the application will be running.
Setting this argument will set the number of seconds the application will run until it is killed.
While the application is waiting for timeout, it is still possible to kill it via signal.
Default value ``0`` means that the application will run forever (until kill via signal).

.. _user_manual_user_interface_debug_argument:

Debug Argument
^^^^^^^^^^^^^^

This argument enables the |ddsrouter| logs so the execution can be followed by internal debugging information.
This argument sets :ref:`user_manual_user_interface_log_verbosity_argument` to ``info`` and
:ref:`user_manual_user_interface_log_filter_argument` to ``DDSROUTER``.
For more information about debugging options, refer to :ref:`user_manual_user_interface_log`.

.. note::

    If this argument is used with any of the other arguments of debugging, the behavior depends on the order
    of parser of the arguments.

.. _user_manual_user_interface_log_verbosity_argument:

Log Verbosity Argument
^^^^^^^^^^^^^^^^^^^^^^

Set the verbosity level so only log messages with equal or higher importance level are shown.

.. _user_manual_user_interface_log_filter_argument:

Log Filter Argument
^^^^^^^^^^^^^^^^^^^

Set a regex string as filter.
Only log messages with a category that matches this regex will be printed
(``ERROR`` messages will be always shown unless :ref:`user_manual_user_interface_log_verbosity_argument` is
set to ``ERROR``).


.. _user_manual_user_interface_configuration_file:

Configuration File
------------------

A |ddsrouter| **requires** one and only one *YAML* configuration file as the operation of this application is
configured via this *YAML* configuration file.
Please refer to :ref:`user_manual_configuration` for more information on how to build this
configuration file.

This *YAML* configuration file must be passed as argument to the |ddsrouter| when executed.
If no configuration file is provided as argument, the |ddsrouter| will attempt to load a file named
``DDS_ROUTER_CONFIGURATION.yaml`` that must be in the same directory where the application is executed.
If no configuration file is passed as argument, and the default configuration file does not exist
in the current directory, the application will fail.


.. _user_manual_user_interface_reload_topics:

Reload Topics
-------------

The topics that the |ddsrouter| is routing could be changed at runtime.
Including topics in configuration's ``allowlist`` will create new :term:`Writers <DataWriter>` and
:term:`Readers <DataReader>` for each Participant in the Router.
Removing a topic from ``allowlist`` will disable this topic, and so it will stop routing data in such topic.
Be aware that disabling a topic does not eliminate the entities of that topic.
So, if a topic has been active before, the Writers and Readers will still be present in the |ddsrouter| and will still
receive data.

There exist two methods to reload the list of allowed topics, an active and a passive one.
Both methods work over the same configuration file with which the |ddsrouter| has been initialized.


File Watcher
^^^^^^^^^^^^

A File Watcher is a process that runs in the background and tracks changes in the |ddsrouter| configuration file.
Every time the file is changed, the OS sends a notification, and the File Watcher listens such notification
and interacts with the |ddsrouter| in order to reload the topics.
This event occurs every time the configuration file is saved.

FileWatcher is used in every |ddsrouter| execution by default.
However, this method does not work properly in specific scenarios where the file being watched is not a real file but
a link (e.g. Kubernetes executions).

.. _user_manual_user_interface_reload_timer:

Reload Timer
^^^^^^^^^^^^

A timer can be set in order to periodically reload the configuration file.
The configuration file will be automatically reloaded according to the specified time period.


.. _user_manual_user_interface_log:

Log
---

Log module of |ddsrouter| uses the |fastdds| logging module.
This log has 3 severity levels: ``INFO``, ``WARNING`` and ``ERROR``.
Every log has also a category associated.
This is how a log looks like:

.. code::

    Date                     Category         Severity     Log message                   Function
    2022-11-16 14:58:13.375 [MODULE_SUBMODULE Error] It has happen ... because of ... -> Function main

Every log entry has several parts:

- **Date**: format: ``year-month-day hour::minute::second::millisecond`` with millisecond accuracy.
  This is the time when the log was added to the log queue, not when it is printed.
- **Category**: Reference to the module where the log was raised. It is used to filter logs.
- **Severity**: Could be ``Info``, ``Warning`` or ``Error``.
- **Log message**: The actual log message.
- **Function**: Name of the function or method that has produced this log entry.


.. note::

    For ``INFO`` logs to be compiled, the |ddsrouter| must have been compiled with CMake option ``CMAKE_BUILD_TYPE=Debug``,
    or compiled with CMake option ``LOG_INFO=ON``.

    If Fast DDS has been compiled in debug mode, it will print the logs of the DDS Router and Fast DDS mixed.
    In order to skip Fast DDS logs, compile ``fastrtps`` library with CMake option ``-DLOG_NO_INFO=ON``
    or ``CMAKE_BUILD_TYPE`` different to ``Debug``, or use the argument ``


.. _user_manual_user_interface_close_application:

Close Application
-----------------

In order to stop a |ddsrouter| application, use one of the following OS signals:

SIGINT
^^^^^^

Send an interruption ``SIGINT | ^C`` signal *(signal value 2)* to the process.
Press ``Ctrl + C`` in the terminal where the process is running.

SIGTERM
^^^^^^^

Send an interruption ``SIGTERM`` signal *(signal value 15)* to the process by executing the command ``kill <pid>`` in a different terminal, where ``<pid>`` is the id of the process running the |ddsrouter|.
Use the ``ps`` or ``top`` programs to check the processes' ids.

TIMEOUT
^^^^^^^

Setting a maximum amount of seconds that the application will work using argument ``--timeout`` will close the
application once the time has expired.
