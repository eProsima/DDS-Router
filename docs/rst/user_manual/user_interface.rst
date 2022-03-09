.. include:: ../exports/alias.include

.. _user_manual_user_interface:

##############
User Interface
##############

|eddsrouter| is an application executed from command line.

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

    *   - :ref:`user_manual_user_interface_configuration_file_argument`
        - ``-c``
        - ``--config-path``
        - Readable File Path
        - ``./DDS_ROUTER_CONFIGURATION.yaml``

    *   - :ref:`user_manual_user_interface_reload_time_argument`
        - ``-r``
        - ``--reload-time``
        - Unsigned Integer
        - 0

    *   - :ref:`user_manual_user_interface_debug_argument`
        - ``-d``
        - ``--debug``
        -
        -


.. _user_manual_user_interface_help_argument:

Help Argument
^^^^^^^^^^^^^

It shows the usage information of the application.

.. code-block:: console

    Usage: Fast DDS Router
    Connect different DDS networks via DDS through LAN or WAN.
    It will build a communication bridge between the different Participants included in the provided configuration file.
    To close the execution gracefully use SIGINT (C^) or SIGTERM (kill).
    General options:
    -h --help         Print this help message.
    -c --config-path  Path to the Configuration File (yaml format) [Default: ./DDS_ROUTER_CONFIGURATION.yaml].
    -r --reload-time  Time period in seconds to reload configuration file. This is needed when FileWatcher functionality is not available (e.g. config file is a symbolic link).
                        Value 0 does not reload file. [Default: 0].
    -d --debug        Activate debug Logs (be aware that some logs may require specific CMAKE compilation options).


.. _user_manual_user_interface_configuration_file_argument:

Configuration File Argument
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Set the :ref:`user_manual_user_interface_configuration_file`.


.. _user_manual_user_interface_reload_time_argument:

Reload Time Argument
^^^^^^^^^^^^^^^^^^^^

Set the :ref:`user_manual_user_interface_reload_timer` in **seconds**.


.. _user_manual_user_interface_debug_argument:

Debug Argument
^^^^^^^^^^^^^^

Activate ``INFO`` and ``DEBUG`` logs for the |ddsrouter| execution.
For this argument to work, the |ddsrouter| must have been compiled with CMake option ``CMAKE_BUILD_TYPE=Debug``,
or compiled with CMake option ``LOG_INFO=ON``.

.. note::

    If this option is enabled and Fast DDS has been compiled in debug mode, it will print the logs of the DDS Router
    and Fast DDS mixed.
    In order to skip Fast DDS logs, compile ``fastrtps`` library with CMake option ``-DLOG_NO_INFO=ON``
    or ``CMAKE_BUILD_TYPE`` different to ``Debug``.



.. _user_manual_user_interface_configuration_file:

Configuration File
------------------

A |ddsrouter| **requires** one and only one *YAML* configuration file.
Check section :ref:`user_manual_configuration` in order to know how to write this configuration file.

This *YAML* configuration file must be passed as argument to the |ddsrouter| when executed.
If no configuration file is provided as argument, the |ddsrouter| will attempt to load a file named
``DDS_ROUTER_CONFIGURATION.yaml`` that must be in the same directory where the application is executed.
If no configuration file is passed as argument, and the default configuration file does not exist
in the current directory, the application will fail.


.. _user_manual_user_interface_reload_topics:

Reload Topics
-------------

The topics that the |ddsrouter| is routing could be changed at runtime.
Including topics in configuration's ``allowedlist`` will create new :term:`Writers <DataWriter>` and
:term:`Readers <DataReader>` for each Participant in the Router.
Removing a topic from ``allowedlist`` will disable this topic, and so it will stop routing data in such topic.
Be aware that disabling a topic does not eliminate the entities of that topic.
So, if a topic has been active before, the Writers and Readers will still be present in the |ddsrouter| and will still
receive data.

There exist two methods to reload the list of allowed topics, an active and a passive one.
Both methods work over the same configuration file with which the |ddsrouter| has been initialized.


File Watcher
^^^^^^^^^^^^

A File Watcher is a process that runs in the background and watches for changes in the |ddsrouter| configuration file.
Every time the file is changed, the OS sends a notification, and the File Watcher listens such notification
and interacts with the |ddsrouter| in order to reload the topics.
This event occurs every time the configuration file is saved.

FileWatcher is used in every |ddsrouter| execution by default.
However, this method does not work properly in specific scenarios where the file being watched is not a real file but
a link (e.g. Kubernetes executions).

.. _user_manual_user_interface_reload_timer:

Reload Timer
^^^^^^^^^^^^

A timer could be set in order to periodically reload the configuration file.
The configuration file will be automatically reloaded according to the specified time period.


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

Send an interruption ``SIGTERM`` signal *(signal value 15)* to the process.
Write command ``kill <pid>`` in a different terminal, where ``<pid>`` is the id of the process running the |ddsrouter|.
Use ``ps`` or ``top`` programs to check the process ids.
