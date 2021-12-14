.. include:: ../exports/alias.include

.. _user_manual_user_interface:

##############
User Interface
##############

The |ddsrouter| is an application that is executed from command line.

.. contents::
    :local:
    :backlinks: none
    :depth: 1

Source Dependency Libraries
---------------------------

The |ddsrouter| depends on Fast DDS ``fastrtps`` and ``fastcdr`` libraries.
In order to correctly execute the Router, be sure that ``fastrtps`` and ``fastcdr`` are sourced.

.. code-block: bash

    source <path-to-fastdds-installation>/install/setup.bash

.. note:

    If Fast DDS has been installed in the system, these libraries would be sourced by default.


Application Arguments
---------------------

The |ddsrouter| application support several input arguments:

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

It shows the Manual information for the application.

.. code-block:: console

    Usage: Fast DDS Router
    Connect different DDS networks via DDS through LAN or WAN.
    It will build a communication bridge between the different Participants included in the provided configuration file.
    General options:
    -h --help         Print this help message.
    -c --config-path  Path to the Configuration File (yaml format) [Default: ./DDS_ROUTER_CONFIGURATION.yaml].
    -r --reload-time  Time period in milliseconds to reload configuration file. This is needed when FileWatcher functionality is not available (e.g. config file is a symbolic link).
                        Value 0 does not reload file. [Default: 0].
    -d --debug        Activate debug Logs. (Be aware that some logs may require specific CMAKE compilation options.compilation options)


.. _user_manual_user_interface_configuration_file_argument:

Configuration File Argument
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Set the :ref:`user_manual_user_interface_configuration_file`.


.. _user_manual_user_interface_reload_time_argument:

Reload Time Argument
^^^^^^^^^^^^^^^^^^^^

Set the :ref:`user_manual_user_interface_reload_timer` in **milliseconds**.


.. _user_manual_user_interface_debug_argument:

Debug Argument
^^^^^^^^^^^^^^

Activate ``INFO`` and ``DEBUG`` logs for the |ddsrouter| execution.
For this argument to work, the |ddsrouter| must have been compiled in ``debug`` CMake build type, or compiled with
CMake option ``LOG_INFO=ON``.


.. _user_manual_user_interface_configuration_file:

Configuration File
------------------

A |ddsrouter| **requires** one and only one *YAML* configuration file.
Check section :ref:`user_manual_configuration` in order to know how to write this configuration file.

This *YAML* configuration file must be passed as argument to the |ddsrouter| when executed.
If configuration file is not defined as argument, the |ddsrouter| will be initialized with a file with
default name ``DDS_ROUTER_CONFIGURATION.yaml`` that must be in the directory where the application has been executed.
If the configuration file is not passed as argument, and it does not exist a file with the default configuration
file name in the current directory, the application will fail.


Reload Topics
-------------

The topics that the |ddsrouter| is routing could be changed in run time.
Including topics in ``allowedlist`` in the configuration will create new :term:`Writers <DataWriter>` and
:term:`Readers <DataReader>` for each Participant in the Router.
Removing a topic from ``allowedlist`` will disable this topic, and so it will stop routing data in such topic.
Be aware that disabling a topic does not eliminate the entities of that topic.
So, if a topic has been active before, the Writers and Readers will still be present in the |ddsrouter| and will still
receive data.

There exist two methods to reload the topics, an active and a passive one.
Both methods work over the same configuration file which the |ddsrouter| has been initialized with.


File Watcher
^^^^^^^^^^^^

A File Watcher is a process that runs in background and watch the |ddsrouter| configuration file.
Every time that the file is changed, the OS sends a notification, and the File Watcher listens such notification
and interact with the |ddsrouter| in order to reload the topics.
This event occurs every time the configuration file is saved.

FileWatcher is used in every |ddsrouter| execution by default.
However, this method does not work properly in specific scenarios where the file being watched is not a real file but
a link (e.g. Kubernetes executions).

.. _user_manual_user_interface_reload_timer:

Reload Timer
^^^^^^^^^^^^

A timer could be set in order to periodically reload the configuration file.
The configuration file will be automatically reloaded with the period set.


Close Application
-----------------

In order to stop a |ddsrouter| application, just send an interruption ``SIGINT | ^C`` signal *(signal value 2)* to the
process.

