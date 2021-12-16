.. include:: ../exports/alias.include

.. _release_notes:

##############
Version v0.0.0
##############

This is the first release of eProsima |ddsrouter|.

This release includes several **features** regarding the routing of DDS data, the configuration and user interaction
with the |ddsrouter|, the different DDS configurations that the application is able to reproduce, etc.

This release includes the following **User Interface features**:

* Application executable.
* Application executable arguments.
* Signal handler to close the application.
* FileWatcher thread to watch and reload the configuration file.
* Periodic timer to force reload configuration.
* Application run-time user logs.
* Application run-time debug logs.
* Error handling:

    * Error log and exit program when reading configuration fails.
    * Error log and exit program when initializing Participants fails.
    * Error log and continuing execution when execution error occurs.


This release includes the following **Configuration features**:

* Allow to execute application with a *YAML* configuration file.
* Support initial topics in allowlist.
* Support block topic filters.
* Different Participant configurations:

    * Domain Id.
    * Discovery Server GuidPrefix.
    * Listening addresses.
    * Connection addresses.


This release includes the following **Routing features**:

* Support to route Topics specified in allowlist regarding Topic name and Topic Type name.
* Support connect to new Topics in run time (by reloading configuration).
* Support disabling a Topic in run time.
* Support enabling a Topic that has been disabled in run time.
* Route messages of each Participant to all the other Participants.
* Data types agnostic.


This release includes the following **DDS features**:

* Allow UDP, TCP and SHM transport communication.
* Allow dynamic discovery of new entities.
* Using Fast DDS RTPS layer for discovery, publication and subscription.


This release includes the following **Participant features**:

* **Echo Participant**.
* **Simple Participant**, able to connect to a Simple Discovery UDP DDS network.
* **Local Discovery Server Participant**, able to connect to a local Discovery Server as Client or Server.
* **WAN Participant**, able to connect to a WAN Discovery Server network as Client or Server.


This release includes the following **Examples**:

* **Echo Example**, to monitor a local simple network.
* **Domain Change Example**, to connect two different domains.
* **ROS 2 Discovery Server Example**, to connect a simple DDS with Discovery Server using ROS 2 configuration.
* **WAN Example**, to connect two DDS networks in different LANs.


This release includes the following **Documentation features**:

* This Documentation.
