.. raw:: html

  <h1>
    eProsima DDS Router Documentation
  </h1>

.. image:: /rst/figures/logo.png
  :height: 100px
  :width: 100px
  :align: left
  :alt: eProsima
  :target: http://www.eprosima.com/

*eProsima DDS Router* is an end-user software application that enables the connection of distributed DDS networks.
That is, DDS entities such as publishers and subscribers deployed in one geographic location and using a dedicated
local network will be able to communicate with other DDS entities deployed in different geographic areas on their own
dedicated local networks as if they were all on the same network through the use of *eProsima DDS Router*.
This is achieved by deploying a *DDS Router* on an edge device of each local network so that the
*DDS Router* routes DDS traffic from one network to the other through WAN communication.

Furthermore, *DDS Router* is a software designed for various forms of distributed networks,
such as mesh networks in which nodes are deployed in different private local networks that are auto-discovered
without any centralized network node, or cloud-based networks where there is a data processing cloud and
multiple geographically distributed edge devices.

########
Overview
########

Following are some of the key features of *eProsima DDS Router*:

  1. **WAN communication over TCP**: it supports WAN over TCP communication to establish DDS communications over the
     Internet.
  2. **Distributed nature**: the user may deploy intermediate *DDS Router* nodes to discover new entities that enter and
     leave the network dynamically.
  3. **Efficient data routing**: *DDS Router* avoids data introspection achieving a zero-copy system in data
     forwarding.
  4. **Easy deployment**: it is based on an easily configurable modular system for users with no knowledge of computer
     networks.
  5. **Topic allowlisting**: it is possible to configure a *DDS Router* to forward just the user data belonging to a
     topic specified by the user.
  6. **Dynamic topic discovery**: underlying entities required for communication under a specific topic are created only
     when needed, thus reducing resources utilization to the minimum.

.. _fig_entities_diagram:

.. figure:: /rst/figures/ddsrouter_cloud.png

###############################
Contacts and Commercial support
###############################

Find more about us at `eProsima's webpage <https://eprosima.com/>`_.

Support available at:

* Email: support@eprosima.com
* Phone: +34 91 804 34 48

#################################
Contributing to the documentation
#################################

*DDS Router Documentation* is an open source project, and as such all contributions, both in the form of
feedback and content generation, are most welcomed.
To make such contributions, please refer to the
`Contribution Guidelines <https://github.com/eProsima/all-docs/blob/master/CONTRIBUTING.md>`_ hosted in our GitHub
repository.

##############################
Structure of the documentation
##############################

This documentation is organized into the sections below.

* :ref:`Installation Manual <installation_manual_linux>`
* :ref:`Getting Started <getting_started>`
* :ref:`User Manual <user_manual_user_interface>`
* :ref:`Examples <examples_echo_example>`
* :ref:`Use cases <ros_cloud>`
* :ref:`Developer Manual <developer_manual_installation_sources_linux>`
* :ref:`Release Notes <release_notes>`
