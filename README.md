# eProsima DDS Router

<a href="http://www.eprosima.com"><img src="https://encrypted-tbn3.gstatic.com/images?q=tbn:ANd9GcSd0PDlVz1U_7MgdTe0FRIWD0Jc9_YH-gGi0ZpLkr-qgCI6ZEoJZ5GBqQ" align="left" hspace="8" vspace="2" width="100" height="100" ></a>

[![License](https://img.shields.io/github/license/eProsima/DDS-Router.svg)](https://opensource.org/licenses/Apache-2.0)
[![Releases](https://img.shields.io/github/v/release/eProsima/DDS-Router?sort=semver)](https://github.com/eProsima/DDS-Router/releases)
[![Issues](https://img.shields.io/github/issues/eProsima/DDS-Router.svg)](https://github.com/eProsima/DDS-Router/issues)
[![Forks](https://img.shields.io/github/forks/eProsima/DDS-Router.svg)](https://github.com/eProsima/DDS-Router/network/members)
[![Stars](https://img.shields.io/github/stars/eProsima/DDS-Router.svg)](https://github.com/eProsima/DDS-Router/stargazers)
[![test](https://github.com/eProsima/DDS-Router/actions/workflows/test.yml/badge.svg)](https://github.com/eProsima/DDS-Router/actions/workflows/test.yml)
[![codecov](https://codecov.io/gh/eProsima/DDS-Router/branch/main/graph/badge.svg?token=6NA5PVA9QL)](https://codecov.io/gh/eProsima/DDS-Router)
<!-- [![Documentation Status](https://readthedocs.org/projects/dds-router/badge/?version=latest)](https://dds-router.readthedocs.io/en/latest/) -->

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

Following are some of the key features of *eProsima DDS Router*:

* **WAN communication over TCP**: it supports WAN over TCP communication to establish DDS communications over the
  Internet.
* **Distributed nature**: the user may deploy intermediate *DDS Router* nodes to discover new entities that enter and
  leave the network dynamically.
* **Efficient data routing**: *DDS Router* avoids data introspection achieving a zero-copy system in data
  forwarding.
* **Easy deployment**: it is based on an easily configurable modular system for users with no knowledge of computer
  networks.
* **Topic allowlisting**: it is possible to configure a *DDS Router* to forward just the user data belonging to a
  topic specified by the user.
