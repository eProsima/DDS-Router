[![DDS Router](resources/images/github_banner_ddsrouter.png)](https://eprosima.com/middleware/tools/eprosima-dds-router)

<br>

<div class="menu" align="center">
    <strong>
        <a href="https://eprosima.com/index.php/downloads-all">Download</a>
        <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
        <a href="https://eprosima-dds-router.readthedocs.io/en/latest/">Docs</a>
        <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
        <a href="https://eprosima.com/index.php/company-all/news">News</a>
        <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
        <a href="https://x.com/EProsima">X</a>
        <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
        <a href="mailto:info@eprosima.com">Contact Us</a>
    </strong>
</div>

<br><br>

<div class="badges" align="center">
    <a href="https://opensource.org/licenses/Apache-2.0"><img alt="License" src="https://img.shields.io/github/license/eProsima/DDS-Router.svg"/></a>
    <a href="https://github.com/eProsima/DDS-Router/releases"><img alt="Releases" src="https://img.shields.io/github/v/release/eProsima/DDS-Router?sort=semver"/></a>
    <a href="https://github.com/eProsima/DDS-Router/issues"><img alt="Issues" src="https://img.shields.io/github/issues/eProsima/DDS-Router.svg"/></a>
    <a href="https://github.com/eProsima/DDS-Router/network/members"><img alt="Forks" src="https://img.shields.io/github/forks/eProsima/DDS-Router.svg"/></a>
    <a href="https://github.com/eProsima/DDS-Router/stargazers"><img alt="Stars" src="https://img.shields.io/github/stars/eProsima/DDS-Router.svg"/></a>
    <br>
    <a href="https://eprosima-dds-router.readthedocs.io"><img alt="Documentation badge" src="https://img.shields.io/readthedocs/eprosima-dds-router.svg"/></a>
    <a href="https://github.com/eProsima/DDS-Router/actions/workflows/nightly-windows-ci.yml"><img alt="Windows CI" src="https://img.shields.io/github/actions/workflow/status/eProsima/DDS-Router/nightly-windows-ci.yml?label=Windows%20CI"></a>
    <a href="https://github.com/eProsima/DDS-Router/actions/workflows/nightly-ubuntu-ci.yml"><img alt="Ubuntu CI" src="https://img.shields.io/github/actions/workflow/status/eProsima/DDS-Router/nightly-ubuntu-ci.yml?label=Ubuntu%20CI"></a>
    <a href="https://github.com/eProsima/DDS-Router/actions/workflows/nightly-system-ci.yml"><img alt="System CI" src="https://img.shields.io/github/actions/workflow/status/eProsima/DDS-Router/nightly-system-ci.yml?label=System%20CI"></a>
</div>

<br><br>

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
* **Dynamic topic discovery**: the user does not need to fully specify over which topics to communicate (i.e. provide
  concrete topic names and types). The discovery of topics matching the allowlisting rules automatically triggers the
  creation of all entities required for communication.

![eProsima DDS Router overall architecture](docs/rst/figures/ddsrouter_cloud_white_background.png)

## Commercial support

Looking for commercial support? Write us to info@eprosima.com

Find more about us at [eProsima’s webpage](https://eprosima.com/).

## Documentation

You can access the documentation online, which is hosted on [Read the Docs](https://eprosima-dds-router.readthedocs.io).

* [Introduction](https://eprosima-dds-router.readthedocs.io/en/latest/rst/formalia/titlepage.html)
* [Getting Started](https://eprosima-dds-router.readthedocs.io/en/latest/rst/getting_started/project_overview.html)
* [Installation Manual](https://eprosima-dds-router.readthedocs.io/en/latest/rst/developer_manual/installation/sources/linux.html)
* [User Manual](https://eprosima-dds-router.readthedocs.io/en/latest/rst/user_manual/user_interface.html)
* [Examples](https://eprosima-dds-router.readthedocs.io/en/latest/rst/examples/echo_example.html)
* [Use Cases](https://eprosima-dds-router.readthedocs.io/en/latest/rst/use_cases/ros_cloud.html)
* [Developer Manual](https://eprosima-dds-router.readthedocs.io/en/latest/rst/developer_manual/installation/sources/linux.html)
* [Release Notes](https://eprosima-dds-router.readthedocs.io/en/latest/rst/notes/notes.html)
