# eProsima DDS Router

<a href="http://www.eprosima.com"><img src="https://encrypted-tbn3.gstatic.com/images?q=tbn:ANd9GcSd0PDlVz1U_7MgdTe0FRIWD0Jc9_YH-gGi0ZpLkr-qgCI6ZEoJZ5GBqQ" align="left" hspace="8" vspace="2" width="100" height="100" ></a>

[![License](https://img.shields.io/github/license/eProsima/DDS-Router.svg)](https://opensource.org/licenses/Apache-2.0)
[![Releases](https://img.shields.io/github/v/release/eProsima/DDS-Router?sort=semver)](https://github.com/eProsima/DDS-Router/releases)
[![Issues](https://img.shields.io/github/issues/eProsima/DDS-Router.svg)](https://github.com/eProsima/DDS-Router/issues)
[![Forks](https://img.shields.io/github/forks/eProsima/DDS-Router.svg)](https://github.com/eProsima/DDS-Router/network/members)
[![Stars](https://img.shields.io/github/stars/eProsima/DDS-Router.svg)](https://github.com/eProsima/DDS-Router/stargazers)
[![test](https://github.com/eProsima/DDS-Router/actions/workflows/test.yml/badge.svg)](https://github.com/eProsima/DDS-Router/actions/workflows/test.yml)
[![codecov](https://codecov.io/gh/eProsima/DDS-Router/branch/main/graph/badge.svg?token=6NA5PVA9QL)](https://codecov.io/gh/eProsima/DDS-Router)
[![Documentation Status](https://readthedocs.org/projects/eprosima-dds-router/badge/?version=latest)](https://eprosima-dds-router.readthedocs.io/en/latest/)

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


## Documentation

You can access the documentation online, which is hosted on [Read the Docs](https://eprosima-dds-router.readthedocs.io).

* [Introduction](https://eprosima-dds-router.readthedocs.io/en/latest/rst/formalia/titlepage.html)
* [Getting Started](https://eprosima-dds-router.readthedocs.io/en/latest/rst/getting_started/project_overview.html)
* [User Manual](https://eprosima-dds-router.readthedocs.io/en/latest/rst/user_manual/user_interface.html)
* [Examples](https://eprosima-dds-router.readthedocs.io/en/latest/rst/examples/echo_example.html)
* [Use Cases](https://eprosima-dds-router.readthedocs.io/en/latest/rst/use_cases/ros_cloud.html)
* [Developer Manual](https://eprosima-dds-router.readthedocs.io/en/latest/rst/developer_manual/installation/sources/linux.html)
* [Release Notes](https://eprosima-dds-router.readthedocs.io/en/latest/rst/notes/notes.html)


## Installation Guide

The instructions for installing the *DDS Router* application from sources and its required dependencies on a Linux
environment are provided below. These installation instructions are a summarized version of the complete
[installation guide](https://eprosima-dds-router.readthedocs.io/en/latest/rst/developer_manual/installation/sources/linux.html) available online. Instructions for installing *DDS Router* on a **Windows** platform can be found
[here](https://eprosima-dds-router.readthedocs.io/en/latest/rst/developer_manual/installation/sources/windows.html).

### Requirements

*eProsima DDS Router* requires the following tools to be installed in the system:
* [CMake](https://cmake.org/), [g++](https://gcc.gnu.org/), [pip](https://pypi.org/project/pip/), [wget](https://www.gnu.org/software/wget/) and [git](https://git-scm.com/)
* [Colcon](https://colcon.readthedocs.io/en/released/) [optional, not required for CMake-only installation]
* [Gtest](https://github.com/google/googletest) [for test only]
* [PyYAML](https://pyyaml.org/) [for YAML Validator only]
* [jsonschema](https://python-jsonschema.readthedocs.io/) [for YAML Validator only]

#### CMake, g++, pip, wget and git

These packages provide the tools required to install DDS Router and its dependencies from command line. Install
[CMake](https://cmake.org/), [g++](https://gcc.gnu.org/), [pip](https://pypi.org/project/pip/), [wget](https://www.gnu.org/software/wget/) and [git](https://git-scm.com/) using the package manager of the appropriate Linux distribution. For
example, on Ubuntu use the command:

```bash
sudo apt install cmake g++ pip wget git
```

#### Colcon

[colcon](https://colcon.readthedocs.io/en/released/) is a command line tool based on [CMake](https://cmake.org/) aimed at building sets of software packages. Install the ROS 2 development tools ([colcon](https://colcon.readthedocs.io/en/released/) and [vcstool](https://pypi.org/project/vcstool/)) by executing the following command:

```bash
pip3 install -U colcon-common-extensions vcstool
```

If this fails due to an Environment Error, add the `--user` flag to the `pip3` installation command.

#### Gtest

[Gtest](https://github.com/google/googletest) is a unit testing library for C++. By default, *DDS Router* does not
compile tests. It is possible to activate them with the opportune [CMake options](https://colcon.readthedocs.io/en/released/reference/verb/build.html#cmake-options) when calling [colcon](https://colcon.readthedocs.io/en/released/) or
[CMake](https://cmake.org/). For a detailed description of the Gtest installation process, please refer to the
[Gtest Installation Guide](https://github.com/google/googletest).

#### PyYAML

[PyYAML](https://pyyaml.org/) is a YAML parser and emitter for Python.
It is used by the DDS-Router YAML Validator for loading the content of configuration files.
Install `pyyaml` by executing the following command:

```bash
pip3 install -U pyyaml
```

#### jsonschema

[jsonschema](https://python-jsonschema.readthedocs.io/) is an implementation of the JSON Schema specification for
Python.
It is used by the DDS-Router YAML Validator for performing validation of configuration files against a given JSON
schema.
Install `jsonschema` by executing the following command:

```bash
pip3 install -U jsonschema
```

### Dependencies

#### Asio and TinyXML2 libraries

Asio is a cross-platform C++ library for network and low-level I/O programming, which provides a consistent asynchronous
model. TinyXML2 is a simple, small and efficient C++ XML parser. Install these libraries using the package manager of
the appropriate Linux distribution. For example, on Ubuntu use the command:

```bash
sudo apt install libasio-dev libtinyxml2-dev
```

#### OpenSSL

[OpenSSL](https://www.openssl.org/) is a robust toolkit for the TLS and SSL protocols and a general-purpose cryptography
library. Install OpenSSL using the package manager of the appropriate Linux distribution. For example, on Ubuntu use the
command:

```bash
sudo apt install libssl-dev
```

#### yaml-cpp

[yaml-cpp](https://github.com/jbeder/yaml-cpp) is a [YAML](http://www.yaml.org/) parser and emitter in C++ matching the
[YAML 1.2 spec](http://www.yaml.org/spec/1.2/spec.html), and is used by *DDS Router* application to parse the provided
configuration files. Install yaml-cpp using the package manager of the appropriate Linux distribution. For example, on
Ubuntu use the command:

```bash
sudo apt install libyaml-cpp-dev
```

#### eProsima dependencies

If it already exists in the system an installation of *Fast DDS* library with version greater than *2.4.0*, just source
this library when building the *DDS Router* application by using the command:

```bash
source <fastdds-installation-path>/install/setup.bash
```

In other case, just download *Fast DDS* project from sources and build it together with *DDS Router* using colcon as it
is explained in the following section.

### Colcon installation

1. Create a `DDS-Router` directory and download the `.repos` file that will be used to install *DDS Router* and its dependencies:

```bash
mkdir -p ~/DDS-Router/src
cd ~/DDS-Router
wget https://raw.githubusercontent.com/eProsima/DDS-Router/main/ddsrouter.repos
vcs import src < ddsrouter.repos
```

2. Build the packages:

```bash
colcon build --packages-select-regex ddsrouter
```

This repository holds several colcon packages.
These packages are:

* `ddsrouter_core`: library with the main functionality of the DDS Router.
* `ddsrouter_yaml`: library to configure a DDS Router from a YAML.
* `ddsrouter_tool`: application to execute a DDS Router from a YAML configuration file.
* `ddsrouter_docs`: package to generate the DDS Router documentation using sphinx.
* `ddsrouter_yaml_validator`: application to validate DDS Router YAML configuration files.

> *NOTE:* Those packages could be installed and use independently (according with each package dependency).
  In order to compile only a package and its dependencies, use the colcon argument `--packages-up-to <package>`.
  In order to explicitly skip some of these packages, use the colcon argument
  `--packages-skip <package1> [<package2> ...]`.

### Run an application

To run the *DDS Router* application, source the installation environment and execute the executable file that has been
installed in `<install-path>/ddsrouter_tool/bin/ddsrouter`:

```bash
# Source installation
source <install-path>/setup.bash

# Execute DDS Router
./<install-path>/ddsrouter_tool/bin/ddsrouter
```

### Validate a configuration file

To validate a *DDS Router* YAML configuration file, execute the following commands:

```bash
# Source installation
source <install-path>/setup.bash

# Validate a DDS Router configuration file
ddsrouter_yaml_validator --config-file ddsrouter-config.yaml
```

### Testing

By default, *DDS Router* does not compile tests. However, they can be activated by downloading and installing
[Gtest](https://github.com/google/googletest) and building with CMake option `-DBUILD_TESTS=ON`. Once done, tests
can be run with the following command:

```bash
colcon test --packages-select-regex ddsrouter --event-handler=console_direct+
```

## Getting Help

If you need support you can reach us by mail at `support@eProsima.com` or by phone at `+34 91 804 34 48`.
