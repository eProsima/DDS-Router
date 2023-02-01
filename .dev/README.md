# eProsima DDS Router Developer Directory

This directory and this documentation is only targeted to developers and users highly familiarized with this
project and the repository.

---

## Repository Setup

A colcon package is a code package that is able to be compiled, installed and used independently.
All this subdirectory packages have a file `package.xml` that specify how they have to be treated by colcon,
and its dependencies.

This repository follows the ROS 2 convention of holding various colcon packages within the same git repository.
This way there could be different packages that could be used independently, not being forced to used them if
the requirements or dependencies of higher level packages are not fulfilled
(e.g. `yamlcpp` is no longer a requirement for the whole project, but only for `ddsrouter_yaml` and `ddsrouter_tool`).

The modules contained in this repository are the following:

* **ddsrouter_core** This is the main library that implements the operation of the DDS Router.
  The `DDSRouter` class could be instantiate and has some methods to handle the operation of a DDS Router
  (e.g `start`, `reload`, `stop`).
  This package is divided in:
  * `Types`: all the types that are used to configure the DDS Router and in its internal functionality.
  * `Configuration`: Configuration classes to configure the DDS Router
  * `Core`: The main functionality and internal classes. This is not exported.

* **ddsrouter_yaml** This library allows to get a DDS Router configuration object from a `yaml`.
  It depends on `yaml-cpp` external library.

* **ddsrouter_tool** This is an executable that instantiates a DDS Router from a `yaml` file.

* **ddsrouter_docs** (hosted in directory `docs`)
  This package contains the user documentation built with sphinx.

* **ddsrouter_yaml_validator** This is an application used for validating DDS Router `yaml` configuration files.

This is the dependency graph of the packages:

```sh
#  *  = dependency
# (*) = test dependency

ddsrouter_docs  +
ddsrouter_core     +  *  *
ddsrouter_yaml        +  *
ddsrouter_tool           +
```

The reasons of the packages division are:

1. Using the DDS Router as a library will be very useful in future developments, avoiding to interact with it as an
  independent process.
1. The dependencies of one package are no longer dependencies of the whole project.
  Thus, the DDS Router could be used without requiring the `yaml` library.
1. It is easier to organize and maintain small packages than a big one.
1. In the future utils and events are intended to be generic, and not only part of this project.
  (e.g. having a common Log library for many projects would be very useful).

---

## Packages policy

Packages are intended to be independent.
However, it is useful to maintain a common development policy:

1. All packages use same version, and are released at the same time in the github repository.
1. Every package must be contained in its directory and every file needed for it must be inside as well.
1. There could not be cyclic dependencies between packages.
1. Every CMake package is configured by a `project_settings.cmake` and should try to use the generic CMakeLists.

### Project settings

There is a special file in each of the packages called `project_settings.cmake`.
This CMake file configure the specific values required to parametrize the common `CMakeLists` that are used in every
project.
The values that must be initialized are:

* `MODULE_NAME` project name
* `MODULE_SUMMARY` project summary
* `MODULE_FIND_PACKAGES` packages that the project needs to find by `find_package` cmake command
* `MODULE_DEPENDENCIES` project dependencies (usually: `${MODULE_FIND_PACKAGES}`)

Apart from this variable, there are other variables that may be useful to set.
Check `cmake_utils` README to know the different values used.

---

## Contributing guidelines

### License

Every file must have its license at its head.
This license be consistent with the following one (change the year to the year the file has been created):

```cpp
// Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
```

### Headers

**Headers must not contain any function implementation**, just the signatures.
Each header must start with a unique define.
In order to assure that they are unique, use the path of the file in it.

Example:

```cpp
#ifndef _DDSROUTERCORE_CORE_DDSROUTERCORE_HPP_
```

### Template implementations

The templates must be instantiated in a header file when no specialized.
For this propose, a special file is used:
`impl/<header>.ipp` file should be allocated in the directory of the header that declares the template,
and the template methods without specialization must be implemented in it.

### Log Verbosity

These are the log levels and when to use them
(some of them are not implemented separately, so they use others existing calls.)

| Level           | Macro       | Visible              | Implementation | Used when:                                            |
|-----------------|-------------|----------------------|----------------|-------------------------------------------------------|
| Error           | logError    | always               | logError       | an error must break execution                         |
| Developer Error | logDevError | only debug           | logWarning     | an internal error occurred but does not affect user   |
| Warning         | logWarning  | with -d option       | logWarning     | something went wrong but the execution could continue |
| User            | logUser     | always               | std::cout      | showing message to the user                           |
| Info            | logInfo     | only debug -d option | logInfo        | showing important information about the execution     |
| Debug           | logDebug    | only debug -d option | logInfo        | showing non important information                     |

### Configurations

Configurations are data structures with public access to their internal values.
This makes it easier to work with them: to create, manage and copy them.
The default values will be defined in the header, so every user could know which are them, and always
have a default constructor.

From every internal object that requires a configuration to be created, the configuration argument
must be created as an internal value and/or const to the object,
or create some way that the configuration cannot be changed from outside without the internal object notice it.

Also, configurations must support an `is_valid` method to check that it is in a coherent state.

---

## WorkArounds

> // TODO

### RPC in Core refactor

Doing the refactor to separate DDS Router from Core we encountered a difficulty because `RPCBridge` depends on `rtps::CommonReader`.
This makes impossible so far to separate Participants from the Core.
This should be solved in future versions by implementing a generic RPC Bridge without dependencies.
