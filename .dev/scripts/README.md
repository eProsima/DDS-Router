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

* **ddsrouter_utils** This is intend to be a generic utils library that could in the future be a generic utils for
  other libraries. It contains:
  * `Log`: `fastrtps` log module
  * `Exception`: centralize exception and subclasses
  * `Types`: generic types, as Time, ReturnCode or serialization helpers
  * `utils`: generic methods

* **ddsrouter_event** This is intend to be a generic utils library to handle a generic event. This classes could be
  inherited so they handle different events.
  An EventHandler listens to a specific event and raises a callback when it is call.
  It also provides functionality to wait till the event has occurred.
  * `Signal`: raise with Operating System (SO)-dependent signals
  * `FileWatcher`: watch a specific file and raise when it has been written up / modified
  * `PeriodicTimer`: raise every periodic time
  * `Log`: raise with every log consumed (could change the verbosity listened) (very useful for testing).

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

This is the dependency graph of the packages:

```sh
#  *  = dependency
# (*) = test dependency

ddsrouter_docs   +
ddsrouter_utils     +  *  *  *  *
ddsrouter_event        + (*)(*) *
ddsrouter_core            +  *  *
ddsrouter_yaml               +  *
ddsrouter_tool                  +
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

## Repeated files

Along all the subdirectories many files are repeated (e.g. VERSION, CMakeLists.txt, etc.).

In order to be able to maintain a single version for all this files, there has been decided to keep a common directory
(`.common`) with every repeated file;
being every other files in every subpackage a *hard link* of the files contained in this directory.
Thus, the file in `.common/` directory should be the one modified, and it will automatically modify every other
file linked.

> **NOTE:**  ddsrouter_docs does not use common files.

The main points supporting this decision are:

1. colcon packages could and should not use files outside their directory.
1. symbolic links are not supported by windows.

### Files Linked

* **VERSION** version of the project. CMake will take the version from here.
* **LICENSE** license of the project. It is installed along with each package.
* **cmake** many cmake functions and macros are reused in most of the subpackages.
* **test** gtest aux includes needed for all tests.
* **CMakeLists.txt** the main CMakeLists.txt is the same for all packages, parametrized by `project_settings.cmake`.

There are some files that are not generic for all packages, but only for those that generates a library
or an application.

Use the script `update_links.bash` to create all the common files and link them with hard links.
When downloading from github, these links are lost.

#### Only library packages

* **include/<package_name>** every library requires the same files to generate the .lib/.so . These files are parametrized (`.in`)
  so the CMakeLists.txt produces and installs the files headers.
  * `config.h.in` parameterizable file to configure version and project metaparameters.
  * `eprosima_auto_link.h.in` parameterizable file to create dll
  * `library_dll.h.in` parameterizable file to set macros to create dll

* **src/cpp** every library works with the same generic CMakeLists.txt parametrized by `project_settings.cmake`.

#### Only app packages

* **src/cpp** every app works with the same generic CMakeLists.txt parametrized by `project_settings.cmake`.

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

* `SUBMODULE_PROJECT_NAME` project name
* `SUBMODULE_PROJECT_SUMMARY` project summary
* `SUBMODULE_PROJECT_FIND_PACKAGES` packages that the project needs to find by `find_package` cmake command
* `SUBMODULE_PROJECT_DEPENDENCIES` project dependencies (usually: `${SUBMODULE_PROJECT_FIND_PACKAGES}`)
* `SUBMODULE_PROJECT_MACROS` project macros name (usually: `$SUBMODULE_PROJECT_NAME` in capital letters). This is used for example to set the dll macro: ${SUBMODULE_PROJECT_MACROS}_DllAPI

---

## Design decisions

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
