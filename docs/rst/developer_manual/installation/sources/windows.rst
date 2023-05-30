.. include:: ../../../exports/alias.include
.. include:: ../../../exports/roles.include

.. _developer_manual_installation_sources_windows:

#################################
Windows installation from sources
#################################

The instructions for installing the |ddsrouter| application from sources and its required
dependencies are provided in this page.
It is organized as follows:

.. contents::
    :local:
    :backlinks: none
    :depth: 2


Dependencies installation
=========================

|ddsrouter| depends on *eProsima Fast DDS* library and certain Debian packages.
This section describes the instructions for installing |ddsrouter| dependencies and requirements in a Windows
environment from sources.
The following packages will be installed:

- ``foonathan_memory_vendor``, an STL compatible C++ memory allocation library.
- ``fastcdr``, a C++ library that serializes according to the standard CDR serialization mechanism.
- ``fastrtps``, the core library of eProsima Fast DDS library.
- ``cmake_utils``, an eProsima utils library for CMake.
- ``cpp_utils``, an eProsima utils library for C++.

First of all, the :ref:`Requirements <windows_sources_requirements>` and
:ref:`Dependencies <windows_sources_dependencies>` detailed below need to be met.
Afterwards, the user can choose whether to follow either the :ref:`colcon <windows_sources_colcon_installation>` or the
:ref:`CMake <windows_sources_cmake_installation>` installation instructions.


.. _windows_sources_requirements:

Requirements
------------

The installation of *eProsima Fast DDS* in a Windows environment from sources requires the following tools to be
installed in the system:

* :ref:`windows_sources_visual_studio`
* :ref:`windows_sources_chocolatey`
* :ref:`windows_sources_cmake_pip3_wget_git`
* :ref:`windows_sources_colcon_install` [optional]
* :ref:`windows_sources_gtest` [for test only]
* :ref:`windows_py_yaml` [for YAML Validator only]
* :ref:`windows_json_schema` [for YAML Validator only]

.. _windows_sources_visual_studio:

Visual Studio
^^^^^^^^^^^^^

`Visual Studio <https://visualstudio.microsoft.com/>`_ is required to
have a C++ compiler in the system. For this purpose, make sure to check the
:code:`Desktop development with C++` option during the Visual Studio installation process.

If Visual Studio is already installed but the Visual C++ Redistributable packages are not,
open Visual Studio and go to :code:`Tools` -> :code:`Get Tools and Features` and in the :code:`Workloads` tab enable
:code:`Desktop development with C++`. Finally, click :code:`Modify` at the bottom right.

.. _windows_sources_chocolatey:

Chocolatey
^^^^^^^^^^

Chocolatey is a Windows package manager. It is needed to install some of *eProsima Fast DDS*'s dependencies.
Download and install it directly from the `website <https://chocolatey.org/>`_.

.. _windows_sources_cmake_pip3_wget_git:

CMake, pip3, wget and git
^^^^^^^^^^^^^^^^^^^^^^^^^

These packages provide the tools required to install *eProsima Fast DDS* and its dependencies from command line.
Download and install CMake_, pip3_, wget_ and git_ by following the instructions detailed in the respective
websites.
Once installed, add the path to the executables to the :code:`PATH` from the
*Edit the system environment variables* control panel.

.. _windows_sources_colcon_install:

Colcon
^^^^^^

colcon_ is a command line tool based on CMake_ aimed at building sets of software packages.
Install the ROS 2 development tools (colcon_ and vcstool_) by executing the following command:

.. code-block:: bash

    pip3 install -U colcon-common-extensions vcstool

.. note::

    If this fails due to an Environment Error, add the :code:`--user` flag to the :code:`pip3` installation command.


.. _windows_sources_gtest:

Gtest
^^^^^

Gtest is a unit testing library for C++.
By default, |ddsrouter| does not compile tests.
It is possible to activate them with the opportune
`CMake options <https://colcon.readthedocs.io/en/released/reference/verb/build.html#cmake-options>`_
when calling colcon_ or CMake_.
For more details, please refer to the :ref:`cmake_options` section.

Run the following commands on your workspace to install Gtest.

.. code-block:: bash

    git clone https://github.com/google/googletest.git
    cmake -DCMAKE_INSTALL_PREFIX='C:\Program Files\gtest' -Dgtest_force_shared_crt=ON -DBUILD_GMOCK=ON ^
        -B build\gtest -A x64 -T host=x64 googletest
    cmake --build build\gtest --config Release --target install

or refer to the
`Gtest Installation Guide <https://github.com/google/googletest>`_ for a detailed description of the Gtest installation
process.


.. _windows_py_yaml:

PyYAML
^^^^^^

`PyYAML <https://pyyaml.org/>`_ is a YAML parser and emitter for Python.

It is used by the DDS-Router :ref:`yaml_validator` for loading the content of configuration files.

Install ``pyyaml`` by executing the following command:

.. code-block:: bash

    pip3 install -U pyyaml


.. _windows_json_schema:

jsonschema
^^^^^^^^^^

`jsonschema <https://python-jsonschema.readthedocs.io/>`_ is an implementation of the JSON Schema specification for
Python.

It is used by the DDS-Router :ref:`yaml_validator` for performing validation of configuration files against a given
JSON schema.

Install ``jsonschema`` by executing the following command:

.. code-block:: bash

    pip3 install -U jsonschema

.. _windows_sources_dependencies:

Dependencies
------------

|ddsrouter| has the following dependencies, when installed from sources in a Windows environment:

* :ref:`windows_sources_asiotinyxml2`
* :ref:`windows_sources_openssl`
* :ref:`windows_sources_yamlcpp`
* :ref:`windows_sources_eprosima_dependencies`

.. _windows_sources_asiotinyxml2:

Asio and TinyXML2 libraries
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Asio is a cross-platform C++ library for network and low-level I/O programming, which provides a consistent
asynchronous model.
TinyXML2 is a simple, small and efficient C++ XML parser.
They can be downloaded directly from the links below:

* `Asio <https://github.com/ros2/choco-packages/releases/download/2020-02-24/asio.1.12.1.nupkg>`_
* `TinyXML2 <https://github.com/ros2/choco-packages/releases/download/2020-02-24/tinyxml2.6.0.0.nupkg>`_

After downloading these packages, open an administrative shell with *PowerShell* and execute the following command:

.. code-block:: bash

    choco install -y -s <PATH_TO_DOWNLOADS> asio tinyxml2

where :code:`<PATH_TO_DOWNLOADS>` is the folder into which the packages have been downloaded.

.. _windows_sources_openssl:

OpenSSL
^^^^^^^

OpenSSL is a robust toolkit for the TLS and SSL protocols and a general-purpose cryptography library.
Download and install the latest OpenSSL version for Windows at this
`link <https://slproweb.com/products/Win32OpenSSL.html>`_.
After installing, add the environment variable :code:`OPENSSL_ROOT_DIR` pointing to the installation root directory.

For example:

.. code-block:: bash

   OPENSSL_ROOT_DIR=C:\Program Files\OpenSSL-Win64

.. _windows_sources_yamlcpp:

yaml-cpp
^^^^^^^^

yaml-cpp is a YAML parser and emitter in C++ matching the YAML 1.2 spec, and is used by *DDS Router* application to
parse the provided configuration files.
From an administrative shell with *PowerShell*, execute the following commands in order to download and install yaml-cpp
for Windows:

.. code-block:: bash

   git clone --branch yaml-cpp-0.7.0 https://github.com/jbeder/yaml-cpp
   cmake -DCMAKE_INSTALL_PREFIX='C:\Program Files\yamlcpp' -B build\yamlcpp yaml-cpp
   cmake --build build\yamlcpp --target install    # If building in Debug mode, add --config Debug

.. _windows_sources_eprosima_dependencies:

eProsima dependencies
^^^^^^^^^^^^^^^^^^^^^

If it already exists in the system an installation of *Fast DDS* library with version greater than `2.4.0`, just source
this library when building the |ddsrouter| application by using the command:

.. code-block:: bash

    source <fastdds-installation-path>/install/setup.bash

In other case, just download *Fast DDS* project from sources and build it together with |ddsrouter| using colcon
as it is explained in section :ref:`windows_sources_colcon_installation`.


.. _windows_sources_colcon_installation:

Colcon installation
===================

.. important::

    Run colcon within a Visual Studio prompt. To do so, launch a *Developer Command Prompt* from the
    search engine.

#.  Create a :code:`DDS-Router` directory and download the :code:`.repos` file that will be used to install
    |ddsrouter| and its dependencies:

    .. code-block:: bash

        mkdir <path\to\user\workspace>\DDS-Router
        cd <path\to\user\workspace>\DDS-Router
        mkdir src
        wget https://raw.githubusercontent.com/eProsima/DDS-Router/main/ddsrouter.repos
        vcs import src < ddsrouter.repos

    .. note::

        In case there is already a *Fast DDS* installation in the system it is not required to download and build
        every dependency in the :code:`.repos` file.
        It is just needed to download and build the |ddsrouter| project having sourced its dependencies.
        Refer to section :ref:`eprosima_dependencies` in order to check how to source *Fast DDS* library.

#.  Build the packages:

    .. code-block:: bash

        colcon build

.. note::

    Being based on CMake_, it is possible to pass the CMake configuration options to the :code:`colcon build`
    command. For more information on the specific syntax, please refer to the
    `CMake specific arguments <https://colcon.readthedocs.io/en/released/reference/verb/build.html#cmake-specific-arguments>`_
    page of the colcon_ manual.


.. _windows_sources_cmake_installation:

CMake installation
==================

This section explains how to compile |ddsrouter| with CMake_, either
:ref:`locally <windows_sources_local_installation>` or :ref:`globally <windows_sources_global_installation>`.

.. _windows_sources_local_installation:

Local installation
------------------

#.  Open a command prompt, and create a :code:`DDS-Router` directory where to download and build |ddsrouter| and
    its dependencies:

    .. code-block:: bash

        mkdir <path\to\user\workspace>\DDS-Router
        mkdir <path\to\user\workspace>\DDS-Router\src
        mkdir <path\to\user\workspace>\DDS-Router\build
        cd <path\to\user\workspace>\DDS-Router
        wget https://raw.githubusercontent.com/eProsima/DDS-Router/main/ddsrouter.repos
        vcs import src < ddsrouter.repos

#.  Compile all dependencies using CMake_.

    *  `Foonathan memory <https://github.com/foonathan/memory>`_

        .. code-block:: bash

            cd <path\to\user\workspace>\DDS-Router
            mkdir build\foonathan_memory_vendor
            cd build\foonathan_memory_vendor
            cmake <path\to\user\workspace>\DDS-Router\src\foonathan_memory_vendor -DCMAKE_INSTALL_PREFIX=<path\to\user\workspace>\DDS-Router\install ^
                -DBUILD_SHARED_LIBS=ON
            cmake --build . --config Release --target install

    *  `Fast CDR <https://github.com/eProsima/Fast-CDR>`_

        .. code-block:: bash

            cd <path\to\user\workspace>\DDS-Router
            mkdir build\fastcdr
            cd build\fastcdr
            cmake <path\to\user\workspace>\DDS-Router\src\fastcdr -DCMAKE_INSTALL_PREFIX=<path\to\user\workspace>\DDS-Router\install
            cmake --build . --config Release --target install

    *  `Fast DDS <https://github.com/eProsima/Fast-DDS>`_

        .. code-block:: bash

            cd <path\to\user\workspace>\DDS-Router
            mkdir build\fastdds
            cd build\fastdds
            cmake <path\to\user\workspace>\DDS-Router\src\fastdds -DCMAKE_INSTALL_PREFIX=<path\to\user\workspace>\DDS-Router\install ^
                -DCMAKE_PREFIX_PATH=<path\to\user\workspace>\DDS-Router\install
            cmake --build . --config Release --target install

    * `Dev Utils <https://github.com/eProsima/dev-utils>`_

        .. code-block:: bash

            # CMake Utils
            cd <path\to\user\workspace>\DDS-Router
            mkdir build\cmake_utils
            cd build\cmake_utils
            cmake <path\to\user\workspace>\DDS-Router\src\dev-utils\cmake_utils -DCMAKE_INSTALL_PREFIX=<path\to\user\workspace>\DDS-Router\install ^
                -DCMAKE_PREFIX_PATH=<path\to\user\workspace>\DDS-Router\install
            cmake --build . --config Release --target install

            # C++ Utils
            cd <path\to\user\workspace>\DDS-Router
            mkdir build\cpp_utils
            cd build\cpp_utils
            cmake <path\to\user\workspace>\DDS-Router\src\dev-utils\cpp_utils -DCMAKE_INSTALL_PREFIX=<path\to\user\workspace>\DDS-Router\install ^
                -DCMAKE_PREFIX_PATH=<path\to\user\workspace>\DDS-Router\install
            cmake --build . --config Release --target install

    * `DDS Pipe <https://github.com/eProsima/DDS-Pipe>`_

        .. code-block:: bash

            # ddspipe_core
            cd <path\to\user\workspace>\DDS-Router
            mkdir build\ddspipe_core
            cd build\ddspipe_core
            cmake <path\to\user\workspace>\DDS-Router\src\ddspipe\ddspipe_core -DCMAKE_INSTALL_PREFIX=<path\to\user\workspace>\DDS-Router\install ^
                -DCMAKE_PREFIX_PATH=<path\to\user\workspace>\DDS-Router\install
            cmake --build . --config Release --target install

            # ddspipe_participants
            cd <path\to\user\workspace>\DDS-Router
            mkdir build\ddspipe_participants
            cd build\ddspipe_participants
            cmake <path\to\user\workspace>\DDS-Router\src\ddspipe\ddspipe_participants -DCMAKE_INSTALL_PREFIX=<path\to\user\workspace>\DDS-Router\install ^
                -DCMAKE_PREFIX_PATH=<path\to\user\workspace>\DDS-Router\install
            cmake --build . --config Release --target install

            # ddspipe_yaml
            cd <path\to\user\workspace>\DDS-Router
            mkdir build\ddspipe_yaml
            cd build\ddspipe_yaml
            cmake <path\to\user\workspace>\DDS-Router\src\ddspipe\ddspipe_yaml -DCMAKE_INSTALL_PREFIX=<path\to\user\workspace>\DDS-Router\install ^
                -DCMAKE_PREFIX_PATH=<path\to\user\workspace>\DDS-Router\install
            cmake --build . --config Release --target install

#.  Once all dependencies are installed, install |ddsrouter|:

    .. code-block:: bash

        # ddsrouter_core
        cd <path\to\user\workspace>\DDS-Router
        mkdir build\ddsrouter_core
        cd build\ddsrouter_core
        cmake <path\to\user\workspace>\DDS-Router\src\ddsrouter\ddsrouter_core ^
            -DCMAKE_INSTALL_PREFIX=<path\to\user\workspace>\DDS-Router\install -DCMAKE_PREFIX_PATH=<path\to\user\workspace>\DDS-Router\install
        cmake --build . --config Release --target install

        # ddsrouter_yaml
        cd <path\to\user\workspace>\DDS-Router
        mkdir build\ddsrouter_yaml
        cd build\ddsrouter_yaml
        cmake <path\to\user\workspace>\DDS-Router\src\ddsrouter\ddsrouter_yaml -DCMAKE_INSTALL_PREFIX=<path\to\user\workspace>\DDS-Router\install ^
            -DCMAKE_PREFIX_PATH=<path\to\user\workspace>\DDS-Router\install
        cmake --build . --config Release --target install

        # ddsrouter_tool
        cd <path\to\user\workspace>\DDS-Router
        mkdir build\ddsrouter_tool
        cd build\ddsrouter_tool
        cmake <path\to\user\workspace>\DDS-Router\src\ddsrouter\tools\ddsrouter_tool -DCMAKE_INSTALL_PREFIX=<path\to\user\workspace>\DDS-Router\install ^
            -DCMAKE_PREFIX_PATH=<path\to\user\workspace>\DDS-Router\install
        cmake --build . --config Release --target install


.. note::

    By default, |ddsrouter| does not compile tests.
    However, they can be activated by downloading and installing `Gtest <https://github.com/google/googletest>`_
    and building with CMake option ``-DBUILD_TESTS=ON``.


.. _windows_sources_global_installation:

Global installation
-------------------

To install |ddsrouter| system-wide instead of locally, remove all the flags that
appear in the configuration steps of :code:`foonathan_memory_vendor`, :code:`Fast-CDR`, :code:`Fast-DDS`, and
:code:`DDS-Router`


Run an application
==================

If the |ddsrouter| was compiled using colcon, when running an instance of a |ddsrouter|, the colcon overlay built in the
dedicated :code:`DDS-Router` directory must be sourced.
There are two possibilities:

* Every time a new shell is opened, prepare the environment locally by typing the
  command:

  .. code-block:: bash

      setup.bat

* Add the sourcing of the colcon overlay permanently, by opening the
  *Edit the system environment variables* control panel, and adding :code:`~/Fast-DDS/install/setup.bat`
  to the :code:`PATH`.

However, when running an instance of a |ddsrouter| compiled using CMake, it must be linked with its dependencies where
the packages have been installed. This can be done by opening the *Edit system environment variables* control panel and
adding to the ``PATH`` the |ddsrouter|, *Fast DDS* and *Fast CDR* installation directories:

*   *Fast DDS*: C:\\Program Files\\fastrtps
*   *Fast CDR*: C:\\Program Files\\fastcdr
*   |ddsrouter|: C:\\Program Files\\ddsrouter


.. External links

.. _colcon: https://colcon.readthedocs.io/en/released/
.. _CMake: https://cmake.org
.. _pip3: https://docs.python.org/3/installing/index.html
.. _wget: https://www.gnu.org/software/wget/
.. _git: https://git-scm.com/
.. _vcstool: https://pypi.org/project/vcstool/
.. _Gtest: https://github.com/google/googletest
