.. include:: ../../../exports/alias.include
.. include:: ../../../exports/roles.include

.. _cmake_options:

#############
CMake options
#############

*eProsima DDS Router* provides numerous CMake options for changing the behavior and configuration of
*DDS Router*.
These options allow the developer to enable/disable certain *DDS Router* settings by defining these options to
``ON``/``OFF`` at the CMake execution, or set the required path to certain dependencies.

.. warning::
    These options are only for developers who installed *eProsima DDS Router* following the compilation steps
    described in :ref:`developer_manual_installation_sources_linux`.

.. list-table::
    :header-rows: 1

    *   - Option
        - Description
        - Possible values
        - Default
    *   - :class:`CMAKE_BUILD_TYPE`
        - CMake optimization build type.
        - ``Release`` |br|
          ``Debug``
        - ``Release``
    *   - :class:`BUILD_ALL`
        - Build the *DDS Router* application tool `ddsrouter`. |br|
          Setting to ``ON`` sets to ``ON`` |br|
          :class:`BUILD_TOOL`, :class:`BUILD_LIBRARY`, and |br|
          :class:`BUILD_DOCS`.
        - ``OFF`` |br|
          ``ON``
        - ``OFF``
    *   - :class:`BUILD_LIBRARY`
        - Build the *DDS Router* documentation.
          It is set to ``ON`` if :class:`BUILD_ALL` is set to ``ON``.
        - ``OFF`` |br|
          ``ON``
        - ``ON``
    *   - :class:`BUILD_TOOL`
        - Build the *DDS Router* documentation.
          It is set to ``ON`` if :class:`BUILD_ALL` is set to ``ON``.
        - ``OFF`` |br|
          ``ON``
        - ``ON``
    *   - :class:`BUILD_DOCS`
        - Build the *DDS Router* documentation.
          It is set to ``ON`` if :class:`BUILD_ALL` is set to ``ON``.
        - ``OFF`` |br|
          ``ON``
        - ``OFF``
    *   - :class:`BUILD_TESTS`
        - Build the *DDS Router* application and documentation |br|
          tests. Setting :class:`BUILD_TESTS` to ``ON`` sets |br|
          :class:`BUILD_ALL`, :class:`BUILD_LIBRARY_TESTS`, |br|
          :class:`BUILD_TOOL_TESTS`, and :class:`BUILD_DOCS_TESTS` |br|
          to ``ON``.
        - ``OFF`` |br|
          ``ON``
        - ``OFF``
    *   - :class:`BUILD_LIBRARY_TESTS`
        - Build the *DDS Router* library tests. It is |br|
          set to ``ON`` if :class:`BUILD_TESTS` is set to ``ON``. |br|
        - ``OFF`` |br|
          ``ON``
        - ``OFF``
    *   - :class:`BUILD_TOOL_TESTS`
        - Build the *DDS Router* application tests. It is |br|
          set to ``ON`` if :class:`BUILD_TESTS` is set to ``ON``. |br|
        - ``OFF`` |br|
          ``ON``
        - ``OFF``
    *   - :class:`BUILD_DOCS_TESTS`
        - Build the *DDS Router* documentation tests. It is |br|
          set to ``ON`` if :class:`BUILD_TESTS` is set to ``ON``. |br|
        - ``OFF`` |br|
          ``ON``
        - ``OFF``
    *   - :class:`LOG_INFO`
        - Activate *DDS Router* execution logs. It is |br|
          set to ``ON`` if :class:`CMAKE_BUILD_TYPE` is set |br|
          to ``Debug``.
        - ``OFF`` |br|
          ``ON``
        - ``ON`` if ``Debug`` |br|
          ``OFF`` otherwise
    *   - :class:`ASAN_BUILD`
        - Activate address sanitizer build.
        - ``OFF`` |br|
          ``ON``
        - ``OFF``
    *   - :class:`TSAN_BUILD`
        - Activate thread sanitizer build.
        - ``OFF`` |br|
          ``ON``
        - ``OFF``
