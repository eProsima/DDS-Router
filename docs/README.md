# eProsima DDS Router docs

This package generates the DDS Router documentation.
[Here](https://eprosima-dds-router.readthedocs.io/en/latest/) it can be seen the online documentation hosted in
[readthedocs](https://readthedocs.org/).
This packages is powered by [sphinx](https://www.sphinx-doc.org/en/master/).

## Documentation generation

In order to install this package independently, use the following command:

```bash
colcon build --packages-select ddsrouter_docs
```

In order to compile and execute the package **tests**, a specific CMake option is required: `BUILD_DOCUMENTATION_TESTS`.

```bash
colcon build --packages-select ddsrouter_docs --cmake-args -DBUILD_DOCUMENTATION_TESTS
colcon test --packages-select ddsrouter_docs --event-handler console_direct+
```

## Library documentation

This documentation is focused on user experience.
To learn about the repository structure, the design decisions, etc. (developer information), every package is
commented separately and code is commented using Doxygen.
In directory `.dev` there is a generic `README.md` with the main information needed by a developer.
