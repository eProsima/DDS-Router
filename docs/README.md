# eProsima DDS Router docs

This package generates the DDS Router documentation.
[Here](https://eprosima-dds-router.readthedocs.io/en/latest/) it can be seen the online documentation hosted in
[readthedocs](https://readthedocs.org/).
This packages is powered by [sphinx](https://www.sphinx-doc.org/en/master/).

---

## Documentation generation

### Dependencies

Before being able to build the documentation, some dependencies need to be installed:

```bash
sudo apt update
sudo apt install -y \
    doxygen \
    python3 \
    python3-pip \
    python3-venv \
    python3-sphinxcontrib.spelling \
    imagemagick
pip3 install -U -r src/ddsrouter/docs/requirements.txt
```

### Build documentation

In order to install this package independently, use the following command:

```bash
colcon build --packages-select ddsrouter_docs
```

In order to compile and execute the package **tests**, a specific CMake option is required: `BUILD_DOCS_TESTS`.

```bash
colcon build --packages-select ddsrouter_docs --cmake-args -DBUILD_DOCS_TESTS=ON
colcon test --packages-select ddsrouter_docs --event-handler console_direct+
```

---

## Library documentation

This documentation is focused on the user manual for installing and working with DDS Router.
To learn about the repository structure, design decisions, development guidelines, etc.,
each package is documented separately and the source code is commented using Doxygen format.
In directory `.dev` there is a generic `README.md` with the main information needed by a developer.
