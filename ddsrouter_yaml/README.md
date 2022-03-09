# eProsima DDS Router Yaml Module

This library implement functions to generate DDS Router configurations from a YAML.

> *TODO:* write the design and functionality of this library

---

## Example of use

```cpp
// GET DDS ROUTER CONFIGURATION FROM FILE

core::configuration::DDSRouterConfiguration router_configuration =
                yaml::YamlReaderConfiguration::load_ddsrouter_configuration_from_file("configuration.yaml");
```

---

## Dependencies

* `yaml-cpp`
* `ddsrouter_utils`
* `ddsrouter_core`

Only for test:

* `ddsrouter_event`

---

## CMake options

* `BUILD_TESTS`
* `BUILD_APPLICATION_TESTS`
* `LOG_INFO`

---

## How to use it in your project

Just has to import in CMake the library `ddsrouter_yaml`

```cmake
find_package(ddsrouter_yaml)
target_link_libraries(${LIBRARY_TARGET_NAME} ddsrouter_yaml)
```
