# eProsima DDS Router Yaml Module

This library implement functions to generate DDS Router configurations from a YAML.
It is powered by `yaml-cpp` library.

It provides methods to:

- create every type of object from a yaml node.
- read a yaml file
- to interact with yaml using `YAML::Node` class from `yaml-cpp`

---

## Example of use

```cpp
// GET DDS ROUTER CONFIGURATION FROM FILE

core::configuration::DDSRouterConfiguration router_configuration =
                yaml::YamlReaderConfiguration::load_ddsrouter_configuration_from_file("configuration.yaml");
```

---

## YamlReader

The main functionality of Yaml parser is based on static methods centralized in the same class `YamlReader`.
This class contain static template methods that help to parse every object required.
It also support multiple versions at the same time to get objects from a yaml.

### How to implement a new class getter

In order to get a new type of class from a yaml, only specialize the method `get<Foo>` and add it the
functionality required to read such element.

```cpp
// Class to read
struct Foo
{
    int a;             // required int element that must be inside tag <a>
    std::set<float> b; // required int vector set that must be inside tag <b>
    std::string c;     // not required element that must be inside tag <c>
    std::string c;     // not required element that must be inside tag <c>
};

// Getter method
Foo YamlReader::get<Foo>(const Yaml& yml, const YamlReaderVersion version = LATEST)
{
    // Force to get element int a inside tag <a>
    int a = get_scalar<int>(yml, "a");

    // Force to get set element int a inside tag <b>
    std::set<int> b = get_set<float>(yml, "b");

    // Optional get c
    std::string c;
    if (is_tag_present(yml, "c"))
    {
        c = get_scalar<std::string>(yml, "c");
    }

    return Foo(a, b, c);
}

// Use function to get an object Foo inside tag <foo> in a yaml
void get_foo(const Yaml& yml)
{
    Foo foo = get<Foo>(yml, "foo");
}

/*
 * valid yaml examples
 *
 * - foo:{a:3, b:[]}
 * - foo:{a:5, b:[1.1, 2.2], c="foo value"}
 */
```

### How to support new version getters

If the yaml format changes, there is no need to change every method.
It is enough changing the behavior of the get method that modifies the behavior in the old version.

```cpp
// Class that contains a foo
struct FooMayor
{
    Foo internal_foo;   // Required value.
    // In version v1.0 foo comes inside tag <foo-tag>
    // From v2.0 foo comes in tag <FOO-tag>
};

// Getter method
FooMayor YamlReader::get<FooMayor>(const Yaml& yml, const YamlReaderVersion version = LATEST)
{
    switch (version)
    {
        case V_1_0:
            return FooMayor(            // create FooMayor by getting a Foo object inside tag <foo>
                get<Foo>(yml, "foo-tag"));
        default:
            return FooMayor(            // create FooMayor by getting a Foo object inside tag <FOO>
                get<Foo>(yml, "FOO-tag"));
}
// THERE IS NO NEED TO REIMPLEMENT Foo GETTER IF IT DOES NOT CHANGE FROM A VERSION TO OTHER

// Use function to get an object Foo inside tag <foo> in a yaml
void get_foo_mayor(const Yaml& yml)
{
    FooMayor foo = get<FooMayor>(yml, "foo", V_1_0);
}

/*
 * valid yaml examples for v1.0
 * - foo:{a:3, b:[]}
 * valid yaml examples for v2.0
 * - FOO:{a:5, b:[1.1, 2.2], c="foo value"}
 */
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
