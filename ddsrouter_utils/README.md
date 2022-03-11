# eProsima DDS Router Utils Module

This is a library with generic utils classes and functions.
It is intended to be generic and not just for DDS Router.
This could be a project of its own which could be used for several other projects in the future.

It contains the following functionality:

* **Log**: uses the *eProsima Fast DDS* Log module.

* **Exception**: each exception inherits from the same parent class, and each subclass is specific to a particular case.
  This makes it easier to catch exceptions throughout the code, differentiating between exception causes,
  source of the issue, and internal and external exceptions.

* **utils**: contains generic independent functions for very different solutions.
  (e.g. utilities for containes with smart pointers, string utilities, etc.)

* **Generic Types**: types that do not depend on the project. e.g. **Time**.

* **Return codes**: uses *eProsima Fast DDS* `ReturnCodes`.

* **macros**: generic cpp macros to handle types and templates.

---

## Formatter

`Formatter` is an utility class that serves to concatenate string streams.
For example, when creating an Exception message, it could not use a standard `<<` concatenation.
For this propose, `Formatter` comes in hand.
Using a default instance of a `Formatter` object, it can concatenate strings on it by using `<<`.

### Example of usage

```cpp
// CONCATENATE AN INTEGER IN A STRING

int i = 0;
Exception(STR_ENTRY << "Integer raise: " << i << ".");  // STR_ENTRY = Formatter()

// This code is equivalent to:

int i;
std::stringstream ss;
ss << "Integer raise: " << i << ".";
Exception(ss..str().c_str());
```

---

## Dependencies

* `fastrtps`

Only for Windows:

* `iphlpapi`
* `Shlwapi`

---

## CMake options

* `BUILD_TESTS`
* `BUILD_APPLICATION_TESTS`
* `LOG_INFO`

---

## How to use it in your project

Just import library `ddsrouter_utils` into your CMake project.

```cmake
find_package(ddsrouter_utils)
target_link_libraries(${LIBRARY_TARGET_NAME} ddsrouter_utils)
```
