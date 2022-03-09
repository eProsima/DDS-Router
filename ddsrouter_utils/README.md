# eProsima DDS Router Utils Module

This is a library with generic utils classes and functions.
It is intended to be generic and not only for DDS Router propose.
It may in the future be its own project that could be used for several ones.

It contain the following functionality:

* **Log**: uses the `fastrtps` Log module.

* **Exception**: every exception inherits from same parent class, and each subclass is specific for one cause.
  This facilitates the captured of exceptions along the code, differentiating between one class and another, and
  by internal or external exceptions.

* **utils**: contains generic independent functions for very different solutions.
  (e.g. utilities for containes with smart pointers, string utilities, etc.)

* **Generic Types**: types that do not depend on the project. e.g. **Time**.

* **Return codes**: uses `fastrtps` ReturnCodes.

* **macros**: generic cpp macros for handle types and templates.

---

## Formatter

`Formatter` is a useful class that serves for concatenate streams of text.
For example, when creating an Exception message, it could not be used a standard `<<` concatenation.
For this propose, `Formatter` comes in hand.
Using a default instance of a `Formatter` object, it can concatenate strings on it by using `<<`.

### Example of use

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

Just has to import in CMake the library `ddsrouter_utils`

```cmake
find_package(ddsrouter_utils)
target_link_libraries(${LIBRARY_TARGET_NAME} ddsrouter_utils)
```
