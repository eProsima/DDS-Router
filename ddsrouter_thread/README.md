# eProsima DDS Router Utils Module

This is a library with generic classes to work using Thread Pools.
Thread Pool is a paradigm to execute a program using asynchronous calls and at the same time
limiting the number of threads used. This allows to avoid the creation of threads and the change of contexts,
very expensive operations.

---

## Dependencies

* `ddsrouter_utils`
* `ddsrouter_event`

---

## How to use it in your project

Just import library `ddsrouter_thread` into your CMake project.

```cmake
find_package(ddsrouter_thread)
target_link_libraries(${LIBRARY_TARGET_NAME} ddsrouter_thread)
```
