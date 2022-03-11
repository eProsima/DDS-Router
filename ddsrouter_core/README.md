# eProsima DDS Router Core Library

This library contain main functionality of the DDS Router.
Include module is the public API used to configure a DDS Router and to interact with a running one:

* **Types**: data types used to configure a router.
  * Address
  * DDS info (Guid, Qos, Topic, etc.)

* **Configuration**: configuration objects that contains the information needed for a DDS Router. Are divided in:
  * *ParticipantConfiguration*: configuration for each participant inside the DDS Router.
  * *DDSRouterConfiguration*: configuration to execute a DDS Router, with its Participants and allowed lists.
  * *DDSRouterReloadConfiguration*: configuration to change topics in a running DDS Router.

* **Core**: it only contains the proxy of DDS Router class, which implementation is inside private modules.
  It allows to execute a DDS Router, and to interact with it while running.

---

## Example of usage

```cpp
// START AND STOP DDS ROUTER FROM CONFIGURATION

core::configuration::DDSRouterConfiguration router_configuration;

// ... populate router_configuration

core::DDSRouter router(router_configuration);
router.start();

// ... wait for event

router.stop();
```

---

## Dependencies

* `fastrtps`
* `ddsrouter_utils`

Only for test:

* `ddsrouter_event`

---

## CMake options

* `BUILD_TESTS`
* `BUILD_APPLICATION_TESTS`
* `LOG_INFO`

---

## How to use it in your project

Just import library `ddsrouter_core` into your CMake project.

```cmake
find_package(ddsrouter_core)
target_link_libraries(${LIBRARY_TARGET_NAME} ddsrouter_core)
```
