# eProsima DDS Router Core Library

This library contain main functionality of the DDS Router.
It is divided in public and private modules.
Public module is the public API used to configure a DDS Router and to interact with a running one:

* **Types**: data types used to configure a router.
  * Address
  * DDS info (Guid, Qos, Topic, etc.)

* **Configuration**: configuration objects that contains the information needed for a DDS Router. Are divided in:
  * *ParticipantConfiguration*: configuration for each participant inside the DDS Router.
  * *DDSRouterConfiguration*: configuration to execute a DDS Router, with its Participants and allowed lists.
  * *DDSRouterReloadConfiguration*: configuration to change topics in a running DDS Router.

* **Core**: it only contains the proxy of DDS Router class, which implementation is inside private modules.
  It allows to execute a DDS Router, and to interact with it while running.

The private part of the DDS Router is not public in the API, and it is divided in the following modules:

* **Communication**: implement the communication module that sends data from Reader to Writers of different
  Participants.
  It is based on a `Bridge` for each `Topic`, and each `Bridge` is divided in a `Track` for each `Reader`.
* **Core**: implement the core object of the DDS Router.
* **Efficiency**: allow to communicate messages without copying the value.
* **Dynamic**: allow to dynamically create entities to communicate in new topics.
* **Participant**: implement every Participant kind (DDSRouterParticipant is not a DDSParticipant).
* **Reader/Writer**: implement Readers and Writers for the Participants.

---

## Example of use

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

Just has to import in CMake the library `ddsrouter_core`

```cmake
find_package(ddsrouter_core)
target_link_libraries(${LIBRARY_TARGET_NAME} ddsrouter_core)
```
