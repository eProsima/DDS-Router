# eProsima DDS Router Event Module

This library contain classes with generic functionality to handle events.
It is intended to be generic and not just for DDS Router.
This could be a project of its own which could be used for several other projects in the future.

An `EventHandler` is an object instantiated with a callback, and this callback will be called whenever the event
this object handles occur.
This object stores a counter with the number of times this event has happened.
It also implements the functionality of waiting for 1 or `n` events to occur.

There are some event handlers implemented, but more could be added by inheriting the class `EventHandler`.
It contain the following handlers:

* **Signal**: handles a specific signal from the OS.
  The event occurs whenever a signal with the number the object specifies is triggered within the process.

* **FileWatcher**: handles file modification events.

* **PeriodicTimer**: triggers a timer and each time the timer ends, the event is triggered and the callback is called.

* **LogConsumer**: activates every time a log is consumed (printed).
  It could also be filtered to only some verbosity levels.

* **MultipleHandler**: allow to register `EventHandlers`, so every time any of the handlers catch an event,
  `MultipleHandler` event callback will be triggered.
  For example, this is useful to do a handler that waits for two different signals.

---

## Example of usage

```cpp
// Print message every 5 seconds
event::PeriodicEventHandler periodic_event_handler(
    utils::Duration_ms(5000),
    ()[]{std::cout << "timer ellapsed." << std::endl});
```

```cpp
// Wait for a signal
event::SignalHandler<event::SIGNAL_SIGINT> signal_handler;
signal_handler.wait_for_event();
```

---

## Dependencies

* `ddsrouter_utils`

---

## CMake options

* `BUILD_TESTS`
* `BUILD_APPLICATION_TESTS`
* `LOG_INFO`

---

## How to use it in your project

Just import library `ddsrouter_event` into your CMake project.

```cmake
find_package(ddsrouter_event)
target_link_libraries(${LIBRARY_TARGET_NAME} ddsrouter_event)
```
