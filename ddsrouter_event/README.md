# eProsima DDS Router Event Module

This library contain classes with generic functionality to handle events.
It is intended to be generic and not only for DDS Router propose.
It may in the future be its own project that could be used for several ones.

An EventHandler is an object instantiated with a callback, and this callback will be called whenever the event
this object handles occur.
This object stores a counter with the number of times this event has happened.
It also implements the functionality to wait for 1 or n events occur.

There are some event handlers implemented, but more could be added by inheriting the class `EventHandler`.
It contain the following handlers:

* **Signal**: handles a specific signal from the OS.
  The event occurs every time a signal with number the object specifies arises in the process.
  This object handles the `signal` method and creates its own static variables to handle thread calls in specific
  thread.

* **FileWatcher**: handles the listening on modifications over a file.

* **PeriodicTimer**: activates a timer and every time the timer elapsed, the event arises and call the callback.

* **LogConsumer**: activates every time a log is consumed (printed).
  It could also be filtered to only some verbosity levels.

* **MultipleHandler**: allow to register EventHandlers inside, so every time any of the handlers listen an event,
  it will call the event of the MultipleHandler.
  For example, this is useful to do a handler that waits in two different signals.

---

## Example of use

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

Just has to import in CMake the library `ddsrouter_event`

```cmake
find_package(ddsrouter_event)
target_link_libraries(${LIBRARY_TARGET_NAME} ddsrouter_event)
```
