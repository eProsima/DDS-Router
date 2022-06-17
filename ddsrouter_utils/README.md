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

* **Math**: generic cpp math functions improved to accelerate performance

* **Time**: generic cpp classes and functions related with time values.

* **Memory**: New smart pointer implementations to handle shared objects with a strong ownership.

* **Event**: An `EventHandler` is an object instantiated with a callback, and this callback will be called whenever the event
  this object handles occur.
  This object stores a counter with the number of times this event has happened.
  It also implements the functionality of waiting for 1 or `n` events to occur.
  There are some event handlers implemented, but more could be added by inheriting the class `EventHandler`.
  The ones available are:
  * **Signal**: handles a specific signal from the OS.
    The event occurs whenever a signal with the number the object specifies is triggered within the process.
  * **FileWatcher**: handles file modification events.
  * **PeriodicTimer**: triggers a timer and each time the timer ends, the event is triggered and the callback is called.
  * **LogConsumer**: activates every time a log is consumed (printed).
    It could also be filtered to only some verbosity levels.
  * **MultipleHandler**: allow to register `EventHandlers`, so every time any of the handlers catch an event,
    `MultipleHandler` event callback will be triggered.
    For example, this is useful to do a handler that waits for two different signals.

* **Wait**: An `WaitHandler` is an object that implements an easy way to block threads depending on variables and
  conditions. This is done to free the user of creating mutexes, conditions variables and the management required
  to add enable/disable functionality to this wait conditions.
  There are some handlers implemented, but more could be added by inheriting the class `WaitHandler`.
  The ones available are:
  * **Boolean**: awaits in a boolean variable that could be *open* or *close*
  * **Int**: awaits in an integer variable and implement wait specific functions to wait for a specific value, or a greater or lower value
  * **Counter**: awaits in an integer that awakes as many threads as the value inside, decreasing by one per each thread awaken
  * **DBQueueWaitHandler**: this is a consumer handler implemented with an internal double queue that allows to wait for
    a thread to be elements added to the queue and consume one of them. The consumer thread will take first element in the queue (FIFO)
    or wait in case the queue is empty to an element to be added.

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

## Event

### PeriodicEventHandler Example of usage

```cpp
// Print message every 5 seconds
utils::PeriodicEventHandler periodic_event_handler(
    utils::Duration_ms(5000),
    ()[]{
        std::cout << "timer ellapsed." << std::endl;
        /* do something else */
    });
```

### SignalEventHandler Example of usage

```cpp
// Wait for a signal
utils::SignalEventHandler<utils::SIGNAL_SIGINT> signal_handler;
signal_handler.wait_for_event();
```

---

## Wait

### BooleanWaitHandler Example of usage

```cpp
// Wait for another thread to set wait_handler as open
utils::BooleanWaitHandler handler;

/* Waiting thread */
handler.wait(); // wait for other tread to awake this

/* Manager thread */
handler.open(); // This awakes the threads waiting in this handler
handler.close(); // This makes new threads that call wait() to wait until next open
```

---

## Dependencies

* `fastrtps`

Only for Windows:

* `iphlpapi`
* `Shlwapi`

---

## How to use it in your project

Just import library `ddsrouter_utils` into your CMake project.

```cmake
find_package(ddsrouter_utils)
target_link_libraries(${LIBRARY_TARGET_NAME} ddsrouter_utils)
```
