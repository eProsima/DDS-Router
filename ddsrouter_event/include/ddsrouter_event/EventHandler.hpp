// Copyright 2021 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file EventHandler.hpp
 */

#ifndef _DDSROUTEREVENT_EVENTHANDLER_HPP_
#define _DDSROUTEREVENT_EVENTHANDLER_HPP_

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>

namespace eprosima {
namespace ddsrouter {
namespace event {

/**
 * @brief Parent class for EventHandler
 *
 * \c EventHandler class is a template, and so there is no common parent class for every EventHandler object.
 * This class represents this common parent class without template, so it could be created a common
 * interface of every kind of EventHandler .
 *
 * This class does not implement nor define any method or variable required. It is merely an auxiliar
 * class for container of EventHandlers.
 */
class IBaseEventHandler
{
public:

    //! This virtual destructor is required so objects could be destroyed from its common interface.
    virtual ~IBaseEventHandler()
    {
    }

};

/**
 * This class is an interface for any class that implements a handler of any kind of event.
 * Consider an event every signal that a process can receive externally.
 * For example: signals, file updates, periodic alarms, etc.
 * Consider a Handler a class that listens for one of these events, and at the moment of the event it raises a
 * previously set callback.
 *
 * This class implements the main methods for every Event Handler.
 * Setting a callback enables the handler.
 * Unsetting a callback disables the handler.
 *
 * It also implements a wait method that will make a thread passively wait for n events of a specific kind.
 * These threads will exit if the number of threads has been reached (since they are active) or if the
 * EventHandler object is disabled.
 *
 * It is a template regarding the arguments that the Event Handler needs in its callback.
 *
 * It inherits from \c IBaseEventHandler so every \c EventHandler specialization has a common interface.
 *
 * Inherit:
 * In order to create child classes, be aware of:
 * - While EventHandler is disabled (unset callback) the event should not repeat.
 *   This is because that could lead to a problem if when destroying the EventHandler an event occurs.
 *   It will try to access data of a destroyed object, and will crash.
 *   Disable the event listener when \c callback_unset_nts_ is called, and enable it with \c callback_set_nts_
 * - TIP: avoid setting listeners in constructor or unsetting them in destructor. Use \c callback_unset_nts_
 *   and \c callback_set_nts_ instead.
 * - WARNING: always implement destructor in childs and call \c unset_callback
 */
template <typename ... Args>
class EventHandler : public IBaseEventHandler
{
public:

    /**
     * @brief Default constructor
     *
     * It is initialized as if it would have no callback
     */
    EventHandler();

    /**
     * @brief Set the callback, and enable EventHandler if callback was not set
     *
     * If callback is already set, it overwrites it.
     * If callback is not set yet, it uses this callback in advance and enables the Handler.
     *
     * It calls the internal method \c callback_set_ once the callback is set so
     * child classes can add functionality when a callback is set.
     *
     * @param callback : new callback for this Event
     */
    void set_callback(
            std::function<void(Args...)> callback) noexcept;

    /**
     * @brief Unset the callback and set this object as disabled
     *
     * It calls the internal method \c callback_unset_ once the callback is unset so
     * child classes can add functionality when a callback is unset.
     *
     * It waits in \c event_mutex_ if the callback is being called.
     */
    void unset_callback() noexcept;

    /**
     * @brief Wait passively until the \c n th event has arrived.
     *
     * It does not start in 0, \c n makes reference to the total amount of events received.
     * Otherwise, it could get stuck if the event occurs before this wait.
     *
     * @param n : number of events at which this thread will awake
     *
     * @return \c true if exits wait due to number of events
     * @return \c false if exits wait due to other reasons (disable Handler)
     */
    bool wait_for_event(
            uint32_t n = 1) const noexcept;

    //! Return the times the event has occurred since it has started.
    uint32_t event_count() const noexcept;

    //! Simulate as if the event had occurred
    void simulate_event_occurred(
            Args... args) noexcept;

protected:

    /**
     * @brief Call callback function if Handler enabled
     *
     * This method will be called every time the event occurs or by calling \c force_callback
     *
     * @param arg : argument of the callback to call
     */
    void event_occurred_(
            Args... args) noexcept;

    /**
     * @brief Do not leave this method until every thread waiting in \c wait_for_event has exited
     *
     * This method must be called only when \c is_callback_set_ is false, otherwise events could never end.
     */
    void awake_all_waiting_threads_nts_() noexcept;

    /**
     * Protected method to overwrite in child classes if specific functionality is required
     * when a new callback is set.
     *
     * If it is not overwritten, it does nothing.
     *
     * It is already guarded by \c event_mutex_ .
     */
    virtual void callback_set_nts_() noexcept;

    /**
     * Protected method to overwrite in child classes if specific functionality is required
     * when callback was set and it change to a new one.
     *
     * If it is not overwritten, it does nothing.
     *
     * It is already guarded by \c event_mutex_ .
     */
    virtual void callback_change_nts_() noexcept;

    /**
     * Protected method to overwrite in child classes if specific functionality is required
     * when a new callback is unset.
     *
     * If it is not overwritten, it does nothing.
     *
     * It is already guarded by \c event_mutex_ .
     */
    virtual void callback_unset_nts_() noexcept;

    //! Internal callback reference
    std::function<void(Args...)> callback_;

    /**
     * @brief Whether the callback of this Handler is set
     *
     * This variable also references the enable/disable status of the EventHandler.
     *
     * Guard by \c wait_mutex_
     */
    std::atomic<bool> is_callback_set_;

    //! Mutex to block EventHandler execution while lambda is being set/unset or called
    mutable std::recursive_mutex internal_callback_mutex_;

    //! Mutex to block set and unset callbacks from outside the class
    mutable std::recursive_mutex event_mutex_;

    /**
     * @brief Number of times the event has occurred so far
     *
     * Guard by \c wait_mutex_
     */
    std::atomic<uint32_t> number_of_events_registered_;

    /**
     * @brief Number of threads that are currently waiting in \c wait_for_event
     *
     * Guard by \c wait_mutex_
     */
    mutable std::atomic<uint32_t> threads_waiting_;

    /**
     * @brief Condition variable to wait until the event occurs
     *
     * Guard by \c wait_mutex_
     */
    mutable std::condition_variable wait_condition_variable_;

    //! Guard access to \c wait_condition_variable_
    mutable std::mutex wait_mutex_;

    /**
     * @brief Default callback.
     *
     * @note This callback should never be called as the callback is not called
     * unless \c is_callback_set_ is true. However, it is useful for constructor without callback
     */
    static const std::function<void(Args...)> DEFAULT_CALLBACK_;
};

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter_event/impl/EventHandler.ipp>

#endif /* _DDSROUTEREVENT_EVENTHANDLER_HPP_ */
