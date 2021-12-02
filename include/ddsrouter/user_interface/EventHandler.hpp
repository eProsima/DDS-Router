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

#ifndef _DDSROUTER_USERINTERFACE_EVENTHANDLER_HPP_
#define _DDSROUTER_USERINTERFACE_EVENTHANDLER_HPP_

#include <functional>

namespace eprosima {
namespace ddsrouter {
namespace ui {

/**
 * This class is an interface for any class that implements a handler of any kind of event.
 *
 * Consider an event every signal that a process can receive externally.
 * For example: signals, file updates, periodic alarms, etc.
 *
 * Consider a Handler a class that listens for one of these events, and at the moment of the event it raises a
 * previously set callback.
 *
 * This class implements the main methods for every Event Handler.
 *
 * It also implements a wait method that will make a thread passively wait for n events of a specific kind.
 *
 * It is a template regarding the arguments that the Event Handler needs in its callback.
 */
template <typename ... Args>
class EventHandler
{
public:

    /**
     * @brief Default constructor
     *
     * It is initialized as if it would have no callback
     */
    EventHandler();

    /**
     * @brief Construct an EventHandler with a specific callback.
     *
     * @param callback : function that will be called when the event raises.
     */
    EventHandler(
            std::function<void(Args...)> callback);

    /**
     * @brief Set the callback
     *
     * If callback is already set, it overwrites it.
     * If callback is not set yet, it uses this callback in advance.
     *
     * It calls the internal method \c callback_set_ once the callback is set so
     * child classes can add functionality when a callback is set.
     *
     * @param callback : new callback for this Event
     */
    void set_callback(
            std::function<void(Args...)> callback) noexcept;

    /**
     * @brief Unset the callback and set this object as if it had no callback
     *
     * It calls the internal method \c callback_unset_ once the callback is unset so
     * child classes can add functionality when a callback is unset.
     */
    void unset_callback() noexcept;

    /**
     * @brief Wait passively until the \c n th event has arrived.
     *
     * It does not start in 0, \c n makes reference to the total amount of events received.
     * Otherwise, it could get stuck if the event occurs before this wait.
     *
     * @param n : number of events at which this thread will awake
     */
    void wait_for_event(
            uint32_t n = 1) const noexcept;

    //! Simulate as if the event had occurred
    void simulate_event_occurred(
            Args... args) noexcept;

protected:

    /**
     * @brief Call callback function
     *
     * This method will be called every time the event occurs or by calling \c force_callback
     *
     * @param arg : argument of the callback to call
     */
    void event_occurred_(
            Args... args) noexcept;

    /**
     * Protected method to overwrite in child classes if specific functionality is required
     * when a new callback is set.
     *
     * If it is not overwritten, it does nothing.
     */
    virtual void callback_set_() noexcept;

    /**
     * Protected method to overwrite in child classes if specific functionality is required
     * when a new callback is unset.
     *
     * If it is not overwritten, it does nothing.
     */
    virtual void callback_unset_() noexcept;

    //! Internal callback reference
    std::function<void(Args...)> callback_;

    //! Number of times the event has occurred so far
    std::atomic<uint32_t> number_of_events_registered_;

    //! Whether the callback of this Handler is set
    std::atomic<bool> is_callback_set_;

    /**
     * @brief Default callback. It shows a warning that callback is not set
     *
     * @note This callback should never be called as the callback is not called
     * unless \c is_callback_set_ is true. However, it is useful for unset callback.
     */
    static const std::function<void(Args...)> DEFAULT_CALLBACK_;

    //! Condition variable to wait until the event occurs
    mutable std::condition_variable wait_condition_variable_;

    //! Guard access to \c wait_condition_variable_
    mutable std::mutex wait_mutex_;
};

} /* namespace ui */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter/user_interface/impl/EventHandler.ipp>

#endif /* _DDSROUTER_USERINTERFACE_EVENTHANDLER_HPP_ */
