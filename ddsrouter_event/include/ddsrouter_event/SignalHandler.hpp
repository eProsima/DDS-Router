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
 * @file SignalHandler.hpp
 */

#ifndef _DDSROUTEREVENT_SIGNALHANDLER_HPP_
#define _DDSROUTEREVENT_SIGNALHANDLER_HPP_

#include <atomic>
#include <condition_variable>
#include <csignal>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include <ddsrouter_event/EventHandler.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {

//! Available data types for SignalHandler class
enum Signals
{
    SIGNAL_SIGINT   = SIGINT,   //! SIGINT = ^C
    SIGNAL_SIGTERM  = SIGTERM,  //! SIGTERM = kill
};

/**
 * It raises a callback each time the signal specified by \c SigNum is received. Signals may be captured from any
 * running thread, but handling is performed here in a dedicated thread (\c signal_handler_thread_) in order to avoid
 * unexpected behaviors.
 *
 * This class is special because signals must be handled statically. Thus it stores every object
 * created in a static array, and for every signal that arrives it calls the callback of each object
 * in this array.
 * There could be several Handlers for the same signal, and all the callbacks will be called when
 * the signal arrives.
 *
 * The template \c SigNum references the signal that the object will handle following the
 * values in \c Signals .
 * Be aware that objects for different signals are completely independent as they are different objects
 * that only share the same templatization.
 */
template <int SigNum>
class SignalHandler : public EventHandler<int>
{
public:

    /**
     * @brief Default constructor that intialized the EventHandler with a default callback.
     *
     * It is initialized with a callback that only prints a Log message when the signal has arrived.
     *
     * In this class, handled events will be used mostly for wait interruption, so a default callback is set.
     * This default callback only logs that a signal has been received.
     *
     * @note Adds \c this to a static list of active \c SignalHandlers .
     *
     * @warning Default callback is set in this constructor, so EventHandler is enabled.
     */
    SignalHandler() noexcept;

    /**
     * @brief Construct a new Signal Handler object with specific callback
     *
     * @param callback : function that will be called when the signal raises.
     *
     * @note Adds \c this to a static list of active \c SignalHandlers .
     * If it is the first one, set the static signal handler function.
     */
    SignalHandler(
            std::function<void(int)> callback) noexcept;

    /**
     * @brief Destroy Signal Handler object
     *
     * Calls \c unset_callback
     *
     * @note It eliminates \c this from static list of \c SignalHandlers .
     * If it is the last one, unset the static signal handler function.
     */
    ~SignalHandler();

protected:

    //! Specific set method that adds \c this to \c active_handlers_
    void callback_set_nts_() noexcept override;

    //! Specific set method that removes \c this from \c active_handlers_
    void callback_unset_nts_() noexcept override;

    //! Add \c this to the active handlers list. Called when callback is set.
    void add_to_active_handlers_() noexcept;

    //! Remove \c this to the active handlers list. Called when callback is unset.
    void erase_from_active_handlers_() noexcept;

    /**
     * @brief Method that will be called each time the signal arrives.
     *
     * This method will call callback of every \c SignalHandler in \c active_handlers_ .
     */
    static void signal_handler_routine_() noexcept;

    //! Set for while process the signal handler routine.
    static void set_signal_handler_() noexcept;

    //! Unset for while process the signal handler routine.
    static void unset_signal_handler_() noexcept;

    //! Routine performed by dedicated signal handling thread
    static void signal_handler_thread_routine_() noexcept;

    /**
     * @brief List of active \c SignalHandlers
     *
     * Every time a callback is set to a \c SignalHandlers , the handler is added to this list.
     * Every time a callback is unset to a \c SignalHandlers , the handler is erased from this list.
     */
    static std::vector<SignalHandler*> active_handlers_;

    //! Guards access to variable \c active_handlers_
    static std::mutex active_handlers_mutex_;

    //! Handle of thread dedicated to listening for signal arrival
    static std::thread signal_handler_thread_;

    //! Flag used to terminate \c signal_handler_thread_
    static std::atomic<bool> signal_handler_active_;

    //! Counter incremented when a signal arrives and decremented after being read by \c signal_handler_thread_
    static std::atomic<uint32_t> signals_received_;

    //! Condition variable to wait in \c signal_handler_thread_ until a signal arrives or signal handler is unset
    static std::condition_variable signal_received_cv_;

    //! Guards access to \c signal_received_cv_
    static std::mutex signal_received_cv_mutex_;
};

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter_event/impl/SignalHandler.ipp>

#endif /* _DDSROUTEREVENT_SIGNALHANDLER_HPP_ */
