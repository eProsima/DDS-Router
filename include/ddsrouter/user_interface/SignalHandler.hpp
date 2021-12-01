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

#ifndef _DDSROUTER_USERINTERFACE_SIGNALHANDLER_HPP_
#define _DDSROUTER_USERINTERFACE_SIGNALHANDLER_HPP_

#include <csignal>
#include <string>
#include <functional>

#include <ddsrouter/user_interface/EventHandler.hpp>

namespace eprosima {
namespace ddsrouter {
namespace ui {

//! Available data types for SignalHandler class
enum Signals
{
    SIGNAL_SIGINT = SIGINT, //! SIGINT = C^
};

/**
 * It raises a callback each time the signal specified by \c SigNum is received.
 *
 * This class is special because signals must be handled statically. Thus it stores every object
 * created in a static array, and every signal it arrives it call the callback for each object
 * in this array.
 * It could be several Handlers for the same signal, and all the callbacks will be called when
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
     * @brief Default constructor
     *
     * It is initialized with a callback that only add a Log that the signals has arrived.
     *
     * This event will be used mostly for wait, so the default callback is already set.
     *
     * @note Adds \c this to a static list of active \c SignalHandlers .
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
    SignalHandler(std::function<void(int)> callback) noexcept;

    /**
     * @brief Destroy Signal Handler object
     *
     * @note It eliminates \c this from static list of \c SignalHandlers .
     * If it is the last one, unset the static signal handler function.
     */
    ~SignalHandler();

protected:

    //! Specific set method that adds \c this to \c active_handlers_
    void callback_set_() noexcept override;

    //! Specific set method that removes \c this from \c active_handlers_
    void callback_unset_() noexcept override;

    //! Add \c this to the active handlers list. Called when callback is set.
    void add_to_active_handlers_() noexcept;

    //! Remove \c this to the active handlers list. Called when callback is unset.
    void erase_from_active_handlers_() noexcept;

    /**
     * @brief Method that will be called each time the signal arrives.
     *
     * This method will call callback of every \c SignalHandler in \c active_handlers_ .
     *
     * @param signum : signal value (it must be equal to \c SigNum )
     */
    static void signal_handler_routine_(int signum) noexcept;

    //! Set for while process the signal handler routine.
    static void set_signal_handler_() noexcept;

    //! Unset for while process the signal handler routine.
    static void unset_signal_handler_() noexcept;

    /**
     * @brief List of active \c SignalHandlers
     *
     * Every time a callback is set to a \c SignalHandlers , the handler is added to this list.
     * Every time a callback is unset to a \c SignalHandlers , the handler is erased from this list.
     */
    static std::vector<SignalHandler*> active_handlers_;

    //! Guards access to variable \c active_handlers_
    static std::mutex active_handlers_mutex_;
};

} /* namespace ui */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter/user_interface/impl/SignalHandler.ipp>

#endif /* _DDSROUTER_USERINTERFACE_SIGNALHANDLER_HPP_ */
