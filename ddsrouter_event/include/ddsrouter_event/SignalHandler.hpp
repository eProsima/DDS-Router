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
#include <functional>
#include <mutex>
#include <thread>

#include <ddsrouter_event/EventHandler.hpp>
#include <ddsrouter_event/SignalManager.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {

/**
 * @brief Parent class for SignalEventHandler
 *
 * \c SignalEventHandler class is a template, and so there is no common parent class for every SignalEventHandler object.
 * This class represents this common parent class without template, so it could be created a common
 * interface of every kind of SignalEventHandler .
 *
 * This class does not implement nor define any method or variable required. It is merely an auxiliar
 * class for container of SignalEventHandlers.
 */
class IBaseSignalHandler
{
public:

    //! This virtual destructor is required so objects could be destroyed from its common interface.
    virtual ~IBaseSignalHandler()
    {
    }

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
class SignalHandler : public EventHandler<int> , public IBaseSignalHandler
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

    void signal_received_callback_() noexcept;

    std::atomic<bool> callback_set_in_manager_;

    std::atomic<UniqueCallbackId> callback_id_;
};

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter_event/impl/SignalHandler.ipp>

#endif /* _DDSROUTEREVENT_SIGNALHANDLER_HPP_ */
