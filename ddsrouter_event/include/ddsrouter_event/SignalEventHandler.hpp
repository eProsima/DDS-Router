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
 * @file SignalEventHandler.hpp
 */

#ifndef _DDSROUTEREVENT_SIGNALEVENTHANDLER_HPP_
#define _DDSROUTEREVENT_SIGNALEVENTHANDLER_HPP_

#include <atomic>
#include <functional>

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
class IBaseSignalEventHandler
{
public:

    //! This virtual destructor is required so objects could be destroyed from its common interface.
    virtual ~IBaseSignalEventHandler()
    {
    }

};

/**
 * It raises a callback each time the signal specified by \c SigVal is received.
 *
 * This class uses \c SignalManager to handle the signal.
 * This registers a callback in \c SignalManager so each signal received calls \c event_occured_
 *
 * The template \c SigVal references the signal that the object will handle following the
 * values in \c Signal .
 * Be aware that objects for different signals are completely independent as they are different objects
 * that only share the same templatization.
 */
template <Signal SigVal>
class SignalEventHandler : public EventHandler<Signal>, public IBaseSignalEventHandler
{
public:

    /**
     * @brief Default constructor that intialized the EventHandler with a default callback.
     *
     * It is initialized with a callback that only prints a Log message when the signal has arrived.
     * In this class, handled events will be used mostly for wait interruption, so a default callback is set.
     * This default callback only logs that a signal has been received.
     *
     * @warning Default callback is set in this constructor, so EventHandler is enabled.
     */
    SignalEventHandler() noexcept;

    /**
     * @brief Construct a new Signal Handler object with specific callback
     *
     * @param callback : function that will be called when the signal raises.
     */
    SignalEventHandler(
            std::function<void(Signal)> callback) noexcept;

    /**
     * @brief Destroy Signal Handler object
     *
     * Calls \c unset_callback
     */
    ~SignalEventHandler();

protected:

    //! Specific set method that adds \c this to \c active_handlers_
    void callback_set_nts_() noexcept override;

    //! Specific set method that removes \c this from \c active_handlers_
    void callback_unset_nts_() noexcept override;

    //! Method called in the callback registered in \c SignalManager that calls \c event_occured_ .
    void signal_received_callback_() noexcept;

    //! Whether a callback has been registered in \c SignalManager or not yet.
    std::atomic<bool> callback_set_in_manager_;

    //! Id of the callback set in \c SignalManager .
    std::atomic<UniqueCallbackId> callback_id_;
};

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter_event/impl/SignalEventHandler.ipp>

#endif /* _DDSROUTEREVENT_SIGNALEVENTHANDLER_HPP_ */
