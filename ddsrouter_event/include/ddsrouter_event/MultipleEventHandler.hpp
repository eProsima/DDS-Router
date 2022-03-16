// Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file MultipleEventHandler.hpp
 */

#ifndef _DDSROUTEREVENT_MULTIPLEEVENTHANDLER_HPP_
#define _DDSROUTEREVENT_MULTIPLEEVENTHANDLER_HPP_

#include <list>
#include <memory>

#include <ddsrouter_event/EventHandler.hpp>
#include <ddsrouter_event/library/library_dll.h>

namespace eprosima {
namespace ddsrouter {
namespace event {

/**
 * @brief This class is an \c EventHandler which trigger is an event that triggers any of its internal EventHandlers.
 *
 * Inside a MultipleEventHandler there will be any number of other EventHandlers.
 * This EventHandlers are owned by this object, and they lose their previous callback (if set) and cannot be used
 * outside this object.
 * Every internal EventHandler callback is set so they call \c event_occurred_ of this object, and thus every
 * event that trigger any of the internal handlers, will trigger this one.
 *
 * This class is very useful to wait on several events.
 *
 * Recommendation of use:
 * Create every EventHandler using make_unique in the same call of \c register_event_handler . e.g.:
 *  signal_handlers.register_event_handler<EventHandler<int>, int>(make_unique<SignalEventHandler<SIGNAL_SIGINT>>());
 */
class MultipleEventHandler : public EventHandler<>
{
public:

    /**
     * @brief Construct a new MultipleEventHandler object with specific callback
     *
     * @param callback : function that will be called when the signal raises.
     */
    DDSROUTER_EVENT_DllAPI MultipleEventHandler(
            std::function<void()> callback);

    /**
     * @brief Default constructor that intialized the EventHandler with a default callback.
     *
     * In this class, handled events will be used mostly for wait interruption, so a default callback is set.
     * This default callback only logs that a signal has been received.
     *
     * @warning Default callback is set in this constructor, so EventHandler is enabled.
     */
    DDSROUTER_EVENT_DllAPI MultipleEventHandler();

    /**
     * @brief Destroy MultipleEventHandler object
     *
     * Releases every inside EventHandler.
     * Calls \c unset_callback .
     */
    DDSROUTER_EVENT_DllAPI ~MultipleEventHandler();

    /**
     * @brief Register a new \c EventHandler
     *
     * Every handler registered will change its callback to target this object callback.
     *
     * @tparam T typename of the EventHandler class (e.g. EventHandler<int> for SignalEventHandler)
     * @tparam Args typename of the arguments for the class set in T (e.g. int for SignalEventHandler)
     * @warning \c T must be the form of T<Args>. This is because C++ could not handle the template substitution
     * otherwise.
     *
     * @param handler pointer and ownership for the handler
     */
    template <typename T, typename ... Args>
    void register_event_handler(
            std::unique_ptr<T> handler) noexcept;

protected:

    /**
     * @brief List of EventHandlers that are registered inside this.
     *
     * Each of this handlers belong to this object and cannot and wont be used from outside.
     * Each handler callback is redirected to this object callback.
     *
     * It must be of class \c IBaseEventHandler because \c Eventhandler works with static polymorphism (template)
     * and thus there is no common interface to collect them.
     */
    std::list<std::unique_ptr<IBaseEventHandler>> handlers_registered_;
};

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter_event/impl/MultipleEventHandler.ipp>

#endif /* _DDSROUTEREVENT_MULTIPLEEVENTHANDLER_HPP_ */
