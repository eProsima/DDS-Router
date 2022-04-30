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
 * @file CollectionWaitHandler.hpp
 */

#ifndef _DDSROUTEREVENT_WAIT_COLLECTIONWAITHANDLER_HPP_
#define _DDSROUTEREVENT_WAIT_COLLECTIONWAITHANDLER_HPP_

#include <ddsrouter_event/wait/CounterWaitHandler.hpp>

namespace eprosima {
namespace ddsrouter {
namespace event {

/**
 *TODO
 */
template <typename T>
class CollectionWaitHandler : protected CounterWaitHandler
{
public:

    // Make this methods public
    using CounterWaitHandler::enable;
    using CounterWaitHandler::disable;
    using CounterWaitHandler::enabled;
    using CounterWaitHandler::stop_and_continue;

    /////
    // Add values methods

    void add_value(T&& value);

    /////
    // Get values methods

    T get_next_value(
        const utils::Duration_ms& timeout = 0);

protected:

    virtual T get_next_value_() = 0;

    virtual T add_value_(T&& value) = 0;
};

} /* namespace event */
} /* namespace ddsrouter */
} /* namespace eprosima */

// Include implementation template file
#include <ddsrouter_event/wait/impl/CollectionWaitHandler.ipp>

#endif /* _DDSROUTEREVENT_WAIT_COLLECTIONWAITHANDLER_HPP_ */
